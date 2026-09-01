const std = @import("std");
const solver = @import("solver.zig");
const vrp = @import("vrp.zig");
const vrptw = @import("vrptw.zig");
const pdptw = @import("pdptw.zig");
const pdptw_sisr = @import("pdptw_sisr.zig");
const asymmetric = @import("asymmetric.zig");

// =============================================================================
// commiv C ABI — the stable FFI surface (mirrors include/commiv.h).
//
// Zig is the underbone; nobody outside this repo should have to write it. Every
// binding (Python, the REST server clients, anything that can call C) goes
// through the functions in this file. Rules of the surface:
//   * plain C types only: u32 cost matrices, u32 demands, size_t counts;
//   * results come back as an opaque handle plus accessor functions, so the
//     struct layout can change without breaking callers;
//   * zero-initialized `commiv_options` means "all defaults";
//   * return codes, never Zig errors: 0 ok, negative = COMMIV_ERR_*.
// =============================================================================

const version_string = @import("version.zig").string;

// Thread-safe, libc-free allocator: the library must work when dlopen'd from
// arbitrary hosts, so no global init/deinit entry points to forget.
const gpa = std.heap.smp_allocator;

pub const COMMIV_OK: c_int = 0;
pub const COMMIV_ERR_INVALID_ARGUMENT: c_int = -1;
pub const COMMIV_ERR_OUT_OF_MEMORY: c_int = -2;
pub const COMMIV_ERR_INFEASIBLE: c_int = -3;
pub const COMMIV_ERR_INTERNAL: c_int = -4;

/// Mirrors `commiv_options` in commiv.h. Zero value = default for every field,
/// so callers can `memset(&opts, 0, sizeof opts)` (or pass NULL) and get the
/// documented defaults. Field meanings:
///   seed            RNG seed; 0 -> 1. Same seed + threads<=1 = identical output.
///   sisr_iters      SISR ruin&recreate iterations (CVRP and VRPTW); 0 -> 300k.
///   trials          ATSP trial budget; 0 -> solver default.
///   veh_penalty     Per-route penalty biasing toward fewer vehicles (VRPTW, PDPTW).
///   threads         0 or 1 = single-threaded (deterministic); >1 = parallel
///                   SISR chains (CVRP and VRPTW; result depends on the count).
///   fleet_min       Nonzero -> hierarchical fleet-minimization driver (VRPTW,
///                   PDPTW): minimize vehicles first, then distance. 0 = off.
///   wall_ms         Wall-clock budget in ms for SISR/fleet-min (VRPTW, PDPTW);
///                   0 = no wall cap (bounded by sisr_iters only).
///   max_vehicles    Hard cap on route count (VRPTW, PDPTW); 0 = uncapped. For
///                   PDPTW a positive cap selects the pinned (exact-count) driver.
///   time_penalty    PDPTW ONLY money objective: cost per matrix time-unit of
///                   route DURATION (travel + service + waiting); 0 = pure
///                   distance. Ignored by CVRP and VRPTW.
///   max_route_duration  Shift-length cap (VRPTW, PDPTW): no route may exceed
///                   this depot->..->depot duration (travel + service +
///                   waiting). 0 = uncapped. Ignored by CVRP.
///   break_duration / break_earliest / break_latest  Driver break (PDPTW
///                   ONLY, v1): break_duration > 0 requires every route whose
///                   depart-at-0 schedule finishes after break_earliest to
///                   contain ONE break of break_duration time units starting
///                   within [break_earliest, break_latest]. The break absorbs
///                   waiting first and counts into route duration (and thus
///                   the money objective). All-zero = no break. Unsupported
///                   with fleet_min / max_vehicles (INVALID_ARGUMENT);
///                   ignored by CVRP and VRPTW.
pub const CommivOptions = extern struct {
    seed: u64 = 0,
    sisr_iters: u64 = 0,
    trials: u64 = 0,
    veh_penalty: u64 = 0,
    threads: u32 = 0,
    fleet_min: u32 = 0, // was `reserved`; nonzero selects the fleet-min driver
    wall_ms: u64 = 0, // wall-clock budget in ms (VRPTW/PDPTW SISR); 0 = none
    max_vehicles: u64 = 0, // hard route cap; 0 = uncapped
    time_penalty: u64 = 0, // PDPTW money objective; ignored by CVRP/VRPTW
    max_route_duration: u64 = 0, // shift-length cap (VRPTW/PDPTW); 0 = uncapped
    break_duration: u32 = 0, // driver break length (PDPTW); 0 = no break
    break_earliest: u32 = 0, // break must START within [earliest, latest]
    break_latest: u32 = 0,
    reserved: u32 = 0, // keeps the struct 8-aligned at 88 bytes
};

/// Opaque to C; accessed through commiv_routes_* only. Routes are flattened:
/// route i is nodes[offsets[i]..offsets[i+1]], customer indices 1..n.
pub const CommivRoutes = struct {
    total_cost: u64,
    offsets: []usize, // len = route count + 1
    nodes: []u32,
    // Per-route vehicle-type index (commiv_solve_pdptw_typed only); empty for
    // uniform-fleet solves. Read through commiv_routes_type.
    types: []u32 = &.{},
};

fn resolveOptions(opt: ?*const CommivOptions) CommivOptions {
    var o: CommivOptions = if (opt) |p| p.* else .{};
    if (o.seed == 0) o.seed = 1;
    return o;
}

fn mapError(err: anyerror) c_int {
    return switch (err) {
        error.OutOfMemory => COMMIV_ERR_OUT_OF_MEMORY,
        error.Infeasible, error.NoFeasibleSplit, error.NoCompleteSolution => COMMIV_ERR_INFEASIBLE,
        error.InvalidInstance, error.InvalidMatrix, error.InstanceTooLargeForTransform => COMMIV_ERR_INVALID_ARGUMENT,
        else => COMMIV_ERR_INTERNAL,
    };
}

fn makeRoutes(routes: []const []const usize, total_cost: u64) !*CommivRoutes {
    return makeRoutesTyped(routes, null, total_cost);
}

fn makeRoutesTyped(routes: []const []const usize, route_types: ?[]const usize, total_cost: u64) !*CommivRoutes {
    var total_nodes: usize = 0;
    for (routes) |r| total_nodes += r.len;

    const h = try gpa.create(CommivRoutes);
    errdefer gpa.destroy(h);
    const offsets = try gpa.alloc(usize, routes.len + 1);
    errdefer gpa.free(offsets);
    const nodes = try gpa.alloc(u32, total_nodes);
    errdefer gpa.free(nodes);
    var types: []u32 = &.{};
    if (route_types) |rt| {
        types = try gpa.alloc(u32, rt.len);
        for (rt, 0..) |t, i| types[i] = @intCast(t);
    }
    errdefer if (types.len != 0) gpa.free(types);

    var at: usize = 0;
    for (routes, 0..) |r, i| {
        offsets[i] = at;
        for (r) |c| {
            nodes[at] = @intCast(c);
            at += 1;
        }
    }
    offsets[routes.len] = at;
    h.* = .{ .total_cost = total_cost, .offsets = offsets, .nodes = nodes, .types = types };
    return h;
}

/// Shared argument validation for the CVRP/VRPTW entry points. Returns 0 when
/// the instance shape is usable, a COMMIV_ERR_* otherwise.
fn checkVrpArgs(n_customers: usize, demand: [*]const u32, capacity: u32) c_int {
    if (n_customers == 0 or capacity == 0) return COMMIV_ERR_INVALID_ARGUMENT;
    // Node indices are returned as u32; (n+1)^2 must not overflow the matrix index.
    if (n_customers >= std.math.maxInt(u32)) return COMMIV_ERR_INVALID_ARGUMENT;
    _ = std.math.mul(usize, n_customers + 1, n_customers + 1) catch return COMMIV_ERR_INVALID_ARGUMENT;
    if (demand[0] != 0) return COMMIV_ERR_INVALID_ARGUMENT; // depot has no demand
    for (demand[1 .. n_customers + 1]) |d| {
        // A customer whose demand exceeds capacity can never be packed, even
        // with an unlimited fleet: fail fast with a semantic code instead of
        // whatever the solver would surface.
        if (d > capacity) return COMMIV_ERR_INFEASIBLE;
    }
    return COMMIV_OK;
}

// ---- exported API -----------------------------------------------------------

export fn commiv_version() [*:0]const u8 {
    return version_string;
}

/// Directed CVRP (uncapped fleet, SISR). matrix is (n+1)*(n+1) row-major,
/// matrix[a*(n+1)+b] = cost a->b, depot = 0. demand has n+1 entries, demand[0]=0.
/// On success *out owns the solution; free with commiv_routes_free.
export fn commiv_solve_cvrp(
    matrix: ?[*]const u32,
    n_customers: usize,
    demand: ?[*]const u32,
    capacity: u32,
    options: ?*const CommivOptions,
    out: ?**CommivRoutes,
) c_int {
    const m = matrix orelse return COMMIV_ERR_INVALID_ARGUMENT;
    const dem = demand orelse return COMMIV_ERR_INVALID_ARGUMENT;
    const out_p = out orelse return COMMIV_ERR_INVALID_ARGUMENT;
    const rc = checkVrpArgs(n_customers, dem, capacity);
    if (rc != COMMIV_OK) return rc;

    const o = resolveOptions(options);
    const dim = n_customers + 1;
    const inst = vrp.CvrpInstance{
        .n = n_customers,
        .matrix = m[0 .. dim * dim],
        .demand = dem[0..dim],
        .capacity = capacity,
    };
    var params: vrp.SisrParams = .{};
    if (o.sisr_iters != 0) params.iters = @intCast(o.sisr_iters);
    const solve_options: solver.SolveOptions = .{ .seed = o.seed };

    var result = blk: {
        if (o.threads > 1) {
            break :blk vrp.solveCvrpSisrParallel(gpa, inst, solve_options, params, o.threads) catch |err| return mapError(err);
        }
        break :blk vrp.solveCvrpSisr(gpa, inst, solve_options, params) catch |err| return mapError(err);
    };
    defer result.deinit();

    out_p.* = makeRoutes(result.routes, result.total_cost) catch |err| return mapError(err);
    return COMMIV_OK;
}

/// Directed VRPTW. Same matrix/demand/capacity contract as commiv_solve_cvrp;
/// ready/due/service are n+1 entries each (index 0 = depot: ready[0]=0, due[0]
/// = horizon, service[0]=0). Time windows constrain the START of service.
export fn commiv_solve_vrptw(
    matrix: ?[*]const u32,
    n_customers: usize,
    demand: ?[*]const u32,
    capacity: u32,
    ready: ?[*]const u32,
    due: ?[*]const u32,
    service: ?[*]const u32,
    options: ?*const CommivOptions,
    out: ?**CommivRoutes,
) c_int {
    const m = matrix orelse return COMMIV_ERR_INVALID_ARGUMENT;
    const dem = demand orelse return COMMIV_ERR_INVALID_ARGUMENT;
    const rdy = ready orelse return COMMIV_ERR_INVALID_ARGUMENT;
    const du = due orelse return COMMIV_ERR_INVALID_ARGUMENT;
    const srv = service orelse return COMMIV_ERR_INVALID_ARGUMENT;
    const out_p = out orelse return COMMIV_ERR_INVALID_ARGUMENT;
    const rc = checkVrpArgs(n_customers, dem, capacity);
    if (rc != COMMIV_OK) return rc;

    const o = resolveOptions(options);
    const dim = n_customers + 1;
    const inst = vrptw.VrptwInstance{
        .n = n_customers,
        .matrix = m[0 .. dim * dim],
        .demand = dem[0..dim],
        .capacity = capacity,
        .ready = rdy[0..dim],
        .due = du[0..dim],
        .service = srv[0..dim],
    };
    var result = blk: {
        var params: vrptw.VrptwSisrParams = .{ .veh_penalty = o.veh_penalty };
        if (o.sisr_iters != 0) params.iters = @intCast(o.sisr_iters);
        if (o.wall_ms != 0) params.time_ms = o.wall_ms;
        if (o.max_vehicles != 0) params.max_vehicles = @intCast(o.max_vehicles);
        params.max_route_dur = o.max_route_duration;
        if (o.fleet_min != 0) {
            // Hierarchical fleet minimization needs a wall budget; default 10s if unset.
            // fleet_min takes precedence over threads (the fleet-min driver is serial).
            const budget: u64 = if (o.wall_ms != 0) o.wall_ms else 10_000;
            break :blk vrptw.solveVrptwSisrFleetMin(gpa, inst, .{ .seed = o.seed }, params, budget) catch |err| return mapError(err);
        }
        if (o.threads > 1) {
            break :blk vrptw.solveVrptwSisrParallel(gpa, inst, .{ .seed = o.seed }, params, o.threads) catch |err| return mapError(err);
        }
        break :blk vrptw.solveVrptwSisr(gpa, inst, .{ .seed = o.seed }, params) catch |err| return mapError(err);
    };
    defer result.deinit();

    out_p.* = makeRoutes(result.routes, result.total_cost) catch |err| return mapError(err);
    return COMMIV_OK;
}

/// Argument validation + coverage/pairing pass for commiv_solve_pdptw. Fills the
/// caller-provided `seen` scratch (len == dim). Returns 0 when the instance shape
/// is usable, a COMMIV_ERR_* otherwise.
fn checkPdpArgs(
    n_pairs: usize,
    dim: usize,
    pickup_node: [*]const u32,
    delivery_node: [*]const u32,
    demand: [*]const u32,
    capacity: u32,
    ready: [*]const u32,
    service: [*]const u32,
    seen: []bool,
) c_int {
    if (n_pairs == 0 or capacity == 0) return COMMIV_ERR_INVALID_ARGUMENT;
    // Time-window depot sanity (due[0] = horizon, any value allowed).
    if (ready[0] != 0 or service[0] != 0) return COMMIV_ERR_INVALID_ARGUMENT;

    @memset(seen, false);
    for (0..n_pairs) |i| {
        const p = pickup_node[i];
        const q = delivery_node[i];
        if (p == 0 or q == 0 or p >= dim or q >= dim or p == q) return COMMIV_ERR_INVALID_ARGUMENT;
        if (seen[p] or seen[q]) return COMMIV_ERR_INVALID_ARGUMENT; // duplicate use of a node
        seen[p] = true;
        seen[q] = true;
        // A request whose load exceeds capacity can never be packed: fast-fail.
        if (demand[i] > capacity) return COMMIV_ERR_INFEASIBLE;
    }
    // Coverage: every node 1..dim-1 must be named exactly once.
    for (seen[1..dim]) |s| {
        if (!s) return COMMIV_ERR_INVALID_ARGUMENT;
    }
    return COMMIV_OK;
}

/// Directed PDPTW (pickup-and-delivery with time windows), SISR solver.
/// dim = 2*n_pairs + 1; node 0 is the depot. Each request is one pickup node
/// and one delivery node carried on the same route, pickup before delivery,
/// capacity respected along the route. Objective: directed distance, plus a
/// money charge on route duration when options->time_penalty > 0 (PDPTW-only).
export fn commiv_solve_pdptw(
    matrix: ?[*]const u32,
    n_pairs: usize,
    pickup_node: ?[*]const u32,
    delivery_node: ?[*]const u32,
    demand: ?[*]const u32,
    capacity: u32,
    ready: ?[*]const u32,
    due: ?[*]const u32,
    service: ?[*]const u32,
    options: ?*const CommivOptions,
    out: ?**CommivRoutes,
) c_int {
    const m = matrix orelse return COMMIV_ERR_INVALID_ARGUMENT;
    const pick = pickup_node orelse return COMMIV_ERR_INVALID_ARGUMENT;
    const drop = delivery_node orelse return COMMIV_ERR_INVALID_ARGUMENT;
    const dem = demand orelse return COMMIV_ERR_INVALID_ARGUMENT;
    const rdy = ready orelse return COMMIV_ERR_INVALID_ARGUMENT;
    const du = due orelse return COMMIV_ERR_INVALID_ARGUMENT;
    const srv = service orelse return COMMIV_ERR_INVALID_ARGUMENT;
    const out_p = out orelse return COMMIV_ERR_INVALID_ARGUMENT;

    if (n_pairs == 0 or capacity == 0) return COMMIV_ERR_INVALID_ARGUMENT;
    // Keep node ids in u32 and dim = 2*n_pairs+1 from overflowing.
    if (n_pairs >= (std.math.maxInt(u32) - 1) / 2) return COMMIV_ERR_INVALID_ARGUMENT;
    const dim = 2 * n_pairs + 1;
    _ = std.math.mul(usize, dim, dim) catch return COMMIV_ERR_INVALID_ARGUMENT;

    // Synthesized instance arrays + the validation scratch, all length dim.
    const seen = gpa.alloc(bool, dim) catch return COMMIV_ERR_OUT_OF_MEMORY;
    defer gpa.free(seen);
    const rc = checkPdpArgs(n_pairs, dim, pick, drop, dem, capacity, rdy, srv, seen);
    if (rc != COMMIV_OK) return rc;

    const pair_of = gpa.alloc(usize, dim) catch return COMMIV_ERR_OUT_OF_MEMORY;
    defer gpa.free(pair_of);
    const is_pickup = gpa.alloc(bool, dim) catch return COMMIV_ERR_OUT_OF_MEMORY;
    defer gpa.free(is_pickup);
    const demand_signed = gpa.alloc(i64, dim) catch return COMMIV_ERR_OUT_OF_MEMORY;
    defer gpa.free(demand_signed);

    pair_of[0] = 0;
    is_pickup[0] = false;
    demand_signed[0] = 0;
    for (0..n_pairs) |i| {
        const p = pick[i];
        const q = drop[i];
        pair_of[p] = q;
        pair_of[q] = p;
        is_pickup[p] = true;
        is_pickup[q] = false;
        demand_signed[p] = @as(i64, dem[i]);
        demand_signed[q] = -@as(i64, dem[i]);
    }

    const o = resolveOptions(options);
    const inst = pdptw.PdpInstance{
        .n_pairs = n_pairs,
        .matrix = m[0 .. dim * dim],
        .capacity = @intCast(capacity),
        .pair_of = pair_of,
        .is_pickup = is_pickup,
        .demand_signed = demand_signed,
        .ready = rdy[0..dim],
        .due = du[0..dim],
        .service = srv[0..dim],
    };

    var params: pdptw_sisr.PdpSisrParams = .{
        .seed = o.seed,
        .veh_penalty = o.veh_penalty,
        .time_penalty = o.time_penalty,
        .max_route_dur = o.max_route_duration,
    };
    if (o.sisr_iters != 0) params.iters = @intCast(o.sisr_iters);
    if (o.wall_ms != 0) params.time_ms = o.wall_ms;
    if (o.max_vehicles != 0) params.max_vehicles = @intCast(o.max_vehicles);
    if (o.break_duration != 0) {
        // v1: the fleet-min / pinned drivers don't understand breaks.
        if (o.fleet_min != 0 or o.max_vehicles != 0) return COMMIV_ERR_INVALID_ARGUMENT;
        params.brk = .{ .dur = o.break_duration, .earliest = o.break_earliest, .latest = o.break_latest };
    }
    // The wall-driven drivers (fleet-min, pinned) need a budget; default 10s.
    const budget: u64 = if (o.wall_ms != 0) o.wall_ms else 10_000;

    var result = blk: {
        if (o.fleet_min != 0) {
            break :blk pdptw_sisr.solvePdptwSisrFleetMin(gpa, inst, params, budget) catch |err| return mapError(err);
        }
        if (o.max_vehicles != 0) {
            break :blk pdptw_sisr.solvePdptwSisrPinned(gpa, inst, params, budget, @intCast(o.max_vehicles)) catch |err| return mapError(err);
        }
        break :blk pdptw_sisr.solvePdptwSisr(gpa, inst, params) catch |err| return mapError(err);
    };
    defer result.deinit();

    out_p.* = makeRoutes(result.routes, result.total_cost) catch |err| return mapError(err);
    return COMMIV_OK;
}

/// Heterogeneous-fleet PDPTW: commiv_solve_pdptw with a vehicle-type table
/// instead of one uniform capacity. veh_type_capacity / veh_type_fixed_cost /
/// veh_type_count are parallel arrays of n_types entries (1..8): each route in
/// the answer is served by exactly one type — its capacity bounds the route
/// load, its fixed cost replaces options->veh_penalty in the objective, and at
/// most `count` routes of a type run simultaneously (count 0 = unlimited).
/// Read the assignment back with commiv_routes_type. options->fleet_min and
/// options->max_vehicles are not supported with vehicle types (use per-type
/// counts to bound the fleet): COMMIV_ERR_INVALID_ARGUMENT.
export fn commiv_solve_pdptw_typed(
    matrix: ?[*]const u32,
    n_pairs: usize,
    pickup_node: ?[*]const u32,
    delivery_node: ?[*]const u32,
    demand: ?[*]const u32,
    ready: ?[*]const u32,
    due: ?[*]const u32,
    service: ?[*]const u32,
    veh_type_capacity: ?[*]const u32,
    veh_type_fixed_cost: ?[*]const u64,
    veh_type_count: ?[*]const u32,
    n_types: usize,
    options: ?*const CommivOptions,
    out: ?**CommivRoutes,
) c_int {
    const m = matrix orelse return COMMIV_ERR_INVALID_ARGUMENT;
    const pick = pickup_node orelse return COMMIV_ERR_INVALID_ARGUMENT;
    const drop = delivery_node orelse return COMMIV_ERR_INVALID_ARGUMENT;
    const dem = demand orelse return COMMIV_ERR_INVALID_ARGUMENT;
    const rdy = ready orelse return COMMIV_ERR_INVALID_ARGUMENT;
    const du = due orelse return COMMIV_ERR_INVALID_ARGUMENT;
    const srv = service orelse return COMMIV_ERR_INVALID_ARGUMENT;
    const tcap = veh_type_capacity orelse return COMMIV_ERR_INVALID_ARGUMENT;
    const tfix = veh_type_fixed_cost orelse return COMMIV_ERR_INVALID_ARGUMENT;
    const tcnt = veh_type_count orelse return COMMIV_ERR_INVALID_ARGUMENT;
    const out_p = out orelse return COMMIV_ERR_INVALID_ARGUMENT;

    if (n_types == 0 or n_types > pdptw_sisr.MAX_VEH_TYPES) return COMMIV_ERR_INVALID_ARGUMENT;
    var vt: [pdptw_sisr.MAX_VEH_TYPES]pdptw_sisr.VehType = undefined;
    var maxcap: u32 = 0;
    for (0..n_types) |i| {
        if (tcap[i] == 0) return COMMIV_ERR_INVALID_ARGUMENT;
        vt[i] = .{ .capacity = @intCast(tcap[i]), .fixed_cost = tfix[i], .count = tcnt[i] };
        maxcap = @max(maxcap, tcap[i]);
    }

    const o = resolveOptions(options);
    // Fleet-min / pinned drivers don't understand the type ledger (v1);
    // per-type counts are the supported way to bound a heterogeneous fleet.
    if (o.fleet_min != 0 or o.max_vehicles != 0) return COMMIV_ERR_INVALID_ARGUMENT;

    if (n_pairs == 0) return COMMIV_ERR_INVALID_ARGUMENT;
    if (n_pairs >= (std.math.maxInt(u32) - 1) / 2) return COMMIV_ERR_INVALID_ARGUMENT;
    const dim = 2 * n_pairs + 1;
    _ = std.math.mul(usize, dim, dim) catch return COMMIV_ERR_INVALID_ARGUMENT;

    const seen = gpa.alloc(bool, dim) catch return COMMIV_ERR_OUT_OF_MEMORY;
    defer gpa.free(seen);
    // maxcap plays the uniform capacity's role in the shape checks (a demand
    // no type can carry fast-fails as INFEASIBLE, same as the uniform door).
    const rc = checkPdpArgs(n_pairs, dim, pick, drop, dem, maxcap, rdy, srv, seen);
    if (rc != COMMIV_OK) return rc;

    const pair_of = gpa.alloc(usize, dim) catch return COMMIV_ERR_OUT_OF_MEMORY;
    defer gpa.free(pair_of);
    const is_pickup = gpa.alloc(bool, dim) catch return COMMIV_ERR_OUT_OF_MEMORY;
    defer gpa.free(is_pickup);
    const demand_signed = gpa.alloc(i64, dim) catch return COMMIV_ERR_OUT_OF_MEMORY;
    defer gpa.free(demand_signed);

    pair_of[0] = 0;
    is_pickup[0] = false;
    demand_signed[0] = 0;
    for (0..n_pairs) |i| {
        const p = pick[i];
        const q = drop[i];
        pair_of[p] = q;
        pair_of[q] = p;
        is_pickup[p] = true;
        is_pickup[q] = false;
        demand_signed[p] = @as(i64, dem[i]);
        demand_signed[q] = -@as(i64, dem[i]);
    }

    const inst = pdptw.PdpInstance{
        .n_pairs = n_pairs,
        .matrix = m[0 .. dim * dim],
        .capacity = @intCast(maxcap),
        .pair_of = pair_of,
        .is_pickup = is_pickup,
        .demand_signed = demand_signed,
        .ready = rdy[0..dim],
        .due = du[0..dim],
        .service = srv[0..dim],
    };

    var params: pdptw_sisr.PdpSisrParams = .{
        .seed = o.seed,
        .veh_penalty = o.veh_penalty,
        .time_penalty = o.time_penalty,
        .max_route_dur = o.max_route_duration,
        .veh_types = vt[0..n_types],
    };
    if (o.sisr_iters != 0) params.iters = @intCast(o.sisr_iters);
    if (o.wall_ms != 0) params.time_ms = o.wall_ms;
    if (o.break_duration != 0)
        params.brk = .{ .dur = o.break_duration, .earliest = o.break_earliest, .latest = o.break_latest };

    var result = pdptw_sisr.solvePdptwSisr(gpa, inst, params) catch |err| return mapError(err);
    defer result.deinit();

    out_p.* = makeRoutesTyped(result.routes, result.types, result.total_cost) catch |err| return mapError(err);
    return COMMIV_OK;
}

/// Validate the current-plan + locked-prefix contract for
/// commiv_solve_pdptw_dispatch. dim/pair_of/is_pickup describe the instance
/// (already validated by checkPdpArgs). `cur_offsets` has n_routes + 1
/// entries (route r = cur_nodes[cur_offsets[r]..cur_offsets[r+1]]), `locked`
/// has n_routes entries. `seen` is caller scratch, length dim. Only called
/// when n_routes > 0 (the n_routes == 0 cold-start case is trivially valid
/// and skips this entirely). Returns 0 when the plan is usable, a
/// COMMIV_ERR_* otherwise.
fn checkDispatchArgs(
    dim: usize,
    pair_of: []const usize,
    is_pickup: []const bool,
    n_routes: usize,
    cur_offsets: [*]const usize,
    cur_nodes: ?[*]const u32,
    locked: [*]const usize,
    seen: []bool,
) c_int {
    if (cur_offsets[0] != 0) return COMMIV_ERR_INVALID_ARGUMENT;
    for (0..n_routes) |i| {
        if (cur_offsets[i + 1] < cur_offsets[i]) return COMMIV_ERR_INVALID_ARGUMENT;
        if (locked[i] > cur_offsets[i + 1] - cur_offsets[i]) return COMMIV_ERR_INVALID_ARGUMENT;
    }
    const total = cur_offsets[n_routes];
    if (total > 0 and cur_nodes == null) return COMMIV_ERR_INVALID_ARGUMENT;
    const nodes: []const u32 = if (total > 0) cur_nodes.?[0..total] else &[_]u32{};

    // Every node absent from `nodes` is fine (unrouted / new); every node
    // present must be a real customer index and appear at most once total.
    @memset(seen, false);
    for (nodes) |c| {
        if (c == 0 or c >= dim or seen[c]) return COMMIV_ERR_INVALID_ARGUMENT;
        seen[c] = true;
    }
    // Locked-pair invariant: a locked delivery's pickup must also be locked,
    // in the SAME route's prefix. The engine only std.debug.asserts this
    // (compiled out under ReleaseFast), so the C boundary must catch it.
    for (0..n_routes) |i| {
        const route = nodes[cur_offsets[i]..cur_offsets[i + 1]];
        const lk = locked[i];
        for (route[0..lk]) |c| {
            if (is_pickup[c]) continue;
            var found = false;
            for (route[0..lk]) |x| {
                if (x == pair_of[c]) {
                    found = true;
                    break;
                }
            }
            if (!found) return COMMIV_ERR_INVALID_ARGUMENT;
        }
    }
    return COMMIV_OK;
}

/// Rolling-horizon PDPTW re-solve: keeps the instance contract of
/// commiv_solve_pdptw and adds the CURRENT plan plus its LOCKED prefixes.
/// Route r of the current plan is cur_nodes[cur_offsets[r]..cur_offsets[r+1]]
/// (n_routes + 1 offsets, monotone non-decreasing, cur_offsets[0] == 0);
/// locked[r] is how many of route r's leading stops are committed (in
/// progress or already served) and must not move. A locked delivery's pickup
/// must also be locked, in the same route (COMMIV_ERR_INVALID_ARGUMENT
/// otherwise). n_routes == 0 is a legal cold start (dispatch from an empty
/// plan): cur_offsets/cur_nodes/locked are unused then and may be NULL.
///
/// Node-id stability: when new orders arrive, rebuild a LARGER instance that
/// keeps every existing node's index unchanged and appends the new pairs at
/// the end — the old routes stay valid warm input for the next call.
///
/// options->fleet_min and options->max_vehicles are IGNORED here: dispatch
/// keeps the current fleet shape (routes open/close only as ruin-and-recreate
/// naturally does around the locked prefixes). Every other option (seed,
/// sisr_iters, veh_penalty, time_penalty, wall_ms) behaves as in
/// commiv_solve_pdptw.
export fn commiv_solve_pdptw_dispatch(
    matrix: ?[*]const u32,
    n_pairs: usize,
    pickup_node: ?[*]const u32,
    delivery_node: ?[*]const u32,
    demand: ?[*]const u32,
    capacity: u32,
    ready: ?[*]const u32,
    due: ?[*]const u32,
    service: ?[*]const u32,
    cur_offsets: ?[*]const usize,
    n_routes: usize,
    cur_nodes: ?[*]const u32,
    locked: ?[*]const usize,
    options: ?*const CommivOptions,
    out: ?**CommivRoutes,
) c_int {
    const m = matrix orelse return COMMIV_ERR_INVALID_ARGUMENT;
    const pick = pickup_node orelse return COMMIV_ERR_INVALID_ARGUMENT;
    const drop = delivery_node orelse return COMMIV_ERR_INVALID_ARGUMENT;
    const dem = demand orelse return COMMIV_ERR_INVALID_ARGUMENT;
    const rdy = ready orelse return COMMIV_ERR_INVALID_ARGUMENT;
    const du = due orelse return COMMIV_ERR_INVALID_ARGUMENT;
    const srv = service orelse return COMMIV_ERR_INVALID_ARGUMENT;
    const out_p = out orelse return COMMIV_ERR_INVALID_ARGUMENT;
    if (n_routes > 0 and (cur_offsets == null or locked == null)) return COMMIV_ERR_INVALID_ARGUMENT;

    if (n_pairs == 0 or capacity == 0) return COMMIV_ERR_INVALID_ARGUMENT;
    // Keep node ids in u32 and dim = 2*n_pairs+1 from overflowing.
    if (n_pairs >= (std.math.maxInt(u32) - 1) / 2) return COMMIV_ERR_INVALID_ARGUMENT;
    const dim = 2 * n_pairs + 1;
    _ = std.math.mul(usize, dim, dim) catch return COMMIV_ERR_INVALID_ARGUMENT;

    const seen = gpa.alloc(bool, dim) catch return COMMIV_ERR_OUT_OF_MEMORY;
    defer gpa.free(seen);
    const rc = checkPdpArgs(n_pairs, dim, pick, drop, dem, capacity, rdy, srv, seen);
    if (rc != COMMIV_OK) return rc;

    const pair_of = gpa.alloc(usize, dim) catch return COMMIV_ERR_OUT_OF_MEMORY;
    defer gpa.free(pair_of);
    const is_pickup = gpa.alloc(bool, dim) catch return COMMIV_ERR_OUT_OF_MEMORY;
    defer gpa.free(is_pickup);
    const demand_signed = gpa.alloc(i64, dim) catch return COMMIV_ERR_OUT_OF_MEMORY;
    defer gpa.free(demand_signed);

    pair_of[0] = 0;
    is_pickup[0] = false;
    demand_signed[0] = 0;
    for (0..n_pairs) |i| {
        const p = pick[i];
        const q = drop[i];
        pair_of[p] = q;
        pair_of[q] = p;
        is_pickup[p] = true;
        is_pickup[q] = false;
        demand_signed[p] = @as(i64, dem[i]);
        demand_signed[q] = -@as(i64, dem[i]);
    }

    // Dispatch-specific: validate the current plan + the locked-prefix contract.
    if (n_routes > 0) {
        const plan_seen = gpa.alloc(bool, dim) catch return COMMIV_ERR_OUT_OF_MEMORY;
        defer gpa.free(plan_seen);
        const rc2 = checkDispatchArgs(dim, pair_of, is_pickup, n_routes, cur_offsets.?, cur_nodes, locked.?, plan_seen);
        if (rc2 != COMMIV_OK) return rc2;
    }

    const o = resolveOptions(options);
    const inst = pdptw.PdpInstance{
        .n_pairs = n_pairs,
        .matrix = m[0 .. dim * dim],
        .capacity = @intCast(capacity),
        .pair_of = pair_of,
        .is_pickup = is_pickup,
        .demand_signed = demand_signed,
        .ready = rdy[0..dim],
        .due = du[0..dim],
        .service = srv[0..dim],
    };

    var params: pdptw_sisr.PdpSisrParams = .{
        .seed = o.seed,
        .veh_penalty = o.veh_penalty,
        .time_penalty = o.time_penalty,
        .max_route_dur = o.max_route_duration,
    };
    if (o.sisr_iters != 0) params.iters = @intCast(o.sisr_iters);
    if (o.wall_ms != 0) params.time_ms = o.wall_ms;
    if (o.break_duration != 0)
        params.brk = .{ .dur = o.break_duration, .earliest = o.break_earliest, .latest = o.break_latest };

    // Marshal the current plan into the usize node slices the engine wants:
    // cur_nodes is u32 but the engine indexes routes with usize, so copy into
    // one flat buffer and point each route's slice into it.
    const total: usize = if (n_routes > 0) cur_offsets.?[n_routes] else 0;
    const node_buf = gpa.alloc(usize, total) catch return COMMIV_ERR_OUT_OF_MEMORY;
    defer gpa.free(node_buf);
    const route_slices = gpa.alloc([]const usize, n_routes) catch return COMMIV_ERR_OUT_OF_MEMORY;
    defer gpa.free(route_slices);
    const locked_slice = gpa.alloc(usize, n_routes) catch return COMMIV_ERR_OUT_OF_MEMORY;
    defer gpa.free(locked_slice);
    if (n_routes > 0) {
        const offs = cur_offsets.?;
        const lks = locked.?;
        if (total > 0) {
            const nodes = cur_nodes.?[0..total];
            for (nodes, 0..) |c, i| node_buf[i] = c;
        }
        for (0..n_routes) |i| {
            route_slices[i] = node_buf[offs[i]..offs[i + 1]];
            locked_slice[i] = lks[i];
        }
    }

    var result = pdptw_sisr.solvePdptwSisrDispatch(gpa, inst, params, route_slices, locked_slice) catch |err| return mapError(err);
    defer result.deinit();

    out_p.* = makeRoutes(result.routes, result.total_cost) catch |err| return mapError(err);
    return COMMIV_OK;
}

/// Directed TSP (ATSP). matrix is n*n row-major, matrix[i*n+j] = cost i->j.
/// out_tour must hold n entries; receives the visit order (a permutation of
/// 0..n-1). out_cost receives the directed tour length.
export fn commiv_solve_atsp(
    matrix: ?[*]const u32,
    n: usize,
    options: ?*const CommivOptions,
    out_tour: ?[*]u32,
    out_cost: ?*u64,
) c_int {
    const m = matrix orelse return COMMIV_ERR_INVALID_ARGUMENT;
    const tour_p = out_tour orelse return COMMIV_ERR_INVALID_ARGUMENT;
    const cost_p = out_cost orelse return COMMIV_ERR_INVALID_ARGUMENT;
    if (n == 0 or n >= std.math.maxInt(u32)) return COMMIV_ERR_INVALID_ARGUMENT;
    _ = std.math.mul(usize, n, n) catch return COMMIV_ERR_INVALID_ARGUMENT;
    if (n == 1) {
        tour_p[0] = 0;
        cost_p.* = 0;
        return COMMIV_OK;
    }

    const o = resolveOptions(options);
    var solve_options: solver.SolveOptions = .{ .seed = o.seed };
    if (o.trials != 0) solve_options.budget.trials = @intCast(o.trials);

    var result = asymmetric.solveAtsp(gpa, m[0 .. n * n], n, solve_options) catch |err| return mapError(err);
    defer result.deinit();

    for (result.tour, 0..) |node, i| tour_p[i] = @intCast(node);
    cost_p.* = result.length;
    return COMMIV_OK;
}

export fn commiv_routes_cost(r: ?*const CommivRoutes) u64 {
    const h = r orelse return 0;
    return h.total_cost;
}

export fn commiv_routes_count(r: ?*const CommivRoutes) usize {
    const h = r orelse return 0;
    return h.offsets.len - 1;
}

export fn commiv_routes_len(r: ?*const CommivRoutes, route: usize) usize {
    const h = r orelse return 0;
    if (route + 1 >= h.offsets.len) return 0;
    return h.offsets[route + 1] - h.offsets[route];
}

/// Pointer to route's customer indices (valid until commiv_routes_free).
/// NULL if the route index is out of range.
export fn commiv_routes_get(r: ?*const CommivRoutes, route: usize) ?[*]const u32 {
    const h = r orelse return null;
    if (route + 1 >= h.offsets.len) return null;
    return h.nodes[h.offsets[route]..].ptr;
}

/// Vehicle-type index of `route` (index into the veh_type_* arrays passed to
/// commiv_solve_pdptw_typed). Returns COMMIV_NO_TYPE (UINT32_MAX) for solves
/// without vehicle types or an out-of-range route index.
export fn commiv_routes_type(r: ?*const CommivRoutes, route: usize) u32 {
    const h = r orelse return std.math.maxInt(u32);
    if (route >= h.types.len) return std.math.maxInt(u32);
    return h.types[route];
}

export fn commiv_routes_free(r: ?*CommivRoutes) void {
    const h = r orelse return;
    if (h.types.len != 0) gpa.free(h.types);
    gpa.free(h.nodes);
    gpa.free(h.offsets);
    gpa.destroy(h);
}

// ---- tests -------------------------------------------------------------------

const testing = std.testing;

// The README/quickstart instance: 3 customers, directed costs, capacity 10.
const test_matrix = [_]u32{
    0,  10, 14, 12,
    11, 0,  9,  20,
    15, 8,  0,  7,
    13, 18, 6,  0,
};
const test_demand = [_]u32{ 0, 4, 6, 5 };

test "capi cvrp solves the quickstart instance and routes are consistent" {
    var routes: *CommivRoutes = undefined;
    const rc = commiv_solve_cvrp(&test_matrix, 3, &test_demand, 10, null, &routes);
    try testing.expectEqual(COMMIV_OK, rc);
    defer commiv_routes_free(routes);

    try testing.expect(commiv_routes_cost(routes) > 0);
    const k = commiv_routes_count(routes);
    try testing.expect(k >= 2); // demands 4+6+5 over capacity 10 need >= 2 vehicles

    // Every customer exactly once, capacity respected, cost re-adds correctly.
    var seen = [_]bool{false} ** 4;
    var recomputed: u64 = 0;
    for (0..k) |i| {
        const len = commiv_routes_len(routes, i);
        const nodes = commiv_routes_get(routes, i).?;
        var load: u64 = 0;
        var prev: usize = 0;
        for (nodes[0..len]) |c| {
            try testing.expect(c >= 1 and c <= 3);
            try testing.expect(!seen[c]);
            seen[c] = true;
            load += test_demand[c];
            recomputed += test_matrix[prev * 4 + c];
            prev = c;
        }
        recomputed += test_matrix[prev * 4]; // back to depot
        try testing.expect(load <= 10);
    }
    for (seen[1..]) |s| try testing.expect(s);
    try testing.expectEqual(commiv_routes_cost(routes), recomputed);
}

test "capi cvrp rejects bad arguments and impossible demand" {
    var routes: *CommivRoutes = undefined;
    try testing.expectEqual(
        COMMIV_ERR_INVALID_ARGUMENT,
        commiv_solve_cvrp(null, 3, &test_demand, 10, null, &routes),
    );
    const heavy = [_]u32{ 0, 4, 11, 5 }; // customer 2 exceeds capacity 10
    try testing.expectEqual(
        COMMIV_ERR_INFEASIBLE,
        commiv_solve_cvrp(&test_matrix, 3, &heavy, 10, null, &routes),
    );
}

test "capi atsp returns a permutation with the directed cost" {
    const m = [_]u32{
        0, 1, 9, 9,
        9, 0, 1, 9,
        9, 9, 0, 1,
        1, 9, 9, 0,
    };
    var tour: [4]u32 = undefined;
    var cost: u64 = 0;
    const rc = commiv_solve_atsp(&m, 4, null, &tour, &cost);
    try testing.expectEqual(COMMIV_OK, rc);
    try testing.expectEqual(@as(u64, 4), cost); // the directed ring 0->1->2->3->0
    var seen = [_]bool{false} ** 4;
    for (tour) |t| {
        try testing.expect(t < 4);
        seen[t] = true;
    }
    for (seen) |s| try testing.expect(s);
}

test "capi vrptw honors windows and returns feasible routes" {
    // Generous windows so the instance is trivially schedulable.
    const ready = [_]u32{ 0, 0, 0, 0 };
    const due = [_]u32{ 1000, 500, 500, 500 };
    const service = [_]u32{ 0, 5, 5, 5 };
    var routes: *CommivRoutes = undefined;
    var opts: CommivOptions = .{ .seed = 7 };
    const rc = commiv_solve_vrptw(&test_matrix, 3, &test_demand, 10, &ready, &due, &service, &opts, &routes);
    try testing.expectEqual(COMMIV_OK, rc);
    defer commiv_routes_free(routes);
    try testing.expect(commiv_routes_cost(routes) > 0);

    // Independent check through the library's own validator.
    var slices: [4][]const usize = undefined;
    var storage: [4][3]usize = undefined;
    const k = commiv_routes_count(routes);
    for (0..k) |i| {
        const len = commiv_routes_len(routes, i);
        const nodes = commiv_routes_get(routes, i).?;
        for (nodes[0..len], 0..) |c, j| storage[i][j] = c;
        slices[i] = storage[i][0..len];
    }
    const inst = vrptw.VrptwInstance{
        .n = 3,
        .matrix = &test_matrix,
        .demand = &test_demand,
        .capacity = 10,
        .ready = &ready,
        .due = &due,
        .service = &service,
    };
    const validated = vrptw.validate(inst, slices[0..k]);
    try testing.expect(validated != null);
    try testing.expectEqual(commiv_routes_cost(routes), validated.?);
}

// A tiny 2-pair PDPTW: dim=5, depot 0. Requests (1->3) and (2->4).
const pdp_matrix = [_]u32{
    0,  10, 12, 14, 16,
    10, 0,  6,  8,  10,
    12, 6,  0,  7,  9,
    14, 8,  7,  0,  6,
    16, 10, 9,  6,  0,
};
const pdp_pickup = [_]u32{ 1, 2 };
const pdp_delivery = [_]u32{ 3, 4 };
const pdp_demand = [_]u32{ 4, 5 };
const pdp_ready = [_]u32{ 0, 0, 0, 0, 0 };
const pdp_due = [_]u32{ 10_000, 10_000, 10_000, 10_000, 10_000 };
const pdp_service = [_]u32{ 0, 3, 3, 3, 3 };

// Rebuild the PdpInstance the C entry synthesizes, for the validator oracle.
fn buildPdpTestInstance(
    pair_of: *[5]usize,
    is_pickup: *[5]bool,
    demand_signed: *[5]i64,
) pdptw.PdpInstance {
    pair_of[0] = 0;
    is_pickup[0] = false;
    demand_signed[0] = 0;
    for (0..2) |i| {
        const p = pdp_pickup[i];
        const q = pdp_delivery[i];
        pair_of[p] = q;
        pair_of[q] = p;
        is_pickup[p] = true;
        is_pickup[q] = false;
        demand_signed[p] = @as(i64, pdp_demand[i]);
        demand_signed[q] = -@as(i64, pdp_demand[i]);
    }
    return .{
        .n_pairs = 2,
        .matrix = &pdp_matrix,
        .capacity = 10,
        .pair_of = pair_of,
        .is_pickup = is_pickup,
        .demand_signed = demand_signed,
        .ready = &pdp_ready,
        .due = &pdp_due,
        .service = &pdp_service,
    };
}

test "capi pdptw solves a tiny instance and validates against the oracle" {
    var routes: *CommivRoutes = undefined;
    var opts: CommivOptions = .{ .seed = 5, .sisr_iters = 2000 };
    const rc = commiv_solve_pdptw(&pdp_matrix, 2, &pdp_pickup, &pdp_delivery, &pdp_demand, 10, &pdp_ready, &pdp_due, &pdp_service, &opts, &routes);
    try testing.expectEqual(COMMIV_OK, rc);
    defer commiv_routes_free(routes);
    try testing.expect(commiv_routes_cost(routes) > 0);

    // Reconstruct route slices and check with the engine's own validator.
    var slices: [5][]const usize = undefined;
    var storage: [5][4]usize = undefined;
    const k = commiv_routes_count(routes);
    for (0..k) |i| {
        const len = commiv_routes_len(routes, i);
        const nodes = commiv_routes_get(routes, i).?;
        for (nodes[0..len], 0..) |c, j| storage[i][j] = c;
        slices[i] = storage[i][0..len];
    }
    var pair_of: [5]usize = undefined;
    var is_pickup: [5]bool = undefined;
    var demand_signed: [5]i64 = undefined;
    const inst = buildPdpTestInstance(&pair_of, &is_pickup, &demand_signed);
    const validated = pdptw.validate(inst, slices[0..k]);
    try testing.expect(validated != null);
    try testing.expectEqual(commiv_routes_cost(routes), validated.?);
}

test "capi pdptw money objective still returns a feasible complete plan" {
    var routes: *CommivRoutes = undefined;
    var opts: CommivOptions = .{ .seed = 5, .sisr_iters = 2000, .time_penalty = 3 };
    const rc = commiv_solve_pdptw(&pdp_matrix, 2, &pdp_pickup, &pdp_delivery, &pdp_demand, 10, &pdp_ready, &pdp_due, &pdp_service, &opts, &routes);
    try testing.expectEqual(COMMIV_OK, rc);
    defer commiv_routes_free(routes);

    var slices: [5][]const usize = undefined;
    var storage: [5][4]usize = undefined;
    const k = commiv_routes_count(routes);
    for (0..k) |i| {
        const len = commiv_routes_len(routes, i);
        const nodes = commiv_routes_get(routes, i).?;
        for (nodes[0..len], 0..) |c, j| storage[i][j] = c;
        slices[i] = storage[i][0..len];
    }
    var pair_of: [5]usize = undefined;
    var is_pickup: [5]bool = undefined;
    var demand_signed: [5]i64 = undefined;
    const inst = buildPdpTestInstance(&pair_of, &is_pickup, &demand_signed);
    // The money objective changes reported cost, but the plan must stay a
    // complete, feasible PDPTW solution (distance-validated by the oracle).
    try testing.expect(pdptw.validate(inst, slices[0..k]) != null);
}

test "capi pdptw rejects impossible demand and self-paired requests" {
    var routes: *CommivRoutes = undefined;
    // demand[0]=11 > capacity 10 -> infeasible.
    const heavy = [_]u32{ 11, 5 };
    try testing.expectEqual(
        COMMIV_ERR_INFEASIBLE,
        commiv_solve_pdptw(&pdp_matrix, 2, &pdp_pickup, &pdp_delivery, &heavy, 10, &pdp_ready, &pdp_due, &pdp_service, null, &routes),
    );
    // delivery_node[0] == pickup_node[0] -> invalid argument.
    const bad_delivery = [_]u32{ 1, 4 };
    try testing.expectEqual(
        COMMIV_ERR_INVALID_ARGUMENT,
        commiv_solve_pdptw(&pdp_matrix, 2, &pdp_pickup, &bad_delivery, &pdp_demand, 10, &pdp_ready, &pdp_due, &pdp_service, null, &routes),
    );
}

test "capi pdptw dispatch preserves a locked prefix and validates" {
    var routes: *CommivRoutes = undefined;
    var opts: CommivOptions = .{ .seed = 5, .sisr_iters = 2000 };
    const rc = commiv_solve_pdptw(&pdp_matrix, 2, &pdp_pickup, &pdp_delivery, &pdp_demand, 10, &pdp_ready, &pdp_due, &pdp_service, &opts, &routes);
    try testing.expectEqual(COMMIV_OK, rc);
    defer commiv_routes_free(routes);

    const k = commiv_routes_count(routes);
    var cur_offsets: [8]usize = undefined;
    var cur_nodes: [4]u32 = undefined;
    var locked: [8]usize = [_]usize{0} ** 8;
    var at: usize = 0;
    for (0..k) |i| {
        cur_offsets[i] = at;
        const len = commiv_routes_len(routes, i);
        const nodes = commiv_routes_get(routes, i).?;
        for (nodes[0..len], 0..) |c, j| cur_nodes[at + j] = c;
        at += len;
    }
    cur_offsets[k] = at;

    // Lock the ENTIRE first nonempty route: any complete route is
    // pair-closed by construction (a pair never spans two routes), so
    // locking all of it always satisfies the locked-pair invariant.
    var lock_route: usize = k;
    for (0..k) |i| {
        if (cur_offsets[i + 1] > cur_offsets[i]) {
            lock_route = i;
            break;
        }
    }
    try testing.expect(lock_route < k);
    locked[lock_route] = cur_offsets[lock_route + 1] - cur_offsets[lock_route];
    const locked_len = locked[lock_route];
    const locked_prefix = cur_nodes[cur_offsets[lock_route]..cur_offsets[lock_route + 1]];

    var dispatch_routes: *CommivRoutes = undefined;
    var dopts: CommivOptions = .{ .seed = 9, .sisr_iters = 3000 };
    const drc = commiv_solve_pdptw_dispatch(
        &pdp_matrix,
        2,
        &pdp_pickup,
        &pdp_delivery,
        &pdp_demand,
        10,
        &pdp_ready,
        &pdp_due,
        &pdp_service,
        &cur_offsets,
        k,
        &cur_nodes,
        &locked,
        &dopts,
        &dispatch_routes,
    );
    try testing.expectEqual(COMMIV_OK, drc);
    defer commiv_routes_free(dispatch_routes);

    const dk = commiv_routes_count(dispatch_routes);
    var found = false;
    for (0..dk) |i| {
        const len = commiv_routes_len(dispatch_routes, i);
        if (len < locked_len) continue;
        const nodes = commiv_routes_get(dispatch_routes, i).?;
        if (std.mem.eql(u32, nodes[0..locked_len], locked_prefix)) found = true;
    }
    try testing.expect(found);

    // Independent full-solution feasibility + cost check via the validator.
    var slices: [5][]const usize = undefined;
    var storage: [5][4]usize = undefined;
    for (0..dk) |i| {
        const len = commiv_routes_len(dispatch_routes, i);
        const nodes = commiv_routes_get(dispatch_routes, i).?;
        for (nodes[0..len], 0..) |c, j| storage[i][j] = c;
        slices[i] = storage[i][0..len];
    }
    var pair_of: [5]usize = undefined;
    var is_pickup: [5]bool = undefined;
    var demand_signed: [5]i64 = undefined;
    const inst = buildPdpTestInstance(&pair_of, &is_pickup, &demand_signed);
    try testing.expect(pdptw.validate(inst, slices[0..dk]) != null);
}

test "capi pdptw dispatch rejects a locked delivery whose pickup is not locked" {
    // route0 = [3, 1] (delivery then its pickup), route1 = [2, 4].
    const cur_nodes = [_]u32{ 3, 1, 2, 4 };
    const cur_offsets = [_]usize{ 0, 2, 4 };
    const locked = [_]usize{ 1, 0 }; // locks only node 3 (the delivery) in route0
    var out: *CommivRoutes = undefined;
    const rc = commiv_solve_pdptw_dispatch(
        &pdp_matrix, 2, &pdp_pickup, &pdp_delivery, &pdp_demand, 10, &pdp_ready, &pdp_due, &pdp_service,
        &cur_offsets, 2, &cur_nodes, &locked, null, &out,
    );
    try testing.expectEqual(COMMIV_ERR_INVALID_ARGUMENT, rc);
}

test "capi pdptw dispatch rejects non-monotone offsets" {
    const cur_nodes = [_]u32{ 1, 3, 2, 4 };
    const cur_offsets = [_]usize{ 0, 3, 2 }; // decreases: 3 -> 2
    const locked = [_]usize{ 0, 0 };
    var out: *CommivRoutes = undefined;
    const rc = commiv_solve_pdptw_dispatch(
        &pdp_matrix, 2, &pdp_pickup, &pdp_delivery, &pdp_demand, 10, &pdp_ready, &pdp_due, &pdp_service,
        &cur_offsets, 2, &cur_nodes, &locked, null, &out,
    );
    try testing.expectEqual(COMMIV_ERR_INVALID_ARGUMENT, rc);
}

test "capi pdptw dispatch rejects a node duplicated across routes" {
    const cur_nodes = [_]u32{ 1, 3, 1, 4 }; // node 1 appears in both routes
    const cur_offsets = [_]usize{ 0, 2, 4 };
    const locked = [_]usize{ 0, 0 };
    var out: *CommivRoutes = undefined;
    const rc = commiv_solve_pdptw_dispatch(
        &pdp_matrix, 2, &pdp_pickup, &pdp_delivery, &pdp_demand, 10, &pdp_ready, &pdp_due, &pdp_service,
        &cur_offsets, 2, &cur_nodes, &locked, null, &out,
    );
    try testing.expectEqual(COMMIV_ERR_INVALID_ARGUMENT, rc);
}

test "capi pdptw C path is bit-identical to the internal engine" {
    // The M1 acceptance contract: the C shim (instance synthesis + param
    // mapping) adds ZERO divergence versus calling the engine directly with
    // the same seed and params — including under the money objective.
    var opts: CommivOptions = .{ .seed = 5, .sisr_iters = 2000, .time_penalty = 3, .veh_penalty = 100 };
    var routes: *CommivRoutes = undefined;
    const rc = commiv_solve_pdptw(&pdp_matrix, 2, &pdp_pickup, &pdp_delivery, &pdp_demand, 10, &pdp_ready, &pdp_due, &pdp_service, &opts, &routes);
    try testing.expectEqual(COMMIV_OK, rc);
    defer commiv_routes_free(routes);

    var pair_of: [5]usize = undefined;
    var is_pickup: [5]bool = undefined;
    var demand_signed: [5]i64 = undefined;
    const inst = buildPdpTestInstance(&pair_of, &is_pickup, &demand_signed);
    var direct = try pdptw_sisr.solvePdptwSisr(testing.allocator, inst, .{
        .seed = 5,
        .iters = 2000,
        .time_penalty = 3,
        .veh_penalty = 100,
    });
    defer direct.deinit();

    try testing.expectEqual(direct.total_cost, commiv_routes_cost(routes));
    try testing.expectEqual(direct.routes.len, commiv_routes_count(routes));
    for (direct.routes, 0..) |r, i| {
        const len = commiv_routes_len(routes, i);
        try testing.expectEqual(r.len, len);
        const nodes = commiv_routes_get(routes, i).?;
        for (r, nodes[0..len]) |a, b| try testing.expectEqual(a, @as(usize, b));
    }
}

test "capi pdptw typed: per-type cap enforced, types readable" {
    var routes: *CommivRoutes = undefined;
    var opts: CommivOptions = .{ .seed = 3, .sisr_iters = 2000 };
    // One type of capacity 5 (< the uniform 10): loads 4 and 5 can share a
    // route only interleaved (peak 5), never nested (peak 9).
    const tcap = [_]u32{5};
    const tfix = [_]u64{10};
    const tcnt = [_]u32{0};
    const rc = commiv_solve_pdptw_typed(&pdp_matrix, 2, &pdp_pickup, &pdp_delivery, &pdp_demand, &pdp_ready, &pdp_due, &pdp_service, &tcap, &tfix, &tcnt, 1, &opts, &routes);
    try testing.expectEqual(COMMIV_OK, rc);
    defer commiv_routes_free(routes);
    const k = commiv_routes_count(routes);
    try testing.expect(k >= 1);
    // Independent recompute: every prefix load within the TYPE cap (5), all
    // 4 nodes served, pickup precedes delivery.
    var pair_of: [5]usize = undefined;
    var is_pickup: [5]bool = undefined;
    var demand_signed: [5]i64 = undefined;
    _ = buildPdpTestInstance(&pair_of, &is_pickup, &demand_signed);
    var served: usize = 0;
    for (0..k) |i| {
        try testing.expectEqual(@as(u32, 0), commiv_routes_type(routes, i));
        const len = commiv_routes_len(routes, i);
        const nodes = commiv_routes_get(routes, i).?;
        var load: i64 = 0;
        for (nodes[0..len]) |c| {
            load += demand_signed[c];
            try testing.expect(load >= 0 and load <= 5);
            served += 1;
        }
    }
    try testing.expectEqual(@as(usize, 4), served);
    try testing.expectEqual(std.math.maxInt(u32), commiv_routes_type(routes, k));
}

test "capi pdptw typed: rejects fleet drivers and impossible demand" {
    var routes: *CommivRoutes = undefined;
    const tcap = [_]u32{5};
    const tfix = [_]u64{10};
    const tcnt = [_]u32{0};
    var opts: CommivOptions = .{ .fleet_min = 1 };
    try testing.expectEqual(
        COMMIV_ERR_INVALID_ARGUMENT,
        commiv_solve_pdptw_typed(&pdp_matrix, 2, &pdp_pickup, &pdp_delivery, &pdp_demand, &pdp_ready, &pdp_due, &pdp_service, &tcap, &tfix, &tcnt, 1, &opts, &routes),
    );
    // No type carries the 5-demand pair.
    const small = [_]u32{4};
    try testing.expectEqual(
        COMMIV_ERR_INFEASIBLE,
        commiv_solve_pdptw_typed(&pdp_matrix, 2, &pdp_pickup, &pdp_delivery, &pdp_demand, &pdp_ready, &pdp_due, &pdp_service, &small, &tfix, &tcnt, 1, null, &routes),
    );
    // Untyped solves answer COMMIV_NO_TYPE through the accessor.
    var opts2: CommivOptions = .{ .seed = 3 };
    var plain: *CommivRoutes = undefined;
    try testing.expectEqual(COMMIV_OK, commiv_solve_pdptw(&pdp_matrix, 2, &pdp_pickup, &pdp_delivery, &pdp_demand, 10, &pdp_ready, &pdp_due, &pdp_service, &opts2, &plain));
    defer commiv_routes_free(plain);
    try testing.expectEqual(std.math.maxInt(u32), commiv_routes_type(plain, 0));
}

// ---- ABI stability canaries (docs/abi.md) ----------------------------------
// commiv_options is the append-only ABI contract: size and every field offset
// are pinned so an accidental reorder, retype, or non-appended insertion
// fails the test suite instead of shipping a silent ABI break.

test "capi ABI: commiv_options layout is pinned (88 bytes, append-only)" {
    try testing.expectEqual(@as(usize, 88), @sizeOf(CommivOptions));
    try testing.expectEqual(@as(usize, 0), @offsetOf(CommivOptions, "seed"));
    try testing.expectEqual(@as(usize, 8), @offsetOf(CommivOptions, "sisr_iters"));
    try testing.expectEqual(@as(usize, 16), @offsetOf(CommivOptions, "trials"));
    try testing.expectEqual(@as(usize, 24), @offsetOf(CommivOptions, "veh_penalty"));
    try testing.expectEqual(@as(usize, 32), @offsetOf(CommivOptions, "threads"));
    try testing.expectEqual(@as(usize, 36), @offsetOf(CommivOptions, "fleet_min"));
    try testing.expectEqual(@as(usize, 40), @offsetOf(CommivOptions, "wall_ms"));
    try testing.expectEqual(@as(usize, 48), @offsetOf(CommivOptions, "max_vehicles"));
    try testing.expectEqual(@as(usize, 56), @offsetOf(CommivOptions, "time_penalty"));
    try testing.expectEqual(@as(usize, 64), @offsetOf(CommivOptions, "max_route_duration"));
    try testing.expectEqual(@as(usize, 72), @offsetOf(CommivOptions, "break_duration"));
    try testing.expectEqual(@as(usize, 76), @offsetOf(CommivOptions, "break_earliest"));
    try testing.expectEqual(@as(usize, 80), @offsetOf(CommivOptions, "break_latest"));
    try testing.expectEqual(@as(usize, 84), @offsetOf(CommivOptions, "reserved"));
    // Zero value = documented defaults: the append-only contract's foundation.
    const zeroed = std.mem.zeroes(CommivOptions);
    try testing.expectEqual(@as(u64, 0), zeroed.seed);
    try testing.expectEqual(@as(u32, 0), zeroed.break_duration);
}
