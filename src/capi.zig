const std = @import("std");
const solver = @import("solver.zig");
const vrp = @import("vrp.zig");
const vrptw = @import("vrptw.zig");
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

const version_string = "0.2.0";

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
///   vrptw_rounds    Setting either of these selects the legacy VRPTW ILS engine
///   vrptw_restarts  instead of the default SISR; 0/0 -> SISR.
///   veh_penalty     VRPTW per-route penalty biasing toward fewer vehicles.
///   threads         0 or 1 = single-threaded (deterministic); >1 = parallel
///                   SISR chains (CVRP and VRPTW; result depends on the count).
pub const CommivOptions = extern struct {
    seed: u64 = 0,
    sisr_iters: u64 = 0,
    trials: u64 = 0,
    vrptw_rounds: u64 = 0,
    vrptw_restarts: u64 = 0,
    veh_penalty: u64 = 0,
    threads: u32 = 0,
    reserved: u32 = 0, // keep the struct 8-aligned; must be zero
};

/// Opaque to C; accessed through commiv_routes_* only. Routes are flattened:
/// route i is nodes[offsets[i]..offsets[i+1]], customer indices 1..n.
pub const CommivRoutes = struct {
    total_cost: u64,
    offsets: []usize, // len = route count + 1
    nodes: []u32,
};

fn resolveOptions(opt: ?*const CommivOptions) CommivOptions {
    var o: CommivOptions = if (opt) |p| p.* else .{};
    if (o.seed == 0) o.seed = 1;
    return o;
}

fn mapError(err: anyerror) c_int {
    return switch (err) {
        error.OutOfMemory => COMMIV_ERR_OUT_OF_MEMORY,
        error.Infeasible, error.NoFeasibleSplit => COMMIV_ERR_INFEASIBLE,
        error.InvalidInstance, error.InvalidMatrix, error.InstanceTooLargeForTransform => COMMIV_ERR_INVALID_ARGUMENT,
        else => COMMIV_ERR_INTERNAL,
    };
}

fn makeRoutes(routes: []const []const usize, total_cost: u64) !*CommivRoutes {
    var total_nodes: usize = 0;
    for (routes) |r| total_nodes += r.len;

    const h = try gpa.create(CommivRoutes);
    errdefer gpa.destroy(h);
    const offsets = try gpa.alloc(usize, routes.len + 1);
    errdefer gpa.free(offsets);
    const nodes = try gpa.alloc(u32, total_nodes);
    errdefer gpa.free(nodes);

    var at: usize = 0;
    for (routes, 0..) |r, i| {
        offsets[i] = at;
        for (r) |c| {
            nodes[at] = @intCast(c);
            at += 1;
        }
    }
    offsets[routes.len] = at;
    h.* = .{ .total_cost = total_cost, .offsets = offsets, .nodes = nodes };
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
    // SISR (the flagship engine) is the default. Setting the legacy ILS budget
    // knobs (vrptw_rounds / vrptw_restarts) selects the giant-tour ILS instead.
    var result = blk: {
        if (o.vrptw_rounds != 0 or o.vrptw_restarts != 0) {
            var params: vrptw.VrptwParams = .{ .veh_penalty = o.veh_penalty };
            if (o.vrptw_rounds != 0) params.rounds = @intCast(o.vrptw_rounds);
            if (o.vrptw_restarts != 0) params.restarts = @intCast(o.vrptw_restarts);
            break :blk vrptw.solveVrptw(gpa, inst, .{ .seed = o.seed }, params) catch |err| return mapError(err);
        }
        var params: vrptw.VrptwSisrParams = .{ .veh_penalty = o.veh_penalty };
        if (o.sisr_iters != 0) params.iters = @intCast(o.sisr_iters);
        if (o.threads > 1) {
            break :blk vrptw.solveVrptwSisrParallel(gpa, inst, .{ .seed = o.seed }, params, o.threads) catch |err| return mapError(err);
        }
        break :blk vrptw.solveVrptwSisr(gpa, inst, .{ .seed = o.seed }, params) catch |err| return mapError(err);
    };
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

export fn commiv_routes_free(r: ?*CommivRoutes) void {
    const h = r orelse return;
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
