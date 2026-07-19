const std = @import("std");
const builtin = @import("builtin");
const commiv = @import("commiv");

// commiv-serve: the language-agnostic front door. One static binary, JSON over
// HTTP, no dependencies — run it next to your app and every language that can
// speak HTTP gets near-optimal directed routes. See docs/rest.md for the schema
// and client snippets.
//
//   POST /solve/cvrp   {matrix, demand, capacity, seed?, iters?, threads?}
//   POST /solve/vrptw  {matrix, demand, capacity, ready, due, service,
//                       seed?, veh_penalty?, fleet_min?, max_vehicles?, wall_ms?}
//   POST /solve/pdptw  {matrix, pickups, deliveries, demand, capacity, ready,
//                       due, service, seed?, iters?, veh_penalty?, time_penalty?,
//                       fleet_min?, max_vehicles?, wall_ms?, max_route_duration?,
//                       vehicle_types? [[cap,fixed,count],..], driver_break?
//                       [dur,earliest,latest]}
//   POST /solve/pdptw/dispatch  same fields as /solve/pdptw (no fleet_min/
//                       max_vehicles: dispatch keeps the current fleet shape)
//                       plus current, locked — rolling-horizon re-solve
//                       around a committed plan; see docs/rest.md.
//   POST /solve/atsp   {matrix, seed?, trials?}
//   GET  /health
//
// Every /solve/* path also accepts the CMV1 binary framing (sniffed by magic,
// no separate path): "CMV1" + u32 LE header length + the endpoint's JSON body
// minus "matrix" + the matrix as raw little-endian u32, row-major. The matrix
// is ~90%+ of a large body and JSON-parsing it dominates request handling from
// n≈2000 (~175 MB of text at n=5000); the binary framing makes it a memcpy.
// See docs/rest.md for client snippets.
//
// Env: COMMIV_HOST (default 127.0.0.1), COMMIV_PORT (default 8080),
//      COMMIV_MAX_BODY_MB (default 256).
//
// Requests are handled sequentially: a solve occupies the core budget anyway,
// so admission control is the client's job for now (front it with a queue if
// you need one). Parallelism inside one solve comes from `threads` in the body.

const version_string = "0.4.0";

const ServerConfig = struct {
    host: []const u8,
    port: u16,
    max_body: usize,
};

pub fn main(init: std.process.Init) !void {
    const gpa = init.gpa;
    const io = init.io;
    const env = init.environ_map;

    const cfg = ServerConfig{
        .host = env.get("COMMIV_HOST") orelse "127.0.0.1",
        .port = try std.fmt.parseInt(u16, env.get("COMMIV_PORT") orelse "8080", 10),
        .max_body = (try std.fmt.parseInt(usize, env.get("COMMIV_MAX_BODY_MB") orelse "256", 10)) * 1024 * 1024,
    };

    const addr = try std.Io.net.IpAddress.parse(cfg.host, cfg.port);
    var server = try addr.listen(io, .{ .reuse_address = true });
    defer server.deinit(io);
    std.debug.print("commiv-serve {s} listening on http://{s}:{d}\n", .{ version_string, cfg.host, cfg.port });

    while (true) {
        const stream = server.accept(io) catch |err| {
            std.debug.print("accept failed: {t}\n", .{err});
            continue;
        };
        handleConnection(gpa, io, stream, cfg) catch {}; // per-connection errors never kill the server
    }
}

fn handleConnection(gpa: std.mem.Allocator, io: std.Io, stream: std.Io.net.Stream, cfg: ServerConfig) !void {
    defer stream.close(io);
    var rbuf: [64 * 1024]u8 = undefined;
    var wbuf: [64 * 1024]u8 = undefined;
    var sr = stream.reader(io, &rbuf);
    var sw = stream.writer(io, &wbuf);
    var http = std.http.Server.init(&sr.interface, &sw.interface);

    while (true) {
        var req = http.receiveHead() catch return; // client closed or bad head
        const keep_alive = req.head.keep_alive;
        handleRequest(gpa, &req, cfg) catch return; // I/O failure: drop connection
        if (!keep_alive) return;
    }
}

fn handleRequest(gpa: std.mem.Allocator, req: *std.http.Server.Request, cfg: ServerConfig) !void {
    // Everything a request allocates (body, parsed JSON, flattened matrices,
    // solver result, response JSON) lives in one arena.
    var arena_state = std.heap.ArenaAllocator.init(gpa);
    defer arena_state.deinit();
    const arena = arena_state.allocator();

    const target = req.head.target;
    if (std.mem.eql(u8, target, "/health")) {
        if (req.head.method != .GET) return respondError(req, .method_not_allowed, "use GET");
        return respondJson(arena, req, .{ .status = "ok", .version = version_string });
    }

    const Route = enum { cvrp, vrptw, pdptw, pdptw_dispatch, atsp, compat_vroom };
    const route: Route = if (std.mem.eql(u8, target, "/solve/cvrp"))
        .cvrp
    else if (std.mem.eql(u8, target, "/solve/vrptw"))
        .vrptw
    else if (std.mem.eql(u8, target, "/solve/pdptw"))
        .pdptw
    else if (std.mem.eql(u8, target, "/solve/pdptw/dispatch"))
        .pdptw_dispatch
    else if (std.mem.eql(u8, target, "/solve/atsp"))
        .atsp
    else if (std.mem.eql(u8, target, "/compat/vroom"))
        .compat_vroom
    else
        return respondError(req, .not_found, "unknown path; see /health and docs/rest.md");
    if (req.head.method != .POST) return respondError(req, .method_not_allowed, "use POST");

    var body_buf: [16 * 1024]u8 = undefined;
    const body_reader = req.readerExpectContinue(&body_buf) catch return error.WriteFailed;
    const body = body_reader.allocRemaining(arena, .limited(cfg.max_body)) catch |err| switch (err) {
        error.StreamTooLong => return respondError(req, .payload_too_large, "body exceeds COMMIV_MAX_BODY_MB"),
        error.OutOfMemory => return respondError(req, .internal_server_error, "out of memory"),
        else => return error.WriteFailed,
    };

    switch (route) {
        .cvrp => return solveCvrp(arena, req, body),
        .vrptw => return solveVrptw(arena, req, body),
        .pdptw => return solvePdptw(arena, req, body),
        .pdptw_dispatch => return solvePdptwDispatch(arena, req, body),
        .atsp => return solveAtsp(arena, req, body),
        .compat_vroom => return solveCompatVroom(arena, req, body),
    }
}

// ---- endpoint handlers -------------------------------------------------------

const CvrpRequest = struct {
    matrix: []const []const u32 = &.{}, // (n+1) rows of n+1 directed costs; depot = 0 (absent in CMV1 bodies)
    demand: []const u32,
    capacity: u32,
    seed: u64 = 1,
    iters: u64 = 0, // SISR iterations; 0 = default
    threads: u32 = 1, // >1 = parallel islands (result depends on the count)
    marathon: bool = false, // long-run constants profile (active at iters >= 1M)
    // Granular neighbor-list proximity key: "sum" (default), "min"
    // (direction-tolerant - measured win on strongly one-way street grids),
    // "out". Unknown strings fall back to "sum".
    nbr_key: []const u8 = "sum",
    gk: u64 = 0, // neighbor-list size, 0 = auto (20)
};

fn solveCvrp(arena: std.mem.Allocator, req: *std.http.Server.Request, body: []const u8) !void {
    const p = parseSolveRequest(CvrpRequest, arena, req, body) orelse return;
    const r = p.request;
    const flat = p.matrix;
    const n = p.dim - 1;
    if (p.dim < 2 or r.demand.len != n + 1)
        return respondError(req, .unprocessable_entity, "need >= 1 customer and demand of length n+1");

    const inst = commiv.CvrpInstance{ .n = n, .matrix = flat, .demand = r.demand, .capacity = r.capacity };
    var params: commiv.CvrpSisrParams = .{};
    if (r.iters != 0) params.iters = @intCast(r.iters);
    params.marathon = r.marathon;
    params.nbr_key = if (std.mem.eql(u8, r.nbr_key, "min")) .min else if (std.mem.eql(u8, r.nbr_key, "out")) .out else .sum;
    params.gk = @intCast(r.gk);
    const opts: commiv.SolveOptions = .{ .seed = r.seed };

    const result = blk: {
        if (r.threads > 1)
            break :blk commiv.solveCvrpSisrParallel(arena, inst, opts, params, r.threads) catch |err|
                return respondSolverError(req, err);
        break :blk commiv.solveCvrpSisr(arena, inst, opts, params) catch |err|
            return respondSolverError(req, err);
    };

    return respondJson(arena, req, .{
        .total_cost = result.total_cost,
        .vehicles = result.routes.len,
        .routes = result.routes,
    });
}

const VrptwRequest = struct {
    matrix: []const []const u32 = &.{}, // absent in CMV1 bodies
    demand: []const u32,
    capacity: u32,
    ready: []const u32, // earliest service start, ready[0] = 0
    due: []const u32, // latest service start, due[0] = depot horizon
    service: []const u32, // service durations, service[0] = 0
    seed: u64 = 1,
    engine: []const u8 = "sisr", // "sisr" is the only engine; field kept for compat
    iters: u64 = 0, // SISR iterations; 0 = default
    threads: u32 = 1, // SISR: >1 = best-of-K parallel chains
    veh_penalty: u64 = 0,
    fleet_min: bool = false, // hierarchical vehicle minimization (SISR engine)
    max_vehicles: u64 = 0, // hard route cap; 0 = uncapped
    wall_ms: u64 = 0, // wall-clock budget (ms) for SISR/fleet-min; 0 = iters-bounded
    max_route_duration: u64 = 0, // shift-length cap on route duration; 0 = uncapped
    // Long-run quality levers (see VrptwSisrParams); all default off, best
    // measured together at iters >= ~1M ("combo"): polish + stress_rate 0.5 +
    // tabu_tenure 10000 + marathon.
    polish: bool = false,
    stress_rate: f64 = 0.0,
    tabu_tenure: u64 = 0,
    marathon: bool = false,
    nbr_key: []const u8 = "sum", // granular-list key: "sum" | "min" | "out"
    gk: u64 = 0, // granular-list size, 0 = auto (20)
};

fn solveVrptw(arena: std.mem.Allocator, req: *std.http.Server.Request, body: []const u8) !void {
    const p = parseSolveRequest(VrptwRequest, arena, req, body) orelse return;
    const r = p.request;
    const flat = p.matrix;
    const n = p.dim - 1;
    if (p.dim < 2 or r.demand.len != n + 1 or r.ready.len != n + 1 or r.due.len != n + 1 or r.service.len != n + 1)
        return respondError(req, .unprocessable_entity, "demand/ready/due/service must all have length n+1");

    const inst = commiv.VrptwInstance{
        .n = n,
        .matrix = flat,
        .demand = r.demand,
        .capacity = r.capacity,
        .ready = r.ready,
        .due = r.due,
        .service = r.service,
    };
    // SISR is the only VRPTW engine.
    const result = blk: {
        var params: commiv.VrptwSisrParams = .{ .veh_penalty = r.veh_penalty };
        if (r.iters != 0) params.iters = @intCast(r.iters);
        if (r.wall_ms != 0) params.time_ms = r.wall_ms;
        if (r.max_vehicles != 0) params.max_vehicles = @intCast(r.max_vehicles);
        params.max_route_dur = r.max_route_duration;
        params.polish = r.polish;
        params.stress_rate = r.stress_rate;
        params.tabu_tenure = @intCast(r.tabu_tenure);
        params.marathon = r.marathon;
        params.nbr_key = if (std.mem.eql(u8, r.nbr_key, "min")) .min else if (std.mem.eql(u8, r.nbr_key, "out")) .out else .sum;
        params.gk = @intCast(r.gk);
        if (r.fleet_min) {
            // Hierarchical fleet minimization needs a wall budget; default 10s.
            // It takes precedence over threads (the fleet-min driver is serial).
            const budget: u64 = if (r.wall_ms != 0) r.wall_ms else 10_000;
            break :blk commiv.solveVrptwSisrFleetMin(arena, inst, .{ .seed = r.seed }, params, budget) catch |err|
                return respondSolverError(req, err);
        }
        if (r.threads > 1)
            break :blk commiv.solveVrptwSisrParallel(arena, inst, .{ .seed = r.seed }, params, r.threads) catch |err|
                return respondSolverError(req, err);
        break :blk commiv.solveVrptwSisr(arena, inst, .{ .seed = r.seed }, params) catch |err|
            return respondSolverError(req, err);
    };

    return respondJson(arena, req, .{
        .total_cost = result.total_cost,
        .vehicles = result.routes.len,
        .routes = result.routes,
    });
}

const PdptwRequest = struct {
    matrix: []const []const u32 = &.{}, // dim x dim, dim = 2*n_pairs+1; depot = 0 (absent in CMV1 bodies)
    pickups: []const u32, // n_pairs pickup node ids (1..dim-1)
    deliveries: []const u32, // n_pairs delivery node ids (1..dim-1)
    demand: []const u32, // n_pairs loads (one per request)
    capacity: u32,
    ready: []const u32, // dim entries; earliest service start, ready[0] = 0
    due: []const u32, // dim entries; latest service start, due[0] = depot horizon
    service: []const u32, // dim entries; service durations, service[0] = 0
    seed: u64 = 1,
    iters: u64 = 0, // SISR iterations; 0 = default
    veh_penalty: u64 = 0,
    time_penalty: u64 = 0, // money objective: charge route duration (PDPTW-only)
    fleet_min: bool = false, // hierarchical vehicle minimization
    max_vehicles: u64 = 0, // positive = PINNED driver (targets EXACTLY this fleet)
    wall_ms: u64 = 0, // wall-clock budget (ms) for the wall-driven drivers; 0 = 10s
    max_route_duration: u64 = 0, // shift-length cap on route duration; 0 = uncapped
    // Heterogeneous fleet (v1): up to 8 [capacity, fixed_cost, count] triples
    // (count 0 = unlimited). Nonempty = typed solve; `capacity` above is
    // ignored, fleet_min/max_vehicles rejected, response gains "types".
    vehicle_types: []const [3]u64 = &.{},
    // Driver break (v1): [duration, earliest, latest]. Empty = no break.
    driver_break: []const u32 = &.{},
};

fn solvePdptw(arena: std.mem.Allocator, req: *std.http.Server.Request, body: []const u8) !void {
    const p = parseSolveRequest(PdptwRequest, arena, req, body) orelse return;
    const r = p.request;
    const flat = p.matrix;
    const dim = p.dim;
    const n_pairs = r.pickups.len;
    if (dim < 3 or n_pairs == 0 or dim != 2 * n_pairs + 1)
        return respondError(req, .unprocessable_entity, "PDPTW needs a dim x dim matrix with dim = 2*n_pairs+1 and n_pairs >= 1");
    if (r.deliveries.len != n_pairs or r.demand.len != n_pairs)
        return respondError(req, .unprocessable_entity, "pickups/deliveries/demand must all have length n_pairs");
    if (r.ready.len != dim or r.due.len != dim or r.service.len != dim)
        return respondError(req, .unprocessable_entity, "ready/due/service must all have length dim (2*n_pairs+1)");
    if (r.capacity == 0 and r.vehicle_types.len == 0)
        return respondError(req, .unprocessable_entity, "capacity must be > 0");
    if (r.ready[0] != 0 or r.service[0] != 0)
        return respondError(req, .unprocessable_entity, "depot slots ready[0] and service[0] must be 0");
    if (r.vehicle_types.len > commiv.internal.pdptw_sisr.MAX_VEH_TYPES)
        return respondError(req, .unprocessable_entity, "vehicle_types supports at most 8 types");
    if (r.vehicle_types.len != 0 and (r.fleet_min or r.max_vehicles != 0))
        return respondError(req, .unprocessable_entity, "vehicle_types is incompatible with fleet_min/max_vehicles; bound the fleet with per-type counts");
    if (r.driver_break.len != 0 and r.driver_break.len != 3)
        return respondError(req, .unprocessable_entity, "driver_break must be [duration, earliest, latest]");
    if (r.driver_break.len == 3 and (r.fleet_min or r.max_vehicles != 0))
        return respondError(req, .unprocessable_entity, "driver_break is incompatible with fleet_min/max_vehicles (v1)");
    if (r.driver_break.len == 3 and r.driver_break[1] > r.driver_break[2])
        return respondError(req, .unprocessable_entity, "driver_break earliest must be <= latest");
    // Typed fleet: effective uniform capacity for the shape checks = largest type.
    var vt: [commiv.internal.pdptw_sisr.MAX_VEH_TYPES]commiv.internal.pdptw_sisr.VehType = undefined;
    var eff_cap: u32 = r.capacity;
    if (r.vehicle_types.len != 0) {
        eff_cap = 0;
        for (r.vehicle_types, 0..) |t, i| {
            if (t[0] == 0) return respondError(req, .unprocessable_entity, "vehicle type capacity must be > 0");
            vt[i] = .{ .capacity = @intCast(t[0]), .fixed_cost = t[1], .count = @intCast(t[2]) };
            eff_cap = @max(eff_cap, @as(u32, @intCast(t[0])));
        }
    }

    // Synthesize the pairing arrays the engine consumes and validate coverage:
    // every node 1..dim-1 must be named exactly once across pickups/deliveries.
    const pair_of = arena.alloc(usize, dim) catch return respondError(req, .internal_server_error, "out of memory");
    const is_pickup = arena.alloc(bool, dim) catch return respondError(req, .internal_server_error, "out of memory");
    const demand_signed = arena.alloc(i64, dim) catch return respondError(req, .internal_server_error, "out of memory");
    const seen = arena.alloc(bool, dim) catch return respondError(req, .internal_server_error, "out of memory");
    @memset(seen, false);
    pair_of[0] = 0;
    is_pickup[0] = false;
    demand_signed[0] = 0;
    for (0..n_pairs) |i| {
        const pu: usize = r.pickups[i];
        const qu: usize = r.deliveries[i];
        if (pu == 0 or qu == 0 or pu >= dim or qu >= dim or pu == qu)
            return respondError(req, .unprocessable_entity, "pickup/delivery ids must be distinct and in 1..dim-1");
        if (seen[pu] or seen[qu])
            return respondError(req, .unprocessable_entity, "each node may appear only once across pickups/deliveries");
        if (r.demand[i] > eff_cap)
            return respondError(req, .unprocessable_entity, "a request's demand exceeds capacity (infeasible)");
        seen[pu] = true;
        seen[qu] = true;
        pair_of[pu] = qu;
        pair_of[qu] = pu;
        is_pickup[pu] = true;
        is_pickup[qu] = false;
        demand_signed[pu] = @as(i64, r.demand[i]);
        demand_signed[qu] = -@as(i64, r.demand[i]);
    }
    for (seen[1..dim]) |s| {
        if (!s) return respondError(req, .unprocessable_entity, "every node 1..dim-1 must be named exactly once across pickups/deliveries");
    }

    const inst = commiv.PdpInstance{
        .n_pairs = n_pairs,
        .matrix = flat,
        .capacity = @intCast(eff_cap),
        .pair_of = pair_of,
        .is_pickup = is_pickup,
        .demand_signed = demand_signed,
        .ready = r.ready,
        .due = r.due,
        .service = r.service,
    };

    var params: commiv.PdpSisrParams = .{
        .seed = r.seed,
        .veh_penalty = r.veh_penalty,
        .time_penalty = r.time_penalty,
        .max_route_dur = r.max_route_duration,
    };
    if (r.iters != 0) params.iters = @intCast(r.iters);
    if (r.wall_ms != 0) params.time_ms = r.wall_ms;
    if (r.max_vehicles != 0) params.max_vehicles = @intCast(r.max_vehicles);
    if (r.vehicle_types.len != 0) params.veh_types = vt[0..r.vehicle_types.len];
    if (r.driver_break.len == 3 and r.driver_break[0] != 0)
        params.brk = .{ .dur = r.driver_break[0], .earliest = r.driver_break[1], .latest = r.driver_break[2] };
    // The wall-driven drivers (fleet-min, pinned) need a budget; default 10s.
    const budget: u64 = if (r.wall_ms != 0) r.wall_ms else 10_000;

    var result = blk: {
        if (r.fleet_min)
            break :blk commiv.internal.pdptw_sisr.solvePdptwSisrFleetMin(arena, inst, params, budget) catch |err|
                return respondSolverError(req, err);
        if (r.max_vehicles != 0)
            break :blk commiv.internal.pdptw_sisr.solvePdptwSisrPinned(arena, inst, params, budget, @intCast(r.max_vehicles)) catch |err|
                return respondSolverError(req, err);
        break :blk commiv.solvePdptwSisr(arena, inst, params) catch |err|
            return respondSolverError(req, err);
    };
    defer result.deinit();

    if (result.types) |tys| {
        return respondJson(arena, req, .{
            .total_cost = result.total_cost,
            .vehicles = result.routes.len,
            .routes = result.routes,
            .types = tys,
        });
    }
    return respondJson(arena, req, .{
        .total_cost = result.total_cost,
        .vehicles = result.routes.len,
        .routes = result.routes,
    });
}

const PdptwDispatchRequest = struct {
    matrix: []const []const u32 = &.{}, // dim x dim, dim = 2*n_pairs+1; depot = 0 (absent in CMV1 bodies)
    pickups: []const u32, // n_pairs pickup node ids (1..dim-1)
    deliveries: []const u32, // n_pairs delivery node ids (1..dim-1)
    demand: []const u32, // n_pairs loads (one per request)
    capacity: u32,
    ready: []const u32, // dim entries; earliest service start, ready[0] = 0
    due: []const u32, // dim entries; latest service start, due[0] = depot horizon
    service: []const u32, // dim entries; service durations, service[0] = 0
    current: []const []const u32 = &.{}, // one array of node ids per existing route
    locked: []const u64 = &.{}, // one entry per route in `current`
    seed: u64 = 1,
    iters: u64 = 0, // SISR iterations; 0 = default
    veh_penalty: u64 = 0,
    time_penalty: u64 = 0, // money objective: charge route duration (PDPTW-only)
    wall_ms: u64 = 0, // wall-clock budget (ms); 0 = default
    max_route_duration: u64 = 0, // shift-length cap on route duration; 0 = uncapped
    driver_break: []const u32 = &.{}, // [duration, earliest, latest]; empty = none
};

/// Rolling-horizon PDPTW re-solve: the /solve/pdptw instance contract plus
/// the CURRENT plan and its LOCKED prefixes. fleet_min/max_vehicles are not
/// accepted here — dispatch keeps the current fleet shape. See docs/rest.md.
fn solvePdptwDispatch(arena: std.mem.Allocator, req: *std.http.Server.Request, body: []const u8) !void {
    const p = parseSolveRequest(PdptwDispatchRequest, arena, req, body) orelse return;
    const r = p.request;
    const flat = p.matrix;
    const dim = p.dim;
    const n_pairs = r.pickups.len;
    if (dim < 3 or n_pairs == 0 or dim != 2 * n_pairs + 1)
        return respondError(req, .unprocessable_entity, "PDPTW needs a dim x dim matrix with dim = 2*n_pairs+1 and n_pairs >= 1");
    if (r.deliveries.len != n_pairs or r.demand.len != n_pairs)
        return respondError(req, .unprocessable_entity, "pickups/deliveries/demand must all have length n_pairs");
    if (r.ready.len != dim or r.due.len != dim or r.service.len != dim)
        return respondError(req, .unprocessable_entity, "ready/due/service must all have length dim (2*n_pairs+1)");
    if (r.capacity == 0)
        return respondError(req, .unprocessable_entity, "capacity must be > 0");
    if (r.ready[0] != 0 or r.service[0] != 0)
        return respondError(req, .unprocessable_entity, "depot slots ready[0] and service[0] must be 0");
    if (r.locked.len != r.current.len)
        return respondError(req, .unprocessable_entity, "locked must have exactly one entry per route in current");

    // Synthesize the pairing arrays the engine consumes and validate coverage
    // (identical to /solve/pdptw): every node 1..dim-1 named exactly once.
    const pair_of = arena.alloc(usize, dim) catch return respondError(req, .internal_server_error, "out of memory");
    const is_pickup = arena.alloc(bool, dim) catch return respondError(req, .internal_server_error, "out of memory");
    const demand_signed = arena.alloc(i64, dim) catch return respondError(req, .internal_server_error, "out of memory");
    const seen = arena.alloc(bool, dim) catch return respondError(req, .internal_server_error, "out of memory");
    @memset(seen, false);
    pair_of[0] = 0;
    is_pickup[0] = false;
    demand_signed[0] = 0;
    for (0..n_pairs) |i| {
        const pu: usize = r.pickups[i];
        const qu: usize = r.deliveries[i];
        if (pu == 0 or qu == 0 or pu >= dim or qu >= dim or pu == qu)
            return respondError(req, .unprocessable_entity, "pickup/delivery ids must be distinct and in 1..dim-1");
        if (seen[pu] or seen[qu])
            return respondError(req, .unprocessable_entity, "each node may appear only once across pickups/deliveries");
        if (r.demand[i] > r.capacity)
            return respondError(req, .unprocessable_entity, "a request's demand exceeds capacity (infeasible)");
        seen[pu] = true;
        seen[qu] = true;
        pair_of[pu] = qu;
        pair_of[qu] = pu;
        is_pickup[pu] = true;
        is_pickup[qu] = false;
        demand_signed[pu] = @as(i64, r.demand[i]);
        demand_signed[qu] = -@as(i64, r.demand[i]);
    }
    for (seen[1..dim]) |s| {
        if (!s) return respondError(req, .unprocessable_entity, "every node 1..dim-1 must be named exactly once across pickups/deliveries");
    }

    // Validate the current plan + the locked-prefix contract, then convert
    // node ids (u32 over the wire) into the usize slices the engine wants.
    const current = arena.alloc([]const usize, r.current.len) catch return respondError(req, .internal_server_error, "out of memory");
    const locked = arena.alloc(usize, r.locked.len) catch return respondError(req, .internal_server_error, "out of memory");
    const plan_seen = arena.alloc(bool, dim) catch return respondError(req, .internal_server_error, "out of memory");
    @memset(plan_seen, false);
    for (r.current, 0..) |route, i| {
        const conv = arena.alloc(usize, route.len) catch return respondError(req, .internal_server_error, "out of memory");
        for (route, 0..) |c, j| conv[j] = c;
        current[i] = conv;
        locked[i] = @intCast(r.locked[i]);
    }
    for (current, 0..) |route, i| {
        if (locked[i] > route.len)
            return respondError(req, .unprocessable_entity, "locked[i] cannot exceed the length of current[i]");
        for (route) |c| {
            if (c == 0 or c >= dim or plan_seen[c])
                return respondError(req, .unprocessable_entity, "current: node ids must be in 1..dim-1 and appear at most once across all routes (absent is fine)");
            plan_seen[c] = true;
        }
    }
    for (current, 0..) |route, i| {
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
            if (!found) return respondError(req, .unprocessable_entity, "a locked delivery's pickup must also be locked, in the same route");
        }
    }

    const inst = commiv.PdpInstance{
        .n_pairs = n_pairs,
        .matrix = flat,
        .capacity = @intCast(r.capacity),
        .pair_of = pair_of,
        .is_pickup = is_pickup,
        .demand_signed = demand_signed,
        .ready = r.ready,
        .due = r.due,
        .service = r.service,
    };

    var params: commiv.PdpSisrParams = .{
        .seed = r.seed,
        .veh_penalty = r.veh_penalty,
        .time_penalty = r.time_penalty,
        .max_route_dur = r.max_route_duration,
    };
    if (r.iters != 0) params.iters = @intCast(r.iters);
    if (r.wall_ms != 0) params.time_ms = r.wall_ms;
    if (r.driver_break.len == 3 and r.driver_break[0] != 0) {
        if (r.driver_break[1] > r.driver_break[2])
            return respondError(req, .unprocessable_entity, "driver_break earliest must be <= latest");
        params.brk = .{ .dur = r.driver_break[0], .earliest = r.driver_break[1], .latest = r.driver_break[2] };
    } else if (r.driver_break.len != 0 and r.driver_break.len != 3)
        return respondError(req, .unprocessable_entity, "driver_break must be [duration, earliest, latest]");

    var result = commiv.internal.pdptw_sisr.solvePdptwSisrDispatch(arena, inst, params, current, locked) catch |err|
        return respondSolverError(req, err);
    defer result.deinit();

    return respondJson(arena, req, .{
        .total_cost = result.total_cost,
        .vehicles = result.routes.len,
        .routes = result.routes,
    });
}

const AtspRequest = struct {
    matrix: []const []const u32 = &.{}, // n x n directed costs (absent in CMV1 bodies)
    seed: u64 = 1,
    trials: u64 = 0,
};

fn solveAtsp(arena: std.mem.Allocator, req: *std.http.Server.Request, body: []const u8) !void {
    const p = parseSolveRequest(AtspRequest, arena, req, body) orelse return;
    const r = p.request;
    const flat = p.matrix;
    const n = p.dim;
    if (n < 2) return respondError(req, .unprocessable_entity, "need at least 2 nodes");

    var opts: commiv.SolveOptions = .{ .seed = r.seed };
    if (r.trials != 0) opts.budget.trials = @intCast(r.trials);

    const result = commiv.solveAtsp(arena, flat, n, opts) catch |err|
        return respondSolverError(req, err);

    return respondJson(arena, req, .{ .cost = result.length, .tour = result.tour });
}

// ---- VROOM-compatible adapter ------------------------------------------------
//
// POST /compat/vroom accepts a VROOM request JSON and answers in VROOM's own
// response shape, so an existing VROOM client can point at commiv unchanged and
// pick up the money objective (route-duration + waiting pricing) VROOM cannot
// express. v1 is matrix-based and single-depot on a homogeneous fleet:
//
//   * matrices.car.durations REQUIRED (no geocoding). Optional matrices.car.costs
//     is the basis for the reported "cost"; it defaults to durations.
//   * vehicles[] must all have start_index/end_index 0 or absent (single depot),
//     the same capacity:[c], and the same optional costs.fixed (-> veh_penalty).
//     vehicles[0].time_window[1] is the depot horizon (due[0]).
//   * EITHER shipments[] (-> PDPTW) OR jobs[] (-> VRPTW), never both. The tasks'
//     location_index values ARE the matrix rows: v1 requires the matrix to cover
//     exactly the depot plus every task node once (no unused rows).
//   * An optional "commiv" block {wall_ms, time_penalty, seed} opts into the
//     money objective (time_penalty is PDPTW/shipments-only).
//
// Errors are VROOM-shaped: {"code":1,"error":"..."} with a 4xx status.

const VroomCosts = struct {
    fixed: u64 = 0,
};

const VroomProfile = struct {
    durations: []const []const u32 = &.{},
    costs: []const []const u32 = &.{},
};

const VroomMatrices = struct {
    car: VroomProfile = .{},
};

const VroomTask = struct {
    id: ?u64 = null,
    location_index: u32,
    time_windows: []const []const u32 = &.{}, // [[a,b],...]; v1 uses the first
    service: u32 = 0,
};

const VroomShipment = struct {
    amount: []const u32 = &.{}, // [q]
    pickup: VroomTask,
    delivery: VroomTask,
};

const VroomJob = struct {
    id: ?u64 = null,
    location_index: u32,
    delivery: []const u32 = &.{}, // [q]
    amount: []const u32 = &.{}, // fallback when delivery absent
    time_windows: []const []const u32 = &.{},
    service: u32 = 0,
};

const VroomVehicle = struct {
    id: ?u64 = null,
    start_index: ?i64 = null, // v1: 0 or absent
    end_index: ?i64 = null, // v1: 0 or absent
    capacity: []const u32 = &.{}, // [c]
    time_window: []const u32 = &.{}, // [a,b]; [1] = depot horizon
    costs: VroomCosts = .{},
};

const VroomCommiv = struct {
    wall_ms: u64 = 0,
    time_penalty: u64 = 0,
    seed: u64 = 1,
};

const VroomRequest = struct {
    matrices: VroomMatrices = .{},
    vehicles: []const VroomVehicle = &.{},
    shipments: []const VroomShipment = &.{},
    jobs: []const VroomJob = &.{},
    commiv: VroomCommiv = .{},
};

const NodeKind = enum(u8) { none, pickup, delivery, job };

/// One task's [ready, due], falling back to [0, horizon] when it has no window.
fn taskWindow(tw: []const []const u32, horizon: u32) [2]u32 {
    if (tw.len > 0 and tw[0].len >= 2) return .{ tw[0][0], tw[0][1] };
    return .{ 0, horizon };
}

fn solveCompatVroom(arena: std.mem.Allocator, req: *std.http.Server.Request, body: []const u8) !void {
    const vr = std.json.parseFromSliceLeaky(VroomRequest, arena, body, .{ .ignore_unknown_fields = true }) catch
        return respondVroomError(req, .bad_request, "invalid VROOM JSON body; see docs/rest.md /compat/vroom");

    // --- matrix (durations REQUIRED, no geocoding) ---
    const durations = vr.matrices.car.durations;
    if (durations.len == 0)
        return respondVroomError(req, .unprocessable_entity, "matrices.car.durations is required (v1 is matrix-based; no geocoding)");
    const flat = flattenSquare(arena, durations) orelse
        return respondVroomError(req, .unprocessable_entity, "matrices.car.durations must be square (dim rows of dim entries)");
    const dim = durations.len;
    // cost basis: matrices.car.costs when a matching square, else durations.
    var cost_flat = flat;
    if (vr.matrices.car.costs.len == dim) {
        if (flattenSquare(arena, vr.matrices.car.costs)) |cf| cost_flat = cf;
    }

    // --- fleet: single depot, homogeneous capacity + fixed cost ---
    if (vr.vehicles.len == 0)
        return respondVroomError(req, .unprocessable_entity, "at least one vehicle is required");
    for (vr.vehicles) |v| {
        if (v.start_index) |s| if (s != 0)
            return respondVroomError(req, .unprocessable_entity, "v1 is single-depot: every vehicle start_index must be 0 or absent");
        if (v.end_index) |e| if (e != 0)
            return respondVroomError(req, .unprocessable_entity, "v1 is single-depot: every vehicle end_index must be 0 or absent");
        if (v.capacity.len == 0)
            return respondVroomError(req, .unprocessable_entity, "every vehicle needs a capacity:[c]");
    }
    const cap = vr.vehicles[0].capacity[0];
    if (cap == 0)
        return respondVroomError(req, .unprocessable_entity, "capacity must be > 0");
    const fixed = vr.vehicles[0].costs.fixed;
    for (vr.vehicles) |v| {
        if (v.capacity[0] != cap)
            return respondVroomError(req, .unprocessable_entity, "v1 requires a homogeneous fleet: all vehicles must share capacity");
        if (v.costs.fixed != fixed)
            return respondVroomError(req, .unprocessable_entity, "v1 requires a homogeneous fleet: all vehicles must share costs.fixed");
    }
    const horizon: u32 = if (vr.vehicles[0].time_window.len >= 2) vr.vehicles[0].time_window[1] else 1_000_000_000;

    // --- exactly one of jobs / shipments ---
    const has_jobs = vr.jobs.len > 0;
    const has_ships = vr.shipments.len > 0;
    if (has_jobs and has_ships)
        return respondVroomError(req, .unprocessable_entity, "v1 supports jobs or shipments, not both");
    if (!has_jobs and !has_ships)
        return respondVroomError(req, .unprocessable_entity, "provide jobs (VRPTW) or shipments (PDPTW)");

    // Per-node arrays shared by the solver instance and the schedule walk.
    const ready = arena.alloc(u32, dim) catch return respondVroomError(req, .internal_server_error, "out of memory");
    const due = arena.alloc(u32, dim) catch return respondVroomError(req, .internal_server_error, "out of memory");
    const service = arena.alloc(u32, dim) catch return respondVroomError(req, .internal_server_error, "out of memory");
    const node_id = arena.alloc(u64, dim) catch return respondVroomError(req, .internal_server_error, "out of memory");
    const node_kind = arena.alloc(NodeKind, dim) catch return respondVroomError(req, .internal_server_error, "out of memory");
    for (0..dim) |i| {
        ready[i] = 0;
        due[i] = horizon;
        service[i] = 0;
        node_id[i] = @intCast(i);
        node_kind[i] = .none;
    }

    var result_routes: [][]usize = undefined;

    if (has_ships) {
        // ---- shipments -> PDPTW ----
        const n_pairs = vr.shipments.len;
        if (dim != 2 * n_pairs + 1)
            return respondVroomError(req, .unprocessable_entity, "v1 shipments: matrix dim must equal 2*n_pairs+1 (depot + one row per pickup and delivery; no unused rows)");

        const pair_of = arena.alloc(usize, dim) catch return respondVroomError(req, .internal_server_error, "out of memory");
        const is_pickup = arena.alloc(bool, dim) catch return respondVroomError(req, .internal_server_error, "out of memory");
        const demand_signed = arena.alloc(i64, dim) catch return respondVroomError(req, .internal_server_error, "out of memory");
        const seen = arena.alloc(bool, dim) catch return respondVroomError(req, .internal_server_error, "out of memory");
        @memset(seen, false);
        pair_of[0] = 0;
        is_pickup[0] = false;
        demand_signed[0] = 0;

        for (vr.shipments) |sh| {
            const p: usize = sh.pickup.location_index;
            const q: usize = sh.delivery.location_index;
            if (p == 0 or q == 0 or p >= dim or q >= dim or p == q)
                return respondVroomError(req, .unprocessable_entity, "pickup/delivery location_index must be distinct and in 1..dim-1");
            if (seen[p] or seen[q])
                return respondVroomError(req, .unprocessable_entity, "each location may appear at most once across pickups/deliveries");
            const amt: u32 = if (sh.amount.len > 0) sh.amount[0] else 0;
            if (amt > cap)
                return respondVroomError(req, .unprocessable_entity, "a shipment's amount exceeds capacity (infeasible)");
            seen[p] = true;
            seen[q] = true;
            pair_of[p] = q;
            pair_of[q] = p;
            is_pickup[p] = true;
            is_pickup[q] = false;
            demand_signed[p] = @as(i64, amt);
            demand_signed[q] = -@as(i64, amt);
            const pw = taskWindow(sh.pickup.time_windows, horizon);
            const dw = taskWindow(sh.delivery.time_windows, horizon);
            ready[p] = pw[0];
            due[p] = pw[1];
            service[p] = sh.pickup.service;
            ready[q] = dw[0];
            due[q] = dw[1];
            service[q] = sh.delivery.service;
            node_id[p] = sh.pickup.id orelse @as(u64, @intCast(p));
            node_id[q] = sh.delivery.id orelse @as(u64, @intCast(q));
            node_kind[p] = .pickup;
            node_kind[q] = .delivery;
        }
        for (seen[1..dim]) |s| {
            if (!s) return respondVroomError(req, .unprocessable_entity, "the matrix must cover exactly depot + every pickup/delivery node once");
        }

        const inst = commiv.PdpInstance{
            .n_pairs = n_pairs,
            .matrix = flat,
            .capacity = @intCast(cap),
            .pair_of = pair_of,
            .is_pickup = is_pickup,
            .demand_signed = demand_signed,
            .ready = ready,
            .due = due,
            .service = service,
        };
        var params: commiv.PdpSisrParams = .{
            .seed = vr.commiv.seed,
            .veh_penalty = fixed,
            .time_penalty = vr.commiv.time_penalty,
        };
        if (vr.commiv.wall_ms != 0) params.time_ms = vr.commiv.wall_ms;
        const res = commiv.solvePdptwSisr(arena, inst, params) catch |err|
            return respondVroomSolverError(req, err);
        result_routes = res.routes;
    } else {
        // ---- jobs -> VRPTW ----
        if (vr.commiv.time_penalty != 0)
            return respondVroomError(req, .unprocessable_entity, "time_penalty (money objective) is PDPTW-only; supply shipments for it");
        const n = dim - 1;
        const demand = arena.alloc(u32, dim) catch return respondVroomError(req, .internal_server_error, "out of memory");
        const seen = arena.alloc(bool, dim) catch return respondVroomError(req, .internal_server_error, "out of memory");
        @memset(demand, 0);
        @memset(seen, false);

        for (vr.jobs) |jb| {
            const L: usize = jb.location_index;
            if (L == 0 or L >= dim)
                return respondVroomError(req, .unprocessable_entity, "job location_index must be in 1..dim-1");
            if (seen[L])
                return respondVroomError(req, .unprocessable_entity, "each location may host at most one job");
            const amt: u32 = if (jb.delivery.len > 0) jb.delivery[0] else if (jb.amount.len > 0) jb.amount[0] else 0;
            if (amt > cap)
                return respondVroomError(req, .unprocessable_entity, "a job's demand exceeds capacity (infeasible)");
            seen[L] = true;
            demand[L] = amt;
            const w = taskWindow(jb.time_windows, horizon);
            ready[L] = w[0];
            due[L] = w[1];
            service[L] = jb.service;
            node_id[L] = jb.id orelse @as(u64, @intCast(L));
            node_kind[L] = .job;
        }
        for (seen[1..dim]) |s| {
            if (!s) return respondVroomError(req, .unprocessable_entity, "v1 jobs: the matrix must cover exactly depot + every job node once (no unused rows)");
        }
        demand[0] = 0;

        const inst = commiv.VrptwInstance{
            .n = n,
            .matrix = flat,
            .demand = demand,
            .capacity = cap,
            .ready = ready,
            .due = due,
            .service = service,
        };
        var params: commiv.VrptwSisrParams = .{ .veh_penalty = fixed };
        if (vr.commiv.wall_ms != 0) params.time_ms = vr.commiv.wall_ms;
        const res = commiv.solveVrptwSisr(arena, inst, .{ .seed = vr.commiv.seed }, params) catch |err|
            return respondVroomSolverError(req, err);
        result_routes = res.routes;
    }

    return emitVroomResponse(arena, req, .{
        .routes = result_routes,
        .dim = dim,
        .time_matrix = flat,
        .cost_matrix = cost_flat,
        .ready = ready,
        .due = due,
        .service = service,
        .node_id = node_id,
        .node_kind = node_kind,
        .vehicles = vr.vehicles,
        .fixed = fixed,
    });
}

const VroomEmit = struct {
    routes: [][]usize,
    dim: usize,
    time_matrix: []const u32,
    cost_matrix: []const u32,
    ready: []const u32,
    due: []const u32,
    service: []const u32,
    node_id: []const u64,
    node_kind: []const NodeKind,
    vehicles: []const VroomVehicle,
    fixed: u64,
};

fn jNum(buf: *std.ArrayList(u8), a: std.mem.Allocator, v: u64) !void {
    var tmp: [24]u8 = undefined;
    try buf.appendSlice(a, std.fmt.bufPrint(&tmp, "{d}", .{v}) catch unreachable);
}

/// Serialize a solved instance into VROOM's response shape, deriving all summary
/// numbers by walking the same schedule recurrence the audits use: leave the
/// depot at t=0, travel, wait for free until ready[j], serve, and return.
fn emitVroomResponse(arena: std.mem.Allocator, req: *std.http.Server.Request, e: VroomEmit) !void {
    const dim = e.dim;
    var buf: std.ArrayList(u8) = .empty;

    var sum_travel: u64 = 0;
    var sum_service: u64 = 0;
    var sum_wait: u64 = 0;
    var sum_cost: u64 = 0;

    try buf.appendSlice(arena, "{\"code\":0,\"summary\":");
    // Reserve summary spot by building routes first into a separate buffer.
    var rbuf: std.ArrayList(u8) = .empty;
    try rbuf.appendSlice(arena, "[");
    for (e.routes, 0..) |route, ri| {
        if (ri != 0) try rbuf.append(arena, ',');
        const veh_id: u64 = if (ri < e.vehicles.len) (e.vehicles[ri].id orelse @as(u64, @intCast(ri + 1))) else @as(u64, @intCast(ri + 1));
        var r_travel: u64 = 0; // travel time (durations)
        var r_cost: u64 = 0; // travel cost (cost matrix)
        var r_service: u64 = 0;
        var r_wait: u64 = 0;

        try rbuf.appendSlice(arena, "{\"vehicle\":");
        try jNum(&rbuf, arena, veh_id);
        try rbuf.appendSlice(arena, ",\"steps\":[{\"type\":\"start\",\"location_index\":0,\"arrival\":0,\"duration\":0}");

        var prev: usize = 0;
        var t: u64 = 0; // departure clock from prev
        for (route) |node| {
            const travel: u64 = e.time_matrix[prev * dim + node];
            const cost_travel: u64 = e.cost_matrix[prev * dim + node];
            r_travel += travel;
            r_cost += cost_travel;
            const arrival = t + travel;
            const rdy: u64 = e.ready[node];
            const wait: u64 = if (rdy > arrival) rdy - arrival else 0;
            r_wait += wait;
            const svc: u64 = e.service[node];
            r_service += svc;
            const kind = e.node_kind[node];
            const type_str = switch (kind) {
                .pickup => "pickup",
                .delivery => "delivery",
                .job => "job",
                .none => "job",
            };
            try rbuf.appendSlice(arena, ",{\"type\":\"");
            try rbuf.appendSlice(arena, type_str);
            try rbuf.appendSlice(arena, "\",\"location_index\":");
            try jNum(&rbuf, arena, @intCast(node));
            try rbuf.appendSlice(arena, ",\"id\":");
            try jNum(&rbuf, arena, e.node_id[node]);
            try rbuf.appendSlice(arena, ",\"arrival\":");
            try jNum(&rbuf, arena, arrival);
            try rbuf.appendSlice(arena, ",\"duration\":");
            try jNum(&rbuf, arena, r_travel);
            try rbuf.appendSlice(arena, ",\"waiting_time\":");
            try jNum(&rbuf, arena, wait);
            try rbuf.appendSlice(arena, ",\"service\":");
            try jNum(&rbuf, arena, svc);
            try rbuf.append(arena, '}');
            t = arrival + wait + svc;
            prev = node;
        }
        // return to depot
        const back: u64 = e.time_matrix[prev * dim + 0];
        const back_cost: u64 = e.cost_matrix[prev * dim + 0];
        r_travel += back;
        r_cost += back_cost;
        const depot_arrival = t + back;
        try rbuf.appendSlice(arena, ",{\"type\":\"end\",\"location_index\":0,\"arrival\":");
        try jNum(&rbuf, arena, depot_arrival);
        try rbuf.appendSlice(arena, ",\"duration\":");
        try jNum(&rbuf, arena, r_travel);
        try rbuf.appendSlice(arena, "}]");

        const route_cost = r_cost + e.fixed;
        try rbuf.appendSlice(arena, ",\"cost\":");
        try jNum(&rbuf, arena, route_cost);
        try rbuf.appendSlice(arena, ",\"duration\":");
        try jNum(&rbuf, arena, r_travel);
        try rbuf.appendSlice(arena, ",\"service\":");
        try jNum(&rbuf, arena, r_service);
        try rbuf.appendSlice(arena, ",\"waiting_time\":");
        try jNum(&rbuf, arena, r_wait);
        try rbuf.append(arena, '}');

        sum_travel += r_travel;
        sum_service += r_service;
        sum_wait += r_wait;
        sum_cost += route_cost;
    }
    try rbuf.append(arena, ']');

    // summary
    try buf.appendSlice(arena, "{\"cost\":");
    try jNum(&buf, arena, sum_cost);
    try buf.appendSlice(arena, ",\"routes\":");
    try jNum(&buf, arena, @intCast(e.routes.len));
    try buf.appendSlice(arena, ",\"unassigned\":0,\"duration\":");
    try jNum(&buf, arena, sum_travel);
    try buf.appendSlice(arena, ",\"service\":");
    try jNum(&buf, arena, sum_service);
    try buf.appendSlice(arena, ",\"waiting_time\":");
    try jNum(&buf, arena, sum_wait);
    try buf.appendSlice(arena, "},\"routes\":");
    try buf.appendSlice(arena, rbuf.items);
    try buf.appendSlice(arena, ",\"unassigned\":[]}");

    try req.respond(buf.items, .{
        .extra_headers = &.{.{ .name = "content-type", .value = "application/json" }},
    });
}

fn respondVroomSolverError(req: *std.http.Server.Request, err: anyerror) !void {
    return switch (err) {
        error.Infeasible, error.NoFeasibleSplit, error.NoCompleteSolution => respondVroomError(req, .unprocessable_entity, "instance is infeasible under the given capacity/time windows/fleet"),
        error.InvalidInstance, error.InvalidMatrix => respondVroomError(req, .unprocessable_entity, "instance is invalid: check matrix shape and task windows"),
        error.OutOfMemory => respondVroomError(req, .internal_server_error, "out of memory"),
        else => respondVroomError(req, .internal_server_error, "solver failed"),
    };
}

fn respondVroomError(req: *std.http.Server.Request, status: std.http.Status, message: []const u8) !void {
    var buf: [512]u8 = undefined;
    const json = std.fmt.bufPrint(&buf, "{{\"code\":1,\"error\":\"{s}\"}}", .{message}) catch "{\"code\":1,\"error\":\"internal\"}";
    try req.respond(json, .{
        .status = status,
        .extra_headers = &.{.{ .name = "content-type", .value = "application/json" }},
    });
}

// ---- plumbing ----------------------------------------------------------------

fn parseBody(comptime T: type, arena: std.mem.Allocator, req: *std.http.Server.Request, body: []const u8) ?T {
    return std.json.parseFromSliceLeaky(T, arena, body, .{ .ignore_unknown_fields = true }) catch {
        respondError(req, .bad_request, "invalid JSON body: matrix/demand/window entries must be non-negative integers < 2^32 (scale floats to e.g. whole seconds); see docs/rest.md") catch {};
        return null;
    };
}

fn Parsed(comptime T: type) type {
    return struct { request: T, matrix: []u32, dim: usize };
}

/// Decode a /solve/* body in either encoding into (request params, flat
/// row-major matrix, dim). JSON: the whole body is T, matrix flattened from
/// its rows. CMV1 binary (sniffed by magic): u32 LE header length at offset 4,
/// then a JSON header (T minus "matrix"), then the matrix as raw little-endian
/// u32 — decoded by one aligned memcpy instead of parsing ~7 bytes of text per
/// entry, which is the difference between ~0.1 s of JSON and ~4 ms at n=2000.
/// dim is inferred from the byte count (must be 4*dim^2). On any error this
/// responds to the client and returns null.
fn parseSolveRequest(comptime T: type, arena: std.mem.Allocator, req: *std.http.Server.Request, body: []const u8) ?Parsed(T) {
    if (std.mem.startsWith(u8, body, "CMV1")) {
        if (body.len < 8) {
            respondError(req, .bad_request, "CMV1 body truncated (need magic + u32 header length)") catch {};
            return null;
        }
        const hlen: usize = std.mem.readInt(u32, body[4..8], .little);
        if (hlen > body.len - 8) {
            respondError(req, .bad_request, "CMV1 header length exceeds body") catch {};
            return null;
        }
        const r = parseBody(T, arena, req, body[8 .. 8 + hlen]) orelse return null;
        const mbytes = body[8 + hlen ..];
        if (mbytes.len == 0 or mbytes.len % 4 != 0) {
            respondError(req, .bad_request, "CMV1 matrix must be a non-empty sequence of little-endian u32") catch {};
            return null;
        }
        const cells = mbytes.len / 4;
        const dim = std.math.sqrt(cells);
        if (dim * dim != cells) {
            respondError(req, .bad_request, "CMV1 matrix byte count must be 4*dim^2 (square, row-major)") catch {};
            return null;
        }
        const flat = arena.alloc(u32, cells) catch {
            respondError(req, .internal_server_error, "out of memory") catch {};
            return null;
        };
        @memcpy(std.mem.sliceAsBytes(flat), mbytes);
        if (comptime builtin.cpu.arch.endian() == .big) {
            for (flat) |*v| v.* = @byteSwap(v.*);
        }
        return .{ .request = r, .matrix = flat, .dim = dim };
    }

    const r = parseBody(T, arena, req, body) orelse return null;
    if (r.matrix.len == 0) {
        respondError(req, .unprocessable_entity, "missing matrix") catch {};
        return null;
    }
    const flat = flattenSquare(arena, r.matrix) orelse {
        respondError(req, .unprocessable_entity, "matrix must be square (dim rows of dim entries)") catch {};
        return null;
    };
    return .{ .request = r, .matrix = flat, .dim = r.matrix.len };
}

/// Rows -> contiguous row-major matrix; null unless every row has rows.len entries.
fn flattenSquare(arena: std.mem.Allocator, rows: []const []const u32) ?[]u32 {
    const dim = rows.len;
    if (dim == 0) return null;
    const flat = arena.alloc(u32, dim * dim) catch return null;
    for (rows, 0..) |row, i| {
        if (row.len != dim) return null;
        @memcpy(flat[i * dim ..][0..dim], row);
    }
    return flat;
}

fn respondSolverError(req: *std.http.Server.Request, err: anyerror) !void {
    return switch (err) {
        error.Infeasible, error.NoFeasibleSplit, error.NoCompleteSolution => respondError(req, .unprocessable_entity, "instance is infeasible (capacity, time windows, or vehicle cap cannot be satisfied)"),
        error.InvalidInstance, error.InvalidMatrix => respondError(req, .unprocessable_entity, "instance is invalid: check array lengths and that the depot slots (demand[0], ready[0], service[0]) are 0"),
        error.OutOfMemory => respondError(req, .internal_server_error, "out of memory"),
        else => respondError(req, .internal_server_error, "solver failed"),
    };
}

fn respondJson(arena: std.mem.Allocator, req: *std.http.Server.Request, value: anytype) !void {
    const json = std.json.Stringify.valueAlloc(arena, value, .{}) catch
        return respondError(req, .internal_server_error, "out of memory");
    try req.respond(json, .{
        .extra_headers = &.{.{ .name = "content-type", .value = "application/json" }},
    });
}

fn respondError(req: *std.http.Server.Request, status: std.http.Status, message: []const u8) !void {
    var buf: [512]u8 = undefined;
    const json = std.fmt.bufPrint(&buf, "{{\"error\":\"{s}\"}}", .{message}) catch "{\"error\":\"internal\"}";
    try req.respond(json, .{
        .status = status,
        .extra_headers = &.{.{ .name = "content-type", .value = "application/json" }},
    });
}
