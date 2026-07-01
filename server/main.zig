const std = @import("std");
const commiv = @import("commiv");

// commiv-serve: the language-agnostic front door. One static binary, JSON over
// HTTP, no dependencies — run it next to your app and every language that can
// speak HTTP gets near-optimal directed routes. See docs/rest.md for the schema
// and client snippets.
//
//   POST /solve/cvrp   {matrix, demand, capacity, seed?, iters?, threads?}
//   POST /solve/vrptw  {matrix, demand, capacity, ready, due, service,
//                       seed?, rounds?, restarts?, veh_penalty?}
//   POST /solve/atsp   {matrix, seed?, trials?}
//   GET  /health
//
// Env: COMMIV_HOST (default 127.0.0.1), COMMIV_PORT (default 8080),
//      COMMIV_MAX_BODY_MB (default 256).
//
// Requests are handled sequentially: a solve occupies the core budget anyway,
// so admission control is the client's job for now (front it with a queue if
// you need one). Parallelism inside one solve comes from `threads` in the body.

const version_string = "0.2.0";

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

    const Route = enum { cvrp, vrptw, atsp };
    const route: Route = if (std.mem.eql(u8, target, "/solve/cvrp"))
        .cvrp
    else if (std.mem.eql(u8, target, "/solve/vrptw"))
        .vrptw
    else if (std.mem.eql(u8, target, "/solve/atsp"))
        .atsp
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
        .atsp => return solveAtsp(arena, req, body),
    }
}

// ---- endpoint handlers -------------------------------------------------------

const CvrpRequest = struct {
    matrix: []const []const u32, // (n+1) rows of n+1 directed costs; depot = 0
    demand: []const u32,
    capacity: u32,
    seed: u64 = 1,
    iters: u64 = 0, // SISR iterations; 0 = default
    threads: u32 = 1, // >1 = parallel islands (result depends on the count)
};

fn solveCvrp(arena: std.mem.Allocator, req: *std.http.Server.Request, body: []const u8) !void {
    const r = parseBody(CvrpRequest, arena, req, body) orelse return;
    const flat = flattenSquare(arena, r.matrix) orelse
        return respondError(req, .unprocessable_entity, "matrix must be square (n+1 rows of n+1 entries)");
    const n = r.matrix.len - 1;
    if (r.matrix.len < 2 or r.demand.len != n + 1)
        return respondError(req, .unprocessable_entity, "need >= 1 customer and demand of length n+1");

    const inst = commiv.CvrpInstance{ .n = n, .matrix = flat, .demand = r.demand, .capacity = r.capacity };
    var params: commiv.CvrpSisrParams = .{};
    if (r.iters != 0) params.iters = @intCast(r.iters);
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
    matrix: []const []const u32,
    demand: []const u32,
    capacity: u32,
    ready: []const u32, // earliest service start, ready[0] = 0
    due: []const u32, // latest service start, due[0] = depot horizon
    service: []const u32, // service durations, service[0] = 0
    seed: u64 = 1,
    rounds: u64 = 0,
    restarts: u64 = 0,
    veh_penalty: u64 = 0,
};

fn solveVrptw(arena: std.mem.Allocator, req: *std.http.Server.Request, body: []const u8) !void {
    const r = parseBody(VrptwRequest, arena, req, body) orelse return;
    const flat = flattenSquare(arena, r.matrix) orelse
        return respondError(req, .unprocessable_entity, "matrix must be square (n+1 rows of n+1 entries)");
    const n = r.matrix.len - 1;
    if (r.matrix.len < 2 or r.demand.len != n + 1 or r.ready.len != n + 1 or r.due.len != n + 1 or r.service.len != n + 1)
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
    var params: commiv.VrptwParams = .{ .veh_penalty = r.veh_penalty };
    if (r.rounds != 0) params.rounds = @intCast(r.rounds);
    if (r.restarts != 0) params.restarts = @intCast(r.restarts);

    const result = commiv.solveVrptw(arena, inst, .{ .seed = r.seed }, params) catch |err|
        return respondSolverError(req, err);

    return respondJson(arena, req, .{
        .total_cost = result.total_cost,
        .vehicles = result.routes.len,
        .routes = result.routes,
    });
}

const AtspRequest = struct {
    matrix: []const []const u32, // n x n directed costs
    seed: u64 = 1,
    trials: u64 = 0,
};

fn solveAtsp(arena: std.mem.Allocator, req: *std.http.Server.Request, body: []const u8) !void {
    const r = parseBody(AtspRequest, arena, req, body) orelse return;
    const flat = flattenSquare(arena, r.matrix) orelse
        return respondError(req, .unprocessable_entity, "matrix must be square");
    const n = r.matrix.len;
    if (n < 2) return respondError(req, .unprocessable_entity, "need at least 2 nodes");

    var opts: commiv.SolveOptions = .{ .seed = r.seed };
    if (r.trials != 0) opts.budget.trials = @intCast(r.trials);

    const result = commiv.solveAtsp(arena, flat, n, opts) catch |err|
        return respondSolverError(req, err);

    return respondJson(arena, req, .{ .cost = result.length, .tour = result.tour });
}

// ---- plumbing ----------------------------------------------------------------

fn parseBody(comptime T: type, arena: std.mem.Allocator, req: *std.http.Server.Request, body: []const u8) ?T {
    return std.json.parseFromSliceLeaky(T, arena, body, .{ .ignore_unknown_fields = true }) catch {
        respondError(req, .bad_request, "invalid JSON body; see docs/rest.md for the schema") catch {};
        return null;
    };
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
        error.Infeasible, error.NoFeasibleSplit => respondError(req, .unprocessable_entity, "instance is infeasible (capacity or time windows cannot be satisfied)"),
        error.InvalidInstance, error.InvalidMatrix => respondError(req, .unprocessable_entity, "instance shape is invalid"),
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
