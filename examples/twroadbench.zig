const std = @import("std");
const commiv = @import("commiv");

// Road-network VRPTW benchmark: directed .road travel-time matrix + the
// deterministic time-window overlay every competitor gets (make_windows in
// tools/competitors/vrptw_moscow.py; dump it once with
// tools/competitors/dump_windows.py <city>). This is the harness behind the
// README's NYC/Berlin/Moscow TW cells.
// Env: TP_DIR (default vendor/road), TP_TW_DIR (where .tw overlays live,
//      default vendor/road), TP_FILE (instance name), TP_ITERS, TP_THREADS,
//      TP_SEED, TP_COMBO=1 (polish+stress0.5+tabu10000+marathon),
//      TP_POLISH_EVERY, TP_NBR (sum|min|out), TP_GK.
// Prints: instance,cost,vehicles,ms
pub fn main(init: std.process.Init) !void {
    const allocator = init.gpa;
    const env = init.environ_map;
    const dir = env.get("TP_DIR") orelse "vendor/road";
    const tw_dir = env.get("TP_TW_DIR") orelse "vendor/road";
    const name = env.get("TP_FILE") orelse "berlin-1000";
    const iters = try std.fmt.parseInt(usize, env.get("TP_ITERS") orelse "300000", 10);
    const threads = try std.fmt.parseInt(usize, env.get("TP_THREADS") orelse "3", 10);
    const seed = try std.fmt.parseInt(u64, env.get("TP_SEED") orelse "12345", 10);
    const combo = std.mem.eql(u8, env.get("TP_COMBO") orelse "0", "1");
    const nbr = env.get("TP_NBR") orelse "sum";
    const gk = try std.fmt.parseInt(usize, env.get("TP_GK") orelse "0", 10);

    var path_buf: [512]u8 = undefined;
    const path = try std.fmt.bufPrint(&path_buf, "{s}/{s}.road", .{ dir, name });
    const bytes = try std.Io.Dir.cwd().readFileAlloc(init.io, path, allocator, std.Io.Limit.limited(512 << 20));
    defer allocator.free(bytes);
    var road = try parseRoad(allocator, bytes);
    defer road.deinit(allocator);
    const dim = road.dim;

    var tw_buf: [512]u8 = undefined;
    const tw_path = try std.fmt.bufPrint(&tw_buf, "{s}/{s}.tw", .{ tw_dir, name });
    const tw_bytes = try std.Io.Dir.cwd().readFileAlloc(init.io, tw_path, allocator, std.Io.Limit.limited(16 << 20));
    defer allocator.free(tw_bytes);
    const ready = try allocator.alloc(u32, dim);
    defer allocator.free(ready);
    const due = try allocator.alloc(u32, dim);
    defer allocator.free(due);
    const service = try allocator.alloc(u32, dim);
    defer allocator.free(service);
    {
        var lines = std.mem.tokenizeScalar(u8, tw_bytes, '\n');
        const dline = lines.next() orelse return error.BadTw;
        var dtok = std.mem.tokenizeAny(u8, dline, " \t\r");
        const tw_dim = try std.fmt.parseInt(usize, dtok.next() orelse return error.BadTw, 10);
        if (tw_dim != dim) return error.TwDimMismatch;
        var i: usize = 0;
        while (lines.next()) |line| : (i += 1) {
            if (i >= dim) return error.BadTw;
            var tok = std.mem.tokenizeAny(u8, line, " \t\r");
            ready[i] = try std.fmt.parseInt(u32, tok.next() orelse return error.BadTw, 10);
            due[i] = try std.fmt.parseInt(u32, tok.next() orelse return error.BadTw, 10);
            service[i] = try std.fmt.parseInt(u32, tok.next() orelse return error.BadTw, 10);
        }
        if (i != dim) return error.BadTw;
    }

    const inst = commiv.VrptwInstance{
        .n = dim - 1,
        .matrix = road.matrix,
        .demand = road.demand,
        .capacity = road.capacity,
        .ready = ready,
        .due = due,
        .service = service,
    };
    const params = commiv.VrptwSisrParams{
        .iters = iters,
        .polish = combo,
        .stress_rate = if (combo) 0.5 else 0.0,
        .tabu_tenure = if (combo) 10000 else 0,
        .marathon = combo,
        .polish_every = try std.fmt.parseInt(usize, env.get("TP_POLISH_EVERY") orelse "1", 10),
        .nbr_key = if (std.mem.eql(u8, nbr, "min")) .min else if (std.mem.eql(u8, nbr, "out")) .out else .sum,
        .gk = gk,
        .final_ls = std.mem.eql(u8, env.get("TP_FINAL_LS") orelse "0", "1"),
    };
    const solve_opts = commiv.SolveOptions{
        .seed = seed,
        .budget = .{ .trials = @min(dim, 100), .trial_extension_factor = 2, .max_passes = 60 },
        .candidates = .{ .candidate_count = @min(@as(usize, 10), dim - 2), .candidate_mode = .alpha_nearness, .sparse_min_dimension = 0 },
        .search = .{ .enable_lk = true, .lk_max_depth = 5 },
    };
    const t0 = nanos();
    var res = if (threads > 1)
        try commiv.solveVrptwSisrParallel(allocator, inst, solve_opts, params, threads)
    else
        try commiv.solveVrptwSisr(allocator, inst, solve_opts, params);
    defer res.deinit();
    const ms = @as(f64, @floatFromInt(nanos() - t0)) / 1e6;
    std.debug.print("{s},{},{},{d:.0}\n", .{ name, res.total_cost, res.vehicles, ms });
}

fn nanos() u64 {
    var ts: std.os.linux.timespec = undefined;
    _ = std.os.linux.clock_gettime(std.os.linux.CLOCK.MONOTONIC, &ts);
    return @as(u64, @intCast(ts.sec)) * 1_000_000_000 + @as(u64, @intCast(ts.nsec));
}

const Road = struct {
    dim: usize,
    capacity: u32,
    demand: []u32,
    matrix: []u32,
    fn deinit(self: *Road, allocator: std.mem.Allocator) void {
        allocator.free(self.demand);
        allocator.free(self.matrix);
        self.* = undefined;
    }
};

fn parseRoad(allocator: std.mem.Allocator, bytes: []const u8) !Road {
    const Mode = enum { header, coords, matrix };
    var mode: Mode = .header;
    var dim: usize = 0;
    var capacity: u32 = 0;
    var demand: []u32 = &.{};
    var matrix: []u32 = &.{};
    errdefer if (demand.len > 0) allocator.free(demand);
    errdefer if (matrix.len > 0) allocator.free(matrix);
    var row: usize = 0;
    var lines = std.mem.tokenizeScalar(u8, bytes, '\n');
    while (lines.next()) |line| {
        var tok = std.mem.tokenizeAny(u8, line, " \t\r");
        const first = tok.next() orelse continue;
        if (std.mem.eql(u8, first, "NAME")) {
            continue;
        } else if (std.mem.eql(u8, first, "DIM")) {
            dim = try std.fmt.parseInt(usize, tok.next() orelse return error.BadFormat, 10);
            if (dim < 1 or dim > 100_000) return error.BadFormat;
            const cells = std.math.mul(usize, dim, dim) catch return error.BadFormat;
            demand = try allocator.alloc(u32, dim);
            @memset(demand, 0);
            matrix = try allocator.alloc(u32, cells);
            @memset(matrix, 0);
        } else if (std.mem.eql(u8, first, "CAPACITY")) {
            capacity = try std.fmt.parseInt(u32, tok.next() orelse return error.BadFormat, 10);
        } else if (std.mem.eql(u8, first, "COORDS")) {
            mode = .coords;
        } else if (std.mem.eql(u8, first, "MATRIX")) {
            mode = .matrix;
        } else switch (mode) {
            .coords => {
                const idx = try std.fmt.parseInt(usize, first, 10);
                _ = tok.next();
                _ = tok.next();
                const dem = try std.fmt.parseInt(u32, tok.next() orelse "0", 10);
                if (idx >= dim) return error.BadFormat;
                demand[idx] = dem;
            },
            .matrix => {
                if (row >= dim) return error.BadFormat;
                var col: usize = 0;
                var t = first;
                while (true) {
                    if (col >= dim) return error.BadFormat;
                    matrix[row * dim + col] = try std.fmt.parseInt(u32, t, 10);
                    col += 1;
                    t = tok.next() orelse break;
                }
                if (col != dim) return error.BadFormat;
                row += 1;
            },
            .header => {},
        }
    }
    if (dim == 0 or row != dim) return error.BadFormat;
    return .{ .dim = dim, .capacity = capacity, .demand = demand, .matrix = matrix };
}
