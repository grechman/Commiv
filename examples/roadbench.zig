const std = @import("std");
const commiv = @import("commiv");

// REAL-DATA asymmetric road benchmark — the answer to "does directional routing
// actually pay off on a real city?" Unlike cvrpbench's CB_ASYM (which perturbs a
// symmetric TSPLIB instance into a synthetic directed one), this loads a directed
// travel-time matrix fetched from OSRM on a real road network (see
// tools/fetch_road_matrix.py and vendor/road/*.road) and reports:
//
//   1. CONSERVATIVENESS — the Helmholtz-Hodge decomposition of the asymmetry:
//      asym_magnitude (how directional) and curl_fraction (how much of that
//      asymmetry is non-conservative, i.e. survives a closed route and therefore
//      can change optimal routes). This is the scientific gate.
//   2. IGNORE-DIRECTION PENALTY — solve natively-asymmetric (SISR on the true
//      directed matrix) vs symmetric-treatment (symmetrize, solve, then score the
//      symmetric solution back on the TRUE directed matrix). The % gap is the real
//      cost of pretending the roads are undirected.
//
// Env: RB_DIR (default vendor/road), RB_FILES (comma list, default moscow-100),
//      RB_ITERS (SISR iterations), RB_THREADS, RB_SEED.
pub fn main(init: std.process.Init) !void {
    const allocator = init.gpa;
    const env = init.environ_map;
    const dir = env.get("RB_DIR") orelse "vendor/road";
    const files = env.get("RB_FILES") orelse "moscow-100";
    const seed = try std.fmt.parseInt(u64, env.get("RB_SEED") orelse "12345", 10);
    const iters = try std.fmt.parseInt(usize, env.get("RB_ITERS") orelse "300000", 10);
    const threads = try std.fmt.parseInt(usize, env.get("RB_THREADS") orelse "3", 10);
    const do_sym = !std.mem.eql(u8, env.get("RB_SYM") orelse "1", "0"); // RB_SYM=0 -> native-only, skip the symmetric-treatment solve

    std.debug.print("instance,dim,asym_mag,curl_frac,mean_ratio,native_cost,sym_cost,ignore_dir_penalty_pct,routes,ms\n", .{});
    var it = std.mem.tokenizeScalar(u8, files, ',');
    while (it.next()) |name| {
        const bytes = try readRoadBytes(init.io, allocator, dir, name);
        defer allocator.free(bytes);

        var road = try parseRoad(allocator, bytes);
        defer road.deinit(allocator);
        const dim = road.dim;
        const inst = commiv.CvrpInstance{
            .n = dim - 1,
            .matrix = road.matrix,
            .demand = road.demand,
            .capacity = road.capacity,
        };

        // 1. conservativeness / curl decomposition (the scientific gate)
        const cons = try commiv.conservativeness(allocator, road.matrix, dim);

        const sp = commiv.CvrpSisrParams{ .iters = iters };
        const solve_opts = commiv.SolveOptions{
            .seed = seed,
            .budget = .{ .trials = @min(dim, 100), .trial_extension_factor = 2, .max_passes = 60 },
            .candidates = .{ .candidate_count = @min(@as(usize, 10), dim - 2), .candidate_mode = .alpha_nearness, .sparse_min_dimension = 0 },
            .search = .{ .enable_lk = true, .lk_max_depth = 5 },
        };

        // 2a. native asymmetric solve on the TRUE directed matrix
        const t0 = nanos();
        var native = try commiv.solveCvrpSisrParallel(allocator, inst, solve_opts, sp, threads);
        defer native.deinit();
        const ms = @as(f64, @floatFromInt(nanos() - t0)) / 1e6;

        // 2b. symmetric-treatment: symmetrize, solve, score back on the directed matrix.
        // Skipped under RB_SYM=0 (native-only timing — the second solve is pure overhead
        // when all you want is our cost on the true directed matrix).
        var cost_sym: u64 = 0;
        if (do_sym) {
            const sym = try allocator.alloc(u32, dim * dim);
            defer allocator.free(sym);
            for (0..dim) |i| for (0..dim) |j| {
                sym[i * dim + j] = if (i == j) 0 else @intCast((@as(u64, road.matrix[i * dim + j]) + road.matrix[j * dim + i]) / 2);
            };
            var sym_res = try commiv.solveCvrpSisrParallel(allocator, .{ .n = dim - 1, .matrix = sym, .demand = road.demand, .capacity = road.capacity }, solve_opts, sp, threads);
            defer sym_res.deinit();
            for (sym_res.routes) |route| {
                var prev: usize = 0;
                for (route) |c| {
                    cost_sym += road.matrix[prev * dim + c];
                    prev = c;
                }
                cost_sym += road.matrix[prev * dim + 0];
            }
        }

        const pen = if (cost_sym == 0) 0.0 else 100.0 * (@as(f64, @floatFromInt(cost_sym)) - @as(f64, @floatFromInt(native.total_cost))) /
            @as(f64, @floatFromInt(native.total_cost));
        std.debug.print("{s},{},{d:.4},{d:.4},{d:.4},{},{},{d:.2},{},{d:.0}\n", .{
            name, dim, cons.asym_magnitude, cons.curl_fraction, cons.mean_ratio,
            native.total_cost, cost_sym, pen, native.routes.len, ms,
        });
    }
}

fn nanos() u64 {
    var ts: std.os.linux.timespec = undefined;
    _ = std.os.linux.clock_gettime(std.os.linux.CLOCK.MONOTONIC, &ts);
    return @as(u64, @intCast(ts.sec)) * 1_000_000_000 + @as(u64, @intCast(ts.nsec));
}

const Road = struct {
    dim: usize,
    capacity: u32,
    demand: []u32, // length dim, demand[0]=0
    matrix: []u32, // dim*dim, row-major, directed seconds
    fn deinit(self: *Road, allocator: std.mem.Allocator) void {
        allocator.free(self.demand);
        allocator.free(self.matrix);
        self.* = undefined;
    }
};

/// Load the raw bytes of `<dir>/<name>.road`. The moscow-5000 matrix is committed
/// only as `moscow-5000.road.gz` (the plain `.road` is gitignored), so on a fresh
/// checkout the uncompressed file is absent. When the `.road` read fails with
/// FileNotFound, fall back to a sibling `<name>.road.gz` and gzip-inflate it
/// in-process, so `zig build roadbench` works with no manual `gunzip` step. The
/// default (moscow-100) ships a plain `.road`, so it takes the direct path and the
/// inflate branch is never touched. Caller owns the returned slice.
fn readRoadBytes(io: std.Io, allocator: std.mem.Allocator, dir: []const u8, name: []const u8) ![]u8 {
    const cap = std.Io.Limit.limited(512 << 20);
    var path_buf: [512]u8 = undefined;
    const path = try std.fmt.bufPrint(&path_buf, "{s}/{s}.road", .{ dir, name });
    if (std.Io.Dir.cwd().readFileAlloc(io, path, allocator, cap)) |bytes| {
        return bytes;
    } else |err| switch (err) {
        error.FileNotFound => {}, // .road absent: try the committed .road.gz below
        else => return err,
    }

    var gz_path_buf: [512]u8 = undefined;
    const gz_path = try std.fmt.bufPrint(&gz_path_buf, "{s}/{s}.road.gz", .{ dir, name });
    // If neither <name>.road nor <name>.road.gz exists this propagates FileNotFound,
    // which is the genuine "no such instance" case (not the gzip-only case G18 targets).
    const gz_bytes = try std.Io.Dir.cwd().readFileAlloc(io, gz_path, allocator, cap);
    defer allocator.free(gz_bytes);

    // Inflate the gzip stream in-process (Zig 0.16 std.compress.flate).
    const flate = std.compress.flate;
    var input = std.Io.Reader.fixed(gz_bytes);
    const window = try allocator.alloc(u8, flate.max_window_len);
    defer allocator.free(window);
    var dec = flate.Decompress.init(&input, .gzip, window);
    return dec.reader.allocRemaining(allocator, cap);
}

/// Parse the line-based .road format written by tools/fetch_road_matrix.py.
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
            // Cap the dimension and use a checked multiply: a hostile/garbled file
            // could otherwise wrap dim*dim (silent in ReleaseFast) into an
            // undersized buffer and OOB-write, or demand an absurd allocation.
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
                // "<idx> <lng> <lat> <demand>" — keep only idx + demand
                const idx = try std.fmt.parseInt(usize, first, 10);
                _ = tok.next(); // lng
                _ = tok.next(); // lat
                const dem = try std.fmt.parseInt(u32, tok.next() orelse "0", 10);
                if (idx >= dim) return error.BadFormat;
                demand[idx] = dem;
            },
            .matrix => {
                if (row >= dim) return error.BadFormat;
                var col: usize = 0;
                // first token already consumed
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
