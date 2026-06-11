const std = @import("std");
const commiv = @import("commiv");

// Standalone single-instance profile driver mirroring bench.zig's mode
// parameters; used with `perf record` to chase per-node cost pathologies.
pub fn main(init: std.process.Init) !void {
    const allocator = init.gpa;
    const path = init.environ_map.get("PROF_PATH") orelse return error.MissingArg;
    const trials_arg = init.environ_map.get("PROF_TRIALS") orelse "4";
    const trials = try std.fmt.parseInt(usize, trials_arg, 10);

    const bytes = try std.Io.Dir.cwd().readFileAlloc(init.io, path, allocator, .limited(16 * 1024 * 1024));
    defer allocator.free(bytes);
    var diag: commiv.ParseDiagnostic = .{};
    var p = try commiv.parseTsplib(allocator, bytes, .{
        .diagnostic = &diag,
        .max_dimension = 10_000,
        .max_matrix_weights = 25_000_000,
    });
    defer p.deinit();
    const n = p.dimension;

    const no_b3o = init.environ_map.get("PROF_NO_B3O") != null;
    const no_oropt = init.environ_map.get("PROF_NO_OROPT") != null;
    const nonseq0 = init.environ_map.get("PROF_NONSEQ0") != null;
    const depth_arg = init.environ_map.get("PROF_DEPTH") orelse "5";
    const backtrack_arg = init.environ_map.get("PROF_BACKTRACK") orelse "";

    const start_ns = monotonicNanos();
    var result = try commiv.solve(allocator, &p, .{
        .seed = 12345,
        .trials = trials,
        .candidate_count = if (n >= 500) 8 else 4,
        .candidate_mode = .alpha_nearness,
        .max_passes = if (n >= 500) 48 else 80,
        .enable_lk = true,
        .enable_bounded_three_opt_cleanup = !no_b3o,
        .enable_or_opt = !no_oropt,
        .lk_nonseq_branch_limit = if (nonseq0) 0 else 2,
        .lk_max_depth = try std.fmt.parseInt(usize, depth_arg, 10),
        .lk_backtrack_limit = if (backtrack_arg.len > 0) try std.fmt.parseInt(usize, backtrack_arg, 10) else if (n >= 500) 60_000 else 80_000,
        .max_distance_cache_weights = n * n,
    });
    defer result.deinit();
    const elapsed = monotonicNanos() - start_ns;
    std.debug.print("{s} n={} len={} time={d:.0}ms nodes={}\n", .{ p.name, n, result.length, @as(f64, @floatFromInt(elapsed)) / 1e6, result.stats.lk_search_nodes });
}

fn monotonicNanos() u64 {
    var ts: std.os.linux.timespec = undefined;
    const rc = std.os.linux.clock_gettime(std.os.linux.CLOCK.MONOTONIC, &ts);
    std.debug.assert(rc == 0);
    return @as(u64, @intCast(ts.sec)) * 1_000_000_000 + @as(u64, @intCast(ts.nsec));
}
