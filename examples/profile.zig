const std = @import("std");
const commiv = @import("commiv");

// Standalone single-instance profile driver mirroring bench.zig's mode
// parameters; used with `perf record` to chase per-node cost pathologies.
pub fn main(init: std.process.Init) !void {
    const allocator = init.gpa;
    const path = init.environ_map.get("PROF_PATH") orelse return error.MissingArg;
    const trials_arg = init.environ_map.get("PROF_TRIALS") orelse "0";
    const trials_in = try std.fmt.parseInt(usize, trials_arg, 10);

    const bytes = try std.Io.Dir.cwd().readFileAlloc(init.io, path, allocator, .limited(16 * 1024 * 1024));
    defer allocator.free(bytes);
    var diag: commiv.ParseDiagnostic = .{};
    var p = try commiv.parseTsplib(allocator, bytes, .{
        .diagnostic = &diag,
        .max_dimension = 20_000,
        .max_matrix_weights = 25_000_000,
    });
    defer p.deinit();
    const n = p.dimension;

    // Defaults mirror the bench's headline alpha-w8-kick mode exactly:
    // dimension trials, width 8, max_passes 64, backtrack limit 80k.
    const trials = if (trials_in == 0) n else trials_in;
    const btdepth_arg = init.environ_map.get("PROF_BTDEPTH") orelse "";
    const depth_arg = init.environ_map.get("PROF_DEPTH") orelse "5";
    const ext_arg = init.environ_map.get("PROF_EXT") orelse "0";
    const seed_arg = init.environ_map.get("PROF_SEED") orelse "12345";
    const width_arg = init.environ_map.get("PROF_WIDTH") orelse "0";
    // Roadmap item 6 knob: max distance-cache budget in BYTES. Default n*n*4
    // (force the full u32 matrix, as the bench does). Set PROF_MAXCACHE=0 to
    // force the on-the-fly coordinate path (no matrix) and measure the
    // cached-vs-uncached wall-clock at n>=10k — the item-6 gating measurement.
    const maxcache_arg = init.environ_map.get("PROF_MAXCACHE") orelse "";

    const start_ns = monotonicNanos();
    var result = try commiv.solve(allocator, &p, .{
        .seed = try std.fmt.parseInt(u64, seed_arg, 10),
        .trials = trials,
        .trial_extension_factor = try std.fmt.parseInt(usize, ext_arg, 10),
        .candidate_count = blk: {
            const w = try std.fmt.parseInt(usize, width_arg, 10);
            break :blk if (w != 0) w else if (n >= 1000) @as(usize, 5) else @as(usize, 8);
        },
        .candidate_mode = .alpha_nearness,
        .max_passes = 64,
        .enable_lk = true,
        .lk_max_depth = try std.fmt.parseInt(usize, depth_arg, 10),
        .lk_backtrack_depth = if (btdepth_arg.len > 0) try std.fmt.parseInt(usize, btdepth_arg, 10) else null,
        .lk_backtrack_limit = 80_000,
        .max_distance_cache_bytes = if (maxcache_arg.len > 0) try std.fmt.parseInt(usize, maxcache_arg, 10) else n * n * @sizeOf(u32),
    });
    defer result.deinit();
    const elapsed = monotonicNanos() - start_ns;
    std.debug.print("{s} n={} trials={} len={} time={d:.0}ms nodes={} best_trial={} max_prog_gap={} final_prog_gap={} worst_ratio={}\n", .{ p.name, n, trials, result.length, @as(f64, @floatFromInt(elapsed)) / 1e6, result.stats.lk_search_nodes, result.stats.best_trial, result.stats.eax_max_progress_gap, result.stats.eax_final_progress_gap, result.stats.eax_worst_gap_ratio_x100 });

    // Roadmap item-1 per-trial cost breakdown (gates items 2/6/8). Trial-loop
    // only; the one-time candidate build is excluded by oracle.resetCounters.
    const st = result.stats;
    const trials_f = @as(f64, @floatFromInt(@max(st.trials, 1)));
    std.debug.print(
        "  cost: dist_lookups={} ({d:.0}/trial) length_scans={} tour_rebuilds={} flip_ops={} flip_elements={} lk_nodes={} ({d:.0}/trial)\n",
        .{
            st.distance_lookups,
            @as(f64, @floatFromInt(st.distance_lookups)) / trials_f,
            st.tour_length_scans,
            st.tour_rebuilds,
            st.flip_ops,
            st.flip_elements,
            st.lk_search_nodes,
            @as(f64, @floatFromInt(st.lk_search_nodes)) / trials_f,
        },
    );

    if (init.environ_map.get("PROF_TOUR_OUT")) |out_path| {
        var buf: [64]u8 = undefined;
        var file = try std.Io.Dir.cwd().createFile(init.io, out_path, .{});
        defer file.close(init.io);
        var writer_buf: [4096]u8 = undefined;
        var fw = file.writer(init.io, &writer_buf);
        for (result.tour) |node| {
            const line = try std.fmt.bufPrint(&buf, "{}\n", .{node + 1});
            try fw.interface.writeAll(line);
        }
        try fw.interface.flush();
    }
}

fn monotonicNanos() u64 {
    var ts: std.os.linux.timespec = undefined;
    const rc = std.os.linux.clock_gettime(std.os.linux.CLOCK.MONOTONIC, &ts);
    std.debug.assert(rc == 0);
    return @as(u64, @intCast(ts.sec)) * 1_000_000_000 + @as(u64, @intCast(ts.nsec));
}
