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
    // Roadmap item 6 knob: max distance-cache weights. Default n*n (force the
    // full matrix, as the bench does). Set PROF_MAXCACHE=0 to force the
    // on-the-fly coordinate path (no matrix) and measure the cached-vs-uncached
    // wall-clock at n>=10k — the item-6 gating measurement.
    const maxcache_arg = init.environ_map.get("PROF_MAXCACHE") orelse "";
    // Roadmap item 3 measurement knobs (default off => bit-identical baseline).
    const freeze_arg = init.environ_map.get("PROF_FREEZE") orelse "0";
    const freeze_minvotes_arg = init.environ_map.get("PROF_FREEZE_MINVOTES") orelse "64";
    const freeze_frac_arg = init.environ_map.get("PROF_FREEZE_FRAC") orelse "85";
    // PROF_FREEZE_MODE: 0=gated trials, 1=distinct incumbents. PROF_FREEZE_LK:
    // 1=LK respects frozen edges (default), 0=kick-only freeze.
    const freeze_mode_arg = init.environ_map.get("PROF_FREEZE_MODE") orelse "0";
    const freeze_lk_arg = init.environ_map.get("PROF_FREEZE_LK") orelse "1";
    const freeze_stale_arg = init.environ_map.get("PROF_FREEZE_STALE") orelse "0";
    const freeze_soft_arg = init.environ_map.get("PROF_FREEZE_SOFT") orelse "0";

    var frozen_edges: std.ArrayList(u32) = .empty;
    defer frozen_edges.deinit(allocator);
    const want_frozen = init.environ_map.get("PROF_FROZEN_OUT");

    // Item-3 revival: load an injected backbone (1-indexed "u v" per line),
    // pack lo<<32|hi and sort ascending for binary search inside the solver.
    var inject: std.ArrayList(u64) = .empty;
    defer inject.deinit(allocator);
    if (init.environ_map.get("PROF_FROZEN_IN")) |in_path| {
        const txt = try std.Io.Dir.cwd().readFileAlloc(init.io, in_path, allocator, .limited(8 * 1024 * 1024));
        defer allocator.free(txt);
        var it = std.mem.tokenizeAny(u8, txt, " \n\r\t");
        while (it.next()) |a_tok| {
            const b_tok = it.next() orelse break;
            const a = (try std.fmt.parseInt(usize, a_tok, 10)) - 1;
            const b = (try std.fmt.parseInt(usize, b_tok, 10)) - 1;
            const lo = @min(a, b);
            const hi = @max(a, b);
            try inject.append(allocator, (@as(u64, @intCast(lo)) << 32) | @as(u64, @intCast(hi)));
        }
        std.mem.sort(u64, inject.items, {}, std.sort.asc(u64));
        std.debug.print("  inject: loaded {} frozen backbone edges\n", .{inject.items.len});
    }

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
        .max_distance_cache_weights = if (maxcache_arg.len > 0) try std.fmt.parseInt(usize, maxcache_arg, 10) else n * n,
        .enable_edge_freeze = (try std.fmt.parseInt(u8, freeze_arg, 10)) != 0,
        .edge_freeze_min_votes = try std.fmt.parseInt(u32, freeze_minvotes_arg, 10),
        .edge_freeze_fraction_x100 = try std.fmt.parseInt(u32, freeze_frac_arg, 10),
        .edge_freeze_vote_mode = if ((try std.fmt.parseInt(u8, freeze_mode_arg, 10)) != 0) .distinct_incumbents else .gated_trials,
        .edge_freeze_lk_respect = (try std.fmt.parseInt(u8, freeze_lk_arg, 10)) != 0,
        .edge_freeze_stale_window = try std.fmt.parseInt(usize, freeze_stale_arg, 10),
        .edge_freeze_soft = (try std.fmt.parseInt(u8, freeze_soft_arg, 10)) != 0,
        .frozen_edges_out = if (want_frozen != null) &frozen_edges else null,
        .inject_frozen = inject.items,
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
    if (st.freeze_votes > 0) {
        std.debug.print(
            "  freeze: votes={} decrements={} move_rejections={} frozen_edges_final={}/{}\n",
            .{ st.freeze_votes, st.freeze_decrements, st.freeze_move_rejections, st.frozen_edges_final, n },
        );
    }

    if (want_frozen) |out_path| {
        var buf: [64]u8 = undefined;
        var file = try std.Io.Dir.cwd().createFile(init.io, out_path, .{});
        defer file.close(init.io);
        var writer_buf: [4096]u8 = undefined;
        var fw = file.writer(init.io, &writer_buf);
        var i: usize = 0;
        while (i + 1 < frozen_edges.items.len) : (i += 2) {
            // 1-indexed to match the .tour / PROF_TOUR_OUT convention.
            const line = try std.fmt.bufPrint(&buf, "{} {}\n", .{ frozen_edges.items[i] + 1, frozen_edges.items[i + 1] + 1 });
            try fw.interface.writeAll(line);
        }
        try fw.interface.flush();
    }

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
