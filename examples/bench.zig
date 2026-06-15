const std = @import("std");
const commiv = @import("commiv");

const Mode = struct {
    name: []const u8,
    enable_lk: bool,
    candidate_mode: commiv.solver.CandidateMode,
    trials: ?usize = null,
    // 0 = fixed trial count from `trials`; k > 0 = k * dimension trials.
    dimension_trials_scale: usize = 0,
    candidate_count: ?usize = null,
    max_passes: ?usize = null,
    lk_backtrack_limit: ?usize = null,
    lk_max_depth: usize = 5,
    extend_trials: bool = false,
};

const TsplibFixture = struct {
    name: []const u8,
    path: []const u8,
    tour_path: []const u8,
    optimum: u64,
    // Probe-budget rows for very large instances: run only the headline mode,
    // single seed, fixed trial count, no extension. The row tracks progress
    // across rounds; a full dimension-scaled budget would take hours.
    headline_only: bool = false,
    fixed_trials: ?usize = null,
};

const modes = [_]Mode{
    .{ .name = "warmup-only", .enable_lk = false, .candidate_mode = .nearest_distance },
    .{ .name = "nearest-lk", .enable_lk = true, .candidate_mode = .nearest_distance },
    .{ .name = "alpha-lk", .enable_lk = true, .candidate_mode = .alpha_nearness },
};

const tsplib_matrix_modes = [_]Mode{
    .{ .name = "alpha-w12-t4", .enable_lk = true, .candidate_mode = .alpha_nearness, .candidate_count = 12, .trials = 4, .max_passes = 64, .lk_backtrack_limit = 80_000 },
    .{ .name = "alpha-w24-t4", .enable_lk = true, .candidate_mode = .alpha_nearness, .candidate_count = 24, .trials = 4, .max_passes = 64, .lk_backtrack_limit = 80_000 },
    .{ .name = "alpha-w24-t8", .enable_lk = true, .candidate_mode = .alpha_nearness, .candidate_count = 24, .trials = 8, .max_passes = 64, .lk_backtrack_limit = 80_000 },
    // LKH-style trial budget: MaxTrials = DIMENSION with iterated kicks from the
    // best tour, narrow alpha candidates (coverage is 100% at width 8), and
    // stagnation-based extension so runs that are still improving at the
    // dimension budget keep going (the backtracking discipline made trials
    // several times cheaper than the budget convention assumes).
    .{ .name = "alpha-w8-kick", .enable_lk = true, .candidate_mode = .alpha_nearness, .dimension_trials_scale = 1, .max_passes = 64, .lk_backtrack_limit = 80_000, .extend_trials = true },
};

const fixtures = [_]TsplibFixture{
    .{ .name = "berlin52", .path = "vendor/tsplib/berlin52.tsp", .tour_path = ".zig-cache/lkh-tours/berlin52.tour", .optimum = 7542 },
    .{ .name = "eil76", .path = "vendor/tsplib/eil76.tsp", .tour_path = ".zig-cache/lkh-tours/eil76.tour", .optimum = 538 },
    .{ .name = "kroA100", .path = "vendor/tsplib/kroA100.tsp", .tour_path = ".zig-cache/lkh-tours/kroA100.tour", .optimum = 21282 },
    .{ .name = "bier127", .path = "vendor/tsplib/bier127.tsp", .tour_path = ".zig-cache/lkh-tours/bier127.tour", .optimum = 118282 },
    .{ .name = "rat195", .path = "vendor/tsplib/rat195.tsp", .tour_path = ".zig-cache/lkh-tours/rat195.tour", .optimum = 2323 },
    .{ .name = "ts225", .path = "vendor/tsplib/ts225.tsp", .tour_path = ".zig-cache/lkh-tours/ts225.tour", .optimum = 126643 },
    .{ .name = "a280", .path = "vendor/tsplib/a280.tsp", .tour_path = ".zig-cache/lkh-tours/a280.tour", .optimum = 2579 },
    .{ .name = "lin318", .path = "vendor/tsplib/lin318.tsp", .tour_path = ".zig-cache/lkh-tours/lin318.tour", .optimum = 42029 },
    .{ .name = "rd400", .path = "vendor/tsplib/rd400.tsp", .tour_path = ".zig-cache/lkh-tours/rd400.tour", .optimum = 15281 },
    .{ .name = "fl417", .path = "vendor/tsplib/fl417.tsp", .tour_path = ".zig-cache/lkh-tours/fl417.tour", .optimum = 11861 },
    .{ .name = "pcb442", .path = "vendor/tsplib/pcb442.tsp", .tour_path = ".zig-cache/lkh-tours/pcb442.tour", .optimum = 50778 },
    .{ .name = "att532", .path = "vendor/tsplib/att532.tsp", .tour_path = ".zig-cache/lkh-tours/att532.tour", .optimum = 27686 },
    .{ .name = "u574", .path = "vendor/tsplib/u574.tsp", .tour_path = ".zig-cache/lkh-tours/u574.tour", .optimum = 36905 },
    .{ .name = "rat575", .path = "vendor/tsplib/rat575.tsp", .tour_path = ".zig-cache/lkh-tours/rat575.tour", .optimum = 6773 },
    .{ .name = "d657", .path = "vendor/tsplib/d657.tsp", .tour_path = ".zig-cache/lkh-tours/d657.tour", .optimum = 48912 },
    .{ .name = "pr1002", .path = "vendor/tsplib/pr1002.tsp", .tour_path = ".zig-cache/lkh-tours/pr1002.tour", .optimum = 259045 },
    .{ .name = "fl1577", .path = "vendor/tsplib/fl1577.tsp", .tour_path = ".zig-cache/lkh-tours/fl1577.tour", .optimum = 22249 },
    .{ .name = "rl11849", .path = "vendor/tsplib/rl11849.tsp", .tour_path = ".zig-cache/lkh-tours/rl11849.tour", .optimum = 923288, .headline_only = true, .fixed_trials = 400 },
    // d18512 (n=18512) is gated OUT of the always-run suite: the 1.37 GB cached
    // matrix makes fixed_trials=400 take ~hours (item-0 TODO). Re-enable once
    // item 6 (on-the-fly distances) drops the big matrix. Probe it manually via
    // commiv-profile when needed.
    // .{ .name = "d18512", .path = "vendor/tsplib/d18512.tsp", .tour_path = ".zig-cache/lkh-tours/d18512.tour", .optimum = 645238, .headline_only = true, .fixed_trials = 400 },
};

pub fn main(init: std.process.Init) !void {
    const allocator = init.gpa;

    std.debug.print("instance,dimension,mode,seed,trials,candidate_count,length,optimum,gap_percent,time_ms,lk_moves,bounded_three_opt_cleanup_moves,bounded_three_opt_cleanup_attempts,lk_search_nodes,max_depth_reached,lk_nonseq_attempts,lk_nonseq_accepted,lk_nonseq_rejected,lk_nonseq_deepest_accepted_depth,lk_completion_attempts,lk_completion_accepted,lk_completion_2opt_hits,lk_completion_3opt_hits,lk_completion_patch_hits,lk_completion_rejected,candidate_nearest_edges,candidate_alpha_edges,candidate_geometric_edges,candidate_patch_edges,move_plan_attempts,move_plan_direct_applies,move_plan_invalid_fallbacks,move_plan_multi_component_fallbacks,move_plan_apply_fallbacks,move_plan_fallback_successes,move_plan_patch_attempts,move_plan_patch_hits,move_plan_patch_rejected,eax_merge_attempts,eax_merge_cycles,eax_merge_wins,guided_trials,guided_polishes,best_trial,guided_search_nodes,merge_search_nodes\n", .{});
    try runGeneratedInstance(allocator, "clustered80", 80);
    try runGeneratedInstance(allocator, "clustered160", 160);
    try runTsplibFixtures(allocator, init.io);
}

fn runGeneratedInstance(allocator: std.mem.Allocator, name: []const u8, n: usize) !void {
    const coords = try allocator.alloc(commiv.Coord, n);
    defer allocator.free(coords);
    makeClustered(coords);

    var p = try commiv.Problem.initCoords(allocator, name, .euc_2d, coords);
    defer p.deinit();

    try runProblem(allocator, &p, null, null);
}

fn runTsplibFixtures(allocator: std.mem.Allocator, io: std.Io) !void {
    var loaded: usize = 0;
    for (fixtures) |fixture| {
        const bytes = std.Io.Dir.cwd().readFileAlloc(io, fixture.path, allocator, .limited(16 * 1024 * 1024)) catch |err| switch (err) {
            error.FileNotFound => {
                std.debug.print("# missing fixture: {s} expected at {s}\n", .{ fixture.name, fixture.path });
                continue;
            },
            else => return err,
        };
        defer allocator.free(bytes);

        var diag: commiv.ParseDiagnostic = .{};
        var p = commiv.parseTsplib(allocator, bytes, .{
            .diagnostic = &diag,
            .max_dimension = 20_000,
            .max_matrix_weights = 25_000_000,
        }) catch |err| {
            std.debug.print("# failed fixture: {s} line {} {s}\n", .{ fixture.name, diag.line, diag.message });
            return err;
        };
        defer p.deinit();
        if (!std.mem.eql(u8, p.name, fixture.name)) {
            std.debug.print("# fixture name mismatch: expected {s} parsed {s}\n", .{ fixture.name, p.name });
            return error.InvalidFixtureName;
        }
        try reportCandidateCoverage(allocator, io, &p, fixture);
        try runProblem(allocator, &p, fixture.optimum, fixture);
        loaded += 1;
    }
    if (loaded == 0) std.debug.print("# no TSPLIB fixtures loaded; add .tsp files under vendor/tsplib to enable gap benchmarks\n", .{});
}

fn reportCandidateCoverage(allocator: std.mem.Allocator, io: std.Io, p: *const commiv.Problem, fixture: TsplibFixture) !void {
    const tour_bytes = std.Io.Dir.cwd().readFileAlloc(io, fixture.tour_path, allocator, .limited(2 * 1024 * 1024)) catch |err| switch (err) {
        error.FileNotFound => return,
        else => return err,
    };
    defer allocator.free(tour_bytes);

    const tour = try parseTourFile(allocator, tour_bytes, p.dimension);
    defer allocator.free(tour);
    try p.validateTour(tour);

    const widths = [_]usize{ 4, 8, 12, 24 };
    for (widths) |requested_width| {
        const width = @min(requested_width, p.dimension - 1);
        var oracle = try commiv.solver.DistanceOracle.init(allocator, p, p.dimension * p.dimension);
        defer oracle.deinit();
        var stats: commiv.solver.CandidateBuildStats = .{};
        var candidates = try commiv.solver.buildCandidates(allocator, &oracle, width, .alpha_nearness, 32, 2, &stats);
        defer candidates.deinit();
        const covered = countTourEdgesInCandidates(&candidates, tour);
        const total = p.dimension;
        const pct = 100.0 * @as(f64, @floatFromInt(covered)) / @as(f64, @floatFromInt(total));
        std.debug.print("# candidate_coverage,{s},alpha,{},{},{d:.2}\n", .{ p.name, width, covered, pct });
    }
}

fn parseTourFile(allocator: std.mem.Allocator, bytes: []const u8, dimension: usize) ![]usize {
    var tour = try allocator.alloc(usize, dimension);
    errdefer allocator.free(tour);
    var count: usize = 0;
    var in_section = false;
    var it = std.mem.tokenizeAny(u8, bytes, " \t\r\n");
    while (it.next()) |token| {
        if (!in_section) {
            if (std.mem.eql(u8, token, "TOUR_SECTION")) in_section = true;
            continue;
        }
        const raw = try std.fmt.parseInt(isize, token, 10);
        if (raw == -1) break;
        if (raw <= 0 or @as(usize, @intCast(raw)) > dimension) return error.InvalidTourFile;
        if (count >= dimension) return error.InvalidTourFile;
        tour[count] = @as(usize, @intCast(raw)) - 1;
        count += 1;
    }
    if (count != dimension) return error.InvalidTourFile;
    return tour;
}

fn countTourEdgesInCandidates(candidates: *const commiv.solver.Candidates, tour: []const usize) usize {
    var covered: usize = 0;
    for (tour, 0..) |a, idx| {
        const b = tour[(idx + 1) % tour.len];
        if (candidateRowContains(candidates.row(a), b) or candidateRowContains(candidates.row(b), a)) covered += 1;
    }
    return covered;
}

fn candidateRowContains(row: []const usize, node: usize) bool {
    for (row) |candidate| {
        if (candidate == node) return true;
    }
    return false;
}

// The headline mode runs three seeds: single-seed rows misranked variants
// four times in rounds 11-14 (knife-edge optima flip on trajectory luck).
// The remaining modes are diagnostics and stay single-seed.
const headline_seeds = [_]u64{ 12345, 7, 99 };

fn runProblem(allocator: std.mem.Allocator, p: *const commiv.Problem, optimum: ?u64, fixture: ?TsplibFixture) !void {
    const probe = fixture != null and fixture.?.headline_only;
    if (!probe) {
        for (modes) |mode| try runMode(allocator, p, optimum, mode, 12345, null);
    }
    if (optimum != null) {
        for (tsplib_matrix_modes) |mode| {
            if (!mode.extend_trials) {
                if (!probe) try runMode(allocator, p, optimum, mode, 12345, null);
                continue;
            }
            if (probe) {
                try runMode(allocator, p, optimum, mode, 12345, fixture.?.fixed_trials);
            } else {
                for (headline_seeds) |seed| try runMode(allocator, p, optimum, mode, seed, null);
            }
        }
    }
}

fn runMode(allocator: std.mem.Allocator, p: *const commiv.Problem, optimum: ?u64, mode: Mode, seed: u64, fixed_trials: ?usize) !void {
    const n = p.dimension;
    const trials = fixed_trials orelse if (mode.dimension_trials_scale > 0) n * mode.dimension_trials_scale else mode.trials orelse if (n >= 500) @as(usize, 4) else @as(usize, 8);
    const candidate_count = mode.candidate_count orelse if (mode.extend_trials)
        (if (n >= 1000) @as(usize, 5) else @as(usize, 8))
    else if (n >= 500) @as(usize, 8) else @as(usize, 4);
    const max_passes = mode.max_passes orelse if (n >= 500) @as(usize, 48) else @as(usize, 80);
    const lk_backtrack_limit = mode.lk_backtrack_limit orelse if (n >= 500) @as(usize, 60_000) else @as(usize, 80_000);
    const start_ns = monotonicNanos();
    // Very large instances pay several ms per trial and keep finding
    // hairline improvements; factor 2 buys the same quality as 4 there at
    // half the extension cost.
    const trial_extension_factor: usize = if (!mode.extend_trials or fixed_trials != null) 0 else if (n >= 1000) 2 else 4;
    var result = try commiv.solve(allocator, p, .{
        .seed = seed,
        .trials = trials,
        .trial_extension_factor = trial_extension_factor,
        .candidate_count = candidate_count,
        .candidate_mode = mode.candidate_mode,
        .max_passes = max_passes,
        .enable_lk = mode.enable_lk,
        .lk_max_depth = mode.lk_max_depth,
        .lk_backtrack_limit = lk_backtrack_limit,
        .max_distance_cache_weights = n * n,
    });
    defer result.deinit();
    const elapsed_ns = monotonicNanos() - start_ns;
    try p.validateTour(result.tour);

    const opt = optimum orelse 0;
    const gap_percent = if (optimum) |known|
        100.0 * (@as(f64, @floatFromInt(result.length)) - @as(f64, @floatFromInt(known))) / @as(f64, @floatFromInt(known))
    else
        0.0;
    std.debug.print("{s},{},{s},{},{},{},{},{},{d:.3},{d:.3},{},{},{},{},{},{},{},{},{}", .{
        p.name,
        n,
        mode.name,
        seed,
        result.stats.trials,
        result.stats.candidate_count,
        result.length,
        opt,
        gap_percent,
        @as(f64, @floatFromInt(elapsed_ns)) / 1_000_000.0,
        result.stats.lk_moves,
        result.stats.bounded_three_opt_cleanup_moves,
        result.stats.bounded_three_opt_cleanup_attempts,
        result.stats.lk_search_nodes,
        result.stats.max_depth_reached,
        result.stats.lk_nonseq_attempts,
        result.stats.lk_nonseq_accepted,
        result.stats.lk_nonseq_rejected,
        result.stats.lk_nonseq_deepest_accepted_depth,
    });
    std.debug.print(",{},{},{},{},{},{}", .{
        result.stats.lk_completion_attempts,
        result.stats.lk_completion_accepted,
        result.stats.lk_completion_2opt_hits,
        result.stats.lk_completion_3opt_hits,
        result.stats.lk_completion_patch_hits,
        result.stats.lk_completion_rejected,
    });
    std.debug.print(",{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{}\n", .{
        result.stats.candidate_nearest_edges,
        result.stats.candidate_alpha_edges,
        result.stats.candidate_geometric_edges,
        result.stats.candidate_patch_edges,
        result.stats.move_plan_attempts,
        result.stats.move_plan_direct_applies,
        result.stats.move_plan_invalid_fallbacks,
        result.stats.move_plan_multi_component_fallbacks,
        result.stats.move_plan_apply_fallbacks,
        result.stats.move_plan_fallback_successes,
        result.stats.move_plan_patch_attempts,
        result.stats.move_plan_patch_hits,
        result.stats.move_plan_patch_rejected,
        result.stats.eax_merge_attempts,
        result.stats.eax_merge_cycles,
        result.stats.eax_merge_wins,
        result.stats.guided_trials,
        result.stats.guided_polishes,
        result.stats.best_trial,
        result.stats.guided_search_nodes,
        result.stats.merge_search_nodes,
    });
}

fn monotonicNanos() u64 {
    var ts: std.os.linux.timespec = undefined;
    const rc = std.os.linux.clock_gettime(std.os.linux.CLOCK.MONOTONIC, &ts);
    std.debug.assert(rc == 0);
    return @as(u64, @intCast(ts.sec)) * 1_000_000_000 + @as(u64, @intCast(ts.nsec));
}

fn makeClustered(coords: []commiv.Coord) void {
    var prng = std.Random.DefaultPrng.init(0xC0FFEE);
    const random = prng.random();
    const centers = [_]commiv.Coord{
        .{ .x = 100, .y = 100 },
        .{ .x = 900, .y = 120 },
        .{ .x = 820, .y = 820 },
        .{ .x = 140, .y = 760 },
        .{ .x = 500, .y = 480 },
    };
    for (coords, 0..) |*coord, i| {
        const c = centers[i % centers.len];
        const ring: f64 = @floatFromInt((i * 37) % 53);
        const jitter_x: f64 = @floatFromInt(random.intRangeLessThan(i32, -35, 36));
        const jitter_y: f64 = @floatFromInt(random.intRangeLessThan(i32, -35, 36));
        coord.* = .{
            .x = c.x + ring * 2.7 + jitter_x,
            .y = c.y + @mod(ring * 17.0, 91.0) + jitter_y,
        };
    }
}
