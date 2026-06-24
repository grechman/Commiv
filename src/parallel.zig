const std = @import("std");
const problem = @import("problem.zig");
const solver = @import("solver.zig");

const SolveOptions = solver.SolveOptions;
const SolveResult = solver.SolveResult;
const SolveStats = solver.SolveStats;

// Island-model parallel solving. The trial budget is split across K independent
// islands (trials/K each) with distinct seeds; each solve() is fully
// self-contained (own oracle, candidates, workspace, elite pool, RNG seeded from
// options.seed) and the Problem is read-only, so there is no shared mutable
// state. The best island wins (min length, low-index tie-break), which makes the
// whole thing DETERMINISTIC per (seed, thread-count) -- a fixed ~K x speedup at a
// small, reproducible accuracy cost.
//
// (A cooperative variant that migrated tours between islands mid-search was built
// and removed: in-search migration content is thread-timing-dependent and the
// merge-then-adopt step amplifies it into chaotic run-to-run variance. The only
// deterministic way to recover the accuracy split gives up is to redo serial's
// recombination work -- i.e. just run serial, which already hits the optimum.)

pub const ParallelOptions = struct {
    /// 0 = auto: max(1, cpuCount - 1), always leaving one core free for the rest
    /// of the machine. 1 = serial: routes straight to solve() (bit-identical).
    /// >1 = that many islands.
    threads: usize = 0,
    /// Accuracy mode (2026-06-17). When false (default): SPLIT — the trial
    /// budget is divided across islands and the best island wins (a ~Kx SPEED
    /// mode at a small accuracy cost). When true: RECOMBINE — every island runs
    /// the FULL budget as an independent restart, then their K tours are
    /// EAX-merged (solver.recombineTours) into one. This converts idle cores
    /// into accuracy at fixed wall time: the independent restarts diverge
    /// (~73% of a single run's missed optimal edges appear in another run) and
    /// the merge stitches their shared backbone, beating the best single island
    /// (which best-of/split throw that diversity away). The merge can only
    /// improve on the best island, so this never loses to split's best-of.
    recombine: bool = false,
};

/// Resolve a requested thread count to an actual island count, leaving one core
/// free on auto.
pub fn resolveThreadCount(requested: usize) usize {
    if (requested != 0) return @max(requested, 1);
    const cpus = std.Thread.getCpuCount() catch 1;
    return @max(cpus -| 1, 1);
}

const IslandSlot = struct {
    tour: []usize, // caller-owned buffer the worker copies its result into
    length: u64,
    stats: SolveStats,
    ok: bool,
};

fn islandWorker(p: *const problem.Problem, options: SolveOptions, slot: *IslandSlot) void {
    // Per-island arena over the thread-safe page allocator: the solve allocates
    // everything here and it is torn down before the worker returns. The result
    // tour (arena-owned) is copied into the parent-owned slot buffer first, so
    // nothing the parent reads outlives this frame's arena.
    var arena = std.heap.ArenaAllocator.init(std.heap.page_allocator);
    defer arena.deinit();
    const result = solver.solve(arena.allocator(), p, options) catch {
        slot.ok = false;
        return;
    };
    @memcpy(slot.tour, result.tour);
    slot.length = result.length;
    slot.stats = result.stats;
    slot.ok = true;
}

fn islandOptions(base: SolveOptions, idx: usize, k: usize, total_trials: usize, split: bool) SolveOptions {
    var opts = base;
    opts.seed = base.seed +% @as(u64, idx);
    // Split mode rations the budget across islands (speed); recombine mode runs
    // each island at the full budget as an independent restart (accuracy).
    opts.budget.trials = if (split) @max((total_trials + k - 1) / k, 1) else @max(total_trials, 1);
    return opts;
}

/// Run the solver across `par.threads` islands and return the best result, owned
/// by `allocator`. threads<=1 calls solve() directly (bit-identical serial).
pub fn solveParallel(
    allocator: std.mem.Allocator,
    p: *const problem.Problem,
    options: SolveOptions,
    par: ParallelOptions,
) !SolveResult {
    const k = resolveThreadCount(par.threads);
    if (k <= 1) return solver.solve(allocator, p, options);

    const n = p.dimension;
    const total_trials = @max(options.budget.trials, 1);

    const slots = try allocator.alloc(IslandSlot, k);
    defer allocator.free(slots);

    var allocated: usize = 0;
    errdefer for (slots[0..allocated]) |s| allocator.free(s.tour);
    for (slots) |*s| {
        s.* = .{
            .tour = try allocator.alloc(usize, n),
            .length = std.math.maxInt(u64),
            .stats = .{},
            .ok = false,
        };
        allocated += 1;
    }

    const threads = try allocator.alloc(std.Thread, k);
    defer allocator.free(threads);

    const split = !par.recombine;
    var spawned: usize = 0;
    for (0..k) |i| {
        const opts = islandOptions(options, i, k, total_trials, split);
        threads[i] = std.Thread.spawn(.{}, islandWorker, .{ p, opts, &slots[i] }) catch break;
        spawned += 1;
    }
    // If a spawn failed (e.g. thread limit), run the remainder inline so every
    // island still produces a result.
    for (spawned..k) |i| {
        const opts = islandOptions(options, i, k, total_trials, split);
        islandWorker(p, opts, &slots[i]);
    }
    for (0..spawned) |i| threads[i].join();

    var best: ?usize = null;
    for (slots, 0..) |s, i| {
        if (!s.ok) continue;
        if (best == null or s.length < slots[best.?].length) best = i;
    }
    const winner = best orelse return error.AllIslandsFailed;

    // Recombine mode: EAX-merge every island's full-budget tour into one. The
    // merge is seeded with the best island, so the result can only improve on
    // best-of. Falls through to best-of when fewer than two islands succeeded
    // (nothing to recombine).
    if (par.recombine) {
        var ok_count: usize = 0;
        for (slots) |s| {
            if (s.ok) ok_count += 1;
        }
        if (ok_count >= 2) {
            const tours = try allocator.alloc([]const usize, ok_count);
            defer allocator.free(tours);
            const lens = try allocator.alloc(u64, ok_count);
            defer allocator.free(lens);
            var w: usize = 0;
            for (slots) |s| {
                if (!s.ok) continue;
                tours[w] = s.tour;
                lens[w] = s.length;
                w += 1;
            }
            if (solver.recombineTours(allocator, p, options, tours, lens)) |merged| {
                for (slots) |s| allocator.free(s.tour);
                return merged;
            } else |_| {
                // Recombination failed (e.g. OOM building its oracle); fall back
                // to the best island rather than failing the whole solve.
            }
        }
    }

    for (slots, 0..) |s, i| {
        if (i != winner) allocator.free(s.tour);
    }
    return SolveResult{
        .allocator = allocator,
        .tour = slots[winner].tour,
        .length = slots[winner].length,
        .stats = slots[winner].stats,
    };
}

const ring_coords = [_]problem.Coord{
    .{ .x = 0, .y = 0 },   .{ .x = 2, .y = 0 }, .{ .x = 4, .y = 0 },
    .{ .x = 6, .y = 1 },   .{ .x = 6, .y = 4 }, .{ .x = 4, .y = 6 },
    .{ .x = 2, .y = 6 },   .{ .x = 0, .y = 4 }, .{ .x = 1, .y = 2 },
    .{ .x = 5, .y = 3 },   .{ .x = 3, .y = 2 },
};

test "solveParallel: threads=1 routes to solve() (bit-identical)" {
    const allocator = std.testing.allocator;
    var p = try problem.Problem.initCoords(allocator, "ring", .euc_2d, &ring_coords);
    defer p.deinit();
    const opts = SolveOptions{ .seed = 42, .budget = .{ .trials = 8, .max_passes = 40 }, .candidates = .{ .candidate_count = 6 } };

    var serial = try solver.solve(allocator, &p, opts);
    defer serial.deinit();
    var par = try solveParallel(allocator, &p, opts, .{ .threads = 1 });
    defer par.deinit();
    try std.testing.expectEqual(serial.length, par.length);
    try std.testing.expectEqualSlices(usize, serial.tour, par.tour);
}

test "solveParallel: split islands produce a valid self-consistent tour" {
    const allocator = std.testing.allocator;
    var p = try problem.Problem.initCoords(allocator, "ring", .euc_2d, &ring_coords);
    defer p.deinit();
    const opts = SolveOptions{ .seed = 1, .budget = .{ .trials = 12, .max_passes = 40 }, .candidates = .{ .candidate_count = 6 } };

    var par = try solveParallel(allocator, &p, opts, .{ .threads = 3 });
    defer par.deinit();
    try p.validateTour(par.tour);
    try std.testing.expectEqual(par.length, try p.tourLength(par.tour));
}

test "solveParallel: recombine mode produces a valid tour no worse than serial" {
    const allocator = std.testing.allocator;
    var p = try problem.Problem.initCoords(allocator, "ring", .euc_2d, &ring_coords);
    defer p.deinit();
    const opts = SolveOptions{ .seed = 5, .budget = .{ .trials = 12, .max_passes = 40 }, .candidates = .{ .candidate_count = 6 } };

    // Island 0 uses the base seed at full budget == the serial run, so the
    // recombined result is structurally <= serial length (best-of includes it,
    // the merge only improves on best-of).
    var serial = try solver.solve(allocator, &p, opts);
    defer serial.deinit();
    var par = try solveParallel(allocator, &p, opts, .{ .threads = 3, .recombine = true });
    defer par.deinit();
    try p.validateTour(par.tour);
    try std.testing.expectEqual(par.length, try p.tourLength(par.tour));
    try std.testing.expect(par.length <= serial.length);
}

test "resolveThreadCount leaves one core free on auto and honors explicit" {
    try std.testing.expectEqual(@as(usize, 4), resolveThreadCount(4));
    try std.testing.expectEqual(@as(usize, 1), resolveThreadCount(1));
    try std.testing.expect(resolveThreadCount(0) >= 1);
}
