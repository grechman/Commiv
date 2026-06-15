const std = @import("std");
const problem = @import("problem.zig");
const solver = @import("solver.zig");

const SolveOptions = solver.SolveOptions;
const SolveResult = solver.SolveResult;
const SolveStats = solver.SolveStats;

// Island-model parallel solving. Trials are independent units of work and each
// solve() call is fully self-contained (own oracle, candidates, workspace,
// elite pool, and RNG seeded from options.seed), so K independent islands with
// distinct seeds run with zero shared mutable state. The Problem is read-only
// and shared. Results are NOT bit-identical to the single-core run (independent
// exploration) -- that is the point: we spend the cores the serial path cannot.
// Determinism is preserved per (seed, thread-count): island i uses seed+i and
// the winner is chosen by min length with a low-index tie-break, independent of
// thread scheduling.

pub const ParallelMode = enum {
    /// Each island runs the FULL trial budget with a distinct seed; the best
    /// tour wins. ~K x the compute at ~the single-island wall-time -> maximize
    /// quality.
    best_of_islands,
    /// The trial budget is split across islands (trials/K each); ~K x faster
    /// wall-time at roughly single-island quality -> maximize speed.
    split_budget,
};

pub const ParallelOptions = struct {
    /// 0 = auto: max(1, cpuCount - 1), always leaving one core free for the
    /// rest of the machine. 1 = serial: routes straight to solve() (so the
    /// single-core path stays bit-identical). >1 = that many islands.
    threads: usize = 0,
    mode: ParallelMode = .best_of_islands,
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
    // Per-island arena over the thread-safe page allocator: solve() allocates
    // everything here, and it is torn down before the worker returns. The
    // result tour (arena-owned) is copied into the parent-owned slot buffer
    // first, so nothing the parent reads outlives this frame's arena.
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

/// Run the solver across `par.threads` independent islands and return the best
/// result, owned by `allocator`. threads<=1 calls solve() directly.
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

    const islandOptions = struct {
        fn make(base: SolveOptions, idx: usize, kk: usize, total: usize, mode: ParallelMode) SolveOptions {
            var opts = base;
            opts.seed = base.seed +% @as(u64, idx);
            if (mode == .split_budget) opts.budget.trials = @max((total + kk - 1) / kk, 1);
            return opts;
        }
    }.make;

    var spawned: usize = 0;
    for (0..k) |i| {
        const opts = islandOptions(options, i, k, total_trials, par.mode);
        threads[i] = std.Thread.spawn(.{}, islandWorker, .{ p, opts, &slots[i] }) catch break;
        spawned += 1;
    }
    // If a spawn failed (e.g. thread limit), run the remainder inline so every
    // island still produces a result.
    for (spawned..k) |i| {
        const opts = islandOptions(options, i, k, total_trials, par.mode);
        islandWorker(p, opts, &slots[i]);
    }
    for (0..spawned) |i| threads[i].join();

    var best: ?usize = null;
    for (slots, 0..) |s, i| {
        if (!s.ok) continue;
        if (best == null or s.length < slots[best.?].length) best = i;
    }
    const winner = best orelse return error.AllIslandsFailed;

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

test "solveParallel: threads=1 routes to solve() (bit-identical)" {
    const allocator = std.testing.allocator;
    const coords = [_]problem.Coord{
        .{ .x = 0, .y = 0 },   .{ .x = 2, .y = 0 }, .{ .x = 4, .y = 0 },
        .{ .x = 6, .y = 1 },   .{ .x = 6, .y = 4 }, .{ .x = 4, .y = 6 },
        .{ .x = 2, .y = 6 },   .{ .x = 0, .y = 4 }, .{ .x = 1, .y = 2 },
        .{ .x = 5, .y = 3 },   .{ .x = 3, .y = 2 },
    };
    var p = try problem.Problem.initCoords(allocator, "ring", .euc_2d, &coords);
    defer p.deinit();
    const opts = SolveOptions{ .seed = 42, .budget = .{ .trials = 8, .max_passes = 40 }, .candidates = .{ .candidate_count = 6 } };

    var serial = try solver.solve(allocator, &p, opts);
    defer serial.deinit();
    var par = try solveParallel(allocator, &p, opts, .{ .threads = 1 });
    defer par.deinit();
    try std.testing.expectEqual(serial.length, par.length);
    try std.testing.expectEqualSlices(usize, serial.tour, par.tour);
}

test "solveParallel: best_of_islands is a valid tour no worse than serial" {
    const allocator = std.testing.allocator;
    const coords = [_]problem.Coord{
        .{ .x = 0, .y = 0 },   .{ .x = 2, .y = 0 }, .{ .x = 4, .y = 0 },
        .{ .x = 6, .y = 1 },   .{ .x = 6, .y = 4 }, .{ .x = 4, .y = 6 },
        .{ .x = 2, .y = 6 },   .{ .x = 0, .y = 4 }, .{ .x = 1, .y = 2 },
        .{ .x = 5, .y = 3 },   .{ .x = 3, .y = 2 },
    };
    var p = try problem.Problem.initCoords(allocator, "ring", .euc_2d, &coords);
    defer p.deinit();
    const opts = SolveOptions{ .seed = 1, .budget = .{ .trials = 8, .max_passes = 40 }, .candidates = .{ .candidate_count = 6 } };

    var serial = try solver.solve(allocator, &p, opts);
    defer serial.deinit();
    inline for (.{ ParallelMode.best_of_islands, ParallelMode.split_budget }) |mode| {
        var par = try solveParallel(allocator, &p, opts, .{ .threads = 3, .mode = mode });
        defer par.deinit();
        try p.validateTour(par.tour);
        try std.testing.expectEqual(par.length, try p.tourLength(par.tour));
        // best_of runs >= the serial budget per island -> never worse; split
        // divides the budget so only assert validity + self-consistency there.
        if (mode == .best_of_islands) try std.testing.expect(par.length <= serial.length);
    }
}

test "resolveThreadCount leaves one core free on auto and honors explicit" {
    try std.testing.expectEqual(@as(usize, 4), resolveThreadCount(4));
    try std.testing.expectEqual(@as(usize, 1), resolveThreadCount(1));
    try std.testing.expect(resolveThreadCount(0) >= 1);
}
