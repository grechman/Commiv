const std = @import("std");
const problem = @import("problem.zig");
const solver = @import("solver.zig");
const recombine = @import("recombine.zig");

const SolveOptions = solver.SolveOptions;
const SolveResult = solver.SolveResult;
const SolveStats = solver.SolveStats;
const SharedElitePool = recombine.SharedElitePool;

// Island-model parallel solving. Trials are independent units of work and each
// solve() call is fully self-contained (own oracle, candidates, workspace,
// elite pool, RNG seeded from options.seed), so islands with distinct seeds run
// with no shared mutable state beyond the optional cooperative pool. The Problem
// is read-only and shared. Results are NOT bit-identical to the single-core run
// (independent exploration), and cooperative runs are non-deterministic
// (migration timing); the winner is still chosen by min length with a low-index
// tie-break.
//
// Both modes split the trial budget across islands (trials/K each) for the ~K x
// wall-time speedup. `cooperative` additionally shares each island's best tour
// through a thread-safe pool, so islands recombine each other's discoveries --
// the experiment to claw back the accuracy `split_budget` gives up.

pub const ParallelMode = enum {
    /// Independent islands, budget split K ways. ~K x faster, lower accuracy.
    split_budget,
    /// Split-budget islands that migrate best tours through a shared pool, to
    /// recover accuracy without giving up the speedup.
    cooperative,
};

pub const ParallelOptions = struct {
    /// 0 = auto: max(1, cpuCount - 1), always leaving one core free for the rest
    /// of the machine. 1 = serial: routes straight to solve() (bit-identical).
    /// >1 = that many islands.
    threads: usize = 0,
    mode: ParallelMode = .split_budget,
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

fn islandWorker(p: *const problem.Problem, options: SolveOptions, shared: ?*SharedElitePool, slot: *IslandSlot) void {
    // Per-island arena over the thread-safe page allocator: the solve allocates
    // everything here and it is torn down before the worker returns. The result
    // tour (arena-owned) is copied into the parent-owned slot buffer first, so
    // nothing the parent reads outlives this frame's arena.
    var arena = std.heap.ArenaAllocator.init(std.heap.page_allocator);
    defer arena.deinit();
    const result = solver.solveWithSharedPool(arena.allocator(), p, options, shared) catch {
        slot.ok = false;
        return;
    };
    @memcpy(slot.tour, result.tour);
    slot.length = result.length;
    slot.stats = result.stats;
    slot.ok = true;
}

fn islandOptions(base: SolveOptions, idx: usize, k: usize, total_trials: usize) SolveOptions {
    var opts = base;
    opts.seed = base.seed +% @as(u64, idx);
    opts.budget.trials = @max((total_trials + k - 1) / k, 1);
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

    var shared: ?SharedElitePool = if (par.mode == .cooperative) try SharedElitePool.init(allocator, n) else null;
    defer if (shared) |*sp| sp.deinit();
    const shared_ptr: ?*SharedElitePool = if (shared) |*sp| sp else null;

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

    var spawned: usize = 0;
    for (0..k) |i| {
        const opts = islandOptions(options, i, k, total_trials);
        threads[i] = std.Thread.spawn(.{}, islandWorker, .{ p, opts, shared_ptr, &slots[i] }) catch break;
        spawned += 1;
    }
    // If a spawn failed (e.g. thread limit), run the remainder inline so every
    // island still produces a result.
    for (spawned..k) |i| {
        const opts = islandOptions(options, i, k, total_trials);
        islandWorker(p, opts, shared_ptr, &slots[i]);
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

test "solveParallel: split and cooperative produce valid self-consistent tours" {
    const allocator = std.testing.allocator;
    var p = try problem.Problem.initCoords(allocator, "ring", .euc_2d, &ring_coords);
    defer p.deinit();
    const opts = SolveOptions{ .seed = 1, .budget = .{ .trials = 12, .max_passes = 40 }, .candidates = .{ .candidate_count = 6 } };

    inline for (.{ ParallelMode.split_budget, ParallelMode.cooperative }) |mode| {
        var par = try solveParallel(allocator, &p, opts, .{ .threads = 3, .mode = mode });
        defer par.deinit();
        try p.validateTour(par.tour);
        try std.testing.expectEqual(par.length, try p.tourLength(par.tour));
    }
}

test "resolveThreadCount leaves one core free on auto and honors explicit" {
    try std.testing.expectEqual(@as(usize, 4), resolveThreadCount(4));
    try std.testing.expectEqual(@as(usize, 1), resolveThreadCount(1));
    try std.testing.expect(resolveThreadCount(0) >= 1);
}
