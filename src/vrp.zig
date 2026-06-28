const std = @import("std");
const solver = @import("solver.zig");
const cvrp_types = @import("cvrp_types.zig");
const cvrp_split = @import("cvrp_split.zig");
const cvrp_solution = @import("cvrp_solution.zig");
const cvrp_hgs = @import("cvrp_hgs.zig");
const cvrp_sisr = @import("cvrp_sisr.zig");

// Capacitated VRP (CVRP) facade. The implementation was split along
// responsibility seams (G11) into cvrp_types/cvrp_split/cvrp_solution/cvrp_hgs/
// cvrp_sisr; this file re-exports the exact public surface unchanged (Zig 0.16
// has no usingnamespace, so every symbol is an explicit `pub const`). The thin
// dispatch wrappers and the CVRP test-suite stay here.

// ---- public types ----
pub const CvrpInstance = cvrp_types.CvrpInstance;
pub const CvrpResult = cvrp_types.CvrpResult;
pub const HgsParams = cvrp_hgs.HgsParams;
pub const SisrParams = cvrp_sisr.SisrParams;

// ---- public solvers ----
pub const solveCvrpHgs = cvrp_hgs.solveCvrpHgs;
pub const solveCvrpHgsParallel = cvrp_hgs.solveCvrpHgsParallel;
pub const solveCvrpSisr = cvrp_sisr.solveCvrpSisr;
pub const solveCvrpSisrParallel = cvrp_sisr.solveCvrpSisrParallel;
pub const validate = cvrp_solution.validate;

// ---- internal aliases for the dispatch wrappers + tests below ----
const solveCvrpImpl = cvrp_solution.solveCvrpImpl;
const splitDp = cvrp_split.splitDp;

// Pull the split-out modules into the test/build graph (their unit tests, if
// any, are discovered via root.zig's `_ = @import("vrp.zig")`).
test {
    _ = @import("cvrp_types.zig");
    _ = @import("cvrp_split.zig");
    _ = @import("cvrp_solution.zig");
    _ = @import("cvrp_hgs.zig");
    _ = @import("cvrp_sisr.zig");
}


/// Solve CVRP with the default, highest-quality strategy: SISR (ruin-and-recreate
/// + simulated annealing), which is the strongest solver here for large and/or
/// directed (asymmetric) instances and a solid choice at every size. Reach for a
/// specific entry point when you need it: `solveCvrpMulti` / `solveCvrpFleet` for
/// the giant-tour ILS variants, `solveCvrpHgs` for the mid-size population method.
/// Capacity is enforced exactly. Tune the search via `CvrpSisrParams` on
/// `solveCvrpSisr` directly. Returns `error.NoFeasibleSplit` if no packing exists.
pub fn solveCvrp(allocator: std.mem.Allocator, inst: CvrpInstance, options: solver.SolveOptions) !CvrpResult {
    return solveCvrpSisr(allocator, inst, options, .{});
}

/// Tuning for the giant-tour ILS variant `solveCvrpFleet`. `rounds` is the ILS
/// perturbation steps per chain; `restarts` is the number of independent best-of-K
/// chains; `max_vehicles` is a hard fleet cap (0 = unlimited). One named field per
/// knob so the three same-typed `usize` values can't be silently transposed.
pub const CvrpFleetParams = struct {
    rounds: usize = 30,
    restarts: usize = 1,
    max_vehicles: usize = 0,
};

/// Tuning for the uncapped giant-tour ILS variant `solveCvrpMulti`: `rounds`
/// perturbation steps per chain over `restarts` independent best-of-K chains.
pub const CvrpMultiParams = struct {
    rounds: usize = 30,
    restarts: usize = 1,
};

/// solveCvrpMulti, but with a hard fleet cap: at most `params.max_vehicles` routes
/// (0 = unlimited). For fixed-fleet ACVRP / real distribution where the vehicle
/// count is a constraint, not free.
pub fn solveCvrpFleet(allocator: std.mem.Allocator, inst: CvrpInstance, options: solver.SolveOptions, params: CvrpFleetParams) !CvrpResult {
    return solveCvrpImpl(allocator, inst, options, params.rounds, params.restarts, params.max_vehicles);
}

/// As solveCvrp but with `params.restarts` independent ILS chains sharing one giant
/// tour, each with a distinct perturbation seed, keeping the global best. The
/// ILS is high-variance per chain (a single unlucky seed can sit 3% above the
/// optimum), so best-of-K reliably tightens the gap for a roughly K-times budget.
pub fn solveCvrpMulti(allocator: std.mem.Allocator, inst: CvrpInstance, options: solver.SolveOptions, params: CvrpMultiParams) !CvrpResult {
    return solveCvrpImpl(allocator, inst, options, params.rounds, params.restarts, 0);
}


test "CVRP end-to-end: valid, feasible, beats one-per-route baseline" {
    const allocator = std.testing.allocator;
    const n = 8;
    const dim = n + 1;
    var prng = std.Random.DefaultPrng.init(0xCAFE);
    const rng = prng.random();
    const matrix = try allocator.alloc(u32, dim * dim);
    defer allocator.free(matrix);
    for (0..dim) |i| {
        for (0..dim) |j| matrix[i * dim + j] = if (i == j) 0 else rng.intRangeAtMost(u32, 1, 100);
    }
    const demand = try allocator.alloc(u32, dim);
    defer allocator.free(demand);
    demand[0] = 0;
    for (1..dim) |i| demand[i] = rng.intRangeAtMost(u32, 1, 4);
    const inst = CvrpInstance{ .n = n, .matrix = matrix, .demand = demand, .capacity = 10 };

    var res = try solveCvrpMulti(allocator, inst, .{
        .seed = 3,
        .budget = .{ .trials = 30, .max_passes = 40 },
        .candidates = .{ .candidate_count = 6, .candidate_mode = .alpha_nearness, .sparse_min_dimension = 0 },
        .search = .{ .enable_lk = true, .lk_max_depth = 5 },
    }, .{ .rounds = 30, .restarts = 1 });
    defer res.deinit();

    // independently validated cost equals the reported cost
    const checked = validate(inst, res.routes) orelse return error.InfeasibleSolution;
    try std.testing.expectEqual(res.total_cost, checked);

    // beats the trivial baseline of one customer per route
    var baseline: u64 = 0;
    for (1..dim) |c| baseline += inst.d(0, c) + inst.d(c, 0);
    try std.testing.expect(res.total_cost < baseline);
}

test "CVRP returns a clean error for an over-capacity customer" {
    const allocator = std.testing.allocator;
    // demand[2] = 6 exceeds capacity 5, so no feasible split exists. The solver
    // must surface a clean error rather than a maxInt cost over an uninitialized
    // pred chain (regression for the missing splitDp feasibility guard, C1).
    const m = [_]u32{
        0, 3, 4,
        3, 0, 5,
        4, 5, 0,
    };
    const demand = [_]u32{ 0, 1, 6 };
    const inst = CvrpInstance{ .n = 2, .matrix = &m, .demand = &demand, .capacity = 5 };
    try std.testing.expectError(error.NoFeasibleSplit, solveCvrpMulti(allocator, inst, .{ .seed = 1 }, .{ .rounds = 5, .restarts = 1 }));
}

test "CVRP single-customer instance returns the trivial route" {
    const allocator = std.testing.allocator;
    // n=1: depot + one customer, directed costs d(0,1)=7, d(1,0)=9. The giant-tour
    // path can't seed n<2 (asymmetric.solveAtsp rejects it), so this exercises the
    // solveCvrpImpl short-circuit reached via solveCvrp's n<=2 dispatch.
    const m = [_]u32{
        0, 7,
        9, 0,
    };
    const demand = [_]u32{ 0, 3 };

    // feasible (demand 3 <= capacity 5): one route [1], cost d(0,1)+d(1,0) = 16.
    const inst = CvrpInstance{ .n = 1, .matrix = &m, .demand = &demand, .capacity = 5 };
    var res = try solveCvrp(allocator, inst, .{ .seed = 1 });
    defer res.deinit();
    try std.testing.expectEqual(@as(usize, 1), res.routes.len);
    try std.testing.expectEqual(@as(usize, 1), res.routes[0].len);
    try std.testing.expectEqual(@as(usize, 1), res.routes[0][0]);
    try std.testing.expectEqual(@as(u64, 16), res.total_cost);
    const checked = validate(inst, res.routes) orelse return error.Infeasible;
    try std.testing.expectEqual(res.total_cost, checked);

    // demand exceeds capacity -> no feasible packing.
    const tight = CvrpInstance{ .n = 1, .matrix = &m, .demand = &demand, .capacity = 2 };
    try std.testing.expectError(error.NoFeasibleSplit, solveCvrp(allocator, tight, .{ .seed = 1 }));
}

fn bruteForceOptimum(allocator: std.mem.Allocator, inst: CvrpInstance) !u64 {
    const n = inst.n;
    const perm = try allocator.alloc(usize, n);
    defer allocator.free(perm);
    for (0..n) |i| perm[i] = i + 1;
    var best: u64 = std.math.maxInt(u64);
    try permRec(allocator, inst, perm, 0, &best);
    return best;
}
fn permRec(allocator: std.mem.Allocator, inst: CvrpInstance, perm: []usize, k: usize, best: *u64) !void {
    if (k == perm.len) {
        const sp = try splitDp(allocator, inst, perm);
        defer allocator.free(sp.pred);
        if (sp.cost < best.*) best.* = sp.cost;
        return;
    }
    for (k..perm.len) |i| {
        std.mem.swap(usize, &perm[k], &perm[i]);
        try permRec(allocator, inst, perm, k + 1, best);
        std.mem.swap(usize, &perm[k], &perm[i]);
    }
}

test "CVRP engine finds the brute-force optimum on a small instance" {
    const allocator = std.testing.allocator;
    const n = 7;
    const dim = n + 1;
    var prng = std.Random.DefaultPrng.init(0x1234);
    const rng = prng.random();
    const matrix = try allocator.alloc(u32, dim * dim);
    defer allocator.free(matrix);
    for (0..dim) |i| {
        for (0..dim) |j| matrix[i * dim + j] = if (i == j) 0 else rng.intRangeAtMost(u32, 1, 50);
    }
    const demand = try allocator.alloc(u32, dim);
    defer allocator.free(demand);
    demand[0] = 0;
    for (1..dim) |i| demand[i] = rng.intRangeAtMost(u32, 1, 5);
    const inst = CvrpInstance{ .n = n, .matrix = matrix, .demand = demand, .capacity = 12 };

    const opt = try bruteForceOptimum(allocator, inst);
    var res = try solveCvrpMulti(allocator, inst, .{
        .seed = 11,
        .budget = .{ .trials = 30, .max_passes = 40 },
        .candidates = .{ .candidate_count = 6, .candidate_mode = .alpha_nearness, .sparse_min_dimension = 0 },
        .search = .{ .enable_lk = true, .lk_max_depth = 5 },
    }, .{ .rounds = 200, .restarts = 1 });
    defer res.deinit();
    const checked = validate(inst, res.routes) orelse return error.Infeasible;
    try std.testing.expectEqual(res.total_cost, checked);
    // ILS + optimal Split should reach the exact optimum on n=7.
    try std.testing.expectEqual(opt, res.total_cost);
}

test "CVRP HGS finds the brute-force optimum and is deterministic" {
    const allocator = std.testing.allocator;
    const n = 9;
    const dim = n + 1;
    var prng = std.Random.DefaultPrng.init(0xB175);
    const rng = prng.random();
    const matrix = try allocator.alloc(u32, dim * dim);
    defer allocator.free(matrix);
    for (0..dim) |i| {
        for (0..dim) |j| matrix[i * dim + j] = if (i == j) 0 else rng.intRangeAtMost(u32, 1, 60);
    }
    const demand = try allocator.alloc(u32, dim);
    defer allocator.free(demand);
    demand[0] = 0;
    for (1..dim) |i| demand[i] = rng.intRangeAtMost(u32, 1, 5);
    const inst = CvrpInstance{ .n = n, .matrix = matrix, .demand = demand, .capacity = 12 };
    const opt = try bruteForceOptimum(allocator, inst);
    const opts = solver.SolveOptions{
        .seed = 3,
        .budget = .{ .trials = 20, .max_passes = 40 },
        .candidates = .{ .candidate_count = 6, .candidate_mode = .alpha_nearness, .sparse_min_dimension = 0 },
        .search = .{ .enable_lk = true, .lk_max_depth = 5 },
    };
    const params = HgsParams{ .mu = 10, .lambda = 15, .generations = 30 };
    var r1 = try solveCvrpHgs(allocator, inst, opts, params, 0);
    defer r1.deinit();
    const c1 = validate(inst, r1.routes) orelse return error.Infeasible;
    try std.testing.expectEqual(r1.total_cost, c1);
    try std.testing.expectEqual(opt, r1.total_cost);
    // identical inputs must give identical output (no uninitialized-read dependence)
    var r2 = try solveCvrpHgs(allocator, inst, opts, params, 0);
    defer r2.deinit();
    try std.testing.expectEqual(r1.total_cost, r2.total_cost);
}

test "CVRP SISR is feasible, deterministic, and finds the brute-force optimum" {
    const allocator = std.testing.allocator;
    const n = 9;
    const dim = n + 1;
    var prng = std.Random.DefaultPrng.init(0x515A);
    const rng = prng.random();
    const matrix = try allocator.alloc(u32, dim * dim);
    defer allocator.free(matrix);
    for (0..dim) |i| {
        for (0..dim) |j| matrix[i * dim + j] = if (i == j) 0 else rng.intRangeAtMost(u32, 1, 60);
    }
    const demand = try allocator.alloc(u32, dim);
    defer allocator.free(demand);
    demand[0] = 0;
    for (1..dim) |i| demand[i] = rng.intRangeAtMost(u32, 1, 5);
    const inst = CvrpInstance{ .n = n, .matrix = matrix, .demand = demand, .capacity = 12 };
    const opt = try bruteForceOptimum(allocator, inst);
    const opts = solver.SolveOptions{
        .seed = 7,
        .budget = .{ .trials = 20, .max_passes = 40 },
        .candidates = .{ .candidate_count = 6, .candidate_mode = .alpha_nearness, .sparse_min_dimension = 0 },
        .search = .{ .enable_lk = true, .lk_max_depth = 5 },
    };
    // Enough iters that the in-loop debug invariant (incremental == recomputed
    // distance) is checked many times AND the anneal reaches the optimum.
    const params = SisrParams{ .iters = 20000 };
    var r1 = try solveCvrpSisr(allocator, inst, opts, params);
    defer r1.deinit();
    const c1 = validate(inst, r1.routes) orelse return error.Infeasible;
    try std.testing.expectEqual(r1.total_cost, c1);
    try std.testing.expectEqual(opt, r1.total_cost);
    // identical inputs must give identical output (deterministic anneal)
    var r2 = try solveCvrpSisr(allocator, inst, opts, params);
    defer r2.deinit();
    try std.testing.expectEqual(r1.total_cost, r2.total_cost);

    // split-string mode on: exercises sisrRuinSplit + its undo under the in-loop
    // Debug invariant (incremental == recompute, every iter) and the optimum.
    const sparams = SisrParams{ .iters = 20000, .split_rate = 0.5, .split_alpha = 0.5 };
    var rs1 = try solveCvrpSisr(allocator, inst, opts, sparams);
    defer rs1.deinit();
    const cs = validate(inst, rs1.routes) orelse return error.Infeasible;
    try std.testing.expectEqual(rs1.total_cost, cs);
    try std.testing.expectEqual(opt, rs1.total_cost);
    var rs2 = try solveCvrpSisr(allocator, inst, opts, sparams);
    defer rs2.deinit();
    try std.testing.expectEqual(rs1.total_cost, rs2.total_cost);
}

test "CVRP fleet cap: solveCvrpFleet stays feasible and respects a generous cap" {
    const allocator = std.testing.allocator;
    const n = 12;
    const dim = n + 1;
    var prng = std.Random.DefaultPrng.init(0xF1EE7);
    const rng = prng.random();
    const matrix = try allocator.alloc(u32, dim * dim);
    defer allocator.free(matrix);
    for (0..dim) |i| {
        for (0..dim) |j| matrix[i * dim + j] = if (i == j) 0 else rng.intRangeAtMost(u32, 1, 100);
    }
    const demand = try allocator.alloc(u32, dim);
    defer allocator.free(demand);
    demand[0] = 0;
    for (1..dim) |i| demand[i] = rng.intRangeAtMost(u32, 1, 5);
    const inst = CvrpInstance{ .n = n, .matrix = matrix, .demand = demand, .capacity = 12 };
    // A cap >= the number of routes the free solver uses should not exclude the
    // free optimum, and the result must be feasible and within the cap.
    const cap_k: usize = 8;
    var res = try solveCvrpFleet(allocator, inst, .{
        .seed = 2,
        .budget = .{ .trials = 20, .max_passes = 40 },
        .candidates = .{ .candidate_count = 6, .candidate_mode = .alpha_nearness, .sparse_min_dimension = 0 },
        .search = .{ .enable_lk = true, .lk_max_depth = 5 },
    }, .{ .rounds = 30, .restarts = 2, .max_vehicles = cap_k });
    defer res.deinit();
    const checked = validate(inst, res.routes) orelse return error.Infeasible;
    try std.testing.expectEqual(res.total_cost, checked);
    try std.testing.expect(res.routes.len <= cap_k);
}

test "CVRP fleet cap: route elimination hits a tight cap on an asymmetric instance" {
    // Equal demands so a capacity-minimum packing is trivially achievable; an
    // asymmetric (directional) matrix so the FREE engine prefers extra routes,
    // forcing the fleet repair to redistribute down to the cap. cap = ceil(n/per).
    const allocator = std.testing.allocator;
    const n = 12;
    const dim = n + 1;
    var prng = std.Random.DefaultPrng.init(0xCAB5);
    const rng = prng.random();
    const matrix = try allocator.alloc(u32, dim * dim);
    defer allocator.free(matrix);
    for (0..dim) |i| {
        for (0..dim) |j| matrix[i * dim + j] = if (i == j) 0 else rng.intRangeAtMost(u32, 1, 100);
    }
    const demand = try allocator.alloc(u32, dim);
    defer allocator.free(demand);
    demand[0] = 0;
    for (1..dim) |i| demand[i] = 3; // 3 per customer, capacity 10 -> 3 per route
    const inst = CvrpInstance{ .n = n, .matrix = matrix, .demand = demand, .capacity = 10 };
    const cap_k: usize = 4; // ceil(12/3) = 4 routes is the capacity minimum
    var res = try solveCvrpFleet(allocator, inst, .{
        .seed = 7,
        .budget = .{ .trials = 20, .max_passes = 40 },
        .candidates = .{ .candidate_count = 6, .candidate_mode = .alpha_nearness, .sparse_min_dimension = 0 },
        .search = .{ .enable_lk = true, .lk_max_depth = 5 },
    }, .{ .rounds = 40, .restarts = 3, .max_vehicles = cap_k });
    defer res.deinit();
    const checked = validate(inst, res.routes) orelse return error.Infeasible;
    try std.testing.expectEqual(res.total_cost, checked);
    try std.testing.expect(res.routes.len <= cap_k);
}

test "CVRP asymmetric local search terminates and stays feasible" {
    // Regression: the adjacent-swap delta dropped the reversed middle edge, which
    // is nonzero only for DIRECTIONAL costs — it made a non-improving swap look
    // improving and the local search looped forever on asymmetric instances (all
    // prior benches were symmetric, so it went unseen). A long single route makes
    // adjacent swaps fire; this would hang with the bug.
    const allocator = std.testing.allocator;
    const n = 14;
    const dim = n + 1;
    var prng = std.Random.DefaultPrng.init(0xA5717);
    const rng = prng.random();
    const matrix = try allocator.alloc(u32, dim * dim);
    defer allocator.free(matrix);
    for (0..dim) |i| {
        for (0..dim) |j| matrix[i * dim + j] = if (i == j) 0 else rng.intRangeAtMost(u32, 1, 100);
    }
    const demand = try allocator.alloc(u32, dim);
    defer allocator.free(demand);
    demand[0] = 0;
    for (1..dim) |i| demand[i] = rng.intRangeAtMost(u32, 1, 3);
    // high capacity -> few long routes -> many adjacent-swap candidates
    const inst = CvrpInstance{ .n = n, .matrix = matrix, .demand = demand, .capacity = 40 };
    var res = try solveCvrpMulti(allocator, inst, .{
        .seed = 1,
        .budget = .{ .trials = 20, .max_passes = 40 },
        .candidates = .{ .candidate_count = 6, .candidate_mode = .alpha_nearness, .sparse_min_dimension = 0 },
        .search = .{ .enable_lk = true, .lk_max_depth = 5 },
    }, .{ .rounds = 40, .restarts = 1 });
    defer res.deinit();
    const checked = validate(inst, res.routes) orelse return error.Infeasible;
    try std.testing.expectEqual(res.total_cost, checked);
}

test "Split DP: trivial two-customer capacity split" {
    const allocator = std.testing.allocator;
    // depot=0, customers 1,2. capacity 1, each demand 1 => must be two routes.
    const n = 2;
    const matrix = [_]u32{
        0, 10, 10,
        10, 0,  3,
        10, 3,  0,
    };
    const demand = [_]u32{ 0, 1, 1 };
    const inst = CvrpInstance{ .n = n, .matrix = &matrix, .demand = &demand, .capacity = 1 };
    const giant = [_]usize{ 1, 2 };
    const out = try splitDp(allocator, inst, &giant);
    defer allocator.free(out.pred);
    // two separate routes: (0-1-0)=20 + (0-2-0)=20 = 40
    try std.testing.expectEqual(@as(u64, 40), out.cost);
}

test "Split DP: combine when capacity allows" {
    const allocator = std.testing.allocator;
    const n = 2;
    const matrix = [_]u32{
        0, 10, 10,
        10, 0,  3,
        10, 3,  0,
    };
    const demand = [_]u32{ 0, 1, 1 };
    const inst = CvrpInstance{ .n = n, .matrix = &matrix, .demand = &demand, .capacity = 2 };
    const giant = [_]usize{ 1, 2 };
    const out = try splitDp(allocator, inst, &giant);
    defer allocator.free(out.pred);
    // one route 0-1-2-0 = 10+3+10 = 23, beats two routes (40)
    try std.testing.expectEqual(@as(u64, 23), out.cost);
}
