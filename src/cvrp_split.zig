const std = @import("std");
const cvrp_types = @import("cvrp_types.zig");
const CvrpInstance = cvrp_types.CvrpInstance;


/// Optimal cost of splitting `giant` (a permutation of customers 1..n) into
/// capacity-feasible routes, via the Prins shortest-path DP. Also returns the
/// route-end positions (exclusive) in `breaks` (caller owns). O(n * maxRouteLen).
pub const SplitOutcome = struct { cost: u64, pred: []usize };

pub fn splitDp(allocator: std.mem.Allocator, inst: CvrpInstance, giant: []const usize) !SplitOutcome {
    const n = inst.n;
    const INF = std.math.maxInt(u64);
    // p[i] = min cost to serve the first i customers of `giant`; pred[i] = the
    // start index of the last route covering giant[pred[i]..i].
    const p = try allocator.alloc(u64, n + 1);
    defer allocator.free(p);
    const pred = try allocator.alloc(usize, n + 1);
    errdefer allocator.free(pred);
    @memset(p, INF);
    p[0] = 0;
    pred[0] = 0;

    for (0..n) |i| {
        if (p[i] == INF) continue;
        var load: u64 = 0;
        var route: u64 = 0;
        var j = i;
        while (j < n) : (j += 1) {
            const cust = giant[j];
            load += inst.demand[cust];
            if (load > inst.capacity) break;
            if (j == i) {
                route = inst.d(0, cust) + inst.d(cust, 0); // depot -> single -> depot
            } else {
                // extend: remove prev->depot, add prev->cust + cust->depot
                const prev = giant[j - 1];
                route = route - inst.d(prev, 0) + inst.d(prev, cust) + inst.d(cust, 0);
            }
            const cand = p[i] + route;
            if (cand < p[j + 1]) {
                p[j + 1] = cand;
                pred[j + 1] = i;
            }
        }
    }
    // No contiguous capacity-feasible split of this order exists (only possible
    // when some customer's demand exceeds capacity, i.e. an infeasible instance).
    // Return a clean error instead of a maxInt cost + uninitialized pred chain.
    if (p[n] == INF) return error.NoFeasibleSplit;
    return .{ .cost = p[n], .pred = pred };
}

// Per-excess-route penalty for the fleet cap. Dominates any realistic distance so
// solutions within the fleet are always preferred, but stays soft (never fails)
// because a given giant-tour ORDER may need more than K contiguous routes even
// when K vehicles can serve the demand after reordering — the ILS finds those.
pub const FLEET_PENALTY: u64 = 1 << 40;

// Capacity-penalty coefficient used when pen_coeff == 0 (hard feasibility gate):
// large enough that any move increasing overload is rejected and any move reducing
// it is accepted, reproducing the old gate behaviour from a feasible state.
pub const GATE_PEN: i64 = 1 << 34;

/// Customer-count threshold for the n-adaptive regimes: at/below it the HGS
/// population stays full and SISR uses plain string removal; above it the pop goes
/// lean and SISR enables split-string ("slack induction"). Both regimes change
/// character around the same scale, so they share one named threshold.
pub const POP_CROSSOVER_N: usize = 250;
// Upper bound of the regret-recreate auto-gate. Regret wins on mid-size instances
// where greedy has plateaued (X-n303/X-n502), but at large n the search is
// iteration-starved and regret's slower-but-deterministic recreate loses on BOTH
// quality and wall (X-n1001 at 1M iters best-of-3: greedy 1.49%@7s vs regret
// 1.67%@12s). Conservative: no vendored X instance lies in (502, 1001) to place
// this more tightly, so the gate stops just past the largest confirmed win.
pub const REGRET_MAX_N: usize = 600;
/// UCB1 exploration coefficient (= sqrt 2, the standard choice for rewards in [0,1]).
pub const UCB_C: f64 = 1.4142135623730951;

// Overload of one route: max(0, load - capacity), as a signed delta-friendly value.
pub inline fn capExcess(load: u32, cap: u32) i64 {
    return if (load > cap) @as(i64, @intCast(load - cap)) else 0;
}

/// Prins Split with a soft cap of `max_routes` vehicles: a 2-D DP over (customers,
/// routes) minimizing distance + FLEET_PENALTY * max(0, routes - max_routes). The
/// route dimension is bounded a little above the cap for efficiency. Returns a 1-D
/// pred chain (same shape as splitDp) for the chosen route count.
pub fn splitDpK(allocator: std.mem.Allocator, inst: CvrpInstance, giant: []const usize, max_routes: usize) !SplitOutcome {
    return splitDpKImpl(allocator, inst, giant, max_routes, false);
}

fn splitDpKImpl(allocator: std.mem.Allocator, inst: CvrpInstance, giant: []const usize, max_routes: usize, force_full: bool) !SplitOutcome {
    const n = inst.n;
    const INF = std.math.maxInt(u64);
    // The old table always carried all 0..n route counts. At n=5000 its cost and
    // predecessor grids alone occupied about 382 MiB on every capped education,
    // even when a 50-route fleet made nearly all columns irrelevant.
    //
    // For this fixed giant order, greedy maximal capacity packing gives the exact
    // minimum feasible number of contiguous routes. If one fleet penalty is larger
    // than an upper bound on *any* split distance, no route count above
    // max(max_routes, min_routes) can win the soft objective. This is a proof-based
    // bound, not a heuristic; exotic huge arc costs fall back to the full table.
    var min_routes: usize = 0;
    var greedy_load: u64 = 0;
    var max_edge: u64 = 0;
    for (giant, 0..) |cust, pos| {
        const dem = inst.demand[cust];
        if (dem > inst.capacity) return error.NoFeasibleSplit;
        if (min_routes == 0 or greedy_load + dem > inst.capacity) {
            min_routes += 1;
            greedy_load = dem;
        } else {
            greedy_load += dem;
        }
        max_edge = @max(max_edge, inst.d(0, cust));
        max_edge = @max(max_edge, inst.d(cust, 0));
        if (pos > 0) max_edge = @max(max_edge, inst.d(giant[pos - 1], cust));
    }
    const distance_bound = std.math.mul(u64, @as(u64, @intCast(n)) *| 2, max_edge) catch
        std.math.maxInt(u64);
    const bounded_kmax = @min(n, @max(max_routes, min_routes));
    const kmax = if (!force_full and distance_bound < FLEET_PENALTY) bounded_kmax else n;
    const stride = kmax + 1;
    const cells = std.math.mul(usize, n + 1, stride) catch return error.OutOfMemory;
    const p = try allocator.alloc(u64, cells);
    defer allocator.free(p);
    const pr = try allocator.alloc(usize, cells);
    defer allocator.free(pr);
    @memset(p, INF);
    p[0] = 0; // p[0][0]
    for (0..n) |i| {
        for (0..kmax) |k| {
            const pik = p[i * stride + k];
            if (pik == INF) continue;
            var load: u64 = 0;
            var route: u64 = 0;
            var j = i;
            while (j < n) : (j += 1) {
                const cust = giant[j];
                load += inst.demand[cust];
                if (load > inst.capacity) break;
                if (j == i) {
                    route = inst.d(0, cust) + inst.d(cust, 0);
                } else {
                    const prev = giant[j - 1];
                    route = route - inst.d(prev, 0) + inst.d(prev, cust) + inst.d(cust, 0);
                }
                const cand = pik + route;
                const idx = (j + 1) * stride + (k + 1);
                if (cand < p[idx]) {
                    p[idx] = cand;
                    pr[idx] = i;
                }
            }
        }
    }
    // pick k minimizing distance + fleet penalty for routes beyond max_routes
    var best_obj: u64 = INF;
    var best_k: usize = 0;
    for (1..kmax + 1) |k| {
        const dist = p[n * stride + k];
        if (dist == INF) continue;
        const excess: u64 = if (k > max_routes) k - max_routes else 0;
        const obj = dist + excess * FLEET_PENALTY;
        if (obj < best_obj) {
            best_obj = obj;
            best_k = k;
        }
    }
    if (best_k == 0) return error.NoFeasibleSplit; // a customer's demand exceeds capacity
    const pred = try allocator.alloc(usize, n + 1);
    errdefer allocator.free(pred);
    var i = n;
    var k = best_k;
    while (i > 0) {
        const st = pr[i * stride + k];
        pred[i] = st;
        i = st;
        k -= 1;
    }
    return .{ .cost = p[n * stride + best_k], .pred = pred };
}


test "fleet-capped split bound matches the full route-count DP" {
    const allocator = std.testing.allocator;
    var prng = std.Random.DefaultPrng.init(0x51_17_cafe);
    const rng = prng.random();

    for (0..100) |_| {
        const n = 2 + rng.uintLessThan(usize, 29);
        const dim = n + 1;
        const matrix = try allocator.alloc(u32, dim * dim);
        defer allocator.free(matrix);
        const demand = try allocator.alloc(u32, dim);
        defer allocator.free(demand);
        const giant = try allocator.alloc(usize, n);
        defer allocator.free(giant);

        const capacity = rng.intRangeAtMost(u32, 10, 60);
        demand[0] = 0;
        for (0..dim) |i| {
            for (0..dim) |j| matrix[i * dim + j] = if (i == j) 0 else
                rng.intRangeAtMost(u32, 1, 10_000);
        }
        for (0..n) |i| {
            demand[i + 1] = rng.intRangeAtMost(u32, 1, capacity);
            giant[i] = i + 1;
        }
        rng.shuffle(usize, giant);
        const max_routes = 1 + rng.uintLessThan(usize, n);
        const inst = CvrpInstance{
            .n = n,
            .matrix = matrix,
            .demand = demand,
            .capacity = capacity,
        };

        const bounded = try splitDpKImpl(allocator, inst, giant, max_routes, false);
        defer allocator.free(bounded.pred);
        const full = try splitDpKImpl(allocator, inst, giant, max_routes, true);
        defer allocator.free(full.pred);
        try std.testing.expectEqual(full.cost, bounded.cost);

        // Only the selected predecessor chain is initialized; compare that chain,
        // not the unused entries of the returned n+1 buffers.
        var pos = n;
        while (pos > 0) {
            try std.testing.expectEqual(full.pred[pos], bounded.pred[pos]);
            try std.testing.expect(bounded.pred[pos] < pos);
            pos = bounded.pred[pos];
        }
    }
}
