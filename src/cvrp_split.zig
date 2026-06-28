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
    const n = inst.n;
    const INF = std.math.maxInt(u64);
    // Route dimension up to n: a capacity-unfriendly giant-tour ORDER can need many
    // more than max_routes CONTIGUOUS routes even when the fleet can serve the
    // demand after reordering, so the DP must be able to represent that (one route
    // per customer always fits). The soft penalty then drives the ILS to <= cap.
    const kmax = n;
    const stride = kmax + 1;
    const p = try allocator.alloc(u64, (n + 1) * stride);
    defer allocator.free(p);
    const pr = try allocator.alloc(usize, (n + 1) * stride);
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
