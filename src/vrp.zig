const std = @import("std");
const builtin = @import("builtin");
const problem = @import("problem.zig");
const solver = @import("solver.zig");
const asymmetric = @import("asymmetric.zig");

// Capacitated VRP (CVRP) core, asymmetric-capable, on the Prins "giant tour +
// Split" method that underpins HGS: order all customers in one giant tour (the
// TSP core's job), then optimally cut it into capacity-feasible vehicle routes
// (each depot->...->depot) with a shortest-path DP. Local search reorders the
// giant tour and re-splits; the Split DP guarantees the best partition of any
// given order, so the search only has to find a good order.
//
// Node 0 is the depot; customers are 1..n. `matrix` is (n+1)x(n+1) DIRECTIONAL
// (matrix[a*(n+1)+b] = cost a->b), so real asymmetric road costs work directly.

pub const CvrpInstance = struct {
    n: usize, // customer count (excludes depot)
    matrix: []const u32, // (n+1)*(n+1), directional, depot=0
    demand: []const u32, // length n+1, demand[0]=0
    capacity: u32,

    fn dim(self: CvrpInstance) usize {
        return self.n + 1;
    }
    fn d(self: CvrpInstance, a: usize, b: usize) u64 {
        return self.matrix[a * self.dim() + b];
    }
};

pub const CvrpResult = struct {
    allocator: std.mem.Allocator,
    routes: [][]usize, // each route: customer indices in visit order (depot implied at both ends)
    total_cost: u64,

    pub fn deinit(self: *CvrpResult) void {
        for (self.routes) |r| self.allocator.free(r);
        self.allocator.free(self.routes);
        self.* = undefined;
    }
};

/// Optimal cost of splitting `giant` (a permutation of customers 1..n) into
/// capacity-feasible routes, via the Prins shortest-path DP. Also returns the
/// route-end positions (exclusive) in `breaks` (caller owns). O(n * maxRouteLen).
const SplitOutcome = struct { cost: u64, pred: []usize };

fn splitDp(allocator: std.mem.Allocator, inst: CvrpInstance, giant: []const usize) !SplitOutcome {
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
    return .{ .cost = p[n], .pred = pred };
}

// Per-excess-route penalty for the fleet cap. Dominates any realistic distance so
// solutions within the fleet are always preferred, but stays soft (never fails)
// because a given giant-tour ORDER may need more than K contiguous routes even
// when K vehicles can serve the demand after reordering — the ILS finds those.
const FLEET_PENALTY: u64 = 1 << 40;

// Capacity-penalty coefficient used when pen_coeff == 0 (hard feasibility gate):
// large enough that any move increasing overload is rejected and any move reducing
// it is accepted, reproducing the old gate behaviour from a feasible state.
const GATE_PEN: i64 = 1 << 34;

/// Customer-count threshold for the n-adaptive regimes: at/below it the HGS
/// population stays full and SISR uses plain string removal; above it the pop goes
/// lean and SISR enables split-string ("slack induction"). Both regimes change
/// character around the same scale, so they share one named threshold.
const POP_CROSSOVER_N: usize = 250;
// Upper bound of the regret-recreate auto-gate. Regret wins on mid-size instances
// where greedy has plateaued (X-n303/X-n502), but at large n the search is
// iteration-starved and regret's slower-but-deterministic recreate loses on BOTH
// quality and wall (X-n1001 at 1M iters best-of-3: greedy 1.49%@7s vs regret
// 1.67%@12s). Conservative: no vendored X instance lies in (502, 1001) to place
// this more tightly, so the gate stops just past the largest confirmed win.
const REGRET_MAX_N: usize = 600;
/// UCB1 exploration coefficient (= sqrt 2, the standard choice for rewards in [0,1]).
const UCB_C: f64 = 1.4142135623730951;

// Overload of one route: max(0, load - capacity), as a signed delta-friendly value.
inline fn capExcess(load: u32, cap: u32) i64 {
    return if (load > cap) @as(i64, @intCast(load - cap)) else 0;
}

/// Prins Split with a soft cap of `max_routes` vehicles: a 2-D DP over (customers,
/// routes) minimizing distance + FLEET_PENALTY * max(0, routes - max_routes). The
/// route dimension is bounded a little above the cap for efficiency. Returns a 1-D
/// pred chain (same shape as splitDp) for the chosen route count.
fn splitDpK(allocator: std.mem.Allocator, inst: CvrpInstance, giant: []const usize, max_routes: usize) !SplitOutcome {
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

/// Solve CVRP: construct a giant tour over the customers (asymmetric TSP core),
/// then Prins-Split it, then improve the order with relocate/swap moves that are
/// re-split each time, keeping the best. Capacity is enforced exactly by Split.
pub fn solveCvrp(allocator: std.mem.Allocator, inst: CvrpInstance, options: solver.SolveOptions, rounds: usize) !CvrpResult {
    return solveCvrpMulti(allocator, inst, options, rounds, 1);
}

/// solveCvrpMulti, but with a hard fleet cap: at most `max_vehicles` routes (0 =
/// unlimited). For fixed-fleet ACVRP / real distribution where the vehicle count
/// is a constraint, not free.
pub fn solveCvrpFleet(allocator: std.mem.Allocator, inst: CvrpInstance, options: solver.SolveOptions, rounds: usize, restarts: usize, max_vehicles: usize) !CvrpResult {
    return solveCvrpImpl(allocator, inst, options, rounds, restarts, max_vehicles);
}

/// As solveCvrp but with `restarts` independent ILS chains sharing one giant
/// tour, each with a distinct perturbation seed, keeping the global best. The
/// ILS is high-variance per chain (a single unlucky seed can sit 3% above the
/// optimum), so best-of-K reliably tightens the gap for a roughly K-times budget.
pub fn solveCvrpMulti(allocator: std.mem.Allocator, inst: CvrpInstance, options: solver.SolveOptions, rounds: usize, restarts: usize) !CvrpResult {
    return solveCvrpImpl(allocator, inst, options, rounds, restarts, 0);
}

fn solveCvrpImpl(allocator: std.mem.Allocator, inst: CvrpInstance, options: solver.SolveOptions, rounds: usize, restarts: usize, max_vehicles: usize) !CvrpResult {
    const n = inst.n;
    std.debug.assert(inst.demand.len == n + 1 and inst.matrix.len == (n + 1) * (n + 1));

    // Giant tour: solve an ATSP over just the customers (sub-matrix 1..n).
    const sub = try allocator.alloc(u32, n * n);
    defer allocator.free(sub);
    for (0..n) |a| {
        for (0..n) |b| sub[a * n + b] = inst.matrix[(a + 1) * (n + 1) + (b + 1)];
    }
    var atsp = try asymmetric.solveAtsp(allocator, sub, n, options);
    defer atsp.deinit();

    const giant = try allocator.alloc(usize, n);
    defer allocator.free(giant);
    for (atsp.tour, 0..) |c, idx| giant[idx] = c + 1; // sub indices 0..n-1 -> customers 1..n

    const gk: usize = @min(@as(usize, 20), if (n > 1) n - 1 else 0);
    const gran = try buildCvrpNeighbors(allocator, inst, @max(gk, 1));
    defer allocator.free(gran);

    const split0 = if (max_vehicles > 0) try splitDpK(allocator, inst, giant, max_vehicles) else try splitDp(allocator, inst, giant);
    defer allocator.free(split0.pred);
    var sol = try Solution.fromPred(allocator, inst, giant, split0.pred);
    sol.max_vehicles = max_vehicles;
    sol.gran = gran;
    sol.gk = gk;
    sol.recompute(); // re-fold cost with the fleet penalty now that the cap is set
    defer sol.deinit();

    // Route-based local search (relocate + or-opt + 2-opt + swap) to a local
    // optimum = the shared, deterministic restart point for every ILS chain.
    try sol.localSearch();
    var base = try sol.clone(); // independent restart point (initial local opt)
    defer base.deinit();
    var best = try sol.clone(); // global best across chains
    defer best.deinit();
    var inc = try sol.clone(); // current chain's incumbent
    defer inc.deinit();

    var chain: usize = 0;
    while (chain < restarts) : (chain += 1) {
        sol.copyFrom(base);
        inc.copyFrom(base);
        var prng = std.Random.DefaultPrng.init(options.seed +% chain *% 0x9E3779B97F4A7C15);
        const rng = prng.random();
        var round: usize = 0;
        while (round < rounds) : (round += 1) {
            sol.perturb(rng);
            try sol.localSearch();
            if (sol.cost < inc.cost) {
                inc.copyFrom(sol);
            } else {
                sol.copyFrom(inc); // restart from the chain incumbent
            }
        }
        if (inc.cost < best.cost) best.copyFrom(inc);
    }

    return best.toResult(allocator);
}

/// k-nearest neighbour lists by directional proximity d(c,j)+d(j,c) (customers
/// 1..n, 1-indexed; gran[(c-1)*k + i] = the i-th nearest, 0-padded). Restricts
/// SWAP* to spatially close customers — the granular neighbourhood that makes the
/// search fast and scalable. Caller owns the returned slice.
fn buildCvrpNeighbors(allocator: std.mem.Allocator, inst: CvrpInstance, k: usize) ![]usize {
    const n = inst.n;
    const gran = try allocator.alloc(usize, n * k);
    @memset(gran, 0);
    const kk = @min(k, if (n > 1) n - 1 else 0);
    const idx = try allocator.alloc(usize, n);
    defer allocator.free(idx);
    const keyc = try allocator.alloc(u64, n + 1);
    defer allocator.free(keyc);
    for (1..n + 1) |c| {
        var m: usize = 0;
        for (1..n + 1) |j| {
            if (j == c) continue;
            keyc[j] = inst.d(c, j) + inst.d(j, c);
            idx[m] = j;
            m += 1;
        }
        std.sort.pdq(usize, idx[0..m], keyc, struct {
            fn lt(key: []const u64, a: usize, b: usize) bool {
                // index tiebreak: canonical neighbour order independent of the unstable
                // pdq pivot, so equidistant neighbours never depend on sort internals.
                return key[a] < key[b] or (key[a] == key[b] and a < b);
            }
        }.lt);
        for (0..kk) |i| gran[(c - 1) * k + i] = idx[i];
    }
    return gran;
}

// =========================== HGS population layer ============================
// Hybrid Genetic Search (Vidal) for CVRP/ACVRP: a population of giant tours
// evolved by order crossover (OX) + education (Split + local search), with
// biased-fitness parent selection and diversity-aware survivor selection. This
// is the GLOBAL search the single-incumbent ILS lacks — crossover recombines
// whole route assignments across individuals, reaching solutions a local search
// cannot (e.g. repacking a near-full fleet that single-route elimination can't).

pub const HgsParams = struct {
    mu: usize = 0, // population floor (survivors per generation); 0 = auto by n
    lambda: usize = 0, // offspring generated per generation; 0 = auto by n
    generations: usize = 100,
    n_elite: usize = 10, // elites whose cost rank dominates biased fitness
    n_close: usize = 5, // neighbours averaged for the diversity contribution
    restart_gens: usize = 20, // gentle diversification after this many stagnant gens (0 = never)
    infeasible_search: bool = true, // explore capacity-infeasible space under an adaptive penalty
};

const Individual = struct {
    giant: []usize, // educated giant tour (the chromosome)
    edges: []u64, // sorted giant-cycle edges, for the broken-pairs diversity metric
    cost: u64, // penalized cost (distance + fleet penalty), the fitness
    fn deinit(self: *Individual, allocator: std.mem.Allocator) void {
        allocator.free(self.giant);
        allocator.free(self.edges);
    }
};

/// Undirected giant-cycle edge set (sorted) used as a cheap structural fingerprint
/// for the broken-pairs diversity distance between two individuals.
fn cvrpBuildEdges(allocator: std.mem.Allocator, giant: []const usize, n: usize) ![]u64 {
    const e = try allocator.alloc(u64, n);
    const base: u64 = @intCast(n + 1);
    for (0..n) |k| {
        const a = giant[k];
        const b = giant[(k + 1) % n];
        const lo: u64 = @intCast(@min(a, b));
        const hi: u64 = @intCast(@max(a, b));
        e[k] = lo * base + hi;
    }
    std.mem.sort(u64, e, {}, std.sort.asc(u64));
    return e;
}

fn cvrpEdgeOverlap(a: []const u64, b: []const u64) usize {
    var i: usize = 0;
    var j: usize = 0;
    var c: usize = 0;
    while (i < a.len and j < b.len) {
        if (a[i] == b[j]) {
            c += 1;
            i += 1;
            j += 1;
        } else if (a[i] < b[j]) {
            i += 1;
        } else {
            j += 1;
        }
    }
    return c;
}

/// Order crossover: copy a random slice of p1, fill the rest with p2's customers
/// in cyclic order (skipping those already taken). Yields a valid permutation.
fn cvrpOxCrossover(allocator: std.mem.Allocator, p1: []const usize, p2: []const usize, n: usize, rng: std.Random) ![]usize {
    const child = try allocator.alloc(usize, n);
    errdefer allocator.free(child);
    const used = try allocator.alloc(bool, n + 1);
    defer allocator.free(used);
    @memset(used, false);
    var i = rng.uintLessThan(usize, n);
    var j = rng.uintLessThan(usize, n);
    if (i > j) {
        const t = i;
        i = j;
        j = t;
    }
    var k = i;
    while (k <= j) : (k += 1) {
        child[k] = p1[k];
        used[p1[k]] = true;
    }
    var pos = (j + 1) % n;
    var idx = (j + 1) % n;
    var remaining = n - (j - i + 1);
    while (remaining > 0) {
        const city = p2[idx];
        if (!used[city]) {
            child[pos] = city;
            used[city] = true;
            pos = (pos + 1) % n;
            remaining -= 1;
        }
        idx = (idx + 1) % n;
    }
    return child;
}

/// Education = Split the giant under the fleet cap, then drive to a local optimum.
/// Returns the educated Solution (caller flattens .order back to a giant tour).
fn educateGiant(allocator: std.mem.Allocator, inst: CvrpInstance, giant: []const usize, max_vehicles: usize, gran: []const usize, gk: usize, pen_coeff: u64) !Solution {
    const sp = if (max_vehicles > 0)
        try splitDpK(allocator, inst, giant, max_vehicles)
    else
        try splitDp(allocator, inst, giant);
    defer allocator.free(sp.pred);
    var sol = try Solution.fromPred(allocator, inst, giant, sp.pred);
    errdefer sol.deinit();
    sol.max_vehicles = max_vehicles;
    sol.gran = gran;
    sol.gk = gk;
    sol.pen_coeff = pen_coeff; // >0 => local search may explore capacity-infeasible space
    sol.recompute();
    try sol.localSearch();
    // Repair every infeasible education back to feasibility under a punitive penalty.
    // This is not just to update the incumbent — it keeps the feasible subpopulation
    // (the source of good crossovers) healthy; pruning it on the incumbent collapses
    // quality. Then restore the working penalty so costs stay comparable.
    if (sol.cap_excess > 0 and pen_coeff > 0) {
        sol.pen_coeff = 1 << 34;
        try sol.localSearch();
        sol.pen_coeff = pen_coeff;
        sol.recompute();
    }
    return sol;
}

/// Biased fitness = costRank + (1 - n_elite/N) * diversityRank (Vidal). Lower is
/// better; clones (identical edge set to an at-least-as-good member) get a big
/// penalty so they are culled first.
fn cvrpComputeBiased(allocator: std.mem.Allocator, pop: []const Individual, out: []f64, params: HgsParams, n: usize) !void {
    const N = pop.len;
    const cost_rank = try allocator.alloc(usize, N);
    defer allocator.free(cost_rank);
    const div_rank = try allocator.alloc(usize, N);
    defer allocator.free(div_rank);
    const idxbuf = try allocator.alloc(usize, N);
    defer allocator.free(idxbuf);
    const div = try allocator.alloc(f64, N);
    defer allocator.free(div);
    const dists = try allocator.alloc(usize, N);
    defer allocator.free(dists);
    const clone = try allocator.alloc(bool, N);
    defer allocator.free(clone);

    for (0..N) |k| idxbuf[k] = k;
    std.sort.pdq(usize, idxbuf, pop, struct {
        fn lt(p: []const Individual, a: usize, b: usize) bool {
            return p[a].cost < p[b].cost or (p[a].cost == p[b].cost and a < b);
        }
    }.lt);
    for (0..N) |r| cost_rank[idxbuf[r]] = r;

    const nc = @min(params.n_close, if (N > 0) N - 1 else 0);
    for (0..N) |i| {
        var m: usize = 0;
        var is_clone = false;
        for (0..N) |jj| {
            if (jj == i) continue;
            const dist = n - cvrpEdgeOverlap(pop[i].edges, pop[jj].edges);
            dists[m] = dist;
            m += 1;
            if (dist == 0 and (pop[jj].cost < pop[i].cost or (pop[jj].cost == pop[i].cost and jj < i))) is_clone = true;
        }
        clone[i] = is_clone;
        std.sort.pdq(usize, dists[0..m], {}, std.sort.asc(usize));
        var sum: f64 = 0;
        const take = @min(nc, m);
        for (0..take) |t| sum += @floatFromInt(dists[t]);
        div[i] = if (take > 0) sum / @as(f64, @floatFromInt(take)) else 0;
    }
    for (0..N) |k| idxbuf[k] = k;
    std.sort.pdq(usize, idxbuf, div, struct {
        fn lt(d: []const f64, a: usize, b: usize) bool {
            return d[a] > d[b] or (d[a] == d[b] and a < b);
        }
    }.lt);
    for (0..N) |r| div_rank[idxbuf[r]] = r;

    const elite_frac = 1.0 - @as(f64, @floatFromInt(params.n_elite)) / @as(f64, @floatFromInt(N));
    const big: f64 = @floatFromInt(2 * N);
    for (0..N) |i| {
        out[i] = @as(f64, @floatFromInt(cost_rank[i])) + elite_frac * @as(f64, @floatFromInt(div_rank[i]));
        if (clone[i]) out[i] += big;
    }
}

// Binary tournament over the first `count` individuals (the parents whose biased
// fitness was computed this generation — NOT the offspring being appended, whose
// biased[] slots are still uninitialized).
fn cvrpTournament(biased: []const f64, count: usize, rng: std.Random) usize {
    const a = rng.uintLessThan(usize, count);
    const b = rng.uintLessThan(usize, count);
    return if (biased[a] <= biased[b]) a else b;
}

/// Educate one giant tour, append it to the population, and update the incumbent.
fn cvrpInsert(allocator: std.mem.Allocator, inst: CvrpInstance, max_vehicles: usize, giant: []const usize, ebuf: []usize, n: usize, pop: *std.ArrayList(Individual), best: *CvrpResult, best_cost: *u64, have_best: *bool, gran: []const usize, gk: usize, pen_coeff: u64) !bool {
    var sol = try educateGiant(allocator, inst, giant, max_vehicles, gran, gk, pen_coeff);
    defer sol.deinit();
    @memcpy(ebuf, sol.order); // .order is the giant tour grouped by routes
    try pop.append(allocator, .{
        .giant = try allocator.dupe(usize, ebuf),
        .edges = try cvrpBuildEdges(allocator, ebuf, n),
        .cost = sol.cost,
    });
    const feasible = sol.cap_excess == 0;
    // Only a capacity-feasible solution may become the reported incumbent. When
    // feasible, cost = distance + fleet-count penalty (no capacity term), so the
    // comparison still prefers fewer vehicles under a fleet cap.
    if (feasible and (!have_best.* or sol.cost < best_cost.*)) {
        if (have_best.*) best.deinit();
        best.* = try sol.toResult(allocator);
        best_cost.* = sol.cost;
        have_best.* = true;
    }
    return feasible;
}

/// HGS solve for CVRP/ACVRP. `max_vehicles` (0 = unlimited) is the fleet cap; the
/// population's crossover reaches tight-fleet repackings the local search can't.
pub fn solveCvrpHgs(allocator: std.mem.Allocator, inst: CvrpInstance, options: solver.SolveOptions, params: HgsParams, max_vehicles: usize) !CvrpResult {
    const n = inst.n;
    std.debug.assert(inst.demand.len == n + 1 and inst.matrix.len == (n + 1) * (n + 1));
    if (n <= 2) return solveCvrpImpl(allocator, inst, options, 10, 1, max_vehicles);

    // Population size is n-adaptive (0 = auto): large instances cannot afford enough
    // generations to evolve a big population within budget, so they get a lean pop
    // (more generations per unit work — measured 3.45%->2.92% on X-n1001). Small,
    // multi-modal instances need the diversity of a full pop (a lean pop collapsed
    // X-n153 to 2.7%). Crossover ~n=250. Explicit non-zero params override.
    const mu = if (params.mu != 0) params.mu else (if (n <= POP_CROSSOVER_N) @as(usize, 25) else 10);
    const lambda = if (params.lambda != 0) params.lambda else (if (n <= POP_CROSSOVER_N) @as(usize, 40) else 20);

    // seed giant tour from the ATSP core
    const sub = try allocator.alloc(u32, n * n);
    defer allocator.free(sub);
    for (0..n) |a| {
        for (0..n) |b| sub[a * n + b] = inst.matrix[(a + 1) * (n + 1) + (b + 1)];
    }
    var atsp = try asymmetric.solveAtsp(allocator, sub, n, options);
    defer atsp.deinit();
    const seed_giant = try allocator.alloc(usize, n);
    defer allocator.free(seed_giant);
    for (atsp.tour, 0..) |c, idx| seed_giant[idx] = c + 1;

    const gk: usize = @min(@as(usize, 20), n - 1);
    const gran = try buildCvrpNeighbors(allocator, inst, gk);
    defer allocator.free(gran);

    // Adaptive capacity penalty for infeasible search (HGS-CVRP's key feature): the
    // local search may overload routes at a per-unit price, crossing infeasible
    // space the feasibility-gated search cannot. Initialise ~ mean arc cost over the
    // granular neighbourhoods (a representative move scale), then steer it each
    // generation toward a target feasible fraction.
    // Gated to the uncapped fleet (max_vehicles == 0): under a hard vehicle cap the
    // capacity-overload search collides with the route-count repair, and that joint
    // regime needs its own tuning — keep it on the proven strict-feasible path.
    // Infeasible search only pays off when the instance CONVERGES enough to exploit
    // the extra search space. It broke the ceiling for n<=~500 but is pure 2x-cost
    // overhead at n=1000 (which never converges in budget): X-n1001 lean+penalty
    // 4.08% vs lean+no-penalty 2.78%. So gate it off past ~600 customers.
    const use_infeasible = max_vehicles == 0 and params.infeasible_search and n <= 600;
    var pen_coeff: u64 = if (!use_infeasible) 0 else blk: {
        var sum: u64 = 0;
        var cnt: u64 = 0;
        for (1..n + 1) |a| {
            for (gran[(a - 1) * gk ..][0..gk]) |b| {
                if (b == 0) continue;
                sum += inst.d(a, b);
                cnt += 1;
            }
        }
        break :blk @max(@as(u64, 1), if (cnt > 0) sum / cnt else 1);
    };

    var prng = std.Random.DefaultPrng.init(options.seed);
    const rng = prng.random();
    const ebuf = try allocator.alloc(usize, n);
    defer allocator.free(ebuf);

    var pop: std.ArrayList(Individual) = .empty;
    defer {
        for (pop.items) |*ind| ind.deinit(allocator);
        pop.deinit(allocator);
    }

    var best: CvrpResult = undefined;
    var have_best = false;
    var best_cost: u64 = std.math.maxInt(u64);
    errdefer if (have_best) best.deinit();

    // seed individual 0 from the ATSP giant; the rest from random permutations
    _ = try cvrpInsert(allocator, inst, max_vehicles, seed_giant, ebuf, n, &pop, &best, &best_cost, &have_best, gran, gk, pen_coeff);
    var s: usize = 1;
    while (s < mu) : (s += 1) {
        for (0..n) |k| ebuf[k] = k + 1;
        var t = n;
        while (t > 1) {
            t -= 1;
            const r = rng.uintLessThan(usize, t + 1);
            std.mem.swap(usize, &ebuf[t], &ebuf[r]);
        }
        _ = try cvrpInsert(allocator, inst, max_vehicles, ebuf, ebuf, n, &pop, &best, &best_cost, &have_best, gran, gk, pen_coeff);
    }

    const cap = mu + lambda;
    const biased = try allocator.alloc(f64, cap);
    defer allocator.free(biased);
    const order = try allocator.alloc(usize, cap);
    defer allocator.free(order);
    const keep = try allocator.alloc(bool, cap);
    defer allocator.free(keep);

    var gen: usize = 0;
    var stagnation: usize = 0;
    while (gen < params.generations) : (gen += 1) {
        const before = best_cost;
        try cvrpComputeBiased(allocator, pop.items, biased[0..pop.items.len], params, n);
        const nparents = pop.items.len; // freeze: offspring are not eligible as parents
        var off: usize = 0;
        var feas: usize = 0;
        while (off < lambda) : (off += 1) {
            const p1 = cvrpTournament(biased[0..nparents], nparents, rng);
            const p2 = cvrpTournament(biased[0..nparents], nparents, rng);
            const cg = try cvrpOxCrossover(allocator, pop.items[p1].giant, pop.items[p2].giant, n, rng);
            defer allocator.free(cg);
            if (try cvrpInsert(allocator, inst, max_vehicles, cg, ebuf, n, &pop, &best, &best_cost, &have_best, gran, gk, pen_coeff)) feas += 1;
        }
        // Adapt the capacity penalty toward a ~25% feasible-offspring target: too few
        // feasible => raise the price of overload; too many => lower it to explore
        // deeper into infeasible space. Bounded to avoid runaway.
        if (use_infeasible) {
            const frac = @as(f64, @floatFromInt(feas)) / @as(f64, @floatFromInt(lambda));
            if (frac < 0.2) {
                pen_coeff = @min(pen_coeff + pen_coeff / 5 + 1, 1 << 33);
            } else if (frac > 0.3) {
                pen_coeff = @max(@as(u64, 1), pen_coeff - pen_coeff / 8);
            }
        }
        // survivor selection: keep mu best by biased fitness; always keep the
        // lowest-cost individual (elitism keeps the incumbent's genes in the pool).
        try cvrpComputeBiased(allocator, pop.items, biased[0..pop.items.len], params, n);
        var best_idx: usize = 0;
        for (1..pop.items.len) |k| {
            if (pop.items[k].cost < pop.items[best_idx].cost) best_idx = k;
        }
        for (0..pop.items.len) |k| order[k] = k;
        std.sort.pdq(usize, order[0..pop.items.len], biased, struct {
            fn lt(bf: []const f64, a: usize, b: usize) bool {
                return bf[a] < bf[b] or (bf[a] == bf[b] and a < b);
            }
        }.lt);
        @memset(keep[0..pop.items.len], false);
        var kept: usize = 0;
        keep[best_idx] = true;
        kept += 1;
        for (order[0..pop.items.len]) |idx| {
            if (kept >= mu) break;
            if (!keep[idx]) {
                keep[idx] = true;
                kept += 1;
            }
        }
        var w: usize = 0;
        for (0..pop.items.len) |idx| {
            if (keep[idx]) {
                pop.items[w] = pop.items[idx];
                w += 1;
            } else {
                var m = pop.items[idx];
                m.deinit(allocator);
            }
        }
        pop.items.len = w;

        // gentle diversification: on stagnation, keep the better half (by cost)
        // and refill with fresh random individuals — diversity without discarding
        // the good genes a full restart would. (An aggressive keep-1 restart was
        // tried and regressed: the short cycles between frequent restarts don't
        // re-converge deeply enough. The stochastic plateaus are better escaped by
        // external multi-start than by disrupting a single run mid-convergence.)
        if (best_cost < before) stagnation = 0 else stagnation += 1;
        if (params.restart_gens > 0 and stagnation >= params.restart_gens) {
            for (0..pop.items.len) |k| order[k] = k;
            std.sort.pdq(usize, order[0..pop.items.len], pop.items, struct {
                fn lt(p: []const Individual, a: usize, b: usize) bool {
                    return p[a].cost < p[b].cost or (p[a].cost == p[b].cost and a < b);
                }
            }.lt);
            const surv = @max(@as(usize, 1), mu / 2);
            @memset(keep[0..pop.items.len], false);
            for (order[0..surv]) |idx| keep[idx] = true;
            var w2: usize = 0;
            for (0..pop.items.len) |idx| {
                if (keep[idx]) {
                    pop.items[w2] = pop.items[idx];
                    w2 += 1;
                } else {
                    var m = pop.items[idx];
                    m.deinit(allocator);
                }
            }
            pop.items.len = w2;
            while (pop.items.len < mu) {
                for (0..n) |k| ebuf[k] = k + 1;
                var t = n;
                while (t > 1) {
                    t -= 1;
                    const r = rng.uintLessThan(usize, t + 1);
                    std.mem.swap(usize, &ebuf[t], &ebuf[r]);
                }
                _ = try cvrpInsert(allocator, inst, max_vehicles, ebuf, ebuf, n, &pop, &best, &best_cost, &have_best, gran, gk, pen_coeff);
            }
            stagnation = 0;
        }
    }
    return best;
}

// ---- SISR: single-solution ruin-and-recreate (Christiaens & Vanden Berghe 2020) -
// HGS plateaus on large CVRP (X-n1001 ~2.78%) because a population cannot afford
// enough generations at scale. SISR is the scale answer: one solution, millions of
// cheap O(removed) ruin+recreate moves under simulated-annealing acceptance. Ruin
// removes spatially-adjacent strings of customers; recreate greedily re-inserts
// them (cheapest position, granular candidates, small "blink" skip probability for
// diversity). Runs entirely on the linked route rep; capacity-feasible throughout
// (recreate only inserts where it fits, or opens a route), so the reported distance
// is the true objective.

pub const SisrParams = struct {
    iters: usize = 300_000, // ruin+recreate iterations
    l_max: usize = 10, // maximum string cardinality (L^max)
    cbar: f64 = 10.0, // average number of customers removed per ruin
    blink: f64 = 0.01, // recreate: probability of skipping a candidate position
    t0_factor: f64 = 1.0, // initial SA temperature = t0_factor * (dist0 / n)
    tf_factor: f64 = 0.01, // final SA temperature   = tf_factor * (dist0 / n)
    split_rate: f64 = -1.0, // prob a ruin uses split-string; <0 = auto (on for large n), 0 = off
    split_alpha: f64 = 0.5, // split mode: geometric growth of the preserved-block size
    bandit: bool = false, // UCB1 online choice of split vs plain (overrides split_rate)
    // Recreate strategy: probability a given recreate uses regret-2 (insert the
    // customer with the largest best/2nd-best insertion-cost gap) instead of
    // greedy+blink. <0 = auto = on only for 250 <= n <= 600 (see REGRET_MAX_N).
    // Regret reconstructs MID-size instances (X-n303/X-n502) better than greedy can
    // at any budget (greedy plateaus there), but its determinism + per-recreate cost
    // hurt small/tight instances (n<=200) AND large iteration-starved ones (X-n1001
    // loses on both quality and wall). 0 = always greedy. 1 = always regret.
    regret_rate: f64 = -1.0,
};

// Per-iteration ruin/recreate scratch (caller-owned slices). `present` is the
// invariant: all-true between iterations (ruin flips customers out, recreate
// restores them), so it never needs reset. The search mutates a single solution
// in place and rolls back rejected moves via the undo records below, so each
// iteration is O(removed), not O(n) — no full-state snapshot copy.
const SisrCtx = struct {
    present: []bool, // present[c]: customer c currently in a route
    removed: []usize, // customers removed this iteration, in removal order
    rprev: []usize, // rprev[j]: original predecessor of removed[j] (0 = route start)
    rroute: []usize, // rroute[j]: original route slot of removed[j]
    ins: []usize, // removed customers in recreate insertion order (for undo)
    nrem: usize = 0,
    touched: []usize, // route slots ruined this iteration (to clear rmark)
    ntouched: usize = 0,
    rmark: []bool, // route slot already ruined this iteration
    blink: f64,
    l_max: usize,
    cbar: f64,
    split_rate: f64,
    split_alpha: f64,
    force_split: i8 = -1, // bandit override per iteration: -1 = use split_rate, 0/1 = off/on
    regret_rate: f64 = 0, // resolved recreate strategy: P(use regret-2 this recreate)
};

/// SISR solver for (symmetric or asymmetric) CVRP, uncapped fleet. Builds a feasible
/// start from the ATSP-seed giant tour, then runs `params.iters` ruin+recreate steps
/// under a geometric SA schedule, returning the best solution found.
pub fn solveCvrpSisr(allocator: std.mem.Allocator, inst: CvrpInstance, options: solver.SolveOptions, params: SisrParams) !CvrpResult {
    const n = inst.n;
    std.debug.assert(inst.demand.len == n + 1 and inst.matrix.len == (n + 1) * (n + 1));
    if (n <= 2) return solveCvrpImpl(allocator, inst, options, 10, 1, 0);

    const gk: usize = @min(@as(usize, 20), n - 1);
    const gran = try buildCvrpNeighbors(allocator, inst, gk);
    defer allocator.free(gran);

    // initial solution: ATSP giant -> Split -> local optimum (strict-feasible).
    // Seed read straight off inst.matrix's customer block (stride n+1, off 1) — no n*n
    // sub copy, no 2n transform. SISR ruins/recreates the seed away, so a fast native
    // directed tour is all it needs; pointing at the existing matrix holds RAM at the
    // base footprint (was ~100MB/thread of sub at n=5000).
    var atsp = try asymmetric.solveAtspNativeView(allocator, inst.matrix, n, n + 1, 1, options);
    defer atsp.deinit();
    const seed_giant = try allocator.alloc(usize, n);
    defer allocator.free(seed_giant);
    for (atsp.tour, 0..) |c, idx| seed_giant[idx] = c + 1;

    var cur = try educateGiant(allocator, inst, seed_giant, 0, gran, gk, 0);
    defer cur.deinit();
    cur.buildLinks(); // guarantee links match order

    // `best` is the only full-state copy; refreshed only when cur improves (rare),
    // so the hot loop carries no O(n) per-iteration snapshot.
    var best = try cur.clone();
    defer best.deinit();
    best.copyLiveFrom(&cur);
    var best_dist = cur.distance;

    const present = try allocator.alloc(bool, n + 1);
    defer allocator.free(present);
    @memset(present, true);
    const removed = try allocator.alloc(usize, n);
    defer allocator.free(removed);
    const rprev = try allocator.alloc(usize, n);
    defer allocator.free(rprev);
    const rroute = try allocator.alloc(usize, n);
    defer allocator.free(rroute);
    const ins = try allocator.alloc(usize, n);
    defer allocator.free(ins);
    const touched = try allocator.alloc(usize, n);
    defer allocator.free(touched);
    const rmark = try allocator.alloc(bool, n);
    defer allocator.free(rmark);
    @memset(rmark, false);
    // Split-string ("slack induction") is a large-n lever: it helps where there is
    // room to redistribute (X-n303/502/1001 all improve; X-n1001 1.41% -> 0.94%) but
    // disrupts small, tight instances (X-n200 regressed). Auto (split_rate < 0) gates
    // it on for n >= 250 — the same regime split the HGS population uses.
    const eff_split: f64 = if (params.split_rate < 0) (if (n >= POP_CROSSOVER_N) @as(f64, 0.5) else 0.0) else params.split_rate;
    // Regret recreate is a mid/large-n lever (see SisrParams.regret_rate): auto-gate
    // it on for n >= 250 (the split-string boundary), off below.
    const eff_regret: f64 = if (params.regret_rate < 0) (if (n >= POP_CROSSOVER_N and n <= REGRET_MAX_N) @as(f64, 1.0) else 0.0) else params.regret_rate;
    var ctx = SisrCtx{ .present = present, .removed = removed, .rprev = rprev, .rroute = rroute, .ins = ins, .touched = touched, .rmark = rmark, .blink = params.blink, .l_max = params.l_max, .cbar = params.cbar, .split_rate = eff_split, .split_alpha = params.split_alpha, .regret_rate = eff_regret };

    var prng = std.Random.DefaultPrng.init(options.seed);
    const rng = prng.random();

    const unit = @as(f64, @floatFromInt(cur.distance)) / @as(f64, @floatFromInt(n));
    const t0 = @max(1e-9, params.t0_factor * unit);
    const tf = @max(1e-9, params.tf_factor * unit);
    const iters = @max(@as(usize, 1), params.iters);
    const cf = std.math.pow(f64, tf / t0, 1.0 / @as(f64, @floatFromInt(iters)));
    var temp = t0;

    // UCB1 bandit over {plain, split} ruin: learns the best mix online instead of the
    // static n>=250 gate. Q = mean reward (move improved), N = pulls, exploration ~sqrt2.
    var bq = [2]f64{ 0, 0 };
    var bn = [2]f64{ 1, 1 };
    var bt: f64 = 2;

    // In-place ruin+recreate with O(removed) rollback on reject (no snapshot copy).
    var it: usize = 0;
    while (it < iters) : (it += 1) {
        const saved_dist = cur.distance;
        const saved_nroutes = cur.nroutes;
        var arm: usize = 0;
        if (params.bandit) {
            const ucb_plain = bq[0] + UCB_C * @sqrt(@log(bt) / bn[0]);
            const ucb_split = bq[1] + UCB_C * @sqrt(@log(bt) / bn[1]);
            arm = if (ucb_split > ucb_plain) 1 else 0;
            ctx.force_split = @intCast(arm);
        }
        cur.sisrRuin(&ctx, rng);
        const use_regret = ctx.regret_rate >= 1.0 or (ctx.regret_rate > 0 and rng.float(f64) < ctx.regret_rate);
        if (use_regret) cur.sisrRecreateRegret(&ctx) else cur.sisrRecreate(&ctx, rng);
        const dt = @as(i64, @intCast(cur.distance)) - @as(i64, @intCast(saved_dist));
        if (params.bandit) {
            const reward: f64 = if (dt < 0) 1 else 0;
            bn[arm] += 1;
            bt += 1;
            bq[arm] += (reward - bq[arm]) / bn[arm];
        }
        // Threshold Accepting (Dueck & Scheuer): accept any move not worse than the
        // current threshold `temp`. Deterministic — no exp(), no acceptance RNG draw —
        // vs Metropolis exp(-dt/temp). Same geometric schedule drives the threshold.
        const accept = @as(f64, @floatFromInt(dt)) < temp;
        if (accept) {
            if (cur.distance < best_dist) {
                best.copyLiveFrom(&cur);
                best_dist = cur.distance;
            }
        } else {
            cur.sisrUndo(&ctx, saved_dist, saved_nroutes);
        }
        // Debug invariant: the live structure's true distance must match the value
        // maintained incrementally through ruin/recreate (and restored by undo). Run
        // every iteration in Debug so every reject+undo path is validated (tests are
        // tiny-n); release builds skip it entirely.
        if (builtin.mode == .Debug) {
            const inc = cur.distance;
            cur.flushLinks();
            std.debug.assert(cur.distance == inc);
        }
        temp *= cf;
    }

    best.flushLinks(); // order/route_end/load/distance/cost from the linked rep
    return best.toResult(allocator);
}

// --- Best-of-K parallel SISR -------------------------------------------------
// K independent SISR chains (seed + i) run concurrently, each on its own arena
// over the thread-safe page allocator; the lowest-cost feasible result wins. This
// is the compute-bound engine's speed/accuracy lever: on K cores it costs ~one
// chain's wall but searches K independent trajectories (SISR is stochastic, so the
// best of K beats a single chain). Falls back to inline when a spawn fails.
const SisrSlot = struct {
    inst: CvrpInstance,
    options: solver.SolveOptions,
    params: SisrParams,
    seed: u64,
    order: []usize, // parent-owned flat customer order (size n)
    ends: []usize, // parent-owned route-end boundaries (size n)
    nroutes: usize = 0,
    cost: u64 = std.math.maxInt(u64),
    ok: bool = false,
};

fn sisrWorker(slot: *SisrSlot) void {
    var arena = std.heap.ArenaAllocator.init(std.heap.page_allocator);
    defer arena.deinit();
    var opts = slot.options;
    opts.seed = slot.seed;
    const res = solveCvrpSisr(arena.allocator(), slot.inst, opts, slot.params) catch {
        slot.ok = false;
        return;
    };
    // flatten arena-owned routes into the parent-owned slot buffers before teardown
    var w: usize = 0;
    for (res.routes, 0..) |route, ri| {
        @memcpy(slot.order[w .. w + route.len], route);
        w += route.len;
        slot.ends[ri] = w;
    }
    slot.nroutes = res.routes.len;
    slot.cost = res.total_cost;
    slot.ok = true;
}

/// Run `threads` independent SISR chains and return the best. threads<=1 is the
/// plain serial path. Each chain uses options.seed + chain index.
pub fn solveCvrpSisrParallel(allocator: std.mem.Allocator, inst: CvrpInstance, options: solver.SolveOptions, params: SisrParams, threads: usize) !CvrpResult {
    const cpus = std.Thread.getCpuCount() catch 1;
    const k = if (threads == 0) @max(@as(usize, 1), cpus -| 1) else threads;
    if (k <= 1 or inst.n <= 2) return solveCvrpSisr(allocator, inst, options, params);
    const n = inst.n;

    const slots = try allocator.alloc(SisrSlot, k);
    defer allocator.free(slots);
    var allocated: usize = 0;
    errdefer for (slots[0..allocated]) |s| {
        allocator.free(s.order);
        allocator.free(s.ends);
    };
    for (slots, 0..) |*s, i| {
        s.* = .{
            .inst = inst,
            .options = options,
            .params = params,
            .seed = options.seed +% i,
            .order = try allocator.alloc(usize, n),
            .ends = try allocator.alloc(usize, n),
        };
        allocated += 1;
    }

    const ths = try allocator.alloc(std.Thread, k);
    defer allocator.free(ths);
    var spawned: usize = 0;
    for (0..k) |i| {
        ths[i] = std.Thread.spawn(.{}, sisrWorker, .{&slots[i]}) catch break;
        spawned += 1;
    }
    for (spawned..k) |i| sisrWorker(&slots[i]); // inline fallback if a spawn failed
    for (0..spawned) |i| ths[i].join();

    var best: ?usize = null;
    for (slots, 0..) |s, i| {
        if (!s.ok) continue;
        if (best == null or s.cost < slots[best.?].cost) best = i;
    }
    const winner = best orelse return error.AllChainsFailed;

    // build the result from the winning slot's flat order/ends in the parent allocator
    const bs = slots[winner];
    const routes = try allocator.alloc([]usize, bs.nroutes);
    var start: usize = 0;
    for (0..bs.nroutes) |ri| {
        const end = bs.ends[ri];
        routes[ri] = try allocator.dupe(usize, bs.order[start..end]);
        start = end;
    }
    for (slots) |s| {
        allocator.free(s.order);
        allocator.free(s.ends);
    }
    return .{ .allocator = allocator, .routes = routes, .total_cost = bs.cost };
}

// --- Best-of-K parallel HGS (same idea, for the fleet-capped / asymmetric path) ---
const HgsSlot = struct {
    inst: CvrpInstance,
    options: solver.SolveOptions,
    params: HgsParams,
    max_vehicles: usize,
    seed: u64,
    order: []usize,
    ends: []usize,
    nroutes: usize = 0,
    cost: u64 = std.math.maxInt(u64),
    ok: bool = false,
};

fn hgsWorker(slot: *HgsSlot) void {
    var arena = std.heap.ArenaAllocator.init(std.heap.page_allocator);
    defer arena.deinit();
    var opts = slot.options;
    opts.seed = slot.seed;
    const res = solveCvrpHgs(arena.allocator(), slot.inst, opts, slot.params, slot.max_vehicles) catch {
        slot.ok = false;
        return;
    };
    var w: usize = 0;
    for (res.routes, 0..) |route, ri| {
        @memcpy(slot.order[w .. w + route.len], route);
        w += route.len;
        slot.ends[ri] = w;
    }
    slot.nroutes = res.routes.len;
    slot.cost = res.total_cost;
    slot.ok = true;
}

/// Run `threads` independent HGS searches (seed + i) and return the best. threads<=1
/// is the plain serial path. Brings best-of-K to the fleet-capped CVRP/ACVRP path.
pub fn solveCvrpHgsParallel(allocator: std.mem.Allocator, inst: CvrpInstance, options: solver.SolveOptions, params: HgsParams, max_vehicles: usize, threads: usize) !CvrpResult {
    const cpus = std.Thread.getCpuCount() catch 1;
    const k = if (threads == 0) @max(@as(usize, 1), cpus -| 1) else threads;
    if (k <= 1 or inst.n <= 2) return solveCvrpHgs(allocator, inst, options, params, max_vehicles);
    const n = inst.n;

    const slots = try allocator.alloc(HgsSlot, k);
    defer allocator.free(slots);
    var allocated: usize = 0;
    errdefer for (slots[0..allocated]) |s| {
        allocator.free(s.order);
        allocator.free(s.ends);
    };
    for (slots, 0..) |*s, i| {
        s.* = .{
            .inst = inst,
            .options = options,
            .params = params,
            .max_vehicles = max_vehicles,
            .seed = options.seed +% i,
            .order = try allocator.alloc(usize, n),
            .ends = try allocator.alloc(usize, n),
        };
        allocated += 1;
    }
    const ths = try allocator.alloc(std.Thread, k);
    defer allocator.free(ths);
    var spawned: usize = 0;
    for (0..k) |i| {
        ths[i] = std.Thread.spawn(.{}, hgsWorker, .{&slots[i]}) catch break;
        spawned += 1;
    }
    for (spawned..k) |i| hgsWorker(&slots[i]);
    for (0..spawned) |i| ths[i].join();

    var best: ?usize = null;
    for (slots, 0..) |s, i| {
        if (!s.ok) continue;
        if (best == null or s.cost < slots[best.?].cost) best = i;
    }
    const winner = best orelse return error.AllChainsFailed;
    const bs = slots[winner];
    const routes = try allocator.alloc([]usize, bs.nroutes);
    var start: usize = 0;
    for (0..bs.nroutes) |ri| {
        const end = bs.ends[ri];
        routes[ri] = try allocator.dupe(usize, bs.order[start..end]);
        start = end;
    }
    for (slots) |s| {
        allocator.free(s.order);
        allocator.free(s.ends);
    }
    return .{ .allocator = allocator, .routes = routes, .total_cost = bs.cost };
}

// Don't-look queue: a FIFO ring of customers awaiting (re-)examination, with an
// `active` bitset so each customer is queued at most once. The local search only
// touches customers whose neighbourhood changed since they were last seen clean,
// turning each pass from O(n) into O(dirty). Backed by caller-owned slices.
const Dlq = struct {
    q: []usize, // ring buffer, capacity = q.len
    active: []bool, // active[c] = c currently in the ring
    head: usize = 0,
    tail: usize = 0,
    count: usize = 0,

    fn push(self: *Dlq, u: usize) void {
        if (u == 0 or self.active[u]) return;
        self.q[self.tail] = u;
        self.tail = (self.tail + 1) % self.q.len;
        self.count += 1;
        self.active[u] = true;
    }
    fn pop(self: *Dlq) usize {
        const u = self.q[self.head];
        self.head = (self.head + 1) % self.q.len;
        self.count -= 1;
        self.active[u] = false;
        return u;
    }
    fn pushAll(self: *Dlq, nodes: []const usize) void {
        for (nodes) |x| self.push(x);
    }
};

// Solution = routes over customers 1..n, stored as a contiguous `order` array
// partitioned by `route_end` boundaries, plus per-route load. Relocate/swap
// rebuild affected loads; cost is maintained incrementally where cheap and
// recomputed on structural change.
const Solution = struct {
    allocator: std.mem.Allocator,
    inst: CvrpInstance,
    order: []usize, // customers, grouped by route
    route_end: []usize, // exclusive end index of each route in `order`
    nroutes: usize,
    load: []u32, // per route
    cost: u64, // distance + capacity penalty + fleet penalty (the comparison key)
    distance: u64 = 0, // pure travel distance (the reported objective)
    pen_coeff: u64 = 0, // capacity-excess penalty per unit overload (0 = hard feasibility gate)
    cap_excess: u64 = 0, // total capacity overload = sum_r max(0, load[r] - capacity)
    scratch: []usize, // n scratch for perturb
    max_vehicles: usize = 0, // fleet cap for perturb's re-Split (0 = unlimited)
    pos: []usize, // customer -> absolute index in `order` (rebuilt by recompute)
    rof: []usize, // customer -> route index (rebuilt by recompute)
    gran: []const usize = &.{}, // borrowed k-nearest lists: gran[(c-1)*gk + i]; empty = off
    gk: usize = 0,
    // Doubly-linked route view (scratch, rebuilt from order/route_end at each
    // localSearch entry by buildLinks, flushed back by flushLinks). Lets every
    // move be an O(1)/O(seg) splice instead of an O(n) array shift + recompute.
    // Indexed by customer 1..n (0 = depot/route boundary). NOT part of the
    // logical state — clone/copyFrom don't carry it; it's recomputed on demand.
    next: []usize = &.{}, // successor of customer c in its route (0 = route end)
    prev: []usize = &.{}, // predecessor (0 = route start)
    head: []usize = &.{}, // head[r] = first customer of route r (0 = empty)
    tail: []usize = &.{}, // tail[r] = last customer of route r (0 = empty)
    active: []bool = &.{}, // don't-look bit: customer queued for re-examination

    fn fromPred(allocator: std.mem.Allocator, inst: CvrpInstance, giant: []const usize, pred: []const usize) !Solution {
        const n = inst.n;
        var s = Solution{
            .allocator = allocator,
            .inst = inst,
            .order = try allocator.alloc(usize, n),
            .route_end = try allocator.alloc(usize, n),
            .nroutes = 0,
            .load = try allocator.alloc(u32, n),
            .cost = 0,
            .scratch = try allocator.alloc(usize, n),
            .pos = try allocator.alloc(usize, n + 1),
            .rof = try allocator.alloc(usize, n + 1),
            .next = try allocator.alloc(usize, n + 1),
            .prev = try allocator.alloc(usize, n + 1),
            .head = try allocator.alloc(usize, n),
            .tail = try allocator.alloc(usize, n),
            .active = try allocator.alloc(bool, n + 1),
        };
        @memcpy(s.order, giant);
        // boundaries from pred chain
        var bounds: [4096]usize = undefined;
        var nb: usize = 0;
        var i = n;
        while (i > 0) {
            bounds[nb] = i;
            nb += 1;
            i = pred[i];
        }
        // bounds are descending ends; reverse into route_end ascending
        for (0..nb) |k| s.route_end[k] = bounds[nb - 1 - k];
        s.nroutes = nb;
        s.recompute();
        return s;
    }

    fn deinit(self: *Solution) void {
        self.allocator.free(self.order);
        self.allocator.free(self.route_end);
        self.allocator.free(self.load);
        self.allocator.free(self.scratch);
        self.allocator.free(self.pos);
        self.allocator.free(self.rof);
        self.allocator.free(self.next);
        self.allocator.free(self.prev);
        self.allocator.free(self.head);
        self.allocator.free(self.tail);
        self.allocator.free(self.active);
        self.* = undefined;
    }

    fn routeStart(self: *const Solution, r: usize) usize {
        return if (r == 0) 0 else self.route_end[r - 1];
    }

    fn dd(self: *const Solution, a: usize, b: usize) i64 {
        return @intCast(self.inst.d(a, b));
    }

    // i-th element of route order[s..e] with relative index `ru` skipped.
    fn sAt(self: *const Solution, s: usize, ru: usize, i: usize) usize {
        return self.order[s + (if (i < ru) i else i + 1)];
    }

    const InsertResult = struct { delta: i64, gap: usize };

    /// Best gap to insert customer `x` into route order[s..e] with relative index
    /// `ru` removed. Returns the marginal distance delta and the gap (in the
    /// skipped-route coordinates: gap g inserts before the g-th remaining element).
    fn bestInsertSkip(self: *const Solution, s: usize, e: usize, ru: usize, x: usize) InsertResult {
        const ls = (e - s) - 1; // length after removing one
        var best: i64 = std.math.maxInt(i64);
        var best_g: usize = 0;
        var g: usize = 0;
        while (g <= ls) : (g += 1) {
            const prev = if (g == 0) 0 else self.sAt(s, ru, g - 1);
            const next = if (g == ls) 0 else self.sAt(s, ru, g);
            const dl = self.dd(prev, x) + self.dd(x, next) - self.dd(prev, next);
            if (dl < best) {
                best = dl;
                best_g = g;
            }
        }
        return .{ .delta = best, .gap = best_g };
    }

    /// Build into `scratch` the route order[s..e] with relative index `ru` removed
    /// and customer `x` inserted at gap `g`. Length is unchanged (e-s).
    fn buildSkipInsert(self: *Solution, s: usize, e: usize, ru: usize, x: usize, g: usize) void {
        const ls = (e - s) - 1;
        var w: usize = 0;
        var i: usize = 0;
        while (i <= ls) : (i += 1) {
            if (i == g) {
                self.scratch[w] = x;
                w += 1;
            }
            if (i < ls) {
                self.scratch[w] = self.sAt(s, ru, i);
                w += 1;
            }
        }
    }

    fn recompute(self: *Solution) void {
        var total: u64 = 0;
        var nonempty: usize = 0;
        for (0..self.nroutes) |r| {
            const s = self.routeStart(r);
            const e = self.route_end[r];
            if (e > s) nonempty += 1;
            var load: u32 = 0;
            var prev: usize = 0;
            for (self.order[s..e], s..) |c, idx| {
                total += self.inst.d(prev, c);
                load += self.inst.demand[c];
                prev = c;
                self.pos[c] = idx; // customer -> absolute index
                self.rof[c] = r; // customer -> route
            }
            total += self.inst.d(prev, 0);
            self.load[r] = load;
        }
        self.distance = total;
        // Capacity overload across all routes (0 when feasible). Penalised in the
        // comparison cost so the search can pass through infeasible space.
        var capx: u64 = 0;
        for (0..self.nroutes) |r| {
            if (self.load[r] > self.inst.capacity) capx += self.load[r] - self.inst.capacity;
        }
        self.cap_excess = capx;
        // Penalty is on the ACTUAL vehicle count (non-empty routes, what toResult
        // reports), so emptying a route via relocate/elimination lowers cost.
        const excess: u64 = if (self.max_vehicles > 0 and nonempty > self.max_vehicles) nonempty - self.max_vehicles else 0;
        self.cost = total + capx * self.pen_coeff + excess * FLEET_PENALTY;
    }

    fn clone(self: *const Solution) !Solution {
        const c = Solution{
            .allocator = self.allocator,
            .inst = self.inst,
            .order = try self.allocator.dupe(usize, self.order),
            .route_end = try self.allocator.dupe(usize, self.route_end),
            .nroutes = self.nroutes,
            .load = try self.allocator.dupe(u32, self.load),
            .cost = self.cost,
            .distance = self.distance,
            .pen_coeff = self.pen_coeff,
            .cap_excess = self.cap_excess,
            .scratch = try self.allocator.alloc(usize, self.order.len),
            .max_vehicles = self.max_vehicles,
            .pos = try self.allocator.dupe(usize, self.pos),
            .rof = try self.allocator.dupe(usize, self.rof),
            .gran = self.gran, // borrowed, shared read-only
            .gk = self.gk,
            // link view is scratch — allocate fresh, rebuilt at next localSearch
            .next = try self.allocator.alloc(usize, self.next.len),
            .prev = try self.allocator.alloc(usize, self.prev.len),
            .head = try self.allocator.alloc(usize, self.head.len),
            .tail = try self.allocator.alloc(usize, self.tail.len),
            .active = try self.allocator.alloc(bool, self.active.len),
        };
        return c;
    }
    fn copyFrom(self: *Solution, o: Solution) void {
        self.distance = o.distance;
        @memcpy(self.order, o.order);
        @memcpy(self.route_end, o.route_end);
        @memcpy(self.load, o.load);
        @memcpy(self.pos, o.pos);
        @memcpy(self.rof, o.rof);
        self.nroutes = o.nroutes;
        self.cost = o.cost;
        self.cap_excess = o.cap_excess;
        self.pen_coeff = o.pen_coeff;
    }

    // node at (route r, position p); 0 = depot for p<0 or p>=len handled by caller
    fn at(self: *const Solution, r: usize, p: usize) usize {
        return self.order[self.routeStart(r) + p];
    }

    /// Relocate + swap local search to a local optimum (first-improvement sweeps).
    /// Dispatches to the O(1)-move linked-list engine when granular lists are on
    /// (the real path; n>=2), falling back to the array engine only for the
    /// degenerate gk==0 case (n<=1, nothing to move).
    fn localSearch(self: *Solution) !void {
        if (self.gk == 0) return self.localSearchArray();
        return self.localSearchLinked();
    }

    fn localSearchArray(self: *Solution) !void {
        const d = struct {
            fn f(s: *const Solution, a: usize, b: usize) i64 {
                return @intCast(s.inst.d(a, b));
            }
        }.f;
        var improved = true;
        while (improved) {
            improved = false;
            if (self.gk > 0) {
                // RELOCATE (granular): try moving each customer u next to one of its
                // k-nearest neighbours w — between (pred(w),w) or (w,succ(w)) in w's
                // route. O(n*k) per sweep instead of O(n^2); same delta math.
                var u: usize = 1;
                while (u <= self.inst.n) : (u += 1) {
                    const ri = self.rof[u];
                    const si = self.routeStart(ri);
                    const ei = self.route_end[ri];
                    const pi = self.pos[u];
                    const a = if (pi == si) 0 else self.order[pi - 1];
                    const b = if (pi + 1 == ei) 0 else self.order[pi + 1];
                    const removal = d(self, a, u) + d(self, u, b) - d(self, a, b);
                    const nbrs = self.gran[(u - 1) * self.gk ..][0..self.gk];
                    // apply-and-continue: on an improving move, recompute (refreshes
                    // pos/rof) and advance to the next customer rather than restarting
                    // the whole cascade — converges in a few sweeps, not O(moves) sweeps.
                    relnb: for (nbrs) |w| {
                        if (w == 0) continue;
                        const rj = self.rof[w];
                        if (rj != ri and self.load[rj] + self.inst.demand[u] > self.inst.capacity) continue;
                        const sj = self.routeStart(rj);
                        const ej = self.route_end[rj];
                        const pw = self.pos[w];
                        for ([_]usize{ pw, pw + 1 }) |qj| {
                            if (qj > ej) continue;
                            if (rj == ri and (qj == pi or qj == pi + 1)) continue;
                            const c = if (qj == sj) 0 else self.order[qj - 1];
                            const e = if (qj == ej) 0 else self.order[qj];
                            const insert = d(self, c, u) + d(self, u, e) - d(self, c, e);
                            if (insert - removal < 0) {
                                self.applyRelocate(ri, pi, rj, qj);
                                self.recompute();
                                improved = true;
                                break :relnb;
                            }
                        }
                    }
                }
                if (improved) continue;
            } else {
                // RELOCATE (full): move customer u (route ri, pos pi) before qj in rj.
                var ri: usize = 0;
                while (ri < self.nroutes) : (ri += 1) {
                    const si = self.routeStart(ri);
                    const ei = self.route_end[ri];
                    var pi: usize = si;
                    while (pi < ei) : (pi += 1) {
                        const u = self.order[pi];
                        const a = if (pi == si) 0 else self.order[pi - 1];
                        const b = if (pi + 1 == ei) 0 else self.order[pi + 1];
                        const removal = d(self, a, u) + d(self, u, b) - d(self, a, b);
                        var rj: usize = 0;
                        while (rj < self.nroutes) : (rj += 1) {
                            if (rj != ri and self.load[rj] + self.inst.demand[u] > self.inst.capacity) continue;
                            const sj = self.routeStart(rj);
                            const ej = self.route_end[rj];
                            var qj: usize = sj;
                            while (qj <= ej) : (qj += 1) {
                                if (rj == ri and (qj == pi or qj == pi + 1)) continue;
                                const c = if (qj == sj) 0 else self.order[qj - 1];
                                const e = if (qj == ej) 0 else self.order[qj];
                                const insert = d(self, c, u) + d(self, u, e) - d(self, c, e);
                                if (insert - removal < 0) {
                                    self.applyRelocate(ri, pi, rj, qj);
                                    improved = true;
                                    self.recompute();
                                    break;
                                }
                            }
                            if (improved) break;
                        }
                        if (improved) break;
                    }
                    if (improved) break;
                }
                if (improved) continue;
            }
            // OR-OPT (granular): relocate a chain of `len` consecutive customers
            // (len 2,3) next to one of seg_first's k-nearest neighbours, reversal-
            // free (the chain keeps its direction), so asymmetric-safe. Captures
            // moving pairs/triples that single relocate and swap miss.
            or_opt: for ([_]usize{ 2, 3 }) |len| {
                var rri: usize = 0;
                while (rri < self.nroutes) : (rri += 1) {
                    const si = self.routeStart(rri);
                    const ei = self.route_end[rri];
                    if (ei < si + len) continue;
                    var i: usize = si;
                    while (i + len <= ei) : (i += 1) {
                        const seg_first = self.order[i];
                        const seg_last = self.order[i + len - 1];
                        var seg_demand: u32 = 0;
                        for (self.order[i .. i + len]) |c| seg_demand += self.inst.demand[c];
                        const a = if (i == si) 0 else self.order[i - 1];
                        const b = if (i + len == ei) 0 else self.order[i + len];
                        const removal = d(self, a, seg_first) + d(self, seg_last, b) - d(self, a, b);
                        const nbrs = self.gran[(seg_first - 1) * self.gk ..][0..self.gk];
                        for (nbrs) |w| {
                            if (w == 0) continue;
                            const rrj = self.rof[w];
                            const pw = self.pos[w];
                            if (rrj == rri and pw >= i and pw < i + len) continue; // w inside the block
                            if (rrj != rri and self.load[rrj] + seg_demand > self.inst.capacity) continue;
                            const sj = self.routeStart(rrj);
                            const ej = self.route_end[rrj];
                            for ([_]usize{ pw, pw + 1 }) |qj| {
                                if (qj > ej) continue;
                                if (rrj == rri and qj >= i and qj <= i + len) continue; // gap inside/adjacent to block
                                const c = if (qj == sj) 0 else self.order[qj - 1];
                                const e = if (qj == ej) 0 else self.order[qj];
                                const insert = d(self, c, seg_first) + d(self, seg_last, e) - d(self, c, e);
                                if (insert - removal < 0) {
                                    self.applyOrOpt(rri, i, len, rrj, qj);
                                    self.recompute();
                                    improved = true;
                                    break :or_opt;
                                }
                            }
                        }
                    }
                }
            }
            if (improved) continue;
            // INTRA-ROUTE 2-OPT: reverse order[i..=j] within a route. The delta
            // sums the reversed internal arcs explicitly, so it is exact for
            // DIRECTIONAL costs too (for symmetric instances the internal term is
            // zero and this is the classic two boundary-edge swap).
            var rr: usize = 0;
            two_opt: while (rr < self.nroutes) : (rr += 1) {
                const s = self.routeStart(rr);
                const e = self.route_end[rr];
                if (e < s + 2) continue;
                var i: usize = s;
                while (i + 1 < e) : (i += 1) {
                    const a = if (i == s) 0 else self.order[i - 1];
                    const u = self.order[i];
                    var j: usize = i + 1;
                    while (j < e) : (j += 1) {
                        const v = self.order[j];
                        const b = if (j + 1 == e) 0 else self.order[j + 1];
                        var delta: i64 = d(self, a, v) + d(self, u, b) - d(self, a, u) - d(self, v, b);
                        var k: usize = i;
                        while (k < j) : (k += 1) {
                            delta += d(self, self.order[k + 1], self.order[k]) - d(self, self.order[k], self.order[k + 1]);
                        }
                        if (delta < 0) {
                            std.mem.reverse(usize, self.order[i .. j + 1]);
                            self.recompute();
                            improved = true;
                            break :two_opt;
                        }
                    }
                }
            }
            if (improved) continue;
            // SWAP (granular): exchange customer u with a k-nearest neighbour v in
            // their current slots (route lengths unchanged). Covers intra-route
            // adjacent reordering (v=succ(u) is normally a near neighbour) that the
            // inter-route-only SWAP* below cannot. Directional-safe adjacency terms.
            {
                var u: usize = 1;
                while (u <= self.inst.n) : (u += 1) {
                    const ru = self.rof[u];
                    const pu = self.pos[u];
                    const su = self.routeStart(ru);
                    const eu = self.route_end[ru];
                    const a1 = if (pu == su) 0 else self.order[pu - 1];
                    const b1 = if (pu + 1 == eu) 0 else self.order[pu + 1];
                    const nbrs = self.gran[(u - 1) * self.gk ..][0..self.gk];
                    swnb: for (nbrs) |v| {
                        if (v == 0 or v == u) continue;
                        const rv = self.rof[v];
                        if (ru != rv) {
                            if (self.load[ru] - self.inst.demand[u] + self.inst.demand[v] > self.inst.capacity) continue;
                            if (self.load[rv] - self.inst.demand[v] + self.inst.demand[u] > self.inst.capacity) continue;
                        }
                        const pv = self.pos[v];
                        const sv = self.routeStart(rv);
                        const ev = self.route_end[rv];
                        const a2 = if (pv == sv) 0 else self.order[pv - 1];
                        const b2 = if (pv + 1 == ev) 0 else self.order[pv + 1];
                        var delta: i64 = 0;
                        if (ru == rv and pv == pu + 1) {
                            delta = d(self, a1, v) + d(self, v, u) + d(self, u, b2) - d(self, a1, u) - d(self, u, v) - d(self, v, b2);
                        } else if (ru == rv and pu == pv + 1) {
                            delta = d(self, a2, u) + d(self, u, v) + d(self, v, b1) - d(self, a2, v) - d(self, v, u) - d(self, u, b1);
                        } else {
                            delta = d(self, a1, v) + d(self, v, b1) - d(self, a1, u) - d(self, u, b1) +
                                d(self, a2, u) + d(self, u, b2) - d(self, a2, v) - d(self, v, b2);
                        }
                        if (delta < 0) {
                            self.order[pu] = v;
                            self.order[pv] = u;
                            self.recompute();
                            improved = true;
                            break :swnb;
                        }
                    }
                }
            }
            if (improved) continue;
            // SWAP*: exchange uu (route ri) with a near neighbour vv (route rj != ri),
            // each REINSERTED at its own best position in the other route — not just
            // trading slots. The strongest inter-route move (Vidal), restricted to
            // k-nearest neighbours for speed. Each route trades one customer for one,
            // so route lengths and boundaries are unchanged (only contents + loads).
            if (self.gk > 0) {
                var uu: usize = 1;
                while (uu <= self.inst.n) : (uu += 1) {
                    const sri = self.rof[uu];
                    const s1 = self.routeStart(sri);
                    const e1 = self.route_end[sri];
                    if (e1 == s1) continue;
                    const pu = self.pos[uu];
                    const ru = pu - s1;
                    const a_u = if (pu == s1) 0 else self.order[pu - 1];
                    const b_u = if (pu + 1 == e1) 0 else self.order[pu + 1];
                    const gu: i64 = self.dd(a_u, uu) + self.dd(uu, b_u) - self.dd(a_u, b_u);
                    const nbrs = self.gran[(uu - 1) * self.gk ..][0..self.gk];
                    ssnb: for (nbrs) |vv| {
                        if (vv == 0) continue;
                        const srj = self.rof[vv];
                        if (srj == sri) continue;
                        if (self.load[sri] - self.inst.demand[uu] + self.inst.demand[vv] > self.inst.capacity) continue;
                        if (self.load[srj] - self.inst.demand[vv] + self.inst.demand[uu] > self.inst.capacity) continue;
                        const s2 = self.routeStart(srj);
                        const e2 = self.route_end[srj];
                        const pv = self.pos[vv];
                        const rv = pv - s2;
                        const c_v = if (pv == s2) 0 else self.order[pv - 1];
                        const e_v = if (pv + 1 == e2) 0 else self.order[pv + 1];
                        const gv: i64 = self.dd(c_v, vv) + self.dd(vv, e_v) - self.dd(c_v, e_v);
                        const bi_v = self.bestInsertSkip(s1, e1, ru, vv); // vv -> ri\uu
                        const bi_u = self.bestInsertSkip(s2, e2, rv, uu); // uu -> rj\vv
                        const delta = (bi_v.delta - gu) + (bi_u.delta - gv);
                        if (delta < 0) {
                            self.buildSkipInsert(s1, e1, ru, vv, bi_v.gap);
                            @memcpy(self.order[s1..e1], self.scratch[0 .. e1 - s1]);
                            self.buildSkipInsert(s2, e2, rv, uu, bi_u.gap);
                            @memcpy(self.order[s2..e2], self.scratch[0 .. e2 - s2]);
                            self.recompute();
                            improved = true;
                            break :ssnb;
                        }
                    }
                }
            }
            if (improved) continue;
            // FLEET REPAIR: distance moves have converged. If we're over the vehicle
            // cap, empty one route into the others (accepting the distance cost — the
            // penalty drop dominates), then let the distance moves re-optimise the
            // new structure. Each success strictly lowers the non-empty route count,
            // so this terminates.
            if (self.max_vehicles > 0 and self.nonEmptyCount() > self.max_vehicles) {
                if (try self.eliminateOneRoute()) {
                    self.compact();
                    self.recompute();
                    improved = true;
                }
            }
        }
    }

    // ---- Linked-list local search (the O(1)-move engine) -----------------
    // Same moves and same delta math as localSearchArray, but routes are a
    // doubly-linked list (next/prev/head/tail) so each accepted move is an
    // O(1) splice (O(seg) for or-opt/2-opt) instead of an O(n) array shift +
    // full recompute. The move sequence is bit-identical to the array engine;
    // only the bookkeeping changed. order/pos/distance/cost are stale during
    // the sweep and resynced once by flushLinks() at the end (and around fleet
    // repair, which still runs on the array view).

    /// Rebuild next/prev/head/tail from the array view. O(n). Assumes load/rof
    /// already correct (recompute set them); refreshes rof anyway for safety.
    fn buildLinks(self: *Solution) void {
        for (0..self.nroutes) |r| {
            const s = self.routeStart(r);
            const e = self.route_end[r];
            if (e == s) {
                self.head[r] = 0;
                self.tail[r] = 0;
                continue;
            }
            self.head[r] = self.order[s];
            self.tail[r] = self.order[e - 1];
            var pr: usize = 0;
            for (self.order[s..e]) |c| {
                self.prev[c] = pr;
                if (pr != 0) self.next[pr] = c;
                self.rof[c] = r;
                pr = c;
            }
            self.next[pr] = 0;
        }
    }

    /// Walk the linked routes back into order/route_end, then recompute() to
    /// refresh pos/distance/cost/load exactly. O(n).
    fn flushLinks(self: *Solution) void {
        var w: usize = 0;
        for (0..self.nroutes) |r| {
            var c = self.head[r];
            while (c != 0) : (c = self.next[c]) {
                self.order[w] = c;
                w += 1;
            }
            self.route_end[r] = w;
        }
        self.recompute();
    }

    /// Splice customer u out of its route (prev/next/head/tail/load).
    fn linkRemove(self: *Solution, u: usize) void {
        const r = self.rof[u];
        const p = self.prev[u];
        const nx = self.next[u];
        if (p == 0) self.head[r] = nx else self.next[p] = nx;
        if (nx == 0) self.tail[r] = p else self.prev[nx] = p;
        self.load[r] -= self.inst.demand[u];
    }

    /// Insert customer u into route r between adjacent nodes a and b (0 = depot
    /// boundary; a must currently precede b in r).
    fn linkInsert(self: *Solution, u: usize, r: usize, a: usize, b: usize) void {
        self.prev[u] = a;
        self.next[u] = b;
        if (a == 0) self.head[r] = u else self.next[a] = u;
        if (b == 0) self.tail[r] = u else self.prev[b] = u;
        self.rof[u] = r;
        self.load[r] += self.inst.demand[u];
    }

    /// Splice the contiguous block first..last (internal links intact) out.
    fn linkRemoveBlock(self: *Solution, first: usize, last: usize, seg_demand: u32) void {
        const r = self.rof[first];
        const p = self.prev[first];
        const nx = self.next[last];
        if (p == 0) self.head[r] = nx else self.next[p] = nx;
        if (nx == 0) self.tail[r] = p else self.prev[nx] = p;
        self.load[r] -= seg_demand;
    }

    /// Insert block first..last between a and b in route r; set rof of all nodes.
    fn linkInsertBlock(self: *Solution, first: usize, last: usize, r: usize, a: usize, b: usize, seg_demand: u32) void {
        self.prev[first] = a;
        self.next[last] = b;
        if (a == 0) self.head[r] = first else self.next[a] = first;
        if (b == 0) self.tail[r] = last else self.prev[b] = last;
        var c = first;
        while (true) {
            self.rof[c] = r;
            if (c == last) break;
            c = self.next[c];
        }
        self.load[r] += seg_demand;
    }

    /// Reverse segment x..last (inclusive) within one route (2-opt apply).
    fn reverseSeg(self: *Solution, x: usize, last: usize) void {
        const r = self.rof[x];
        const lb = self.prev[x]; // node before the segment (0 = head boundary)
        const rb = self.next[last]; // node after the segment (0 = tail boundary)
        var cur = x;
        while (true) {
            const nx = self.next[cur];
            const t = self.prev[cur];
            self.prev[cur] = self.next[cur];
            self.next[cur] = t;
            if (cur == last) break;
            cur = nx;
        }
        self.prev[last] = lb; // 'last' is the new first
        self.next[x] = rb; // 'x' is the new last
        if (lb == 0) self.head[r] = last else self.next[lb] = last;
        if (rb == 0) self.tail[r] = x else self.prev[rb] = x;
    }

    /// 2-opt* tail exchange between u's route r1 and d's route r2 (r2 != r1):
    /// new edges u->d and c->b where b=next(u), c=prev(d). Route r1 keeps head..u
    /// then takes d..tail(r2); r2 keeps head..c then takes b..tail(r1). Reversal-
    /// free (both tails keep direction), so directional-safe. new1/new2 = the
    /// caller's precomputed loads for r1/r2.
    fn apply2optStar(self: *Solution, u: usize, d: usize, new1: u32, new2: u32) void {
        const r1 = self.rof[u];
        const r2 = self.rof[d];
        const b = self.next[u];
        const c = self.prev[d];
        const t1 = self.tail[r1];
        const t2 = self.tail[r2];
        // r1: ...u -> d..t2
        self.next[u] = d;
        self.prev[d] = u;
        self.tail[r1] = t2;
        // r2: ...c -> b..t1  (ends at c when b == 0)
        if (c == 0) self.head[r2] = b else self.next[c] = b;
        if (b != 0) {
            self.prev[b] = c;
            self.tail[r2] = t1;
        } else {
            self.tail[r2] = c;
        }
        // relabel route membership of the two swapped tails
        {
            var x = d;
            while (true) : (x = self.next[x]) {
                self.rof[x] = r1;
                if (x == t2) break;
            }
        }
        if (b != 0) {
            var x = b;
            while (true) : (x = self.next[x]) {
                self.rof[x] = r2;
                if (x == t1) break;
            }
        }
        self.load[r1] = new1;
        self.load[r2] = new2;
    }

    fn inBlock(self: *const Solution, x: usize, first: usize, last: usize) bool {
        var c = first;
        while (true) {
            if (c == x) return true;
            if (c == last) return false;
            c = self.next[c];
        }
    }

    const InsLink = struct { delta: i64, c: usize, e: usize };

    /// Best gap to insert x into route r with node `skip` removed. Returns the
    /// marginal delta and the gap endpoints (c precedes e; 0 = depot boundary).
    fn bestInsertSkipLink(self: *const Solution, r: usize, skip: usize, x: usize) InsLink {
        var best: i64 = std.math.maxInt(i64);
        var best_c: usize = 0;
        var best_e: usize = 0;
        var c: usize = 0; // depot start
        var cur = self.head[r];
        while (true) {
            while (cur == skip) cur = self.next[cur];
            const e = cur; // node or 0 (depot end)
            const dl = self.dd(c, x) + self.dd(x, e) - self.dd(c, e);
            if (dl < best) {
                best = dl;
                best_c = c;
                best_e = e;
            }
            if (cur == 0) break;
            c = cur;
            cur = self.next[cur];
        }
        return .{ .delta = best, .c = best_c, .e = best_e };
    }

    /// Exchange customers u and v (swap their slots). Handles adjacency and
    /// cross-route membership; loads updated only when routes differ.
    fn swapNodes(self: *Solution, u: usize, v: usize) void {
        const ru = self.rof[u];
        const rv = self.rof[v];
        const a1 = self.prev[u];
        const b1 = self.next[u];
        const a2 = self.prev[v];
        const b2 = self.next[v];
        if (b1 == v) {
            // u immediately before v (same route): a1, v, u, b2
            self.prev[v] = a1;
            self.next[v] = u;
            self.prev[u] = v;
            self.next[u] = b2;
            if (a1 == 0) self.head[ru] = v else self.next[a1] = v;
            if (b2 == 0) self.tail[ru] = u else self.prev[b2] = u;
        } else if (b2 == u) {
            // v immediately before u (same route): a2, u, v, b1
            self.prev[u] = a2;
            self.next[u] = v;
            self.prev[v] = u;
            self.next[v] = b1;
            if (a2 == 0) self.head[ru] = u else self.next[a2] = u;
            if (b1 == 0) self.tail[ru] = v else self.prev[b1] = v;
        } else {
            // disjoint slots (same or different routes)
            self.prev[v] = a1;
            self.next[v] = b1;
            if (a1 == 0) self.head[ru] = v else self.next[a1] = v;
            if (b1 == 0) self.tail[ru] = v else self.prev[b1] = v;
            self.rof[v] = ru;
            self.prev[u] = a2;
            self.next[u] = b2;
            if (a2 == 0) self.head[rv] = u else self.next[a2] = u;
            if (b2 == 0) self.tail[rv] = u else self.prev[b2] = u;
            self.rof[u] = rv;
            if (ru != rv) {
                self.load[ru] = self.load[ru] - self.inst.demand[u] + self.inst.demand[v];
                self.load[rv] = self.load[rv] - self.inst.demand[v] + self.inst.demand[u];
            }
        }
    }

    fn localSearchLinked(self: *Solution) !void {
        self.buildLinks();
        const cap = self.inst.capacity;
        const n = self.inst.n;
        @memset(self.active[0 .. n + 1], false);
        var dq = Dlq{ .q = self.scratch[0..n], .active = self.active };

        while (true) {
            // Seed every customer once and drain the queue. Each applied move
            // re-activates only the customers whose incident edges changed (plus
            // their k-nearest, approximating reverse-neighbours), so passes cost
            // O(dirty), not O(n). A single drain — deliberately NOT iterated to a
            // perfect local optimum: at scale, a fast loose education that feeds
            // HGS more generations beats a thorough one (the FILO/SVC finding,
            // confirmed here — re-seeding to convergence was slower AND no better
            // at n>=100, helping only sub-100 toy instances).
            dq.head = 0;
            dq.tail = 0;
            dq.count = 0;
            @memset(self.active[0 .. n + 1], false);
            var seed: usize = 1;
            while (seed <= n) : (seed += 1) dq.push(seed);
            while (dq.count > 0) {
                const u = dq.pop();
                while (self.improveAt(u, cap, &dq)) {}
            }

            // FLEET REPAIR: distance moves converged. If over the vehicle cap,
            // empty one route on the array view, rebuild links and re-seed.
            if (self.max_vehicles > 0) {
                var ne: usize = 0;
                for (0..self.nroutes) |r| {
                    if (self.head[r] != 0) ne += 1;
                }
                if (ne > self.max_vehicles) {
                    self.flushLinks();
                    if (try self.eliminateOneRoute()) {
                        self.compact();
                        self.recompute();
                        self.buildLinks();
                        continue;
                    }
                }
            }
            break;
        }
        self.flushLinks(); // resync order/pos/distance/cost for the caller
    }

    /// Try every move type rooted at customer u (relocate, or-opt, 2-opt, swap,
    /// swap*), in priority order, and apply the first improving one. On a move,
    /// re-activate the endpoints of the changed edges (so their own moves get a
    /// fresh look) and return true. Returns false when u is a local optimum.
    fn improveAt(self: *Solution, u: usize, cap: u32, dq: *Dlq) bool {
        const nbrs = self.gran[(u - 1) * self.gk ..][0..self.gk];
        // Capacity penalty: pen_coeff>0 admits overloaded moves at a price (infeasible
        // search); pen_coeff==0 falls back to GATE_PEN, an effective hard feasibility
        // gate. Acceptance everywhere is dist_delta + PEN * excess_delta < 0.
        const PEN: i64 = if (self.pen_coeff == 0) GATE_PEN else @intCast(self.pen_coeff);
        const du: u32 = self.inst.demand[u];

        // RELOCATE: move u into the gap before/after a near neighbour w.
        {
            const ri = self.rof[u];
            const a = self.prev[u];
            const b = self.next[u];
            const removal = self.dd(a, u) + self.dd(u, b) - self.dd(a, b);
            for (nbrs) |w| {
                if (w == 0) continue;
                const rj = self.rof[w];
                const pen: i64 = if (rj == ri) 0 else PEN *
                    ((capExcess(self.load[rj] + du, cap) - capExcess(self.load[rj], cap)) +
                        (capExcess(self.load[ri] - du, cap) - capExcess(self.load[ri], cap)));
                var side: usize = 0;
                while (side < 2) : (side += 1) {
                    const c = if (side == 0) self.prev[w] else w;
                    const e = if (side == 0) w else self.next[w];
                    if (c == u or e == u) continue; // u's own slot (no-op)
                    if (self.dd(c, u) + self.dd(u, e) - self.dd(c, e) - removal + pen < 0) {
                        self.linkRemove(u);
                        self.linkInsert(u, rj, c, e);
                        dq.pushAll(&.{ a, b, c, e, w, u });
                        dq.pushAll(nbrs); // reverse-neighbours of u (kNN ~symmetric)
                        return true;
                    }
                }
            }
        }

        // OR-OPT: relocate the chain u..last (len 2,3), reversal-free, next to a
        // near neighbour of u.
        for ([_]usize{ 2, 3 }) |len| {
            var last = u;
            var ok = true;
            var cnt: usize = 1;
            while (cnt < len) : (cnt += 1) {
                last = self.next[last];
                if (last == 0) {
                    ok = false;
                    break;
                }
            }
            if (!ok) continue;
            const ri = self.rof[u];
            var seg_demand: u32 = 0;
            {
                var c = u;
                while (true) {
                    seg_demand += self.inst.demand[c];
                    if (c == last) break;
                    c = self.next[c];
                }
            }
            const a = self.prev[u];
            const bb = self.next[last];
            const removal = self.dd(a, u) + self.dd(last, bb) - self.dd(a, bb);
            for (nbrs) |w| {
                if (w == 0) continue;
                if (self.inBlock(w, u, last)) continue;
                const rj = self.rof[w];
                const pen: i64 = if (rj == ri) 0 else PEN *
                    ((capExcess(self.load[rj] + seg_demand, cap) - capExcess(self.load[rj], cap)) +
                        (capExcess(self.load[ri] - seg_demand, cap) - capExcess(self.load[ri], cap)));
                var side: usize = 0;
                while (side < 2) : (side += 1) {
                    const c = if (side == 0) self.prev[w] else w;
                    const e = if (side == 0) w else self.next[w];
                    if (self.inBlock(c, u, last) or self.inBlock(e, u, last)) continue;
                    if (self.dd(c, u) + self.dd(last, e) - self.dd(c, e) - removal + pen < 0) {
                        const mid = self.next[u]; // middle node (only distinct for len 3)
                        self.linkRemoveBlock(u, last, seg_demand);
                        self.linkInsertBlock(u, last, rj, c, e, seg_demand);
                        dq.pushAll(&.{ a, bb, c, e, w, u, mid, last });
                        dq.pushAll(nbrs);
                        return true;
                    }
                }
            }
        }

        // INTRA-ROUTE 2-OPT: reverse u..y (u as the left endpoint). Internal arc
        // sum accumulates as y advances, exact for directional costs.
        {
            const a = self.prev[u];
            var internal: i64 = 0;
            var y = self.next[u];
            while (y != 0) : (y = self.next[y]) {
                const yp = self.prev[y];
                internal += self.dd(y, yp) - self.dd(yp, y);
                const b = self.next[y];
                if (self.dd(a, y) + self.dd(u, b) - self.dd(a, u) - self.dd(y, b) + internal < 0) {
                    self.reverseSeg(u, y);
                    dq.pushAll(&.{ a, u, y, b });
                    dq.pushAll(nbrs);
                    return true;
                }
            }
        }

        // SWAP: exchange u with a near neighbour v.
        {
            const ru = self.rof[u];
            const a1 = self.prev[u];
            const b1 = self.next[u];
            for (nbrs) |v| {
                if (v == 0 or v == u) continue;
                const rv = self.rof[v];
                const dv: u32 = self.inst.demand[v];
                const pen: i64 = if (ru == rv) 0 else PEN *
                    ((capExcess(self.load[ru] - du + dv, cap) - capExcess(self.load[ru], cap)) +
                        (capExcess(self.load[rv] - dv + du, cap) - capExcess(self.load[rv], cap)));
                const a2 = self.prev[v];
                const b2 = self.next[v];
                var delta: i64 = 0;
                if (ru == rv and b1 == v) {
                    delta = self.dd(a1, v) + self.dd(v, u) + self.dd(u, b2) - self.dd(a1, u) - self.dd(u, v) - self.dd(v, b2);
                } else if (ru == rv and b2 == u) {
                    delta = self.dd(a2, u) + self.dd(u, v) + self.dd(v, b1) - self.dd(a2, v) - self.dd(v, u) - self.dd(u, b1);
                } else {
                    delta = self.dd(a1, v) + self.dd(v, b1) - self.dd(a1, u) - self.dd(u, b1) +
                        self.dd(a2, u) + self.dd(u, b2) - self.dd(a2, v) - self.dd(v, b2);
                }
                if (delta + pen < 0) {
                    self.swapNodes(u, v);
                    dq.pushAll(&.{ a1, b1, a2, b2, u, v });
                    dq.pushAll(nbrs);
                    dq.pushAll(self.gran[(v - 1) * self.gk ..][0..self.gk]);
                    return true;
                }
            }
        }

        // SWAP*: trade u and a near neighbour vv in a different route, each
        // reinserted at its own best position.
        {
            const sri = self.rof[u];
            const a_u = self.prev[u];
            const b_u = self.next[u];
            const gu: i64 = self.dd(a_u, u) + self.dd(u, b_u) - self.dd(a_u, b_u);
            for (nbrs) |vv| {
                if (vv == 0) continue;
                const srj = self.rof[vv];
                if (srj == sri) continue;
                const dvv: u32 = self.inst.demand[vv];
                const pen: i64 = PEN *
                    ((capExcess(self.load[sri] - du + dvv, cap) - capExcess(self.load[sri], cap)) +
                        (capExcess(self.load[srj] - dvv + du, cap) - capExcess(self.load[srj], cap)));
                const c_v = self.prev[vv];
                const e_v = self.next[vv];
                const gv: i64 = self.dd(c_v, vv) + self.dd(vv, e_v) - self.dd(c_v, e_v);
                const bi_v = self.bestInsertSkipLink(sri, u, vv); // vv -> ri\u
                const bi_u = self.bestInsertSkipLink(srj, vv, u); // u -> rj\vv
                if ((bi_v.delta - gu) + (bi_u.delta - gv) + pen < 0) {
                    self.linkRemove(u);
                    self.linkRemove(vv);
                    self.linkInsert(vv, sri, bi_v.c, bi_v.e);
                    self.linkInsert(u, srj, bi_u.c, bi_u.e);
                    dq.pushAll(&.{ a_u, b_u, c_v, e_v, bi_v.c, bi_v.e, bi_u.c, bi_u.e, u, vv });
                    dq.pushAll(nbrs);
                    dq.pushAll(self.gran[(vv - 1) * self.gk ..][0..self.gk]);
                    return true;
                }
            }
        }

        // 2-OPT*: exchange the tail after u with the tail after c=prev(d), for a
        // near neighbour d in another route (creates edge u->d). Repartitions which
        // customers belong to which route — the move single relocate/swap can't make.
        {
            const r1 = self.rof[u];
            const b = self.next[u];
            const L1: u32 = self.load[r1];
            // prefix load up to u inclusive in r1 (computed once)
            var pu: u32 = 0;
            {
                var x = self.head[r1];
                while (true) : (x = self.next[x]) {
                    pu += self.inst.demand[x];
                    if (x == u) break;
                }
            }
            for (nbrs) |d| {
                if (d == 0) continue;
                const r2 = self.rof[d];
                if (r2 == r1) continue;
                const c = self.prev[d];
                const L2: u32 = self.load[r2];
                var pc: u32 = 0; // prefix load up to c inclusive in r2 (0 if c is depot)
                if (c != 0) {
                    var x = self.head[r2];
                    while (true) : (x = self.next[x]) {
                        pc += self.inst.demand[x];
                        if (x == c) break;
                    }
                }
                const dist_delta = self.dd(u, d) + self.dd(c, b) - self.dd(u, b) - self.dd(c, d);
                const new1: u32 = pu + (L2 - pc);
                const new2: u32 = pc + (L1 - pu);
                const pen: i64 = PEN *
                    ((capExcess(new1, cap) - capExcess(L1, cap)) + (capExcess(new2, cap) - capExcess(L2, cap)));
                if (dist_delta + pen < 0) {
                    self.apply2optStar(u, d, new1, new2);
                    dq.pushAll(&.{ u, b, c, d });
                    dq.pushAll(nbrs);
                    dq.pushAll(self.gran[(d - 1) * self.gk ..][0..self.gk]);
                    return true;
                }
            }
        }

        return false;
    }

    fn nonEmptyCount(self: *const Solution) usize {
        var c: usize = 0;
        var prev: usize = 0;
        for (0..self.nroutes) |r| {
            if (self.route_end[r] > prev) c += 1;
            prev = self.route_end[r];
        }
        return c;
    }

    /// Drop zero-length routes from `route_end` (the `order` array is untouched —
    /// an empty route is just two equal consecutive boundaries). nroutes shrinks;
    /// loads are stale until the caller recompute()s.
    fn compact(self: *Solution) void {
        var w: usize = 0;
        var prev: usize = 0;
        for (0..self.nroutes) |r| {
            const end = self.route_end[r];
            if (end > prev) {
                self.route_end[w] = end;
                w += 1;
                prev = end;
            }
        }
        self.nroutes = w;
    }

    /// Try to empty exactly one route into the others, under a fleet cap. A feasible
    /// redistribution is found first (best-fit by remaining capacity over the route's
    /// customers, heaviest first); only if every customer fits is it applied (each to
    /// its best-distance position in the chosen route), so no restore is needed. Tries
    /// the lightest-load route first. Returns true if a route was emptied.
    fn eliminateOneRoute(self: *Solution) !bool {
        const alloc = self.allocator;
        const cap = self.inst.capacity;
        const nr = self.nroutes;
        const cands = try alloc.alloc(usize, nr);
        defer alloc.free(cands);
        var nne: usize = 0;
        {
            var prev: usize = 0;
            for (0..nr) |r| {
                if (self.route_end[r] > prev) {
                    cands[nne] = r;
                    nne += 1;
                }
                prev = self.route_end[r];
            }
        }
        if (nne <= 1) return false;
        // lightest first: easiest to redistribute
        std.sort.pdq(usize, cands[0..nne], self, struct {
            fn lt(s: *const Solution, a: usize, b: usize) bool {
                return s.load[a] < s.load[b] or (s.load[a] == s.load[b] and a < b);
            }
        }.lt);

        const wload = try alloc.alloc(u32, nr);
        defer alloc.free(wload);
        const tgt = try alloc.alloc(usize, self.inst.n + 1);
        defer alloc.free(tgt);
        const custs = try alloc.alloc(usize, self.inst.n);
        defer alloc.free(custs);

        for (cands[0..nne]) |r| {
            const s = self.routeStart(r);
            const e = self.route_end[r];
            const m = e - s;
            @memcpy(custs[0..m], self.order[s..e]);
            // assign heaviest customers first (bin-packing best-fit-decreasing)
            std.sort.pdq(usize, custs[0..m], self.inst, struct {
                fn gt(inst: CvrpInstance, a: usize, b: usize) bool {
                    return inst.demand[a] > inst.demand[b] or (inst.demand[a] == inst.demand[b] and a < b);
                }
            }.gt);
            @memcpy(wload, self.load);
            var feasible = true;
            for (custs[0..m]) |c| {
                var best_rj: usize = nr; // sentinel
                var best_slack: i64 = -1;
                var prev: usize = 0;
                for (0..nr) |rj| {
                    const nonempty = self.route_end[rj] > prev;
                    prev = self.route_end[rj];
                    if (!nonempty or rj == r) continue;
                    if (wload[rj] + self.inst.demand[c] > cap) continue;
                    const slack: i64 = @as(i64, cap) - @as(i64, wload[rj]) - @as(i64, self.inst.demand[c]);
                    if (slack > best_slack) {
                        best_slack = slack;
                        best_rj = rj;
                    }
                }
                if (best_rj == nr) {
                    feasible = false;
                    break;
                }
                tgt[c] = best_rj;
                wload[best_rj] += self.inst.demand[c];
            }
            if (!feasible) continue;
            // Apply: relocate r's customers one at a time (always the route's current
            // first customer, which walks the original order) to the planned route's
            // best-distance gap. Capacity is guaranteed by the plan.
            while (self.route_end[r] > self.routeStart(r)) {
                const pi = self.routeStart(r);
                const c = self.order[pi];
                const rj = tgt[c];
                const sj = self.routeStart(rj);
                const ej = self.route_end[rj];
                var best_q: usize = sj;
                var best_delta: i64 = std.math.maxInt(i64);
                var qj: usize = sj;
                while (qj <= ej) : (qj += 1) {
                    const pa = if (qj == sj) 0 else self.order[qj - 1];
                    const pb = if (qj == ej) 0 else self.order[qj];
                    const delta = @as(i64, @intCast(self.inst.d(pa, c))) + @as(i64, @intCast(self.inst.d(c, pb))) - @as(i64, @intCast(self.inst.d(pa, pb)));
                    if (delta < best_delta) {
                        best_delta = delta;
                        best_q = qj;
                    }
                }
                self.applyRelocate(r, pi, rj, best_q);
            }
            return true;
        }
        return false;
    }

    fn applyRelocate(self: *Solution, ri: usize, pi: usize, rj: usize, qj: usize) void {
        const u = self.order[pi];
        // remove pi, then insert at qj' (adjust if qj after pi in the array)
        const insert_at = if (qj > pi) qj - 1 else qj;
        // shift-remove
        std.mem.copyForwards(usize, self.order[pi .. self.order.len - 1], self.order[pi + 1 ..]);
        // shift-insert
        std.mem.copyBackwards(usize, self.order[insert_at + 1 ..], self.order[insert_at .. self.order.len - 1]);
        self.order[insert_at] = u;
        // fix boundaries: route ri loses one before its end region, rj gains one.
        // Easiest correct fix: rebuild route_end by walking with the known move.
        // ri's end and all ends between shrink by 1 down to the insertion side.
        if (ri == rj) return; // same route: boundaries unchanged
        if (ri < rj) {
            var r = ri;
            while (r < rj) : (r += 1) self.route_end[r] -= 1;
        } else {
            var r = rj;
            while (r < ri) : (r += 1) self.route_end[r] += 1;
        }
    }

    /// Move the length-`len` block at absolute index `i` (route ri) to the gap
    /// before position `qj` in route rj (reversal-free; the block keeps its
    /// internal order). Boundary bookkeeping mirrors applyRelocate with step len.
    fn applyOrOpt(self: *Solution, ri: usize, i: usize, len: usize, rj: usize, qj: usize) void {
        var tmp: [3]usize = undefined;
        @memcpy(tmp[0..len], self.order[i .. i + len]);
        const insert_at = if (qj >= i + len) qj - len else qj;
        // shift-remove the block
        std.mem.copyForwards(usize, self.order[i .. self.order.len - len], self.order[i + len ..]);
        // shift-insert len slots at insert_at
        std.mem.copyBackwards(usize, self.order[insert_at + len ..], self.order[insert_at .. self.order.len - len]);
        @memcpy(self.order[insert_at .. insert_at + len], tmp[0..len]);
        if (ri == rj) return;
        if (ri < rj) {
            var r = ri;
            while (r < rj) : (r += 1) self.route_end[r] -= len;
        } else {
            var r = rj;
            while (r < ri) : (r += 1) self.route_end[r] += len;
        }
    }

    /// Perturb: rebuild a giant tour from the current routes, apply a double-bridge
    /// (4-opt) kick on the giant order, re-Split. Keeps it a valid solution to
    /// re-optimise from. Double-bridge is the canonical ILS kick: large enough to
    /// escape a local optimum, structured enough not to destroy good sub-paths.
    fn perturb(self: *Solution, rng: std.Random) void {
        const n = self.order.len;
        if (n >= 8) {
            // 3 cut points 0<p1<p2<p3<n; reassemble A C B D.
            const p1 = 1 + rng.uintLessThan(usize, n - 3);
            const p2 = p1 + 1 + rng.uintLessThan(usize, n - p1 - 2);
            const p3 = p2 + 1 + rng.uintLessThan(usize, n - p2 - 1);
            var k: usize = 0;
            for (self.order[0..p1]) |c| {
                self.scratch[k] = c;
                k += 1;
            }
            for (self.order[p2..p3]) |c| {
                self.scratch[k] = c;
                k += 1;
            }
            for (self.order[p1..p2]) |c| {
                self.scratch[k] = c;
                k += 1;
            }
            for (self.order[p3..]) |c| {
                self.scratch[k] = c;
                k += 1;
            }
        } else {
            @memcpy(self.scratch, self.order);
            if (n >= 4) {
                for (0..3) |_| {
                    const i = rng.uintLessThan(usize, n);
                    const j = rng.uintLessThan(usize, n);
                    std.mem.swap(usize, &self.scratch[i], &self.scratch[j]);
                }
            }
        }
        const sp = (if (self.max_vehicles > 0)
            splitDpK(self.allocator, self.inst, self.scratch, self.max_vehicles)
        else
            splitDp(self.allocator, self.inst, self.scratch)) catch return;
        defer self.allocator.free(sp.pred);
        @memcpy(self.order, self.scratch);
        var bounds: [4096]usize = undefined;
        var nb: usize = 0;
        var i = n;
        while (i > 0) {
            bounds[nb] = i;
            nb += 1;
            i = sp.pred[i];
        }
        for (0..nb) |k| self.route_end[k] = bounds[nb - 1 - k];
        self.nroutes = nb;
        self.recompute();
    }

    // ---- SISR (ruin-and-recreate) working methods on the linked rep ----------
    // The SISR loop keeps two Solution buffers and copies the *live* linked state
    // (next/prev/head/tail/rof/load + nroutes + distance) each iteration; order/
    // route_end/pos are only refreshed at the very end (flushLinks). distance is
    // maintained incrementally through every ruin/recreate splice.

    /// Copy only the live linked state from `o` (no order/route_end/pos rebuild).
    fn copyLiveFrom(self: *Solution, o: *const Solution) void {
        @memcpy(self.next, o.next);
        @memcpy(self.prev, o.prev);
        @memcpy(self.head, o.head);
        @memcpy(self.tail, o.tail);
        @memcpy(self.rof, o.rof);
        @memcpy(self.load, o.load);
        self.nroutes = o.nroutes;
        self.distance = o.distance;
    }

    /// Grab a route slot for a brand-new single-customer route: reuse the first
    /// empty (head==0) slot, else append. nroutes <= n always (each non-empty
    /// route has >=1 customer), so the appended index stays in bounds.
    fn sisrOpenRoute(self: *Solution) usize {
        for (0..self.nroutes) |r| {
            if (self.head[r] == 0) {
                self.load[r] = 0;
                return r;
            }
        }
        const r = self.nroutes;
        self.nroutes += 1;
        self.head[r] = 0;
        self.tail[r] = 0;
        self.load[r] = 0;
        return r;
    }

    /// Remove a string of <= `l_s_max` contiguous customers anchored at present
    /// customer `m` (random length, random left offset, clamped at route ends).
    /// Updates distance incrementally and records the removed nodes in ctx.
    /// Returns true if it ruined m's route (false = m absent or route already hit).
    fn sisrRuinAnchor(self: *Solution, ctx: *SisrCtx, rng: std.Random, m: usize, l_s_max: usize) bool {
        if (m == 0 or !ctx.present[m]) return false;
        const r = self.rof[m];
        if (ctx.rmark[r]) return false;
        const l = 1 + rng.uintLessThan(usize, l_s_max); // [1, l_s_max]
        // Split-string ("slack induction"): remove l customers from a WIDER window,
        // preserving a contiguous block — so the removed customers come from a larger
        // span and reinsertion has more room. `and` short-circuits when split_rate==0,
        // so the plain path below stays bit-identical (no extra RNG draw).
        const do_split = if (ctx.force_split >= 0) (ctx.force_split == 1) else (ctx.split_rate > 0 and rng.float(f64) < ctx.split_rate);
        if (do_split) {
            self.sisrRuinSplit(ctx, rng, m, r, l, l_s_max);
            ctx.rmark[r] = true;
            ctx.touched[ctx.ntouched] = r;
            ctx.ntouched += 1;
            return true;
        }
        const left = rng.uintLessThan(usize, l); // [0, l-1] customers before m
        var start = m;
        var s: usize = 0;
        while (s < left and self.prev[start] != 0) : (s += 1) start = self.prev[start];
        const before = self.prev[start]; // original predecessor of the string's first node
        // collect up to l nodes from `start` rightward (links still intact). Record
        // each node's original predecessor (rprev) + route for exact undo: re-inserting
        // removed[j] after rprev[j], forward in removal order, rebuilds the string.
        var last = start;
        var cnt: usize = 0;
        var node = start;
        var seg_demand: u32 = 0;
        var internal: i64 = 0;
        var pn: usize = 0; // previous collected node (0 before the first)
        while (cnt < l and node != 0) : (cnt += 1) {
            if (pn != 0) internal += self.dd(pn, node);
            ctx.present[node] = false;
            ctx.removed[ctx.nrem] = node;
            ctx.rprev[ctx.nrem] = if (cnt == 0) before else pn;
            ctx.rroute[ctx.nrem] = r;
            ctx.nrem += 1;
            seg_demand += self.inst.demand[node];
            last = node;
            pn = node;
            node = self.next[node];
        }
        const after = self.next[last];
        const removed_edges = self.dd(before, start) + internal + self.dd(last, after);
        const added = self.dd(before, after);
        self.distance = @intCast(@as(i64, @intCast(self.distance)) + added - removed_edges);
        self.linkRemoveBlock(start, last, seg_demand);
        ctx.rmark[r] = true;
        ctx.touched[ctx.ntouched] = r;
        ctx.ntouched += 1;
        return true;
    }

    /// Split-string removal of `l` customers anchored at `m` in route `r`: collect a
    /// window of l + (geometrically-grown) preserved customers, keep a random
    /// contiguous block that does not cover the anchor, and remove the rest one node
    /// at a time (single-node distance delta; original-predecessor undo records, so
    /// forward re-insertion in removal order rebuilds the original chain through the
    /// preserved gap exactly).
    fn sisrRuinSplit(self: *Solution, ctx: *SisrCtx, rng: std.Random, m: usize, r: usize, l: usize, l_s_max: usize) void {
        var pres: usize = 1; // preserved-block size, grown geometrically (slack)
        while (rng.float(f64) < ctx.split_alpha and pres < l_s_max) pres += 1;
        const w_target = l + pres;
        const left = rng.uintLessThan(usize, w_target); // anchor offset within the window
        var start = m;
        var s: usize = 0;
        while (s < left and self.prev[start] != 0) : (s += 1) start = self.prev[start];
        const before = self.prev[start];
        // Window holds w_target = l + pres <= 2*l_max customers; sized for l_max <= 32.
        // The `cnt < win.len` guard below is the hard safety net (silently shortens the
        // window rather than overflowing) if a caller sets an out-of-range l_max.
        var win: [64]usize = undefined;
        std.debug.assert(w_target <= win.len);
        var cnt: usize = 0;
        var anchor_idx: usize = 0;
        var node = start;
        while (cnt < w_target and cnt < win.len and node != 0) : (cnt += 1) {
            win[cnt] = node;
            if (node == m) anchor_idx = cnt;
            node = self.next[node];
        }
        const remove_count = @min(l, cnt);
        var preserve_count = cnt - remove_count;
        // pick a preserved-block start that does not cover the anchor (else preserve none)
        var ps: usize = 0;
        if (preserve_count > 0) {
            var nvalid: usize = 0;
            var p: usize = 0;
            while (p + preserve_count <= cnt) : (p += 1) {
                if (anchor_idx < p or anchor_idx > p + preserve_count - 1) nvalid += 1;
            }
            if (nvalid == 0) {
                preserve_count = 0;
            } else {
                var pick = rng.uintLessThan(usize, nvalid);
                p = 0;
                while (p + preserve_count <= cnt) : (p += 1) {
                    if (anchor_idx < p or anchor_idx > p + preserve_count - 1) {
                        if (pick == 0) {
                            ps = p;
                            break;
                        }
                        pick -= 1;
                    }
                }
            }
        }
        var i: usize = 0;
        while (i < cnt) : (i += 1) {
            if (preserve_count > 0 and i >= ps and i < ps + preserve_count) continue;
            const w = win[i];
            ctx.present[w] = false;
            ctx.removed[ctx.nrem] = w;
            ctx.rprev[ctx.nrem] = if (i == 0) before else win[i - 1];
            ctx.rroute[ctx.nrem] = r;
            ctx.nrem += 1;
            const p2 = self.prev[w];
            const nx = self.next[w];
            self.distance = @intCast(@as(i64, @intCast(self.distance)) + self.dd(p2, nx) - self.dd(p2, w) - self.dd(w, nx));
            self.linkRemove(w);
        }
    }

    /// SISR ruin: pick a seed, remove spatially-adjacent strings (string-removal).
    fn sisrRuin(self: *Solution, ctx: *SisrCtx, rng: std.Random) void {
        const n = self.inst.n;
        var ne: usize = 0;
        for (0..self.nroutes) |r| {
            if (self.head[r] != 0) ne += 1;
        }
        if (ne == 0) return;
        const avg_card = @max(@as(usize, 1), n / ne);
        const l_s_max = @min(ctx.l_max, avg_card);
        const ks_max_f = (4.0 * ctx.cbar) / @as(f64, @floatFromInt(1 + l_s_max)) - 1.0;
        const ks_max = @max(@as(usize, 1), @as(usize, @intFromFloat(@max(1.0, ks_max_f))));
        const k_s = 1 + rng.uintLessThan(usize, ks_max); // [1, ks_max]
        ctx.nrem = 0;
        ctx.ntouched = 0;
        const seed = 1 + rng.uintLessThan(usize, n);
        var done: usize = 0;
        if (self.sisrRuinAnchor(ctx, rng, seed, l_s_max)) done += 1;
        if (done < k_s) {
            for (self.gran[(seed - 1) * self.gk ..][0..self.gk]) |mm| {
                if (done >= k_s) break;
                if (self.sisrRuinAnchor(ctx, rng, mm, l_s_max)) done += 1;
            }
        }
        for (ctx.touched[0..ctx.ntouched]) |r| ctx.rmark[r] = false;
    }

    /// Full-route scan fallback: cheapest feasible gap over all non-empty routes.
    fn sisrFullScan(self: *const Solution, c: usize, dem: u32, cap: u32, best: *i64, ba: *usize, bb: *usize, br: *usize) bool {
        var found = false;
        for (0..self.nroutes) |r| {
            if (self.head[r] == 0) continue;
            if (self.load[r] + dem > cap) continue;
            var a: usize = 0;
            var node = self.head[r];
            while (true) {
                const delta = self.dd(a, c) + self.dd(c, node) - self.dd(a, node);
                if (delta < best.*) {
                    best.* = delta;
                    ba.* = a;
                    bb.* = node;
                    br.* = r;
                    found = true;
                }
                if (node == 0) break;
                a = node;
                node = self.next[node];
            }
        }
        return found;
    }

    /// SISR recreate: greedy cheapest-insertion with blinks, granular candidate
    /// set (gaps adjacent to each removed customer's present k-nearest), full-scan
    /// fallback, new route as last resort. Hard-to-place (high demand) first.
    fn sisrRecreate(self: *Solution, ctx: *SisrCtx, rng: std.Random) void {
        const cap = self.inst.capacity;
        // Insert in demand-desc order (hard-to-place first), but keep ctx.removed in
        // removal order (aligned with rprev/rroute for undo) — sort a separate copy.
        const rem = ctx.ins[0..ctx.nrem];
        @memcpy(rem, ctx.removed[0..ctx.nrem]);
        std.sort.pdq(usize, rem, self.inst, struct {
            fn lt(inst: CvrpInstance, a: usize, b: usize) bool {
                return inst.demand[a] > inst.demand[b] or (inst.demand[a] == inst.demand[b] and a < b);
            }
        }.lt);
        for (rem) |c| {
            const dem = self.inst.demand[c];
            var best: i64 = std.math.maxInt(i64); // blinked choice
            var ba: usize = 0;
            var bb: usize = 0;
            var br: usize = 0;
            var found = false;
            var bany: i64 = std.math.maxInt(i64); // unblinked feasible backup
            var aa: usize = 0;
            var ab: usize = 0;
            var ar: usize = 0;
            var anyf = false;
            for (self.gran[(c - 1) * self.gk ..][0..self.gk]) |m| {
                if (m == 0 or !ctx.present[m]) continue;
                const r = self.rof[m];
                if (self.load[r] + dem > cap) continue;
                const p = self.prev[m];
                const nx = self.next[m];
                const d1 = self.dd(p, c) + self.dd(c, m) - self.dd(p, m);
                const d2 = self.dd(m, c) + self.dd(c, nx) - self.dd(m, nx);
                if (d1 < bany) {
                    bany = d1;
                    aa = p;
                    ab = m;
                    ar = r;
                    anyf = true;
                }
                if (rng.float(f64) >= ctx.blink and d1 < best) {
                    best = d1;
                    ba = p;
                    bb = m;
                    br = r;
                    found = true;
                }
                if (d2 < bany) {
                    bany = d2;
                    aa = m;
                    ab = nx;
                    ar = r;
                    anyf = true;
                }
                if (rng.float(f64) >= ctx.blink and d2 < best) {
                    best = d2;
                    ba = m;
                    bb = nx;
                    br = r;
                    found = true;
                }
            }
            if (!found and anyf) {
                best = bany;
                ba = aa;
                bb = ab;
                br = ar;
                found = true;
            }
            if (!found) found = self.sisrFullScan(c, dem, cap, &best, &ba, &bb, &br);
            if (!found) {
                br = self.sisrOpenRoute();
                ba = 0;
                bb = 0;
                best = self.dd(0, c) + self.dd(c, 0);
            }
            self.linkInsert(c, br, ba, bb);
            ctx.present[c] = true;
            self.distance = @intCast(@as(i64, @intCast(self.distance)) + best);
        }
    }

    /// SISR recreate by REGRET-2: at each step insert the still-unplaced customer
    /// whose (2nd-best - best) feasible insertion cost is largest — the one we'll
    /// regret most if we defer it. Deterministic (no blink). O(rem^2 * gk) vs greedy's
    /// O(rem * gk): stronger per-recreate, but fewer iterations at equal wall — this
    /// is the experiment. Records insertion order into ctx.ins (rem) for undo.
    fn sisrRecreateRegret(self: *Solution, ctx: *SisrCtx) void {
        const cap = self.inst.capacity;
        const rem = ctx.ins[0..ctx.nrem];
        @memcpy(rem, ctx.removed[0..ctx.nrem]);
        var i: usize = 0;
        while (i < rem.len) : (i += 1) {
            // Choose, among the not-yet-placed rem[i..], the max-regret customer.
            var pick: usize = i;
            var pick_reg: i64 = std.math.minInt(i64);
            var pick_c1: i64 = 0;
            var pick_a: usize = 0;
            var pick_b: usize = 0;
            var pick_r: usize = 0;
            var pick_newroute = false;
            for (i..rem.len) |j| {
                const c = rem[j];
                const dem = self.inst.demand[c];
                var c1: i64 = std.math.maxInt(i64); // best feasible insertion cost
                var c2: i64 = std.math.maxInt(i64); // 2nd-best feasible insertion cost
                var a1: usize = 0;
                var b1: usize = 0;
                var r1: usize = 0;
                var any = false;
                for (self.gran[(c - 1) * self.gk ..][0..self.gk]) |m| {
                    if (m == 0 or !ctx.present[m]) continue;
                    const r = self.rof[m];
                    if (self.load[r] + dem > cap) continue;
                    const p = self.prev[m];
                    const nx = self.next[m];
                    const d1 = self.dd(p, c) + self.dd(c, m) - self.dd(p, m);
                    const d2 = self.dd(m, c) + self.dd(c, nx) - self.dd(m, nx);
                    if (d1 < c1) {
                        c2 = c1;
                        c1 = d1;
                        a1 = p;
                        b1 = m;
                        r1 = r;
                        any = true;
                    } else if (d1 < c2) c2 = d1;
                    if (d2 < c1) {
                        c2 = c1;
                        c1 = d2;
                        a1 = m;
                        b1 = nx;
                        r1 = r;
                        any = true;
                    } else if (d2 < c2) c2 = d2;
                }
                var newroute = false;
                if (!any) {
                    var fb: i64 = std.math.maxInt(i64);
                    var fa: usize = 0;
                    var fbb: usize = 0;
                    var fr: usize = 0;
                    if (self.sisrFullScan(c, dem, cap, &fb, &fa, &fbb, &fr)) {
                        c1 = fb;
                        a1 = fa;
                        b1 = fbb;
                        r1 = fr;
                    } else {
                        c1 = @intCast(self.dd(0, c) + self.dd(c, 0));
                        newroute = true;
                    }
                }
                // Regret = gap to the 2nd option; a unique feasible slot (c2 == inf)
                // is maximally urgent. Tie-break: higher best-cost (harder), lower index.
                const reg: i64 = if (c2 == std.math.maxInt(i64)) std.math.maxInt(i64) else c2 - c1;
                if (reg > pick_reg or
                    (reg == pick_reg and c1 > pick_c1) or
                    (reg == pick_reg and c1 == pick_c1 and j < pick))
                {
                    pick = j;
                    pick_reg = reg;
                    pick_c1 = c1;
                    pick_a = a1;
                    pick_b = b1;
                    pick_r = r1;
                    pick_newroute = newroute;
                }
            }
            const c = rem[pick];
            var br = pick_r;
            var ba = pick_a;
            var bb = pick_b;
            if (pick_newroute) {
                br = self.sisrOpenRoute();
                ba = 0;
                bb = 0;
            }
            self.linkInsert(c, br, ba, bb);
            ctx.present[c] = true;
            self.distance = @intCast(@as(i64, @intCast(self.distance)) + pick_c1);
            // Record insertion order (move picked into slot i) so undo can unwind it.
            rem[pick] = rem[i];
            rem[i] = c;
        }
    }

    /// Roll back a rejected ruin+recreate, restoring the exact pre-iteration state.
    /// (1) undo recreate: remove the inserted customers in reverse insertion order;
    /// (2) undo ruin: re-insert each removed customer after its original predecessor,
    /// forward in removal order (rebuilds every string — strings are in distinct
    /// routes, so order between them is immaterial). distance/nroutes are restored
    /// from the caller's saved scalars (nroutes drops any routes recreate appended).
    fn sisrUndo(self: *Solution, ctx: *SisrCtx, saved_dist: u64, saved_nroutes: usize) void {
        var j = ctx.nrem;
        while (j > 0) {
            j -= 1;
            self.linkRemove(ctx.ins[j]);
        }
        for (0..ctx.nrem) |k| {
            const c = ctx.removed[k];
            const r = ctx.rroute[k];
            const a = ctx.rprev[k];
            const b = if (a == 0) self.head[r] else self.next[a];
            self.linkInsert(c, r, a, b);
        }
        self.distance = saved_dist;
        self.nroutes = saved_nroutes;
    }

    fn toResult(self: *const Solution, allocator: std.mem.Allocator) !CvrpResult {
        // Skip empty routes: relocate/swap can empty a route without compacting
        // the boundary, leaving a zero-length (zero-cost) phantom. The real
        // vehicle count is the number of non-empty routes.
        var nonempty: usize = 0;
        for (0..self.nroutes) |r| {
            if (self.route_end[r] > self.routeStart(r)) nonempty += 1;
        }
        const routes = try allocator.alloc([]usize, nonempty);
        var k: usize = 0;
        for (0..self.nroutes) |r| {
            const s = self.routeStart(r);
            const e = self.route_end[r];
            if (e == s) continue;
            routes[k] = try allocator.dupe(usize, self.order[s..e]);
            k += 1;
        }
        return .{ .allocator = allocator, .routes = routes, .total_cost = self.distance };
    }
};

/// Sum the true directional cost of a CVRP solution (each route depot->...->depot)
/// and check feasibility. Returns the cost, or null if any constraint is violated.
pub fn validate(inst: CvrpInstance, routes: []const []const usize) ?u64 {
    const seen = std.heap.page_allocator.alloc(bool, inst.n + 1) catch return null;
    defer std.heap.page_allocator.free(seen);
    @memset(seen, false);
    var total: u64 = 0;
    for (routes) |r| {
        if (r.len == 0) continue;
        var load: u64 = 0;
        var prev: usize = 0; // depot
        for (r) |c| {
            if (c == 0 or c > inst.n or seen[c]) return null; // invalid/duplicate customer
            seen[c] = true;
            load += inst.demand[c];
            total += inst.d(prev, c);
            prev = c;
        }
        total += inst.d(prev, 0); // back to depot
        if (load > inst.capacity) return null; // capacity exceeded
    }
    for (1..inst.n + 1) |c| if (!seen[c]) return null; // every customer served
    return total;
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

    var res = try solveCvrp(allocator, inst, .{
        .seed = 3,
        .budget = .{ .trials = 30, .max_passes = 40 },
        .candidates = .{ .candidate_count = 6, .candidate_mode = .alpha_nearness, .sparse_min_dimension = 0 },
        .search = .{ .enable_lk = true, .lk_max_depth = 5 },
    }, 30);
    defer res.deinit();

    // independently validated cost equals the reported cost
    const checked = validate(inst, res.routes) orelse return error.InfeasibleSolution;
    try std.testing.expectEqual(res.total_cost, checked);

    // beats the trivial baseline of one customer per route
    var baseline: u64 = 0;
    for (1..dim) |c| baseline += inst.d(0, c) + inst.d(c, 0);
    try std.testing.expect(res.total_cost < baseline);
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
    var res = try solveCvrp(allocator, inst, .{
        .seed = 11,
        .budget = .{ .trials = 30, .max_passes = 40 },
        .candidates = .{ .candidate_count = 6, .candidate_mode = .alpha_nearness, .sparse_min_dimension = 0 },
        .search = .{ .enable_lk = true, .lk_max_depth = 5 },
    }, 200);
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
    }, 30, 2, cap_k);
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
    }, 40, 3, cap_k);
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
    var res = try solveCvrp(allocator, inst, .{
        .seed = 1,
        .budget = .{ .trials = 20, .max_passes = 40 },
        .candidates = .{ .candidate_count = 6, .candidate_mode = .alpha_nearness, .sparse_min_dimension = 0 },
        .search = .{ .enable_lk = true, .lk_max_depth = 5 },
    }, 40);
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
