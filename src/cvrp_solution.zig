const std = @import("std");
const asymmetric = @import("asymmetric.zig");
const solver = @import("solver.zig");
const cvrp_types = @import("cvrp_types.zig");
const cvrp_split = @import("cvrp_split.zig");
const CvrpInstance = cvrp_types.CvrpInstance;
const CvrpResult = cvrp_types.CvrpResult;
const splitDp = cvrp_split.splitDp;
const splitDpK = cvrp_split.splitDpK;
const FLEET_PENALTY = cvrp_split.FLEET_PENALTY;
const GATE_PEN = cvrp_split.GATE_PEN;
const capExcess = cvrp_split.capExcess;


pub fn solveCvrpImpl(allocator: std.mem.Allocator, inst: CvrpInstance, options: solver.SolveOptions, rounds: usize, restarts: usize, max_vehicles: usize) !CvrpResult {
    const n = inst.n;
    if (inst.demand.len != n + 1 or inst.matrix.len != (std.math.mul(usize, n + 1, n + 1) catch return error.InvalidInstance)) return error.InvalidInstance;

    // Trivial fleets: the giant-tour seed routes through asymmetric.solveAtsp,
    // which rejects n < 2, so handle the degenerate sizes directly. n == 0: no
    // customers, an empty plan. n == 1: the lone customer is its own route iff it
    // fits the vehicle. Allocate routes with the passed allocator so the returned
    // CvrpResult.deinit() frees them exactly as the giant-tour path's toResult does.
    if (n <= 1) {
        if (n == 0) {
            const routes = try allocator.alloc([]usize, 0);
            errdefer allocator.free(routes);
            if (validate(inst, routes) == null) return error.Infeasible;
            return .{ .allocator = allocator, .routes = routes, .total_cost = 0 };
        }
        if (inst.demand[1] > inst.capacity) return error.NoFeasibleSplit;
        const routes = try allocator.alloc([]usize, 1);
        errdefer allocator.free(routes);
        routes[0] = try allocator.dupe(usize, &[_]usize{1});
        errdefer allocator.free(routes[0]);
        if (validate(inst, routes) == null) return error.Infeasible;
        return .{ .allocator = allocator, .routes = routes, .total_cost = inst.d(0, 1) + inst.d(1, 0) };
    }

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

    var result = try best.toResult(allocator);
    errdefer result.deinit();
    if (validate(inst, result.routes) == null) return error.Infeasible;
    return result;
}

/// k-nearest neighbour lists by directional proximity d(c,j)+d(j,c) (customers
/// 1..n, 1-indexed; gran[(c-1)*k + i] = the i-th nearest, 0-padded). Restricts
/// SWAP* to spatially close customers — the granular neighbourhood that makes the
/// search fast and scalable. Caller owns the returned slice.
pub fn buildCvrpNeighbors(allocator: std.mem.Allocator, inst: CvrpInstance, k: usize) ![]usize {
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

/// Education = Split the giant under the fleet cap, then drive to a local optimum.
/// Returns the educated Solution (caller flattens .order back to a giant tour).
pub fn educateGiant(allocator: std.mem.Allocator, inst: CvrpInstance, giant: []const usize, max_vehicles: usize, gran: []const usize, gk: usize, pen_coeff: u64) !Solution {
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

// Per-iteration ruin/recreate scratch (caller-owned slices). `present` is the
// invariant: all-true between iterations (ruin flips customers out, recreate
// restores them), so it never needs reset. The search mutates a single solution
// in place and rolls back rejected moves via the undo records below, so each
// iteration is O(removed), not O(n) — no full-state snapshot copy.
pub const SisrCtx = struct {
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
pub const Solution = struct {
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
        // boundaries from pred chain: write the descending route ends straight
        // into route_end (sized n; route count is always <= n), then reverse the
        // used prefix in place. No fixed-size stack buffer, so a tight-capacity
        // instance with > n/few-per-route routes can never overrun it.
        var nb: usize = 0;
        var i = n;
        while (i > 0) {
            s.route_end[nb] = i;
            nb += 1;
            i = pred[i];
        }
        std.mem.reverse(usize, s.route_end[0..nb]);
        s.nroutes = nb;
        s.recompute();
        return s;
    }

    pub fn deinit(self: *Solution) void {
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

    pub fn clone(self: *const Solution) !Solution {
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
    pub fn buildLinks(self: *Solution) void {
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
    pub fn flushLinks(self: *Solution) void {
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
        // descending route ends -> route_end (sized n, route count <= n), then
        // reverse the used prefix in place. No fixed [4096] buffer to overrun.
        var nb: usize = 0;
        var i = n;
        while (i > 0) {
            self.route_end[nb] = i;
            nb += 1;
            i = sp.pred[i];
        }
        std.mem.reverse(usize, self.route_end[0..nb]);
        self.nroutes = nb;
        self.recompute();
    }

    // ---- SISR (ruin-and-recreate) working methods on the linked rep ----------
    // The SISR loop keeps two Solution buffers and copies the *live* linked state
    // (next/prev/head/tail/rof/load + nroutes + distance) each iteration; order/
    // route_end/pos are only refreshed at the very end (flushLinks). distance is
    // maintained incrementally through every ruin/recreate splice.

    /// Copy only the live linked state from `o` (no order/route_end/pos rebuild).
    pub fn copyLiveFrom(self: *Solution, o: *const Solution) void {
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
    pub fn sisrRuin(self: *Solution, ctx: *SisrCtx, rng: std.Random) void {
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
    pub fn sisrRecreate(self: *Solution, ctx: *SisrCtx, rng: std.Random) void {
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
    pub fn sisrRecreateRegret(self: *Solution, ctx: *SisrCtx) void {
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
    pub fn sisrUndo(self: *Solution, ctx: *SisrCtx, saved_dist: u64, saved_nroutes: usize) void {
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

    pub fn toResult(self: *const Solution, allocator: std.mem.Allocator) !CvrpResult {
        // Skip empty routes: relocate/swap can empty a route without compacting
        // the boundary, leaving a zero-length (zero-cost) phantom. The real
        // vehicle count is the number of non-empty routes.
        var nonempty: usize = 0;
        for (0..self.nroutes) |r| {
            if (self.route_end[r] > self.routeStart(r)) nonempty += 1;
        }
        const routes = try allocator.alloc([]usize, nonempty);
        var k: usize = 0;
        errdefer {
            for (routes[0..k]) |rt| allocator.free(rt);
            allocator.free(routes);
        }
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
