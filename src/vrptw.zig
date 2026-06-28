const std = @import("std");
const problem = @import("problem.zig");
const solver = @import("solver.zig");
const asymmetric = @import("asymmetric.zig");
const hgs_core = @import("hgs_core.zig"); // shared HGS giant-tour operators (G8)

// VRP with Time Windows (VRPTW), asymmetric-capable. Builds on the same giant-
// tour + Split method as the CVRP core, but every route must also be schedulable:
// each customer i has a window [ready, due] for the START of service and a
// service duration; the vehicle leaves the depot at t=0, may wait if it arrives
// early, must not start service after `due`, and must return to the depot by
// `due[0]` (the horizon). Travel time = the directional distance matrix, so real
// asymmetric road times work directly. Objective: minimize total travel distance
// over TW- and capacity-feasible routes (waiting is free). Vehicle count is
// reported; an optional per-route penalty can bias Split toward fewer vehicles.

pub const VrptwInstance = struct {
    n: usize, // customers, excludes depot
    matrix: []const u32, // (n+1)^2 directional travel time = distance; depot=0
    demand: []const u32, // n+1, demand[0]=0
    capacity: u32,
    ready: []const u32, // n+1 earliest service start (ready[0]=0)
    due: []const u32, // n+1 latest service start; due[0] = horizon / depot close
    service: []const u32, // n+1 service duration (service[0]=0)

    fn dim(self: VrptwInstance) usize {
        return self.n + 1;
    }
    fn d(self: VrptwInstance, a: usize, b: usize) u64 {
        return self.matrix[a * self.dim() + b];
    }
};

/// Schedule a route (depot -> nodes... -> depot) and return its travel distance,
/// or null if it violates capacity, any customer's time window, or the depot
/// horizon. Leave the depot at t=0; wait for free if early; service starts at
/// max(arrival, ready) and must be <= due. This is the rounding-independent core
/// of all VRPTW feasibility — every move is gated by it.
pub fn scheduleSlice(inst: VrptwInstance, nodes: []const usize) ?u64 {
    var load: u64 = 0;
    var t: u64 = 0; // current time (depart depot at 0)
    var prev: usize = 0;
    var dist: u64 = 0;
    for (nodes) |c| {
        const tr = inst.d(prev, c);
        dist += tr;
        load += inst.demand[c];
        if (load > inst.capacity) return null;
        const arrive = t + tr;
        const start = @max(arrive, inst.ready[c]);
        if (start > inst.due[c]) return null;
        t = start + inst.service[c];
        prev = c;
    }
    const back = inst.d(prev, 0);
    dist += back;
    if (t + back > inst.due[0]) return null; // late return to depot
    return dist;
}

// Time-Window Segment (Vidal / PyVRP): a constant-size summary of a contiguous
// node sequence that lets two segments be CONCATENATED in O(1), giving the merged
// segment's total time warp (0 == time-window-feasible) without rescheduling. This
// is what turns every local-search move from O(route length) into O(1): a candidate
// route is a concatenation of a few precomputed prefix/suffix segments + the moved
// nodes. Distance and load are tracked separately (both O(1) via deltas).
const Tws = struct {
    dur: i64, // duration spanned (travel + service + forced waiting)
    tw: i64, // accumulated time warp (infeasibility); 0 == feasible
    early: i64, // earliest feasible start at the segment's first node
    late: i64, // latest feasible start at the segment's first node

    fn client(inst: VrptwInstance, c: usize) Tws {
        return .{ .dur = @intCast(inst.service[c]), .tw = 0, .early = @intCast(inst.ready[c]), .late = @intCast(inst.due[c]) };
    }
    fn depotNode(inst: VrptwInstance) Tws {
        return .{ .dur = 0, .tw = 0, .early = 0, .late = @intCast(inst.due[0]) };
    }
    /// Merge `left` then `right`, connected by an edge of travel time `edge`.
    fn merge(left: Tws, edge: i64, right: Tws) Tws {
        const delta = left.dur - left.tw + edge;
        const d_wait = @max(right.early - delta - left.late, 0);
        const d_tw = @max(left.early + delta - right.late, 0);
        return .{
            .dur = left.dur + right.dur + edge + d_wait,
            .tw = left.tw + right.tw + d_tw,
            .early = @max(right.early - delta, left.early) - d_wait,
            .late = @min(right.late - delta, left.late) + d_tw,
        };
    }
};

/// Whole-route TWS (depot -> nodes -> depot). tw == 0 iff time-window feasible.
fn routeTws(inst: VrptwInstance, nodes: []const usize) Tws {
    var acc = Tws.depotNode(inst);
    var prev: usize = 0;
    for (nodes) |c| {
        acc = Tws.merge(acc, @intCast(inst.d(prev, c)), Tws.client(inst, c));
        prev = c;
    }
    acc = Tws.merge(acc, @intCast(inst.d(prev, 0)), Tws.depotNode(inst));
    return acc;
}

pub const VrptwResult = struct {
    allocator: std.mem.Allocator,
    routes: [][]usize,
    total_cost: u64, // total travel distance
    vehicles: usize,

    pub fn deinit(self: *VrptwResult) void {
        for (self.routes) |r| self.allocator.free(r);
        self.allocator.free(self.routes);
        self.* = undefined;
    }
};

/// Prins Split for VRPTW: optimal min-distance partition of giant[0..n] into
/// contiguous TW- and capacity-feasible routes. `veh_penalty` is added per route
/// to bias toward fewer vehicles (0 = pure distance). Returns pred chain.
const SplitTw = struct { cost: u64, pred: []usize };
fn splitDpTw(allocator: std.mem.Allocator, inst: VrptwInstance, giant: []const usize, veh_penalty: u64) !SplitTw {
    const n = inst.n;
    const INF = std.math.maxInt(u64);
    const p = try allocator.alloc(u64, n + 1);
    defer allocator.free(p);
    const pred = try allocator.alloc(usize, n + 1);
    errdefer allocator.free(pred);
    @memset(p, INF);
    p[0] = 0;
    pred[0] = 0;

    for (0..n) |i| {
        if (p[i] == INF) continue;
        // extend a route starting at giant[i], scheduling incrementally.
        var load: u64 = 0;
        var t: u64 = 0;
        var prev: usize = 0;
        var dist: u64 = 0;
        var j = i;
        while (j < n) : (j += 1) {
            const c = giant[j];
            const tr = inst.d(prev, c);
            load += inst.demand[c];
            if (load > inst.capacity) break; // capacity monotone -> stop
            const arrive = t + tr;
            const start = @max(arrive, inst.ready[c]);
            if (start > inst.due[c]) break; // TW monotone for a fixed prefix -> stop
            dist += tr;
            t = start + inst.service[c];
            prev = c;
            const back = inst.d(c, 0);
            if (t + back <= inst.due[0]) { // route [i..j] closes feasibly
                const cand = p[i] + dist + back + veh_penalty;
                if (cand < p[j + 1]) {
                    p[j + 1] = cand;
                    pred[j + 1] = i;
                }
            }
            // else: this last node returns late, but a longer route ends elsewhere
            // and may close in time -> keep extending (do not break).
        }
    }
    // No TW- and capacity-feasible split of this order exists (a customer whose
    // window is unreachable even as a singleton route). Return a clean error
    // instead of a maxInt cost over an uninitialized pred chain that
    // rebuildFromGiant would walk into OOB / a non-terminating loop.
    if (p[n] == INF) return error.NoFeasibleSplit;
    return .{ .cost = p[n], .pred = pred };
}

/// Tuning for `solveVrptw` (giant-tour ILS). `rounds` is the perturbation steps
/// per chain; `restarts` is the number of independent chains; `veh_penalty` is the
/// per-route penalty added in Split to bias toward fewer vehicles (0 = pure
/// distance). One named field per knob so the two same-typed `usize` values can't
/// be silently transposed.
pub const VrptwParams = struct {
    rounds: usize = 100,
    restarts: usize = 10,
    veh_penalty: u64 = 0,
};

/// Solve VRPTW: giant tour over customers (asymmetric TSP core) -> TW Split ->
/// route local search (relocate/or-opt/2-opt/swap, all TW+capacity gated) ->
/// multi-start ILS (double-bridge on the giant order + re-Split). Minimizes total
/// distance; reports vehicle count. `params.veh_penalty` biases Split toward fewer routes.
pub fn solveVrptw(allocator: std.mem.Allocator, inst: VrptwInstance, options: solver.SolveOptions, params: VrptwParams) !VrptwResult {
    const rounds = params.rounds;
    const restarts = params.restarts;
    const veh_penalty = params.veh_penalty;
    const n = inst.n;
    if (inst.demand.len != n + 1 or inst.matrix.len != (std.math.mul(usize, n + 1, n + 1) catch return error.InvalidInstance)) return error.InvalidInstance;
    if (inst.ready.len != n + 1 or inst.due.len != n + 1 or inst.service.len != n + 1) return error.InvalidInstance;

    // Giant tour: ATSP over the customer sub-matrix (1..n).
    const sub = try allocator.alloc(u32, n * n);
    defer allocator.free(sub);
    for (0..n) |a| {
        for (0..n) |b| sub[a * n + b] = inst.matrix[(a + 1) * (n + 1) + (b + 1)];
    }
    var atsp = try asymmetric.solveAtsp(allocator, sub, n, options);
    defer atsp.deinit();
    const giant = try allocator.alloc(usize, n);
    defer allocator.free(giant);
    for (atsp.tour, 0..) |c, idx| giant[idx] = c + 1;

    const gk: usize = @min(@as(usize, 20), if (n > 1) n - 1 else 0);
    const gran = try buildNeighbors(allocator, inst, gk);
    defer allocator.free(gran);

    var sol = try Solution.fromGiant(allocator, inst, giant, veh_penalty, gran, gk);
    defer sol.deinit();
    try sol.localSearch();

    var base = try sol.clone();
    defer base.deinit();
    var best = try sol.clone();
    defer best.deinit();
    var inc = try sol.clone();
    defer inc.deinit();

    var chain: usize = 0;
    while (chain < restarts) : (chain += 1) {
        try sol.copyFrom(base);
        try inc.copyFrom(base);
        var prng = std.Random.DefaultPrng.init(options.seed +% chain *% 0x9E3779B97F4A7C15);
        const rng = prng.random();
        var round: usize = 0;
        while (round < rounds) : (round += 1) {
            try sol.perturb(rng);
            try sol.localSearch();
            if (sol.cost < inc.cost) {
                try inc.copyFrom(sol);
            } else {
                try sol.copyFrom(inc);
            }
        }
        if (inc.cost < best.cost) try best.copyFrom(inc);
    }
    var result = try best.toResult(allocator);
    errdefer result.deinit();
    if (validate(inst, result.routes) == null) return error.Infeasible;
    return result;
}

const Route = std.ArrayList(usize);

/// k-nearest-neighbour lists (granular search): gran[(c-1)*k + i] is the i-th
/// closest customer to c by round-trip proximity d(c,j)+d(j,c), padded with 0.
/// Restricting moves to these pairs is the other half of fast HGS — it cuts the
/// per-sweep candidate count from O(routes^2 * len^2) to O(n * k). Built ONCE per
/// instance (shared, read-only across the whole population).
fn buildNeighbors(allocator: std.mem.Allocator, inst: VrptwInstance, k: usize) ![]usize {
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
                return key[a] < key[b];
            }
        }.lt);
        for (0..kk) |i| gran[(c - 1) * k + i] = idx[i];
    }
    return gran;
}

/// Copy `src` minus element `pos` into `dst`; return the filled slice.
fn removeInto(dst: []usize, src: []const usize, pos: usize) []usize {
    @memcpy(dst[0..pos], src[0..pos]);
    @memcpy(dst[pos .. src.len - 1], src[pos + 1 ..]);
    return dst[0 .. src.len - 1];
}

/// Copy `src` with `city` inserted at gap `pos` into `dst`; return the slice.
fn insertInto(dst: []usize, src: []const usize, pos: usize, city: usize) []usize {
    @memcpy(dst[0..pos], src[0..pos]);
    dst[pos] = city;
    @memcpy(dst[pos + 1 .. src.len + 1], src[pos..]);
    return dst[0 .. src.len + 1];
}

// Solution = a list of routes plus a per-route cached distance. Cost = total
// distance + veh_penalty*nroutes (so emptying a route is rewarded when biased).
// Every move is evaluated by re-scheduling the affected candidate route(s) with
// scheduleSlice, which returns exact distance and feasibility together — no delta
// formulas, so directional costs and time windows are always handled correctly.
const Solution = struct {
    allocator: std.mem.Allocator,
    inst: VrptwInstance,
    routes: std.ArrayList(Route),
    rdist: std.ArrayList(u64),
    scratch: std.ArrayList(usize),
    giant_buf: []usize,
    veh_penalty: u64,
    cost: u64,
    swap_star: bool = false, // enable the (costly) SWAP* move — used by HGS education
    granular: bool = false, // restrict inter-route moves to k-nearest neighbours (HGS); ILS keeps the full neighbourhood
    // prefix/suffix TWS scratch (sized n+2) for O(1) move feasibility
    pa: []Tws,
    sa: []Tws,
    pb: []Tws,
    sb: []Tws,
    la: []u32, // prefix loads of route A: la[i] = sum demand of A[0..i]
    lb: []u32, // prefix loads of route B
    // reusable per-Solution move-copy scratch (sized n+2): every local-search
    // route copy goes through these instead of fixed-size stack buffers, so an
    // instance with a route longer than the old [512]/[4096] limits can't smash
    // the stack. Up to four are simultaneously live (trySwapStar and the granular
    // SWAP* block hold amu+bmv+abuf+bbuf at once).
    mbuf0: []usize,
    mbuf1: []usize,
    mbuf2: []usize,
    mbuf3: []usize,
    gran: []const usize, // shared k-nearest neighbour lists (not owned)
    gk: usize, // neighbours per customer in `gran`
    loc_route: []usize, // loc_route[c] = route index currently holding customer c
    loc_pos: []usize, // loc_pos[c] = position of c within that route

    fn fromGiant(allocator: std.mem.Allocator, inst: VrptwInstance, giant: []const usize, veh_penalty: u64, gran: []const usize, gk: usize) !Solution {
        var s = Solution{
            .allocator = allocator,
            .inst = inst,
            .routes = .empty,
            .rdist = .empty,
            .scratch = .empty,
            .giant_buf = try allocator.alloc(usize, inst.n),
            .veh_penalty = veh_penalty,
            .cost = 0,
            .pa = try allocator.alloc(Tws, inst.n + 2),
            .sa = try allocator.alloc(Tws, inst.n + 2),
            .pb = try allocator.alloc(Tws, inst.n + 2),
            .sb = try allocator.alloc(Tws, inst.n + 2),
            .la = try allocator.alloc(u32, inst.n + 2),
            .lb = try allocator.alloc(u32, inst.n + 2),
            .mbuf0 = try allocator.alloc(usize, inst.n + 2),
            .mbuf1 = try allocator.alloc(usize, inst.n + 2),
            .mbuf2 = try allocator.alloc(usize, inst.n + 2),
            .mbuf3 = try allocator.alloc(usize, inst.n + 2),
            .gran = gran,
            .gk = gk,
            .loc_route = try allocator.alloc(usize, inst.n + 1),
            .loc_pos = try allocator.alloc(usize, inst.n + 1),
        };
        // rebuildFromGiant can fail (no TW-feasible split); free the just-built
        // Solution on that path instead of leaking its buffers (ZIG-3).
        errdefer s.deinit();
        try s.rebuildFromGiant(giant);
        return s;
    }

    fn deinit(self: *Solution) void {
        for (self.routes.items) |*r| r.deinit(self.allocator);
        self.routes.deinit(self.allocator);
        self.rdist.deinit(self.allocator);
        self.scratch.deinit(self.allocator);
        self.allocator.free(self.giant_buf);
        self.allocator.free(self.pa);
        self.allocator.free(self.sa);
        self.allocator.free(self.pb);
        self.allocator.free(self.sb);
        self.allocator.free(self.la);
        self.allocator.free(self.lb);
        self.allocator.free(self.mbuf0);
        self.allocator.free(self.mbuf1);
        self.allocator.free(self.mbuf2);
        self.allocator.free(self.mbuf3);
        self.allocator.free(self.loc_route);
        self.allocator.free(self.loc_pos);
        self.* = undefined;
    }

    fn clearRoutes(self: *Solution) void {
        for (self.routes.items) |*r| r.deinit(self.allocator);
        self.routes.clearRetainingCapacity();
        self.rdist.clearRetainingCapacity();
    }

    /// Split `giant` into TW-feasible routes and install them.
    fn rebuildFromGiant(self: *Solution, giant: []const usize) !void {
        const sp = try splitDpTw(self.allocator, self.inst, giant, self.veh_penalty);
        defer self.allocator.free(sp.pred);
        self.clearRoutes();
        // Reconstruct routes from the pred chain: collect (start,end) pairs by
        // walking back from n, then emit in forward order.
        var pairs: std.ArrayList([2]usize) = .empty;
        defer pairs.deinit(self.allocator);
        var i = self.inst.n;
        while (i > 0) {
            const st = sp.pred[i];
            try pairs.append(self.allocator, .{ st, i });
            i = st;
        }
        var idx = pairs.items.len;
        while (idx > 0) {
            idx -= 1;
            const st = pairs.items[idx][0];
            const en = pairs.items[idx][1];
            var route: Route = .empty;
            try route.appendSlice(self.allocator, giant[st..en]);
            try self.routes.append(self.allocator, route);
            try self.rdist.append(self.allocator, 0);
        }
        self.recompute();
    }

    fn recompute(self: *Solution) void {
        var total: u64 = 0;
        for (self.routes.items, 0..) |r, ri| {
            const dist = scheduleSlice(self.inst, r.items) orelse unreachable;
            self.rdist.items[ri] = dist;
            total += dist;
        }
        self.cost = total + self.veh_penalty * self.routes.items.len;
    }

    /// Recompute `cost` from the cached per-route distances — O(routes), not O(n).
    /// Callers that touched a route must refresh its rdist first (scheduleRoute).
    fn finalizeCost(self: *Solution) void {
        var total: u64 = 0;
        for (self.rdist.items) |d| total += d;
        self.cost = total + self.veh_penalty * self.routes.items.len;
    }
    fn scheduleRoute(self: *Solution, r: usize) void {
        self.rdist.items[r] = scheduleSlice(self.inst, self.routes.items[r].items) orelse unreachable;
    }
    /// Re-index the customer->position map for a single route. O(route len).
    fn updateLocForRoute(self: *Solution, r: usize) void {
        for (self.routes.items[r].items, 0..) |c, p| {
            self.loc_route[c] = r;
            self.loc_pos[c] = p;
        }
    }

    fn dropEmptyRoutes(self: *Solution) void {
        var w: usize = 0;
        for (self.routes.items, 0..) |r, ri| {
            if (r.items.len == 0) {
                var rr = r;
                rr.deinit(self.allocator);
            } else {
                self.routes.items[w] = self.routes.items[ri];
                self.rdist.items[w] = self.rdist.items[ri];
                w += 1;
            }
        }
        self.routes.items.len = w;
        self.rdist.items.len = w;
    }

    fn clone(self: *const Solution) !Solution {
        var s = Solution{
            .allocator = self.allocator,
            .inst = self.inst,
            .routes = .empty,
            .rdist = .empty,
            .scratch = .empty,
            .giant_buf = try self.allocator.alloc(usize, self.inst.n),
            .veh_penalty = self.veh_penalty,
            .cost = self.cost,
            .pa = try self.allocator.alloc(Tws, self.inst.n + 2),
            .sa = try self.allocator.alloc(Tws, self.inst.n + 2),
            .pb = try self.allocator.alloc(Tws, self.inst.n + 2),
            .sb = try self.allocator.alloc(Tws, self.inst.n + 2),
            .la = try self.allocator.alloc(u32, self.inst.n + 2),
            .lb = try self.allocator.alloc(u32, self.inst.n + 2),
            .mbuf0 = try self.allocator.alloc(usize, self.inst.n + 2),
            .mbuf1 = try self.allocator.alloc(usize, self.inst.n + 2),
            .mbuf2 = try self.allocator.alloc(usize, self.inst.n + 2),
            .mbuf3 = try self.allocator.alloc(usize, self.inst.n + 2),
            .gran = self.gran,
            .gk = self.gk,
            .loc_route = try self.allocator.alloc(usize, self.inst.n + 1),
            .loc_pos = try self.allocator.alloc(usize, self.inst.n + 1),
        };
        for (self.routes.items) |r| {
            var nr: Route = .empty;
            try nr.appendSlice(self.allocator, r.items);
            try s.routes.append(self.allocator, nr);
        }
        try s.rdist.appendSlice(self.allocator, self.rdist.items);
        return s;
    }

    fn copyFrom(self: *Solution, o: Solution) !void {
        // resize routes list to match o
        while (self.routes.items.len > o.routes.items.len) {
            var r = self.routes.pop().?;
            r.deinit(self.allocator);
            _ = self.rdist.pop();
        }
        while (self.routes.items.len < o.routes.items.len) {
            try self.routes.append(self.allocator, .empty);
            try self.rdist.append(self.allocator, 0);
        }
        for (o.routes.items, 0..) |r, ri| {
            self.routes.items[ri].clearRetainingCapacity();
            try self.routes.items[ri].appendSlice(self.allocator, r.items);
            self.rdist.items[ri] = o.rdist.items[ri];
        }
        self.cost = o.cost;
    }

    fn flattenInto(self: *const Solution, out: []usize) void {
        var k: usize = 0;
        for (self.routes.items) |r| {
            for (r.items) |c| {
                out[k] = c;
                k += 1;
            }
        }
    }

    /// Local search to a local optimum over relocate / or-opt(2,3) / 2-opt(intra)
    /// / swap, first-improvement. Each candidate is scored by re-scheduling the
    /// affected route(s); moves are accepted only if feasible and strictly better.
    fn localSearch(self: *Solution) !void {
        var improved = true;
        while (improved) {
            improved = false;
            if (self.granular) {
                // Granular: inter-route moves restricted to k-nearest neighbours
                // (fast — for HGS education, where the population recovers what the
                // restriction loses). Plus full intra-route moves (routes are short).
                if (try self.granularMoves()) {
                    improved = true;
                    continue;
                }
                if (try self.tryTwoOpt()) {
                    improved = true;
                    continue;
                }
                if (try self.tryIntra()) {
                    improved = true;
                    continue;
                }
            } else {
                // Full neighbourhood (the ILS single incumbent needs the richest
                // descent; granular restriction measurably weakens it).
                if (try self.tryRelocate()) {
                    improved = true;
                    continue;
                }
                if (try self.tryOrOpt()) {
                    improved = true;
                    continue;
                }
                if (try self.tryTwoOpt()) {
                    improved = true;
                    continue;
                }
                if (try self.trySwap()) {
                    improved = true;
                    continue;
                }
                if (try self.tryTwoOptStar()) {
                    improved = true;
                    continue;
                }
                if (self.swap_star and try self.trySwapStar()) {
                    improved = true;
                    continue;
                }
            }
            // Vehicle minimization: eject a whole route's customers into the others
            // once distance moves converge (first-improvement distance moves can't
            // consolidate one customer at a time). The dominant VRPTW objective.
            if (try self.tryEliminate()) {
                improved = true;
                continue;
            }
        }
    }

    /// Cheapest TW/capacity-feasible insertion of `city` into `base` (whose own
    /// distance is `base_dist`, prefix/suffix TWS `pre`/`suf` precomputed). Insertion
    /// distance is exact and O(1) per gap (base_dist + edge delta) and feasibility is
    /// O(1) via the precomputed segments, so this is a single O(len) scan for the
    /// cheapest feasible gap — no scheduling, no sort.
    fn bestInsert(self: *Solution, base: []const usize, pre: []const Tws, suf: []const Tws, base_dist: u64, city: usize) ?struct { dist: u64, pos: usize } {
        const L = base.len;
        var best_delta: i64 = std.math.maxInt(i64);
        var best_pos: usize = 0;
        var found = false;
        var q: usize = 0;
        while (q <= L) : (q += 1) {
            const prev = if (q == 0) 0 else base[q - 1];
            const next = if (q == L) 0 else base[q];
            const delta = self.dd(prev, city) + self.dd(city, next) - self.dd(prev, next);
            if (delta >= best_delta) continue; // can't beat the current best gap
            if (!self.insertOneFeas(base, pre, suf, q, city)) continue;
            best_delta = delta;
            best_pos = q;
            found = true;
        }
        if (!found) return null;
        return .{ .dist = @intCast(@as(i64, @intCast(base_dist)) + best_delta), .pos = best_pos };
    }

    /// SWAP*: exchange a customer u (route A) with v (route B), but RE-INSERT each
    /// at its best feasible position in the other route rather than the swapped
    /// slot. This is the neighbourhood that made HGS state of the art — it reaches
    /// improving exchanges that position-preserving swap misses.
    fn trySwapStar(self: *Solution) !bool {
        const nr = self.routes.items.len;
        var ri: usize = 0;
        while (ri < nr) : (ri += 1) {
            const loadA = self.routeLoad(ri);
            var rj: usize = ri + 1;
            while (rj < nr) : (rj += 1) {
                const loadB = self.routeLoad(rj);
                const lenA = self.routes.items[ri].items.len;
                const lenB = self.routes.items[rj].items.len;
                const A = self.routes.items[ri].items;
                const B = self.routes.items[rj].items;
                var pu: usize = 0;
                while (pu < lenA) : (pu += 1) {
                    const u = A[pu];
                    // dist(A\u) = rdist[ri] - removal delta of u, exact and O(1)
                    const au = if (pu == 0) 0 else A[pu - 1];
                    const bu = if (pu + 1 == lenA) 0 else A[pu + 1];
                    const base_a: u64 = @intCast(@as(i64, @intCast(self.rdist.items[ri])) - (self.dd(au, u) + self.dd(u, bu) - self.dd(au, bu)));
                    const am = removeInto(self.mbuf0, A, pu);
                    self.fillPreSuf(am, self.pa, self.sa); // A\u prefix/suffix, once per pu
                    var pv: usize = 0;
                    while (pv < lenB) : (pv += 1) {
                        const v = B[pv];
                        if (loadA - self.inst.demand[u] + self.inst.demand[v] > self.inst.capacity) continue;
                        if (loadB - self.inst.demand[v] + self.inst.demand[u] > self.inst.capacity) continue;
                        const av = if (pv == 0) 0 else B[pv - 1];
                        const bv = if (pv + 1 == lenB) 0 else B[pv + 1];
                        const base_b: u64 = @intCast(@as(i64, @intCast(self.rdist.items[rj])) - (self.dd(av, v) + self.dd(v, bv) - self.dd(av, bv)));
                        const bm = removeInto(self.mbuf1, B, pv);
                        self.fillPreSuf(bm, self.pb, self.sb); // B\v prefix/suffix
                        const ia = self.bestInsert(am, self.pa, self.sa, base_a, v) orelse continue; // v into A\u
                        const ib = self.bestInsert(bm, self.pb, self.sb, base_b, u) orelse continue; // u into B\v
                        const old = self.rdist.items[ri] + self.rdist.items[rj];
                        if (ia.dist + ib.dist < old) {
                            // materialize A' = am with v at ia.pos, B' = bm with u at ib.pos
                            const na = insertInto(self.mbuf2, am, ia.pos, v);
                            const nb = insertInto(self.mbuf3, bm, ib.pos, u);
                            try self.setRoute(ri, na);
                            try self.setRoute(rj, nb);
                            self.recompute();
                            return true;
                        }
                    }
                }
            }
        }
        return false;
    }

    /// Inter-route 2-opt*: exchange the tails of two routes (A = headA|tailA,
    /// B = headB|tailB -> A' = headA|tailB, B' = headB|tailA). Reversal-free, so
    /// asymmetric-safe, and moves a chunk of customers between routes in one move
    /// — repartitioning the customer-to-vehicle assignment that single relocate/
    /// swap can't reach. Gated by the exact O(1) endpoint-edge delta.
    fn tryTwoOptStar(self: *Solution) !bool {
        const nr = self.routes.items.len;
        var ri: usize = 0;
        while (ri < nr) : (ri += 1) {
            const A = self.routes.items[ri].items;
            const lenA = A.len;
            self.fillPreSuf(A, self.pa, self.sa);
            self.fillPrefixLoad(A, self.la);
            const totalA = self.la[lenA];
            var rj: usize = ri + 1;
            while (rj < nr) : (rj += 1) {
                const B = self.routes.items[rj].items;
                const lenB = B.len;
                self.fillPreSuf(B, self.pb, self.sb);
                self.fillPrefixLoad(B, self.lb);
                const totalB = self.lb[lenB];
                var pA: usize = 0;
                while (pA <= lenA) : (pA += 1) {
                    const last_head_a = if (pA == 0) 0 else A[pA - 1];
                    const first_tail_a = if (pA == lenA) 0 else A[pA];
                    var pB: usize = 0;
                    while (pB <= lenB) : (pB += 1) {
                        // skip the two no-ops (keep both tails / swap whole routes)
                        if (pA == lenA and pB == lenB) continue;
                        if (pA == 0 and pB == 0) continue;
                        const last_head_b = if (pB == 0) 0 else B[pB - 1];
                        const first_tail_b = if (pB == lenB) 0 else B[pB];
                        const delta: i64 = self.dd(last_head_a, first_tail_b) + self.dd(last_head_b, first_tail_a) -
                            self.dd(last_head_a, first_tail_a) - self.dd(last_head_b, first_tail_b);
                        const newa_empty = pA == 0 and pB == lenB; // A' = headA(empty)|tailB(empty)
                        const newb_empty = pB == 0 and pA == lenA;
                        const empties = newa_empty or newb_empty;
                        if (!empties and delta >= 0) continue; // O(1) distance prune
                        // capacity: A' = headA|tailB, B' = headB|tailA
                        const loadA2 = self.la[pA] + (totalB - self.lb[pB]);
                        const loadB2 = self.lb[pB] + (totalA - self.la[pA]);
                        if (loadA2 > self.inst.capacity or loadB2 > self.inst.capacity) continue;
                        // O(1) TW feasibility via prefix(head) ⊕ suffix(tail)
                        const fa = Tws.merge(self.pa[pA], @intCast(self.inst.d(last_head_a, first_tail_b)), self.sb[pB]);
                        if (fa.tw != 0) continue;
                        const fb = Tws.merge(self.pb[pB], @intCast(self.inst.d(last_head_b, first_tail_a)), self.sa[pA]);
                        if (fb.tw != 0) continue;
                        const pen: i64 = if (empties) -@as(i64, @intCast(self.veh_penalty)) else 0;
                        if (delta + pen < 0) {
                            // build A' = headA ++ tailB, B' = headB ++ tailA
                            const bufA = self.mbuf0;
                            const bufB = self.mbuf1;
                            const na = pA + (lenB - pB);
                            const nb = pB + (lenA - pA);
                            @memcpy(bufA[0..pA], A[0..pA]);
                            @memcpy(bufA[pA..na], B[pB..lenB]);
                            @memcpy(bufB[0..pB], B[0..pB]);
                            @memcpy(bufB[pB..nb], A[pA..lenA]);
                            try self.setRoute(ri, bufA[0..na]);
                            try self.setRoute(rj, bufB[0..nb]);
                            self.dropEmptyRoutes();
                            self.recompute();
                            return true;
                        }
                    }
                }
            }
        }
        return false;
    }

    /// Try to empty one route by relocating every one of its customers into the
    /// other routes (best feasible insertion, greedy sequential). Tries routes
    /// smallest-first. On success the route is removed (one fewer vehicle); on
    /// failure the solution is restored. Returns true if a route was eliminated.
    fn tryEliminate(self: *Solution) !bool {
        const nr = self.routes.items.len;
        if (nr <= 1) return false;
        // order route indices by ascending customer count (easiest to empty first)
        const order = self.mbuf0[0..nr];
        for (0..nr) |i| order[i] = i;
        std.sort.pdq(usize, order, self, struct {
            fn lt(s: *Solution, a: usize, b: usize) bool {
                return s.routes.items[a].items.len < s.routes.items[b].items.len;
            }
        }.lt);

        for (order) |target| {
            if (self.routes.items[target].items.len == 0) continue;
            var snapshot = try self.clone();
            defer snapshot.deinit();
            if (try self.ejectRoute(target)) {
                self.dropEmptyRoutes();
                self.recompute();
                return true;
            }
            try self.copyFrom(snapshot); // restore and try the next route
        }
        return false;
    }

    /// Relocate every customer of route `target` into some other route via best
    /// feasible insertion. Mutates routes in place; returns false (leaving partial
    /// state for the caller to restore) if any customer has no feasible insertion.
    fn ejectRoute(self: *Solution, target: usize) !bool {
        const victims = self.mbuf1;
        const vn = self.routes.items[target].items.len;
        @memcpy(victims[0..vn], self.routes.items[target].items);
        self.routes.items[target].clearRetainingCapacity();
        self.rdist.items[target] = scheduleSlice(self.inst, &.{}) orelse 0;

        for (victims[0..vn]) |c| {
            var best_rj: usize = std.math.maxInt(usize);
            var best_q: usize = 0;
            var best_dist: u64 = std.math.maxInt(u64);
            var rj: usize = 0;
            while (rj < self.routes.items.len) : (rj += 1) {
                if (rj == target) continue;
                if (self.routes.items[rj].items.len == 0) continue;
                if (self.routeLoad(rj) + self.inst.demand[c] > self.inst.capacity) continue;
                const rjtems = self.routes.items[rj].items;
                var q: usize = 0;
                while (q <= rjtems.len) : (q += 1) {
                    const cand = try self.buildCandidate(rjtems, null, 0, q, &[_]usize{c});
                    const cd = scheduleSlice(self.inst, cand) orelse continue;
                    if (cd < best_dist) {
                        best_dist = cd;
                        best_rj = rj;
                        best_q = q;
                    }
                }
            }
            if (best_rj == std.math.maxInt(usize)) return false; // c cannot be placed
            const cand = try self.buildCandidate(self.routes.items[best_rj].items, null, 0, best_q, &[_]usize{c});
            try self.setRoute(best_rj, cand);
            self.rdist.items[best_rj] = scheduleSlice(self.inst, self.routes.items[best_rj].items) orelse unreachable;
        }
        return true;
    }

    fn routeLoad(self: *const Solution, r: usize) u32 {
        var load: u32 = 0;
        for (self.routes.items[r].items) |c| load += self.inst.demand[c];
        return load;
    }

    // Build into scratch a copy of route `src` with the block removed at [rm, rm+len)
    // (if rm != null) and `seg` inserted at gap `ins` (in the post-removal index
    // space). Returns the scratch slice.
    fn dd(self: *const Solution, a: usize, b: usize) i64 {
        return @intCast(self.inst.d(a, b));
    }

    // ---- O(1) move feasibility via prefix/suffix time-window segments ----
    // pre[i] summarizes [depot, route[0..i]); suf[i] summarizes [route[i..], depot].
    // A candidate route is a concatenation of a couple of these + the moved nodes,
    // so its TW feasibility is O(1) (merge .tw == 0) instead of an O(len) reschedule.

    fn fillPreSuf(self: *const Solution, route: []const usize, pre: []Tws, suf: []Tws) void {
        const L = route.len;
        pre[0] = Tws.depotNode(self.inst);
        var prev: usize = 0;
        for (0..L) |i| {
            pre[i + 1] = Tws.merge(pre[i], @intCast(self.inst.d(prev, route[i])), Tws.client(self.inst, route[i]));
            prev = route[i];
        }
        suf[L] = Tws.depotNode(self.inst);
        var i: usize = L;
        while (i > 0) {
            i -= 1;
            const nxt: usize = if (i + 1 == L) 0 else route[i + 1];
            suf[i] = Tws.merge(Tws.client(self.inst, route[i]), @intCast(self.inst.d(route[i], nxt)), suf[i + 1]);
        }
    }

    /// prefix loads: out[i] = total demand of route[0..i] (out has len+1 entries).
    fn fillPrefixLoad(self: *const Solution, route: []const usize, out: []u32) void {
        out[0] = 0;
        for (0..route.len) |i| out[i + 1] = out[i] + self.inst.demand[route[i]];
    }

    /// TWS of the segment route[p..p+len] standing alone (first node route[p]).
    fn segTws(self: *const Solution, route: []const usize, p: usize, len: usize) Tws {
        var acc = Tws.client(self.inst, route[p]);
        var k: usize = 1;
        while (k < len) : (k += 1) {
            acc = Tws.merge(acc, @intCast(self.inst.d(route[p + k - 1], route[p + k])), Tws.client(self.inst, route[p + k]));
        }
        return acc;
    }

    /// Feasible to insert single node `v` into `route` at gap q? (pre/suf precomputed)
    fn insertOneFeas(self: *const Solution, route: []const usize, pre: []const Tws, suf: []const Tws, q: usize, v: usize) bool {
        const c: usize = if (q == 0) 0 else route[q - 1];
        const e: usize = if (q == route.len) 0 else route[q];
        const m1 = Tws.merge(pre[q], @intCast(self.inst.d(c, v)), Tws.client(self.inst, v));
        const m2 = Tws.merge(m1, @intCast(self.inst.d(v, e)), suf[q]);
        return m2.tw == 0;
    }

    /// Feasible to insert a precomputed segment (first/last node, seg TWS) at gap q?
    fn insertSegFeas(self: *const Solution, route: []const usize, pre: []const Tws, suf: []const Tws, q: usize, seg: Tws, first: usize, last: usize) bool {
        const c: usize = if (q == 0) 0 else route[q - 1];
        const e: usize = if (q == route.len) 0 else route[q];
        const m1 = Tws.merge(pre[q], @intCast(self.inst.d(c, first)), seg);
        const m2 = Tws.merge(m1, @intCast(self.inst.d(last, e)), suf[q]);
        return m2.tw == 0;
    }

    /// Feasible to replace the node at position p with `v`? (swap helper)
    fn replaceFeas(self: *const Solution, route: []const usize, pre: []const Tws, suf: []const Tws, p: usize, v: usize) bool {
        const a: usize = if (p == 0) 0 else route[p - 1];
        const b: usize = if (p + 1 == route.len) 0 else route[p + 1];
        const m1 = Tws.merge(pre[p], @intCast(self.inst.d(a, v)), Tws.client(self.inst, v));
        const m2 = Tws.merge(m1, @intCast(self.inst.d(v, b)), suf[p + 1]);
        return m2.tw == 0;
    }

    /// Intra-route relocate + or-opt (move a length-1..3 block to another gap in
    /// the SAME route). Routes are short, so this is evaluated in full (not
    /// neighbour-restricted) by rescheduling the candidate. Complements the
    /// granular inter-route pass, which skips same-route pairs.
    fn tryIntra(self: *Solution) !bool {
        const nr = self.routes.items.len;
        var r: usize = 0;
        while (r < nr) : (r += 1) {
            const items = self.routes.items[r].items;
            const len_r = items.len;
            for ([_]usize{ 1, 2, 3 }) |L| {
                if (len_r < L + 1) continue;
                var pi: usize = 0;
                while (pi + L <= len_r) : (pi += 1) {
                    var segbuf: [3]usize = undefined;
                    @memcpy(segbuf[0..L], items[pi .. pi + L]);
                    const seg = segbuf[0..L];
                    var q: usize = 0;
                    while (q <= len_r) : (q += 1) {
                        if (q >= pi and q <= pi + L) continue; // inside/adjacent to block
                        const ins = if (q > pi) q - L else q;
                        const cand = try self.buildCandidate(self.routes.items[r].items, pi, L, ins, seg);
                        const cd = scheduleSlice(self.inst, cand) orelse continue;
                        if (cd < self.rdist.items[r]) {
                            try self.applyCandidate(r, cand);
                            self.scheduleRoute(r);
                            self.finalizeCost();
                            return true;
                        }
                    }
                }
            }
        }
        return false;
    }

    /// Rebuild the customer->(route,pos) index from the current routes. O(n).
    fn rebuildLoc(self: *Solution) void {
        for (self.routes.items, 0..) |r, ri| {
            for (r.items, 0..) |c, p| {
                self.loc_route[c] = ri;
                self.loc_pos[c] = p;
            }
        }
    }

    /// Move block route[ri][pi..pi+len] into route rj at gap `gap` (rj != ri).
    fn applyBlockMove(self: *Solution, ri: usize, pi: usize, len: usize, rj: usize, gap: usize) !void {
        var segbuf: [3]usize = undefined;
        @memcpy(segbuf[0..len], self.routes.items[ri].items[pi .. pi + len]);
        const B = self.routes.items[rj].items;
        const bbuf = self.mbuf0;
        @memcpy(bbuf[0..gap], B[0..gap]);
        @memcpy(bbuf[gap .. gap + len], segbuf[0..len]);
        @memcpy(bbuf[gap + len .. B.len + len], B[gap..]);
        const A = self.routes.items[ri].items;
        const abuf = self.mbuf1;
        @memcpy(abuf[0..pi], A[0..pi]);
        @memcpy(abuf[pi .. A.len - len], A[pi + len ..]);
        try self.setRoute(rj, bbuf[0 .. B.len + len]);
        try self.setRoute(ri, abuf[0 .. A.len - len]);
    }

    /// 2-opt* tail exchange: cut ri at pA, rj at pB; swap the tails.
    fn applyTwoOptStar(self: *Solution, ri: usize, pA: usize, rj: usize, pB: usize) !void {
        const A = self.routes.items[ri].items;
        const B = self.routes.items[rj].items;
        const lenA = A.len;
        const lenB = B.len;
        const bufA = self.mbuf0;
        const bufB = self.mbuf1;
        const na = pA + (lenB - pB);
        const nb = pB + (lenA - pA);
        @memcpy(bufA[0..pA], A[0..pA]);
        @memcpy(bufA[pA..na], B[pB..lenB]);
        @memcpy(bufB[0..pB], B[0..pB]);
        @memcpy(bufB[pB..nb], A[pA..lenA]);
        try self.setRoute(ri, bufA[0..na]);
        try self.setRoute(rj, bufB[0..nb]);
    }

    /// Post-move bookkeeping: refresh the two touched routes' cached distances and
    /// position index incrementally (O(route len)), drop emptied routes, and update
    /// `cost` from the cache (O(routes)). Avoids the O(n) full recompute/rebuild per
    /// accepted move — this is what makes the granular sweep fast at scale.
    fn afterMove(self: *Solution, ri: usize, rj: usize, may_drop: bool) void {
        self.scheduleRoute(ri);
        self.scheduleRoute(rj);
        if (may_drop) {
            const nr0 = self.routes.items.len;
            self.dropEmptyRoutes();
            if (self.routes.items.len != nr0) {
                self.rebuildLoc();
            } else {
                self.updateLocForRoute(ri);
                self.updateLocForRoute(rj);
            }
        } else {
            self.updateLocForRoute(ri);
            self.updateLocForRoute(rj);
        }
        self.finalizeCost();
    }

    /// Granular local-search SWEEP: for every customer u and each of its k nearest
    /// neighbours v in a DIFFERENT route, evaluate the inter-route moves that bring
    /// u and v together — relocate/or-opt (len 1-3), swap, 2-opt* creating edge
    /// (u,v), and SWAP*. Every candidate is O(1) (distance delta + TWS feasibility).
    /// Applies improvements as it goes (first-improvement per u, then next u — NOT a
    /// restart) with incremental updates; returns whether anything improved.
    fn granularMoves(self: *Solution) !bool {
        const n = self.inst.n;
        const cap = self.inst.capacity;
        self.rebuildLoc();
        var any = false;
        outer: while (true) {
        var u: usize = 1;
        while (u <= n) : (u += 1) {
            const ri = self.loc_route[u];
            const A = self.routes.items[ri].items;
            const lenA = A.len;
            const pi = self.loc_pos[u];
            self.fillPreSuf(A, self.pa, self.sa);
            self.fillPrefixLoad(A, self.la);
            const totalA = self.la[lenA];
            const au = if (pi == 0) 0 else A[pi - 1];
            const cu = if (pi + 1 == lenA) 0 else A[pi + 1];
            var gi: usize = 0;
            while (gi < self.gk) : (gi += 1) {
                const v = self.gran[(u - 1) * self.gk + gi];
                if (v == 0) continue;
                const rj = self.loc_route[v];
                if (rj == ri) continue; // inter-route only
                const B = self.routes.items[rj].items;
                const lenB = B.len;
                const pj = self.loc_pos[v];
                self.fillPreSuf(B, self.pb, self.sb);
                self.fillPrefixLoad(B, self.lb);
                const totalB = self.lb[lenB];
                const av = if (pj == 0) 0 else B[pj - 1];
                const cv = if (pj + 1 == lenB) 0 else B[pj + 1];

                // relocate / or-opt: block A[pi..pi+L] next to v in rj
                for ([_]usize{ 1, 2, 3 }) |L| {
                    if (pi + L > lenA) break;
                    const seg_first = A[pi];
                    const seg_last = A[pi + L - 1];
                    const after_seg = if (pi + L == lenA) 0 else A[pi + L];
                    const removal: i64 = self.dd(au, seg_first) + self.dd(seg_last, after_seg) - self.dd(au, after_seg);
                    var seg_load: u32 = 0;
                    for (A[pi .. pi + L]) |c| seg_load += self.inst.demand[c];
                    if (totalB + seg_load > cap) continue;
                    const seg = self.segTws(A, pi, L);
                    const ins_before: i64 = self.dd(av, seg_first) + self.dd(seg_last, v) - self.dd(av, v);
                    if (ins_before < removal and self.insertSegFeas(B, self.pb, self.sb, pj, seg, seg_first, seg_last)) {
                        try self.applyBlockMove(ri, pi, L, rj, pj);
                        self.afterMove(ri, rj, true);
                        any = true;
                        continue :outer;
                    }
                    const ins_after: i64 = self.dd(v, seg_first) + self.dd(seg_last, cv) - self.dd(v, cv);
                    if (ins_after < removal and self.insertSegFeas(B, self.pb, self.sb, pj + 1, seg, seg_first, seg_last)) {
                        try self.applyBlockMove(ri, pi, L, rj, pj + 1);
                        self.afterMove(ri, rj, true);
                        any = true;
                        continue :outer;
                    }
                }

                // swap u and v
                if (totalA - self.inst.demand[u] + self.inst.demand[v] <= cap and
                    totalB - self.inst.demand[v] + self.inst.demand[u] <= cap)
                {
                    const delta = self.dd(au, v) + self.dd(v, cu) - self.dd(au, u) - self.dd(u, cu) +
                        self.dd(av, u) + self.dd(u, cv) - self.dd(av, v) - self.dd(v, cv);
                    if (delta < 0 and self.replaceFeas(A, self.pa, self.sa, pi, v) and self.replaceFeas(B, self.pb, self.sb, pj, u)) {
                        self.routes.items[ri].items[pi] = v;
                        self.routes.items[rj].items[pj] = u;
                        self.afterMove(ri, rj, false);
                        any = true;
                        continue :outer;
                    }
                }

                // 2-opt* creating edge (u, v): cut after u in ri, before v in rj
                {
                    const pA = pi + 1;
                    const pB = pj;
                    const delta: i64 = self.dd(u, v) + self.dd(av, cu) - self.dd(u, cu) - self.dd(av, v);
                    const loadA2 = self.la[pA] + (totalB - self.lb[pB]);
                    const loadB2 = self.lb[pB] + (totalA - self.la[pA]);
                    if (delta < 0 and loadA2 <= cap and loadB2 <= cap) {
                        const fa = Tws.merge(self.pa[pA], @intCast(self.inst.d(u, v)), self.sb[pB]);
                        const fb = Tws.merge(self.pb[pB], @intCast(self.inst.d(av, cu)), self.sa[pA]);
                        if (fa.tw == 0 and fb.tw == 0) {
                            try self.applyTwoOptStar(ri, pA, rj, pB);
                            self.afterMove(ri, rj, true);
                        any = true;
                        continue :outer;
                        }
                    }
                }

                // SWAP*: exchange u and v, each reinserted at its best position
                if (self.swap_star and
                    totalA - self.inst.demand[u] + self.inst.demand[v] <= cap and
                    totalB - self.inst.demand[v] + self.inst.demand[u] <= cap)
                {
                    const am = removeInto(self.mbuf0, A, pi);
                    const bm = removeInto(self.mbuf1, B, pj);
                    const base_a: u64 = @intCast(@as(i64, @intCast(self.rdist.items[ri])) - (self.dd(au, u) + self.dd(u, cu) - self.dd(au, cu)));
                    const base_b: u64 = @intCast(@as(i64, @intCast(self.rdist.items[rj])) - (self.dd(av, v) + self.dd(v, cv) - self.dd(av, cv)));
                    // use pb/sb scratch for both bases so A's pa/sa stays valid for the next neighbour
                    self.fillPreSuf(am, self.pb, self.sb);
                    const ia = self.bestInsert(am, self.pb, self.sb, base_a, v);
                    if (ia) |ria| {
                        self.fillPreSuf(bm, self.pb, self.sb);
                        const ib = self.bestInsert(bm, self.pb, self.sb, base_b, u);
                        if (ib) |rib| {
                            if (ria.dist + rib.dist < self.rdist.items[ri] + self.rdist.items[rj]) {
                                const na = insertInto(self.mbuf2, am, ria.pos, v);
                                const nb = insertInto(self.mbuf3, bm, rib.pos, u);
                                try self.setRoute(ri, na);
                                try self.setRoute(rj, nb);
                                self.afterMove(ri, rj, false);
                        any = true;
                        continue :outer;
                            }
                        }
                    }
                }
            }
        }
        break;
        }
        return any;
    }

    fn buildCandidate(self: *Solution, src: []const usize, rm: ?usize, len: usize, ins: usize, seg: []const usize) ![]usize {
        self.scratch.clearRetainingCapacity();
        // first materialize src minus the removed block
        if (rm) |r| {
            try self.scratch.appendSlice(self.allocator, src[0..r]);
            try self.scratch.appendSlice(self.allocator, src[r + len ..]);
        } else {
            try self.scratch.appendSlice(self.allocator, src);
        }
        // now insert seg at `ins`
        try self.scratch.insertSlice(self.allocator, ins, seg);
        return self.scratch.items;
    }

    fn tryRelocate(self: *Solution) !bool {
        const nr = self.routes.items.len;
        var ri: usize = 0;
        while (ri < nr) : (ri += 1) {
            const ritems = self.routes.items[ri].items;
            var pi: usize = 0;
            while (pi < ritems.len) : (pi += 1) {
                const u = ritems[pi];
                const seg = ritems[pi .. pi + 1];
                // exact O(1) distance saved by removing u from ri (a-u-b -> a-b)
                const a = if (pi == 0) 0 else ritems[pi - 1];
                const b = if (pi + 1 == ritems.len) 0 else ritems[pi + 1];
                const removal: i64 = self.dd(a, u) + self.dd(u, b) - self.dd(a, b);
                // route ri after removing u
                const ri_after = try self.buildCandidate(self.routes.items[ri].items, pi, 1, pi, &.{});
                const ri_dist = scheduleSlice(self.inst, ri_after) orelse unreachable; // removal keeps feasibility
                // need a stable copy of ri_after for two-route moves (scratch reused)
                const ri_after_buf = self.mbuf0;
                const ri_after_len = ri_after.len;
                @memcpy(ri_after_buf[0..ri_after_len], ri_after);
                var rj: usize = 0;
                while (rj < nr) : (rj += 1) {
                    const rjtems = self.routes.items[rj].items;
                    var q: usize = 0;
                    while (q <= rjtems.len) : (q += 1) {
                        if (ri == rj) {
                            // intra: move u from pi to gap q (in original index space)
                            if (q == pi or q == pi + 1) continue;
                            const ins = if (q > pi) q - 1 else q;
                            const cand = try self.buildCandidate(self.routes.items[ri].items, pi, 1, ins, seg);
                            const cd = scheduleSlice(self.inst, cand) orelse continue;
                            if (cd < self.rdist.items[ri]) {
                                try self.applyCandidate(ri, cand);
                                self.recompute();
                                return true;
                            }
                        } else {
                            // O(1) prune: skip scheduling unless this strictly cuts
                            // distance (or empties ri, a vehicle win). Removing u and
                            // inserting it change ri/rj distance by exactly removal/insert.
                            const c = if (q == 0) 0 else rjtems[q - 1];
                            const e = if (q == rjtems.len) 0 else rjtems[q];
                            const insert: i64 = self.dd(c, u) + self.dd(u, e) - self.dd(c, e);
                            const empties = ri_after_len == 0;
                            if (!empties and insert >= removal) continue;
                            const old = self.rdist.items[ri] + self.rdist.items[rj];
                            // rj with u inserted at q (schedule = TW feasibility check)
                            const cand = try self.buildCandidate(self.routes.items[rj].items, null, 0, q, seg);
                            const cd = scheduleSlice(self.inst, cand) orelse continue;
                            const new_cost = ri_dist + cd;
                            const pen: i64 = if (empties) -@as(i64, @intCast(self.veh_penalty)) else 0;
                            if (@as(i64, @intCast(new_cost)) + pen < @as(i64, @intCast(old))) {
                                // apply: set rj to cand, ri to ri_after
                                try self.setRoute(rj, cand);
                                try self.setRoute(ri, ri_after_buf[0..ri_after_len]);
                                self.dropEmptyRoutes();
                                self.recompute();
                                return true;
                            }
                        }
                    }
                }
            }
        }
        return false;
    }

    fn tryOrOpt(self: *Solution) !bool {
        const nr = self.routes.items.len;
        for ([_]usize{ 2, 3 }) |len| {
            var ri: usize = 0;
            while (ri < nr) : (ri += 1) {
                const ritems = self.routes.items[ri].items;
                if (ritems.len < len) continue;
                var pi: usize = 0;
                while (pi + len <= ritems.len) : (pi += 1) {
                    var segbuf: [3]usize = undefined;
                    @memcpy(segbuf[0..len], ritems[pi .. pi + len]);
                    const seg = segbuf[0..len];
                    const seg_first = seg[0];
                    const seg_last = seg[len - 1];
                    const a = if (pi == 0) 0 else ritems[pi - 1];
                    const b = if (pi + len == ritems.len) 0 else ritems[pi + len];
                    const removal: i64 = self.dd(a, seg_first) + self.dd(seg_last, b) - self.dd(a, b);
                    const ri_after = try self.buildCandidate(self.routes.items[ri].items, pi, len, pi, &.{});
                    const ri_dist = scheduleSlice(self.inst, ri_after) orelse unreachable;
                    const ri_after_buf = self.mbuf0;
                    const ri_after_len = ri_after.len;
                    @memcpy(ri_after_buf[0..ri_after_len], ri_after);
                    var rj: usize = 0;
                    while (rj < nr) : (rj += 1) {
                        const rjtems = self.routes.items[rj].items;
                        var q: usize = 0;
                        const qmax = if (ri == rj) ri_after_len else rjtems.len;
                        while (q <= qmax) : (q += 1) {
                            if (ri == rj) {
                                const cand = try self.buildCandidate(self.routes.items[ri].items, pi, len, q, seg);
                                const cd = scheduleSlice(self.inst, cand) orelse continue;
                                if (cd < self.rdist.items[ri]) {
                                    try self.applyCandidate(ri, cand);
                                    self.recompute();
                                    return true;
                                }
                            } else {
                                const c = if (q == 0) 0 else rjtems[q - 1];
                                const e = if (q == rjtems.len) 0 else rjtems[q];
                                const insert: i64 = self.dd(c, seg_first) + self.dd(seg_last, e) - self.dd(c, e);
                                const empties = ri_after_len == 0;
                                if (!empties and insert >= removal) continue;
                                const old = self.rdist.items[ri] + self.rdist.items[rj];
                                const cand = try self.buildCandidate(self.routes.items[rj].items, null, 0, q, seg);
                                const cd = scheduleSlice(self.inst, cand) orelse continue;
                                const pen: i64 = if (empties) -@as(i64, @intCast(self.veh_penalty)) else 0;
                                if (@as(i64, @intCast(ri_dist + cd)) + pen < @as(i64, @intCast(old))) {
                                    try self.setRoute(rj, cand);
                                    try self.setRoute(ri, ri_after_buf[0..ri_after_len]);
                                    self.dropEmptyRoutes();
                                    self.recompute();
                                    return true;
                                }
                            }
                        }
                    }
                }
            }
        }
        return false;
    }

    fn tryTwoOpt(self: *Solution) !bool {
        const nr = self.routes.items.len;
        var r: usize = 0;
        while (r < nr) : (r += 1) {
            const items = self.routes.items[r].items;
            if (items.len < 2) continue;
            var i: usize = 0;
            while (i + 1 < items.len) : (i += 1) {
                var j: usize = i + 1;
                while (j < items.len) : (j += 1) {
                    // candidate = items with [i..j] reversed
                    self.scratch.clearRetainingCapacity();
                    try self.scratch.appendSlice(self.allocator, items);
                    std.mem.reverse(usize, self.scratch.items[i .. j + 1]);
                    const cd = scheduleSlice(self.inst, self.scratch.items) orelse continue;
                    if (cd < self.rdist.items[r]) {
                        try self.applyCandidate(r, self.scratch.items);
                        self.scheduleRoute(r);
                        self.finalizeCost();
                        return true;
                    }
                }
            }
        }
        return false;
    }

    fn trySwap(self: *Solution) !bool {
        const nr = self.routes.items.len;
        var ri: usize = 0;
        while (ri < nr) : (ri += 1) {
            const A = self.routes.items[ri].items;
            const lenA = A.len;
            const loadA = self.routeLoad(ri);
            self.fillPreSuf(A, self.pa, self.sa);
            var rj: usize = ri;
            while (rj < nr) : (rj += 1) {
                if (rj == ri) {
                    // intra swap: positions interact, evaluate by full reschedule.
                    var pi: usize = 0;
                    while (pi < lenA) : (pi += 1) {
                        var pj: usize = pi + 1;
                        while (pj < lenA) : (pj += 1) {
                            self.scratch.clearRetainingCapacity();
                            try self.scratch.appendSlice(self.allocator, A);
                            self.scratch.items[pi] = A[pj];
                            self.scratch.items[pj] = A[pi];
                            const cd = scheduleSlice(self.inst, self.scratch.items) orelse continue;
                            if (cd < self.rdist.items[ri]) {
                                try self.applyCandidate(ri, self.scratch.items);
                                self.recompute();
                                return true;
                            }
                        }
                    }
                    continue;
                }
                const B = self.routes.items[rj].items;
                const lenB = B.len;
                const loadB = self.routeLoad(rj);
                self.fillPreSuf(B, self.pb, self.sb);
                var pi: usize = 0;
                while (pi < lenA) : (pi += 1) {
                    const u = A[pi];
                    const a1: usize = if (pi == 0) 0 else A[pi - 1];
                    const b1: usize = if (pi + 1 == lenA) 0 else A[pi + 1];
                    var pj: usize = 0;
                    while (pj < lenB) : (pj += 1) {
                        const v = B[pj];
                        if (loadA - self.inst.demand[u] + self.inst.demand[v] > self.inst.capacity) continue;
                        if (loadB - self.inst.demand[v] + self.inst.demand[u] > self.inst.capacity) continue;
                        const a2: usize = if (pj == 0) 0 else B[pj - 1];
                        const b2: usize = if (pj + 1 == lenB) 0 else B[pj + 1];
                        // exact O(1) distance delta of the exchange
                        const delta = self.dd(a1, v) + self.dd(v, b1) - self.dd(a1, u) - self.dd(u, b1) +
                            self.dd(a2, u) + self.dd(u, b2) - self.dd(a2, v) - self.dd(v, b2);
                        if (delta >= 0) continue;
                        if (!self.replaceFeas(A, self.pa, self.sa, pi, v)) continue;
                        if (!self.replaceFeas(B, self.pb, self.sb, pj, u)) continue;
                        self.routes.items[ri].items[pi] = v;
                        self.routes.items[rj].items[pj] = u;
                        self.recompute();
                        return true;
                    }
                }
            }
        }
        return false;
    }

    // Replace route r's contents with `items` (single-route in-place move).
    fn applyCandidate(self: *Solution, r: usize, items: []const usize) !void {
        self.routes.items[r].clearRetainingCapacity();
        try self.routes.items[r].appendSlice(self.allocator, items);
    }
    fn setRoute(self: *Solution, r: usize, items: []const usize) !void {
        self.routes.items[r].clearRetainingCapacity();
        try self.routes.items[r].appendSlice(self.allocator, items);
    }

    /// Double-bridge kick on the flattened giant order, then re-Split.
    fn perturb(self: *Solution, rng: std.Random) !void {
        const n = self.inst.n;
        self.flattenInto(self.giant_buf);
        if (n >= 8) {
            const g = self.giant_buf;
            const p1 = 1 + rng.uintLessThan(usize, n - 3);
            const p2 = p1 + 1 + rng.uintLessThan(usize, n - p1 - 2);
            const p3 = p2 + 1 + rng.uintLessThan(usize, n - p2 - 1);
            var tmp: std.ArrayList(usize) = .empty;
            defer tmp.deinit(self.allocator);
            try tmp.appendSlice(self.allocator, g[0..p1]);
            try tmp.appendSlice(self.allocator, g[p2..p3]);
            try tmp.appendSlice(self.allocator, g[p1..p2]);
            try tmp.appendSlice(self.allocator, g[p3..]);
            @memcpy(self.giant_buf, tmp.items);
        }
        try self.rebuildFromGiant(self.giant_buf);
    }

    fn toResult(self: *const Solution, allocator: std.mem.Allocator) !VrptwResult {
        var total: u64 = 0;
        var count: usize = 0;
        for (self.routes.items) |r| {
            if (r.items.len > 0) count += 1;
        }
        const routes = try allocator.alloc([]usize, count);
        var k: usize = 0;
        for (self.routes.items, 0..) |r, ri| {
            if (r.items.len == 0) continue;
            routes[k] = try allocator.dupe(usize, r.items);
            total += self.rdist.items[ri];
            k += 1;
        }
        return .{ .allocator = allocator, .routes = routes, .total_cost = total, .vehicles = count };
    }
};

/// Independent feasibility + cost checker. Returns total distance or null if any
/// route violates TW/capacity/horizon, or any customer is missed/duplicated.
pub fn validate(inst: VrptwInstance, routes: []const []const usize) ?u64 {
    const seen = std.heap.page_allocator.alloc(bool, inst.n + 1) catch return null;
    defer std.heap.page_allocator.free(seen);
    @memset(seen, false);
    var total: u64 = 0;
    for (routes) |r| {
        if (r.len == 0) continue;
        for (r) |c| {
            if (c == 0 or c > inst.n or seen[c]) return null;
            seen[c] = true;
        }
        const dist = scheduleSlice(inst, r) orelse return null;
        total += dist;
    }
    for (1..inst.n + 1) |c| if (!seen[c]) return null;
    return total;
}

// ===================== HGS population layer =====================
// Hybrid Genetic Search (Vidal): a population of giant tours, each decoded by
// Split + educated by the route local search. Offspring come from OX crossover of
// two parents (selected by biased fitness = quality + diversity), then educated.
// Diversity in the survivor/selection pressure is what escapes the single-
// incumbent traps that ILS can't (e.g. the rc101 vehicle-count plateau). This
// reuses the same Solution/Split/localSearch as solveVrptw, so it serves CVRP too
// (CVRP = infinite windows).

pub const HgsParams = struct {
    mu: usize = 40, // population floor (survivors per generation)
    lambda: usize = 60, // offspring generated per generation
    generations: usize = 60,
    n_elite: usize = 12, // elites whose cost rank dominates biased fitness
    n_close: usize = 6, // neighbours averaged for the diversity contribution
    restart_gens: usize = 20, // gentle diversification (keep better half) after this many stagnant gens (0 = never)
    veh_penalty: u64 = 0, // per-route penalty added in Split to bias toward fewer vehicles (0 = pure distance)
};

const Individual = struct {
    giant: []usize, // educated giant tour (the chromosome)
    edges: []u64, // sorted undirected giant-cycle edges, for diversity
    cost: u64, // penalized cost (distance + veh_penalty * routes)
    fn deinit(self: *Individual, allocator: std.mem.Allocator) void {
        allocator.free(self.giant);
        allocator.free(self.edges);
    }
};

fn educate(allocator: std.mem.Allocator, inst: VrptwInstance, giant: []const usize, veh_penalty: u64, gran: []const usize, gk: usize) !Solution {
    var sol = try Solution.fromGiant(allocator, inst, giant, veh_penalty, gran, gk);
    errdefer sol.deinit();
    sol.swap_star = true; // HGS education uses the full neighbourhood incl. SWAP*
    sol.granular = true; // and restricts inter-route moves to k-nearest neighbours (fast)
    try sol.localSearch();
    return sol;
}

/// Biased fitness = costRank + (1 - n_elite/N) * diversityRank (Vidal). Lower is
/// better. Diversity = average broken-pairs distance to the n_close nearest pop
/// members (more isolated -> better diversity rank).
fn computeBiased(allocator: std.mem.Allocator, pop: []const Individual, out: []f64, params: HgsParams, n: usize) !void {
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

    // cost rank
    for (0..N) |k| idxbuf[k] = k;
    std.sort.pdq(usize, idxbuf, pop, struct {
        fn lt(p: []const Individual, a: usize, b: usize) bool {
            return p[a].cost < p[b].cost;
        }
    }.lt);
    for (0..N) |r| cost_rank[idxbuf[r]] = r;

    // diversity contribution: avg distance to n_close nearest; also flag clones
    // (identical edge set to an at-least-as-good individual) for culling.
    const clone = try allocator.alloc(bool, N);
    defer allocator.free(clone);
    const nc = @min(params.n_close, if (N > 0) N - 1 else 0);
    for (0..N) |i| {
        var m: usize = 0;
        var is_clone = false;
        for (0..N) |jj| {
            if (jj == i) continue;
            const dist = n - hgs_core.edgeOverlap(pop[i].edges, pop[jj].edges);
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
    // diversity rank: higher div -> better (rank 0)
    for (0..N) |k| idxbuf[k] = k;
    std.sort.pdq(usize, idxbuf, div, struct {
        fn lt(d: []const f64, a: usize, b: usize) bool {
            return d[a] > d[b];
        }
    }.lt);
    for (0..N) |r| div_rank[idxbuf[r]] = r;

    const elite_frac = 1.0 - @as(f64, @floatFromInt(params.n_elite)) / @as(f64, @floatFromInt(N));
    const big: f64 = @floatFromInt(2 * N); // clone penalty: dominates rank sums
    for (0..N) |i| {
        out[i] = @as(f64, @floatFromInt(cost_rank[i])) + elite_frac * @as(f64, @floatFromInt(div_rank[i]));
        if (clone[i]) out[i] += big;
    }
}

/// Generate one random educated individual, append to the population, update best.
fn seedRandom(allocator: std.mem.Allocator, inst: VrptwInstance, veh_penalty: u64, ebuf: []usize, rng: std.Random, pop: *std.ArrayList(Individual), best: *VrptwResult, best_cost: *u64, best_giant: []usize, n: usize, gran: []const usize, gk: usize) !void {
    for (0..n) |k| ebuf[k] = k + 1;
    var t = n;
    while (t > 1) {
        t -= 1;
        const r = rng.uintLessThan(usize, t + 1);
        std.mem.swap(usize, &ebuf[t], &ebuf[r]);
    }
    var sol = try educate(allocator, inst, ebuf, veh_penalty, gran, gk);
    defer sol.deinit();
    sol.flattenInto(ebuf);
    try pop.append(allocator, .{ .giant = try allocator.dupe(usize, ebuf), .edges = try hgs_core.buildEdges(allocator, ebuf, n), .cost = sol.cost });
    if (sol.cost < best_cost.*) {
        best_cost.* = sol.cost;
        best.deinit();
        best.* = try sol.toResult(allocator);
        @memcpy(best_giant, ebuf);
    }
}

// Binary tournament over the first `count` individuals — the parents whose biased
// fitness was computed this generation, NOT the offspring being appended (whose
// biased[] slots are still uninitialized, which would make selection depend on
// stale heap contents).
fn tournament(biased: []const f64, count: usize, rng: std.Random) usize {
    const a = rng.uintLessThan(usize, count);
    const b = rng.uintLessThan(usize, count);
    return if (biased[a] <= biased[b]) a else b;
}

/// HGS solve: population of giant tours evolved by OX + education with biased-
/// fitness selection and diversity-aware survivor selection. Reports the best
/// feasible solution found. `params.veh_penalty` biases toward fewer vehicles.
pub fn solveVrptwHgs(allocator: std.mem.Allocator, inst: VrptwInstance, options: solver.SolveOptions, params: HgsParams) !VrptwResult {
    const veh_penalty = params.veh_penalty;
    const n = inst.n;
    if (inst.demand.len != n + 1 or inst.matrix.len != (std.math.mul(usize, n + 1, n + 1) catch return error.InvalidInstance)) return error.InvalidInstance;
    if (inst.ready.len != n + 1 or inst.due.len != n + 1 or inst.service.len != n + 1) return error.InvalidInstance;

    // initial giant tour via the ATSP core (one good seed for the population)
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

    const gk: usize = @min(@as(usize, 20), if (n > 1) n - 1 else 0);
    const gran = try buildNeighbors(allocator, inst, gk);
    defer allocator.free(gran);

    var prng = std.Random.DefaultPrng.init(options.seed);
    const rng = prng.random();

    const ebuf = try allocator.alloc(usize, n);
    defer allocator.free(ebuf);
    const best_giant = try allocator.alloc(usize, n);
    defer allocator.free(best_giant);

    var pop: std.ArrayList(Individual) = .empty;
    defer {
        for (pop.items) |*ind| ind.deinit(allocator);
        pop.deinit(allocator);
    }

    var best: VrptwResult = undefined;
    var have_best = false;
    var best_cost: u64 = std.math.maxInt(u64);
    errdefer if (have_best) best.deinit();

    // seed individual 0 from the ATSP giant; the rest from random permutations.
    {
        var sol = try educate(allocator, inst, seed_giant, veh_penalty, gran, gk);
        defer sol.deinit();
        sol.flattenInto(ebuf);
        try pop.append(allocator, .{ .giant = try allocator.dupe(usize, ebuf), .edges = try hgs_core.buildEdges(allocator, ebuf, n), .cost = sol.cost });
        best = try sol.toResult(allocator);
        have_best = true;
        best_cost = sol.cost;
        @memcpy(best_giant, ebuf);
    }
    var s: usize = 1;
    while (s < params.mu) : (s += 1) {
        try seedRandom(allocator, inst, veh_penalty, ebuf, rng, &pop, &best, &best_cost, best_giant, n, gran, gk);
    }

    const cap = params.mu + params.lambda;
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
        try computeBiased(allocator, pop.items, biased[0..pop.items.len], params, n);
        const nparents = pop.items.len; // freeze: offspring are not eligible as parents
        var off: usize = 0;
        while (off < params.lambda) : (off += 1) {
            const p1 = tournament(biased[0..nparents], nparents, rng);
            const p2 = tournament(biased[0..nparents], nparents, rng);
            const cg = try hgs_core.oxCrossover(allocator, pop.items[p1].giant, pop.items[p2].giant, n, rng);
            defer allocator.free(cg);
            var sol = try educate(allocator, inst, cg, veh_penalty, gran, gk);
            defer sol.deinit();
            sol.flattenInto(ebuf);
            try pop.append(allocator, .{ .giant = try allocator.dupe(usize, ebuf), .edges = try hgs_core.buildEdges(allocator, ebuf, n), .cost = sol.cost });
            if (sol.cost < best_cost) {
                best_cost = sol.cost;
                best.deinit();
                best = try sol.toResult(allocator);
                @memcpy(best_giant, ebuf);
            }
        }
        // survivor selection: keep the mu best by biased fitness, but always keep
        // the lowest-cost individual (elitism — the incumbent's genes stay in the
        // pool even if it happens to be in a dense, low-diversity region).
        try computeBiased(allocator, pop.items, biased[0..pop.items.len], params, n);
        var best_idx: usize = 0;
        for (1..pop.items.len) |k| {
            if (pop.items[k].cost < pop.items[best_idx].cost) best_idx = k;
        }
        for (0..pop.items.len) |k| order[k] = k;
        std.sort.pdq(usize, order[0..pop.items.len], biased, struct {
            fn lt(bf: []const f64, a: usize, b: usize) bool {
                return bf[a] < bf[b];
            }
        }.lt);
        @memset(keep[0..pop.items.len], false);
        var kept: usize = 0;
        keep[best_idx] = true;
        kept += 1;
        for (order[0..pop.items.len]) |idx| {
            if (kept >= params.mu) break;
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

        // Gentle diversification: if the incumbent has stalled, keep the better
        // half of the population (by cost) and refill the rest with fresh random
        // individuals. This injects diversity without throwing away the good genes
        // a full restart would (full-random regen measured worse in short budgets).
        if (best_cost < before) {
            stagnation = 0;
        } else {
            stagnation += 1;
        }
        if (params.restart_gens > 0 and stagnation >= params.restart_gens) {
            for (0..pop.items.len) |k| order[k] = k;
            std.sort.pdq(usize, order[0..pop.items.len], pop.items, struct {
                fn lt(p: []const Individual, a: usize, b: usize) bool {
                    return p[a].cost < p[b].cost;
                }
            }.lt);
            const surv = @max(@as(usize, 1), params.mu / 2);
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
            while (pop.items.len < params.mu) {
                try seedRandom(allocator, inst, veh_penalty, ebuf, rng, &pop, &best, &best_cost, best_giant, n, gran, gk);
            }
            stagnation = 0;
        }
    }
    if (validate(inst, best.routes) == null) return error.Infeasible;
    return best;
}

// ---- tests ----

test "VRPTW returns a clean error for a TW-unreachable customer" {
    const allocator = std.testing.allocator;
    // Customer 2 is due at 10 but every arc into it costs 100 (from the depot AND
    // from customer 1), so it cannot be served in time in any route position and
    // no TW-feasible split exists. The solver must surface a clean error rather
    // than walk an uninitialized pred chain (regression for splitDpTw, ZIG-1).
    const m = [_]u32{
        0,   5, 100,
        5,   0, 100,
        100, 5, 0,
    };
    const demand = [_]u32{ 0, 1, 1 };
    const ready = [_]u32{ 0, 0, 0 };
    const due = [_]u32{ 1000, 1000, 10 };
    const service = [_]u32{ 0, 0, 0 };
    const inst = VrptwInstance{
        .n = 2,
        .matrix = &m,
        .demand = &demand,
        .capacity = 10,
        .ready = &ready,
        .due = &due,
        .service = &service,
    };
    try std.testing.expectError(error.NoFeasibleSplit, solveVrptw(allocator, inst, .{ .seed = 1 }, .{ .rounds = 5, .restarts = 1, .veh_penalty = 0 }));
}

test "TWS concatenation feasibility matches the scheduler (oracle)" {
    const allocator = std.testing.allocator;
    var prng = std.Random.DefaultPrng.init(0x7503);
    const rng = prng.random();
    // many random instances + random routes; TWS tw==0 must equal scheduleSlice
    // feasibility (capacity made non-binding so only time windows decide).
    var trial: usize = 0;
    while (trial < 400) : (trial += 1) {
        const n = rng.intRangeAtMost(usize, 2, 9);
        const dim = n + 1;
        const matrix = try allocator.alloc(u32, dim * dim);
        defer allocator.free(matrix);
        for (0..dim) |i| {
            for (0..dim) |j| matrix[i * dim + j] = if (i == j) 0 else rng.intRangeAtMost(u32, 1, 40);
        }
        const demand = try allocator.alloc(u32, dim);
        defer allocator.free(demand);
        const ready = try allocator.alloc(u32, dim);
        defer allocator.free(ready);
        const due = try allocator.alloc(u32, dim);
        defer allocator.free(due);
        const service = try allocator.alloc(u32, dim);
        defer allocator.free(service);
        @memset(demand, 0);
        ready[0] = 0;
        due[0] = rng.intRangeAtMost(u32, 200, 600);
        service[0] = 0;
        for (1..dim) |i| {
            ready[i] = rng.intRangeAtMost(u32, 0, 200);
            due[i] = ready[i] + rng.intRangeAtMost(u32, 0, 200); // sometimes very tight
            service[i] = rng.intRangeAtMost(u32, 0, 15);
        }
        const inst = VrptwInstance{ .n = n, .matrix = matrix, .demand = demand, .capacity = 1_000_000, .ready = ready, .due = due, .service = service };
        // random subset/order of customers as a route
        const route = try allocator.alloc(usize, n);
        defer allocator.free(route);
        for (0..n) |i| route[i] = i + 1;
        var k = n;
        while (k > 1) {
            k -= 1;
            const r = rng.uintLessThan(usize, k + 1);
            std.mem.swap(usize, &route[k], &route[r]);
        }
        const len = rng.intRangeAtMost(usize, 1, n);
        const sched_feasible = scheduleSlice(inst, route[0..len]) != null;
        const tws_feasible = routeTws(inst, route[0..len]).tw == 0;
        try std.testing.expectEqual(sched_feasible, tws_feasible);
    }
}

test "concat insert/replace feasibility matches the scheduler (oracle)" {
    const allocator = std.testing.allocator;
    var prng = std.Random.DefaultPrng.init(0x9001);
    const rng = prng.random();
    var trial: usize = 0;
    while (trial < 300) : (trial += 1) {
        const n = rng.intRangeAtMost(usize, 2, 8);
        const dim = n + 1;
        const matrix = try allocator.alloc(u32, dim * dim);
        defer allocator.free(matrix);
        for (0..dim) |i| {
            for (0..dim) |j| matrix[i * dim + j] = if (i == j) 0 else rng.intRangeAtMost(u32, 1, 30);
        }
        const demand = try allocator.alloc(u32, dim);
        defer allocator.free(demand);
        const ready = try allocator.alloc(u32, dim);
        defer allocator.free(ready);
        const due = try allocator.alloc(u32, dim);
        defer allocator.free(due);
        const service = try allocator.alloc(u32, dim);
        defer allocator.free(service);
        @memset(demand, 0);
        ready[0] = 0;
        due[0] = rng.intRangeAtMost(u32, 150, 400);
        service[0] = 0;
        for (1..dim) |i| {
            ready[i] = rng.intRangeAtMost(u32, 0, 150);
            due[i] = ready[i] + rng.intRangeAtMost(u32, 0, 150);
            service[i] = rng.intRangeAtMost(u32, 0, 10);
        }
        const inst = VrptwInstance{ .n = n, .matrix = matrix, .demand = demand, .capacity = 1_000_000, .ready = ready, .due = due, .service = service };

        // a random non-empty route over a subset of customers
        var perm = try allocator.alloc(usize, n);
        defer allocator.free(perm);
        for (0..n) |i| perm[i] = i + 1;
        var k = n;
        while (k > 1) {
            k -= 1;
            const r = rng.uintLessThan(usize, k + 1);
            std.mem.swap(usize, &perm[k], &perm[r]);
        }
        const len = rng.intRangeAtMost(usize, 1, n - 1);
        const route = perm[0..len];
        const extra = perm[len]; // a customer not in the route

        // build pre/suf exactly as fillPreSuf does
        const pre = try allocator.alloc(Tws, len + 1);
        defer allocator.free(pre);
        const suf = try allocator.alloc(Tws, len + 1);
        defer allocator.free(suf);
        pre[0] = Tws.depotNode(inst);
        var prev: usize = 0;
        for (0..len) |i| {
            pre[i + 1] = Tws.merge(pre[i], @intCast(inst.d(prev, route[i])), Tws.client(inst, route[i]));
            prev = route[i];
        }
        suf[len] = Tws.depotNode(inst);
        var i: usize = len;
        while (i > 0) {
            i -= 1;
            const nxt: usize = if (i + 1 == len) 0 else route[i + 1];
            suf[i] = Tws.merge(Tws.client(inst, route[i]), @intCast(inst.d(route[i], nxt)), suf[i + 1]);
        }

        const cand = try allocator.alloc(usize, len + 1);
        defer allocator.free(cand);
        // INSERT `extra` at every gap q
        for (0..len + 1) |q| {
            const c: usize = if (q == 0) 0 else route[q - 1];
            const e: usize = if (q == len) 0 else route[q];
            const m1 = Tws.merge(pre[q], @intCast(inst.d(c, extra)), Tws.client(inst, extra));
            const m2 = Tws.merge(m1, @intCast(inst.d(extra, e)), suf[q]);
            const feas_tws = m2.tw == 0;
            @memcpy(cand[0..q], route[0..q]);
            cand[q] = extra;
            @memcpy(cand[q + 1 .. len + 1], route[q..]);
            const feas_sched = scheduleSlice(inst, cand[0 .. len + 1]) != null;
            try std.testing.expectEqual(feas_sched, feas_tws);
        }
        // REPLACE node at every position p with `extra`
        for (0..len) |p| {
            const a: usize = if (p == 0) 0 else route[p - 1];
            const b: usize = if (p + 1 == len) 0 else route[p + 1];
            const m1 = Tws.merge(pre[p], @intCast(inst.d(a, extra)), Tws.client(inst, extra));
            const m2 = Tws.merge(m1, @intCast(inst.d(extra, b)), suf[p + 1]);
            const feas_tws = m2.tw == 0;
            @memcpy(cand[0..len], route);
            cand[p] = extra;
            const feas_sched = scheduleSlice(inst, cand[0..len]) != null;
            try std.testing.expectEqual(feas_sched, feas_tws);
        }
    }
}

test "scheduleSlice: feasible vs one-unit-late infeasible" {
    const n = 2;
    const matrix = [_]u32{
        0, 10, 10,
        10, 0, 10,
        10, 10, 0,
    };
    const demand = [_]u32{ 0, 1, 1 };
    const service = [_]u32{ 0, 5, 5 };
    // route 0->1->2->0: arrive1=10, start1=10, depart=15; arrive2=25
    var inst = VrptwInstance{
        .n = n,
        .matrix = &matrix,
        .demand = &demand,
        .capacity = 10,
        .ready = &[_]u32{ 0, 0, 0 },
        .due = &[_]u32{ 1000, 100, 25 },
        .service = &service,
    };
    const route = [_]usize{ 1, 2 };
    try std.testing.expect(scheduleSlice(inst, &route) != null); // due2=25 == arrive2 ok
    inst.due = &[_]u32{ 1000, 100, 24 }; // now 1 unit too tight
    try std.testing.expect(scheduleSlice(inst, &route) == null);
}

test "VRPTW Split avoids a TW-infeasible merge that CVRP would take" {
    const allocator = std.testing.allocator;
    const n = 2;
    // 1 and 2 are cheap to serve together by distance, but their windows force
    // separate routes: serving 1 then 2 arrives at 2 far past its due time.
    const matrix = [_]u32{
        0, 10, 10,
        10, 0, 2,
        10, 2, 0,
    };
    const demand = [_]u32{ 0, 1, 1 };
    const inst = VrptwInstance{
        .n = n,
        .matrix = &matrix,
        .demand = &demand,
        .capacity = 10,
        .ready = &[_]u32{ 0, 0, 0 },
        // node 2 reachable alone (arrive depot->2 at t=10 <= 15) but the merge
        // 0-1-2 arrives at 2 at t=22 > 15, so a single route is TW-infeasible.
        .due = &[_]u32{ 1000, 100, 15 },
        .service = &[_]u32{ 0, 10, 0 },
    };
    // giant 1,2: route 0-1-2 reaches node 2 too late; must split into two routes
    const giant = [_]usize{ 1, 2 };
    const sp = try splitDpTw(allocator, inst, &giant, 0);
    defer allocator.free(sp.pred);
    // two routes: (0-1-0)=20 + (0-2-0)=20 = 40 (merge 0-1-2-0 is TW-infeasible)
    try std.testing.expectEqual(@as(u64, 40), sp.cost);
}

fn bruteForceTw(allocator: std.mem.Allocator, inst: VrptwInstance) !u64 {
    const n = inst.n;
    const perm = try allocator.alloc(usize, n);
    defer allocator.free(perm);
    for (0..n) |i| perm[i] = i + 1;
    var best: u64 = std.math.maxInt(u64);
    try permRecTw(allocator, inst, perm, 0, &best);
    return best;
}
fn permRecTw(allocator: std.mem.Allocator, inst: VrptwInstance, perm: []usize, k: usize, best: *u64) !void {
    if (k == perm.len) {
        const sp = try splitDpTw(allocator, inst, perm, 0);
        defer allocator.free(sp.pred);
        if (sp.cost < best.*) best.* = sp.cost;
        return;
    }
    for (k..perm.len) |i| {
        std.mem.swap(usize, &perm[k], &perm[i]);
        try permRecTw(allocator, inst, perm, k + 1, best);
        std.mem.swap(usize, &perm[k], &perm[i]);
    }
}

test "VRPTW engine reaches the brute-force optimum on a small instance" {
    const allocator = std.testing.allocator;
    const n = 6;
    const dim = n + 1;
    var prng = std.Random.DefaultPrng.init(0xBEEF);
    const rng = prng.random();
    const matrix = try allocator.alloc(u32, dim * dim);
    defer allocator.free(matrix);
    for (0..dim) |i| {
        for (0..dim) |j| matrix[i * dim + j] = if (i == j) 0 else rng.intRangeAtMost(u32, 1, 40);
    }
    const demand = try allocator.alloc(u32, dim);
    defer allocator.free(demand);
    const ready = try allocator.alloc(u32, dim);
    defer allocator.free(ready);
    const due = try allocator.alloc(u32, dim);
    defer allocator.free(due);
    const service = try allocator.alloc(u32, dim);
    defer allocator.free(service);
    demand[0] = 0;
    ready[0] = 0;
    due[0] = 1000;
    service[0] = 0;
    for (1..dim) |i| {
        demand[i] = rng.intRangeAtMost(u32, 1, 4);
        ready[i] = rng.intRangeAtMost(u32, 0, 50);
        due[i] = ready[i] + rng.intRangeAtMost(u32, 30, 120);
        service[i] = rng.intRangeAtMost(u32, 1, 8);
    }
    const inst = VrptwInstance{ .n = n, .matrix = matrix, .demand = demand, .capacity = 8, .ready = ready, .due = due, .service = service };

    const opt = try bruteForceTw(allocator, inst);
    if (opt == std.math.maxInt(u64)) return; // no feasible split for this random draw; skip
    var res = try solveVrptw(allocator, inst, .{
        .seed = 5,
        .budget = .{ .trials = 30, .max_passes = 40 },
        .candidates = .{ .candidate_count = 5, .candidate_mode = .alpha_nearness, .sparse_min_dimension = 0 },
        .search = .{ .enable_lk = true, .lk_max_depth = 5 },
    }, .{ .rounds = 80, .restarts = 4, .veh_penalty = 0 });
    defer res.deinit();
    const checked = validate(inst, res.routes) orelse return error.Infeasible;
    try std.testing.expectEqual(res.total_cost, checked);
    try std.testing.expectEqual(opt, res.total_cost);
}

test "VRPTW HGS: feasible and no worse than the ILS engine" {
    const allocator = std.testing.allocator;
    const n = 12;
    const dim = n + 1;
    var prng = std.Random.DefaultPrng.init(0x515);
    const rng = prng.random();
    const matrix = try allocator.alloc(u32, dim * dim);
    defer allocator.free(matrix);
    for (0..dim) |i| {
        for (0..dim) |j| matrix[i * dim + j] = if (i == j) 0 else rng.intRangeAtMost(u32, 1, 60);
    }
    const demand = try allocator.alloc(u32, dim);
    defer allocator.free(demand);
    const ready = try allocator.alloc(u32, dim);
    defer allocator.free(ready);
    const due = try allocator.alloc(u32, dim);
    defer allocator.free(due);
    const service = try allocator.alloc(u32, dim);
    defer allocator.free(service);
    demand[0] = 0;
    ready[0] = 0;
    due[0] = 100000;
    service[0] = 0;
    for (1..dim) |i| {
        demand[i] = rng.intRangeAtMost(u32, 1, 5);
        ready[i] = 0;
        due[i] = 100000;
        service[i] = rng.intRangeAtMost(u32, 1, 5);
    }
    const inst = VrptwInstance{ .n = n, .matrix = matrix, .demand = demand, .capacity = 15, .ready = ready, .due = due, .service = service };
    const opts = solver.SolveOptions{
        .seed = 3,
        .budget = .{ .trials = 30, .max_passes = 40 },
        .candidates = .{ .candidate_count = 6, .candidate_mode = .alpha_nearness, .sparse_min_dimension = 0 },
        .search = .{ .enable_lk = true, .lk_max_depth = 5 },
    };
    var ils = try solveVrptw(allocator, inst, opts, .{ .rounds = 40, .restarts = 4, .veh_penalty = 0 });
    defer ils.deinit();
    var hgs = try solveVrptwHgs(allocator, inst, opts, .{ .mu = 10, .lambda = 15, .generations = 15, .veh_penalty = 0 });
    defer hgs.deinit();
    const hchk = validate(inst, hgs.routes) orelse return error.Infeasible;
    try std.testing.expectEqual(hgs.total_cost, hchk);
    // HGS (population) should match or beat the single-incumbent ILS.
    try std.testing.expect(hgs.total_cost <= ils.total_cost);
}

test "VRPTW end-to-end: feasible and beats one-per-route" {
    const allocator = std.testing.allocator;
    const n = 12;
    const dim = n + 1;
    var prng = std.Random.DefaultPrng.init(0x77);
    const rng = prng.random();
    const matrix = try allocator.alloc(u32, dim * dim);
    defer allocator.free(matrix);
    for (0..dim) |i| {
        for (0..dim) |j| matrix[i * dim + j] = if (i == j) 0 else rng.intRangeAtMost(u32, 1, 60);
    }
    const demand = try allocator.alloc(u32, dim);
    defer allocator.free(demand);
    const ready = try allocator.alloc(u32, dim);
    defer allocator.free(ready);
    const due = try allocator.alloc(u32, dim);
    defer allocator.free(due);
    const service = try allocator.alloc(u32, dim);
    defer allocator.free(service);
    demand[0] = 0;
    ready[0] = 0;
    due[0] = 100000;
    service[0] = 0;
    for (1..dim) |i| {
        demand[i] = rng.intRangeAtMost(u32, 1, 5);
        ready[i] = 0;
        due[i] = 100000; // wide windows -> always feasible, exercises the engine
        service[i] = rng.intRangeAtMost(u32, 1, 5);
    }
    const inst = VrptwInstance{ .n = n, .matrix = matrix, .demand = demand, .capacity = 15, .ready = ready, .due = due, .service = service };

    var res = try solveVrptw(allocator, inst, .{
        .seed = 9,
        .budget = .{ .trials = 30, .max_passes = 40 },
        .candidates = .{ .candidate_count = 6, .candidate_mode = .alpha_nearness, .sparse_min_dimension = 0 },
        .search = .{ .enable_lk = true, .lk_max_depth = 5 },
    }, .{ .rounds = 60, .restarts = 4, .veh_penalty = 0 });
    defer res.deinit();
    const checked = validate(inst, res.routes) orelse return error.Infeasible;
    try std.testing.expectEqual(res.total_cost, checked);
    var baseline: u64 = 0;
    for (1..dim) |c| baseline += inst.d(0, c) + inst.d(c, 0);
    try std.testing.expect(res.total_cost < baseline);
}
