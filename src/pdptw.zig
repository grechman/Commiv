const std = @import("std");

// Pickup-and-Delivery Problem with Time Windows (PDPTW). Each customer is
// either a pickup or its paired dropoff (nodes 1..2*n_pairs; node 0 is the
// depot). A route must carry every pickup before its dropoff, in the same
// route, and must never violate vehicle capacity (walking the prefix load)
// or the per-node time window [ready, due] under free waiting. Travel time
// is a directed distance matrix, so real asymmetric road times work
// directly. Objective: minimize total directed travel distance over
// feasible routes. Provides an independent feasibility/cost oracle
// (`validate`), the Lseg prefix-load algebra, and the construction +
// pair-relocate local search building blocks used by the PDPTW-SISR engine
// (pdptw_sisr.zig).

pub const PdpInstance = struct {
    n_pairs: usize,
    matrix: []const u32, // dim*dim, row-major, DIRECTED: matrix[a*dim+b] = a->b
    capacity: i64,
    pair_of: []const usize, // dim; pair_of[0]=0; pair_of[pair_of[c]] == c for c>=1
    is_pickup: []const bool, // dim; is_pickup[0]=false
    demand_signed: []const i64, // dim; +q at pickup, -q at its dropoff, 0 at depot
    ready: []const u32, // dim; ready[0]=0
    due: []const u32, // dim; due[0] = horizon (depot close)
    service: []const u32, // dim; service[0]=0

    pub fn dim(self: PdpInstance) usize {
        return 2 * self.n_pairs + 1;
    }

    pub fn d(self: PdpInstance, a: usize, b: usize) u64 {
        return self.matrix[a * self.dim() + b];
    }
};

pub const PairingError = error{ PairIdOutOfRange, PairNodeReused, DemandExceedsCapacity, NodeUncovered, OutOfMemory };

pub const Pairing = struct {
    allocator: std.mem.Allocator,
    pair_of: []usize,
    is_pickup: []bool,
    demand_signed: []i64,

    pub fn deinit(self: *Pairing) void {
        self.allocator.free(self.pair_of);
        self.allocator.free(self.is_pickup);
        self.allocator.free(self.demand_signed);
        self.* = undefined;
    }
};

pub fn buildPairing(allocator: std.mem.Allocator, dim: usize, pickups: []const u32, deliveries: []const u32, demand: []const u32, capacity: u32) PairingError!Pairing {
    const pair_of = try allocator.alloc(usize, dim);
    errdefer allocator.free(pair_of);
    const is_pickup = try allocator.alloc(bool, dim);
    errdefer allocator.free(is_pickup);
    const demand_signed = try allocator.alloc(i64, dim);
    errdefer allocator.free(demand_signed);
    const seen = try allocator.alloc(bool, dim);
    defer allocator.free(seen);
    @memset(seen, false);
    pair_of[0] = 0;
    is_pickup[0] = false;
    demand_signed[0] = 0;
    for (pickups, deliveries, demand) |pu32, qu32, dm| {
        const pu: usize = pu32;
        const qu: usize = qu32;
        if (pu == 0 or qu == 0 or pu >= dim or qu >= dim or pu == qu) return error.PairIdOutOfRange;
        if (seen[pu] or seen[qu]) return error.PairNodeReused;
        if (dm > capacity) return error.DemandExceedsCapacity;
        seen[pu] = true;
        seen[qu] = true;
        pair_of[pu] = qu;
        pair_of[qu] = pu;
        is_pickup[pu] = true;
        is_pickup[qu] = false;
        demand_signed[pu] = @as(i64, dm);
        demand_signed[qu] = -@as(i64, dm);
    }
    for (seen[1..dim]) |s| {
        if (!s) return error.NodeUncovered;
    }
    return .{ .allocator = allocator, .pair_of = pair_of, .is_pickup = is_pickup, .demand_signed = demand_signed };
}

test "buildPairing: accepts a valid pairing and reports each violation in scan order" {
    const allocator = std.testing.allocator;
    var ok = try buildPairing(allocator, 5, &.{ 1, 2 }, &.{ 3, 4 }, &.{ 4, 5 }, 10);
    defer ok.deinit();
    try std.testing.expectEqualSlices(usize, &.{ 0, 3, 4, 1, 2 }, ok.pair_of);
    try std.testing.expectEqualSlices(bool, &.{ false, true, true, false, false }, ok.is_pickup);
    try std.testing.expectEqualSlices(i64, &.{ 0, 4, 5, -4, -5 }, ok.demand_signed);
    try std.testing.expectError(error.PairIdOutOfRange, buildPairing(allocator, 5, &.{ 1, 2 }, &.{ 1, 4 }, &.{ 4, 5 }, 10));
    try std.testing.expectError(error.PairIdOutOfRange, buildPairing(allocator, 5, &.{ 1, 2 }, &.{ 3, 5 }, &.{ 4, 5 }, 10));
    try std.testing.expectError(error.PairNodeReused, buildPairing(allocator, 5, &.{ 1, 2 }, &.{ 3, 3 }, &.{ 4, 5 }, 10));
    try std.testing.expectError(error.DemandExceedsCapacity, buildPairing(allocator, 5, &.{ 1, 2 }, &.{ 3, 4 }, &.{ 11, 5 }, 10));
    try std.testing.expectError(error.DemandExceedsCapacity, buildPairing(allocator, 5, &.{ 1, 2 }, &.{ 3, 3 }, &.{ 11, 5 }, 10));
    try std.testing.expectError(error.NodeUncovered, buildPairing(allocator, 7, &.{ 1, 2 }, &.{ 3, 4 }, &.{ 4, 5 }, 10));
}

/// Independent feasibility + cost checker for a whole solution.
/// Returns the recomputed total directed distance, or null on ANY violation
/// (membership / duplicate / unvisited / pairing / precedence / capacity / TW).
pub fn validate(inst: PdpInstance, routes: []const []const usize) ?u64 {
    const dim = inst.dim();
    const seen = std.heap.page_allocator.alloc(bool, dim) catch return null;
    defer std.heap.page_allocator.free(seen);
    @memset(seen, false);

    // 1. membership / duplicate
    for (routes) |r| {
        for (r) |c| {
            if (c == 0 or c >= dim or seen[c]) return null;
            seen[c] = true;
        }
    }

    // 2. pairing (same route)
    for (routes) |r| {
        for (r) |c| {
            const m = inst.pair_of[c];
            var found = false;
            for (r) |c2| {
                if (c2 == m) {
                    found = true;
                    break;
                }
            }
            if (!found) return null;
        }
    }

    // 3. precedence
    for (routes) |r| {
        for (r, 0..) |c, i| {
            if (!inst.is_pickup[c]) continue;
            const m = inst.pair_of[c];
            var j_idx: ?usize = null;
            for (r, 0..) |c2, j| {
                if (c2 == m) {
                    j_idx = j;
                    break;
                }
            }
            const j = j_idx orelse return null;
            if (!(i < j)) return null;
        }
    }

    // 4. capacity
    for (routes) |r| {
        var load: i64 = 0;
        for (r) |c| {
            load += inst.demand_signed[c];
            if (load < 0 or load > inst.capacity) return null;
        }
    }

    // 5. time windows + cost
    var total: u64 = 0;
    for (routes) |r| {
        if (r.len == 0) continue;
        var t: u64 = 0;
        var prev: usize = 0;
        for (r) |c| {
            const arrive = t + inst.d(prev, c);
            const start = @max(arrive, @as(u64, inst.ready[c]));
            if (start > inst.due[c]) return null;
            t = start + inst.service[c];
            prev = c;
        }
        t += inst.d(prev, 0);
        if (t > inst.due[0]) return null;

        // recompute cost of the route directly (distance only, no waiting)
        var cost: u64 = 0;
        var p: usize = 0;
        for (r) |c| {
            cost += inst.d(p, c);
            p = c;
        }
        cost += inst.d(p, 0);
        total += cost;
    }

    // 6. global: everyone served
    for (1..dim) |c| if (!seen[c]) return null;

    return total;
}

/// validate() for a heterogeneous fleet: identical checks, but route r's
/// capacity bound is caps[r] (per-route vehicle-type capacity) instead of the
/// uniform inst.capacity. caps.len must equal routes.len. The existing
/// validate() is untouched — benches and uniform-fleet callers keep using it.
pub fn validateTyped(inst: PdpInstance, routes: []const []const usize, caps: []const i64) ?u64 {
    if (caps.len != routes.len) return null;
    var relaxed = inst;
    var maxcap: i64 = 0;
    for (caps) |c| maxcap = @max(maxcap, c);
    relaxed.capacity = maxcap;
    const total = validate(relaxed, routes) orelse return null;
    for (routes, caps) |r, cap| {
        var load: i64 = 0;
        for (r) |c| {
            load += inst.demand_signed[c];
            if (load < 0 or load > cap) return null;
        }
    }
    return total;
}

/// validate() plus the driver-break contract: every route whose depart-at-0
/// schedule finishes after `earliest` must admit ONE break of `dur` time
/// units starting within [earliest, latest] at SOME inter-stop gap (depot
/// departure and pre-return included) with every window still met. This
/// oracle brute-forces every gap — deliberately NOT the engine's shortcut —
/// so the two can disagree only if one of them is wrong. Returns the total
/// directed distance or null on any violation.
pub fn validateWithBreak(inst: PdpInstance, routes: []const []const usize, brk_dur: u32, brk_earliest: u32, brk_latest: u32) ?u64 {
    if (brk_earliest > brk_latest) return null;
    // A with-break-feasible schedule is also feasible without it (dropping
    // the break only moves arrivals earlier), so the uniform checks apply.
    const total = validate(inst, routes) orelse return null;
    for (routes) |r| {
        if (r.len == 0) continue;
        // No-break completion: ends before the window opens = no break owed.
        var t: u64 = 0;
        var prev: usize = 0;
        for (r) |c| {
            t = @max(t + inst.d(prev, c), @as(u64, inst.ready[c])) + inst.service[c];
            prev = c;
        }
        if (t + inst.d(prev, 0) <= brk_earliest) continue;

        var any = false;
        gaps: for (0..r.len + 1) |g| {
            t = 0;
            prev = 0;
            for (r, 0..) |c, i| {
                if (i == g) {
                    const bs = @max(t, @as(u64, brk_earliest));
                    if (bs > brk_latest) continue :gaps;
                    t = bs + brk_dur;
                }
                const start = @max(t + inst.d(prev, c), @as(u64, inst.ready[c]));
                if (start > inst.due[c]) continue :gaps;
                t = start + inst.service[c];
                prev = c;
            }
            if (g == r.len) {
                const bs = @max(t, @as(u64, brk_earliest));
                if (bs > brk_latest) continue :gaps;
                t = bs + brk_dur;
            }
            if (t + inst.d(prev, 0) > inst.due[0]) continue :gaps;
            any = true;
            break;
        }
        if (!any) return null;
    }
    return total;
}

/// Load-segment algebra: shared O(1)-concat capacity primitive (see
/// core/lseg.zig), instantiated over PdpInstance.
pub const Lseg = @import("core/lseg.zig").Lseg(PdpInstance);

/// Fold a node sequence into one Lseg (left to right, starting from empty()).
pub fn routeLseg(inst: PdpInstance, nodes: []const usize) Lseg {
    var acc = Lseg.empty();
    for (nodes) |c| {
        acc = Lseg.merge(acc, Lseg.node(inst, c));
    }
    return acc;
}

/// Feasibility + exact directed cost of `depot -> nodes -> depot` for ONE route.
/// Returns null if the sequence violates membership/duplicate/pairing/precedence/
/// capacity/TW. `pos` is a caller-owned scratch of length `inst.dim()` that must be
/// ALL ZERO on entry; it is restored to all-zero before return (including on the
/// null paths).
pub fn routeCost(inst: PdpInstance, nodes: []const usize, pos: []usize) ?u64 {
    if (nodes.len == 0) return 0;
    defer for (nodes) |c| {
        if (c != 0 and c < inst.dim()) pos[c] = 0;
    };

    const dim = inst.dim();

    // 1. index + membership + duplicates
    for (nodes, 0..) |c, i| {
        if (c == 0 or c >= dim or pos[c] != 0) return null;
        pos[c] = i + 1;
    }

    // 2. pairing + precedence
    for (nodes) |c| {
        const m = inst.pair_of[c];
        if (pos[m] == 0) return null;
        if (inst.is_pickup[c] and pos[c] > pos[m]) return null;
    }

    // 3. capacity
    const seg = routeLseg(inst, nodes);
    if (!seg.feasible(inst.capacity)) return null;

    // 4. TW + distance
    var t: u64 = 0;
    var prev: usize = 0;
    var cost: u64 = 0;
    for (nodes) |c| {
        const arc = inst.d(prev, c);
        const arrive = t + arc;
        const start = @max(arrive, @as(u64, inst.ready[c]));
        if (start > inst.due[c]) return null;
        t = start + inst.service[c];
        cost += arc;
        prev = c;
    }
    const back = inst.d(prev, 0);
    t += back;
    cost += back;
    if (t > inst.due[0]) return null;

    return cost;
}

// ---------------------------------------------------------------------------
// Test fixtures
// ---------------------------------------------------------------------------

const Fixture2 = struct {
    matrix: [25]u32,
    pair_of: [5]usize,
    is_pickup: [5]bool,
    demand_signed: [5]i64,
    ready: [5]u32,
    due: [5]u32,
    service: [5]u32,

    fn inst(self: *const Fixture2, capacity: i64) PdpInstance {
        return .{
            .n_pairs = 2,
            .matrix = self.matrix[0..],
            .capacity = capacity,
            .pair_of = self.pair_of[0..],
            .is_pickup = self.is_pickup[0..],
            .demand_signed = self.demand_signed[0..],
            .ready = self.ready[0..],
            .due = self.due[0..],
            .service = self.service[0..],
        };
    }
};

fn fixture2() Fixture2 {
    return .{
        .matrix = .{
            0,  10, 12, 30, 32,
            11, 0,  5,  7,  20,
            13, 6,  0,  21, 8,
            31, 9,  22, 0,  4,
            33, 23, 9,  3,  0,
        },
        .pair_of = .{ 0, 3, 4, 1, 2 },
        .is_pickup = .{ false, true, true, false, false },
        .demand_signed = .{ 0, 2, 2, -2, -2 },
        .ready = .{ 0, 0, 0, 0, 0 },
        .due = .{ 1000, 1000, 1000, 1000, 1000 },
        .service = .{ 0, 0, 0, 0, 0 },
    };
}

// ---------------------------------------------------------------------------
// Random-instance generator (test helper)
// ---------------------------------------------------------------------------

pub const RandInst = struct {
    matrix: []u32,
    pair_of: []usize,
    is_pickup: []bool,
    demand_signed: []i64,
    ready: []u32,
    due: []u32,
    service: []u32,
    n_pairs: usize,

    pub fn inst(self: *const RandInst) PdpInstance {
        return .{
            .n_pairs = self.n_pairs,
            .matrix = self.matrix,
            .capacity = 5,
            .pair_of = self.pair_of,
            .is_pickup = self.is_pickup,
            .demand_signed = self.demand_signed,
            .ready = self.ready,
            .due = self.due,
            .service = self.service,
        };
    }

    pub fn deinit(self: *RandInst, allocator: std.mem.Allocator) void {
        allocator.free(self.matrix);
        allocator.free(self.pair_of);
        allocator.free(self.is_pickup);
        allocator.free(self.demand_signed);
        allocator.free(self.ready);
        allocator.free(self.due);
        allocator.free(self.service);
        self.* = undefined;
    }
};

/// Directed matrix in [1,100], depot arcs biased larger so combining pairs pays.
/// demand q in [1,3] at pickups, -q at dropoffs. capacity = 5.
/// ready = 0, due = horizon for all (TW off) when `tw == false`;
/// otherwise windows derived from a feasible reference schedule so at least one
/// feasible solution provably exists.
// pub for cross-module tests (pdptw_sisr.zig); test-grade helper, not API.
pub fn randomInstance(allocator: std.mem.Allocator, n_pairs: usize, seed: u64, tw: bool) !RandInst {
    var prng = std.Random.DefaultPrng.init(seed);
    const rng = prng.random();

    const dim = 2 * n_pairs + 1;
    const matrix = try allocator.alloc(u32, dim * dim);
    errdefer allocator.free(matrix);
    const pair_of = try allocator.alloc(usize, dim);
    errdefer allocator.free(pair_of);
    const is_pickup = try allocator.alloc(bool, dim);
    errdefer allocator.free(is_pickup);
    const demand_signed = try allocator.alloc(i64, dim);
    errdefer allocator.free(demand_signed);
    const ready = try allocator.alloc(u32, dim);
    errdefer allocator.free(ready);
    const due = try allocator.alloc(u32, dim);
    errdefer allocator.free(due);
    const service = try allocator.alloc(u32, dim);
    errdefer allocator.free(service);

    const horizon: u32 = 100000;

    for (0..dim) |a| {
        for (0..dim) |b| {
            if (a == b) {
                matrix[a * dim + b] = 0;
            } else if (a == 0 or b == 0) {
                matrix[a * dim + b] = rng.intRangeAtMost(u32, 60, 100);
            } else {
                matrix[a * dim + b] = rng.intRangeAtMost(u32, 1, 30);
            }
        }
    }

    pair_of[0] = 0;
    is_pickup[0] = false;
    demand_signed[0] = 0;
    ready[0] = 0;
    due[0] = horizon;
    service[0] = 0;

    for (1..n_pairs + 1) |i| {
        const p = i;
        const q = n_pairs + i;
        pair_of[p] = q;
        pair_of[q] = p;
        is_pickup[p] = true;
        is_pickup[q] = false;
        const demand = rng.intRangeAtMost(i64, 1, 3);
        demand_signed[p] = demand;
        demand_signed[q] = -demand;
        service[p] = 0;
        service[q] = 0;
        ready[p] = 0;
        ready[q] = 0;
        due[p] = horizon;
        due[q] = horizon;
    }

    if (tw) {
        const slack: u32 = 40;
        var dim_getter: PdpInstance = .{
            .n_pairs = n_pairs,
            .matrix = matrix,
            .capacity = 5,
            .pair_of = pair_of,
            .is_pickup = is_pickup,
            .demand_signed = demand_signed,
            .ready = ready,
            .due = due,
            .service = service,
        };
        due[0] = horizon;
        for (1..n_pairs + 1) |i| {
            const p = i;
            const q = n_pairs + i;
            const arrive_p = dim_getter.d(0, p);
            const arrive_q = arrive_p + dim_getter.d(p, q);
            ready[p] = 0;
            due[p] = @as(u32, @intCast(arrive_p)) + slack;
            ready[q] = 0;
            due[q] = @as(u32, @intCast(arrive_q)) + slack;
        }
    }

    return .{
        .matrix = matrix,
        .pair_of = pair_of,
        .is_pickup = is_pickup,
        .demand_signed = demand_signed,
        .ready = ready,
        .due = due,
        .service = service,
        .n_pairs = n_pairs,
    };
}

test "PDPTW Lseg: empty() is the merge identity" {
    var prng = std.Random.DefaultPrng.init(42);
    const rng = prng.random();
    for (0..50) |_| {
        const q: i64 = rng.intRangeAtMost(i64, -5, 5);
        const seg = Lseg{ .delta = q, .lo = @min(0, q), .hi = @max(0, q) };
        const left = Lseg.merge(Lseg.empty(), seg);
        const right = Lseg.merge(seg, Lseg.empty());
        try std.testing.expectEqual(seg.delta, left.delta);
        try std.testing.expectEqual(seg.lo, left.lo);
        try std.testing.expectEqual(seg.hi, left.hi);
        try std.testing.expectEqual(seg.delta, right.delta);
        try std.testing.expectEqual(seg.lo, right.lo);
        try std.testing.expectEqual(seg.hi, right.hi);
    }
}

fn randomSignedRoute(rng: std.Random, allocator: std.mem.Allocator, len: usize) ![]i64 {
    const xs = try allocator.alloc(i64, len);
    for (xs) |*x| x.* = rng.intRangeAtMost(i64, -5, 5);
    return xs;
}

fn lsegFromDeltas(deltas: []const i64) Lseg {
    var acc = Lseg.empty();
    for (deltas) |q| {
        const seg = Lseg{ .delta = q, .lo = @min(0, q), .hi = @max(0, q) };
        acc = Lseg.merge(acc, seg);
    }
    return acc;
}

test "PDPTW Lseg: merge matches a brute prefix scan on 1000 random routes" {
    const allocator = std.testing.allocator;
    var prng = std.Random.DefaultPrng.init(7);
    const rng = prng.random();

    for (0..1000) |_| {
        const len = rng.intRangeAtMost(usize, 1, 20);
        const deltas = try randomSignedRoute(rng, allocator, len);
        defer allocator.free(deltas);

        const seg = lsegFromDeltas(deltas);

        var sum: i64 = 0;
        var lo: i64 = 0;
        var hi: i64 = 0;
        var prefix: i64 = 0;
        for (deltas) |q| {
            prefix += q;
            lo = @min(lo, prefix);
            hi = @max(hi, prefix);
        }
        sum = prefix;

        try std.testing.expectEqual(sum, seg.delta);
        try std.testing.expectEqual(lo, seg.lo);
        try std.testing.expectEqual(hi, seg.hi);
    }
}

test "PDPTW Lseg: merge is associative at every split point" {
    const allocator = std.testing.allocator;
    var prng = std.Random.DefaultPrng.init(99);
    const rng = prng.random();

    for (0..200) |_| {
        const len = rng.intRangeAtMost(usize, 1, 15);
        const deltas = try randomSignedRoute(rng, allocator, len);
        defer allocator.free(deltas);

        const whole = lsegFromDeltas(deltas);

        for (0..len + 1) |k| {
            const a = lsegFromDeltas(deltas[0..k]);
            const b = lsegFromDeltas(deltas[k..]);
            const merged = Lseg.merge(a, b);
            try std.testing.expectEqual(whole.delta, merged.delta);
            try std.testing.expectEqual(whole.lo, merged.lo);
            try std.testing.expectEqual(whole.hi, merged.hi);
        }
    }
}

test "PDPTW routeCost: agrees with validate on random single-route solutions" {
    const allocator = std.testing.allocator;
    var ri = try randomInstance(allocator, 4, 123, true);
    defer ri.deinit(allocator);
    const inst = ri.inst();
    const dim = inst.dim();

    const pos = try allocator.alloc(usize, dim);
    defer allocator.free(pos);
    @memset(pos, 0);

    const perm = try allocator.alloc(usize, dim - 1);
    defer allocator.free(perm);
    for (0..dim - 1) |i| perm[i] = i + 1;

    var prng = std.Random.DefaultPrng.init(55);
    const rng = prng.random();

    var saw_feasible = false;
    var saw_infeasible = false;

    for (0..500) |_| {
        rng.shuffle(usize, perm);
        const rc = routeCost(inst, perm, pos);
        const routes = [_][]const usize{perm};
        const vc = validate(inst, routes[0..]);
        try std.testing.expectEqual(vc, rc);
        if (rc != null) saw_feasible = true;
        if (rc == null) saw_infeasible = true;
    }

    try std.testing.expect(saw_feasible);
    try std.testing.expect(saw_infeasible);
}

test "PDPTW validate: accepts the valid fixture and recomputes its cost" {
    const f = fixture2();
    const inst = f.inst(5);

    const r1a = [_]usize{ 1, 3 };
    const r1b = [_]usize{ 2, 4 };
    const routes1 = [_][]const usize{ r1a[0..], r1b[0..] };
    try std.testing.expectEqual(@as(?u64, 101), validate(inst, routes1[0..]));

    const r2 = [_]usize{ 1, 2, 4, 3 };
    const routes2 = [_][]const usize{r2[0..]};
    try std.testing.expectEqual(@as(?u64, 57), validate(inst, routes2[0..]));
}

test "PDPTW validate: rejects precedence violation" {
    const f = fixture2();
    const inst = f.inst(5);
    const r = [_]usize{ 2, 3, 1, 4 };
    const routes = [_][]const usize{r[0..]};
    try std.testing.expectEqual(@as(?u64, null), validate(inst, routes[0..]));
}

test "PDPTW validate: rejects pairing split across routes" {
    const f = fixture2();
    const inst = f.inst(5);
    const r1 = [_]usize{ 1, 4 };
    const r2 = [_]usize{ 2, 3 };
    const routes = [_][]const usize{ r1[0..], r2[0..] };
    try std.testing.expectEqual(@as(?u64, null), validate(inst, routes[0..]));
}

test "PDPTW validate: rejects capacity prefix overload (route total is zero)" {
    const f = fixture2();
    const inst_bad = f.inst(3);
    const r = [_]usize{ 1, 2, 4, 3 };
    const routes = [_][]const usize{r[0..]};
    try std.testing.expectEqual(@as(?u64, null), validate(inst_bad, routes[0..]));

    // this one still passes at capacity 3 (max prefix 2)
    const r1a = [_]usize{ 1, 3 };
    const r1b = [_]usize{ 2, 4 };
    const routes2 = [_][]const usize{ r1a[0..], r1b[0..] };
    try std.testing.expectEqual(@as(?u64, 101), validate(inst_bad, routes2[0..]));
}

test "PDPTW validate: rejects time-window violation" {
    var f = fixture2();
    f.due[4] = 1;
    const inst = f.inst(5);
    const r1a = [_]usize{ 1, 3 };
    const r1b = [_]usize{ 2, 4 };
    const routes = [_][]const usize{ r1a[0..], r1b[0..] };
    try std.testing.expectEqual(@as(?u64, null), validate(inst, routes[0..]));
}

test "PDPTW validate: rejects late depot return" {
    var f = fixture2();
    f.due[0] = 1;
    const inst = f.inst(5);
    const r1a = [_]usize{ 1, 3 };
    const r1b = [_]usize{ 2, 4 };
    const routes = [_][]const usize{ r1a[0..], r1b[0..] };
    try std.testing.expectEqual(@as(?u64, null), validate(inst, routes[0..]));
}

test "PDPTW validate: rejects unvisited, duplicate, and depot-in-route" {
    const f = fixture2();
    const inst = f.inst(5);

    // unvisited
    {
        const r = [_]usize{ 1, 3 };
        const routes = [_][]const usize{r[0..]};
        try std.testing.expectEqual(@as(?u64, null), validate(inst, routes[0..]));
    }
    // duplicate
    {
        const r1 = [_]usize{ 1, 3 };
        const r2 = [_]usize{ 1, 3 };
        const routes = [_][]const usize{ r1[0..], r2[0..] };
        try std.testing.expectEqual(@as(?u64, null), validate(inst, routes[0..]));
    }
    // depot in route
    {
        const r1 = [_]usize{ 0, 1, 3 };
        const r2 = [_]usize{ 2, 4 };
        const routes = [_][]const usize{ r1[0..], r2[0..] };
        try std.testing.expectEqual(@as(?u64, null), validate(inst, routes[0..]));
    }
}

// ---------------------------------------------------------------------------
// T4 — construction
// ---------------------------------------------------------------------------

pub const Sol = std.ArrayList(std.ArrayList(usize));

pub fn freeSol(allocator: std.mem.Allocator, sol: *Sol) void {
    for (sol.items) |*r| r.deinit(allocator);
    sol.deinit(allocator);
}

// pub for pdptw_sisr.zig / probes; not part of the stable API.
pub fn collectPickups(allocator: std.mem.Allocator, inst: PdpInstance) ![]usize {
    var out: std.ArrayList(usize) = .empty;
    errdefer out.deinit(allocator);
    const dim = inst.dim();
    var c: usize = 1;
    while (c < dim) : (c += 1) {
        if (inst.is_pickup[c]) try out.append(allocator, c);
    }
    return out.toOwnedSlice(allocator);
}

fn solSlices(allocator: std.mem.Allocator, sol: *const Sol) ![][]const usize {
    var out: std.ArrayList([]const usize) = .empty;
    errdefer out.deinit(allocator);
    for (sol.items) |r| {
        if (r.items.len != 0) try out.append(allocator, r.items);
    }
    return out.toOwnedSlice(allocator);
}

/// Build the (len+2)-element candidate `route` with `p` inserted at gap `i` and `q`
/// inserted at gap `j` (`j` measured in the array that already contains `p`, so
/// `i < j`). `out` is cleared and filled; `i`/`j` are final positions since p's slot
/// never moves when q is inserted after it.
fn buildInsertion(allocator: std.mem.Allocator, out: *std.ArrayList(usize), route: []const usize, p: usize, q: usize, i: usize, j: usize) !void {
    out.clearRetainingCapacity();
    const len = route.len;
    var oi: usize = 0;
    var out_idx: usize = 0;
    while (out_idx < len + 2) : (out_idx += 1) {
        if (out_idx == i) {
            try out.append(allocator, p);
        } else if (out_idx == j) {
            try out.append(allocator, q);
        } else {
            try out.append(allocator, route[oi]);
            oi += 1;
        }
    }
}

const NO_ARRIVAL: i64 = -(1 << 62);

const RouteLabels = struct {
    depart: std.ArrayList(i64) = .empty,
    load: std.ArrayList(i64) = .empty,
    pre_d: std.ArrayList(u64) = .empty,
    latest: std.ArrayList(i64) = .empty,
    tail_d: std.ArrayList(u64) = .empty,

    fn deinit(self: *RouteLabels, allocator: std.mem.Allocator) void {
        self.depart.deinit(allocator);
        self.load.deinit(allocator);
        self.pre_d.deinit(allocator);
        self.latest.deinit(allocator);
        self.tail_d.deinit(allocator);
    }

    fn fill(self: *RouteLabels, allocator: std.mem.Allocator, inst: PdpInstance, it: []const usize) !void {
        const L = it.len;
        try self.depart.resize(allocator, L + 1);
        try self.load.resize(allocator, L + 1);
        try self.pre_d.resize(allocator, L + 1);
        try self.latest.resize(allocator, L + 1);
        try self.tail_d.resize(allocator, L + 1);
        self.depart.items[0] = 0;
        self.load.items[0] = 0;
        self.pre_d.items[0] = 0;
        var prev: usize = 0;
        for (it, 0..) |c, k| {
            const arc: i64 = @intCast(inst.d(prev, c));
            const start = @max(self.depart.items[k] + arc, @as(i64, @intCast(inst.ready[c])));
            self.depart.items[k + 1] = start + @as(i64, @intCast(inst.service[c]));
            self.load.items[k + 1] = self.load.items[k] + inst.demand_signed[c];
            self.pre_d.items[k + 1] = self.pre_d.items[k] + inst.d(prev, c);
            prev = c;
        }
        self.latest.items[L] = @intCast(inst.due[0]);
        self.tail_d.items[L] = 0;
        var k = L;
        while (k > 0) {
            k -= 1;
            const c = it[k];
            const nxt: usize = if (k + 1 == L) 0 else it[k + 1];
            const arc: i64 = @intCast(inst.d(c, nxt));
            const x = @min(@as(i64, @intCast(inst.due[c])), self.latest.items[k + 1] - @as(i64, @intCast(inst.service[c])) - arc);
            self.latest.items[k] = if (x < @as(i64, @intCast(inst.ready[c]))) NO_ARRIVAL else x;
            self.tail_d.items[k] = inst.d(c, nxt) + self.tail_d.items[k + 1];
        }
    }
};

// pub for pdptw_sisr.zig (seed solution); not part of the stable API.
pub fn construct(allocator: std.mem.Allocator, inst: PdpInstance, order: []const usize, pos: []usize) !Sol {
    var sol: Sol = .empty;
    errdefer freeSol(allocator, &sol);

    var labels: RouteLabels = .{};
    defer labels.deinit(allocator);

    for (order) |p| {
        const q = inst.pair_of[p];
        const p_dem = inst.demand_signed[p];
        const p_ready: i64 = @intCast(inst.ready[p]);
        const p_due: i64 = @intCast(inst.due[p]);
        const p_service: i64 = @intCast(inst.service[p]);
        const q_ready: i64 = @intCast(inst.ready[q]);
        const q_due: i64 = @intCast(inst.due[q]);
        const q_service: i64 = @intCast(inst.service[q]);

        var best_delta: i64 = 0;
        var best_ri: ?usize = null;
        var best_i: usize = 0;
        var best_j: usize = 0;
        var have_best = false;

        for (sol.items, 0..) |route, ri| {
            const it = route.items;
            const L = it.len;
            try labels.fill(allocator, inst, it);
            const depart_at = labels.depart.items;
            const load_at = labels.load.items;
            const pre_d = labels.pre_d.items;
            const latest = labels.latest.items;
            const tail_d = labels.tail_d.items;
            const base_dist: i64 = @intCast(inst.d(0, it[0]) + tail_d[0]);

            for (0..L + 1) |a| {
                const prev_a: usize = if (a == 0) 0 else it[a - 1];
                const p_start = @max(depart_at[a] + @as(i64, @intCast(inst.d(prev_a, p))), p_ready);
                if (p_start > p_due) continue;
                var depart = p_start + p_service;
                var load = load_at[a] + p_dem;
                if (load < 0 or load > inst.capacity) continue;
                var mid_d: u64 = pre_d[a] + inst.d(prev_a, p);
                var last = p;

                var b = a;
                while (b <= L) : (b += 1) {
                    const nxt: usize = if (b == L) 0 else it[b];
                    const q_start = @max(depart + @as(i64, @intCast(inst.d(last, q))), q_ready);
                    if (q_start <= q_due) {
                        const nxt_arrival = q_start + q_service + @as(i64, @intCast(inst.d(q, nxt)));
                        if (nxt_arrival <= latest[b]) {
                            const new_dist: i64 = @intCast(mid_d + inst.d(last, q) + inst.d(q, nxt) + tail_d[b]);
                            const delta = new_dist - base_dist;
                            if (!have_best or delta < best_delta) {
                                have_best = true;
                                best_delta = delta;
                                best_ri = ri;
                                best_i = a;
                                best_j = b + 1;
                            }
                        }
                    }
                    if (b < L) {
                        const c = it[b];
                        const start = @max(depart + @as(i64, @intCast(inst.d(last, c))), @as(i64, @intCast(inst.ready[c])));
                        if (start > inst.due[c]) break;
                        depart = start + @as(i64, @intCast(inst.service[c]));
                        load += inst.demand_signed[c];
                        if (load < 0 or load > inst.capacity) break;
                        mid_d += inst.d(last, c);
                        last = c;
                    }
                }
            }
        }

        if (have_best) {
            const ri = best_ri.?;
            const route = &sol.items[ri];
            var newr: std.ArrayList(usize) = .empty;
            errdefer newr.deinit(allocator);
            try buildInsertion(allocator, &newr, route.items, p, q, best_i, best_j);
            route.deinit(allocator);
            route.* = newr;
        } else {
            const pair = [_]usize{ p, q };
            if (routeCost(inst, pair[0..], pos) == null) return error.InfeasibleInstance;
            var newr: std.ArrayList(usize) = .empty;
            errdefer newr.deinit(allocator);
            try newr.append(allocator, p);
            try newr.append(allocator, q);
            try sol.append(allocator, newr);
        }
    }

    return sol;
}

fn referenceConstruct(allocator: std.mem.Allocator, inst: PdpInstance, order: []const usize, pos: []usize) !Sol {
    var sol: Sol = .empty;
    errdefer freeSol(allocator, &sol);
    var cand: std.ArrayList(usize) = .empty;
    defer cand.deinit(allocator);
    for (order) |p| {
        const q = inst.pair_of[p];
        var best_delta: i64 = 0;
        var best_ri: ?usize = null;
        var best_i: usize = 0;
        var best_j: usize = 0;
        var have_best = false;
        for (sol.items, 0..) |route, ri| {
            const c_old = routeCost(inst, route.items, pos) orelse unreachable;
            const len = route.items.len;
            var i: usize = 0;
            while (i <= len) : (i += 1) {
                var j: usize = i + 1;
                while (j <= len + 1) : (j += 1) {
                    try buildInsertion(allocator, &cand, route.items, p, q, i, j);
                    const c_new = routeCost(inst, cand.items, pos) orelse continue;
                    const delta = @as(i64, @intCast(c_new)) - @as(i64, @intCast(c_old));
                    if (!have_best or delta < best_delta) {
                        have_best = true;
                        best_delta = delta;
                        best_ri = ri;
                        best_i = i;
                        best_j = j;
                    }
                }
            }
        }
        if (have_best) {
            const ri = best_ri.?;
            const route = &sol.items[ri];
            var newr: std.ArrayList(usize) = .empty;
            errdefer newr.deinit(allocator);
            try buildInsertion(allocator, &newr, route.items, p, q, best_i, best_j);
            route.deinit(allocator);
            route.* = newr;
        } else {
            const pair = [_]usize{ p, q };
            if (routeCost(inst, pair[0..], pos) == null) return error.InfeasibleInstance;
            var newr: std.ArrayList(usize) = .empty;
            errdefer newr.deinit(allocator);
            try newr.append(allocator, p);
            try newr.append(allocator, q);
            try sol.append(allocator, newr);
        }
    }
    return sol;
}

fn expectSameSol(a: *const Sol, b: *const Sol) !void {
    try std.testing.expectEqual(a.items.len, b.items.len);
    for (a.items, b.items) |ra, rb| try std.testing.expectEqualSlices(usize, ra.items, rb.items);
}

test "PDPTW construct: route-for-route identical to the reference routeCost construction" {
    const allocator = std.testing.allocator;
    var checked: usize = 0;
    var seed: u64 = 1;
    while (seed <= 40) : (seed += 1) {
        for ([_]bool{ false, true }) |clocked| {
            var ri = if (clocked) try randomInstanceClocked(allocator, 8, seed) else try randomInstance(allocator, 8, seed, seed % 2 == 0);
            defer ri.deinit(allocator);
            const inst = ri.inst();
            const pos = try allocator.alloc(usize, inst.dim());
            defer allocator.free(pos);
            @memset(pos, 0);
            const pickups = try collectPickups(allocator, inst);
            defer allocator.free(pickups);
            var prng = std.Random.DefaultPrng.init(seed);
            prng.random().shuffle(usize, pickups);
            var fast = try construct(allocator, inst, pickups, pos);
            defer freeSol(allocator, &fast);
            var ref = try referenceConstruct(allocator, inst, pickups, pos);
            defer freeSol(allocator, &ref);
            try expectSameSol(&fast, &ref);
            checked += 1;
        }
    }
    try std.testing.expectEqual(@as(usize, 80), checked);
}

test "PDPTW construct: output passes validate on the hand fixture" {
    const allocator = std.testing.allocator;
    const f = fixture2();
    const inst = f.inst(5);
    const pos = try allocator.alloc(usize, inst.dim());
    defer allocator.free(pos);
    @memset(pos, 0);

    const order = [_]usize{ 1, 2 };
    var sol = try construct(allocator, inst, order[0..], pos);
    defer freeSol(allocator, &sol);

    const slices = try solSlices(allocator, &sol);
    defer allocator.free(slices);

    try std.testing.expect(validate(inst, slices) != null);
}

// ---------------------------------------------------------------------------
// T5 — pair-relocate local search
// ---------------------------------------------------------------------------

/// Build `out` as `route` with both `p` and `q` removed (order preserved).
fn buildRemoval(allocator: std.mem.Allocator, out: *std.ArrayList(usize), route: []const usize, p: usize, q: usize) !void {
    out.clearRetainingCapacity();
    for (route) |c| {
        if (c == p or c == q) continue;
        try out.append(allocator, c);
    }
}

fn pairRelocate(allocator: std.mem.Allocator, inst: PdpInstance, sol: *Sol, pos: []usize, max_passes: usize) !void {
    var base: std.ArrayList(usize) = .empty;
    defer base.deinit(allocator);
    var cand: std.ArrayList(usize) = .empty;
    defer cand.deinit(allocator);
    var newr: std.ArrayList(usize) = .empty;
    defer newr.deinit(allocator);

    var pass: usize = 0;
    while (pass < max_passes) : (pass += 1) {
        var applied = false;

        ri_loop: for (sol.items, 0..) |route, ri| {
            const c_ri = routeCost(inst, route.items, pos) orelse unreachable;

            var pidx: usize = 0;
            while (pidx < route.items.len) : (pidx += 1) {
                const p = route.items[pidx];
                if (!inst.is_pickup[p]) continue;
                const q = inst.pair_of[p];

                try buildRemoval(allocator, &base, route.items, p, q);
                const c_base = routeCost(inst, base.items, pos) orelse continue;

                for (sol.items, 0..) |rj_route, rj| {
                    const target: []const usize = if (rj == ri) base.items else rj_route.items;
                    const c_rj: u64 = if (rj == ri) c_ri else routeCost(inst, rj_route.items, pos) orelse unreachable;

                    const len = target.len;
                    var i: usize = 0;
                    while (i <= len) : (i += 1) {
                        var j: usize = i + 1;
                        while (j <= len + 1) : (j += 1) {
                            try buildInsertion(allocator, &cand, target, p, q, i, j);
                            const c_new = routeCost(inst, cand.items, pos) orelse continue;

                            const gain: i64 = if (rj == ri)
                                @as(i64, @intCast(c_new)) - @as(i64, @intCast(c_ri))
                            else
                                (@as(i64, @intCast(c_base)) + @as(i64, @intCast(c_new))) -
                                    (@as(i64, @intCast(c_ri)) + @as(i64, @intCast(c_rj)));

                            if (gain < 0) {
                                // apply: ri := base, rj := cand
                                try newr.resize(allocator, base.items.len);
                                @memcpy(newr.items, base.items);
                                sol.items[ri].deinit(allocator);
                                sol.items[ri] = newr;
                                newr = .empty;

                                try newr.resize(allocator, cand.items.len);
                                @memcpy(newr.items, cand.items);
                                sol.items[rj].deinit(allocator);
                                sol.items[rj] = newr;
                                newr = .empty;

                                applied = true;
                                break :ri_loop;
                            }
                        }
                    }
                }
            }
        }

        if (!applied) break;
    }

    // Drop empty routes once, at the very end.
    var i: usize = 0;
    while (i < sol.items.len) {
        if (sol.items[i].items.len == 0) {
            sol.items[i].deinit(allocator);
            _ = sol.orderedRemove(i);
        } else {
            i += 1;
        }
    }
}

test "PDPTW pairRelocate: never worsens the construction cost" {
    const allocator = std.testing.allocator;
    var seed: u64 = 1;
    while (seed <= 29) : (seed += 1) {
        var ri = try randomInstance(allocator, 5, seed, true);
        defer ri.deinit(allocator);
        const inst = ri.inst();

        const pos = try allocator.alloc(usize, inst.dim());
        defer allocator.free(pos);
        @memset(pos, 0);

        const pickups = try collectPickups(allocator, inst);
        defer allocator.free(pickups);

        const Ctx = struct {
            inst: PdpInstance,
            fn lessThan(ctx: @This(), a: usize, b: usize) bool {
                if (ctx.inst.due[a] != ctx.inst.due[b]) return ctx.inst.due[a] < ctx.inst.due[b];
                return a < b;
            }
        };
        std.mem.sort(usize, pickups, Ctx{ .inst = inst }, Ctx.lessThan);

        var sol = try construct(allocator, inst, pickups, pos);
        defer freeSol(allocator, &sol);

        const before_slices = try solSlices(allocator, &sol);
        const before = validate(inst, before_slices).?;
        allocator.free(before_slices);

        try pairRelocate(allocator, inst, &sol, pos, 1000);

        const after_slices = try solSlices(allocator, &sol);
        defer allocator.free(after_slices);
        const after = validate(inst, after_slices).?;

        try std.testing.expect(after <= before);
    }
}

test "PDPTW pairRelocate: output stays feasible on 50 random instances" {
    const allocator = std.testing.allocator;
    var seed: u64 = 1;
    while (seed <= 50) : (seed += 1) {
        var ri = try randomInstance(allocator, 5, seed, true);
        defer ri.deinit(allocator);
        const inst = ri.inst();

        const pos = try allocator.alloc(usize, inst.dim());
        defer allocator.free(pos);
        @memset(pos, 0);

        const pickups = try collectPickups(allocator, inst);
        defer allocator.free(pickups);

        const Ctx = struct {
            inst: PdpInstance,
            fn lessThan(ctx: @This(), a: usize, b: usize) bool {
                if (ctx.inst.due[a] != ctx.inst.due[b]) return ctx.inst.due[a] < ctx.inst.due[b];
                return a < b;
            }
        };
        std.mem.sort(usize, pickups, Ctx{ .inst = inst }, Ctx.lessThan);

        var sol = try construct(allocator, inst, pickups, pos);
        defer freeSol(allocator, &sol);

        try pairRelocate(allocator, inst, &sol, pos, 1000);

        const slices = try solSlices(allocator, &sol);
        defer allocator.free(slices);

        try std.testing.expect(validate(inst, slices) != null);
    }
}

// ---------------------------------------------------------------------------
// T6 — solver entry point
// ---------------------------------------------------------------------------

pub const PdpResult = struct {
    allocator: std.mem.Allocator,
    routes: [][]usize,
    total_cost: u64,
    vehicles: usize,
    // Per-route vehicle-type index (heterogeneous fleet). null for uniform-fleet
    // solves; when set, types.len == routes.len and types[r] is the index into
    // the caller's veh_types of the vehicle assigned to route r.
    types: ?[]usize = null,

    pub fn deinit(self: *PdpResult) void {
        for (self.routes) |r| self.allocator.free(r);
        self.allocator.free(self.routes);
        if (self.types) |t| self.allocator.free(t);
        self.* = undefined;
    }
};

/// Naive segment cost for the brute-force optimum: pairing check (O(len^2)),
/// prefix-load walk, forward-time walk, depot return. Returns null if infeasible.
fn segCost(inst: PdpInstance, seg: []const usize) ?u64 {
    if (seg.len == 0) return 0;

    // pairing: every node's pair must also be in this segment
    for (seg) |c| {
        var found = false;
        for (seg) |c2| {
            if (c2 == inst.pair_of[c]) {
                found = true;
                break;
            }
        }
        if (!found) return null;
    }

    // capacity: prefix load walk
    var load: i64 = 0;
    for (seg) |c| {
        load += inst.demand_signed[c];
        if (load < 0 or load > inst.capacity) return null;
    }

    // TW + distance
    var t: u64 = 0;
    var prev: usize = 0;
    var cost: u64 = 0;
    for (seg) |c| {
        const arc = inst.d(prev, c);
        const arrive = t + arc;
        const start = @max(arrive, @as(u64, inst.ready[c]));
        if (start > inst.due[c]) return null;
        t = start + inst.service[c];
        cost += arc;
        prev = c;
    }
    const back = inst.d(prev, 0);
    t += back;
    cost += back;
    if (t > inst.due[0]) return null;

    return cost;
}

/// Exhaustive optimum for tiny instances (n_pairs <= 4). Test-only.
// pub for cross-module tests (pdptw_sisr.zig); test-grade helper, not API.
pub fn bruteForceOptimum(allocator: std.mem.Allocator, inst: PdpInstance) !u64 {
    const n = 2 * inst.n_pairs;
    const perm = try allocator.alloc(usize, n);
    defer allocator.free(perm);
    const used = try allocator.alloc(bool, inst.dim());
    defer allocator.free(used);
    @memset(used, false);

    var best: ?u64 = null;

    const Rec = struct {
        fn go(ins: PdpInstance, pm: []usize, us: []bool, depth: usize, bst: *?u64) void {
            if (depth == pm.len) {
                const mask_count = pm.len - 1;
                var mask: usize = 0;
                while (mask < (@as(usize, 1) << @intCast(mask_count))) : (mask += 1) {
                    var total: u64 = 0;
                    var feasible = true;
                    var start: usize = 0;
                    var i: usize = 0;
                    while (i < mask_count) : (i += 1) {
                        if ((mask >> @intCast(i)) & 1 == 1) {
                            const seg = pm[start .. i + 1];
                            const c = segCost(ins, seg) orelse {
                                feasible = false;
                                break;
                            };
                            total += c;
                            start = i + 1;
                        }
                    }
                    if (feasible) {
                        const seg = pm[start..pm.len];
                        const c = segCost(ins, seg) orelse continue;
                        total += c;
                        if (bst.* == null or total < bst.*.?) bst.* = total;
                    }
                }
                return;
            }
            var c: usize = 1;
            while (c < ins.dim()) : (c += 1) {
                if (us[c]) continue;
                if (!ins.is_pickup[c] and !us[ins.pair_of[c]]) continue;
                us[c] = true;
                pm[depth] = c;
                go(ins, pm, us, depth + 1, bst);
                us[c] = false;
            }
        }
    };

    Rec.go(inst, perm, used, 0, &best);
    return best orelse error.InfeasibleInstance;
}

test "PDPTW construct: output passes validate on 50 random instances" {
    const allocator = std.testing.allocator;
    var seed: u64 = 1;
    while (seed <= 50) : (seed += 1) {
        var ri = try randomInstance(allocator, 5, seed, true);
        defer ri.deinit(allocator);
        const inst = ri.inst();

        const pos = try allocator.alloc(usize, inst.dim());
        defer allocator.free(pos);
        @memset(pos, 0);

        const pickups = try collectPickups(allocator, inst);
        defer allocator.free(pickups);

        const Ctx = struct {
            inst: PdpInstance,
            fn lessThan(ctx: @This(), a: usize, b: usize) bool {
                if (ctx.inst.due[a] != ctx.inst.due[b]) return ctx.inst.due[a] < ctx.inst.due[b];
                return a < b;
            }
        };
        std.mem.sort(usize, pickups, Ctx{ .inst = inst }, Ctx.lessThan);

        var sol = try construct(allocator, inst, pickups, pos);
        defer freeSol(allocator, &sol);

        const slices = try solSlices(allocator, &sol);
        defer allocator.free(slices);

        try std.testing.expect(validate(inst, slices) != null);
    }
}

test "PDPTW pairRelocate: strictly improves construction on known seeds" {
    // Kills a no-op local search. These seeds were measured to have an
    // improving pair-relocate move after construction (deterministic).
    const allocator = std.testing.allocator;
    for ([_]u64{ 1, 4, 5, 7, 8 }) |seed| {
        var ri = try randomInstance(allocator, 5, seed, true);
        defer ri.deinit(allocator);
        const inst = ri.inst();

        const pos = try allocator.alloc(usize, inst.dim());
        defer allocator.free(pos);
        @memset(pos, 0);

        const pickups = try collectPickups(allocator, inst);
        defer allocator.free(pickups);
        const Ctx = struct {
            inst: PdpInstance,
            fn lessThan(ctx: @This(), a: usize, b: usize) bool {
                if (ctx.inst.due[a] != ctx.inst.due[b]) return ctx.inst.due[a] < ctx.inst.due[b];
                return a < b;
            }
        };
        std.mem.sort(usize, pickups, Ctx{ .inst = inst }, Ctx.lessThan);

        var sol = try construct(allocator, inst, pickups, pos);
        defer freeSol(allocator, &sol);

        const bs = try solSlices(allocator, &sol);
        const before = validate(inst, bs).?;
        allocator.free(bs);

        try pairRelocate(allocator, inst, &sol, pos, 1000);

        const as = try solSlices(allocator, &sol);
        defer allocator.free(as);
        const after = validate(inst, as).?;

        try std.testing.expect(after < before);
    }
}

test "PDPTW clock: waiting and service shift the schedule in all three checkers" {
    // ready[2]=25 forces waiting (arrive at 2 at t=15), service[2]=4 delays
    // departure. On route {1,2,4,3}: arrive at 4 at t = 25+4+8 = 37. A clock
    // missing service gives 33, missing waiting 27, missing both 23 - so the
    // due[4]=36 boundary rejects ONLY under the correct arithmetic, in
    // validate, routeCost, and (via bruteForceOptimum) segCost alike.
    const allocator = std.testing.allocator;
    var f = fixture2();
    f.ready[2] = 25;
    f.service[2] = 4;

    const pos = try allocator.alloc(usize, 5);
    defer allocator.free(pos);
    @memset(pos, 0);

    const r = [_]usize{ 1, 2, 4, 3 };
    const routes = [_][]const usize{r[0..]};

    // boundary-feasible: start at 4 exactly at due[4]=37; cost excludes wait+service
    f.due[4] = 37;
    {
        const inst = f.inst(5);
        try std.testing.expectEqual(@as(?u64, 57), validate(inst, routes[0..]));
        try std.testing.expectEqual(@as(?u64, 57), routeCost(inst, r[0..], pos));
        try std.testing.expectEqual(@as(u64, 57), try bruteForceOptimum(allocator, inst));
    }

    // one unit tighter: infeasible under the true clock, feasible under any stub.
    // The split {(1,3),(2,4)} also waits at 2 and reaches 4 at 37 > 36, so the
    // whole instance is infeasible and segCost must prove it.
    f.due[4] = 36;
    {
        const inst = f.inst(5);
        try std.testing.expectEqual(@as(?u64, null), validate(inst, routes[0..]));
        try std.testing.expectEqual(@as(?u64, null), routeCost(inst, r[0..], pos));
        try std.testing.expectError(error.InfeasibleInstance, bruteForceOptimum(allocator, inst));
    }

    // depot-return leg carries the waiting+service delay: t back = 40+31 = 71
    f.due[4] = 37;
    f.due[0] = 71;
    {
        const inst = f.inst(5);
        try std.testing.expectEqual(@as(?u64, 57), validate(inst, routes[0..]));
        try std.testing.expectEqual(@as(?u64, 57), routeCost(inst, r[0..], pos));
    }
    f.due[0] = 70;
    {
        const inst = f.inst(5);
        try std.testing.expectEqual(@as(?u64, null), validate(inst, routes[0..]));
        try std.testing.expectEqual(@as(?u64, null), routeCost(inst, r[0..], pos));
    }
}

/// randomInstance, then nonzero service times and dropoff ready-windows that
/// force waiting. Reference solution (one route {p,q} per pair) stays feasible
/// by construction: due is derived from its schedule including wait + service.
pub fn randomInstanceClocked(allocator: std.mem.Allocator, n_pairs: usize, seed: u64) !RandInst {
    var ri = try randomInstance(allocator, n_pairs, seed, true);
    var prng = std.Random.DefaultPrng.init(seed ^ 0xC10C);
    const rng = prng.random();
    const slack: u32 = 40;
    const inst = ri.inst();
    for (1..n_pairs + 1) |i| {
        const p = i;
        const q = n_pairs + i;
        ri.service[p] = rng.intRangeAtMost(u32, 1, 10);
        ri.service[q] = rng.intRangeAtMost(u32, 1, 10);
        const arrive_p: u32 = @intCast(inst.d(0, p));
        const arrive_q = arrive_p + ri.service[p] + @as(u32, @intCast(inst.d(p, q)));
        ri.ready[q] = arrive_q + rng.intRangeAtMost(u32, 1, 15); // waiting always binds
        ri.due[p] = arrive_p + slack;
        ri.due[q] = ri.ready[q] + slack;
    }
    return ri;
}

test "PDPTW routeCost: agrees with validate under nonzero service and waiting" {
    const allocator = std.testing.allocator;
    var seed: u64 = 1;
    while (seed <= 3) : (seed += 1) {
        var ri = try randomInstanceClocked(allocator, 4, seed);
        defer ri.deinit(allocator);
        const inst = ri.inst();
        const dim = inst.dim();

        const pos = try allocator.alloc(usize, dim);
        defer allocator.free(pos);
        @memset(pos, 0);

        // the guaranteed-feasible reference: one route {p, q} per pair, with
        // routeCost and validate agreeing on it route by route and in total
        var total: u64 = 0;
        for (1..inst.n_pairs + 1) |i| {
            const r = [_]usize{ i, inst.n_pairs + i };
            const rc = routeCost(inst, r[0..], pos);
            try std.testing.expect(rc != null);
            total += rc.?;
        }
        const ref = [_][]const usize{ &.{ 1, 5 }, &.{ 2, 6 }, &.{ 3, 7 }, &.{ 4, 8 } };
        try std.testing.expectEqual(@as(?u64, total), validate(inst, ref[0..]));

        // differential on random full-permutation single routes
        const perm = try allocator.alloc(usize, dim - 1);
        defer allocator.free(perm);
        for (0..dim - 1) |i| perm[i] = i + 1;
        var prng = std.Random.DefaultPrng.init(seed *% 7919);
        const rng = prng.random();
        var saw_infeasible = false;
        for (0..500) |_| {
            rng.shuffle(usize, perm);
            const routes = [_][]const usize{perm};
            try std.testing.expectEqual(validate(inst, routes[0..]), routeCost(inst, perm, pos));
            if (routeCost(inst, perm, pos) == null) saw_infeasible = true;
        }
        try std.testing.expect(saw_infeasible);
    }
}
