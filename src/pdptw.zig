const std = @import("std");

// Pickup-and-Delivery Problem with Time Windows (PDPTW). Each customer is
// either a pickup or its paired dropoff (nodes 1..2*n_pairs; node 0 is the
// depot). A route must carry every pickup before its dropoff, in the same
// route, and must never violate vehicle capacity (walking the prefix load)
// or the per-node time window [ready, due] under free waiting. Travel time
// is a directed distance matrix, so real asymmetric road times work
// directly. Objective: minimize total directed travel distance over
// feasible routes. This file currently provides only the instance type and
// an independent feasibility/cost oracle (`validate`); the Lseg capacity
// algebra and the construction/local-search solver land in later tasks.

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

/// Load-segment algebra: O(1)-concat capacity primitive. `lo`/`hi` are the
/// min/max prefix sums over the segment, including the empty prefix (0).
pub const Lseg = struct {
    delta: i64, // net load change over the segment
    lo: i64, // min prefix sum (<= 0)
    hi: i64, // max prefix sum (>= 0)

    pub fn empty() Lseg {
        return .{ .delta = 0, .lo = 0, .hi = 0 };
    }

    pub fn node(inst: PdpInstance, c: usize) Lseg {
        const q = inst.demand_signed[c];
        return .{ .delta = q, .lo = @min(0, q), .hi = @max(0, q) };
    }

    /// Concatenate `a` then `b`.
    pub fn merge(a: Lseg, b: Lseg) Lseg {
        return .{
            .delta = a.delta + b.delta,
            .lo = @min(a.lo, a.delta + b.lo),
            .hi = @max(a.hi, a.delta + b.hi),
        };
    }

    /// Feasible from a starting load of 0 under `capacity`.
    pub fn feasible(self: Lseg, capacity: i64) bool {
        return self.lo >= 0 and self.hi <= capacity;
    }
};

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

const RandInst = struct {
    matrix: []u32,
    pair_of: []usize,
    is_pickup: []bool,
    demand_signed: []i64,
    ready: []u32,
    due: []u32,
    service: []u32,
    n_pairs: usize,

    fn inst(self: *const RandInst) PdpInstance {
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

    fn deinit(self: *RandInst, allocator: std.mem.Allocator) void {
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
fn randomInstance(allocator: std.mem.Allocator, n_pairs: usize, seed: u64, tw: bool) !RandInst {
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
