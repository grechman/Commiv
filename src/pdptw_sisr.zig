const std = @import("std");
const pdp = @import("pdptw.zig");

// SISR engine for PDPTW: ruin-and-recreate under threshold accepting, ported
// from the flagship VRPTW engine (vrptw.zig) onto the session-1 PDPTW
// primitives. Differences forced by the pairing constraint:
// - Ruin is PAIR-ATOMIC: removing any node also removes its partner (same
//   route by invariant). Capacity feasibility of the remainder is automatic
//   (remaining prefix loads only shrink when whole pairs leave), but time
//   windows are NOT: on directed non-metric matrices removing nodes can push
//   arrivals later, so every removal is gated by a Tws walk of the remainder
//   (the 725f7cd bug class).
// - Recreate inserts whole pairs: for a candidate route of length L, the
//   (a, b) gap pairs (pickup at gap a, dropoff at gap b >= a) are enumerated
//   in O(L^2) with O(1) evaluation each, via prefix/suffix Tws + Lseg arrays
//   and an incremental middle segment; time-warp and load-prefix violations
//   are monotone under extension, so infeasible prefixes prune the inner loop.
// - Rollback is whole-route snapshots (the shipped SISR-TW started the same
//   way); fine-grained undo journals are a later throughput lever.

pub const PdpSisrParams = struct {
    seed: u64 = 1,
    iters: usize = 100_000,
    time_ms: u64 = 0, // wall-clock cap, checked every 256 iters (0 = none)
    cbar: f64 = 10.0, // average removed nodes per ruin
    l_max: usize = 10, // max string length (nodes)
    blink: f64 = 0.01, // probability of skipping a candidate gap pair
    t0_factor: f64 = 1.0, // initial threshold = t0 * (seed distance / n_nodes)
    tf_factor: f64 = 0.01, // final threshold factor
    veh_penalty: u64 = 0, // per-route cost bias toward fewer vehicles
    gk: usize = 20, // kNN list length per node
    nbr_key: NbrKey = .sum,
};

pub const NbrKey = enum { sum, min, out };

// Time-window segment (Vidal / PyVRP), same algebra as vrptw.zig's Tws but
// over PdpInstance. tw == 0 iff the segment is schedulable.
const Tws = struct {
    dur: i64,
    tw: i64,
    early: i64,
    late: i64,

    fn client(inst: pdp.PdpInstance, c: usize) Tws {
        return .{ .dur = @intCast(inst.service[c]), .tw = 0, .early = @intCast(inst.ready[c]), .late = @intCast(inst.due[c]) };
    }
    fn depotNode(inst: pdp.PdpInstance) Tws {
        return .{ .dur = 0, .tw = 0, .early = 0, .late = @intCast(inst.due[0]) };
    }
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

const NO_ROUTE = std.math.maxInt(usize);

const Route = struct {
    items: std.ArrayList(usize) = .empty,
    dist: u64 = 0, // arc sum depot -> items -> depot (0 when empty)
    pre_t: std.ArrayList(Tws) = .empty, // pre_t[i] = depot..items[i-1]; len items+1
    suf_t: std.ArrayList(Tws) = .empty, // suf_t[i] = items[i]..depot;   len items+1
    pre_l: std.ArrayList(pdp.Lseg) = .empty,
    suf_l: std.ArrayList(pdp.Lseg) = .empty,
    pre_d: std.ArrayList(u64) = .empty, // arc sum depot->..->items[i-1]; len items+1
    tail_d: std.ArrayList(u64) = .empty, // arc sum items[i]->..->depot;  len items+1 (tail_d[len]=0)
    dirty: bool = true,

    fn deinit(r: *Route, allocator: std.mem.Allocator) void {
        r.items.deinit(allocator);
        r.pre_t.deinit(allocator);
        r.suf_t.deinit(allocator);
        r.pre_l.deinit(allocator);
        r.suf_l.deinit(allocator);
        r.pre_d.deinit(allocator);
        r.tail_d.deinit(allocator);
    }
};

const Snap = struct { ri: usize, items: []usize, dist: u64 };

const S = struct {
    allocator: std.mem.Allocator,
    inst: pdp.PdpInstance,
    routes: std.ArrayList(Route) = .empty,
    nonempty: usize = 0,
    cost: u64 = 0, // total dist + veh_penalty * nonempty
    veh_penalty: u64,
    gran: []const usize,
    gk: usize,
    loc_route: []usize, // node -> route index (NO_ROUTE when removed)
    loc_pos: []usize, // node -> position within its route
    removed: std.ArrayList(usize) = .empty, // pickup ids awaiting reinsertion
    cand_mark: std.ArrayList(u64) = .empty, // route -> generation of last candidate visit
    ruin_mark: std.ArrayList(u64) = .empty, // route -> generation of last ruin
    generation: u64 = 0,
    // whole-route snapshot journal for the current iteration
    snaps: std.ArrayList(Snap) = .empty,
    snap_mark: std.ArrayList(u64) = .empty, // route -> snap generation
    snap_gen: u64 = 0,
    min_empty_hint: usize = 0,
    keep_buf: std.ArrayList(usize) = .empty, // scratch for removals
    drop_buf: []bool, // scratch: per-position removal flags (sized 2n)

    fn init(allocator: std.mem.Allocator, inst: pdp.PdpInstance, veh_penalty: u64, gran: []const usize, gk: usize) !S {
        const dim = inst.dim();
        const s = S{
            .allocator = allocator,
            .inst = inst,
            .veh_penalty = veh_penalty,
            .gran = gran,
            .gk = gk,
            .loc_route = try allocator.alloc(usize, dim),
            .loc_pos = try allocator.alloc(usize, dim),
            .drop_buf = try allocator.alloc(bool, dim),
        };
        @memset(s.loc_route, NO_ROUTE);
        @memset(s.loc_pos, 0);
        @memset(s.drop_buf, false);
        return s;
    }

    fn deinit(s: *S) void {
        for (s.routes.items) |*r| r.deinit(s.allocator);
        s.routes.deinit(s.allocator);
        s.allocator.free(s.loc_route);
        s.allocator.free(s.loc_pos);
        s.allocator.free(s.drop_buf);
        s.removed.deinit(s.allocator);
        s.cand_mark.deinit(s.allocator);
        s.ruin_mark.deinit(s.allocator);
        for (s.snaps.items) |sn| s.allocator.free(sn.items);
        s.snaps.deinit(s.allocator);
        s.snap_mark.deinit(s.allocator);
        s.keep_buf.deinit(s.allocator);
        s.* = undefined;
    }

    fn arcSum(inst: pdp.PdpInstance, items: []const usize) u64 {
        if (items.len == 0) return 0;
        var d: u64 = inst.d(0, items[0]);
        for (0..items.len - 1) |i| d += inst.d(items[i], items[i + 1]);
        return d + inst.d(items[items.len - 1], 0);
    }

    fn addSlot(s: *S) !usize {
        try s.routes.append(s.allocator, .{});
        try s.cand_mark.append(s.allocator, 0);
        try s.ruin_mark.append(s.allocator, 0);
        try s.snap_mark.append(s.allocator, 0);
        return s.routes.items.len - 1;
    }

    /// Record route `ri`'s pre-iteration content once per iteration.
    fn snapshot(s: *S, ri: usize) !void {
        if (s.snap_mark.items[ri] == s.snap_gen) return;
        s.snap_mark.items[ri] = s.snap_gen;
        const r = &s.routes.items[ri];
        try s.snaps.append(s.allocator, .{
            .ri = ri,
            .items = try s.allocator.dupe(usize, r.items.items),
            .dist = r.dist,
        });
    }

    /// Set route `ri`'s content to `nodes`, updating dist/loc/nonempty/cost.
    fn install(s: *S, ri: usize, nodes: []const usize) !void {
        const r = &s.routes.items[ri];
        const was_empty = r.items.items.len == 0;
        const old_dist = r.dist;
        r.items.clearRetainingCapacity();
        try r.items.appendSlice(s.allocator, nodes);
        r.dist = arcSum(s.inst, nodes);
        r.dirty = true;
        for (nodes, 0..) |c, p| {
            s.loc_route[c] = ri;
            s.loc_pos[c] = p;
        }
        const now_empty = nodes.len == 0;
        s.cost = s.cost + r.dist - old_dist;
        if (was_empty and !now_empty) {
            s.nonempty += 1;
            s.cost += s.veh_penalty;
        } else if (!was_empty and now_empty) {
            s.nonempty -= 1;
            s.cost -= s.veh_penalty;
            if (ri < s.min_empty_hint) s.min_empty_hint = ri;
        }
    }

    fn beginIter(s: *S) void {
        s.snap_gen += 1;
        for (s.snaps.items) |sn| s.allocator.free(sn.items);
        s.snaps.clearRetainingCapacity();
    }

    /// Restore every snapshotted route; loc entries of their members are
    /// rebuilt after all installs (order-independent).
    fn rollback(s: *S, saved_cost: u64, saved_nonempty: usize) !void {
        for (s.snaps.items) |sn| {
            const r = &s.routes.items[sn.ri];
            const was_empty = r.items.items.len == 0;
            r.items.clearRetainingCapacity();
            try r.items.appendSlice(s.allocator, sn.items);
            r.dist = sn.dist;
            r.dirty = true;
            if (!was_empty and sn.items.len == 0 and sn.ri < s.min_empty_hint) s.min_empty_hint = sn.ri;
        }
        for (s.snaps.items) |sn| {
            for (s.routes.items[sn.ri].items.items, 0..) |c, p| {
                s.loc_route[c] = sn.ri;
                s.loc_pos[c] = p;
            }
        }
        s.cost = saved_cost;
        s.nonempty = saved_nonempty;
    }

    /// Rebuild prefix/suffix Tws + Lseg + distance arrays for route `ri`.
    fn freshen(s: *S, ri: usize) !void {
        const r = &s.routes.items[ri];
        if (!r.dirty) return;
        const it = r.items.items;
        const L = it.len;
        try r.pre_t.resize(s.allocator, L + 1);
        try r.suf_t.resize(s.allocator, L + 1);
        try r.pre_l.resize(s.allocator, L + 1);
        try r.suf_l.resize(s.allocator, L + 1);
        try r.pre_d.resize(s.allocator, L + 1);
        try r.tail_d.resize(s.allocator, L + 1);
        r.pre_t.items[0] = Tws.depotNode(s.inst);
        r.pre_l.items[0] = pdp.Lseg.empty();
        r.pre_d.items[0] = 0;
        var prev: usize = 0;
        for (it, 0..) |c, i| {
            r.pre_t.items[i + 1] = Tws.merge(r.pre_t.items[i], @intCast(s.inst.d(prev, c)), Tws.client(s.inst, c));
            r.pre_l.items[i + 1] = pdp.Lseg.merge(r.pre_l.items[i], pdp.Lseg.node(s.inst, c));
            r.pre_d.items[i + 1] = r.pre_d.items[i] + s.inst.d(prev, c);
            prev = c;
        }
        r.suf_t.items[L] = Tws.depotNode(s.inst);
        r.suf_l.items[L] = pdp.Lseg.empty();
        r.tail_d.items[L] = 0;
        var i = L;
        while (i > 0) {
            i -= 1;
            const nxt: usize = if (i + 1 == L) 0 else it[i + 1];
            r.suf_t.items[i] = Tws.merge(Tws.client(s.inst, it[i]), @intCast(s.inst.d(it[i], nxt)), r.suf_t.items[i + 1]);
            r.suf_l.items[i] = pdp.Lseg.merge(pdp.Lseg.node(s.inst, it[i]), r.suf_l.items[i + 1]);
            r.tail_d.items[i] = s.inst.d(it[i], nxt) + r.tail_d.items[i + 1];
        }
        r.dirty = false;
    }

    /// Whole-sequence TW feasibility (depot -> nodes -> depot).
    fn seqFeasible(s: *S, nodes: []const usize) bool {
        var acc = Tws.depotNode(s.inst);
        var prev: usize = 0;
        for (nodes) |c| {
            acc = Tws.merge(acc, @intCast(s.inst.d(prev, c)), Tws.client(s.inst, c));
            if (acc.tw != 0) return false;
            prev = c;
        }
        acc = Tws.merge(acc, @intCast(s.inst.d(prev, 0)), Tws.depotNode(s.inst));
        return acc.tw == 0;
    }

    /// Remove the string [start, start+l) of route `ri` plus every partner of
    /// a removed node, IF the remainder stays TW-feasible. Removed pairs are
    /// appended to s.removed (pickup id once each). Returns true if removed.
    fn removeStringPaired(s: *S, ri: usize, start: usize, l: usize) !bool {
        const r = &s.routes.items[ri];
        const it = r.items.items;
        for (it[start .. start + l]) |c| {
            s.drop_buf[c] = true;
            s.drop_buf[s.inst.pair_of[c]] = true;
        }
        s.keep_buf.clearRetainingCapacity();
        for (it) |c| {
            if (!s.drop_buf[c]) try s.keep_buf.append(s.allocator, c);
        }
        // remainder feasibility (TW only; capacity is automatic for pair-
        // atomic removal); the full-route walk is O(len), same as install.
        if (!s.seqFeasible(s.keep_buf.items)) {
            for (it) |c| s.drop_buf[c] = false;
            return false;
        }
        try s.snapshot(ri);
        for (it) |c| {
            if (s.drop_buf[c]) {
                s.drop_buf[c] = false;
                s.loc_route[c] = NO_ROUTE;
                if (s.inst.is_pickup[c]) try s.removed.append(s.allocator, c);
            }
        }
        try s.install(ri, s.keep_buf.items);
        return true;
    }

    const Ins = struct { a: usize, b: usize, delta: i64 };

    /// Best feasible insertion of pair (p, q) into route `ri`: p at gap `a`,
    /// q at gap `b` (both in the ORIGINAL array, a <= b, so q lands right
    /// after the block items[a..b)). O(L^2) gap pairs, O(1) eval each via the
    /// freshened prefix/suffix arrays + an incremental middle segment.
    /// Time-warp and load violations are monotone under rightward extension,
    /// so an infeasible middle prunes the rest of its `b` loop.
    fn evalPairInsert(s: *S, ri: usize, p: usize, q: usize, blink: f64, rng: std.Random) ?Ins {
        const inst = s.inst;
        const r = &s.routes.items[ri];
        const it = r.items.items;
        const L = it.len;
        var best: ?Ins = null;
        const q_t = Tws.client(inst, q);
        const q_l = pdp.Lseg.node(inst, q);

        for (0..L + 1) |a| {
            const prev_a: usize = if (a == 0) 0 else it[a - 1];
            // middle segment: pre[a] + p, extended one node per b step
            var m_t = Tws.merge(r.pre_t.items[a], @intCast(inst.d(prev_a, p)), Tws.client(inst, p));
            if (m_t.tw != 0) continue; // deeper a only arrives later, but other a gaps may differ
            var m_l = pdp.Lseg.merge(r.pre_l.items[a], pdp.Lseg.node(inst, p));
            if (m_l.lo < 0 or m_l.hi > inst.capacity) continue;
            var m_d: u64 = r.pre_d.items[a] + inst.d(prev_a, p);
            var last = p;

            var b = a;
            while (b <= L) : (b += 1) {
                blk: {
                    if (rng.float(f64) < blink) break :blk;
                    const nxt: usize = if (b == L) 0 else it[b];
                    const f1 = Tws.merge(m_t, @intCast(inst.d(last, q)), q_t);
                    const f2 = Tws.merge(f1, @intCast(inst.d(q, nxt)), r.suf_t.items[b]);
                    if (f2.tw != 0) break :blk;
                    const f_l = pdp.Lseg.merge(pdp.Lseg.merge(m_l, q_l), r.suf_l.items[b]);
                    if (!(f_l.lo >= 0 and f_l.hi <= inst.capacity)) break :blk;
                    const new_dist = m_d + inst.d(last, q) + inst.d(q, nxt) + r.tail_d.items[b];
                    const delta = @as(i64, @intCast(new_dist)) - @as(i64, @intCast(r.dist));
                    if (best == null or delta < best.?.delta) best = .{ .a = a, .b = b, .delta = delta };
                }
                if (b < L) {
                    m_t = Tws.merge(m_t, @intCast(inst.d(last, it[b])), Tws.client(inst, it[b]));
                    if (m_t.tw != 0) break; // no later b for this a can heal
                    m_l = pdp.Lseg.merge(m_l, pdp.Lseg.node(inst, it[b]));
                    if (m_l.lo < 0 or m_l.hi > inst.capacity) break;
                    m_d += inst.d(last, it[b]);
                    last = it[b];
                }
            }
        }
        return best;
    }

    /// Apply an insertion found by evalPairInsert.
    fn insertPair(s: *S, ri: usize, p: usize, q: usize, a: usize, b: usize) !void {
        try s.snapshot(ri);
        const r = &s.routes.items[ri];
        const it = r.items.items;
        s.keep_buf.clearRetainingCapacity();
        try s.keep_buf.appendSlice(s.allocator, it[0..a]);
        try s.keep_buf.append(s.allocator, p);
        try s.keep_buf.appendSlice(s.allocator, it[a..b]);
        try s.keep_buf.append(s.allocator, q);
        try s.keep_buf.appendSlice(s.allocator, it[b..]);
        try s.install(ri, s.keep_buf.items);
    }

    fn ruin(s: *S, params: PdpSisrParams, rng: std.Random) !void {
        const n_nodes = 2 * s.inst.n_pairs;
        s.generation += 1;

        const seed_c = 1 + rng.uintLessThan(usize, n_nodes);
        const avg_len = @max(@as(usize, 1), n_nodes / @max(@as(usize, 1), @max(s.nonempty, 1)));
        const ls_max = @max(@as(usize, 1), @min(params.l_max, avg_len));
        const ks_max = @max(@as(usize, 1), @as(usize, @intFromFloat((4.0 * params.cbar) / (1.0 + @as(f64, @floatFromInt(ls_max))))));
        const ks = 1 + rng.uintLessThan(usize, ks_max);

        var strings: usize = 0;
        var t: usize = 0;
        var c = seed_c;
        while (strings < ks and t <= s.gk) : (t += 1) {
            if (t > 0) c = s.gran[(seed_c - 1) * s.gk + (t - 1)];
            if (c == 0) continue; // kNN padding
            const ri = s.loc_route[c];
            if (ri == NO_ROUTE) continue;
            if (s.ruin_mark.items[ri] == s.generation) continue;
            s.ruin_mark.items[ri] = s.generation;
            const rlen = s.routes.items[ri].items.items.len;
            const l = 1 + rng.uintLessThan(usize, @min(ls_max, rlen));
            const p = s.loc_pos[c];
            const lo = if (p + 1 >= l) p + 1 - l else 0;
            const hi = @min(p, rlen - l);
            const start = lo + rng.uintLessThan(usize, hi - lo + 1);
            if (try s.removeStringPaired(ri, start, l)) strings += 1;
        }
    }

    fn recreate(s: *S, params: PdpSisrParams, rng: std.Random) !void {
        rng.shuffle(usize, s.removed.items);
        for (s.removed.items) |p| {
            const q = s.inst.pair_of[p];
            s.generation += 1;
            var best_ri: usize = NO_ROUTE;
            var best_ins: Ins = undefined;

            while (s.min_empty_hint < s.routes.items.len and s.routes.items[s.min_empty_hint].items.items.len != 0) : (s.min_empty_hint += 1) {}
            const empty_slot: usize = if (s.min_empty_hint < s.routes.items.len) s.min_empty_hint else NO_ROUTE;

            // candidate routes: those holding a kNN neighbour of p or of q
            for ([_]usize{ p, q }) |anchor| {
                for (0..s.gk) |ti| {
                    const nb = s.gran[(anchor - 1) * s.gk + ti];
                    if (nb == 0) continue;
                    const ri = s.loc_route[nb];
                    if (ri == NO_ROUTE) continue;
                    if (s.cand_mark.items[ri] == s.generation) continue;
                    s.cand_mark.items[ri] = s.generation;
                    try s.freshen(ri);
                    const ins = s.evalPairInsert(ri, p, q, params.blink, rng) orelse continue;
                    if (best_ri == NO_ROUTE or ins.delta < best_ins.delta) {
                        best_ri = ri;
                        best_ins = ins;
                    }
                }
            }

            const singleton: i64 = @intCast(s.inst.d(0, p) + s.inst.d(p, q) + s.inst.d(q, 0) + s.veh_penalty);
            if (best_ri == NO_ROUTE or singleton < best_ins.delta) {
                const slot = if (empty_slot != NO_ROUTE) empty_slot else try s.addSlot();
                try s.snapshot(slot);
                s.keep_buf.clearRetainingCapacity();
                try s.keep_buf.append(s.allocator, p);
                try s.keep_buf.append(s.allocator, q);
                try s.install(slot, s.keep_buf.items);
            } else {
                try s.insertPair(best_ri, p, q, best_ins.a, best_ins.b);
            }
        }
        s.removed.clearRetainingCapacity();
    }

    fn toResult(s: *S, allocator: std.mem.Allocator) !pdp.PdpResult {
        var out: std.ArrayList([]usize) = .empty;
        errdefer {
            for (out.items) |r| allocator.free(r);
            out.deinit(allocator);
        }
        var dist: u64 = 0;
        for (s.routes.items) |r| {
            if (r.items.items.len == 0) continue;
            try out.append(allocator, try allocator.dupe(usize, r.items.items));
            dist += r.dist;
        }
        const routes = try out.toOwnedSlice(allocator);
        return .{ .allocator = allocator, .routes = routes, .total_cost = dist, .vehicles = routes.len };
    }
};

fn buildNeighbors(allocator: std.mem.Allocator, inst: pdp.PdpInstance, k: usize, key_mode: NbrKey) ![]usize {
    const n = 2 * inst.n_pairs;
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
            keyc[j] = switch (key_mode) {
                .sum => inst.d(c, j) + inst.d(j, c),
                .min => @min(inst.d(c, j), inst.d(j, c)),
                .out => inst.d(c, j),
            };
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

fn nanos() u64 {
    var ts: std.os.linux.timespec = undefined;
    _ = std.os.linux.clock_gettime(std.os.linux.CLOCK.MONOTONIC, &ts);
    return @as(u64, @intCast(ts.sec)) * 1_000_000_000 + @as(u64, @intCast(ts.nsec));
}

pub fn solvePdptwSisr(allocator: std.mem.Allocator, inst: pdp.PdpInstance, params: PdpSisrParams) !pdp.PdpResult {
    const n_nodes = 2 * inst.n_pairs;
    if (inst.n_pairs == 0) return error.InvalidInstance;

    // seed: the session-1 cheapest pair-insertion construction, due-sorted
    const pos = try allocator.alloc(usize, inst.dim());
    defer allocator.free(pos);
    @memset(pos, 0);
    const pickups = try pdp.collectPickups(allocator, inst);
    defer allocator.free(pickups);
    const Ctx = struct {
        inst: pdp.PdpInstance,
        fn lessThan(ctx: @This(), a: usize, b: usize) bool {
            if (ctx.inst.due[a] != ctx.inst.due[b]) return ctx.inst.due[a] < ctx.inst.due[b];
            return a < b;
        }
    };
    std.mem.sort(usize, pickups, Ctx{ .inst = inst }, Ctx.lessThan);
    var seed_sol = try pdp.construct(allocator, inst, pickups, pos);
    defer pdp.freeSol(allocator, &seed_sol);

    const gk: usize = @min(if (params.gk == 0) @as(usize, 20) else params.gk, if (n_nodes > 1) n_nodes - 1 else 1);
    const gran = try buildNeighbors(allocator, inst, gk, params.nbr_key);
    defer allocator.free(gran);

    var s = try S.init(allocator, inst, params.veh_penalty, gran, gk);
    defer s.deinit();
    for (seed_sol.items) |r| {
        if (r.items.len == 0) continue;
        const ri = try s.addSlot();
        try s.install(ri, r.items);
    }

    var best = try s.toResult(allocator);
    errdefer best.deinit();
    var best_cost = s.cost;

    var prng = std.Random.DefaultPrng.init(params.seed ^ 0x50_44_50_7457);
    const rng = prng.random();

    const seed_distance = s.cost - s.veh_penalty * s.nonempty;
    const unit = @as(f64, @floatFromInt(seed_distance)) / @as(f64, @floatFromInt(n_nodes));
    const t0 = @max(1e-9, params.t0_factor * unit);
    const tf = @max(1e-9, params.tf_factor * unit);
    const iters = @max(@as(usize, 1), params.iters);
    const cf = std.math.pow(f64, tf / t0, 1.0 / @as(f64, @floatFromInt(iters)));
    var temp = t0;

    const t_start = nanos();
    var it: usize = 0;
    while (it < iters) : (it += 1) {
        if (params.time_ms > 0 and it % 256 == 0 and (nanos() - t_start) / std.time.ns_per_ms >= params.time_ms) break;
        const saved_cost = s.cost;
        const saved_nonempty = s.nonempty;
        s.beginIter();
        try s.ruin(params, rng);
        try s.recreate(params, rng);
        const dt = @as(i64, @intCast(s.cost)) - @as(i64, @intCast(saved_cost));
        if (@as(f64, @floatFromInt(dt)) < temp) {
            if (s.cost < best_cost) {
                best_cost = s.cost;
                best.deinit();
                best = try s.toResult(allocator);
            }
        } else {
            try s.rollback(saved_cost, saved_nonempty);
        }
        temp *= cf;
    }
    return best;
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

test "PDPTW SISR: feasible and cost-honest on random instances" {
    const allocator = std.testing.allocator;
    var seed: u64 = 1;
    while (seed <= 20) : (seed += 1) {
        var ri = try pdp.randomInstance(allocator, 10, seed, true);
        defer ri.deinit(allocator);
        const inst = ri.inst();
        var res = try solvePdptwSisr(allocator, inst, .{ .seed = seed, .iters = 2000 });
        defer res.deinit();
        const rc = try allocator.alloc([]const usize, res.routes.len);
        defer allocator.free(rc);
        for (res.routes, 0..) |r, i| rc[i] = r;
        const vc = pdp.validate(inst, rc) orelse return error.Infeasible;
        try std.testing.expectEqual(vc, res.total_cost);
        try std.testing.expectEqual(res.routes.len, res.vehicles);
    }
}

test "PDPTW SISR: never worse than the construction seed" {
    const allocator = std.testing.allocator;
    var seed: u64 = 1;
    while (seed <= 20) : (seed += 1) {
        var ri = try pdp.randomInstance(allocator, 8, seed, true);
        defer ri.deinit(allocator);
        const inst = ri.inst();

        // construction cost (same order solvePdptwSisr seeds from)
        const pos = try allocator.alloc(usize, inst.dim());
        defer allocator.free(pos);
        @memset(pos, 0);
        const pickups = try pdp.collectPickups(allocator, inst);
        defer allocator.free(pickups);
        const Ctx = struct {
            inst: pdp.PdpInstance,
            fn lessThan(ctx: @This(), a: usize, b: usize) bool {
                if (ctx.inst.due[a] != ctx.inst.due[b]) return ctx.inst.due[a] < ctx.inst.due[b];
                return a < b;
            }
        };
        std.mem.sort(usize, pickups, Ctx{ .inst = inst }, Ctx.lessThan);
        var sol = try pdp.construct(allocator, inst, pickups, pos);
        defer pdp.freeSol(allocator, &sol);
        var cons: u64 = 0;
        for (sol.items) |r| cons += S.arcSum(inst, r.items);

        var res = try solvePdptwSisr(allocator, inst, .{ .seed = seed, .iters = 1500 });
        defer res.deinit();
        try std.testing.expect(res.total_cost <= cons);
    }
}

test "PDPTW SISR: deterministic for a fixed seed" {
    const allocator = std.testing.allocator;
    var ri = try pdp.randomInstance(allocator, 8, 42, true);
    defer ri.deinit(allocator);
    const inst = ri.inst();
    var a = try solvePdptwSisr(allocator, inst, .{ .seed = 7, .iters = 1000 });
    defer a.deinit();
    var b = try solvePdptwSisr(allocator, inst, .{ .seed = 7, .iters = 1000 });
    defer b.deinit();
    try std.testing.expectEqual(a.total_cost, b.total_cost);
    try std.testing.expectEqual(a.vehicles, b.vehicles);
}

test "PDPTW SISR: never beats the brute-force optimum" {
    const allocator = std.testing.allocator;
    var seed: u64 = 1;
    while (seed <= 8) : (seed += 1) {
        var ri = try pdp.randomInstance(allocator, 3, seed, true);
        defer ri.deinit(allocator);
        const inst = ri.inst();
        const opt = try pdp.bruteForceOptimum(allocator, inst);
        var res = try solvePdptwSisr(allocator, inst, .{ .seed = seed, .iters = 3000 });
        defer res.deinit();
        try std.testing.expect(res.total_cost >= opt);
    }
}

test "PDPTW SISR: quality floor vs brute optimum on 30 known seeds" {
    // Same yardstick as the baseline's floor test: n_pairs=3, seeds 1..30.
    // The baseline (restarts=8) matches 30/30; SISR must be no less capable.
    const allocator = std.testing.allocator;
    var matches: usize = 0;
    var seed: u64 = 1;
    while (seed <= 30) : (seed += 1) {
        var ri = try pdp.randomInstance(allocator, 3, seed, true);
        defer ri.deinit(allocator);
        const inst = ri.inst();
        const opt = try pdp.bruteForceOptimum(allocator, inst);
        var res = try solvePdptwSisr(allocator, inst, .{ .seed = seed, .iters = 3000 });
        defer res.deinit();
        if (res.total_cost == opt) matches += 1;
    }
    try std.testing.expect(matches >= 29);
}

test "PDPTW SISR: all-reject run returns exactly the construction seed" {
    // tf = t0 = ~0 threshold means every non-improving iteration must roll
    // back perfectly; with a tiny iter budget the engine exercises rollback
    // heavily and the final best must still validate and match its cost.
    const allocator = std.testing.allocator;
    var ri = try pdp.randomInstance(allocator, 8, 5, true);
    defer ri.deinit(allocator);
    const inst = ri.inst();
    var res = try solvePdptwSisr(allocator, inst, .{ .seed = 5, .iters = 800, .t0_factor = 1e-12, .tf_factor = 1e-12 });
    defer res.deinit();
    const rc = try allocator.alloc([]const usize, res.routes.len);
    defer allocator.free(rc);
    for (res.routes, 0..) |r, i| rc[i] = r;
    const vc = pdp.validate(inst, rc) orelse return error.Infeasible;
    try std.testing.expectEqual(vc, res.total_cost);
}

test "PDPTW SISR: clocked instances stay feasible" {
    const allocator = std.testing.allocator;
    var seed: u64 = 1;
    while (seed <= 10) : (seed += 1) {
        var ri = try pdp.randomInstanceClocked(allocator, 8, seed);
        defer ri.deinit(allocator);
        const inst = ri.inst();
        var res = try solvePdptwSisr(allocator, inst, .{ .seed = seed, .iters = 1500 });
        defer res.deinit();
        const rc = try allocator.alloc([]const usize, res.routes.len);
        defer allocator.free(rc);
        for (res.routes, 0..) |r, i| rc[i] = r;
        const vc = pdp.validate(inst, rc) orelse return error.Infeasible;
        try std.testing.expectEqual(vc, res.total_cost);
    }
}
