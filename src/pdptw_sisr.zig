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
    fleet_ruin_rate: f64 = 0.1, // chance to empty the smallest route per ruin (veh_penalty > 0 only)
    gk: usize = 20, // kNN list length per node
    nbr_key: NbrKey = .sum,
    // Fleet cap (0 = uncapped). When set, recreate may not open a route past
    // the cap; pairs with no feasible insertion sit in an unassigned pool
    // (request bank) under a penalty that dominates everything else, and the
    // engine keeps retrying them each iteration. The solve only reports
    // solutions with an empty pool; error.NoCompleteSolution if none found.
    max_vehicles: usize = 0,
    // Granular gap restriction for whole-pair insertion: on routes with at
    // least `gran_gap_min_len` nodes, only pickup gaps adjacent to a kNN
    // neighbour of p (route ends always allowed) are scanned, and dropoff
    // gaps not adjacent to a kNN neighbour of q are walked but not evaluated
    // (the adjacent b == a gap is always evaluated). Cuts the O(L^2) recreate
    // scan on long routes; off by default until measured.
    gran_gaps: u2 = 0, // bit 0: gate pickup gaps; bit 1: gate dropoff evals (the measured win)
    gran_gap_min_len: usize = 24,
    // Granular gating only pays in the few-long-routes regime (lr2-style);
    // with many routes it starves fleet consolidation (measured n=1000 lr2:
    // fleet 19 -> 23). Auto-off above this route count.
    gran_max_routes: usize = 8,
    // GES-style squeeze fallback (Nagata & Kobayashi 2010) for capped runs:
    // when a banked pair has no feasible insertion and no route may open,
    // insert it at the least-violating position (time warp + load excess)
    // and eject the cheapest single resident pair that restores feasibility.
    // Ejection counters steer away from cycling. Only fires when
    // max_vehicles > 0 blocks opening a route; default off.
    eject: bool = false,
    // Max residents ejected per squeeze when a single eject can't restore
    // feasibility (Nagata & Kobayashi 2010 k-eject ladder): rung 1 is the
    // existing single-pair ejectCandidate; rungs 2 and 3 brute-force small
    // subsets of resident pairs. Measured: on packed PDPTW instances
    // (lc1_10_3 at fleet cap 86) 63% of squeezes find no single resident
    // pair whose removal restores feasibility — ejecting a subset unlocks
    // those. Clamped to [1, 3]; 0 is treated as 1. Only meaningful when
    // eject = true.
    eject_k: u8 = 1,
    // Inter-route pair exchange (SWAP*-shaped, Vidal 2022): every swap_kick
    // iterations, pick a random pair and try exchanging it with a kNN pair
    // from another route, each reinserted at its own best position. Strict
    // improvement only. 0 = off.
    swap_kick: usize = 0,
    // Share of the fleet-min drivers' total budget spent on the initial
    // uncapped run, in percent. The descent gets (100 - fleet_p0_pct) * 3/2
    // clamped as before. The SISR paper front-loads route minimization; lower
    // values hand the descent more time.
    fleet_p0_pct: u8 = 40,
    // Money objective: cost per matrix-time-unit of route DURATION (travel +
    // service + unavoidable waiting) added to acceptance cost and insertion
    // deltas, alongside distance and veh_penalty. Duration comes from the
    // Tws algebra, so each route is charged its departure-time-optimized
    // schedule (Savelsbergh shifting is inherent). 0 = off (pure
    // distance + veh_penalty, bit-identical to the historic objective).
    time_penalty: u64 = 0,
    // Max route duration (shift-length cap): reject any insertion / route
    // mutation whose resulting depot->...->depot Tws duration (travel +
    // service + unavoidable waiting) exceeds this cap. 0 = off (no cap,
    // byte-identical to before this knob existed). Independent of
    // time_penalty: the cap is a hard feasibility bound, the penalty a soft
    // cost term; either being nonzero turns on route-duration maintenance.
    max_route_dur: u64 = 0,
    // Intra-search parallelism: split one pair's candidate-route insertion
    // evaluation (the recreate hot loop) across a persistent thread pool,
    // deterministic order-equivalent reduction. 0/1 = serial, bit-identical
    // to the historic engine. >=2 deepens ONE search trajectory (unlike
    // solvePdptwSisrFleetMinParallel's width). Uses a per-route deterministic
    // blink PRNG, so >=2 is thread-count-invariant (2==6==12) but NOT
    // bit-identical to the serial shared-stream trajectory.
    eval_threads: usize = 0,
    // Heterogeneous fleet (v1): up to 8 vehicle types. Empty = uniform fleet
    // from inst.capacity + veh_penalty (bit-identical to before this knob
    // existed). Nonempty: every route is assigned a type when it opens — the
    // cheapest (by fixed_cost, ties to larger capacity) type with a count
    // available whose capacity fits the opening load. Capacity checks then use
    // the route's type capacity, and its fixed_cost replaces veh_penalty in
    // the objective. count = 0 means unlimited. A route keeps its type until
    // it empties (no mid-life upgrades in v1). inst.capacity must be the
    // LARGEST type capacity (the seed construction packs to it); seed routes
    // that fit no available type start in the request bank instead.
    // Not supported with the fleet-min / pinned drivers (rejected at capi).
    veh_types: []const VehType = &.{},
    // Driver break (v1): one optional break per route, same spec for every
    // vehicle. null = off (NOT ONE INSTRUCTION of new work on the hot paths —
    // every break-aware site is guarded at its outermost level). Semantics:
    // a route whose depart-at-0 schedule finishes after `earliest` must
    // contain one break of `dur` seconds starting within [earliest, latest];
    // the break absorbs waiting first and delays every later stop by the
    // remainder. Break time counts into route duration (the driver is on the
    // clock), so it interacts with time_penalty naturally. Placement is the
    // LAST inter-stop gap whose no-break departure is <= latest — provably
    // feasibility-dominant (an earlier break delays a superset of stops by at
    // least as much), so one O(route) walk decides feasibility exactly.
    // Duration bookkeeping under a break uses the depart-at-0 schedule (not
    // the Tws-optimized departure). Not supported with fleet_min / pinned /
    // max_vehicles (rejected at capi) or eval_threads >= 2 (InvalidInstance).
    brk: ?Break = null,
};

pub const NbrKey = enum { sum, min, out };

pub const Break = struct {
    dur: u32, // break length, matrix time units
    earliest: u32, // break must START within [earliest, latest]
    latest: u32,
};

pub const VehType = struct {
    capacity: i64,
    fixed_cost: u64,
    count: u32, // max simultaneous routes of this type; 0 = unlimited
};

pub const MAX_VEH_TYPES = 8;

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

/// Full-route duration (depot -> nodes -> depot) under the Tws algebra:
/// travel + service + unavoidable waiting at the duration-minimizing
/// departure time. Only meaningful for TW-feasible sequences; transient
/// infeasible installs (squeeze) get a consistent bookkeeping value that
/// snapshot rollback restores exactly.
pub fn routeDuration(inst: pdp.PdpInstance, nodes: []const usize) u64 {
    if (nodes.len == 0) return 0;
    var acc = Tws.depotNode(inst);
    var prev: usize = 0;
    for (nodes) |c| {
        acc = Tws.merge(acc, @intCast(inst.d(prev, c)), Tws.client(inst, c));
        prev = c;
    }
    acc = Tws.merge(acc, @intCast(inst.d(prev, 0)), Tws.depotNode(inst));
    return @intCast(@max(acc.dur, 0));
}

const BreakWalk = struct { ok: bool, dur: u64 };

/// Depart-at-0 schedule of depot -> nodes -> depot with the single break
/// placed greedily (see PdpSisrParams.brk). Returns .ok plus the completion
/// time (= route duration under a break regime). When the no-break schedule
/// already violates a window the walk is infeasible (a break only delays);
/// its no-break completion is still returned as a consistent bookkeeping
/// value, which snapshot rollback restores exactly.
pub fn walkWithBreak(inst: pdp.PdpInstance, nodes: []const usize, brk: Break) BreakWalk {
    if (nodes.len == 0) return .{ .ok = true, .dur = 0 };
    // Pass 1: no-break schedule. Track TW feasibility, the completion time,
    // and g_star = the last gap (0..len, len = before depot return) whose
    // departure time is <= latest.
    var t: u64 = 0;
    var prev: usize = 0;
    var ok = true;
    var g_star: usize = 0; // gap 0 (before the first stop) departs at t=0
    for (nodes, 0..) |c, i| {
        if (t <= brk.latest) g_star = i;
        const arrive = t + inst.d(prev, c);
        const start = @max(arrive, @as(u64, inst.ready[c]));
        if (start > inst.due[c]) ok = false;
        t = start + inst.service[c];
        prev = c;
    }
    if (t <= brk.latest) g_star = nodes.len;
    const completion_nobrk = t + inst.d(prev, 0);
    if (completion_nobrk > inst.due[0]) ok = false;
    if (!ok) return .{ .ok = false, .dur = completion_nobrk };
    // Ends before the window opens: no break required.
    if (completion_nobrk <= brk.earliest) return .{ .ok = true, .dur = completion_nobrk };

    // Pass 2: replay with the break at g_star. Stops before g_star keep the
    // pass-1 (earliest possible) times; the break start absorbs any gap up to
    // `earliest`, then everything after shifts by the remainder.
    t = 0;
    prev = 0;
    for (nodes, 0..) |c, i| {
        if (i == g_star) t = @max(t, @as(u64, brk.earliest)) + brk.dur;
        const arrive = t + inst.d(prev, c);
        const start = @max(arrive, @as(u64, inst.ready[c]));
        if (start > inst.due[c]) return .{ .ok = false, .dur = completion_nobrk };
        t = start + inst.service[c];
        prev = c;
    }
    if (g_star == nodes.len) t = @max(t, @as(u64, brk.earliest)) + brk.dur;
    const completion = t + inst.d(prev, 0);
    if (completion > inst.due[0]) return .{ .ok = false, .dur = completion_nobrk };
    return .{ .ok = true, .dur = completion };
}

const NO_ROUTE = std.math.maxInt(usize);

const Route = struct {
    items: std.ArrayList(usize) = .empty,
    dist: u64 = 0, // arc sum depot -> items -> depot (0 when empty)
    dur: u64 = 0, // full-route Tws duration; maintained only when time_penalty > 0
    pre_t: std.ArrayList(Tws) = .empty, // pre_t[i] = depot..items[i-1]; len items+1
    suf_t: std.ArrayList(Tws) = .empty, // suf_t[i] = items[i]..depot;   len items+1
    pre_l: std.ArrayList(pdp.Lseg) = .empty,
    suf_l: std.ArrayList(pdp.Lseg) = .empty,
    pre_d: std.ArrayList(u64) = .empty, // arc sum depot->..->items[i-1]; len items+1
    tail_d: std.ArrayList(u64) = .empty, // arc sum items[i]->..->depot;  len items+1 (tail_d[len]=0)
    brk_ok: bool = true, // break regime only: walkWithBreak(items).ok as of last install
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

const Snap = struct { ri: usize, items: []usize, dist: u64, dur: u64, rtype: u8, brk_ok: bool };

const S = struct {
    allocator: std.mem.Allocator,
    inst: pdp.PdpInstance,
    routes: std.ArrayList(Route) = .empty,
    lock: std.ArrayList(usize) = .empty, // per-route locked leading positions (0 = unlocked)
    nonempty: usize = 0,
    cost: u64 = 0, // total dist + veh_penalty * nonempty + time_penalty * total dur
    veh_penalty: u64,
    time_penalty: u64,
    max_route_dur: u64, // shift-length cap; 0 = off (see PdpSisrParams.max_route_dur)
    // Heterogeneous fleet ledger (empty veh_types = uniform, everything below
    // inert). rtype[ri] is route ri's type index, meaningful while nonempty
    // and overwritten at the next opening; type_used counts nonempty routes
    // per type. Both are restored on rollback (rtype via the route snapshots,
    // type_used from saved_type_used captured with the iteration's cost).
    veh_types: []const VehType,
    rtype: std.ArrayList(u8) = .empty,
    type_used: [MAX_VEH_TYPES]u32 = @splat(0),
    saved_type_used: [MAX_VEH_TYPES]u32 = @splat(0),
    // Driver break regime (null = off, every site below guarded on it).
    brk: ?Break,
    brk_scratch: []usize, // preallocated (dim + 2): candidate sequences for break-aware eval
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
    n_unassigned: usize = 0, // pairs currently in the request bank (loc_route == NO_ROUTE)
    keep_buf: std.ArrayList(usize) = .empty, // scratch for removals
    cand_list: std.ArrayList(usize) = .empty, // recreatePar: ranked candidate routes (rank = append index)
    drop_buf: []bool, // scratch: per-position removal flags (sized 2n)
    nbr_mark_p: []u64, // granular gaps: node -> stamp of last kNN(p) marking
    nbr_mark_q: []u64,
    nbr_gen: u64 = 0,
    eject_pen: []u32, // pickup id -> times it forced a squeeze (GES guidance)
    // True when the current iteration began with an empty request bank —
    // i.e. a capped run is in pure distance-polish mode, not consolidation.
    // Granular pruning is safe there (see the gran gate in evalPairInsert).
    iter_start_complete: bool = false,

    fn init(allocator: std.mem.Allocator, inst: pdp.PdpInstance, veh_penalty: u64, time_penalty: u64, max_route_dur: u64, veh_types: []const VehType, brk: ?Break, gran: []const usize, gk: usize) !S {
        const dim = inst.dim();
        const s = S{
            .allocator = allocator,
            .inst = inst,
            .veh_penalty = veh_penalty,
            .time_penalty = time_penalty,
            .max_route_dur = max_route_dur,
            .veh_types = veh_types,
            .brk = brk,
            .gran = gran,
            .gk = gk,
            .loc_route = try allocator.alloc(usize, dim),
            .loc_pos = try allocator.alloc(usize, dim),
            .brk_scratch = try allocator.alloc(usize, dim + 2),
            .drop_buf = try allocator.alloc(bool, dim),
            .nbr_mark_p = try allocator.alloc(u64, dim),
            .nbr_mark_q = try allocator.alloc(u64, dim),
            .eject_pen = try allocator.alloc(u32, dim),
        };
        @memset(s.loc_route, NO_ROUTE);
        @memset(s.loc_pos, 0);
        @memset(s.drop_buf, false);
        @memset(s.nbr_mark_p, 0);
        @memset(s.nbr_mark_q, 0);
        @memset(s.eject_pen, 0);
        return s;
    }

    fn deinit(s: *S) void {
        for (s.routes.items) |*r| r.deinit(s.allocator);
        s.routes.deinit(s.allocator);
        s.lock.deinit(s.allocator);
        s.rtype.deinit(s.allocator);
        s.allocator.free(s.brk_scratch);
        s.allocator.free(s.loc_route);
        s.allocator.free(s.loc_pos);
        s.allocator.free(s.drop_buf);
        s.allocator.free(s.nbr_mark_p);
        s.allocator.free(s.nbr_mark_q);
        s.allocator.free(s.eject_pen);
        s.removed.deinit(s.allocator);
        s.cand_mark.deinit(s.allocator);
        s.ruin_mark.deinit(s.allocator);
        for (s.snaps.items) |sn| s.allocator.free(sn.items);
        s.snaps.deinit(s.allocator);
        s.snap_mark.deinit(s.allocator);
        s.keep_buf.deinit(s.allocator);
        s.cand_list.deinit(s.allocator);
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
        try s.lock.append(s.allocator, 0);
        try s.rtype.append(s.allocator, 0);
        try s.cand_mark.append(s.allocator, 0);
        try s.ruin_mark.append(s.allocator, 0);
        try s.snap_mark.append(s.allocator, 0);
        return s.routes.items.len - 1;
    }

    /// Effective capacity of route `ri`: uniform inst.capacity, or its
    /// assigned type's capacity when a heterogeneous fleet is active.
    fn routeCap(s: *const S, ri: usize) i64 {
        return if (s.veh_types.len == 0) s.inst.capacity else s.veh_types[s.rtype.items[ri]].capacity;
    }

    /// Per-route fixed cost: uniform veh_penalty, or the route's type
    /// fixed_cost when a heterogeneous fleet is active.
    fn penOf(s: *const S, ri: usize) u64 {
        return if (s.veh_types.len == 0) s.veh_penalty else s.veh_types[s.rtype.items[ri]].fixed_cost;
    }

    /// Cheapest type (fixed_cost asc, ties to larger capacity) with a count
    /// available whose capacity fits `load` (max prefix load of the opening
    /// content). null when no type can open — the route may not be created.
    fn chooseType(s: *const S, load: i64) ?u8 {
        var best: ?u8 = null;
        for (s.veh_types, 0..) |t, i| {
            if (t.capacity < load) continue;
            if (t.count > 0 and s.type_used[i] >= t.count) continue;
            if (best) |bi| {
                const b = s.veh_types[bi];
                if (t.fixed_cost < b.fixed_cost or
                    (t.fixed_cost == b.fixed_cost and t.capacity > b.capacity)) best = @as(u8, @intCast(i));
            } else best = @as(u8, @intCast(i));
        }
        return best;
    }

    /// Capture the type-count ledger next to the iteration's saved cost;
    /// rollback restores it. No-op state on the uniform path.
    fn saveLedger(s: *S) void {
        s.saved_type_used = s.type_used;
    }

    /// Locked leading positions of route `ri` (dispatch mode); set once before
    /// the loop and never changed. Rollback restores route CONTENT, which
    /// always preserves the locked prefix because no mutation may touch it.
    fn lockOf(s: *const S, ri: usize) usize {
        return if (ri < s.lock.items.len) s.lock.items[ri] else 0;
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
            .dur = r.dur,
            .rtype = if (s.veh_types.len == 0) 0 else s.rtype.items[ri],
            .brk_ok = r.brk_ok,
        });
    }

    /// Set route `ri`'s content to `nodes`, updating dist/loc/nonempty/cost.
    fn install(s: *S, ri: usize, nodes: []const usize) !void {
        const r = &s.routes.items[ri];
        const was_empty = r.items.items.len == 0;
        const old_dist = r.dist;
        const old_dur = r.dur;
        r.items.clearRetainingCapacity();
        try r.items.appendSlice(s.allocator, nodes);
        r.dist = arcSum(s.inst, nodes);
        if (s.brk) |bk| {
            // Break regime: duration is the depart-at-0 completion including
            // the break, and the walk's verdict is the route's break flag.
            const w = walkWithBreak(s.inst, nodes, bk);
            r.dur = w.dur;
            r.brk_ok = w.ok;
        } else {
            r.dur = if (s.time_penalty > 0 or s.max_route_dur > 0) routeDuration(s.inst, nodes) else 0;
        }
        r.dirty = true;
        for (nodes, 0..) |c, p| {
            s.loc_route[c] = ri;
            s.loc_pos[c] = p;
        }
        const now_empty = nodes.len == 0;
        s.cost = s.cost + r.dist + s.time_penalty * r.dur - old_dist - s.time_penalty * old_dur;
        if (was_empty and !now_empty) {
            s.nonempty += 1;
            s.cost += s.penOf(ri);
            if (s.veh_types.len != 0) s.type_used[s.rtype.items[ri]] += 1;
        } else if (!was_empty and now_empty) {
            s.nonempty -= 1;
            s.cost -= s.penOf(ri);
            if (s.veh_types.len != 0) s.type_used[s.rtype.items[ri]] -= 1;
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
        // Nodes sitting in a mutated route right now may not be in its
        // restored content (a banked pair inserted this iteration): clear
        // their loc first or they keep a stale position (index-out-of-range
        // in the next ruin). Every mutation snapshots its route, so the snap
        // set covers every membership change.
        for (s.snaps.items) |sn| {
            for (s.routes.items[sn.ri].items.items) |c| s.loc_route[c] = NO_ROUTE;
        }
        for (s.snaps.items) |sn| {
            const r = &s.routes.items[sn.ri];
            const was_empty = r.items.items.len == 0;
            r.items.clearRetainingCapacity();
            try r.items.appendSlice(s.allocator, sn.items);
            r.dist = sn.dist;
            r.dur = sn.dur;
            r.brk_ok = sn.brk_ok;
            r.dirty = true;
            if (!was_empty and sn.items.len == 0 and sn.ri < s.min_empty_hint) s.min_empty_hint = sn.ri;
        }
        for (s.snaps.items) |sn| {
            for (s.routes.items[sn.ri].items.items, 0..) |c, p| {
                s.loc_route[c] = sn.ri;
                s.loc_pos[c] = p;
            }
        }
        if (s.veh_types.len != 0) {
            for (s.snaps.items) |sn| s.rtype.items[sn.ri] = sn.rtype;
            s.type_used = s.saved_type_used;
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
        const lk = s.lockOf(ri);
        if (start < lk) return false;
        const r = &s.routes.items[ri];
        const it = r.items.items;
        for (it[start .. start + l]) |c| {
            // A locked pickup pins its unlocked dropoff: pair-atomic removal
            // would drag the locked node out.
            if (lk > 0 and s.loc_pos[s.inst.pair_of[c]] < lk) {
                for (it) |x| s.drop_buf[x] = false;
                return false;
            }
            s.drop_buf[c] = true;
            s.drop_buf[s.inst.pair_of[c]] = true;
        }
        s.keep_buf.clearRetainingCapacity();
        for (it) |c| {
            if (!s.drop_buf[c]) try s.keep_buf.append(s.allocator, c);
        }
        // remainder feasibility (TW only; capacity is automatic for pair-
        // atomic removal); the full-route walk is O(len), same as install.
        // Break regime: the remainder must stay break-schedulable too
        // (removal shifts arrivals EARLIER, which can move the break's g_star
        // and change absorption — re-derive, don't assume).
        const rem_ok = if (s.brk) |bk| walkWithBreak(s.inst, s.keep_buf.items, bk).ok else s.seqFeasible(s.keep_buf.items);
        if (!rem_ok) {
            for (it) |c| s.drop_buf[c] = false;
            return false;
        }
        try s.snapshot(ri);
        for (it) |c| {
            if (s.drop_buf[c]) {
                s.drop_buf[c] = false;
                s.loc_route[c] = NO_ROUTE;
                if (s.inst.is_pickup[c]) {
                    try s.removed.append(s.allocator, c);
                    s.n_unassigned += 1;
                }
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
    fn evalPairInsert(s: *S, ri: usize, p: usize, q: usize, params: PdpSisrParams, rng: std.Random) ?Ins {
        // Break regime routes to its own O(route)-per-candidate evaluator;
        // the null branch below is the pre-break code verbatim.
        if (s.brk != null) return s.evalPairInsertBrk(ri, p, q, params, rng);
        const blink = params.blink;
        const inst = s.inst;
        const rcap: i64 = s.routeCap(ri);
        const r = &s.routes.items[ri];
        const it = r.items.items;
        const L = it.len;
        var best: ?Ins = null;
        const q_t = Tws.client(inst, q);
        const q_l = pdp.Lseg.node(inst, q);

        // Granular gate: marks are the UNION of kNN(p) and kNN(q) (a gap
        // near the dropoff's neighbours is a fine pickup spot too, and vice
        // versa). If the route holds fewer than 4 marked nodes the gate is
        // dropped for this route: too sparse a mark set cannot consolidate
        // (measured: n=1000 lr2 fleet blows 19 -> 30 without the fallback).
        // Never gate while a fleet cap is active AND the iteration started
        // with a nonempty request bank: capped consolidation needs the
        // awkward insertions the gate skips (measured: lrc2 200-series loses
        // a vehicle on 2/7 cells otherwise). Capped-complete iterations
        // (bank empty at iteration start) are pure distance polish — the
        // same regime as uncapped, where gating is the measured win.
        var granular = params.gran_gaps != 0 and (params.max_vehicles == 0 or s.iter_start_complete) and
            L >= params.gran_gap_min_len and s.nonempty <= params.gran_max_routes;
        var genp: u64 = 0;
        if (granular) {
            s.nbr_gen += 1;
            genp = s.nbr_gen;
            for (0..s.gk) |ti| {
                const np = s.gran[(p - 1) * s.gk + ti];
                if (np != 0) s.nbr_mark_p[np] = genp;
                const nq = s.gran[(q - 1) * s.gk + ti];
                if (nq != 0) s.nbr_mark_p[nq] = genp;
            }
            var hits: usize = 0;
            for (it) |c| hits += @intFromBool(s.nbr_mark_p[c] == genp);
            if (hits < 4) granular = false;
        }

        for (s.lockOf(ri)..L + 1) |a| {
            if (granular and (params.gran_gaps & 1) != 0 and a != 0 and a != L and
                s.nbr_mark_p[it[a - 1]] != genp and s.nbr_mark_p[it[a]] != genp) continue;
            const prev_a: usize = if (a == 0) 0 else it[a - 1];
            // middle segment: pre[a] + p, extended one node per b step
            var m_t = Tws.merge(r.pre_t.items[a], @intCast(inst.d(prev_a, p)), Tws.client(inst, p));
            if (m_t.tw != 0) continue; // deeper a only arrives later, but other a gaps may differ
            var m_l = pdp.Lseg.merge(r.pre_l.items[a], pdp.Lseg.node(inst, p));
            if (m_l.lo < 0 or m_l.hi > rcap) continue;
            var m_d: u64 = r.pre_d.items[a] + inst.d(prev_a, p);
            var last = p;

            var b = a;
            while (b <= L) : (b += 1) {
                blk: {
                    if (rng.float(f64) < blink) break :blk;
                    const nxt: usize = if (b == L) 0 else it[b];
                    if (granular and (params.gran_gaps & 2) != 0 and b != a and b != L and
                        s.nbr_mark_p[last] != genp and s.nbr_mark_p[nxt] != genp) break :blk;
                    const f1 = Tws.merge(m_t, @intCast(inst.d(last, q)), q_t);
                    const f2 = Tws.merge(f1, @intCast(inst.d(q, nxt)), r.suf_t.items[b]);
                    if (f2.tw != 0) break :blk;
                    const f_l = pdp.Lseg.merge(pdp.Lseg.merge(m_l, q_l), r.suf_l.items[b]);
                    if (!(f_l.lo >= 0 and f_l.hi <= rcap)) break :blk;
                    // Shift-length cap: reject if the resulting route duration
                    // exceeds it. Duration is not monotone in b, so this only
                    // skips THIS (a,b) candidate (break :blk), never the loop.
                    if (s.max_route_dur > 0 and f2.dur > @as(i64, @intCast(s.max_route_dur))) break :blk;
                    const new_dist = m_d + inst.d(last, q) + inst.d(q, nxt) + r.tail_d.items[b];
                    var delta = @as(i64, @intCast(new_dist)) - @as(i64, @intCast(r.dist));
                    if (s.time_penalty > 0)
                        delta += @as(i64, @intCast(s.time_penalty)) * (f2.dur - @as(i64, @intCast(r.dur)));
                    if (best == null or delta < best.?.delta) best = .{ .a = a, .b = b, .delta = delta };
                }
                if (b < L) {
                    m_t = Tws.merge(m_t, @intCast(inst.d(last, it[b])), Tws.client(inst, it[b]));
                    if (m_t.tw != 0) break; // no later b for this a can heal
                    m_l = pdp.Lseg.merge(m_l, pdp.Lseg.node(inst, it[b]));
                    if (m_l.lo < 0 or m_l.hi > rcap) break;
                    m_d += inst.d(last, it[b]);
                    last = it[b];
                }
            }
        }
        return best;
    }

    /// Break-regime insertion eval: same gap enumeration and Tws/load pruning
    /// as evalPairInsert (both remain NECESSARY conditions — a break only adds
    /// time), but each surviving (a, b) candidate is confirmed by an O(route)
    /// walkWithBreak over the materialized sequence, which also yields the
    /// duration the money objective prices. No granular gating (break routes
    /// are shift-bounded and short; correctness first for v1).
    fn evalPairInsertBrk(s: *S, ri: usize, p: usize, q: usize, params: PdpSisrParams, rng: std.Random) ?Ins {
        const blink = params.blink;
        const inst = s.inst;
        const rcap: i64 = s.routeCap(ri);
        const bk = s.brk.?;
        const r = &s.routes.items[ri];
        const it = r.items.items;
        const L = it.len;
        var best: ?Ins = null;
        const q_t = Tws.client(inst, q);
        const q_l = pdp.Lseg.node(inst, q);

        for (s.lockOf(ri)..L + 1) |a| {
            const prev_a: usize = if (a == 0) 0 else it[a - 1];
            var m_t = Tws.merge(r.pre_t.items[a], @intCast(inst.d(prev_a, p)), Tws.client(inst, p));
            if (m_t.tw != 0) continue;
            var m_l = pdp.Lseg.merge(r.pre_l.items[a], pdp.Lseg.node(inst, p));
            if (m_l.lo < 0 or m_l.hi > rcap) continue;
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
                    if (!(f_l.lo >= 0 and f_l.hi <= rcap)) break :blk;
                    // Materialize it[0..a] ++ p ++ it[a..b] ++ q ++ it[b..]
                    // and let the break walk decide feasibility + duration.
                    const cand = s.brk_scratch[0 .. L + 2];
                    @memcpy(cand[0..a], it[0..a]);
                    cand[a] = p;
                    @memcpy(cand[a + 1 .. b + 1], it[a..b]);
                    cand[b + 1] = q;
                    @memcpy(cand[b + 2 ..], it[b..]);
                    const w = walkWithBreak(inst, cand, bk);
                    if (!w.ok) break :blk;
                    if (s.max_route_dur > 0 and w.dur > s.max_route_dur) break :blk;
                    const new_dist = m_d + inst.d(last, q) + inst.d(q, nxt) + r.tail_d.items[b];
                    var delta = @as(i64, @intCast(new_dist)) - @as(i64, @intCast(r.dist));
                    if (s.time_penalty > 0)
                        delta += @as(i64, @intCast(s.time_penalty)) * (@as(i64, @intCast(w.dur)) - @as(i64, @intCast(r.dur)));
                    if (best == null or delta < best.?.delta) best = .{ .a = a, .b = b, .delta = delta };
                }
                if (b < L) {
                    m_t = Tws.merge(m_t, @intCast(inst.d(last, it[b])), Tws.client(inst, it[b]));
                    if (m_t.tw != 0) break;
                    m_l = pdp.Lseg.merge(m_l, pdp.Lseg.node(inst, it[b]));
                    if (m_l.lo < 0 or m_l.hi > rcap) break;
                    m_d += inst.d(last, it[b]);
                    last = it[b];
                }
            }
        }
        return best;
    }

    const InsV = struct { a: usize, b: usize, viol: i64, delta: i64 };

    /// Least-violating insertion of pair (p, q) into route `ri`: same gap
    /// enumeration as evalPairInsert but WITHOUT feasibility pruning; every
    /// (a, b) is scored by viol = final time warp + 10 * load excess, ties by
    /// distance delta. Used only by the squeeze fallback, so no blinks and no
    /// granular gating. Route must be freshened.
    fn evalPairInsertViol(s: *S, ri: usize, p: usize, q: usize) ?InsV {
        const inst = s.inst;
        const rcap: i64 = s.routeCap(ri);
        const r = &s.routes.items[ri];
        const it = r.items.items;
        const L = it.len;
        var best: ?InsV = null;
        const q_t = Tws.client(inst, q);
        const q_l = pdp.Lseg.node(inst, q);

        for (s.lockOf(ri)..L + 1) |a| {
            const prev_a: usize = if (a == 0) 0 else it[a - 1];
            var m_t = Tws.merge(r.pre_t.items[a], @intCast(inst.d(prev_a, p)), Tws.client(inst, p));
            var m_l = pdp.Lseg.merge(r.pre_l.items[a], pdp.Lseg.node(inst, p));
            var m_d: u64 = r.pre_d.items[a] + inst.d(prev_a, p);
            var last = p;

            var b = a;
            while (b <= L) : (b += 1) {
                const nxt: usize = if (b == L) 0 else it[b];
                const f1 = Tws.merge(m_t, @intCast(inst.d(last, q)), q_t);
                const f2 = Tws.merge(f1, @intCast(inst.d(q, nxt)), r.suf_t.items[b]);
                const f_l = pdp.Lseg.merge(pdp.Lseg.merge(m_l, q_l), r.suf_l.items[b]);
                const load_ex: i64 = @max(f_l.hi - rcap, 0);
                const viol: i64 = f2.tw + 10 * load_ex;
                const new_dist = m_d + inst.d(last, q) + inst.d(q, nxt) + r.tail_d.items[b];
                const delta = @as(i64, @intCast(new_dist)) - @as(i64, @intCast(r.dist));
                if (best == null or viol < best.?.viol or (viol == best.?.viol and delta < best.?.delta)) {
                    best = .{ .a = a, .b = b, .viol = viol, .delta = delta };
                }
                if (b < L) {
                    m_t = Tws.merge(m_t, @intCast(inst.d(last, it[b])), Tws.client(inst, it[b]));
                    m_l = pdp.Lseg.merge(m_l, pdp.Lseg.node(inst, it[b]));
                    m_d += inst.d(last, it[b]);
                    last = it[b];
                }
            }
        }
        return best;
    }

    /// After a squeeze insertion made route `ri` infeasible, find the resident
    /// pair (excluding skip_p) whose removal restores full feasibility (TW via
    /// seqFeasible + load via a prefix walk), preferring the lowest ejection
    /// counter, ties by lowest pickup id. Returns the pickup id, or null.
    fn ejectCandidate(s: *S, ri: usize, skip_p: usize) !?usize {
        const inst = s.inst;
        const rcap: i64 = s.routeCap(ri);
        const it = s.routes.items[ri].items.items;
        const lk = s.lockOf(ri);
        var best: ?usize = null;
        for (it) |e| {
            if (!inst.is_pickup[e] or e == skip_p) continue;
            if (s.loc_pos[e] < lk or s.loc_pos[inst.pair_of[e]] < lk) continue;
            const f = inst.pair_of[e];
            s.keep_buf.clearRetainingCapacity();
            for (it) |c| {
                if (c != e and c != f) try s.keep_buf.append(s.allocator, c);
            }
            if (!s.seqFeasible(s.keep_buf.items)) continue;
            const lg = pdp.routeLseg(inst, s.keep_buf.items);
            if (!(lg.lo >= 0 and lg.hi <= rcap)) continue;
            if (best == null or s.eject_pen[e] < s.eject_pen[best.?] or
                (s.eject_pen[e] == s.eject_pen[best.?] and e < best.?)) best = e;
        }
        return best;
    }

    const EjectSubset = struct { buf: [3]usize, len: usize };

    /// Sorted-and-capped candidate pool for the eject_k >= 2 ladder rungs:
    /// every resident pickup of route `ri` other than `skip_p`, excluding
    /// locked pairs (same filter as ejectCandidate), ordered by
    /// (eject_pen asc, pickup id asc) and truncated to the 12 lowest-key
    /// entries — the ladder brute-forces subsets over this pool, so it must
    /// stay small (C(12,3) = 220 feasibility checks worst case). Returns the
    /// count written into `buf`.
    fn ejectPool(s: *S, ri: usize, skip_p: usize, buf: *[12]usize) usize {
        const inst = s.inst;
        const it = s.routes.items[ri].items.items;
        const lk = s.lockOf(ri);
        var n: usize = 0;
        for (it) |e| {
            if (!inst.is_pickup[e] or e == skip_p) continue;
            if (s.loc_pos[e] < lk or s.loc_pos[inst.pair_of[e]] < lk) continue;
            if (n < buf.len) {
                var pos = n;
                while (pos > 0 and s.eject_pool_less(e, buf[pos - 1])) : (pos -= 1) buf[pos] = buf[pos - 1];
                buf[pos] = e;
                n += 1;
            } else if (s.eject_pool_less(e, buf[n - 1])) {
                var pos = n - 1;
                while (pos > 0 and s.eject_pool_less(e, buf[pos - 1])) : (pos -= 1) buf[pos] = buf[pos - 1];
                buf[pos] = e;
            }
        }
        return n;
    }

    fn eject_pool_less(s: *const S, a: usize, b: usize) bool {
        if (s.eject_pen[a] != s.eject_pen[b]) return s.eject_pen[a] < s.eject_pen[b];
        return a < b;
    }

    /// Rungs 2-3 of the eject ladder: enumerate `size`-subsets (2 or 3) of
    /// ejectPool(ri, skip_p) in lexicographic index order, feasibility-check
    /// each (seqFeasible + the same Lseg load check ejectCandidate uses),
    /// and keep the FEASIBLE subset minimizing (sum eject_pen over the
    /// subset, then lexicographically smallest pickup-id tuple). Deterministic,
    /// no RNG. Returns the chosen subset or null if no `size`-subset of the
    /// pool restores feasibility.
    fn ejectSubsetK(s: *S, ri: usize, skip_p: usize, size: usize) !?EjectSubset {
        const rcap: i64 = s.routeCap(ri);
        var pool: [12]usize = undefined;
        const n = s.ejectPool(ri, skip_p, &pool);
        if (n < size) return null;

        const it = s.routes.items[ri].items.items;
        var idx: [3]usize = undefined;
        for (0..size) |i| idx[i] = i;

        var best: ?EjectSubset = null;
        var best_pen: u64 = undefined;

        while (true) {
            var cand: [3]usize = undefined;
            for (0..size) |i| cand[i] = pool[idx[i]];
            // insertion-sort cand[0..size] by id (size <= 3) for a canonical
            // lexicographic-tuple comparison independent of pool order.
            for (1..size) |i| {
                const v = cand[i];
                var j = i;
                while (j > 0 and cand[j - 1] > v) : (j -= 1) cand[j] = cand[j - 1];
                cand[j] = v;
            }

            s.keep_buf.clearRetainingCapacity();
            route: for (it) |c| {
                for (cand[0..size]) |e| {
                    if (c == e or c == s.inst.pair_of[e]) continue :route;
                }
                try s.keep_buf.append(s.allocator, c);
            }
            feasible: {
                if (!s.seqFeasible(s.keep_buf.items)) break :feasible;
                const lg = pdp.routeLseg(s.inst, s.keep_buf.items);
                if (!(lg.lo >= 0 and lg.hi <= rcap)) break :feasible;
                var pen: u64 = 0;
                for (cand[0..size]) |e| pen += s.eject_pen[e];
                const better = best == null or pen < best_pen or
                    (pen == best_pen and lexLess(cand[0..size], best.?.buf[0..best.?.len]));
                if (better) {
                    best = .{ .buf = cand, .len = size };
                    best_pen = pen;
                }
            }

            // advance idx to the next lexicographic size-combination over 0..n
            var i = size;
            var done = true;
            while (i > 0) {
                i -= 1;
                if (idx[i] != i + n - size) {
                    idx[i] += 1;
                    for (i + 1..size) |j| idx[j] = idx[j - 1] + 1;
                    done = false;
                    break;
                }
            }
            if (done) break;
        }
        return best;
    }

    fn lexLess(a: []const usize, b: []const usize) bool {
        for (a, b) |x, y| {
            if (x != y) return x < y;
        }
        return false;
    }

    /// GES squeeze: insert (p, q) at the least-violating position among the
    /// kNN candidate routes, then eject residents to restore feasibility —
    /// rung 1 (unchanged) tries a single resident pair; if that fails and
    /// eject_k allows it, rungs 2-3 brute-force small subsets (see
    /// ejectSubsetK). The ejected pairs go to the request bank (NOT to
    /// s.removed — recreate is iterating it). Returns true on success; on
    /// total failure the route is restored exactly and (p, q) stays banked.
    fn squeezeInsert(s: *S, p: usize, q: usize, params: PdpSisrParams) !bool {
        s.generation += 1;
        var best_ri: usize = NO_ROUTE;
        var best_ins: InsV = undefined;
        for ([_]usize{ p, q }) |anchor| {
            for (0..s.gk) |ti| {
                const nb = s.gran[(anchor - 1) * s.gk + ti];
                if (nb == 0) continue;
                const ri = s.loc_route[nb];
                if (ri == NO_ROUTE) continue;
                if (s.cand_mark.items[ri] == s.generation) continue;
                s.cand_mark.items[ri] = s.generation;
                try s.freshen(ri);
                const ins = s.evalPairInsertViol(ri, p, q) orelse continue;
                if (best_ri == NO_ROUTE or ins.viol < best_ins.viol or
                    (ins.viol == best_ins.viol and ins.delta < best_ins.delta))
                {
                    best_ri = ri;
                    best_ins = ins;
                }
            }
        }
        if (best_ri == NO_ROUTE) return false;

        // keep an exact copy for the undo path
        const before = try s.allocator.dupe(usize, s.routes.items[best_ri].items.items);
        defer s.allocator.free(before);

        try s.insertPair(best_ri, p, q, best_ins.a, best_ins.b);

        // Rung 1: existing single-eject candidate, unchanged selection.
        if (try s.ejectCandidate(best_ri, p)) |ec| {
            const ef = s.inst.pair_of[ec];
            const cur = s.routes.items[best_ri].items.items;
            s.keep_buf.clearRetainingCapacity();
            for (cur) |c| {
                if (c != ec and c != ef) try s.keep_buf.append(s.allocator, c);
            }
            try s.install(best_ri, s.keep_buf.items);
            s.loc_route[ec] = NO_ROUTE;
            s.loc_route[ef] = NO_ROUTE;
            s.n_unassigned += 1;
            s.eject_pen[p] +|= 1;
            return true;
        }

        // Rungs 2-3: k-eject when a single resident pair can't restore
        // feasibility (see PdpSisrParams.eject_k).
        const eject_k: usize = @min(@max(params.eject_k, 1), 3);
        var size: usize = 2;
        while (size <= eject_k) : (size += 1) {
            const sub = (try s.ejectSubsetK(best_ri, p, size)) orelse continue;
            const cur = s.routes.items[best_ri].items.items;
            s.keep_buf.clearRetainingCapacity();
            route: for (cur) |c| {
                for (sub.buf[0..sub.len]) |e| {
                    if (c == e or c == s.inst.pair_of[e]) continue :route;
                }
                try s.keep_buf.append(s.allocator, c);
            }
            try s.install(best_ri, s.keep_buf.items);
            for (sub.buf[0..sub.len]) |e| {
                s.loc_route[e] = NO_ROUTE;
                s.loc_route[s.inst.pair_of[e]] = NO_ROUTE;
            }
            s.n_unassigned += sub.len;
            s.eject_pen[p] +|= 1;
            return true;
        }

        // undo: restore content, then clear the stale loc of p and q —
        // install only rewrites loc for members of the restored content.
        try s.install(best_ri, before);
        s.loc_route[p] = NO_ROUTE;
        s.loc_route[q] = NO_ROUTE;
        s.eject_pen[p] +|= 1;
        return false;
    }

    /// One pair-exchange attempt: remove pair (p1) from its route and a kNN
    /// pair (p2) from another route, reinsert each into the other route at
    /// its best feasible position. Kept only on strict total-cost improvement;
    /// otherwise rolled back exactly. Runs between iterations, so it manages
    /// its own snapshot generation.
    fn pairExchangeKick(s: *S, params: PdpSisrParams, rng: std.Random) !void {
        if (s.nonempty < 2) return;
        const n_nodes = 2 * s.inst.n_pairs;
        const c0 = 1 + rng.uintLessThan(usize, n_nodes);
        const p1 = if (s.inst.is_pickup[c0]) c0 else s.inst.pair_of[c0];
        const q1 = s.inst.pair_of[p1];
        const r1 = s.loc_route[p1];
        if (r1 == NO_ROUTE) return;

        const saved_cost = s.cost;
        const saved_nonempty = s.nonempty;
        const saved_unassigned = s.n_unassigned;
        s.saveLedger();
        s.beginIter();
        s.generation += 1;

        for (0..s.gk) |ti| {
            const nb = s.gran[(p1 - 1) * s.gk + ti];
            if (nb == 0) continue;
            const p2 = if (s.inst.is_pickup[nb]) nb else s.inst.pair_of[nb];
            const q2 = s.inst.pair_of[p2];
            const r2 = s.loc_route[p2];
            if (r2 == NO_ROUTE or r2 == r1) continue;
            if (s.cand_mark.items[r2] == s.generation) continue;
            s.cand_mark.items[r2] = s.generation;

            trial: {
                if (!try s.removeStringPaired(r1, s.loc_pos[p1], 1)) break :trial;
                if (!try s.removeStringPaired(r2, s.loc_pos[p2], 1)) break :trial;
                try s.freshen(r2);
                const ins1 = s.evalPairInsert(r2, p1, q1, params, rng) orelse break :trial;
                try s.insertPair(r2, p1, q1, ins1.a, ins1.b);
                try s.freshen(r1);
                const ins2 = s.evalPairInsert(r1, p2, q2, params, rng) orelse break :trial;
                try s.insertPair(r1, p2, q2, ins2.a, ins2.b);
                if (s.cost < saved_cost) {
                    s.removed.clearRetainingCapacity();
                    s.n_unassigned = saved_unassigned;
                    return; // improvement kept
                }
            }
            // failed or non-improving: restore the iteration-start state and
            // try the next candidate (rollback is idempotent per generation).
            try s.rollback(saved_cost, saved_nonempty);
            s.n_unassigned = saved_unassigned;
            s.removed.clearRetainingCapacity();
        }
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

        // request bank: unassigned pairs re-enter recreate every iteration
        if (s.n_unassigned > s.removed.items.len) {
            s.removed.clearRetainingCapacity();
            for (1..s.inst.dim()) |c| {
                if (s.inst.is_pickup[c] and s.loc_route[c] == NO_ROUTE) try s.removed.append(s.allocator, c);
            }
        }

        var seed_c = 1 + rng.uintLessThan(usize, n_nodes);
        if ((s.veh_penalty > 0 or s.veh_types.len != 0) and s.nonempty > 1 and rng.float(f64) < params.fleet_ruin_rate) {
            // Fleet-min ruin (vrptw.zig lever): empty the smallest route
            // outright; its pairs reinsert into the slack the strings below
            // open around them, and veh_penalty settles it in acceptance.
            // Emptying a whole route always passes the removal gate (the
            // remainder is empty), so this cannot be rejected.
            var smallest: usize = NO_ROUTE;
            var slen: usize = std.math.maxInt(usize);
            for (s.routes.items, 0..) |r, i| {
                if (s.lockOf(i) > 0) continue;
                const len = r.items.items.len;
                if (len > 0 and len < slen) {
                    smallest = i;
                    slen = len;
                }
            }
            if (smallest != NO_ROUTE) {
                seed_c = s.routes.items[smallest].items.items[rng.uintLessThan(usize, slen)];
                s.ruin_mark.items[smallest] = s.generation;
                _ = try s.removeStringPaired(smallest, 0, slen);
            }
        }
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
                    const ins = s.evalPairInsert(ri, p, q, params, rng) orelse continue;
                    if (best_ri == NO_ROUTE or ins.delta < best_ins.delta) {
                        best_ri = ri;
                        best_ins = ins;
                    }
                }
            }

            var sdur: u64 = if (s.time_penalty > 0 or s.max_route_dur > 0) routeDuration(s.inst, &[_]usize{ p, q }) else 0;
            // Break regime: a fresh [p,q] route must be break-schedulable, and
            // its priced duration is the break walk's (break time included).
            var brk_open_ok = true;
            if (s.brk) |bk| {
                const w = walkWithBreak(s.inst, &[_]usize{ p, q }, bk);
                brk_open_ok = w.ok;
                sdur = w.dur;
            }
            // Opening a fresh [p,q] route requires it to fit the shift cap too;
            // fold that into may_open so an over-cap singleton is never created
            // (the pair falls through to squeeze / stays banked instead).
            var may_open = brk_open_ok and (params.max_vehicles == 0 or s.nonempty < params.max_vehicles) and
                (s.max_route_dur == 0 or sdur <= s.max_route_dur);
            // Heterogeneous fleet: opening also needs a type with a count
            // available that fits the pair's load; the singleton is charged
            // that type's fixed cost instead of the uniform veh_penalty.
            var open_pen: u64 = s.veh_penalty;
            var open_type: u8 = 0;
            if (s.veh_types.len != 0) {
                if (s.chooseType(s.inst.demand_signed[p])) |ti| {
                    open_type = ti;
                    open_pen = s.veh_types[ti].fixed_cost;
                } else may_open = false;
            }
            var singleton: i64 = @intCast(s.inst.d(0, p) + s.inst.d(p, q) + s.inst.d(q, 0) + open_pen);
            if (s.time_penalty > 0)
                singleton += @intCast(s.time_penalty * sdur);
            if (best_ri == NO_ROUTE and !may_open) {
                if (params.eject and try s.squeezeInsert(p, q, params)) {
                    s.n_unassigned -= 1;
                }
                continue; // otherwise stays in the request bank
            }
            if ((best_ri == NO_ROUTE or singleton < best_ins.delta) and may_open) {
                const slot = if (empty_slot != NO_ROUTE) empty_slot else try s.addSlot();
                try s.snapshot(slot);
                if (s.veh_types.len != 0) s.rtype.items[slot] = open_type;
                s.keep_buf.clearRetainingCapacity();
                try s.keep_buf.append(s.allocator, p);
                try s.keep_buf.append(s.allocator, q);
                try s.install(slot, s.keep_buf.items);
            } else {
                try s.insertPair(best_ri, p, q, best_ins.a, best_ins.b);
            }
            s.n_unassigned -= 1;
        }
        s.removed.clearRetainingCapacity();
    }

    /// True unless a shift cap is active and some nonempty route exceeds it.
    /// r.dur is maintained (install) whenever max_route_dur > 0, so this is an
    /// O(routes) read. Off path (cap == 0) returns true immediately — the seed
    /// construction can hand us an over-cap route that no insertion site would,
    /// so best-capture is gated on this to keep the returned plan cap-clean.
    fn withinDurCap(s: *const S) bool {
        if (s.max_route_dur == 0) return true;
        for (s.routes.items) |r| {
            if (r.items.items.len != 0 and r.dur > s.max_route_dur) return false;
        }
        return true;
    }

    /// Break regime: true when every nonempty route is break-schedulable
    /// (r.brk_ok maintained by install, restored by rollback). Always true
    /// when breaks are off. Gates best-capture like withinDurCap.
    fn brkAllOk(s: *const S) bool {
        if (s.brk == null) return true;
        for (s.routes.items) |r| {
            if (r.items.items.len != 0 and !r.brk_ok) return false;
        }
        return true;
    }

    /// Number of nonempty routes that are not break-schedulable. 0 when
    /// breaks are off. Folded into acceptance (BRK counts under CAP_PEN) so
    /// the search is driven out of break violations the seed may contain —
    /// same lesson as the duration cap: capture-gating alone lets an
    /// infeasible incumbent squat and roll back every feasible move.
    fn brkViolCount(s: *const S) u64 {
        if (s.brk == null) return 0;
        var n: u64 = 0;
        for (s.routes.items) |r| {
            if (r.items.items.len != 0 and !r.brk_ok) n += 1;
        }
        return n;
    }

    /// Number of nonempty routes exceeding the shift cap. 0 when the cap is off.
    /// Folded into the acceptance objective (CAP_PEN weight) so the search is
    /// driven OUT of over-cap states the way UNASSIGNED_PEN drives it out of an
    /// incomplete bank. Without this, the seed's over-cap giant route can be a
    /// cheaper (by distance) incumbent than the only cap-feasible layout, so the
    /// threshold-accept rolls back every feasible-but-costlier split and best is
    /// never captured — a genuinely feasible instance reported infeasible.
    fn overCapCount(s: *const S) u64 {
        if (s.max_route_dur == 0) return 0;
        var n: u64 = 0;
        for (s.routes.items) |r| {
            if (r.items.items.len != 0 and r.dur > s.max_route_dur) n += 1;
        }
        return n;
    }

    fn toResult(s: *S, allocator: std.mem.Allocator) !pdp.PdpResult {
        var out: std.ArrayList([]usize) = .empty;
        errdefer {
            for (out.items) |r| allocator.free(r);
            out.deinit(allocator);
        }
        var tout: std.ArrayList(usize) = .empty;
        errdefer tout.deinit(allocator);
        var dist: u64 = 0;
        for (s.routes.items, 0..) |r, ri| {
            if (r.items.items.len == 0) continue;
            try out.append(allocator, try allocator.dupe(usize, r.items.items));
            if (s.veh_types.len != 0) try tout.append(allocator, s.rtype.items[ri]);
            dist += r.dist;
        }
        const routes = try out.toOwnedSlice(allocator);
        errdefer {
            for (routes) |r| allocator.free(r);
            allocator.free(routes);
        }
        const types: ?[]usize = if (s.veh_types.len != 0) try tout.toOwnedSlice(allocator) else null;
        return .{ .allocator = allocator, .routes = routes, .total_cost = dist, .vehicles = routes.len, .types = types };
    }
};

// ---------------------------------------------------------------------------
// Intra-search parallelism (eval_threads >= 2): the recreate candidate-route
// evaluation is the hot loop at n >= 1000. One removed pair's candidate routes
// are frozen (discovery + freshen on the main thread), then evaluated across a
// persistent thread pool. The winner is chosen by a deterministic, thread-
// count-invariant reduction over (delta asc, rank asc) where rank is the
// discovery index of each route (unique => strict total order => result is
// independent of the partition and worker count, and equals the same batch
// evaluated inline on one thread). See the head-of-file design note.
//
// Blink source: evalPairInsertPar seeds a per-route stack-local PRNG from
// mixSeed(seed, iter, p, ri) instead of the shared stream, because a data-
// dependent number of shared draws per route cannot be parallelized without
// redoing the O(L^2) scan. The draw sequence then depends only on this route's
// frozen content, so it is identical inline, on any worker, and across thread
// counts. This is the deviation from today's serial trajectory (a diversity
// knob on which 1%-rate gaps get skipped); OFF (eval_threads <= 1) keeps the
// original shared-stream serial path untouched and bit-identical.

const PAR_MIN_WORK: usize = 512; // Sum-of-candidate-route-lengths gate: below this a batch evaluates inline (barrier would not pay). Tune on the winserver.

// Park/wake barrier primitive. Zig 0.16's std moved threading sync into the Io
// async model (std.Io.Mutex/Condition) and dropped std.Thread.Futex, so the
// pool parks helpers on raw Linux v1 futexes directly — consistent with this
// file already depending on std.os.linux for nanos(). Private futex (same
// process), no timeout; the callers re-check their atomic and tolerate spurious
// wakeups. futex_4arg (WAIT) / futex_3arg (WAKE) both ignore the args the op
// doesn't use. Linux-only, which the engine already is.
const linux = std.os.linux;

inline fn futexWait(v: *const std.atomic.Value(u32), expect: u32) void {
    _ = linux.futex_4arg(@ptrCast(&v.raw), .{ .cmd = .WAIT, .private = true }, expect, null);
}
inline fn futexWake(v: *const std.atomic.Value(u32), n: u32) void {
    _ = linux.futex_3arg(@ptrCast(&v.raw), .{ .cmd = .WAKE, .private = true }, n);
}

fn mixSeed(seed: u64, iter: u64, p: usize, ri: usize) u64 {
    var buf: [32]u8 = undefined;
    std.mem.writeInt(u64, buf[0..8], seed, .little);
    std.mem.writeInt(u64, buf[8..16], iter, .little);
    std.mem.writeInt(u64, buf[16..24], @intCast(p), .little);
    std.mem.writeInt(u64, buf[24..32], @intCast(ri), .little);
    return std.hash.Wyhash.hash(0, &buf);
}

const EvalResult = struct { has: bool = false, delta: i64 = 0, rank: usize = 0, a: usize = 0, b: usize = 0 };

/// Total order for the reduction: a candidate beats the current best iff it is
/// present and (strictly smaller delta) or (equal delta and smaller rank).
/// rank is unique per route => strict total order => winner is independent of
/// how candidates are partitioned across workers.
fn betterResult(cand: EvalResult, cur: EvalResult) bool {
    if (!cand.has) return false;
    if (!cur.has) return true;
    if (cand.delta != cur.delta) return cand.delta < cur.delta;
    return cand.rank < cur.rank;
}

const EvalWorker = struct {
    best: EvalResult align(64) = .{}, // own cache line: the hot per-batch write
    nbr_mark_p: []u64 = &.{}, // per-worker granular scratch (dim), alloc once
    nbr_mark_q: []u64 = &.{},
    nbr_gen: u64 = 0,
    _pad: [64]u8 = undefined, // pad struct past a cache line to kill false sharing
};

const EvalBatch = struct {
    cands: []const usize = &.{}, // ranked candidate route ids (index == rank)
    p: usize = 0,
    q: usize = 0,
    iter: u64 = 0,
};

/// Persistent futex-parked pool, one per core-solve call (spawn/join is µs and
/// happens tens of times over a fleet-min run — cheap; not threaded through the
/// drivers). W = eval_threads participants: worker 0 is the calling thread,
/// W-1 helpers park between batches so they burn no CPU during ruin/apply/
/// rollback. Determinism of the batch winner is independent of the partition,
/// so strided round-robin (worker w handles cands[w], cands[w+W], ...) is a
/// free load-balancing choice.
const EvalPool = struct {
    s: *S,
    params: PdpSisrParams,
    helpers: []std.Thread,
    workers: []EvalWorker,
    batch: EvalBatch = .{},
    W: usize,
    spawned: usize = 0,
    start_gen: std.atomic.Value(u32) = .init(0), // main bumps to release a batch
    done: std.atomic.Value(u32) = .init(0), // helpers increment on finish
    shutdown: std.atomic.Value(bool) = .init(false),

    /// Allocate workers + per-worker scratch + the helper handle array. Does
    /// NOT spawn threads (the returned value is moved to its final address by
    /// the caller first); call start() afterwards.
    fn init(allocator: std.mem.Allocator, s: *S, params: PdpSisrParams, w_count: usize) !EvalPool {
        const dim = s.inst.dim();
        const workers = try allocator.alloc(EvalWorker, w_count);
        errdefer allocator.free(workers);
        var alloced: usize = 0;
        errdefer for (workers[0..alloced]) |*wk| {
            allocator.free(wk.nbr_mark_p);
            allocator.free(wk.nbr_mark_q);
        };
        for (workers) |*wk| {
            wk.* = .{
                .nbr_mark_p = try allocator.alloc(u64, dim),
                .nbr_mark_q = try allocator.alloc(u64, dim),
            };
            @memset(wk.nbr_mark_p, 0);
            @memset(wk.nbr_mark_q, 0);
            alloced += 1;
        }
        const helpers = try allocator.alloc(std.Thread, w_count - 1);
        return .{ .s = s, .params = params, .helpers = helpers, .workers = workers, .W = w_count };
    }

    /// Spawn the W-1 helper threads. Must be called after the pool sits at its
    /// final stable address (helpers capture `self`). On partial spawn failure
    /// the already-spawned helpers are shut down and the error bubbles up.
    fn start(self: *EvalPool) !void {
        self.spawned = 0;
        for (self.helpers, 0..) |*h, i| {
            h.* = std.Thread.spawn(.{}, helperLoop, .{ self, i + 1 }) catch |err| {
                self.shutdownHelpers();
                return err;
            };
            self.spawned += 1;
        }
    }

    fn shutdownHelpers(self: *EvalPool) void {
        if (self.spawned == 0) return;
        self.shutdown.store(true, .release);
        _ = self.start_gen.fetchAdd(1, .release);
        futexWake(&self.start_gen, @intCast(self.spawned));
        for (self.helpers[0..self.spawned]) |h| h.join();
        self.spawned = 0;
    }

    fn deinit(self: *EvalPool, allocator: std.mem.Allocator) void {
        self.shutdownHelpers();
        allocator.free(self.helpers);
        for (self.workers) |*wk| {
            allocator.free(wk.nbr_mark_p);
            allocator.free(wk.nbr_mark_q);
        }
        allocator.free(self.workers);
    }

    fn helperLoop(self: *EvalPool, wid: usize) void {
        var seen: u32 = 0;
        while (true) {
            futexWait(&self.start_gen, seen);
            const g = self.start_gen.load(.acquire);
            if (g == seen) continue; // spurious wakeup
            seen = g;
            if (self.shutdown.load(.acquire)) return;
            self.runPartition(wid);
            _ = self.done.fetchAdd(1, .acq_rel);
            futexWake(&self.done, 1);
        }
    }

    /// Evaluate worker `wid`'s strided share of the batch into workers[wid].best.
    fn runPartition(self: *EvalPool, wid: usize) void {
        const w = &self.workers[wid];
        w.best = .{};
        const cands = self.batch.cands;
        var i = wid;
        while (i < cands.len) : (i += self.W) {
            const ri = cands[i];
            if (evalPairInsertPar(self.s, ri, self.batch.p, self.batch.q, self.params, self.batch.iter, w)) |ins| {
                const cand = EvalResult{ .has = true, .delta = ins.delta, .rank = i, .a = ins.a, .b = ins.b };
                if (betterResult(cand, w.best)) w.best = cand;
            }
        }
    }

    /// Small batch: evaluate every candidate inline on worker 0. rank == index,
    /// identical reduction to dispatch() — a pure perf switch, never changes the
    /// result.
    fn evalInline(self: *EvalPool, cands: []const usize, p: usize, q: usize, iter: u64) EvalResult {
        const w = &self.workers[0];
        w.best = .{};
        for (cands, 0..) |ri, i| {
            if (evalPairInsertPar(self.s, ri, p, q, self.params, iter, w)) |ins| {
                const cand = EvalResult{ .has = true, .delta = ins.delta, .rank = i, .a = ins.a, .b = ins.b };
                if (betterResult(cand, w.best)) w.best = cand;
            }
        }
        return w.best;
    }

    /// Release the batch to the helpers, run worker 0's share inline, join, then
    /// reduce the W worker-local bests into the global winner.
    fn dispatch(self: *EvalPool, cands: []const usize, p: usize, q: usize, iter: u64) EvalResult {
        self.batch = .{ .cands = cands, .p = p, .q = q, .iter = iter };
        self.done.store(0, .release);
        _ = self.start_gen.fetchAdd(1, .release); // publishes batch to helpers
        futexWake(&self.start_gen, @intCast(self.W - 1));

        self.runPartition(0);

        while (true) {
            const obs = self.done.load(.acquire);
            if (obs >= self.W - 1) break;
            futexWait(&self.done, obs);
        }

        var g = self.workers[0].best;
        for (self.workers[1..self.W]) |wk| {
            if (betterResult(wk.best, g)) g = wk.best;
        }
        return g;
    }
};

/// Parallel-safe copy of evalPairInsert: reads only immutable batch state of
/// route `ri` (freshened by the discovery pass), blinks from a per-route local
/// PRNG, and uses the per-worker granular scratch. No shared writes, so any
/// number of workers may run this on distinct routes concurrently.
fn evalPairInsertPar(s: *const S, ri: usize, p: usize, q: usize, params: PdpSisrParams, iter: u64, w: *EvalWorker) ?S.Ins {
    var lr = std.Random.DefaultPrng.init(mixSeed(params.seed, iter, p, ri));
    const lrng = lr.random();
    const blink = params.blink;
    const inst = s.inst;
    const rcap: i64 = s.routeCap(ri);
    const r = &s.routes.items[ri];
    const it = r.items.items;
    const L = it.len;
    var best: ?S.Ins = null;
    const q_t = Tws.client(inst, q);
    const q_l = pdp.Lseg.node(inst, q);

    var granular = params.gran_gaps != 0 and (params.max_vehicles == 0 or s.iter_start_complete) and
        L >= params.gran_gap_min_len and s.nonempty <= params.gran_max_routes;
    var genp: u64 = 0;
    if (granular) {
        w.nbr_gen += 1;
        genp = w.nbr_gen;
        for (0..s.gk) |ti| {
            const np = s.gran[(p - 1) * s.gk + ti];
            if (np != 0) w.nbr_mark_p[np] = genp;
            const nq = s.gran[(q - 1) * s.gk + ti];
            if (nq != 0) w.nbr_mark_p[nq] = genp;
        }
        var hits: usize = 0;
        for (it) |c| hits += @intFromBool(w.nbr_mark_p[c] == genp);
        if (hits < 4) granular = false;
    }

    for (s.lockOf(ri)..L + 1) |a| {
        if (granular and (params.gran_gaps & 1) != 0 and a != 0 and a != L and
            w.nbr_mark_p[it[a - 1]] != genp and w.nbr_mark_p[it[a]] != genp) continue;
        const prev_a: usize = if (a == 0) 0 else it[a - 1];
        var m_t = Tws.merge(r.pre_t.items[a], @intCast(inst.d(prev_a, p)), Tws.client(inst, p));
        if (m_t.tw != 0) continue;
        var m_l = pdp.Lseg.merge(r.pre_l.items[a], pdp.Lseg.node(inst, p));
        if (m_l.lo < 0 or m_l.hi > rcap) continue;
        var m_d: u64 = r.pre_d.items[a] + inst.d(prev_a, p);
        var last = p;

        var b = a;
        while (b <= L) : (b += 1) {
            blk: {
                if (lrng.float(f64) < blink) break :blk;
                const nxt: usize = if (b == L) 0 else it[b];
                if (granular and (params.gran_gaps & 2) != 0 and b != a and b != L and
                    w.nbr_mark_p[last] != genp and w.nbr_mark_p[nxt] != genp) break :blk;
                const f1 = Tws.merge(m_t, @intCast(inst.d(last, q)), q_t);
                const f2 = Tws.merge(f1, @intCast(inst.d(q, nxt)), r.suf_t.items[b]);
                if (f2.tw != 0) break :blk;
                const f_l = pdp.Lseg.merge(pdp.Lseg.merge(m_l, q_l), r.suf_l.items[b]);
                if (!(f_l.lo >= 0 and f_l.hi <= rcap)) break :blk;
                if (s.max_route_dur > 0 and f2.dur > @as(i64, @intCast(s.max_route_dur))) break :blk;
                const new_dist = m_d + inst.d(last, q) + inst.d(q, nxt) + r.tail_d.items[b];
                var delta = @as(i64, @intCast(new_dist)) - @as(i64, @intCast(r.dist));
                if (s.time_penalty > 0)
                    delta += @as(i64, @intCast(s.time_penalty)) * (f2.dur - @as(i64, @intCast(r.dur)));
                if (best == null or delta < best.?.delta) best = .{ .a = a, .b = b, .delta = delta };
            }
            if (b < L) {
                m_t = Tws.merge(m_t, @intCast(inst.d(last, it[b])), Tws.client(inst, it[b]));
                if (m_t.tw != 0) break;
                m_l = pdp.Lseg.merge(m_l, pdp.Lseg.node(inst, it[b]));
                if (m_l.lo < 0 or m_l.hi > rcap) break;
                m_d += inst.d(last, it[b]);
                last = it[b];
            }
        }
    }
    return best;
}

/// Parallel-eval copy of recreate: identical shuffle / open-new / singleton /
/// empty-slot logic, but the inner candidate scan is a serial discovery pass
/// (dedup + freshen + ranked collect) followed by a parallel/inline evaluation
/// and a deterministic (delta, rank) reduction. Bit-for-bit equivalent to
/// recreate would require today's shared blink stream (impossible under real
/// parallelism); the winner-selection order is preserved exactly.
fn recreatePar(s: *S, params: PdpSisrParams, rng: std.Random, pool: *EvalPool, iter: u64) !void {
    rng.shuffle(usize, s.removed.items);
    for (s.removed.items) |p| {
        const q = s.inst.pair_of[p];
        s.generation += 1;
        var best_ri: usize = NO_ROUTE;
        var best_ins: S.Ins = undefined;

        while (s.min_empty_hint < s.routes.items.len and s.routes.items[s.min_empty_hint].items.items.len != 0) : (s.min_empty_hint += 1) {}
        const empty_slot: usize = if (s.min_empty_hint < s.routes.items.len) s.min_empty_hint else NO_ROUTE;

        // Serial discovery: dedup candidate routes (anchor-major, ti-minor,
        // first-seen — identical order to recreate), freshen each, and collect
        // into cand_list so the append index is the route's rank.
        s.cand_list.clearRetainingCapacity();
        var est_work: usize = 0;
        for ([_]usize{ p, q }) |anchor| {
            for (0..s.gk) |ti| {
                const nb = s.gran[(anchor - 1) * s.gk + ti];
                if (nb == 0) continue;
                const cri = s.loc_route[nb];
                if (cri == NO_ROUTE) continue;
                if (s.cand_mark.items[cri] == s.generation) continue;
                s.cand_mark.items[cri] = s.generation;
                try s.freshen(cri);
                try s.cand_list.append(s.allocator, cri);
                est_work += s.routes.items[cri].items.items.len;
            }
        }

        const cands = s.cand_list.items;
        if (cands.len != 0) {
            // Auto-gate: inline when the batch is too small to pay for the
            // barrier. Inline and pooled use the same evalPairInsertPar + the
            // same (delta, rank) reduction, so the gate never changes the result.
            const g = if (cands.len < 2 or est_work < PAR_MIN_WORK)
                pool.evalInline(cands, p, q, iter)
            else
                pool.dispatch(cands, p, q, iter);
            if (g.has) {
                best_ri = cands[g.rank];
                best_ins = .{ .a = g.a, .b = g.b, .delta = g.delta };
            }
        }

        const sdur: u64 = if (s.time_penalty > 0 or s.max_route_dur > 0) routeDuration(s.inst, &[_]usize{ p, q }) else 0;
        var may_open = (params.max_vehicles == 0 or s.nonempty < params.max_vehicles) and
            (s.max_route_dur == 0 or sdur <= s.max_route_dur);
        var open_pen: u64 = s.veh_penalty;
        var open_type: u8 = 0;
        if (s.veh_types.len != 0) {
            if (s.chooseType(s.inst.demand_signed[p])) |ti| {
                open_type = ti;
                open_pen = s.veh_types[ti].fixed_cost;
            } else may_open = false;
        }
        var singleton: i64 = @intCast(s.inst.d(0, p) + s.inst.d(p, q) + s.inst.d(q, 0) + open_pen);
        if (s.time_penalty > 0)
            singleton += @intCast(s.time_penalty * sdur);
        if (best_ri == NO_ROUTE and !may_open) {
            if (params.eject and try s.squeezeInsert(p, q, params)) {
                s.n_unassigned -= 1;
            }
            continue; // otherwise stays in the request bank
        }
        if ((best_ri == NO_ROUTE or singleton < best_ins.delta) and may_open) {
            const slot = if (empty_slot != NO_ROUTE) empty_slot else try s.addSlot();
            try s.snapshot(slot);
            if (s.veh_types.len != 0) s.rtype.items[slot] = open_type;
            s.keep_buf.clearRetainingCapacity();
            try s.keep_buf.append(s.allocator, p);
            try s.keep_buf.append(s.allocator, q);
            try s.install(slot, s.keep_buf.items);
        } else {
            try s.insertPair(best_ri, p, q, best_ins.a, best_ins.b);
        }
        s.n_unassigned -= 1;
    }
    s.removed.clearRetainingCapacity();
}

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
    return solvePdptwSisrFrom(allocator, inst, params, &.{});
}

/// Same engine, seeded from `warm` routes instead of the construction heuristic
/// (used by the fleet-min driver so capped descents keep the incumbent's
/// packing instead of restarting cold). `warm` must be a feasible solution;
/// pass empty for the normal cold start.
pub fn solvePdptwSisrFrom(allocator: std.mem.Allocator, inst: pdp.PdpInstance, params: PdpSisrParams, warm: []const []const usize) !pdp.PdpResult {
    return solvePdptwSisrFromLocked(allocator, inst, params, warm, &.{}, false);
}

/// Seeded runner with dispatch locks: `locks[i]` pins the first locks[i]
/// stops of warm[i] (empty = no locking, the plain warm path).
fn solvePdptwSisrFromLocked(allocator: std.mem.Allocator, inst: pdp.PdpInstance, params: PdpSisrParams, warm: []const []const usize, locks: []const usize, partial_ok: bool) !pdp.PdpResult {
    const n_nodes = 2 * inst.n_pairs;
    if (inst.n_pairs == 0) return error.InvalidInstance;
    if (params.veh_types.len > MAX_VEH_TYPES) return error.InvalidInstance;
    if (params.brk) |bk| {
        if (bk.earliest > bk.latest) return error.InvalidInstance;
        // The parallel-eval path has no break-aware evaluator (gated-off
        // lever); refuse the combination instead of silently degrading.
        if (params.eval_threads >= 2) return error.InvalidInstance;
    }

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
    var seed_sol: pdp.Sol = .empty;
    defer pdp.freeSol(allocator, &seed_sol);
    if (warm.len == 0) {
        pdp.freeSol(allocator, &seed_sol);
        seed_sol = try pdp.construct(allocator, inst, pickups, pos);
    } else {
        // Dispatch warms may omit pairs (new orders enter via the request
        // bank), so they get a partial check: the routes GIVEN must be
        // feasible and pair-consistent, completeness is not required.
        const ok = if (partial_ok)
            try validatePartialWarm(allocator, inst, warm)
        else
            pdp.validate(inst, warm) != null;
        if (!ok) return error.InvalidWarmStart;
        for (warm) |r| {
            var route: std.ArrayList(usize) = .empty;
            errdefer route.deinit(allocator);
            try route.appendSlice(allocator, r);
            try seed_sol.append(allocator, route);
        }
    }

    const gk: usize = @min(if (params.gk == 0) @as(usize, 20) else params.gk, if (n_nodes > 1) n_nodes - 1 else 1);
    const gran = try buildNeighbors(allocator, inst, gk, params.nbr_key);
    defer allocator.free(gran);

    var s = try S.init(allocator, inst, params.veh_penalty, params.time_penalty, params.max_route_dur, params.veh_types, params.brk, gran, gk);
    defer s.deinit();

    // Intra-search eval pool: eval_threads <= 1 keeps the untouched serial
    // recreate; >= 2 builds a persistent pool (deferred deinit joins helpers
    // before s.deinit). start() runs after the pool sits at its stable address.
    const ew: usize = if (params.eval_threads <= 1) 0 else params.eval_threads;
    var pool: ?EvalPool = if (ew >= 2) try EvalPool.init(allocator, &s, params, ew) else null;
    defer if (pool) |*pl| pl.deinit(allocator);
    if (pool) |*pl| try pl.start();
    for (seed_sol.items, 0..) |r, wi| {
        if (r.items.len == 0) continue;
        // Heterogeneous fleet: a seed route must find a type (count available,
        // capacity fits its max prefix load) before it may exist. Routes that
        // fit no type start in the request bank instead — except locked
        // dispatch routes, which cannot be banked: no fitting type there is a
        // caller error (their committed load exceeds the declared fleet).
        var seed_type: u8 = 0;
        if (s.veh_types.len != 0) {
            if (s.chooseType(pdp.routeLseg(inst, r.items).hi)) |ti| {
                seed_type = ti;
            } else if (wi < locks.len and locks[wi] > 0) {
                return error.InvalidWarmStart;
            } else continue;
        }
        const ri = try s.addSlot();
        if (s.veh_types.len != 0) s.rtype.items[ri] = seed_type;
        try s.install(ri, r.items);
        if (wi < locks.len) s.lock.items[ri] = locks[wi];
    }

    // Dispatch/partial warms: pairs absent from every seed route start in the
    // request bank (new orders arriving in a rolling-horizon re-solve, which
    // the bank re-derivation feeds back into recreate each iteration).
    // Complete warms and cold constructs leave this at zero.
    for (1..inst.dim()) |c| {
        if (inst.is_pickup[c] and s.loc_route[c] == NO_ROUTE) s.n_unassigned += 1;
    }

    // Under a fleet cap the seed may exceed it: empty the smallest surplus
    // routes into the request bank before the loop starts.
    if (params.max_vehicles > 0) {
        while (s.nonempty > params.max_vehicles) {
            var smallest: usize = NO_ROUTE;
            var slen: usize = std.math.maxInt(usize);
            for (s.routes.items, 0..) |r, i| {
                if (s.lockOf(i) > 0) continue;
                const len = r.items.items.len;
                if (len > 0 and len < slen) {
                    smallest = i;
                    slen = len;
                }
            }
            if (smallest == NO_ROUTE) break;
            _ = try s.removeStringPaired(smallest, 0, slen);
        }
        s.removed.clearRetainingCapacity(); // bank is re-derived each ruin
    }

    var best: ?pdp.PdpResult = if (s.n_unassigned == 0 and s.withinDurCap() and s.brkAllOk()) try s.toResult(allocator) else null;
    errdefer if (best) |*b| b.deinit();
    var best_cost = s.cost;

    var prng = std.Random.DefaultPrng.init(params.seed ^ 0x50_44_50_7457);
    const rng = prng.random();

    // Request-bank penalty: dominates veh_penalty, which dominates distance.
    const UNASSIGNED_PEN: u64 = 1_000_000_000;
    // Over-cap penalty: same magnitude, drives the search out of shift-cap
    // violations. Always 0 when max_route_dur == 0 (overCapCount short-circuits),
    // so the uncapped acceptance objective is bit-identical to before.
    const CAP_PEN: u64 = 1_000_000_000;
    // Fleet cost of the seed: uniform arithmetic unchanged; per-route fixed
    // costs summed when a heterogeneous fleet is active.
    var seed_fleet: u64 = s.veh_penalty * s.nonempty;
    if (s.veh_types.len != 0) {
        seed_fleet = 0;
        for (s.routes.items, 0..) |r, ri| {
            if (r.items.items.len != 0) seed_fleet += s.penOf(ri);
        }
    }
    const seed_distance = s.cost - seed_fleet;
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
        const saved_unassigned = s.n_unassigned;
        const saved_overcap = s.overCapCount() + s.brkViolCount();
        s.saveLedger();
        s.beginIter();
        s.iter_start_complete = s.n_unassigned == 0;
        try s.ruin(params, rng);
        if (pool) |*pl| {
            try recreatePar(&s, params, rng, pl, it);
        } else {
            try s.recreate(params, rng);
        }
        const eff = s.cost + UNASSIGNED_PEN * @as(u64, @intCast(s.n_unassigned)) + CAP_PEN * (s.overCapCount() + s.brkViolCount());
        const saved_eff = saved_cost + UNASSIGNED_PEN * @as(u64, @intCast(saved_unassigned)) + CAP_PEN * saved_overcap;
        const dt = @as(i64, @intCast(eff)) - @as(i64, @intCast(saved_eff));
        if (@as(f64, @floatFromInt(dt)) < temp) {
            if (s.n_unassigned == 0 and s.withinDurCap() and s.brkAllOk() and (best == null or s.cost < best_cost)) {
                best_cost = s.cost;
                if (best) |*b| b.deinit();
                best = try s.toResult(allocator);
            }
        } else {
            try s.rollback(saved_cost, saved_nonempty);
            s.n_unassigned = saved_unassigned;
        }
        temp *= cf;
        if (params.swap_kick > 0 and it % params.swap_kick == params.swap_kick - 1) {
            try s.pairExchangeKick(params, rng);
            if (s.n_unassigned == 0 and s.withinDurCap() and s.brkAllOk() and s.cost < best_cost and best != null) {
                best_cost = s.cost;
                best.?.deinit();
                best = try s.toResult(allocator);
            }
        }
    }
    return best orelse error.NoCompleteSolution;
}

/// Hierarchical fleet minimization (the Li & Lim objective): one uncapped run
/// finds a starting fleet size, then capped runs with a request bank walk the
/// vehicle count down while time remains. Each success becomes the incumbent;
/// the first failed cap stops the descent. total_time_ms is split: half for
/// the uncapped run, the rest per descent attempt.
pub fn solvePdptwSisrFleetMin(allocator: std.mem.Allocator, inst: pdp.PdpInstance, params: PdpSisrParams, total_time_ms: u64) !pdp.PdpResult {
    var p0 = params;
    const p0_pct: u64 = @min(@as(u64, params.fleet_p0_pct), 90);
    p0.time_ms = @max(total_time_ms * p0_pct / 100, 1);
    p0.max_vehicles = 0;
    var best = try solvePdptwSisr(allocator, inst, p0);
    errdefer best.deinit();

    const t_start = nanos();
    var target: usize = best.vehicles;
    descent: while (target > 1) {
        target -= 1;
        const spent = (nanos() - t_start) / std.time.ns_per_ms;
        const budget = total_time_ms * (100 - p0_pct) / 100;
        if (spent + 500 >= budget) break;
        var pk = params;
        pk.max_vehicles = target;
        // half the remaining descent budget per attempt; the terminal polish
        // below gets whatever is left when an attempt fails
        pk.time_ms = @max((budget - spent) / 2, 500);
        // Packing into a below-natural fleet needs bigger ruins than distance
        // polishing (measured on lc103/lc109: cbar 10 fails at 120 s where
        // cbar 16 succeeds in 30 s).
        pk.cbar = @max(pk.cbar, 16.0);
        pk.l_max = @max(pk.l_max, 16);
        const warm = try allocator.alloc([]const usize, best.routes.len);
        defer allocator.free(warm);
        for (best.routes, 0..) |r, i| warm[i] = r;
        // Warm start keeps the incumbent's packing but can trap the descent
        // in its basin (a tight K+1 solution leaves no slack for the banked
        // pairs); a cold construct starts looser. Try warm, then cold.
        pk.time_ms = @max(pk.time_ms / 2, 250);
        var res = solvePdptwSisrFrom(allocator, inst, pk, warm) catch |err| switch (err) {
            error.NoCompleteSolution => blk: {
                break :blk solvePdptwSisr(allocator, inst, pk) catch |err2| switch (err2) {
                    error.NoCompleteSolution => break :descent,
                    else => return err2,
                };
            },
            else => return err,
        };
        if (res.vehicles <= target) {
            best.deinit();
            best = res;
        } else {
            res.deinit();
            break :descent;
        }
    }

    // terminal polish: whatever time remains, warm-started at the final fleet
    const spent = (nanos() - t_start) / std.time.ns_per_ms;
    const p0_ms = @min(p0.time_ms, total_time_ms);
    if (spent + p0_ms + 500 < total_time_ms) {
        var pp = params;
        pp.max_vehicles = best.vehicles;
        pp.time_ms = total_time_ms - p0_ms - spent;
        pp.seed = params.seed +% 0x9E3779B97F4A7C15;
        const warm = try allocator.alloc([]const usize, best.routes.len);
        defer allocator.free(warm);
        for (best.routes, 0..) |r, i| warm[i] = r;
        var res = solvePdptwSisrFrom(allocator, inst, pp, warm) catch |err| switch (err) {
            error.NoCompleteSolution => return best,
            else => return err,
        };
        if (res.vehicles <= best.vehicles and res.total_cost < best.total_cost) {
            best.deinit();
            best = res;
        } else {
            res.deinit();
        }
    }
    return best;
}

/// Feasibility of a PARTIAL warm start (dispatch): every given route must be
/// TW- and capacity-feasible with pairing and precedence satisfied within it,
/// and no node may appear twice — but pairs may be absent entirely (they go
/// to the request bank as new orders).
fn validatePartialWarm(allocator: std.mem.Allocator, inst: pdp.PdpInstance, warm: []const []const usize) !bool {
    const seen = try allocator.alloc(bool, inst.dim());
    defer allocator.free(seen);
    @memset(seen, false);
    for (warm) |r| {
        var acc = Tws.depotNode(inst);
        var prev: usize = 0;
        for (r) |c| {
            if (c == 0 or c >= inst.dim() or seen[c]) return false;
            seen[c] = true;
            acc = Tws.merge(acc, @intCast(inst.d(prev, c)), Tws.client(inst, c));
            if (acc.tw != 0) return false;
            prev = c;
        }
        acc = Tws.merge(acc, @intCast(inst.d(prev, 0)), Tws.depotNode(inst));
        if (acc.tw != 0) return false;
        if (r.len > 0) {
            const lg = pdp.routeLseg(inst, r);
            if (!(lg.lo >= 0 and lg.hi <= inst.capacity)) return false;
        }
        for (r, 0..) |c, i| {
            var found = false;
            if (inst.is_pickup[c]) {
                for (r[i + 1 ..]) |x| found = found or x == inst.pair_of[c];
            } else {
                for (r[0..i]) |x| found = found or x == inst.pair_of[c];
            }
            if (!found) return false;
        }
    }
    return true;
}

/// Rolling-horizon dispatch re-solve: `current[i]` is vehicle i's present
/// route and `locked[i]` how many of its leading stops are committed
/// (in progress or already served) and must not move. Unlocked stops and
/// banked/new pairs are re-optimized around them. Locked prefixes must be
/// pairwise-consistent (a locked dropoff's pickup is locked too) — caller's
/// contract, checked in Debug builds only.
pub fn solvePdptwSisrDispatch(allocator: std.mem.Allocator, inst: pdp.PdpInstance, params: PdpSisrParams, current: []const []const usize, locked: []const usize) !pdp.PdpResult {
    std.debug.assert(locked.len == current.len);
    if (std.debug.runtime_safety) {
        for (current, locked) |r, lk| {
            std.debug.assert(lk <= r.len);
            for (r[0..lk]) |c| {
                if (inst.is_pickup[c]) continue;
                var found = false;
                for (r[0..lk]) |x| found = found or x == inst.pair_of[c];
                std.debug.assert(found);
            }
        }
    }
    return solvePdptwSisrFromLocked(allocator, inst, params, current, locked, true);
}

/// Enterprise pinned-fleet driver: find the best solution using at most
/// `pin` vehicles, spending the whole budget on that goal. Route: short
/// uncapped warm-up (finds a packing basin), descent from the warm-up fleet
/// down to `pin` (warm then cold per step, like fleet-min, but a failed
/// attempt RETRIES with fresh seeds while budget remains instead of giving
/// up), then all remaining time polishes distance at the pinned cap, warm
/// from the incumbent. Returns error.NoCompleteSolution if `pin` vehicles
/// were never reached within the budget.
pub fn solvePdptwSisrPinned(allocator: std.mem.Allocator, inst: pdp.PdpInstance, params: PdpSisrParams, total_time_ms: u64, pin: usize) !pdp.PdpResult {
    const t_start = nanos();
    var p0 = params;
    p0.time_ms = @max(total_time_ms * 25 / 100, 1);
    p0.max_vehicles = 0;
    var best = try solvePdptwSisr(allocator, inst, p0);
    errdefer best.deinit();

    // Descent toward pin. Unlike fleet-min, a failed target is retried with a
    // fresh seed while budget remains: reaching pin is the whole objective,
    // so giving up on the first failure would waste the rest of the budget.
    // A quarter of the time left to the 85% mark per attempt (half warm, half
    // cold): measured against half-of-remaining attempts, MORE retries beat
    // BIGGER attempts on giant-route pins (they are seed-luck-dominated —
    // halving lost lrc2_2_10's pin outright and cost 1.7% on lr2_6_4).
    var seed = params.seed;
    const descent_limit = total_time_ms * 85 / 100;
    while (best.vehicles > pin and best.vehicles > 1) {
        const spent = (nanos() - t_start) / std.time.ns_per_ms;
        if (spent + 500 >= descent_limit) break;
        const target = best.vehicles - 1;
        var pk = params;
        pk.seed = seed;
        pk.max_vehicles = target;
        pk.cbar = @max(pk.cbar, 16.0);
        pk.l_max = @max(pk.l_max, 16);
        pk.eject = true;
        pk.time_ms = @max((descent_limit - spent) / 4, 1000) / 2;
        const warm = try allocator.alloc([]const usize, best.routes.len);
        defer allocator.free(warm);
        for (best.routes, 0..) |r, i| warm[i] = r;
        var res = solvePdptwSisrFrom(allocator, inst, pk, warm) catch |err| switch (err) {
            error.NoCompleteSolution => solvePdptwSisr(allocator, inst, pk) catch |err2| switch (err2) {
                error.NoCompleteSolution => {
                    seed +%= 0x9E3779B97F4A7C15;
                    continue;
                },
                else => return err2,
            },
            else => return err,
        };
        if (res.vehicles <= target) {
            best.deinit();
            best = res;
        } else {
            res.deinit();
            seed +%= 0x9E3779B97F4A7C15;
        }
    }
    if (best.vehicles > pin) return error.NoCompleteSolution;

    // Final polish: all remaining time at the pinned cap, warm from the
    // incumbent. veh_penalty stays as given so hierarchical cost ordering is
    // preserved.
    const spent = (nanos() - t_start) / std.time.ns_per_ms;
    if (spent + 500 < total_time_ms) {
        var pp = params;
        pp.max_vehicles = pin;
        pp.time_ms = total_time_ms - spent;
        pp.seed = seed +% 0x9E3779B97F4A7C15;
        const warm = try allocator.alloc([]const usize, best.routes.len);
        defer allocator.free(warm);
        for (best.routes, 0..) |r, i| warm[i] = r;
        var res = solvePdptwSisrFrom(allocator, inst, pp, warm) catch |err| switch (err) {
            error.NoCompleteSolution => return best,
            else => return err,
        };
        if (res.vehicles <= pin and (res.vehicles < best.vehicles or res.total_cost < best.total_cost)) {
            best.deinit();
            best = res;
        } else {
            res.deinit();
        }
    }
    return best;
}

// Worker slot for the parallel fleet-min driver: one independent SISR run per
// thread, results flattened into parent-owned buffers (nothing crosses threads
// except these slices).
const FmTask = struct {
    inst: pdp.PdpInstance,
    params: PdpSisrParams,
    warm: []const []const usize = &.{},
    order: []usize, // parent-owned, flat route nodes
    ends: []usize, // parent-owned, route end offsets
    nroutes: usize = 0,
    dist: u64 = 0,
    veh: usize = 0,
    ok: bool = false,
};

fn fmWorker(t: *FmTask) void {
    // Not an arena: the engine's per-iteration churn (route snapshots, squeeze
    // undo copies) is alloc/free-balanced, and an arena retains all of it —
    // measured 2.8 GB RSS in 5 minutes at hour-long walls.
    var res = solvePdptwSisrFrom(std.heap.smp_allocator, t.inst, t.params, t.warm) catch {
        t.ok = false;
        return;
    };
    defer res.deinit();
    var w: usize = 0;
    for (res.routes, 0..) |route, ri| {
        @memcpy(t.order[w .. w + route.len], route);
        w += route.len;
        t.ends[ri] = w;
    }
    t.nroutes = res.routes.len;
    t.veh = res.vehicles;
    t.dist = res.total_cost;
    t.ok = true;
}

fn fmWave(allocator: std.mem.Allocator, tasks: []FmTask) !void {
    const ths = try allocator.alloc(std.Thread, tasks.len);
    defer allocator.free(ths);
    var spawned: usize = 0;
    for (tasks, 0..) |*t, i| {
        ths[i] = std.Thread.spawn(.{}, fmWorker, .{t}) catch break;
        spawned += 1;
    }
    for (tasks[spawned..]) |*t| fmWorker(t);
    for (ths[0..spawned]) |th| th.join();
}

fn fmSlotResult(allocator: std.mem.Allocator, t: FmTask) !pdp.PdpResult {
    const routes = try allocator.alloc([]usize, t.nroutes);
    var filled: usize = 0;
    errdefer {
        for (routes[0..filled]) |r| allocator.free(r);
        allocator.free(routes);
    }
    var start: usize = 0;
    for (0..t.nroutes) |ri| {
        routes[ri] = try allocator.dupe(usize, t.order[start..t.ends[ri]]);
        filled += 1;
        start = t.ends[ri];
    }
    return .{ .allocator = allocator, .routes = routes, .total_cost = t.dist, .vehicles = t.veh };
}

/// Parallel fleet minimization: each descent step runs `threads` independent
/// capped attempts as one wave (warm incumbent, cold restarts, plus one
/// deeper-cap probe), so one wave's wall buys several serial attempts and a
/// success can skip a vehicle. Phase 1 (uncapped) and the terminal polish run
/// as seed-diverse waves too. threads <= 1 is exactly the serial driver.
/// Deterministic for a fixed (seed, threads).
pub fn solvePdptwSisrFleetMinParallel(allocator: std.mem.Allocator, inst: pdp.PdpInstance, params: PdpSisrParams, total_time_ms: u64, threads: usize) !pdp.PdpResult {
    const cpus = std.Thread.getCpuCount() catch 1;
    const k = if (threads == 0) @max(@as(usize, 1), cpus -| 1) else threads;
    if (k <= 1 or inst.n_pairs == 0) return solvePdptwSisrFleetMin(allocator, inst, params, total_time_ms);
    const n2 = 2 * inst.n_pairs;
    const SEED_STRIDE: u64 = 0x9E3779B97F4A7C15;

    const tasks = try allocator.alloc(FmTask, k);
    defer allocator.free(tasks);
    var bufs_alloc: usize = 0;
    defer for (tasks[0..bufs_alloc]) |t| {
        allocator.free(t.order);
        allocator.free(t.ends);
    };
    for (tasks) |*t| {
        t.* = .{
            .inst = inst,
            .params = params,
            .order = try allocator.alloc(usize, n2),
            .ends = try allocator.alloc(usize, inst.n_pairs),
        };
        bufs_alloc += 1;
    }
    const warm_buf = try allocator.alloc([]const usize, inst.n_pairs);
    defer allocator.free(warm_buf);

    // phase 1: seed-diverse uncapped wave
    for (tasks, 0..) |*t, i| {
        t.params = params;
        t.params.max_vehicles = 0;
        t.params.time_ms = @max(total_time_ms * @min(@as(u64, params.fleet_p0_pct), 90) / 100, 1);
        t.params.seed = params.seed +% @as(u64, @intCast(i)) *% SEED_STRIDE;
        t.warm = &.{};
        t.ok = false;
    }
    try fmWave(allocator, tasks);
    var best: ?pdp.PdpResult = null;
    errdefer if (best) |*b| b.deinit();
    var win: ?usize = null;
    for (tasks, 0..) |t, i| {
        if (!t.ok) continue;
        if (win == null or t.veh < tasks[win.?].veh or (t.veh == tasks[win.?].veh and t.dist < tasks[win.?].dist)) win = i;
    }
    best = try fmSlotResult(allocator, tasks[win orelse return error.NoCompleteSolution]);

    // phase 2: descent waves
    const t_start = nanos();
    var wave_no: u64 = 0;
    descent: while (best.?.vehicles > 1) {
        const target = best.?.vehicles - 1;
        const spent = (nanos() - t_start) / std.time.ns_per_ms;
        const budget = total_time_ms * (100 - @min(@as(u64, params.fleet_p0_pct), 90)) / 100;
        if (spent + 500 >= budget) break;
        wave_no += 1;
        for (best.?.routes, 0..) |r, i| warm_buf[i] = r;
        const warm = warm_buf[0..best.?.routes.len];
        for (tasks, 0..) |*t, i| {
            t.params = params;
            t.params.time_ms = @max((budget - spent) / 2, 500);
            t.params.cbar = @max(t.params.cbar, 16.0);
            t.params.l_max = @max(t.params.l_max, 16);
            t.params.seed = params.seed +% wave_no *% 0xD1B54A32D192ED03 +% @as(u64, @intCast(i)) *% SEED_STRIDE;
            // slot 0 warm at K-1, last slot probes K-2 cold, the rest cold at K-1
            const deeper = k >= 3 and i == k - 1 and target > 1;
            t.params.max_vehicles = if (deeper) target - 1 else target;
            t.warm = if (i == 0) warm else &.{};
            t.ok = false;
        }
        try fmWave(allocator, tasks);
        win = null;
        for (tasks, 0..) |t, i| {
            if (!t.ok or t.veh > target) continue;
            if (win == null or t.veh < tasks[win.?].veh or (t.veh == tasks[win.?].veh and t.dist < tasks[win.?].dist)) win = i;
        }
        const w = win orelse break :descent;
        const res = try fmSlotResult(allocator, tasks[w]);
        best.?.deinit();
        best = res;
    }

    // phase 3: seed-diverse warm polish at the final fleet
    const spent = (nanos() - t_start) / std.time.ns_per_ms;
    const p0_ms = @min(@max(total_time_ms * @min(@as(u64, params.fleet_p0_pct), 90) / 100, 1), total_time_ms);
    if (spent + p0_ms + 500 < total_time_ms) {
        for (best.?.routes, 0..) |r, i| warm_buf[i] = r;
        const warm = warm_buf[0..best.?.routes.len];
        for (tasks, 0..) |*t, i| {
            t.params = params;
            t.params.max_vehicles = best.?.vehicles;
            t.params.time_ms = total_time_ms - p0_ms - spent;
            t.params.seed = params.seed +% 0xA0761D6478BD642F +% @as(u64, @intCast(i)) *% SEED_STRIDE;
            t.warm = warm;
            t.ok = false;
        }
        try fmWave(allocator, tasks);
        win = null;
        for (tasks, 0..) |t, i| {
            if (!t.ok or t.veh > best.?.vehicles) continue;
            if (win == null or t.veh < tasks[win.?].veh or (t.veh == tasks[win.?].veh and t.dist < tasks[win.?].dist)) win = i;
        }
        if (win) |w| {
            const t = tasks[w];
            if (t.veh < best.?.vehicles or (t.veh == best.?.vehicles and t.dist < best.?.total_cost)) {
                const res = try fmSlotResult(allocator, t);
                best.?.deinit();
                best = res;
            }
        }
    }
    return best.?;
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

test "PDPTW SISR: fleet cap succeeds at the uncapped count and fails at 1" {
    const allocator = std.testing.allocator;
    var ri = try pdp.randomInstance(allocator, 8, 3, true);
    defer ri.deinit(allocator);
    const inst = ri.inst();

    var free_run = try solvePdptwSisr(allocator, inst, .{ .seed = 3, .iters = 2000, .veh_penalty = 10_000_000 });
    defer free_run.deinit();

    // same cap as the uncapped result: must succeed with <= that many routes
    var capped = try solvePdptwSisr(allocator, inst, .{ .seed = 3, .iters = 2000, .veh_penalty = 10_000_000, .max_vehicles = free_run.vehicles });
    defer capped.deinit();
    try std.testing.expect(capped.vehicles <= free_run.vehicles);
    const rc = try allocator.alloc([]const usize, capped.routes.len);
    defer allocator.free(rc);
    for (capped.routes, 0..) |r, i| rc[i] = r;
    const vc = pdp.validate(inst, rc) orelse return error.Infeasible;
    try std.testing.expectEqual(vc, capped.total_cost);

    // one vehicle cannot serve 8 pairs under these windows
    try std.testing.expectError(error.NoCompleteSolution, solvePdptwSisr(allocator, inst, .{ .seed = 3, .iters = 300, .veh_penalty = 10_000_000, .max_vehicles = 1 }));
}

test "PDPTW SISR: parallel fleet-min validates, deterministic, threads=1 == serial" {
    const allocator = std.testing.allocator;
    var ri = try pdp.randomInstance(allocator, 8, 11, true);
    defer ri.deinit(allocator);
    const inst = ri.inst();
    const params = PdpSisrParams{ .seed = 11, .iters = 100_000_000, .veh_penalty = 10_000_000 };

    var a = try solvePdptwSisrFleetMinParallel(allocator, inst, params, 1500, 3);
    defer a.deinit();
    const rc = try allocator.alloc([]const usize, a.routes.len);
    defer allocator.free(rc);
    for (a.routes, 0..) |r, i| rc[i] = r;
    const vc = pdp.validate(inst, rc) orelse return error.Infeasible;
    try std.testing.expectEqual(vc, a.total_cost);

    // threads <= 1 must be exactly the serial driver
    var s1 = try solvePdptwSisrFleetMinParallel(allocator, inst, params, 800, 1);
    defer s1.deinit();
    var s2 = try solvePdptwSisrFleetMin(allocator, inst, params, 800);
    defer s2.deinit();
    try std.testing.expectEqual(s2.vehicles, s1.vehicles);
    try std.testing.expectEqual(s2.total_cost, s1.total_cost);
}

test "PDPTW SISR: fleet-min driver output validates and never exceeds uncapped" {
    const allocator = std.testing.allocator;
    var ri = try pdp.randomInstance(allocator, 8, 7, true);
    defer ri.deinit(allocator);
    const inst = ri.inst();
    var free_run = try solvePdptwSisr(allocator, inst, .{ .seed = 7, .iters = 2000, .veh_penalty = 10_000_000 });
    defer free_run.deinit();
    var res = try solvePdptwSisrFleetMin(allocator, inst, .{ .seed = 7, .iters = 100_000_000, .veh_penalty = 10_000_000 }, 2000);
    defer res.deinit();
    try std.testing.expect(res.vehicles <= free_run.vehicles);
    const rc = try allocator.alloc([]const usize, res.routes.len);
    defer allocator.free(rc);
    for (res.routes, 0..) |r, i| rc[i] = r;
    const vc = pdp.validate(inst, rc) orelse return error.Infeasible;
    try std.testing.expectEqual(vc, res.total_cost);
}

test "PDPTW SISR: capped run below the natural fleet churns the bank without corruption" {
    // Regression for the stale-loc rollback bug: a banked pair inserted in a
    // rejected iteration kept its old position and the next ruin indexed past
    // the route end (debug: integer overflow; ReleaseFast: segfault).
    const allocator = std.testing.allocator;
    var seed: u64 = 1;
    while (seed <= 5) : (seed += 1) {
        var ri = try pdp.randomInstanceClocked(allocator, 10, seed);
        defer ri.deinit(allocator);
        const inst = ri.inst();
        var free_run = try solvePdptwSisr(allocator, inst, .{ .seed = seed, .iters = 800, .veh_penalty = 10_000_000 });
        const target = if (free_run.vehicles > 1) free_run.vehicles - 1 else 1;
        free_run.deinit();
        var res = solvePdptwSisr(allocator, inst, .{ .seed = seed, .iters = 4000, .veh_penalty = 10_000_000, .max_vehicles = target }) catch |err| switch (err) {
            error.NoCompleteSolution => continue, // legal outcome; the point is not crashing
            else => return err,
        };
        defer res.deinit();
        try std.testing.expect(res.vehicles <= target);
        const rc = try allocator.alloc([]const usize, res.routes.len);
        defer allocator.free(rc);
        for (res.routes, 0..) |r, i| rc[i] = r;
        const vc = pdp.validate(inst, rc) orelse return error.Infeasible;
        try std.testing.expectEqual(vc, res.total_cost);
    }
}

test "PDPTW SISR eject: heavy squeeze churn stays valid and cost-honest" {
    // At cap = natural-1 on random non-metric instances the squeeze path
    // fires tens of thousands of times (measured ~38k/20k iters), constantly
    // inserting-with-violation, ejecting, and undoing across accepted AND
    // rejected iterations. Most of these caps are genuinely infeasible
    // (single-route TW packing), so solving is NOT required here — the test
    // pins state integrity under maximum squeeze churn: no crash, no stale
    // loc, and any returned solution passes the independent oracle at the
    // exact reported cost. Effectiveness is measured on Li & Lim outside the
    // suite.
    const allocator = std.testing.allocator;
    var seed: u64 = 1;
    while (seed <= 10) : (seed += 1) {
        var ri = try pdp.randomInstance(allocator, 8, seed, true);
        defer ri.deinit(allocator);
        const inst = ri.inst();
        var free_run = try solvePdptwSisr(allocator, inst, .{ .seed = seed, .iters = 2000, .veh_penalty = 10_000_000 });
        defer free_run.deinit();
        if (free_run.vehicles <= 1) continue;
        var res = solvePdptwSisr(allocator, inst, .{
            .seed = seed,
            .iters = 20000,
            .veh_penalty = 10_000_000,
            .max_vehicles = free_run.vehicles - 1,
            .eject = true,
        }) catch |err| switch (err) {
            error.NoCompleteSolution => continue, // legal; the point is not corrupting
            else => return err,
        };
        defer res.deinit();
        try std.testing.expect(res.vehicles <= free_run.vehicles - 1);
        const rc = try allocator.alloc([]const usize, res.routes.len);
        defer allocator.free(rc);
        for (res.routes, 0..) |r, i| rc[i] = r;
        const vc = pdp.validate(inst, rc) orelse return error.Infeasible;
        try std.testing.expectEqual(vc, res.total_cost);
    }
}

test "PDPTW SISR eject: off by default leaves engine bit-identical" {
    const allocator = std.testing.allocator;
    var ri = try pdp.randomInstance(allocator, 10, 5, true);
    defer ri.deinit(allocator);
    const inst = ri.inst();
    var a = try solvePdptwSisr(allocator, inst, .{ .seed = 5, .iters = 3000, .veh_penalty = 10_000_000 });
    defer a.deinit();
    var b = try solvePdptwSisr(allocator, inst, .{ .seed = 5, .iters = 3000, .veh_penalty = 10_000_000, .eject = false });
    defer b.deinit();
    try std.testing.expectEqual(a.total_cost, b.total_cost);
    try std.testing.expectEqual(a.vehicles, b.vehicles);
}

test "PDPTW SISR eject_k: constructed 2-eject unlocks what single eject cannot" {
    // Hand-built route: pairs A(1,2) and B(3,4), demand 5 each, are already
    // resident; pair C(5,6), demand 6, is squeezed in ahead of both (as
    // squeezeInsert leaves the route after insertPair, before ejection).
    // Stacking all three pickups before any dropoff peaks the load at
    // 5+5+6=16 against capacity 10. Removing either resident pair alone
    // still leaves 5+6=11 > 10; removing BOTH leaves just C's own 6 <= 10.
    // Wide TWs so only load binds, per the squeeze motivation.
    const allocator = std.testing.allocator;
    var matrix: [49]u32 = undefined;
    for (0..7) |a| for (0..7) |b| {
        matrix[a * 7 + b] = if (a == b) 0 else 10;
    };
    const inst = pdp.PdpInstance{
        .n_pairs = 3,
        .matrix = matrix[0..],
        .capacity = 10,
        .pair_of = &[_]usize{ 0, 2, 1, 4, 3, 6, 5 },
        .is_pickup = &[_]bool{ false, true, false, true, false, true, false },
        .demand_signed = &[_]i64{ 0, 5, -5, 5, -5, 6, -6 },
        .ready = &[_]u32{ 0, 0, 0, 0, 0, 0, 0 },
        .due = &[_]u32{ 100_000, 100_000, 100_000, 100_000, 100_000, 100_000, 100_000 },
        .service = &[_]u32{ 0, 0, 0, 0, 0, 0, 0 },
    };

    const gran = try buildNeighbors(allocator, inst, 6, .sum);
    defer allocator.free(gran);
    var s = try S.init(allocator, inst, 10_000_000, 0, 0, &.{}, null, gran, 6);
    defer s.deinit();
    const ri = try s.addSlot();
    try s.install(ri, &[_]usize{ 1, 3, 5, 2, 4, 6 });

    // rung 1: no single resident pair restores feasibility
    try std.testing.expectEqual(@as(?usize, null), try s.ejectCandidate(ri, 5));
    // rung 2: ejecting both A and B does
    const sub = (try s.ejectSubsetK(ri, 5, 2)) orelse return error.TestExpectedEject;
    try std.testing.expectEqual(@as(usize, 2), sub.len);
    var got = [_]usize{ sub.buf[0], sub.buf[1] };
    std.mem.sort(usize, &got, {}, std.sort.asc(usize));
    try std.testing.expectEqualSlices(usize, &[_]usize{ 1, 3 }, &got);
}

test "PDPTW SISR eject_k: default 1 leaves engine bit-identical" {
    const allocator = std.testing.allocator;
    var seed: u64 = 1;
    while (seed <= 5) : (seed += 1) {
        var ri = try pdp.randomInstance(allocator, 8, seed, true);
        defer ri.deinit(allocator);
        const inst = ri.inst();
        var free_run = try solvePdptwSisr(allocator, inst, .{ .seed = seed, .iters = 2000, .veh_penalty = 10_000_000 });
        defer free_run.deinit();
        if (free_run.vehicles <= 1) continue;
        const base = PdpSisrParams{
            .seed = seed,
            .iters = 8000,
            .veh_penalty = 10_000_000,
            .max_vehicles = free_run.vehicles - 1,
            .eject = true,
        };
        var explicit_k1 = base;
        explicit_k1.eject_k = 1;
        const ra = solvePdptwSisr(allocator, inst, base);
        const rb = solvePdptwSisr(allocator, inst, explicit_k1);
        if (ra) |a_ok| {
            var a = a_ok;
            defer a.deinit();
            var b = try rb;
            defer b.deinit();
            try std.testing.expectEqual(a.total_cost, b.total_cost);
            try std.testing.expectEqual(a.vehicles, b.vehicles);
        } else |err| {
            try std.testing.expectError(err, rb);
        }
    }
}

test "PDPTW SISR eject_k: deep ejection churn (k=3) stays valid and cost-honest" {
    // Same harness as the heavy-squeeze-churn test above, but with the
    // ladder's deepest rung enabled: asserts no crash, no stale loc, and any
    // returned solution passes the independent oracle at the exact reported
    // cost under maximum 3-eject churn. Effectiveness is measured on Li &
    // Lim outside the suite.
    const allocator = std.testing.allocator;
    var seed: u64 = 1;
    while (seed <= 10) : (seed += 1) {
        var ri = try pdp.randomInstance(allocator, 8, seed, true);
        defer ri.deinit(allocator);
        const inst = ri.inst();
        var free_run = try solvePdptwSisr(allocator, inst, .{ .seed = seed, .iters = 2000, .veh_penalty = 10_000_000 });
        defer free_run.deinit();
        if (free_run.vehicles <= 1) continue;
        var res = solvePdptwSisr(allocator, inst, .{
            .seed = seed,
            .iters = 20000,
            .veh_penalty = 10_000_000,
            .max_vehicles = free_run.vehicles - 1,
            .eject = true,
            .eject_k = 3,
        }) catch |err| switch (err) {
            error.NoCompleteSolution => continue, // legal; the point is not corrupting
            else => return err,
        };
        defer res.deinit();
        try std.testing.expect(res.vehicles <= free_run.vehicles - 1);
        const rc = try allocator.alloc([]const usize, res.routes.len);
        defer allocator.free(rc);
        for (res.routes, 0..) |r, i| rc[i] = r;
        const vc = pdp.validate(inst, rc) orelse return error.Infeasible;
        try std.testing.expectEqual(vc, res.total_cost);
    }
}

test "PDPTW SISR swap kick: stays valid and never worse than kick-off at fixed seed count" {
    const allocator = std.testing.allocator;
    var seed: u64 = 1;
    while (seed <= 10) : (seed += 1) {
        var ri = try pdp.randomInstance(allocator, 10, seed, true);
        defer ri.deinit(allocator);
        const inst = ri.inst();
        var res = try solvePdptwSisr(allocator, inst, .{ .seed = seed, .iters = 3000, .veh_penalty = 10_000_000, .swap_kick = 50 });
        defer res.deinit();
        const rc = try allocator.alloc([]const usize, res.routes.len);
        defer allocator.free(rc);
        for (res.routes, 0..) |r, i| rc[i] = r;
        const vc = pdp.validate(inst, rc) orelse return error.Infeasible;
        try std.testing.expectEqual(vc, res.total_cost);
    }
}

test "PDPTW SISR swap kick: off by default leaves engine bit-identical" {
    const allocator = std.testing.allocator;
    var ri = try pdp.randomInstance(allocator, 10, 9, true);
    defer ri.deinit(allocator);
    const inst = ri.inst();
    var a = try solvePdptwSisr(allocator, inst, .{ .seed = 9, .iters = 3000, .veh_penalty = 10_000_000 });
    defer a.deinit();
    var b = try solvePdptwSisr(allocator, inst, .{ .seed = 9, .iters = 3000, .veh_penalty = 10_000_000, .swap_kick = 0 });
    defer b.deinit();
    try std.testing.expectEqual(a.total_cost, b.total_cost);
    try std.testing.expectEqual(a.vehicles, b.vehicles);
}

test "PDPTW SISR pinned: reaches pin at natural fleet" {
    const allocator = std.testing.allocator;
    var ri = try pdp.randomInstance(allocator, 8, 3, true);
    defer ri.deinit(allocator);
    const inst = ri.inst();
    var free_run = try solvePdptwSisr(allocator, inst, .{ .seed = 3, .iters = 2000, .veh_penalty = 10_000_000 });
    defer free_run.deinit();
    var res = try solvePdptwSisrPinned(allocator, inst, .{ .seed = 3, .iters = 100_000_000, .veh_penalty = 10_000_000 }, 1500, free_run.vehicles);
    defer res.deinit();
    try std.testing.expect(res.vehicles <= free_run.vehicles);
    const rc = try allocator.alloc([]const usize, res.routes.len);
    defer allocator.free(rc);
    for (res.routes, 0..) |r, i| rc[i] = r;
    const vc = pdp.validate(inst, rc) orelse return error.Infeasible;
    try std.testing.expectEqual(vc, res.total_cost);
}

test "PDPTW SISR pinned: pin above natural returns immediately-valid result" {
    const allocator = std.testing.allocator;
    var ri = try pdp.randomInstance(allocator, 10, 7, true);
    defer ri.deinit(allocator);
    const inst = ri.inst();
    var free_run = try solvePdptwSisr(allocator, inst, .{ .seed = 7, .iters = 2000, .veh_penalty = 10_000_000 });
    defer free_run.deinit();
    var res = try solvePdptwSisrPinned(allocator, inst, .{ .seed = 7, .iters = 100_000_000, .veh_penalty = 10_000_000 }, 1000, free_run.vehicles + 3);
    defer res.deinit();
    try std.testing.expect(res.vehicles <= free_run.vehicles + 3);
    const rc = try allocator.alloc([]const usize, res.routes.len);
    defer allocator.free(rc);
    for (res.routes, 0..) |r, i| rc[i] = r;
    const vc = pdp.validate(inst, rc) orelse return error.Infeasible;
    try std.testing.expectEqual(vc, res.total_cost);
}

test "PDPTW SISR pinned: impossible pin errors" {
    const allocator = std.testing.allocator;
    // n_pairs=20: the smallest size where these random tight-TW instances
    // need 3+ vehicles (12..16 pairs pack into 2)
    var ri = try pdp.randomInstance(allocator, 20, 1, true);
    defer ri.deinit(allocator);
    const inst = ri.inst();
    // honesty gate: this instance genuinely needs several vehicles, so pin=1
    // is out of reach for the descent no matter the seed
    var free_run = try solvePdptwSisr(allocator, inst, .{ .seed = 1, .iters = 2000, .veh_penalty = 10_000_000 });
    defer free_run.deinit();
    try std.testing.expect(free_run.vehicles >= 3);
    try std.testing.expectError(error.NoCompleteSolution, solvePdptwSisrPinned(allocator, inst, .{ .seed = 1, .iters = 100_000_000, .veh_penalty = 10_000_000 }, 800, 1));
}

test "PDPTW SISR pinned: deterministic for a fixed seed" {
    // iters-bounded (600 iters on 10 pairs finishes far inside every phase's
    // wall cap), so the time-based phase splits never bind and two calls walk
    // identical trajectories.
    const allocator = std.testing.allocator;
    var ri = try pdp.randomInstance(allocator, 10, 42, true);
    defer ri.deinit(allocator);
    const inst = ri.inst();
    const params = PdpSisrParams{ .seed = 7, .iters = 600, .veh_penalty = 10_000_000 };
    var free_run = try solvePdptwSisr(allocator, inst, .{ .seed = 7, .iters = 600, .veh_penalty = 10_000_000 });
    defer free_run.deinit();
    var a = try solvePdptwSisrPinned(allocator, inst, params, 2000, free_run.vehicles);
    defer a.deinit();
    var b = try solvePdptwSisrPinned(allocator, inst, params, 2000, free_run.vehicles);
    defer b.deinit();
    try std.testing.expectEqual(a.total_cost, b.total_cost);
    try std.testing.expectEqual(a.vehicles, b.vehicles);
}

test "PDPTW SISR gran under cap: capped complete run stays valid and cost-honest" {
    const allocator = std.testing.allocator;
    var ri = try pdp.randomInstance(allocator, 10, 3, true);
    defer ri.deinit(allocator);
    const inst = ri.inst();
    var free_run = try solvePdptwSisr(allocator, inst, .{ .seed = 3, .iters = 2000, .veh_penalty = 10_000_000 });
    defer free_run.deinit();
    var res = try solvePdptwSisr(allocator, inst, .{
        .seed = 3,
        .iters = 4000,
        .veh_penalty = 10_000_000,
        .max_vehicles = free_run.vehicles,
        .gran_gaps = 2,
        .eject = true,
    });
    defer res.deinit();
    try std.testing.expect(res.vehicles <= free_run.vehicles);
    const rc = try allocator.alloc([]const usize, res.routes.len);
    defer allocator.free(rc);
    for (res.routes, 0..) |r, i| rc[i] = r;
    const vc = pdp.validate(inst, rc) orelse return error.Infeasible;
    try std.testing.expectEqual(vc, res.total_cost);
}

test "PDPTW SISR gran under cap: gran off stays bit-identical under cap" {
    const allocator = std.testing.allocator;
    var ri = try pdp.randomInstance(allocator, 10, 3, true);
    defer ri.deinit(allocator);
    const inst = ri.inst();
    var free_run = try solvePdptwSisr(allocator, inst, .{ .seed = 3, .iters = 2000, .veh_penalty = 10_000_000 });
    defer free_run.deinit();
    var a = try solvePdptwSisr(allocator, inst, .{ .seed = 3, .iters = 4000, .veh_penalty = 10_000_000, .max_vehicles = free_run.vehicles });
    defer a.deinit();
    var b = try solvePdptwSisr(allocator, inst, .{ .seed = 3, .iters = 4000, .veh_penalty = 10_000_000, .max_vehicles = free_run.vehicles, .gran_gaps = 0 });
    defer b.deinit();
    try std.testing.expectEqual(a.total_cost, b.total_cost);
    try std.testing.expectEqual(a.vehicles, b.vehicles);
}

test "PDPTW SISR time penalty: money mode is valid and deterministic" {
    const allocator = std.testing.allocator;
    var ri = try pdp.randomInstance(allocator, 10, 5, true);
    defer ri.deinit(allocator);
    const inst = ri.inst();
    var a = try solvePdptwSisr(allocator, inst, .{ .seed = 5, .iters = 4000, .veh_penalty = 10_000_000, .time_penalty = 500 });
    defer a.deinit();
    const rc = try allocator.alloc([]const usize, a.routes.len);
    defer allocator.free(rc);
    for (a.routes, 0..) |r, i| rc[i] = r;
    const vc = pdp.validate(inst, rc) orelse return error.Infeasible;
    try std.testing.expectEqual(vc, a.total_cost);
    // duration of a real route is at least its service content
    var dur: u64 = 0;
    for (a.routes) |r| dur += routeDuration(inst, r);
    var service: u64 = 0;
    for (1..inst.dim()) |c| service += inst.service[c];
    try std.testing.expect(dur >= service);
    var b = try solvePdptwSisr(allocator, inst, .{ .seed = 5, .iters = 4000, .veh_penalty = 10_000_000, .time_penalty = 500 });
    defer b.deinit();
    try std.testing.expectEqual(a.total_cost, b.total_cost);
    try std.testing.expectEqual(a.vehicles, b.vehicles);
}

test "PDPTW SISR dispatch: locked prefixes survive re-solve" {
    const allocator = std.testing.allocator;
    var teeth = false;
    var seed: u64 = 21;
    while (seed <= 25) : (seed += 1) {
        var ri = try pdp.randomInstance(allocator, 10, seed, true);
        defer ri.deinit(allocator);
        const inst = ri.inst();
        var base = try solvePdptwSisr(allocator, inst, .{ .seed = seed, .iters = 3000 });
        defer base.deinit();

        const cur = try allocator.alloc([]const usize, base.routes.len);
        defer allocator.free(cur);
        for (base.routes, 0..) |r, i| cur[i] = r;
        const locked = try allocator.alloc(usize, base.routes.len);
        defer allocator.free(locked);
        @memset(locked, 0);

        // Lock the longest proper pair-closed prefix of the first route with
        // >= 4 items (pair-closed: every locked node's partner is inside the
        // prefix, i.e. no pickup in it is still open).
        for (cur, 0..) |r, i| {
            if (r.len < 4) continue;
            var open: usize = 0;
            var k: usize = 0;
            for (r, 0..) |c, pos| {
                if (inst.is_pickup[c]) open += 1 else open -= 1;
                if (open == 0 and pos + 1 < r.len) k = pos + 1;
            }
            locked[i] = k;
            if (k > 0) teeth = true;
            break;
        }

        var res = try solvePdptwSisrDispatch(allocator, inst, .{ .seed = seed + 100, .iters = 4000 }, cur, locked);
        defer res.deinit();
        const rc = try allocator.alloc([]const usize, res.routes.len);
        defer allocator.free(rc);
        for (res.routes, 0..) |r, i| rc[i] = r;
        const vc = pdp.validate(inst, rc) orelse return error.Infeasible;
        try std.testing.expectEqual(vc, res.total_cost);

        // every locked prefix must survive verbatim at the head of some route
        // (toResult drops empty routes but never reorders nonempty ones)
        for (cur, locked) |r, lk| {
            if (lk == 0) continue;
            var found = false;
            for (res.routes) |r2| {
                if (r2.len >= lk and std.mem.eql(usize, r2[0..lk], r[0..lk])) found = true;
            }
            try std.testing.expect(found);
        }
    }
    // at least one seed must have produced a nonempty pair-closed prefix
    try std.testing.expect(teeth);
}

test "PDPTW SISR dispatch: zero locks is bit-identical to plain warm solve" {
    const allocator = std.testing.allocator;
    var ri = try pdp.randomInstance(allocator, 10, 21, true);
    defer ri.deinit(allocator);
    const inst = ri.inst();
    var base = try solvePdptwSisr(allocator, inst, .{ .seed = 21, .iters = 2000 });
    defer base.deinit();
    const cur = try allocator.alloc([]const usize, base.routes.len);
    defer allocator.free(cur);
    for (base.routes, 0..) |r, i| cur[i] = r;
    const locked = try allocator.alloc(usize, base.routes.len);
    defer allocator.free(locked);
    @memset(locked, 0);
    var a = try solvePdptwSisrDispatch(allocator, inst, .{ .seed = 9, .iters = 3000 }, cur, locked);
    defer a.deinit();
    var b = try solvePdptwSisrFrom(allocator, inst, .{ .seed = 9, .iters = 3000 }, cur);
    defer b.deinit();
    try std.testing.expectEqual(b.total_cost, a.total_cost);
    try std.testing.expectEqual(b.vehicles, a.vehicles);
}

test "PDPTW SISR time penalty: off is bit-identical to default" {
    const allocator = std.testing.allocator;
    var ri = try pdp.randomInstance(allocator, 10, 11, true);
    defer ri.deinit(allocator);
    const inst = ri.inst();
    var a = try solvePdptwSisr(allocator, inst, .{ .seed = 11, .iters = 3000, .veh_penalty = 10_000_000 });
    defer a.deinit();
    var b = try solvePdptwSisr(allocator, inst, .{ .seed = 11, .iters = 3000, .veh_penalty = 10_000_000, .time_penalty = 0 });
    defer b.deinit();
    try std.testing.expectEqual(a.total_cost, b.total_cost);
    try std.testing.expectEqual(a.vehicles, b.vehicles);
}

test "PDPTW SISR dispatch: new orders enter via the bank" {
    // Rolling-horizon case: the warm start covers all but one pair (a new
    // order); the solve must place it and return a complete solution with
    // every locked prefix intact.
    const allocator = std.testing.allocator;
    var ri = try pdp.randomInstance(allocator, 10, 33, true);
    defer ri.deinit(allocator);
    const inst = ri.inst();
    var base = try solvePdptwSisr(allocator, inst, .{ .seed = 33, .iters = 3000 });
    defer base.deinit();

    // drop the pair of the LAST node of the last route from the warm start
    const last_route = base.routes[base.routes.len - 1];
    const drop_c = last_route[last_route.len - 1];
    const drop_p = if (inst.is_pickup[drop_c]) drop_c else inst.pair_of[drop_c];
    const drop_q = inst.pair_of[drop_p];
    var warm: std.ArrayList([]usize) = .empty;
    defer {
        for (warm.items) |r| allocator.free(r);
        warm.deinit(allocator);
    }
    for (base.routes) |r| {
        var keep: std.ArrayList(usize) = .empty;
        errdefer keep.deinit(allocator);
        for (r) |c| {
            if (c != drop_p and c != drop_q) try keep.append(allocator, c);
        }
        try warm.append(allocator, try keep.toOwnedSlice(allocator));
    }
    const cur = try allocator.alloc([]const usize, warm.items.len);
    defer allocator.free(cur);
    for (warm.items, 0..) |r, i| cur[i] = r;
    const locked = try allocator.alloc(usize, warm.items.len);
    defer allocator.free(locked);
    @memset(locked, 0);
    // lock the first two stops of route 0 when they form a pair-closed prefix
    if (cur[0].len >= 2 and inst.pair_of[cur[0][0]] == cur[0][1]) locked[0] = 2;

    var res = try solvePdptwSisrDispatch(allocator, inst, .{ .seed = 77, .iters = 5000 }, cur, locked);
    defer res.deinit();
    const rc = try allocator.alloc([]const usize, res.routes.len);
    defer allocator.free(rc);
    var served: usize = 0;
    for (res.routes, 0..) |r, i| {
        rc[i] = r;
        served += r.len;
    }
    try std.testing.expectEqual(2 * inst.n_pairs, served); // the new order was placed
    const vc = pdp.validate(inst, rc) orelse return error.Infeasible;
    try std.testing.expectEqual(vc, res.total_cost);
    if (locked[0] > 0) {
        var found = false;
        for (res.routes) |r2| {
            if (r2.len >= locked[0] and std.mem.eql(usize, r2[0..locked[0]], cur[0][0..locked[0]])) found = true;
        }
        try std.testing.expect(found);
    }
}

test "PDPTW SISR eval_threads: off (0 == 1) is the serial path and validates" {
    // eval_threads <= 1 => ew = 0 => the untouched serial recreate. 0 and 1
    // must be identical to each other (and, by construction, to the historic
    // engine, whose default is 0).
    const allocator = std.testing.allocator;
    var seed: u64 = 1;
    while (seed <= 6) : (seed += 1) {
        var ri = try pdp.randomInstanceClocked(allocator, 12, seed);
        defer ri.deinit(allocator);
        const inst = ri.inst();
        var a = try solvePdptwSisr(allocator, inst, .{ .seed = seed, .iters = 3000, .eval_threads = 0 });
        defer a.deinit();
        var b = try solvePdptwSisr(allocator, inst, .{ .seed = seed, .iters = 3000, .eval_threads = 1 });
        defer b.deinit();
        try std.testing.expectEqual(a.total_cost, b.total_cost);
        try std.testing.expectEqual(a.vehicles, b.vehicles);
        const rc = try allocator.alloc([]const usize, a.routes.len);
        defer allocator.free(rc);
        for (a.routes, 0..) |r, i| rc[i] = r;
        const vc = pdp.validate(inst, rc) orelse return error.Infeasible;
        try std.testing.expectEqual(vc, a.total_cost);
    }
}

test "PDPTW SISR eval_threads: parallel reduction is thread-count invariant (2 == 6 == 12)" {
    // The achievable GATE-1 identity: the per-route deterministic blink + the
    // (delta, rank) reduction make the ON trajectory independent of worker
    // count, so 2 == 6 == 12 bit-for-bit on an iteration-bound run. (Money
    // objective on to also exercise the time_penalty branch of the reduction.)
    const allocator = std.testing.allocator;
    var ri = try pdp.randomInstanceClocked(allocator, 20, 9);
    defer ri.deinit(allocator);
    const inst = ri.inst();
    const base = PdpSisrParams{ .seed = 9, .iters = 4000, .veh_penalty = 500, .time_penalty = 3 };

    var r2 = try solvePdptwSisr(allocator, inst, blk: {
        var p = base;
        p.eval_threads = 2;
        break :blk p;
    });
    defer r2.deinit();
    var r6 = try solvePdptwSisr(allocator, inst, blk: {
        var p = base;
        p.eval_threads = 6;
        break :blk p;
    });
    defer r6.deinit();
    var r12 = try solvePdptwSisr(allocator, inst, blk: {
        var p = base;
        p.eval_threads = 12;
        break :blk p;
    });
    defer r12.deinit();

    try std.testing.expectEqual(r2.total_cost, r6.total_cost);
    try std.testing.expectEqual(r2.total_cost, r12.total_cost);
    try std.testing.expectEqual(r2.vehicles, r6.vehicles);
    try std.testing.expectEqual(r2.vehicles, r12.vehicles);

    const rc = try allocator.alloc([]const usize, r2.routes.len);
    defer allocator.free(rc);
    for (r2.routes, 0..) |r, i| rc[i] = r;
    const vc = pdp.validate(inst, rc) orelse return error.Infeasible;
    try std.testing.expectEqual(vc, r2.total_cost);
}

test "PDPTW SISR veh types: per-route load fits its type cap, cheap small vans used" {
    const allocator = std.testing.allocator;
    var ri = try pdp.randomInstance(allocator, 20, 9, true);
    defer ri.deinit(allocator);
    const inst = ri.inst();
    const types = [_]VehType{
        .{ .capacity = @max(1, @divTrunc(inst.capacity, 2)), .fixed_cost = 100, .count = 0 },
        .{ .capacity = inst.capacity, .fixed_cost = 10_000, .count = 0 },
    };
    var a = try solvePdptwSisr(allocator, inst, .{ .seed = 9, .iters = 5000, .veh_types = &types });
    defer a.deinit();
    const tys = a.types orelse return error.TestUnexpectedResult;
    try std.testing.expectEqual(a.routes.len, tys.len);
    const rc = try allocator.alloc([]const usize, a.routes.len);
    defer allocator.free(rc);
    const caps = try allocator.alloc(i64, a.routes.len);
    defer allocator.free(caps);
    for (a.routes, 0..) |r, i| {
        rc[i] = r;
        try std.testing.expect(tys[i] < types.len);
        caps[i] = types[tys[i]].capacity;
    }
    // The independent oracle recomputes everything, per-route caps included.
    const vc = pdp.validateTyped(inst, rc, caps) orelse return error.Infeasible;
    try std.testing.expectEqual(vc, a.total_cost);
    // Determinism: same seed, same answer (incl. the type assignment).
    var b = try solvePdptwSisr(allocator, inst, .{ .seed = 9, .iters = 5000, .veh_types = &types });
    defer b.deinit();
    try std.testing.expectEqual(a.total_cost, b.total_cost);
    try std.testing.expectEqualSlices(usize, tys, b.types.?);
}

test "PDPTW SISR veh types: scarce cheap type count is respected" {
    const allocator = std.testing.allocator;
    var ri = try pdp.randomInstance(allocator, 20, 11, true);
    defer ri.deinit(allocator);
    const inst = ri.inst();
    const types = [_]VehType{
        .{ .capacity = inst.capacity, .fixed_cost = 100, .count = 1 },
        .{ .capacity = inst.capacity, .fixed_cost = 50_000, .count = 0 },
    };
    var a = try solvePdptwSisr(allocator, inst, .{ .seed = 11, .iters = 5000, .veh_types = &types });
    defer a.deinit();
    var cheap: usize = 0;
    for (a.types.?) |t| cheap += @intFromBool(t == 0);
    try std.testing.expect(cheap <= 1);
    const rc = try allocator.alloc([]const usize, a.routes.len);
    defer allocator.free(rc);
    for (a.routes, 0..) |r, i| rc[i] = r;
    try std.testing.expect(pdp.validate(inst, rc) != null);
}

test "PDPTW SISR veh types: no fitting type is infeasible, not a crash" {
    const allocator = std.testing.allocator;
    var ri = try pdp.randomInstance(allocator, 5, 13, true);
    defer ri.deinit(allocator);
    const inst = ri.inst();
    var md: i64 = std.math.maxInt(i64);
    for (1..inst.dim()) |c| {
        if (inst.is_pickup[c]) md = @min(md, inst.demand_signed[c]);
    }
    const types = [_]VehType{.{ .capacity = @max(md - 1, 0), .fixed_cost = 100, .count = 0 }};
    try std.testing.expectError(error.NoCompleteSolution, solvePdptwSisr(allocator, inst, .{ .seed = 13, .iters = 500, .veh_types = &types }));
}

test "PDPTW SISR driver break: plans are break-schedulable per the oracle" {
    const allocator = std.testing.allocator;
    var found_spanning = false;
    var seed: u64 = 31;
    while (seed <= 35) : (seed += 1) {
        var ri = try pdp.randomInstance(allocator, 12, seed, true);
        defer ri.deinit(allocator);
        const inst = ri.inst();
        // Anchor the window to the ACTUAL no-break makespan, not the depot
        // horizon (routes finish far before due[0] on these instances, and a
        // horizon-derived window never bites -> the test proves nothing).
        var probe = try solvePdptwSisr(allocator, inst, .{ .seed = seed, .iters = 2000, .veh_penalty = 10_000_000 });
        var makespan: u64 = 0;
        for (probe.routes) |r| makespan = @max(makespan, routeDuration(inst, r));
        probe.deinit();
        if (makespan < 40) continue;
        const bk = Break{ .dur = @intCast(makespan / 10), .earliest = @intCast(makespan / 4), .latest = @intCast(@min(makespan, inst.due[0] / 2)) };
        var a = solvePdptwSisr(allocator, inst, .{ .seed = seed, .iters = 6000, .veh_penalty = 10_000_000, .brk = bk }) catch |err| switch (err) {
            error.NoCompleteSolution => continue, // tight instance + break: legal outcome
            else => return err,
        };
        defer a.deinit();
        const rc = try allocator.alloc([]const usize, a.routes.len);
        defer allocator.free(rc);
        for (a.routes, 0..) |r, i| rc[i] = r;
        // The independent oracle (brute-force gap search) must agree.
        const vc = pdp.validateWithBreak(inst, rc, bk.dur, bk.earliest, bk.latest) orelse return error.BreakOracleRejected;
        try std.testing.expectEqual(vc, a.total_cost);
        // At least one route across the sweep must actually owe a break
        // (finish after earliest), or this test proves nothing.
        for (a.routes) |r| {
            var t: u64 = 0;
            var prev: usize = 0;
            for (r) |c| {
                t = @max(t + inst.d(prev, c), @as(u64, inst.ready[c])) + inst.service[c];
                prev = c;
            }
            if (t + inst.d(prev, 0) > bk.earliest) found_spanning = true;
        }
    }
    try std.testing.expect(found_spanning);
}

test "PDPTW SISR driver break: with zero waiting the break adds exactly its length" {
    const allocator = std.testing.allocator;
    // tw = false -> wide-open windows, so no route ever waits and break
    // absorption is impossible: every route's with-break completion must be
    // its no-break completion + exactly one break length.
    var ri = try pdp.randomInstance(allocator, 8, 17, false);
    defer ri.deinit(allocator);
    const inst = ri.inst();
    const bk = Break{ .dur = 500, .earliest = 0, .latest = inst.due[0] };
    var with = try solvePdptwSisr(allocator, inst, .{ .seed = 17, .iters = 4000, .veh_penalty = 10_000_000, .time_penalty = 100, .brk = bk });
    defer with.deinit();
    for (with.routes) |r| {
        var t: u64 = 0;
        var prev: usize = 0;
        for (r) |c| {
            t = @max(t + inst.d(prev, c), @as(u64, inst.ready[c])) + inst.service[c];
            prev = c;
        }
        const nobrk = t + inst.d(prev, 0);
        const w = walkWithBreak(inst, r, bk);
        try std.testing.expect(w.ok);
        try std.testing.expectEqual(nobrk + bk.dur, w.dur);
    }
    const rc = try allocator.alloc([]const usize, with.routes.len);
    defer allocator.free(rc);
    for (with.routes, 0..) |r, i| rc[i] = r;
    try std.testing.expect(pdp.validateWithBreak(inst, rc, bk.dur, bk.earliest, bk.latest) != null);
}
