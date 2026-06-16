const std = @import("std");
const distance = @import("distance.zig");
const candidates_mod = @import("candidates.zig");
const DistanceOracle = distance.DistanceOracle;
const Candidates = candidates_mod.Candidates;

// --- Iterative Partial Transcription (tour merging) ------------------------
//
// Mobius, Freisleben, Merz, Schreiber, "Combinatorial Optimization by
// Iterative Partial Transcription", Phys. Rev. E 59(4), 1999 — the mechanism
// behind LKH's MergeWithTour. Two Hamiltonian cycles over the same nodes are
// decomposed into shared portions and pairs of alternative subpaths between
// common endpoints; each independent differing section is resolved to the
// cheaper alternative, so the merged tour can be strictly shorter than both
// parents.

pub const IptScratch = struct {
    allocator: std.mem.Allocator,
    tour_a: []usize,
    tour_b: []usize,
    merged: []usize,
    pos_a: []usize,
    pos_b: []usize,
    rank_a: []usize,
    rank_b: []usize,
    seq_a: []usize,
    seq_b: []usize,
    cum_a: []u64,
    cum_b: []u64,
    essential: []bool,
    boundary: []usize,

    pub fn init(allocator: std.mem.Allocator, n: usize) !IptScratch {
        const tour_a = try allocator.alloc(usize, n);
        errdefer allocator.free(tour_a);
        const tour_b = try allocator.alloc(usize, n);
        errdefer allocator.free(tour_b);
        const merged = try allocator.alloc(usize, n);
        errdefer allocator.free(merged);
        const pos_a = try allocator.alloc(usize, n);
        errdefer allocator.free(pos_a);
        const pos_b = try allocator.alloc(usize, n);
        errdefer allocator.free(pos_b);
        const rank_a = try allocator.alloc(usize, n);
        errdefer allocator.free(rank_a);
        const rank_b = try allocator.alloc(usize, n);
        errdefer allocator.free(rank_b);
        const seq_a = try allocator.alloc(usize, n);
        errdefer allocator.free(seq_a);
        const seq_b = try allocator.alloc(usize, n);
        errdefer allocator.free(seq_b);
        const cum_a = try allocator.alloc(u64, n + 1);
        errdefer allocator.free(cum_a);
        const cum_b = try allocator.alloc(u64, n + 1);
        errdefer allocator.free(cum_b);
        const essential = try allocator.alloc(bool, n);
        errdefer allocator.free(essential);
        const boundary = try allocator.alloc(usize, n);
        errdefer allocator.free(boundary);
        return .{
            .allocator = allocator,
            .tour_a = tour_a,
            .tour_b = tour_b,
            .merged = merged,
            .pos_a = pos_a,
            .pos_b = pos_b,
            .rank_a = rank_a,
            .rank_b = rank_b,
            .seq_a = seq_a,
            .seq_b = seq_b,
            .cum_a = cum_a,
            .cum_b = cum_b,
            .essential = essential,
            .boundary = boundary,
        };
    }

    pub fn deinit(self: *IptScratch) void {
        self.allocator.free(self.tour_a);
        self.allocator.free(self.tour_b);
        self.allocator.free(self.merged);
        self.allocator.free(self.pos_a);
        self.allocator.free(self.pos_b);
        self.allocator.free(self.rank_a);
        self.allocator.free(self.rank_b);
        self.allocator.free(self.seq_a);
        self.allocator.free(self.seq_b);
        self.allocator.free(self.cum_a);
        self.allocator.free(self.cum_b);
        self.allocator.free(self.essential);
        self.allocator.free(self.boundary);
        self.* = undefined;
    }
};

pub const IptOutcome = struct {
    length: u64,
    winner_is_a: bool,
    transcriptions: usize,
    boundary_count: usize,
};

/// Cumulative path cost along `tour` starting at the essential node sitting at
/// `start_pos`: cum[r] = cost of the tour path from essential rank 0 to
/// essential rank r (cum[d] = full tour length). Shared shrunken-out runs are
/// included; they appear identically inside any matched window of both tours,
/// so they cancel in every gain comparison.
fn iptFillCumulative(
    dist: *DistanceOracle,
    tour: []const usize,
    start_pos: usize,
    essential: []const bool,
    cum: []u64,
) void {
    const n = tour.len;
    var acc: u64 = 0;
    var rank: usize = 0;
    for (0..n) |t| {
        const u = tour[(start_pos + t) % n];
        if (essential[u]) {
            cum[rank] = acc;
            rank += 1;
        }
        acc += dist.distance(u, tour[(start_pos + t + 1) % n]);
    }
    cum[rank] = acc;
}

/// Cost of the shrunken-tour path covering k edges starting at rank i.
fn iptPathCost(cum: []const u64, d: usize, i: usize, k: usize) u64 {
    const j = i + k;
    if (j <= d) return cum[j] - cum[i];
    return (cum[d] - cum[i]) + cum[j - d];
}

/// Merge `tour_a` (mutated in place) with `best_tour` (copied into
/// `scratch.tour_b`, then mutated). Returns null when the tours share every
/// edge or no cost-differing matched section exists. On success
/// `scratch.boundary[0..boundary_count]` holds the endpoints of every
/// transcribed section and the shorter of the two merged tours is reported;
/// when `winner_is_a` is false the winning tour lives in `scratch.tour_b`.
pub fn iptMergeTours(
    dist: *DistanceOracle,
    tour_a: []usize,
    len_a_in: u64,
    best_tour: []const usize,
    len_b_in: u64,
    scratch: *IptScratch,
) ?IptOutcome {
    const n = tour_a.len;
    std.debug.assert(best_tour.len == n and scratch.tour_b.len == n);
    const tour_b = scratch.tour_b;
    @memcpy(tour_b, best_tour);
    var len_a = len_a_in;
    var len_b = len_b_in;
    var transcriptions: usize = 0;
    var boundary_count: usize = 0;

    // Shrink once: a node is shared-interior when its undirected neighbor
    // pair agrees in both tours; everything else is an endpoint of a
    // differing edge and survives the shrink. The essential set (and with it
    // the section-size cap d/2) stays FIXED across transcriptions — resolving
    // one section must not tighten the cap for the remaining ones.
    for (tour_a, 0..) |node, i| scratch.pos_a[node] = i;
    for (tour_b, 0..) |node, i| scratch.pos_b[node] = i;
    var d: usize = 0;
    for (0..n) |v| {
        const pa = scratch.pos_a[v];
        const pb = scratch.pos_b[v];
        const a1 = tour_a[(pa + n - 1) % n];
        const a2 = tour_a[(pa + 1) % n];
        const b1 = tour_b[(pb + n - 1) % n];
        const b2 = tour_b[(pb + 1) % n];
        const shared = (a1 == b1 and a2 == b2) or (a1 == b2 and a2 == b1);
        scratch.essential[v] = !shared;
        if (!shared) d += 1;
    }
    // The smallest transcribable section spans 3 shrunken edges and the cap
    // is half the shrunken dimension, so fewer than 6 essential nodes cannot
    // produce a section.
    if (d < 6) return null;

    outer: while (true) {
        for (tour_a, 0..) |node, i| scratch.pos_a[node] = i;
        for (tour_b, 0..) |node, i| scratch.pos_b[node] = i;

        var ia: usize = 0;
        var ib: usize = 0;
        for (tour_a) |v| {
            if (scratch.essential[v]) {
                scratch.seq_a[ia] = v;
                scratch.rank_a[v] = ia;
                ia += 1;
            }
        }
        for (tour_b) |v| {
            if (scratch.essential[v]) {
                scratch.seq_b[ib] = v;
                scratch.rank_b[v] = ib;
                ib += 1;
            }
        }
        std.debug.assert(ia == d and ib == d);

        iptFillCumulative(dist, tour_a, scratch.pos_a[scratch.seq_a[0]], scratch.essential, scratch.cum_a);
        iptFillCumulative(dist, tour_b, scratch.pos_b[scratch.seq_b[0]], scratch.essential, scratch.cum_b);

        // Find the smallest matched differing section: a set of nodes that is
        // a contiguous rank interval in BOTH shrunken tours (pigeonhole: after
        // k steps along B, landing exactly k ranks ahead in A with every
        // intermediate rank distance below k means the k+1 visited nodes are
        // exactly A's ranks si..si+k). Sections larger than d/2 are skipped;
        // their complement is a section too and is found from the other side.
        const max_k = d / 2;
        var best_k: usize = max_k + 1;
        var best_si: usize = 0;
        var best_dir_fwd = false;
        var best_gain: i64 = 0;
        var best_v: usize = 0;
        var found = false;

        scan: for (0..d) |si| {
            const start = scratch.seq_a[si];
            // A section whose first A-edge is shared with B contains a
            // smaller section starting one rank later; skip such starts.
            const a_succ = scratch.seq_a[(si + 1) % d];
            const rb = scratch.rank_b[start];
            if (a_succ == scratch.seq_b[(rb + 1) % d] or a_succ == scratch.seq_b[(rb + d - 1) % d]) continue;

            var dir: usize = 0;
            while (dir < 2) : (dir += 1) {
                const forward = dir == 0;
                var max_sub1: usize = 0;
                var k: usize = 1;
                while (k <= max_k and k < best_k) : (k += 1) {
                    const vrank_b = if (forward) (rb + k) % d else (rb + d - k) % d;
                    const v = scratch.seq_b[vrank_b];
                    const sub1 = (scratch.rank_a[v] + d - scratch.rank_a[start]) % d;
                    if (sub1 >= best_k or sub1 > max_k) break;
                    if (sub1 > max_sub1) {
                        if (sub1 == k) {
                            const cost_a = iptPathCost(scratch.cum_a, d, si, k);
                            const cost_b = if (forward)
                                iptPathCost(scratch.cum_b, d, rb, k)
                            else
                                iptPathCost(scratch.cum_b, d, vrank_b, k);
                            if (cost_a != cost_b) {
                                found = true;
                                best_k = k;
                                best_si = si;
                                best_dir_fwd = forward;
                                best_gain = @as(i64, @intCast(cost_a)) - @as(i64, @intCast(cost_b));
                                best_v = v;
                                if (best_k <= 3) break :scan;
                            }
                            break;
                        }
                        max_sub1 = sub1;
                    }
                }
            }
        }
        if (!found) break :outer;

        // Transcribe the cheaper alternative into the more expensive tour.
        // The full (unshrunken) windows cover identical node sets, so a plain
        // positional copy keeps both tours Hamiltonian.
        const start = scratch.seq_a[best_si];
        const v = best_v;
        const pa_s = scratch.pos_a[start];
        const pa_v = scratch.pos_a[v];
        const span = ((pa_v + n - pa_s) % n) + 1;
        if (best_gain > 0) {
            const pb_s = scratch.pos_b[start];
            if (best_dir_fwd) {
                std.debug.assert(((scratch.pos_b[v] + n - pb_s) % n) + 1 == span);
                for (0..span) |t| tour_a[(pa_s + t) % n] = tour_b[(pb_s + t) % n];
            } else {
                std.debug.assert(((pb_s + n - scratch.pos_b[v]) % n) + 1 == span);
                for (0..span) |t| tour_a[(pa_s + t) % n] = tour_b[(pb_s + n - t) % n];
            }
            len_a -= @intCast(best_gain);
        } else {
            if (best_dir_fwd) {
                const pb_s = scratch.pos_b[start];
                for (0..span) |t| tour_b[(pb_s + t) % n] = tour_a[(pa_s + t) % n];
            } else {
                // B traverses the section v..start in its own forward
                // direction, so write A's window reversed.
                const pb_v = scratch.pos_b[v];
                for (0..span) |t| tour_b[(pb_v + t) % n] = tour_a[(pa_v + n - t) % n];
            }
            len_b -= @intCast(-best_gain);
        }
        transcriptions += 1;
        if (boundary_count + 2 <= scratch.boundary.len) {
            scratch.boundary[boundary_count] = start;
            scratch.boundary[boundary_count + 1] = v;
            boundary_count += 2;
        }
    }

    if (transcriptions == 0) return null;
    return .{
        .length = @min(len_a, len_b),
        .winner_is_a = len_a <= len_b,
        .transcriptions = transcriptions,
        .boundary_count = boundary_count,
    };
}


// --- EAX-lite tour merging (single AB-cycle edge assembly crossover) --------
//
// Nagata & Kobayashi, "Edge Assembly Crossover: A High-power Genetic
// Algorithm for the Traveling Salesman Problem" (ICGA 1997), restricted to
// its single-AB-cycle strategy. The symmetric difference of two Hamiltonian
// cycles over the same nodes decomposes into AB-cycles: closed walks
// alternating A-only and B-only edges (every node carries as many A-only as
// B-only incidences, so a greedy alternating walk can never get stuck and
// closes exactly when a B-edge returns to its start). Applying one cycle to
// a parent removes that parent's edges of the cycle and installs the other
// parent's atomically. A contiguous-section difference is a non-splitting
// AB-cycle — exactly the IPT transcription move this replaced — while
// interleaved differing sections, which IPT could not touch by construction,
// form splitting cycles whose subtours are reconnected with candidate-row
// 2-opt bridges (LKH PatchCycles shape). Cycle deltas are local, so
// equal-length parents (plateau siblings) still expose strictly negative
// cycles — merge material the IPT gain test discarded as zero.

const eax_none = std.math.maxInt(usize);

pub const EaxScratch = struct {
    allocator: std.mem.Allocator,
    tour_a: []usize,
    tour_b: []usize,
    merged: []usize,
    adj_a0: []usize,
    adj_a1: []usize,
    adj_b0: []usize,
    adj_b1: []usize,
    // Unconsumed symmetric-difference half-edges, compacted per node
    // (slot 0 fills before slot 1, eax_none marks empty).
    sd_a0: []usize,
    sd_a1: []usize,
    sd_b0: []usize,
    sd_b1: []usize,
    // AB-cycles as concatenated traversal node lists plus per-cycle metadata;
    // edge i of a cycle runs nodes[i] -> nodes[(i + 1) % len], even i are
    // A-edges. delta = cost(B-edges) - cost(A-edges).
    cycle_nodes: []usize,
    cycle_start: []usize,
    cycle_len: []usize,
    cycle_delta: []i64,
    cycle_order: []usize,
    // Working adjacency for one application attempt + component labeling.
    work0: []usize,
    work1: []usize,
    comp: []usize,
    comp_size: []usize,
    comp_members: []usize,
    boundary: []usize,

    pub fn init(allocator: std.mem.Allocator, n: usize) !EaxScratch {
        var self: EaxScratch = undefined;
        self.allocator = allocator;
        const fields = [_]*[]usize{
            &self.tour_a,      &self.tour_b,    &self.merged,
            &self.adj_a0,      &self.adj_a1,    &self.adj_b0,
            &self.adj_b1,      &self.sd_a0,     &self.sd_a1,
            &self.sd_b0,       &self.sd_b1,     &self.cycle_start,
            &self.cycle_len,   &self.cycle_order, &self.work0,
            &self.work1,       &self.comp,      &self.comp_size,
            &self.comp_members, &self.boundary,
        };
        var allocated: usize = 0;
        errdefer for (fields[0..allocated]) |field| allocator.free(field.*);
        for (fields) |field| {
            field.* = try allocator.alloc(usize, n);
            allocated += 1;
        }
        self.cycle_nodes = try allocator.alloc(usize, 2 * n);
        errdefer allocator.free(self.cycle_nodes);
        self.cycle_delta = try allocator.alloc(i64, n);
        return self;
    }

    pub fn deinit(self: *EaxScratch) void {
        const fields = [_][]usize{
            self.tour_a,      self.tour_b,    self.merged,
            self.adj_a0,      self.adj_a1,    self.adj_b0,
            self.adj_b1,      self.sd_a0,     self.sd_a1,
            self.sd_b0,       self.sd_b1,     self.cycle_start,
            self.cycle_len,   self.cycle_order, self.work0,
            self.work1,       self.comp,      self.comp_size,
            self.comp_members, self.boundary,  self.cycle_nodes,
        };
        for (fields) |field| self.allocator.free(field);
        self.allocator.free(self.cycle_delta);
        self.* = undefined;
    }
};

// --- Elite pool -------------------------------------------------------------
//
// Small population of diverse elite tours used as EAX merge references at
// n >= eax_min_dimension. Research-backed: population-based EAX is the state
// of the art at 10k+ nodes, and the kick-only regime otherwise starves the
// merger for structurally different parents. Replacement policy: exact
// duplicates are dropped (identical edge sets imply identical length, so
// only equal-length members are compared), otherwise the worst member is
// replaced once the pool is full and the offer beats it. Kicks still come
// from the single incumbent — pool-sourced kicks were measured dead in
// round 4 (they dilute intensification).
const elite_pool_capacity = 6;

pub const ElitePool = struct {
    allocator: std.mem.Allocator,
    tours: [elite_pool_capacity][]usize,
    lens: [elite_pool_capacity]u64,
    count: usize,

    pub fn init(allocator: std.mem.Allocator, n: usize) !ElitePool {
        var self: ElitePool = undefined;
        self.allocator = allocator;
        self.count = 0;
        var allocated: usize = 0;
        errdefer for (self.tours[0..allocated]) |t| allocator.free(t);
        for (&self.tours) |*slot| {
            slot.* = try allocator.alloc(usize, n);
            allocated += 1;
        }
        return self;
    }

    pub fn deinit(self: *ElitePool) void {
        for (self.tours) |t| self.allocator.free(t);
        self.* = undefined;
    }
};

/// True when the tours have identical undirected edge sets (rotations and
/// reflections of one another). Uses the scratch adjacency arrays.
fn eaxToursShareAllEdges(scratch: *EaxScratch, a: []const usize, b: []const usize) bool {
    eaxFillAdjacency(a, scratch.adj_a0, scratch.adj_a1);
    eaxFillAdjacency(b, scratch.adj_b0, scratch.adj_b1);
    for (scratch.adj_a0, scratch.adj_a1, scratch.adj_b0, scratch.adj_b1) |a0, a1, b0, b1| {
        if (!((a0 == b0 and a1 == b1) or (a0 == b1 and a1 == b0))) return false;
    }
    return true;
}

pub fn elitePoolOffer(pool: *ElitePool, scratch: *EaxScratch, tour: []const usize, len: u64) void {
    for (0..pool.count) |i| {
        if (pool.lens[i] == len and eaxToursShareAllEdges(scratch, pool.tours[i], tour)) return;
    }
    if (pool.count < elite_pool_capacity) {
        @memcpy(pool.tours[pool.count], tour);
        pool.lens[pool.count] = len;
        pool.count += 1;
        return;
    }
    var worst: usize = 0;
    for (1..elite_pool_capacity) |i| {
        if (pool.lens[i] > pool.lens[worst]) worst = i;
    }
    if (len < pool.lens[worst]) {
        @memcpy(pool.tours[worst], tour);
        pool.lens[worst] = len;
    }
}

const EaxOutcome = struct {
    length: u64,
    winner_is_a: bool,
    cycles_applied: usize,
    boundary_count: usize,
    // A-only half-edge count of the initial symmetric difference; 0 means the
    // trial and the reference share every edge — the trial generator
    // re-converged into the incumbent and produced no new tour material.
    // This is the solver's convergence (diversity-exhaustion) signal.
    initial_symdiff: usize,
};

fn eaxFillAdjacency(tour: []const usize, nbr0: []usize, nbr1: []usize) void {
    const n = tour.len;
    for (tour, 0..) |node, i| {
        nbr0[node] = tour[(i + n - 1) % n];
        nbr1[node] = tour[(i + 1) % n];
    }
}

fn eaxSlotAdd(s0: []usize, s1: []usize, node: usize, value: usize) void {
    if (s0[node] == eax_none) {
        s0[node] = value;
    } else {
        std.debug.assert(s1[node] == eax_none);
        s1[node] = value;
    }
}

fn eaxSlotRemove(s0: []usize, s1: []usize, node: usize, value: usize) void {
    if (s0[node] == value) {
        s0[node] = s1[node];
        s1[node] = eax_none;
    } else {
        std.debug.assert(s1[node] == value);
        s1[node] = eax_none;
    }
}

/// Fill the symmetric-difference half-edge slots from the parents' adjacency.
/// Returns the number of A-only directed half-edges (== B-only count; 0 means
/// the tours share every edge).
fn eaxFillSymdiff(scratch: *EaxScratch, n: usize) usize {
    @memset(scratch.sd_a0[0..n], eax_none);
    @memset(scratch.sd_a1[0..n], eax_none);
    @memset(scratch.sd_b0[0..n], eax_none);
    @memset(scratch.sd_b1[0..n], eax_none);
    var count: usize = 0;
    for (0..n) |v| {
        for ([2]usize{ scratch.adj_a0[v], scratch.adj_a1[v] }) |u| {
            if (u != scratch.adj_b0[v] and u != scratch.adj_b1[v]) {
                eaxSlotAdd(scratch.sd_a0, scratch.sd_a1, v, u);
                count += 1;
            }
        }
        for ([2]usize{ scratch.adj_b0[v], scratch.adj_b1[v] }) |u| {
            if (u != scratch.adj_a0[v] and u != scratch.adj_a1[v]) {
                eaxSlotAdd(scratch.sd_b0, scratch.sd_b1, v, u);
            }
        }
    }
    return count;
}

/// Decompose the symmetric difference into AB-cycles by greedy alternating
/// walks, consuming the half-edge slots. Deterministic (always slot 0 first).
fn eaxExtractCycles(dist: *DistanceOracle, scratch: *EaxScratch, n: usize) usize {
    var cycle_count: usize = 0;
    var buf_used: usize = 0;
    for (0..n) |v| {
        while (scratch.sd_a0[v] != eax_none) {
            const start = buf_used;
            var delta: i64 = 0;
            var cur = v;
            while (true) {
                const au = scratch.sd_a0[cur];
                eaxSlotRemove(scratch.sd_a0, scratch.sd_a1, cur, au);
                eaxSlotRemove(scratch.sd_a0, scratch.sd_a1, au, cur);
                scratch.cycle_nodes[buf_used] = cur;
                buf_used += 1;
                delta -= @as(i64, dist.distance(cur, au));
                cur = au;
                const bu = scratch.sd_b0[cur];
                eaxSlotRemove(scratch.sd_b0, scratch.sd_b1, cur, bu);
                eaxSlotRemove(scratch.sd_b0, scratch.sd_b1, bu, cur);
                scratch.cycle_nodes[buf_used] = cur;
                buf_used += 1;
                delta += @as(i64, dist.distance(cur, bu));
                cur = bu;
                if (cur == v) break;
            }
            scratch.cycle_start[cycle_count] = start;
            scratch.cycle_len[cycle_count] = buf_used - start;
            scratch.cycle_delta[cycle_count] = delta;
            cycle_count += 1;
        }
    }
    return cycle_count;
}

/// Reconnect the subtours left by a splitting cycle application into one
/// Hamiltonian cycle: repeatedly merge the smallest live component into
/// another via the cheapest candidate-row 2-opt bridge (LKH PatchCycles
/// shape). Returns the summed bridge delta, or null when some component has
/// no candidate edge leaving it. Bridge endpoints are appended to `boundary`.
fn eaxRepairComponents(
    dist: *DistanceOracle,
    candidates: *const Candidates,
    scratch: *EaxScratch,
    n: usize,
    comp_count: usize,
    boundary_count: *usize,
) ?i64 {
    var remaining = comp_count;
    var total: i64 = 0;
    while (remaining > 1) {
        var small: usize = eax_none;
        for (0..comp_count) |cid| {
            if (scratch.comp_size[cid] == 0) continue;
            if (small == eax_none or scratch.comp_size[cid] < scratch.comp_size[small]) small = cid;
        }
        var member_count: usize = 0;
        for (0..n) |node| {
            if (scratch.comp[node] == small) {
                scratch.comp_members[member_count] = node;
                member_count += 1;
            }
        }
        var best_delta: i64 = std.math.maxInt(i64);
        var best_a: usize = 0;
        var best_a2: usize = 0;
        var best_c: usize = 0;
        var best_c2: usize = 0;
        for (scratch.comp_members[0..member_count]) |a| {
            const a_nbrs = [2]usize{ scratch.work0[a], scratch.work1[a] };
            for (candidates.row(a)) |c| {
                if (scratch.comp[c] == small) continue;
                const c_nbrs = [2]usize{ scratch.work0[c], scratch.work1[c] };
                const d_ac = @as(i64, dist.distance(a, c));
                for (a_nbrs) |a2| {
                    for (c_nbrs) |c2| {
                        const delta = d_ac +
                            @as(i64, dist.distance(a2, c2)) -
                            @as(i64, dist.distance(a, a2)) -
                            @as(i64, dist.distance(c, c2));
                        if (delta < best_delta) {
                            best_delta = delta;
                            best_a = a;
                            best_a2 = a2;
                            best_c = c;
                            best_c2 = c2;
                        }
                    }
                }
            }
        }
        if (best_delta == std.math.maxInt(i64)) return null;
        eaxSlotRemove(scratch.work0, scratch.work1, best_a, best_a2);
        eaxSlotRemove(scratch.work0, scratch.work1, best_a2, best_a);
        eaxSlotRemove(scratch.work0, scratch.work1, best_c, best_c2);
        eaxSlotRemove(scratch.work0, scratch.work1, best_c2, best_c);
        eaxSlotAdd(scratch.work0, scratch.work1, best_a, best_c);
        eaxSlotAdd(scratch.work0, scratch.work1, best_c, best_a);
        eaxSlotAdd(scratch.work0, scratch.work1, best_a2, best_c2);
        eaxSlotAdd(scratch.work0, scratch.work1, best_c2, best_a2);
        const target = scratch.comp[best_c];
        for (scratch.comp_members[0..member_count]) |node| scratch.comp[node] = target;
        scratch.comp_size[target] += member_count;
        scratch.comp_size[small] = 0;
        remaining -= 1;
        total += best_delta;
        for ([4]usize{ best_a, best_a2, best_c, best_c2 }) |node| {
            if (boundary_count.* >= scratch.boundary.len) break;
            scratch.boundary[boundary_count.*] = node;
            boundary_count.* += 1;
        }
    }
    return total;
}

fn eaxMaterialize(work0: []const usize, work1: []const usize, tour: []usize) void {
    var prev: usize = eax_none;
    var cur: usize = 0;
    for (tour) |*slot| {
        slot.* = cur;
        const nxt = if (work0[cur] != prev) work0[cur] else work1[cur];
        prev = cur;
        cur = nxt;
    }
    std.debug.assert(cur == 0);
}

/// Apply one AB-cycle to `target_tour` (the A parent when `to_a`): remove the
/// target's cycle edges, install the other parent's, repair any subtour split,
/// and commit only on a strict length improvement. Returns the new length on
/// acceptance; the target tour and `boundary` are untouched on rejection.
fn eaxTryApplyCycle(
    dist: *DistanceOracle,
    candidates: *const Candidates,
    scratch: *EaxScratch,
    cycle: usize,
    to_a: bool,
    target_tour: []usize,
    len_target: u64,
    allow_split: bool,
    boundary_count: *usize,
) ?u64 {
    const n = target_tour.len;
    eaxFillAdjacency(target_tour, scratch.work0, scratch.work1);
    const nodes = scratch.cycle_nodes[scratch.cycle_start[cycle]..][0..scratch.cycle_len[cycle]];
    // Removals before additions: per cycle node the removed and added
    // incidence counts match, so the adjacency never exceeds two slots.
    for (nodes, 0..) |u, i| {
        if ((i % 2 == 0) == to_a) {
            const w = nodes[(i + 1) % nodes.len];
            eaxSlotRemove(scratch.work0, scratch.work1, u, w);
            eaxSlotRemove(scratch.work0, scratch.work1, w, u);
        }
    }
    for (nodes, 0..) |u, i| {
        if ((i % 2 == 0) != to_a) {
            const w = nodes[(i + 1) % nodes.len];
            eaxSlotAdd(scratch.work0, scratch.work1, u, w);
            eaxSlotAdd(scratch.work0, scratch.work1, w, u);
        }
    }

    var comp_count: usize = 0;
    @memset(scratch.comp[0..n], eax_none);
    for (0..n) |s| {
        if (scratch.comp[s] != eax_none) continue;
        var size: usize = 0;
        var prev: usize = eax_none;
        var cur = s;
        while (true) {
            scratch.comp[cur] = comp_count;
            size += 1;
            const nxt = if (scratch.work0[cur] != prev) scratch.work0[cur] else scratch.work1[cur];
            prev = cur;
            cur = nxt;
            if (cur == s) break;
        }
        scratch.comp_size[comp_count] = size;
        comp_count += 1;
    }

    var total_delta: i64 = if (to_a) scratch.cycle_delta[cycle] else -scratch.cycle_delta[cycle];
    const boundary_before = boundary_count.*;
    if (comp_count > 1) {
        if (!allow_split) return null;
        total_delta += eaxRepairComponents(dist, candidates, scratch, n, comp_count, boundary_count) orelse {
            boundary_count.* = boundary_before;
            return null;
        };
    }
    const new_len_signed = @as(i64, @intCast(len_target)) + total_delta;
    if (new_len_signed < 0 or @as(u64, @intCast(new_len_signed)) >= len_target) {
        boundary_count.* = boundary_before;
        return null;
    }
    for (nodes) |node| {
        if (boundary_count.* >= scratch.boundary.len) break;
        scratch.boundary[boundary_count.*] = node;
        boundary_count.* += 1;
    }
    eaxMaterialize(scratch.work0, scratch.work1, target_tour);
    return @intCast(new_len_signed);
}

/// Per round, only the cheapest few improving cycles are attempted: a
/// non-splitting improving cycle always commits, so the cap can only skip
/// splitting cycles whose repair already ate the gain for cheaper siblings.
const eax_max_attempts_per_round = 8;

/// Merge `tour_a` (mutated in place) with `best_tour` (copied into
/// `scratch.tour_b`, then mutated): repeatedly apply the AB-cycle application
/// with the best estimated outcome until none improves. Always returns a
/// report; `cycles_applied` == 0 means no application committed (and
/// `initial_symdiff` == 0 additionally means the tours share every edge). On
/// success `scratch.boundary[0..boundary_count]` holds the endpoints of every
/// changed edge and the shorter of the two merged tours is reported; when
/// `winner_is_a` is false the winning tour lives in `scratch.tour_b`.
pub fn eaxMergeTours(
    dist: *DistanceOracle,
    candidates: *const Candidates,
    tour_a: []usize,
    len_a_in: u64,
    best_tour: []const usize,
    len_b_in: u64,
    allow_split: bool,
    scratch: *EaxScratch,
) EaxOutcome {
    const n = tour_a.len;
    std.debug.assert(best_tour.len == n and scratch.tour_b.len == n);
    @memcpy(scratch.tour_b, best_tour);
    var len_a = len_a_in;
    var len_b = len_b_in;
    var cycles_applied: usize = 0;
    var boundary_count: usize = 0;
    var initial_symdiff: usize = 0;
    var first_round = true;

    // Each committed application strictly shrinks len_a + len_b, so the loop
    // terminates without an iteration cap.
    outer: while (true) {
        eaxFillAdjacency(tour_a, scratch.adj_a0, scratch.adj_a1);
        eaxFillAdjacency(scratch.tour_b, scratch.adj_b0, scratch.adj_b1);
        const symdiff = eaxFillSymdiff(scratch, n);
        if (first_round) {
            initial_symdiff = symdiff;
            first_round = false;
        }
        if (symdiff == 0) break;
        const cycle_count = eaxExtractCycles(dist, scratch, n);

        // A negative-delta cycle improves A, a positive one improves B;
        // order the improving applications by estimated resulting length.
        // (Smallest-cycle-first, IPT's order, was measured: it recovers d657
        // but loses lin318 seeds and worsens pr1002 — another reshuffle, not
        // a win; the gate below keeps IPT itself where IPT is better.)
        var order_count: usize = 0;
        for (0..cycle_count) |c| {
            const delta = scratch.cycle_delta[c];
            if (delta == 0) continue;
            const est = if (delta < 0)
                len_a - @as(u64, @intCast(-delta))
            else
                len_b - @as(u64, @intCast(delta));
            var slot = order_count;
            while (slot > 0) : (slot -= 1) {
                const other = scratch.cycle_order[slot - 1];
                const odelta = scratch.cycle_delta[other];
                const oest = if (odelta < 0)
                    len_a - @as(u64, @intCast(-odelta))
                else
                    len_b - @as(u64, @intCast(odelta));
                if (oest <= est) break;
                scratch.cycle_order[slot] = other;
            }
            scratch.cycle_order[slot] = c;
            order_count += 1;
        }

        for (scratch.cycle_order[0..@min(order_count, eax_max_attempts_per_round)]) |c| {
            const to_a = scratch.cycle_delta[c] < 0;
            const target_tour = if (to_a) tour_a else scratch.tour_b;
            const len_target = if (to_a) len_a else len_b;
            if (eaxTryApplyCycle(dist, candidates, scratch, c, to_a, target_tour, len_target, allow_split, &boundary_count)) |new_len| {
                if (to_a) len_a = new_len else len_b = new_len;
                cycles_applied += 1;
                continue :outer;
            }
        }
        break :outer;
    }

    return .{
        .length = @min(len_a, len_b),
        .winner_is_a = len_a <= len_b,
        .cycles_applied = cycles_applied,
        .boundary_count = boundary_count,
        .initial_symdiff = initial_symdiff,
    };
}

