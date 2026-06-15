const std = @import("std");
const distance = @import("distance.zig");
const candidates_mod = @import("candidates.zig");
const tour_mod = @import("tour.zig");
const solver = @import("solver.zig");

const DistanceOracle = distance.DistanceOracle;
const Candidates = candidates_mod.Candidates;
const TourEdge = tour_mod.TourEdge;
const MovePlan = tour_mod.MovePlan;
const TourView = tour_mod.TourView;
const useSegmentTour = tour_mod.useSegmentTour;
const segmentTargetSize = tour_mod.segmentTargetSize;
const tourEdgeInSlice = tour_mod.tourEdgeInSlice;
const removeTourEdgeFromSlice = tour_mod.removeTourEdgeFromSlice;
const sameUndirectedEdge = tour_mod.sameUndirectedEdge;
const SolveStats = solver.SolveStats;
const SolverWorkspace = solver.SolverWorkspace;

/// Caller-owned hard constraint: edges the local search must never break. The
/// legitimate, non-heuristic successor to the deleted edge-freeze voting
/// subsystem — the caller owns the constraint outright, no votes, no staleness.
/// Type-erased for the future VRP layer. Dormant: `LocalSearch.pinned_edges`
/// defaults null and no move consults it yet; the move-apply integration lands
/// with the VRP work, at which point edge-removal sites gate on
/// `if (pinned_edges) |pe| if (pe.isPinned(a, b)) continue;`.
pub const PinnedEdges = struct {
    ctx: *const anyopaque,
    isPinnedFn: *const fn (ctx: *const anyopaque, a: usize, b: usize) bool,

    pub fn isPinned(self: *const PinnedEdges, a: usize, b: usize) bool {
        return self.isPinnedFn(self.ctx, a, b);
    }
};

pub const LocalSearch = struct {
    dist: *DistanceOracle,
    // Trial-loop cost counters (roadmap item 1) accumulate here directly:
    // tour mutations (reverseSegment / applyEdges rebuilds) live in this
    // struct, not in the ephemeral TourView, so they vote into stats in place.
    stats: *SolveStats,
    candidates: *const Candidates,
    // The tour-order representation. Kept as its own field (NOT in ws) because
    // the IPT/EAX merge path shallow-copies the LocalSearch and repoints just
    // this slice at a shadow tour, sharing every other buffer — see the round-5
    // aliasing note. pos/next/prev are a cache over this tour and stay in ws
    // (the merge rebuilds them), so they are shared exactly as before.
    tour: []usize,
    // All other per-trial scratch buffers live in the SolverWorkspace, reached
    // through this pointer (H3): one owner, no shallow-copied slice headers to
    // keep in sync between the workspace and the search. The hot fields
    // (ws.pos/next/prev, ws.removed_*/added_*/lk_t) are one extra register hop
    // the optimizer hoists, so this is bit-identical and perf-neutral.
    ws: *SolverWorkspace,
    max_passes: usize,
    enable_or_opt: bool,
    enable_bounded_three_opt_cleanup: bool,
    lk_completion_patch_min_gain: i64,
    max_lk_depth: usize,
    lk_backtrack_limit: usize,
    lk_backtrack_depth: usize,
    lk_nonseq_branch_limit: usize,
    // Dormant pinned-edge seam (item 8): caller-owned edges the search must not
    // break. Null in the whole current solve path, so all moves are unchanged.
    pinned_edges: ?*const PinnedEdges = null,
    lk_nodes_this_pass: usize = 0,
    lk_active_head: usize = 0,
    lk_active_count: usize = 0,
    // Roadmap item 2 (incremental bookkeeping): the tour length, maintained
    // delta-style so the trial loop reads it in O(1) instead of rescanning the
    // whole tour (the per-trial full-array scans the item-1 counters measured).
    // syncLength() reseeds it from one scan after each construction/kick; every
    // move applier folds in its exact edge delta. Debug builds assert it against
    // a fresh scan at the end of each trial, so any drift fails the test suite.
    current_length: u64 = 0,

    // Active-node queue ("don't-look bits", Helsgaun Sec. 3/LKH StoreTour):
    // only nodes whose neighborhood changed since their last failed search are
    // re-examined; an improving move reactivates every endpoint it touched.
    pub fn lkActivateAll(self: *LocalSearch) void {
        const n = self.tour.len;
        @memset(self.ws.lk_active, true);
        @memcpy(self.ws.lk_active_queue, self.tour);
        self.lk_active_head = 0;
        self.lk_active_count = n;
    }

    pub fn lkResetActive(self: *LocalSearch) void {
        @memset(self.ws.lk_active, false);
        self.lk_active_head = 0;
        self.lk_active_count = 0;
    }

    pub fn lkActivate(self: *LocalSearch, node: usize) void {
        if (self.ws.lk_active[node]) return;
        self.ws.lk_active[node] = true;
        const slot = (self.lk_active_head + self.lk_active_count) % self.ws.lk_active_queue.len;
        self.ws.lk_active_queue[slot] = node;
        self.lk_active_count += 1;
    }

    pub fn lkPopActive(self: *LocalSearch) ?usize {
        if (self.lk_active_count == 0) return null;
        const node = self.ws.lk_active_queue[self.lk_active_head];
        self.lk_active_head = (self.lk_active_head + 1) % self.ws.lk_active_queue.len;
        self.lk_active_count -= 1;
        self.ws.lk_active[node] = false;
        return node;
    }

    pub fn lkActivateMoveEndpoints(self: *LocalSearch, removed_count: usize, added_count: usize) void {
        for (0..removed_count) |i| {
            self.lkActivate(self.ws.removed_a[i]);
            self.lkActivate(self.ws.removed_b[i]);
        }
        for (0..added_count) |i| {
            self.lkActivate(self.ws.added_a[i]);
            self.lkActivate(self.ws.added_b[i]);
        }
    }

    pub fn improveWarmup(self: *LocalSearch) !u64 {
        var moves: u64 = 0;
        for (0..self.max_passes) |_| {
            if (try self.improve2Opt()) {
                moves += 1;
                continue;
            }
            if (self.enable_or_opt and try self.improveOrOpt1()) {
                moves += 1;
                continue;
            }
            break;
        }
        return moves;
    }

    // full=false is the cheap kicked-trial descent: sequential LK from the
    // seeded queue only. The fallback sweeps (Gain23 bridge, 4-opt, bounded
    // 3-opt cleanup) are all O(n)-per-call full scans; on a kicked trial they
    // would re-scan a tour that differs from the already-polished best tour in
    // only a handful of edges, so they are reserved for tours that actually
    // improved on the best (the caller re-runs with full=true to polish).
    pub fn improveLK(self: *LocalSearch, stats: *SolveStats, activate_all: bool, full: bool) !u64 {
        var moves: u64 = 0;
        if (activate_all) self.lkActivateAll();
        for (0..self.max_passes) |_| {
            self.lk_nodes_this_pass = 0;
            const sweep_moves = self.findLKMove(stats);
            if (sweep_moves > 0) {
                moves += sweep_moves;
                stats.lk_moves += sweep_moves;
                continue;
            }
            if (!full) break;
            // Nonsequential fallbacks run only once the sequential search has
            // drained the active queue (paper: Gain23 after sequential moves fail).
            if (self.tour.len >= 256 and self.tour.len < 512 and self.improveGain23Bridge(stats)) {
                moves += 1;
                stats.lk_moves += 1;
                continue;
            }
            if (self.enable_bounded_three_opt_cleanup and self.improveBoundedThreeOptCleanup(stats)) {
                moves += 1;
                stats.bounded_three_opt_cleanup_moves += 1;
                continue;
            }
            break;
        }
        if (full and self.enable_bounded_three_opt_cleanup) {
            const bounded_cleanup_passes = @max(self.max_passes / 4, 1);
            for (0..bounded_cleanup_passes) |_| {
                if (!self.improveBoundedThreeOptCleanup(stats)) break;
                moves += 1;
                stats.bounded_three_opt_cleanup_moves += 1;
            }
        }
        return moves;
    }

    pub fn improveGain23Bridge(self: *LocalSearch, stats: *SolveStats) bool {
        if (self.lk_nonseq_branch_limit == 0 or self.tour.len < 8) return false;

        const n = self.tour.len;
        for (0..n) |i| {
            const s1 = self.tour[i];
            const s2 = self.tourNext(s1);
            const removed_first: i64 = @intCast(self.dist.distance(s1, s2));
            var breadth: usize = 0;

            for (self.candidates.row(s2), 0..) |s3, ci| {
                if (breadth >= self.lk_nonseq_branch_limit) break;
                if (s3 == s1 or s3 == s2 or self.isTourEdge(s2, s3)) continue;
                const s4 = self.tourNext(s3);
                if (s4 == s1 or s4 == s2) continue;
                if (self.tourSeq(s3) <= self.tourSeq(s2)) continue;
                if (!self.segmentIsNoMoreThanHalf(s2, s3)) continue;

                const gain =
                    removed_first -
                    @as(i64, @intCast(self.candidates.candDist(s2, ci))) +
                    @as(i64, @intCast(self.dist.distance(s3, s4))) -
                    @as(i64, @intCast(self.dist.distance(s4, s1)));
                if (gain <= 0) continue;

                if (!self.recordLKNode(stats)) return false;
                stats.lk_completion_attempts += 1;
                stats.lk_nonseq_attempts += 1;
                breadth += 1;

                self.ws.removed_a[0] = s1;
                self.ws.removed_b[0] = s2;
                self.ws.removed_a[1] = s3;
                self.ws.removed_b[1] = s4;
                self.ws.added_a[0] = s2;
                self.ws.added_b[0] = s3;
                self.ws.added_a[1] = s4;
                self.ws.added_b[1] = s1;

                if (self.testAndApplyGain23BridgeMove(2, stats)) {
                    stats.lk_completion_accepted += 1;
                    stats.lk_nonseq_accepted += 1;
                    stats.lk_nonseq_depth_total += 4;
                    stats.lk_nonseq_deepest_accepted_depth = @max(stats.lk_nonseq_deepest_accepted_depth, 4);
                    return true;
                }
                stats.lk_completion_rejected += 1;
                stats.lk_nonseq_rejected += 1;

                if (self.tryGain23ThreeEdgeBridge(s1, s2, s3, s4, gain, stats)) return true;
                if (self.tryGain23BridgeGain2Opt(s1, s2, s3, s4, gain, stats)) return true;
            }
        }
        return false;
    }

    pub fn tryGain23BridgeGain2Opt(
        self: *LocalSearch,
        s1: usize,
        s2: usize,
        s3: usize,
        s4: usize,
        base_gain: i64,
        stats: *SolveStats,
    ) bool {
        const segment = self.smallerSegmentEndpoints(s2, s3, s4, s1);
        var t1 = segment.from;
        var scanned: usize = 0;
        while (t1 != segment.to and scanned < self.tour.len) : (scanned += 1) {
            const t2 = self.tourNext(t1);
            defer t1 = t2;
            if (self.isExcludedGain23BaseEdge(t1, t2, s1, s2, s3, s4, std.math.maxInt(usize), std.math.maxInt(usize))) continue;

            const gain0 = base_gain + @as(i64, @intCast(self.dist.distance(t1, t2)));
            var breadth2: usize = 0;
            for (self.candidates.row(t2), 0..) |t3, ci| {
                if (breadth2 >= self.lk_nonseq_branch_limit) break;
                if (t3 == t1 or t3 == t2 or self.isTourEdge(t2, t3)) continue;
                if (self.nodeInCircularSegment(segment.from, t3, segment.to)) continue;
                const gain1 = gain0 - @as(i64, @intCast(self.candidates.candDist(t2, ci)));
                if (gain1 <= 0) continue;
                breadth2 += 1;

                var choices = [2]usize{ self.tourNext(t3), self.tourPrev(t3) };
                self.orderTourEdgeChoices(t3, &choices);
                for (choices) |t4| {
                    if (t4 == t1 or t4 == t2) continue;
                    if (self.isExcludedGain23BaseEdge(t3, t4, s1, s2, s3, s4, t1, t2)) continue;
                    if (sameUndirectedEdge(t4, t1, s2, s3) or sameUndirectedEdge(t4, t1, s4, s1)) continue;

                    const gain =
                        gain1 +
                        @as(i64, @intCast(self.dist.distance(t3, t4))) -
                        @as(i64, @intCast(self.dist.distance(t4, t1)));
                    if (gain <= 0) continue;

                    if (!self.recordLKNode(stats)) return false;
                    stats.lk_completion_attempts += 1;
                    stats.lk_nonseq_attempts += 1;

                    self.ws.removed_a[0] = s1;
                    self.ws.removed_b[0] = s2;
                    self.ws.removed_a[1] = s3;
                    self.ws.removed_b[1] = s4;
                    self.ws.removed_a[2] = t1;
                    self.ws.removed_b[2] = t2;
                    self.ws.removed_a[3] = t3;
                    self.ws.removed_b[3] = t4;
                    self.ws.added_a[0] = s2;
                    self.ws.added_b[0] = s3;
                    self.ws.added_a[1] = s4;
                    self.ws.added_b[1] = s1;
                    self.ws.added_a[2] = t2;
                    self.ws.added_b[2] = t3;
                    self.ws.added_a[3] = t4;
                    self.ws.added_b[3] = t1;

                    if (self.testAndApplyGain23BridgeMove(4, stats)) {
                        stats.lk_completion_accepted += 1;
                        stats.lk_nonseq_accepted += 1;
                        stats.lk_nonseq_depth_total += 6;
                        stats.lk_nonseq_deepest_accepted_depth = @max(stats.lk_nonseq_deepest_accepted_depth, 6);
                        return true;
                    }
                    stats.lk_completion_rejected += 1;
                    stats.lk_nonseq_rejected += 1;
                }
            }
        }
        return false;
    }

    const SegmentEndpoints = struct {
        from: usize,
        to: usize,
    };

    pub fn smallerSegmentEndpoints(self: *const LocalSearch, a: usize, b: usize, c: usize, d: usize) SegmentEndpoints {
        const ab = self.circularSegmentSize(a, b);
        const cd = self.circularSegmentSize(c, d);
        return if (ab <= cd) .{ .from = a, .to = b } else .{ .from = c, .to = d };
    }

    pub fn circularSegmentSize(self: *const LocalSearch, from: usize, to: usize) usize {
        const n = self.tour.len;
        const from_pos = self.tourSeq(from);
        const to_pos = self.tourSeq(to);
        return if (to_pos >= from_pos) to_pos - from_pos + 1 else n - from_pos + to_pos + 1;
    }

    pub fn nodeInCircularSegment(self: *const LocalSearch, from: usize, node: usize, to: usize) bool {
        const from_pos = self.tourSeq(from);
        const node_pos = self.tourSeq(node);
        const to_pos = self.tourSeq(to);
        if (from_pos <= to_pos) return from_pos <= node_pos and node_pos <= to_pos;
        return node_pos >= from_pos or node_pos <= to_pos;
    }

    pub fn isExcludedGain23BaseEdge(
        self: *const LocalSearch,
        a: usize,
        b: usize,
        s1: usize,
        s2: usize,
        s3: usize,
        s4: usize,
        t1: usize,
        t2: usize,
    ) bool {
        _ = self;
        return sameUndirectedEdge(a, b, s1, s2) or
            sameUndirectedEdge(a, b, s3, s4) or
            sameUndirectedEdge(a, b, t1, t2);
    }

    pub fn tryGain23ThreeEdgeBridge(
        self: *LocalSearch,
        s1: usize,
        s2: usize,
        s3: usize,
        s4: usize,
        base_gain: i64,
        stats: *SolveStats,
    ) bool {
        var breadth4: usize = 0;
        for (self.candidates.row(s4), 0..) |s5, ci| {
            if (breadth4 >= self.lk_nonseq_branch_limit) break;
            if (s5 == s1 or s5 == s2 or s5 == s3 or s5 == s4) continue;
            if (self.isTourEdge(s4, s5)) continue;

            const after_second_add = base_gain - @as(i64, @intCast(self.candidates.candDist(s4, ci)));
            if (after_second_add <= 0) continue;
            breadth4 += 1;

            var choices = [2]usize{ self.tourNext(s5), self.tourPrev(s5) };
            self.orderTourEdgeChoices(s5, &choices);
            for (choices) |s6| {
                if (s6 == s1 or s6 == s2 or s6 == s3 or s6 == s4) continue;
                if (sameUndirectedEdge(s5, s6, s1, s2) or sameUndirectedEdge(s5, s6, s3, s4)) continue;

                const gain =
                    after_second_add +
                    @as(i64, @intCast(self.dist.distance(s5, s6))) -
                    @as(i64, @intCast(self.dist.distance(s6, s1)));
                if (gain <= 0) continue;

                if (!self.recordLKNode(stats)) return false;
                stats.lk_completion_attempts += 1;
                stats.lk_nonseq_attempts += 1;

                self.ws.removed_a[0] = s1;
                self.ws.removed_b[0] = s2;
                self.ws.removed_a[1] = s3;
                self.ws.removed_b[1] = s4;
                self.ws.removed_a[2] = s5;
                self.ws.removed_b[2] = s6;
                self.ws.added_a[0] = s2;
                self.ws.added_b[0] = s3;
                self.ws.added_a[1] = s4;
                self.ws.added_b[1] = s5;
                self.ws.added_a[2] = s6;
                self.ws.added_b[2] = s1;

                if (self.testAndApplyGain23BridgeMove(3, stats)) {
                    stats.lk_completion_accepted += 1;
                    stats.lk_nonseq_accepted += 1;
                    stats.lk_nonseq_depth_total += 5;
                    stats.lk_nonseq_deepest_accepted_depth = @max(stats.lk_nonseq_deepest_accepted_depth, 5);
                    return true;
                }
                stats.lk_completion_rejected += 1;
                stats.lk_nonseq_rejected += 1;
            }
        }
        return false;
    }

    pub fn segmentIsNoMoreThanHalf(self: *const LocalSearch, from: usize, to: usize) bool {
        const n = self.tour.len;
        const from_pos = self.tourSeq(from);
        const to_pos = self.tourSeq(to);
        const span = if (to_pos >= from_pos) to_pos - from_pos + 1 else n - from_pos + to_pos + 1;
        return 2 * span <= n;
    }

    pub fn findLKMove(self: *LocalSearch, stats: *SolveStats) u64 {
        var moves: u64 = 0;
        while (self.lkPopActive()) |t1| {
            var choices = [2]usize{ self.tourNext(t1), self.tourPrev(t1) };
            self.orderTourEdgeChoices(t1, &choices);

            for (choices) |t2| {
                if (self.lk_nodes_this_pass >= self.lk_backtrack_limit) {
                    // Budget slice exhausted: keep t1 queued so the next pass
                    // resumes the descent instead of dropping it.
                    self.lkActivate(t1);
                    return moves;
                }
                stats.lk_attempts += 1;
                self.ws.lk_t[0] = t1;
                self.ws.lk_t[1] = t2;
                self.ws.removed_a[0] = t1;
                self.ws.removed_b[0] = t2;
                const gain: i64 = @intCast(self.dist.distance(t1, t2));
                if (self.searchAdded(1, t2, gain, stats)) {
                    moves += 1;
                    self.lkActivate(t1);
                    break;
                }
            }
        }
        return moves;
    }

    pub fn orderTourEdgeChoices(self: *LocalSearch, base: usize, choices: *[2]usize) void {
        const d0 = self.dist.distance(base, choices[0]);
        const d1 = self.dist.distance(base, choices[1]);
        if (d1 > d0 or (d1 == d0 and choices[1] < choices[0])) {
            std.mem.swap(usize, &choices[0], &choices[1]);
        }
    }

    pub fn recordLKNode(self: *LocalSearch, stats: *SolveStats) bool {
        if (self.lk_nodes_this_pass >= self.lk_backtrack_limit) {
            stats.lk_backtrack_cutoff_hits += 1;
            return false;
        }
        self.lk_nodes_this_pass += 1;
        stats.lk_search_nodes += 1;
        return true;
    }

    pub fn searchAdded(self: *LocalSearch, depth: usize, even: usize, gain: i64, stats: *SolveStats) bool {
        if (!self.recordLKNode(stats)) return false;
        const sequence_len = 2 * depth;
        const t1 = self.ws.lk_t[0];
        // Backtracking discipline: beyond lk_backtrack_depth the search
        // commits to the first viable candidate instead of retrying siblings
        // after a failed subtree.
        const greedy = depth > self.lk_backtrack_depth;
        for (self.candidates.row(even), 0..) |odd_next, ci| {
            if (odd_next == t1) continue;
            if (self.vertexInSequence(odd_next, sequence_len)) continue;
            if (self.isTourEdge(even, odd_next)) continue;
            if (self.edgeInList(even, odd_next, self.ws.removed_a, self.ws.removed_b, depth)) continue;
            if (self.edgeInList(even, odd_next, self.ws.added_a, self.ws.added_b, depth - 1)) continue;

            const edge_cost: i64 = @intCast(self.candidates.candDist(even, ci));
            const next_gain = gain - edge_cost;
            // R5 admissible early-break: with a distance-sorted row, edge_cost is
            // monotone non-decreasing, so once the added edge alone wipes out the
            // gain every later candidate does too. The skipped candidates would
            // have hit the same `continue` (no recordLKNode/searchRemoved here),
            // so this is bit-identical, never a budget change. Alpha-nearness
            // rows are not distance-sorted, so the scan stays exhaustive.
            if (next_gain <= 0) {
                if (self.candidates.dist_sorted) break;
                continue;
            }

            self.ws.added_a[depth - 1] = even;
            self.ws.added_b[depth - 1] = odd_next;
            self.ws.lk_t[sequence_len] = odd_next;
            if (self.searchRemoved(depth + 1, odd_next, next_gain, stats)) return true;
            if (greedy) return false;
        }
        return false;
    }

    pub fn searchRemoved(self: *LocalSearch, depth: usize, odd: usize, gain: i64, stats: *SolveStats) bool {
        if (!self.recordLKNode(stats)) return false;
        stats.max_depth_reached = @max(stats.max_depth_reached, depth);

        var choices = [2]usize{ self.tourNext(odd), self.tourPrev(odd) };
        self.orderTourEdgeChoices(odd, &choices);
        const sequence_len_before_even = 2 * depth - 1;
        const t1 = self.ws.lk_t[0];
        const greedy = depth > self.lk_backtrack_depth;

        for (choices) |even| {
            if (even == t1) continue;
            if (self.vertexInSequence(even, sequence_len_before_even)) continue;
            if (self.edgeInList(odd, even, self.ws.removed_a, self.ws.removed_b, depth - 1)) continue;
            if (self.edgeInList(odd, even, self.ws.added_a, self.ws.added_b, depth - 1)) continue;

            self.ws.removed_a[depth - 1] = odd;
            self.ws.removed_b[depth - 1] = even;
            self.ws.lk_t[sequence_len_before_even] = even;
            const gain_with_removed = gain + @as(i64, @intCast(self.dist.distance(odd, even)));
            const closing_cost: i64 = @intCast(self.dist.distance(even, t1));
            const closing_gain = gain_with_removed - closing_cost;

            if (closing_gain > 0 and !self.edgeInList(even, t1, self.ws.added_a, self.ws.added_b, depth - 1)) {
                self.ws.added_a[depth - 1] = even;
                self.ws.added_b[depth - 1] = t1;
                if (self.testAndApplyMove(depth, depth, stats)) return true;
                stats.lk_rejected_closing_moves += 1;
            }

            if (self.tryLKCompletionOracle(depth, even, gain_with_removed, stats)) return true;

            if (depth < self.max_lk_depth) {
                if (self.searchAdded(depth, even, gain_with_removed, stats)) return true;
            }
            if (greedy) return false;
        }
        return false;
    }

    pub fn tryLKCompletionOracle(self: *LocalSearch, depth: usize, even: usize, gain: i64, stats: *SolveStats) bool {
        if (depth < 3 or self.lk_nonseq_branch_limit == 0) return false;
        if (depth + 1 <= self.max_lk_depth and self.tryLKCompletion2Opt(depth, even, gain, stats)) return true;
        if (depth + 2 <= self.max_lk_depth and self.tryLKCompletion3Opt(depth, even, gain, stats)) return true;
        return false;
    }

    pub fn tryLKCompletion2Opt(self: *LocalSearch, depth: usize, even: usize, gain: i64, stats: *SolveStats) bool {
        const t1 = self.ws.lk_t[0];
        var tried: usize = 0;
        for (self.candidates.row(even), 0..) |u, ci| {
            if (tried >= self.lk_nonseq_branch_limit) break;
            if (u == t1 or self.vertexInSequence(u, 2 * depth)) continue;
            if (self.isTourEdge(even, u)) continue;
            if (self.edgeInList(even, u, self.ws.removed_a, self.ws.removed_b, depth)) continue;
            if (self.edgeInList(even, u, self.ws.added_a, self.ws.added_b, depth - 1)) continue;
            if (!self.recordLKNode(stats)) return false;
            stats.lk_completion_attempts += 1;
            tried += 1;

            const after_first_add = gain - @as(i64, @intCast(self.candidates.candDist(even, ci)));
            if (after_first_add <= 0) {
                stats.lk_completion_rejected += 1;
                continue;
            }

            var choices = [2]usize{ self.tourNext(u), self.tourPrev(u) };
            self.orderTourEdgeChoices(u, &choices);
            for (choices) |v| {
                if (v == t1 or self.vertexInSequence(v, 2 * depth)) continue;
                if (self.edgeInList(u, v, self.ws.removed_a, self.ws.removed_b, depth)) continue;
                if (self.edgeInList(u, v, self.ws.added_a, self.ws.added_b, depth - 1)) continue;

                const after_remove = after_first_add + @as(i64, @intCast(self.dist.distance(u, v)));
                if (after_remove <= @as(i64, @intCast(self.dist.distance(v, t1)))) {
                    stats.lk_completion_rejected += 1;
                    continue;
                }
                if (sameUndirectedEdge(v, t1, even, u) or self.edgeInList(v, t1, self.ws.added_a, self.ws.added_b, depth - 1)) {
                    stats.lk_completion_rejected += 1;
                    continue;
                }

                self.ws.added_a[depth - 1] = even;
                self.ws.added_b[depth - 1] = u;
                self.ws.removed_a[depth] = u;
                self.ws.removed_b[depth] = v;
                self.ws.added_a[depth] = v;
                self.ws.added_b[depth] = t1;
                if (self.testAndApplyCompletionMove(depth + 1, depth + 1, stats)) {
                    stats.lk_completion_accepted += 1;
                    stats.lk_completion_2opt_hits += 1;
                    return true;
                }
                stats.lk_completion_rejected += 1;
            }
        }
        return false;
    }

    pub fn tryLKCompletion3Opt(self: *LocalSearch, depth: usize, even: usize, gain: i64, stats: *SolveStats) bool {
        const t1 = self.ws.lk_t[0];
        var tried: usize = 0;
        for (self.candidates.row(even), 0..) |u, ci| {
            if (tried >= self.lk_nonseq_branch_limit) break;
            if (u == t1 or self.vertexInSequence(u, 2 * depth)) continue;
            if (self.isTourEdge(even, u)) continue;
            if (self.edgeInList(even, u, self.ws.removed_a, self.ws.removed_b, depth)) continue;
            if (self.edgeInList(even, u, self.ws.added_a, self.ws.added_b, depth - 1)) continue;
            if (!self.recordLKNode(stats)) return false;
            stats.lk_completion_attempts += 1;
            tried += 1;

            const after_first_add = gain - @as(i64, @intCast(self.candidates.candDist(even, ci)));
            if (after_first_add <= 0) {
                stats.lk_completion_rejected += 1;
                continue;
            }

            var first_remove_choices = [2]usize{ self.tourNext(u), self.tourPrev(u) };
            self.orderTourEdgeChoices(u, &first_remove_choices);
            for (first_remove_choices) |v| {
                if (v == t1 or self.vertexInSequence(v, 2 * depth)) continue;
                if (self.edgeInList(u, v, self.ws.removed_a, self.ws.removed_b, depth)) continue;
                if (self.edgeInList(u, v, self.ws.added_a, self.ws.added_b, depth - 1)) continue;

                const after_first_remove = after_first_add + @as(i64, @intCast(self.dist.distance(u, v)));
                if (after_first_remove <= 0) {
                    stats.lk_completion_rejected += 1;
                    continue;
                }

                for (self.candidates.row(v), 0..) |w, wi| {
                    if (w == t1 or w == even or w == u) continue;
                    if (self.vertexInSequence(w, 2 * depth)) continue;
                    if (self.isTourEdge(v, w)) continue;
                    if (self.edgeInList(v, w, self.ws.removed_a, self.ws.removed_b, depth)) continue;
                    if (sameUndirectedEdge(v, w, even, u)) continue;
                    if (self.edgeInList(v, w, self.ws.added_a, self.ws.added_b, depth - 1)) continue;

                    const after_second_add = after_first_remove - @as(i64, @intCast(self.candidates.candDist(v, wi)));
                    if (after_second_add <= 0) continue;

                    var second_remove_choices = [2]usize{ self.tourNext(w), self.tourPrev(w) };
                    self.orderTourEdgeChoices(w, &second_remove_choices);
                    for (second_remove_choices) |x| {
                        if (x == t1 or x == even or x == u or x == v) continue;
                        if (self.vertexInSequence(x, 2 * depth)) continue;
                        if (sameUndirectedEdge(w, x, u, v)) continue;
                        if (self.edgeInList(w, x, self.ws.removed_a, self.ws.removed_b, depth)) continue;
                        if (sameUndirectedEdge(w, x, even, u) or sameUndirectedEdge(w, x, v, w)) continue;
                        if (self.edgeInList(w, x, self.ws.added_a, self.ws.added_b, depth - 1)) continue;

                        const after_second_remove = after_second_add + @as(i64, @intCast(self.dist.distance(w, x)));
                        if (after_second_remove <= @as(i64, @intCast(self.dist.distance(x, t1)))) {
                            stats.lk_completion_rejected += 1;
                            continue;
                        }
                        if (sameUndirectedEdge(x, t1, even, u) or sameUndirectedEdge(x, t1, v, w) or self.edgeInList(x, t1, self.ws.added_a, self.ws.added_b, depth - 1)) {
                            stats.lk_completion_rejected += 1;
                            continue;
                        }

                        self.ws.added_a[depth - 1] = even;
                        self.ws.added_b[depth - 1] = u;
                        self.ws.removed_a[depth] = u;
                        self.ws.removed_b[depth] = v;
                        self.ws.added_a[depth] = v;
                        self.ws.added_b[depth] = w;
                        self.ws.removed_a[depth + 1] = w;
                        self.ws.removed_b[depth + 1] = x;
                        self.ws.added_a[depth + 1] = x;
                        self.ws.added_b[depth + 1] = t1;
                        if (self.testAndApplyCompletionMove(depth + 2, depth + 2, stats)) {
                            stats.lk_completion_accepted += 1;
                            stats.lk_completion_3opt_hits += 1;
                            return true;
                        }
                        stats.lk_completion_rejected += 1;
                    }
                }
            }
        }
        return false;
    }

    pub fn improveBoundedThreeOptCleanup(self: *LocalSearch, stats: *SolveStats) bool {
        const n = self.tour.len;
        if (n < 6) return false;

        for (0..n) |i| {
            const a = self.tour[i];
            // Bounded 3-opt cleanup sweep: next/prev cache is stale (see
            // improve2Opt note) — read successors from tour[]; tourSeq is fine.
            const b = self.tour[(i + 1) % n];
            const ab = @as(u64, self.dist.distance(a, b));

            for (self.candidates.row(a)) |c| {
                const j = self.tourSeq(c);
                if (j <= i + 1 or j + 1 >= n) continue;
                const d = self.tour[(j + 1) % n];
                if (d == a or d == b) continue;
                const cd = @as(u64, self.dist.distance(c, d));

                for (self.candidates.row(d)) |e| {
                    const k = self.tourSeq(e);
                    if (k <= j + 1 or k + 1 >= n) continue;
                    const f = self.tour[(k + 1) % n];
                    if (f == a or f == b or f == c or f == d) continue;

                    stats.bounded_three_opt_cleanup_attempts += 1;
                    const removed = ab + cd + self.dist.distance(e, f);
                    if (self.tryBoundedThreeOptCleanupPattern(removed, a, b, c, d, e, f, .case_a, stats)) return true;
                    if (self.tryBoundedThreeOptCleanupPattern(removed, a, b, c, d, e, f, .case_b, stats)) return true;
                    if (self.tryBoundedThreeOptCleanupPattern(removed, a, b, c, d, e, f, .case_c, stats)) return true;
                    if (self.tryBoundedThreeOptCleanupPattern(removed, a, b, c, d, e, f, .case_d, stats)) return true;
                }
            }
        }
        return false;
    }

    const BoundedThreeOptCleanupPattern = enum {
        case_a,
        case_b,
        case_c,
        case_d,
    };

    pub fn tryBoundedThreeOptCleanupPattern(
        self: *LocalSearch,
        removed_cost: u64,
        a: usize,
        b: usize,
        c: usize,
        d: usize,
        e: usize,
        f: usize,
        pattern: BoundedThreeOptCleanupPattern,
        stats: *SolveStats,
    ) bool {
        self.ws.removed_a[0] = a;
        self.ws.removed_b[0] = b;
        self.ws.removed_a[1] = c;
        self.ws.removed_b[1] = d;
        self.ws.removed_a[2] = e;
        self.ws.removed_b[2] = f;

        const added_cost: u64 = switch (pattern) {
            .case_a => blk: {
                self.ws.added_a[0] = a;
                self.ws.added_b[0] = c;
                self.ws.added_a[1] = b;
                self.ws.added_b[1] = e;
                self.ws.added_a[2] = d;
                self.ws.added_b[2] = f;
                break :blk self.dist.distance(a, c) + @as(u64, self.dist.distance(b, e)) + self.dist.distance(d, f);
            },
            .case_b => blk: {
                self.ws.added_a[0] = a;
                self.ws.added_b[0] = d;
                self.ws.added_a[1] = e;
                self.ws.added_b[1] = b;
                self.ws.added_a[2] = c;
                self.ws.added_b[2] = f;
                break :blk self.dist.distance(a, d) + @as(u64, self.dist.distance(e, b)) + self.dist.distance(c, f);
            },
            .case_c => blk: {
                self.ws.added_a[0] = a;
                self.ws.added_b[0] = e;
                self.ws.added_a[1] = d;
                self.ws.added_b[1] = b;
                self.ws.added_a[2] = c;
                self.ws.added_b[2] = f;
                break :blk self.dist.distance(a, e) + @as(u64, self.dist.distance(d, b)) + self.dist.distance(c, f);
            },
            .case_d => blk: {
                self.ws.added_a[0] = a;
                self.ws.added_b[0] = c;
                self.ws.added_a[1] = b;
                self.ws.added_b[1] = d;
                self.ws.added_a[2] = e;
                self.ws.added_b[2] = f;
                break :blk self.dist.distance(a, c) + @as(u64, self.dist.distance(b, d)) + self.dist.distance(e, f);
            },
        };

        if (added_cost >= removed_cost) return false;
        return self.testAndApplyMove(3, 3, stats);
    }

    pub fn testAndApplyMove(self: *LocalSearch, removed_count: usize, added_count: usize, stats: *SolveStats) bool {
        if (removed_count == 2 and added_count == 2 and self.applyDepth2ClosingMove()) {
            self.applyLengthDeltaArrays(removed_count, added_count);
            self.lkActivateMoveEndpoints(removed_count, added_count);
            stats.lk_applied_depth_total += removed_count;
            stats.lk_deepest_applied_depth = @max(stats.lk_deepest_applied_depth, removed_count);
            if (std.debug.runtime_safety) std.debug.assert(self.debugTourIsValid());
            if (std.debug.runtime_safety) std.debug.assert(self.debugSegmentMatchesFlatMaterialization());
            return true;
        }
        if (!self.planAndApplyMove(removed_count, added_count, stats)) return false;
        stats.lk_applied_depth_total += removed_count;
        stats.lk_deepest_applied_depth = @max(stats.lk_deepest_applied_depth, removed_count);
        if (std.debug.runtime_safety) std.debug.assert(self.debugTourIsValid());
        if (std.debug.runtime_safety) std.debug.assert(self.debugSegmentMatchesFlatMaterialization());
        return true;
    }

    pub fn testAndApplyCompletionMove(self: *LocalSearch, removed_count: usize, added_count: usize, stats: *SolveStats) bool {
        const patch_hits_before = stats.move_plan_patch_hits;
        if (!self.planAndApplyMoveInternal(removed_count, added_count, stats, true, false)) return false;
        stats.lk_applied_depth_total += removed_count;
        stats.lk_deepest_applied_depth = @max(stats.lk_deepest_applied_depth, removed_count);
        if (stats.move_plan_patch_hits > patch_hits_before) stats.lk_completion_patch_hits += 1;
        if (std.debug.runtime_safety) std.debug.assert(self.debugTourIsValid());
        if (std.debug.runtime_safety) std.debug.assert(self.debugSegmentMatchesFlatMaterialization());
        return true;
    }

    pub fn testAndApplyGain23BridgeMove(self: *LocalSearch, edge_count: usize, stats: *SolveStats) bool {
        const patch_hits_before = stats.move_plan_patch_hits;
        if (!self.planAndApplyMoveInternal(edge_count, edge_count, stats, true, true)) return false;
        if (stats.move_plan_patch_hits > patch_hits_before) stats.lk_completion_patch_hits += 1;
        const depth = edge_count + 2;
        stats.lk_applied_depth_total += depth;
        stats.lk_deepest_applied_depth = @max(stats.lk_deepest_applied_depth, depth);
        if (std.debug.runtime_safety) std.debug.assert(self.debugTourIsValid());
        if (std.debug.runtime_safety) std.debug.assert(self.debugSegmentMatchesFlatMaterialization());
        return true;
    }

    pub fn planAndApplyMove(self: *LocalSearch, removed_count: usize, added_count: usize, stats: *SolveStats) bool {
        return self.planAndApplyMoveInternal(removed_count, added_count, stats, false, false);
    }

    pub fn planAndApplyMoveInternal(self: *LocalSearch, removed_count: usize, added_count: usize, stats: *SolveStats, allow_completion_patch: bool, skip_structurally_impossible_fallback: bool) bool {
        stats.move_plan_attempts += 1;
        for (0..removed_count) |i| {
            self.ws.move_edges[i] = .{ .a = self.ws.removed_a[i], .b = self.ws.removed_b[i] };
        }
        for (0..added_count) |i| {
            self.ws.move_edges[removed_count + i] = .{ .a = self.ws.added_a[i], .b = self.ws.added_b[i] };
        }

        const removed_edges = self.ws.move_edges[0..removed_count];
        const added_edges = self.ws.move_edges[removed_count .. removed_count + added_count];
        var view = self.tourView();
        if (skip_structurally_impossible_fallback and !self.moveDeltaHasValidEdgeSet(&view, removed_edges, added_edges)) return false;
        var plan = MovePlan.init(removed_edges, added_edges);
        @memcpy(self.ws.candidate_tour, self.tour);
        if (!plan.validate(
            &view,
            self.ws.move_degree_delta,
            self.ws.scratch_neighbor0,
            self.ws.scratch_neighbor1,
            self.ws.move_component,
            self.ws.move_component_size,
            self.ws.scratch_seen,
        )) {
            if (skip_structurally_impossible_fallback) return false;
            stats.move_plan_invalid_fallbacks += 1;
            return self.applyMoveWithHamiltonianFallback(removed_count, added_count, stats);
        }
        if (plan.component_count != 1) {
            stats.move_plan_multi_component_fallbacks += 1;
            if (allow_completion_patch and self.tryPatchTwoComponents(&plan, removed_count, added_count, stats, self.lk_completion_patch_min_gain)) return true;
            if (self.applyMoveWithHamiltonianFallback(removed_count, added_count, stats)) return true;
            return false;
        }
        if (!view.applyEdges(removed_edges, added_edges)) {
            stats.move_plan_apply_fallbacks += 1;
            return self.applyMoveWithHamiltonianFallback(removed_count, added_count, stats);
        }
        self.stats.tour_rebuilds += 1;
        // applyEdges only succeeds after walking a single Hamiltonian cycle and
        // rebuilding; the O(n) re-validation is debug-build paranoia.
        if (std.debug.runtime_safety and (!self.debugTourIsValid() or !self.debugSegmentMatchesFlatMaterialization())) {
            stats.move_plan_apply_fallbacks += 1;
            return self.applyMoveWithHamiltonianFallback(removed_count, added_count, stats);
        }
        self.applyLengthDeltaArrays(removed_count, added_count);
        self.lkActivateMoveEndpoints(removed_count, added_count);
        stats.move_plan_direct_applies += 1;
        return true;
    }

    pub fn moveDeltaHasValidEdgeSet(self: *LocalSearch, view: *const TourView, removed_edges: []const TourEdge, added_edges: []const TourEdge) bool {
        const n = view.len();
        if (removed_edges.len == 0 or removed_edges.len != added_edges.len) return false;
        @memset(self.ws.move_degree_delta, 0);

        for (removed_edges, 0..) |edge, i| {
            if (!MovePlan.validEdge(edge, n)) return false;
            if (!view.isTourEdge(edge.a, edge.b)) return false;
            if (tourEdgeInSlice(edge, removed_edges[0..i])) return false;
            self.ws.move_degree_delta[edge.a] -= 1;
            self.ws.move_degree_delta[edge.b] -= 1;
        }
        for (added_edges, 0..) |edge, i| {
            if (!MovePlan.validEdge(edge, n)) return false;
            if (view.isTourEdge(edge.a, edge.b)) return false;
            if (tourEdgeInSlice(edge, added_edges[0..i])) return false;
            if (tourEdgeInSlice(edge, removed_edges)) return false;
            self.ws.move_degree_delta[edge.a] += 1;
            self.ws.move_degree_delta[edge.b] += 1;
        }
        for (self.ws.move_degree_delta) |delta| {
            if (delta != 0) return false;
        }
        return true;
    }

    pub fn tryPatchTwoComponents(self: *LocalSearch, plan: *const MovePlan, removed_count: usize, added_count: usize, stats: *SolveStats, min_gain: i64) bool {
        if (plan.component_count != 2) return false;
        stats.move_plan_patch_attempts += 1;

        const n = self.tour.len;
        var best_cut0: TourEdge = undefined;
        var best_cut1: TourEdge = undefined;
        var best_bridge0: TourEdge = undefined;
        var best_bridge1: TourEdge = undefined;
        var best_gain: i64 = 0;
        const patched_start = removed_count + added_count;
        const patched_removed = self.ws.move_edges[patched_start .. patched_start + removed_count + 2];
        const patched_added = self.ws.move_edges[patched_start + removed_count + 2 .. patched_start + removed_count + added_count + 4];

        // Candidate-row scan over the smaller component only (LKH PatchCycles:
        // in-edges come from candidate sets). The previous exhaustive O(n^2)
        // edge-pair scan for n > 128 dominated total runtime once patching
        // started firing on every nonsequential close.
        const smaller_component = if (self.ws.move_component_size[0] <= self.ws.move_component_size[1]) @as(usize, 0) else @as(usize, 1);
        for (0..n) |a| {
            if (self.ws.move_component[a] != smaller_component) continue;
            const neighbors = [2]usize{ self.ws.scratch_neighbor0[a], self.ws.scratch_neighbor1[a] };
            for (neighbors) |b| {
                if (b == std.math.maxInt(usize) or a > b) continue;
                if (self.ws.move_component[a] != self.ws.move_component[b]) continue;
                self.tryPatchCandidatesFromEndpoint(
                    a,
                    b,
                    removed_count,
                    added_count,
                    patched_removed,
                    patched_added,
                    &best_gain,
                    &best_cut0,
                    &best_cut1,
                    &best_bridge0,
                    &best_bridge1,
                );
                self.tryPatchCandidatesFromEndpoint(
                    b,
                    a,
                    removed_count,
                    added_count,
                    patched_removed,
                    patched_added,
                    &best_gain,
                    &best_cut0,
                    &best_cut1,
                    &best_bridge0,
                    &best_bridge1,
                );
            }
        }
        // Any positive total gain qualifies (paper: patching accepts on positive
        // cumulative gain); the final tour-length comparison below is authoritative.
        const required_gain = @max(min_gain, 1);
        if (best_gain < required_gain or !self.patchBridgesAreSupported(best_bridge0, best_bridge1, best_gain, required_gain)) {
            stats.move_plan_patch_rejected += 1;
            return false;
        }

        const final_counts = self.buildPatchedDelta(
            removed_count,
            added_count,
            &.{ best_cut0, best_cut1 },
            &.{ best_bridge0, best_bridge1 },
            patched_removed,
            patched_added,
        ) orelse {
            stats.move_plan_patch_rejected += 1;
            return false;
        };
        const final_removed = patched_removed[0..final_counts.removed];
        const final_added = patched_added[0..final_counts.added];

        var view = self.tourView();
        var patched_plan = MovePlan.init(final_removed, final_added);
        if (!patched_plan.validate(
            &view,
            self.ws.move_degree_delta,
            self.ws.scratch_neighbor0,
            self.ws.scratch_neighbor1,
            self.ws.move_component,
            self.ws.move_component_size,
            self.ws.scratch_seen,
        ) or patched_plan.component_count != 1) {
            stats.move_plan_patch_rejected += 1;
            return false;
        }

        if (!view.applyEdges(final_removed, final_added)) {
            stats.move_plan_patch_rejected += 1;
            return false;
        }
        self.stats.tour_rebuilds += 1;
        if (std.debug.runtime_safety and (!self.debugTourIsValid() or !self.debugSegmentMatchesFlatMaterialization())) {
            @memcpy(self.tour, self.ws.candidate_tour);
            self.rebuildState();
            stats.move_plan_patch_rejected += 1;
            return false;
        }
        const before_len = self.dist.tourLengthUnchecked(self.ws.candidate_tour) catch {
            @memcpy(self.tour, self.ws.candidate_tour);
            self.rebuildState();
            stats.move_plan_patch_rejected += 1;
            return false;
        };
        const after_len = self.dist.tourLengthUnchecked(self.tour) catch {
            @memcpy(self.tour, self.ws.candidate_tour);
            self.rebuildState();
            stats.move_plan_patch_rejected += 1;
            return false;
        };
        if (after_len >= before_len) {
            @memcpy(self.tour, self.ws.candidate_tour);
            self.rebuildState();
            stats.move_plan_patch_rejected += 1;
            return false;
        }
        // Patch rewrites the edge set; current_length comes straight from the
        // after-scan this path already computed (no extra cost, exact).
        self.current_length = after_len;
        for (final_removed) |edge| {
            self.lkActivate(edge.a);
            self.lkActivate(edge.b);
        }
        for (final_added) |edge| {
            self.lkActivate(edge.a);
            self.lkActivate(edge.b);
        }
        stats.move_plan_patch_hits += 1;
        return true;
    }

    pub fn tryPatchCandidatesFromEndpoint(
        self: *LocalSearch,
        endpoint: usize,
        mate: usize,
        removed_count: usize,
        added_count: usize,
        scratch_removed: []TourEdge,
        scratch_added: []TourEdge,
        best_gain: *i64,
        best_cut0: *TourEdge,
        best_cut1: *TourEdge,
        best_bridge0: *TourEdge,
        best_bridge1: *TourEdge,
    ) void {
        for (self.candidates.row(endpoint)) |other| {
            if (self.ws.move_component[other] == self.ws.move_component[endpoint]) continue;
            const neighbor_choices = [2]usize{ self.ws.scratch_neighbor0[other], self.ws.scratch_neighbor1[other] };
            for (neighbor_choices) |other_mate| {
                if (other_mate == std.math.maxInt(usize)) continue;
                if (self.ws.move_component[other_mate] != self.ws.move_component[other]) continue;
                if (other_mate == endpoint or other_mate == mate) continue;

                self.recordPatchCandidate(
                    removed_count,
                    added_count,
                    .{ .a = endpoint, .b = mate },
                    .{ .a = other, .b = other_mate },
                    .{ .a = endpoint, .b = other },
                    .{ .a = mate, .b = other_mate },
                    scratch_removed,
                    scratch_added,
                    best_gain,
                    best_cut0,
                    best_cut1,
                    best_bridge0,
                    best_bridge1,
                );
                self.recordPatchCandidate(
                    removed_count,
                    added_count,
                    .{ .a = endpoint, .b = mate },
                    .{ .a = other, .b = other_mate },
                    .{ .a = endpoint, .b = other_mate },
                    .{ .a = mate, .b = other },
                    scratch_removed,
                    scratch_added,
                    best_gain,
                    best_cut0,
                    best_cut1,
                    best_bridge0,
                    best_bridge1,
                );
            }
        }
    }

    pub fn recordPatchCandidate(
        self: *LocalSearch,
        removed_count: usize,
        added_count: usize,
        cut0: TourEdge,
        cut1: TourEdge,
        bridge0: TourEdge,
        bridge1: TourEdge,
        scratch_removed: []TourEdge,
        scratch_added: []TourEdge,
        best_gain: *i64,
        best_cut0: *TourEdge,
        best_cut1: *TourEdge,
        best_bridge0: *TourEdge,
        best_bridge1: *TourEdge,
    ) void {
        const candidate_gain = self.patchedDeltaGain(
            removed_count,
            added_count,
            &.{ cut0, cut1 },
            &.{ bridge0, bridge1 },
            scratch_removed,
            scratch_added,
        ) orelse return;
        if (candidate_gain <= best_gain.*) return;
        best_gain.* = candidate_gain;
        best_cut0.* = cut0;
        best_cut1.* = cut1;
        best_bridge0.* = bridge0;
        best_bridge1.* = bridge1;
    }

    pub fn patchBridgesAreSupported(self: *const LocalSearch, bridge0: TourEdge, bridge1: TourEdge, gain: i64, required_gain: i64) bool {
        const n = self.tour.len;
        if (n < 128) return true;
        const supported = @as(usize, @intFromBool(self.isCandidateEdge(bridge0.a, bridge0.b))) +
            @as(usize, @intFromBool(self.isCandidateEdge(bridge1.a, bridge1.b)));
        if (supported == 2) return true;
        if (supported == 1 and gain >= required_gain * 2) return true;
        return gain >= required_gain * 4;
    }

    pub fn isCandidateEdge(self: *const LocalSearch, a: usize, b: usize) bool {
        for (self.candidates.row(a)) |candidate| {
            if (candidate == b) return true;
        }
        for (self.candidates.row(b)) |candidate| {
            if (candidate == a) return true;
        }
        return false;
    }

    const PatchedDeltaCounts = struct {
        removed: usize,
        added: usize,
    };

    pub fn buildPatchedDelta(
        self: *LocalSearch,
        removed_count: usize,
        added_count: usize,
        cuts: []const TourEdge,
        bridges: []const TourEdge,
        out_removed: []TourEdge,
        out_added: []TourEdge,
    ) ?PatchedDeltaCounts {
        var removed_len: usize = 0;
        var added_len: usize = 0;
        for (0..removed_count) |i| {
            out_removed[removed_len] = .{ .a = self.ws.removed_a[i], .b = self.ws.removed_b[i] };
            removed_len += 1;
        }
        for (0..added_count) |i| {
            out_added[added_len] = .{ .a = self.ws.added_a[i], .b = self.ws.added_b[i] };
            added_len += 1;
        }

        for (cuts) |cut| {
            if (removeTourEdgeFromSlice(out_added[0..added_len], cut)) |idx| {
                added_len -= 1;
                out_added[idx] = out_added[added_len];
            } else {
                if (!self.isTourEdge(cut.a, cut.b)) return null;
                if (tourEdgeInSlice(cut, out_removed[0..removed_len])) return null;
                out_removed[removed_len] = cut;
                removed_len += 1;
            }
        }

        for (bridges) |bridge| {
            if (bridge.a == bridge.b) return null;
            if (self.isTourEdge(bridge.a, bridge.b)) {
                if (removeTourEdgeFromSlice(out_removed[0..removed_len], bridge)) |idx| {
                    removed_len -= 1;
                    out_removed[idx] = out_removed[removed_len];
                } else {
                    return null;
                }
            } else {
                if (tourEdgeInSlice(bridge, out_added[0..added_len])) return null;
                out_added[added_len] = bridge;
                added_len += 1;
            }
        }
        if (removed_len != added_len) return null;
        return .{ .removed = removed_len, .added = added_len };
    }

    pub fn patchedDeltaGain(
        self: *LocalSearch,
        removed_count: usize,
        added_count: usize,
        cuts: []const TourEdge,
        bridges: []const TourEdge,
        scratch_removed: []TourEdge,
        scratch_added: []TourEdge,
    ) ?i64 {
        const counts = self.buildPatchedDelta(
            removed_count,
            added_count,
            cuts,
            bridges,
            scratch_removed,
            scratch_added,
        ) orelse return null;
        var removed_cost: i64 = 0;
        for (scratch_removed[0..counts.removed]) |edge| {
            removed_cost += @intCast(self.dist.distance(edge.a, edge.b));
        }
        var added_cost: i64 = 0;
        for (scratch_added[0..counts.added]) |edge| {
            added_cost += @intCast(self.dist.distance(edge.a, edge.b));
        }
        return removed_cost - added_cost;
    }

    pub fn applyMoveWithHamiltonianFallback(self: *LocalSearch, removed_count: usize, added_count: usize, stats: *SolveStats) bool {
        @memcpy(self.ws.move_component_size, self.ws.candidate_tour);
        @memcpy(self.tour, self.ws.candidate_tour);
        self.rebuildState();
        if (!self.buildMoveTour(removed_count, added_count, self.ws.candidate_tour)) {
            @memcpy(self.tour, self.ws.move_component_size);
            @memcpy(self.ws.candidate_tour, self.ws.move_component_size);
            self.rebuildState();
            return false;
        }
        @memcpy(self.tour, self.ws.candidate_tour);
        self.rebuildState();
        const valid = !std.debug.runtime_safety or
            (self.debugTourIsValid() and self.debugSegmentMatchesFlatMaterialization());
        if (!valid) {
            @memcpy(self.tour, self.ws.move_component_size);
            @memcpy(self.ws.candidate_tour, self.ws.move_component_size);
            self.rebuildState();
            return false;
        }
        self.applyLengthDeltaArrays(removed_count, added_count);
        self.lkActivateMoveEndpoints(removed_count, added_count);
        stats.move_plan_fallback_successes += 1;
        return true;
    }

    pub fn applyDepth2ClosingMove(self: *LocalSearch) bool {
        const a = self.ws.removed_a[0];
        const b = self.ws.removed_b[0];
        const c = self.ws.removed_a[1];
        const d = self.ws.removed_b[1];
        if (!self.isTourEdge(a, b) or !self.isTourEdge(c, d)) return false;
        if (!sameUndirectedEdge(self.ws.added_a[0], self.ws.added_b[0], b, c)) return false;
        if (!sameUndirectedEdge(self.ws.added_a[1], self.ws.added_b[1], d, a)) return false;

        // The close adds (b,c) and (d,a). That is only a single-cycle 2-opt when
        // the removed edges face each other, i.e. tour ...a->b ... d->c...; the
        // reversal of the b..d segment then removes exactly {(a,b),(d,c)} and adds
        // exactly {(b,c),(d,a)} — the move whose gain the search verified. The
        // ...a->b ... c->d... orientation closes into two cycles and must fall
        // through to the validating applier instead of being reversed blindly.
        const pb = self.tourSeq(b);
        const pd = self.tourSeq(d);
        if (self.tourNext(a) == b and self.tourNext(d) == c and pb <= pd) {
            self.reverseBetween(b, d);
            self.rebuildState();
            return true;
        }
        return false;
    }

    pub fn debugTourIsValid(self: *LocalSearch) bool {
        @memset(self.ws.scratch_seen, false);
        for (self.tour) |node| {
            if (node >= self.tour.len or self.ws.scratch_seen[node]) return false;
            self.ws.scratch_seen[node] = true;
        }
        for (self.tour, 0..) |node, idx| {
            if (self.ws.next[node] != self.tour[(idx + 1) % self.tour.len]) return false;
            if (self.ws.prev[node] != self.tour[(idx + self.tour.len - 1) % self.tour.len]) return false;
        }
        return true;
    }

    pub fn debugSegmentMatchesFlatMaterialization(self: *LocalSearch) bool {
        if (!useSegmentTour(self.tour.len)) return true;
        var view = self.tourView();
        // Must not materialize into candidate_tour: callers (tryPatchTwoComponents,
        // planAndApplyMoveInternal) rely on candidate_tour holding the pre-move
        // snapshot for gain comparison and restore-on-reject.
        view.materialize(self.ws.move_component);
        if (!std.mem.eql(usize, self.tour, self.ws.move_component)) return false;

        const n = self.tour.len;
        const size = segmentTargetSize(n);
        var segment_count: usize = 0;
        var start: usize = 0;
        while (start < n) : (segment_count += 1) {
            const len = @min(size, n - start);
            if (self.ws.segment_start[segment_count] != start) return false;
            if (self.ws.segment_len[segment_count] != len) return false;
            if (self.ws.segment_reversed[segment_count]) return false;
            for (0..len) |rank| {
                const node = self.tour[start + rank];
                if (self.ws.segment_of_node[node] != segment_count) return false;
                if (self.ws.rank_in_segment[node] != rank) return false;
            }
            start += len;
        }
        return segment_count > 0;
    }

    pub fn buildMoveTour(self: *LocalSearch, removed_count: usize, added_count: usize, out: []usize) bool {
        const n = self.tour.len;
        for (0..n) |node| {
            self.ws.scratch_neighbor0[node] = self.tourPrev(node);
            self.ws.scratch_neighbor1[node] = self.tourNext(node);
        }

        for (0..removed_count) |i| {
            if (!self.removeScratchEdge(self.ws.removed_a[i], self.ws.removed_b[i])) return false;
        }
        for (0..added_count) |i| {
            if (!self.addScratchEdge(self.ws.added_a[i], self.ws.added_b[i])) return false;
        }

        @memset(self.ws.scratch_seen, false);
        const start = self.tour[0];
        var previous: usize = std.math.maxInt(usize);
        var current = start;
        for (0..n) |idx| {
            if (self.ws.scratch_seen[current]) return false;
            self.ws.scratch_seen[current] = true;
            out[idx] = current;
            const a = self.ws.scratch_neighbor0[current];
            const b = self.ws.scratch_neighbor1[current];
            if (a == std.math.maxInt(usize) or b == std.math.maxInt(usize)) return false;
            const next_node = if (idx == 0)
                self.preferredFirstNeighbor(start, a, b)
            else if (a == previous)
                b
            else if (b == previous)
                a
            else
                return false;
            previous = current;
            current = next_node;
        }
        if (current != start) return false;
        return true;
    }

    pub fn preferredFirstNeighbor(self: *LocalSearch, start: usize, a: usize, b: usize) usize {
        if (self.tourNext(start) == a) return a;
        if (self.tourNext(start) == b) return b;
        return @min(a, b);
    }

    pub fn removeScratchEdge(self: *LocalSearch, a: usize, b: usize) bool {
        return self.removeScratchNeighbor(a, b) and self.removeScratchNeighbor(b, a);
    }

    pub fn removeScratchNeighbor(self: *LocalSearch, a: usize, b: usize) bool {
        if (self.ws.scratch_neighbor0[a] == b) {
            self.ws.scratch_neighbor0[a] = std.math.maxInt(usize);
            return true;
        }
        if (self.ws.scratch_neighbor1[a] == b) {
            self.ws.scratch_neighbor1[a] = std.math.maxInt(usize);
            return true;
        }
        return false;
    }

    pub fn addScratchEdge(self: *LocalSearch, a: usize, b: usize) bool {
        if (a == b) return false;
        if (self.ws.scratch_neighbor0[a] == b or self.ws.scratch_neighbor1[a] == b) return false;
        if (self.ws.scratch_neighbor0[b] == a or self.ws.scratch_neighbor1[b] == a) return false;
        return self.addScratchNeighbor(a, b) and self.addScratchNeighbor(b, a);
    }

    pub fn addScratchNeighbor(self: *LocalSearch, a: usize, b: usize) bool {
        if (self.ws.scratch_neighbor0[a] == std.math.maxInt(usize)) {
            self.ws.scratch_neighbor0[a] = b;
            return true;
        }
        if (self.ws.scratch_neighbor1[a] == std.math.maxInt(usize)) {
            self.ws.scratch_neighbor1[a] = b;
            return true;
        }
        return false;
    }

    pub fn improve2Opt(self: *LocalSearch) !bool {
        const n = self.tour.len;
        for (0..n) |i| {
            const a = self.tour[i];
            // Warm-up sweeps mutate via reverseSegment (tour+pos only), so the
            // next/prev cache is stale here — read successors from tour[]/pos[].
            // Only tourSeq/reverseBetween (pos-based, always current) use the seam.
            const b = self.tour[(i + 1) % n];
            const old_ab = self.dist.distance(a, b);

            for (self.candidates.row(a), 0..) |c, ci| {
                const j = self.tourSeq(c);
                if (j <= i + 1) continue;
                if (i == 0 and j == n - 1) continue;
                const d = self.tour[(j + 1) % n];
                if (b == c or a == d) continue;

                const old_cd = self.dist.distance(c, d);
                const new_ac = self.candidates.candDist(a, ci);
                const new_bd = self.dist.distance(b, d);
                if (@as(u64, old_ab) + old_cd > @as(u64, new_ac) + new_bd) {
                    self.reverseBetween(b, c);
                    return true;
                }
            }
        }
        return false;
    }

    pub fn improveOrOpt1(self: *LocalSearch) !bool {
        const n = self.tour.len;
        if (n < 5) return false;

        for (0..n) |i| {
            const b = self.tour[i];
            // Warm-up sweep: next/prev cache is stale (see improve2Opt note).
            const a = self.tour[(i + n - 1) % n];
            const c = self.tour[(i + 1) % n];
            const remove_old = @as(u64, self.dist.distance(a, b)) + self.dist.distance(b, c);
            const remove_new = self.dist.distance(a, c);

            for (self.candidates.row(b), 0..) |x, ci| {
                const j = self.tourSeq(x);
                const y = self.tour[(j + 1) % n];
                if (x == a or x == b or x == c or y == a or y == b) continue;
                if ((j + 1) % n == i) continue;

                const insert_old = self.dist.distance(x, y);
                // candDist(b, ci) == d(b, x) == d(x, b) by symmetry (R2 cache).
                const insert_new = @as(u64, self.candidates.candDist(b, ci)) + self.dist.distance(b, y);
                if (remove_old + insert_old > @as(u64, remove_new) + insert_new) {
                    if (i < j) {
                        const moved = self.tour[i];
                        std.mem.copyForwards(usize, self.tour[i..j], self.tour[i + 1 .. j + 1]);
                        self.tour[j] = moved;
                    } else {
                        const moved = self.tour[i];
                        std.mem.copyBackwards(usize, self.tour[j + 2 .. i + 1], self.tour[j + 1 .. i]);
                        self.tour[j + 1] = moved;
                    }
                    self.rebuildPositions();
                    return true;
                }
            }
        }
        return false;
    }

    // --- Tour ADT seam (architecture H2) ------------------------------------
    // Every node-keyed tour query in the search/move logic now goes through this
    // surface, never the raw next/prev/pos arrays (verified by grep: the only
    // remaining raw reads are these accessor bodies, the reverseSegment/
    // rebuildPositions impl, and debugTourIsValid, which validates the cache).
    // Full seam surface, with the R1 (two-level list) reimplementation target:
    //   tourNext / tourPrev          -> O(1) linked successors    (next / prev)
    //   tourSeq                      -> opaque tour-order token    (sequenceId)
    //   nodeInCircularSegment        -> betweenness on tokens      (order a,b,c)
    //   circularSegmentSize /
    //     segmentIsNoMoreThanHalf    -> segment span on tokens
    //   isTourEdge                   -> adjacency
    //   reverseBetween               -> segment reversal           (reverse a,b)
    //   tour[] (read-only iteration) -> materialized order         (materialize)
    // For the array impl the first three are identity wrappers (inlined, zero
    // cost). A token is opaque: comparing two tokens is valid in any rep;
    // arithmetic on it (token+1) is array-specific and confined to the warm-up
    // sweeps, which iterate the materialized tour[] directly and which R1
    // rewrites as next()-walks.
    //
    // STALENESS CONTRACT (the item-11 HARD GATE for R1): next/prev is a CACHE,
    // rebuilt wholesale by rebuildState() (via TourView.rebuild). reverseSegment
    // updates tour+pos but NOT next/prev, so tourSeq is always current after a
    // flip while tourNext/tourPrev are STALE until the next rebuildState. The LK
    // core rebuilds after every applied move, so it reads next/prev only when
    // current; the warm-up sweeps deliberately avoid next/prev (tourSeq + tour[]
    // only). R1's prerequisite is to make next/prev always-current (maintained
    // incrementally inside reverseSegment) so the sweeps can become next-walks
    // and the per-move O(n) rebuildState can be dropped — that change is R1's,
    // not this item's (it is a trajectory-neutral perf change, not a no-op).
    inline fn tourNext(self: *const LocalSearch, c: usize) usize {
        return self.ws.next[c];
    }
    inline fn tourPrev(self: *const LocalSearch, c: usize) usize {
        return self.ws.prev[c];
    }
    inline fn tourSeq(self: *const LocalSearch, c: usize) usize {
        return self.ws.pos[c];
    }
    // Reverse the tour segment running from `first_node` to `last_node` in tour
    // order (caller guarantees first precedes last). Node-based so callers never
    // name raw positions.
    pub fn reverseBetween(self: *LocalSearch, first_node: usize, last_node: usize) void {
        self.reverseSegment(self.tourSeq(first_node), self.tourSeq(last_node));
    }

    pub fn reverseSegment(self: *LocalSearch, first: usize, last: usize) void {
        self.stats.flip_ops += 1;
        self.stats.flip_elements += last - first + 1;
        std.mem.reverse(usize, self.tour[first .. last + 1]);
        for (first..last + 1) |idx| {
            self.ws.pos[self.tour[idx]] = idx;
        }
    }

    pub fn rebuildPositions(self: *LocalSearch) void {
        for (self.tour, 0..) |node, idx| {
            self.ws.pos[node] = idx;
        }
    }

    pub fn rebuildState(self: *LocalSearch) void {
        self.stats.tour_rebuilds += 1;
        var view = self.tourView();
        view.rebuild();
    }

    // Roadmap item 2: reseed current_length from a single full scan. Called
    // once per trial after the tour is constructed/kicked/warmed-up, just
    // before the first LK descent; everything after maintains it by delta.
    pub fn syncLength(self: *LocalSearch) !void {
        self.current_length = try self.dist.tourLengthUnchecked(self.tour);
    }

    // Fold an applied move's exact length delta into current_length. The move
    // is read from removed_a/b + added_a/b: the direct-apply path, the
    // Hamiltonian fallback, and the depth-2 closing move all apply exactly that
    // edge set. (Patch moves rewrite the edge set, so they set current_length
    // from their own after-scan instead of calling this.)
    pub fn applyLengthDeltaArrays(self: *LocalSearch, removed_count: usize, added_count: usize) void {
        var added_sum: u64 = 0;
        for (0..added_count) |i| added_sum += self.dist.distance(self.ws.added_a[i], self.ws.added_b[i]);
        var removed_sum: u64 = 0;
        for (0..removed_count) |i| removed_sum += self.dist.distance(self.ws.removed_a[i], self.ws.removed_b[i]);
        // current_length includes the removed edges, so current_length +
        // added_sum >= removed_sum; no unsigned underflow.
        self.current_length = self.current_length + added_sum - removed_sum;
    }

    pub fn isTourEdge(self: *const LocalSearch, a: usize, b: usize) bool {
        return self.tourNext(a) == b or self.tourPrev(a) == b;
    }

    pub fn tourView(self: anytype) TourView {
        if (useSegmentTour(self.tour.len)) {
            return TourView.initSegment(
                self.tour,
                self.ws.pos,
                self.ws.next,
                self.ws.prev,
                self.ws.scratch_neighbor0,
                self.ws.scratch_neighbor1,
                self.ws.scratch_seen,
                self.ws.segment_of_node,
                self.ws.rank_in_segment,
                self.ws.segment_start,
                self.ws.segment_len,
                self.ws.segment_reversed,
            );
        }
        return TourView.initFlat(self.tour, self.ws.pos, self.ws.next, self.ws.prev, self.ws.scratch_neighbor0, self.ws.scratch_neighbor1, self.ws.scratch_seen);
    }

    pub fn vertexInSequence(self: *const LocalSearch, node: usize, len: usize) bool {
        for (self.ws.lk_t[0..len]) |existing| {
            if (existing == node) return true;
        }
        return false;
    }

    pub fn edgeInList(
        self: *const LocalSearch,
        a: usize,
        b: usize,
        list_a: []const usize,
        list_b: []const usize,
        len: usize,
    ) bool {
        _ = self;
        for (0..len) |i| {
            if (sameUndirectedEdge(a, b, list_a[i], list_b[i])) return true;
        }
        return false;
    }
};
