const std = @import("std");
const build_options = @import("build_options");
const problem = @import("problem.zig");
const exact = @import("exact.zig");
const tsplib = @import("tsplib.zig");

pub const SolveOptions = struct {
    seed: u64 = 1,
    trials: usize = 16,
    // Stagnation-based trial extension: when > 0, the trial loop keeps
    // running past `trials` as long as the incumbent improved within the
    // last `trials` trials, up to `trial_extension_factor * trials` total.
    // The backtracking discipline made trials 3-10x cheaper than the LKH
    // budget the `trials = dimension` convention was calibrated against;
    // without extension, runs on larger instances stop while still
    // improving (rd400 found its best tour on its final trial). Converged
    // runs stop at the stagnation window, so small instances pay nothing.
    trial_extension_factor: usize = 0,
    candidate_count: usize = 24,
    candidate_mode: CandidateMode = .nearest_distance,
    max_passes: usize = 80,
    randomized_starts: bool = true,
    enable_or_opt: bool = true,
    enable_lk: bool = true,
    enable_bounded_three_opt_cleanup: bool = true,
    enable_move_patching: bool = false,
    move_patch_min_gain: i64 = 1,
    lk_completion_patch_min_gain: i64 = 1,
    lk_max_depth: usize = 5,
    lk_backtrack_limit: usize = 100_000,
    // LKH backtracking discipline (paper p.13): sibling alternatives are
    // explored only at chain levels <= this depth; deeper levels commit to
    // the first viable continuation. Without it the search tree branches
    // width x 2 at every level, which explodes on clustered instances where
    // a long removed edge makes the positive-gain bound prune nothing.
    // null = auto: exhaustive backtracking below 400 nodes (affordable, and
    // measurably needed for the optimum on the small TSPLIB fixtures),
    // depth 2 at or above.
    lk_backtrack_depth: ?usize = null,
    lk_nonseq_branch_limit: usize = 2,
    alpha_ascent_iterations: usize = 32,
    alpha_nearest_patch_count: usize = 2,
    max_distance_cache_weights: usize = 4_000_000,
    // ============================================================================
    // ITEM 3 (edge voting-freeze) IS CLOSED — DELETE CANDIDATE (round 18, 2026-06-13).
    // Proven a structural dead end: freezing ANY edge (even a 100%-pure subset of
    // the known-optimal edges) loses accuracy because LK reaches better tours by
    // temporarily breaking-and-rebuilding edges the final tour keeps. Purity is
    // NOT the blocker; soft freeze, staleness-gating, and diverse vote sources all
    // fail too. Only gain is a 0.037% rat575 niche behind a single-fixture overfit.
    // This whole subsystem (the fields below + vote_node/vote_count workspace, the
    // vote primitives, segmentExchangeKickAvoidingFrozen, the moveRemovesFrozenEdge
    // /soft_freeze guards, and the round-18 measurement scaffolding) should be
    // removed in the next cleanup pass. Kept default-off until then. Full
    // post-mortem: item3.md. Do NOT pursue items 5/9 to "fix" this.
    // ============================================================================
    // Roadmap item 3: edge voting-freeze (Misra-Gries, k=2 counter slots per
    // node). Off by default so every existing trajectory stays bit-identical.
    // When on, each merge-gated (near-incumbent) trial votes its two tour
    // neighbours per node; an edge whose BOTH endpoints' counters clear the
    // confidence threshold is frozen. The GENERATOR (kick + LK descent)
    // preserves frozen edges; the COMBINER (EAX/IPT) never does, so joint
    // section moves still cut through frozen regions (the measured pr1002
    // mechanism). Counters auto-thaw on disagreement (Misra-Gries decrement),
    // which doubles as a confidence dial. The threshold is relative because the
    // kick-correlated vote stream inflates counters: an edge freezes only once
    // it has survived `edge_freeze_fraction_x100`% of all votes cast, and never
    // before `edge_freeze_min_votes` votes (avoids locking onto an early,
    // still-suboptimal incumbent).
    // DEFAULT OFF. Measured outcome (round 17, seeds {12345,7,99}): the
    // literal-spec variant (edge_freeze_lk_respect = true) over-constrains the
    // descent and is strictly worse everywhere. The kick-only variant
    // (lk_respect = false, defaults below) is the only useful one: it unlocks
    // rat575 across all three seeds (6779/6779/6788 -> 6776/6777/6777, the
    // long-stuck plateau target) by steering perturbation off the consensus
    // backbone while LK keeps full power. But the same freezing aggressiveness
    // regresses d657 (+100) and pr1002 (+363) with no threshold separating the
    // effects, so it fails the suite gate and stays opt-in. The defaults below
    // are the validated rat575-class config for when it is enabled.
    enable_edge_freeze: bool = false,
    edge_freeze_min_votes: u32 = 384,
    edge_freeze_fraction_x100: u32 = 95,
    // Vote source: .gated_trials votes every near-incumbent trial tour (high
    // count, but kick-correlated so it over-freezes); .distinct_incumbents
    // votes only each newly-adopted incumbent + restart optima (a sparse but
    // genuinely cross-basin stream, so the frozen set converges on the true
    // backbone instead of the current attractor).
    edge_freeze_vote_mode: EdgeFreezeVoteMode = .gated_trials,
    // When false the generator's LK keeps full power (only the kick avoids
    // frozen edges); isolates "intensify perturbation on contested regions"
    // from "forbid LK from restructuring across the backbone". Measured: false
    // (kick-only) is the only variant that ever beats baseline; true is the
    // literal spec and is strictly worse, so the default is false.
    edge_freeze_lk_respect: bool = false,
    // Item-3 revival (round 18, MEASURED — does NOT yield a win): staleness
    // gate. Freeze is only ACTED ON once the incumbent has been stale this many
    // trials; voting always accumulates. 0 = act whenever the vote threshold is
    // ready (original behaviour). Intent: protect still-productive instances
    // (d657/pr1002) while still escaping saturated ones (rat575). Outcome: it
    // NEUTRALIZES the feature — d657 is protected at window>=256, but the same
    // gate reverts rat575 to 6779 (gain gone). The rat575 escape needs
    // CONTINUOUS freeze from early (it reshapes the whole trajectory), not
    // intensify-on-stall, so no single window wins both. See item3.md.
    edge_freeze_stale_window: usize = 0,
    // Item-3 revival (round 18, MEASURED — does NOT escape the accuracy loss):
    // soft freeze (the LKH-style mechanism). Instead of forbidding a move that
    // breaks a frozen edge (edge_freeze_lk_respect, which loses accuracy
    // structurally even with a perfectly pure backbone — LK must break-and-
    // rebuild even optimal edges en route to a better tour), only skip
    // INITIATING a sequential search from an interior-backbone node (both tour
    // edges frozen). Frozen edges stay breakable by deeper search. Outcome:
    // marginal speed (lk_search_nodes is not the dominant cost — tour_rebuilds
    // is) and STILL loses accuracy (pr1002 259410 at recall 0.5). See item3.md.
    edge_freeze_soft: bool = false,
    // Diagnostic (item-3 revival): if set, the final frozen undirected edge set
    // is appended here as flattened (u, v) pairs after the trial loop. Off-path
    // and default-null so it never affects a normal solve.
    frozen_edges_out: ?*std.ArrayList(u32) = null,
    // Diagnostic (item-3 revival): a statically injected frozen backbone (packed
    // lo<<32|hi, sorted ascending), frozen from trial 0, bypassing the voted
    // set. Requires enable_edge_freeze. The upper-bound prober: feeding it a
    // subset of the KNOWN-OPTIMAL edges proved that even a 100%-pure backbone
    // loses ~0.4% on pr1002 under LK-respect — freezing fails for a structural
    // reason (LK must break-and-rebuild even optimal edges), not because the
    // vote stream is impure. Empty = inert. See item3.md.
    inject_frozen: []const u64 = &.{},
};

pub const EdgeFreezeVoteMode = enum { gated_trials, distinct_incumbents };

pub const CandidateMode = enum {
    nearest_distance,
    alpha_nearness,
    alpha_nearness_cgal,
};

pub const cgal_available = build_options.with_cgal;

pub const SolveStats = struct {
    trials: usize = 0,
    warmup_moves: u64 = 0,
    improving_moves: u64 = 0,
    lk_attempts: u64 = 0,
    lk_search_nodes: u64 = 0,
    lk_moves: u64 = 0,
    lk_rejected_closing_moves: u64 = 0,
    lk_backtrack_cutoff_hits: u64 = 0,
    lk_applied_depth_total: u64 = 0,
    lk_deepest_applied_depth: usize = 0,
    lk_nonseq_attempts: u64 = 0,
    lk_nonseq_accepted: u64 = 0,
    lk_nonseq_rejected: u64 = 0,
    lk_nonseq_depth_total: u64 = 0,
    lk_nonseq_deepest_accepted_depth: usize = 0,
    lk_chain_nonseq_depth_attempts: [8]u64 = .{0} ** 8,
    lk_chain_nonseq_depth_accepted: [8]u64 = .{0} ** 8,
    lk_chain_nonseq_depth_gain_rejected: [8]u64 = .{0} ** 8,
    lk_chain_nonseq_depth_apply_rejected: [8]u64 = .{0} ** 8,
    lk_completion_attempts: u64 = 0,
    lk_completion_accepted: u64 = 0,
    lk_completion_2opt_hits: u64 = 0,
    lk_completion_3opt_hits: u64 = 0,
    lk_completion_patch_hits: u64 = 0,
    lk_completion_rejected: u64 = 0,
    bounded_three_opt_cleanup_moves: u64 = 0,
    bounded_three_opt_cleanup_attempts: u64 = 0,
    eax_merge_attempts: u64 = 0,
    eax_merge_cycles: u64 = 0,
    eax_merge_wins: u64 = 0,
    eax_max_progress_gap: u64 = 0,
    eax_final_progress_gap: u64 = 0,
    eax_worst_gap_ratio_x100: u64 = 0,
    guided_trials: u64 = 0,
    guided_polishes: u64 = 0,
    guided_search_nodes: u64 = 0,
    merge_search_nodes: u64 = 0,
    candidate_nearest_edges: u64 = 0,
    candidate_alpha_edges: u64 = 0,
    candidate_geometric_edges: u64 = 0,
    candidate_patch_edges: u64 = 0,
    move_plan_attempts: u64 = 0,
    move_plan_direct_applies: u64 = 0,
    move_plan_invalid_fallbacks: u64 = 0,
    move_plan_multi_component_fallbacks: u64 = 0,
    move_plan_apply_fallbacks: u64 = 0,
    move_plan_fallback_successes: u64 = 0,
    move_plan_patch_attempts: u64 = 0,
    move_plan_patch_hits: u64 = 0,
    move_plan_patch_rejected: u64 = 0,
    max_depth_reached: usize = 0,
    exact_permutations: u64 = 0,
    best_trial: usize = 0,
    candidate_count: usize = 0,
    alpha_ascent_iterations: usize = 0,
    alpha_ascent_best_lower_bound: i64 = 0,
    distance_cache_nodes: usize = 0,
    distance_cache_weights: usize = 0,
    uncached_coordinate_distances: u64 = 0,
    // Per-trial cost counters (roadmap item 1): the levers below are accepted
    // or rejected against these. distance_lookups gates on-the-fly distances
    // (item 6); tour_length_scans/tour_rebuilds gate incremental bookkeeping
    // (item 2); flip_ops/flip_elements gate the B-tree tour rep (item 8); LK
    // node ops already live in lk_search_nodes. All measured over the trial
    // loop only (the one-time candidate build is excluded, cf. resetCounters).
    distance_lookups: u64 = 0,
    tour_length_scans: u64 = 0,
    tour_rebuilds: u64 = 0,
    flip_ops: u64 = 0,
    flip_elements: u64 = 0,
    // Roadmap item 3 diagnostics: votes cast, Misra-Gries decrement (thaw)
    // events, generator moves rejected for touching a frozen edge, and the
    // number of edges frozen at the final threshold. These answer "is the thaw
    // actually firing / is freezing over-constraining the search".
    freeze_votes: u64 = 0,
    freeze_decrements: u64 = 0,
    freeze_move_rejections: u64 = 0,
    frozen_edges_final: usize = 0,
};

pub const SolveResult = struct {
    allocator: std.mem.Allocator,
    tour: []usize,
    length: u64,
    stats: SolveStats = .{},

    pub fn deinit(self: *SolveResult) void {
        self.allocator.free(self.tour);
        self.* = undefined;
    }
};

const SolverError = error{
    DistanceCacheTooLarge,
};

extern fn commiv_cgal_delaunay_edges(
    xy: [*]const f64,
    point_count: usize,
    out_edges: [*]u32,
    max_edges: usize,
) usize;

const TourEdge = struct {
    a: usize,
    b: usize,
};

const MovePlan = struct {
    removed: []const TourEdge,
    added: []const TourEdge,
    component_count: usize = 0,
    smallest_component_size: usize = 0,

    fn init(removed: []const TourEdge, added: []const TourEdge) MovePlan {
        return .{ .removed = removed, .added = added };
    }

    fn validate(
        self: *MovePlan,
        view: *const TourView,
        degree_delta: []i8,
        neighbor0: []usize,
        neighbor1: []usize,
        component: []usize,
        component_size: []usize,
        seen: []bool,
    ) bool {
        const n = view.len();
        if (self.removed.len == 0 or self.removed.len != self.added.len) return false;
        @memset(degree_delta, 0);
        @memset(component, std.math.maxInt(usize));
        @memset(component_size, 0);

        for (self.removed, 0..) |edge, i| {
            if (!validEdge(edge, n)) return false;
            if (!view.isTourEdge(edge.a, edge.b)) return false;
            if (tourEdgeInSlice(edge, self.removed[0..i])) return false;
            degree_delta[edge.a] -= 1;
            degree_delta[edge.b] -= 1;
        }

        for (self.added, 0..) |edge, i| {
            if (!validEdge(edge, n)) return false;
            if (view.isTourEdge(edge.a, edge.b)) return false;
            if (tourEdgeInSlice(edge, self.added[0..i])) return false;
            if (tourEdgeInSlice(edge, self.removed)) return false;
            degree_delta[edge.a] += 1;
            degree_delta[edge.b] += 1;
        }

        for (degree_delta) |delta| {
            if (delta != 0) return false;
        }
        if (!self.analyzeComponents(view, neighbor0, neighbor1, component, component_size, seen)) return false;
        return true;
    }

    fn analyzeComponents(
        self: *MovePlan,
        view: *const TourView,
        neighbor0: []usize,
        neighbor1: []usize,
        component: []usize,
        component_size: []usize,
        seen: []bool,
    ) bool {
        const n = view.len();
        for (0..n) |node| {
            neighbor0[node] = view.prev(node);
            neighbor1[node] = view.next(node);
        }
        for (self.removed) |edge| {
            if (!removePlanEdge(edge, neighbor0, neighbor1)) return false;
        }
        for (self.added) |edge| {
            if (!addPlanEdge(edge, neighbor0, neighbor1)) return false;
        }

        @memset(seen, false);
        self.component_count = 0;
        self.smallest_component_size = std.math.maxInt(usize);
        for (0..n) |start| {
            if (seen[start]) continue;
            var current = start;
            var previous: usize = std.math.maxInt(usize);
            var size: usize = 0;
            while (true) {
                if (current >= n) return false;
                if (seen[current]) {
                    if (current != start) return false;
                    break;
                }
                seen[current] = true;
                component[current] = self.component_count;
                size += 1;
                const a = neighbor0[current];
                const b = neighbor1[current];
                if (a == std.math.maxInt(usize) or b == std.math.maxInt(usize)) return false;
                const next_node = if (previous == std.math.maxInt(usize) or previous == b) a else b;
                previous = current;
                current = next_node;
            }
            component_size[self.component_count] = size;
            self.smallest_component_size = @min(self.smallest_component_size, size);
            self.component_count += 1;
        }
        return self.component_count > 0;
    }

    fn removePlanEdge(edge: TourEdge, neighbor0: []usize, neighbor1: []usize) bool {
        return removePlanNeighbor(edge.a, edge.b, neighbor0, neighbor1) and removePlanNeighbor(edge.b, edge.a, neighbor0, neighbor1);
    }

    fn removePlanNeighbor(a: usize, b: usize, neighbor0: []usize, neighbor1: []usize) bool {
        if (neighbor0[a] == b) {
            neighbor0[a] = std.math.maxInt(usize);
            return true;
        }
        if (neighbor1[a] == b) {
            neighbor1[a] = std.math.maxInt(usize);
            return true;
        }
        return false;
    }

    fn addPlanEdge(edge: TourEdge, neighbor0: []usize, neighbor1: []usize) bool {
        if (edge.a == edge.b) return false;
        if (neighbor0[edge.a] == edge.b or neighbor1[edge.a] == edge.b) return false;
        if (neighbor0[edge.b] == edge.a or neighbor1[edge.b] == edge.a) return false;
        return addPlanNeighbor(edge.a, edge.b, neighbor0, neighbor1) and addPlanNeighbor(edge.b, edge.a, neighbor0, neighbor1);
    }

    fn addPlanNeighbor(a: usize, b: usize, neighbor0: []usize, neighbor1: []usize) bool {
        if (neighbor0[a] == std.math.maxInt(usize)) {
            neighbor0[a] = b;
            return true;
        }
        if (neighbor1[a] == std.math.maxInt(usize)) {
            neighbor1[a] = b;
            return true;
        }
        return false;
    }

    fn validEdge(edge: TourEdge, n: usize) bool {
        return edge.a < n and edge.b < n and edge.a != edge.b;
    }
};

fn tourEdgeInSlice(edge: TourEdge, edges: []const TourEdge) bool {
    for (edges) |existing| {
        if (sameUndirectedEdge(edge.a, edge.b, existing.a, existing.b)) return true;
    }
    return false;
}

fn removeTourEdgeFromSlice(edges: []const TourEdge, edge: TourEdge) ?usize {
    for (edges, 0..) |existing, idx| {
        if (sameUndirectedEdge(edge.a, edge.b, existing.a, existing.b)) return idx;
    }
    return null;
}

const FlatTourView = struct {
    tour: []usize,
    pos: []usize,
    next_nodes: []usize,
    prev_nodes: []usize,
    scratch_neighbor0: []usize,
    scratch_neighbor1: []usize,
    scratch_seen: []bool,

    fn rebuild(self: *FlatTourView) void {
        const n = self.tour.len;
        for (self.tour, 0..) |node, idx| {
            self.pos[node] = idx;
            self.prev_nodes[node] = self.tour[(idx + n - 1) % n];
            self.next_nodes[node] = self.tour[(idx + 1) % n];
        }
    }

    fn next(self: *const FlatTourView, node: usize) usize {
        return self.next_nodes[node];
    }

    fn prev(self: *const FlatTourView, node: usize) usize {
        return self.prev_nodes[node];
    }

    fn between(self: *const FlatTourView, a: usize, b: usize, c: usize) bool {
        if (a == b or b == c) return false;
        const pa = self.pos[a];
        const pb = self.pos[b];
        const pc = self.pos[c];
        if (pa <= pc) return pa < pb and pb < pc;
        return pb > pa or pb < pc;
    }

    fn flipPath(self: *FlatTourView, first_node: usize, last_node: usize) void {
        var first = self.pos[first_node];
        var last = self.pos[last_node];
        if (first > last) std.mem.swap(usize, &first, &last);
        std.mem.reverse(usize, self.tour[first .. last + 1]);
        for (first..last + 1) |idx| {
            self.pos[self.tour[idx]] = idx;
        }
        self.rebuild();
    }

    fn applyEdges(self: *FlatTourView, removed: []const TourEdge, added: []const TourEdge) bool {
        const n = self.tour.len;
        for (0..n) |node| {
            self.scratch_neighbor0[node] = self.prev_nodes[node];
            self.scratch_neighbor1[node] = self.next_nodes[node];
        }

        for (removed) |edge| {
            if (!self.removeScratchEdge(edge.a, edge.b)) return false;
        }
        for (added) |edge| {
            if (!self.addScratchEdge(edge.a, edge.b)) return false;
        }

        @memset(self.scratch_seen, false);
        const start = self.tour[0];
        var previous: usize = std.math.maxInt(usize);
        var current = start;
        for (0..n) |idx| {
            if (self.scratch_seen[current]) return false;
            self.scratch_seen[current] = true;
            self.tour[idx] = current;
            const a = self.scratch_neighbor0[current];
            const b = self.scratch_neighbor1[current];
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
        self.rebuild();
        return true;
    }

    fn materialize(self: *const FlatTourView, out: []usize) void {
        std.debug.assert(out.len == self.tour.len);
        @memcpy(out, self.tour);
    }

    fn preferredFirstNeighbor(self: *const FlatTourView, start: usize, a: usize, b: usize) usize {
        if (self.next_nodes[start] == a) return a;
        if (self.next_nodes[start] == b) return b;
        return @min(a, b);
    }

    fn removeScratchEdge(self: *FlatTourView, a: usize, b: usize) bool {
        return self.removeScratchNeighbor(a, b) and self.removeScratchNeighbor(b, a);
    }

    fn removeScratchNeighbor(self: *FlatTourView, a: usize, b: usize) bool {
        if (self.scratch_neighbor0[a] == b) {
            self.scratch_neighbor0[a] = std.math.maxInt(usize);
            return true;
        }
        if (self.scratch_neighbor1[a] == b) {
            self.scratch_neighbor1[a] = std.math.maxInt(usize);
            return true;
        }
        return false;
    }

    fn addScratchEdge(self: *FlatTourView, a: usize, b: usize) bool {
        if (a == b) return false;
        if (self.scratch_neighbor0[a] == b or self.scratch_neighbor1[a] == b) return false;
        if (self.scratch_neighbor0[b] == a or self.scratch_neighbor1[b] == a) return false;
        return self.addScratchNeighbor(a, b) and self.addScratchNeighbor(b, a);
    }

    fn addScratchNeighbor(self: *FlatTourView, a: usize, b: usize) bool {
        if (self.scratch_neighbor0[a] == std.math.maxInt(usize)) {
            self.scratch_neighbor0[a] = b;
            return true;
        }
        if (self.scratch_neighbor1[a] == std.math.maxInt(usize)) {
            self.scratch_neighbor1[a] = b;
            return true;
        }
        return false;
    }
};

const SegmentTourView = struct {
    flat: FlatTourView,
    segment_of_node: []usize,
    rank_in_segment: []usize,
    segment_start: []usize,
    segment_len: []usize,
    segment_reversed: []bool,
    target_segment_size: usize,
    segment_count: usize = 0,

    fn rebuild(self: *SegmentTourView) void {
        self.flat.rebuild();
        self.rebuildSegments();
    }

    fn next(self: *const SegmentTourView, node: usize) usize {
        return self.flat.next(node);
    }

    fn prev(self: *const SegmentTourView, node: usize) usize {
        return self.flat.prev(node);
    }

    fn between(self: *const SegmentTourView, a: usize, b: usize, c: usize) bool {
        return self.flat.between(a, b, c);
    }

    fn flipPath(self: *SegmentTourView, first_node: usize, last_node: usize) void {
        self.flat.flipPath(first_node, last_node);
        self.rebuildSegments();
    }

    fn applyEdges(self: *SegmentTourView, removed: []const TourEdge, added: []const TourEdge) bool {
        if (!self.flat.applyEdges(removed, added)) return false;
        self.rebuildSegments();
        return true;
    }

    fn materialize(self: *const SegmentTourView, out: []usize) void {
        self.flat.materialize(out);
    }

    fn rebuildSegments(self: *SegmentTourView) void {
        const n = self.flat.tour.len;
        const size = @max(self.target_segment_size, 1);
        self.segment_count = 0;
        var start: usize = 0;
        while (start < n) : (self.segment_count += 1) {
            const len = @min(size, n - start);
            self.segment_start[self.segment_count] = start;
            self.segment_len[self.segment_count] = len;
            self.segment_reversed[self.segment_count] = false;
            for (0..len) |rank| {
                const node = self.flat.tour[start + rank];
                self.segment_of_node[node] = self.segment_count;
                self.rank_in_segment[node] = rank;
            }
            start += len;
        }
    }
};

const TourView = union(enum) {
    flat: FlatTourView,
    segment: SegmentTourView,

    fn initFlat(
        tour: []usize,
        pos: []usize,
        next_nodes: []usize,
        prev_nodes: []usize,
        scratch_neighbor0: []usize,
        scratch_neighbor1: []usize,
        scratch_seen: []bool,
    ) TourView {
        return .{ .flat = .{
            .tour = tour,
            .pos = pos,
            .next_nodes = next_nodes,
            .prev_nodes = prev_nodes,
            .scratch_neighbor0 = scratch_neighbor0,
            .scratch_neighbor1 = scratch_neighbor1,
            .scratch_seen = scratch_seen,
        } };
    }

    fn initSegment(
        tour: []usize,
        pos: []usize,
        next_nodes: []usize,
        prev_nodes: []usize,
        scratch_neighbor0: []usize,
        scratch_neighbor1: []usize,
        scratch_seen: []bool,
        segment_of_node: []usize,
        rank_in_segment: []usize,
        segment_start: []usize,
        segment_len: []usize,
        segment_reversed: []bool,
    ) TourView {
        return .{ .segment = .{
            .flat = .{
                .tour = tour,
                .pos = pos,
                .next_nodes = next_nodes,
                .prev_nodes = prev_nodes,
                .scratch_neighbor0 = scratch_neighbor0,
                .scratch_neighbor1 = scratch_neighbor1,
                .scratch_seen = scratch_seen,
            },
            .segment_of_node = segment_of_node,
            .rank_in_segment = rank_in_segment,
            .segment_start = segment_start,
            .segment_len = segment_len,
            .segment_reversed = segment_reversed,
            .target_segment_size = segmentTargetSize(tour.len),
        } };
    }

    fn rebuild(self: *TourView) void {
        switch (self.*) {
            .flat => |*view| view.rebuild(),
            .segment => |*view| view.rebuild(),
        }
    }

    fn next(self: *const TourView, node: usize) usize {
        return switch (self.*) {
            .flat => |*view| view.next(node),
            .segment => |*view| view.next(node),
        };
    }

    fn prev(self: *const TourView, node: usize) usize {
        return switch (self.*) {
            .flat => |*view| view.prev(node),
            .segment => |*view| view.prev(node),
        };
    }

    fn len(self: *const TourView) usize {
        return switch (self.*) {
            .flat => |*view| view.tour.len,
            .segment => |*view| view.flat.tour.len,
        };
    }

    fn isTourEdge(self: *const TourView, a: usize, b: usize) bool {
        return self.next(a) == b or self.prev(a) == b;
    }

    fn between(self: *const TourView, a: usize, b: usize, c: usize) bool {
        return switch (self.*) {
            .flat => |*view| view.between(a, b, c),
            .segment => |*view| view.between(a, b, c),
        };
    }

    fn flipPath(self: *TourView, first_node: usize, last_node: usize) void {
        switch (self.*) {
            .flat => |*view| view.flipPath(first_node, last_node),
            .segment => |*view| view.flipPath(first_node, last_node),
        }
    }

    fn applyEdges(self: *TourView, removed: []const TourEdge, added: []const TourEdge) bool {
        return switch (self.*) {
            .flat => |*view| view.applyEdges(removed, added),
            .segment => |*view| view.applyEdges(removed, added),
        };
    }

    fn materialize(self: *const TourView, out: []usize) void {
        switch (self.*) {
            .flat => |*view| view.materialize(out),
            .segment => |*view| view.materialize(out),
        }
    }
};

const segmentTourThreshold: usize = 512;

fn useSegmentTour(n: usize) bool {
    return n >= segmentTourThreshold;
}

fn segmentTargetSize(n: usize) usize {
    var size: usize = 1;
    while (size * size < n) : (size += 1) {}
    return @max(size, 1);
}

pub const Candidates = struct {
    allocator: std.mem.Allocator,
    width: usize,
    data: []usize,
    alpha: []u64,

    pub fn deinit(self: *Candidates) void {
        self.allocator.free(self.data);
        self.allocator.free(self.alpha);
        self.* = undefined;
    }

    pub fn row(self: *const Candidates, node: usize) []const usize {
        const start = node * self.width;
        return self.data[start .. start + self.width];
    }

    pub fn alphaRow(self: *const Candidates, node: usize) []const u64 {
        const start = node * self.width;
        return self.alpha[start .. start + self.width];
    }
};

pub const CandidateBuildStats = struct {
    iterations: usize = 0,
    best_lower_bound: i64 = 0,
    nearest_edges: u64 = 0,
    alpha_edges: u64 = 0,
    geometric_edges: u64 = 0,
    patched_edges: u64 = 0,
};

pub const DistanceOracle = struct {
    allocator: std.mem.Allocator,
    p: *const problem.Problem,
    matrix: []const u32,
    owned_matrix: []u32,
    uncached_coordinate_distances: u64 = 0,
    lookups: u64 = 0,
    length_scans: u64 = 0,

    pub fn init(
        allocator: std.mem.Allocator,
        p: *const problem.Problem,
        max_cached_weights: usize,
    ) !DistanceOracle {
        if (p.distance_kind == .explicit_full_matrix) {
            return .{
                .allocator = allocator,
                .p = p,
                .matrix = p.matrix,
                .owned_matrix = &.{},
            };
        }

        const total = std.math.mul(usize, p.dimension, p.dimension) catch return SolverError.DistanceCacheTooLarge;
        if (max_cached_weights == 0 or total > max_cached_weights) {
            return .{
                .allocator = allocator,
                .p = p,
                .matrix = &.{},
                .owned_matrix = &.{},
            };
        }

        const matrix = try allocator.alloc(u32, total);
        errdefer allocator.free(matrix);

        for (0..p.dimension) |row| {
            const offset = row * p.dimension;
            for (0..p.dimension) |col| {
                matrix[offset + col] = p.distanceUnchecked(row, col);
            }
        }

        return .{
            .allocator = allocator,
            .p = p,
            .matrix = matrix,
            .owned_matrix = matrix,
        };
    }

    pub fn deinit(self: *DistanceOracle) void {
        if (self.owned_matrix.len != 0) self.allocator.free(self.owned_matrix);
        self.* = undefined;
    }

    fn isCached(self: *const DistanceOracle) bool {
        return self.matrix.len != 0;
    }

    // Zeroes the per-trial cost counters after the one-time candidate build so
    // they measure only the trial loop (roadmap item 1).
    fn resetCounters(self: *DistanceOracle) void {
        self.uncached_coordinate_distances = 0;
        self.lookups = 0;
        self.length_scans = 0;
    }

    fn distance(self: *DistanceOracle, a: usize, b: usize) u32 {
        self.lookups += 1;
        if (self.matrix.len != 0) return self.matrix[a * self.p.dimension + b];
        if (self.p.distance_kind != .explicit_full_matrix) self.uncached_coordinate_distances += 1;
        return self.p.distanceUnchecked(a, b);
    }

    fn tourLengthUnchecked(self: *DistanceOracle, tour: []const usize) problem.ProblemError!u64 {
        std.debug.assert(tour.len == self.p.dimension);
        self.length_scans += 1;
        var total: u64 = 0;
        for (0..tour.len) |i| {
            const a = tour[i];
            const b = tour[(i + 1) % tour.len];
            total = std.math.add(u64, total, @as(u64, self.distance(a, b))) catch {
                return problem.ProblemError.DistanceOverflow;
            };
        }
        return total;
    }
};

const SolverWorkspace = struct {
    allocator: std.mem.Allocator,
    best_tour: []usize,
    prev_best_tour: []usize,
    tour: []usize,
    pos: []usize,
    used: []bool,
    next: []usize,
    prev: []usize,
    candidate_tour: []usize,
    scratch_neighbor0: []usize,
    scratch_neighbor1: []usize,
    scratch_seen: []bool,
    segment_of_node: []usize,
    rank_in_segment: []usize,
    segment_start: []usize,
    segment_len: []usize,
    segment_reversed: []bool,
    move_degree_delta: []i8,
    move_component: []usize,
    move_component_size: []usize,
    move_edges: []TourEdge,
    lk_t: []usize,
    removed_a: []usize,
    removed_b: []usize,
    added_a: []usize,
    added_b: []usize,
    lk_active: []bool,
    lk_active_queue: []usize,
    guide_next: [2][]usize,
    guide_prev: [2][]usize,
    // Roadmap item 3: per-node Misra-Gries edge vote counters, k=2 slots each.
    // vote_node[2*u + s] is the neighbour node id occupying slot s of node u
    // (maxInt = empty); vote_count is its tally. Sized 2n; zero-cost unless
    // edge freezing is enabled.
    vote_node: []usize,
    vote_count: []u32,

    fn init(allocator: std.mem.Allocator, n: usize, max_lk_depth: usize) !SolverWorkspace {
        const best_tour = try allocator.alloc(usize, n);
        errdefer allocator.free(best_tour);
        const prev_best_tour = try allocator.alloc(usize, n);
        errdefer allocator.free(prev_best_tour);
        const tour = try allocator.alloc(usize, n);
        errdefer allocator.free(tour);
        const pos = try allocator.alloc(usize, n);
        errdefer allocator.free(pos);
        const used = try allocator.alloc(bool, n);
        errdefer allocator.free(used);
        const next = try allocator.alloc(usize, n);
        errdefer allocator.free(next);
        const prev = try allocator.alloc(usize, n);
        errdefer allocator.free(prev);
        const candidate_tour = try allocator.alloc(usize, n);
        errdefer allocator.free(candidate_tour);
        const scratch_neighbor0 = try allocator.alloc(usize, n);
        errdefer allocator.free(scratch_neighbor0);
        const scratch_neighbor1 = try allocator.alloc(usize, n);
        errdefer allocator.free(scratch_neighbor1);
        const scratch_seen = try allocator.alloc(bool, n);
        errdefer allocator.free(scratch_seen);
        const segment_of_node = try allocator.alloc(usize, n);
        errdefer allocator.free(segment_of_node);
        const rank_in_segment = try allocator.alloc(usize, n);
        errdefer allocator.free(rank_in_segment);
        const segment_start = try allocator.alloc(usize, n);
        errdefer allocator.free(segment_start);
        const segment_len = try allocator.alloc(usize, n);
        errdefer allocator.free(segment_len);
        const segment_reversed = try allocator.alloc(bool, n);
        errdefer allocator.free(segment_reversed);
        const move_degree_delta = try allocator.alloc(i8, n);
        errdefer allocator.free(move_degree_delta);
        const move_component = try allocator.alloc(usize, n);
        errdefer allocator.free(move_component);
        const move_component_size = try allocator.alloc(usize, n);
        errdefer allocator.free(move_component_size);
        // Completion closes may extend a depth-d chain by up to 2 extra removed
        // edges (3-opt-style close writes index depth+1), so the move arrays are
        // sized max_lk_depth + 2 and move_edges sized for patching the longest
        // possible delta: 2 * (removed + added) + 4 slots.
        const max_move_edges = max_lk_depth + 2;
        const move_edges = try allocator.alloc(TourEdge, 4 * max_move_edges + 4);
        errdefer allocator.free(move_edges);
        const t_len = std.math.add(usize, std.math.mul(usize, 2, max_lk_depth) catch return error.OutOfMemory, 1) catch return error.OutOfMemory;
        const lk_t = try allocator.alloc(usize, t_len);
        errdefer allocator.free(lk_t);
        const removed_a = try allocator.alloc(usize, max_move_edges);
        errdefer allocator.free(removed_a);
        const removed_b = try allocator.alloc(usize, max_move_edges);
        errdefer allocator.free(removed_b);
        const added_a = try allocator.alloc(usize, max_move_edges);
        errdefer allocator.free(added_a);
        const added_b = try allocator.alloc(usize, max_move_edges);
        errdefer allocator.free(added_b);
        const lk_active = try allocator.alloc(bool, n);
        errdefer allocator.free(lk_active);
        const lk_active_queue = try allocator.alloc(usize, n);
        errdefer allocator.free(lk_active_queue);
        const guide_next_0 = try allocator.alloc(usize, n);
        errdefer allocator.free(guide_next_0);
        const guide_prev_0 = try allocator.alloc(usize, n);
        errdefer allocator.free(guide_prev_0);
        const guide_next_1 = try allocator.alloc(usize, n);
        errdefer allocator.free(guide_next_1);
        const guide_prev_1 = try allocator.alloc(usize, n);
        errdefer allocator.free(guide_prev_1);
        const vote_node = try allocator.alloc(usize, 2 * n);
        errdefer allocator.free(vote_node);
        @memset(vote_node, std.math.maxInt(usize));
        const vote_count = try allocator.alloc(u32, 2 * n);
        errdefer allocator.free(vote_count);
        @memset(vote_count, 0);

        return .{
            .allocator = allocator,
            .best_tour = best_tour,
            .prev_best_tour = prev_best_tour,
            .tour = tour,
            .pos = pos,
            .used = used,
            .next = next,
            .prev = prev,
            .candidate_tour = candidate_tour,
            .scratch_neighbor0 = scratch_neighbor0,
            .scratch_neighbor1 = scratch_neighbor1,
            .scratch_seen = scratch_seen,
            .segment_of_node = segment_of_node,
            .rank_in_segment = rank_in_segment,
            .segment_start = segment_start,
            .segment_len = segment_len,
            .segment_reversed = segment_reversed,
            .move_degree_delta = move_degree_delta,
            .move_component = move_component,
            .move_component_size = move_component_size,
            .move_edges = move_edges,
            .lk_t = lk_t,
            .removed_a = removed_a,
            .removed_b = removed_b,
            .added_a = added_a,
            .added_b = added_b,
            .lk_active = lk_active,
            .lk_active_queue = lk_active_queue,
            .guide_next = .{ guide_next_0, guide_next_1 },
            .guide_prev = .{ guide_prev_0, guide_prev_1 },
            .vote_node = vote_node,
            .vote_count = vote_count,
        };
    }

    fn deinit(self: *SolverWorkspace) void {
        self.allocator.free(self.best_tour);
        self.allocator.free(self.prev_best_tour);
        self.allocator.free(self.tour);
        self.allocator.free(self.pos);
        self.allocator.free(self.used);
        self.allocator.free(self.next);
        self.allocator.free(self.prev);
        self.allocator.free(self.candidate_tour);
        self.allocator.free(self.scratch_neighbor0);
        self.allocator.free(self.scratch_neighbor1);
        self.allocator.free(self.scratch_seen);
        self.allocator.free(self.segment_of_node);
        self.allocator.free(self.rank_in_segment);
        self.allocator.free(self.segment_start);
        self.allocator.free(self.segment_len);
        self.allocator.free(self.segment_reversed);
        self.allocator.free(self.move_degree_delta);
        self.allocator.free(self.move_component);
        self.allocator.free(self.move_component_size);
        self.allocator.free(self.move_edges);
        self.allocator.free(self.lk_t);
        self.allocator.free(self.removed_a);
        self.allocator.free(self.removed_b);
        self.allocator.free(self.added_a);
        self.allocator.free(self.added_b);
        self.allocator.free(self.lk_active);
        self.allocator.free(self.lk_active_queue);
        for (self.guide_next) |slice| self.allocator.free(slice);
        for (self.guide_prev) |slice| self.allocator.free(slice);
        self.allocator.free(self.vote_node);
        self.allocator.free(self.vote_count);
        self.* = undefined;
    }
};

pub fn solve(
    allocator: std.mem.Allocator,
    p: *const problem.Problem,
    options: SolveOptions,
) !SolveResult {
    const n = p.dimension;
    if (n <= 10) {
        const exact_result = exact.bruteForce(allocator, p, .{ .max_nodes = 10 }) catch |err| switch (err) {
            error.InstanceTooLarge => unreachable,
            else => |e| return e,
        };
        return .{
            .allocator = allocator,
            .tour = exact_result.tour,
            .length = exact_result.length,
            .stats = .{
                .trials = 1,
                .exact_permutations = exact_result.iterations,
            },
        };
    }

    const trials = @max(options.trials, 1);
    var oracle = try DistanceOracle.init(allocator, p, options.max_distance_cache_weights);
    defer oracle.deinit();

    const width = candidateWidth(n, options.candidate_count);
    var candidate_stats: CandidateBuildStats = .{};
    var candidates = try buildCandidates(allocator, &oracle, width, options.candidate_mode, options.alpha_ascent_iterations, options.alpha_nearest_patch_count, &candidate_stats);
    defer candidates.deinit();
    oracle.resetCounters();

    const min_lk_depth: usize = if (options.enable_bounded_three_opt_cleanup) 3 else 2;
    const max_lk_depth = if (options.enable_lk) @min(@max(options.lk_max_depth, min_lk_depth), n - 1) else min_lk_depth;
    var workspace = try SolverWorkspace.init(allocator, n, max_lk_depth);
    defer workspace.deinit();

    var eax = try EaxScratch.init(allocator, n);
    defer eax.deinit();
    var ipt = try IptScratch.init(allocator, n);
    defer ipt.deinit();
    var elite = try ElitePool.init(allocator, n);
    defer elite.deinit();

    var prng = std.Random.DefaultPrng.init(options.seed);
    var random = prng.random();

    var stats = SolveStats{
        .trials = trials,
        .candidate_count = width,
        .alpha_ascent_iterations = candidate_stats.iterations,
        .alpha_ascent_best_lower_bound = candidate_stats.best_lower_bound,
        .candidate_nearest_edges = candidate_stats.nearest_edges,
        .candidate_alpha_edges = candidate_stats.alpha_edges,
        .candidate_geometric_edges = candidate_stats.geometric_edges,
        .candidate_patch_edges = candidate_stats.patched_edges,
        .distance_cache_nodes = if (oracle.isCached()) n else 0,
        .distance_cache_weights = oracle.matrix.len,
    };
    var best_len: u64 = std.math.maxInt(u64);

    // Iterated local search state: consecutive kicked trials without improvement
    // first escalate the perturbation strength (more simultaneous double
    // bridges), and when even that stagnates, fall back to a cold restart with
    // exponentially backed-off frequency so dimension-scale trial counts spend
    // almost all their budget on cheap kicks.
    var stale_kicks: usize = 0;
    var restart_threshold: usize = 4;
    // Very large instances spend more per descent, so guided recombination
    // material is rationed at half the rate to keep the kick budget intact.
    const guided_restart_cadence: usize = if (n >= 800) 8 else 4;
    const guided_full_descent_below: usize = 300;
    // With the backtracking discipline, guided light descents are affordable
    // at every size; the old n < 512 gate predates it (light repairs used to
    // explode on full-width search trees).
    const guided_max_dimension: usize = std.math.maxInt(usize);
    // Merger size gate (measured 2026-06-12, rounds 11-12): below the gate
    // the tuned kick/guided ILS dynamics dominate and any merger more eager
    // than IPT degrades the pinned-seed scoreboard and run time (four EAX
    // variants measured; each reshuffles knife-edge optima and lengthens
    // runs through win-driven window re-arms). At and above the gate the
    // kick-only regime starves for recombination material and EAX is
    // strictly stronger (fl1577 22254 beats LKH's 22262; IPT never went
    // below 22262). The elite-pool build is expected to replace both
    // mergers with one structure (consolidation rule).
    const eax_min_dimension: usize = 1000;
    var kick_touched: [4][6]usize = undefined;
    var kick_count: usize = 0;
    var plateau_touched: [24]usize = undefined;
    var plateau_count: usize = 0;
    // Or-opt drift only at the EAX sizes: it adds a second plateau-move
    // shape (zero-delta segment relocation — the measured residual sections
    // on pr1002-class geometries are size <= 2, i.e. relocations, which
    // 2-opt reversal drift cannot express). Gated so the sub-1000 rows stay
    // bit-identical to the tuned round-10 trajectories.
    const plateau_or_opt = n >= eax_min_dimension;
    const plateau_stride: usize = if (plateau_or_opt) 6 else 4;
    // Exhaustive backtracking is affordable and measurably needed below 400
    // nodes; extension-phase trials are stale grinding, so they always use
    // the cheap discipline.
    const base_backtrack_depth: usize = options.lk_backtrack_depth orelse
        (if (n < 400) std.math.maxInt(usize) else @as(usize, 2));
    // Shadow incumbent for EAX tour merging: best tour ever produced by a
    // merge (+ polish). Kept out of the kick/restart loop so the baseline
    // trajectory is undisturbed; folded into the result after the loop.
    var merged_len: u64 = std.math.maxInt(u64);
    // Progress events: main-incumbent improvement OR shadow (merged_len)
    // improvement. The adaptive stop below keys off the gap between them.
    var last_progress_trial: usize = 0;
    var max_progress_gap: usize = 0;
    // Previous distinct incumbent: the second reference tour for guided
    // construction (LKH's InNextBestTour). The contested sections between it
    // and the current best are exactly where constructions should explore.
    var prev_best_len: u64 = std.math.maxInt(u64);
    const max_trials = if (options.trial_extension_factor > 1)
        std.math.mul(usize, trials, options.trial_extension_factor) catch trials
    else
        trials;
    // No adaptive convergence stop: factor-8 progress-gap patience (stop when
    // quiet 8x longer than the run's longest productive quiet, floor 64) was
    // measured and REJECTED — at the pinned seed it cut a280/fl417/ts225
    // time 35-70% with identical lengths, but across 6 seeds it cost rat195
    // 2 of 4 winning seeds and fl417 3 of 5 with zero rows improving.
    // Improvement gaps are heavy-tailed (worst must-survive gap = 11x the
    // prior max); the n-trial window is the insurance premium for expected
    // accuracy until the generator/combiner finds optima earlier.
    var last_improvement_trial: usize = 0;
    // Roadmap item 3: how many gated trials have voted so far. The freeze
    // threshold is relative to this, so freezing ramps up as evidence
    // accumulates rather than locking onto an early incumbent.
    var votes_cast: u32 = 0;
    var trial: usize = 0;
    while (trial < max_trials and trial - last_improvement_trial < trials) : (trial += 1) {
        // After the first descent, trials are iterated local search: perturb the
        // best tour and let LK re-optimize only the perturbed neighborhood,
        // instead of paying for a cold construction + full descent every trial.
        // Guided restarts are cheap and feed the merger, so they run on a
        // fixed cadence; only cold restarts (no incumbent to guide from)
        // stay exponentially backed off. Above the big-instance boundary
        // (cf. the bridge gate) guided trials don't pay: the light repair is
        // too weak to escape the incumbent basin and the full descent too
        // expensive, so those instances keep the kick/cold-restart schedule.
        const guided_available = options.enable_lk and trial > 0 and
            best_len != std.math.maxInt(u64) and n < guided_max_dimension;
        const restart_limit = if (guided_available) guided_restart_cadence else restart_threshold;
        const kick_trial = options.enable_lk and trial > 0 and n >= 8 and
            best_len != std.math.maxInt(u64) and stale_kicks < restart_limit;
        var guided_trial = false;
        // Roadmap item 3: this trial's freeze threshold (maxInt = nothing frozen
        // yet). Relative to votes cast, gated below a minimum so early trials
        // never freeze.
        // Item-3 revival: an injected static backbone freezes from trial 0,
        // independent of the voted threshold (which it sets to maxInt so only
        // the injected set fires).
        const inject_active = options.enable_edge_freeze and options.inject_frozen.len > 0;
        // Staleness gate: only act on the frozen set once the incumbent has been
        // stuck long enough (voting keeps accumulating regardless, below).
        const stale_ok = options.edge_freeze_stale_window == 0 or
            (trial - last_improvement_trial >= options.edge_freeze_stale_window);
        const freeze_threshold: u32 = if (options.enable_edge_freeze and !inject_active and stale_ok and votes_cast >= options.edge_freeze_min_votes)
            @max(1, votes_cast * options.edge_freeze_fraction_x100 / 100)
        else
            std.math.maxInt(u32);
        const freeze_active = (inject_active and stale_ok) or freeze_threshold != std.math.maxInt(u32);
        if (kick_trial) {
            @memcpy(workspace.tour, workspace.best_tour);
            kick_count = @min(1 + stale_kicks / 4, kick_touched.len);
            for (0..kick_count) |ki| {
                if (freeze_active) {
                    segmentExchangeKickAvoidingFrozen(workspace.tour, &random, &kick_touched[ki], workspace.vote_node, workspace.vote_count, freeze_threshold, options.inject_frozen);
                } else {
                    segmentExchangeKick(workspace.tour, &random, &kick_touched[ki]);
                }
            }
            // Extension-phase kicks add plateau drift: the base budget has
            // stalled, so the remaining gap is most likely hiding behind
            // cost-equal reconnections that bridges alone revisit forever.
            // (Arming drift during the base phase at EAX sizes was measured
            // and rejected: pinned-seed pr1002 259706 looked great, but
            // across seeds both big rows got strictly worse — fl1577 22264
            // -> 22537 at seed 7. Same heavy-tail lesson as the stopping
            // rules: base-budget trajectories stay untouched.)
            plateau_count = if (trial >= trials)
                plateauKick(&oracle, &candidates, workspace.tour, workspace.pos, &random, 4, plateau_or_opt, &plateau_touched)
            else
                0;
        } else {
            if (trial > 0 and stale_kicks >= restart_limit) {
                if (!guided_available) restart_threshold *= 2;
                stale_kicks = 0;
            }
            if (guided_available) {
                // Restarts are LKH-style guided constructions seeded by the
                // incumbent backbone, with the previous distinct incumbent
                // as the second reference (cf. InBestTour/InNextBestTour).
                // The second reference is only sound where the full descent
                // runs: the light-repair path assumes reference edges are
                // LK-converged, which holds for the current best but not for
                // a previous basin's tour.
                const full_descent = n < guided_full_descent_below;
                const next_ref: ?[]const usize = if (full_descent and prev_best_len != std.math.maxInt(u64))
                    workspace.prev_best_tour
                else
                    null;
                guided_trial = true;
                // Below the light-descent size cutoff the construction is
                // the faithful LKH ladder (unbounded divergence) because the
                // descent is a full one; above it the divergence budget
                // keeps the light repair localized.
                const max_divergence: usize = if (full_descent)
                    std.math.maxInt(usize)
                else
                    12;
                guidedBackboneTour(&oracle, &candidates, .{ workspace.best_tour, next_ref }, workspace.guide_next, workspace.guide_prev, max_divergence, &random, workspace.tour, workspace.used);
            } else if (trial % 4 == 1 and n >= 300) {
                farthestInsertionTour(&oracle, workspace.tour, workspace.candidate_tour, workspace.used);
            } else {
                nearestNeighborTour(&oracle, &candidates, &random, trial, options.randomized_starts, workspace.tour, workspace.used);
                if (trial > 0 and n >= 8) {
                    segmentExchangeKick(workspace.tour, &random, &kick_touched[0]);
                }
            }
        }

        var search = LocalSearch{
            .dist = &oracle,
            .stats = &stats,
            .candidates = &candidates,
            .tour = workspace.tour,
            .pos = workspace.pos,
            .next = workspace.next,
            .prev = workspace.prev,
            .candidate_tour = workspace.candidate_tour,
            .scratch_neighbor0 = workspace.scratch_neighbor0,
            .scratch_neighbor1 = workspace.scratch_neighbor1,
            .scratch_seen = workspace.scratch_seen,
            .segment_of_node = workspace.segment_of_node,
            .rank_in_segment = workspace.rank_in_segment,
            .segment_start = workspace.segment_start,
            .segment_len = workspace.segment_len,
            .segment_reversed = workspace.segment_reversed,
            .move_degree_delta = workspace.move_degree_delta,
            .move_component = workspace.move_component,
            .move_component_size = workspace.move_component_size,
            .move_edges = workspace.move_edges,
            .lk_t = workspace.lk_t,
            .removed_a = workspace.removed_a,
            .removed_b = workspace.removed_b,
            .added_a = workspace.added_a,
            .added_b = workspace.added_b,
            .lk_active = workspace.lk_active,
            .lk_active_queue = workspace.lk_active_queue,
            .max_passes = options.max_passes,
            .enable_or_opt = options.enable_or_opt,
            .enable_bounded_three_opt_cleanup = options.enable_bounded_three_opt_cleanup,
            .enable_move_patching = options.enable_move_patching,
            .move_patch_min_gain = options.move_patch_min_gain,
            .lk_completion_patch_min_gain = options.lk_completion_patch_min_gain,
            .max_lk_depth = max_lk_depth,
            .lk_backtrack_limit = options.lk_backtrack_limit,
            .lk_backtrack_depth = if (trial >= trials) @min(base_backtrack_depth, 2) else base_backtrack_depth,
            .lk_nonseq_branch_limit = options.lk_nonseq_branch_limit,
            // Roadmap item 3: the generator honours frozen edges. The combiner's
            // polish search (merge_search) clears respect_frozen below.
            .vote_node = workspace.vote_node,
            .vote_count = workspace.vote_count,
            .freeze_threshold = freeze_threshold,
            .inject_frozen = options.inject_frozen,
            .respect_frozen = options.enable_edge_freeze and options.edge_freeze_lk_respect,
            .soft_freeze = options.enable_edge_freeze and options.edge_freeze_soft and freeze_active,
        };
        search.rebuildState();
        if (kick_trial) {
            search.lkResetActive();
            for (kick_touched[0..kick_count]) |touched| {
                for (touched) |node| search.lkActivate(node);
            }
            for (plateau_touched[0 .. plateau_stride * plateau_count]) |node| search.lkActivate(node);
            try search.syncLength();
            const lk_moves = try search.improveLK(&stats, false, false);
            stats.improving_moves += lk_moves;
            // Only tours that beat the incumbent earn the expensive fallback
            // sweeps (Gain23 bridge / 4-opt / bounded 3-opt polish).
            if (search.current_length < best_len) {
                const polish_moves = try search.improveLK(&stats, false, true);
                stats.improving_moves += polish_moves;
            }
        } else if (guided_trial) {
            stats.guided_trials += 1;
            const nodes_before = stats.lk_search_nodes;
            if (n < guided_full_descent_below) {
                // Small instances re-converge straight back into the
                // incumbent's basin under a light repair (zero merge wins on
                // rat195); a full descent from the guided tour lands in
                // genuinely different local optima and is affordable here.
                const warmup_moves = try search.improveWarmup();
                stats.warmup_moves += warmup_moves;
                stats.improving_moves += warmup_moves;
                search.rebuildState();
                try search.syncLength();
                const lk_moves = try search.improveLK(&stats, true, true);
                stats.improving_moves += lk_moves;
            } else {
                // Guided tours are reference material everywhere except the
                // off-backbone edges flagged by the construction; reactivate
                // only those neighborhoods, polish only on improvement (same
                // pattern as the kick path).
                search.lkResetActive();
                for (workspace.used, 0..) |touched, node| {
                    if (touched) search.lkActivate(node);
                }
                try search.syncLength();
                const lk_moves = try search.improveLK(&stats, false, false);
                stats.improving_moves += lk_moves;
                if (search.current_length < best_len) {
                    stats.guided_polishes += 1;
                    const polish_moves = try search.improveLK(&stats, false, true);
                    stats.improving_moves += polish_moves;
                }
            }
            stats.guided_search_nodes += stats.lk_search_nodes - nodes_before;
        } else {
            const warmup_moves = try search.improveWarmup();
            stats.warmup_moves += warmup_moves;
            stats.improving_moves += warmup_moves;
            search.rebuildState();
            try search.syncLength();
            if (options.enable_lk) {
                const lk_moves = try search.improveLK(&stats, true, true);
                stats.improving_moves += lk_moves;
            }
        }

        // IPT tour merging (LKH's MergeWithTour): recombine the trial tour
        // with the merge incumbent; every independent differing section
        // resolves to its shorter alternative, so the merge can beat both
        // parents even when the trial itself did not. The merge product is
        // accumulated in a shadow incumbent (ipt.merged) and folded in after
        // the trial loop: the kick/restart trajectory stays bit-for-bit
        // identical to the merge-free search, so merge gains are pure upside.
        // Gated to trials within ~3% of the incumbent so hopeless tours don't
        // pay the scan.
        if (n < eax_min_dimension and options.enable_lk and best_len != std.math.maxInt(u64)) {
            const use_merged = merged_len < best_len;
            const ref_tour: []const usize = if (use_merged) ipt.merged else workspace.best_tour;
            const ref_len = if (use_merged) merged_len else best_len;
            const trial_len = search.current_length;
            if (trial_len <= ref_len + ref_len / 32) {
                stats.eax_merge_attempts += 1;
                @memcpy(ipt.tour_a, workspace.tour);
                if (iptMergeTours(&oracle, ipt.tour_a, trial_len, ref_tour, ref_len, &ipt)) |outcome| {
                    stats.eax_merge_cycles += outcome.transcriptions;
                    if (outcome.length < ref_len) {
                        stats.eax_merge_wins += 1;
                        const merge_nodes_before = stats.lk_search_nodes;
                        if (!outcome.winner_is_a) @memcpy(ipt.tour_a, ipt.tour_b);
                        // Re-optimize only the neighborhoods around the
                        // transcribed section boundaries, mirroring the kick
                        // path's light-descent-then-polish pattern. LK is
                        // deterministic (no RNG), so polishing the shadow
                        // tour cannot perturb the main trajectory.
                        var merge_search = search;
                        merge_search.tour = ipt.tour_a;
                        // Roadmap item 3: the combiner cuts through frozen edges
                        // (joint section moves are the measured merge mechanism).
                        merge_search.respect_frozen = false;
                        merge_search.soft_freeze = false;
                        merge_search.rebuildState();
                        merge_search.lkResetActive();
                        for (ipt.boundary[0..outcome.boundary_count]) |node| merge_search.lkActivate(node);
                        const merge_moves = try merge_search.improveLK(&stats, false, false);
                        stats.improving_moves += merge_moves;
                        const polish_moves = try merge_search.improveLK(&stats, false, true);
                        stats.improving_moves += polish_moves;
                        const merged_now = try oracle.tourLengthUnchecked(ipt.tour_a);
                        if (merged_now < merged_len) {
                            merged_len = merged_now;
                            @memcpy(ipt.merged, ipt.tour_a);
                        }
                        // Adopt the merge product as the main incumbent
                        // (LKH keeps the merged tour as BetterTour): kicks
                        // and guided constructions re-base onto it, so
                        // recombination gains compound instead of sitting in
                        // the shadow until the end. Adoption follows the
                        // guided-trial size gate: on big kick-only instances
                        // the shadow accumulator measured strictly better
                        // (kick trajectories are hypersensitive to incumbent
                        // swaps), so they keep it.
                        if (n < guided_max_dimension and merged_now < best_len) {
                            prev_best_len = best_len;
                            @memcpy(workspace.prev_best_tour, workspace.best_tour);
                            best_len = merged_now;
                            stats.best_trial = trial;
                            last_improvement_trial = trial;
                            @memcpy(workspace.best_tour, ipt.tour_a);
                            stale_kicks = 0;
                        }
                        stats.merge_search_nodes += stats.lk_search_nodes - merge_nodes_before;
                    }
                }
            }
        }

        // EAX-lite tour merging: recombine the trial tour with the merge
        // incumbent by applying improving AB-cycles from their symmetric
        // difference. Subsumes the IPT section transcription it replaced (a
        // contiguous section swap is one non-splitting AB-cycle) and
        // additionally moves interleaved differing bundles atomically, so the
        // merge can beat both parents even when the trial itself did not. The
        // merge product is accumulated in a shadow incumbent (eax.merged) and
        // folded in after the trial loop: the kick/restart trajectory stays
        // bit-for-bit identical to the merge-free search, so merge gains are
        // pure upside. Gated to trials within ~3% of the incumbent so
        // hopeless tours don't pay the scan.
        if (n >= eax_min_dimension and options.enable_lk and best_len != std.math.maxInt(u64)) {
            const trial_len = search.current_length;
            // References come from the elite pool: a small population of
            // diverse high-quality tours, each one a structurally different
            // parent whose symmetric difference against the trial exposes
            // different AB-cycles. A win must beat the global standard
            // min(best, merged), not just its own reference. The member
            // count is snapshotted because wins offer their polished
            // products back into the pool mid-loop; lens/tours are read per
            // iteration so a replaced slot stays a consistent pair.
            const member_count = elite.count;
            for (0..member_count) |pi| {
                const ref_tour: []const usize = elite.tours[pi];
                const ref_len = elite.lens[pi];
                if (trial_len > ref_len + ref_len / 32) continue;
                const standard = @min(best_len, merged_len);
                stats.eax_merge_attempts += 1;
                @memcpy(eax.tour_a, workspace.tour);
                {
                    const outcome = eaxMergeTours(&oracle, &candidates, eax.tour_a, trial_len, ref_tour, ref_len, true, &eax);
                    stats.eax_merge_cycles += outcome.cycles_applied;
                    if (outcome.cycles_applied > 0 and outcome.length < standard) {
                        stats.eax_merge_wins += 1;
                        const merge_nodes_before = stats.lk_search_nodes;
                        if (!outcome.winner_is_a) @memcpy(eax.tour_a, eax.tour_b);
                        // Re-optimize only the neighborhoods around the
                        // changed edges, mirroring the kick path's
                        // light-descent-then-polish pattern. LK is
                        // deterministic (no RNG), so polishing the shadow
                        // tour cannot perturb the main trajectory.
                        var merge_search = search;
                        merge_search.tour = eax.tour_a;
                        // Roadmap item 3: the combiner cuts through frozen edges
                        // (joint section moves are the measured merge mechanism).
                        merge_search.respect_frozen = false;
                        merge_search.soft_freeze = false;
                        merge_search.rebuildState();
                        merge_search.lkResetActive();
                        for (eax.boundary[0..outcome.boundary_count]) |node| merge_search.lkActivate(node);
                        const merge_moves = try merge_search.improveLK(&stats, false, false);
                        stats.improving_moves += merge_moves;
                        const polish_moves = try merge_search.improveLK(&stats, false, true);
                        stats.improving_moves += polish_moves;
                        const merged_now = try oracle.tourLengthUnchecked(eax.tour_a);
                        if (merged_now < merged_len) {
                            merged_len = merged_now;
                            @memcpy(eax.merged, eax.tour_a);
                            const gap = trial - last_progress_trial;
                            stats.eax_worst_gap_ratio_x100 = @max(stats.eax_worst_gap_ratio_x100, gap * 100 / @max(max_progress_gap, 32));
                            max_progress_gap = @max(max_progress_gap, gap);
                            last_progress_trial = trial;
                        }
                        elitePoolOffer(&elite, &eax, eax.tour_a, merged_now);
                        // Adopt the merge product as the main incumbent
                        // (LKH keeps the merged tour as BetterTour): kicks
                        // and guided constructions re-base onto it, so
                        // recombination gains compound instead of sitting in
                        // the shadow until the end. Both halves are measured
                        // load-bearing for EAX exactly as they were for IPT:
                        // dropping adoption loses lin318/rd400/u574/rat575
                        // outright, and keeping adoption without the
                        // staleness resets loses lin318/rd400/pcb442/u574 —
                        // extension-dependent rows need merge wins to re-arm
                        // the stagnation window.
                        if (n < guided_max_dimension and merged_now < best_len) {
                            prev_best_len = best_len;
                            @memcpy(workspace.prev_best_tour, workspace.best_tour);
                            best_len = merged_now;
                            stats.best_trial = trial;
                            last_improvement_trial = trial;
                            @memcpy(workspace.best_tour, eax.tour_a);
                            stale_kicks = 0;
                        }
                        stats.merge_search_nodes += stats.lk_search_nodes - merge_nodes_before;
                    }
                }
            }
            // Every LK-converged trial is pool material: equal-length
            // plateau siblings and near-elite basins are exactly what the
            // merger recombines profitably; replace-worst keeps it elite.
            elitePoolOffer(&elite, &eax, workspace.tour, trial_len);
        }

        // Roadmap item 3: a near-incumbent (merge-gated) trial votes its edges
        // into the Misra-Gries counters. Same ~3% gate the mergers use, so only
        // genuinely good tours shape the frozen set. Voting happens before the
        // incumbent update so the gate compares against this trial's reference.
        if (options.enable_edge_freeze and options.edge_freeze_vote_mode == .gated_trials and
            best_len != std.math.maxInt(u64) and search.current_length <= best_len + best_len / 32)
        {
            voteTourEdges(workspace.vote_node, workspace.vote_count, workspace.tour, &stats.freeze_decrements);
            votes_cast +|= 1;
        }

        // Roadmap item 2: read the delta-maintained length instead of rescanning.
        // Debug builds verify it against a fresh scan so any missed/incorrect
        // move delta surfaces as a test failure rather than a silent drift.
        if (std.debug.runtime_safety) {
            const scanned = try oracle.tourLengthUnchecked(workspace.tour);
            std.debug.assert(search.current_length == scanned);
        }
        const len = search.current_length;
        if (len < best_len) {
            if (best_len != std.math.maxInt(u64)) {
                prev_best_len = best_len;
                @memcpy(workspace.prev_best_tour, workspace.best_tour);
            }
            best_len = len;
            stats.best_trial = trial;
            last_improvement_trial = trial;
            @memcpy(workspace.best_tour, workspace.tour);
            stale_kicks = 0;
            // Roadmap item 3 (distinct-incumbent vote mode): vote only genuinely
            // adopted incumbents — a sparse, low-correlation stream whose
            // consensus is the true backbone rather than the current attractor.
            if (options.enable_edge_freeze and options.edge_freeze_vote_mode == .distinct_incumbents) {
                voteTourEdges(workspace.vote_node, workspace.vote_count, workspace.tour, &stats.freeze_decrements);
                votes_cast +|= 1;
            }
            if (n >= eax_min_dimension and options.enable_lk) elitePoolOffer(&elite, &eax, workspace.tour, len);
            const gap = trial - last_progress_trial;
            stats.eax_worst_gap_ratio_x100 = @max(stats.eax_worst_gap_ratio_x100, gap * 100 / @max(max_progress_gap, 32));
            max_progress_gap = @max(max_progress_gap, gap);
            last_progress_trial = trial;
        } else if (kick_trial) {
            stale_kicks += 1;
        }
    }
    stats.trials = trial;
    stats.eax_max_progress_gap = max_progress_gap;
    stats.eax_final_progress_gap = trial - last_progress_trial;
    // Roadmap item 3 diagnostics: snapshot how many edges ended up frozen at
    // the final threshold (each undirected edge counted once, from the lower
    // endpoint), plus the total votes cast.
    stats.freeze_votes = votes_cast;
    if (options.enable_edge_freeze and votes_cast >= options.edge_freeze_min_votes) {
        const final_threshold = @max(@as(u32, 1), votes_cast * options.edge_freeze_fraction_x100 / 100);
        var frozen: usize = 0;
        for (0..n) |u| {
            for ([2]usize{ workspace.vote_node[2 * u], workspace.vote_node[2 * u + 1] }) |v| {
                if (v != std.math.maxInt(usize) and u < v and
                    edgeIsFrozen(workspace.vote_node, workspace.vote_count, final_threshold, u, v))
                {
                    frozen += 1;
                    if (options.frozen_edges_out) |out| {
                        try out.append(allocator, @intCast(u));
                        try out.append(allocator, @intCast(v));
                    }
                }
            }
        }
        stats.frozen_edges_final = frozen;
    }

    if (merged_len < best_len) {
        best_len = merged_len;
        @memcpy(workspace.best_tour, if (n < eax_min_dimension) ipt.merged else eax.merged);
    }

    const result_tour = try allocator.dupe(usize, workspace.best_tour);
    errdefer allocator.free(result_tour);
    stats.uncached_coordinate_distances = oracle.uncached_coordinate_distances;
    stats.distance_lookups = oracle.lookups;
    stats.tour_length_scans = oracle.length_scans;
    return .{
        .allocator = allocator,
        .tour = result_tour,
        .length = best_len,
        .stats = stats,
    };
}

fn candidateWidth(n: usize, requested: usize) usize {
    std.debug.assert(n > 2);
    return @min(@max(requested, 2), n - 1);
}

pub fn buildCandidates(
    allocator: std.mem.Allocator,
    dist_oracle: *DistanceOracle,
    width: usize,
    mode: CandidateMode,
    alpha_ascent_iterations: usize,
    alpha_nearest_patch_count: usize,
    candidate_stats: *CandidateBuildStats,
) !Candidates {
    return switch (mode) {
        .nearest_distance => buildNearestCandidates(allocator, dist_oracle, width, candidate_stats),
        .alpha_nearness => buildAlphaCandidates(allocator, dist_oracle, width, alpha_ascent_iterations, alpha_nearest_patch_count, candidate_stats),
        .alpha_nearness_cgal => buildAlphaCgalCandidates(allocator, dist_oracle, width, alpha_ascent_iterations, alpha_nearest_patch_count, candidate_stats),
    };
}

fn buildNearestCandidates(allocator: std.mem.Allocator, dist_oracle: *DistanceOracle, width: usize, candidate_stats: *CandidateBuildStats) !Candidates {
    const n = dist_oracle.p.dimension;
    const total_candidates = std.math.mul(usize, n, width) catch return error.OutOfMemory;
    var data = try allocator.alloc(usize, total_candidates);
    errdefer allocator.free(data);
    var alpha = try allocator.alloc(u64, total_candidates);
    errdefer allocator.free(alpha);
    var dist = try allocator.alloc(u64, width);
    defer allocator.free(dist);

    for (0..n) |i| {
        @memset(dist, std.math.maxInt(u64));
        const row = data[i * width .. i * width + width];
        const alpha_row = alpha[i * width .. i * width + width];
        @memset(row, std.math.maxInt(usize));
        @memset(alpha_row, std.math.maxInt(u64));

        for (0..n) |j| {
            if (i == j) continue;
            const d = @as(u64, dist_oracle.distance(i, j));
            var slot: ?usize = null;
            for (0..width) |k| {
                if (d < dist[k] or (d == dist[k] and j < row[k])) {
                    slot = k;
                    break;
                }
            }
            if (slot) |k| {
                if (k + 1 < width) {
                    std.mem.copyBackwards(u64, dist[k + 1 ..], dist[k .. width - 1]);
                    std.mem.copyBackwards(usize, row[k + 1 ..], row[k .. width - 1]);
                    std.mem.copyBackwards(u64, alpha_row[k + 1 ..], alpha_row[k .. width - 1]);
                }
                dist[k] = d;
                row[k] = j;
                alpha_row[k] = d;
            }
        }

        validateCandidateRow(i, row);
    }

    candidate_stats.nearest_edges += @as(u64, @intCast(n * width));
    return .{ .allocator = allocator, .width = width, .data = data, .alpha = alpha };
}

fn buildAlphaCandidates(
    allocator: std.mem.Allocator,
    dist_oracle: *DistanceOracle,
    width: usize,
    ascent_iterations: usize,
    nearest_patch_count: usize,
    candidate_stats: *CandidateBuildStats,
) !Candidates {
    const n = dist_oracle.p.dimension;
    const total_candidates = std.math.mul(usize, n, width) catch return error.OutOfMemory;
    var data = try allocator.alloc(usize, total_candidates);
    errdefer allocator.free(data);
    var alpha = try allocator.alloc(u64, total_candidates);
    errdefer allocator.free(alpha);
    var row_dist = try allocator.alloc(u64, width);
    defer allocator.free(row_dist);

    const parent = try allocator.alloc(usize, n);
    defer allocator.free(parent);
    const mst_edge = try allocator.alloc(i64, n);
    defer allocator.free(mst_edge);
    const in_tree = try allocator.alloc(bool, n);
    defer allocator.free(in_tree);
    const degree = try allocator.alloc(i32, n);
    defer allocator.free(degree);
    const pi = try allocator.alloc(i64, n);
    defer allocator.free(pi);
    const best_pi = try allocator.alloc(i64, n);
    defer allocator.free(best_pi);
    const best_parent = try allocator.alloc(usize, n);
    defer allocator.free(best_parent);
    const best_mst_edge = try allocator.alloc(i64, n);
    defer allocator.free(best_mst_edge);
    const last_degree = try allocator.alloc(i32, n);
    defer allocator.free(last_degree);
    const nearest_patch = try allocator.alloc(usize, @min(@min(nearest_patch_count, width), 8));
    defer allocator.free(nearest_patch);
    var root_edges: [2]usize = undefined;
    var best_root_edges: [2]usize = undefined;

    runAlphaAscent(
        dist_oracle,
        @max(ascent_iterations, 1),
        parent,
        mst_edge,
        in_tree,
        degree,
        pi,
        best_pi,
        best_parent,
        best_mst_edge,
        last_degree,
        &root_edges,
        &best_root_edges,
        candidate_stats,
    );

    // O(n^2)-total alpha computation: one tree traversal per row yields the
    // 1-tree path bottleneck (Helsgaun's beta) to every other node, instead
    // of an O(depth^2) ancestor walk per pair — which degenerates to O(n^4)
    // total on chain-shaped MSTs (36 s of the 38 s candidate build on
    // fl1577). The MST spans nodes 1..n-1; node 0 attaches via root_edges
    // and uses the second-cheapest 0-edge as its alpha reference.
    const edge_slots = if (n >= 3) 2 * (n - 2) else 0;
    const adj_start = try allocator.alloc(usize, n + 1);
    defer allocator.free(adj_start);
    const adj_node = try allocator.alloc(usize, edge_slots);
    defer allocator.free(adj_node);
    const adj_weight = try allocator.alloc(i64, edge_slots);
    defer allocator.free(adj_weight);
    const bottleneck = try allocator.alloc(i64, n);
    defer allocator.free(bottleneck);
    const bfs_queue = try allocator.alloc(usize, n);
    defer allocator.free(bfs_queue);

    @memset(adj_start, 0);
    for (2..n) |node| {
        adj_start[node + 1] += 1;
        adj_start[best_parent[node] + 1] += 1;
    }
    for (1..n + 1) |k| adj_start[k] += adj_start[k - 1];
    @memcpy(bfs_queue, adj_start[0..n]);
    for (2..n) |node| {
        const dad = best_parent[node];
        const weight = best_mst_edge[node];
        adj_node[bfs_queue[node]] = dad;
        adj_weight[bfs_queue[node]] = weight;
        bfs_queue[node] += 1;
        adj_node[bfs_queue[dad]] = node;
        adj_weight[bfs_queue[dad]] = weight;
        bfs_queue[dad] += 1;
    }

    var second_root_cost: i64 = std.math.maxInt(i64);
    {
        var first_root_cost: i64 = std.math.maxInt(i64);
        for (1..n) |node| {
            const cost = adjustedCost(dist_oracle, best_pi, 0, node);
            if (cost < first_root_cost) {
                second_root_cost = first_root_cost;
                first_root_cost = cost;
            } else if (cost < second_root_cost) {
                second_root_cost = cost;
            }
        }
    }

    for (0..n) |i| {
        if (i != 0) fillTreeBottleneck(i, adj_start, adj_node, adj_weight, in_tree, bfs_queue, bottleneck);
        @memset(row_dist, std.math.maxInt(u64));
        const row = data[i * width .. i * width + width];
        const alpha_row = alpha[i * width .. i * width + width];
        @memset(row, std.math.maxInt(usize));
        @memset(alpha_row, std.math.maxInt(u64));

        for (0..n) |j| {
            if (i == j) continue;
            const d = @as(u64, dist_oracle.distance(i, j));
            const a = rowAlphaScore(dist_oracle, i, j, best_pi, best_parent, best_root_edges, second_root_cost, bottleneck);
            var slot: ?usize = null;
            for (0..width) |k| {
                if (a < alpha_row[k] or
                    (a == alpha_row[k] and d < row_dist[k]) or
                    (a == alpha_row[k] and d == row_dist[k] and j < row[k]))
                {
                    slot = k;
                    break;
                }
            }
            if (slot) |k| {
                if (k + 1 < width) {
                    std.mem.copyBackwards(u64, alpha_row[k + 1 ..], alpha_row[k .. width - 1]);
                    std.mem.copyBackwards(u64, row_dist[k + 1 ..], row_dist[k .. width - 1]);
                    std.mem.copyBackwards(usize, row[k + 1 ..], row[k .. width - 1]);
                }
                alpha_row[k] = a;
                row_dist[k] = d;
                row[k] = j;
            }
        }

        const patch_count = buildNearestPatch(dist_oracle, i, nearest_patch);
        for (nearest_patch[0..patch_count]) |patch_node| {
            if (rowContains(row, patch_node)) continue;
            const d = @as(u64, dist_oracle.distance(i, patch_node));
            const a = rowAlphaScore(dist_oracle, i, patch_node, best_pi, best_parent, best_root_edges, second_root_cost, bottleneck);
            if (!candidateLess(a, d, patch_node, alpha_row[width - 1], row_dist[width - 1], row[width - 1])) continue;
            row[width - 1] = patch_node;
            alpha_row[width - 1] = a;
            row_dist[width - 1] = d;
            sortCandidateRow(row, alpha_row, row_dist);
            candidate_stats.patched_edges += 1;
        }

        validateCandidateRow(i, row);
    }

    candidate_stats.patched_edges += symmetrizeCandidateRows(dist_oracle, data, alpha, width);

    const total_edges: u64 = @intCast(n * width);
    candidate_stats.alpha_edges += total_edges - candidate_stats.patched_edges;
    candidate_stats.nearest_edges += candidate_stats.patched_edges;
    return .{ .allocator = allocator, .width = width, .data = data, .alpha = alpha };
}

fn buildAlphaCgalCandidates(
    allocator: std.mem.Allocator,
    dist_oracle: *DistanceOracle,
    width: usize,
    ascent_iterations: usize,
    nearest_patch_count: usize,
    candidate_stats: *CandidateBuildStats,
) !Candidates {
    if (!build_options.with_cgal) return error.CgalUnavailable;

    var candidates = try buildAlphaCandidates(allocator, dist_oracle, width, ascent_iterations, nearest_patch_count, candidate_stats);
    errdefer candidates.deinit();

    if (dist_oracle.p.coords.len == 0) return candidates;

    const inserted = try applyCgalDelaunayPatch(allocator, dist_oracle, &candidates);
    candidate_stats.geometric_edges += inserted;
    candidate_stats.alpha_edges -|= inserted;
    return candidates;
}

fn symmetrizeCandidateRows(
    dist_oracle: *DistanceOracle,
    data: []usize,
    alpha: []u64,
    width: usize,
) u64 {
    const n = dist_oracle.p.dimension;
    if (width > 64) return 0;
    var inserted: u64 = 0;

    var i: usize = 0;
    while (i < n) : (i += 1) {
        const row = data[i * width .. i * width + width];
        const alpha_row = alpha[i * width .. i * width + width];
        for (row, 0..) |j, k| {
            if (j >= n) continue;
            const reverse_row = data[j * width .. j * width + width];
            if (rowContains(reverse_row, i)) continue;
            const reverse_alpha = alpha[j * width .. j * width + width];
            const a = alpha_row[k];
            const d = dist_oracle.distance(j, i);
            const worst = width - 1;
            const worst_dist = dist_oracle.distance(j, reverse_row[worst]);
            if (!candidateLess(a, d, i, reverse_alpha[worst], worst_dist, reverse_row[worst])) continue;
            reverse_row[worst] = i;
            reverse_alpha[worst] = a;
            var reverse_dist: [64]u64 = undefined;
            for (reverse_row, 0..) |entry, idx| reverse_dist[idx] = dist_oracle.distance(j, entry);
            sortCandidateRow(reverse_row, reverse_alpha, reverse_dist[0..width]);
            validateCandidateRow(j, reverse_row);
            inserted += 1;
        }
    }

    return inserted;
}

fn applyCgalDelaunayPatch(
    allocator: std.mem.Allocator,
    dist_oracle: *DistanceOracle,
    candidates: *Candidates,
) !u64 {
    const n = dist_oracle.p.dimension;
    const max_edges = @max(3 * n, 1);

    const xy = try allocator.alloc(f64, 2 * n);
    defer allocator.free(xy);
    for (dist_oracle.p.coords, 0..) |coord, i| {
        xy[2 * i] = coord.x;
        xy[2 * i + 1] = coord.y;
    }

    const edge_pairs = try allocator.alloc(u32, 2 * max_edges);
    defer allocator.free(edge_pairs);
    const edge_count = commiv_cgal_delaunay_edges(xy.ptr, n, edge_pairs.ptr, max_edges);
    if (edge_count == std.math.maxInt(usize)) return error.CgalDelaunayFailed;
    if (edge_count > max_edges) return error.CgalDelaunayEdgeBufferTooSmall;

    const row_dist = try allocator.alloc(u64, candidates.width);
    defer allocator.free(row_dist);

    var inserted: u64 = 0;
    for (0..edge_count) |edge_idx| {
        const a: usize = edge_pairs[2 * edge_idx];
        const b: usize = edge_pairs[2 * edge_idx + 1];
        if (a >= n or b >= n or a == b) return error.InvalidCgalDelaunayEdge;
        if (insertGeometricCandidate(dist_oracle, candidates, row_dist, a, b)) inserted += 1;
        if (insertGeometricCandidate(dist_oracle, candidates, row_dist, b, a)) inserted += 1;
    }
    return inserted;
}

fn insertGeometricCandidate(
    dist_oracle: *DistanceOracle,
    candidates: *Candidates,
    row_dist: []u64,
    node: usize,
    candidate: usize,
) bool {
    const row = candidates.data[node * candidates.width .. node * candidates.width + candidates.width];
    const alpha_row = candidates.alpha[node * candidates.width .. node * candidates.width + candidates.width];
    if (rowContains(row, candidate)) return false;
    for (row, 0..) |entry, i| row_dist[i] = dist_oracle.distance(node, entry);

    const weakest = candidates.width - 1;
    if (alpha_row[weakest] == 0) return false;
    const dist = @as(u64, dist_oracle.distance(node, candidate));
    if (dist >= row_dist[weakest]) return false;

    row[weakest] = candidate;
    alpha_row[weakest] = alpha_row[weakest] +| 1;
    row_dist[weakest] = dist;
    sortCandidateRow(row, alpha_row, row_dist);
    validateCandidateRow(node, row);
    return true;
}

fn runAlphaAscent(
    dist_oracle: *DistanceOracle,
    max_iterations: usize,
    parent: []usize,
    mst_edge: []i64,
    in_tree: []bool,
    degree: []i32,
    pi: []i64,
    best_pi: []i64,
    best_parent: []usize,
    best_mst_edge: []i64,
    last_degree: []i32,
    root_edges: *[2]usize,
    best_root_edges: *[2]usize,
    stats: *CandidateBuildStats,
) void {
    @memset(pi, 0);
    @memset(best_pi, 0);
    @memset(last_degree, 0);
    var step: i64 = initialAscentStep(dist_oracle);
    var period: usize = @max(max_iterations / 2, 1);
    var initial_phase = true;
    var best_bound: i64 = std.math.minInt(i64);
    var best_norm: i64 = std.math.maxInt(i64);

    var iter: usize = 0;
    while (iter < max_iterations and step > 0 and period > 0) {
        var p: usize = 0;
        while (iter < max_iterations and p < period and step > 0) : ({
            iter += 1;
            p += 1;
        }) {
            const adjusted_tree_cost = buildOneTreeApprox(dist_oracle, pi, parent, mst_edge, in_tree, degree, root_edges);
            var pi_sum: i64 = 0;
            for (pi) |value| pi_sum += value;
            const lower_bound = adjusted_tree_cost - 2 * pi_sum;

            var norm: i64 = 0;
            for (degree) |deg| {
                const deficit = deg - 2;
                norm += @as(i64, deficit) * @as(i64, deficit);
            }

            if (lower_bound > best_bound or (lower_bound == best_bound and norm < best_norm)) {
                best_bound = lower_bound;
                best_norm = norm;
                @memcpy(best_pi, pi);
                @memcpy(best_parent, parent);
                @memcpy(best_mst_edge, mst_edge);
                best_root_edges.* = root_edges.*;
                if (initial_phase and norm > 0) step = step * 2;
                if (p + 1 == period and period < max_iterations / 2) period *= 2;
            }

            if (norm == 0) {
                iter += 1;
                stats.iterations = iter;
                stats.best_lower_bound = best_bound;
                return;
            }

            for (pi, degree, last_degree) |*penalty, deg, last| {
                const deficit = deg - 2;
                const last_deficit = if (iter == 0) deficit else last - 2;
                if (deficit != 0) penalty.* += @divTrunc(step * @as(i64, 7 * deficit + 3 * last_deficit), 10);
            }
            @memcpy(last_degree, degree);
            if (initial_phase and p > period / 2) {
                initial_phase = false;
                p = 0;
                step = @divTrunc(3 * step, 4);
            }
        }
        period /= 2;
        step = @divTrunc(step, 2);
    }

    if (best_bound == std.math.minInt(i64)) {
        _ = buildOneTreeApprox(dist_oracle, pi, best_parent, best_mst_edge, in_tree, degree, best_root_edges);
        best_bound = 0;
    }
    stats.iterations = iter;
    stats.best_lower_bound = best_bound;
}

fn initialAscentStep(dist_oracle: *DistanceOracle) i64 {
    const n = dist_oracle.p.dimension;
    var total: u64 = 0;
    for (0..n) |i| {
        var best: u64 = std.math.maxInt(u64);
        for (0..n) |j| {
            if (i == j) continue;
            best = @min(best, dist_oracle.distance(i, j));
        }
        total += best;
    }
    return @max(@as(i64, @intCast(@max(total / @max(n, 1), 1))), 1);
}

fn buildOneTreeApprox(
    dist_oracle: *DistanceOracle,
    pi: []const i64,
    parent: []usize,
    mst_edge: []i64,
    in_tree: []bool,
    degree: []i32,
    root_edges: *[2]usize,
) i64 {
    const n = dist_oracle.p.dimension;
    @memset(parent, std.math.maxInt(usize));
    @memset(mst_edge, std.math.maxInt(i64));
    @memset(in_tree, false);
    @memset(degree, 0);
    root_edges.* = .{ std.math.maxInt(usize), std.math.maxInt(usize) };

    if (n <= 2) return 0;
    in_tree[1] = true;
    mst_edge[1] = 0;
    for (2..n) |node| {
        parent[node] = 1;
        mst_edge[node] = adjustedCost(dist_oracle, pi, 1, node);
    }

    var adjusted_tree_cost: i64 = 0;
    var added: usize = 1;
    while (added < n - 1) : (added += 1) {
        var best: usize = std.math.maxInt(usize);
        var best_cost: i64 = std.math.maxInt(i64);
        for (1..n) |node| {
            if (!in_tree[node] and (mst_edge[node] < best_cost or (mst_edge[node] == best_cost and node < best))) {
                best = node;
                best_cost = mst_edge[node];
            }
        }
        std.debug.assert(best != std.math.maxInt(usize));
        in_tree[best] = true;
        adjusted_tree_cost += mst_edge[best];
        degree[best] += 1;
        degree[parent[best]] += 1;
        for (1..n) |node| {
            if (in_tree[node]) continue;
            const d = adjustedCost(dist_oracle, pi, best, node);
            if (d < mst_edge[node] or (d == mst_edge[node] and best < parent[node])) {
                parent[node] = best;
                mst_edge[node] = d;
            }
        }
    }

    var root_costs = [_]i64{ std.math.maxInt(i64), std.math.maxInt(i64) };
    for (1..n) |node| {
        const d = adjustedCost(dist_oracle, pi, 0, node);
        if (d < root_costs[0] or (d == root_costs[0] and node < root_edges.*[0])) {
            root_costs[1] = root_costs[0];
            root_edges.*[1] = root_edges.*[0];
            root_costs[0] = d;
            root_edges.*[0] = node;
        } else if (d < root_costs[1] or (d == root_costs[1] and node < root_edges.*[1])) {
            root_costs[1] = d;
            root_edges.*[1] = node;
        }
    }
    adjusted_tree_cost += root_costs[0] + root_costs[1];
    degree[0] = 2;
    degree[root_edges.*[0]] += 1;
    degree[root_edges.*[1]] += 1;
    return adjusted_tree_cost;
}

/// Alpha score for the pair (i, j) given `bottleneck` filled for row i by
/// fillTreeBottleneck (unused when i or j is the 1-tree root 0, whose
/// reference is the precomputed second-cheapest 0-edge).
fn rowAlphaScore(
    dist_oracle: *DistanceOracle,
    i: usize,
    j: usize,
    pi: []const i64,
    parent: []const usize,
    root_edges: [2]usize,
    second_root_cost: i64,
    bottleneck: []const i64,
) u64 {
    if (treeContainsEdge(i, j, parent, root_edges)) return 0;
    const adjusted = adjustedCost(dist_oracle, pi, i, j);
    if (i == 0 or j == 0) return positiveAlpha(adjusted, second_root_cost);
    return positiveAlpha(adjusted, bottleneck[j]);
}

/// BFS over the MST (CSR adjacency, nodes 1..n-1) from `root`, filling
/// `bottleneck[j]` with the maximum adjusted edge cost on the tree path
/// root..j. `visited` and `queue` are caller-provided scratch.
fn fillTreeBottleneck(
    root: usize,
    adj_start: []const usize,
    adj_node: []const usize,
    adj_weight: []const i64,
    visited: []bool,
    queue: []usize,
    bottleneck: []i64,
) void {
    @memset(visited, false);
    var head: usize = 0;
    var tail: usize = 0;
    visited[root] = true;
    bottleneck[root] = std.math.minInt(i64);
    queue[tail] = root;
    tail += 1;
    while (head < tail) {
        const u = queue[head];
        head += 1;
        for (adj_start[u]..adj_start[u + 1]) |k| {
            const v = adj_node[k];
            if (visited[v]) continue;
            visited[v] = true;
            bottleneck[v] = @max(bottleneck[u], adj_weight[k]);
            queue[tail] = v;
            tail += 1;
        }
    }
}

fn adjustedCost(dist_oracle: *DistanceOracle, pi: []const i64, a: usize, b: usize) i64 {
    return @as(i64, @intCast(dist_oracle.distance(a, b))) + pi[a] + pi[b];
}

fn positiveAlpha(adjusted: i64, reference: i64) u64 {
    if (adjusted <= reference) return 0;
    return @intCast(adjusted - reference);
}

fn buildNearestPatch(dist_oracle: *DistanceOracle, node: usize, out: []usize) usize {
    const n = dist_oracle.p.dimension;
    var out_dist: [8]u64 = .{
        std.math.maxInt(u64),
        std.math.maxInt(u64),
        std.math.maxInt(u64),
        std.math.maxInt(u64),
        std.math.maxInt(u64),
        std.math.maxInt(u64),
        std.math.maxInt(u64),
        std.math.maxInt(u64),
    };
    std.debug.assert(out.len <= out_dist.len);
    for (out) |*slot| slot.* = std.math.maxInt(usize);

    for (0..n) |other| {
        if (other == node) continue;
        const d = @as(u64, dist_oracle.distance(node, other));
        var slot: ?usize = null;
        for (0..out.len) |i| {
            if (d < out_dist[i] or (d == out_dist[i] and other < out[i])) {
                slot = i;
                break;
            }
        }
        if (slot) |insert_at| {
            var i = out.len - 1;
            while (i > insert_at) : (i -= 1) {
                out_dist[i] = out_dist[i - 1];
                out[i] = out[i - 1];
            }
            out_dist[insert_at] = d;
            out[insert_at] = other;
        }
    }

    var count: usize = 0;
    while (count < out.len and out[count] != std.math.maxInt(usize)) : (count += 1) {}
    return count;
}

fn rowContains(row: []const usize, node: usize) bool {
    for (row) |candidate| {
        if (candidate == node) return true;
    }
    return false;
}

fn sortCandidateRow(row: []usize, alpha: []u64, dist: []u64) void {
    for (1..row.len) |i| {
        var j = i;
        while (j > 0 and candidateLess(alpha[j], dist[j], row[j], alpha[j - 1], dist[j - 1], row[j - 1])) : (j -= 1) {
            std.mem.swap(usize, &row[j], &row[j - 1]);
            std.mem.swap(u64, &alpha[j], &alpha[j - 1]);
            std.mem.swap(u64, &dist[j], &dist[j - 1]);
        }
    }
}

fn candidateLess(alpha_a: u64, dist_a: u64, node_a: usize, alpha_b: u64, dist_b: u64, node_b: usize) bool {
    return alpha_a < alpha_b or
        (alpha_a == alpha_b and dist_a < dist_b) or
        (alpha_a == alpha_b and dist_a == dist_b and node_a < node_b);
}

fn treeContainsEdge(a: usize, b: usize, parent: []const usize, root_edges: [2]usize) bool {
    if (a != 0 and parent[a] == b) return true;
    if (b != 0 and parent[b] == a) return true;
    return (a == 0 and (b == root_edges[0] or b == root_edges[1])) or
        (b == 0 and (a == root_edges[0] or a == root_edges[1]));
}

fn validateCandidateRow(node: usize, row: []const usize) void {
    for (row, 0..) |candidate, k| {
        std.debug.assert(candidate != node);
        for (row[0..k]) |previous| std.debug.assert(previous != candidate);
    }
}

fn nearestNeighborTour(
    dist_oracle: *DistanceOracle,
    candidates: *const Candidates,
    random: *std.Random,
    trial: usize,
    randomized: bool,
    tour: []usize,
    used: []bool,
) void {
    const n = dist_oracle.p.dimension;
    std.debug.assert(used.len == n);
    @memset(used, false);

    var current = trial % n;
    if (randomized and trial > 0) current = random.intRangeLessThan(usize, 0, n);

    for (0..n) |idx| {
        tour[idx] = current;
        used[current] = true;
        if (idx + 1 == n) break;

        var best_nodes: [4]usize = undefined;
        var best_dist: [4]u64 = .{ std.math.maxInt(u64), std.math.maxInt(u64), std.math.maxInt(u64), std.math.maxInt(u64) };
        var found: usize = 0;

        for (candidates.row(current)) |candidate| {
            if (!used[candidate]) {
                insertCandidate(candidate, dist_oracle.distance(current, candidate), &best_nodes, &best_dist, &found);
            }
        }
        if (found == 0) {
            for (0..n) |node| {
                if (!used[node]) {
                    insertCandidate(node, dist_oracle.distance(current, node), &best_nodes, &best_dist, &found);
                }
            }
        }
        std.debug.assert(found != 0);

        const choice_count = if (randomized and trial > 0) @min(found, 3) else 1;
        const chosen_idx = if (choice_count > 1) random.intRangeLessThan(usize, 0, choice_count) else 0;
        current = best_nodes[chosen_idx];
    }
}

fn farthestInsertionTour(
    dist_oracle: *DistanceOracle,
    tour: []usize,
    scratch: []usize,
    used: []bool,
) void {
    const n = dist_oracle.p.dimension;
    std.debug.assert(tour.len == n);
    std.debug.assert(scratch.len == n);
    std.debug.assert(used.len == n);
    @memset(used, false);

    var first: usize = 0;
    var second: usize = 1;
    var best_dist: u32 = 0;
    for (0..n) |a| {
        for (a + 1..n) |b| {
            const d = dist_oracle.distance(a, b);
            if (d > best_dist) {
                best_dist = d;
                first = a;
                second = b;
            }
        }
    }

    tour[0] = first;
    tour[1] = second;
    used[first] = true;
    used[second] = true;
    var len: usize = 2;

    while (len < n) {
        var next_node: usize = std.math.maxInt(usize);
        var farthest: u32 = 0;
        for (0..n) |node| {
            if (used[node]) continue;
            var nearest_to_tour: u32 = std.math.maxInt(u32);
            for (tour[0..len]) |tour_node| {
                nearest_to_tour = @min(nearest_to_tour, dist_oracle.distance(node, tour_node));
            }
            if (nearest_to_tour > farthest or (nearest_to_tour == farthest and node < next_node)) {
                farthest = nearest_to_tour;
                next_node = node;
            }
        }
        std.debug.assert(next_node != std.math.maxInt(usize));

        var insert_after: usize = 0;
        var best_delta: i64 = std.math.maxInt(i64);
        for (0..len) |idx| {
            const a = tour[idx];
            const b = tour[(idx + 1) % len];
            const delta = @as(i64, @intCast(dist_oracle.distance(a, next_node))) +
                @as(i64, @intCast(dist_oracle.distance(next_node, b))) -
                @as(i64, @intCast(dist_oracle.distance(a, b)));
            if (delta < best_delta or (delta == best_delta and a < tour[insert_after])) {
                best_delta = delta;
                insert_after = idx;
            }
        }

        @memcpy(scratch[0..len], tour[0..len]);
        const insert_at = insert_after + 1;
        @memcpy(tour[0..insert_at], scratch[0..insert_at]);
        tour[insert_at] = next_node;
        @memcpy(tour[insert_at + 1 .. len + 1], scratch[insert_at..len]);
        used[next_node] = true;
        len += 1;
    }
}

fn insertCandidate(
    node: usize,
    dist: u32,
    best_nodes: *[4]usize,
    best_dist: *[4]u64,
    found: *usize,
) void {
    const candidate_dist = @as(u64, dist);
    var slot: ?usize = null;
    for (0..best_dist.len) |i| {
        if (candidate_dist < best_dist[i] or (candidate_dist == best_dist[i] and (found.* <= i or node < best_nodes[i]))) {
            slot = i;
            break;
        }
    }
    const pos = slot orelse return;
    var i = best_dist.len - 1;
    while (i > pos) : (i -= 1) {
        best_dist[i] = best_dist[i - 1];
        best_nodes[i] = best_nodes[i - 1];
    }
    best_dist[pos] = candidate_dist;
    best_nodes[pos] = node;
    found.* = @min(best_dist.len, found.* + 1);
}

// Double-bridge perturbation: split the tour into A B C D and reconnect as
// A C B D (segment exchange, no reversals). Exactly three tour edges change;
// their six endpoints are reported so the LK queue can be seeded with only the
// perturbed neighborhood (cf. LKH's kick + InBestTour activation gating).
// Roadmap item 3 vote primitives (file scope so both the kick and the
// LocalSearch generator share one implementation).

// Misra-Gries update: fold neighbour v into one of node u's two slots. Returns
// true if it took the decrement (auto-thaw) branch.
fn voteEdgeSlot(vote_node: []usize, vote_count: []u32, u: usize, v: usize) bool {
    const base = 2 * u;
    if (vote_node[base] == v) {
        vote_count[base] +|= 1;
        return false;
    }
    if (vote_node[base + 1] == v) {
        vote_count[base + 1] +|= 1;
        return false;
    }
    if (vote_count[base] == 0) {
        vote_node[base] = v;
        vote_count[base] = 1;
        return false;
    }
    if (vote_count[base + 1] == 0) {
        vote_node[base + 1] = v;
        vote_count[base + 1] = 1;
        return false;
    }
    // Both slots taken by other neighbours: decrement (the auto-thaw step).
    vote_count[base] -= 1;
    vote_count[base + 1] -= 1;
    return true;
}

// Vote every edge of `tour` from both endpoints (each node votes its two tour
// neighbours into its own slots). Accumulates the number of auto-thaw
// decrements into `decrements` for diagnostics.
fn voteTourEdges(vote_node: []usize, vote_count: []u32, tour: []const usize, decrements: *u64) void {
    const n = tour.len;
    for (0..n) |idx| {
        const u = tour[idx];
        if (voteEdgeSlot(vote_node, vote_count, u, tour[(idx + 1) % n])) decrements.* += 1;
        if (voteEdgeSlot(vote_node, vote_count, u, tour[(idx + n - 1) % n])) decrements.* += 1;
    }
}

// Item-3 revival: a statically injected frozen edge set (packed lo<<32|hi,
// sorted ascending). Lets an externally-computed backbone (e.g. a diverse
// elite/restart consensus) be frozen from trial 0, bypassing the voted set, so
// the upper bound of freezing a PURE backbone can be measured directly.
fn packEdge(u: usize, v: usize) u64 {
    const lo = @min(u, v);
    const hi = @max(u, v);
    return (@as(u64, @intCast(lo)) << 32) | @as(u64, @intCast(hi));
}

fn injectedFrozen(inject: []const u64, u: usize, v: usize) bool {
    if (inject.len == 0) return false;
    const key = packEdge(u, v);
    var lo: usize = 0;
    var hi: usize = inject.len;
    while (lo < hi) {
        const mid = lo + (hi - lo) / 2;
        if (inject[mid] < key) lo = mid + 1 else hi = mid;
    }
    return lo < inject.len and inject[lo] == key;
}

// Frozen check that ORs the static injected set over the voted Misra-Gries set.
fn edgeIsFrozenWith(vote_node: []const usize, vote_count: []const u32, threshold: u32, inject: []const u64, u: usize, v: usize) bool {
    if (injectedFrozen(inject, u, v)) return true;
    return edgeIsFrozen(vote_node, vote_count, threshold, u, v);
}

fn voteSlotCount(vote_node: []const usize, vote_count: []const u32, u: usize, v: usize) u32 {
    const base = 2 * u;
    if (vote_node[base] == v) return vote_count[base];
    if (vote_node[base + 1] == v) return vote_count[base + 1];
    return 0;
}

fn edgeIsFrozen(vote_node: []const usize, vote_count: []const u32, threshold: u32, u: usize, v: usize) bool {
    if (threshold == std.math.maxInt(u32)) return false;
    return voteSlotCount(vote_node, vote_count, u, v) >= threshold and
        voteSlotCount(vote_node, vote_count, v, u) >= threshold;
}

fn segmentExchangeKick(tour: []usize, random: *std.Random, touched: *[6]usize) void {
    const n = tour.len;
    std.debug.assert(n >= 8);
    const i = random.intRangeLessThan(usize, 1, n - 2);
    const j = random.intRangeLessThan(usize, i + 1, n - 1);
    const k = random.intRangeLessThan(usize, j + 1, n);
    touched.* = .{ tour[i - 1], tour[i], tour[j - 1], tour[j], tour[k - 1], tour[k] };
    std.mem.rotate(usize, tour[i..k], j - i);
}

// Roadmap item 3: like segmentExchangeKick, but redraw the three cut points
// (bounded) until none of the broken edges is frozen, so the generator's
// perturbation preserves consensus structure. Falls back to an unconstrained
// kick if every attempt hits a frozen edge. Only invoked when freezing is
// active, so the off-path RNG stream above stays bit-identical.
fn segmentExchangeKickAvoidingFrozen(
    tour: []usize,
    random: *std.Random,
    touched: *[6]usize,
    vote_node: []const usize,
    vote_count: []const u32,
    threshold: u32,
    inject: []const u64,
) void {
    const n = tour.len;
    std.debug.assert(n >= 8);
    var attempt: usize = 0;
    while (attempt < 16) : (attempt += 1) {
        const i = random.intRangeLessThan(usize, 1, n - 2);
        const j = random.intRangeLessThan(usize, i + 1, n - 1);
        const k = random.intRangeLessThan(usize, j + 1, n);
        if (edgeIsFrozenWith(vote_node, vote_count, threshold, inject, tour[i - 1], tour[i]) or
            edgeIsFrozenWith(vote_node, vote_count, threshold, inject, tour[j - 1], tour[j]) or
            edgeIsFrozenWith(vote_node, vote_count, threshold, inject, tour[k - 1], tour[k])) continue;
        touched.* = .{ tour[i - 1], tour[i], tour[j - 1], tour[j], tour[k - 1], tour[k] };
        std.mem.rotate(usize, tour[i..k], j - i);
        return;
    }
    segmentExchangeKick(tour, random, touched);
}

// Plateau kick: apply up to `moves` zero-delta reconnections anchored at
// random tour positions. On degenerate integer geometries (rattled grids,
// drilling patterns) locally optimal tours sit on broad cost-equal plateaus:
// the residual gap to the optimum hides in scattered micro-sections whose
// better variant is cost-equal until a neighboring section also changes
// (measured on rat575: 67 differing edges vs the optimum in 59 sections of
// size <= 2). Length-preserving moves walk the plateau without giving up
// quality, so the follow-up descent starts from a genuinely different tour
// of equal length. With `or_opt` set, half the attempts are zero-delta
// segment relocations (1-3 nodes, both orientations) — the scattered
// residual sections are size <= 2, i.e. relocations, which reversal drift
// cannot express; the stride of `touched` grows from 4 to 6 per move and
// the RNG consumption changes, so callers gate it to keep small-instance
// trajectories untouched. Returns the number of applied moves.
fn plateauKick(
    dist_oracle: *DistanceOracle,
    candidates: *const Candidates,
    tour: []usize,
    pos: []usize,
    random: *std.Random,
    moves: usize,
    or_opt: bool,
    touched: []usize,
) usize {
    const n = tour.len;
    const stride: usize = if (or_opt) 6 else 4;
    std.debug.assert(touched.len >= stride * moves);
    for (tour, 0..) |node, idx| pos[node] = idx;

    var applied: usize = 0;
    var attempts: usize = 0;
    const max_attempts = 8 * moves;
    while (applied < moves and attempts < max_attempts) : (attempts += 1) {
        if (or_opt and random.intRangeLessThan(usize, 0, 2) == 1) {
            applied += plateauOrOptMove(dist_oracle, candidates, tour, pos, random, touched[stride * applied ..]);
            continue;
        }
        const i = random.intRangeLessThan(usize, 0, n);
        const a = tour[i];
        const b = tour[(i + 1) % n];
        const d_ab = @as(i64, @intCast(dist_oracle.distance(a, b)));
        for (candidates.row(a)) |c| {
            if (c == b or c == a) continue;
            const j = pos[c];
            const d = tour[(j + 1) % n];
            if (d == a) continue;
            const delta = @as(i64, @intCast(dist_oracle.distance(a, c))) +
                @as(i64, @intCast(dist_oracle.distance(b, d))) -
                d_ab - @as(i64, @intCast(dist_oracle.distance(c, d)));
            if (delta != 0) continue;
            // 2-opt: reverse the path b..c (positions i+1..j, possibly
            // wrapping); normalize so the reversed span lies in-bounds.
            var lo = (i + 1) % n;
            var hi = j;
            if (lo > hi) {
                // Reverse the complementary span d..a instead; same cycle.
                lo = (j + 1) % n;
                hi = i;
                if (lo > hi) continue;
            }
            var x = lo;
            var y = hi;
            while (x < y) : ({
                x += 1;
                y -= 1;
            }) {
                std.mem.swap(usize, &tour[x], &tour[y]);
                pos[tour[x]] = x;
                pos[tour[y]] = y;
            }
            pos[tour[x]] = x;
            touched[stride * applied + 0] = a;
            touched[stride * applied + 1] = b;
            touched[stride * applied + 2] = c;
            touched[stride * applied + 3] = d;
            if (or_opt) {
                touched[stride * applied + 4] = a;
                touched[stride * applied + 5] = b;
            }
            applied += 1;
            break;
        }
    }
    return applied;
}

// One zero-delta Or-opt drift attempt: relocate the 1-3 node segment at a
// random position to sit after a candidate neighbor of its head node, in
// either orientation, when the relocation is exactly cost-neutral. Returns
// 1 and records 6 endpoints into `touched` on success, 0 otherwise.
// Wrapping segments and wrapping insertions are skipped (the array splice
// stays a single contiguous shift; the anchor position is uniform anyway).
fn plateauOrOptMove(
    dist_oracle: *DistanceOracle,
    candidates: *const Candidates,
    tour: []usize,
    pos: []usize,
    random: *std.Random,
    touched: []usize,
) usize {
    const n = tour.len;
    const seg_len = random.intRangeLessThan(usize, 1, 4);
    const i = random.intRangeLessThan(usize, 0, n);
    if (i == 0 or i + seg_len >= n) return 0;
    const p = tour[i - 1];
    const s0 = tour[i];
    const s1 = tour[i + seg_len - 1];
    const q = tour[i + seg_len];
    const removal_gain = @as(i64, @intCast(dist_oracle.distance(p, s0))) +
        @as(i64, @intCast(dist_oracle.distance(s1, q))) -
        @as(i64, @intCast(dist_oracle.distance(p, q)));
    if (removal_gain == 0) return 0;

    for (candidates.row(s0)) |c| {
        const jc = pos[c];
        // c inside the segment or directly before it (re-insertion no-op).
        if (jc + 1 >= i and jc < i + seg_len) continue;
        if (jc == n - 1) continue;
        const cn = tour[jc + 1];
        const d_ccn = @as(i64, @intCast(dist_oracle.distance(c, cn)));
        const ins_fwd = @as(i64, @intCast(dist_oracle.distance(c, s0))) +
            @as(i64, @intCast(dist_oracle.distance(s1, cn))) - d_ccn;
        const ins_rev = @as(i64, @intCast(dist_oracle.distance(c, s1))) +
            @as(i64, @intCast(dist_oracle.distance(s0, cn))) - d_ccn;
        const fwd = ins_fwd == removal_gain;
        if (!fwd and ins_rev != removal_gain) continue;

        var seg: [3]usize = undefined;
        @memcpy(seg[0..seg_len], tour[i .. i + seg_len]);
        if (!fwd) std.mem.reverse(usize, seg[0..seg_len]);
        if (jc > i) {
            // Shift the gap left, drop the segment in after c.
            std.mem.copyForwards(usize, tour[i .. jc + 1 - seg_len], tour[i + seg_len .. jc + 1]);
            @memcpy(tour[jc + 1 - seg_len .. jc + 1], seg[0..seg_len]);
            for (i..jc + 1) |idx| pos[tour[idx]] = idx;
        } else {
            // Shift the gap right, drop the segment in after c.
            std.mem.copyBackwards(usize, tour[jc + 1 + seg_len .. i + seg_len], tour[jc + 1 .. i]);
            @memcpy(tour[jc + 1 .. jc + 1 + seg_len], seg[0..seg_len]);
            for (jc + 1..i + seg_len) |idx| pos[tour[idx]] = idx;
        }
        touched[0] = p;
        touched[1] = q;
        touched[2] = c;
        touched[3] = cn;
        touched[4] = s0;
        touched[5] = s1;
        return 1;
    }
    return 0;
}

// Guided restart construction, ported from LKH's ChooseInitialTour cases
// C/D/E (Trial > 1): starting at a random node, repeatedly extend with
//   (C) a random unchosen candidate neighbor whose edge has alpha == 0 and
//       lies in the best or next-best reference tour, else
//   (D) a random unchosen candidate neighbor, else
//   (E) the nearest unchosen node.
// Retaining the alpha-zero backbone keeps the tour near-elite while every
// alpha>0 stretch diverges through case D — structurally different parents
// that double-bridge kicks cannot produce and that give EAX merging
// independent differing sections to recombine.
// On return `used` no longer means "chosen": it flags the endpoints of every
// tour edge absent from both reference tours, i.e. the only neighborhoods a
// follow-up LK descent needs to reactivate.
fn guidedBackboneTour(
    dist_oracle: *DistanceOracle,
    candidates: *const Candidates,
    refs: [2]?[]const usize,
    ref_next: [2][]usize,
    ref_prev: [2][]usize,
    max_divergence: usize,
    random: *std.Random,
    tour: []usize,
    used: []bool,
) void {
    const n = dist_oracle.p.dimension;
    std.debug.assert(used.len == n);
    @memset(used, false);
    for (refs, ref_next, ref_prev) |maybe_ref, rn, rp| {
        const ref = maybe_ref orelse continue;
        std.debug.assert(ref.len == n);
        for (ref, 0..) |node, idx| {
            rn[node] = ref[(idx + 1) % n];
            rp[node] = ref[(idx + n - 1) % n];
        }
    }

    var current = random.intRangeLessThan(usize, 0, n);
    var divergences: usize = 0;
    for (0..n) |idx| {
        tour[idx] = current;
        used[current] = true;
        if (idx + 1 == n) break;

        const row = candidates.row(current);
        const alpha_row = candidates.alphaRow(current);
        var count: usize = 0;
        var pick: usize = 0;
        // Case C: alpha-zero candidate edges on a reference tour, picked
        // uniformly via reservoir sampling (LKH picks uniformly among the
        // collected alternatives).
        for (row, alpha_row) |cand, alpha| {
            if (used[cand] or alpha != 0) continue;
            if (!guideEdgeOnRefs(refs, ref_next, ref_prev, current, cand)) continue;
            count += 1;
            if (count == 1 or random.intRangeLessThan(usize, 0, count) == 0) pick = cand;
        }
        // Case C-and-a-half (deviation from LKH, active only when a finite
        // divergence budget is set): a candidate reference-tour edge
        // regardless of alpha. While under the budget it is followed with
        // probability 3/4 so that divergence strikes at random alpha>0
        // stretches; once the budget is spent it is followed
        // unconditionally. Unbounded case-D divergence makes the follow-up
        // light descent nearly as expensive as a cold one; the budget keeps
        // guided tours near-elite — few neighborhoods to reactivate, and
        // localized independent differences, which is exactly what EAX
        // merging wants. With max_divergence == maxInt the construction is
        // the faithful LKH C/D/E ladder for full-descent callers.
        if (count == 0 and max_divergence != std.math.maxInt(usize) and
            (divergences >= max_divergence or random.intRangeLessThan(usize, 0, 4) != 0))
        {
            for (row) |cand| {
                if (used[cand]) continue;
                if (!guideEdgeOnRefs(refs, ref_next, ref_prev, current, cand)) continue;
                count += 1;
                if (count == 1 or random.intRangeLessThan(usize, 0, count) == 0) pick = cand;
            }
        }
        // Case D: any unchosen candidate edge.
        if (count == 0) {
            divergences += 1;
            for (row) |cand| {
                if (used[cand]) continue;
                count += 1;
                if (count == 1 or random.intRangeLessThan(usize, 0, count) == 0) pick = cand;
            }
        }
        // Case E: nearest unchosen node.
        if (count == 0) {
            var best_dist: u64 = std.math.maxInt(u64);
            for (0..n) |node| {
                if (used[node]) continue;
                const d = @as(u64, dist_oracle.distance(current, node));
                if (d < best_dist) {
                    best_dist = d;
                    pick = node;
                }
            }
        }
        current = pick;
    }

    @memset(used, false);
    for (tour, 0..) |a, idx| {
        const b = tour[(idx + 1) % n];
        if (!guideEdgeOnRefs(refs, ref_next, ref_prev, a, b)) {
            used[a] = true;
            used[b] = true;
        }
    }
}

fn guideEdgeOnRefs(refs: [2]?[]const usize, ref_next: [2][]usize, ref_prev: [2][]usize, a: usize, b: usize) bool {
    for (refs, ref_next, ref_prev) |maybe_ref, rn, rp| {
        if (maybe_ref == null) continue;
        if (rn[a] == b or rp[a] == b) return true;
    }
    return false;
}

// --- Iterative Partial Transcription (tour merging) ------------------------
//
// Mobius, Freisleben, Merz, Schreiber, "Combinatorial Optimization by
// Iterative Partial Transcription", Phys. Rev. E 59(4), 1999 — the mechanism
// behind LKH's MergeWithTour. Two Hamiltonian cycles over the same nodes are
// decomposed into shared portions and pairs of alternative subpaths between
// common endpoints; each independent differing section is resolved to the
// cheaper alternative, so the merged tour can be strictly shorter than both
// parents.

const IptScratch = struct {
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

    fn init(allocator: std.mem.Allocator, n: usize) !IptScratch {
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

    fn deinit(self: *IptScratch) void {
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

const IptOutcome = struct {
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
fn iptMergeTours(
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

const EaxScratch = struct {
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

    fn init(allocator: std.mem.Allocator, n: usize) !EaxScratch {
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

    fn deinit(self: *EaxScratch) void {
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

const ElitePool = struct {
    allocator: std.mem.Allocator,
    tours: [elite_pool_capacity][]usize,
    lens: [elite_pool_capacity]u64,
    count: usize,

    fn init(allocator: std.mem.Allocator, n: usize) !ElitePool {
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

    fn deinit(self: *ElitePool) void {
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

fn elitePoolOffer(pool: *ElitePool, scratch: *EaxScratch, tour: []const usize, len: u64) void {
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
fn eaxMergeTours(
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

const LocalSearch = struct {
    dist: *DistanceOracle,
    // Trial-loop cost counters (roadmap item 1) accumulate here directly:
    // tour mutations (reverseSegment / applyEdges rebuilds) live in this
    // struct, not in the ephemeral TourView, so they vote into stats in place.
    stats: *SolveStats,
    candidates: *const Candidates,
    tour: []usize,
    pos: []usize,
    next: []usize,
    prev: []usize,
    candidate_tour: []usize,
    scratch_neighbor0: []usize,
    scratch_neighbor1: []usize,
    scratch_seen: []bool,
    segment_of_node: []usize,
    rank_in_segment: []usize,
    segment_start: []usize,
    segment_len: []usize,
    segment_reversed: []bool,
    move_degree_delta: []i8,
    move_component: []usize,
    move_component_size: []usize,
    move_edges: []TourEdge,
    lk_t: []usize,
    removed_a: []usize,
    removed_b: []usize,
    added_a: []usize,
    added_b: []usize,
    max_passes: usize,
    enable_or_opt: bool,
    enable_bounded_three_opt_cleanup: bool,
    enable_move_patching: bool,
    move_patch_min_gain: i64,
    lk_completion_patch_min_gain: i64,
    max_lk_depth: usize,
    lk_backtrack_limit: usize,
    lk_backtrack_depth: usize,
    lk_nonseq_branch_limit: usize,
    lk_nodes_this_pass: usize = 0,
    lk_active: []bool,
    lk_active_queue: []usize,
    lk_active_head: usize = 0,
    lk_active_count: usize = 0,
    // Roadmap item 2 (incremental bookkeeping): the tour length, maintained
    // delta-style so the trial loop reads it in O(1) instead of rescanning the
    // whole tour (the per-trial full-array scans the item-1 counters measured).
    // syncLength() reseeds it from one scan after each construction/kick; every
    // move applier folds in its exact edge delta. Debug builds assert it against
    // a fresh scan at the end of each trial, so any drift fails the test suite.
    current_length: u64 = 0,
    // Roadmap item 3: edge voting-freeze. vote_node/vote_count are the shared
    // per-node Misra-Gries slots (see SolverWorkspace). respect_frozen is set
    // on the GENERATOR search and cleared on the COMBINER's polish search, so
    // the combiner can still cut through frozen regions. freeze_threshold is
    // the per-trial absolute vote count an edge endpoint must reach to count as
    // frozen (maxInt = nothing frozen this trial). Defaults leave the feature
    // inert, so direct LocalSearch constructions (tests) are unaffected.
    vote_node: []const usize = &.{},
    vote_count: []const u32 = &.{},
    freeze_threshold: u32 = std.math.maxInt(u32),
    // Item-3 revival: statically injected frozen backbone (see SolveOptions).
    inject_frozen: []const u64 = &.{},
    respect_frozen: bool = false,
    // Item-3 revival: soft freeze — prune LK search initiation in interior
    // backbone, never forbid a move (see findLKMove).
    soft_freeze: bool = false,

    // Active-node queue ("don't-look bits", Helsgaun Sec. 3/LKH StoreTour):
    // only nodes whose neighborhood changed since their last failed search are
    // re-examined; an improving move reactivates every endpoint it touched.
    fn lkActivateAll(self: *LocalSearch) void {
        const n = self.tour.len;
        @memset(self.lk_active, true);
        @memcpy(self.lk_active_queue, self.tour);
        self.lk_active_head = 0;
        self.lk_active_count = n;
    }

    fn lkResetActive(self: *LocalSearch) void {
        @memset(self.lk_active, false);
        self.lk_active_head = 0;
        self.lk_active_count = 0;
    }

    fn lkActivate(self: *LocalSearch, node: usize) void {
        if (self.lk_active[node]) return;
        self.lk_active[node] = true;
        const slot = (self.lk_active_head + self.lk_active_count) % self.lk_active_queue.len;
        self.lk_active_queue[slot] = node;
        self.lk_active_count += 1;
    }

    fn lkPopActive(self: *LocalSearch) ?usize {
        if (self.lk_active_count == 0) return null;
        const node = self.lk_active_queue[self.lk_active_head];
        self.lk_active_head = (self.lk_active_head + 1) % self.lk_active_queue.len;
        self.lk_active_count -= 1;
        self.lk_active[node] = false;
        return node;
    }

    fn lkActivateMoveEndpoints(self: *LocalSearch, removed_count: usize, added_count: usize) void {
        for (0..removed_count) |i| {
            self.lkActivate(self.removed_a[i]);
            self.lkActivate(self.removed_b[i]);
        }
        for (0..added_count) |i| {
            self.lkActivate(self.added_a[i]);
            self.lkActivate(self.added_b[i]);
        }
    }

    fn improveWarmup(self: *LocalSearch) !u64 {
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
    fn improveLK(self: *LocalSearch, stats: *SolveStats, activate_all: bool, full: bool) !u64 {
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

    fn improveGain23Bridge(self: *LocalSearch, stats: *SolveStats) bool {
        if (self.lk_nonseq_branch_limit == 0 or self.tour.len < 8) return false;

        const n = self.tour.len;
        for (0..n) |i| {
            const s1 = self.tour[i];
            const s2 = self.next[s1];
            const removed_first: i64 = @intCast(self.dist.distance(s1, s2));
            var breadth: usize = 0;

            for (self.candidates.row(s2)) |s3| {
                if (breadth >= self.lk_nonseq_branch_limit) break;
                if (s3 == s1 or s3 == s2 or self.isTourEdge(s2, s3)) continue;
                const s4 = self.next[s3];
                if (s4 == s1 or s4 == s2) continue;
                if (self.pos[s3] <= self.pos[s2]) continue;
                if (!self.segmentIsNoMoreThanHalf(s2, s3)) continue;

                const gain =
                    removed_first -
                    @as(i64, @intCast(self.dist.distance(s2, s3))) +
                    @as(i64, @intCast(self.dist.distance(s3, s4))) -
                    @as(i64, @intCast(self.dist.distance(s4, s1)));
                if (gain <= 0) continue;

                if (!self.recordLKNode(stats)) return false;
                stats.lk_completion_attempts += 1;
                stats.lk_nonseq_attempts += 1;
                breadth += 1;

                self.removed_a[0] = s1;
                self.removed_b[0] = s2;
                self.removed_a[1] = s3;
                self.removed_b[1] = s4;
                self.added_a[0] = s2;
                self.added_b[0] = s3;
                self.added_a[1] = s4;
                self.added_b[1] = s1;

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

    fn tryGain23BridgeGain2Opt(
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
            const t2 = self.next[t1];
            defer t1 = t2;
            if (self.isExcludedGain23BaseEdge(t1, t2, s1, s2, s3, s4, std.math.maxInt(usize), std.math.maxInt(usize))) continue;

            const gain0 = base_gain + @as(i64, @intCast(self.dist.distance(t1, t2)));
            var breadth2: usize = 0;
            for (self.candidates.row(t2)) |t3| {
                if (breadth2 >= self.lk_nonseq_branch_limit) break;
                if (t3 == t1 or t3 == t2 or self.isTourEdge(t2, t3)) continue;
                if (self.nodeInCircularSegment(segment.from, t3, segment.to)) continue;
                const gain1 = gain0 - @as(i64, @intCast(self.dist.distance(t2, t3)));
                if (gain1 <= 0) continue;
                breadth2 += 1;

                var choices = [2]usize{ self.next[t3], self.prev[t3] };
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

                    self.removed_a[0] = s1;
                    self.removed_b[0] = s2;
                    self.removed_a[1] = s3;
                    self.removed_b[1] = s4;
                    self.removed_a[2] = t1;
                    self.removed_b[2] = t2;
                    self.removed_a[3] = t3;
                    self.removed_b[3] = t4;
                    self.added_a[0] = s2;
                    self.added_b[0] = s3;
                    self.added_a[1] = s4;
                    self.added_b[1] = s1;
                    self.added_a[2] = t2;
                    self.added_b[2] = t3;
                    self.added_a[3] = t4;
                    self.added_b[3] = t1;

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

    fn smallerSegmentEndpoints(self: *const LocalSearch, a: usize, b: usize, c: usize, d: usize) SegmentEndpoints {
        const ab = self.circularSegmentSize(a, b);
        const cd = self.circularSegmentSize(c, d);
        return if (ab <= cd) .{ .from = a, .to = b } else .{ .from = c, .to = d };
    }

    fn circularSegmentSize(self: *const LocalSearch, from: usize, to: usize) usize {
        const n = self.tour.len;
        const from_pos = self.pos[from];
        const to_pos = self.pos[to];
        return if (to_pos >= from_pos) to_pos - from_pos + 1 else n - from_pos + to_pos + 1;
    }

    fn nodeInCircularSegment(self: *const LocalSearch, from: usize, node: usize, to: usize) bool {
        const from_pos = self.pos[from];
        const node_pos = self.pos[node];
        const to_pos = self.pos[to];
        if (from_pos <= to_pos) return from_pos <= node_pos and node_pos <= to_pos;
        return node_pos >= from_pos or node_pos <= to_pos;
    }

    fn isExcludedGain23BaseEdge(
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

    fn tryGain23ThreeEdgeBridge(
        self: *LocalSearch,
        s1: usize,
        s2: usize,
        s3: usize,
        s4: usize,
        base_gain: i64,
        stats: *SolveStats,
    ) bool {
        var breadth4: usize = 0;
        for (self.candidates.row(s4)) |s5| {
            if (breadth4 >= self.lk_nonseq_branch_limit) break;
            if (s5 == s1 or s5 == s2 or s5 == s3 or s5 == s4) continue;
            if (self.isTourEdge(s4, s5)) continue;

            const after_second_add = base_gain - @as(i64, @intCast(self.dist.distance(s4, s5)));
            if (after_second_add <= 0) continue;
            breadth4 += 1;

            var choices = [2]usize{ self.next[s5], self.prev[s5] };
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

                self.removed_a[0] = s1;
                self.removed_b[0] = s2;
                self.removed_a[1] = s3;
                self.removed_b[1] = s4;
                self.removed_a[2] = s5;
                self.removed_b[2] = s6;
                self.added_a[0] = s2;
                self.added_b[0] = s3;
                self.added_a[1] = s4;
                self.added_b[1] = s5;
                self.added_a[2] = s6;
                self.added_b[2] = s1;

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

    fn segmentIsNoMoreThanHalf(self: *const LocalSearch, from: usize, to: usize) bool {
        const n = self.tour.len;
        const from_pos = self.pos[from];
        const to_pos = self.pos[to];
        const span = if (to_pos >= from_pos) to_pos - from_pos + 1 else n - from_pos + to_pos + 1;
        return 2 * span <= n;
    }

    fn findLKMove(self: *LocalSearch, stats: *SolveStats) u64 {
        var moves: u64 = 0;
        while (self.lkPopActive()) |t1| {
            // Item-3 revival (soft freeze, the LKH mechanism): don't INITIATE a
            // sequential search from an interior-backbone node (both its tour
            // edges frozen). The frozen edges can still be broken when a search
            // started elsewhere reaches them deeper, so escape paths are kept —
            // unlike the hard LK-respect reject. Pure search-initiation pruning.
            if (self.soft_freeze and
                self.softFrozenEdge(t1, self.next[t1]) and
                self.softFrozenEdge(t1, self.prev[t1]))
            {
                stats.freeze_move_rejections += 1;
                continue;
            }
            var choices = [2]usize{ self.next[t1], self.prev[t1] };
            self.orderTourEdgeChoices(t1, &choices);

            for (choices) |t2| {
                if (self.lk_nodes_this_pass >= self.lk_backtrack_limit) {
                    // Budget slice exhausted: keep t1 queued so the next pass
                    // resumes the descent instead of dropping it.
                    self.lkActivate(t1);
                    return moves;
                }
                stats.lk_attempts += 1;
                self.lk_t[0] = t1;
                self.lk_t[1] = t2;
                self.removed_a[0] = t1;
                self.removed_b[0] = t2;
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

    fn orderTourEdgeChoices(self: *LocalSearch, base: usize, choices: *[2]usize) void {
        const d0 = self.dist.distance(base, choices[0]);
        const d1 = self.dist.distance(base, choices[1]);
        if (d1 > d0 or (d1 == d0 and choices[1] < choices[0])) {
            std.mem.swap(usize, &choices[0], &choices[1]);
        }
    }

    fn recordLKNode(self: *LocalSearch, stats: *SolveStats) bool {
        if (self.lk_nodes_this_pass >= self.lk_backtrack_limit) {
            stats.lk_backtrack_cutoff_hits += 1;
            return false;
        }
        self.lk_nodes_this_pass += 1;
        stats.lk_search_nodes += 1;
        return true;
    }

    fn searchAdded(self: *LocalSearch, depth: usize, even: usize, gain: i64, stats: *SolveStats) bool {
        if (!self.recordLKNode(stats)) return false;
        const sequence_len = 2 * depth;
        const t1 = self.lk_t[0];
        // Backtracking discipline: beyond lk_backtrack_depth the search
        // commits to the first viable candidate instead of retrying siblings
        // after a failed subtree.
        const greedy = depth > self.lk_backtrack_depth;
        for (self.candidates.row(even)) |odd_next| {
            if (odd_next == t1) continue;
            if (self.vertexInSequence(odd_next, sequence_len)) continue;
            if (self.isTourEdge(even, odd_next)) continue;
            if (self.edgeInList(even, odd_next, self.removed_a, self.removed_b, depth)) continue;
            if (self.edgeInList(even, odd_next, self.added_a, self.added_b, depth - 1)) continue;

            const edge_cost: i64 = @intCast(self.dist.distance(even, odd_next));
            const next_gain = gain - edge_cost;
            if (next_gain <= 0) continue;

            self.added_a[depth - 1] = even;
            self.added_b[depth - 1] = odd_next;
            self.lk_t[sequence_len] = odd_next;
            if (self.searchRemoved(depth + 1, odd_next, next_gain, stats)) return true;
            if (greedy) return false;
        }
        return false;
    }

    fn searchRemoved(self: *LocalSearch, depth: usize, odd: usize, gain: i64, stats: *SolveStats) bool {
        if (!self.recordLKNode(stats)) return false;
        stats.max_depth_reached = @max(stats.max_depth_reached, depth);

        var choices = [2]usize{ self.next[odd], self.prev[odd] };
        self.orderTourEdgeChoices(odd, &choices);
        const sequence_len_before_even = 2 * depth - 1;
        const t1 = self.lk_t[0];
        const greedy = depth > self.lk_backtrack_depth;

        for (choices) |even| {
            if (even == t1) continue;
            if (self.vertexInSequence(even, sequence_len_before_even)) continue;
            if (self.edgeInList(odd, even, self.removed_a, self.removed_b, depth - 1)) continue;
            if (self.edgeInList(odd, even, self.added_a, self.added_b, depth - 1)) continue;

            self.removed_a[depth - 1] = odd;
            self.removed_b[depth - 1] = even;
            self.lk_t[sequence_len_before_even] = even;
            const gain_with_removed = gain + @as(i64, @intCast(self.dist.distance(odd, even)));
            const closing_cost: i64 = @intCast(self.dist.distance(even, t1));
            const closing_gain = gain_with_removed - closing_cost;

            if (closing_gain > 0 and !self.edgeInList(even, t1, self.added_a, self.added_b, depth - 1)) {
                self.added_a[depth - 1] = even;
                self.added_b[depth - 1] = t1;
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

    fn tryLKCompletionOracle(self: *LocalSearch, depth: usize, even: usize, gain: i64, stats: *SolveStats) bool {
        if (depth < 3 or self.lk_nonseq_branch_limit == 0) return false;
        if (depth + 1 <= self.max_lk_depth and self.tryLKCompletion2Opt(depth, even, gain, stats)) return true;
        if (depth + 2 <= self.max_lk_depth and self.tryLKCompletion3Opt(depth, even, gain, stats)) return true;
        return false;
    }

    fn tryLKCompletion2Opt(self: *LocalSearch, depth: usize, even: usize, gain: i64, stats: *SolveStats) bool {
        const t1 = self.lk_t[0];
        var tried: usize = 0;
        for (self.candidates.row(even)) |u| {
            if (tried >= self.lk_nonseq_branch_limit) break;
            if (u == t1 or self.vertexInSequence(u, 2 * depth)) continue;
            if (self.isTourEdge(even, u)) continue;
            if (self.edgeInList(even, u, self.removed_a, self.removed_b, depth)) continue;
            if (self.edgeInList(even, u, self.added_a, self.added_b, depth - 1)) continue;
            if (!self.recordLKNode(stats)) return false;
            stats.lk_completion_attempts += 1;
            tried += 1;

            const after_first_add = gain - @as(i64, @intCast(self.dist.distance(even, u)));
            if (after_first_add <= 0) {
                stats.lk_completion_rejected += 1;
                continue;
            }

            var choices = [2]usize{ self.next[u], self.prev[u] };
            self.orderTourEdgeChoices(u, &choices);
            for (choices) |v| {
                if (v == t1 or self.vertexInSequence(v, 2 * depth)) continue;
                if (self.edgeInList(u, v, self.removed_a, self.removed_b, depth)) continue;
                if (self.edgeInList(u, v, self.added_a, self.added_b, depth - 1)) continue;

                const after_remove = after_first_add + @as(i64, @intCast(self.dist.distance(u, v)));
                if (after_remove <= @as(i64, @intCast(self.dist.distance(v, t1)))) {
                    stats.lk_completion_rejected += 1;
                    continue;
                }
                if (sameUndirectedEdge(v, t1, even, u) or self.edgeInList(v, t1, self.added_a, self.added_b, depth - 1)) {
                    stats.lk_completion_rejected += 1;
                    continue;
                }

                self.added_a[depth - 1] = even;
                self.added_b[depth - 1] = u;
                self.removed_a[depth] = u;
                self.removed_b[depth] = v;
                self.added_a[depth] = v;
                self.added_b[depth] = t1;
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

    fn tryLKCompletion3Opt(self: *LocalSearch, depth: usize, even: usize, gain: i64, stats: *SolveStats) bool {
        const t1 = self.lk_t[0];
        var tried: usize = 0;
        for (self.candidates.row(even)) |u| {
            if (tried >= self.lk_nonseq_branch_limit) break;
            if (u == t1 or self.vertexInSequence(u, 2 * depth)) continue;
            if (self.isTourEdge(even, u)) continue;
            if (self.edgeInList(even, u, self.removed_a, self.removed_b, depth)) continue;
            if (self.edgeInList(even, u, self.added_a, self.added_b, depth - 1)) continue;
            if (!self.recordLKNode(stats)) return false;
            stats.lk_completion_attempts += 1;
            tried += 1;

            const after_first_add = gain - @as(i64, @intCast(self.dist.distance(even, u)));
            if (after_first_add <= 0) {
                stats.lk_completion_rejected += 1;
                continue;
            }

            var first_remove_choices = [2]usize{ self.next[u], self.prev[u] };
            self.orderTourEdgeChoices(u, &first_remove_choices);
            for (first_remove_choices) |v| {
                if (v == t1 or self.vertexInSequence(v, 2 * depth)) continue;
                if (self.edgeInList(u, v, self.removed_a, self.removed_b, depth)) continue;
                if (self.edgeInList(u, v, self.added_a, self.added_b, depth - 1)) continue;

                const after_first_remove = after_first_add + @as(i64, @intCast(self.dist.distance(u, v)));
                if (after_first_remove <= 0) {
                    stats.lk_completion_rejected += 1;
                    continue;
                }

                for (self.candidates.row(v)) |w| {
                    if (w == t1 or w == even or w == u) continue;
                    if (self.vertexInSequence(w, 2 * depth)) continue;
                    if (self.isTourEdge(v, w)) continue;
                    if (self.edgeInList(v, w, self.removed_a, self.removed_b, depth)) continue;
                    if (sameUndirectedEdge(v, w, even, u)) continue;
                    if (self.edgeInList(v, w, self.added_a, self.added_b, depth - 1)) continue;

                    const after_second_add = after_first_remove - @as(i64, @intCast(self.dist.distance(v, w)));
                    if (after_second_add <= 0) continue;

                    var second_remove_choices = [2]usize{ self.next[w], self.prev[w] };
                    self.orderTourEdgeChoices(w, &second_remove_choices);
                    for (second_remove_choices) |x| {
                        if (x == t1 or x == even or x == u or x == v) continue;
                        if (self.vertexInSequence(x, 2 * depth)) continue;
                        if (sameUndirectedEdge(w, x, u, v)) continue;
                        if (self.edgeInList(w, x, self.removed_a, self.removed_b, depth)) continue;
                        if (sameUndirectedEdge(w, x, even, u) or sameUndirectedEdge(w, x, v, w)) continue;
                        if (self.edgeInList(w, x, self.added_a, self.added_b, depth - 1)) continue;

                        const after_second_remove = after_second_add + @as(i64, @intCast(self.dist.distance(w, x)));
                        if (after_second_remove <= @as(i64, @intCast(self.dist.distance(x, t1)))) {
                            stats.lk_completion_rejected += 1;
                            continue;
                        }
                        if (sameUndirectedEdge(x, t1, even, u) or sameUndirectedEdge(x, t1, v, w) or self.edgeInList(x, t1, self.added_a, self.added_b, depth - 1)) {
                            stats.lk_completion_rejected += 1;
                            continue;
                        }

                        self.added_a[depth - 1] = even;
                        self.added_b[depth - 1] = u;
                        self.removed_a[depth] = u;
                        self.removed_b[depth] = v;
                        self.added_a[depth] = v;
                        self.added_b[depth] = w;
                        self.removed_a[depth + 1] = w;
                        self.removed_b[depth + 1] = x;
                        self.added_a[depth + 1] = x;
                        self.added_b[depth + 1] = t1;
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

    fn improveBoundedThreeOptCleanup(self: *LocalSearch, stats: *SolveStats) bool {
        const n = self.tour.len;
        if (n < 6) return false;

        for (0..n) |i| {
            const a = self.tour[i];
            const b = self.tour[(i + 1) % n];
            const ab = @as(u64, self.dist.distance(a, b));

            for (self.candidates.row(a)) |c| {
                const j = self.pos[c];
                if (j <= i + 1 or j + 1 >= n) continue;
                const d = self.tour[(j + 1) % n];
                if (d == a or d == b) continue;
                const cd = @as(u64, self.dist.distance(c, d));

                for (self.candidates.row(d)) |e| {
                    const k = self.pos[e];
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

    fn tryBoundedThreeOptCleanupPattern(
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
        self.removed_a[0] = a;
        self.removed_b[0] = b;
        self.removed_a[1] = c;
        self.removed_b[1] = d;
        self.removed_a[2] = e;
        self.removed_b[2] = f;

        const added_cost: u64 = switch (pattern) {
            .case_a => blk: {
                self.added_a[0] = a;
                self.added_b[0] = c;
                self.added_a[1] = b;
                self.added_b[1] = e;
                self.added_a[2] = d;
                self.added_b[2] = f;
                break :blk self.dist.distance(a, c) + @as(u64, self.dist.distance(b, e)) + self.dist.distance(d, f);
            },
            .case_b => blk: {
                self.added_a[0] = a;
                self.added_b[0] = d;
                self.added_a[1] = e;
                self.added_b[1] = b;
                self.added_a[2] = c;
                self.added_b[2] = f;
                break :blk self.dist.distance(a, d) + @as(u64, self.dist.distance(e, b)) + self.dist.distance(c, f);
            },
            .case_c => blk: {
                self.added_a[0] = a;
                self.added_b[0] = e;
                self.added_a[1] = d;
                self.added_b[1] = b;
                self.added_a[2] = c;
                self.added_b[2] = f;
                break :blk self.dist.distance(a, e) + @as(u64, self.dist.distance(d, b)) + self.dist.distance(c, f);
            },
            .case_d => blk: {
                self.added_a[0] = a;
                self.added_b[0] = c;
                self.added_a[1] = b;
                self.added_b[1] = d;
                self.added_a[2] = e;
                self.added_b[2] = f;
                break :blk self.dist.distance(a, c) + @as(u64, self.dist.distance(b, d)) + self.dist.distance(e, f);
            },
        };

        if (added_cost >= removed_cost) return false;
        return self.testAndApplyMove(3, 3, stats);
    }

    fn testAndApplyMove(self: *LocalSearch, removed_count: usize, added_count: usize, stats: *SolveStats) bool {
        if (self.moveRemovesFrozenEdge(removed_count)) return false;
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

    fn testAndApplyCompletionMove(self: *LocalSearch, removed_count: usize, added_count: usize, stats: *SolveStats) bool {
        if (self.moveRemovesFrozenEdge(removed_count)) return false;
        const patch_hits_before = stats.move_plan_patch_hits;
        if (!self.planAndApplyMoveInternal(removed_count, added_count, stats, true, true, false)) return false;
        stats.lk_applied_depth_total += removed_count;
        stats.lk_deepest_applied_depth = @max(stats.lk_deepest_applied_depth, removed_count);
        if (stats.move_plan_patch_hits > patch_hits_before) stats.lk_completion_patch_hits += 1;
        if (std.debug.runtime_safety) std.debug.assert(self.debugTourIsValid());
        if (std.debug.runtime_safety) std.debug.assert(self.debugSegmentMatchesFlatMaterialization());
        return true;
    }

    fn testAndApplyGain23BridgeMove(self: *LocalSearch, edge_count: usize, stats: *SolveStats) bool {
        if (self.moveRemovesFrozenEdge(edge_count)) return false;
        const patch_hits_before = stats.move_plan_patch_hits;
        if (!self.planAndApplyMoveInternal(edge_count, edge_count, stats, true, true, true)) return false;
        if (stats.move_plan_patch_hits > patch_hits_before) stats.lk_completion_patch_hits += 1;
        const depth = edge_count + 2;
        stats.lk_applied_depth_total += depth;
        stats.lk_deepest_applied_depth = @max(stats.lk_deepest_applied_depth, depth);
        if (std.debug.runtime_safety) std.debug.assert(self.debugTourIsValid());
        if (std.debug.runtime_safety) std.debug.assert(self.debugSegmentMatchesFlatMaterialization());
        return true;
    }

    fn planAndApplyMove(self: *LocalSearch, removed_count: usize, added_count: usize, stats: *SolveStats) bool {
        return self.planAndApplyMoveInternal(removed_count, added_count, stats, false, false, false);
    }

    fn planAndApplyMoveInternal(self: *LocalSearch, removed_count: usize, added_count: usize, stats: *SolveStats, allow_completion_patch: bool, suppress_configured_patch: bool, skip_structurally_impossible_fallback: bool) bool {
        stats.move_plan_attempts += 1;
        for (0..removed_count) |i| {
            self.move_edges[i] = .{ .a = self.removed_a[i], .b = self.removed_b[i] };
        }
        for (0..added_count) |i| {
            self.move_edges[removed_count + i] = .{ .a = self.added_a[i], .b = self.added_b[i] };
        }

        const removed_edges = self.move_edges[0..removed_count];
        const added_edges = self.move_edges[removed_count .. removed_count + added_count];
        var view = self.tourView();
        if (skip_structurally_impossible_fallback and !self.moveDeltaHasValidEdgeSet(&view, removed_edges, added_edges)) return false;
        var plan = MovePlan.init(removed_edges, added_edges);
        @memcpy(self.candidate_tour, self.tour);
        if (!plan.validate(
            &view,
            self.move_degree_delta,
            self.scratch_neighbor0,
            self.scratch_neighbor1,
            self.move_component,
            self.move_component_size,
            self.scratch_seen,
        )) {
            if (skip_structurally_impossible_fallback) return false;
            stats.move_plan_invalid_fallbacks += 1;
            return self.applyMoveWithHamiltonianFallback(removed_count, added_count, stats);
        }
        if (plan.component_count != 1) {
            stats.move_plan_multi_component_fallbacks += 1;
            const patch_min_gain = if (allow_completion_patch) self.lk_completion_patch_min_gain else self.move_patch_min_gain;
            const allow_configured_patch = self.enable_move_patching and !suppress_configured_patch and self.tour.len < 256;
            const allow_patch = allow_configured_patch or allow_completion_patch;
            if (allow_patch and self.tryPatchTwoComponents(&plan, removed_count, added_count, stats, patch_min_gain)) return true;
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

    fn moveDeltaHasValidEdgeSet(self: *LocalSearch, view: *const TourView, removed_edges: []const TourEdge, added_edges: []const TourEdge) bool {
        const n = view.len();
        if (removed_edges.len == 0 or removed_edges.len != added_edges.len) return false;
        @memset(self.move_degree_delta, 0);

        for (removed_edges, 0..) |edge, i| {
            if (!MovePlan.validEdge(edge, n)) return false;
            if (!view.isTourEdge(edge.a, edge.b)) return false;
            if (tourEdgeInSlice(edge, removed_edges[0..i])) return false;
            self.move_degree_delta[edge.a] -= 1;
            self.move_degree_delta[edge.b] -= 1;
        }
        for (added_edges, 0..) |edge, i| {
            if (!MovePlan.validEdge(edge, n)) return false;
            if (view.isTourEdge(edge.a, edge.b)) return false;
            if (tourEdgeInSlice(edge, added_edges[0..i])) return false;
            if (tourEdgeInSlice(edge, removed_edges)) return false;
            self.move_degree_delta[edge.a] += 1;
            self.move_degree_delta[edge.b] += 1;
        }
        for (self.move_degree_delta) |delta| {
            if (delta != 0) return false;
        }
        return true;
    }

    fn tryPatchTwoComponents(self: *LocalSearch, plan: *const MovePlan, removed_count: usize, added_count: usize, stats: *SolveStats, min_gain: i64) bool {
        if (plan.component_count != 2) return false;
        stats.move_plan_patch_attempts += 1;

        const n = self.tour.len;
        var best_cut0: TourEdge = undefined;
        var best_cut1: TourEdge = undefined;
        var best_bridge0: TourEdge = undefined;
        var best_bridge1: TourEdge = undefined;
        var best_gain: i64 = 0;
        const patched_start = removed_count + added_count;
        const patched_removed = self.move_edges[patched_start .. patched_start + removed_count + 2];
        const patched_added = self.move_edges[patched_start + removed_count + 2 .. patched_start + removed_count + added_count + 4];

        // Candidate-row scan over the smaller component only (LKH PatchCycles:
        // in-edges come from candidate sets). The previous exhaustive O(n^2)
        // edge-pair scan for n > 128 dominated total runtime once patching
        // started firing on every nonsequential close.
        const smaller_component = if (self.move_component_size[0] <= self.move_component_size[1]) @as(usize, 0) else @as(usize, 1);
        for (0..n) |a| {
            if (self.move_component[a] != smaller_component) continue;
            const neighbors = [2]usize{ self.scratch_neighbor0[a], self.scratch_neighbor1[a] };
            for (neighbors) |b| {
                if (b == std.math.maxInt(usize) or a > b) continue;
                if (self.move_component[a] != self.move_component[b]) continue;
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
            self.move_degree_delta,
            self.scratch_neighbor0,
            self.scratch_neighbor1,
            self.move_component,
            self.move_component_size,
            self.scratch_seen,
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
            @memcpy(self.tour, self.candidate_tour);
            self.rebuildState();
            stats.move_plan_patch_rejected += 1;
            return false;
        }
        const before_len = self.dist.tourLengthUnchecked(self.candidate_tour) catch {
            @memcpy(self.tour, self.candidate_tour);
            self.rebuildState();
            stats.move_plan_patch_rejected += 1;
            return false;
        };
        const after_len = self.dist.tourLengthUnchecked(self.tour) catch {
            @memcpy(self.tour, self.candidate_tour);
            self.rebuildState();
            stats.move_plan_patch_rejected += 1;
            return false;
        };
        if (after_len >= before_len) {
            @memcpy(self.tour, self.candidate_tour);
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

    fn tryPatchCandidatesFromEndpoint(
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
            if (self.move_component[other] == self.move_component[endpoint]) continue;
            const neighbor_choices = [2]usize{ self.scratch_neighbor0[other], self.scratch_neighbor1[other] };
            for (neighbor_choices) |other_mate| {
                if (other_mate == std.math.maxInt(usize)) continue;
                if (self.move_component[other_mate] != self.move_component[other]) continue;
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

    fn recordPatchCandidate(
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

    fn patchBridgesAreSupported(self: *const LocalSearch, bridge0: TourEdge, bridge1: TourEdge, gain: i64, required_gain: i64) bool {
        const n = self.tour.len;
        if (n < 128) return true;
        const supported = @as(usize, @intFromBool(self.isCandidateEdge(bridge0.a, bridge0.b))) +
            @as(usize, @intFromBool(self.isCandidateEdge(bridge1.a, bridge1.b)));
        if (supported == 2) return true;
        if (supported == 1 and gain >= required_gain * 2) return true;
        return gain >= required_gain * 4;
    }

    fn isCandidateEdge(self: *const LocalSearch, a: usize, b: usize) bool {
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

    fn buildPatchedDelta(
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
            out_removed[removed_len] = .{ .a = self.removed_a[i], .b = self.removed_b[i] };
            removed_len += 1;
        }
        for (0..added_count) |i| {
            out_added[added_len] = .{ .a = self.added_a[i], .b = self.added_b[i] };
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

    fn patchedDeltaGain(
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

    fn applyMoveWithHamiltonianFallback(self: *LocalSearch, removed_count: usize, added_count: usize, stats: *SolveStats) bool {
        @memcpy(self.move_component_size, self.candidate_tour);
        @memcpy(self.tour, self.candidate_tour);
        self.rebuildState();
        if (!self.buildMoveTour(removed_count, added_count, self.candidate_tour)) {
            @memcpy(self.tour, self.move_component_size);
            @memcpy(self.candidate_tour, self.move_component_size);
            self.rebuildState();
            return false;
        }
        @memcpy(self.tour, self.candidate_tour);
        self.rebuildState();
        const valid = !std.debug.runtime_safety or
            (self.debugTourIsValid() and self.debugSegmentMatchesFlatMaterialization());
        if (!valid) {
            @memcpy(self.tour, self.move_component_size);
            @memcpy(self.candidate_tour, self.move_component_size);
            self.rebuildState();
            return false;
        }
        self.applyLengthDeltaArrays(removed_count, added_count);
        self.lkActivateMoveEndpoints(removed_count, added_count);
        stats.move_plan_fallback_successes += 1;
        return true;
    }

    fn applyDepth2ClosingMove(self: *LocalSearch) bool {
        const a = self.removed_a[0];
        const b = self.removed_b[0];
        const c = self.removed_a[1];
        const d = self.removed_b[1];
        if (!self.isTourEdge(a, b) or !self.isTourEdge(c, d)) return false;
        if (!sameUndirectedEdge(self.added_a[0], self.added_b[0], b, c)) return false;
        if (!sameUndirectedEdge(self.added_a[1], self.added_b[1], d, a)) return false;

        // The close adds (b,c) and (d,a). That is only a single-cycle 2-opt when
        // the removed edges face each other, i.e. tour ...a->b ... d->c...; the
        // reversal of the b..d segment then removes exactly {(a,b),(d,c)} and adds
        // exactly {(b,c),(d,a)} — the move whose gain the search verified. The
        // ...a->b ... c->d... orientation closes into two cycles and must fall
        // through to the validating applier instead of being reversed blindly.
        const pb = self.pos[b];
        const pd = self.pos[d];
        if (self.next[a] == b and self.next[d] == c and pb <= pd) {
            self.reverseSegment(pb, pd);
            self.rebuildState();
            return true;
        }
        return false;
    }

    fn debugTourIsValid(self: *LocalSearch) bool {
        @memset(self.scratch_seen, false);
        for (self.tour) |node| {
            if (node >= self.tour.len or self.scratch_seen[node]) return false;
            self.scratch_seen[node] = true;
        }
        for (self.tour, 0..) |node, idx| {
            if (self.next[node] != self.tour[(idx + 1) % self.tour.len]) return false;
            if (self.prev[node] != self.tour[(idx + self.tour.len - 1) % self.tour.len]) return false;
        }
        return true;
    }

    fn debugSegmentMatchesFlatMaterialization(self: *LocalSearch) bool {
        if (!useSegmentTour(self.tour.len)) return true;
        var view = self.tourView();
        // Must not materialize into candidate_tour: callers (tryPatchTwoComponents,
        // planAndApplyMoveInternal) rely on candidate_tour holding the pre-move
        // snapshot for gain comparison and restore-on-reject.
        view.materialize(self.move_component);
        if (!std.mem.eql(usize, self.tour, self.move_component)) return false;

        const n = self.tour.len;
        const size = segmentTargetSize(n);
        var segment_count: usize = 0;
        var start: usize = 0;
        while (start < n) : (segment_count += 1) {
            const len = @min(size, n - start);
            if (self.segment_start[segment_count] != start) return false;
            if (self.segment_len[segment_count] != len) return false;
            if (self.segment_reversed[segment_count]) return false;
            for (0..len) |rank| {
                const node = self.tour[start + rank];
                if (self.segment_of_node[node] != segment_count) return false;
                if (self.rank_in_segment[node] != rank) return false;
            }
            start += len;
        }
        return segment_count > 0;
    }

    fn buildMoveTour(self: *LocalSearch, removed_count: usize, added_count: usize, out: []usize) bool {
        const n = self.tour.len;
        for (0..n) |node| {
            self.scratch_neighbor0[node] = self.prev[node];
            self.scratch_neighbor1[node] = self.next[node];
        }

        for (0..removed_count) |i| {
            if (!self.removeScratchEdge(self.removed_a[i], self.removed_b[i])) return false;
        }
        for (0..added_count) |i| {
            if (!self.addScratchEdge(self.added_a[i], self.added_b[i])) return false;
        }

        @memset(self.scratch_seen, false);
        const start = self.tour[0];
        var previous: usize = std.math.maxInt(usize);
        var current = start;
        for (0..n) |idx| {
            if (self.scratch_seen[current]) return false;
            self.scratch_seen[current] = true;
            out[idx] = current;
            const a = self.scratch_neighbor0[current];
            const b = self.scratch_neighbor1[current];
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

    fn preferredFirstNeighbor(self: *LocalSearch, start: usize, a: usize, b: usize) usize {
        if (self.next[start] == a) return a;
        if (self.next[start] == b) return b;
        return @min(a, b);
    }

    fn removeScratchEdge(self: *LocalSearch, a: usize, b: usize) bool {
        return self.removeScratchNeighbor(a, b) and self.removeScratchNeighbor(b, a);
    }

    fn removeScratchNeighbor(self: *LocalSearch, a: usize, b: usize) bool {
        if (self.scratch_neighbor0[a] == b) {
            self.scratch_neighbor0[a] = std.math.maxInt(usize);
            return true;
        }
        if (self.scratch_neighbor1[a] == b) {
            self.scratch_neighbor1[a] = std.math.maxInt(usize);
            return true;
        }
        return false;
    }

    fn addScratchEdge(self: *LocalSearch, a: usize, b: usize) bool {
        if (a == b) return false;
        if (self.scratch_neighbor0[a] == b or self.scratch_neighbor1[a] == b) return false;
        if (self.scratch_neighbor0[b] == a or self.scratch_neighbor1[b] == a) return false;
        return self.addScratchNeighbor(a, b) and self.addScratchNeighbor(b, a);
    }

    fn addScratchNeighbor(self: *LocalSearch, a: usize, b: usize) bool {
        if (self.scratch_neighbor0[a] == std.math.maxInt(usize)) {
            self.scratch_neighbor0[a] = b;
            return true;
        }
        if (self.scratch_neighbor1[a] == std.math.maxInt(usize)) {
            self.scratch_neighbor1[a] = b;
            return true;
        }
        return false;
    }

    fn improve2Opt(self: *LocalSearch) !bool {
        const n = self.tour.len;
        for (0..n) |i| {
            const a = self.tour[i];
            const b = self.tour[(i + 1) % n];
            const old_ab = self.dist.distance(a, b);

            for (self.candidates.row(a)) |c| {
                const j = self.pos[c];
                if (j <= i + 1) continue;
                if (i == 0 and j == n - 1) continue;
                const d = self.tour[(j + 1) % n];
                if (b == c or a == d) continue;

                const old_cd = self.dist.distance(c, d);
                const new_ac = self.dist.distance(a, c);
                const new_bd = self.dist.distance(b, d);
                if (@as(u64, old_ab) + old_cd > @as(u64, new_ac) + new_bd) {
                    self.reverseSegment(i + 1, j);
                    return true;
                }
            }
        }
        return false;
    }

    fn improveOrOpt1(self: *LocalSearch) !bool {
        const n = self.tour.len;
        if (n < 5) return false;

        for (0..n) |i| {
            const b = self.tour[i];
            const a = self.tour[(i + n - 1) % n];
            const c = self.tour[(i + 1) % n];
            const remove_old = @as(u64, self.dist.distance(a, b)) + self.dist.distance(b, c);
            const remove_new = self.dist.distance(a, c);

            for (self.candidates.row(b)) |x| {
                const j = self.pos[x];
                const y = self.tour[(j + 1) % n];
                if (x == a or x == b or x == c or y == a or y == b) continue;
                if ((j + 1) % n == i) continue;

                const insert_old = self.dist.distance(x, y);
                const insert_new = @as(u64, self.dist.distance(x, b)) + self.dist.distance(b, y);
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

    fn reverseSegment(self: *LocalSearch, first: usize, last: usize) void {
        self.stats.flip_ops += 1;
        self.stats.flip_elements += last - first + 1;
        std.mem.reverse(usize, self.tour[first .. last + 1]);
        for (first..last + 1) |idx| {
            self.pos[self.tour[idx]] = idx;
        }
    }

    fn rebuildPositions(self: *LocalSearch) void {
        for (self.tour, 0..) |node, idx| {
            self.pos[node] = idx;
        }
    }

    fn rebuildState(self: *LocalSearch) void {
        self.stats.tour_rebuilds += 1;
        var view = self.tourView();
        view.rebuild();
    }

    // Roadmap item 2: reseed current_length from a single full scan. Called
    // once per trial after the tour is constructed/kicked/warmed-up, just
    // before the first LK descent; everything after maintains it by delta.
    fn syncLength(self: *LocalSearch) !void {
        self.current_length = try self.dist.tourLengthUnchecked(self.tour);
    }

    // An edge is frozen when BOTH endpoints' counters clear this trial's
    // threshold — mutual high confidence, not one-sided. Only the generator
    // honours it (respect_frozen); the combiner's polish search leaves it off.
    fn edgeFrozen(self: *const LocalSearch, u: usize, v: usize) bool {
        if (!self.respect_frozen) return false;
        return edgeIsFrozenWith(self.vote_node, self.vote_count, self.freeze_threshold, self.inject_frozen, u, v);
    }

    // Soft-freeze frozenness check (no respect_frozen gate — soft freeze prunes
    // search initiation, it does not forbid moves).
    fn softFrozenEdge(self: *const LocalSearch, u: usize, v: usize) bool {
        return edgeIsFrozenWith(self.vote_node, self.vote_count, self.freeze_threshold, self.inject_frozen, u, v);
    }

    // Generator guard: reject any move that would delete a frozen edge. Reuses
    // the existing "move failed" path in the LK search, so a rejected move just
    // makes the search try a different continuation.
    fn moveRemovesFrozenEdge(self: *const LocalSearch, removed_count: usize) bool {
        if (!self.respect_frozen or (self.freeze_threshold == std.math.maxInt(u32) and self.inject_frozen.len == 0)) return false;
        for (0..removed_count) |i| {
            if (self.edgeFrozen(self.removed_a[i], self.removed_b[i])) {
                self.stats.freeze_move_rejections += 1;
                return true;
            }
        }
        return false;
    }

    // Fold an applied move's exact length delta into current_length. The move
    // is read from removed_a/b + added_a/b: the direct-apply path, the
    // Hamiltonian fallback, and the depth-2 closing move all apply exactly that
    // edge set. (Patch moves rewrite the edge set, so they set current_length
    // from their own after-scan instead of calling this.)
    fn applyLengthDeltaArrays(self: *LocalSearch, removed_count: usize, added_count: usize) void {
        var added_sum: u64 = 0;
        for (0..added_count) |i| added_sum += self.dist.distance(self.added_a[i], self.added_b[i]);
        var removed_sum: u64 = 0;
        for (0..removed_count) |i| removed_sum += self.dist.distance(self.removed_a[i], self.removed_b[i]);
        // current_length includes the removed edges, so current_length +
        // added_sum >= removed_sum; no unsigned underflow.
        self.current_length = self.current_length + added_sum - removed_sum;
    }

    fn isTourEdge(self: *const LocalSearch, a: usize, b: usize) bool {
        var view = self.tourView();
        return view.next(a) == b or view.prev(a) == b;
    }

    fn tourView(self: anytype) TourView {
        if (useSegmentTour(self.tour.len)) {
            return TourView.initSegment(
                self.tour,
                self.pos,
                self.next,
                self.prev,
                self.scratch_neighbor0,
                self.scratch_neighbor1,
                self.scratch_seen,
                self.segment_of_node,
                self.rank_in_segment,
                self.segment_start,
                self.segment_len,
                self.segment_reversed,
            );
        }
        return TourView.initFlat(self.tour, self.pos, self.next, self.prev, self.scratch_neighbor0, self.scratch_neighbor1, self.scratch_seen);
    }

    fn vertexInSequence(self: *const LocalSearch, node: usize, len: usize) bool {
        for (self.lk_t[0..len]) |existing| {
            if (existing == node) return true;
        }
        return false;
    }

    fn edgeInList(
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

fn sameUndirectedEdge(a: usize, b: usize, c: usize, d: usize) bool {
    return (a == c and b == d) or (a == d and b == c);
}

test "flat TourView exposes tour primitives" {
    var tour = [_]usize{ 0, 1, 2, 3, 4, 5 };
    var pos: [tour.len]usize = undefined;
    var next: [tour.len]usize = undefined;
    var prev: [tour.len]usize = undefined;
    var scratch0: [tour.len]usize = undefined;
    var scratch1: [tour.len]usize = undefined;
    var seen: [tour.len]bool = undefined;
    var out: [tour.len]usize = undefined;

    var view = TourView.initFlat(&tour, &pos, &next, &prev, &scratch0, &scratch1, &seen);
    view.rebuild();

    try std.testing.expectEqual(@as(usize, 2), view.next(1));
    try std.testing.expectEqual(@as(usize, 0), view.prev(1));
    try std.testing.expect(view.between(1, 3, 5));
    try std.testing.expect(view.between(5, 1, 3));
    try std.testing.expect(!view.between(1, 1, 3));
    try std.testing.expect(!view.between(1, 3, 3));

    view.flipPath(1, 4);
    view.materialize(&out);
    try std.testing.expectEqualSlices(usize, &.{ 0, 4, 3, 2, 1, 5 }, &out);
    try std.testing.expectEqual(@as(usize, 3), view.next(4));
    try std.testing.expectEqual(@as(usize, 0), view.prev(4));
}

test "flat TourView applies edge deltas and rejects invalid edge sets" {
    var tour = [_]usize{ 0, 1, 2, 3, 4, 5 };
    var pos: [tour.len]usize = undefined;
    var next: [tour.len]usize = undefined;
    var prev: [tour.len]usize = undefined;
    var scratch0: [tour.len]usize = undefined;
    var scratch1: [tour.len]usize = undefined;
    var seen: [tour.len]bool = undefined;
    var out: [tour.len]usize = undefined;

    var view = TourView.initFlat(&tour, &pos, &next, &prev, &scratch0, &scratch1, &seen);
    view.rebuild();

    try std.testing.expect(view.applyEdges(
        &.{ .{ .a = 1, .b = 2 }, .{ .a = 4, .b = 5 } },
        &.{ .{ .a = 1, .b = 4 }, .{ .a = 2, .b = 5 } },
    ));
    view.materialize(&out);
    try std.testing.expectEqualSlices(usize, &.{ 0, 1, 4, 3, 2, 5 }, &out);
    try std.testing.expectEqual(@as(usize, 4), view.next(1));
    try std.testing.expectEqual(@as(usize, 2), view.prev(5));

    try std.testing.expect(!view.applyEdges(
        &.{.{ .a = 1, .b = 2 }},
        &.{.{ .a = 1, .b = 1 }},
    ));
}

test "MovePlan validates balanced edge deltas" {
    var tour = [_]usize{ 0, 1, 2, 3, 4, 5 };
    var pos: [tour.len]usize = undefined;
    var next: [tour.len]usize = undefined;
    var prev: [tour.len]usize = undefined;
    var scratch0: [tour.len]usize = undefined;
    var scratch1: [tour.len]usize = undefined;
    var seen: [tour.len]bool = undefined;
    var degree_delta: [tour.len]i8 = undefined;
    var component: [tour.len]usize = undefined;
    var component_size: [tour.len]usize = undefined;
    var view = TourView.initFlat(&tour, &pos, &next, &prev, &scratch0, &scratch1, &seen);
    view.rebuild();

    var valid = MovePlan.init(
        &.{ .{ .a = 1, .b = 2 }, .{ .a = 4, .b = 5 } },
        &.{ .{ .a = 1, .b = 4 }, .{ .a = 2, .b = 5 } },
    );
    try std.testing.expect(valid.validate(&view, &degree_delta, &scratch0, &scratch1, &component, &component_size, &seen));
    try std.testing.expectEqual(@as(usize, 1), valid.component_count);
    try std.testing.expectEqual(@as(usize, tour.len), valid.smallest_component_size);

    var invalid_degree = MovePlan.init(
        &.{.{ .a = 1, .b = 2 }},
        &.{.{ .a = 1, .b = 4 }},
    );
    try std.testing.expect(!invalid_degree.validate(&view, &degree_delta, &scratch0, &scratch1, &component, &component_size, &seen));

    var duplicate_removed = MovePlan.init(
        &.{ .{ .a = 1, .b = 2 }, .{ .a = 2, .b = 1 } },
        &.{ .{ .a = 1, .b = 4 }, .{ .a = 2, .b = 5 } },
    );
    try std.testing.expect(!duplicate_removed.validate(&view, &degree_delta, &scratch0, &scratch1, &component, &component_size, &seen));

    var self_edge = MovePlan.init(
        &.{.{ .a = 1, .b = 2 }},
        &.{.{ .a = 1, .b = 1 }},
    );
    try std.testing.expect(!self_edge.validate(&view, &degree_delta, &scratch0, &scratch1, &component, &component_size, &seen));

    var removes_non_tour_edge = MovePlan.init(
        &.{.{ .a = 1, .b = 3 }},
        &.{.{ .a = 1, .b = 4 }},
    );
    try std.testing.expect(!removes_non_tour_edge.validate(&view, &degree_delta, &scratch0, &scratch1, &component, &component_size, &seen));

    var adds_existing_tour_edge = MovePlan.init(
        &.{.{ .a = 1, .b = 2 }},
        &.{.{ .a = 3, .b = 4 }},
    );
    try std.testing.expect(!adds_existing_tour_edge.validate(&view, &degree_delta, &scratch0, &scratch1, &component, &component_size, &seen));

    var subtours = MovePlan.init(
        &.{ .{ .a = 5, .b = 0 }, .{ .a = 2, .b = 3 } },
        &.{ .{ .a = 0, .b = 2 }, .{ .a = 3, .b = 5 } },
    );
    try std.testing.expect(subtours.validate(&view, &degree_delta, &scratch0, &scratch1, &component, &component_size, &seen));
    try std.testing.expectEqual(@as(usize, 2), subtours.component_count);
    try std.testing.expectEqual(@as(usize, 3), subtours.smallest_component_size);
}

test "MovePlan one-cycle plans apply through TourView" {
    var tour = [_]usize{ 0, 1, 2, 3, 4, 5 };
    var pos: [tour.len]usize = undefined;
    var next: [tour.len]usize = undefined;
    var prev: [tour.len]usize = undefined;
    var scratch0: [tour.len]usize = undefined;
    var scratch1: [tour.len]usize = undefined;
    var seen: [tour.len]bool = undefined;
    var degree_delta: [tour.len]i8 = undefined;
    var component: [tour.len]usize = undefined;
    var component_size: [tour.len]usize = undefined;
    var out: [tour.len]usize = undefined;
    var view = TourView.initFlat(&tour, &pos, &next, &prev, &scratch0, &scratch1, &seen);
    view.rebuild();

    const removed = [_]TourEdge{ .{ .a = 1, .b = 2 }, .{ .a = 4, .b = 5 } };
    const added = [_]TourEdge{ .{ .a = 1, .b = 4 }, .{ .a = 2, .b = 5 } };
    var plan = MovePlan.init(&removed, &added);
    try std.testing.expect(plan.validate(&view, &degree_delta, &scratch0, &scratch1, &component, &component_size, &seen));
    try std.testing.expectEqual(@as(usize, 1), plan.component_count);
    try std.testing.expect(view.applyEdges(&removed, &added));
    view.materialize(&out);
    try std.testing.expectEqualSlices(usize, &.{ 0, 1, 4, 3, 2, 5 }, &out);
}

test "segment TourView agrees with flat backend" {
    var flat_tour = [_]usize{ 0, 1, 2, 3, 4, 5, 6, 7, 8 };
    var segment_tour = flat_tour;
    var flat_pos: [flat_tour.len]usize = undefined;
    var flat_next: [flat_tour.len]usize = undefined;
    var flat_prev: [flat_tour.len]usize = undefined;
    var flat_scratch0: [flat_tour.len]usize = undefined;
    var flat_scratch1: [flat_tour.len]usize = undefined;
    var flat_seen: [flat_tour.len]bool = undefined;
    var segment_pos: [segment_tour.len]usize = undefined;
    var segment_next: [segment_tour.len]usize = undefined;
    var segment_prev: [segment_tour.len]usize = undefined;
    var segment_scratch0: [segment_tour.len]usize = undefined;
    var segment_scratch1: [segment_tour.len]usize = undefined;
    var segment_seen: [segment_tour.len]bool = undefined;
    var segment_of_node: [segment_tour.len]usize = undefined;
    var rank_in_segment: [segment_tour.len]usize = undefined;
    var segment_start: [segment_tour.len]usize = undefined;
    var segment_len: [segment_tour.len]usize = undefined;
    var segment_reversed: [segment_tour.len]bool = undefined;
    var flat_out: [flat_tour.len]usize = undefined;
    var segment_out: [segment_tour.len]usize = undefined;

    var flat = TourView.initFlat(&flat_tour, &flat_pos, &flat_next, &flat_prev, &flat_scratch0, &flat_scratch1, &flat_seen);
    var segment = TourView.initSegment(
        &segment_tour,
        &segment_pos,
        &segment_next,
        &segment_prev,
        &segment_scratch0,
        &segment_scratch1,
        &segment_seen,
        &segment_of_node,
        &rank_in_segment,
        &segment_start,
        &segment_len,
        &segment_reversed,
    );
    flat.rebuild();
    segment.rebuild();

    try std.testing.expectEqual(@as(usize, 3), segmentTargetSize(segment_tour.len));
    try std.testing.expectEqual(@as(usize, 0), segment_of_node[0]);
    try std.testing.expectEqual(@as(usize, 1), segment_of_node[3]);
    try std.testing.expectEqual(@as(usize, 2), segment_of_node[8]);
    try std.testing.expectEqual(@as(usize, 2), rank_in_segment[5]);

    for (0..segment_tour.len) |node| {
        try std.testing.expectEqual(flat.next(node), segment.next(node));
        try std.testing.expectEqual(flat.prev(node), segment.prev(node));
    }
    try std.testing.expectEqual(flat.between(7, 1, 3), segment.between(7, 1, 3));

    flat.flipPath(2, 6);
    segment.flipPath(2, 6);
    flat.materialize(&flat_out);
    segment.materialize(&segment_out);
    try std.testing.expectEqualSlices(usize, &flat_out, &segment_out);

    try std.testing.expect(flat.applyEdges(
        &.{ .{ .a = 1, .b = 6 }, .{ .a = 7, .b = 8 } },
        &.{ .{ .a = 1, .b = 7 }, .{ .a = 6, .b = 8 } },
    ));
    try std.testing.expect(segment.applyEdges(
        &.{ .{ .a = 1, .b = 6 }, .{ .a = 7, .b = 8 } },
        &.{ .{ .a = 1, .b = 7 }, .{ .a = 6, .b = 8 } },
    ));
    flat.materialize(&flat_out);
    segment.materialize(&segment_out);
    try std.testing.expectEqualSlices(usize, &flat_out, &segment_out);
}

test "segment tour backend threshold starts at 512 nodes" {
    try std.testing.expect(!useSegmentTour(511));
    try std.testing.expect(useSegmentTour(512));
}

test "solver is deterministic and finds square optimum through exact path" {
    const allocator = std.testing.allocator;
    const coords = [_]problem.Coord{
        .{ .x = 0, .y = 0 },
        .{ .x = 1, .y = 0 },
        .{ .x = 1, .y = 1 },
        .{ .x = 0, .y = 1 },
    };
    var p = try problem.Problem.initCoords(allocator, "square", .euc_2d, &coords);
    defer p.deinit();

    var a = try solve(allocator, &p, .{ .seed = 7 });
    defer a.deinit();
    var b = try solve(allocator, &p, .{ .seed = 7 });
    defer b.deinit();
    try std.testing.expectEqual(@as(u64, 4), a.length);
    try std.testing.expectEqual(a.length, b.length);
    try std.testing.expectEqualSlices(usize, a.tour, b.tour);
    try std.testing.expectEqual(@as(u64, 6), a.stats.exact_permutations);
}

test "heuristic improves a non-trivial ring-like instance" {
    const allocator = std.testing.allocator;
    const coords = [_]problem.Coord{
        .{ .x = 0, .y = 0 },
        .{ .x = 2, .y = 0 },
        .{ .x = 4, .y = 0 },
        .{ .x = 6, .y = 1 },
        .{ .x = 6, .y = 4 },
        .{ .x = 4, .y = 6 },
        .{ .x = 2, .y = 6 },
        .{ .x = 0, .y = 4 },
        .{ .x = 1, .y = 2 },
        .{ .x = 5, .y = 3 },
        .{ .x = 3, .y = 2 },
    };
    var p = try problem.Problem.initCoords(allocator, "ring", .euc_2d, &coords);
    defer p.deinit();

    var result = try solve(allocator, &p, .{
        .seed = 42,
        .trials = 8,
        .candidate_count = 6,
        .max_passes = 40,
    });
    defer result.deinit();
    try p.validateTour(result.tour);
    try std.testing.expectEqual(result.length, try p.tourLength(result.tour));
    try std.testing.expectEqual(@as(usize, 8), result.stats.trials);
    try std.testing.expect(result.stats.improving_moves >= result.stats.lk_moves);
    try std.testing.expect(result.stats.lk_attempts > 0);
    try std.testing.expect(result.length <= 25);
}

test "heuristic handles explicit max u32 edge weights without sentinel collision" {
    const allocator = std.testing.allocator;
    const n = 11;
    var matrix: [n * n]u32 = undefined;
    for (0..n) |row| {
        for (0..n) |col| {
            matrix[row * n + col] = if (row == col) 0 else std.math.maxInt(u32);
        }
    }
    var p = try problem.Problem.initFullMatrix(allocator, "max-weight", n, &matrix);
    defer p.deinit();

    var result = try solve(allocator, &p, .{
        .seed = 5,
        .trials = 2,
        .candidate_count = 4,
        .max_passes = 2,
    });
    defer result.deinit();

    try p.validateTour(result.tour);
    try std.testing.expectEqual(@as(u64, n) * std.math.maxInt(u32), result.length);
}

test "heuristic scratch uses solve allocator instead of problem allocator" {
    const solve_allocator = std.testing.allocator;
    const coords = [_]problem.Coord{
        .{ .x = 0, .y = 0 },
        .{ .x = 1, .y = 0 },
        .{ .x = 2, .y = 0 },
        .{ .x = 3, .y = 0 },
        .{ .x = 4, .y = 0 },
        .{ .x = 5, .y = 0 },
        .{ .x = 6, .y = 0 },
        .{ .x = 7, .y = 0 },
        .{ .x = 8, .y = 0 },
        .{ .x = 9, .y = 0 },
        .{ .x = 10, .y = 0 },
    };
    var problem_buffer: [512]u8 = undefined;
    var fixed = std.heap.FixedBufferAllocator.init(&problem_buffer);
    const problem_allocator = fixed.allocator();
    var p = try problem.Problem.initCoords(problem_allocator, "split", .euc_2d, &coords);
    defer p.deinit();
    while (true) {
        _ = problem_allocator.alloc(u8, 1) catch break;
    }

    var result = try solve(solve_allocator, &p, .{
        .seed = 1,
        .trials = 2,
        .candidate_count = 4,
        .max_passes = 8,
    });
    defer result.deinit();

    try problem.validateTourWithAllocator(solve_allocator, p.dimension, result.tour);
    try std.testing.expectEqual(@as(usize, 2), result.stats.trials);
}

test "heuristic reaches known convex perimeter optimum" {
    const allocator = std.testing.allocator;
    const coords = [_]problem.Coord{
        .{ .x = 0, .y = 0 },
        .{ .x = 2, .y = 0 },
        .{ .x = 4, .y = 0 },
        .{ .x = 6, .y = 0 },
        .{ .x = 8, .y = 0 },
        .{ .x = 8, .y = 2 },
        .{ .x = 8, .y = 4 },
        .{ .x = 6, .y = 4 },
        .{ .x = 4, .y = 4 },
        .{ .x = 2, .y = 4 },
        .{ .x = 0, .y = 4 },
        .{ .x = 0, .y = 2 },
    };
    var p = try problem.Problem.initCoords(allocator, "perimeter12", .euc_2d, &coords);
    defer p.deinit();

    var result = try solve(allocator, &p, .{
        .seed = 123,
        .trials = 6,
        .candidate_count = 6,
        .max_passes = 30,
    });
    defer result.deinit();
    try p.validateTour(result.tour);
    try std.testing.expectEqual(@as(u64, 24), result.length);
}

test "bounded LK escapes a constructed 2-opt and Or-opt local optimum" {
    const allocator = std.testing.allocator;
    const n = 11;
    const base6 = [_]u32{
        0,  46, 74, 54, 70, 29,
        46, 0,  32, 61, 50, 18,
        74, 32, 0,  51, 25, 32,
        54, 61, 51, 0,  10, 57,
        70, 50, 25, 10, 0,  29,
        29, 18, 32, 57, 29, 0,
    };
    var matrix: [n * n]u32 = undefined;
    for (0..n) |r| {
        for (0..n) |c| {
            if (r < 6 and c < 6) {
                matrix[r * n + c] = base6[r * 6 + c];
            } else if (r == c) {
                matrix[r * n + c] = 0;
            } else if (r >= 5 and c >= 5 and adjacentOnPath(r, c, 5, n - 1)) {
                matrix[r * n + c] = 5;
            } else if ((r == 0 and c == n - 1) or (c == 0 and r == n - 1)) {
                matrix[r * n + c] = 4;
            } else {
                matrix[r * n + c] = 200;
            }
        }
    }
    var p = try problem.Problem.initFullMatrix(allocator, "lk-escape", n, &matrix);
    defer p.deinit();

    var oracle = try DistanceOracle.init(allocator, &p, 0);
    defer oracle.deinit();
    var candidate_stats: CandidateBuildStats = .{};
    var candidates = try buildCandidates(allocator, &oracle, n - 1, .nearest_distance, 0, 0, &candidate_stats);
    defer candidates.deinit();
    var workspace = try SolverWorkspace.init(allocator, n, 5);
    defer workspace.deinit();
    for (workspace.tour, 0..) |*node, i| node.* = i;

    var stats: SolveStats = .{};
    var search = LocalSearch{
        .dist = &oracle,
        .stats = &stats,
        .candidates = &candidates,
        .tour = workspace.tour,
        .pos = workspace.pos,
        .next = workspace.next,
        .prev = workspace.prev,
        .candidate_tour = workspace.candidate_tour,
        .scratch_neighbor0 = workspace.scratch_neighbor0,
        .scratch_neighbor1 = workspace.scratch_neighbor1,
        .scratch_seen = workspace.scratch_seen,
        .segment_of_node = workspace.segment_of_node,
        .rank_in_segment = workspace.rank_in_segment,
        .segment_start = workspace.segment_start,
        .segment_len = workspace.segment_len,
        .segment_reversed = workspace.segment_reversed,
        .move_degree_delta = workspace.move_degree_delta,
        .move_component = workspace.move_component,
        .move_component_size = workspace.move_component_size,
        .move_edges = workspace.move_edges,
        .lk_t = workspace.lk_t,
        .removed_a = workspace.removed_a,
        .removed_b = workspace.removed_b,
        .added_a = workspace.added_a,
        .added_b = workspace.added_b,
        .lk_active = workspace.lk_active,
        .lk_active_queue = workspace.lk_active_queue,
        .max_passes = 40,
        .enable_or_opt = false,
        .enable_bounded_three_opt_cleanup = false,
        .enable_move_patching = false,
        .move_patch_min_gain = 8,
        .lk_completion_patch_min_gain = 24,
        .max_lk_depth = 5,
        .lk_backtrack_limit = 100_000,
        .lk_backtrack_depth = 2,
        .lk_nonseq_branch_limit = 8,
    };
    search.rebuildState();
    try search.syncLength();
    const start_len = try oracle.tourLengthUnchecked(workspace.tour);
    try std.testing.expectEqual(@as(u64, 197), start_len);
    try std.testing.expect(!try search.improve2Opt());
    try std.testing.expect(!try search.improveOrOpt1());

    const lk_moves = try search.improveLK(&stats, true, true);
    const end_len = try oracle.tourLengthUnchecked(workspace.tour);
    try std.testing.expect(lk_moves > 0);
    try std.testing.expect(stats.max_depth_reached >= 3);
    try std.testing.expect(stats.lk_deepest_applied_depth >= 2);
    try std.testing.expect(stats.lk_applied_depth_total >= stats.lk_deepest_applied_depth);
    try std.testing.expect(end_len < start_len);
    try p.validateTour(workspace.tour);
}

fn adjacentOnPath(a: usize, b: usize, first: usize, last: usize) bool {
    return (a >= first and a <= last and b >= first and b <= last and (a + 1 == b or b + 1 == a));
}

test "fixed seed heuristic path is deterministic above brute force cutoff" {
    const allocator = std.testing.allocator;
    const coords = [_]problem.Coord{
        .{ .x = 0, .y = 0 },
        .{ .x = 4, .y = 0 },
        .{ .x = 8, .y = 1 },
        .{ .x = 11, .y = 4 },
        .{ .x = 10, .y = 8 },
        .{ .x = 7, .y = 11 },
        .{ .x = 3, .y = 10 },
        .{ .x = 0, .y = 7 },
        .{ .x = 2, .y = 4 },
        .{ .x = 5, .y = 5 },
        .{ .x = 8, .y = 6 },
        .{ .x = 5, .y = 8 },
    };
    var p = try problem.Problem.initCoords(allocator, "deterministic12", .euc_2d, &coords);
    defer p.deinit();

    const options = SolveOptions{
        .seed = 321,
        .trials = 10,
        .candidate_count = 8,
        .max_passes = 30,
    };
    var a = try solve(allocator, &p, options);
    defer a.deinit();
    var b = try solve(allocator, &p, options);
    defer b.deinit();
    try std.testing.expectEqual(a.length, b.length);
    try std.testing.expectEqualSlices(usize, a.tour, b.tour);
    try std.testing.expectEqual(a.stats.lk_moves, b.stats.lk_moves);
}

test "cached coordinate heuristic path records no uncached coordinate distances" {
    const allocator = std.testing.allocator;
    const coords = [_]problem.Coord{
        .{ .x = 0, .y = 0 },
        .{ .x = 3, .y = 0 },
        .{ .x = 6, .y = 1 },
        .{ .x = 9, .y = 0 },
        .{ .x = 12, .y = 2 },
        .{ .x = 13, .y = 6 },
        .{ .x = 10, .y = 9 },
        .{ .x = 6, .y = 10 },
        .{ .x = 2, .y = 9 },
        .{ .x = 0, .y = 5 },
        .{ .x = 4, .y = 4 },
        .{ .x = 8, .y = 5 },
    };
    var p = try problem.Problem.initCoords(allocator, "cached12", .euc_2d, &coords);
    defer p.deinit();

    var result = try solve(allocator, &p, .{
        .seed = 11,
        .trials = 4,
        .candidate_count = 4,
        .max_passes = 20,
        .max_distance_cache_weights = coords.len * coords.len,
    });
    defer result.deinit();
    try std.testing.expectEqual(@as(usize, coords.len), result.stats.distance_cache_nodes);
    try std.testing.expectEqual(@as(u64, 0), result.stats.uncached_coordinate_distances);
    try p.validateTour(result.tour);
}

test "candidate count is clamped and rows contain no self or duplicates" {
    const allocator = std.testing.allocator;
    const coords = [_]problem.Coord{
        .{ .x = 0, .y = 0 },
        .{ .x = 1, .y = 0 },
        .{ .x = 2, .y = 0 },
        .{ .x = 3, .y = 0 },
        .{ .x = 4, .y = 0 },
        .{ .x = 5, .y = 0 },
        .{ .x = 6, .y = 0 },
        .{ .x = 7, .y = 0 },
        .{ .x = 8, .y = 0 },
        .{ .x = 9, .y = 0 },
        .{ .x = 10, .y = 0 },
    };
    var p = try problem.Problem.initCoords(allocator, "candidates", .euc_2d, &coords);
    defer p.deinit();
    var oracle = try DistanceOracle.init(allocator, &p, coords.len * coords.len);
    defer oracle.deinit();
    var candidate_stats: CandidateBuildStats = .{};
    var candidates = try buildCandidates(allocator, &oracle, candidateWidth(coords.len, 1000), .nearest_distance, 0, 0, &candidate_stats);
    defer candidates.deinit();
    try std.testing.expectEqual(@as(usize, coords.len - 1), candidates.width);
    for (0..coords.len) |node| {
        const row = candidates.row(node);
        for (row, 0..) |candidate, i| {
            try std.testing.expect(candidate != node);
            for (row[0..i]) |previous| try std.testing.expect(candidate != previous);
        }
    }
}

test "alpha-nearness candidates are deterministic and valid" {
    const allocator = std.testing.allocator;
    const coords = [_]problem.Coord{
        .{ .x = 0, .y = 0 },
        .{ .x = 3, .y = 0 },
        .{ .x = 7, .y = 1 },
        .{ .x = 11, .y = 0 },
        .{ .x = 12, .y = 5 },
        .{ .x = 8, .y = 8 },
        .{ .x = 4, .y = 9 },
        .{ .x = 0, .y = 6 },
        .{ .x = 2, .y = 3 },
        .{ .x = 6, .y = 4 },
        .{ .x = 10, .y = 4 },
    };
    var p = try problem.Problem.initCoords(allocator, "alpha-candidates", .euc_2d, &coords);
    defer p.deinit();
    var oracle_a = try DistanceOracle.init(allocator, &p, coords.len * coords.len);
    defer oracle_a.deinit();
    var oracle_b = try DistanceOracle.init(allocator, &p, coords.len * coords.len);
    defer oracle_b.deinit();
    var stats_a: CandidateBuildStats = .{};
    var a = try buildCandidates(allocator, &oracle_a, 5, .alpha_nearness, 32, 2, &stats_a);
    defer a.deinit();
    var stats_b: CandidateBuildStats = .{};
    var b = try buildCandidates(allocator, &oracle_b, 5, .alpha_nearness, 32, 2, &stats_b);
    defer b.deinit();

    try std.testing.expectEqualSlices(usize, a.data, b.data);
    try std.testing.expectEqualSlices(u64, a.alpha, b.alpha);
    for (0..coords.len) |node| {
        const row = a.row(node);
        for (row, 0..) |candidate, i| {
            try std.testing.expect(candidate != node);
            for (row[0..i]) |previous| try std.testing.expect(candidate != previous);
        }
        const alpha_row = a.alphaRow(node);
        for (alpha_row[1..], 1..) |score, i| {
            try std.testing.expect(alpha_row[i - 1] <= score);
        }
    }
}

test "alpha-nearness mode is not worse than nearest mode on deterministic regression" {
    const allocator = std.testing.allocator;
    const n = 80;
    const coords = try allocator.alloc(problem.Coord, n);
    defer allocator.free(coords);
    makeClusteredRegressionCoords(coords);
    var p = try problem.Problem.initCoords(allocator, "alpha-regression", .euc_2d, coords);
    defer p.deinit();

    var nearest = try solve(allocator, &p, .{
        .seed = 12345,
        .trials = 32,
        .candidate_count = 4,
        .candidate_mode = .nearest_distance,
        .max_passes = 80,
        .lk_max_depth = 5,
        .lk_backtrack_limit = 80_000,
        .max_distance_cache_weights = n * n,
    });
    defer nearest.deinit();
    var alpha = try solve(allocator, &p, .{
        .seed = 12345,
        .trials = 32,
        .candidate_count = 4,
        .candidate_mode = .alpha_nearness,
        .max_passes = 80,
        .lk_max_depth = 5,
        .lk_backtrack_limit = 80_000,
        .max_distance_cache_weights = n * n,
    });
    defer alpha.deinit();

    try p.validateTour(nearest.tour);
    try p.validateTour(alpha.tour);
    try std.testing.expect(alpha.stats.alpha_ascent_iterations > 0);
    try std.testing.expect(alpha.stats.alpha_ascent_best_lower_bound > 0);
    try std.testing.expect(nearest.stats.candidate_nearest_edges > 0);
    try std.testing.expectEqual(@as(u64, 0), nearest.stats.candidate_alpha_edges);
    try std.testing.expect(alpha.stats.candidate_alpha_edges > 0);
    try std.testing.expectEqual(@as(u64, 0), alpha.stats.candidate_geometric_edges);
    // Alpha candidates must not be meaningfully worse than plain nearest
    // candidates. Iterated-kick and guided-restart trials make per-seed
    // outcomes a coin flip within ~1 percent, so this guards against broken
    // alpha generation (which shows up as several percent), not basin luck.
    try std.testing.expect(alpha.length * 100 <= nearest.length * 102);
    try std.testing.expect(alpha.stats.bounded_three_opt_cleanup_attempts > 0);
}

test "CGAL candidate mode records geometric edges when enabled" {
    if (!cgal_available) return;

    const allocator = std.testing.allocator;
    const n = 32;
    const coords = try allocator.alloc(problem.Coord, n);
    defer allocator.free(coords);
    makeClusteredRegressionCoords(coords);
    var p = try problem.Problem.initCoords(allocator, "cgal-candidates", .euc_2d, coords);
    defer p.deinit();

    var result = try solve(allocator, &p, .{
        .seed = 12345,
        .trials = 2,
        .candidate_count = 6,
        .candidate_mode = .alpha_nearness_cgal,
        .max_passes = 8,
        .lk_max_depth = 4,
        .lk_backtrack_limit = 10_000,
        .max_distance_cache_weights = n * n,
    });
    defer result.deinit();

    try p.validateTour(result.tour);
    try std.testing.expect(result.stats.candidate_alpha_edges > 0);
    try std.testing.expect(result.stats.candidate_geometric_edges > 0);
}

test "CGAL geometric augmentation stays behind strong alpha candidates" {
    if (!cgal_available) return;

    const allocator = std.testing.allocator;
    const n = 32;
    const coords = try allocator.alloc(problem.Coord, n);
    defer allocator.free(coords);
    makeClusteredRegressionCoords(coords);
    var p = try problem.Problem.initCoords(allocator, "cgal-tail-rescue", .euc_2d, coords);
    defer p.deinit();

    var oracle = try DistanceOracle.init(allocator, &p, n * n);
    defer oracle.deinit();
    var stats: CandidateBuildStats = .{};
    var candidates = try buildCandidates(allocator, &oracle, 6, .alpha_nearness_cgal, 32, 2, &stats);
    defer candidates.deinit();

    try std.testing.expect(stats.geometric_edges > 0);
    for (0..n) |node| {
        const alpha_row = candidates.alphaRow(node);
        try std.testing.expectEqual(@as(u64, 0), alpha_row[0]);
    }
}

fn makeClusteredRegressionCoords(coords: []problem.Coord) void {
    var prng = std.Random.DefaultPrng.init(0xC0FFEE);
    const random = prng.random();
    const centers = [_]problem.Coord{
        .{ .x = 100, .y = 100 },
        .{ .x = 900, .y = 120 },
        .{ .x = 820, .y = 820 },
        .{ .x = 140, .y = 760 },
        .{ .x = 500, .y = 480 },
    };
    for (coords, 0..) |*coord, i| {
        const c = centers[i % centers.len];
        const ring: f64 = @floatFromInt((i * 37) % 53);
        const jitter_x: f64 = @floatFromInt(random.intRangeLessThan(i32, -35, 36));
        const jitter_y: f64 = @floatFromInt(random.intRangeLessThan(i32, -35, 36));
        coord.* = .{
            .x = c.x + ring * 2.7 + jitter_x,
            .y = c.y + @mod(ring * 17.0, 91.0) + jitter_y,
        };
    }
}

test "LK path improves over warmup-only on regression instance" {
    const allocator = std.testing.allocator;
    const coords = [_]problem.Coord{
        .{ .x = 0, .y = 0 },
        .{ .x = 10, .y = 0 },
        .{ .x = 20, .y = 0 },
        .{ .x = 30, .y = 1 },
        .{ .x = 32, .y = 8 },
        .{ .x = 22, .y = 12 },
        .{ .x = 12, .y = 11 },
        .{ .x = 2, .y = 8 },
        .{ .x = 5, .y = 3 },
        .{ .x = 15, .y = 4 },
        .{ .x = 25, .y = 5 },
        .{ .x = 16, .y = 8 },
    };
    var p = try problem.Problem.initCoords(allocator, "comparison12", .euc_2d, &coords);
    defer p.deinit();

    var warmup = try solve(allocator, &p, .{
        .seed = 77,
        .trials = 1,
        .candidate_count = 8,
        .candidate_mode = .nearest_distance,
        .max_passes = 20,
        .enable_lk = false,
    });
    defer warmup.deinit();
    var lk = try solve(allocator, &p, .{
        .seed = 77,
        .trials = 1,
        .candidate_count = 8,
        .candidate_mode = .nearest_distance,
        .max_passes = 20,
        .enable_lk = true,
        .lk_max_depth = 5,
    });
    defer lk.deinit();
    try p.validateTour(warmup.tour);
    try p.validateTour(lk.tour);
    try std.testing.expect(lk.length <= warmup.length);
    try std.testing.expect(lk.stats.lk_attempts > 0);
}

test "heuristic reaches TSPLIB gr17 hardcoded regression target" {
    const allocator = std.testing.allocator;
    const data =
        \\NAME: gr17-full
        \\TYPE: TSP
        \\COMMENT: Hardcoded regression matrix matching TSPLIB gr17, converted from LOWER_DIAG_ROW to FULL_MATRIX; known optimum 2085.
        \\DIMENSION: 17
        \\EDGE_WEIGHT_TYPE: EXPLICIT
        \\EDGE_WEIGHT_FORMAT: FULL_MATRIX
        \\EDGE_WEIGHT_SECTION
        \\0 633 257 91 412 150 80 134 259 505 353 324 70 211 268 246 121
        \\633 0 390 661 227 488 572 530 555 289 282 638 567 466 420 745 518
        \\257 390 0 228 169 112 196 154 372 262 110 437 191 74 53 472 142
        \\91 661 228 0 383 120 77 105 175 476 324 240 27 182 239 237 84
        \\412 227 169 383 0 267 351 309 338 196 61 421 346 243 199 528 297
        \\150 488 112 120 267 0 63 34 264 360 208 329 83 105 123 364 35
        \\80 572 196 77 351 63 0 29 232 444 292 297 47 150 207 332 29
        \\134 530 154 105 309 34 29 0 249 402 250 314 68 108 165 349 36
        \\259 555 372 175 338 264 232 249 0 495 352 95 189 326 383 202 236
        \\505 289 262 476 196 360 444 402 495 0 154 578 439 336 240 685 390
        \\353 282 110 324 61 208 292 250 352 154 0 435 287 184 140 542 238
        \\324 638 437 240 421 329 297 314 95 578 435 0 254 391 448 157 301
        \\70 567 191 27 346 83 47 68 189 439 287 254 0 145 202 289 55
        \\211 466 74 182 243 105 150 108 326 336 184 391 145 0 57 426 96
        \\268 420 53 239 199 123 207 165 383 240 140 448 202 57 0 483 153
        \\246 745 472 237 528 364 332 349 202 685 542 157 289 426 483 0 336
        \\121 518 142 84 297 35 29 36 236 390 238 301 55 96 153 336 0
        \\EOF
    ;
    var p = try tsplib.parse(allocator, data, .{});
    defer p.deinit();

    var result = try solve(allocator, &p, .{
        .seed = 99,
        .trials = 48,
        .candidate_count = 12,
        .max_passes = 120,
    });
    defer result.deinit();

    try p.validateTour(result.tour);
    try std.testing.expectEqual(@as(u64, 2085), result.length);
}

test "EAX merge combines complementary sections from two tours" {
    const allocator = std.testing.allocator;
    var coords: [16]problem.Coord = undefined;
    for (0..16) |i| {
        const angle = 2.0 * std.math.pi * @as(f64, @floatFromInt(i)) / 16.0;
        coords[i] = .{ .x = 100.0 * @cos(angle), .y = 100.0 * @sin(angle) };
    }
    var p = try problem.Problem.initCoords(allocator, "eax-circle", .euc_2d, &coords);
    defer p.deinit();
    var oracle = try DistanceOracle.init(allocator, &p, p.dimension * p.dimension);
    defer oracle.deinit();
    var candidate_stats: CandidateBuildStats = .{};
    var candidates = try buildCandidates(allocator, &oracle, 8, .nearest_distance, 32, 2, &candidate_stats);
    defer candidates.deinit();

    var base: [16]usize = undefined;
    for (0..16) |i| base[i] = i;
    var tour_a = base;
    var tour_b = base;
    // Tour A scrambles one section, tour B a different one; each tour holds
    // the optimal (circle-order) alternative for the other's bad section.
    // Both differences are non-splitting AB-cycles, one improving each parent.
    std.mem.swap(usize, &tour_a[3], &tour_a[4]);
    std.mem.swap(usize, &tour_b[10], &tour_b[11]);

    const len_opt = try oracle.tourLengthUnchecked(&base);
    const len_a = try oracle.tourLengthUnchecked(&tour_a);
    const len_b = try oracle.tourLengthUnchecked(&tour_b);
    try std.testing.expect(len_a > len_opt);
    try std.testing.expect(len_b > len_opt);

    var scratch = try EaxScratch.init(allocator, 16);
    defer scratch.deinit();
    const outcome = eaxMergeTours(&oracle, &candidates, &tour_a, len_a, &tour_b, len_b, true, &scratch);
    try std.testing.expect(outcome.initial_symdiff > 0);
    try std.testing.expect(outcome.length < @min(len_a, len_b));
    try std.testing.expectEqual(len_opt, outcome.length);
    const winner: []const usize = if (outcome.winner_is_a) &tour_a else scratch.tour_b;
    try p.validateTour(winner);
    try std.testing.expectEqual(outcome.length, try oracle.tourLengthUnchecked(winner));
    try std.testing.expectEqual(@as(usize, 2), outcome.cycles_applied);
}

test "EAX merge handles sections traversed in opposite orientation" {
    const allocator = std.testing.allocator;
    var coords: [16]problem.Coord = undefined;
    for (0..16) |i| {
        const angle = 2.0 * std.math.pi * @as(f64, @floatFromInt(i)) / 16.0;
        coords[i] = .{ .x = 100.0 * @cos(angle), .y = 100.0 * @sin(angle) };
    }
    var p = try problem.Problem.initCoords(allocator, "eax-circle-rev", .euc_2d, &coords);
    defer p.deinit();
    var oracle = try DistanceOracle.init(allocator, &p, p.dimension * p.dimension);
    defer oracle.deinit();
    var candidate_stats: CandidateBuildStats = .{};
    var candidates = try buildCandidates(allocator, &oracle, 8, .nearest_distance, 32, 2, &candidate_stats);
    defer candidates.deinit();

    var base: [16]usize = undefined;
    for (0..16) |i| base[i] = i;
    var tour_a = base;
    std.mem.swap(usize, &tour_a[3], &tour_a[4]);
    // Tour B runs the cycle in the opposite global direction (invisible to
    // the undirected symmetric difference) and scrambles a section A holds
    // in optimal order.
    var tour_b: [16]usize = undefined;
    for (0..16) |i| tour_b[i] = 15 - i;
    std.mem.swap(usize, &tour_b[4], &tour_b[5]);

    const len_opt = try oracle.tourLengthUnchecked(&base);
    const len_a = try oracle.tourLengthUnchecked(&tour_a);
    const len_b = try oracle.tourLengthUnchecked(&tour_b);
    try std.testing.expect(len_a > len_opt);
    try std.testing.expect(len_b > len_opt);

    var scratch = try EaxScratch.init(allocator, 16);
    defer scratch.deinit();
    const outcome = eaxMergeTours(&oracle, &candidates, &tour_a, len_a, &tour_b, len_b, true, &scratch);
    try std.testing.expect(outcome.initial_symdiff > 0);
    try std.testing.expect(outcome.length < @min(len_a, len_b));
    try std.testing.expectEqual(len_opt, outcome.length);
    const winner: []const usize = if (outcome.winner_is_a) &tour_a else scratch.tour_b;
    try p.validateTour(winner);
    try std.testing.expectEqual(outcome.length, try oracle.tourLengthUnchecked(winner));
}

test "EAX merge returns null for tours sharing every edge" {
    const allocator = std.testing.allocator;
    var coords: [12]problem.Coord = undefined;
    for (0..12) |i| {
        const angle = 2.0 * std.math.pi * @as(f64, @floatFromInt(i)) / 12.0;
        coords[i] = .{ .x = 100.0 * @cos(angle), .y = 100.0 * @sin(angle) };
    }
    var p = try problem.Problem.initCoords(allocator, "eax-identical", .euc_2d, &coords);
    defer p.deinit();
    var oracle = try DistanceOracle.init(allocator, &p, p.dimension * p.dimension);
    defer oracle.deinit();
    var candidate_stats: CandidateBuildStats = .{};
    var candidates = try buildCandidates(allocator, &oracle, 8, .nearest_distance, 32, 2, &candidate_stats);
    defer candidates.deinit();

    // Same cycle, rotated and reflected: no differing edges, nothing to merge.
    var tour_a: [12]usize = undefined;
    var tour_b: [12]usize = undefined;
    for (0..12) |i| {
        tour_a[i] = (i + 5) % 12;
        tour_b[i] = (12 - i) % 12;
    }
    const len = try oracle.tourLengthUnchecked(&tour_a);

    var scratch = try EaxScratch.init(allocator, 12);
    defer scratch.deinit();
    const outcome = eaxMergeTours(&oracle, &candidates, &tour_a, len, &tour_b, len, true, &scratch);
    try std.testing.expectEqual(@as(usize, 0), outcome.initial_symdiff);
    try std.testing.expectEqual(@as(usize, 0), outcome.cycles_applied);
    try std.testing.expectEqual(len, outcome.length);
}

test "EAX merge repairs a splitting AB-cycle with candidate bridges" {
    const allocator = std.testing.allocator;
    // Two 2x2 clusters. Tour B visits cluster {0,1,6,7} then {4,5,2,3} (two
    // inter-cluster crossings, length 36). Tour A is B with its two interior
    // segments exchanged (a double bridge), length 76. The cheapest AB-cycle
    // applied to A removes two long edges but splits the tour into two
    // 4-node subtours; the candidate-bridge repair must reconnect them. The
    // best bridge cuts both remaining long edges (delta -20), landing on the
    // length-32 optimum — strictly better than either parent.
    const coords = [8]problem.Coord{
        .{ .x = 0, .y = 0 },
        .{ .x = 0, .y = 2 },
        .{ .x = 12, .y = 0 },
        .{ .x = 12, .y = 2 },
        .{ .x = 14, .y = 2 },
        .{ .x = 14, .y = 0 },
        .{ .x = 2, .y = 2 },
        .{ .x = 2, .y = 0 },
    };
    var p = try problem.Problem.initCoords(allocator, "eax-split", .euc_2d, &coords);
    defer p.deinit();
    var oracle = try DistanceOracle.init(allocator, &p, p.dimension * p.dimension);
    defer oracle.deinit();
    var candidate_stats: CandidateBuildStats = .{};
    var candidates = try buildCandidates(allocator, &oracle, 7, .nearest_distance, 32, 2, &candidate_stats);
    defer candidates.deinit();

    var tour_a = [8]usize{ 0, 5, 2, 7, 4, 1, 6, 3 };
    var tour_b = [8]usize{ 0, 1, 6, 7, 4, 5, 2, 3 };
    const len_a = try oracle.tourLengthUnchecked(&tour_a);
    const len_b = try oracle.tourLengthUnchecked(&tour_b);
    try std.testing.expectEqual(@as(u64, 76), len_a);
    try std.testing.expectEqual(@as(u64, 36), len_b);

    var scratch = try EaxScratch.init(allocator, 8);
    defer scratch.deinit();
    const outcome = eaxMergeTours(&oracle, &candidates, &tour_a, len_a, &tour_b, len_b, true, &scratch);
    try std.testing.expect(outcome.initial_symdiff > 0);
    try std.testing.expectEqual(@as(u64, 32), outcome.length);
    // Two applications: the splitting cycle + repair takes A to 32, then a
    // follow-up cycle lifts B to the same tour before the parents converge.
    try std.testing.expectEqual(@as(usize, 2), outcome.cycles_applied);
    try std.testing.expect(outcome.winner_is_a);
    try p.validateTour(&tour_a);
    try std.testing.expectEqual(outcome.length, try oracle.tourLengthUnchecked(&tour_a));
}

test "IPT merge combines complementary sections from two tours" {
    const allocator = std.testing.allocator;
    var coords: [16]problem.Coord = undefined;
    for (0..16) |i| {
        const angle = 2.0 * std.math.pi * @as(f64, @floatFromInt(i)) / 16.0;
        coords[i] = .{ .x = 100.0 * @cos(angle), .y = 100.0 * @sin(angle) };
    }
    var p = try problem.Problem.initCoords(allocator, "ipt-circle", .euc_2d, &coords);
    defer p.deinit();
    var oracle = try DistanceOracle.init(allocator, &p, p.dimension * p.dimension);
    defer oracle.deinit();

    var base: [16]usize = undefined;
    for (0..16) |i| base[i] = i;
    var tour_a = base;
    var tour_b = base;
    // Tour A scrambles one section, tour B a different one; each tour holds
    // the optimal (circle-order) alternative for the other's bad section.
    std.mem.swap(usize, &tour_a[3], &tour_a[4]);
    std.mem.swap(usize, &tour_b[10], &tour_b[11]);

    const len_opt = try oracle.tourLengthUnchecked(&base);
    const len_a = try oracle.tourLengthUnchecked(&tour_a);
    const len_b = try oracle.tourLengthUnchecked(&tour_b);
    try std.testing.expect(len_a > len_opt);
    try std.testing.expect(len_b > len_opt);

    var scratch = try IptScratch.init(allocator, 16);
    defer scratch.deinit();
    const outcome = iptMergeTours(&oracle, &tour_a, len_a, &tour_b, len_b, &scratch) orelse
        return error.MergeDidNotFire;
    try std.testing.expect(outcome.length < @min(len_a, len_b));
    try std.testing.expectEqual(len_opt, outcome.length);
    const winner: []const usize = if (outcome.winner_is_a) &tour_a else scratch.tour_b;
    try p.validateTour(winner);
    try std.testing.expectEqual(outcome.length, try oracle.tourLengthUnchecked(winner));
    try std.testing.expectEqual(@as(usize, 2), outcome.transcriptions);
    try std.testing.expectEqual(@as(usize, 4), outcome.boundary_count);
}

test "IPT merge handles sections traversed in opposite orientation" {
    const allocator = std.testing.allocator;
    var coords: [16]problem.Coord = undefined;
    for (0..16) |i| {
        const angle = 2.0 * std.math.pi * @as(f64, @floatFromInt(i)) / 16.0;
        coords[i] = .{ .x = 100.0 * @cos(angle), .y = 100.0 * @sin(angle) };
    }
    var p = try problem.Problem.initCoords(allocator, "ipt-circle-rev", .euc_2d, &coords);
    defer p.deinit();
    var oracle = try DistanceOracle.init(allocator, &p, p.dimension * p.dimension);
    defer oracle.deinit();

    var base: [16]usize = undefined;
    for (0..16) |i| base[i] = i;
    var tour_a = base;
    std.mem.swap(usize, &tour_a[3], &tour_a[4]);
    // Tour B runs the cycle in the opposite global direction, so every
    // section B contributes appears reversed relative to A; B additionally
    // scrambles a section A holds in optimal order.
    var tour_b: [16]usize = undefined;
    for (0..16) |i| tour_b[i] = 15 - i;
    std.mem.swap(usize, &tour_b[4], &tour_b[5]);

    const len_opt = try oracle.tourLengthUnchecked(&base);
    const len_a = try oracle.tourLengthUnchecked(&tour_a);
    const len_b = try oracle.tourLengthUnchecked(&tour_b);
    try std.testing.expect(len_a > len_opt);
    try std.testing.expect(len_b > len_opt);

    var scratch = try IptScratch.init(allocator, 16);
    defer scratch.deinit();
    const outcome = iptMergeTours(&oracle, &tour_a, len_a, &tour_b, len_b, &scratch) orelse
        return error.MergeDidNotFire;
    try std.testing.expect(outcome.length < @min(len_a, len_b));
    try std.testing.expectEqual(len_opt, outcome.length);
    const winner: []const usize = if (outcome.winner_is_a) &tour_a else scratch.tour_b;
    try p.validateTour(winner);
    try std.testing.expectEqual(outcome.length, try oracle.tourLengthUnchecked(winner));
}

test "IPT merge returns null for tours sharing every edge" {
    const allocator = std.testing.allocator;
    var coords: [12]problem.Coord = undefined;
    for (0..12) |i| {
        const angle = 2.0 * std.math.pi * @as(f64, @floatFromInt(i)) / 12.0;
        coords[i] = .{ .x = 100.0 * @cos(angle), .y = 100.0 * @sin(angle) };
    }
    var p = try problem.Problem.initCoords(allocator, "ipt-identical", .euc_2d, &coords);
    defer p.deinit();
    var oracle = try DistanceOracle.init(allocator, &p, p.dimension * p.dimension);
    defer oracle.deinit();

    // Same cycle, rotated and reflected: no differing edges, nothing to merge.
    var tour_a: [12]usize = undefined;
    var tour_b: [12]usize = undefined;
    for (0..12) |i| {
        tour_a[i] = (i + 5) % 12;
        tour_b[i] = (12 - i) % 12;
    }
    const len = try oracle.tourLengthUnchecked(&tour_a);

    var scratch = try IptScratch.init(allocator, 12);
    defer scratch.deinit();
    try std.testing.expectEqual(
        @as(?IptOutcome, null),
        iptMergeTours(&oracle, &tour_a, len, &tour_b, len, &scratch),
    );
}
