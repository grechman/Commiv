const std = @import("std");
const problem = @import("problem.zig");
const exact = @import("exact.zig");
const tsplib = @import("tsplib.zig");
const distance = @import("distance.zig");
const tour_mod = @import("tour.zig");
const candidates_mod = @import("candidates.zig");
const construct = @import("construct.zig");
const recombine = @import("recombine.zig");
const search_mod = @import("search.zig");

const SolverError = distance.SolverError;
const LocalSearch = search_mod.LocalSearch;
// Dormant VRP seams (L4): caller-owned additive penalty + caller-owned pinned
// edges. Both default off and are unused by solve(); re-exported so the future
// VRP layer can install them without reaching into the submodules.
pub const PenaltySource = distance.PenaltySource;
pub const PinnedEdges = search_mod.PinnedEdges;
const IptScratch = recombine.IptScratch;
const IptOutcome = recombine.IptOutcome;
const iptMergeTours = recombine.iptMergeTours;
const EaxScratch = recombine.EaxScratch;
const ElitePool = recombine.ElitePool;
const elitePoolOffer = recombine.elitePoolOffer;
const eaxMergeTours = recombine.eaxMergeTours;
pub const DistanceOracle = distance.DistanceOracle;
const nearestNeighborTour = construct.nearestNeighborTour;
const farthestInsertionTour = construct.farthestInsertionTour;
const segmentExchangeKick = construct.segmentExchangeKick;
const plateauKick = construct.plateauKick;
const guidedBackboneTour = construct.guidedBackboneTour;
pub const CandidateMode = candidates_mod.CandidateMode;
pub const Candidates = candidates_mod.Candidates;
pub const CandidateBuildStats = candidates_mod.CandidateBuildStats;
pub const buildCandidates = candidates_mod.buildCandidates;
const candidateWidth = candidates_mod.candidateWidth;
const TourEdge = tour_mod.TourEdge;
const MovePlan = tour_mod.MovePlan;
const tourEdgeInSlice = tour_mod.tourEdgeInSlice;
const removeTourEdgeFromSlice = tour_mod.removeTourEdgeFromSlice;
const TourView = tour_mod.TourView;
const sameUndirectedEdge = tour_mod.sameUndirectedEdge;

pub const SolveOptions = struct {
    seed: u64 = 1,
    budget: Budget = .{},
    candidates: CandidateOptions = .{},
    search: Search = .{},

    // Resource limits: how much work and how much memory the run may spend.
    pub const Budget = struct {
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
        max_passes: usize = 80,
        // Distance-cache budget in BYTES (an L3-sized figure), not a raw weight
        // count. The matrix holds u32 weights, so this default of 16 MB == 4M
        // weights — the historical threshold, exactly. The oracle converts.
        max_distance_cache_bytes: usize = 16_000_000,
    };

    // Candidate-graph construction (1-tree ascent, alpha-nearness, width).
    pub const CandidateOptions = struct {
        candidate_count: usize = 24,
        candidate_mode: CandidateMode = .nearest_distance,
        alpha_ascent_iterations: usize = 32,
        alpha_nearest_patch_count: usize = 2,
        // Sparse (k-NN seeded) alpha build for large geometric instances. Each
        // 1-tree runs over the k-NN graph (O(n*k)) instead of the complete graph
        // (O(n^2)): same candidate quality, far cheaper build (rl11849 ~28s -> ~7s,
        // d18512 ~58s -> ~13s). Gated to n >= sparse_min_dimension so every benched
        // fixture (<= 1577) stays on the bit-identical dense path. Geometric only
        // (needs coordinates). sparse_min_dimension == 0 disables it entirely.
        neighbor_pool_count: usize = 10,
        sparse_ascent_iterations: usize = 100, // 0 => use alpha_ascent_iterations
        sparse_min_dimension: usize = 2000, // 0 => sparse disabled
    };

    // Local-search behaviour: which moves run and how LK explores.
    pub const Search = struct {
        randomized_starts: bool = true,
        enable_or_opt: bool = true,
        enable_lk: bool = true,
        enable_bounded_three_opt_cleanup: bool = true,
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
    };
};


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



pub const SolverWorkspace = struct {
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
        self.* = undefined;
    }
};

// Re-optimize a recombination product in place (M1): the one place the merge
// MECHANISM lives, shared by both size-gated strategies. Copies the trial's
// search state onto `product`, reactivates only the section boundaries the
// merge touched, then runs the kick path's light-descent + polish. LK is
// deterministic (no RNG), so polishing this shadow tour never perturbs the main
// trajectory. Returns the product's exact length. The STRATEGY choice (IPT vs
// EAX) and the shadow/incumbent/pool bookkeeping stay at the two call sites:
// the two recombiners are permanently distinct (see the eax_min_dimension gate),
// so only their common re-optimization tail is folded here.
fn reoptimizeRecombinationProduct(
    search: LocalSearch,
    product: []usize,
    boundary: []const usize,
    stats: *SolveStats,
    oracle: *DistanceOracle,
) !u64 {
    var merge_search = search;
    merge_search.tour = product;
    merge_search.rebuildState();
    merge_search.lkResetActive();
    for (boundary) |node| merge_search.lkActivate(node);
    stats.improving_moves += try merge_search.improveLK(stats, false, false);
    stats.improving_moves += try merge_search.improveLK(stats, false, true);
    return oracle.tourLengthUnchecked(product);
}

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

    const trials = @max(options.budget.trials, 1);
    var oracle = try DistanceOracle.init(allocator, p, options.budget.max_distance_cache_bytes);
    defer oracle.deinit();

    const width = candidateWidth(n, options.candidates.candidate_count);
    var candidate_stats: CandidateBuildStats = .{};
    var candidates = try candidates_mod.buildCandidatesAuto(
        allocator,
        &oracle,
        width,
        options.candidates.candidate_mode,
        options.candidates.alpha_ascent_iterations,
        options.candidates.alpha_nearest_patch_count,
        options.candidates.neighbor_pool_count,
        options.candidates.sparse_ascent_iterations,
        options.candidates.sparse_min_dimension,
        &candidate_stats,
    );
    defer candidates.deinit();
    oracle.resetCounters();

    const min_lk_depth: usize = if (options.search.enable_bounded_three_opt_cleanup) 3 else 2;
    const max_lk_depth = if (options.search.enable_lk) @min(@max(options.search.lk_max_depth, min_lk_depth), n - 1) else min_lk_depth;
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
    // below 22262). This dual strategy is PERMANENT, not transitional (M1):
    // making EAX reproduce the sub-1000 IPT trajectories was measured strictly
    // worse — it reshuffles knife-edge optima, costs ~10% time, and loses
    // lin318/rd400/pcb442/u574-class optima (see HANDOFF do-not-retry). So this
    // is the ONE dispatch point between two permanently-distinct recombiners;
    // their only shared code is reoptimizeRecombinationProduct (the re-opt tail).
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
    const base_backtrack_depth: usize = options.search.lk_backtrack_depth orelse
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
    const max_trials = if (options.budget.trial_extension_factor > 1)
        std.math.mul(usize, trials, options.budget.trial_extension_factor) catch trials
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
        const guided_available = options.search.enable_lk and trial > 0 and
            best_len != std.math.maxInt(u64) and n < guided_max_dimension;
        const restart_limit = if (guided_available) guided_restart_cadence else restart_threshold;
        const kick_trial = options.search.enable_lk and trial > 0 and n >= 8 and
            best_len != std.math.maxInt(u64) and stale_kicks < restart_limit;
        var guided_trial = false;
        if (kick_trial) {
            @memcpy(workspace.tour, workspace.best_tour);
            kick_count = @min(1 + stale_kicks / 4, kick_touched.len);
            for (0..kick_count) |ki| {
                segmentExchangeKick(workspace.tour, &random, &kick_touched[ki]);
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
                nearestNeighborTour(&oracle, &candidates, &random, trial, options.search.randomized_starts, workspace.tour, workspace.used);
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
            .ws = &workspace,
            .max_passes = options.budget.max_passes,
            .enable_or_opt = options.search.enable_or_opt,
            .enable_bounded_three_opt_cleanup = options.search.enable_bounded_three_opt_cleanup,
            .lk_completion_patch_min_gain = options.search.lk_completion_patch_min_gain,
            .max_lk_depth = max_lk_depth,
            .lk_backtrack_limit = options.search.lk_backtrack_limit,
            .lk_backtrack_depth = if (trial >= trials) @min(base_backtrack_depth, 2) else base_backtrack_depth,
            .lk_nonseq_branch_limit = options.search.lk_nonseq_branch_limit,
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
            if (options.search.enable_lk) {
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
        if (n < eax_min_dimension and options.search.enable_lk and best_len != std.math.maxInt(u64)) {
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
                        const merged_now = try reoptimizeRecombinationProduct(search, ipt.tour_a, ipt.boundary[0..outcome.boundary_count], &stats, &oracle);
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
        if (n >= eax_min_dimension and options.search.enable_lk and best_len != std.math.maxInt(u64)) {
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
                        const merged_now = try reoptimizeRecombinationProduct(search, eax.tour_a, eax.boundary[0..outcome.boundary_count], &stats, &oracle);
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
            if (n >= eax_min_dimension and options.search.enable_lk) elitePoolOffer(&elite, &eax, workspace.tour, len);
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
        .budget = .{ .trials = 8, .max_passes = 40 },
        .candidates = .{ .candidate_count = 6 },
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
        .budget = .{ .trials = 2, .max_passes = 2 },
        .candidates = .{ .candidate_count = 4 },
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
        .budget = .{ .trials = 2, .max_passes = 8 },
        .candidates = .{ .candidate_count = 4 },
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
        .budget = .{ .trials = 6, .max_passes = 30 },
        .candidates = .{ .candidate_count = 6 },
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
        .ws = &workspace,
        .max_passes = 40,
        .enable_or_opt = false,
        .enable_bounded_three_opt_cleanup = false,
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
        .budget = .{ .trials = 10, .max_passes = 30 },
        .candidates = .{ .candidate_count = 8 },
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
        .budget = .{ .trials = 4, .max_passes = 20, .max_distance_cache_bytes = coords.len * coords.len * @sizeOf(u32) },
        .candidates = .{ .candidate_count = 4 },
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
    var oracle = try DistanceOracle.init(allocator, &p, coords.len * coords.len * @sizeOf(u32));
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
    var oracle_a = try DistanceOracle.init(allocator, &p, coords.len * coords.len * @sizeOf(u32));
    defer oracle_a.deinit();
    var oracle_b = try DistanceOracle.init(allocator, &p, coords.len * coords.len * @sizeOf(u32));
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
        .budget = .{ .trials = 32, .max_passes = 80, .max_distance_cache_bytes = n * n * @sizeOf(u32) },
        .candidates = .{ .candidate_count = 4, .candidate_mode = .nearest_distance },
        .search = .{ .lk_max_depth = 5, .lk_backtrack_limit = 80_000 },
    });
    defer nearest.deinit();
    var alpha = try solve(allocator, &p, .{
        .seed = 12345,
        .budget = .{ .trials = 32, .max_passes = 80, .max_distance_cache_bytes = n * n * @sizeOf(u32) },
        .candidates = .{ .candidate_count = 4, .candidate_mode = .alpha_nearness },
        .search = .{ .lk_max_depth = 5, .lk_backtrack_limit = 80_000 },
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
        .budget = .{ .trials = 1, .max_passes = 20 },
        .candidates = .{ .candidate_count = 8, .candidate_mode = .nearest_distance },
        .search = .{ .enable_lk = false },
    });
    defer warmup.deinit();
    var lk = try solve(allocator, &p, .{
        .seed = 77,
        .budget = .{ .trials = 1, .max_passes = 20 },
        .candidates = .{ .candidate_count = 8, .candidate_mode = .nearest_distance },
        .search = .{ .enable_lk = true, .lk_max_depth = 5 },
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
        .budget = .{ .trials = 48, .max_passes = 120 },
        .candidates = .{ .candidate_count = 12 },
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
    var oracle = try DistanceOracle.init(allocator, &p, p.dimension * p.dimension * @sizeOf(u32));
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
    var oracle = try DistanceOracle.init(allocator, &p, p.dimension * p.dimension * @sizeOf(u32));
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
    var oracle = try DistanceOracle.init(allocator, &p, p.dimension * p.dimension * @sizeOf(u32));
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
    var oracle = try DistanceOracle.init(allocator, &p, p.dimension * p.dimension * @sizeOf(u32));
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
    var oracle = try DistanceOracle.init(allocator, &p, p.dimension * p.dimension * @sizeOf(u32));
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
    var oracle = try DistanceOracle.init(allocator, &p, p.dimension * p.dimension * @sizeOf(u32));
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
    var oracle = try DistanceOracle.init(allocator, &p, p.dimension * p.dimension * @sizeOf(u32));
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

test "distance oracle applies caller-owned penalty and saturates" {
    const allocator = std.testing.allocator;
    var matrix = [_]u32{
        0,  10, 20,
        10, 0,  30,
        20, 30, 0,
    };
    var p = try problem.Problem.initFullMatrix(allocator, "penalty3", 3, &matrix);
    defer p.deinit();
    var oracle = try DistanceOracle.init(allocator, &p, 0);
    defer oracle.deinit();

    // Null source (the default the whole solve path uses): base distances.
    try std.testing.expectEqual(@as(u32, 10), oracle.distance(0, 1));
    try std.testing.expectEqual(@as(u32, 30), oracle.distance(1, 2));

    // Type-erased context carries the additive penalty added on top of base.
    const Ctx = struct { amount: u32 };
    const Pen = struct {
        fn pen(ctx: *const anyopaque, _: usize, _: usize) u32 {
            const c: *const Ctx = @ptrCast(@alignCast(ctx));
            return c.amount;
        }
    };
    var ctx = Ctx{ .amount = 7 };
    var source = PenaltySource{ .ctx = &ctx, .penaltyFn = Pen.pen };
    oracle.penalty_source = &source;
    try std.testing.expectEqual(@as(u32, 17), oracle.distance(0, 1));
    try std.testing.expectEqual(@as(u32, 37), oracle.distance(1, 2));

    // Penalties saturate into u32 rather than wrapping past the max.
    ctx.amount = std.math.maxInt(u32);
    try std.testing.expectEqual(@as(u32, std.math.maxInt(u32)), oracle.distance(0, 1));

    // Clearing the source restores the bit-identical base path.
    oracle.penalty_source = null;
    try std.testing.expectEqual(@as(u32, 10), oracle.distance(0, 1));
}

test "pinned-edge seam evaluates caller predicate undirected" {
    const Ctx = struct { a: usize, b: usize };
    const Pred = struct {
        fn isPinned(ctx: *const anyopaque, a: usize, b: usize) bool {
            const c: *const Ctx = @ptrCast(@alignCast(ctx));
            return (a == c.a and b == c.b) or (a == c.b and b == c.a);
        }
    };
    var ctx = Ctx{ .a = 2, .b = 5 };
    const pinned = PinnedEdges{ .ctx = &ctx, .isPinnedFn = Pred.isPinned };
    try std.testing.expect(pinned.isPinned(2, 5));
    try std.testing.expect(pinned.isPinned(5, 2));
    try std.testing.expect(!pinned.isPinned(2, 3));
    try std.testing.expect(!pinned.isPinned(0, 1));
}
