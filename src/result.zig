const std = @import("std");

// The canonical result of any tour-producing solver (`solve`, `solveAtsp`,
// `solveAtspNative`, `solveAtspParallel`, `bruteForce`). Lives in its own
// dependency-free module so the exact solver and the asymmetric solver can both
// return it without importing the heuristic core (which imports the exact
// solver) — i.e. without a circular import.

/// Telemetry counters for a solve. No longer embedded in `SolveResult`;
/// populated through the opt-in `solveWithStats` / `bruteForceWithStats`
/// channel (the stats-free `solve` / `solveAtsp*` / `bruteForce` discard them).
/// Most fields are meaningful only for the heuristic search path; the
/// brute-force path records its work in `exact_permutations`.
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
    tour: []usize, // node visit order, length = dimension (directed order for ATSP)
    length: u64, // tour length (true directed length for ATSP)

    pub fn deinit(self: *SolveResult) void {
        self.allocator.free(self.tour);
        self.* = undefined;
    }
};
