const std = @import("std");
const build_options = @import("build_options");
const problem = @import("problem.zig");
const exact = @import("exact.zig");
const tsplib = @import("tsplib.zig");

pub const SolveOptions = struct {
    seed: u64 = 1,
    trials: usize = 16,
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
    lk_nonseq_branch_limit: usize = 2,
    alpha_ascent_iterations: usize = 32,
    alpha_nearest_patch_count: usize = 2,
    max_distance_cache_weights: usize = 4_000_000,
};

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
    ipt_merge_attempts: u64 = 0,
    ipt_merge_transcriptions: u64 = 0,
    ipt_merge_wins: u64 = 0,
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

    fn resetUncachedCounter(self: *DistanceOracle) void {
        self.uncached_coordinate_distances = 0;
    }

    fn distance(self: *DistanceOracle, a: usize, b: usize) u32 {
        if (self.matrix.len != 0) return self.matrix[a * self.p.dimension + b];
        if (self.p.distance_kind != .explicit_full_matrix) self.uncached_coordinate_distances += 1;
        return self.p.distanceUnchecked(a, b);
    }

    fn tourLengthUnchecked(self: *DistanceOracle, tour: []const usize) problem.ProblemError!u64 {
        std.debug.assert(tour.len == self.p.dimension);
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

    fn init(allocator: std.mem.Allocator, n: usize, max_lk_depth: usize) !SolverWorkspace {
        const best_tour = try allocator.alloc(usize, n);
        errdefer allocator.free(best_tour);
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

        return .{
            .allocator = allocator,
            .best_tour = best_tour,
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
        };
    }

    fn deinit(self: *SolverWorkspace) void {
        self.allocator.free(self.best_tour);
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
    oracle.resetUncachedCounter();

    const min_lk_depth: usize = if (options.enable_bounded_three_opt_cleanup) 3 else 2;
    const max_lk_depth = if (options.enable_lk) @min(@max(options.lk_max_depth, min_lk_depth), n - 1) else min_lk_depth;
    var workspace = try SolverWorkspace.init(allocator, n, max_lk_depth);
    defer workspace.deinit();

    var ipt = try IptScratch.init(allocator, n);
    defer ipt.deinit();

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
    var kick_touched: [4][6]usize = undefined;
    var kick_count: usize = 0;
    // Shadow incumbent for IPT tour merging: best tour ever produced by a
    // merge (+ polish). Kept out of the kick/restart loop so the baseline
    // trajectory is undisturbed; folded into the result after the loop.
    var merged_len: u64 = std.math.maxInt(u64);
    for (0..trials) |trial| {
        // After the first descent, trials are iterated local search: perturb the
        // best tour and let LK re-optimize only the perturbed neighborhood,
        // instead of paying for a cold construction + full descent every trial.
        const kick_trial = options.enable_lk and trial > 0 and n >= 8 and
            best_len != std.math.maxInt(u64) and stale_kicks < restart_threshold;
        if (kick_trial) {
            @memcpy(workspace.tour, workspace.best_tour);
            kick_count = @min(1 + stale_kicks / 4, kick_touched.len);
            for (0..kick_count) |ki| {
                segmentExchangeKick(workspace.tour, &random, &kick_touched[ki]);
            }
        } else {
            if (trial > 0 and stale_kicks >= restart_threshold) {
                restart_threshold *= 2;
                stale_kicks = 0;
            }
            if (trial % 4 == 1 and n >= 300) {
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
            .lk_nonseq_branch_limit = options.lk_nonseq_branch_limit,
        };
        search.rebuildState();
        if (kick_trial) {
            search.lkResetActive();
            for (kick_touched[0..kick_count]) |touched| {
                for (touched) |node| search.lkActivate(node);
            }
            const lk_moves = try search.improveLK(&stats, false, false);
            stats.improving_moves += lk_moves;
            // Only tours that beat the incumbent earn the expensive fallback
            // sweeps (Gain23 bridge / 4-opt / bounded 3-opt polish).
            if (try oracle.tourLengthUnchecked(workspace.tour) < best_len) {
                const polish_moves = try search.improveLK(&stats, false, true);
                stats.improving_moves += polish_moves;
            }
        } else {
            const warmup_moves = try search.improveWarmup();
            stats.warmup_moves += warmup_moves;
            stats.improving_moves += warmup_moves;
            search.rebuildState();
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
        if (options.enable_lk and best_len != std.math.maxInt(u64)) {
            const use_merged = merged_len < best_len;
            const ref_tour: []const usize = if (use_merged) ipt.merged else workspace.best_tour;
            const ref_len = if (use_merged) merged_len else best_len;
            const trial_len = try oracle.tourLengthUnchecked(workspace.tour);
            if (trial_len <= ref_len + ref_len / 32) {
                stats.ipt_merge_attempts += 1;
                @memcpy(ipt.tour_a, workspace.tour);
                if (iptMergeTours(&oracle, ipt.tour_a, trial_len, ref_tour, ref_len, &ipt)) |outcome| {
                    stats.ipt_merge_transcriptions += outcome.transcriptions;
                    if (outcome.length < ref_len) {
                        stats.ipt_merge_wins += 1;
                        if (!outcome.winner_is_a) @memcpy(ipt.tour_a, ipt.tour_b);
                        // Re-optimize only the neighborhoods around the
                        // transcribed section boundaries, mirroring the kick
                        // path's light-descent-then-polish pattern. LK is
                        // deterministic (no RNG), so polishing the shadow
                        // tour cannot perturb the main trajectory.
                        var merge_search = search;
                        merge_search.tour = ipt.tour_a;
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
                    }
                }
            }
        }

        const len = try oracle.tourLengthUnchecked(workspace.tour);
        if (len < best_len) {
            best_len = len;
            stats.best_trial = trial;
            @memcpy(workspace.best_tour, workspace.tour);
            stale_kicks = 0;
        } else if (kick_trial) {
            stale_kicks += 1;
        }
    }

    if (merged_len < best_len) {
        best_len = merged_len;
        @memcpy(workspace.best_tour, ipt.merged);
    }

    const result_tour = try allocator.dupe(usize, workspace.best_tour);
    errdefer allocator.free(result_tour);
    stats.uncached_coordinate_distances = oracle.uncached_coordinate_distances;
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

    for (0..n) |i| {
        @memset(row_dist, std.math.maxInt(u64));
        const row = data[i * width .. i * width + width];
        const alpha_row = alpha[i * width .. i * width + width];
        @memset(row, std.math.maxInt(usize));
        @memset(alpha_row, std.math.maxInt(u64));

        for (0..n) |j| {
            if (i == j) continue;
            const d = @as(u64, dist_oracle.distance(i, j));
            const a = alphaScore(dist_oracle, i, j, d, best_pi, best_parent, best_mst_edge, best_root_edges);
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
            const a = alphaScore(dist_oracle, i, patch_node, d, best_pi, best_parent, best_mst_edge, best_root_edges);
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

fn alphaScore(
    dist_oracle: *DistanceOracle,
    a: usize,
    b: usize,
    d: u64,
    pi: []const i64,
    parent: []const usize,
    mst_edge: []const i64,
    root_edges: [2]usize,
) u64 {
    if (treeContainsEdge(a, b, parent, root_edges)) return 0;
    const adjusted = adjustedCost(dist_oracle, pi, a, b);
    const n = dist_oracle.p.dimension;
    if (a == 0 or b == 0) {
        var second: i64 = std.math.maxInt(i64);
        var first: i64 = std.math.maxInt(i64);
        for (1..n) |node| {
            const cost = adjustedCost(dist_oracle, pi, 0, node);
            if (cost < first) {
                second = first;
                first = cost;
            } else if (cost < second) {
                second = cost;
            }
        }
        return positiveAlpha(adjusted, second);
    }
    const bottleneck = maxMstEdgeOnPath(a, b, parent, mst_edge);
    _ = d;
    return positiveAlpha(adjusted, bottleneck);
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

fn maxMstEdgeOnPath(a: usize, b: usize, parent: []const usize, mst_edge: []const i64) i64 {
    var best: i64 = std.math.minInt(i64);
    var x = a;
    while (x != std.math.maxInt(usize)) : (x = parent[x]) {
        var y = b;
        var path_best: i64 = std.math.minInt(i64);
        while (y != std.math.maxInt(usize)) : (y = parent[y]) {
            if (x == y) return @max(best, path_best);
            if (y != 0 and y != std.math.maxInt(usize)) path_best = @max(path_best, mst_edge[y]);
        }
        if (x != 0 and x != std.math.maxInt(usize)) best = @max(best, mst_edge[x]);
    }
    return best;
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
fn segmentExchangeKick(tour: []usize, random: *std.Random, touched: *[6]usize) void {
    const n = tour.len;
    std.debug.assert(n >= 8);
    const i = random.intRangeLessThan(usize, 1, n - 2);
    const j = random.intRangeLessThan(usize, i + 1, n - 1);
    const k = random.intRangeLessThan(usize, j + 1, n);
    touched.* = .{ tour[i - 1], tour[i], tour[j - 1], tour[j], tour[k - 1], tour[k] };
    std.mem.rotate(usize, tour[i..k], j - i);
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

const LocalSearch = struct {
    dist: *DistanceOracle,
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
    lk_nonseq_branch_limit: usize,
    lk_nodes_this_pass: usize = 0,
    lk_active: []bool,
    lk_active_queue: []usize,
    lk_active_head: usize = 0,
    lk_active_count: usize = 0,

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
        if (removed_count == 2 and added_count == 2 and self.applyDepth2ClosingMove()) {
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
        // applyEdges only succeeds after walking a single Hamiltonian cycle and
        // rebuilding; the O(n) re-validation is debug-build paranoia.
        if (std.debug.runtime_safety and (!self.debugTourIsValid() or !self.debugSegmentMatchesFlatMaterialization())) {
            stats.move_plan_apply_fallbacks += 1;
            return self.applyMoveWithHamiltonianFallback(removed_count, added_count, stats);
        }
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
        var view = self.tourView();
        view.rebuild();
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

    var search = LocalSearch{
        .dist = &oracle,
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
        .lk_nonseq_branch_limit = 8,
    };
    search.rebuildState();
    const start_len = try oracle.tourLengthUnchecked(workspace.tour);
    try std.testing.expectEqual(@as(u64, 197), start_len);
    try std.testing.expect(!try search.improve2Opt());
    try std.testing.expect(!try search.improveOrOpt1());

    var stats: SolveStats = .{};
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
    // candidates. Iterated-kick trials make per-seed outcomes a coin flip
    // within a fraction of a percent, so this guards against broken alpha
    // generation (which shows up as several percent), not basin luck.
    try std.testing.expect(alpha.length * 100 <= nearest.length * 101);
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
