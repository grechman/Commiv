const std = @import("std");
const problem = @import("problem.zig");
const exact = @import("exact.zig");
const tsplib = @import("tsplib.zig");

pub const SolveOptions = struct {
    seed: u64 = 1,
    trials: usize = 16,
    candidate_count: usize = 24,
    max_passes: usize = 80,
    randomized_starts: bool = true,
    enable_or_opt: bool = true,
    enable_lk: bool = true,
    lk_max_depth: usize = 5,
    lk_backtrack_limit: usize = 100_000,
    max_distance_cache_weights: usize = 4_000_000,
};

pub const SolveStats = struct {
    trials: usize = 0,
    warmup_moves: u64 = 0,
    improving_moves: u64 = 0,
    lk_attempts: u64 = 0,
    lk_search_nodes: u64 = 0,
    lk_moves: u64 = 0,
    max_depth_reached: usize = 0,
    exact_permutations: u64 = 0,
    best_trial: usize = 0,
    candidate_count: usize = 0,
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

const Candidates = struct {
    allocator: std.mem.Allocator,
    width: usize,
    data: []usize,

    fn deinit(self: *Candidates) void {
        self.allocator.free(self.data);
        self.* = undefined;
    }

    fn row(self: *const Candidates, node: usize) []const usize {
        const start = node * self.width;
        return self.data[start .. start + self.width];
    }
};

const DistanceOracle = struct {
    allocator: std.mem.Allocator,
    p: *const problem.Problem,
    matrix: []const u32,
    owned_matrix: []u32,
    uncached_coordinate_distances: u64 = 0,

    fn init(
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

    fn deinit(self: *DistanceOracle) void {
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
    lk_t: []usize,
    removed_a: []usize,
    removed_b: []usize,
    added_a: []usize,
    added_b: []usize,

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
        const t_len = std.math.add(usize, std.math.mul(usize, 2, max_lk_depth) catch return error.OutOfMemory, 1) catch return error.OutOfMemory;
        const lk_t = try allocator.alloc(usize, t_len);
        errdefer allocator.free(lk_t);
        const removed_a = try allocator.alloc(usize, max_lk_depth);
        errdefer allocator.free(removed_a);
        const removed_b = try allocator.alloc(usize, max_lk_depth);
        errdefer allocator.free(removed_b);
        const added_a = try allocator.alloc(usize, max_lk_depth);
        errdefer allocator.free(added_a);
        const added_b = try allocator.alloc(usize, max_lk_depth);
        errdefer allocator.free(added_b);

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
            .lk_t = lk_t,
            .removed_a = removed_a,
            .removed_b = removed_b,
            .added_a = added_a,
            .added_b = added_b,
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
        self.allocator.free(self.lk_t);
        self.allocator.free(self.removed_a);
        self.allocator.free(self.removed_b);
        self.allocator.free(self.added_a);
        self.allocator.free(self.added_b);
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
    var candidates = try buildCandidates(allocator, &oracle, width);
    defer candidates.deinit();
    oracle.resetUncachedCounter();

    const max_lk_depth = if (options.enable_lk) @min(@max(options.lk_max_depth, 2), n - 1) else 2;
    var workspace = try SolverWorkspace.init(allocator, n, max_lk_depth);
    defer workspace.deinit();

    var prng = std.Random.DefaultPrng.init(options.seed);
    var random = prng.random();

    var stats = SolveStats{
        .trials = trials,
        .candidate_count = width,
        .distance_cache_nodes = if (oracle.isCached()) n else 0,
        .distance_cache_weights = oracle.matrix.len,
    };
    var best_len: u64 = std.math.maxInt(u64);

    for (0..trials) |trial| {
        nearestNeighborTour(&oracle, &candidates, &random, trial, options.randomized_starts, workspace.tour, workspace.used);
        if (trial > 0 and n >= 8) doubleBridgeKick(workspace.tour, &random);

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
            .lk_t = workspace.lk_t,
            .removed_a = workspace.removed_a,
            .removed_b = workspace.removed_b,
            .added_a = workspace.added_a,
            .added_b = workspace.added_b,
            .max_passes = options.max_passes,
            .enable_or_opt = options.enable_or_opt,
            .max_lk_depth = max_lk_depth,
            .lk_backtrack_limit = options.lk_backtrack_limit,
        };
        search.rebuildState();
        const warmup_moves = try search.improveWarmup();
        stats.warmup_moves += warmup_moves;
        stats.improving_moves += warmup_moves;
        search.rebuildState();

        if (options.enable_lk) {
            const lk_moves = try search.improveLK(&stats);
            stats.improving_moves += lk_moves;
        }

        const len = try oracle.tourLengthUnchecked(workspace.tour);
        if (len < best_len) {
            best_len = len;
            stats.best_trial = trial;
            @memcpy(workspace.best_tour, workspace.tour);
        }
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

fn buildCandidates(allocator: std.mem.Allocator, dist_oracle: *DistanceOracle, width: usize) !Candidates {
    const n = dist_oracle.p.dimension;
    const total_candidates = std.math.mul(usize, n, width) catch return error.OutOfMemory;
    var data = try allocator.alloc(usize, total_candidates);
    errdefer allocator.free(data);
    var dist = try allocator.alloc(u64, width);
    defer allocator.free(dist);

    for (0..n) |i| {
        @memset(dist, std.math.maxInt(u64));
        const row = data[i * width .. i * width + width];
        @memset(row, std.math.maxInt(usize));

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
                }
                dist[k] = d;
                row[k] = j;
            }
        }

        for (row, 0..) |node, k| {
            std.debug.assert(node != i);
            for (row[0..k]) |previous| std.debug.assert(previous != node);
        }
    }

    return .{ .allocator = allocator, .width = width, .data = data };
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

fn doubleBridgeKick(tour: []usize, random: *std.Random) void {
    const n = tour.len;
    if (n < 8) return;
    var a = random.intRangeLessThan(usize, 1, n / 4);
    var b = random.intRangeLessThan(usize, n / 4, n / 2);
    var c = random.intRangeLessThan(usize, n / 2, (3 * n) / 4);
    const d = random.intRangeLessThan(usize, (3 * n) / 4, n - 1);
    if (!(a < b and b < c and c < d)) {
        a = n / 4;
        b = n / 2;
        c = (3 * n) / 4;
    }
    std.mem.reverse(usize, tour[a..b]);
    std.mem.reverse(usize, tour[c..]);
    std.mem.reverse(usize, tour[b..c]);
    std.mem.reverse(usize, tour[d..]);
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
    lk_t: []usize,
    removed_a: []usize,
    removed_b: []usize,
    added_a: []usize,
    added_b: []usize,
    max_passes: usize,
    enable_or_opt: bool,
    max_lk_depth: usize,
    lk_backtrack_limit: usize,
    lk_nodes_this_pass: usize = 0,

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

    fn improveLK(self: *LocalSearch, stats: *SolveStats) !u64 {
        var moves: u64 = 0;
        for (0..self.max_passes) |_| {
            self.lk_nodes_this_pass = 0;
            if (self.findLKMove(stats)) {
                moves += 1;
                stats.lk_moves += 1;
                continue;
            }
            break;
        }
        return moves;
    }

    fn findLKMove(self: *LocalSearch, stats: *SolveStats) bool {
        const n = self.tour.len;
        for (0..n) |idx| {
            const t1 = self.tour[idx];
            var choices = [2]usize{ self.next[t1], self.prev[t1] };
            self.orderTourEdgeChoices(t1, &choices);

            for (choices) |t2| {
                if (self.lk_nodes_this_pass >= self.lk_backtrack_limit) return false;
                stats.lk_attempts += 1;
                self.lk_t[0] = t1;
                self.lk_t[1] = t2;
                self.removed_a[0] = t1;
                self.removed_b[0] = t2;
                const gain: i64 = @intCast(self.dist.distance(t1, t2));
                if (self.searchAdded(1, t2, gain, stats)) return true;
            }
        }
        return false;
    }

    fn orderTourEdgeChoices(self: *LocalSearch, base: usize, choices: *[2]usize) void {
        const d0 = self.dist.distance(base, choices[0]);
        const d1 = self.dist.distance(base, choices[1]);
        if (d1 > d0 or (d1 == d0 and choices[1] < choices[0])) {
            std.mem.swap(usize, &choices[0], &choices[1]);
        }
    }

    fn recordLKNode(self: *LocalSearch, stats: *SolveStats) bool {
        if (self.lk_nodes_this_pass >= self.lk_backtrack_limit) return false;
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
                if (self.testAndApplyMove(depth, depth)) return true;
            }

            if (depth < self.max_lk_depth) {
                if (self.searchAdded(depth, even, gain_with_removed, stats)) return true;
            }
        }
        return false;
    }

    fn testAndApplyMove(self: *LocalSearch, removed_count: usize, added_count: usize) bool {
        if (!self.buildMoveTour(removed_count, added_count, self.candidate_tour)) return false;
        @memcpy(self.tour, self.candidate_tour);
        self.rebuildState();
        return true;
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
        const n = self.tour.len;
        for (self.tour, 0..) |node, idx| {
            self.pos[node] = idx;
            self.prev[node] = self.tour[(idx + n - 1) % n];
            self.next[node] = self.tour[(idx + 1) % n];
        }
    }

    fn isTourEdge(self: *const LocalSearch, a: usize, b: usize) bool {
        return self.next[a] == b or self.prev[a] == b;
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
    var candidates = try buildCandidates(allocator, &oracle, n - 1);
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
        .lk_t = workspace.lk_t,
        .removed_a = workspace.removed_a,
        .removed_b = workspace.removed_b,
        .added_a = workspace.added_a,
        .added_b = workspace.added_b,
        .max_passes = 40,
        .enable_or_opt = false,
        .max_lk_depth = 5,
        .lk_backtrack_limit = 100_000,
    };
    search.rebuildState();
    const start_len = try oracle.tourLengthUnchecked(workspace.tour);
    try std.testing.expectEqual(@as(u64, 197), start_len);
    try std.testing.expect(!try search.improve2Opt());
    try std.testing.expect(!try search.improveOrOpt1());

    var stats: SolveStats = .{};
    const lk_moves = try search.improveLK(&stats);
    const end_len = try oracle.tourLengthUnchecked(workspace.tour);
    try std.testing.expect(lk_moves > 0);
    try std.testing.expect(stats.max_depth_reached >= 3);
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
        .candidate_count = 6,
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
    var candidates = try buildCandidates(allocator, &oracle, candidateWidth(coords.len, 1000));
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
        .max_passes = 20,
        .enable_lk = false,
    });
    defer warmup.deinit();
    var lk = try solve(allocator, &p, .{
        .seed = 77,
        .trials = 1,
        .candidate_count = 8,
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
