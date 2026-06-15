const std = @import("std");
const distance = @import("distance.zig");
const DistanceOracle = distance.DistanceOracle;

pub const CandidateMode = enum {
    nearest_distance,
    alpha_nearness,
};

pub const Candidates = struct {
    allocator: std.mem.Allocator,
    width: usize,
    data: []usize,
    alpha: []u64,
    // Base distance d(node, data[node*width+k]) for each candidate, parallel to
    // `data` (R2). Filled once at build time; the LK inner loop reads this
    // contiguous u32 instead of re-issuing a random-access matrix load per
    // candidate. Holds the BASE distance only (no DistanceOracle penalty, which
    // is dormant); a future penalty caller must bypass this cache.
    cand_dist: []u32,

    pub fn deinit(self: *Candidates) void {
        self.allocator.free(self.data);
        self.allocator.free(self.alpha);
        self.allocator.free(self.cand_dist);
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

    /// Precomputed base distance to the k-th candidate of `node`. Equals
    /// `oracle.distance(node, row(node)[k])` under the null-penalty default.
    pub fn candDist(self: *const Candidates, node: usize, k: usize) u32 {
        return self.cand_dist[node * self.width + k];
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

pub fn candidateWidth(n: usize, requested: usize) usize {
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

    const cand_dist = try allocator.alloc(u32, total_candidates);
    errdefer allocator.free(cand_dist);
    fillCandidateDistances(dist_oracle, data, cand_dist, width);

    candidate_stats.nearest_edges += @as(u64, @intCast(n * width));
    return .{ .allocator = allocator, .width = width, .data = data, .alpha = alpha, .cand_dist = cand_dist };
}

/// Fill the SoA distance cache from the FINAL candidate rows (after any patch
/// or symmetrize reordering), so cand_dist[i*width+k] == d(i, data[i*width+k]).
/// Build-time only; oracle.resetCounters() runs after this, so these lookups do
/// not pollute the per-trial cost counters.
fn fillCandidateDistances(dist_oracle: *DistanceOracle, data: []const usize, cand_dist: []u32, width: usize) void {
    const n = dist_oracle.p.dimension;
    for (0..n) |i| {
        for (0..width) |k| {
            cand_dist[i * width + k] = dist_oracle.distance(i, data[i * width + k]);
        }
    }
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

    const cand_dist = try allocator.alloc(u32, total_candidates);
    errdefer allocator.free(cand_dist);
    fillCandidateDistances(dist_oracle, data, cand_dist, width);

    const total_edges: u64 = @intCast(n * width);
    candidate_stats.alpha_edges += total_edges - candidate_stats.patched_edges;
    candidate_stats.nearest_edges += candidate_stats.patched_edges;
    return .{ .allocator = allocator, .width = width, .data = data, .alpha = alpha, .cand_dist = cand_dist };
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

