const std = @import("std");
const problem = @import("problem.zig");
const Coord = problem.Coord;

// Uniform-grid k-nearest-neighbor index for coordinate (geometric) instances.
// Used to seed the sparse 1-tree / alpha-nearness candidate build at large n,
// turning the O(n^2) neighbor scan into ~O(n*k). Ranking is by squared
// Euclidean distance, which is order-identical to every geometric distance kind
// commiv parses (euc_2d/ceil_2d round monotonically; att scales monotonically),
// so the k-nearest set is exact for candidate seeding regardless of metric.
//
// Zero dependencies. No spatial structure exists for explicit_full_matrix
// instances (no coordinates); callers fall back to the dense build there.

/// Directed k-nearest lists in CSR form, nearest-first. Self excluded.
pub const KnnGraph = struct {
    allocator: std.mem.Allocator,
    n: usize,
    k: usize,
    start: []usize, // len n+1
    node: []usize, // len n*k

    pub fn deinit(self: *KnnGraph) void {
        self.allocator.free(self.start);
        self.allocator.free(self.node);
        self.* = undefined;
    }

    pub fn row(self: *const KnnGraph, i: usize) []const usize {
        return self.node[self.start[i] .. self.start[i + 1]];
    }
};

fn sqDist(a: Coord, b: Coord) f64 {
    const dx = a.x - b.x;
    const dy = a.y - b.y;
    return dx * dx + dy * dy;
}

/// Build the k-nearest-neighbor graph over `coords` using a uniform grid.
/// k is clamped to [1, n-1]. Each row holds exactly k neighbors, nearest-first.
pub fn buildKnn(allocator: std.mem.Allocator, coords: []const Coord, k_req: usize) !KnnGraph {
    const n = coords.len;
    std.debug.assert(n >= 2);
    const k = @max(@min(k_req, n - 1), 1);

    // Bounding box.
    var min_x: f64 = coords[0].x;
    var max_x: f64 = coords[0].x;
    var min_y: f64 = coords[0].y;
    var max_y: f64 = coords[0].y;
    for (coords) |c| {
        min_x = @min(min_x, c.x);
        max_x = @max(max_x, c.x);
        min_y = @min(min_y, c.y);
        max_y = @max(max_y, c.y);
    }
    const width = max_x - min_x;
    const height = max_y - min_y;

    // Grid dimensions: aim for ~2 points/cell, proportioned to the bbox, with
    // total cells bounded by ~n (so memory stays O(n)). Degenerate axes -> 1.
    const target_cells: f64 = @max(@as(f64, @floatFromInt(n)) / 2.0, 1.0);
    const nf: f64 = @floatFromInt(n);
    var gx: usize = 1;
    var gy: usize = 1;
    if (width > 0 and height > 0) {
        const cell = @sqrt(width * height / target_cells);
        gx = clampDim(width / cell + 1.0, n);
        gy = clampDim(height / cell + 1.0, n);
        // Bound total cells to ~n.
        while (@as(f64, @floatFromInt(gx)) * @as(f64, @floatFromInt(gy)) > nf and (gx > 1 or gy > 1)) {
            if (gx >= gy and gx > 1) gx -= gx / 2 + 1 else if (gy > 1) gy -= gy / 2 + 1 else break;
        }
    } else if (width > 0) {
        gx = clampDim(target_cells, n);
    } else if (height > 0) {
        gy = clampDim(target_cells, n);
    }
    const cell_w = if (width > 0) width / @as(f64, @floatFromInt(gx)) else 1.0;
    const cell_h = if (height > 0) height / @as(f64, @floatFromInt(gy)) else 1.0;
    const cell_min = @min(cell_w, cell_h);
    const num_cells = gx * gy;

    const cellOf = struct {
        fn idx(c: Coord, mnx: f64, mny: f64, cw: f64, ch: f64, gxx: usize, gyy: usize) usize {
            const cx = clampIdx((c.x - mnx) / cw, gxx);
            const cy = clampIdx((c.y - mny) / ch, gyy);
            return cy * gxx + cx;
        }
    }.idx;

    // Bucket nodes into cells (CSR by cell).
    const cell_start = try allocator.alloc(usize, num_cells + 1);
    defer allocator.free(cell_start);
    @memset(cell_start, 0);
    for (coords) |c| {
        const ci = cellOf(c, min_x, min_y, cell_w, cell_h, gx, gy);
        cell_start[ci + 1] += 1;
    }
    for (1..num_cells + 1) |i| cell_start[i] += cell_start[i - 1];
    const cell_node = try allocator.alloc(usize, n);
    defer allocator.free(cell_node);
    const cursor = try allocator.alloc(usize, num_cells);
    defer allocator.free(cursor);
    @memcpy(cursor, cell_start[0..num_cells]);
    for (coords, 0..) |c, i| {
        const ci = cellOf(c, min_x, min_y, cell_w, cell_h, gx, gy);
        cell_node[cursor[ci]] = i;
        cursor[ci] += 1;
    }

    // Output CSR.
    const start = try allocator.alloc(usize, n + 1);
    errdefer allocator.free(start);
    for (0..n + 1) |i| start[i] = i * k;
    const node = try allocator.alloc(usize, n * k);
    errdefer allocator.free(node);

    // Per-query k-best scratch (insertion-sorted, ascending by sq-dist).
    const best_node = try allocator.alloc(usize, k);
    defer allocator.free(best_node);
    const best_d = try allocator.alloc(f64, k);
    defer allocator.free(best_d);

    const max_ring = gx + gy;
    for (0..n) |i| {
        const here = coords[i];
        const cx = clampIdx((here.x - min_x) / cell_w, gx);
        const cy = clampIdx((here.y - min_y) / cell_h, gy);
        var count: usize = 0;

        var r: usize = 0;
        while (r <= max_ring) : (r += 1) {
            // Visit every cell at Chebyshev ring r around (cx,cy).
            const x_lo = if (cx >= r) cx - r else 0;
            const x_hi = @min(cx + r, gx - 1);
            const y_lo = if (cy >= r) cy - r else 0;
            const y_hi = @min(cy + r, gy - 1);
            var yy = y_lo;
            while (yy <= y_hi) : (yy += 1) {
                const on_y_edge = (yy == cy + r) or (cy >= r and yy == cy - r);
                var xx = x_lo;
                while (xx <= x_hi) : (xx += 1) {
                    // Ring r = cells with Chebyshev distance exactly r.
                    const on_x_edge = (xx == cx + r) or (cx >= r and xx == cx - r);
                    if (r != 0 and !on_x_edge and !on_y_edge) continue;
                    const ci = yy * gx + xx;
                    for (cell_node[cell_start[ci]..cell_start[ci + 1]]) |j| {
                        if (j == i) continue;
                        const d = sqDist(here, coords[j]);
                        count = insertBest(best_node, best_d, count, k, j, d);
                    }
                }
            }
            // Stop once k found and ring r+1 cannot hold anything closer than the
            // current k-th best: nearest point in ring (r+1) is >= r*cell_min away.
            if (count >= k and cell_min > 0) {
                const bound = @as(f64, @floatFromInt(r)) * cell_min;
                if (best_d[k - 1] <= bound * bound) break;
            }
        }

        // Fewer than k reachable only if n-1 < k (excluded by clamp); pad defensively.
        const base = i * k;
        for (0..k) |s| node[base + s] = if (s < count) best_node[s] else best_node[if (count > 0) count - 1 else 0];
    }

    return .{ .allocator = allocator, .n = n, .k = k, .start = start, .node = node };
}

/// Insert (j,d) into the ascending k-best arrays; returns the new count.
fn insertBest(best_node: []usize, best_d: []f64, count: usize, k: usize, j: usize, d: f64) usize {
    if (count == k and d >= best_d[k - 1]) return count;
    var pos = @min(count, k - 1);
    // Shift larger entries right.
    while (pos > 0 and best_d[pos - 1] > d) : (pos -= 1) {
        best_d[pos] = best_d[pos - 1];
        best_node[pos] = best_node[pos - 1];
    }
    best_d[pos] = d;
    best_node[pos] = j;
    return @min(count + 1, k);
}

fn clampDim(v: f64, n: usize) usize {
    if (v < 1.0) return 1;
    const iv: usize = @intFromFloat(v);
    return @max(@min(iv, n), 1);
}

fn clampIdx(v: f64, dim: usize) usize {
    if (v < 0) return 0;
    const iv: usize = @intFromFloat(v);
    return @min(iv, dim - 1);
}

test "buildKnn matches brute-force nearest sets" {
    const allocator = std.testing.allocator;
    var prng = std.Random.DefaultPrng.init(0xC0FFEE);
    const rnd = prng.random();
    const n = 400;
    const k = 8;
    const coords = try allocator.alloc(Coord, n);
    defer allocator.free(coords);
    for (coords) |*c| c.* = .{ .x = rnd.float(f64) * 1000.0, .y = rnd.float(f64) * 1000.0 };

    var knn = try buildKnn(allocator, coords, k);
    defer knn.deinit();

    // Brute-force k-nearest for each node; the grid set must match as a SET
    // (ties at the k-boundary may differ in identity but distances must match).
    const bd = try allocator.alloc(f64, n);
    defer allocator.free(bd);
    for (0..n) |i| {
        for (0..n) |j| bd[j] = if (i == j) std.math.inf(f64) else sqDist(coords[i], coords[j]);
        // k-th smallest distance via selection.
        var kth: f64 = 0;
        {
            const tmp = try allocator.alloc(f64, n);
            defer allocator.free(tmp);
            @memcpy(tmp, bd);
            std.mem.sort(f64, tmp, {}, std.sort.asc(f64));
            kth = tmp[k - 1];
        }
        const grid_row = knn.row(i);
        try std.testing.expectEqual(@as(usize, k), grid_row.len);
        // Every returned neighbor must be within the true k-th distance.
        for (grid_row) |nb| {
            try std.testing.expect(nb != i);
            try std.testing.expect(sqDist(coords[i], coords[nb]) <= kth + 1e-6);
        }
    }
}

test "buildKnn degenerate collinear points" {
    const allocator = std.testing.allocator;
    const n = 50;
    const coords = try allocator.alloc(Coord, n);
    defer allocator.free(coords);
    for (coords, 0..) |*c, i| c.* = .{ .x = @floatFromInt(i), .y = 7.0 };
    var knn = try buildKnn(allocator, coords, 5);
    defer knn.deinit();
    // node 0's nearest must be node 1 (distance 1).
    try std.testing.expectEqual(@as(usize, 1), knn.row(0)[0]);
}
