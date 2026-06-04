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
    distance_cache_max_nodes: usize = 8192,
};

pub const SolveStats = struct {
    trials: usize = 0,
    improving_moves: u64 = 0,
    best_trial: usize = 0,
    distance_cache_nodes: usize = 0,
};

pub const SolveResult = struct {
    allocator: std.mem.Allocator,
    tour: []usize,
    length: u64,
    iterations: u64 = 0,
    stats: SolveStats = .{},

    pub fn deinit(self: *SolveResult) void {
        self.allocator.free(self.tour);
        self.* = undefined;
    }
};

const SolverError = error{
    DistanceCacheTooLarge,
    NoNearestNeighbor,
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

    fn init(allocator: std.mem.Allocator, p: *const problem.Problem, max_cached_nodes: usize) !DistanceOracle {
        if (p.distance_kind == .explicit_full_matrix) {
            return .{
                .allocator = allocator,
                .p = p,
                .matrix = p.matrix,
                .owned_matrix = &.{},
            };
        }

        if (p.dimension > max_cached_nodes) {
            return .{
                .allocator = allocator,
                .p = p,
                .matrix = &.{},
                .owned_matrix = &.{},
            };
        }

        const total = std.math.mul(usize, p.dimension, p.dimension) catch return SolverError.DistanceCacheTooLarge;
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

    fn distance(self: *const DistanceOracle, a: usize, b: usize) u32 {
        if (self.matrix.len != 0) return self.matrix[a * self.p.dimension + b];
        return self.p.distanceUnchecked(a, b);
    }

    fn tourLengthUnchecked(self: *const DistanceOracle, tour: []const usize) problem.ProblemError!u64 {
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
            .iterations = exact_result.iterations,
            .stats = .{
                .trials = 1,
                .improving_moves = 0,
                .best_trial = 0,
                .distance_cache_nodes = 0,
            },
        };
    }

    const trials = @max(options.trials, 1);
    var oracle = try DistanceOracle.init(allocator, p, options.distance_cache_max_nodes);
    defer oracle.deinit();

    var candidates = try buildCandidates(allocator, &oracle, @min(@max(options.candidate_count, 2), n - 1));
    defer candidates.deinit();

    var prng = std.Random.DefaultPrng.init(options.seed);
    var random = prng.random();

    const best_tour = try allocator.alloc(usize, n);
    errdefer allocator.free(best_tour);
    var best_len: u64 = std.math.maxInt(u64);
    var total_moves: u64 = 0;
    var best_trial: usize = 0;
    const tour = try allocator.alloc(usize, n);
    defer allocator.free(tour);
    const pos = try allocator.alloc(usize, n);
    defer allocator.free(pos);
    const used = try allocator.alloc(bool, n);
    defer allocator.free(used);

    for (0..trials) |trial| {
        try nearestNeighborTour(&oracle, &candidates, &random, trial, options.randomized_starts, tour, used);
        if (trial > 0 and n >= 8) doubleBridgeKick(tour, &random);

        var work = LocalSearch{
            .dist = &oracle,
            .candidates = &candidates,
            .tour = tour,
            .pos = pos,
            .max_passes = options.max_passes,
            .enable_or_opt = options.enable_or_opt,
        };
        work.rebuildPositions();
        const moves = try work.improve();
        total_moves += moves;

        const len = try oracle.tourLengthUnchecked(tour);
        if (len < best_len) {
            best_len = len;
            best_trial = trial;
            @memcpy(best_tour, tour);
        }
    }

    return .{
        .allocator = allocator,
        .tour = best_tour,
        .length = best_len,
        .iterations = total_moves,
        .stats = .{
            .trials = trials,
            .improving_moves = total_moves,
            .best_trial = best_trial,
            .distance_cache_nodes = if (oracle.isCached()) n else 0,
        },
    };
}

fn buildCandidates(allocator: std.mem.Allocator, dist_oracle: *const DistanceOracle, width: usize) !Candidates {
    const n = dist_oracle.p.dimension;
    const total_candidates = std.math.mul(usize, n, width) catch return error.OutOfMemory;
    var data = try allocator.alloc(usize, total_candidates);
    errdefer allocator.free(data);
    var dist = try allocator.alloc(u64, width);
    defer allocator.free(dist);

    for (0..n) |i| {
        @memset(dist, std.math.maxInt(u64));
        const row = data[i * width .. i * width + width];
        @memset(row, i);

        for (0..n) |j| {
            if (i == j) continue;
            const d = @as(u64, dist_oracle.distance(i, j));
            var slot: ?usize = null;
            for (0..width) |k| {
                if (d < dist[k]) {
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
    }

    return .{ .allocator = allocator, .width = width, .data = data };
}

fn nearestNeighborTour(
    dist_oracle: *const DistanceOracle,
    candidates: *const Candidates,
    random: *std.Random,
    trial: usize,
    randomized: bool,
    tour: []usize,
    used: []bool,
) !void {
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
        if (found == 0) return SolverError.NoNearestNeighbor;

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
        if (candidate_dist < best_dist[i]) {
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
    dist: *const DistanceOracle,
    candidates: *const Candidates,
    tour: []usize,
    pos: []usize,
    max_passes: usize,
    enable_or_opt: bool,

    fn improve(self: *LocalSearch) !u64 {
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
};

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
    try std.testing.expectEqual(result.iterations, result.stats.improving_moves);
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

test "heuristic reaches TSPLIB gr17 known optimum through parsed explicit full matrix" {
    const allocator = std.testing.allocator;
    const data =
        \\NAME: gr17-full
        \\TYPE: TSP
        \\COMMENT: TSPLIB gr17 converted from LOWER_DIAG_ROW to FULL_MATRIX; optimum 2085
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
