const std = @import("std");
const problem = @import("problem.zig");
const exact = @import("exact.zig");

pub const SolveOptions = struct {
    seed: u64 = 1,
    trials: usize = 16,
    candidate_count: usize = 24,
    max_passes: usize = 80,
    randomized_starts: bool = true,
    enable_or_opt: bool = true,
};

pub const SolveStats = struct {
    trials: usize = 0,
    improving_moves: u64 = 0,
    best_trial: usize = 0,
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

pub fn solve(
    allocator: std.mem.Allocator,
    p: *const problem.Problem,
    options: SolveOptions,
) !problem.TourResult {
    const n = p.dimension;
    if (n <= 10) {
        return exact.bruteForce(allocator, p, .{ .max_nodes = 10 }) catch |err| switch (err) {
            error.InstanceTooLarge => unreachable,
            else => |e| return e,
        };
    }

    const trials = @max(options.trials, 1);
    var candidates = try buildCandidates(allocator, p, @min(@max(options.candidate_count, 2), n - 1));
    defer candidates.deinit();

    var prng = std.Random.DefaultPrng.init(options.seed);
    var random = prng.random();

    const best_tour = try allocator.alloc(usize, n);
    errdefer allocator.free(best_tour);
    var best_len: u64 = std.math.maxInt(u64);
    var total_moves: u64 = 0;
    var best_trial: usize = 0;

    for (0..trials) |trial| {
        const tour = try allocator.alloc(usize, n);
        defer allocator.free(tour);
        try nearestNeighborTour(p, &candidates, &random, trial, options.randomized_starts, tour);
        if (trial > 0 and n >= 8) doubleBridgeKick(tour, &random);

        var work = LocalSearch{
            .p = p,
            .candidates = &candidates,
            .tour = tour,
            .pos = try allocator.alloc(usize, n),
            .max_passes = options.max_passes,
            .enable_or_opt = options.enable_or_opt,
        };
        defer allocator.free(work.pos);
        work.rebuildPositions();
        const moves = try work.improve();
        total_moves += moves;

        const len = try p.tourLength(tour);
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
        .iterations = total_moves + best_trial,
    };
}

fn buildCandidates(allocator: std.mem.Allocator, p: *const problem.Problem, width: usize) !Candidates {
    const n = p.dimension;
    var data = try allocator.alloc(usize, n * width);
    errdefer allocator.free(data);
    var dist = try allocator.alloc(u32, width);
    defer allocator.free(dist);

    for (0..n) |i| {
        @memset(dist, std.math.maxInt(u32));
        const row = data[i * width .. i * width + width];
        @memset(row, i);

        for (0..n) |j| {
            if (i == j) continue;
            const d = p.distanceUnchecked(i, j);
            var slot: ?usize = null;
            for (0..width) |k| {
                if (d < dist[k]) {
                    slot = k;
                    break;
                }
            }
            if (slot) |k| {
                if (k + 1 < width) {
                    std.mem.copyBackwards(u32, dist[k + 1 ..], dist[k .. width - 1]);
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
    p: *const problem.Problem,
    candidates: *const Candidates,
    random: *std.Random,
    trial: usize,
    randomized: bool,
    tour: []usize,
) !void {
    const n = p.dimension;
    var used = try p.allocator.alloc(bool, n);
    defer p.allocator.free(used);
    @memset(used, false);

    var current = trial % n;
    if (randomized and trial > 0) current = random.intRangeLessThan(usize, 0, n);

    for (0..n) |idx| {
        tour[idx] = current;
        used[current] = true;
        if (idx + 1 == n) break;

        var best_nodes: [4]usize = undefined;
        var best_dist: [4]u32 = .{ std.math.maxInt(u32), std.math.maxInt(u32), std.math.maxInt(u32), std.math.maxInt(u32) };
        var found: usize = 0;

        for (candidates.row(current)) |candidate| {
            if (!used[candidate]) {
                insertCandidate(candidate, p.distanceUnchecked(current, candidate), &best_nodes, &best_dist, &found);
            }
        }
        if (found == 0) {
            for (0..n) |node| {
                if (!used[node]) {
                    insertCandidate(node, p.distanceUnchecked(current, node), &best_nodes, &best_dist, &found);
                }
            }
        }

        const choice_count = if (randomized and trial > 0) @min(found, 3) else 1;
        const chosen_idx = if (choice_count > 1) random.intRangeLessThan(usize, 0, choice_count) else 0;
        current = best_nodes[chosen_idx];
    }
}

fn insertCandidate(
    node: usize,
    dist: u32,
    best_nodes: *[4]usize,
    best_dist: *[4]u32,
    found: *usize,
) void {
    var slot: ?usize = null;
    for (0..best_dist.len) |i| {
        if (dist < best_dist[i]) {
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
    best_dist[pos] = dist;
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
    p: *const problem.Problem,
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
            const old_ab = self.p.distanceUnchecked(a, b);

            for (self.candidates.row(a)) |c| {
                const j = self.pos[c];
                if (j <= i + 1) continue;
                if (i == 0 and j == n - 1) continue;
                const d = self.tour[(j + 1) % n];
                if (b == c or a == d) continue;

                const old_cd = self.p.distanceUnchecked(c, d);
                const new_ac = self.p.distanceUnchecked(a, c);
                const new_bd = self.p.distanceUnchecked(b, d);
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
            const remove_old = @as(u64, self.p.distanceUnchecked(a, b)) + self.p.distanceUnchecked(b, c);
            const remove_new = self.p.distanceUnchecked(a, c);

            for (self.candidates.row(b)) |x| {
                const j = self.pos[x];
                const y = self.tour[(j + 1) % n];
                if (x == a or x == b or x == c or y == a or y == b) continue;
                if ((j + 1) % n == i) continue;

                const insert_old = self.p.distanceUnchecked(x, y);
                const insert_new = @as(u64, self.p.distanceUnchecked(x, b)) + self.p.distanceUnchecked(b, y);
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
    try std.testing.expect(result.length <= 25);
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
