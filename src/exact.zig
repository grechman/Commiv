const std = @import("std");
const problem = @import("problem.zig");
const result_mod = @import("result.zig");

pub const ExactOptions = struct {
    max_nodes: usize = 10,
};

pub const ExactError = error{
    InstanceTooLarge,
};

pub fn bruteForce(
    allocator: std.mem.Allocator,
    p: *const problem.Problem,
    options: ExactOptions,
) !result_mod.SolveResult {
    if (p.dimension > options.max_nodes) return ExactError.InstanceTooLarge;

    const perm = try allocator.alloc(usize, p.dimension - 1);
    defer allocator.free(perm);
    for (perm, 0..) |*node, i| node.* = i + 1;

    var best_tour = try allocator.alloc(usize, p.dimension);
    errdefer allocator.free(best_tour);
    var best_len: u64 = std.math.maxInt(u64);
    var iterations: u64 = 0;

    while (true) {
        iterations += 1;
        const len = try permLength(p, perm);
        if (len < best_len) {
            best_len = len;
            best_tour[0] = 0;
            @memcpy(best_tour[1..], perm);
        }
        if (!nextPermutation(usize, perm)) break;
    }

    return .{
        .allocator = allocator,
        .tour = best_tour,
        .length = best_len,
        .stats = .{ .exact_permutations = iterations },
    };
}

fn permLength(p: *const problem.Problem, perm: []const usize) !u64 {
    var total: u64 = 0;
    if (perm.len == 0) return 0;
    total = std.math.add(u64, total, p.distanceUnchecked(0, perm[0])) catch return problem.ProblemError.DistanceOverflow;
    for (0..perm.len - 1) |i| {
        total = std.math.add(u64, total, p.distanceUnchecked(perm[i], perm[i + 1])) catch return problem.ProblemError.DistanceOverflow;
    }
    total = std.math.add(u64, total, p.distanceUnchecked(perm[perm.len - 1], 0)) catch return problem.ProblemError.DistanceOverflow;
    return total;
}

fn nextPermutation(comptime T: type, items: []T) bool {
    if (items.len < 2) return false;
    var i = items.len - 2;
    while (items[i] >= items[i + 1]) {
        if (i == 0) return false;
        i -= 1;
    }

    var j = items.len - 1;
    while (items[j] <= items[i]) : (j -= 1) {}
    std.mem.swap(T, &items[i], &items[j]);
    std.mem.reverse(T, items[i + 1 ..]);
    return true;
}

test "brute force finds square optimum" {
    const allocator = std.testing.allocator;
    const coords = [_]problem.Coord{
        .{ .x = 0, .y = 0 },
        .{ .x = 1, .y = 0 },
        .{ .x = 1, .y = 1 },
        .{ .x = 0, .y = 1 },
    };
    var p = try problem.Problem.initCoords(allocator, "square", .euc_2d, &coords);
    defer p.deinit();

    var result = try bruteForce(allocator, &p, .{});
    defer result.deinit();
    try std.testing.expectEqual(@as(u64, 4), result.length);
    try p.validateTour(result.tour);
}

test "brute force rejects large instances by policy" {
    const allocator = std.testing.allocator;
    const coords = [_]problem.Coord{
        .{ .x = 0, .y = 0 },
        .{ .x = 1, .y = 0 },
        .{ .x = 2, .y = 0 },
        .{ .x = 3, .y = 0 },
    };
    var p = try problem.Problem.initCoords(allocator, "line", .euc_2d, &coords);
    defer p.deinit();
    try std.testing.expectError(ExactError.InstanceTooLarge, bruteForce(allocator, &p, .{ .max_nodes = 3 }));
}
