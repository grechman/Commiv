const std = @import("std");
const problem = @import("problem.zig");

pub const SolverError = error{
    DistanceCacheTooLarge,
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
        max_cached_bytes: usize,
    ) !DistanceOracle {
        if (p.distance_kind == .explicit_full_matrix) {
            return .{
                .allocator = allocator,
                .p = p,
                .matrix = p.matrix,
                .owned_matrix = &.{},
            };
        }

        // Budget is a byte/L3 figure; the matrix stores u32 weights. Convert to
        // a weight count by integer division (a weight fits only if the whole
        // u32 fits), which avoids overflowing total*elem_size for large n.
        const max_cached_weights = max_cached_bytes / @sizeOf(u32);
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

    pub fn isCached(self: *const DistanceOracle) bool {
        return self.matrix.len != 0;
    }

    // Zeroes the per-trial cost counters after the one-time candidate build so
    // they measure only the trial loop (roadmap item 1).
    pub fn resetCounters(self: *DistanceOracle) void {
        self.uncached_coordinate_distances = 0;
        self.lookups = 0;
        self.length_scans = 0;
    }

    pub fn distance(self: *DistanceOracle, a: usize, b: usize) u32 {
        self.lookups += 1;
        if (self.matrix.len != 0) return self.matrix[a * self.p.dimension + b];
        if (self.p.distance_kind != .explicit_full_matrix) self.uncached_coordinate_distances += 1;
        return self.p.distanceUnchecked(a, b);
    }

    pub fn tourLengthUnchecked(self: *DistanceOracle, tour: []const usize) problem.ProblemError!u64 {
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
