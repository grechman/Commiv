const std = @import("std");

pub const DistanceKind = enum {
    euc_2d,
    ceil_2d,
    explicit_full_matrix,
};

pub const Coord = struct {
    x: f64,
    y: f64,
};

pub const ProblemError = error{
    DimensionTooSmall,
    DistanceOverflow,
    DuplicateNode,
    IndexOutOfBounds,
    InvalidCoordinate,
    InvalidMatrix,
    NonSymmetricMatrix,
    TourWrongLength,
};

pub const TourResult = struct {
    allocator: std.mem.Allocator,
    tour: []usize,
    length: u64,
    iterations: u64 = 0,

    pub fn deinit(self: *TourResult) void {
        self.allocator.free(self.tour);
        self.* = undefined;
    }
};

pub const Problem = struct {
    allocator: std.mem.Allocator,
    name: []u8,
    dimension: usize,
    distance_kind: DistanceKind,
    coords: []Coord,
    matrix: []u32,

    pub fn initCoords(
        allocator: std.mem.Allocator,
        name: []const u8,
        kind: DistanceKind,
        coords: []const Coord,
    ) !Problem {
        if (kind == .explicit_full_matrix) return ProblemError.InvalidMatrix;
        if (coords.len < 2) return ProblemError.DimensionTooSmall;

        for (coords) |coord| {
            if (!std.math.isFinite(coord.x) or !std.math.isFinite(coord.y)) {
                return ProblemError.InvalidCoordinate;
            }
        }
        try validateCoordinateRange(kind, coords);

        const owned_name = try allocator.dupe(u8, name);
        errdefer allocator.free(owned_name);
        const owned_coords = try allocator.dupe(Coord, coords);
        errdefer allocator.free(owned_coords);

        return .{
            .allocator = allocator,
            .name = owned_name,
            .dimension = owned_coords.len,
            .distance_kind = kind,
            .coords = owned_coords,
            .matrix = &.{},
        };
    }

    pub fn initFullMatrix(
        allocator: std.mem.Allocator,
        name: []const u8,
        dimension: usize,
        matrix: []const u32,
    ) !Problem {
        if (dimension < 2) return ProblemError.DimensionTooSmall;
        if (matrix.len != try squareLen(dimension)) return ProblemError.InvalidMatrix;

        for (0..dimension) |row| {
            for (0..dimension) |col| {
                const a = matrix[row * dimension + col];
                const b = matrix[col * dimension + row];
                if (a != b) return ProblemError.NonSymmetricMatrix;
            }
        }

        const owned_name = try allocator.dupe(u8, name);
        errdefer allocator.free(owned_name);
        const owned_matrix = try allocator.dupe(u32, matrix);
        errdefer allocator.free(owned_matrix);

        return .{
            .allocator = allocator,
            .name = owned_name,
            .dimension = dimension,
            .distance_kind = .explicit_full_matrix,
            .coords = &.{},
            .matrix = owned_matrix,
        };
    }

    pub fn deinit(self: *Problem) void {
        self.allocator.free(self.name);
        if (self.coords.len != 0) self.allocator.free(self.coords);
        if (self.matrix.len != 0) self.allocator.free(self.matrix);
        self.* = undefined;
    }

    pub fn distance(self: *const Problem, a: usize, b: usize) ProblemError!u32 {
        if (a >= self.dimension or b >= self.dimension) return ProblemError.IndexOutOfBounds;
        return self.distanceUnchecked(a, b);
    }

    pub fn distanceUnchecked(self: *const Problem, a: usize, b: usize) u32 {
        std.debug.assert(a < self.dimension);
        std.debug.assert(b < self.dimension);

        return switch (self.distance_kind) {
            .euc_2d => roundedEuclidean(self.coords[a], self.coords[b], .nearest),
            .ceil_2d => roundedEuclidean(self.coords[a], self.coords[b], .ceil),
            .explicit_full_matrix => self.matrix[a * self.dimension + b],
        };
    }

    pub fn tourLength(self: *const Problem, tour: []const usize) !u64 {
        try self.validateTour(tour);
        return self.tourLengthUnchecked(tour);
    }

    pub fn tourLengthUnchecked(self: *const Problem, tour: []const usize) ProblemError!u64 {
        std.debug.assert(tour.len == self.dimension);
        var total: u64 = 0;
        for (0..tour.len) |i| {
            const a = tour[i];
            const b = tour[(i + 1) % tour.len];
            total = std.math.add(u64, total, @as(u64, self.distanceUnchecked(a, b))) catch {
                return ProblemError.DistanceOverflow;
            };
        }
        return total;
    }

    pub fn validateTour(self: *const Problem, tour: []const usize) !void {
        try validateTourWithAllocator(self.allocator, self.dimension, tour);
    }

    pub fn identityTour(self: *const Problem, allocator: std.mem.Allocator) ![]usize {
        return identityTourFor(allocator, self.dimension);
    }
};

pub fn validateTourWithAllocator(
    allocator: std.mem.Allocator,
    dimension: usize,
    tour: []const usize,
) !void {
    if (tour.len != dimension) return ProblemError.TourWrongLength;
    var seen = try allocator.alloc(bool, dimension);
    defer allocator.free(seen);
    @memset(seen, false);

    for (tour) |node| {
        if (node >= dimension) return ProblemError.IndexOutOfBounds;
        if (seen[node]) return ProblemError.DuplicateNode;
        seen[node] = true;
    }
}

pub fn identityTourFor(allocator: std.mem.Allocator, dimension: usize) ![]usize {
    const tour = try allocator.alloc(usize, dimension);
    for (tour, 0..) |*node, i| node.* = i;
    return tour;
}

fn squareLen(dimension: usize) !usize {
    return std.math.mul(usize, dimension, dimension) catch ProblemError.InvalidMatrix;
}

fn validateCoordinateRange(kind: DistanceKind, coords: []const Coord) ProblemError!void {
    std.debug.assert(coords.len >= 2);
    var min_x = coords[0].x;
    var max_x = coords[0].x;
    var min_y = coords[0].y;
    var max_y = coords[0].y;
    for (coords[1..]) |coord| {
        min_x = @min(min_x, coord.x);
        max_x = @max(max_x, coord.x);
        min_y = @min(min_y, coord.y);
        max_y = @max(max_y, coord.y);
    }

    const dx = max_x - min_x;
    const dy = max_y - min_y;
    const diagonal = std.math.sqrt(dx * dx + dy * dy);
    const worst = switch (kind) {
        .euc_2d => @floor(diagonal + 0.5),
        .ceil_2d => @ceil(diagonal),
        .explicit_full_matrix => unreachable,
    };
    if (worst > @as(f64, @floatFromInt(std.math.maxInt(u32)))) {
        return ProblemError.DistanceOverflow;
    }
}

const Rounding = enum { nearest, ceil };

fn roundedEuclidean(a: Coord, b: Coord, rounding: Rounding) u32 {
    const dx = a.x - b.x;
    const dy = a.y - b.y;
    const distance = std.math.sqrt(dx * dx + dy * dy);
    const rounded = switch (rounding) {
        .nearest => @floor(distance + 0.5),
        .ceil => @ceil(distance),
    };
    std.debug.assert(rounded >= 0);
    std.debug.assert(rounded <= @as(f64, @floatFromInt(std.math.maxInt(u32))));
    return @intFromFloat(rounded);
}

test "coordinate distances and tour validation" {
    const allocator = std.testing.allocator;
    const coords = [_]Coord{
        .{ .x = 0, .y = 0 },
        .{ .x = 1, .y = 0 },
        .{ .x = 1, .y = 1 },
        .{ .x = 0, .y = 1 },
    };
    var p = try Problem.initCoords(allocator, "square", .euc_2d, &coords);
    defer p.deinit();

    try std.testing.expectEqual(@as(usize, 4), p.dimension);
    try std.testing.expectEqual(@as(u32, 1), try p.distance(0, 1));
    try std.testing.expectEqual(@as(u32, 1), try p.distance(1, 2));
    try std.testing.expectEqual(@as(u64, 4), try p.tourLength(&.{ 0, 1, 2, 3 }));
    try std.testing.expectError(ProblemError.DuplicateNode, p.validateTour(&.{ 0, 1, 1, 3 }));
    try std.testing.expectError(ProblemError.IndexOutOfBounds, p.validateTour(&.{ 0, 1, 2, 4 }));
}

test "explicit full matrix is checked for symmetry" {
    const allocator = std.testing.allocator;
    const good = [_]u32{
        0, 2, 3,
        2, 0, 4,
        3, 4, 0,
    };
    var p = try Problem.initFullMatrix(allocator, "tri", 3, &good);
    defer p.deinit();
    try std.testing.expectEqual(@as(u32, 4), try p.distance(1, 2));

    const bad = [_]u32{
        0, 2, 3,
        8, 0, 4,
        3, 4, 0,
    };
    try std.testing.expectError(ProblemError.NonSymmetricMatrix, Problem.initFullMatrix(allocator, "bad", 3, &bad));
}

test "coordinate distance overflow is rejected during construction" {
    const allocator = std.testing.allocator;
    const coords = [_]Coord{
        .{ .x = 0, .y = 0 },
        .{ .x = 5_000_000_000, .y = 0 },
    };
    try std.testing.expectError(ProblemError.DistanceOverflow, Problem.initCoords(allocator, "huge", .euc_2d, &coords));
}
