const std = @import("std");

pub const DistanceKind = enum {
    euc_2d,
    ceil_2d,
    att,
    explicit_full_matrix,
    // Jonker-Volgenant 2n-node ATSP transform computed per lookup from a
    // borrowed n*n directed matrix — never materialized (see initJvTransform).
    jv_transform,
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

// The tour result type now lives in result.zig as the single canonical
// SolveResult, shared by solve / solveAtsp* / bruteForce.

pub const Problem = struct {
    allocator: std.mem.Allocator,
    name: []u8,
    dimension: usize,
    distance_kind: DistanceKind,
    coords: []Coord,
    matrix: []u32,
    // jv_transform only: the BORROWED n*n directed matrix (caller keeps it
    // alive for the Problem's lifetime) and the transform constants.
    jv_asym: []const u32 = &.{},
    jv_n: usize = 0,
    jv_big: u32 = 0,
    jv_inf: u32 = 0,

    pub fn initCoords(
        allocator: std.mem.Allocator,
        name: []const u8,
        kind: DistanceKind,
        coords: []const Coord,
    ) !Problem {
        if (kind == .explicit_full_matrix or kind == .jv_transform) return ProblemError.InvalidMatrix;
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

    /// A Jonker-Volgenant 2n-node view over a BORROWED n*n directed matrix:
    /// dimension = 2n, but the (2n)^2 symmetric transform matrix is never
    /// materialized — distanceUnchecked computes each entry on the fly from
    /// `asym` and the transform constants (see asymmetric.zig for the formula
    /// and how big/inf are sized). Produces exactly the integers the
    /// materialized transform held, so solver trajectories are bit-identical;
    /// the saving is the 16n^2 B transform matrix plus the 16n^2 B copy
    /// initFullMatrix would have made of it.
    pub fn initJvTransform(
        allocator: std.mem.Allocator,
        name: []const u8,
        n: usize,
        asym: []const u32,
        big: u32,
        inf: u32,
    ) !Problem {
        if (n < 2) return ProblemError.DimensionTooSmall;
        if (asym.len != try squareLen(n)) return ProblemError.InvalidMatrix;
        const owned_name = try allocator.dupe(u8, name);
        errdefer allocator.free(owned_name);
        return .{
            .allocator = allocator,
            .name = owned_name,
            .dimension = 2 * n,
            .distance_kind = .jv_transform,
            .coords = &.{},
            .matrix = &.{},
            .jv_asym = asym,
            .jv_n = n,
            .jv_big = big,
            .jv_inf = inf,
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
            .att => attDistance(self.coords[a], self.coords[b]),
            .explicit_full_matrix => self.matrix[a * self.dimension + b],
            .jv_transform => self.jvDistance(a, b),
        };
    }

    // The transform, per lookup: tails are 0..n-1, heads n..2n-1. Pair edge
    // {i, n+i} = 0; arc edge {head(i), tail(j)} = asym[i*n+j] + big; same-side
    // pairs (tail-tail, head-head) = inf (forbidden). Symmetric by construction.
    // No overflow: asym entry + big < inf <= maxInt(u32), checked by the caller
    // that sized the constants (solveAtsp).
    fn jvDistance(self: *const Problem, a: usize, b: usize) u32 {
        if (a == b) return 0;
        const n = self.jv_n;
        const a_head = a >= n;
        if (a_head == (b >= n)) return self.jv_inf;
        const i = if (a_head) a - n else b - n; // the head's city
        const j = if (a_head) b else a; // the tail's city
        if (i == j) return 0;
        return self.jv_asym[i * n + j] + self.jv_big;
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
        .att => @ceil(diagonal / std.math.sqrt(10.0)) + 1,
        .explicit_full_matrix, .jv_transform => unreachable,
    };
    if (worst > @as(f64, @floatFromInt(std.math.maxInt(u32)))) {
        return ProblemError.DistanceOverflow;
    }
}

// TSPLIB pseudo-Euclidean (ATT): rij = sqrt((dx^2 + dy^2) / 10), rounded to
// the nearest integer but bumped up by one when rounding went down.
fn attDistance(a: Coord, b: Coord) u32 {
    const dx = a.x - b.x;
    const dy = a.y - b.y;
    const rij = std.math.sqrt((dx * dx + dy * dy) / 10.0);
    const tij = @floor(rij + 0.5);
    const dij = if (tij < rij) tij + 1 else tij;
    std.debug.assert(dij >= 0);
    std.debug.assert(dij <= @as(f64, @floatFromInt(std.math.maxInt(u32))));
    return @intFromFloat(dij);
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

test "jv_transform view matches the materialized 2n transform entry-for-entry" {
    // The bit-identity gate for the in-place JV view: build the (2n)^2 matrix
    // exactly the way solveAtsp used to (pair edges 0, arc edges asym+big,
    // everything else inf) and compare every cell against jvDistance.
    const allocator = std.testing.allocator;
    var prng = std.Random.DefaultPrng.init(0x1F5A);
    const rng = prng.random();
    const n = 23;
    const asym = try allocator.alloc(u32, n * n);
    defer allocator.free(asym);
    var max_arc: u64 = 0;
    for (0..n) |i| {
        for (0..n) |j| {
            asym[i * n + j] = if (i == j) 0 else rng.intRangeAtMost(u32, 1, 5000);
            if (i != j) max_arc = @max(max_arc, asym[i * n + j]);
        }
    }
    const big: u64 = (n + 1) * max_arc + 1;
    const inf: u64 = big + (n + 2) * (max_arc + 1) + 1;

    const m = 2 * n;
    const sym = try allocator.alloc(u32, m * m);
    defer allocator.free(sym);
    @memset(sym, @intCast(inf));
    for (0..m) |i| sym[i * m + i] = 0;
    for (0..n) |i| {
        const head = n + i;
        sym[i * m + head] = 0;
        sym[head * m + i] = 0;
        for (0..n) |j| {
            if (i == j) continue;
            const w: u32 = @intCast(@as(u64, asym[i * n + j]) + big);
            sym[head * m + j] = w;
            sym[j * m + head] = w;
        }
    }

    var p = try Problem.initJvTransform(allocator, "jv", n, asym, @intCast(big), @intCast(inf));
    defer p.deinit();
    try std.testing.expectEqual(m, p.dimension);
    for (0..m) |a| {
        for (0..m) |b| try std.testing.expectEqual(sym[a * m + b], p.distanceUnchecked(a, b));
    }
}
