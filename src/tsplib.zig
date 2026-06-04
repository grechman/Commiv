const std = @import("std");
const problem = @import("problem.zig");

pub const ParseError = error{
    InvalidTsplib,
    OutOfMemory,
};

pub const ParseDiagnostic = struct {
    line: usize = 0,
    message: []const u8 = "",
};

pub const ParseOptions = struct {
    diagnostic: ?*ParseDiagnostic = null,
    max_dimension: usize = 100_000,
    max_matrix_weights: usize = 25_000_000,
};

const Section = enum {
    header,
    node_coord,
    edge_weight,
};

const Header = struct {
    name: []const u8 = "",
    type_seen: bool = false,
    tsp_type: []const u8 = "",
    dimension: ?usize = null,
    edge_weight_type: []const u8 = "",
    edge_weight_format: []const u8 = "",
};

pub fn parse(
    allocator: std.mem.Allocator,
    input: []const u8,
    options: ParseOptions,
) ParseError!problem.Problem {
    var parser = Parser{
        .allocator = allocator,
        .input = input,
        .diagnostic = options.diagnostic,
        .options = options,
    };
    return parser.parse();
}

const Parser = struct {
    allocator: std.mem.Allocator,
    input: []const u8,
    diagnostic: ?*ParseDiagnostic,
    options: ParseOptions,

    fn parse(self: *Parser) ParseError!problem.Problem {
        var header: Header = .{};
        var section: Section = .header;
        var coords: std.ArrayList(problem.Coord) = .empty;
        defer coords.deinit(self.allocator);
        var coord_seen: std.ArrayList(bool) = .empty;
        defer coord_seen.deinit(self.allocator);
        var coord_count: usize = 0;
        var matrix: std.ArrayList(u32) = .empty;
        defer matrix.deinit(self.allocator);

        var line_no: usize = 0;
        var lines = std.mem.splitScalar(u8, self.input, '\n');
        while (lines.next()) |raw_line| {
            line_no += 1;
            const line = std.mem.trim(u8, raw_line, " \t\r");
            if (line.len == 0) continue;
            if (std.ascii.eqlIgnoreCase(line, "EOF")) {
                break;
            }

            switch (section) {
                .header => {
                    if (std.ascii.eqlIgnoreCase(line, "NODE_COORD_SECTION")) {
                        const dim = header.dimension orelse return self.fail(line_no, "NODE_COORD_SECTION before DIMENSION");
                        try self.validateDimensionLimit(dim, line_no);
                        if (!isCoordType(header.edge_weight_type)) {
                            return self.fail(line_no, "NODE_COORD_SECTION requires EUC_2D or CEIL_2D");
                        }
                        try coords.ensureTotalCapacityPrecise(self.allocator, dim);
                        coords.appendNTimesAssumeCapacity(.{ .x = 0, .y = 0 }, dim);
                        try coord_seen.ensureTotalCapacityPrecise(self.allocator, dim);
                        coord_seen.appendNTimesAssumeCapacity(false, dim);
                        section = .node_coord;
                    } else if (std.ascii.eqlIgnoreCase(line, "EDGE_WEIGHT_SECTION")) {
                        const dim = header.dimension orelse return self.fail(line_no, "EDGE_WEIGHT_SECTION before DIMENSION");
                        try self.validateDimensionLimit(dim, line_no);
                        if (!std.ascii.eqlIgnoreCase(header.edge_weight_type, "EXPLICIT")) {
                            return self.fail(line_no, "EDGE_WEIGHT_SECTION requires EDGE_WEIGHT_TYPE EXPLICIT");
                        }
                        if (!std.ascii.eqlIgnoreCase(header.edge_weight_format, "FULL_MATRIX")) {
                            return self.fail(line_no, "only EDGE_WEIGHT_FORMAT FULL_MATRIX is supported");
                        }
                        const total = std.math.mul(usize, dim, dim) catch return self.fail(line_no, "DIMENSION is too large");
                        if (total > self.options.max_matrix_weights) return self.fail(line_no, "EDGE_WEIGHT_SECTION exceeds max_matrix_weights");
                        section = .edge_weight;
                    } else {
                        try self.parseHeaderLine(&header, line_no, line);
                    }
                },
                .node_coord => try self.parseCoordLine(&coords, &coord_seen, &coord_count, header.dimension.?, line_no, line),
                .edge_weight => try self.parseMatrixLine(&matrix, header.dimension.?, line_no, line),
            }
        }

        if (!header.type_seen) return self.fail(0, "missing TYPE");
        if (!std.ascii.eqlIgnoreCase(header.tsp_type, "TSP")) return self.fail(0, "only TYPE TSP is supported");
        const dim = header.dimension orelse return self.fail(0, "missing DIMENSION");
        if (dim < 2) return self.fail(0, "DIMENSION must be at least 2");
        try self.validateDimensionLimit(dim, 0);
        if (header.edge_weight_type.len == 0) return self.fail(0, "missing EDGE_WEIGHT_TYPE");

        if (isCoordType(header.edge_weight_type)) {
            if (coord_count != dim) return self.fail(0, "NODE_COORD_SECTION does not contain DIMENSION nodes");
            const owned_coords = try coords.toOwnedSlice(self.allocator);
            defer self.allocator.free(owned_coords);
            const kind: problem.DistanceKind = if (std.ascii.eqlIgnoreCase(header.edge_weight_type, "EUC_2D")) .euc_2d else .ceil_2d;
            return problem.Problem.initCoords(self.allocator, header.name, kind, owned_coords) catch |err| {
                return self.problemFail(err);
            };
        }

        if (std.ascii.eqlIgnoreCase(header.edge_weight_type, "EXPLICIT")) {
            if (!std.ascii.eqlIgnoreCase(header.edge_weight_format, "FULL_MATRIX")) {
                return self.fail(0, "EXPLICIT requires EDGE_WEIGHT_FORMAT FULL_MATRIX");
            }
            const expected = std.math.mul(usize, dim, dim) catch return self.fail(0, "DIMENSION is too large");
            if (expected > self.options.max_matrix_weights) return self.fail(0, "EDGE_WEIGHT_SECTION exceeds max_matrix_weights");
            if (matrix.items.len != expected) return self.fail(0, "EDGE_WEIGHT_SECTION does not contain DIMENSION squared weights");
            const owned_matrix = try matrix.toOwnedSlice(self.allocator);
            defer self.allocator.free(owned_matrix);
            return problem.Problem.initFullMatrix(self.allocator, header.name, dim, owned_matrix) catch |err| {
                return self.problemFail(err);
            };
        }

        return self.fail(0, "unsupported EDGE_WEIGHT_TYPE");
    }

    fn validateDimensionLimit(self: *Parser, dimension: usize, line_no: usize) ParseError!void {
        if (dimension > self.options.max_dimension) return self.fail(line_no, "DIMENSION exceeds max_dimension");
    }

    fn parseHeaderLine(self: *Parser, header: *Header, line_no: usize, line: []const u8) ParseError!void {
        const kv = splitHeader(line) orelse return self.fail(line_no, "invalid header line");
        const key = kv.key;
        const value = kv.value;
        if (value.len == 0 and !std.ascii.eqlIgnoreCase(key, "COMMENT")) {
            return self.fail(line_no, "empty header value");
        }

        if (std.ascii.eqlIgnoreCase(key, "NAME")) {
            header.name = value;
        } else if (std.ascii.eqlIgnoreCase(key, "TYPE")) {
            header.type_seen = true;
            header.tsp_type = value;
        } else if (std.ascii.eqlIgnoreCase(key, "DIMENSION")) {
            header.dimension = std.fmt.parseInt(usize, value, 10) catch {
                return self.fail(line_no, "invalid DIMENSION");
            };
        } else if (std.ascii.eqlIgnoreCase(key, "EDGE_WEIGHT_TYPE")) {
            header.edge_weight_type = value;
        } else if (std.ascii.eqlIgnoreCase(key, "EDGE_WEIGHT_FORMAT")) {
            header.edge_weight_format = value;
        } else if (std.ascii.eqlIgnoreCase(key, "COMMENT")) {
            return;
        } else {
            return;
        }
    }

    fn parseCoordLine(
        self: *Parser,
        coords: *std.ArrayList(problem.Coord),
        seen: *std.ArrayList(bool),
        coord_count: *usize,
        dimension: usize,
        line_no: usize,
        line: []const u8,
    ) ParseError!void {
        if (coord_count.* >= dimension) return self.fail(line_no, "too many coordinate rows");
        var tokens = std.mem.tokenizeAny(u8, line, " \t");
        const id_text = tokens.next() orelse return self.fail(line_no, "missing node id");
        const x_text = tokens.next() orelse return self.fail(line_no, "missing x coordinate");
        const y_text = tokens.next() orelse return self.fail(line_no, "missing y coordinate");
        if (tokens.next() != null) return self.fail(line_no, "too many coordinate fields");

        const one_based = std.fmt.parseInt(usize, id_text, 10) catch {
            return self.fail(line_no, "invalid node id");
        };
        if (one_based == 0 or one_based > dimension) return self.fail(line_no, "node id outside DIMENSION");
        const idx = one_based - 1;
        if (seen.items[idx]) return self.fail(line_no, "duplicate node id");

        const x = std.fmt.parseFloat(f64, x_text) catch return self.fail(line_no, "invalid x coordinate");
        const y = std.fmt.parseFloat(f64, y_text) catch return self.fail(line_no, "invalid y coordinate");
        if (!std.math.isFinite(x) or !std.math.isFinite(y)) return self.fail(line_no, "coordinate is not finite");

        coords.items[idx] = .{ .x = x, .y = y };
        seen.items[idx] = true;
        coord_count.* += 1;
    }

    fn parseMatrixLine(
        self: *Parser,
        matrix: *std.ArrayList(u32),
        dimension: usize,
        line_no: usize,
        line: []const u8,
    ) ParseError!void {
        const expected = std.math.mul(usize, dimension, dimension) catch return self.fail(line_no, "DIMENSION is too large");
        var tokens = std.mem.tokenizeAny(u8, line, " \t");
        while (tokens.next()) |token| {
            if (matrix.items.len >= expected) return self.fail(line_no, "too many matrix weights");
            const value = std.fmt.parseInt(u32, token, 10) catch {
                return self.fail(line_no, "invalid matrix weight");
            };
            try matrix.append(self.allocator, value);
        }
    }

    fn fail(self: *Parser, line: usize, message: []const u8) ParseError {
        if (self.diagnostic) |diag| {
            diag.* = .{ .line = line, .message = message };
        }
        return error.InvalidTsplib;
    }

    fn problemFail(self: *Parser, err: anyerror) ParseError {
        return switch (err) {
            error.OutOfMemory => error.OutOfMemory,
            else => self.fail(0, @errorName(err)),
        };
    }
};

const HeaderPair = struct {
    key: []const u8,
    value: []const u8,
};

fn splitHeader(line: []const u8) ?HeaderPair {
    if (std.mem.indexOfScalar(u8, line, ':')) |colon| {
        return .{
            .key = std.mem.trim(u8, line[0..colon], " \t"),
            .value = std.mem.trim(u8, line[colon + 1 ..], " \t"),
        };
    }
    var iter = std.mem.tokenizeAny(u8, line, " \t");
    const key = iter.next() orelse return null;
    const key_end = key.ptr + key.len - line.ptr;
    return .{
        .key = key,
        .value = std.mem.trim(u8, line[key_end..], " \t"),
    };
}

fn isCoordType(kind: []const u8) bool {
    return std.ascii.eqlIgnoreCase(kind, "EUC_2D") or std.ascii.eqlIgnoreCase(kind, "CEIL_2D");
}

fn expectInvalid(input: []const u8, expected_message: []const u8) !void {
    var diag: ParseDiagnostic = .{};
    try std.testing.expectError(error.InvalidTsplib, parse(std.testing.allocator, input, .{ .diagnostic = &diag }));
    try std.testing.expectEqualStrings(expected_message, diag.message);
}

test "parse EUC_2D TSPLIB square" {
    const allocator = std.testing.allocator;
    const data =
        \\NAME: square
        \\TYPE: TSP
        \\DIMENSION: 4
        \\EDGE_WEIGHT_TYPE: EUC_2D
        \\NODE_COORD_SECTION
        \\1 0 0
        \\2 1 0
        \\3 1 1
        \\4 0 1
        \\EOF
    ;
    var diag: ParseDiagnostic = .{};
    var p = try parse(allocator, data, .{ .diagnostic = &diag });
    defer p.deinit();
    try std.testing.expectEqualStrings("square", p.name);
    try std.testing.expectEqual(@as(usize, 4), p.dimension);
    try std.testing.expectEqual(@as(u64, 4), try p.tourLength(&.{ 0, 1, 2, 3 }));
}

test "parse CEIL_2D rounds up" {
    const allocator = std.testing.allocator;
    const data =
        \\NAME: ceil
        \\TYPE: TSP
        \\DIMENSION: 2
        \\EDGE_WEIGHT_TYPE: CEIL_2D
        \\NODE_COORD_SECTION
        \\1 0 0
        \\2 1.1 0
        \\EOF
    ;
    var p = try parse(allocator, data, .{});
    defer p.deinit();
    try std.testing.expectEqual(@as(u32, 2), try p.distance(0, 1));
}

test "parse explicit full matrix" {
    const allocator = std.testing.allocator;
    const data =
        \\NAME: matrix
        \\TYPE: TSP
        \\DIMENSION: 3
        \\EDGE_WEIGHT_TYPE: EXPLICIT
        \\EDGE_WEIGHT_FORMAT: FULL_MATRIX
        \\EDGE_WEIGHT_SECTION
        \\0 2 3
        \\2 0 4
        \\3 4 0
        \\EOF
    ;
    var p = try parse(allocator, data, .{});
    defer p.deinit();
    try std.testing.expectEqual(@as(u64, 9), try p.tourLength(&.{ 0, 1, 2 }));
}

test "parse diagnostics report malformed input" {
    const allocator = std.testing.allocator;
    const data =
        \\NAME: bad
        \\TYPE: TSP
        \\DIMENSION: 2
        \\EDGE_WEIGHT_TYPE: EUC_2D
        \\NODE_COORD_SECTION
        \\1 0 0
        \\1 1 0
        \\EOF
    ;
    var diag: ParseDiagnostic = .{};
    try std.testing.expectError(error.InvalidTsplib, parse(allocator, data, .{ .diagnostic = &diag }));
    try std.testing.expectEqual(@as(usize, 7), diag.line);
    try std.testing.expectEqualStrings("duplicate node id", diag.message);
}

test "parse out-of-order coordinates by node id" {
    const allocator = std.testing.allocator;
    const data =
        \\NAME: shuffled
        \\TYPE: TSP
        \\DIMENSION: 4
        \\EDGE_WEIGHT_TYPE: EUC_2D
        \\NODE_COORD_SECTION
        \\3 1 1
        \\1 0 0
        \\4 0 1
        \\2 1 0
        \\EOF
    ;
    var p = try parse(allocator, data, .{});
    defer p.deinit();
    try std.testing.expectEqual(@as(u64, 4), try p.tourLength(&.{ 0, 1, 2, 3 }));
}

test "parse rejects missing required headers" {
    try expectInvalid(
        \\NAME: missing-type
        \\DIMENSION: 2
        \\EDGE_WEIGHT_TYPE: EUC_2D
        \\NODE_COORD_SECTION
        \\1 0 0
        \\2 1 0
        \\EOF
    , "missing TYPE");

    try expectInvalid(
        \\NAME: missing-dimension
        \\TYPE: TSP
        \\EDGE_WEIGHT_TYPE: EUC_2D
        \\NODE_COORD_SECTION
        \\1 0 0
        \\2 1 0
        \\EOF
    , "NODE_COORD_SECTION before DIMENSION");

    try expectInvalid(
        \\NAME: missing-weight-type
        \\TYPE: TSP
        \\DIMENSION: 2
        \\EOF
    , "missing EDGE_WEIGHT_TYPE");

    try expectInvalid(
        \\NAME: missing-weight-type-before-section
        \\TYPE: TSP
        \\DIMENSION: 2
        \\NODE_COORD_SECTION
        \\1 0 0
        \\2 1 0
        \\EOF
    , "NODE_COORD_SECTION requires EUC_2D or CEIL_2D");
}

test "parse rejects unsupported type and weight formats" {
    try expectInvalid(
        \\NAME: bad-type
        \\TYPE: ATSP
        \\DIMENSION: 2
        \\EDGE_WEIGHT_TYPE: EUC_2D
        \\NODE_COORD_SECTION
        \\1 0 0
        \\2 1 0
        \\EOF
    , "only TYPE TSP is supported");

    try expectInvalid(
        \\NAME: bad-weight-type
        \\TYPE: TSP
        \\DIMENSION: 2
        \\EDGE_WEIGHT_TYPE: GEO
        \\EOF
    , "unsupported EDGE_WEIGHT_TYPE");

    try expectInvalid(
        \\NAME: bad-format
        \\TYPE: TSP
        \\DIMENSION: 3
        \\EDGE_WEIGHT_TYPE: EXPLICIT
        \\EDGE_WEIGHT_FORMAT: UPPER_ROW
        \\EDGE_WEIGHT_SECTION
        \\0 1 2 1 0 3 2 3 0
        \\EOF
    , "only EDGE_WEIGHT_FORMAT FULL_MATRIX is supported");
}

test "parse enforces resource limits before section allocation" {
    const too_many_coords =
        \\NAME: huge
        \\TYPE: TSP
        \\DIMENSION: 11
        \\EDGE_WEIGHT_TYPE: EUC_2D
        \\NODE_COORD_SECTION
        \\1 0 0
        \\EOF
    ;
    var coord_diag: ParseDiagnostic = .{};
    try std.testing.expectError(error.InvalidTsplib, parse(std.testing.allocator, too_many_coords, .{
        .diagnostic = &coord_diag,
        .max_dimension = 10,
    }));
    try std.testing.expectEqualStrings("DIMENSION exceeds max_dimension", coord_diag.message);

    const too_many_matrix_weights =
        \\NAME: huge-matrix
        \\TYPE: TSP
        \\DIMENSION: 3
        \\EDGE_WEIGHT_TYPE: EXPLICIT
        \\EDGE_WEIGHT_FORMAT: FULL_MATRIX
        \\EDGE_WEIGHT_SECTION
        \\0 1 2 1 0 3 2 3 0
        \\EOF
    ;
    var matrix_diag: ParseDiagnostic = .{};
    try std.testing.expectError(error.InvalidTsplib, parse(std.testing.allocator, too_many_matrix_weights, .{
        .diagnostic = &matrix_diag,
        .max_matrix_weights = 8,
    }));
    try std.testing.expectEqualStrings("EDGE_WEIGHT_SECTION exceeds max_matrix_weights", matrix_diag.message);
}

test "parse does not preallocate full matrix capacity for incomplete matrix section" {
    const header_only_matrix =
        \\NAME: incomplete-huge-matrix
        \\TYPE: TSP
        \\DIMENSION: 5000
        \\EDGE_WEIGHT_TYPE: EXPLICIT
        \\EDGE_WEIGHT_FORMAT: FULL_MATRIX
        \\EDGE_WEIGHT_SECTION
        \\EOF
    ;
    var buffer: [128]u8 = undefined;
    var fixed = std.heap.FixedBufferAllocator.init(&buffer);
    var diag: ParseDiagnostic = .{};
    try std.testing.expectError(error.InvalidTsplib, parse(fixed.allocator(), header_only_matrix, .{
        .diagnostic = &diag,
        .max_matrix_weights = 25_000_000,
    }));
    try std.testing.expectEqualStrings("EDGE_WEIGHT_SECTION does not contain DIMENSION squared weights", diag.message);
}
