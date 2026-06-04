const std = @import("std");
const commiv = @import("commiv");

const sample =
    \\NAME: classroom-square
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

pub fn main() !void {
    const allocator = std.heap.page_allocator;
    var diag: commiv.ParseDiagnostic = .{};
    var p = try commiv.parseTsplib(allocator, sample, .{ .diagnostic = &diag });
    defer p.deinit();

    var result = try commiv.solve(allocator, &p, .{ .seed = 1 });
    defer result.deinit();

    std.debug.print("name={s} length={} tour=", .{ p.name, result.length });
    for (result.tour, 0..) |node, i| {
        if (i != 0) std.debug.print("-", .{});
        std.debug.print("{}", .{node + 1});
    }
    std.debug.print("\n", .{});
}
