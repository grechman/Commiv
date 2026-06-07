const std = @import("std");

extern fn commiv_cgal_delaunay_edges(
    xy: [*]const f64,
    point_count: usize,
    out_edges: [*]u32,
    max_edges: usize,
) usize;

pub fn main() !void {
    const coords = [_]f64{
        0.0, 0.0,
        1.0, 0.0,
        0.0, 1.0,
        1.0, 1.0,
        0.5, 0.2,
    };
    var edges: [32]u32 = undefined;
    const count = commiv_cgal_delaunay_edges(&coords, coords.len / 2, &edges, edges.len / 2);
    if (count == std.math.maxInt(usize)) return error.CgalDelaunayFailed;
    if (count < 5) return error.TooFewDelaunayEdges;
    std.debug.print("cgal_delaunay_edges={}\n", .{count});
}
