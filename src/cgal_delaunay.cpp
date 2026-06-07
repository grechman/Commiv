#include "cgal_delaunay.h"

#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Triangulation_vertex_base_with_info_2.h>

#include <limits>
#include <utility>
#include <vector>

using Kernel = CGAL::Exact_predicates_inexact_constructions_kernel;
using VertexBase = CGAL::Triangulation_vertex_base_with_info_2<std::size_t, Kernel>;
using FaceBase = CGAL::Triangulation_face_base_2<Kernel>;
using DataStructure = CGAL::Triangulation_data_structure_2<VertexBase, FaceBase>;
using Delaunay = CGAL::Delaunay_triangulation_2<Kernel, DataStructure>;
using Point = Kernel::Point_2;

extern "C" size_t commiv_cgal_delaunay_edges(
    const double *xy,
    size_t point_count,
    uint32_t *out_edges,
    size_t max_edges) {
    if (point_count < 2) return 0;
    if (point_count > std::numeric_limits<uint32_t>::max()) {
        return std::numeric_limits<size_t>::max();
    }

    try {
        std::vector<std::pair<Point, std::size_t>> points;
        points.reserve(point_count);
        for (std::size_t i = 0; i < point_count; ++i) {
            points.emplace_back(Point(xy[2 * i], xy[2 * i + 1]), i);
        }

        Delaunay triangulation;
        triangulation.insert(points.begin(), points.end());

        std::size_t edge_count = 0;
        for (auto edge = triangulation.finite_edges_begin(); edge != triangulation.finite_edges_end(); ++edge) {
            const auto face = edge->first;
            const int index = edge->second;
            const std::size_t a = face->vertex((index + 1) % 3)->info();
            const std::size_t b = face->vertex((index + 2) % 3)->info();
            if (edge_count < max_edges) {
                out_edges[2 * edge_count] = static_cast<uint32_t>(a);
                out_edges[2 * edge_count + 1] = static_cast<uint32_t>(b);
            }
            ++edge_count;
        }
        return edge_count;
    } catch (...) {
        return std::numeric_limits<size_t>::max();
    }
}
