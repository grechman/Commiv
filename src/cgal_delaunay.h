#ifndef COMMIV_CGAL_DELAUNAY_H
#define COMMIV_CGAL_DELAUNAY_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

size_t commiv_cgal_delaunay_edges(
    const double *xy,
    size_t point_count,
    uint32_t *out_edges,
    size_t max_edges);

#ifdef __cplusplus
}
#endif

#endif
