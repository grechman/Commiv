/* commiv C API — near-optimal directed (asymmetric) TSP / CVRP / VRPTW routes.
 *
 * Build the libraries with `zig build lib -Doptimize=ReleaseFast`; link either
 * zig-out/lib/libcommiv.a (static) or libcommiv.so (shared). This header is the
 * stable FFI contract; every language binding sits on top of it.
 *
 * Conventions
 *   - Cost matrices are row-major uint32_t: matrix[a*dim + b] = cost of a -> b.
 *     They are DIRECTIONAL; a symmetric matrix is just the special case.
 *   - CVRP/VRPTW: dim = n_customers + 1, node 0 is the depot, customers 1..n.
 *   - Solvers return 0 (COMMIV_OK) or a negative COMMIV_ERR_* code.
 *   - A commiv_routes* returned through `out` is owned by the caller: read it
 *     with the accessors, release it with commiv_routes_free().
 *   - Zero-initialized commiv_options (or a NULL options pointer) = defaults.
 */
#ifndef COMMIV_H
#define COMMIV_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define COMMIV_OK 0
#define COMMIV_ERR_INVALID_ARGUMENT (-1)
#define COMMIV_ERR_OUT_OF_MEMORY (-2)
#define COMMIV_ERR_INFEASIBLE (-3)
#define COMMIV_ERR_INTERNAL (-4)

/* Opaque solution handle. */
typedef struct commiv_routes commiv_routes;

/* Zero value = default for every field.
 *   seed            RNG seed; 0 means 1. Same seed + threads<=1 = same output.
 *   sisr_iters      CVRP ruin-and-recreate iterations; 0 = 300000.
 *   trials          ATSP trial budget; 0 = solver default.
 *   vrptw_rounds    VRPTW ILS perturbations per chain; 0 = default.
 *   vrptw_restarts  VRPTW independent chains; 0 = default.
 *   veh_penalty     VRPTW per-route penalty biasing toward fewer vehicles.
 *   threads         0 or 1 = single-threaded deterministic; >1 = parallel
 *                   SISR islands (CVRP only; the result depends on the count).
 *   reserved        Must be zero.
 */
typedef struct commiv_options {
    uint64_t seed;
    uint64_t sisr_iters;
    uint64_t trials;
    uint64_t vrptw_rounds;
    uint64_t vrptw_restarts;
    uint64_t veh_penalty;
    uint32_t threads;
    uint32_t reserved;
} commiv_options;

/* Library version, e.g. "0.2.0". Static storage; do not free. */
const char *commiv_version(void);

/* Directed CVRP with an uncapped fleet (SISR solver).
 *   matrix       (n_customers+1)^2 entries, row-major, depot = node 0.
 *   demand       n_customers+1 entries, demand[0] = 0.
 *   capacity     per-vehicle capacity (> 0).
 *   options      may be NULL for defaults.
 *   out          receives the solution handle on COMMIV_OK.
 */
int commiv_solve_cvrp(const uint32_t *matrix, size_t n_customers,
                      const uint32_t *demand, uint32_t capacity,
                      const commiv_options *options, commiv_routes **out);

/* Directed VRPTW. Same contract as commiv_solve_cvrp, plus per-node arrays of
 * n_customers+1 entries: ready/due bound the START of service (due[0] is the
 * depot horizon every vehicle must return by), service is the service duration
 * (service[0] = 0). Waiting before a window opens is free. */
int commiv_solve_vrptw(const uint32_t *matrix, size_t n_customers,
                       const uint32_t *demand, uint32_t capacity,
                       const uint32_t *ready, const uint32_t *due,
                       const uint32_t *service,
                       const commiv_options *options, commiv_routes **out);

/* Directed TSP (ATSP) over an n x n matrix. out_tour must hold n entries and
 * receives the visit order (a permutation of 0..n-1); out_cost receives the
 * directed tour length. */
int commiv_solve_atsp(const uint32_t *matrix, size_t n,
                      const commiv_options *options,
                      uint32_t *out_tour, uint64_t *out_cost);

/* Solution accessors. Route r is the customer visit order of vehicle r; the
 * depot is implied at both ends. Pointers returned by commiv_routes_get stay
 * valid until commiv_routes_free. */
uint64_t commiv_routes_cost(const commiv_routes *routes);
size_t commiv_routes_count(const commiv_routes *routes);
size_t commiv_routes_len(const commiv_routes *routes, size_t route);
const uint32_t *commiv_routes_get(const commiv_routes *routes, size_t route);
void commiv_routes_free(commiv_routes *routes);

#ifdef __cplusplus
}
#endif

#endif /* COMMIV_H */
