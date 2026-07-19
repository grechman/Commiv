/* commiv C API — near-optimal directed (asymmetric) TSP / ATSP / CVRP / VRPTW / PDPTW routes.
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
 *   sisr_iters      SISR ruin-and-recreate iterations (CVRP and VRPTW); 0 = 300000.
 *   trials          ATSP trial budget; 0 = solver default.
 *   veh_penalty     Per-route penalty biasing toward fewer vehicles (VRPTW, PDPTW).
 *   threads         0 or 1 = single-threaded deterministic; >1 = parallel
 *                   SISR chains (CVRP and VRPTW; the result depends on the count).
 *   fleet_min       Nonzero = run the hierarchical fleet-minimization driver
 *                   (VRPTW, PDPTW): first minimize vehicle count, then distance
 *                   within that count. Needs a wall budget; see wall_ms. 0 = off.
 *   wall_ms         Wall-clock budget in milliseconds for SISR/fleet-min
 *                   (VRPTW, PDPTW). 0 = no wall cap (bounded by sisr_iters only).
 *                   Required to be > 0 when fleet_min is set (else defaults apply).
 *   max_vehicles    Hard cap on the number of routes (VRPTW, PDPTW). 0 = uncapped.
 *                   For PDPTW a positive cap runs the pinned driver (exact count).
 *   time_penalty    PDPTW ONLY: money objective. Cost charged per matrix time-unit
 *                   of route DURATION (travel + service + unavoidable waiting),
 *                   added to distance and veh_penalty. 0 = pure distance.
 *                   IGNORED by CVRP and VRPTW.
 *   max_route_duration  Shift-length cap (VRPTW, PDPTW): a hard bound on each
 *                   route's depot->..->depot DURATION (travel + service +
 *                   unavoidable waiting). No route in the result exceeds it.
 *                   0 = uncapped. IGNORED by CVRP.
 *   break_duration / break_earliest / break_latest  Driver break (PDPTW ONLY,
 *                   v1). break_duration > 0: every route whose depart-at-0
 *                   schedule finishes after break_earliest must contain ONE
 *                   break of break_duration time units STARTING within
 *                   [break_earliest, break_latest]. The break absorbs waiting
 *                   first, delays later stops by the remainder, and counts
 *                   into route duration (so the money objective prices it).
 *                   All-zero = no break. Unsupported with fleet_min /
 *                   max_vehicles (COMMIV_ERR_INVALID_ARGUMENT); IGNORED by
 *                   CVRP and VRPTW.
 */
typedef struct commiv_options {
    uint64_t seed;
    uint64_t sisr_iters;
    uint64_t trials;
    uint64_t veh_penalty;
    uint32_t threads;
    uint32_t fleet_min;      /* was reserved; nonzero = fleet-minimization driver */
    uint64_t wall_ms;        /* wall-clock budget (ms); 0 = none */
    uint64_t max_vehicles;   /* hard route cap; 0 = uncapped */
    uint64_t time_penalty;   /* PDPTW money knob; 0 = off */
    uint64_t max_route_duration; /* shift-length cap (VRPTW/PDPTW); 0 = uncapped */
    uint32_t break_duration; /* driver break length (PDPTW); 0 = no break */
    uint32_t break_earliest; /* break must START within [earliest, latest] */
    uint32_t break_latest;
    uint32_t reserved;       /* keeps the struct 8-aligned at 88 bytes */
} commiv_options;

/* Library version, e.g. "0.4.0". Static storage; do not free. */
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

/* Directed PDPTW (pickup-and-delivery with time windows), SISR solver.
 *
 * dim = 2*n_pairs + 1 nodes: node 0 is the depot, nodes 1..dim-1 are pickups
 * and deliveries. Each of the n_pairs requests is one pickup node and one
 * delivery node carried on the SAME route, pickup before delivery, capacity
 * respected along the whole route. Objective: minimize directed travel
 * distance, plus (options->time_penalty > 0) a money charge on route duration.
 *
 *   matrix        dim*dim entries, row-major, directed, depot = node 0.
 *   n_pairs       number of pickup/delivery requests (> 0).
 *   pickup_node   n_pairs entries; pickup_node[i] in 1..dim-1.
 *   delivery_node n_pairs entries; delivery_node[i] in 1..dim-1.
 *                 Every node 1..dim-1 must appear exactly once across the two
 *                 arrays; a node may not be its own partner.
 *   demand        n_pairs entries; load of request i (> 0, <= capacity).
 *   capacity      per-vehicle capacity (> 0).
 *   ready,due,service   dim entries each; bound the START of service at each
 *                 node. ready[0]=0, due[0]=depot horizon (every vehicle returns
 *                 by then), service[0]=0. Waiting before a window opens is free.
 *   options       may be NULL for defaults. Honors seed, sisr_iters, veh_penalty,
 *                 wall_ms, max_vehicles, fleet_min, time_penalty (the money knob).
 *   out           receives the solution handle on COMMIV_OK.
 *
 * Returns COMMIV_OK, or COMMIV_ERR_INVALID_ARGUMENT (null/shape/index),
 * COMMIV_ERR_INFEASIBLE (demand > capacity, or no complete solution under a
 * vehicle cap), COMMIV_ERR_OUT_OF_MEMORY, COMMIV_ERR_INTERNAL. */
int commiv_solve_pdptw(const uint32_t *matrix, size_t n_pairs,
                       const uint32_t *pickup_node, const uint32_t *delivery_node,
                       const uint32_t *demand, uint32_t capacity,
                       const uint32_t *ready, const uint32_t *due,
                       const uint32_t *service,
                       const commiv_options *options, commiv_routes **out);

/* Heterogeneous-fleet PDPTW: commiv_solve_pdptw with a vehicle-type table
 * instead of one uniform capacity.
 *
 *   veh_type_capacity    n_types entries, each > 0: load capacity of type t.
 *   veh_type_fixed_cost  n_types entries: per-route fixed cost of type t —
 *                        replaces options->veh_penalty in the objective.
 *   veh_type_count       n_types entries: max simultaneous routes of type t;
 *                        0 = unlimited.
 *   n_types              1..8.
 *
 * Every route in the answer is served by exactly one type, chosen by the
 * solver (cheapest fixed cost that fits, refined by the search); read the
 * assignment back with commiv_routes_type(). Route loads are bounded by the
 * route's OWN type capacity. demand[i] must fit the largest type
 * (COMMIV_ERR_INFEASIBLE otherwise). options->fleet_min and
 * options->max_vehicles are not supported here (COMMIV_ERR_INVALID_ARGUMENT):
 * bound the fleet with per-type counts instead. */
int commiv_solve_pdptw_typed(const uint32_t *matrix, size_t n_pairs,
                             const uint32_t *pickup_node, const uint32_t *delivery_node,
                             const uint32_t *demand,
                             const uint32_t *ready, const uint32_t *due,
                             const uint32_t *service,
                             const uint32_t *veh_type_capacity,
                             const uint64_t *veh_type_fixed_cost,
                             const uint32_t *veh_type_count, size_t n_types,
                             const commiv_options *options, commiv_routes **out);

/* Rolling-horizon PDPTW re-solve: the same instance contract as
 * commiv_solve_pdptw, plus the CURRENT plan and its LOCKED prefixes.
 *
 *   cur_offsets   n_routes + 1 entries, monotone non-decreasing,
 *                 cur_offsets[0] == 0. Route r of the current plan is
 *                 cur_nodes[cur_offsets[r] .. cur_offsets[r+1]].
 *   n_routes      number of routes in the current plan; 0 is a legal cold
 *                 start (dispatch from an empty plan) — cur_offsets,
 *                 cur_nodes, and locked are unused then and may be NULL.
 *   cur_nodes     cur_offsets[n_routes] entries; every value in 1..dim-1,
 *                 each node appearing at most once across all routes (a node
 *                 absent from the current plan is fine: unrouted / new).
 *   locked        n_routes entries; locked[r] is how many of route r's
 *                 LEADING stops are committed (in progress or already
 *                 served) and must not move in the result. A locked
 *                 delivery's pickup must also be locked, in the SAME route
 *                 (COMMIV_ERR_INVALID_ARGUMENT otherwise).
 *
 * Node-id stability: when new orders arrive, rebuild a LARGER instance that
 * keeps every existing node's index unchanged and appends the new pairs at
 * the end — the old routes stay valid warm input for the next dispatch call.
 *
 * options->fleet_min and options->max_vehicles are IGNORED here: dispatch
 * keeps the current fleet shape (routes open/close only as ruin-and-recreate
 * naturally does around the locked prefixes). Every other option (seed,
 * sisr_iters, veh_penalty, time_penalty, wall_ms) behaves as in
 * commiv_solve_pdptw. */
int commiv_solve_pdptw_dispatch(const uint32_t *matrix, size_t n_pairs,
                                const uint32_t *pickup_node, const uint32_t *delivery_node,
                                const uint32_t *demand, uint32_t capacity,
                                const uint32_t *ready, const uint32_t *due,
                                const uint32_t *service,
                                const size_t *cur_offsets, size_t n_routes,
                                const uint32_t *cur_nodes, const size_t *locked,
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
/* Vehicle-type index of `route` (into the veh_type_* arrays given to
 * commiv_solve_pdptw_typed). COMMIV_NO_TYPE for uniform-fleet solves or an
 * out-of-range route. */
#define COMMIV_NO_TYPE UINT32_MAX
uint32_t commiv_routes_type(const commiv_routes *routes, size_t route);
void commiv_routes_free(commiv_routes *routes);

#ifdef __cplusplus
}
#endif

#endif /* COMMIV_H */
