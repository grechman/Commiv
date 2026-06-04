# commiv

Zig 0.16.0 library-style implementation of symmetric TSP solving.

The solver now has a real bounded Lin-Kernighan-style core, not just 2-opt. It still is not full LKH-3.

## Implemented Solver

For `n <= 10`, `solve` uses exact brute force with node 0 fixed.

For larger instances, `solve` runs deterministic multi-start heuristic search:

- nearest-neighbor/randomized initial tours from a fixed seed
- nearest-distance candidate rows with deterministic tie handling
- optional 2-opt and one-node Or-opt warmup
- bounded variable-depth sequential LK search
- alternating removed/added edges `x_i`/`y_i`
- positive partial gain pruning
- candidate-set branching for added edges
- two tour-neighbor choices for removed edges
- closing-edge gain evaluation at each depth
- generic k-opt feasibility/application that accepts only one Hamiltonian cycle
- reusable solver workspace with no per-move heap allocation

Default LK depth is `5`. Use `SolveOptions.lk_max_depth` and `SolveOptions.lk_backtrack_limit` to trade runtime for search depth.

## Not Implemented

This is not full Helsgaun LKH/LKH-3 parity. Missing pieces include:

- alpha-nearness candidate generation
- 1-tree ascent and Held-Karp penalties
- Delaunay/quadrant candidate sets
- non-sequential moves, patching, `Gain23`, and recombination
- tree/segment-list tour representation for very large tours
- ATSP, VRP, time windows, and other LKH-3 problem classes

Candidate generation is nearest-distance only.

## Supported Input

- `TYPE: TSP`
- `EDGE_WEIGHT_TYPE: EUC_2D`
- `EDGE_WEIGHT_TYPE: CEIL_2D`
- `EDGE_WEIGHT_TYPE: EXPLICIT` with `EDGE_WEIGHT_FORMAT: FULL_MATRIX`

The TSPLIB parser enforces `max_dimension` and `max_matrix_weights`. It does not preallocate a full dense matrix for incomplete `FULL_MATRIX` input.

## Public API

Import the package module:

```zig
const commiv = @import("commiv");
```

Main entry points:

- `commiv.parseTsplib(allocator, bytes, .{ .diagnostic = &diag })`
- `commiv.solve(allocator, &problem, .{ .seed = 1 })`
- `commiv.bruteForce(allocator, &problem, .{ .max_nodes = 10 })`
- `problem.validateTour(tour)`
- `problem.tourLength(tour)`

`solve` returns `SolveResult` with `tour`, `length`, and `SolveStats`. Stats include `trials`, `warmup_moves`, `lk_attempts`, `lk_search_nodes`, `lk_moves`, `max_depth_reached`, `exact_permutations`, candidate width, and distance-cache usage. The old ambiguous heuristic `iterations` field was removed.

Distance caching is controlled by `SolveOptions.max_distance_cache_weights`, default `4_000_000` weights, about 16 MiB. Set it higher explicitly for larger dense coordinate caches, or `0` to disable coordinate distance caching.

## Verification

Use repo-local Zig caches:

```sh
ZIG_GLOBAL_CACHE_DIR=/home/grechman/commiv/.zig-cache-global \
ZIG_LOCAL_CACHE_DIR=/home/grechman/commiv/.zig-cache \
zig build test
```

Run the embedded example:

```sh
ZIG_GLOBAL_CACHE_DIR=/home/grechman/commiv/.zig-cache-global \
ZIG_LOCAL_CACHE_DIR=/home/grechman/commiv/.zig-cache \
zig build example
```

Current checks include:

- exact brute-force optimum on tiny instances
- deterministic fixed-seed heuristic output above the brute-force cutoff
- constructed matrix where 2-opt and one-node Or-opt are locally stuck but bounded LK improves the tour
- 12-node convex perimeter known optimum length `24`
- hardcoded TSPLIB `gr17` regression target length `2085`
- explicit 11-node `u32.max` edge-weight regression
- candidate row sanity: no self nodes or duplicates
- coordinate-cache smoke test proving cached heuristic search performs zero uncached coordinate distance calls after candidate construction
- TSPLIB parser diagnostics, out-of-order node ids, unsupported input rejection, and parser resource limits

## References

Algorithm structure follows the basic LK outline from Keld Helsgaun, "An Effective Implementation of the Lin-Kernighan Traveling Salesman Heuristic": choose `x_i` tour edges and `y_i` non-tour edges, keep positive gain, evaluate closing moves, and repeat from improvements. This implementation deliberately keeps only a bounded sequential subset.

Useful comparison implementations:

- LKH/LKH-3 by Keld Helsgaun
- Concorde `linkern`
- Neto's LK implementation
