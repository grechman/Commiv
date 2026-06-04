# commiv

Zig 0.16.0 library-style implementation of a symmetric TSP heuristic: candidate sets, deterministic multi-start tours, 2-opt improvement, one-node Or-opt moves, best-tour retention, and exact brute force for tiny instances.

This is not Lin-Kernighan or LKH. Real LK/LKH needs bounded variable-depth sequential edge exchanges and deeper machinery such as gain sequences, feasibility checks, alpha-nearness candidate generation, 1-trees, patching, and recombination. The current code is clean Zig built for correctness, repeatable tests, and further extension without lying about the algorithm.

## Supported Input

- `TYPE: TSP`
- `EDGE_WEIGHT_TYPE: EUC_2D`
- `EDGE_WEIGHT_TYPE: CEIL_2D`
- `EDGE_WEIGHT_TYPE: EXPLICIT` with `EDGE_WEIGHT_FORMAT: FULL_MATRIX`

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

`solve` returns `SolveResult`, including the tour, length, iteration count, and `SolveStats`. It uses brute force automatically for instances up to 10 nodes and the heuristic path above that.

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

Current reference checks include:

- exact brute-force optimum on tiny instances
- 12-node convex perimeter instance with known optimum length `24`
- TSPLIB `gr17` explicit matrix converted to `FULL_MATRIX`, known optimum length `2085`
- explicit 11-node max-weight matrix regression for `u32.max` edge weights
- deterministic repeated solver output for fixed seeds
- TSPLIB parser diagnostics, out-of-order node id handling, missing/unsupported input rejection, and parser resource limits

## Reference Implementations

Useful references for comparison and possible future LK/LKH implementation:

- LKH/LKH-3 by Keld Helsgaun
- Concorde `linkern`
- Neto's LK implementation

Use them to compare solution quality and runtime. Do not claim LK/LKH until bounded variable-depth sequential exchanges and the supporting tests are actually implemented.
