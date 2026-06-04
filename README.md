# commiv

Zig 0.16.0 library-style implementation of a symmetric TSP heuristic based on the practical Lin-Kernighan/LKH shape: candidate sets, deterministic multi-start tours, 2-opt improvement, bounded 3-edge Or-opt moves, and best-tour retention.

This is not a full LKH-3 clone. Full LKH includes deeper machinery such as alpha-nearness candidate generation, 1-trees, patching, and recombination. The current code is clean Zig built for correctness, repeatable tests, and further extension.

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

`solve` uses brute force automatically for instances up to 10 nodes and the heuristic path above that.

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
- deterministic repeated solver output for fixed seeds
- TSPLIB parser diagnostics and out-of-order node id handling

## Reference Implementations

Useful references for comparison and future extension:

- LKH/LKH-3 by Keld Helsgaun
- Concorde `linkern`
- Neto's LK implementation

Use them to compare solution quality and runtime. Do not claim LKH parity unless alpha-nearness, 1-tree candidate generation, patching, and deeper LK backtracking are actually implemented.
