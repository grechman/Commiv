# Commiv benchmark status

Bench SHA: pending · updated 2026-08-18 · windows-server-wsl2

## Current best primary result

| workload | frozen baseline | candidate | result identity | delta |
|---|---:|---:|---|---:|
| Li & Lim `lr2_10_1`, 20k PDPTW iterations, seed 1, one thread | 3686 ms median (`1450221`, n=20) | 3283 ms median (n=20) | exact objective/fleet/duration/wait and byte-identical HTTP response | **12.3% faster** |

Reproduce the maintained primary harness:

```bash
taskset -c 2 python3 bench/run.py --build --runs 5 --iters 20000 --seed 1
```

The command emits raw runs to stderr and one JSON object to stdout. See
`bench/config.json` for the frozen settings and `bench/EXPERIMENTS.md` for every run.

## Kept wins

| lever | measured result | commit |
|---|---:|---|
| PDPTW HTTP work on a freeing allocator | peak RSS -92.5%, exact response | `86f0b61` |
| Packed PDPTW snapshots | peak RSS -91.6%, exact response | `1f74311` |
| Parallel HGS freeing workspaces | peak RSS -85.2%, exact result | `99a63a5` |
| Large ATSP honors requested trials | 37.4x at trials=1; equal-work output exact | `386b18c` |
| Ordinary PDPTW scalar insertion | 12.3% faster, exact trajectory | `1e8b327`, `ffa3d5e` |
| NumPy zero-copy bridge | 66.2x conversion, peak RSS -60.3% | `3b2681d` |
| Large native ATSP bounded top-k | 2.12x, exact unique-key result | `003597b` |
| Fleet-capped Split route bound | 4.86x, peak RSS -71.8%, exact result | `c272171` |
| Parallel VRPTW freeing workspaces | peak RSS -80.2%, exact result | `95d22a2` |

## Dead/reverted

| lever | result | reverted by |
|---|---|---|
| Scalar VRPTW feasibility | 1.0% slower | `7cc58d4` |
| Duplicate converged cleanup shortcut | only 1.66% mean; inconsistent at 2% margin | `fd16622` |
| Freeing allocator for parallel CVRP SISR | 2.55% slower, no RSS win | `60bfce2` |

## Queue

| priority | lever | required gate |
|---:|---|---|
| 1 | Validate `/solve/pdptw/dispatch` and `/compat/vroom` allocator isolation | two replicated RSS runs and exact fixed-work response |
| 2 | Replace quadratic all-roots MST bottleneck candidate traversal | exact candidate sets/tours, large sparse and dense regimes |
| 3 | Make geometric nearest-candidate construction use its spatial grid | cached and on-the-fly TSP regimes, no quality loss |
| 4 | Reduce break-aware PDPTW insertion's cubic confirmation work | differential break feasibility and route hashes |

## Known constraints

- Native Windows benchmark binaries currently depend on Linux clocks; run under WSL2.
- Do not time while the server is running a game or another heavy unit.
- Clean timing uses fixed iterations, explicit seeds/thread counts, CPU affinity, and no profiler.
