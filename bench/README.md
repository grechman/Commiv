# Commiv benchmark status

Bench SHA: 3f556d7 · updated 2026-09-01 · auregat

The Windows/WSL2 machine that produced every number before 2026-09-01 is gone.
The frozen baseline was re-measured on auregat (Debian 13, Ryzen 5 2600X, Zig
0.16.0, `taskset -c 2`) at `887e520`: 2388 / 2383 ms. Rows dated earlier keep
their WSL2 numbers and are not comparable to auregat times.

## Current best primary result

| workload | frozen baseline | candidate | result identity | delta |
|---|---:|---:|---|---:|
| Li & Lim `lr2_10_1`, 20k PDPTW iterations, seed 1, one thread | 2388 / 2383 ms (`887e520` on auregat, 5 runs each) | 1187 / 1206 ms (`3f556d7`, 5 runs each) | exact objective/fleet/duration/wait (25 veh, 46264.058, 153079.687, 96815.629) and byte-identical responses on the 52-case `tools/rest_corpus.py` set | **50.3% / 49.4% less time** |
| same, previous machine (WSL2) | 3682 / 3716 ms (`1450221`) | 2913 / 2872 ms (`ffa3d5e`) | exact objective/fleet/duration/wait and byte-identical HTTP response | 20.9% / 22.7% less time |

Reproduce the maintained primary harness:

```bash
taskset -c 2 python3 bench/run.py --build --runs 5 --iters 20000 --seed 1
```

The command emits raw runs to stderr and one JSON object to stdout. See
`bench/config.json` for the frozen settings and `bench/EXPERIMENTS.md` for every run.

## Kept wins

| lever | measured result | commit |
|---|---:|---|
| PDPTW HTTP work on a freeing allocator | solve/dispatch/VROOM RSS -92.6%/-67.0%/-68.2%, exact responses | `86f0b61` |
| Packed PDPTW snapshots | peak RSS -91.4%, exact response | `1f74311` |
| Parallel HGS freeing workspaces | peak RSS -85.2%, exact result | `99a63a5` |
| Large ATSP honors requested trials | 37.4x at trials=1; equal-work output exact | `386b18c` |
| Ordinary PDPTW scalar insertion | 20.9% / 22.7% less time, exact trajectory | `1e8b327`, `ffa3d5e` |
| NumPy zero-copy bridge | 66.2x conversion, peak RSS -60.3% | `3b2681d` |
| Large native ATSP bounded top-k | 2.12x, exact unique-key result | `003597b` |
| Fleet-capped Split route bound | 4.86x, peak RSS -71.8%, exact result | `c272171` |
| Parallel VRPTW freeing workspaces | peak RSS -80.2%, exact result | `95d22a2` |
| Quadratic PDPTW seed construction (auregat) | 38.0% / 37.9% less time, exact result | `8dc885d` |
| Direct Xoshiro blink draw (auregat) | 20.0% / 17.5% / 17.4% less time vs `8dc885d`, exact result | `5b6ffe0` |

## Dead/reverted

| lever | result | reverted by |
|---|---|---|
| Scalar VRPTW feasibility | 1.0% slower | `7cc58d4` |
| Duplicate converged cleanup shortcut | only 1.66% mean; inconsistent at 2% margin | `fd16622` |
| Freeing allocator for parallel CVRP SISR | 2.55% slower, no RSS win | `60bfce2` |
| Lseg-free money-mode pair insertion | mean -1.97%, absent in one of four replicates | not landed |

## Final quality audit

The completed post-fix campaign is documented in
[`OPPOSITION_FINAL.md`](OPPOSITION_FINAL.md), with 1,611 raw rows,
1,589 journaled cells, full 352-cell PDPTW and money grids, and a derived
1,670-row de-duplicated opposition view. The refreshed academic-money result is
VROOM +$1,266,042/+11.540%; the primary PDPTW comparison is Commiv 283/53/16
when completion, fleet, and distance are respected. Nine of 15 new GH PyVRP
rows failed exact schedule validation and are explicitly excluded rather than
silently scored.

## Money mode is not covered by the PDPTW speedup

Experiment 7's win replaces the time-window algebra with scalar arrival labels.
Money mode (`PB_TIMEPEN=1`) prices the merged route duration those labels cannot
produce, so it keeps the full Tws summary and the fast path is gated off.
Money-mode results are bit-identical on `18b3a5f`, on `66d73f5`, and under the
rejected experiment 13. See `bench/EXPERIMENTS.md` for the measurement.

## Queue

| priority | lever | required gate |
|---:|---|---|
| 0 | PDPTW `freshen` rebuilds six prefix/suffix arrays per touched route (7% of the frozen run); the fast path never reads `suf_l` | exact signature on the frozen harness, above the 3% margin |
| 1 | Replace quadratic all-roots MST bottleneck candidate traversal | exact candidate sets/tours, large sparse and dense regimes |
| 2 | Make geometric nearest-candidate construction use its spatial grid | cached and on-the-fly TSP regimes, no quality loss |
| 3 | Reduce break-aware PDPTW insertion's cubic confirmation work | differential break feasibility and route hashes |

## Known constraints

- Debug `zig build test` skips the two 1M-iteration marathon tests (`error.SkipZigTest`); the ReleaseFast CI job still runs them. Those two tests were 198 s of a 6:03 cold Debug run (compile + run) on auregat; the warm-cache Debug run is now 2:44, ReleaseFast 1:50 either way.
- REST identity gate: `uv run --no-project python tools/rest_corpus.py <tree> <port> <out.json>` starts that tree's `zig-out/bin/commiv-serve` and hashes 52 responses (18 solves, 34 error paths); diff the JSON of two trees.

- Native Windows benchmark binaries currently depend on Linux clocks; run under WSL2.
- Do not time while the server is running a game or another heavy unit.
- Clean timing uses fixed iterations, explicit seeds/thread counts, CPU affinity, and no profiler.
