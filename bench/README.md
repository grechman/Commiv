# Commiv benchmark status

Bench SHA: f7bca03 · updated 2026-09-02 · auregat

The Windows/WSL2 machine that produced every number before 2026-09-01 is gone.
The frozen baseline was re-measured on auregat (Debian 13, Ryzen 5 2600X, Zig
0.16.0, `taskset -c 2`) at `887e520`: 2388 / 2383 ms. Rows dated earlier keep
their WSL2 numbers and are not comparable to auregat times.

## Current best primary result

| workload | frozen baseline | candidate | result identity | delta |
|---|---:|---:|---|---:|
| Li & Lim `lr2_10_1`, 20k PDPTW iterations, seed 1, one thread | 2384 / 2401 / 2392 / 2442 ms (`887e520` on auregat, 5 runs each) | 1130 / 1138 / 1140 ms (`8c32bf3`, 5 runs each, quiet window) | exact objective/fleet/duration/wait (25 veh, 46264.058, 153079.687, 96815.629) and byte-identical responses on the 52-case `tools/rest_corpus.py` set | **52.6% / 52.6% / 52.3% less time** |
| same instance, money mode (`--time-pen 1 --veh-pen 280000`) | 2962 / 2948 ms (`887e520`) | 1660 / 1672 / 1682 ms (`8c32bf3`) | exact 23 veh / 65947.311 / 121213.662 / 45266.351 | **44.0% / 43.3% / 43.2% less time** |
| GH `c1_10_1`, 300k VRPTW SISR iterations, seed 1, one thread (`bench/run_fixed.py vrptw`) | 2427 / 2426 / 2423 ms (`4312d28`) | 2123 / 2114 / 2106 ms (`b233107`) | exact 100 veh / 42504.61 | **12.5% / 12.9% / 13.1% less time** |
| Uchoa `X-n1001-k43`, 600k CVRP SISR iterations, seed 1, one thread (`bench/run_fixed.py cvrp`) | 2486 / 2487 / 2481 ms (`4312d28`) | 2048 / 2037 / 2048 ms (`9be2099`) | exact cost 74102 / 43 routes | **17.6% / 18.1% / 17.5% less time** |
| same, previous machine (WSL2) | 3682 / 3716 ms (`1450221`) | 2913 / 2872 ms (`ffa3d5e`) | exact objective/fleet/duration/wait and byte-identical HTTP response | 20.9% / 22.7% less time |

Reproduce the maintained harnesses (all on an otherwise idle auregat; a 5-core
grid on the same box moves the PDPTW tip from 1190 to 1300-1500 ms):

```bash
taskset -c 2 python3 bench/run.py --build --runs 5 --iters 20000 --seed 1
taskset -c 2 python3 bench/run.py --runs 5 --iters 20000 --seed 1 --time-pen 1 --veh-pen 280000
taskset -c 2 python3 bench/run_fixed.py vrptw --build --runs 5 --iters 300000 --seed 1
taskset -c 2 python3 bench/run_fixed.py cvrp --build --runs 5 --iters 600000 --seed 1
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
| Incremental PDPTW `freshen` (auregat) | 3.7% / 4.1% less time in ordinary mode, none in money, exact result | `f6ccf00`, `885695d` |
| Prefix-distance route arcs plus transposed rows for fixed-target lookups (auregat) | money 7.4-9.5% less time on top of the arcs, ordinary 0.6-2.4%; exact result; one shared copy per solve | `edf23fc`, `8c32bf3`, `f7bca03` |
| VRPTW direct blink draw (auregat) | 12.5% / 12.9% / 13.1% less time, exact result | `b233107` |
| CVRP direct blink draw (auregat) | 17.6% / 18.1% / 17.5% less time, exact result | `9be2099` |

## Dead/reverted

| lever | result | reverted by |
|---|---|---|
| Scalar VRPTW feasibility | 1.0% slower | `7cc58d4` |
| Duplicate converged cleanup shortcut | only 1.66% mean; inconsistent at 2% margin | `fd16622` |
| Freeing allocator for parallel CVRP SISR | 2.55% slower, no RSS win | `60bfce2` |
| Lseg-free money-mode pair insertion | mean -1.97%, absent in one of four replicates | not landed |
| Precomputed dropoff-plus-suffix merges (money) | +14.1% slower | `08bdec4` |
| Transposed rows without the prefix-arc lookup | +5.5% to +13.3% cycles | superseded by `8c32bf3` |
| Suffix arcs from prefix distances inside `freshen` | +0.8% to +2.6% ordinary, noise in money | `b0b0779` |

## Equal-wall old/new quality grids (2026-09-02)

`bench/equal_wall_runner.py` reruns the 352-cell Li & Lim money and ordinary
grids with the `887e520` and `4312d28` binaries paired per cell at equal wall;
`bench/equal_wall_score.py` scores them. Money: new W/T/L 63/289/0,
$10,975,931.77 to $10,969,375.04 (-0.060%), rows in `bench/equal-wall-money.jsonl`.
The ordinary grid runs both binaries side by side at `PB_THREADS=5`, so its
absolute numbers are not comparable with the 2026-08-19 10-thread grid.

## Final quality audit

The completed post-fix campaign is documented in
[`OPPOSITION_FINAL.md`](OPPOSITION_FINAL.md), with 1,611 raw rows,
1,589 journaled cells, full 352-cell PDPTW and money grids, and a derived
1,670-row de-duplicated opposition view. The refreshed academic-money result is
VROOM +$1,266,042/+11.540%; the primary PDPTW comparison is Commiv 283/53/16
when completion, fleet, and distance are respected. Nine of 15 new GH PyVRP
rows failed exact schedule validation and are explicitly excluded rather than
silently scored.

## Money mode and the PDPTW speedup

Experiment 7's win replaces the time-window algebra with scalar arrival labels.
Money mode (`PB_TIMEPEN=1`) prices the merged route duration those labels cannot
produce, so it keeps the full Tws summary and the fast path is gated off.
Money-mode results are bit-identical on `18b3a5f`, on `66d73f5`, and under the
rejected experiment 13. Experiments 14, 15 and 22 do apply to money mode
(seed construction, the blink draw and the matrix access pattern are
objective-independent): 2962 / 2948 ms at `887e520` became 1660 / 1672 / 1682 ms
at `8c32bf3` with the same 23-vehicle signature. See `bench/EXPERIMENTS.md`.

## Queue

| priority | lever | required gate |
|---:|---|---|
| 0 | PDPTW `install` re-walks the route for `arcSum` and, in money mode, `routeDuration` (1.6-3.3% self time) although the chosen candidate's distance and duration are already known to the evaluator | exact signature both objectives; likely below the 3% margin alone |
| 0 | `core/neighbors.zig` full `pdq` sort per row (8% of a 300k-iteration VRPTW run, 1.6% at 2M): a bounded top-k cannot reproduce pdq's unstable tie order, so it is a quality-neutral behaviour change, not an exact lever | equal-wall quality grid, not the frozen harness |
| 1 | Replace quadratic all-roots MST bottleneck candidate traversal | exact candidate sets/tours, large sparse and dense regimes |
| 2 | Make geometric nearest-candidate construction use its spatial grid | cached and on-the-fly TSP regimes, no quality loss |
| 3 | Reduce break-aware PDPTW insertion's cubic confirmation work | differential break feasibility and route hashes |

## Known constraints

- Debug `zig build test` skips the two 1M-iteration marathon tests (`error.SkipZigTest`); the ReleaseFast CI job still runs them. Those two tests were 198 s of a 6:03 cold Debug run (compile + run) on auregat; the warm-cache Debug run is now 2:44, ReleaseFast 1:50 either way.
- REST identity gate: `uv run --no-project python tools/rest_corpus.py <tree> <port> <out.json>` starts that tree's `zig-out/bin/commiv-serve` and hashes 52 responses (18 solves, 34 error paths); diff the JSON of two trees.
- Branch state 2026-09-02: `zig build test` exits 0 in Debug and ReleaseFast on auregat at the tip; the 52-case corpus is byte-identical to `887e520`. Timing rows are only valid from an otherwise idle box; capture `zig build test` exit codes without a pipe (`${PIPESTATUS}` is empty under zsh).

- Native Windows benchmark binaries currently depend on Linux clocks; run under WSL2.
- Do not time while the server is running a game or another heavy unit.
- Clean timing uses fixed iterations, explicit seeds/thread counts, CPU affinity, and no profiler.
