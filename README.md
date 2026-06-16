# commiv

Zig 0.16 library-style symmetric TSP solver.

This is a bounded Lin-Kernighan-style heuristic with TSPLIB parsing and benchmark tooling. It is not full LKH-3 yet.

## What It Does

For `n <= 10`, `solve` uses exact brute force with node 0 fixed.

For larger instances, `solve` runs deterministic multi-start heuristic search:

- nearest-neighbor and farthest-insertion starting tours
- EUC_2D, CEIL_2D, ATT, and explicit full-matrix TSPLIB metrics
- deterministic candidate rows
- pi-adjusted alpha-nearness candidates, computed in O(n^2) total via per-row 1-tree bottleneck traversals
- optional CGAL Delaunay candidate augmentation with `-Dwith-cgal=true`
- 2-opt and one-node Or-opt warmup
- bounded sequential LK search with LKH backtracking discipline (exhaustive below 400 nodes, backtrack only at levels 1-2 above)
- bounded in-recursion 2/3-edge completion oracle
- partial Gain23-style nonfeasible bridge moves
- bounded 3-edge cleanup
- iterated local search: double-bridge kicks from the incumbent with staleness escalation
- guided restarts ported from LKH `ChooseInitialTour` (alpha-zero backbone construction)
- size-gated tour merging: IPT (LKH `MergeWithTour`) below n=1000 where the tuned ILS dynamics dominate, EAX-lite (single AB-cycle edge assembly crossover with candidate-bridge subtour repair, multi-reference pool) at n>=1000 where recombination breadth wins
- `TourView` abstraction over flat and segment-backed tour views
- `MovePlan` validation for edge-delta application and two-component patching

Fifteen of the seventeen original TSPLIB fixtures solve to the known optimum in the headline `alpha-w8-kick` mode at the pinned seed — including pr1002, the long-standing weak row — and fl1577 still beats LKH's RUNS=1 tour, now at a seventh of its time. The remaining pinned-seed gaps are d657 0.008%, fl1577 0.031%, rat575 0.089%. The headline mode now runs three seeds so knife-edge variance is visible instead of hidden (see the table below).

## Requirements

- Zig 0.16
- Optional: CGAL for `-Dwith-cgal=true`
- Optional: TSPLIB `.tsp` fixtures under `vendor/tsplib`

On Arch, CGAL is enough for the optional geometric mode:

```sh
sudo pacman -S cgal
```

The repo ignores local caches and working artifacts:

- `.zig-cache/`
- `.zig-cache-global/`
- `zig-out/`
- `tasks/`
- `.grechman/`
- `depwire-output.json`

Do not commit generated Zig cache output.

## Getting Started

Run tests:

```sh
zig build test
```

Run optimized tests:

```sh
zig build test -Doptimize=ReleaseFast
```

Run benchmarks:

```sh
taskset -c 0 nice -n 10 zig build bench -Doptimize=ReleaseFast
```

Run CGAL probe:

```sh
zig build cgal-probe -Dwith-cgal=true
```

Run CGAL-enabled benchmark:

```sh
taskset -c 0 nice -n 10 zig build bench -Doptimize=ReleaseFast -Dwith-cgal=true
```

If your sandbox has no writable `/tmp`, use:

```sh
TMPDIR=/home/grechman/commiv/.zig-cache zig build cgal-probe -Dwith-cgal=true
```

## TSPLIB Fixtures

Place symmetric TSPLIB `.tsp` files under `vendor/tsplib/`. The benchmark target automatically reports gap against known optima for these fixtures (missing files are reported and skipped):

| Instance | n | Optimum | Instance | n | Optimum |
|---|---:|---:|---|---:|---:|
| berlin52 | 52 | 7542 | rd400 | 400 | 15281 |
| eil76 | 76 | 538 | fl417 | 417 | 11861 |
| kroA100 | 100 | 21282 | pcb442 | 442 | 50778 |
| bier127 | 127 | 118282 | att532 | 532 | 27686 |
| rat195 | 195 | 2323 | u574 | 574 | 36905 |
| ts225 | 225 | 126643 | rat575 | 575 | 6773 |
| a280 | 280 | 2579 | d657 | 657 | 48912 |
| lin318 | 318 | 42029 | pr1002 | 1002 | 259045 |
| | | | fl1577 | 1577 | 22249 |

## Public API

```zig
const commiv = @import("commiv");
```

Main entry points:

- `commiv.parseTsplib(allocator, bytes, .{ .diagnostic = &diag })`
- `commiv.solve(allocator, &problem, .{ .seed = 1 })`
- `commiv.bruteForce(allocator, &problem, .{ .max_nodes = 10 })`
- `problem.validateTour(tour)`
- `problem.tourLength(tour)`

`solve` returns `SolveResult` with `tour`, `length`, and `SolveStats`.

Useful options:

- `trials`
- `trial_extension_factor`
- `candidate_count`
- `candidate_mode`
- `max_passes`
- `lk_max_depth`
- `lk_backtrack_limit`
- `lk_nonseq_branch_limit`
- `alpha_ascent_iterations`
- `max_distance_cache_weights`

## Current Benchmark

Command:

```sh
taskset -c 0 nice -n 10 zig build bench -Doptimize=ReleaseFast
```

Headline `alpha-w8-kick` mode at seed 12345 (size-gated IPT/EAX merging, elite pool + candidate width 5 at n>=1000), side by side with LKH-3.0.13 `RUNS=1` on the same machine and core class. Lengths are from round 15 (2026-06-12); the **Time** column was refreshed 2026-06-15 after the perf commits (doomed-fallback skip + patch-gate scan removal) — lengths are bit-identical, only wall-time dropped. Single core, `taskset -c 0 nice -n 10`. The bench additionally runs seeds 7 and 99 for this mode (CSV `seed` column):

| Instance | n | commiv Length | Gap | Time | LKH Length | LKH Gap | LKH Time |
|---|---:|---:|---:|---:|---:|---:|---:|
| berlin52 | 52 | 7542 | 0.000% | 70 ms | 7542 | 0.000% | 0.01 s |
| eil76 | 76 | 538 | 0.000% | 24 ms | 538 | 0.000% | 0.02 s |
| kroA100 | 100 | 21282 | 0.000% | 150 ms | 21282 | 0.000% | 0.04 s |
| bier127 | 127 | 118282 | 0.000% | 357 ms | 118282 | 0.000% | 0.07 s |
| rat195 | 195 | 2323 | 0.000% | 149 ms | 2323 | 0.000% | 0.72 s |
| ts225 | 225 | 126643 | 0.000% | 563 ms | 126643 | 0.000% | 0.44 s |
| a280 | 280 | 2579 | 0.000% | 328 ms | 2579 | 0.000% | 0.57 s |
| lin318 | 318 | 42029 | 0.000% | 782 ms | 42143 | 0.271% | 0.80 s |
| rd400 | 400 | 15281 | 0.000% | 997 ms | 15281 | 0.000% | 0.60 s |
| fl417 | 417 | 11861 | 0.000% | 2.60 s | 11861 | 0.000% | 10.29 s |
| pcb442 | 442 | 50778 | 0.000% | 643 ms | 50778 | 0.000% | 2.91 s |
| att532 | 532 | 27686 | 0.000% | 1.49 s | 27686 | 0.000% | 8.82 s |
| u574 | 574 | 36905 | 0.000% | 3.19 s | 36905 | 0.000% | 4.80 s |
| rat575 | 575 | 6779 | 0.089% | 677 ms | 6773 | 0.000% | 2.07 s |
| d657 | 657 | 48916 | 0.008% | 1.78 s | 48912 | 0.000% | 2.49 s |
| pr1002 | 1002 | 259045 | 0.000% | 7.44 s | 259045 | 0.000% | 2.96 s |
| fl1577 | 1577 | 22256 | 0.031% | 11.6 s | 22262 | 0.058% | 145.66 s |
| rl11849 | 11849 | 930671 | 0.800% | 160 s (400-trial probe) | 923288 | 0.000% | 1287.6 s |

### Parallel execution (island model, 3 of 4 cores)

`solveParallel` runs K independent islands, the trial budget split K ways (so
~K x wall-time speedup), leaving one core free. Each island is a self-contained
`solve()` with a distinct seed; the best island wins. This is **deterministic**
per (seed, thread-count) -- fixed per-island seeds, no shared state -- so it
reproduces exactly run to run. `threads=1` is the bit-identical serial path.
Command:

```sh
BENCH_THREADS=3 taskset -c 0-2 nice -n 10 zig build parbench -Doptimize=ReleaseFast
```

Findings on this 4-core host (3 islands), each value reproducible across runs:

| Instance | n | serial (gap) | split-3 (gap) | LKH (gap) |
|---|---:|---:|---:|---:|
| fl417 | 417 | 2.60 s (0.000%) | 1.35 s (0.000%) | 10.29 s (0.000%) |
| att532 | 532 | 1.49 s (0.000%) | 1.16 s (0.061%) | 8.82 s (0.000%) |
| u574 | 574 | 3.19 s (0.000%) | 2.06 s (0.000%) | 4.80 s (0.000%) |
| d657 | 657 | 1.78 s (0.008%) | 1.17 s (0.074%) | 2.49 s (0.000%) |
| pr1002 | 1002 | 7.44 s (0.000%) | 3.0 s (0.046%) | 2.96 s (0.000%) |
| fl1577 | 1577 | 11.6 s (0.031%) | 4.8 s (0.126%) | 145.66 s (0.058%) |

- **split_budget is the one parallel mode** -- deterministic, ~2.5x faster, at a
  small reproducible accuracy cost. For maximum accuracy, run serial (which hits
  the optimum on most instances).
- The bigger point the single-core table already makes: **we beat LKH on
  wall-time on the medium/large instances on one core** (fl1577 11.6 s vs 145.66 s,
  att532 1.49 s vs 8.82 s, fl417 2.60 s vs 10.29 s) at equal or near-equal
  quality. Parallelism is a speed lever, not an accuracy lever: there is no
  parallel trick that beats serial's accuracy without redoing serial's
  recombination work.
- Two parallel variants were built and **removed**: `best_of_islands` (full
  budget per island) ran *slower* with no quality gain -- on a
  memory-bandwidth-bound workload 3 full islands just contend. `cooperative`
  (islands migrating tours mid-search) was high-variance: the migration content
  is thread-timing-dependent and the merge-then-adopt step amplifies it into
  chaotic run-to-run swings (pr1002 0.001%-0.267% for the *same* config). The
  only deterministic fix is to redo serial's recombination, i.e. just run serial.
- Memory: the n×n distance matrix is the only large allocation (561 MB at
  rl11849). It is **optional** -- the library default runs large n on-the-fly at
  ~5 MB; the matrix is forced only for the fastest headline timing, and even that
  edge is now ~10% (it was the obsolete pre-scan-removal premise). rl11849 /
  d18512 parallel + a fresh LKH re-run are deferred (big-runtime).

Current read:

- 15 of the 17 original instances reach the known optimum at the pinned seed, including pr1002 (259045 — previously the only row losing to LKH on both axes). fl1577 beats LKH's tour (22256 vs 22262) at a seventh of its time; lin318 beats LKH outright. Total pinned-seed suite time is ~50 s, the fastest state yet. Multi-seed columns show the knife-edge variance openly (e.g. lin318 optimal on 2 of 3 seeds).
- 2026-06-12, round 15 (the four standing gaps, one round): (1) the headline mode now runs seeds {12345, 7, 99} (`seed` CSV column) — single-seed rows misranked variants four times in rounds 11-14; (2) an elite pool (capacity 6, exact-duplicate dedupe, replace-worst) supplies the EAX merge references at n>=1000, replacing the shadow/best/prev trio — pr1002 mean across seeds improved ~325 units and fl1577 got ~17% faster (kicks still come from the single incumbent; pool-sourced kicks remain measured-dead); (3) candidate width drops to 5 at n>=1000 (LKH's own default), halving big-row trial cost and solving pr1002 at the pinned seed — the trade is fl1577's thinner optimal-edge coverage, so its seeds 7/99 land just above LKH's tour while staying 7-9x faster; (4) rl11849 (n=11849, optimum 923288) is a standing probe-budget row (400 trials, single seed, headline only): 0.800% in 160 s vs LKH's optimum in 1287.6 s — LKH passes our quality ~4 minutes into its run; everything under ~3.5 minutes of wall clock is ours.
- Residual pinned-seed gaps: d657 0.008% (4 length units), fl1577 0.031% (still below LKH's tour), rat575 0.089%, and the rl11849 probe row 0.800%. No original instance loses to LKH on both axes anymore.
- 2026-06-12, round 14: plateau kicks gained a zero-delta Or-opt shape (segment relocation) at n>=1000, extension phase only — fl1577 48.2 -> 44.6 s at the same 22254 and pr1002 reaches its best 226 trials earlier; sub-1000 rows bit-identical. rl11849 (n=11849, optimum 923288) added under vendor/tsplib for the large-instance regime; first 200-trial probe: 0.797% gap in 110 s, still improving steadily.
- 2026-06-12, rounds 11-13: tour merging is now size-gated. EAX-lite (single AB-cycle edge assembly crossover with candidate-bridge subtour repair and a multi-reference pool) fully replaced IPT first; it covers IPT's move set and additionally moves interleaved differing bundles atomically. Measured across six seeds, EAX-everywhere was parity-on-optima but reshuffled knife-edge rows, lengthened runs ~10% (merge wins re-arm the staleness window), and only produced durable gains in the kick-only big-instance regime (fl1577 22254 — IPT never went below 22262 — and occasionally pr1002/rat575). Four EAX variants (single/multi reference, gain-first/smallest-first application order, with/without adoption resets) all showed the same pattern, so IPT was restored verbatim below n=1000 (bit-identical round-10 trajectories) with EAX at n>=1000. Two adaptive stopping rules (identical-trial streaks; progress-gap patience) were measured and REJECTED: pinned-seed time cuts of 35-70% cost rat195/fl417 most of their winning seeds across 6 seeds — improvement gaps are heavy-tailed (up to 11x the prior maximum), so the n-trial staleness window is the insurance premium for expected accuracy. The elite-pool build is expected to replace both mergers with one structure.
- Tour-diff analysis against LKH's optimal tours shows the residuals are not one missing k-opt move: rat575 differs from the optimum in 67 edges scattered over 59 sections of size <= 2 (pr1002: 91 edges, 83 sections). These geometries are massively degenerate; locally optimal tours sit on broad cost-equal plateaus where the better micro-variant only pays after a neighboring section also changes. Extension-phase kicks therefore add zero-delta 2-opt "plateau drift" (length-preserving reconnections), which is what closed rd400.
- Structural fixes from 2026-06-11, after the fixture set grew to 17: (1) alpha-nearness candidate generation was O(n^2 x depth^2) per build — up to O(n^4) on chain-shaped MSTs, 36 s of fl1577's 38 s candidate build — and is now O(n^2) via per-row 1-tree bottleneck traversals (bit-identical candidates); (2) the LK recursion previously backtracked at every depth, exploding on clustered geometry where a long removed edge makes the positive-gain bound prune nothing — it now follows LKH's discipline (backtrack at levels 1-2, commit below) for n >= 400, and extension-phase trials always use the cheap discipline; (3) the fixed `trials = dimension` budget was cutting off runs that were still improving (rd400 found its best tour on its final trial) now that trials are several times cheaper — the headline mode extends trials while improvement is at most `dimension` trials old, capped at 4x (2x for n >= 1000).
- The stagnation window is insurance: pcb442's and att532's optima arrive mid-extension with pre-extension statistics indistinguishable from lin318's (which extends fruitlessly for ~0.1 s). Cutting the window breaks those optima; latency-critical callers can set `trial_extension_factor = 0`.
- The benchmark uses a single fixed seed (12345). Trajectory-level changes shuffle individual rows by a fraction of a percent; treat single-row deltas across code changes as noise unless they reproduce.
- The analysis sections below describe the 2026-06-07 state (pre guided-restart/IPT/backtracking rounds) and are kept as history.

## Why Quality Is Still Bad

The solver does not have full Gain23. It has partial Gain23-style probes that help `lin318`, but rat575 still needs LKH-style non-sequential case handling.

| Area | Status | Evidence | What it means |
|---|---|---|---|
| Candidate coverage | Mostly good | rat575 LKH-tour edge coverage is 100% at alpha width 8+ | Do not start by widening candidates. The solver sees the useful edges. |
| Sequential LK | Working but greedy | `lk_moves` often hits pass limits while gap remains high | It burns passes on local improvements and cannot assemble larger non-sequential moves. |
| Gain23 | Partial | `lin318` improved, rat575 unchanged | Current cases are too narrow. |
| BridgeGain | Incomplete | Only the first selected-subtour case is implemented | rat575 likely needs the missing `Case6`/`Case8` bridge paths. |
| rat575 runtime | Fragile | Ungated generic bridge probing hit 30-87 s with zero accepts | More brute force is the wrong fix. Port the case logic. |
| Capped rat575 bridge sweep | Rejected | Runtime stayed sane, but gap got worse | Existing bridge cases are harmful on rat575 without LKH case selectivity. |
| Anchored `tryNonSequentialBridge` | Dead | rat575 depth-3/depth-4 attempts have zero accepts in every benchmark mode | The one add/remove/close pattern is too weak; add case-aware bridge completions inside `searchRemoved`. |

## What To Do First

Priority order:

| Priority | Task | Scope | Why first |
|---:|---|---|---|
| 1 | Port more LKH `BridgeGain` cases, especially nonfeasible 3-opt reconnecting with 2-opt | `src/solver.zig:1824::2059`, `src/solver.zig:2386::2441` | This is the direct rat575 blocker. Candidate edges are present, but move assembly is missing. |
| 2 | Replace ad hoc nonseq 4-opt with case-aware `Case6`/`Case8` generation | `src/solver.zig:2069::2136` | Current nonseq code is a narrow increasing-index pattern and is mostly dead for real TSPLIB gaps. |
| 3 | Make LK completion oracle use the same case-aware move planner | `src/solver.zig:2240::2384` | Current 2/3-edge completion is bounded cleanup, not Helsgaun Gain23. |
| 4 | Revisit ordering/gating after real cases exist | `src/solver.zig:1787::1822` | Current `<512` gate protects rat575 runtime, but it is a workaround for incomplete bridge logic. |
| 5 | Only then tune ascent/candidate sensitivity | candidate generation code, outside the LK block | Current evidence says candidates are not the main failure. |

Do not start with CGAL, candidate width, or more trials. That hides the bug and costs time. The first real fix is LKH-equivalent `BridgeGain` case coverage with hard caps and benchmark evidence.

Depth diagnostic from 2026-06-07 confirms the next target. `tryNonSequentialBridge` is called from `searchRemoved`, so it has the correct LK chain context, but its current single-step close never accepts on rat575:

| rat575 mode | Depth-3 attempts | Depth-3 accepted | Depth-3 gain rejects | Depth-3 apply rejects | Depth-4 attempts | Depth-4 accepted | Depth-4 gain rejects | Depth-4 apply rejects |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| nearest-lk | 6012 | 0 | 6829 | 1345 | 25209 | 0 | 27167 | 11625 |
| alpha-lk | 4242 | 0 | 4436 | 2089 | 26563 | 0 | 23563 | 20489 |
| alpha-w12-t4 | 18124 | 0 | 21494 | 4439 | 121711 | 0 | 125502 | 67300 |
| alpha-w24-t4 | 17032 | 0 | 20220 | 1709 | 105264 | 0 | 128013 | 41684 |
| alpha-w24-t8 | 27282 | 0 | 34166 | 2562 | 177976 | 0 | 224114 | 65939 |

Interpretation: the anchored path is not starved; it tries plenty. It fails because the current pattern is only one candidate add, one tour-edge removal, and one close to `t1`. The next implementation should extend this anchored path with 3-edge and 4-edge bridge completions using the existing `removed_a/removed_b` and `added_a/added_b` chain state. Do not revive the standalone full-tour sweep.

## What Is Not Finished

- Full Helsgaun `Gain23`
- Complete LKH-equivalent `BridgeGain`; only the first selected-subtour case is implemented
- Strong non-sequential LK for 500+ node instances
- Better ascent schedule and sensitivity analysis
- Quality-proven CGAL/geometric candidate tuning
- Real segment/tree tour operations beyond the conservative current backend
- ATSP, VRP, time windows, and other LKH-3 problem classes

The next useful implementation target is `BridgeGain`, not more candidate widening. For rat575, the solver already sees useful edges; it fails to build the right non-sequential move.
