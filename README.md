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
- IPT tour merging (LKH `MergeWithTour`) of trial tours against the incumbent
- `TourView` abstraction over flat and segment-backed tour views
- `MovePlan` validation for edge-delta application and two-component patching

Eleven of the seventeen TSPLIB fixtures solve to the known optimum in the headline `alpha-w8-kick` mode, including u574 and fl417; fl1577 matches LKH's RUNS=1 tour at a sixth of its time and lin318 beats it. The remaining gaps are 0.06-0.56% on six instances (see the table below).

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

Machine-local result from 2026-06-11, one CPU core, headline `alpha-w8-kick` mode (fresh-cache build), side by side with LKH-3.0.13 `RUNS=1` on the same machine and core class:

| Instance | n | commiv Length | Gap | Time | LKH Length | LKH Gap | LKH Time |
|---|---:|---:|---:|---:|---:|---:|---:|
| berlin52 | 52 | 7542 | 0.000% | 71 ms | 7542 | 0.000% | 0.01 s |
| eil76 | 76 | 538 | 0.000% | 22 ms | 538 | 0.000% | 0.02 s |
| kroA100 | 100 | 21282 | 0.000% | 124 ms | 21282 | 0.000% | 0.04 s |
| bier127 | 127 | 118282 | 0.000% | 275 ms | 118282 | 0.000% | 0.07 s |
| rat195 | 195 | 2323 | 0.000% | 142 ms | 2323 | 0.000% | 0.72 s |
| ts225 | 225 | 126643 | 0.000% | 574 ms | 126643 | 0.000% | 0.44 s |
| a280 | 280 | 2579 | 0.000% | 337 ms | 2579 | 0.000% | 0.57 s |
| lin318 | 318 | 42029 | 0.000% | 764 ms | 42143 | 0.271% | 0.80 s |
| rd400 | 400 | 15327 | 0.301% | 389 ms | 15281 | 0.000% | 0.60 s |
| fl417 | 417 | 11861 | 0.000% | 2.62 s | 11861 | 0.000% | 10.29 s |
| pcb442 | 442 | 50832 | 0.106% | 261 ms | 50778 | 0.000% | 2.91 s |
| att532 | 532 | 27703 | 0.061% | 0.97 s | 27686 | 0.000% | 8.82 s |
| u574 | 574 | 36905 | 0.000% | 2.69 s | 36905 | 0.000% | 4.80 s |
| rat575 | 575 | 6779 | 0.089% | 0.55 s | 6773 | 0.000% | 2.07 s |
| d657 | 657 | 48949 | 0.076% | 1.44 s | 48912 | 0.000% | 2.49 s |
| pr1002 | 1002 | 260487 | 0.557% | 5.58 s | 259045 | 0.000% | 2.96 s |
| fl1577 | 1577 | 22262 | 0.058% | 24.5 s | 22262 | 0.058% | 145.66 s |

Current read:

- 11 of 17 instances reach the known optimum, every instance up to lin318 plus fl417 and u574. lin318 beats this LKH run outright (LKH missed the optimum at RUNS=1); fl1577 produces the identical tour length at a sixth of LKH's time.
- Where a gap remains (rd400, pcb442, att532, rat575, d657 at 0.06-0.30%), commiv is 2-11x faster than LKH; pr1002 (0.56%, 5.6 s) is the one instance losing on both axes.
- Two structural fixes landed on 2026-06-11 after the fixture set grew to 17: alpha-nearness candidate generation was O(n^2 x depth^2) per build — up to O(n^4) on chain-shaped MSTs, 36 s of fl1577's 38 s candidate build — and is now O(n^2) via per-row 1-tree bottleneck traversals (bit-identical candidates); and the LK recursion previously backtracked at every depth, exploding on clustered geometry where a long removed edge makes the positive-gain bound prune nothing — it now follows LKH's discipline (backtrack at levels 1-2, commit below) for n >= 400.
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
