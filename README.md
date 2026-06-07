# commiv

Zig 0.16 library-style symmetric TSP solver.

This is a bounded Lin-Kernighan-style heuristic with TSPLIB parsing and benchmark tooling. It is not full LKH-3 yet.

## What It Does

For `n <= 10`, `solve` uses exact brute force with node 0 fixed.

For larger instances, `solve` runs deterministic multi-start heuristic search:

- nearest-neighbor and farthest-insertion starting tours
- deterministic candidate rows
- pi-adjusted alpha-nearness candidates
- optional CGAL Delaunay candidate augmentation with `-Dwith-cgal=true`
- 2-opt and one-node Or-opt warmup
- bounded sequential LK search
- bounded in-recursion 2/3-edge completion oracle
- partial Gain23-style nonfeasible bridge moves
- bounded 3-edge cleanup
- `TourView` abstraction over flat and segment-backed tour views
- `MovePlan` validation for edge-delta application and two-component patching

The current quality blocker is still large-instance non-sequential search. Candidate coverage is already good on the benchmark fixtures; rat575 is bad because the move generator is not close enough to LKH `Gain23/BridgeGain`.

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

Place these files here:

```text
vendor/tsplib/berlin52.tsp
vendor/tsplib/eil76.tsp
vendor/tsplib/rat195.tsp
vendor/tsplib/lin318.tsp
vendor/tsplib/rat575.tsp
```

The benchmark target automatically reports gap against known optima for those fixtures.

Current fixture optima used by the benchmark:

| Instance | Optimum |
|---|---:|
| berlin52 | 7542 |
| eil76 | 538 |
| rat195 | 2323 |
| lin318 | 42029 |
| rat575 | 6773 |

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

Machine-local result from 2026-06-07, one CPU core:

| Instance | Mode | Length | Optimum | Gap | Time |
|---|---|---:|---:|---:|---:|
| berlin52 | alpha-lk | 7542 | 7542 | 0.000% | 52 ms |
| eil76 | alpha-patch-lk | 538 | 538 | 0.000% | 19 ms |
| rat195 | alpha-lk | 2341 | 2323 | 0.775% | 719 ms |
| lin318 | alpha-w24-t4 | 43424 | 42029 | 3.319% | 4.05 s |
| rat575 | alpha-w24-t4 | 7182 | 6773 | 6.039% | 4.32 s |

Recent local LKH baseline with `RUNS=1`:

| Instance | LKH Length | Gap | Time |
|---|---:|---:|---:|
| berlin52 | 7542 | 0.000% | ~0 s |
| eil76 | 538 | 0.000% | 0.01 s |
| rat195 | 2323 | 0.000% | 0.49 s |
| lin318 | 42029 | 0.000% | 0.75 s |
| rat575 | 6774 | 0.015% | 1.46 s |

Current read:

- Small and medium fixtures are acceptable.
- `lin318` improved after adding Gain23-style nonfeasible bridge paths: best gap moved from `4.559%` to `3.319%`.
- `rat575` is still not acceptable. Candidate coverage is 100% at alpha width 8+, so the issue is search/move generation, not missing candidate edges.
- Ungated generic bridge probing on rat575 produced zero accepted non-sequential moves and 30-87 second runtimes, so the current bridge paths stay gated below 512 nodes.
- Capped rat575 bridge probing was tested with a total sweep budget around `lk_nonseq_branch_limit * n / 4`. It did not blow up, but it made quality worse: `rat575 alpha-w24-t4` moved from `7182 / 6.039%` to `7187 / 6.113%`, and `lin318 alpha-w24-t4` regressed from `43424 / 3.319%` to `43945 / 4.559%`. Do not keep that as a default.

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
