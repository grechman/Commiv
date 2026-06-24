# commiv

A fast, dependency-free routing engine in [Zig](https://ziglang.org) (0.16). It solves the
travelling-salesman and vehicle-routing families — **TSP, ATSP, CVRP, ACVRP, VRPTW** — to
within a fraction of a percent of optimal, in **seconds**, and it handles **directed
(asymmetric) cost matrices** — real road networks where A→B ≠ B→A — which most fast solvers
can't ingest at all.

The bet behind the whole engine: you almost never need the last 0.3% of optimality that
costs an hour of compute. You need a near-optimal route *now*. commiv is built for that
operating point — and for real road data, where the asymmetry is the whole problem.

```
zig build                                  # build the library
zig build test                             # 70 tests
zig build cvrpbench -Doptimize=ReleaseFast # build a benchmark, then run zig-out/bin/commiv-cvrpbench
```

- **Near-optimal, fast** — 0.02% from proven optima on standard CVRP, ~0.45% on the hard
  Uchoa X set, all in seconds to a minute on a laptop.
- **Asymmetric-native** — directed travel-time matrices are first-class, not bolted on. On
  real Moscow OSRM data it beats OR-Tools, LKH-3 and VROOM on both cost and wall-clock.
- **Zero dependencies** — one Zig module, no system libraries, no build-time downloads.
- **Lean** — solves a 5000-node directed CVRP in 109 s using **211 MB** (the matrix itself
  is 100 MB of that).

---

## Quick start

### Use it as a library

`build.zig.zon` exposes the module `commiv`. Parse a TSPLIB instance, solve, read the tour:

```zig
const std = @import("std");
const commiv = @import("commiv");

pub fn main() !void {
    const allocator = std.heap.page_allocator;
    var p = try commiv.parseTsplib(allocator, tsplib_text, .{});
    defer p.deinit();

    var result = try commiv.solve(allocator, &p, .{ .seed = 1 });
    defer result.deinit();

    std.debug.print("length={} tour={any}\n", .{ result.length, result.tour });
}
```

For vehicle routing you pass a cost matrix + demands directly — see [API](#api) below. The
asymmetric path is the same call with a non-symmetric matrix.

### Run a benchmark

Each benchmark is its own build step; gap benches install a binary you then run with env vars:

```sh
zig build cvrpbench  -Doptimize=ReleaseFast && ./zig-out/bin/commiv-cvrpbench   # CVRP vs optima
zig build acvrpbench -Doptimize=ReleaseFast && ./zig-out/bin/commiv-acvrpbench  # asymmetric CVRP vs LKH-3
zig build atspbench  -Doptimize=ReleaseFast && ./zig-out/bin/commiv-atspbench   # ATSP vs proven optima
zig build vrptwbench -Doptimize=ReleaseFast && ./zig-out/bin/commiv-vrptwbench  # VRPTW vs SINTEF BKS
zig build roadbench  -Doptimize=ReleaseFast && ./zig-out/bin/commiv-roadbench   # real directed Moscow road matrix
zig build bench      -Doptimize=ReleaseFast                                     # TSP benchmark (runs)
```

## Use cases

- **Last-mile / courier routing on real road networks** — feed a directed OSRM/OSRM-style
  travel-time matrix; get capacity-feasible routes that respect one-way streets and turn
  costs. This is the case commiv is built for.
- **Classical TSP/CVRP/VRPTW** — near-optimal solutions far faster than exact methods.
- **Embeddable core** — a single dependency-free module to drop into a larger planner.

---

## Benchmarks

All gaps are vs the reference shown (proven optimum, published best-known, or a reference
solver). Hardware: laptop, Intel i3-1115G4 (2c/4t). Times are wall-clock at the stated budget.

### Symmetric

| benchmark | reference | instances | commiv gap | budget / time |
|---|---|---:|---:|---|
| TSP (TSPLIB) | proven optima | rat575 / pr1002 / fl1577 / d657 | 0.089% / **0.000%** / 0.031% / 0.008% | ILS, seconds |
| TSP large (rl11849) | proven optimum | 1 | 0.690% | ~77 s, single probe |
| CVRP — Augerat A | proven optima | 12 | **0.021%** | ~1 s/instance |
| CVRP — Uchoa X | best-known | 6 | **0.456%** (SISR 20M, 3 threads) | 50–170 s/instance |
| CVRP — Uchoa X | best-known | 6 | 0.711% (SISR 1M best-of-3) | ~seconds |
| VRPTW — Solomon | SINTEF BKS | 5 | 0.182% distance (vehicle-matched) | ~seconds |

Uchoa X per-instance (SISR 20M): X-n101 **0.120%**, X-n153 0.452%, X-n200 **0.143%**,
X-n303 0.902%, X-n502 **0.087%**, X-n1001 1.034%. The two hard instances (X-n303, X-n1001)
sit near 1%; the rest are 0.09–0.45%.

### Asymmetric

| benchmark | reference | instances | commiv gap | time |
|---|---|---:|---:|---|
| ATSP (TSPLIB classic) | proven optima | 14 | **0.000%** (br17…kro124p) | sub-second–18 s |
| ATSP (rbg stacker-crane) | proven optima | 4 | 0.043% (rbg323/403/443 optimal; rbg358 +0.17%) | 17–24 s |
| ACVRP | LKH-3 (field best) | 30 | **0.228%** | ~1–2 s/instance |

### Real directed road data (Moscow, OSRM)

`moscow-*` is a **custom benchmark** — real OSRM directed travel-time matrices sampled across
central Moscow, not a published instance set, so there is no quotable optimum. It exists to
compare commiv against the solvers that also accept directed matrices, on identical instances,
scored identically (route cost on the true directed matrix, capacity-validated). Harness:
[`tools/competitors/`](tools/competitors/).

| n | commiv (SISR) | OR-Tools 9.15 | LKH-3 (warmstart) | VROOM |
|---|---:|---:|---:|---:|
| 100  | **41,808** @ 0.8 s  | 44,183 @ 8 s   | 43,090 @ 12 s   | 42,490 @ 1.3 s |
| 1000 | **207,406** @ 2 s   | 225,917 @ 60 s | 221,487 @ 456 s | 208,687 @ 315 s |
| 2000 | **366,996** @ 9 s   | 423,800 @ 60 s | 523,233 @ 909 s¹ | 368,373 @ 1607 s |
| 5000 | **779,161** @ 109 s | 868,583 @ 420 s | infeasible²     | did not finish³ |

<sub>cores: commiv 3, the rest effectively 1. ¹ one unfinished LK trial. ² LKH could not reach
a feasible packing even warmstarted. ³ VROOM did not finish n=5000 within an hour. VROOM n≤2000
is exploration level 5; n=5000 level 3.</sub>

commiv is fastest **and** cheapest at every size. The nearest competitor on cost is **VROOM**
(within ~0.4–0.6%), but its wall-clock blows up with scale (20× slower at n=1000, ~180× at
n=2000). **LKH-3 needs a feasible warmstart** just to run on explicit directed matrices and
falls apart past n=1000. There is no published optimum here, but VROOM independently landing
within 0.5% is strong evidence the solutions are near-optimal.

---

## How commiv compares — honestly

**Where it wins**

- **The speed/quality frontier.** Near-optimal in seconds, not the minutes-to-hours the
  reference heuristics spend. For a real planner that has to replan constantly, this is the
  number that matters.
- **Directed real-road matrices.** Asymmetric cost is first-class. FILO and HGS-CVRP — the
  symmetric speed/quality champions — physically cannot read a directed matrix. On real
  Moscow data commiv beats OR-Tools, LKH-3 and VROOM on cost *and* time.
- **Zero dependencies, small footprint.** One Zig module; a 5000-node directed CVRP in 211 MB.

**Where the competition wins — and you should know this**

- **Absolute accuracy at huge budgets.** LKH-3, HGS-CVRP and SISR (the paper) reach lower
  gaps (~0.16–0.39% on Uchoa X) when given far more time. commiv trades that last fraction
  of a percent for a large speed advantage; it is **not** state-of-the-art on accuracy at the
  frontier.
- **Massive symmetric instances.** FILO solves symmetric CVRPs with tens of thousands of
  nodes faster than anything here. commiv targets the routing-scale (hundreds to a few
  thousand) directed regime.
- **Production hardness.** OR-Tools and VROOM are battle-tested stacks with rich constraints
  (time windows, pickup-delivery, skills, breaks) and years of deployment. commiv is a fast,
  focused core, not a complete logistics platform.

---

## Design decisions

### What we settled on, and why

- **SISR (Slack Induction by String Removals) for large and asymmetric CVRP.** Ruin a few
  spatially-adjacent strings, greedily re-insert with random "blinks", accept under
  threshold/SA. The bet: millions of `O(removed)` moves beat thousands of `O(n)` ones. This
  is what cracks the large-n and directed regimes.
- **HGS (population + Prins Split DP) for mid-size CVRP (n ≲ 500).** A genetic population of
  giant tours with optimal capacity splitting and local-search education gives the best
  quality at that scale (the 0.02–0.45% numbers above).
- **Penalty-based infeasible search.** Letting local search cross capacity-infeasible regions
  (paying a penalty) broke a hard ~2% quality ceiling that feasible-only search couldn't.
- **Native directed ATSP for degenerate matrices.** The stacker-crane rbg instances have many
  arcs tying each row minimum; the Jonker-Volgenant 2n transform pays for that twice. A
  direct directed local search (Or-opt + directed 2-opt + double-bridge) reaches the optimum
  faster, on n nodes instead of 2n.
- **Granular don't-look-queue local search.** Restricting moves to spatial neighbours and
  re-activating only changed-edge endpoints made large-n local search 2–4× faster at equal
  quality.
- **In-place matrix-view seed.** The CVRP giant-tour seed reads the cost matrix directly via a
  strided view instead of copying out an `n×n` sub-matrix or building a `2n` transform — which
  dropped n=5000 memory from ~2 GB to 211 MB and the solve from 412 s to 109 s, with identical
  quality (the seed is a throwaway SISR rebuilds).
- **Parallelism is a speed lever, not magic.** Best-of-K seeds and EAX recombination help
  accuracy at equal wall on multiple cores; a deterministic split-budget mode trades a little
  quality for ~2.5× speed.

### What we tried and rejected, and why

- **Static Move Descriptors (SMD).** Our don't-look-queue is 4–5× faster at identical quality;
  the DLQ already captures the locality SMD buys. Dead end.
- **Two-level doubly-linked tour list.** After fixing a fallback that was firing on provably
  doomed rebuilds, tour rebuilds dropped ~10× and the remaining cost was <5% of wall — the
  rewrite's complexity wasn't worth it.
- **Cooperative / best-of parallelism.** High variance and lock contention made it slower than
  independent islands; removed.
- **Decomposition for large n.** A subproblem-resolve win on converged TSP tours did **not**
  generalize to never-converging SISR — plain SISR run longer dominated.
- **Adaptive candidate re-ranking.** Even a perfect oracle re-rank washes out at full ILS
  budget. Candidate *order* is a single-descent lever, not an accuracy lever.
- **Edge-freezing / voting.** Freezing even a pure subset of known-optimal edges loses
  accuracy, because Lin-Kernighan must break and rebuild even optimal edges en route. Structural.
- **Assignment-bound early stop.** The AP lower bound is too loose for capacity-tight CVRP to
  certify near-optimality (19–52% on Moscow); useless as a stopping rule here.

---

## API

All solvers are allocator-first, take an options struct, and return a result with `deinit()`.

**Parsing**
- `parseTsplib(allocator, text, opts) !Problem` — TSPLIB/CVRPLIB parser.

**TSP / ATSP**
- `solve(allocator, *Problem, SolveOptions) !TourResult` — symmetric TSP (Lin-Kernighan + ILS).
- `solveAtsp(allocator, matrix, n, SolveOptions)` — directed TSP via the 2n transform.
- `solveAtspNative(allocator, matrix, n, SolveOptions)` — direct directed search (no transform).
- `bruteForce(allocator, *Problem, ExactOptions)` — exact, tiny n.

**CVRP / ACVRP** (pass a row-major `(n+1)×(n+1)` cost matrix + demands; asymmetric matrices work as-is)
- `solveCvrpHgs(allocator, inst, SolveOptions, CvrpHgsParams)` — best for n ≲ 500.
- `solveCvrpSisr(allocator, inst, SolveOptions, CvrpSisrParams)` — best for large / directed.
- `solveCvrpSisrParallel(...)`, `solveCvrpHgsParallel(...)` — multi-threaded variants.
- `solveCvrpFleet(...)`, `solveCvrp(...)` — fixed-fleet / convenience entry points.

**VRPTW**
- `solveVrptw(allocator, inst, SolveOptions)`, `solveVrptwHgs(...)`.

**Asymmetry analysis**
- `conservativeness(allocator, matrix, dim) !Conservativeness` — Helmholtz-Hodge decomposition
  of a directed matrix: tells you how much of the asymmetry is *structural* (one-ways, turns —
  changes optimal routes) vs a *gradient* (congestion — free to ignore). Point it at any cost
  matrix to decide whether you need directional routing at all.

Full surface in [`src/root.zig`](src/root.zig); each solver has unit tests in its source file.

---

## GPU acceleration (designed, not built)

commiv is CPU-only today. The single largest untapped speedup is a GPU: SISR's hot loop
evaluates millions of independent move-deltas per second — an embarrassingly parallel batched
reduction that maps cleanly onto a GPU, with the directed matrix held device-resident (100 MB
at n=5000 fits any modern card). A full task spec — batched move-delta kernel, massive
best-of-K islands, CUDA FFI, device-resident matrix — is in [`gpu.md`](gpu.md). It is not
implemented (no GPU in the dev environment); it's the most likely path to another order of
magnitude at large n.

---

## Reproducing the benchmarks

Standard instances ship under `vendor/` (TSPLIB, CVRPLIB Augerat + Uchoa X, ATSP, ACVRP,
Solomon, and the Moscow OSRM matrices under `vendor/road/`). The `moscow-5000` matrix is
gzipped (`gunzip vendor/road/moscow-5000.road.gz` before use). Competitor adapters (OR-Tools,
LKH-3 with warmstart, VROOM, and an assignment lower bound) plus setup instructions are in
[`tools/competitors/`](tools/competitors/). Re-fetch a Moscow matrix from a self-hosted OSRM
with [`tools/fetch_road_matrix.py`](tools/fetch_road_matrix.py).

## License

See [`LICENSE`](LICENSE).
