# commiv

[![CI](https://github.com/grechman/Commiv/actions/workflows/ci.yml/badge.svg)](https://github.com/grechman/Commiv/actions/workflows/ci.yml)

**[English](#commiv) · [Русский](#commiv-rus)**

Near-optimal vehicle routes over real road networks, in seconds. Embeddable,
dependency-free, and built for directed (asymmetric) cost where the trip from A to B
is not the same as B to A.

commiv solves the travelling-salesman and vehicle-routing families (TSP, ATSP, CVRP,
ACVRP, VRPTW) to within a fraction of a percent of optimal, and it reads directed
travel-time matrices natively: one-way streets, turn penalties, congestion. The fast
symmetric solvers everyone reaches for, FILO and HGS-CVRP, cannot ingest a directed
matrix at all. commiv is built around that case.

The bet behind the whole engine: you almost never need the last 0.3% of optimality that
costs an hour of compute. You need a near-optimal route now, and on real roads it has to
respect direction. That is the operating point commiv targets.

> For courier, last-mile, and fleet-routing platforms that re-optimize thousands of
> directed-road routes under a tight latency budget, commiv is an embeddable routing core
> that returns near-optimal, capacity-feasible routes in seconds. Unlike LKH-3
> (single-threaded, non-commercial license, and unable to handle explicit directed
> matrices past about n=1000) or FILO and HGS-CVRP (fast, but symmetric-only), it treats
> asymmetric road cost as the main case, not an afterthought.

Why that pays off: at fleet scale, solver compute is a line item that compounds across
thousands of routes re-optimized around the clock. On real Moscow OSRM data, an n=1000
directed CVRP comes out at **207,406 in 2 s** for commiv versus 225,917 in 60 s
(OR-Tools), 221,487 in 456 s (LKH-3), and 208,687 in 315 s (VROOM). Cheaper and faster at
once. And you can check that on your own instances in an afternoon; none of these numbers
ask for your trust.

```sh
zig build                                  # build the library
zig build test                             # run the unit tests
zig build example                          # run the embedded solver example
zig build serve -Doptimize=ReleaseFast     # REST API server (JSON over HTTP)
zig build lib   -Doptimize=ReleaseFast     # C ABI: libcommiv.{a,so} + commiv.h
```

- **Near-optimal, fast.** 0.02% off proven optima on standard CVRP, about 0.45% on the
  hard Uchoa X set, in seconds to a minute on a laptop.
- **Callable from any language.** A REST server, a C ABI, and a Python binding ship in
  this repo. Zig is the engine underneath; you never have to write it.
- **Asymmetric-native.** Directed travel-time matrices are first-class, not bolted on. On
  real Moscow OSRM data commiv beats OR-Tools, LKH-3, and VROOM on both cost and wall-clock.
- **Zero dependencies, deterministic.** One Zig module, no system libraries, no build-time
  downloads. The same seed produces the same routes.
- **Lean.** A 5000-node directed CVRP solves in 109 s using 211 MB, and 100 MB of that is
  the matrix itself.

---

## Integrate commiv into your codebase

The real entry point is not a TSPLIB file. It is your own stops, a cost matrix, and
per-stop demands. Zig is the engine underneath, but you pick the door that matches your
stack — none of them require writing Zig:

| Your stack | Door | Where |
|---|---|---|
| Any language | REST server: one static binary, JSON over HTTP | [`docs/rest.md`](docs/rest.md) |
| Python | Native binding (ctypes over the C ABI, numpy-friendly) | [`bindings/python/`](bindings/python/) |
| C, C++, Go, Rust, ... | C ABI: `libcommiv.{a,so}` + [`include/commiv.h`](include/commiv.h) | `zig build lib` |
| Zig | The module itself | this section |

### REST, from anywhere

```sh
zig build serve -Doptimize=ReleaseFast && ./zig-out/bin/commiv-serve
```

```sh
curl -X POST http://127.0.0.1:8080/solve/cvrp -d '{
  "matrix": [[0,10,14,12],[11,0,9,20],[15,8,0,7],[13,18,6,0]],
  "demand": [0,4,6,5],
  "capacity": 10
}'
# {"total_cost":58,"vehicles":2,"routes":[[2,1],[3]]}
```

That is the whole integration: a directed cost matrix (row `a`, column `b` = cost of
`a -> b`, depot = node 0), demands, a capacity. `/solve/vrptw` adds time windows,
`/solve/atsp` does pure directed ordering. Full schema and Python/JS/Go client snippets
in [`docs/rest.md`](docs/rest.md).

### Python, natively

```python
import commiv  # pip install -e bindings/python; needs zig build lib once

sol = commiv.solve_cvrp(matrix, demand=[0, 4, 6, 5], capacity=10, seed=1)
print(sol.total_cost, sol.routes)  # 58 [[2, 1], [3]]
```

numpy matrices take a fast path; infeasible instances raise instead of returning
garbage. Details in [`bindings/python/README.md`](bindings/python/README.md).

### Zig, embedded

The rest of this section is the full Zig walkthrough.

#### 1. Add the dependency

```sh
zig fetch --save "git+https://github.com/grechman/Commiv"
```

That saves the package under the name `commiv`. Wire the module into your `build.zig`:

```zig
const commiv = b.dependency("commiv", .{
    .target = target,
    .optimize = optimize,
});
exe.root_module.addImport("commiv", commiv.module("commiv"));
```

#### 2. Solve a vehicle-routing problem from your own data

Full working examples live in [`examples/`](examples/): `basic.zig` parses a TSPLIB instance with `parseTsplib`, while `roadbench.zig` and `cvrpbench.zig` read their CVRP and road instances from disk with their own parsers. `parseTsplib` reads TSPLIB symmetric TSP only — a coordinate section (`EUC_2D`, `CEIL_2D`, or `ATT`) or an `EXPLICIT` `FULL_MATRIX` — so CVRP, ACVRP, and ATSP instances do not load through it. Below is the minimal in-memory version on your own data.

You bring a row-major `(n+1) x (n+1)` cost matrix (node 0 is the depot, customers are
`1..n`), a `demand` array, and a vehicle `capacity`. The matrix is directional:
`matrix[a*(n+1) + b]` is the cost of going from `a` to `b`, so real asymmetric road cost
drops in as-is.

```zig
const std = @import("std");
const commiv = @import("commiv");

pub fn main() !void {
    const allocator = std.heap.page_allocator; // swap in your own (gpa, arena, ...)

    // 3 customers + depot. Directed costs (a -> b), row-major, depot = index 0.
    const n: usize = 3;
    const matrix = [_]u32{
        0,  10, 14, 12, // depot -> {depot, c1, c2, c3}
        11, 0,  9,  20, // c1    -> ...
        15, 8,  0,  7,  // c2    -> ...
        13, 18, 6,  0,  // c3    -> ...
    };
    const demand = [_]u32{ 0, 4, 6, 5 }; // demand[0] = 0 (the depot has none)

    const inst = commiv.CvrpInstance{
        .n = n,
        .matrix = &matrix,
        .demand = &demand,
        .capacity = 10,
    };

    // SISR is the default workhorse: best for large and/or directed instances.
    var result = try commiv.solveCvrpSisr(allocator, inst, .{ .seed = 1 }, .{});
    defer result.deinit();

    std.debug.print("total cost = {}\n", .{result.total_cost});
    for (result.routes, 0..) |route, v| {
        std.debug.print("vehicle {}: depot", .{v});
        for (route) |customer| std.debug.print(" -> {}", .{customer});
        std.debug.print(" -> depot\n", .{});
    }
}
```

#### 3. Read the result

`CvrpResult` owns its memory, so call `deinit()` when you are done.

- `result.total_cost` is the total routed cost on your matrix, as a `u64`.
- `result.routes` is one slice per vehicle. Each slice lists the customer indices in visit
  order, with the depot implied at both ends.

#### 4. Pick the solver for your size

| Entry point | Use it when |
|---|---|
| `solveCvrpSisr` | Large and/or directed instances. This is the road-network case. |
| `solveCvrpHgs` | Mid-size CVRP, n up to about 500, where you want the last bit of quality. |
| `solveCvrpSisrParallel` / `solveCvrpHgsParallel` | The same, spread across cores. |
| `solveCvrp` | The no-config default. Runs SISR with default params. |

#### 5. Plain TSP and ATSP

For a pure ordering problem with no capacity, use the TSP entry points. A directed matrix
takes the ATSP path; it is the same shape of call.

```zig
// Symmetric, from coordinates or a TSPLIB instance:
var p = try commiv.parseTsplib(allocator, tsplib_text, .{});
defer p.deinit();
var tour = try commiv.solve(allocator, &p, .{ .seed = 1 });
defer tour.deinit();
// tour.length and tour.tour hold the result.

// Directed, from an n x n matrix:
var atsp = try commiv.solveAtsp(allocator, &cost_matrix, n, .{ .seed = 1 });
defer atsp.deinit();
```

#### Knobs that matter

- `SolveOptions.seed` is the RNG seed. For the single-threaded solvers the same seed gives
  byte-identical output, so runs are reproducible. The `*Parallel` variants also depend on
  the thread count: their default `threads = 0` resolves to the host CPU count, which sets
  the island/chain count and therefore the per-island seeds, so the same seed yields
  different routes on machines with different core counts. For output reproducible across
  machines, pass an explicit non-zero `threads` (`ParallelOptions.threads` for `solveParallel`)
  and pin both the seed and that thread count.
- `SolveOptions.budget.trials` and `.max_passes` control how hard the search works. Larger
  means closer to optimal and more time. The budget is iteration-based, not a wall-clock
  deadline, so size it against your latency target empirically.
- Every returned route respects `capacity`. An instance with no feasible packing returns an
  error rather than a quietly wrong answer.

---

## API reference

Every solver is allocator-first, takes an options struct, and returns a result you free with
`deinit()`. The curated set below is the whole public API (it mirrors
[`src/root.zig`](src/root.zig)); each solver has unit tests in its own source file.

**Parsing**
- `parseTsplib(allocator, text, ParseOptions) !Problem` parses TSPLIB symmetric TSP only — a
  coordinate section (`EUC_2D`, `CEIL_2D`, or `ATT`) or an `EXPLICIT` `FULL_MATRIX`. Pass a
  `ParseDiagnostic` in the options to capture line-level parse errors.

**Problem definition** (coordinate / TSPLIB path)
- `Problem`, built via `Problem.initCoords(...)` or `Problem.initFullMatrix(...)`, plus the
  `Coord` and `DistanceKind` types.

**Shared options and result**
- `SolveOptions` — `seed`, `budget` (`trials`, `max_passes`), candidate and search knobs.
- `SolveResult` — `{ tour, length, stats }`, the one type returned by `solve`, `solveAtsp*`,
  and `bruteForce`. `SolveStats` is the per-run telemetry; `CandidateMode` picks the
  candidate-graph metric.

**TSP (symmetric)**
- `solve(allocator, *Problem, SolveOptions) !SolveResult` — Lin-Kernighan + ILS.
- `solveWithStats(...)` — same as `solve`, also fills `SolveStats` telemetry.
- `solveParallel(allocator, *Problem, SolveOptions, ParallelOptions) !SolveResult` —
  independent islands with optional EAX recombination, or a deterministic split-budget speed
  mode. `ParallelOptions.threads == 0` resolves to the host CPU count, which changes the
  island seeding and therefore the result; pass an explicit non-zero `threads` for output
  reproducible across machines.

**ATSP (directed)** — row-major `n x n` matrix where `matrix[i*n + j]` is the cost of `i → j`
- `solveAtsp(allocator, matrix, n, SolveOptions) !SolveResult` — 2n Jonker-Volgenant transform.
- `solveAtspNative(allocator, matrix, n, SolveOptions) !SolveResult` — direct directed search.
- `solveAtspParallel(allocator, matrix, n, SolveOptions, threads) !SolveResult` — `threads == 0`
  resolves to the host CPU count, changing the result; pass a non-zero `threads` for
  cross-machine reproducibility.

**Exact (tiny n)**
- `bruteForce(allocator, *Problem, ExactOptions) !SolveResult`.

**CVRP / ACVRP** — build a `CvrpInstance { n, matrix, demand, capacity }` with a
`(n+1) x (n+1)` directional matrix (depot = node 0); every solver returns
`CvrpResult { routes, total_cost }`
- `solveCvrp(allocator, inst, SolveOptions) !CvrpResult` — no-config default (runs SISR).
- `solveCvrpSisr(allocator, inst, SolveOptions, CvrpSisrParams)` — large / directed.
- `solveCvrpHgs(allocator, inst, SolveOptions, CvrpHgsParams, max_vehicles)` — n ≲ 500.
- `solveCvrpFleet(allocator, inst, SolveOptions, CvrpFleetParams)` — fixed fleet cap.
- `solveCvrpSisrParallel(allocator, inst, SolveOptions, CvrpSisrParams, threads)` — `threads == 0`
  resolves to the host CPU count, changing the result; pass a non-zero `threads` for
  cross-machine reproducibility.
- `solveCvrpHgsParallel(allocator, inst, SolveOptions, CvrpHgsParams, max_vehicles, threads)` —
  same `threads == 0` caveat as above.
- `solveCvrpMulti(allocator, inst, SolveOptions, CvrpMultiParams)` — uncapped giant-tour ILS
  variant (legacy; SISR usually dominates it).
- `validateCvrp(inst, routes) ?u64` — independent feasibility check of a solution; returns
  the recomputed true cost, or null if any route is infeasible or a customer is missed.

**VRPTW** — build a `VrptwInstance { n, matrix, demand, capacity, ready, due, service }`;
returns `VrptwResult`
- `solveVrptwSisr(allocator, inst, SolveOptions, VrptwSisrParams)` — the default engine:
  SISR with time windows. Use this one.
- `solveVrptwSisrParallel(allocator, inst, SolveOptions, VrptwSisrParams, threads)` —
  best-of-K chains; same `threads == 0` caveat as the CVRP variant.
- `solveVrptw(allocator, inst, SolveOptions, VrptwParams) !VrptwResult` — legacy giant-tour
  ILS (kept for reproducibility; SISR matches or beats it at equal wall).
- `solveVrptwHgs(allocator, inst, SolveOptions, VrptwHgsParams) !VrptwResult`.
- `validateVrptw(inst, routes) ?u64` — independent capacity + time-window feasibility check;
  returns the recomputed cost, or null if infeasible.

**Asymmetry analysis**
- `conservativeness(allocator, matrix, dim) !Conservativeness` runs a Helmholtz-Hodge
  decomposition of a directed matrix. It tells you how much of the asymmetry is structural
  (one-ways and turns, which change the optimal route) versus a gradient (congestion, which
  you can safely ignore). Point it at any cost matrix to decide whether you need directional
  routing at all.

Everything else (`commiv.internal.*`, the raw implementation modules) is unstable detail, not
part of this API and free to change between versions.

---

## Use cases

- **Last-mile and courier routing on real road networks.** Feed a directed OSRM-style
  travel-time matrix and get capacity-feasible routes that respect one-way streets and turn
  costs. This is the case commiv is built for.
- **Classical TSP, CVRP, and VRPTW.** Near-optimal solutions far faster than exact methods.
- **An embeddable core.** A single dependency-free module to drop into a larger planner.
- **Beyond logistics.** Hole-drilling on a printed circuit board, drone survey flyovers, NPC
  routes and patrols in video games, a warehouse picker walking the racks, round planning for
  field crews — any problem where visit order matters and the transitions are asymmetric.

---

## Benchmarks

Every gap is against the reference shown: a proven optimum, a published best-known, or a
reference solver. Hardware is a laptop, Intel i3-1115G4 (2 cores, 4 threads). Times are
wall-clock at the stated budget.

### Symmetric

| benchmark | reference | instances | commiv gap | budget / time |
|---|---|---:|---:|---|
| TSP (TSPLIB) | proven optima | rat575 / pr1002 / fl1577 / d657 | 0.089% / **0.000%** / 0.031% / 0.008% | ILS, seconds |
| TSP large (rl11849) | proven optimum | 1 | 0.690% | ~77 s, single probe |
| CVRP, Augerat A | proven optima | 12 | **0.021%** | ~1 s/instance |
| CVRP, Uchoa X | best-known | 6 | **0.456%** (SISR 20M, 3 threads) | 50–170 s/instance |
| CVRP, Uchoa X | best-known | 6 | 0.711% (SISR 1M best-of-3) | ~seconds |
| VRPTW, Solomon | SINTEF BKS | 5 | 0.182% distance (vehicle-matched) | ~seconds |
| VRPTW, Gehring–Homberger 400/1000 | SINTEF BKS | 12 | fleet matched on 4/12 (see below) | 2–60 s |

Uchoa X per-instance (SISR 20M): X-n101 **0.120%**, X-n153 0.452%, X-n200 **0.143%**,
X-n303 0.902%, X-n502 **0.087%**, X-n1001 1.034%. The two hard instances (X-n303 and
X-n1001) sit near 1%; the rest land between 0.09% and 0.45%.

Gehring–Homberger (first instance of each class at 400 and 1000 customers) is the honest
miss. The objective is lexicographic — fewest vehicles first, distance second — and commiv
matches the BKS fleet only on the easy clustered classes (c1_4_1 +0.09%, c1_10_1 +0.06%,
c2_10_1 +0.78% distance at matched fleet, plus r2_4_1 at +2.3%). On the R and RC classes it
runs 1–6 vehicles over the best known (often at *lower* distance, which is exactly the
trade the lexicographic objective forbids). Root cause is known: commiv has a per-vehicle
penalty and a fleet-emptying ruin, but no dedicated route-minimization phase
(ejection-pool style, as in the solvers that hold these records). That is a real gap, not
a tuning issue.

### Asymmetric

| benchmark | reference | instances | commiv gap | time |
|---|---|---:|---:|---|
| ATSP (TSPLIB classic) | proven optima | 14 | **0.000%** (br17…kro124p) | sub-second to 18 s |
| ATSP (rbg stacker-crane) | proven optima | 4 | 0.043% (rbg323/403/443 optimal; rbg358 +0.17%) | 17–24 s |
| ACVRP | LKH-3 (field best) | 30 | **0.228%** | ~1–2 s/instance |

### Real directed road data (Moscow, OSRM)

`moscow-*` is a custom benchmark: real OSRM directed travel-time matrices sampled across
central Moscow, not a published instance set, so there is no quotable optimum. It exists to
compare commiv against the solvers that also accept directed matrices, on identical
instances, scored the same way (route cost on the true directed matrix, capacity-validated).
The harness is in [`tools/competitors/`](tools/competitors/).

| n | commiv (SISR) | OR-Tools 9.15 | LKH-3 (warmstart) | VROOM |
|---|---:|---:|---:|---:|
| 100  | **41,808** @ 0.8 s  | 44,183 @ 8 s   | 43,090 @ 12 s   | 42,490 @ 1.3 s |
| 1000 | **207,406** @ 2 s   | 225,917 @ 60 s | 221,487 @ 456 s | 208,687 @ 315 s |
| 2000 | **366,996** @ 9 s   | 423,800 @ 60 s | 523,233 @ 909 s¹ | 368,373 @ 1607 s |
| 5000 | **779,161** @ 109 s | 868,583 @ 420 s | infeasible²     | did not finish³ |

<sub>Cores: commiv 3, VROOM 3 (`nb_threads=3`), OR-Tools and LKH-3 run their serial
search on 1. ¹ one unfinished LK trial. ² LKH could not reach a feasible packing even
warmstarted. ³ VROOM did not finish n=5000 within an hour. VROOM n≤2000 runs at
exploration level 5 (its highest-quality setting; lower levels are faster and worse).</sub>

commiv is fastest and cheapest at every size. The nearest competitor on cost is VROOM
(within about 0.4% to 0.6%), but its wall-clock blows up with scale: 20x slower at n=1000,
about 180x at n=2000. LKH-3 needs a feasible warmstart just to run on explicit directed
matrices and falls apart past n=1000. There is no published optimum here, but VROOM landing
independently within 0.5% is strong evidence the solutions are near-optimal.

Robustness checks on the n=1000 instance (so the single-number table above is not hiding
anything): commiv across seeds {1, 7, 42, 99, 777} lands in 206,815–208,355 — the published
seed sits mid-range, and even the worst seed beats every competitor. VROOM's full
exploration-level sweep (its own speed/quality knob, 3 threads) gives 213,406 @ 10 s
(level 0), 213,406 @ 22 s (level 1), 209,669 @ 112 s (level 3), 208,687 @ 315 s (level 5):
commiv's 207,406 @ 2 s dominates that entire frontier. OR-Tools given a 600 s budget (10x
the table) reaches 218,230, still +5.2%. commiv's routes are re-scored and
capacity-validated by the competitor harness's own Python checker, not by the solver's
internal accounting.

### Directed + time windows (Moscow VRPTW)

The same directed matrices with a courier-slot overlay: 60% of customers get one 2 h
delivery slot out of four in an 8 h shift, the rest are flexible; 300 s service per stop;
9 h depot horizon. Deterministic generator and independent schedule validator (every
solution below is checked by the same Python code): [`tools/competitors/vrptw_moscow.py`](tools/competitors/vrptw_moscow.py).
Two budget points per solver, cost @ wall (vehicles).

| n | commiv (SISR-TW) | VROOM level 1 / 5 | OR-Tools 60 s |
|---|---|---|---|
| 100  | **45,993 @ 1.7 s (10 veh)** | 46,486 @ 1.7 s (11) | 47,538 @ 60 s (11) |
| 1000 | **232,544 @ 3.5 s** (51); 229,909 @ 30 s at 3M iters | 246,573 @ 22 s / 239,977 @ 341 s (50) | 267,897 @ 60 s (53) |
| 2000 | **412,602 @ 10 s, 3 threads** (102) | not run¹ | not run¹ |
| 5000 | **884,137 @ 22 s, 3 threads** (257) | not run¹ | not run¹ |

<sub>commiv: `solveVrptwSisr`, default 300k iterations, single-threaded except n≥2000
(best-of-3 chains; n=5000 served over the binary REST framing). VROOM 3 cores, OR-Tools 1.
Seeds {7, 42, 12345} at n=1000 span
231,417–232,544; best-of-3 parallel: 231,892 @ 4.9 s. Objective is pure distance
(`veh_penalty = 0`); VROOM's n=1000 solution uses one vehicle fewer at 3.2% more distance.
¹ n≥2000 competitor runs were skipped: VROOM level 5 already needs 1607 s on the same
matrix without windows, and did not finish n=5000 CVRP within an hour.</sub>

The history of this table is worth telling straight. The first published version was the
one benchmark commiv did not win: the original VRPTW engine (a giant-tour ILS predating
SISR) needed 988 s to reach 238,829 at n=1000 — a wash with VROOM at 3x the wall. The fix
was to port the flagship CVRP engine: SISR with time windows wired into recreate through
O(1) time-slack (Tws) feasibility evaluation. That engine (`solveVrptwSisr`, now the
default everywhere) produces the numbers above: better cost than every competitor point at
every size, in seconds, plus a vehicle saved at n=100. The old ILS remains available as
`solveVrptw` and reproduces the Solomon table; on Solomon, SISR beats it at its default
budget (0.135% vs 0.182% vehicle-matched mean, ~3x faster) and matches the vehicle counts
given budget — including the notoriously tight rc101 at 14 vehicles (2M iterations, 12 s)
via the fleet-minimization ruin that empties the smallest route when `veh_penalty` is set.
One negative result, kept honest: the CVRP split-string ("slack induction") ruin measured
WORSE with time windows on (Moscow n=1000: +0.8%), so it defaults off for VRPTW.

### PyVRP head-to-head, and two more cities

[PyVRP](https://github.com/PyVRP/PyVRP) (0.13.4) is the open-source descendant of HGS and
the strongest freely available quality reference that natively accepts directed matrices
with time windows — the one competitor that plays on commiv's home turf without adapters
bending the problem. Same instances, same window overlay, same independent validator;
PyVRP is single-threaded by design and gets a full core.

Moscow, at commiv's wall and at 6–10x more (cost @ seconds; commiv long runs in
parentheses for the equal-long comparison):

| n / mode | commiv | PyVRP @ equal wall | PyVRP @ more time |
|---|---|---|---|
| 100 CVRP  | 41,808 @ 0.8 s | 42,449 @ 1.0 s | **41,705 @ 6 s** (commiv 41,806 @ 6.3 s) |
| 1000 CVRP | **207,406 @ 2 s** | 214,285 @ 1.5 s | **203,190 @ 60 s** (commiv 205,315 @ 36 s) |
| 2000 CVRP | **366,996 @ 9 s** | 379,446 @ 6 s | **357,454 @ 161 s** (commiv 361,000 @ 16 s) |
| 100 TW    | **45,993 @ 1.7 s** | 46,505 @ 1.7 s | 46,267 @ 17 s |
| 1000 TW   | **232,544 @ 3.5 s** | 240,081 @ 3.7 s | **227,581 @ 341 s** (commiv 228,243 @ 299 s) |
| 2000 TW   | **412,602 @ 10 s** | 426,882 @ 11 s | **400,437 @ 501 s** (commiv 406,038 @ 50 s) |

At equal wall commiv wins every Moscow cell by 1.2–3.5%. Given ~10x the time, PyVRP
crosses on five of six cells, by 0.2–1.4% (only 100 TW holds against any PyVRP budget
tried). The crossing wall grows with size — at n=2000 PyVRP needs 161 s (CVRP) and 501 s
(TW) to pass numbers commiv produced in 9–50 s. That is the shape of the claim: commiv is
not "better than HGS at convergence" — it reaches ~98–99% of PyVRP's multi-minute quality
in 1–10% of the time.

To test that this is not a Moscow artifact, the same protocol ran on two more real
cities with opposite road topologies: NYC (Manhattan one-way grid, measured asymmetry
ratio 1.13–1.17, higher than Moscow's ~1.11) and Berlin (ring city, nearly symmetric at
1.06). All 12 commiv cells validate feasible; equal-wall margins (PyVRP cost vs commiv
cost, positive = commiv wins):

| cell | commiv | PyVRP | margin |
|---|---|---|---:|
| nyc-100 CVRP | **32,563** @ 0.9 s | 32,718 @ 0.9 s | +0.5% |
| nyc-1000 CVRP | **95,057** @ 2.0 s | 96,070 @ 2.2 s | +1.1% |
| nyc-2000 CVRP | **139,772** @ 9.0 s | 141,241 @ 10.0 s | +1.1% |
| berlin-100 CVRP | **34,046** @ 0.9 s | 34,292 @ 0.9 s | +0.7% |
| berlin-1000 CVRP | **111,631** @ 2.3 s | 111,855 @ 2.5 s | +0.2% |
| berlin-2000 CVRP | 167,232 @ 9.9 s | **166,234** @ 10.8 s | **−0.6%** |
| nyc-100 TW | 36,992 @ 1.7 s | **36,925** @ 1.7 s | **−0.2%** |
| nyc-1000 TW | **135,873** @ 5.0 s | 138,044 @ 5.2 s | +1.6% |
| nyc-2000 TW | **221,184** @ 14.5 s | 229,132 @ 17.2 s | +3.6% |
| berlin-100 TW | **36,967** @ 1.6 s | 37,335 @ 1.6 s | +1.0% |
| berlin-1000 TW | **152,784** @ 6.1 s | 154,941 @ 6.3 s | +1.4% |
| berlin-2000 TW | **247,678** @ 16.3 s | infeasible @ 19 s¹ | +2.7%¹ |

<sub>¹ At commiv's 16 s wall PyVRP found no feasible schedule at all on berlin-2000 TW;
its first feasible solution needed 63 s and still cost 254,459, +2.7% over commiv's 16 s
result.</sub>

Score at equal short wall: 10 of 12 cells to commiv, two thin losses (berlin-2000 CVRP
−0.6%, nyc-100 TW −0.2%). The pattern matches the physics: the margin tracks the asymmetry
of the city. NYC's one-way grid (most asymmetric) gives the widest margins; near-symmetric
Berlin is where PyVRP — an engine born symmetric — gets closest, and takes its one CVRP
cell.

The same cells at ~10x budgets (commiv given matching wall via more iterations):

| cell | commiv | PyVRP | margin |
|---|---|---|---:|
| nyc-100 CVRP | 32,371 @ 7.4 s | **32,285** @ 9 s | −0.3% |
| nyc-1000 CVRP | **92,815** @ 20.2 s | 92,843 @ 20.2 s | +0.03% |
| nyc-2000 CVRP | 134,779 @ 88 s | **134,478** @ 91 s | −0.2% |
| berlin-100 CVRP | 34,012 @ 8.4 s | **33,809** @ 9 s | −0.6% |
| berlin-1000 CVRP | 110,254 @ 24 s² | **109,717** @ 23 s | −0.5% |
| berlin-2000 CVRP | 163,860 @ 89 s | **162,243** @ 100 s | −1.0% |
| nyc-100 TW | 36,827 @ 16.4 s | **36,526** @ 17 s | −0.8% |
| nyc-1000 TW | **134,885** @ 50 s | 135,171 @ 50 s | +0.2% |
| nyc-2000 TW | 220,036 @ 141 s | **214,471** @ 148 s | −2.5% |
| berlin-100 TW | 37,008 @ 16.4 s | **36,910** @ 16 s | −0.3% |
| berlin-1000 TW | 152,889 @ 60 s | **152,053** @ 61 s | −0.6% |
| berlin-2000 TW | **246,602** @ 164 s | 247,822 @ 166 s | +0.5% |

<sub>² commiv's 3M-iteration run at 11.7 s scored 109,774, better than the 6M run shown —
long SISR trajectories are not monotone in budget (a known limitation of the current
threshold schedule; also visible at berlin-100 TW and Moscow 100 TW).</sub>

So the honest division of the map: commiv owns the seconds regime nearly everywhere;
give both engines minutes and the HGS machinery grinds past on most cells by 0.05–2.5%,
with nyc-2000 TW its biggest win (−2.5%) and two deep directed+windowed cells still
holding for commiv (nyc-1000 TW, berlin-2000 TW). Directedness sets how *long* commiv's
lead survives: the more asymmetric and windowed the cell, the further out the crossing.

### Reclaiming the minutes regime (opt-in VRPTW levers)

The table above is the **baseline** engine. commiv also ships an opt-in set of long-run
levers — a post-accept local-search polish (FILO-style education), stress-guided ruin
centers, a reinsertion tabu, and a colder large-neighborhood "marathon" profile — all off
by default and enabled per call (`polish`, `stress_rate`, `tabu_tenure`, `marathon`; or
`VT_COMBO=1` in the bench). They spend per-iteration wall, so they lose in the seconds
regime and exist for the minutes budgets where the table above handed cells to PyVRP. With
them on, the deep directed+windowed cells flip back:

| n=1000 TW | commiv (levers on) | PyVRP | margin |
|---|---|---|---:|
| berlin | **151,417** @ 132 s | 151,948 @ 150 s | +0.35% |
| nyc | **132,996** @ 151 s | 135,054 @ 150 s | +1.52% |
| moscow | 229,544 @ 141 s | **227,581** @ 341 s | −0.86% |

Two of the three long-budget TW cells come back to commiv at equal-or-less wall. Moscow —
the most-tuned instance in the suite — is the one PyVRP keeps: its HGS reaches a basin the
SISR trajectory plateaus above (10 M iterations move it only from 229,511 to 229,544). The
levers scale where there is slack to recover: at n=2000 they take nyc TW to 213,500 (−3.0 %
and two vehicles under baseline, at roughly twice the baseline wall — the equal-wall n=2000
rerun is still open), and at n=5000 they even take *Moscow* TW to 866,679 (−0.7 %, one
vehicle fewer) at near-equal wall — the larger, less-converged instances benefit most.

The `nbr_key`/`gk` knobs (below) and the `polish_every` cadence apply to the TW engine
too, and together they retake nyc-2000 TW — PyVRP's biggest win in the baseline tables —
at *equal-or-less* wall, no 2x asterisk (`combo + nbr_key=min + polish_every=8`, 800 k
iterations, 3 seeds, `commiv-twroadbench` protocol):

| nyc-2000 TW @ ≤150 s | seed 12345 | seed 7 | seed 99 | mean |
|---|---|---|---|---:|
| commiv (109–123 s) | 215,026 | 215,052 | **214,233** | **214,770** |
| PyVRP (150 s) | 214,812 | 220,158 | 216,533 | 217,168 |

commiv's *worst* seed beats PyVRP's 3-seed mean (+1.1 %). Sparse polish cadence is the
n=2000 ingredient: per-accept polish costs too much wall at that scale, and
`polish_every=8` bought both the iterations and three vehicles over every-accept polish.
On nyc-1000 TW at 150 s commiv holds either way (~133.2 k vs PyVRP's 133.9 k 3-seed mean,
17–19 vehicles vs their 19–20); `min` is equal-wall-neutral there, and Berlin TW measured
neutral too, so `.sum` stays the TW default as well.

On the CVRP engine, three of the four ports (polish, stress, tabu) measured dead across
cities — the window-free flagship is already near-converged and they only perturb it.
`marathon` is alive there (an earlier "measured dead" verdict traced to a stale bench
binary that silently ignored the flag): at 1M+ iterations it fixes the long-run
non-monotonicity on near-symmetric instances. berlin-1000 baseline *worsens* with budget
(109,774 @ 13 s at 3 M → 110,254 @ 30 s at 6 M); marathon gives 109,181 @ 18 s (4 M) and
109,051 @ 29 s (6 M), better than baseline on 3/3 seeds, taking the berlin-1000 minute
cell under the tables' fixed-seed protocol (PyVRP rerun at its 23 s wall: 109,522). NYC —
the most asymmetric city — is the mirror image: every deviation from the base constants
(marathon, colder tf alone, education at any cadence) measured *worse* there, and Moscow
is a seed-level wash, so `marathon` stays opt-in.

NYC's real lever turned out to be structural, not a constant: the granular neighbor
lists were built with a symmetrized key `d(c,j)+d(j,c)`, which buries one-way-close
pairs — on the NYC grid ~3 % of each customer's five nearest *directional* arcs never
made the top-20 list (Berlin: 0.4 %), so recreate and every local-search move were
blind to exactly the arcs that make NYC asymmetric-favorable, while the ATSP seed
(min-key) could see them. `nbr_key = .min` (+ `gk` list-size knob; both default to the
old behavior bit-identically) restores them at zero per-iteration cost:

| CVRP minutes, 3 seeds each | commiv (`nbr_key=min`) | PyVRP | mean margin |
|---|---|---|---:|
| nyc-1000 (min, 4 M, ≤18 s) | **91,639 / 91,793 / 92,159** | 92,751 / 93,678 / 93,391 @ 20 s | **+1.51 %** |
| nyc-2000 (min, gk=40, 5 M, ≤91 s) | **133,815 / 133,894 / 134,303** | 134,549 / 133,421 / 134,676 @ 91 s | **+0.16 %** |

Every nyc-1000 seed beats PyVRP's best seed; nyc-1000's *seconds* cell also improves
(95,057 → 93,832 @ 2.4 s with `min`). At the 60-second wall the margin holds and grows
earlier: commiv 13 M `min` iterations finish in ~49 s at 91,405 / 91,714 / 91,973 vs
PyVRP's 92,525 / 93,484 / 92,786 at 60 s — every commiv seed beats PyVRP's best seed
with ten seconds to spare (+1.33 % on means). The key is topology-gated like marathon:
Moscow (ratio 1.11) and Berlin (1.06) measured neutral-to-worse on `min`, so `.sum`
stays the default. Seed-honest summary of the CVRP minute regime with both levers:
nyc-1000 is commiv's outright, nyc-2000 commiv on 3-seed means, berlin-1000
tie-to-commiv, the moscow cells stay PyVRP's by 0.4–0.8 %.

Reproduce: `tools/competitors/pyvrp_road.py <instance.road> {cvrp|vrptw} <seconds> [seed]`;
the NYC and Berlin matrices live in `vendor/road/` next to Moscow. TW levers are the
`solveVrptwSisr` flags above (see [`docs/rest.md`](docs/rest.md) for the REST fields); the
CVRP cells (`zig build roadbench -Doptimize=ReleaseFast` first):
`RB_FILES=berlin-1000 RB_ITERS=4000000 RB_THREADS=3 RB_SYM=0 RB_MARATHON=1
./zig-out/bin/commiv-roadbench` → 109,181;
`RB_FILES=nyc-1000 RB_ITERS=4000000 RB_THREADS=3 RB_SYM=0 RB_NBR=min
./zig-out/bin/commiv-roadbench` → 91,639;
`RB_FILES=nyc-2000 RB_ITERS=5000000 RB_THREADS=3 RB_SYM=0 RB_NBR=min RB_GK=40
./zig-out/bin/commiv-roadbench` → 133,815.
The road-TW cells: `zig build twroadbench -Doptimize=ReleaseFast && python3
tools/competitors/dump_windows.py nyc-2000 && TP_FILE=nyc-2000 TP_ITERS=800000
TP_THREADS=3 TP_COMBO=1 TP_NBR=min TP_POLISH_EVERY=8 ./zig-out/bin/commiv-twroadbench`
→ 215,026, 33 vehicles, ~123 s (PyVRP side: `pyvrp_road.py vendor/road/nyc-2000.road
vrptw 150 [seed]`).

---

## How commiv compares, honestly

**Where it wins**

- **The speed and quality frontier.** Near-optimal in seconds, not the minutes-to-hours the
  reference heuristics spend. For a planner that has to replan constantly, this is the
  number that matters.
- **Directed real-road matrices.** Asymmetric cost is first-class. FILO and HGS-CVRP, the
  symmetric speed and quality champions, physically cannot read a directed matrix. On real
  Moscow data commiv beats OR-Tools, LKH-3, and VROOM on cost and time.
- **Zero dependencies, small footprint.** One Zig module, and a 5000-node directed CVRP in
  211 MB.

**Where the competition wins, and you should know it**

- **Absolute accuracy at huge budgets.** LKH-3, HGS-CVRP, and SISR (the paper) reach lower
  gaps (about 0.16% to 0.39% on Uchoa X) when given far more time, and PyVRP crosses
  commiv's seconds-scale numbers by 0.05–2.5% when both get minutes — the crossing wall
  grows with instance size and asymmetry, but it exists on almost every cell. commiv
  trades that last fraction of a percent for a large speed advantage. It is not
  state-of-the-art on accuracy at the frontier.
- **Dedicated fleet minimization.** On the vehicles-first Gehring–Homberger objective
  commiv matches the best-known fleet on only 4 of 12 instances; the record holders run an
  ejection-pool route-minimization phase commiv does not have yet.
- **Massive symmetric instances.** FILO solves symmetric CVRPs with tens of thousands of
  nodes faster than anything here. commiv targets the routing-scale (hundreds to a few
  thousand) directed regime.
- **Production hardness.** OR-Tools and VROOM are battle-tested stacks with rich constraints
  (time windows, pickup and delivery, skills, breaks) and years of deployment. commiv is a
  fast, focused core, not a complete logistics platform.

---

## Design decisions

### What we settled on, and why

- **SISR (Slack Induction by String Removals) for large and asymmetric CVRP — and VRPTW.**
  Ruin a few spatially-adjacent strings, greedily re-insert with random blinks, accept
  under a threshold or SA. The bet: millions of `O(removed)` moves beat thousands of
  `O(n)` ones. This is what cracks the large-n and directed regimes. For time windows the
  same loop runs with per-route prefix/suffix time-slack (Tws) structures, so "is this
  insertion feasible and what does it cost" is O(1) per candidate gap — that one change
  took Moscow n=1000 VRPTW from 988 s (ILS) to 4 s at better cost.
- **HGS (population plus Prins Split DP) for mid-size CVRP, n up to about 500.** A genetic
  population of giant tours with optimal capacity splitting and local-search education gives
  the best quality at that scale (the 0.02% to 0.45% numbers above).
- **Penalty-based infeasible search.** Letting local search cross capacity-infeasible
  regions, at a penalty, broke a hard ~2% quality ceiling that feasible-only search could
  not.
- **Native directed ATSP for degenerate matrices.** The stacker-crane rbg instances have
  many arcs tying each row minimum, and the Jonker-Volgenant 2n transform pays for that
  twice. A direct directed local search (Or-opt plus directed 2-opt plus double-bridge)
  reaches the optimum faster, on n nodes instead of 2n.
- **Granular don't-look-queue local search.** Restricting moves to spatial neighbours and
  re-activating only changed-edge endpoints made large-n local search 2x to 4x faster at
  equal quality.
- **In-place matrix-view seed.** The CVRP giant-tour seed reads the cost matrix directly via
  a strided view instead of copying out an `n x n` sub-matrix or building a 2n transform.
  That dropped n=5000 memory from about 2 GB to 211 MB and the solve from 412 s to 109 s,
  with identical quality (the seed is a throwaway that SISR rebuilds).
- **Parallelism is a speed lever, not magic.** Best-of-K seeds and EAX recombination help
  accuracy at equal wall-clock on multiple cores; a deterministic split-budget mode trades a
  little quality for about 2.5x speed.

### What we tried and rejected, and why

- **Static Move Descriptors (SMD).** Our don't-look-queue is 4x to 5x faster at identical
  quality; the DLQ already captures the locality SMD buys. Dead end.
- **Two-level doubly-linked tour list.** After fixing a fallback that was firing on provably
  doomed rebuilds, tour rebuilds dropped about 10x. Re-measured 2026-07: the entire remaining
  target (`applyEdges`, the O(n) retrace + rebuild per accepted move) is 2.5–2.8% of
  wall-clock at n=575–1577 and 5.6% at n=11849 — the rewrite's ceiling, before paying the
  per-query segment indirection that every `next`/`prev` read in the LK inner loop would eat.
  Closed.
- **Cooperative and best-of parallelism.** High variance and lock contention made it slower
  than independent islands. Removed.
- **Decomposition for large n.** A subproblem-resolve win on converged TSP tours did not
  generalize to never-converging SISR. Plain SISR run longer dominated.
- **Adaptive candidate re-ranking.** Even a perfect oracle re-rank washes out at full ILS
  budget. Candidate order is a single-descent lever, not an accuracy lever.
- **Edge-freezing and voting.** Freezing even a pure subset of known-optimal edges loses
  accuracy, because Lin-Kernighan has to break and rebuild even optimal edges along the way.
  Structural.
- **Assignment-bound early stop.** The AP lower bound is too loose for capacity-tight CVRP to
  certify near-optimality (19% to 52% on Moscow). Useless as a stopping rule here.
- **Route-pool recombination across parallel SISR chains.** Adaptive-memory offspring
  (cheapest disjoint routes from every island, clash-stripped with a TW re-check, leftovers
  as singletons, short SISR polish) never beat plain best-of-K on Moscow VRPTW at n=100 or
  n=1000 — even with a perfect repair the offspring starts ~46% above the best island, and
  spending the polish wall on extra best-of-K iterations wins instead. The TSP recombination
  gain does not transfer: SISR's threshold schedule makes the seed nearly irrelevant (measured
  before — swapping the seed tour changes nothing), so there is no incumbent-trajectory
  memory for an offspring to inject. LK islands have exactly that memory, which is why EAX
  recombination pays there and not here.

---

## GPU acceleration (designed, not built)

commiv is CPU-only today. The single largest untapped speedup is a GPU. SISR's hot loop
evaluates millions of independent move-deltas per second, an embarrassingly parallel batched
reduction that maps cleanly onto a GPU with the directed matrix held device-resident (100 MB
at n=5000 fits any modern card). A full task spec (batched move-delta kernel, massive
best-of-K islands, CUDA FFI, device-resident matrix) is in [`gpu.md`](gpu.md). It is not
implemented (there is no GPU in the dev environment), but it is the most likely path to
another order of magnitude at large n.

---

## Reproducing the benchmarks

Standard instances ship under `vendor/` (TSPLIB, CVRPLIB Augerat and Uchoa X, ATSP, ACVRP,
Solomon, and the Moscow OSRM matrices under `vendor/road/`). The `moscow-5000` matrix is
gzipped, so run `gunzip vendor/road/moscow-5000.road.gz` before using it. Competitor
adapters (OR-Tools, LKH-3 with warmstart, VROOM, and an assignment lower bound) plus setup
notes are in [`tools/competitors/`](tools/competitors/). To re-fetch a Moscow matrix from a
self-hosted OSRM, use [`tools/fetch_road_matrix.py`](tools/fetch_road_matrix.py).

The gap benchmarks build their own binary that you then run:

```sh
zig build cvrpbench  -Doptimize=ReleaseFast && ./zig-out/bin/commiv-cvrpbench   # CVRP vs optima
zig build acvrpbench -Doptimize=ReleaseFast && ./zig-out/bin/commiv-acvrpbench  # asymmetric CVRP vs LKH-3
zig build atspbench  -Doptimize=ReleaseFast && ./zig-out/bin/commiv-atspbench   # ATSP vs proven optima
zig build vrptwbench -Doptimize=ReleaseFast && ./zig-out/bin/commiv-vrptwbench  # VRPTW vs SINTEF BKS
zig build roadbench  -Doptimize=ReleaseFast && ./zig-out/bin/commiv-roadbench   # real directed Moscow matrix
zig build bench      -Doptimize=ReleaseFast                                     # TSP benchmark (runs)
```

## License

See [`LICENSE`](LICENSE).

<br>

---
---

<br>

<a name="commiv-rus"></a>
# commiv (Русский)

**[English](#commiv) · [Русский](#commiv-rus)**

Поиск почти оптимального маршрута за секунды. Рассчитан на асимметричные маршруты, приближенные к реальным условиям.
Без зависимостей, полностью написан на языке Zig.

commiv решает семейства задач коммивояжёра и маршрутизации транспорта (TSP, ATSP, CVRP,
ACVRP, VRPTW) с точностью до доли процента от оптимума и читает ориентированные матрицы
времени в пути напрямую: улицы с односторонним движением, повороты, заторы.

На реальных данных Moscow OSRM направленная CVRP при n=1000 решается за **207 406 за 2 с** —
против 225 917 за 60 с у OR-Tools, 221 487 за 456 с у LKH-3 и 208 687 за 315 с у VROOM.
Дешевле и быстрее одновременно, и это можно перепроверить на своих инстансах за вечер.

> Для платформ курьерской доставки, последней мили и маршрутизации автопарков, которые
> пересчитывают тысячи направленных дорожных маршрутов под жёстким лимитом по времени,
> commiv — встраиваемое ядро маршрутизации, возвращающее почти оптимальные и допустимые по
> вместимости маршруты за секунды. В отличие от LKH-3 (однопоточный, некоммерческая лицензия
> и неспособный обрабатывать явные направленные матрицы после ~n=1000) или FILO и HGS-CVRP
> (быстрые, но только симметричные), он считает асимметричную дорожную стоимость основным
> случаем, а не довеском.

```sh
zig build                                  # собрать библиотеку
zig build test                             # запустить модульные тесты
zig build example                          # запустить пример встроенного солвера
zig build serve -Doptimize=ReleaseFast     # REST API сервер (JSON поверх HTTP)
zig build lib   -Doptimize=ReleaseFast     # C ABI: libcommiv.{a,so} + commiv.h
```

- **Приближено к оптимуму.** 0.02% от доказанных оптимумов на стандартной CVRP, около
  0.45% на тяжёлом наборе Uchoa X, за секунды на ноутбуке.
- **Вызывается из любого языка.** REST-сервер, C ABI и биндинг для Python лежат прямо в
  репозитории. Zig — это движок под капотом; писать на нём не нужно.
- **Нативная асимметрия.** Ориентированные матрицы времени в пути нативно поддерживаются. На реальных данных Moscow OSRM commiv обходит OR-Tools, LKH-3 и VROOM и
  по стоимости, и по времени.
- **Ноль зависимостей** Один модуль на Zig, без системных библиотек, без
  загрузок при сборке.
- **Экономично по памяти.** Ориентированная CVRP на 5000 узлов решается за 109с при 211 МБ, и 100 МБ
  из них — сама матрица.

---

## Работа с commiv

Zig — движок под капотом, но входную дверь вы выбираете под свой стек, и ни одна из них
не требует писать на Zig:

| Ваш стек | Дверь | Где |
|---|---|---|
| Любой язык | REST-сервер: один статический бинарник, JSON поверх HTTP | [`docs/rest.md`](docs/rest.md) |
| Python | Нативный биндинг (ctypes поверх C ABI, дружит с numpy) | [`bindings/python/`](bindings/python/) |
| C, C++, Go, Rust, ... | C ABI: `libcommiv.{a,so}` + [`include/commiv.h`](include/commiv.h) | `zig build lib` |
| Zig | Сам модуль | этот раздел |

### REST — из чего угодно

```sh
zig build serve -Doptimize=ReleaseFast && ./zig-out/bin/commiv-serve
```

```sh
curl -X POST http://127.0.0.1:8080/solve/cvrp -d '{
  "matrix": [[0,10,14,12],[11,0,9,20],[15,8,0,7],[13,18,6,0]],
  "demand": [0,4,6,5],
  "capacity": 10
}'
# {"total_cost":58,"vehicles":2,"routes":[[2,1],[3]]}
```

Это вся интеграция: направленная матрица стоимостей (строка `a`, столбец `b` = стоимость
`a -> b`, депо = узел 0), спрос, вместимость. `/solve/vrptw` добавляет временные окна,
`/solve/atsp` — чистое направленное упорядочивание. Полная схема и примеры клиентов на
Python/JS/Go — в [`docs/rest.md`](docs/rest.md).

### Python — нативно

```python
import commiv  # pip install -e bindings/python; один раз zig build lib

sol = commiv.solve_cvrp(matrix, demand=[0, 4, 6, 5], capacity=10, seed=1)
print(sol.total_cost, sol.routes)  # 58 [[2, 1], [3]]
```

Матрицы numpy идут по быстрому пути; недопустимая задача поднимает исключение, а не
возвращает мусор. Подробности — в [`bindings/python/README.md`](bindings/python/README.md).

### Zig — встраивание

Дальше — полное руководство по встраиванию на Zig.

#### 1. Добавить зависимость

```sh
zig fetch --save "git+https://github.com/grechman/Commiv"
```

Пакет сохранится под именем `commiv`. Подключите модуль в `build.zig`:

```zig
const commiv = b.dependency("commiv", .{
    .target = target,
    .optimize = optimize,
});
exe.root_module.addImport("commiv", commiv.module("commiv"));
```

#### 2. Решить задачу маршрутизации на своих данных

Полные рабочие примеры лежат в каталоге [`examples/`](examples/): `basic.zig` разбирает инстанс TSPLIB через `parseTsplib`, а `roadbench.zig` и `cvrpbench.zig` читают свои инстансы CVRP и дорожных матриц с диска собственными парсерами. `parseTsplib` читает только симметричную TSP в формате TSPLIB — секцию координат (`EUC_2D`, `CEIL_2D` или `ATT`) либо `EXPLICIT` `FULL_MATRIX`, — поэтому инстансы CVRP, ACVRP и ATSP через него не загружаются. Ниже — минимальный встроенный вариант на своих данных.

Вы передаёте матрицу стоимостей `(n+1) x (n+1)` в строковом порядке (узел 0 — это депо,
клиенты — `1..n`), массив `demand` и вместимость `capacity`. Матрица направленная:
`matrix[a*(n+1) + b]` — это стоимость пути из `a` в `b`, так что реальная асимметричная
дорожная стоимость подставляется как есть.

```zig
const std = @import("std");
const commiv = @import("commiv");

pub fn main() !void {
    const allocator = std.heap.page_allocator; // swap in your own (gpa, arena, ...)

    // 3 клиента + депо. Направленные стоимости (a -> b), строковый порядок, депо = индекс 0.
    const n: usize = 3;
    const matrix = [_]u32{
        0,  10, 14, 12, // депо -> {депо, c1, c2, c3}
        11, 0,  9,  20, // c1   -> ...
        15, 8,  0,  7,  // c2   -> ...
        13, 18, 6,  0,  // c3   -> ...
    };
    const demand = [_]u32{ 0, 4, 6, 5 }; // demand[0] = 0 (у депо спроса нет)

    const inst = commiv.CvrpInstance{
        .n = n,
        .matrix = &matrix,
        .demand = &demand,
        .capacity = 10,
    };

    // SISR — рабочая лошадка по умолчанию: лучше всего для больших и/или направленных задач.
    var result = try commiv.solveCvrpSisr(allocator, inst, .{ .seed = 1 }, .{});
    defer result.deinit();

    std.debug.print("total cost = {}\n", .{result.total_cost});
    for (result.routes, 0..) |route, v| {
        std.debug.print("vehicle {}: depot", .{v});
        for (route) |customer| std.debug.print(" -> {}", .{customer});
        std.debug.print(" -> depot\n", .{});
    }
}
```

#### 3. Прочитать результат

`CvrpResult` владеет своей памятью, так что вызовите `deinit()`, когда закончите.

- `result.total_cost` — суммарная стоимость маршрутов по вашей матрице, тип `u64`.
- `result.routes` — по одному срезу на машину. Каждый срез перечисляет индексы клиентов в
  порядке посещения, депо подразумевается на обоих концах.

#### 4. Выбрать солвер под размер задачи

| Точка входа | Когда использовать |
|---|---|
| `solveCvrpSisr` | Большие и/или направленные задачи. Это случай дорожной сети. |
| `solveCvrpHgs` | Средняя CVRP, n примерно до 500, когда нужна последняя доля качества. |
| `solveCvrpSisrParallel` / `solveCvrpHgsParallel` | То же самое, но по нескольким ядрам. |
| `solveCvrp` | Точка входа по умолчанию без настройки. Запускает SISR. |

#### 5. Обычные TSP и ATSP

Для чистой задачи упорядочивания без вместимости используйте точки входа TSP. Направленная
матрица идёт по пути ATSP; форма вызова та же.

```zig
// Симметрично, из координат или инстанса TSPLIB:
var p = try commiv.parseTsplib(allocator, tsplib_text, .{});
defer p.deinit();
var tour = try commiv.solve(allocator, &p, .{ .seed = 1 });
defer tour.deinit();
// tour.length и tour.tour содержат результат.

// Направленно, из матрицы n x n:
var atsp = try commiv.solveAtsp(allocator, &cost_matrix, n, .{ .seed = 1 });
defer atsp.deinit();
```

#### Важные настройки

- `SolveOptions.seed` — сид генератора случайных чисел. Для однопоточных солверов один и тот же сид даёт побайтово идентичный результат, так что прогоны воспроизводимы. Варианты
  `*Parallel` зависят ещё и от числа потоков: их значение по умолчанию `threads = 0`
  разрешается в число ядер хоста, которое задаёт число островов/цепочек и, значит, посевы
  (seed) каждого острова, поэтому один и тот же сид даёт разные маршруты на машинах с разным
  числом ядер. Для воспроизводимого между машинами результата передавайте явное ненулевое
  `threads` (`ParallelOptions.threads` для `solveParallel`) и фиксируйте и сид, и это число
  потоков.
- `SolveOptions.budget.trials` и `.max_passes` определяют, насколько усердно работает поиск.
  Больше — ближе к оптимуму и дольше по времени. Бюджет считается в итерациях, а не по часам,
  так что подбирайте его под свою цель по задержке эмпирически.
- Каждый возвращённый маршрут соблюдает `capacity`. Задача без допустимой упаковки вернёт
  ошибку.

---

## Документация API

Каждый солвер сначала принимает аллокатор, затем структуру опций и возвращает результат, который
вы освобождаете через `deinit()`. Перечисленный ниже набор — это весь публичный API (он
повторяет [`src/root.zig`](src/root.zig)); у каждого солвера есть модульные тесты в его файле.

**Разбор**
- `parseTsplib(allocator, text, ParseOptions) !Problem` разбирает текст симметричной TSP в
  формате TSPLIB — секцию координат (`EUC_2D`, `CEIL_2D` или `ATT`) либо `EXPLICIT` `FULL_MATRIX`.
  Передайте `ParseDiagnostic` в опциях, чтобы поймать ошибки разбора по строкам.

**Определение задачи** (путь координат / TSPLIB)
- `Problem`, создаётся через `Problem.initCoords(...)` или `Problem.initFullMatrix(...)`, плюс
  типы `Coord` и `DistanceKind`.

**Общие опции и результат**
- `SolveOptions` — `seed`, `budget` (`trials`, `max_passes`), настройки кандидатов и поиска.
- `SolveResult` — `{ tour, length, stats }`, единственный тип, возвращаемый `solve`,
  `solveAtsp*` и `bruteForce`. `SolveStats` — телеметрия прогона; `CandidateMode` выбирает
  метрику графа кандидатов.

**TSP (симметричная)**
- `solve(allocator, *Problem, SolveOptions) !SolveResult` — Lin-Kernighan + ILS.
- `solveWithStats(...)` — то же, что `solve`, дополнительно заполняет телеметрию `SolveStats`.
- `solveParallel(allocator, *Problem, SolveOptions, ParallelOptions) !SolveResult` —
  независимые острова с опциональной рекомбинацией EAX или детерминированный режим деления
  бюджета ради скорости. `ParallelOptions.threads == 0` разрешается в число ядер хоста, что
  меняет посев островов и, значит, результат; для воспроизводимого между машинами результата
  передавайте явное ненулевое `threads`.

**ATSP (направленная)** — матрица `n x n` в строковом порядке, `matrix[i*n + j]` = стоимость `i → j`
- `solveAtsp(allocator, matrix, n, SolveOptions) !SolveResult` — 2n-преобразование Йонкера-Волгенанта.
- `solveAtspNative(allocator, matrix, n, SolveOptions) !SolveResult` — прямой направленный поиск.
- `solveAtspParallel(allocator, matrix, n, SolveOptions, threads) !SolveResult` — `threads == 0`
  разрешается в число ядер хоста, меняя результат; передавайте ненулевое `threads` для
  воспроизводимости между машинами.

**Точное решение (крошечное n)**
- `bruteForce(allocator, *Problem, ExactOptions) !SolveResult`.

**CVRP / ACVRP** — соберите `CvrpInstance { n, matrix, demand, capacity }` с направленной
матрицей `(n+1) x (n+1)` (депо = узел 0); все солверы возвращают `CvrpResult { routes, total_cost }`
- `solveCvrp(allocator, inst, SolveOptions) !CvrpResult` — точка входа по умолчанию (запускает SISR).
- `solveCvrpSisr(allocator, inst, SolveOptions, CvrpSisrParams)` — большие / направленные.
- `solveCvrpHgs(allocator, inst, SolveOptions, CvrpHgsParams, max_vehicles)` — n ≲ 500.
- `solveCvrpFleet(allocator, inst, SolveOptions, CvrpFleetParams)` — фиксированный парк.
- `solveCvrpSisrParallel(allocator, inst, SolveOptions, CvrpSisrParams, threads)` — `threads == 0`
  разрешается в число ядер хоста, меняя результат; передавайте ненулевое `threads` для
  воспроизводимости между машинами.
- `solveCvrpHgsParallel(allocator, inst, SolveOptions, CvrpHgsParams, max_vehicles, threads)` —
  та же оговорка про `threads == 0`, что и выше.
- `solveCvrpMulti(allocator, inst, SolveOptions, CvrpMultiParams)` — вариант ILS по гигантскому
  туру без ограничения парка (устаревший; SISR обычно его превосходит).
- `validateCvrp(inst, routes) ?u64` — независимая проверка допустимости решения; возвращает
  пересчитанную стоимость либо null, если маршрут недопустим или клиент пропущен.

**VRPTW** — соберите `VrptwInstance { n, matrix, demand, capacity, ready, due, service }`;
возвращает `VrptwResult`
- `solveVrptwSisr(allocator, inst, SolveOptions, VrptwSisrParams)` — движок по умолчанию:
  SISR с временными окнами. Используйте его.
- `solveVrptwSisrParallel(allocator, inst, SolveOptions, VrptwSisrParams, threads)` —
  лучшее из K цепочек; та же оговорка про `threads == 0`, что и у CVRP-варианта.
- `solveVrptw(allocator, inst, SolveOptions, VrptwParams) !VrptwResult` — старый ILS по
  гигантскому туру (оставлен для воспроизводимости; SISR равен или лучше при равном времени).
- `solveVrptwHgs(allocator, inst, SolveOptions, VrptwHgsParams) !VrptwResult`.
- `validateVrptw(inst, routes) ?u64` — независимая проверка вместимости и временных окон;
  возвращает пересчитанную стоимость либо null, если решение недопустимо.

**Анализ асимметрии**
- `conservativeness(allocator, matrix, dim) !Conservativeness` выполняет разложение
  Гельмгольца-Ходжа направленной матрицы. Оно показывает, какая часть асимметрии структурна
  (односторонние улицы и повороты, которые меняют оптимальный маршрут), а какая — градиент
  (заторы, которые можно спокойно игнорировать). Наведите его на любую матрицу стоимостей,
  чтобы решить, нужна ли вам вообще направленная маршрутизация.

Всё остальное (`commiv.internal.*`, сырые модули реализации) — нестабильные детали, не входят в
этот API и могут меняться между версиями.

---

## Сценарии применения

- **Последняя миля и курьерская маршрутизация по реальным дорогам.** Подайте направленную
  матрицу времени в пути в стиле OSRM и получите допустимые по вместимости маршруты, которые
  учитывают односторонние улицы и стоимость поворотов. Это случай, под который сделан commiv.
- **Классические TSP, CVRP и VRPTW.** Близкие к оптимальным решения намного быстрее точных
  методов.
- **Встраиваемое ядро.** Один модуль без зависимостей, который кладётся внутрь более крупного
  планировщика.
- **Не только логистика.** Сверловка отверстий на печатной плате, облёт точек съёмки дроном,
  маршруты и патрули NPC в видеоиграх, обход ячеек склада комплектовщиком, планирование
  обходов у выездных бригад — любая задача, где важен порядок посещения, а переходы
  асимметричны.

---

## Бенчмарки

Каждый разрыв указан относительно показанного эталона: доказанного оптимума, опубликованного
лучшего известного результата или эталонного солвера. Железо — Intel i3-1115G4
(2 ядра, 4 потока).

### Симметричные

| бенчмарк | эталон | инстансы | разрыв commiv | бюджет / время |
|---|---|---:|---:|---|
| TSP (TSPLIB) | доказанные оптимумы | rat575 / pr1002 / fl1577 / d657 | 0.089% / **0.000%** / 0.031% / 0.008% | ILS, секунды |
| TSP большая (rl11849) | доказанный оптимум | 1 | 0.690% | ~77 с, один прогон |
| CVRP, Augerat A | доказанные оптимумы | 12 | **0.021%** | ~1 с/инстанс |
| CVRP, Uchoa X | лучший известный | 6 | **0.456%** (SISR 20M, 3 потока) | 50–170 с/инстанс |
| CVRP, Uchoa X | лучший известный | 6 | 0.711% (SISR 1M, лучшее из 3) | ~секунды |
| VRPTW, Solomon | SINTEF BKS | 5 | 0.182% по расстоянию (при равном числе машин) | ~секунды |

Uchoa X по инстансам (SISR 20M): X-n101 **0.120%**, X-n153 0.452%, X-n200 **0.143%**,
X-n303 0.902%, X-n502 **0.087%**, X-n1001 1.034%. Два тяжёлых инстанса (X-n303 и X-n1001)
держатся около 1%; остальные ложатся между 0.09% и 0.45%.

### Асимметричные

| бенчмарк | эталон | инстансы | разрыв commiv | время |
|---|---|---:|---:|---|
| ATSP (классика TSPLIB) | доказанные оптимумы | 14 | **0.000%** (br17…kro124p) | от долей секунды до 18 с |
| ATSP (rbg, кран-штабелёр) | доказанные оптимумы | 4 | 0.043% (rbg323/403/443 оптимальны; rbg358 +0.17%) | 17–24 с |
| ACVRP | LKH-3 (лучший в поле) | 30 | **0.228%** | ~1–2 с/инстанс |

### Реальные направленные дорожные данные (Москва, OSRM)

`moscow-*` — это собственный бенчмарк: реальные ориентированные матрицы времени в пути OSRM,
снятые по центру Москвы, не опубликованный набор инстансов, поэтому цитируемого оптимума нет.
Он существует, чтобы сравнивать commiv с солверами, которые тоже принимают направленные
матрицы, на одинаковых инстансах, при одинаковом подсчёте (стоимость маршрута по настоящей
направленной матрице, с проверкой вместимости). Харнесс — в
[`tools/competitors/`](tools/competitors/).

| n | commiv (SISR) | OR-Tools 9.15 | LKH-3 (тёплый старт) | VROOM |
|---|---:|---:|---:|---:|
| 100  | **41 808** @ 0.8 с  | 44 183 @ 8 с   | 43 090 @ 12 с   | 42 490 @ 1.3 с |
| 1000 | **207 406** @ 2 с   | 225 917 @ 60 с | 221 487 @ 456 с | 208 687 @ 315 с |
| 2000 | **366 996** @ 9 с   | 423 800 @ 60 с | 523 233 @ 909 с¹ | 368 373 @ 1607 с |
| 5000 | **779 161** @ 109 с | 868 583 @ 420 с | недопустимо²     | не завершил³ |

<sub>Ядра: commiv 3, VROOM 3 (`nb_threads=3`), OR-Tools и LKH-3 ведут последовательный
поиск на 1. ¹ один незавершённый прогон LK. ² LKH не смог получить допустимую упаковку даже
с тёплым стартом. ³ VROOM не завершил n=5000 в пределах часа. VROOM при n≤2000 идёт на
уровне исследования 5 (его самый качественный и медленный режим); при n=5000 — на уровне 3.</sub>

commiv быстрее и дешевле на каждом размере. Ближайший конкурент по стоимости — VROOM (в
пределах примерно 0.4–0.6%), но его настенное время взрывается с масштабом: в 20 раз медленнее
при n=1000 и примерно в 180 раз при n=2000. LKH-3 нужен допустимый тёплый старт просто чтобы
запуститься на явных направленных матрицах, и он разваливается после n=1000. Опубликованного
оптимума здесь нет, но то, что VROOM независимо попадает в пределах 0.5%, — сильное
свидетельство того, что решения близки к оптимальным.

Проверки устойчивости на инстансе n=1000 (чтобы единственное число в таблице ничего не
прятало): commiv на сидах {1, 7, 42, 99, 777} даёт 206 815–208 355 — опубликованный сид
лежит в середине, и даже худший сид бьёт каждого конкурента. Полный проход VROOM по его
уровням исследования (его собственная ручка скорость/качество, 3 потока): 213 406 @ 10 с
(уровень 0), 213 406 @ 22 с (уровень 1), 209 669 @ 112 с (уровень 3), 208 687 @ 315 с
(уровень 5) — 207 406 @ 2 с commiv доминирует весь этот фронт. OR-Tools с бюджетом 600 с
(10x от таблицы) достигает 218 230, всё ещё +5.2%. Маршруты commiv пересчитаны и проверены
на вместимость Python-чекером харнесса конкурентов, а не внутренней бухгалтерией солвера.

### Направленные дороги + временные окна (Москва, VRPTW)

Те же направленные матрицы с курьерскими слотами: 60% клиентов получают один 2-часовой слот
из четырёх в 8-часовой смене, остальные гибкие; 300 с обслуживания на точку; горизонт депо
9 ч. Детерминированный генератор и независимый валидатор расписаний (каждое решение ниже
проверено одним и тем же Python-кодом):
[`tools/competitors/vrptw_moscow.py`](tools/competitors/vrptw_moscow.py).
Две точки бюджета на солвер, стоимость @ время (машины).

| n | commiv (SISR-TW) | VROOM уровень 1 / 5 | OR-Tools 60 с |
|---|---|---|---|
| 100  | **45 993 @ 1.7 с (10 машин)** | 46 486 @ 1.7 с (11) | 47 538 @ 60 с (11) |
| 1000 | **232 544 @ 3.5 с** (51); 229 909 @ 30 с при 3M итераций | 246 573 @ 22 с / 239 977 @ 341 с (50) | 267 897 @ 60 с (53) |
| 2000 | **412 602 @ 10 с, 3 потока** (102) | не запускался¹ | не запускался¹ |

<sub>commiv: `solveVrptwSisr`, 300k итераций по умолчанию, однопоточно, кроме n=2000
(лучшее из 3 цепочек). VROOM 3 ядра, OR-Tools 1. Сиды {7, 42, 12345} при n=1000 дают
231 417–232 544; лучшее из 3 параллельно: 231 892 @ 4.9 с. Целевая функция — чистое
расстояние (`veh_penalty = 0`); решение VROOM при n=1000 использует на одну машину меньше
ценой +3.2% расстояния. ¹ Конкуренты при n=2000 не запускались: VROOM уровня 5 тратит
1607 с на той же матрице даже без окон.</sub>

Историю этой таблицы стоит рассказать честно. В первой опубликованной версии это был
единственный бенчмарк, который commiv не выигрывал: старый движок VRPTW (ILS по гигантскому
туру, старше SISR) тратил 988 с на 238 829 при n=1000 — ничья с VROOM при тройном времени.
Исправлением стал перенос флагманского движка CVRP: SISR с временными окнами, встроенными в
recreate через O(1)-проверку допустимости по запасам времени (Tws). Этот движок
(`solveVrptwSisr`, теперь по умолчанию везде) даёт числа выше: стоимость лучше каждой точки
каждого конкурента на каждом размере, за секунды, плюс сэкономленная машина при n=100.
Старый ILS остаётся доступным как `solveVrptw` и воспроизводит таблицу Solomon; на Solomon
SISR лучше него уже на бюджете по умолчанию (0.135% против 0.182% при совпадении машин,
~3x быстрее) и добирает число машин при увеличении бюджета — включая знаменитый rc101 с
14 машинами (2M итераций, 12 с) благодаря fleet-min разрушению, которое опустошает
наименьший маршрут при заданном `veh_penalty`. Один отрицательный результат, честно:
разрушение split-string («индукция запаса») из CVRP с окнами измеримо ХУЖЕ (Москва n=1000:
+0.8%), поэтому для VRPTW оно выключено по умолчанию.

---

## commiv против остальных

**Где он выигрывает**

- **Граница скорости и качества.** Близко к оптимуму за секунды, а не за минуты и часы,
  которые тратят эталонные эвристики. Для планировщика, которому приходится пересчитывать
  маршруты постоянно, важно именно это число.
- **Направленные реальные дорожные матрицы.** Асимметричная стоимость — первоклассный
  случай. FILO и HGS-CVRP, чемпионы симметричной скорости и качества, физически не умеют
  читать направленную матрицу. На реальных данных по Москве commiv обходит OR-Tools, LKH-3
  и VROOM и по стоимости, и по времени.
- **Ноль зависимостей, малый объём памяти.** Один модуль на Zig, и направленная CVRP на
  5000 узлов укладывается в 211 МБ.

**Где выигрывают остальные, и это стоит знать**

- **Абсолютная точность при огромных бюджетах.** LKH-3, HGS-CVRP и SISR (статья) достигают
  меньших разрывов (примерно 0.16–0.39% на Uchoa X), когда им дают намного больше времени.
  commiv меняет эту последнюю долю процента на большое преимущество в скорости. По точности на
  границе он не SOTA.
- **Огромные симметричные инстансы.** FILO решает симметричные CVRP с десятками тысяч узлов
  быстрее всего, что здесь есть. commiv нацелен на направленный режим масштаба маршрутизации
  (от сотен до нескольких тысяч).
- **Производственная зрелость.** OR-Tools и VROOM — обкатанные стеки с богатым набором
  ограничений (временные окна, pickup-and-delivery, навыки, перерывы) и годами эксплуатации.
  commiv — быстрое сфокусированное ядро, а не полноценная логистическая платформа.

---

## Проектные решения

### На чём остановились и почему

- **SISR (Slack Induction by String Removals) для большой и асимметричной CVRP — и VRPTW.**
  Разрушить несколько пространственно соседних строк, жадно вставить обратно со случайными
  «миганиями», принять по порогу или SA. Ставка: миллионы ходов `O(removed)` бьют тысячи
  ходов `O(n)`. Именно это вскрывает режимы большого n и направленности. Для временных окон
  тот же цикл работает со структурами префиксных/суффиксных запасов времени (Tws) на
  маршрут, так что «допустима ли эта вставка и сколько она стоит» — O(1) на кандидата; одно
  это изменение сократило московскую VRPTW n=1000 с 988 с (ILS) до 4 с при лучшей стоимости.
- **HGS (популяция плюс Prins Split DP) для средней CVRP, n примерно до 500.** Генетическая
  популяция гигантских туров с оптимальным разбиением по вместимости и обучением локальным
  поиском даёт лучшее качество на этом масштабе (числа 0.02–0.45% выше).
- **Поиск по недопустимым решениям со штрафом.** Разрешив локальному поиску пересекать
  недопустимые по вместимости области за штраф, удалось пробить жёсткий потолок качества около
  2%, который поиск только по допустимым решениям не мог.
- **Родная направленная ATSP для вырожденных матриц.** У инстансов rbg (кран-штабелёр) много
  дуг, связывающих минимум каждой строки, и 2n-преобразование Йонкера-Волгенанта платит за это
  дважды. Прямой направленный локальный поиск (Or-opt плюс направленный 2-opt плюс
  двойной мост) достигает оптимума быстрее, на n узлах вместо 2n.
- **Гранулярный локальный поиск с очередью «не смотреть».** Ограничение ходов пространственными
  соседями и реактивация только концов изменённых рёбер сделали локальный поиск при большом n
  в 2–4 раза быстрее при равном качестве.
- **Затравка по виду матрицы на месте.** Затравка гигантского тура CVRP читает матрицу
  стоимостей напрямую через шаговый вид, а не копирует подматрицу `n x n` и не строит
  2n-преобразование. Это снизило память при n=5000 примерно с 2 ГБ до 211 МБ, а решение — с
  412 с до 109 с, при идентичном качестве (затравка одноразовая, SISR её перестраивает).
- **Параллелизм — рычаг скорости, а не магия.** Затравки «лучшее из K» и рекомбинация EAX
  помогают точности при равном настенном времени на нескольких ядрах; детерминированный режим
  деления бюджета меняет немного качества на примерно 2.5x скорости.

### Что попробовали и отбросили, и почему

- **Статические дескрипторы ходов (SMD).** Наша очередь «не смотреть» в 4–5 раз быстрее при
  идентичном качестве; DLQ уже ловит ту локальность, которую покупает SMD. Тупик.
- **Двухуровневый двусвязный список тура.** После исправления отката, который срабатывал на
  заведомо обречённых перестроениях, перестроения туров упали примерно в 10 раз, а остаток
  стоил меньше 5% настенного времени. Сложность переписывания того не стоила.
- **Кооперативный и «лучшее из» параллелизм.** Высокая дисперсия и конкуренция за блокировки
  сделали его медленнее независимых островов. Убрано.
- **Декомпозиция для большого n.** Выигрыш от пересчёта подзадач на сошедшихся турах TSP не
  обобщился на никогда не сходящийся SISR. Просто SISR, запущенный дольше, доминировал.
- **Адаптивная переоценка кандидатов.** Даже идеальная переоценка оракулом смывается на полном
  бюджете ILS. Порядок кандидатов — рычаг одного спуска, а не точности.
- **Заморозка рёбер и голосование.** Заморозка даже чистого подмножества заведомо оптимальных
  рёбер теряет точность, потому что Lin-Kernighan вынужден ломать и перестраивать даже
  оптимальные рёбра по пути. Структурно.
- **Ранняя остановка по границе назначения.** Нижняя граница AP слишком рыхлая, чтобы
  заверить близость к оптимуму для CVRP с тугой вместимостью (19–52% на Москве). Как правило
  остановки здесь бесполезна.

---

## Ускорение на GPU (спроектировано, не реализовано)

Сегодня commiv работает только на CPU. Самый крупный нетронутый источник ускорения — это GPU.
Горячий цикл SISR оценивает миллионы независимых дельт ходов в секунду, это до неприличия
параллельная пакетная редукция, которая чисто ложится на GPU, с направленной матрицей,
размещённой в памяти устройства (100 МБ при n=5000 влезает в любую современную карту). Полное
техзадание (пакетное ядро дельт ходов, массивные острова «лучшее из K», CUDA FFI, матрица в
памяти устройства) — в [`gpu.md`](gpu.md). Оно не реализовано (в среде разработки нет GPU), но
это самый вероятный путь к ещё одному порядку величины при большом n.

---

## Воспроизведение бенчмарков

Стандартные инстансы поставляются в `vendor/` (TSPLIB, CVRPLIB Augerat и Uchoa X, ATSP, ACVRP,
Solomon и матрицы Moscow OSRM в `vendor/road/`). Матрица `moscow-5000` сжата, так что выполните
`gunzip vendor/road/moscow-5000.road.gz` перед использованием. Адаптеры конкурентов (OR-Tools,
LKH-3 с тёплым стартом, VROOM и нижняя граница назначения) плюс заметки по настройке — в
[`tools/competitors/`](tools/competitors/). Чтобы заново выкачать матрицу по Москве с
самостоятельно поднятого OSRM, используйте
[`tools/fetch_road_matrix.py`](tools/fetch_road_matrix.py).

Бенчмарки разрывов собирают собственный бинарник, который вы затем запускаете:

```sh
zig build cvrpbench  -Doptimize=ReleaseFast && ./zig-out/bin/commiv-cvrpbench   # CVRP против оптимумов
zig build acvrpbench -Doptimize=ReleaseFast && ./zig-out/bin/commiv-acvrpbench  # асимметричная CVRP против LKH-3
zig build atspbench  -Doptimize=ReleaseFast && ./zig-out/bin/commiv-atspbench   # ATSP против доказанных оптимумов
zig build vrptwbench -Doptimize=ReleaseFast && ./zig-out/bin/commiv-vrptwbench  # VRPTW против SINTEF BKS
zig build roadbench  -Doptimize=ReleaseFast && ./zig-out/bin/commiv-roadbench   # реальная направленная матрица по Москве
zig build bench      -Doptimize=ReleaseFast                                     # бенчмарк TSP (запускается)
```

## Лицензия

См. [`LICENSE`](LICENSE).
