# commiv

[![CI](https://github.com/grechman/Commiv/actions/workflows/ci.yml/badge.svg)](https://github.com/grechman/Commiv/actions/workflows/ci.yml)

**[English](#commiv) · [Русский](#commiv-rus)**

Near-optimal vehicle routes on real road networks, in seconds. Embeddable,
dependency-free, and built for directed (asymmetric) cost, where the trip from A to B
is not the same trip as B to A.

commiv solves the travelling-salesman and vehicle-routing families (TSP, ATSP, CVRP,
ACVRP, VRPTW, PDPTW) to within a fraction of a percent of optimal, and it reads directed
travel-time matrices natively: one-way streets, turn penalties, congestion. I built it
because the standard tools fall apart exactly there. FILO and HGS-CVRP, the fast solvers
everyone reaches for, are symmetric-only and cannot ingest a directed matrix at all.
LKH-3 can, but it is single-threaded, non-commercial, and struggles with explicit
directed matrices past n=1000 or so - which leaves few options when your matrix comes
from an actual road network.

You almost never need the last 0.3% of optimality that costs an hour of compute. You
need a near-optimal route now, and on real roads it has to respect direction.

Real Moscow OSRM data, n=1000 directed CVRP, same matrix for everyone:

| solver | cost | wall |
|---|---|---|
| commiv | **207,406** | **2 s** |
| VROOM 1.14 | 208,687 | 315 s |
| LKH-3 | 221,487 | 456 s |
| OR-Tools | 225,917 | 60 s |

Every number in this README reproduces with one command; see
[Reproducing the benchmarks](#reproducing-the-benchmarks).

```sh
zig build                                  # build the library
zig build test                             # run the unit tests
zig build example                          # run the embedded solver example
zig build serve -Doptimize=ReleaseFast     # REST API server (JSON over HTTP)
zig build lib   -Doptimize=ReleaseFast     # C ABI: libcommiv.{a,so} + commiv.h
```

- 0.02% off proven optima on standard CVRP, about 0.45% on the hard Uchoa X set, in
  seconds to a minute on a laptop.
- Callable from any language: a REST server, a C ABI, and a Python binding ship in this
  repo; you never have to write Zig.
- Zero dependencies, deterministic. One Zig module, no system libraries, no build-time
  downloads. The same seed produces the same routes.
- A 5000-node directed CVRP solves in 109 s using 211 MB, and 100 MB of that is the
  matrix itself.

---

## Integrate commiv into your codebase

In practice you start from your own stops, cost matrix, and per-stop demands, not from
a TSPLIB file. Zig is the engine underneath, but you pick the door that matches your
stack; none of them require writing Zig:

| Your stack | Door | Where |
|---|---|---|
| Any language | REST server: one static binary, JSON over HTTP | [`docs/rest.md`](docs/rest.md) |
| Python | Native binding (ctypes over the C ABI, numpy-friendly) | [`bindings/python/`](bindings/python/) |
| C, C++, Go, Rust, ... | C ABI: `libcommiv.{a,so}` + [`include/commiv.h`](include/commiv.h) | `zig build lib` |
| Zig | The module itself | this section |

### REST

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

The request is a directed cost matrix (row `a`, column `b` = cost of `a -> b`,
depot = node 0), demands, and a capacity. `/solve/vrptw` adds time windows,
`/solve/atsp` does pure directed ordering, `/solve/pdptw/dispatch` re-solves a
rolling-horizon plan around committed (locked) stops. Full schema and Python/JS/Go
client snippets in [`docs/rest.md`](docs/rest.md).

Container: `zig build serve -Dtarget=x86_64-linux-musl -Doptimize=ReleaseFast &&
docker build -t commiv-serve . && docker run --rm -p 8080:8080 commiv-serve` -
one static binary in a from-scratch image, ~10 MB.

### Python

```python
import commiv  # pip install commiv  (prebuilt wheels: linux x86_64, macOS; no toolchain)

sol = commiv.solve_cvrp(matrix, demand=[0, 4, 6, 5], capacity=10, seed=1)
print(sol.total_cost, sol.routes)  # 58 [[2, 1], [3]]
```

numpy matrices take a fast path; infeasible instances raise instead of returning
garbage. `solve_pdptw_dispatch` and `DispatchSession` cover rolling-horizon
re-solve around a committed plan. Details in
[`bindings/python/README.md`](bindings/python/README.md).

### Zig

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

Full working examples live in [`examples/`](examples/): `basic.zig` parses a TSPLIB instance with `parseTsplib`, while `roadbench.zig` and `cvrpbench.zig` read their CVRP and road instances from disk with their own parsers. `parseTsplib` reads TSPLIB symmetric TSP only - a coordinate section (`EUC_2D`, `CEIL_2D`, or `ATT`) or an `EXPLICIT` `FULL_MATRIX` - so CVRP, ACVRP, and ATSP instances do not load through it. Below is the minimal in-memory version on your own data.

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
- Every returned route respects `capacity`. An instance with no feasible packing returns
  an error.

---

## API reference

Every solver is allocator-first, takes an options struct, and returns a result you free with
`deinit()`. The curated set below is the whole public API (it mirrors
[`src/root.zig`](src/root.zig)); each solver has unit tests in its own source file.

**Parsing**
- `parseTsplib(allocator, text, ParseOptions) !Problem` parses TSPLIB symmetric TSP only - a
  coordinate section (`EUC_2D`, `CEIL_2D`, or `ATT`) or an `EXPLICIT` `FULL_MATRIX`. Pass a
  `ParseDiagnostic` in the options to capture line-level parse errors.

**Problem definition** (coordinate / TSPLIB path)
- `Problem`, built via `Problem.initCoords(...)` or `Problem.initFullMatrix(...)`, plus the
  `Coord` and `DistanceKind` types.

**Shared options and result**
- `SolveOptions` - `seed`, `budget` (`trials`, `max_passes`), candidate and search knobs.
- `SolveResult` - `{ tour, length, stats }`, the one type returned by `solve`, `solveAtsp*`,
  and `bruteForce`. `SolveStats` is the per-run telemetry; `CandidateMode` picks the
  candidate-graph metric.

**TSP (symmetric)**
- `solve(allocator, *Problem, SolveOptions) !SolveResult` - Lin-Kernighan + ILS.
- `solveWithStats(...)` - same as `solve`, also fills `SolveStats` telemetry.
- `solveParallel(allocator, *Problem, SolveOptions, ParallelOptions) !SolveResult` -
  independent islands with optional EAX recombination, or a deterministic split-budget speed
  mode. `ParallelOptions.threads == 0` resolves to the host CPU count, which changes the
  island seeding and therefore the result; pass an explicit non-zero `threads` for output
  reproducible across machines.

**ATSP (directed)** - row-major `n x n` matrix where `matrix[i*n + j]` is the cost of `i -> j`
- `solveAtsp(allocator, matrix, n, SolveOptions) !SolveResult` - 2n Jonker-Volgenant transform.
- `solveAtspNative(allocator, matrix, n, SolveOptions) !SolveResult` - direct directed search.
- `solveAtspParallel(allocator, matrix, n, SolveOptions, threads) !SolveResult` - `threads == 0`
  resolves to the host CPU count, changing the result; pass a non-zero `threads` for
  cross-machine reproducibility.

**Exact (tiny n)**
- `bruteForce(allocator, *Problem, ExactOptions) !SolveResult`.

**CVRP / ACVRP** - build a `CvrpInstance { n, matrix, demand, capacity }` with a
`(n+1) x (n+1)` directional matrix (depot = node 0); every solver returns
`CvrpResult { routes, total_cost }`
- `solveCvrp(allocator, inst, SolveOptions) !CvrpResult` - no-config default (runs SISR).
- `solveCvrpSisr(allocator, inst, SolveOptions, CvrpSisrParams)` - large / directed.
- `solveCvrpHgs(allocator, inst, SolveOptions, CvrpHgsParams, max_vehicles)` - n up to about 500.
- `solveCvrpFleet(allocator, inst, SolveOptions, CvrpFleetParams)` - fixed fleet cap.
- `solveCvrpSisrParallel(allocator, inst, SolveOptions, CvrpSisrParams, threads)` - `threads == 0`
  resolves to the host CPU count, changing the result; pass a non-zero `threads` for
  cross-machine reproducibility.
- `solveCvrpHgsParallel(allocator, inst, SolveOptions, CvrpHgsParams, max_vehicles, threads)` -
  same `threads == 0` caveat as above.
- `solveCvrpMulti(allocator, inst, SolveOptions, CvrpMultiParams)` - uncapped giant-tour ILS
  variant (legacy; SISR usually dominates it).
- `validateCvrp(inst, routes) ?u64` - independent feasibility check of a solution; returns
  the recomputed true cost, or null if any route is infeasible or a customer is missed.

**VRPTW** - build a `VrptwInstance { n, matrix, demand, capacity, ready, due, service }`;
returns `VrptwResult`
- `solveVrptwSisr(allocator, inst, SolveOptions, VrptwSisrParams)` - the default engine:
  SISR with time windows.
- `solveVrptwSisrParallel(allocator, inst, SolveOptions, VrptwSisrParams, threads)` -
  best-of-K chains; same `threads == 0` caveat as the CVRP variant.
- `solveVrptw(allocator, inst, SolveOptions, VrptwParams) !VrptwResult` - legacy giant-tour
  ILS (kept for reproducibility; SISR matches or beats it at equal wall).
- `solveVrptwHgs(allocator, inst, SolveOptions, VrptwHgsParams) !VrptwResult`.
- `validateVrptw(inst, routes) ?u64` - independent capacity + time-window feasibility check;
  returns the recomputed cost, or null if infeasible.

**PDPTW (pickup & delivery)** - build a `PdpInstance { n_pairs, matrix, capacity, pair_of,
is_pickup, demand_signed, ready, due, service }`; returns `PdpResult`
- `solvePdptwSisr(allocator, inst, PdpSisrParams)` - the engine: pair-atomic SISR, with an
  optional fleet cap + request bank (`max_vehicles`) for hierarchical vehicle minimization.
  On the Li & Lim 100-series at 10 s per instance, single thread, all four solvers fed the
  SAME integer matrix (`zig build pdptwbench -Doptimize=ReleaseFast` then
  `PB_FLEET=1 ./zig-out/bin/commiv-pdptwbench 2>&1`):

  | solver | complete | exact best-known | mean gap |
  |---|---|---|---|
  | commiv | 56/56 | 53/56 | 0.07% |
  | VROOM 1.14 | 50/56 | 33/56 | 0.61% |
  | LKH-3.0.14 | 37/56 | 26/56 | 0.42% |
  | OR-Tools 9.15 | 23/56 | 11/56 | 4.94% |

  Head-to-head (hierarchical objective, incomplete = loss): 21W/33T/2L vs VROOM,
  30W/26T/0L vs LKH-3, 45W/11T/0L vs OR-Tools. Harnesses: `tools/lkh_pdptw/`,
  `tools/vroom_pdptw.py`, `tools/ortools_pdptw.py`. PyVRP has no paired
  pickup-delivery support (their issue #331).

  Scale (Li & Lim 200-1000, all 296 instances with published BKS routes, equal wall
  30/60/90/120/180 s by size, single thread, same matrix; `PB_DIR=vendor/pdptw/<size>`):
  commiv serves every request on 296/296. VROOM capped at the record fleet completes
  only 64/296 (hierarchical 259W/21T/16L for commiv); VROOM in fleet-min mode
  (`VROOM_FLEETMIN=1`, per-vehicle fixed cost, 2x fleet available) completes everything
  but finds a strictly smaller fleet than commiv on 4/296 vs commiv's 197/296
  (hierarchical 251W/21T/24L for commiv). At the record fleet commiv's distance gap vs
  BKS is 0.2-2.3% by size; where it misses the record it is short by 1-4 vehicles
  (the records took hours to days of compute to set). All 16 capped-mode losses are
  same-fleet distance edges: 13 hairline (<=0.2%) on the easiest clustered family
  (lc1_x_5/6/7), 3 real (4-15%) on long-route lr2 200-series cells. Anytime profile:
  at one-sixth of the wall commiv already holds nearly all its fleet results and is
  within 0.2-1.3% of its own final distance.

  Opt-in levers on top (all measured, all off by default):
  - `PB_EJECT=1` (`eject`): GES-style squeeze fallback (Nagata & Kobayashi) in capped
    runs - a banked pair with no feasible insertion is inserted at the least-violating
    position and one resident pair is ejected to restore feasibility, steered by
    ejection counters. With this on, the 100-series reaches the record fleet on
    **56/56 at 10 s** (lc103 and lc109 cracked; the only remaining n=100 gaps are three
    distance cells: lc103 +2.1%, lc104 +0.5%, lc109 +2.2%). Also cracks lrc1_2_8's
    record at n=200. No effect on deep (5+ vehicle) misses at n=1000.
  - `PB_GRAN=2` (`gran_gaps`): dropoff-gap evaluation gated to kNN neighbourhoods on
    long routes during uncapped search (auto-off above 8 routes, under 24 nodes, or
    while a fleet cap is active) - 200-series fleet 2W/0L, distance 8W/0L vs baseline
    (lr2_2_3 16.75% -> exact BKS; record-fleet mean gap 1.92% -> 0.91%). Recommended
    n>=200 only (seed-marginal lr211 fleet loss at n=100). Combining with eject at
    n=200 churns (fleet 4W/4L) - pick one per regime.
  - `PB_THREADS=3`: parallel fleet-min waves (`solvePdptwSisrFleetMinParallel`) -
    saves 1-2 vehicles on four of the six hardest n=1000 cells at identical wall.
  - `PB_SWAP=N` (`swap_kick`): SWAP*-shaped inter-route pair exchange every N
    iterations - marginal (one 400-series hairline improved, lc104 unmoved).
  - `PB_P0` (`fleet_p0_pct`): uncapped-phase share of the fleet-min budget - front-
    loading (15%) measured a wash on hard n=1000 cells; default 40.
  The snapshot-rollback undo journal was profiled and measured dead (rollback+snapshot
  = 0.3-2.3% of wall; the insertion scan is 46-88%).

  **Money objective** (`time_penalty`, `PB_TIMEPEN`): charge each route's *duration*
  (driving + service + unavoidable waiting, at the departure-time-optimized schedule -
  the time-window algebra computes it natively) on top of distance and the per-vehicle
  cost. This turns the objective into real operating cost:
  `A·vehicles + B·distance + C·hours`, with the engine trading fuel for driver salary
  wherever waiting exists. Measured at 30 s on Li & Lim 200-series (`PB_TIMEPEN=1`,
  time unit priced like a distance unit): lr2_2_1 gives up an exact-best-known distance
  (+10%) to cut paid waiting 24% (4041 -> 3058) - total money **down 153 units**;
  lr1_2_10 cuts waiting 56% for +1.5% distance. Off (`time_penalty = 0`, the default)
  the engine is bit-identical to the historic vehicles-then-distance objective.
  Notably, VROOM cannot price waiting time at all (their open issue #1130).
- `solvePdptwSisrFleetMin(allocator, inst, params, total_time_ms)` - vehicle-count descent
  (uncapped run, then capped request-bank attempts, warm + cold, terminal polish).
- `solvePdptwSisrPinned(allocator, inst, params, total_time_ms, pin)` - enterprise
  pinned-fleet driver: best solution using at most `pin` vehicles, whole budget on that
  goal (uncapped warm-up, retrying descent, all remaining time polishing at the pin).
  On the hardest long-route cells this halves the distance gap vs a cold capped run
  (lr2_2_7 22.8% -> 5.0%) and solves cells where a cold capped run finds nothing.
- `solvePdptwSisrDispatch(allocator, inst, params, current, locked)` - rolling-horizon
  re-solve: `current[i]` is vehicle `i`'s present route, `locked[i]` how many of its
  leading stops are committed and must not move (a locked delivery's pickup must be
  locked too, in the same route). Unlocked stops and new/banked pairs are re-optimized
  around the locks. Exposed through all three doors as `solve_pdptw_dispatch` /
  `commiv_solve_pdptw_dispatch` / `POST /solve/pdptw/dispatch`; see
  [`docs/rest.md`](docs/rest.md) and [`bindings/python/`](bindings/python/).
- Heterogeneous fleet (v1): `PdpSisrParams.veh_types` - up to 8 vehicle types
  `{capacity, fixed_cost, count}` (count 0 = unlimited); every route is served by one
  type, its capacity bounds the route load, its fixed cost replaces `veh_penalty`.
  Doors: `commiv_solve_pdptw_typed` + `commiv_routes_type` (C),
  `solve_pdptw(vehicle_types=[(cap, fixed, count), ...])` -> `.types` (Python).
- Driver break (v1): `PdpSisrParams.brk = {dur, earliest, latest}` - one break per
  route, required of every route whose depart-at-0 schedule finishes after `earliest`;
  starts within `[earliest, latest]`, absorbs waiting first, counts into route duration
  (so the money objective prices it). Doors: `break_duration/earliest/latest` in
  `commiv_options` (C, PDPTW entries), `break_=(dur, earliest, latest)` (Python).
- `solvePdptw(allocator, inst, PdpParams)` - correctness baseline (pair insertion +
  pair relocate, brute-force-verified); use the SISR engine for real work.
- `validatePdptw(inst, routes) ?u64` - independent pairing + precedence + capacity-prefix +
  time-window feasibility check; returns the recomputed cost, or null if infeasible.
  `validatePdptwTyped` adds per-route type capacities; `validatePdptwWithBreak`
  brute-forces the break contract.

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

- Last-mile and courier routing on real road networks: feed a directed OSRM-style
  travel-time matrix, get capacity-feasible routes that respect one-way streets and turn
  costs.
- Classical TSP, CVRP, and VRPTW: near-optimal solutions far faster than exact methods.
- An embeddable core: a single dependency-free module to drop into a larger planner.
- Beyond logistics: hole-drilling on a printed circuit board, drone survey flyovers, NPC
  patrol routes in video games, a warehouse picker walking the racks.

---

## Benchmarks

Full campaign, one machine (Ryzen 5 2600X, 12 threads, WSL2), 2026-07-14: every family,
every real competitor, equal wall-clock anchored on commiv's own runtime, independent
validators wherever one exists. **All tables, per-instance data, and methodology:
[BENCHMARKS.md](BENCHMARKS.md).** The essentials:

| family | field | verdict (equal wall, best of 3 seeds) |
|---|---|---|
| TSP (TSPLIB, 16) | LKH-3 | parity: commiv 0.008% vs LKH 0.006% mean gap |
| ATSP (TSPLIB, 19) | LKH-3 | near-parity: 0.017% vs 0.001% |
| CVRP Augerat (12) | HGS-CVRP, PyVRP | all three at/near optimum; commiv 0.000% |
| CVRP Uchoa X (6) | HGS-CVRP, PyVRP | **commiv 0.320%** vs HGS 0.965%, PyVRP 2.640% |
| ACVRP (30) | LKH-3 field best | +0.010% vs LKH's own best-known values |
| Road CVRP (10 cities/sizes) | PyVRP, VROOM | commiv leads every cell but one (nyc-100 -0.15%); PyVRP +1.2-1.9% behind at n>=1000, VROOM +2-5% |
| Road VRPTW (9) | PyVRP, VROOM | commiv leads 8/9 (nyc-100 -0.01%); VROOM +1-9% behind |
| VRPTW Solomon/GH (18) | PyVRP | Solomon at BKS; GH R/RC fleet counts are the weak spot |
| PDPTW Li & Lim (352) | VROOM | commiv completes **352/352**; VROOM leaves shipments unassigned on 238/352 at the same wall |
| PDPTW money objective (352) | VROOM | cost = vehicles + distance + driver hours, equal wall: VROOM overspends **+11.6%**, from +2.0% at n=100 to +14.1% at n=1000 |

The one systematic weakness: Gehring-Homberger R/RC classes, where commiv runs 1-6
vehicles over best-known (route-min phase exists only in the PDPTW engine so far).
Everything else in that table is a lead or a tie.

One caveat on fleet counts: vehicles-first is academic scoring. When the objective is
money - a fixed cost per vehicle plus distance plus driver hours, waiting included -
an extra vehicle is often cheaper than the detours and paid waiting it removes. commiv
optimizes that objective natively (`time_penalty`); VROOM cannot price driver time at
all (their issue #1130), which is where the +11.6% money-bench gap comes from.


## Speed and memory (2026-08)

A profiling pass over the released engine. Every number below is a fixed-work
A/B - same instance, same seed, same iteration budget, and byte-identical solver
output before and after - so these are speed and memory measurements, not
quality trades. Ledger and reproduce commands: [BENCH.md](BENCH.md).

| path | what changed | measured |
|---|---|---|
| PDPTW insertion, ordinary objective | scalar time-window labels instead of two segment concatenations per candidate gap | **20.9% / 22.7% less time**, identical routes |
| PDPTW over REST | solver work moved off the per-request arena | peak RSS **-92.6%** solve, -67.0% dispatch, -68.2% VROOM compat |
| PDPTW rollback journal | packed snapshots instead of one allocation per touched route | peak RSS **-91.4%** |
| Parallel HGS / VRPTW workers | workers free their own workspaces | peak RSS **-85.2% / -80.2%** |
| Fleet-capped Split DP | route-count columns bounded by proof instead of by n | **4.86x**, RSS -71.8% |
| Large native ATSP | requested trial count honored; bounded top-k candidate build | about **37x** at `trials=1`, **2.12x** on the candidate build |
| Python bindings | contiguous NumPy input passed without copies | **66.2x** conversion, peak RSS -60.3% |

**The money objective is deliberately absent from that list.** The insertion win
comes from replacing the time-window algebra with scalar arrival labels, and the
money objective prices the merged route duration that those labels cannot
produce - so it keeps the exact Tws summary and the fast path stays off. Money
mode is bit-identical before and after this pass, which is also why the money
benchmark above scores the same search either way. An Lseg-only variant for money
mode measured mean -1.97% across four interleaved replicates and was rejected
under the same 2-3% margin every other lever answers to (`bench/EXPERIMENTS.md`, #13).

One behavior change to know about: a large ATSP (above about n=2900) whose matrix
has many arcs tying their row minimum used to multiply your requested trial count
by 100 behind your back. It now runs the trials you asked for - far faster, and a
shorter search unless you raise `trials` yourself.


## Where it wins, where it loses

Where it wins:

- Speed at near-optimal quality: seconds, not the minutes-to-hours the reference
  heuristics spend. For a planner that replans constantly this is the number that
  matters.
- Directed real-road matrices. FILO and HGS-CVRP physically cannot read one. On real
  Moscow data commiv beats OR-Tools, LKH-3, and VROOM on cost and time.
- Zero dependencies and a small footprint: one Zig module, a 5000-node directed CVRP
  in 211 MB.

Where the competition wins:

- Give LKH-3, HGS-CVRP, or the original SISR far more time than the equal-wall budgets
  in [BENCHMARKS.md](BENCHMARKS.md) and they reach lower gaps (0.16% to 0.39% on
  Uchoa X); PyVRP crosses commiv's numbers at roughly 10x the compute, later on bigger
  and more asymmetric instances. At the unlimited-budget frontier commiv is not
  state-of-the-art.
- On the vehicles-first Gehring-Homberger objective commiv matches the best-known fleet
  on 4 of 12 instances. The record holders run an ejection-pool route-minimization
  phase commiv doesn't have. (Priced in money instead of vehicle count, this mostly
  stops mattering - see the money bench in [BENCHMARKS.md](BENCHMARKS.md).)
- FILO solves symmetric CVRPs with tens of thousands of nodes faster than anything here.
- OR-Tools and VROOM have been in production for years and cover skills, multi-depot,
  driver shifts, the long tail of real constraints. commiv covers capacities, time
  windows, pickup-and-delivery, heterogeneous fleets, breaks, fleet pinning, and a
  money objective. That's the whole list.

---

## Design decisions

### What stuck

- **SISR (Slack Induction by String Removals) for large and asymmetric CVRP - and VRPTW.**
  Ruin a few spatially-adjacent strings, greedily re-insert with random blinks, accept
  under a threshold or SA. Millions of `O(removed)` moves beat thousands of
  `O(n)` ones. This is what cracks the large-n and directed regimes. For time windows the
  same loop runs with per-route prefix/suffix time-slack (Tws) structures, so "is this
  insertion feasible and what does it cost" is O(1) per candidate gap - that one change
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
- **Parallelism as a speed lever.** Best-of-K seeds and EAX recombination help
  accuracy at equal wall-clock on multiple cores; a deterministic split-budget mode trades a
  little quality for about 2.5x speed.

### What got tried and thrown out

- **Static Move Descriptors (SMD).** The don't-look-queue is 4x to 5x faster at identical
  quality; the DLQ already captures the locality SMD buys. Dead end.
- **Two-level doubly-linked tour list.** After fixing a fallback that was firing on provably
  doomed rebuilds, tour rebuilds dropped about 10x. Re-measured 2026-07: the entire remaining
  target (`applyEdges`, the O(n) retrace + rebuild per accepted move) is 2.5-2.8% of
  wall-clock at n=575-1577 and 5.6% at n=11849 - the rewrite's ceiling, before paying the
  per-query segment indirection that every `next`/`prev` read in the LK inner loop would
  eat, so the rewrite stays shelved.
- **Cooperative and best-of parallelism.** High variance and lock contention made it slower
  than independent islands, so it came out.
- **Decomposition for large n.** A subproblem-resolve win on converged TSP tours did not
  generalize to never-converging SISR. Plain SISR run longer dominated.
- **Adaptive candidate re-ranking.** Even a perfect oracle re-rank washes out at full ILS
  budget. Candidate order is a single-descent lever, not an accuracy lever.
- **Edge-freezing and voting.** Freezing even a pure subset of known-optimal edges loses
  accuracy, because Lin-Kernighan has to break and rebuild even optimal edges along the
  way; no amount of tuning fixes that.
- **Assignment-bound early stop.** The AP lower bound is too loose for capacity-tight CVRP
  to certify near-optimality (19% to 52% on Moscow), which kills it as a stopping rule.
- **Route-pool recombination across parallel SISR chains.** Adaptive-memory offspring
  (cheapest disjoint routes from every island, clash-stripped with a TW re-check, leftovers
  as singletons, short SISR polish) never beat plain best-of-K on Moscow VRPTW at n=100 or
  n=1000 - even with a perfect repair the offspring starts ~46% above the best island, and
  spending the polish wall on extra best-of-K iterations wins instead. The TSP recombination
  gain does not transfer: SISR's threshold schedule makes the seed nearly irrelevant (measured
  before - swapping the seed tour changes nothing), so there is no incumbent-trajectory
  memory for an offspring to inject. LK islands have exactly that memory, which is why EAX
  recombination pays there and not here.

---

## GPU acceleration (measured: not worth it)

Measured and closed on a GTX 1660 Ti (2026-07-13): the batched move-delta kernel runs
0.11x vs the 12-thread CPU at n=1001, and 1536 parallel on-device chains reach only
0.55x of the CPU's aggregate throughput - the engine's winning moves (sparse don't-look
scans, deep anneals, ejection chains) are exactly the work GPUs are bad at. Full
evidence and reopen conditions: [`tools/gpu-probe/GPU-REPORT.md`](tools/gpu-probe/GPU-REPORT.md);
original design spec kept in [`gpu.md`](gpu.md).


## Reproducing the benchmarks

Standard instances ship under `vendor/` (TSPLIB, CVRPLIB Augerat and Uchoa X, ATSP, ACVRP,
Solomon, Gehring-Homberger, Li & Lim PDPTW, and the road matrices under `vendor/road/` -
`gunzip vendor/road/moscow-5000.road.gz` first). Competitor adapters and setup notes are in
[`tools/competitors/`](tools/competitors/). Each bench builds its own binary:

```sh
zig build cvrpbench  -Doptimize=ReleaseFast && ./zig-out/bin/commiv-cvrpbench   # CVRP vs optima
zig build acvrpbench -Doptimize=ReleaseFast && ./zig-out/bin/commiv-acvrpbench  # asymmetric CVRP
zig build atspbench  -Doptimize=ReleaseFast && ./zig-out/bin/commiv-atspbench   # ATSP vs optima
zig build vrptwbench -Doptimize=ReleaseFast && ./zig-out/bin/commiv-vrptwbench  # VRPTW vs SINTEF BKS
zig build roadbench  -Doptimize=ReleaseFast && ./zig-out/bin/commiv-roadbench   # directed road CVRP
zig build twroadbench -Doptimize=ReleaseFast && ./zig-out/bin/commiv-twroadbench # directed road VRPTW
zig build pdptwbench -Doptimize=ReleaseFast && ./zig-out/bin/commiv-pdptwbench  # PDPTW vs Li & Lim BKS
zig build bench      -Doptimize=ReleaseFast                                     # TSP suite (runs)
```

Per-cell configs, walls, and the campaign driver that produced BENCHMARKS.md are
documented in [BENCHMARKS.md](BENCHMARKS.md) itself.


## License

See [`LICENSE`](LICENSE).

<br>

---
---

<br>

<a name="commiv-rus"></a>
# commiv (Русский)

**[English](#commiv) · [Русский](#commiv-rus)**

Почти оптимальные маршруты по реальной дорожной сети за секунды. Встраивается куда
угодно, зависимостей нет, целиком на Zig. Главная специализация - асимметричная
(направленная) стоимость: когда дорога из A в B и дорога обратно стоят по-разному.

commiv решает семейство задач коммивояжёра и маршрутизации транспорта (TSP, ATSP, CVRP,
ACVRP, VRPTW, PDPTW) с точностью до долей процента от оптимума и ест направленные
матрицы времени в пути как есть: односторонние улицы, повороты, пробки. Я написал его,
потому что стандартные инструменты именно на этом и сыпятся: FILO и HGS-CVRP
направленную матрицу не читают в принципе, а LKH-3 читает, но он однопоточный, с
некоммерческой лицензией и после n~1000 на явных направленных матрицах разваливается.
Для матриц из реальной дорожной сети выбор, по сути, пустой.

Реальные данные Moscow OSRM, направленная CVRP, n=1000, одна и та же матрица у всех:

| солвер | стоимость | время |
|---|---|---|
| commiv | **207 406** | **2 с** |
| VROOM 1.14 | 208 687 | 315 с |
| LKH-3 | 221 487 | 456 с |
| OR-Tools | 225 917 | 60 с |

Каждое число воспроизводится одной командой; см. [BENCHMARKS.md](BENCHMARKS.md).

```sh
zig build                                  # собрать библиотеку
zig build test                             # запустить модульные тесты
zig build example                          # запустить пример встроенного солвера
zig build serve -Doptimize=ReleaseFast     # REST API сервер (JSON поверх HTTP)
zig build lib   -Doptimize=ReleaseFast     # C ABI: libcommiv.{a,so} + commiv.h
```

- 0.02% от доказанных оптимумов на стандартной CVRP, около 0.45% на тяжёлом наборе
  Uchoa X, за секунды на ноутбуке.
- Вызывается из любого языка: REST-сервер, C ABI и питоновский биндинг лежат прямо в
  репозитории; писать на Zig не нужно.
- Ноль зависимостей, детерминизм: один модуль на Zig, без системных библиотек, без
  скачиваний при сборке. Один и тот же сид даёт одни и те же маршруты.
- Экономно по памяти: направленная CVRP на 5000 узлов решается за 109 с в 211 МБ,
  из которых 100 МБ - сама матрица.

---

## Работа с commiv

Движок написан на Zig, но подключаться можно как удобно, и ни один из вариантов не
требует писать на Zig:

| Ваш стек | Точка входа | Где |
|---|---|---|
| Любой язык | REST-сервер: один статический бинарник, JSON поверх HTTP | [`docs/rest.md`](docs/rest.md) |
| Python | Нативный биндинг (ctypes поверх C ABI, дружит с numpy) | [`bindings/python/`](bindings/python/) |
| C, C++, Go, Rust, ... | C ABI: `libcommiv.{a,so}` + [`include/commiv.h`](include/commiv.h) | `zig build lib` |
| Zig | Сам модуль | этот раздел |

### REST

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

Запрос - это направленная матрица стоимостей (строка `a`, столбец `b` = стоимость
`a -> b`, депо = узел 0), спрос и вместимость. `/solve/vrptw` добавляет временные окна,
`/solve/atsp` - чистый порядок объезда без вместимости, `/solve/pdptw/dispatch` пересчитывает
план на скользящем горизонте вокруг уже зафиксированных остановок. Полная схема и примеры
клиентов на Python/JS/Go - в [`docs/rest.md`](docs/rest.md).

### Python

```python
import commiv  # pip install commiv (готовые wheels: linux x86_64, macOS; без тулчейна)

sol = commiv.solve_cvrp(matrix, demand=[0, 4, 6, 5], capacity=10, seed=1)
print(sol.total_cost, sol.routes)  # 58 [[2, 1], [3]]
```

numpy-матрицы идут по быстрому пути; на недопустимой задаче библиотека кидает
исключение, а не возвращает мусор. `solve_pdptw_dispatch` и `DispatchSession` покрывают пересчёт на
скользящем горизонте вокруг уже зафиксированного плана. Подробности - в
[`bindings/python/README.md`](bindings/python/README.md).

### Zig

Дальше - полное руководство по встраиванию на Zig.

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

Полные рабочие примеры лежат в каталоге [`examples/`](examples/): `basic.zig` разбирает инстанс TSPLIB через `parseTsplib`, а `roadbench.zig` и `cvrpbench.zig` читают свои инстансы CVRP и дорожных матриц с диска собственными парсерами. `parseTsplib` читает только симметричную TSP в формате TSPLIB - секцию координат (`EUC_2D`, `CEIL_2D` или `ATT`) либо `EXPLICIT` `FULL_MATRIX`, - поэтому инстансы CVRP, ACVRP и ATSP через него не загружаются. Ниже - минимальный пример прямо в памяти, на своих данных.

Вы передаёте матрицу стоимостей `(n+1) x (n+1)` построчно, row-major (узел 0 - это депо,
клиенты - `1..n`), массив `demand` и вместимость `capacity`. Матрица направленная:
`matrix[a*(n+1) + b]` - это стоимость пути из `a` в `b`, так что реальная асимметричная
дорожная стоимость подставляется как есть.

```zig
const std = @import("std");
const commiv = @import("commiv");

pub fn main() !void {
    const allocator = std.heap.page_allocator; // swap in your own (gpa, arena, ...)

    // 3 клиента + депо. Направленные стоимости (a -> b), построчно, депо = индекс 0.
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

    // SISR - рабочая лошадка по умолчанию: лучше всего для больших и/или направленных задач.
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

- `result.total_cost` - суммарная стоимость маршрутов по вашей матрице, тип `u64`.
- `result.routes` - по одному срезу на машину. Каждый срез перечисляет индексы клиентов в
  порядке посещения, депо подразумевается на обоих концах.

#### 4. Выбрать солвер под размер задачи

| Точка входа | Когда использовать |
|---|---|
| `solveCvrpSisr` | Большие и/или направленные задачи. Это случай дорожной сети. |
| `solveCvrpHgs` | Средняя CVRP, n примерно до 500, когда хочется дожать последние доли процента. |
| `solveCvrpSisrParallel` / `solveCvrpHgsParallel` | То же самое, но по нескольким ядрам. |
| `solveCvrp` | Точка входа по умолчанию без настройки. Запускает SISR. |

#### 5. Обычные TSP и ATSP

Если вместимости нет и нужен только порядок объезда - это TSP/ATSP. Вызов выглядит так
же; направленная матрица уходит в ATSP.

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

- `SolveOptions.seed` - сид ГПСЧ. У однопоточных солверов один и тот же сид даёт
  побайтово одинаковый результат, прогоны полностью воспроизводимы. У вариантов
  `*Parallel` результат зависит ещё и от числа потоков: `threads = 0` (по умолчанию)
  означает «по числу ядер машины», а от числа потоков зависят количество островов и их
  посевы, так что на машинах с разным числом ядер тот же сид даст разные маршруты.
  Нужна воспроизводимость между машинами - фиксируйте и сид, и явное ненулевое
  `threads` (`ParallelOptions.threads` для `solveParallel`).
- `SolveOptions.budget.trials` и `.max_passes` задают объём поиска: больше - ближе к
  оптимуму и дольше. Бюджет считается в итерациях, а не в секундах, так что под свой
  лимит времени его придётся подобрать эмпирически.
- Каждый возвращённый маршрут соблюдает `capacity`. Задача без допустимой упаковки вернёт
  ошибку.

---

## Документация API

Каждый солвер сначала принимает аллокатор, затем структуру опций и возвращает результат, который
вы освобождаете через `deinit()`. Перечисленный ниже набор - это весь публичный API (он
повторяет [`src/root.zig`](src/root.zig)); у каждого солвера есть модульные тесты в его файле.

**Разбор**
- `parseTsplib(allocator, text, ParseOptions) !Problem` разбирает текст симметричной TSP в
  формате TSPLIB - секцию координат (`EUC_2D`, `CEIL_2D` или `ATT`) либо `EXPLICIT` `FULL_MATRIX`.
  Передайте `ParseDiagnostic` в опциях, чтобы получить ошибки разбора с номерами строк.

**Определение задачи** (путь координат / TSPLIB)
- `Problem`, создаётся через `Problem.initCoords(...)` или `Problem.initFullMatrix(...)`, плюс
  типы `Coord` и `DistanceKind`.

**Общие опции и результат**
- `SolveOptions` - `seed`, `budget` (`trials`, `max_passes`), настройки кандидатов и поиска.
- `SolveResult` - `{ tour, length, stats }`, единственный тип, возвращаемый `solve`,
  `solveAtsp*` и `bruteForce`. `SolveStats` - телеметрия прогона; `CandidateMode` выбирает
  метрику графа кандидатов.

**TSP (симметричная)**
- `solve(allocator, *Problem, SolveOptions) !SolveResult` - Lin-Kernighan + ILS.
- `solveWithStats(...)` - то же, что `solve`, дополнительно заполняет телеметрию `SolveStats`.
- `solveParallel(allocator, *Problem, SolveOptions, ParallelOptions) !SolveResult` -
  независимые острова с опциональной рекомбинацией EAX или детерминированный режим деления
  бюджета ради скорости. `ParallelOptions.threads == 0` означает «по числу ядер», а от
  этого зависят посевы островов и результат; для воспроизводимости между машинами
  передавайте явное ненулевое `threads`.

**ATSP (направленная)** - матрица `n x n` построчно (row-major), `matrix[i*n + j]` = стоимость `i -> j`
- `solveAtsp(allocator, matrix, n, SolveOptions) !SolveResult` - 2n-преобразование Йонкера-Волгенанта.
- `solveAtspNative(allocator, matrix, n, SolveOptions) !SolveResult` - прямой направленный поиск.
- `solveAtspParallel(allocator, matrix, n, SolveOptions, threads) !SolveResult` - `threads == 0`
  означает «по числу ядер», что меняет результат; для воспроизводимости между машинами
  передавайте ненулевое `threads`.

**Точное решение (совсем маленькие n)**
- `bruteForce(allocator, *Problem, ExactOptions) !SolveResult`.

**CVRP / ACVRP** - соберите `CvrpInstance { n, matrix, demand, capacity }` с направленной
матрицей `(n+1) x (n+1)` (депо = узел 0); все солверы возвращают `CvrpResult { routes, total_cost }`
- `solveCvrp(allocator, inst, SolveOptions) !CvrpResult` - точка входа по умолчанию (запускает SISR).
- `solveCvrpSisr(allocator, inst, SolveOptions, CvrpSisrParams)` - большие / направленные.
- `solveCvrpHgs(allocator, inst, SolveOptions, CvrpHgsParams, max_vehicles)` - n примерно до 500.
- `solveCvrpFleet(allocator, inst, SolveOptions, CvrpFleetParams)` - фиксированный парк.
- `solveCvrpSisrParallel(allocator, inst, SolveOptions, CvrpSisrParams, threads)` - `threads == 0`
  означает «по числу ядер», что меняет результат; для воспроизводимости между машинами
  передавайте ненулевое `threads`.
- `solveCvrpHgsParallel(allocator, inst, SolveOptions, CvrpHgsParams, max_vehicles, threads)` -
  та же оговорка про `threads == 0`, что и выше.
- `solveCvrpMulti(allocator, inst, SolveOptions, CvrpMultiParams)` - вариант ILS по гигантскому
  туру без ограничения парка (устаревший; SISR обычно его превосходит).
- `validateCvrp(inst, routes) ?u64` - независимая проверка допустимости решения; возвращает
  пересчитанную стоимость либо null, если маршрут недопустим или клиент пропущен.

**VRPTW** - соберите `VrptwInstance { n, matrix, demand, capacity, ready, due, service }`;
возвращает `VrptwResult`
- `solveVrptwSisr(allocator, inst, SolveOptions, VrptwSisrParams)` - движок по умолчанию:
  SISR с временными окнами.
- `solveVrptwSisrParallel(allocator, inst, SolveOptions, VrptwSisrParams, threads)` -
  лучшее из K цепочек; та же оговорка про `threads == 0`, что и у CVRP-варианта.
- `solveVrptw(allocator, inst, SolveOptions, VrptwParams) !VrptwResult` - старый ILS по
  гигантскому туру (оставлен для воспроизводимости; SISR равен или лучше при равном времени).
- `solveVrptwHgs(allocator, inst, SolveOptions, VrptwHgsParams) !VrptwResult`.
- `validateVrptw(inst, routes) ?u64` - независимая проверка вместимости и временных окон;
  возвращает пересчитанную стоимость либо null, если решение недопустимо.

**PDPTW (заборы и доставки с временными окнами)** - соберите `PdpInstance { n_pairs,
matrix, capacity, pair_of, is_pickup, demand_signed, ready, due, service }` с матрицей
`dim x dim`, где `dim = 2*n_pairs+1` (депо = узел 0); возвращает `PdpResult`
- `solvePdptwSisr(allocator, inst, PdpSisrParams)` - движок: попарно-атомарный SISR.
  Каждая заявка - узел забора и узел доставки на одном маршруте, забор раньше доставки,
  вместимость соблюдается по всему маршруту. **Денежная целевая функция**
  (`PdpSisrParams.time_penalty > 0`) добавляет к расстоянию стоимость *длительности*
  маршрута (в пути + обслуживание + вынужденное ожидание), так что движок меняет топливо
  на часы водителя там, где есть ожидание; `time_penalty = 0` - чистое расстояние,
  побайтово совпадает со старой целевой функцией. `veh_penalty` смещает к меньшему
  числу машин; `max_vehicles` ограничивает парк.
- `solvePdptwSisrFleetMin(allocator, inst, params, total_time_ms)` - иерархическая
  минимизация числа машин (сначала парк, затем расстояние); нужен бюджет по времени.
- `solvePdptwSisrPinned(allocator, inst, params, total_time_ms, pin)` - режим
  фиксированного парка: лучшее решение ровно на `pin` машинах, весь бюджет на эту цель.
- `solvePdptwSisrDispatch(allocator, inst, params, current, locked)` - пересчёт на
  скользящем горизонте: `current[i]` - текущий маршрут машины `i`, `locked[i]` - сколько
  первых остановок уже зафиксировано и не должно сдвинуться (забор зафиксированной доставки
  тоже должен быть зафиксирован, на том же маршруте). Незафиксированные остановки и новые
  заявки переоптимизируются вокруг фиксаций. Доступен через все три интерфейса:
  `solve_pdptw_dispatch` / `commiv_solve_pdptw_dispatch` / `POST /solve/pdptw/dispatch`
  ([`docs/rest.md`](docs/rest.md), [`bindings/python/`](bindings/python/)).
- `solvePdptw(allocator, inst, PdpParams)` - эталон корректности (для реальной работы
  используйте SISR-движок).
- `validatePdptw(inst, routes) ?u64` - независимая проверка парности, предшествования,
  префикса вместимости и временных окон; возвращает пересчитанную стоимость либо null.

**Анализ асимметрии**
- `conservativeness(allocator, matrix, dim) !Conservativeness` выполняет разложение
  Гельмгольца-Ходжа направленной матрицы. Оно показывает, какая часть асимметрии структурна
  (односторонние улицы и повороты, которые меняют оптимальный маршрут), а какая - градиент
  (пробки, которые можно спокойно игнорировать). Прогоните через него свою матрицу - и
  станет ясно, нужна ли вам вообще направленная маршрутизация.

Всё остальное (`commiv.internal.*`, внутренние модули) в API не входит и может меняться
от версии к версии.

---

## Сценарии применения

- Последняя миля и курьерская маршрутизация по реальным дорогам: подайте направленную
  матрицу времени в пути в стиле OSRM и получите допустимые по вместимости маршруты,
  которые учитывают односторонние улицы и стоимость поворотов.
- Классические TSP, CVRP и VRPTW: близкие к оптимальным решения намного быстрее точных
  методов.
- Встраиваемое ядро: один модуль без зависимостей, который встраивается в ваш
  планировщик.
- Не только логистика: сверловка отверстий на печатной плате, облёт точек съёмки дроном,
  маршруты патрулей NPC в играх, обход ячеек склада комплектовщиком.

---

## Бенчмарки

Полная кампания на одной машине (Ryzen 5 2600X, 12 потоков, WSL2), 2026-07-14: все
семейства, все реальные конкуренты, равное время на инстанс (по замеренному времени
commiv), независимые валидаторы везде, где они есть. **Все таблицы и методика:
[BENCHMARKS.md](BENCHMARKS.md).** Главное:

| семейство | конкуренты | вердикт (равное время, лучший из 3 сидов) |
|---|---|---|
| TSP (TSPLIB, 16) | LKH-3 | паритет: commiv 0.008% против 0.006% |
| ATSP (19) | LKH-3 | почти паритет: 0.017% против 0.001% |
| CVRP Augerat (12) | HGS-CVRP, PyVRP | все около оптимума; commiv 0.000% |
| CVRP Uchoa X (6) | HGS-CVRP, PyVRP | **commiv 0.320%** против HGS 0.965% и PyVRP 2.640% |
| ACVRP (30) | LKH-3 (лучшие известные) | +0.010% к лучшим значениям LKH |
| Дорожный CVRP (10) | PyVRP, VROOM | commiv впереди во всех ячейках кроме одной (nyc-100 -0.15%) |
| Дорожный VRPTW (9) | PyVRP, VROOM | commiv впереди в 8/9; VROOM отстаёт на 1-9% |
| VRPTW Solomon/GH (18) | PyVRP | Solomon на уровне BKS; парк на GH R/RC - слабое место |
| PDPTW Li & Lim (352) | VROOM | commiv развозит всё в **352/352**; VROOM оставляет заявки неназначенными в 238/352 |
| PDPTW деньги (352) | VROOM | цена = машины + километры + часы водителя, при равном времени VROOM переплачивает **+11.6%**: от +2.0% при n=100 до +14.1% при n=1000 |

Единственная системная слабость - классы R/RC у Gehring-Homberger, где commiv берёт
на 1-6 машин больше, чем в лучших известных решениях (фаза минимизации парка пока
есть только в PDPTW-движке). Всё остальное в таблице - лидерство или ничья.

Важная оговорка про число машин: «сначала машины любой ценой» - это академический
зачёт. Как только целевая функция - деньги (фиксированная цена машины + километры +
часы водителя, включая простой), лишняя машина часто оказывается дешевле, чем крюки и
оплаченное ожидание, которыми её убирают. commiv оптимизирует эту цель напрямую
(`time_penalty`); VROOM цену времени водителя выразить не может вообще (их issue
#1130) - отсюда и +11.6% в денежном бенче.


## Скорость и память (2026-08)

Проход по производительности уже выпущенного движка. Каждое число ниже - A/B при
фиксированной работе: тот же инстанс, тот же сид, тот же бюджет итераций и
побайтово совпадающий результат до и после. Это замеры скорости и памяти, а не
размен качества. Журнал и команды воспроизведения: [BENCH.md](BENCH.md).

| путь | что изменилось | замер |
|---|---|---|
| Вставка пары в PDPTW, обычная цель | скалярные метки времён вместо двух склеек сегментов на каждый кандидат | **на 20.9% / 22.7% меньше времени**, маршруты те же |
| PDPTW через REST | работа солвера ушла с арены запроса | пик RSS **-92.6%** solve, -67.0% dispatch, -68.2% VROOM-совместимость |
| Журнал отката PDPTW | упакованные снимки вместо аллокации на каждый тронутый маршрут | пик RSS **-91.4%** |
| Параллельные воркеры HGS / VRPTW | воркеры освобождают свою память сами | пик RSS **-85.2% / -80.2%** |
| Split DP с ограниченным парком | число колонок ограничено доказуемой границей, а не n | **4.86x**, RSS -71.8% |
| Большой нативный ATSP | запрошенное число попыток соблюдается; отбор кандидатов через ограниченный top-k | около **37x** при `trials=1`, **2.12x** на построении кандидатов |
| Python-биндинги | непрерывный массив NumPy передаётся без копий | **66.2x** на конвертации, пик RSS -60.3% |

**Денежной цели в этом списке нет, и это осознанно.** Выигрыш на вставке взялся
из замены оконной алгебры скалярными метками прибытия, а денежная цель считает
именно суммарную длительность маршрута, которую такие метки не дают - поэтому там
остаётся полная сводка Tws, а быстрый путь выключен. Денежный режим побайтово
одинаков до и после этого прохода; поэтому денежный бенчмарк выше меряет один и
тот же поиск в обоих случаях. Вариант только с заменой Lseg дал в среднем -1.97%
на четырёх чередующихся повторах и был отклонён по тому же порогу 2-3%, по
которому судятся все остальные рычаги (`bench/EXPERIMENTS.md`, #13).

Одно изменение поведения, о котором стоит знать: большой ATSP (примерно от n=2900)
на матрице, где много дуг делят минимум строки, раньше молча умножал запрошенное
число попыток на 100. Теперь он выполняет ровно столько попыток, сколько попросили:
намного быстрее и с более коротким поиском, если не поднять `trials` вручную.


## commiv против остальных

Где он выигрывает:

- Скорость при почти оптимальном качестве: секунды, а не минуты и часы эталонных
  эвристик. Для планировщика, который постоянно пересчитывает маршруты, важно именно
  это.
- Направленные дорожные матрицы. FILO и HGS-CVRP такую физически не прочитают. На
  реальных данных по Москве commiv обходит OR-Tools, LKH-3 и VROOM и по стоимости, и
  по времени.
- Ноль зависимостей и малый объём памяти: один модуль на Zig, направленная CVRP на
  5000 узлов в 211 МБ.

Где выигрывают остальные:

- Дайте LKH-3, HGS-CVRP или оригинальному SISR в разы больше времени, чем в
  [BENCHMARKS.md](BENCHMARKS.md), и они дожмут разрыв до 0.16-0.39% на Uchoa X.
  На безлимитном бюджете commiv не SOTA.
- На иерархической цели Gehring-Homberger (сначала машины, потом расстояние) commiv
  достигает лучшего известного парка на 4 из 12 инстансов. Рекордсмены гоняют фазу
  выталкивания маршрутов (ejection pool), которой здесь нет. (В деньгах, а не в числе
  машин, эта слабость почти не играет - см. денежный бенч в
  [BENCHMARKS.md](BENCHMARKS.md).)
- FILO решает симметричные CVRP с десятками тысяч узлов быстрее всего, что здесь есть.
- OR-Tools и VROOM годами в проде и покрывают навыки, мульти-депо, смены водителей -
  весь длинный хвост реальных ограничений. commiv покрывает вместимости, временные
  окна, pickup-and-delivery, гетерогенный парк, перерывы, фиксацию парка и денежную
  целевую функцию. На этом всё.

---

## Проектные решения

### На чём остановились и почему

- **SISR (Slack Induction by String Removals) для большой и асимметричной CVRP - и VRPTW.**
  Выдернуть несколько соседних в пространстве отрезков маршрутов (strings), жадно
  вставить обратно со случайными пропусками (blinks), принимать по порогу или отжигом.
  Миллионы ходов за `O(removed)` бьют тысячи ходов за `O(n)` - именно это вытягивает
  большие n и направленные матрицы. Для временных окон тот же цикл крутится на
  префиксных/суффиксных запасах времени (Tws) по каждому маршруту, так что «влезет ли
  сюда вставка и сколько она стоит» считается за O(1); одно это изменение сократило
  московскую VRPTW n=1000 с 988 с (ILS) до 4 с, причём с лучшей стоимостью.
- **HGS (популяция плюс Prins Split DP) для средней CVRP, n примерно до 500.** Генетическая
  популяция гигантских туров с оптимальной нарезкой по вместимости и доводкой локальным
  поиском даёт лучшее качество на этом масштабе (те самые 0.02-0.45% выше).
- **Поиск по недопустимым решениям со штрафом.** Разрешив локальному поиску заходить в
  недопустимые по вместимости зоны за штраф, удалось пробить потолок качества ~2%, в
  который упирался поиск только по допустимым решениям.
- **Родной направленный ATSP для вырожденных матриц.** У инстансов rbg (кран-штабелёр)
  минимум в каждой строке матрицы достигается сразу многими дугами, и 2n-преобразование
  Йонкера-Волгенанта платит за это дважды. Прямой направленный локальный поиск (Or-opt,
  направленный 2-opt, double bridge) доходит до оптимума быстрее, на n узлах вместо 2n.
- **Гранулярный локальный поиск с don't-look-очередью.** Ходы только по пространственным
  соседям, будятся только концы изменённых рёбер - локальный поиск при большом n стал
  в 2-4 раза быстрее при том же качестве.
- **Стартовый тур читает матрицу без копирования.** Стартовый гигантский тур CVRP читает
  матрицу стоимостей напрямую, через strided-представление, а не копирует подматрицу
  `n x n` и не строит 2n-преобразование. Это снизило память при n=5000 примерно с 2 ГБ до
  211 МБ, а решение - с 412 с до 109 с, при идентичном качестве (стартовый тур одноразовый,
  SISR его всё равно перестраивает).
- **Параллелизм как рычаг скорости.** Best-of-K запуски и рекомбинация EAX помогают
  точности при равном времени на нескольких ядрах; детерминированный режим деления
  бюджета отдаёт немного качества за ~2.5x скорости.

### Что попробовали и выкинули

- **Статические дескрипторы ходов (SMD).** Наша don't-look-очередь в 4-5 раз быстрее при
  том же качестве: она уже ловит ту локальность, ради которой SMD и существует. Тупик.
- **Двухуровневый двусвязный список тура.** После починки отката, который срабатывал на
  заведомо обречённых перестроениях, перестроений тура стало примерно в 10 раз меньше, а
  остаток стоил меньше 5% общего времени. Переписывать структуру тура ради этого не стоит.
- **Кооперативный и «лучшее из» параллелизм.** Высокая дисперсия и конкуренция за блокировки
  сделали его медленнее независимых островов, так что его убрали.
- **Декомпозиция для большого n.** Выигрыш от пересчёта подзадач на сошедшихся турах TSP
  не перенёсся на SISR, который не сходится никогда. Просто дать SISR больше времени
  оказалось лучше.
- **Адаптивная переранжировка кандидатов.** Даже идеальный порядок от оракула на полном
  бюджете ILS ничего не меняет: порядок кандидатов влияет на один спуск, а не на итоговую
  точность.
- **Заморозка рёбер и голосование.** Даже если замораживать только рёбра, про которые
  точно известно, что они оптимальны, точность падает: Lin-Kernighan по дороге вынужден
  ломать и перестраивать в том числе оптимальные рёбра. Никакой настройкой это не лечится.
- **Ранняя остановка по границе назначения.** Нижняя граница AP слишком слабая, чтобы
  подтвердить близость к оптимуму, когда вместимость забита впритык (19-52% на Москве),
  так что как критерий остановки она бесполезна.

---

## Ускорение на GPU (замерено: не окупается)

Замерено и закрыто на GTX 1660 Ti (2026-07-13): ядро пакетной оценки ходов - 0.11x от
12 потоков CPU при n=1001; 1536 параллельных цепочек на GPU - лишь 0.55x суммарной
пропускной способности CPU. Сильные стороны движка (разреженный поиск, глубокий отжиг,
цепочки выталкивания) - ровно та работа, которую GPU делает плохо. Полные данные и
условия, при которых к теме стоит вернуться:
[`tools/gpu-probe/GPU-REPORT.md`](tools/gpu-probe/GPU-REPORT.md).


## Воспроизведение бенчмарков

Инстансы лежат в `vendor/` (TSPLIB, CVRPLIB, ATSP, ACVRP, Solomon, Gehring-Homberger,
Li & Lim, дорожные матрицы в `vendor/road/`). Адаптеры конкурентов - в
[`tools/competitors/`](tools/competitors/). Команды сборки бенчей - в английской части
выше; конфигурация каждой ячейки задокументирована в [BENCHMARKS.md](BENCHMARKS.md).


## Лицензия

См. [`LICENSE`](LICENSE).
