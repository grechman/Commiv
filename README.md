# commiv

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
zig build test                             # run the 70 unit tests
zig build example                          # run the embedded solver example
```

- **Near-optimal, fast.** 0.02% off proven optima on standard CVRP, about 0.45% on the
  hard Uchoa X set, in seconds to a minute on a laptop.
- **Asymmetric-native.** Directed travel-time matrices are first-class, not bolted on. On
  real Moscow OSRM data commiv beats OR-Tools, LKH-3, and VROOM on both cost and wall-clock.
- **Zero dependencies, deterministic.** One Zig module, no system libraries, no build-time
  downloads. The same seed produces the same routes.
- **Lean.** A 5000-node directed CVRP solves in 109 s using 211 MB, and 100 MB of that is
  the matrix itself.

---

## Integrate commiv into your codebase

The real entry point is not a TSPLIB file. It is your own stops, a cost matrix, and
per-stop demands. Here is the whole integration, start to finish.

### 1. Add the dependency

```sh
zig fetch --save "git+https://github.com/grechman/Path-finding-LKH-Zig-version"
```

That saves the package under the name `commiv`. Wire the module into your `build.zig`:

```zig
const commiv = b.dependency("commiv", .{
    .target = target,
    .optimize = optimize,
});
exe.root_module.addImport("commiv", commiv.module("commiv"));
```

### 2. Solve a vehicle-routing problem from your own data

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

### 3. Read the result

`CvrpResult` owns its memory, so call `deinit()` when you are done.

- `result.total_cost` is the total routed cost on your matrix, as a `u64`.
- `result.routes` is one slice per vehicle. Each slice lists the customer indices in visit
  order, with the depot implied at both ends.

### 4. Pick the solver for your size

| Entry point | Use it when |
|---|---|
| `solveCvrpSisr` | Large and/or directed instances. This is the road-network case. |
| `solveCvrpHgs` | Mid-size CVRP, n up to about 500, where you want the last bit of quality. |
| `solveCvrpSisrParallel` / `solveCvrpHgsParallel` | The same, spread across cores. |
| `solveCvrp` | The no-config default. Runs SISR with default params. |

### 5. Plain TSP and ATSP

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

### Knobs that matter

- `SolveOptions.seed` is the RNG seed. For the single-threaded solvers the same seed gives
  byte-identical output, so runs are reproducible. The `*Parallel` variants also depend on
  the thread count, so pin both the seed and the thread count if you need to reproduce a
  parallel run exactly.
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
- `parseTsplib(allocator, text, ParseOptions) !Problem` parses TSPLIB / CVRPLIB text. Pass a
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
- `solveParallel(allocator, *Problem, SolveOptions, ParallelOptions) !SolveResult` —
  independent islands with optional EAX recombination, or a deterministic split-budget speed
  mode.

**ATSP (directed)** — row-major `n x n` matrix where `matrix[i*n + j]` is the cost of `i → j`
- `solveAtsp(allocator, matrix, n, SolveOptions) !SolveResult` — 2n Jonker-Volgenant transform.
- `solveAtspNative(allocator, matrix, n, SolveOptions) !SolveResult` — direct directed search.
- `solveAtspParallel(allocator, matrix, n, SolveOptions, threads) !SolveResult`.

**Exact (tiny n)**
- `bruteForce(allocator, *Problem, ExactOptions) !SolveResult`.

**CVRP / ACVRP** — build a `CvrpInstance { n, matrix, demand, capacity }` with a
`(n+1) x (n+1)` directional matrix (depot = node 0); every solver returns
`CvrpResult { routes, total_cost }`
- `solveCvrp(allocator, inst, SolveOptions) !CvrpResult` — no-config default (runs SISR).
- `solveCvrpSisr(allocator, inst, SolveOptions, CvrpSisrParams)` — large / directed.
- `solveCvrpHgs(allocator, inst, SolveOptions, CvrpHgsParams, max_vehicles)` — n ≲ 500.
- `solveCvrpFleet(allocator, inst, SolveOptions, rounds, restarts, max_vehicles)` — fixed fleet cap.
- `solveCvrpSisrParallel(allocator, inst, SolveOptions, CvrpSisrParams, threads)`.
- `solveCvrpHgsParallel(allocator, inst, SolveOptions, CvrpHgsParams, max_vehicles, threads)`.

**VRPTW** — build a `VrptwInstance { n, matrix, demand, capacity, ready, due, service }`;
returns `VrptwResult`
- `solveVrptw(allocator, inst, SolveOptions, rounds, restarts, veh_penalty) !VrptwResult`.
- `solveVrptwHgs(allocator, inst, SolveOptions, VrptwHgsParams, veh_penalty) !VrptwResult`.

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

Uchoa X per-instance (SISR 20M): X-n101 **0.120%**, X-n153 0.452%, X-n200 **0.143%**,
X-n303 0.902%, X-n502 **0.087%**, X-n1001 1.034%. The two hard instances (X-n303 and
X-n1001) sit near 1%; the rest land between 0.09% and 0.45%.

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

<sub>Cores: commiv 3, the rest effectively 1. ¹ one unfinished LK trial. ² LKH could not
reach a feasible packing even warmstarted. ³ VROOM did not finish n=5000 within an hour.
VROOM n≤2000 runs at exploration level 5; n=5000 at level 3.</sub>

commiv is fastest and cheapest at every size. The nearest competitor on cost is VROOM
(within about 0.4% to 0.6%), but its wall-clock blows up with scale: 20x slower at n=1000,
about 180x at n=2000. LKH-3 needs a feasible warmstart just to run on explicit directed
matrices and falls apart past n=1000. There is no published optimum here, but VROOM landing
independently within 0.5% is strong evidence the solutions are near-optimal.

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
  gaps (about 0.16% to 0.39% on Uchoa X) when given far more time. commiv trades that last
  fraction of a percent for a large speed advantage. It is not state-of-the-art on accuracy
  at the frontier.
- **Massive symmetric instances.** FILO solves symmetric CVRPs with tens of thousands of
  nodes faster than anything here. commiv targets the routing-scale (hundreds to a few
  thousand) directed regime.
- **Production hardness.** OR-Tools and VROOM are battle-tested stacks with rich constraints
  (time windows, pickup and delivery, skills, breaks) and years of deployment. commiv is a
  fast, focused core, not a complete logistics platform.

---

## Design decisions

### What we settled on, and why

- **SISR (Slack Induction by String Removals) for large and asymmetric CVRP.** Ruin a few
  spatially-adjacent strings, greedily re-insert with random blinks, accept under a
  threshold or SA. The bet: millions of `O(removed)` moves beat thousands of `O(n)` ones.
  This is what cracks the large-n and directed regimes.
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
  doomed rebuilds, tour rebuilds dropped about 10x and the remaining cost was under 5% of
  wall-clock. The rewrite's complexity was not worth it.
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

Близкие к оптимальным маршруты транспорта по реальным дорожным сетям, за секунды.
Встраиваемый движок без зависимостей, рассчитанный на ориентированную (асимметричную)
стоимость, где путь из A в B не равен пути из B в A.

commiv решает семейства задач коммивояжёра и маршрутизации транспорта (TSP, ATSP, CVRP,
ACVRP, VRPTW) с точностью до доли процента от оптимума и читает ориентированные матрицы
времени в пути напрямую: улицы с односторонним движением, штрафы за повороты, заторы.
Быстрые симметричные солверы, к которым все привыкли, FILO и HGS-CVRP, вообще не умеют
принимать ориентированную матрицу. commiv построен именно вокруг этого случая.

Ставка всего движка: последние 0.3% оптимальности, которые стоят часа вычислений, почти
никогда не нужны. Нужен близкий к оптимальному маршрут сейчас, и на реальных дорогах он
обязан учитывать направление. Это и есть рабочая точка, на которую нацелен commiv.

> Для платформ курьерской доставки, последней мили и управления автопарком, которые
> пересчитывают тысячи ориентированных дорожных маршрутов в жёстком бюджете задержки,
> commiv — это встраиваемое ядро маршрутизации, которое возвращает близкие к оптимальным,
> допустимые по вместимости маршруты за секунды. В отличие от LKH-3 (однопоточный,
> некоммерческая лицензия, не справляется с явными ориентированными матрицами свыше
> примерно n=1000) или FILO и HGS-CVRP (быстрые, но только симметричные), он считает
> асимметричную дорожную стоимость основным случаем, а не довеском.

Почему это окупается: на масштабе автопарка вычисления солвера — это статья расходов,
которая растёт с тысячами маршрутов, пересчитываемых круглые сутки. На реальных данных
Moscow OSRM ориентированная CVRP при n=1000 даёт **207 406 за 2 с** у commiv против
225 917 за 60 с (OR-Tools), 221 487 за 456 с (LKH-3) и 208 687 за 315 с (VROOM). Дешевле и
быстрее одновременно. И всё это можно проверить на своих данных за один вечер; ни одно из
этих чисел не просит верить на слово.

```sh
zig build                                  # собрать библиотеку
zig build test                             # запустить 70 модульных тестов
zig build example                          # запустить пример встроенного солвера
```

- **Близко к оптимуму, быстро.** 0.02% от доказанных оптимумов на стандартной CVRP, около
  0.45% на тяжёлом наборе Uchoa X, за секунды-минуту на ноутбуке.
- **Асимметрия как родная.** Ориентированные матрицы времени в пути — первого класса, а не
  прикручены сбоку. На реальных данных Moscow OSRM commiv обходит OR-Tools, LKH-3 и VROOM и
  по стоимости, и по времени.
- **Ноль зависимостей, детерминизм.** Один модуль на Zig, без системных библиотек, без
  загрузок при сборке. Одно и то же зерно даёт одни и те же маршруты.
- **Экономно.** Ориентированная CVRP на 5000 узлов решается за 109 с при 211 МБ, и 100 МБ
  из них — сама матрица.

---

## Встраивание commiv в ваш код

Реальная точка входа — это не файл TSPLIB. Это ваши собственные точки, матрица стоимостей и
спрос на каждую точку. Вот вся интеграция от начала до конца.

### 1. Добавить зависимость

```sh
zig fetch --save "git+https://github.com/grechman/Path-finding-LKH-Zig-version"
```

Пакет сохранится под именем `commiv`. Подключите модуль в `build.zig`:

```zig
const commiv = b.dependency("commiv", .{
    .target = target,
    .optimize = optimize,
});
exe.root_module.addImport("commiv", commiv.module("commiv"));
```

### 2. Решить задачу маршрутизации на своих данных

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

### 3. Прочитать результат

`CvrpResult` владеет своей памятью, так что вызовите `deinit()`, когда закончите.

- `result.total_cost` — суммарная стоимость маршрутов по вашей матрице, тип `u64`.
- `result.routes` — по одному срезу на машину. Каждый срез перечисляет индексы клиентов в
  порядке посещения, депо подразумевается на обоих концах.

### 4. Выбрать солвер под размер задачи

| Точка входа | Когда использовать |
|---|---|
| `solveCvrpSisr` | Большие и/или направленные задачи. Это случай дорожной сети. |
| `solveCvrpHgs` | Средняя CVRP, n примерно до 500, когда нужна последняя доля качества. |
| `solveCvrpSisrParallel` / `solveCvrpHgsParallel` | То же самое, но по нескольким ядрам. |
| `solveCvrp` | Точка входа по умолчанию без настройки. Запускает SISR. |

### 5. Обычные TSP и ATSP

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

### Важные настройки

- `SolveOptions.seed` — зерно генератора случайных чисел. Для однопоточных солверов одно и то
  же зерно даёт побайтово идентичный результат, так что прогоны воспроизводимы. Варианты
  `*Parallel` зависят ещё и от числа потоков, так что для точного воспроизведения
  параллельного прогона фиксируйте и зерно, и число потоков.
- `SolveOptions.budget.trials` и `.max_passes` определяют, насколько усердно работает поиск.
  Больше — ближе к оптимуму и дольше. Бюджет считается в итерациях, а не по настенным часам,
  так что подбирайте его под свою цель по задержке эмпирически.
- Каждый возвращённый маршрут соблюдает `capacity`. Задача без допустимой упаковки вернёт
  ошибку, а не тихо неверный ответ.

---

## Справочник по API

Каждый солвер сначала принимает аллокатор, затем структуру опций и возвращает результат, который
вы освобождаете через `deinit()`. Перечисленный ниже набор — это весь публичный API (он
повторяет [`src/root.zig`](src/root.zig)); у каждого солвера есть модульные тесты в его файле.

**Разбор**
- `parseTsplib(allocator, text, ParseOptions) !Problem` разбирает текст TSPLIB / CVRPLIB.
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
- `solveParallel(allocator, *Problem, SolveOptions, ParallelOptions) !SolveResult` —
  независимые острова с опциональной рекомбинацией EAX или детерминированный режим деления
  бюджета ради скорости.

**ATSP (направленная)** — матрица `n x n` в строковом порядке, `matrix[i*n + j]` = стоимость `i → j`
- `solveAtsp(allocator, matrix, n, SolveOptions) !SolveResult` — 2n-преобразование Йонкера-Волгенанта.
- `solveAtspNative(allocator, matrix, n, SolveOptions) !SolveResult` — прямой направленный поиск.
- `solveAtspParallel(allocator, matrix, n, SolveOptions, threads) !SolveResult`.

**Точное решение (крошечное n)**
- `bruteForce(allocator, *Problem, ExactOptions) !SolveResult`.

**CVRP / ACVRP** — соберите `CvrpInstance { n, matrix, demand, capacity }` с направленной
матрицей `(n+1) x (n+1)` (депо = узел 0); все солверы возвращают `CvrpResult { routes, total_cost }`
- `solveCvrp(allocator, inst, SolveOptions) !CvrpResult` — точка входа по умолчанию (запускает SISR).
- `solveCvrpSisr(allocator, inst, SolveOptions, CvrpSisrParams)` — большие / направленные.
- `solveCvrpHgs(allocator, inst, SolveOptions, CvrpHgsParams, max_vehicles)` — n ≲ 500.
- `solveCvrpFleet(allocator, inst, SolveOptions, rounds, restarts, max_vehicles)` — фиксированный парк.
- `solveCvrpSisrParallel(allocator, inst, SolveOptions, CvrpSisrParams, threads)`.
- `solveCvrpHgsParallel(allocator, inst, SolveOptions, CvrpHgsParams, max_vehicles, threads)`.

**VRPTW** — соберите `VrptwInstance { n, matrix, demand, capacity, ready, due, service }`;
возвращает `VrptwResult`
- `solveVrptw(allocator, inst, SolveOptions, rounds, restarts, veh_penalty) !VrptwResult`.
- `solveVrptwHgs(allocator, inst, SolveOptions, VrptwHgsParams, veh_penalty) !VrptwResult`.

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

---

## Бенчмарки

Каждый разрыв указан относительно показанного эталона: доказанного оптимума, опубликованного
лучшего известного результата или эталонного солвера. Железо — ноутбук, Intel i3-1115G4
(2 ядра, 4 потока). Время — настенные часы при указанном бюджете.

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

<sub>Ядра: commiv 3, остальные фактически 1. ¹ один незавершённый прогон LK. ² LKH не смог
получить допустимую упаковку даже с тёплым стартом. ³ VROOM не завершил n=5000 в пределах
часа. VROOM при n≤2000 идёт на уровне исследования 5; при n=5000 — на уровне 3.</sub>

commiv быстрее и дешевле на каждом размере. Ближайший конкурент по стоимости — VROOM (в
пределах примерно 0.4–0.6%), но его настенное время взрывается с масштабом: в 20 раз медленнее
при n=1000 и примерно в 180 раз при n=2000. LKH-3 нужен допустимый тёплый старт просто чтобы
запуститься на явных направленных матрицах, и он разваливается после n=1000. Опубликованного
оптимума здесь нет, но то, что VROOM независимо попадает в пределах 0.5%, — сильное
свидетельство того, что решения близки к оптимальным.

---

## Как commiv выглядит на фоне других, честно

**Где он выигрывает**

- **Граница скорости и качества.** Близко к оптимуму за секунды, а не за минуты-часы, которые
  тратят эталонные эвристики. Для планировщика, которому приходится постоянно пересчитывать,
  это и есть главное число.
- **Направленные реальные дорожные матрицы.** Асимметричная стоимость — первого класса. FILO
  и HGS-CVRP, чемпионы симметричной скорости и качества, физически не умеют читать
  направленную матрицу. На реальных данных по Москве commiv обходит OR-Tools, LKH-3 и VROOM
  по стоимости и по времени.
- **Ноль зависимостей, малый след.** Один модуль на Zig, и ориентированная CVRP на 5000 узлов
  в 211 МБ.

**Где выигрывает конкуренция, и это надо знать**

- **Абсолютная точность при огромных бюджетах.** LKH-3, HGS-CVRP и SISR (статья) достигают
  меньших разрывов (примерно 0.16–0.39% на Uchoa X), когда им дают намного больше времени.
  commiv меняет эту последнюю долю процента на большое преимущество в скорости. По точности на
  границе он не state-of-the-art.
- **Огромные симметричные инстансы.** FILO решает симметричные CVRP с десятками тысяч узлов
  быстрее всего, что здесь есть. commiv нацелен на направленный режим масштаба маршрутизации
  (от сотен до нескольких тысяч).
- **Промышленная закалённость.** OR-Tools и VROOM — проверенные в бою стеки с богатыми
  ограничениями (временные окна, забор и доставка, навыки, перерывы) и годами эксплуатации.
  commiv — быстрое сфокусированное ядро, а не полноценная логистическая платформа.

---

## Проектные решения

### На чём остановились и почему

- **SISR (Slack Induction by String Removals) для большой и асимметричной CVRP.** Разрушить
  несколько пространственно соседних строк, жадно вставить обратно со случайными «миганиями»,
  принять по порогу или SA. Ставка: миллионы ходов `O(removed)` бьют тысячи ходов `O(n)`.
  Именно это вскрывает режимы большого n и направленности.
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
