# M3 — real-road money proof (head-to-head complete, audited)

## 0a. WINDOW-TIGHTNESS SWEEP — the appointment-window economics (2026-07-16 night)

The dose-response curve: slot openings spread across the 8 h shift
(`MR_STAGGER=28800`), window tightness = `MR_SLACK` at 900/1800/3600/7200 s =
15/30/60/120-minute appointment windows. moscow-1000 + nyc-1000 (499 pairs),
equal wall 60 s, single thread, seed 1, one price model per dump, commiv
best-of-two-drivers vs VROOM, per-width independent audit (feasibility CLEAN
throughout; only wall-noise reproducibility deltas of 0.02–0.25%).

| window | moscow-1000 gap | nyc-1000 gap | VROOM waiting (msk/nyc) | commiv waiting |
|---|---|---|---|---|
| **15 min** | **+5.8%** | **+8.0%** | 79.4 h / 80.6 h | 16.3 h / 15.9 h |
| **30 min** | **+3.7%** | **+5.7%** | 52.1 h / 50.3 h | 6.7 h / 7.9 h |
| **60 min** | **+1.2%** | **+2.5%** | 20.7 h / 20.5 h | 1.2 h / 2.2 h |
| 120 min | +0.6% | +1.4% | 8.2 h / 6.0 h | 0.1 h / 0.1 h |

Monotone in both cities: every halving of window width roughly doubles VROOM's
overspend, and the driver is entirely waiting time VROOM cannot price — at
15-minute windows its plans carry **~80 hours of paid driver idling per depot
per day** (5x commiv's structurally-unavoidable 16 h). In dollars at the §2
prices: the 15-min gap is $946/day (moscow) and $1163/day (nyc) per depot —
$345–425k/year, software-only.

Sales read: the commiv money edge is a function of window tightness. Express /
tight-appointment operations (the 15–30 min promise segment) sit at +4–8%;
generous 2 h slots at +1–3% (consistent with §0 below). Driver note: `plain`
won both 15-min cells, `fleetmin` won at 60/120 min — best-of-two is the
protocol, tightness flips the winner. Caveat: 15–30 min promises measured from
ORDER time (express q-commerce) are a rolling-horizon dispatch regime — this
sweep is its static proxy; the real product for that segment is M6.

Reproduce: winserver `~/money_tightsweep.sh` (binary from 09d97d1), results +
per-width audits `~/campaign/results/money-tightsweep-2026-07-16/`.

## 0. HEADLINE — the real-window head-to-head (2026-07-16 evening, winserver)

The definitive run: moscow + nyc road matrices with the PUBLISHED campaign courier-slot
windows (`vendor/road/*.tw`: 60% of customers in 2 h slots, 300 s service, 9 h horizon),
window-feasible deterministic pairing (`MR_PAIR=window`), byte-identical dumped
instances, ONE price model (embedded in each dump) steering AND scoring both engines,
single thread, seed 1, commiv best-of-two-drivers. Objective stated in advance: mean
dollar gap across 6 cells, sellable threshold >= +5% for commiv.

| cell | wall | commiv $ (veh, driver) | VROOM $ (veh) | gap | VROOM wait | commiv wait | VROOM wall |
|---|---|---|---|---|---|---|---|
| moscow-100  | 30s  | **1117.60** (3, plain)      | 1135.18 (3)   | **+1.6%** | 382 s    | 0 s   | 1.4 s |
| nyc-100     | 30s  | **1024.09** (3, fleetmin)   | 1053.91 (3)   | **+2.9%** | 3568 s   | 143 s | 1.3 s |
| moscow-1000 | 60s  | **10272.31** (28, fleetmin) | 10765.74 (29) | **+4.8%** | 62085 s  | 344 s | 77 s |
| nyc-1000    | 60s  | **8584.76** (24, fleetmin)  | 8862.87 (25)  | **+3.2%** | 30905 s  | 413 s | 80 s |
| moscow-2000 | 120s | 20255.12 (54, fleetmin)     | **19806.90** (53) | -2.2% | 51050 s | 364 s | **215 s** |
| nyc-2000    | 120s | **17095.14** (47, fleetmin) | 17148.22 (48) | +0.3% | 51156 s  | 79 s  | **229 s** |

**Verdict against the pre-stated objective:** commiv wins 5/6 cells; mean gap **+1.8%
as-run** — BELOW the +5% sellable threshold. On the four cells where VROOM honored the
wall (n<=1000) the mean is **+3.1%** and commiv sweeps 4/4. The two n=2000 cells — VROOM's
only win and the near-tie — are exactly where VROOM overshot the 120 s limit to 215/229 s
(1.8–1.9x commiv's compute); `-l` is evidently soft at scale. Reported as-run anyway; an
enforced-equal-compute rerun would need external kill or commiv walls matched to VROOM's
actual spend.

**The waiting wedge is now measured, not theoretical:** with real slots VROOM eats
31k–62k seconds of driver waiting per instance (17.2 h at moscow-1000, ~$650 of the
$493 + fleet gap) because it cannot price waiting; commiv holds waiting to 0–413 s.
This is the number the M4 demo should show.

**Audit:** independent step-walk of every VROOM route (pairing, precedence, capacity,
TWs, arrival arithmetic on the directed matrix) — CLEAN on all 6 cells. n=100 cells
bit-reproduce; the wall-bound moscow-1000/2000 reruns land within 0.004–0.2% (wall-clock
nondeterminism of `-l`, not a scoring bug). commiv rows oracle-gated by `validatePdptw`
in-bench. Fix commits (window pairing, one-price-model, C ABI 0.3.0 stack) validated
server-side the same run: full test suite + Python smoke green.

**Driver note:** fleetmin won 5/6 cells against plain under real windows (tight slots
make fleet structure dominate); keep best-of-two.

Reproduce: winserver `~/money_h2h3.sh` (commit 09d97d1), results + audit
`~/campaign/results/money-h2h-realtw-2026-07-16/`.

---

Below: the earlier 2026-07-16 loose-window round (kept — it maps the other end of the
regime axis: where windows are loose, the money edge shrinks to noise and VROOM ties).

## 1. What this proves

The money objective (PDPTW route-duration + per-vehicle pricing) run on **real directed
OSRM road travel times** — the NYC second-matrices in `vendor/road/nyc-*.road` — with
**real, web-sourced 2025-26 delivery prices**, head-to-head against VROOM 1.14 on
byte-identical instances, equal wall, single thread, seed 1, both plans independently
re-validated. Run on the winserver (Ryzen 5 2600X, WSL2), 2026-07-16.

Real-vs-synthetic split, stated up front and not softened:

- **REAL:** the directed OSRM travel-time matrix (asymmetric seconds; one-ways and turn
  restrictions baked in) and the route **durations** it induces (drive + service + wait),
  plus the three prices ($37.80/hr loaded driver, $0.27/km van operating, $45/veh/shift),
  each cited in §2.
- **SYNTHETIC:** the pickup/delivery **pairing** (fabricated deterministically, seeded),
  the **time windows** (base regime: `ready=0`, slack 3600s; waiting regime:
  `MR_STAGGER=7200` staggered openings with tight `MR_SLACK=300`), the demands, and the
  single **20 km/h speed** used only to price the smaller operating term.

**The headline: the Li&Lim +11.6% money win does NOT transfer wholesale to loose
real-road instances.** VROOM delivered 100% of shipments in every cell and the result is
a 2-2 split by regime (§3). The money edge is real but conditional: it lives where
waiting is structurally unavoidable and where completeness pressure bites — not
everywhere. See §5 for the full reconciliation.

## 2. Cost model

### Units (verified in source)

- **Matrix = SECONDS, directed.** `tools/fetch_road_matrix.py` fetches OSRM
  `annotations=duration` and writes `M[i*DIM+j] = round(dur)` seconds. Sanity:
  `nyc-100.road` row-0 arcs run 664–1355 s = 11–22 min drives; `CAPACITY 48`.
- **Solver money cost** (`src/pdptw_sisr.zig:291`):
  `cost = dist + time_penalty·dur + veh_penalty·nonempty`, where `dist` = sum of matrix
  arcs = driving-seconds (coefficient locked at 1), `dur` = full-route Tws duration
  (travel + service + unavoidable wait) in seconds, `veh_penalty` added once per nonempty
  route. The `dist` channel is **time, not km** — the $/km price is expressed as a
  per-driving-second rate via one assumed speed (the only synthetic number here).

### Real prices (US metro / NYC, 2025-26)

1. **Loaded driver = $37.80/hr.** Base local delivery/box-truck driver ~$28/hr, × 1.35
   employer burden. Sources: hmdtrucking NY driver pay; Glassdoor NYC; Indeed NYC.
2. **Van operating = $0.27/km** (fuel + maint + tires, driver excluded). Cargo-van
   variable $0.35–0.50/mi; $0.27/km = $0.435/mi mid-band. Sources: truxx.ai; ATRI.
3. **Fixed = $45/veh/shift.** Lease/deprec ~$700/mo + commercial insurance ~$300/mo ≈
   $1,000/mo ÷ 22 days. Sources: Edmunds ProMaster NY lease; MoneyGeek; LogRock.
4. **Speed = 20 km/h (SYNTHETIC, the one soft knob).** NYC FY24 congestion data. At
   20 km/h, $0.27/km ⇒ **$5.40 per driving-hour**. Sources: Axios; NY Senate 2024.

### Mapping to engine knobs (exact integers, zero quantization)

Money unit ≜ **$0.0015** = one driving-second ($5.40/hr ÷ 3600).

```
MR_TIMEPEN = 0.0105 / 0.0015 = 7        (EXACT)
MR_VEH_PEN = 45 / 0.0015     = 30000    (EXACT)
dollars    = (dist_sec + 7*dur_sec + 30000*veh) * 0.0015
```

Both engines' plans are scored by this ONE formula from physical outputs
(never the bench's built-in `usd_total` column, which still carries the academic
$140/$0.50 model — a known two-model footgun, do not mix them).

## 3. Head-to-head results (winserver, 2026-07-16, audited)

Byte-identical instances via `MR_DUMP` (both engines read the same arrays). commiv:
`solvePdptwSisr*` money mode, single thread, walls 30s (nyc-100) / 60s (nyc-1000), two
drivers tried (`MR_DRIVER=fleetmin|plain`), best per cell shown. VROOM 1.14: `-x 5 -t 1
-l <wall>`, per-vehicle `costs.fixed=30000` (its best-expressible money proxy; it cannot
price duration or waiting, VROOM-Project/vroom#1130).

| cell | commiv $ (veh, driver) | VROOM $ (veh) | winner | VROOM unassigned |
|---|---|---|---|---|
| nyc-100 base (49 pairs, wait-free) | 1584.70 (13, fleetmin) | **1579.76** (13) | VROOM +0.31% | 0 |
| nyc-100 wait (stagger 7200/slack 300) | **2587.62** (23, tie both drivers) | 2653.52 (23) | commiv +2.55% | 0 |
| nyc-1000 base (499 pairs) | 14210.24 (114, fleetmin) | **13883.32** (115) | VROOM +2.30% | 0 |
| nyc-1000 wait | **21701.83** (166, plain) | 21835.44 (164) | commiv +0.62% | 0 |

Waiting seconds in the wait cells: commiv 161 / 45024–67737 vs VROOM 12179 / 114121 —
commiv's plans structurally avoid the waiting VROOM cannot see; under these prices that
buys 0.6–2.6%. VROOM walls: 1.1s / 0.9s (nyc-100, converged early), 72s / 69s (nyc-1000,
slightly over the 60s limit — noted, favors VROOM ~20% wall there).

**Audit (all clean, `audit.log`):** correct fresh binary (mtime + knob-strings +
twprobe bit-identity 310936/aa832663f4281668 + budget-respect 2s); every commiv row
oracle-gated by `validatePdptw` in-bench; every VROOM plan independently re-walked
step-by-step against the dump (pairing, precedence, capacity, TWs, arrival arithmetic on
the directed matrix) and re-run to bit-reproducibility; recomputed drive/wait/duration
match VROOM's summary exactly. The identical commiv wait-cell rows across drivers are
explained: the shared uncapped prefix (fleet-min's p0 = plain's trajectory) finds the
final best within 12s; neither continuation improves it.

**Reproduce** (winserver, `~/commiv` at `1d0b6a5`+):

```
zig build moneyroadbench twprobe -Doptimize=ReleaseFast   # explicit steps, then guards
bash ~/money_h2h.sh    # round 1: fleetmin driver + VROOM cells -> commiv.csv, vroom.csv
bash ~/money_h2h2.sh   # round 2: MR_DRIVER=plain            -> commiv_plain.csv
bash ~/audit.sh        # independent feasibility + reproducibility audit
# results: ~/campaign/results/money-h2h-2026-07-16/
```

## 4. Driver note (measured, not assumed)

`MR_DRIVER=fleetmin` (hierarchical, minimizes vehicles lexicographically) vs `plain`
(flat SISR, penalties in acceptance): fleetmin won 2 cells, plain won 1, tie 1. The
prior "fleet-min distorts a money objective" hypothesis is only half-true — at nyc-1000
base, fleetmin's staged search found a solution that dominated plain on BOTH fleet and
distance in the same wall. Neither driver closes the base-cell gap to VROOM. Keep both
knobs; report best-of.

## 5. Honest verdict and reconciliation with the published +11.6%

The published money bench (BENCHMARKS.md §Money, Li&Lim, +11.6%/$1.27M) and this result
are both correct; they measure different regimes:

- **Li&Lim windows are tight.** Waiting is frequent and structurally unavoidable, and
  sequencing is hard — VROOM's travel+fixed proxy diverges badly from a duration-priced
  objective, and its overspend GROWS with size (+2.0% at n=100 → +14.1% at n=1000).
- **These road instances are loose** (base: all windows open from t=0). With everything
  deliverable and near-zero waiting, minimizing travel ≈ minimizing duration, so VROOM's
  expressible objective lands nearly on ours and its LS is strong and fast. The money
  edge structurally vanishes — as the published section itself warned ("a reference
  measurement, not a real-world win").
- **Vehicle price matters:** Li&Lim model used $140/veh (~4.7h of duration-price);
  real NYC van economics give $45/veh (~1.2h). Cheaper vehicles leave less room for
  joint fleet/duration optimization to differentiate.
- **Completeness:** the Li&Lim result also rode on VROOM dropping shipments on the
  non-comparable instances; on these loose road instances VROOM delivered 100%.

**What survives as the marketable claim:** commiv prices what VROOM cannot — total route
duration including waiting — and wins where that structure exists (staggered/tight
windows: +0.6–2.6% here, up to +14% on tight academic sets at scale) and wherever VROOM
drops shipments. "Always 11.6% cheaper" would be false on real loose instances; do not
ship that claim. The honest pitch is conditional and the demo (M4) should show a
waiting-heavy scenario, which is also the realistic one for appointment-window delivery.

## 6. Limitations

- Pairing and windows remain synthetic; the directed times, durations, and prices are real.
- Two instances, one city, seed 1, single-seed money cells (the campaign convention).
- VROOM slightly overshot the 60s wall at nyc-1000 (~70s), in its favor.
- The waiting regime is one point (7200/300); the edge grows with window tightness, and a
  tightness sweep would map where the crossover sits — not yet run.
- commiv nyc-1000 base at 60s single-thread is likely under-converged (fleetmin found
  114veh/494k vs plain 116veh/509k — still searching); more wall or threads would help
  but the equal-wall convention binds.

---

**Files:** `examples/moneyroadbench.zig` (+`MR_DRIVER`/`MR_DUMP`/`MR_STAGGER`/`MR_SOLVE`),
`tools/vroom_road_pdptw.py`, `build.zig`. Results + audit: winserver
`~/campaign/results/money-h2h-2026-07-16/`.
