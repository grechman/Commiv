# M3 — real-road money proof

## 1. What this proves

The money objective (PDPTW route-duration + per-vehicle pricing) run on **real directed
OSRM road travel times** — the NYC second-matrices in `vendor/road/nyc-*.road` — with
**real, web-sourced 2025-26 delivery prices**. This is the first time the money proof
touches real road data instead of academic Euclidean coords.

Real-vs-synthetic split, stated up front and not softened:

- **REAL:** the directed OSRM travel-time matrix (asymmetric seconds; one-ways and turn
  restrictions baked in) and the route **durations** it induces (drive + service + wait) —
  exactly what the money objective prices and what VROOM structurally cannot jointly
  express — plus the three prices ($37.80/hr loaded driver, $0.27/km van operating,
  $45/veh/shift), each cited in §2.
- **SYNTHETIC:** the pickup/delivery **pairing** (the `.road` files are plain depot+client
  sets with no pairs — pairs are fabricated deterministically, `custs[2i]`↔`custs[2i+1]`,
  seeded), the **time windows** (synthesized from a provably-feasible reference schedule,
  `ready=0` everywhere), the demands, and the single **20 km/h speed** used only to price
  the smaller operating term. The two dominant terms — driver-duration and vehicle-fixed —
  need no speed assumption.

No VROOM head-to-head number is claimed here; that cell is **pending** (§4), not estimated.

## 2. Cost model

### Units (verified in source)

- **Matrix = SECONDS, directed.** `tools/fetch_road_matrix.py` fetches OSRM
  `annotations=duration` and writes `M[i*DIM+j] = round(dur)` seconds. Sanity:
  `nyc-100.road` row-0 arcs run 664–1355 s = 11–22 min drives; `CAPACITY 48`.
- **Solver money cost** (`src/pdptw_sisr.zig:291`):
  `cost = dist + time_penalty·dur + veh_penalty·nonempty`, where `dist` = sum of matrix
  arcs = driving-seconds (coefficient locked at 1), `dur` = full-route Tws duration
  (travel + service + unavoidable wait) in seconds, `veh_penalty` added once per nonempty
  route. The `dist` channel is **time, not km** — OSRM here fetched durations only, so the
  $/km price must be expressed as a per-driving-second rate, which needs one assumed speed.
  That speed is the only synthetic number in the price mapping.

### Real prices (US metro / NYC, 2025-26)

1. **Loaded driver = $37.80/hr.** Base local delivery/box-truck driver ~$28/hr, × 1.35
   employer burden (FICA + health + PTO + comp) = $37.80/hr.
   Sources: hmdtrucking NY driver pay; Glassdoor NYC delivery driver; Indeed NYC.
2. **Van operating = $0.27/km** (fuel + maint + tires, driver excluded). Cargo-van variable
   $0.35–0.50/mi; $0.27/km = $0.435/mi mid-band.
   Sources: truxx.ai cargo-van cost-per-mile; ATRI via truckinginfo.
3. **Fixed = $45/veh/shift.** Lease/deprec ~$700/mo + commercial insurance ~$300/mo ≈
   $1,000/mo ÷ 22 days. (Cargo-van class — matches cap 48, parcels 1–9.)
   Sources: Edmunds ProMaster NY lease; MoneyGeek NY commercial auto; LogRock cargo-van
   insurance.
4. **Speed = 20 km/h (SYNTHETIC, the one soft knob).** NYC FY24 midtown 4.8 mph / CBD
   6.8 mph; a citywide route blends to ~20 km/h ≈ 12.4 mph. At 20 km/h, $0.27/km ⇒
   **$5.40 per driving-hour**.
   Sources: Axios NYC congestion; NY Senate "Speed Kills" 2024.

### Mapping to engine knobs (exact integers, zero quantization)

Money unit ≜ **$0.0015** = cost of one driving-second ($5.40/driving-hr ÷ 3600), forced by
the locked coefficient-1 `dist` channel. Per-second rates: driver $0.0105/s, operating
$0.0015/driving-s, fixed $45/veh.

```
MR_TIMEPEN = driver_rate / operating_rate = 0.0105 / 0.0015 = 7        (EXACT)
MR_VEH_PEN = fixed_$ / money_unit         = 45     / 0.0015 = 30000    (EXACT)
dollars    = internal_cost_units × $0.0015   (= units / 666.67)
```

`MR_TIMEPEN` is a dimensionless rate ratio (scale-invariant); `MR_VEH_PEN` is fixed-cost in
money units. Both land on exact integers, so there is **no rounding fudge to disclose**.
Coherence: a route of 3600 s drive / 4000 s duration / 1 van costs
`3600 + 7·4000 + 30000 = 61600` units → **$92.40** = operating $5.40 + driver $42.00 +
fixed $45.00. Consistent.

## 3. Results (real directed NYC road matrices)

Money-mode fleet-min (`solvePdptwSisrFleetMin`), single thread, seed 1. `dist_sec` is the
independent `validatePdptw` oracle's pure directed-travel total (a printed row = feasible +
validated; `FAIL_INFEASIBLE` otherwise). Dollars scored under the §2 Prices model.

| instance | wall | veh | dist_sec (drive) | dur_sec (drive+svc+wait) | wait | commiv $ | VROOM $ | overspend % |
|---|---|---|---|---|---|---|---|---|
| nyc-100  | 30s | 13  | 57 583 (16.0 h)  | 86 983 (24.2 h)   | 0 | **$1 584.70**  | pending — winserver | pending |
| nyc-1000 | 60s | 116 | 505 936 (140.5 h) | 805 336 (223.7 h) | 0 | **$14 434.93** | pending — winserver | pending |

Dollar decomposition (operating $0.0015/drive-s + driver $0.0105/s + fixed $45/veh):

- nyc-100:  operating $86.37 + driver $913.32 + fixed $585.00 = **$1 584.70**
  (units `57583 + 7·86983 + 30000·13 = 1 056 464` × $0.0015).
- nyc-1000: operating $758.90 + driver $8 456.03 + fixed $5 220.00 = **$14 434.93**
  (units `505936 + 7·805336 + 30000·116 = 9 623 288` × $0.0015).

**Reproduce (one command per row):**

```
cd /home/grechman/commiv && zig build moneyroadbench -Doptimize=ReleaseFast
MR_FILE=nyc-100  MR_TIMEPEN=7 MR_VEH_PEN=30000 MR_SEED=1 MR_TIME_MS=30000 ./zig-out/bin/commiv-moneyroadbench
MR_FILE=nyc-1000 MR_TIMEPEN=7 MR_VEH_PEN=30000 MR_SEED=1 MR_TIME_MS=60000 ./zig-out/bin/commiv-moneyroadbench
```

**Wall-nondeterminism note:** the search is wall-clock-bounded, so exact `dist_sec`/`dur_sec`
drift a few tenths of a percent between runs (the fleet count is stable, dollars move
~$1 on nyc-100). The table is the run reproduced this session; re-running lands in the same
neighborhood, not bit-identical. This is the standard time-budget noise, not a bug.

**Two-price-model caveat (not a bug):** the bench's built-in `usd_total` CSV column uses the
**academic** BENCHMARKS prices ($140/veh + $0.50/min, no operating term) and printed
2544.86 / 22951.13. Those were **not** used. The search ran the real Prices knobs (7 /
30000) and the plan was scored under the coherent §2 Prices model. Mixing them would be
wrong; the table uses only the model the search actually optimized. Updating the hardcoded
column to the real prices is a cosmetic follow-up.

## 4. Full head-to-head — how to run, what's pending

VROOM is **not installed locally**, and `tools/vroom_pdptw.py` reads academic Li&Lim `.txt`
and builds a Euclidean matrix from coords — it **cannot** ingest the synthesized directed
road matrix. A fair run needs new plumbing:

1. add an instance dump to `moneyroadbench.zig` emitting the compacted K×K directed
   duration matrix + shipments (pickup↔delivery) + windows as JSON;
2. new `tools/vroom_road_pdptw.py` feeding VROOM via `matrices.car.durations` + shipments,
   `costs.fixed` = per-vehicle equiv (VROOM's best-expressible objective), same wall;
3. score VROOM's plan under the §2 Prices model (`dist·0.0015 + dur·0.0105 + veh·45`) →
   overspend %.

Winserver run once VROOM is built in WSL:

```
ssh maxgrechkov@100.123.98.112
# in WSL:
VROOM_BIN=~/cbench/vroom/bin/vroom VROOM_TIME=30 python3 tools/vroom_road_pdptw.py nyc-100
```

Until then the VROOM / overspend cells are **empty, not estimated**.

## 5. Limitations

- **Pairing and windows are synthetic.** The `.road` files carry no P/D structure or time
  windows; both are fabricated (seeded pairing, windows from a feasible reference schedule).
  What is real is the directed travel time between real NYC points and the durations it
  produces.
- **`ready=0` ⇒ `wait=0` on both instances.** The "money prices waiting, VROOM can't" edge
  does **not** bite here — the load-bearing claim on these two runs is pricing **directed
  duration + fleet jointly**, not waiting. To exercise the waiting term, synthesize
  staggered `ready` windows (or use `nyc-2000.tw`, which exists).
- **One synthetic price input: 20 km/h**, and it touches only the smaller operating term
  ($86 of $1585 on nyc-100). The dominant driver-duration and vehicle-fixed terms are
  assumption-free.
- **VROOM head-to-head is unrun.** The overspend claim central to the money story is
  pending the §4 plumbing + winserver run; no number is asserted in its place.
- **Numbers are wall-bounded**, not exact optima — reproducible to the neighborhood, not
  the digit (§3 note).

---

**Files:** `examples/moneyroadbench.zig`, `build.zig` (bench-list entry `moneyroadbench`),
`zig-out/bin/commiv-moneyroadbench`. No commit made; single thread throughout.
