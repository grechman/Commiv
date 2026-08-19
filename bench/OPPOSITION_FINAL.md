# Final post-fix opposition audit

**Candidate:** `26c8a1cca3f97e6d38498a968c6a4e786c8815ca`
**Machine:** Ryzen 5 2600X, Ubuntu/WSL2 on the Windows server
**Campaign completed:** 2026-08-19 06:22 server time
**Scope:** all scheduled quality-affecting large-road, GH-1000, Li & Lim PDPTW,
and synthetic/real-road money reruns. No Commiv or dependency upgrade was made
during this campaign.

This report extends [`OPPOSITION_POSTFIX.md`](OPPOSITION_POSTFIX.md). The earlier
73-row artifact remains immutable. For the 14 overlapping PDPTW/money logical
rows, this campaign's full-grid rows are authoritative.

## Verdict

- The fixed-work performance result remains the causal old/new speed evidence:
  ordinary PDPTW is **20.9%/22.7% faster** with identical output, while retained
  allocator fixes cut peak RSS by **67-93%** on the affected paths. Fresh
  opposition walls are quality evidence, not before/after speed evidence.
- The complete 352-cell academic money rerun gives Commiv **$10,970,675.92**
  versus VROOM **$12,236,718.41**: VROOM spends **$1,266,042.49 more
  (+11.540%)**, with Commiv W/T/L **304/26/22**. The historical headline was
  $1,267,610/+11.556% (published as +11.6%); the refreshed one-decimal headline
  is **+11.5%**.
- On the ordinary 352-cell PDPTW grid, Commiv is validator-complete on all 352.
  The BKS-fleet VROOM adapter reports 114 complete and 238 incomplete cells.
  Completion, then fleet, then distance gives Commiv **283/53/16**. This is a
  protocol-specific fixed-fleet comparison, not a claim that VROOM cannot
  complete those instances with more vehicles.
- The hash-identical NYC-1000 real-road money cell improves from old Commiv
  **$8,584.76** to fresh **$8,545.73** at 24 vehicles. Fresh VROOM uses 25
  vehicles and costs **$8,855.48**, $309.76/+3.625% more, but consumes 77.3 s
  on a nominal 60 s setting.
- Across seven affected large road-CVRP cells, fresh Commiv beats fresh PyVRP
  best-of-three on **6/7** (paired seeds **18/0/3**). Across six road-VRPTW
  cells it wins **5/6 cost-only**, but only **4/6 fleet-first**.
- The GH follow-up is **not a validity-clean six-cell contest**: 9/15 new PyVRP
  rows fail the adapter's independent double-precision time-window check. They
  are excluded below. Commiv's own 15 rows are feasible, so Commiv-only movement
  remains usable; no full GH scoreboard is claimed pending a corrected rerun.

## Artifact and manifest validation

| check | result |
|---|---|
| JSON rows | exactly **1,611**, all strict JSON, final newline, no literal or logical duplicates |
| Journal | exactly **1,589** unique cells; reconstructed manifest matches with no missing/extra cell |
| Family rows | meta 1; real money 24; road 18; road-TW 18; GH 30; academic money 704; PDPTW 816 |
| Journal cells | build 1; real money 24; road 12; road-TW 12; GH 20; academic money 704; PDPTW 816 |
| Li & Lim instances | 56/60/58/60/60/58 by size = **352**; manifest-name SHA `fa3240ddec9c98b4fdb6d53d1a8d4257fbb194dc6829f616cfbdaff3016f4943` |
| Seeds | PDPTW n=100 Commiv 1/2/3; larger PDPTW seed 1; money seed 1; road/road-TW/GH 1/2/3, all exact |
| Candidate guard | exact full SHA above; clean server worktree at launch |
| Completion marker | `CAMPAIGN_COMPLETE 1589 cells`; final service inactive; no remaining benchmark process |

Eleven grouped PyVRP launch cells each emit three seed rows, explaining the
22-row difference between results and journal. Four early, pre-hotfix result
rows lack `_cell` (`build`, Moscow-100 dump, and its two Commiv drivers); they
map unambiguously to journal cells. The retained 14-line error log is one
initial Moscow-100 VROOM prefix-parser failure. That cell was retried once and
has exactly one final row. There are no raw command timeouts or nonzero solver
blocks.

The final runner snapshot is post-hotfix: the first attempt expected `vroom,`
while the adapter emitted `<instance>,`. Therefore it is preserved as the
**final** runner, not falsely described as byte-identical to the initial
launch. WSL restarts were recovered append-only from the journal; completed
rows were never truncated or duplicated.

Re-run the automated audit with:

```bash
python3 bench/audit_postfix.py
# Deliberately exits nonzero until the nine invalid GH opponent rows are resolved:
python3 bench/audit_postfix.py --require-valid-gh
```

## Causal old/new CPU and RSS evidence

These are fixed-work A/B measurements from [`bench/README.md`](README.md), not the
fresh wall-bound campaign:

| retained change | old -> new result |
|---|---:|
| `lr2_10_1`, 20k PDPTW iterations | 3682/3716 ms baseline medians -> 2913/2872 ms; **20.9%/22.7% faster**, exact SHA/objective |
| PDPTW HTTP solve / dispatch / VROOM compatibility | peak RSS **-92.6% / -67.0% / -68.2%** |
| packed PDPTW snapshots | peak RSS **-91.4%** |
| parallel HGS / VRPTW workspaces | peak RSS **-85.2% / -80.2%** |
| capped Split | **4.86x faster, -71.8% RSS**, exact result |
| NumPy bridge | **66.2x faster, -60.3% RSS** |
| large native ATSP dispatch / bounded top-k | about **37x / 2.12x faster** |

## Full 352-cell academic money objective

Declared synthetic objective:

```
$140 * vehicles + $0.50 * distance + $0.50 * full route duration
```

Commiv used `PB_TIMEPEN=1 PB_VEH_PEN=280000`, one thread, seed 1. VROOM
received the same fixed vehicle price and configured limit, but cannot optimize
the duration/waiting term directly. All 352 VROOM rows report complete.

| size | n | old Commiv mean | fresh Commiv mean | fresh VROOM mean | VROOM over fresh C | fresh C W/T/L |
|---:|---:|---:|---:|---:|---:|---:|
| 100 | 56 | $3,806.94 | $3,806.94 | $3,881.35 | +1.955% | 35/20/1 |
| 200 | 60 | $8,634.09 | $8,634.09 | $8,973.34 | +3.929% | 46/5/9 |
| 400 | 58 | $18,125.41 | $18,125.59 | $19,392.34 | +6.989% | 52/0/6 |
| 600 | 60 | $32,899.63 | $32,902.10 | $36,208.64 | +10.050% | 55/0/5 |
| 800 | 60 | $50,679.50 | $50,686.62 | $57,142.35 | +12.737% | 59/0/1 |
| 1000 | 58 | $71,928.39 | $71,945.43 | $81,985.29 | +13.955% | 57/1/0 |
| **all** | **352** | **$10,969,101.88 total** | **$10,970,675.92 total** | **$12,236,718.41 total** | **+$1,266,042.49 / +11.540%** | **304/26/22** |

The fresh Commiv total is $1,574.04 (+0.01435%) above the historical quality
snapshot; W/T/L is unchanged at every size. This is not a causal regression
measurement: the current campaign was pinned and the July publication campaign
was unpinned.

The mechanism is visible in the scored components. Commiv uses 8,876 vehicles
versus VROOM's 9,485 (609 fewer), and its full route duration is 16.61% lower
with waiting 64.30% lower. It travels 8.31% farther, but saves $85,260 in fleet
cost and $1,391,340.69 in duration cost while giving back $210,558.20 in
travel cost. The net is the $1,266,042.49 advantage. This remains a synthetic
academic/unit-mapping capability measurement, not a universal customer-savings
claim.

## Full ordinary Li & Lim PDPTW

For n=100, the best Commiv seed is selected lexicographically; larger sizes use
the published single-seed convention. A complete solution dominates an
incomplete one, then fleet and distance are compared.

| size | n | old C vs V W/T/L | fresh C vs V W/T/L | VROOM complete / incomplete | unassigned tasks |
|---:|---:|---:|---:|---:|---:|
| 100 | 56 | 22/33/1 | 22/33/1 | 50 / 6 | 18 |
| 200 | 60 | 45/12/3 | 44/12/4 | 32 / 28 | 138 |
| 400 | 58 | 53/3/2 | 53/3/2 | 12 / 46 | 588 |
| 600 | 60 | 55/2/3 | 55/2/3 | 8 / 52 | 1,162 |
| 800 | 60 | 55/2/3 | 55/2/3 | 6 / 54 | 1,628 |
| 1000 | 58 | 54/1/3 | 54/1/3 | 6 / 52 | 2,218 |
| **all** | **352** | **284/53/15** | **283/53/16** | **114 / 238** | **5,752** |

`unassigned` is tasks/stops, so 5,752 means 2,876 pickup/delivery pairs, not
5,752 shipments. The ordinary adapter gives VROOM the BKS fleet. Money mode
uses a free fleet and completes all 352, demonstrating why the incompleteness
claim must remain protocol-specific.

Among the 114 both-complete cells, 113 use equal fleet and Commiv distance
W/T/L is 45/53/15. `lr1_2_4` is the sole unequal-fleet pair: Commiv has
11 vehicles / 2871.397 distance versus VROOM 10 / 3614.561, so it is a
fleet-first Commiv loss despite shorter distance. Commiv's old-to-fresh quality
snapshot is 103 better / 208 exact / 41 worse lexicographically; aggregate
fleet falls from 8,810 to 8,799 and BKS-fleet matches rise from 155 to 156.
Again, these wall-bound trajectories are descriptive, not speed A/B evidence.

Commiv rows are gated by `validatePdptw`. VROOM completeness is adapter-reported
from its summary; routes were not retained for an independent full route walk.

## Hash-identical real-road money rerun

All six fresh dump hashes exactly match the archived July dumps. Road matrices,
durations, and prices are real; pickup/delivery pairing and appointment windows
remain deterministic synthetic overlays. Formula:

```
(drive + 7 * duration + 30000 * vehicles) * $0.0015
```

The displayed Commiv value is the better of two full-budget drivers
(`fleetmin`, `plain`), a two-run portfolio rather than a single solve.

| cell | old Commiv | fresh Commiv (veh, driver) | fresh VROOM (veh) | C movement | fresh VROOM over C |
|---|---:|---:|---:|---:|---:|
| Moscow-100 | $1,117.60 | $1,117.60 (3, plain) | $1,135.18 (3) | exact | +1.573% |
| NYC-100 | $1,024.09 | $1,024.09 (3, fleetmin) | $1,053.91 (3) | exact | +2.912% |
| Moscow-1000 | $10,272.31 | $10,301.84 (28, fleetmin) | $10,765.54 (29) | 0.288% worse | +4.501% |
| **NYC-1000** | **$8,584.76** | **$8,545.73 (24, fleetmin)** | **$8,855.48 (25)** | **0.455% better** | **+$309.76 / +3.625%** |
| Moscow-2000 | $20,255.12 | $20,104.67 (54, fleetmin) | **$19,815.74 (53)** | 0.743% better | **-1.437%** |
| NYC-2000 | $17,095.14 | $17,095.14 (47, fleetmin) | $17,156.97 (48) | exact | +0.362% |

Fresh Commiv wins 5/6; the arithmetic mean per-cell VROOM gap is +1.923%
(aggregate-dollar ratio +1.020%). NYC-1000's raw physical totals are old
$8,584.7565 -> fresh $8,545.7265; its dump SHA is
`23c28aaa64b17676593a873047ec6662d675d0a2ffc347ad3a3033e1b9cfb232`.
Fresh VROOM's 77.259 s external wall against the nominal 60 s favors VROOM and
prevents an equal-actual-compute claim.

## Affected large road CVRP

Lower travel cost is better. New PyVRP budgets are freshly anchored to
`ceil(mean Commiv solver-reported milliseconds)`; historical walls differ and
must not be interpreted as speed changes.

| instance | historical C | fresh C (routes) | C movement | fresh PyVRP (routes) | Py relative to C | paired C W/T/L | historical VROOM* |
|---|---:|---:|---:|---:|---:|---:|---:|
| Moscow-1000 | 204320 | 204320 (50) | exact | 206726 (50) | +1.178% | 3/0/0 | 208687 |
| Moscow-2000 | 360075 | 360823 (101) | 0.208% worse | 365564 (101) | +1.314% | 3/0/0 | 368373 |
| Moscow-5000 | 765113 | 764362 (252) | 0.098% better | 769860 (253) | +0.719% | 3/0/0 | 781325 |
| NYC-1000 | 91780 | 91735 (11) | 0.049% better | 93070 (11) | +1.455% | 3/0/0 | 96774 |
| NYC-2000 | 132471 | 132354 (11) | 0.088% better | 132425 (11) | +0.054% | 1/0/2 | 138341 |
| Berlin-1000 | 109367 | 108909 (11) | 0.419% better | 109386 (11) | +0.438% | 3/0/0 | 116251 |
| Berlin-2000 | 162783 | 163478 (11) | 0.427% worse | **163392 (11)** | **-0.053%** | 2/0/1 | 173541 |

Fresh best-of-three: Commiv 6/7; paired seeds 18/0/3. `*`VROOM rows are the
dated deterministic publication rows and were deliberately not rerun.

## Affected road VRPTW

Cost-only and fleet-first answers differ. Parentheses are vehicles.

| instance | historical C | fresh C | fresh PyVRP | Py cost relative to C | paired cost C W/T/L | paired fleet-first C W/T/L | historical VROOM* |
|---|---:|---:|---:|---:|---:|---:|---:|
| Moscow-1000 | 229841 (51) | 230098 (51) | 237541 (52) | +3.235% | 3/0/0 | 3/0/0 | 239977 (50) |
| Moscow-2000 | 406855 (102) | 407127 (102) | 413770 (102) | +1.632% | 3/0/0 | 3/0/0 | 419760 (101) |
| NYC-1000 | 133286 (19) | 133869 (18) | 133865 (20) | -0.003% | 2/0/1 | 3/0/0 | 144517 (18) |
| NYC-2000 | 214611 (34) | 214934 (34) | 215068 (30) | +0.062% | 3/0/0 | **0/0/3** | 233594 (30) |
| Berlin-1000 | 152256 (21) | 152056 (20) | 152670 (22) | +0.404% | 3/0/0 | 3/0/0 | 162971 (19) |
| Berlin-2000 | 245718 (35) | 245605 (35) | 248722 (30) | +1.269% | 3/0/0 | **0/0/3** | 270145 (30) |

Cost-only best is 5/6 and paired 17/0/1; fleet-first best is 4/6 and paired
12/0/6. NYC/Berlin-2000 are cost wins but fleet-first losses. `*`Historical
VROOM rows were not freshly rerun.

## GH-1000 validity-limited result

Objective is fleet first, then distance. Only PyVRP rows whose independent
exact-Euclidean schedule check says `True` are eligible.

| instance | historical C | fresh C | valid fresh PyVRP best | valid seeds | honest status |
|---|---:|---:|---:|---:|---|
| c1_10_1 | 100 / 42486.02 | 100 / 42486.02 | 100 / 42478.98 | 3/3 | PyVRP narrow win (phase 1) |
| c2_10_1 | 30 / 16879.27 | 30 / 16879.27 | 31 / 17130.78 | 1/3 | Commiv win, one-seed sentinel only |
| r1_10_1 | 100 / 55171.25 | 100 / 55171.25 | — | 0/3 | no valid opponent result |
| r2_10_1 | 20 / 42109.77 | 21 / 41469.33 | 32 / 37279.20 | 3/3 | Commiv wins; fresh C lost one vehicle vs old |
| rc1_10_1 | 92 / 47316.37 | 92 / 46997.13 | 95 / 48485.99 | 1/3 | Commiv win, one-seed sentinel; C distance 0.675% better vs old |
| rc2_10_1 | 23 / 29854.43 | 24 / 29400.83 | 29 / 28301.80 | 1/3 | Commiv win, one-seed sentinel; fresh C lost one vehicle vs old |

Invalid PyVRP rows are c2 seeds 2/3; r1 seeds 1/2/3; rc1 seeds 1/2; and
rc2 seeds 1/3. Several tempting published minima match rows now marked invalid,
so historical GH PyVRP minima are not used as a clean fallback. Across only the
nine available valid paired rows, the descriptive count is Commiv 6/0/3, but it
is intentionally not promoted to a family headline because missing invalid
seeds can bias selection.

## Other retained fresh sentinels

- X-n1001-k43: Commiv 72966, PyVRP 73370, HGS-CVRP 74433; Commiv wins all
  three paired seeds against both.
- Moscow-1000 directed CVRP at the fresh 29 s budget: Commiv 204320,
  OR-Tools 235079, LKH warm-start 415634. OR-Tools is deterministic and LKH is
  a single ACVRP sentinel, not a claim about LKH's native TSP strength.
- The affected road tables above plus X, GH, full PDPTW, and money are the
  fresh coverage. TSP/LKH, published small ATSP/LKH, Augerat/ACVRP, road n=100,
  Solomon/GH `_4_`, and deterministic road VROOM remain historical because the
  retained code does not reach those paths or they were outside this manifest.

## Wall and protocol honesty

Configured solver limits are not hard equal external walls:

- Academic money VROOM exceeds nominal wall on **84/352** cells (62 by >10%):
  maximum `lrc2_10_4` **174.382 s at a 90 s setting (1.938x)**. Commiv's maximum
  is 96.247 s at 90 s; its internal deadline is also soft, so the overage is not
  merely process startup.
- Ordinary PDPTW VROOM exits early and has zero external overshoots; this often
  accompanies its fixed-fleet incomplete result. Commiv's largest overruns are
  110.061 s at 90 s and 76.328 s at 60 s.
- Real-road money VROOM takes 75.7/77.3 s at nominal 60 s and 206.0/218.9 s at
  nominal 120 s. The largest ratio is NYC-2000, 1.824x.
- Commiv was pinned; principal competitors were unpinned. That is conservative
  for newly anchored quality comparisons but forbids old/new speed claims from
  these walls. Use the fixed-work A/B harness for speed.
- Academic VROOM feasibility is adapter-reported. Real-money fresh rows retain
  summary outputs but not the July independent route-walk artifact. Claims are
  therefore phrased as reported completion, except Commiv rows that are
  validator-gated in their benches.

## Persisted files

| file | role | SHA-256 / count |
|---|---|---|
| `postfix-remaining.jsonl` | immutable final raw result rows | `2a99114cd125343e05e2506802f58af32e2f238c6a39e400a89983e4dc28a88f`, 1611 |
| `postfix-remaining.cells` | append-only completion journal | `ab8f26ebdadc15da85e061fbb23ab35de89b2c3cbd0e0b4385c8db005d5fdc6a`, 1589 |
| `postfix-remaining-raw.log` | command output and actual walls | `d9fec27f1eb26e8ff86c0a4400d45e051bb7e0f933d2db06281f97c503858938` |
| `postfix-remaining-errors.log` | retained initial parser failure | `833acb026d070da5d961e661910ab16c4b0c0f1db01897eead56a63b9af6060e` |
| `postfix-remaining-runner.py` | final post-hotfix runner snapshot | `d6f2343a9d06db6ff88072252a77fae412b75a14afd899f619f00876e8c92841` |
| `postfix-final-summary.json` | machine-readable derived summary | generated from the raw artifact |
| `audit_postfix.py` | repeatable integrity/objective/validity audit | standard-library-only |
| `postfix-opposition.jsonl` | immutable phase-1 artifact | `93c9d69f7f33b00cbde485ea7dc6c0e503d3bd20c9e7146df153c2e8b3da8d2b`, 73 |
| `postfix-opposition-combined.jsonl` | derived phase-1 + phase-2 view; full rows supersede 14 overlaps | `2aac76f9dd266a2b22bfd72054c6ca08a556392713bf51ad4e2e4dcbe4c84928`, 1670 unique logical rows |

The combined artifact contains the 59 non-overlapping phase-1 rows followed by
all 1,611 full-campaign rows. Raw source artifacts are preserved unchanged.
