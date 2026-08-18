# Post-fix opposition audit

This is the fresh quality audit for commit `26c8a1c` after the performance
campaign. It is deliberately separate from the frozen mechanical A/B results:
quality campaigns compare solvers at a newly anchored wall budget, while the
mechanical benchmarks require identical result signatures.

## Method

- Machine: the Windows server's Ubuntu/WSL2 ext4 worktree.
- No game process was running; the host was otherwise idle.
- Commiv was pinned to CPUs 0-9. PyVRP, HGS, and VROOM were not pinned; the
  secondary OR-Tools/LKH sentinels were pinned to one core each. This is
  conservative for Commiv, but means these wall times must **not** be used as a
  before/after speed comparison with the unpinned July publication campaign.
- Commiv used its published work/configuration: seeds 1, 2, and 3 for road,
  X, GH, and the n=100 PDPTW sentinel; the published single-seed convention for
  PDPTW n>=200 and money. The opponent budget was `ceil(mean Commiv wall)`
  (minimum three seconds).
- The tables report best of three where three seeds were run; single-seed and
  deterministic sentinels are marked. `W/T/L` is paired-seed count. Lower cost
  is better.
- Road VRPTW exposes a multi-objective caveat: both fleet and travel cost are
  shown. Cost-only and lexicographic fleet-first conclusions are distinguished.
- VROOM's PDPTW adapter received the same time limit and 10 threads, but VROOM
  often terminated before the limit. Road VROOM values below are the separately
  validated deterministic publication rows; they were not rerun here.

Raw server artifacts:

- `bench/postfix-opposition.jsonl` (committed copy)
- `/home/grechman/postfix-opposition/results.jsonl`
- `/home/grechman/postfix-opposition/raw.log`
- `/home/grechman/postfix-opposition{,2,3}.log`

## Fresh equal-wall results

### Directed road CVRP versus PyVRP

| instance | wall | Commiv best (routes) | PyVRP best (routes) | Commiv advantage | paired W/T/L |
|---|---:|---:|---:|---:|---:|
| Moscow-1000 | 29 s | 204320 (50) | 206726 (50) | 1.178% | 3/0/0 |
| NYC-1000 | 37 s | 91735 (11) | 93070 (11) | 1.455% | 3/0/0 |
| Berlin-1000 | 39 s | 108909 (11) | 109386 (11) | 0.438% | 3/0/0 |
| NYC-2000 | 183 s | 132354 (11) | 132425 (11) | 0.054% | 1/0/2 |

Commiv wins all four best-of-three cells and 10/12 paired seeds. NYC-2000
has substantial seed variance: its Commiv values were 133378, 133558, and
132354, so the narrow best-of-three lead must not be described as a stable
per-seed win.

The deterministic published VROOM costs were 208687, 96774, 116251, and
138341 respectively, 2.137%, 5.493%, 6.741%, and 4.523% above the fresh
Commiv bests.

A secondary Moscow-1000 sentinel used the same fresh 29-second solver budget:

| solver | cost | routes | relative to Commiv |
|---|---:|---:|---:|
| Commiv | 204320 | 50 | - |
| OR-Tools 9.15.6755 | 235079 | 50 | 15.054% worse |
| LKH-3.0.14 ACVRP, feasible warm start | 415634 | 52 | 103.423% worse |

OR-Tools is deterministic here, and the LKH row is a single seed, so these are
sentinels rather than three-seed W/T/L claims. LKH's native strength is TSP;
its much weaker directed-CVRP row must not be used as a claim about its TSP
quality.

### Uchoa X-n1001-k43

Reference cost is 72355. All solvers received 101 seconds after anchoring to
Commiv's fresh mean wall.

| solver | best | gap to reference | relative to Commiv | paired W/T/L for Commiv |
|---|---:|---:|---:|---:|
| Commiv | 72966 | 0.844% | - | - |
| PyVRP | 73370 | 1.403% | 0.554% worse | 3/0/0 |
| HGS-CVRP | 74433 | 2.872% | 2.011% worse | 3/0/0 |

### Directed road VRPTW versus PyVRP

| instance | wall | Commiv cost (fleet) | PyVRP cost (fleet) | cost-only advantage | paired cost W/T/L |
|---|---:|---:|---:|---:|---:|
| Moscow-1000 | 11 s | 230098 (51) | 237541 (52) | 3.235% | 3/0/0 |
| NYC-1000 | 31 s | 133869 (18) | 133865 (20) | -0.003% | 2/0/1 |
| Berlin-1000 | 30 s | 152056 (20) | 152670 (22) | 0.404% | 3/0/0 |

Cost-only paired W/T/L is 8/0/1. With fleet-first lexicographic comparison,
Commiv wins all nine seeds because it uses fewer vehicles in the only
cost-only loss. Against the deterministic published VROOM rows:

| instance | Commiv cost (fleet) | VROOM cost (fleet) | cost difference |
|---|---:|---:|---:|
| Moscow-1000 | 230098 (51) | 239977 (50) | Commiv 4.293% lower, VROOM one fewer vehicle |
| NYC-1000 | 133869 (18) | 144517 (18) | Commiv 7.954% lower at equal fleet |
| Berlin-1000 | 152056 (20) | 162971 (19) | Commiv 7.178% lower, VROOM one fewer vehicle |

### Gehring-Homberger 1000-customer sentinel

The native top-k path applies to all published GH `_10_` cells. On the tight
`c1_10_1` sentinel, the post-fix result exactly reproduces the publication:

| solver | best distance | vehicles | gap to 42478.95 |
|---|---:|---:|---:|
| Commiv | 42486.02 | 100 | 0.017% |
| PyVRP | 42478.98 | 100 | 0.000% |

PyVRP wins all three paired seeds by a very small amount; this known narrow
loss did not flip.

### Li & Lim PDPTW versus VROOM

| instance | wall | Commiv fleet / distance | VROOM fleet / distance | unassigned | outcome |
|---|---:|---:|---:|---:|---|
| lc104 | 10 s | 9 / 864.176 | 9 / 861.956 | 0 | VROOM 0.257% lower |
| lc2_2_3 | 15 s | 6 / 1844.324 | 6 / 1847.364 | 0 | Commiv 0.165% lower |
| lr2_2_7 | 15 s | 3 / 3230.225 | 3 / 3451.028 | 0 | Commiv 6.836% lower |
| lc2_10_1 | 90 s | 30 / 16879.220 | 30 / 16879.220 | 0 | exact tie |
| lr2_10_1 | 90 s | 19 / 45718.585 | 17 / 59693.203 | 62 | Commiv is complete; VROOM is not |

This stratified sample is 3 wins, 1 tie, and 1 loss for Commiv when completion
and fleet are respected. `lc104` was deliberately selected because it is the
published 100-customer complete cell that Commiv lost; the loss persists.
These five cells do not substitute for a fresh rerun of all 352 publication
instances.

### Money objective sentinel

For `lr2_10_1`, with `$140 * vehicles + $0.50 * (distance + duration)` and a
90-second limit:

- Commiv: 19 vehicles, 51564.144 distance, 112480.229 duration = **$84,682.19**.
- VROOM: 26 vehicles, 47663.195 distance, 160313.305 duration = **$107,628.25**.

VROOM costs 27.097% more on this sentinel. Both solutions are complete.

## Quality movement from the publication state

The large native top-k implementation can change canonical candidate ties, so
large road and X rows were not assumed identical. Best-of-three movement was:

| cell | old | post-fix | movement |
|---|---:|---:|---:|
| Moscow-1000 CVRP | 204320 | 204320 | exact |
| NYC-1000 CVRP | 91780 | 91735 | 0.049% better |
| Berlin-1000 CVRP | 109367 | 108909 | 0.419% better |
| NYC-2000 CVRP | 132471 | 132354 | 0.088% better |
| X-n1001-k43 | 72990 | 72966 | 0.033% better |
| Moscow-1000 VRPTW | 229841 / 51 | 230098 / 51 | distance 0.112% worse |
| NYC-1000 VRPTW | 133286 / 19 | 133869 / 18 | one fewer vehicle, distance 0.437% worse |
| Berlin-1000 VRPTW | 152256 / 21 | 152056 / 20 | one fewer vehicle, distance 0.131% better |
| lr2_2_7 PDPTW | 3248.526 | 3230.225 | 0.563% better |

GH `c1_10_1`, `lc104`, both 1000-customer PDPTW sentinels, and the money
sentinel reproduced their old outputs. Most retained changes are exact-work,
allocator-lifetime, or API-copy fixes, so they improve CPU/RSS without implying
quality movement.

## Publication rows not logically affected

No post-fix rerun is needed to attribute a quality change to the retained code
for these families:

- TSP (published Commiv 0.008% mean gap versus LKH-3 0.006%).
- The 19 published ATSP instances (largest is 443; the changed native path is
  selected only at dimension 512+, while the dispatch fix matters around
  dimension 3000). Published Commiv 0.017% versus LKH-3 0.001%.
- Augerat CVRP, ACVRP, road/road-TW n=100, Solomon, and GH `_4_` rows.

Those numbers remain historical publication results, not fresh opposition
measurements. The fresh OR-Tools and LKH-3 rows in this audit are the
Moscow-1000 directed-CVRP sentinels above; the headline LKH TSP comparison
remains historical because the retained changes do not reach that path.
