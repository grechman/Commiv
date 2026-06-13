# commiv handoff — 2026-06-12 (post round 15)

## What we are building (read this first)

A Zig symmetric-TSP solver core that beats LKH-3 on both axes as n grows: same
or better accuracy, and a time advantage that gets ENORMOUS with instance size
(small instances merely need to stay competitive). The core later powers an
HGS-style VRP layer (constraints via distance-matrix manipulation + penalty
hooks; ATSP via the 2n transform at the problem layer — the core stays
symmetric and untouched). 100k+ nodes is explicitly FUTURE scope, not now.

Architecture today: ILS (double-bridge kicks + staleness escalation + guided
LKH-style restarts) over a bounded-LK descent with alpha-nearness candidates;
size-gated merging — IPT verbatim below n=1000 (those trajectories are tuned
and bit-identical since round 10), EAX-lite + elite pool + candidate width 5
at n>=1000; zero-delta plateau drift (2-opt + Or-opt) in the extension phase.

**Hard targets (user, 2026-06-12):** rl11849 under 15 s and fl1577 under 5 s
at current-or-better accuracy (today: 160 s probe and 20 s). Ambitious on
purpose. Watch row: rat575 — gap stuck at 0.089% since round 3, time is fine;
user is 2/5 satisfied there.

**State:** 44/44 tests both modes. Round-15 bench: 15/17 original fixtures
optimal at pinned seed (pr1002 SOLVED 259045), suite ~50 s, no original row
loses to LKH RUNS=1 on both axes; fl1577 22256 < LKH's 22262 at 7x its speed;
rl11849 probe 0.800%/160 s vs LKH optimum in 1287.6 s. Tree uncommitted.

**First step for a new agent:** items 0-2 DONE/COMMITTED; item 3 is CLOSED as a
structural dead end (round 18 — see below + item3.md). NEXT is item 4
(diversity-aware pool) or item 6 (on-the-fly distances, also unblocks the d18512
row + reveals the real item-2/item-6 win). The dominant speed lever (per-move
O(n) applyEdges rebuild) is item-8-shaped and stays FUTURE.

ROUND-18 RESULT — ITEM 3 (voting-freeze) IS DEAD, PROVEN, NOT JUST SUSPECTED:
The round-17 revival plan blamed vote-stream correlation/impurity and hoped a
diverse vote source would fix it. Round 18 MEASURED that and it is wrong.
- **Purity is irrelevant.** Single-run frozen sets are already ~96% pure
  (rat575 8 traps/241, d657 14 traps/350); de-trapping them (freeze only
  frozen∩optimal) makes results WORSE (rat575 6780->6794, d657 48955->48972).
- **A perfectly pure backbone still loses accuracy.** Injecting a 100%-pure
  subset of the KNOWN-OPTIMAL edges as a hard freeze on pr1002 gives 260027-260217
  (~0.4% worse) vs 259045, at 2.7-5.5x speed. The frozen edges are IN the
  optimum, yet freezing them blocks it: LK reaches better tours by temporarily
  breaking-and-rebuilding edges the final tour keeps. Hard-fixing ANY edge
  removes that trajectory. The speed can't convert to accuracy (8x budget still
  floors at 260027).
- **Soft freeze (LKH-style, deprioritize-don't-forbid): built + measured.**
  Marginal speed (lk_nodes is not the dominant cost; tour_rebuilds is) and STILL
  loses accuracy (pr1002 259410 @ recall 0.5).
- **The only positive** — kick-only freeze gives rat575 a real mean gain (12
  seeds: 6782.83 -> 6780.42), but needs continuous freeze (staleness-gating it
  kills the gain) and regresses still-productive d657/pr1002. A plateau-degeneracy
  gate (tied_node_frac>0.7) ships it with no accuracy regression BUT costs ~8%
  time on big high-plateau fl1577, collapsing to a single-fixture overfit for a
  0.037% gain. Not worth a default change.
- **Recommendation:** stop the diverse-vote-source work (items 5/9 won't rescue
  freeze — purity was never the blocker). Freeze subsystem is a DELETE candidate
  for the next cleanup. Round-18 additions (inject_frozen, edge_freeze_soft,
  edge_freeze_stale_window, frozen_edges_out + tools/edgeset.py, tools/plateau.py)
  are default-off measurement scaffolding kept for reproducibility; OFF path
  bit-identical, 44 tests pass. Full record in item3.md.

ROUND-17 RESULTS:
- **Item 2 (delta-maintained tour length): SHIPPED.** Bit-identical (rat575
  6779/459, pr1002 259045, fl1577 22256; all sub-1000 fixtures unchanged; 44
  tests both modes). Wall-clock NEUTRAL in the cached regime (pinned pr1002
  A/B ~12.64 s vs ~12.71 s — lookups are L2/L3 hits, not DRAM misses, so
  cutting per-trial scans doesn't move time). It removes the per-trial O(n)
  scans, which is real budget in the future UNCACHED n>=10k path (item 6). The
  big cached-regime lever (per-move rebuild) is item 8, out of item-2 scope.
- **Item 3 (voting-freeze): BUILT, MEASURED, SHIPPED DEFAULT-OFF.** It is real
  but instance-specific, NOT a general accuracy win — see the do-not-retry
  table. The literal-spec LK-respecting variant is strictly worse everywhere.
  The kick-only variant unlocks rat575 across all 3 seeds (6779/6779/6788 ->
  6776/6777/6777) but regresses d657 (+100) and pr1002 (+363) by the same
  freezing, with no separating threshold. Defaults are the validated kick-only
  m384/f95 config; enable via SolveOptions.enable_edge_freeze for rat575-class
  plateau instances only. [ROUND 18: the "future diverse vote source" hope here
  is DISPROVEN — purity is not the blocker. See round-18 result above.]

ROUND-17 RESULTS:
- **Item 2 (delta-maintained tour length): SHIPPED.** Bit-identical (rat575
  6779/459, pr1002 259045, fl1577 22256; all sub-1000 fixtures unchanged; 44
  tests both modes). Wall-clock NEUTRAL in the cached regime (pinned pr1002
  A/B ~12.64 s vs ~12.71 s — lookups are L2/L3 hits, not DRAM misses, so
  cutting per-trial scans doesn't move time). It removes the per-trial O(n)
  scans, which is real budget in the future UNCACHED n>=10k path (item 6). The
  big cached-regime lever (per-move rebuild) is item 8, out of item-2 scope.
- **Item 3 (voting-freeze): BUILT, MEASURED, SHIPPED DEFAULT-OFF.** It is real
  but instance-specific, NOT a general accuracy win — see the do-not-retry
  table. The literal-spec LK-respecting variant is strictly worse everywhere.
  The kick-only variant unlocks rat575 across all 3 seeds (6779/6779/6788 ->
  6776/6777/6777) but regresses d657 (+100) and pr1002 (+363) by the same
  freezing, with no separating threshold. Defaults are the validated kick-only
  m384/f95 config; enable via SolveOptions.enable_edge_freeze for rat575-class
  plateau instances only. Voting infra is reusable for a future DIVERSE vote
  source (items 5/9) — the kick-correlated stream is the root limitation.

The d18512 fixture is GATED OUT of the always-run bench (1.37 GB matrix =>
~hours at fixed_trials=400); re-enable once item 6 lands. Probe manually:
PROF_PATH=vendor/tsplib/d18512.tsp via commiv-profile. Round-16 probe (seed
12345, EXT 0, width 5): 652023 / 1.051% / 167.6 s.

Gate every change on: rat575 bit-identical canary + the 3-row quick regression
+ full multi-seed bench at seeds {12345,7,99}.

## Quick regression protocol (after EVERY major change)

Run these 3 via commiv-profile (build line in Verification), ~35 s total.
Do NOT include the 20k row here — it is too slow for a tight loop.

| Row | Command env | Expectation |
|---|---|---|
| rat575 | PROF_PATH=vendor/tsplib/rat575.tsp PROF_EXT=4 | BIT-IDENTICAL (len 6779, best_trial 459) — sub-1000 canary; any drift = you broke the IPT side |
| pr1002 | PROF_PATH=vendor/tsplib/pr1002.tsp PROF_EXT=2 | len 259045 at seed 12345, ~12 s — the accuracy headline; must not regress |
| fl1577 | PROF_PATH=vendor/tsplib/fl1577.tsp PROF_EXT=2 | len <= 22262 (beat LKH), time toward the 5 s target (now 20 s) |

Pinned seed for the loop; before declaring a round done, re-check the touched
rows at seeds {12345, 7, 99} (four pinned-seed mirages in rounds 11-15) and
run the full bench: `taskset -c 0 nice -n 10 zig build bench -Doptimize=ReleaseFast`.

## Roadmap (execution order — agreed with user 2026-06-12)

| # | Build | Est. time win | Est. acc win | Key constraint / design note |
|---|---|---|---|---|
| 0 | ✓DONE r16. SETUP: 20k bench row — `curl -sL -o vendor/tsplib/d18512.tsp https://raw.githubusercontent.com/mastqe/tsplib/master/d18512.tsp` (n=18512, EUC_2D, optimum 645238); probe-budget fixture like rl11849 (headline_only, fixed_trials); raise bench/profile max_dimension to 20_000 | none | none | distance matrix would be 1.37 GB on a 7 GB box — run with candidate-based distances or accept the squeeze until item 6 lands. NOTE r16: fixed_trials mirrored rl11849=400, but the matrix squeeze makes that ~hours; the full bench was NOT run with this row. Lower it (or gate on item 6) before enabling in the always-run suite. |
| 1 | ✓DONE r16. Per-trial cost counters (distance lookups, O(n) passes, flip ops, LK node ops; pr1002/fl1577/rl11849) | none (gate for 2, 6, 8) | none | everything below is accepted/rejected against these numbers. See "Item-1 measured counters" below. |
| 2 | ✓DONE r17 (334f860). Delta-maintained tour length. Bit-identical; wall-clock NEUTRAL in the cached regime (the part that ships). Undo-log kicks / killing the per-move rebuild are item-8-shaped (array rep forces O(n)/move via between()/pos[]) and were correctly NOT attempted — out of item-2's zero-acc-risk scope. | NEUTRAL cached / real in uncached n>=10k (item 6) | none | the roadmap's "2-4x" assumed the per-move rebuild dies too; it doesn't here (item 8). |
| 3 | ✗CLOSED r18 — STRUCTURAL DEAD END (proven). Voting-freeze gains nothing general: a 100%-pure backbone still loses accuracy under freeze (LK needs break-and-rebuild of even optimal edges), soft freeze too, and purity was never the blocker (r17's diverse-vote-source plan is void). Only positive is a 0.037% rat575 niche gated by an overfit detector. Delete candidate. | none | none (rat575 niche 0.037%) | full post-mortem in item3.md. Do NOT pursue items 5/9 to "fix" freeze. |
| 4 | Diversity-aware pool replacement (HGS biased fitness: rank by cost + diversity contribution via symdiff; never evict best) | small | medium on long runs / big n | v1 replace-worst WILL clone-collapse at scale; symdiff machinery already computes the metric |
| 5 | Pool-pair crossover restarts (seed occasional trials from the EAX product of two pool members) | none | small-medium | true EAX-GA generational step; machinery exists |
| 6 | On-the-fly distances at n>=10k (drop the big matrix) | 1.5-3x at n>=10k | none | every matrix lookup is a DRAM miss; candidate-distance option half-exists; unblocks the 20k row properly |
| 7 | FUTURE (100k+ tier): sparse ascent / POPMUSIC candidates | kills O(n^2) build at n>=20k | none | prerequisite for 100k+, which is out of scope for now |
| 8 | FUTURE (100k+ tier): B-tree tour representation | ~2-3% at n=11849 — not yet | none | flips are ~3% of trial cost (segment two-level rep active at n>=512); build only when item-1 counters show flips >= 10% (projected ~1e5 nodes) |
| 9 | Multithreaded independent trial streams into one elite pool | ~cores x | medium (best-of-streams) | LAST (user doctrine); converts seed variance into accuracy; LKH is single-threaded |
| 10 | ATSP via Jonker-Volgenant 2n transform (problem.zig layer) | none (costs 2-4x on asymmetric inputs) | none | capability only; deferred until VRP work |

## Item-1 measured counters (round 16, 2026-06-13)

Counters added to `SolveStats` (always-on, like lk_search_nodes): `distance_lookups`,
`tour_length_scans` (full O(n) tour-length recompute), `tour_rebuilds` (full O(n) tour-state
rebuild = every applyEdges move-apply + rebuildState), `flip_ops`/`flip_elements` (segment
reversals). LK node ops stay in `lk_search_nodes`. Reset after the one-time candidate build,
so they measure the trial loop only. `commiv-profile` prints a `cost:` line. Non-perturbing:
rat575 still 6779/best_trial 459, pr1002 259045, fl1577 22256, rl11849 0.800%/156s — all
bit-exact vs round 15.

Pinned seed 12345. Distances are CACHED at every n here (matrix fits), so each lookup is an
L2/L3 hit, not the DRAM miss item 6 attacks — these numbers are *counts*, not cycles.

| Row | n | dist_lookups/trial | length_scans | tour_rebuilds | tour_rebuilds × n | flip_ops | flip_elements | lk_nodes/trial |
|---|---|---|---|---|---|---|---|---|
| rat575 | 575 | 42,315 | 10,154 | 86,108 | 4.95e7 | 809 | 60,532 | 1,174 |
| pr1002 | 1002 | 92,216 | 42,926 | 906,700 | 9.09e8 | 4,529 | 278,264 | 4,207 |
| fl1577 | 1577 | 75,256 | 40,647 | 1,036,650 | 1.63e9 | 4,843 | 614,930 | 1,961 |
| rl11849 | 11849 | 1,846,407 | 32,784 | 698,474 | 8.28e9 | 4,141 | 2,015,171 | 17,848 |

Gate readings (direct from the plan's own thresholds):
- **Item 2 (incremental bookkeeping) is the dominant lever and it grows with n, exactly as
  predicted.** Every accepted LK move pays a full O(n) `applyEdges` rebuild. The O(n)
  rebuild element-work (`tour_rebuilds × n`) is ~5x the distance-lookup count at pr1002 and
  ~11x at rl11849. Killing the full rebuild (delta-maintained next/prev + segment patch
  instead of whole-tour walk) is the main lever for the 15 s / 5 s targets.
- **Item 8 (B-tree tour rep) stays FUTURE — gate NOT met.** `flip_elements` is tiny:
  2.0e6 at rl11849 vs 8.3e9 rebuild element-work (<0.03%), far under the "flips >= 10% of
  trial cost" build trigger. The 2-opt/Or-opt segment reversals are not where the time goes.
- **Item 6 (on-the-fly distances) unverifiable from these rows** — all cached, so lookups are
  cheap hits. The lever only appears when the matrix is dropped (n>=10k uncached); needs the
  forced-uncached measurement item 6 introduces. d18512 single-trial probe (cached, 1.37 GB)
  = 56 s, len 656749 (~1.78%).

## Measured do-not-retry (each cost a round to learn)

| Rejected | Evidence |
|---|---|
| Any stopping rule cutting the n-trial staleness window | improvement gaps heavy-tailed (up to 11x prior max); factor-8 patience cost rat195 4/6->2/6, fl417 5/6->2/6 across seeds |
| EAX merging below n=1000 (any variant: single/multi-ref, gain/smallest-first, adoption tweaks) | reshuffles knife-edge optima, +10% time; IPT verbatim is strictly better there |
| EAX without incumbent adoption, or adoption without staleness resets | loses lin318/rd400/pcb442/u574-class optima |
| Non-splitting-only EAX (no bridge repair) | loses lin318/rd400/pcb442 |
| Candidate width 6 at n>=1000 | strictly worse than both 5 and 8 |
| Base-phase plateau drift at n>=1000 | pinned-seed mirage; fl1577 22264->22537 at seed 7 |
| Pool-sourced kicks | dilutes intensification (round 4) |
| 5-opt enumeration, hash revisited-tour cutoff, ML candidates | rounds 6-10 decisions; see git history + memory |
| Identical-trial-streak convergence signal | guided restarts interleave divergent tours every ~4 trials; max streak ~14 |
| Single-seed gating of any change | four pinned-seed mirages in rounds 11-15; always check seeds {12345, 7, 99} |
| Voting-freeze enabled GLOBALLY / by default (item 3) | r17: helps rat575 (all 3 seeds, 6782->6777 mean) but regresses d657 (+100) and pr1002 (+363); no threshold separates the gain from the regression (when rat575 gains, d657 loses, and vice versa). Shipped default-off as opt-in. |
| Voting-freeze with LK respecting frozen edges (the literal spec) | r17: strictly worse everywhere — over-constrains the descent. min64/frac85 freezes 90% of rat575's edges (515/575), 254k move rejections -> 6836. Only the KICK-ONLY variant (LK full power, perturbation avoids frozen edges) ever beats baseline. |
| Voting-freeze over a kick-correlated / distinct-incumbent stream | r17 suspected, **r18 DISPROVEN as the cause**: a diverse vote source does give a near-pure backbone (16-seed consensus 99.8-100% pure) but it does NOT help — see the structural rows below. Purity was never the blocker. |
| Voting-freeze: ANY diverse/purer vote source (item 3 idea #1, items 5/9) | r18: single-run frozen sets are already ~96% pure; **de-trapping them (freeze only frozen∩optimal) makes results WORSE** (rat575 6780->6794, d657 48955->48972). The traps aren't the problem. Building a diverse vote source is wasted effort. |
| Voting-freeze: hard LK-respect, even of a 100%-PURE backbone | r18: injecting a subset of the KNOWN-OPTIMAL edges as a hard freeze on pr1002 gives 260027-260217 (~0.4% worse) vs 259045, at 2.7-5.5x speed. Structural: LK reaches better tours by temporarily breaking-and-rebuilding edges the final tour keeps; hard-fixing any edge (even optimal) removes that trajectory. 8x budget still floors at 260027 — speed can't convert to accuracy. |
| Voting-freeze: soft freeze (LKH-style, skip search initiation in backbone, never forbid) | r18: built + measured. Marginal speed (lk_search_nodes is not the dominant cost — tour_rebuilds is) and STILL loses accuracy (pr1002 259410 @ recall 0.5) — skipping initiation misses the same break-and-rebuild improvements. |
| Voting-freeze: staleness-gating to protect productive instances | r18: NEUTRALIZES the feature. d657 protected at window>=256 but rat575 reverts to 6779 (gain gone) — the rat575 escape needs CONTINUOUS freeze from early. No single window wins both. |
| Voting-freeze: plateau-degeneracy auto-gate (tied_node_frac) | r18: separates rat575 (0.824) from d657/pr1002 (0.303/0.467) and ships the rat575 +2.5-mean gain with no accuracy regression — BUT fl1577 is high-plateau (0.983) and freeze costs it ~8% TIME for zero gain (hard target <5s). Collapses to a small-AND-plateau single-fixture overfit for 0.037%. Item 3 closed; freeze is a delete candidate. |

## Verification

| What | Command |
|---|---|
| Tests | `zig build test` and `zig build test -Doptimize=ReleaseFast` (44) |
| Full bench (multi-seed headline + rl11849 probe row) | `taskset -c 0 nice -n 10 zig build bench -Doptimize=ReleaseFast` — NEVER run anything else concurrently |
| Single instance | `commiv-profile` via env PROF_PATH/PROF_TRIALS/PROF_EXT/PROF_SEED/PROF_WIDTH(0=auto)/PROF_TOUR_OUT; build: `zig build-exe -O ReleaseFast --name commiv-profile --dep commiv -Mroot=examples/profile.zig --dep build_options -Mcommiv=src/root.zig -Mbuild_options=<stub: pub const with_cgal = false;>` |
| LKH baseline | binary `.zig-cache/lkh-bench/LKH`, par pattern `/tmp/lkh-runs/*.par`, optimal tours `.zig-cache/lkh-tours/*.tour` (incl. rl11849) |
| Canary rule | sub-1000 rows must stay bit-identical (IPT side); lengths reproduce exactly, times are +-15% noise |

Deferred cleanups (do NOT trim mid-build): split solver.zig inline tests,
modularize candidates/EAX/constructions, delete farthestInsertionTour + dead
chain-nonseq bridge path + default-off move-patching machinery, drop CGAL.
