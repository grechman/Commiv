# Item 3: Voting-Freeze — definitive post-mortem (round 18, 2026-06-13)

Handoff for a fresh model. This supersedes the round-17 revival plan: that plan
was built on a hypothesis (the vote stream is too correlated, so the frozen set
is impure; a diverse vote source would fix it). **Round 18 measured that
hypothesis directly and it is WRONG.** The real reason item 3 cannot be a
general win is structural and deeper. Read this, then `HANDOFF.md`.

## TL;DR — item 3 is structurally dead for general gain

Freezing edges to gain accuracy or free speed does not work, and the reason is
not vote purity. Three things were proven, not suspected:

1. **Purity is irrelevant.** A single-run frozen set is already ~96% pure
   (rat575 8 traps / 241; d657 14 traps / 350). Removing the traps (freezing
   only `frozen ∩ optimal`) makes results **worse**, not better (rat575
   6780→6794, d657 48955→48972). A diverse 16-seed consensus reaches 99.8–100%
   purity, but a pure backbone doesn't help either (below).

2. **A perfectly pure backbone still loses accuracy when frozen.** Injecting a
   100%-pure subset of the *known LKH-optimal* edges as a hard (LK-respecting)
   freeze on pr1002 gives 260027–260217 vs the 259045 baseline — a ~0.4% loss —
   while running 2.7–5.5× faster. The frozen edges are literally in the optimum,
   yet freezing them blocks the answer. Root cause: **LK reaches a better tour
   by temporarily breaking-and-rebuilding edges the final tour will keep.**
   Hard-fixing any edge — even an optimal one — removes that trajectory.

3. **The speed cannot be converted to accuracy.** With the pure-backbone freeze
   active, raising the trial budget 8× (EXT2→EXT16) leaves pr1002 stuck at
   260027 — the frozen set imposes a hard quality floor; more restricted trials
   converge to the restricted optimum, never to 259045.

Soft freeze (the real LKH mechanism: deprioritize search initiation in solved
regions, never forbid a move) was built and measured too: it gives only
marginal speed (lk_search_nodes is NOT the dominant cost — `tour_rebuilds` is,
cf. item-1 counters) and **still loses accuracy** (pr1002 259410 at recall 0.5),
because skipping initiation in backbone regions misses the same break-and-rebuild
improvements.

The only positive effect survives in exactly one niche (below) and is too small
and too overfit-prone to ship as a default.

## The one real positive (and why it still doesn't ship)

Kick-only freeze (LK keeps full power, only the double-bridge avoids frozen
edges) gives **rat575 a real mean improvement**: over 12 seeds, base mean 6782.83
→ 6780.42 (m384/f95) / 6780.08 (m128/f95). At the three bench seeds it's
6779/6779/6788 → 6776/6777/6777. Not a pinned-seed mirage. But:

- The gain is ~2.5 absolute (**0.037%**) on one fixture, still ~7 above the 6773
  optimum. rat575 is a small instance; the mission's value is big-instance time.
- It needs **continuous** freeze from early in the run (staleness-gating it to
  "act only when stuck" kills the gain — stale≥64 reverts rat575 to 6779 —
  because the gain comes from reshaping the whole trajectory, not from
  intensifying on stall).
- Continuous freeze **regresses the still-productive instances**: d657
  48916→49016, pr1002 259045→259408+. These keep improving when frozen-kicks
  starve the moves that reach their optima. rat575 only gains because it is
  budget-saturated early (best@459, flat across EXT1–32) — there is nothing left
  to starve, only a plateau basin to escape.

### The gate that almost works — and its fatal flaw

A static plateau-degeneracy detector (`tied_node_frac` over a node's k-NN
distances; rattled grids on small integer coords tie heavily) separates the
fixtures: rat575 0.824, the 5 small high-plateau fixtures 0.84–1.00, vs the
regressors d657 0.303 and pr1002 0.467. A `tied_node_frac > 0.7` gate would
enable freeze on {rat575 + 5 small} and disable it on {d657, pr1002}. Verified:
pcb442/a280/fl417 are **bit-identical** under freeze, so no accuracy regression.

**But fl1577 is high-plateau (0.983) and freeze is accuracy-neutral there at an
~8% TIME cost** (19.8s→21.5s — the bounded kick redraws). fl1577's hard target is
<5 s, so paying 8% for zero gain is a time regression on a headline instance.
Excluding it forces the gate down to "small AND high-plateau," which is a
transparent overfit to the single rat575 fixture for a 0.037% gain. Not worth a
default change against the codebase's hard-won anti-overfit doctrine.

## Why each revival idea fails (the round-17 list, all resolved)

| Idea | Verdict |
|---|---|
| 1. Diverse vote source → pure backbone | Achievable (99.8–100% purity from 16 diverse seeds) but **purity is not the problem**; a pure injected backbone still loses 0.4% (proof #2). Dead. |
| 2. Speed-budget conversion (freeze→more trials) | Dead. Pure-backbone freeze floors pr1002 at 260027 even at 8× budget (proof #3). |
| 3. Soft freeze (deprioritize, don't forbid) | Built + measured. Marginal speed (wrong cost center — rebuilds, not nodes) and still loses accuracy. Dead. |
| 4. Constructions follow frozen edges | Would freeze *more* from trial 0; static-from-trial-0 injection is already worse than the ramped voted run (rat575 6776→6780/6794). Dead. |
| 5. Confidence-weighted / decaying votes | Moot — changes *which* edges freeze, but freezing any edge loses accuracy structurally. |
| 6. Per-instance auto-gating | Plateau gate works for accuracy but costs time on big high-plateau (fl1577); collapses to a single-fixture overfit. |
| 7. Combiner-side leverage | Already included: the measured pr1002 kick-only regression (+363) was WITH EAX+pool cutting through frozen regions. The combiner does not recover it. Dead. |

## Root cause (the durable lesson)

Lin-Kernighan is a *sequential* edge-swap search. Reaching a better local
optimum routinely passes through intermediate tours that break an edge the final
tour re-adds. This is true even for backbone edges that the global optimum keeps.
**Any hard constraint that forbids breaking an edge — regardless of how certainly
that edge belongs in the optimum — removes reachable trajectories and lowers the
quality ceiling.** Edge-fixing in LKH buys speed at exactly this accuracy cost;
it is a budget knob, not a free lunch, and our trial-extension factor already
controls that trade more cleanly. Freezing only "helps" when the baseline is
already saturated below the optimum on a degenerate plateau (rat575), where
re-aiming perturbation at the contested region escapes a basin — a narrow,
instance-specific effect, not a general mechanism.

## Recommendation

**Close item 3.** Do not pursue the diverse-vote-source work (items 5/9 won't
rescue it — purity was never the blocker). The kick-only freeze stays a
documented, default-off opt-in for anyone who wants the rat575-class plateau
escape and accepts the regression elsewhere. The whole freeze subsystem
(`enable_edge_freeze` and the round-18 variants below) is now a **candidate for
deletion** in the next cleanup pass (alongside the default-off move-patching
machinery the handoff already lists) — it has no path to a net win.

## Code state (round 18) — all default-OFF, OFF path bit-identical, 44 tests pass

Round-18 additions to `src/solver.zig` (`SolveOptions`), built to run the proofs
above; kept as documented-off so the findings are reproducible until the cleanup:

- `inject_frozen: []const u64` — a static frozen backbone (packed `lo<<32|hi`,
  sorted) frozen from trial 0, bypassing the voted set. The upper-bound prober:
  feed it a subset of the optimal edges to measure pure-backbone freezing.
- `edge_freeze_soft: bool` — soft freeze: skip *initiating* LK search from an
  interior-backbone node (both tour edges frozen) instead of forbidding moves
  (`findLKMove`). Measured: marginal speed, still loses accuracy.
- `edge_freeze_stale_window: usize` — only act on the frozen set once the
  incumbent has been stale this many trials (voting always accumulates).
  Measured: neutralizes the feature (protects d657 at ≥256, kills rat575 gain).
- `frozen_edges_out: ?*std.ArrayList(u32)` — dumps the final frozen undirected
  edge set (flattened pairs) for purity analysis.

Analysis tooling (`tools/`, independent of solver internals):
- `edgeset.py` — tour/edge-set overlap: `sweep_k` (consensus purity vs k),
  `consensus`/`emit_consensus`, `purity` (frozen dump vs optimal), `intersect`
  (de-trap), `emit_optimal_subset` (pure backbone of controlled recall).
- `plateau.py` — the `tied_node_frac` plateau-degeneracy detector.

## How to reproduce the key proofs

Build the profile driver (knobs: `PROF_FREEZE`, `PROF_FREEZE_LK`,
`PROF_FREEZE_SOFT`, `PROF_FREEZE_STALE`, `PROF_FREEZE_MINVOTES`,
`PROF_FREEZE_FRAC`, `PROF_FROZEN_IN`, `PROF_FROZEN_OUT`, `PROF_TOUR_OUT`):
```
printf 'pub const with_cgal = false;\n' > /tmp/bo_stub.zig
zig build-exe -O ReleaseFast --name commiv-profile --dep commiv \
  -Mroot=examples/profile.zig --dep build_options -Mcommiv=src/root.zig \
  -Mbuild_options=/tmp/bo_stub.zig
```
Proof #2 (pure backbone still loses): emit a 70%-recall optimal subset and
inject it with LK-respect:
```
python3 tools/edgeset.py emit_optimal_subset /tmp/opt70.txt 0.7 12345 .zig-cache/lkh-tours/pr1002.tour
PROF_PATH=vendor/tsplib/pr1002.tsp PROF_EXT=2 PROF_FREEZE=1 PROF_FREEZE_LK=1 PROF_FROZEN_IN=/tmp/opt70.txt ./commiv-profile  # -> 260027, ~3s
```
Proof #1 (de-trapping hurts): dump the voted frozen set, intersect with optimal,
inject the pure remainder kick-only — it gets worse, not better.
Gate every change on: rat575 OFF-path bit-identical (6779/459) + 44 tests both
modes + full `zig build bench` (d18512 gated out).

## Commits
- `e89460b` items 0+1; `334f860` item 2; `6e3c654` item 3 (original, default-off);
  `322d5b2` round-17 docs. Round-18 = this post-mortem + measurement scaffolding.
