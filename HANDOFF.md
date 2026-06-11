# commiv handoff — 2026-06-12

Working tree state: all changes from rounds 8-10 are **uncommitted** (src/solver.zig,
src/problem.zig, src/tsplib.zig, examples/bench.zig, examples/profile.zig, README.md,
12 new vendor/tsplib fixtures). Tests: 41/41 in debug and ReleaseFast. Final table is
fresh-cache verified and reproduced bit-identically.

## Where we are

Headline mode `alpha-w8-kick`, seed 12345, one core, vs LKH-3.0.13 RUNS=1 on the same
machine: **14/17 TSPLIB fixtures at the known optimum.** Full table in README.md.

- Beat LKH outright: lin318 (LKH missed the optimum), fl417 (3x faster), pcb442 (5x),
  att532 (4x), rat195 (5x), fl1577 (identical tour, 43 s vs 146 s).
- Residual gaps: d657 0.008% (4 units), rat575 0.089% (6 units), pr1002 0.381%.
- pr1002 is the only row losing to LKH on both axes.

## Decisions, each backed by what we measured

1. **Alpha generation rewritten to O(n^2)** (was O(n^2 x depth^2), up to O(n^4) on
   chain MSTs). Evidence: 36 s of fl1577's 38 s candidate build was `maxMstEdgeOnPath`'s
   nested ancestor walk per node pair. Now one BFS per row over MST CSR adjacency
   (`fillTreeBottleneck`/`rowAlphaScore`). Verified bit-identical candidates (same
   lengths AND same search-node counts).

2. **LKH backtracking discipline** (`lk_backtrack_depth`, null = auto): sibling
   alternatives only at chain levels 1-2, first-viable commit below; exhaustive below
   n=400. Evidence: full-width DFS exploded on clustered geometry (fl417 guided
   descents 174k nodes each vs pcb442 33k; long removed edges make the gain bound
   prune nothing); depth-2 everywhere lost rat195/lin318 optima, exhaustive below 400
   restored them. Depth 3 tested: no better. Extension-phase trials always use the
   cheap discipline (lin318 1.20 -> 0.91 s).

3. **Guided restarts (LKH ChooseInitialTour C/D/E) at every size** + IPT merge
   adoption. Evidence: removing the old n<512 gate took u574 0.547% -> optimal and
   fl1577 1.95% -> 0.045%. Divergence budget 12 with 3/4 follow-ref throttle for
   light-descent sizes; full descent + prev-best second reference below n=300.

4. **Stagnation-based trial extension** (`trial_extension_factor`; bench: 4, 2 for
   n>=1000). Evidence: every gapped row was still improving at the `trials = n`
   cutoff (rd400's best tour arrived on its final trial; optimal rows converged by
   trial ~50). Extension took pcb442/att532 to optimal. The window (= n trials of
   staleness) is irreducible insurance: pcb442's optimum arrives mid-extension with
   pre-extension stats strictly staler than lin318's fruitless tail — any rule that
   trims lin318 provably breaks pcb442. Latency-bound callers set factor 0.

5. **Plateau kicks (zero-delta 2-opt drift), extension phase only.** Evidence:
   tour-diff vs LKH optimal tours showed residuals are NOT a missing k-opt move —
   rat575 differs in 67 edges across 59 sections of size <= 2 (pr1002: 91/83);
   degenerate geometries put local optima on cost-equal plateaus where the better
   micro-variant only pays after a neighboring section also changes. Length-preserving
   drift walks the plateau for free; it closed rd400 to optimal. More drift shapes
   (Or-opt/3-opt) are ON HOLD pending EAX (see below).

6. **Dropped permanently, with reasons:**
   - *Hash revisited-tour cutoff*: memory is fine (fixed ~1 MB table), but LKH's
     cutoff fires mid-descent and saves little in our light-descent architecture —
     and it actively fights plateau drift (abandons descents that pass through seen
     tours en route to new ones).
   - *5-opt enumeration*: tour-diff proved there is no missing-move target; high
     complexity; would tax all rows. Out of scope.
   - *ML/neural*: user decision; only credible slot would be learned candidate
     generation (NeuroLKH-style), which stays pluggable if ever revisited.
   - Measured dead knobs (do not retry): lk_max_depth 6-8 (noisy, slower),
     uniform extension factor 4 at n>=1000 (pr1002 +7 s for nothing), guided
     cadence 8 below n=800, divergence cap 24, kick-escalation-through-guided.

7. **Tour representation roadmap (consolidation rule applies):** flat array is right
   up to ~2k nodes (flips are 1-3% of runtime). Beyond: ONE counted B-tree with lazy
   reversal flags, fanout as the dial (B=sqrt(n), height 2 = the classic two-level
   list; B~128, height 3 = millions of nodes). Chosen over splay trees: query-dominant
   workload (10-100 queries per update), splay reads mutate the tree, B-tree reads are
   cache-friendly pure reads with deterministic worst case and reader-friendly
   concurrency. Literature (Fredman/Johnson/McGeoch/Ostheimer 1995): arrays to ~1k,
   two-level to ~1e5, splay only past 1e5-1e6 — and memory latency has worsened since.
   When the B-tree lands it must REPLACE both the flat path and the segment-backed
   TourView fallback (A-and-sometimes-B rule: keep only the general structure).

8. **Next build: EAX-lite (single AB-cycle crossover), replacing IPT.** Rationale:
   the proven failure mode is scattered sections that only pay JOINTLY; IPT by
   construction only swaps independently-shorter contiguous sections; an AB-cycle
   applies interleaved bundles atomically and harvests profit even from equal-length
   plateau-sibling trials (which IPT discards). Subsumption: a contiguous section
   swap is exactly one non-splitting AB-cycle, so EAX-1AB covers IPT's move set —
   after bench parity is verified, DELETE IPT (structure count stays flat).
   Design: build symmetric-difference adjacency trial-vs-incumbent (O(n), degree<=4),
   extract AB-cycles, apply best negative-delta cycle, repair subtours with the
   existing two-component patching, polish boundaries, accept on improvement; same
   gating as IPT today. ~200-300 lines.
   Pipeline roles after it lands: zero-delta drift = plateau SAMPLER (generator),
   EAX = COMBINER. Both stages are needed; they are not redundant.

9. **Expected cost of EAX-lite + drift (estimate, not measured):** drift is already
   in the tree and paid for. EAX-1AB adds an O(n) attempt per merge slot (~10-30 us,
   x500-3000 attempts/run) plus a light polish per win — roughly +2-5% on typical
   rows, worst ~10-15% on extension-heavy big rows (pr1002/fl1577); partially or
   fully offset where earlier optima close the stagnation window sooner (rd400
   pattern). The magnitude-level lead over LKH is not at risk.

10. **Real-life direction** (benchmarks are only a canary): constraints via distance
    matrix manipulation + penalty hooks (the LKH-3 recipe), eventually an HGS-style
    VRP layer with our core optimizing the giant tour. Multithreading only at the
    very end (independent trial streams + crossover merging, default threads=1).
    CGAL: deprecate the dependency (no measured benefit). GEO metric still
    unsupported; the n^2 distance cache must yield to candidate-based distances
    beyond ~10-20k nodes (option already exists).

11. **Deferred cleanups for solver.zig (~5k lines, do NOT trim mid-build):** split
    inline tests (~1.4k lines) into a test module; modularize candidates/IPT(EAX)/
    constructions; delete farthestInsertionTour (near-unreachable), the dead
    chain-nonseq bridge path (zero accepts in every mode), default-off move-patching
    machinery, and the CGAL path when deprecation lands.

## How to verify any change

- `zig build test` and `zig build test -Doptimize=ReleaseFast` (41 tests).
- `taskset -c 0 nice -n 10 zig build bench -Doptimize=ReleaseFast` — compare all 17
  rows against README's table; fresh-cache final runs via `--cache-dir`.
- Single-instance work: `examples/profile.zig` (env: PROF_PATH, PROF_TRIALS,
  PROF_EXT, PROF_DEPTH, PROF_BTDEPTH, PROF_TOUR_OUT; build line in memory notes).
  PROF_TOUR_OUT + the awk edge-diff is how the plateau diagnosis was made.
- LKH baseline: binary at .zig-cache/lkh-bench/LKH, par/logs pattern in /tmp/lkh-runs,
  optimal tours in .zig-cache/lkh-tours/*.tour (also feed bench coverage rows).
- Single fixed seed (12345): treat one-row deltas as noise unless they reproduce.
