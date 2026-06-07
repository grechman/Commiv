# commiv — Nonsequential Bridge Fix Handoff

**Context:** This document was written at a session boundary. It contains everything needed to implement the rat575 quality fix from scratch. Do not guess at context; it is all here.

---

## Current benchmark state

```
berlin52  alpha-lk        7542  /  7542  0.000%   52ms
eil76     alpha-patch-lk   538  /   538  0.000%   19ms
rat195    alpha-lk        2341  /  2323  0.775%  719ms
lin318    alpha-w24-t4   43424  / 42029  3.319%  4.05s
rat575    alpha-w24-t4    7182  /  6773  6.039%  4.32s
```

berlin52 and eil76 are at optimum. rat195 is acceptable. lin318 and rat575 are the open problems. rat575 is the primary blocker.

---

## What the paper says to do

Source: Helsgaun, "An Effective Implementation of the Lin-Kernighan Traveling Salesman Heuristic", Section 4.3, page 29-30.

The nonsequential move set is defined as:

> any nonfeasible 2-opt move (producing two cycles) followed by any 2- or 3-opt move, which produces a feasible tour (by joining the two cycles);
> any nonfeasible 3-opt move (producing two cycles) followed by any 2-opt move, which produces a feasible tour (by joining the two cycles).

This gives four move types by combination:

| Type | Nonfeasible opening | Feasible close | Total edges |
|------|---------------------|----------------|-------------|
| A    | 2-opt (2 removed)   | 2-opt (2 more) | 4           |
| B    | 2-opt (2 removed)   | 3-opt (3 more) | 5           |
| C    | 3-opt (3 removed)   | 2-opt (2 more) | 5           |
| D    | (original LK nonseq 4-opt, subset of A) | — | 4 |

The paper states (p.30) this search is **not only a post-optimization maneuver** — if an improvement is found, further sequential and nonsequential attempts continue. The search is bounded by the positive gain criterion and candidate sets, so runtime cost is small.

The paper also specifies (p.36, `LinKernighan` pseudocode) that `Gain23()` is called inside the main loop after sequential moves fail, and the loop repeats if it finds improvement. This is exactly the structure in `improveLK` at `src/solver.zig:1791`.

---

## What the code currently does

### `improveLK` — `src/solver.zig:1791`

```zig
fn improveLK(self: *LocalSearch, stats: *SolveStats) !u64 {
    for (0..self.max_passes) |_| {
        self.lk_nodes_this_pass = 0;
        if (self.tour.len >= 256 and self.tour.len < 512 and self.improveGain23Bridge(stats)) { ... }
        if (self.findLKMove(stats)) { ... }
        if (self.improveNonSequential4Opt(stats)) { ... }
        if (self.enable_bounded_three_opt_cleanup and ...) { ... }
        break;
    }
}
```

**Gate at line 1795:** `self.tour.len >= 256 and self.tour.len < 512`

rat575 has 575 nodes. `improveGain23Bridge` **never runs on rat575**. This gate was added as a workaround after ungated probing caused 30-87 second runtimes with zero accepted moves. Do not remove this gate without fixing the underlying probe — see "Rejected approach" below.

### `improveGain23Bridge` — `src/solver.zig:1828`

Standalone sweep. Iterates all `n` nodes as `s1`. For each, tries:
1. Plain 2-edge selected-subtour swap (Type A, closes immediately if gain > 0)
2. `tryGain23ThreeEdgeBridge` — Type C: 3-edge nonfeasible open + 2-opt close (`src/solver.zig:1999`)
3. `tryGain23BridgeGain2Opt` — Type B: 2-edge nonfeasible open + additional 2-opt inside smaller segment (`src/solver.zig:1880`)

These are structurally correct implementations of Types A, B, C. The problem is they run as an O(n·k²) standalone sweep with no chain state, so they have no selectivity and blow up on large instances.

### `improveNonSequential4Opt` — `src/solver.zig:2069`

Enforces strict increasing index order (`j <= i+1`, `k <= j+1`, `l <= k+1`). This means it only finds moves where all four cut points are in strictly forward position order. A well-optimised tour produced by LK almost never has such a move. This function is effectively dead on TSPLIB instances and can be ignored.

### `searchRemoved` — `src/solver.zig:2204`

The main LK search recursion. At each depth it:
1. Tries a direct sequential close if `closing_gain > 0` (line 2227)
2. Calls `tryLKCompletionOracle` (lines 2244-2248) — this tries 2-opt and 3-opt completions **sequentially**, not nonsequential
3. Recurses deeper if `depth < max_lk_depth`
4. Calls `tryNonSequentialBridge` if `depth >= 3` (line 2239)

### `tryNonSequentialBridge` — `src/solver.zig:2390`

This is the anchored nonsequential attempt, called from inside `searchRemoved` with the LK chain state intact. Current implementation:

```
from even (open chain endpoint):
  pick u from candidates (not a tour edge)
  pick v = next[u] or prev[u]
  close with (v → t1)
```

This is **not** a nonfeasible 2-opt + 2-opt close. It is a single-step extension that tries to directly close the chain after adding one more add/remove pair. The structural problem: after adding `(even→u)` and removing `(u→v)`, you have two open ends: the original chain's open end and now `v`. Closing with `(v→t1)` only works if `v` and the chain form a single Hamiltonian path back to `t1`. This is not checked.

### Diagnostic results (added to `SolveStats` at lines 52-55)

```
rat575 alpha-w24-t4, tryNonSequentialBridge:
  depth 3:  17032 attempts, 0 accepts, 20220 gain rejects, 1709 apply rejects
  depth 4: 105264 attempts, 0 accepts, 128013 gain rejects, 41684 apply rejects
```

**Zero accepts at every depth.** Two failure modes:

1. **Gain rejects (majority):** The LK chain accumulated cost by depth 3-4 is large. Adding one more edge `(even→u)` and removing `(u→v)` can't recover enough gain to make `(v→t1)` positive. The single-step close requires too much gain from one remove.

2. **Apply rejects (~40% of gain-passing attempts):** The move passes the gain filter but `testAndApplyNonSequentialMove` rejects it at line 2440. Looking at `planAndApplyMoveInternal` (`src/solver.zig:2609`): when `skip_structurally_impossible_fallback=true` (which `testAndApplyNonSequentialMove` uses via `planAndApplyMoveInternal(..., false, false, true)`), it calls `moveDeltaHasValidEdgeSet` first (line 2621), then `plan.validate` (line 2624). If `plan.component_count != 1` and `skip_structurally_impossible_fallback` is true, it returns false (line 2633). The move is producing disconnected graphs because `(v→t1)` is not the right reconnection edge for the two-component configuration.

---

## Root cause

`tryNonSequentialBridge` does not distinguish between the two cases that arise after breaking `(u→v)`:

**Case α:** `v` and `t1` are in **different components** after the nonfeasible break. Then `(v→t1)` is a valid bridge close — it joins both components. This should work but the gain is usually not recoverable.

**Case β:** `v` and `t1` are in the **same component** after the nonfeasible break. Then `(v→t1)` creates a cycle that doesn't include the whole tour — it produces a disconnected result. This is what generates the 41k apply rejects. These moves cannot close with a single edge; they need a 3-opt close (Type B from the paper).

The code makes no such distinction and tries `(v→t1)` regardless. This is why apply rejects are high and the function never accepts anything.

---

## The fix

The fix is to make `tryNonSequentialBridge` handle both cases correctly, implementing proper Type A and Type B from the paper, anchored to the existing LK chain state.

### Step 1: Add a component membership check

After the gain filter passes for a candidate `(u, v)` pair, before attempting to close, determine which component `t1` is in relative to the two-component graph that would result from:
- removing `(u→v)` from the current tour
- adding `(even→u)` to the open chain

The relevant infrastructure already exists. `MovePlan.analyzeComponents` (`src/solver.zig:162`) does exactly this, and `self.scratch_neighbor0`, `self.scratch_neighbor1`, `self.move_component`, `self.move_component_size`, `self.scratch_seen` are all available in `LocalSearch`. However, calling full `analyzeComponents` per candidate is expensive.

A cheaper check: use `self.pos` to determine whether `v` is reachable from `t1` along the path formed by the chain edges. The chain defines a partial path `t1 → t2 → ... → even`. After breaking `(u→v)` and adding `(even→u)`, the two chain endpoints are `t1` and `v`. If `v` is between `t1` and `even` in tour order (adjusting for chain reversals), the close `(v→t1)` is valid. Otherwise, a 3-opt close is needed.

A simpler conservative approach that avoids the full topological check: use `segmentIsNoMoreThanHalf` (already exists at `src/solver.zig:2061`) or `nodeInCircularSegment` (already exists at `src/solver.zig:1974`) to determine which segment `t1` falls into relative to `v` and `even`. This is the same positional guard used in `improveGain23Bridge` (line 1843).

### Step 2: Type A close (when components are compatible)

When the check passes (Case α — different components), close with `(v→t1)` exactly as now, but only after verifying `(v,t1)` is not already in the added list. This is the current code path; it just needs the component check gating it.

### Step 3: Type B close (when components are same, or Type A fails)

When Case β applies, try a 3-opt close instead. From `v`, enumerate candidates `w` (not a tour edge, not in sequence, not already in add list). Remove `(w→x)` (both choices). Close with `(x→t1)`. The gain condition is:

```
gain_with_removed          // accumulated from the LK chain
- dist(even, u)            // cost of first nonseq add
+ dist(u, v)               // gain from breaking (u,v)
- dist(v, w)               // cost of second nonseq add
+ dist(w, x)               // gain from breaking (w,x)
- dist(x, t1)              // cost of final close
> 0
```

This is exactly what `tryLKCompletion3Opt` does for sequential completions (`src/solver.zig:2304`) but with the nonfeasible break `(even→u, u→v)` already accumulated. The new code should mirror `tryLKCompletion3Opt`'s structure, using `gain_with_removed - dist(even,u) + dist(u,v)` as the incoming gain and then proceeding identically.

### Step 4: Wire the edge arrays correctly

For Type B the edge arrays become:

```zig
// existing chain: removed_a/b[0..depth-1], added_a/b[0..depth-1]
// new entries at depth and depth+1:
self.added_a[depth - 1] = even;   self.added_b[depth - 1] = u;
self.removed_a[depth]   = u;      self.removed_b[depth]   = v;
self.added_a[depth]     = v;      self.added_b[depth]     = w;
self.removed_a[depth+1] = w;      self.removed_b[depth+1] = x;
self.added_a[depth+1]   = x;      self.added_b[depth+1]   = t1;
// call testAndApplyNonSequentialMove(depth+2, depth+2, stats)
```

For Type A the edge arrays are (current code, just needs the component gate):

```zig
self.added_a[depth - 1] = even;   self.added_b[depth - 1] = u;
self.removed_a[depth]   = u;      self.removed_b[depth]   = v;
self.added_a[depth]     = v;      self.added_b[depth]     = t1;
// call testAndApplyNonSequentialMove(depth+1, depth+1, stats)
```

Note: `testAndApplyNonSequentialMove` calls `planAndApplyMoveInternal(..., false, false, true)` — the last `true` is `skip_structurally_impossible_fallback`. This means it will return false silently if the edge set is still topologically broken, without asserting. The move validator (`moveDeltaHasValidEdgeSet`) will catch degree imbalances. Trust it; do not add additional structural checks before the call.

### Step 5: Do not touch the gate at line 1795

Leave `improveGain23Bridge` gated at `>= 256 and < 512`. The fix is entirely inside `tryNonSequentialBridge`, which is called from `searchRemoved` with the chain state. The standalone sweep (`improveGain23Bridge`) is a separate mechanism for smaller instances and has been experimentally confirmed to be net-negative when ungated on rat575.

---

## Rejected approach: lifting the gate with a cap

This was tested. Adding a total-probe cap to `improveGain23Bridge` and lifting the `< 512` gate:

```
rat575 alpha-w24-t4: 7182 / 6.039%  →  7187 / 6.113%   (worse)
lin318 alpha-w24-t4: 43424 / 3.319% →  43945 / 4.559%  (worse)
```

The standalone sweep with current case logic is actively harmful on rat575. The capped sweep degrades both instances. This is because the sweep has no chain state context and produces moves that pass the local gain criterion but break global tour structure. Do not revisit this. The evidence is conclusive.

---

## What not to touch

- **Candidate width** — Coverage is already 100% at alpha width 8+ for rat575. Do not widen.
- **`improveNonSequential4Opt`** — Dead code on TSPLIB. Leave it; removing it changes nothing.
- **`tryLKCompletionOracle`** — This is sequential completion (non-bridge), works correctly for berlin52/eil76. Do not change.
- **CGAL** — Not relevant to this fix.
- **Ascent schedule** — Not relevant to this fix. Candidate quality is not the bottleneck.

---

## Verification procedure

After implementing the fix, run:

```sh
zig build test -Doptimize=ReleaseFast
```

All tests must pass. Then:

```sh
taskset -c 0 nice -n 10 zig build bench -Doptimize=ReleaseFast
```

Expected outcome:
- rat575 gap should drop below 6.039% (current) toward the LKH baseline of 0.015%
- lin318 must not regress from 3.319%
- berlin52 and eil76 must remain at 0.000%
- rat195 must not regress from 0.775%
- Runtime for rat575 must stay under ~10 seconds (current: 4.32s; some increase is acceptable, 30s+ is not)

If rat575 improves but lin318 regresses, the edge array wiring is wrong — check that `depth` indexing in the new Type B close matches exactly what `tryLKCompletion3Opt` uses for its own depth accounting.

If accept rate goes from zero to nonzero but quality does not improve, the component check is letting through Case β moves that should only accept Type B — check that `testAndApplyNonSequentialMove` is being called, not `testAndApplyGain23BridgeMove` (they differ in how they account for depth in stats and in their `planAndApplyMoveInternal` flags).

---

## File index

```
src/solver.zig:36-85      SolveStats — diagnostic bucket fields at lines 52-55
src/solver.zig:1791       improveLK — gate for improveGain23Bridge
src/solver.zig:1828       improveGain23Bridge — standalone sweep (gated, do not change)
src/solver.zig:1880       tryGain23BridgeGain2Opt — Type B in standalone sweep
src/solver.zig:1999       tryGain23ThreeEdgeBridge — Type C in standalone sweep
src/solver.zig:2061       segmentIsNoMoreThanHalf — useful for component position check
src/solver.zig:1974       nodeInCircularSegment — useful for component position check
src/solver.zig:2069       improveNonSequential4Opt — dead code, ignore
src/solver.zig:2138       findLKMove — entry point for sequential LK
src/solver.zig:2204       searchRemoved — calls tryNonSequentialBridge at line 2239
src/solver.zig:2390       tryNonSequentialBridge — THE FUNCTION TO FIX
src/solver.zig:2557       testAndApplyMove — sequential close applier
src/solver.zig:2573       testAndApplyCompletionMove — sequential completion applier
src/solver.zig:2584       testAndApplyGain23BridgeMove — standalone bridge applier
src/solver.zig:2596       testAndApplyNonSequentialMove — USE THIS for the fix
src/solver.zig:2609       planAndApplyMoveInternal — move validator/applier
src/solver.zig:2658       moveDeltaHasValidEdgeSet — degree-balance check
```
