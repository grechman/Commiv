# commiv complexity ledger — every step, constants kept

This is the engineering account of where every microsecond and byte goes, and the
honest list of what could be faster. Two rules differ from textbook big-O:

1. **Constants are kept.** `O(2kn)` and `O(kn/2)` are different numbers here — a 4x
   constant is a 4x wall-clock difference and we care.
2. **Every row gets a verdict.** `DEFENDED` = we can argue this step is at or near
   the floor for what it computes. `LEVER {target time, target mem, est. gain}` =
   we cannot defend it; this is a place to attack.

Symbols: `n` customers · `m` non-empty routes · `L̄ = n/m` mean route length ·
`k` granular neighbours (default 20; TSP candidate width `W`, default 24) ·
`c̄` mean customers removed per SISR iteration (default 10) · `I` SISR iterations
(default 300k) · `D` LK depth (default 5) · `T` trials. Bracketed numbers `[...]`
instantiate at the Moscow n=1000 benchmark (m≈51, L̄≈20, k=20, c̄≈10, I=300k).
One matrix entry = `u32` = 4 B; one `Tws` = 32 B; one `usize` = 8 B.

A global constraint on every lever below: **same seed ⇒ same routes** is a product
guarantee. Optimizations that introduce hash-iteration order, atomics races, or
parallel reduction order are rejected regardless of speed.

---

## 1. Shared preprocessing

| step | time | memory | verdict |
|---|---|---|---|
| Instance matrix (given) | — | `4(n+1)²` B [4 MB; n=5000: 100 MB] | DEFENDED: it *is* the input. Any sub-quadratic representation assumes a metric; commiv's contract is arbitrary directed costs. |
| kNN neighbour lists, CVRP/VRPTW (`buildCvrpNeighbors`, `buildNeighbors`) | per customer: `2n` matrix reads + full sort `n·log n` cmp ⇒ total `2n² + n²·log n` [1000²·(2+10) ≈ 1.2·10⁷] | `8nk` B out + `16n` scratch [160 KB] | **LEVER {`2n² + n·(n + k·log k)`, same mem, ~5–10x on this stage at n≥5000}**: we sort all n−1 keys to keep the first k=20. Partial selection (heap or `pdq` partial) removes the `log n` factor. One-time cost, ~0.2 s at n=5000 — attack only when large-n build time matters. |
| TSP candidate graph, dense alpha (`buildCandidates`) | 1-tree ascent: `32 · O(n²)` [3.2·10⁷] | `16nW` B (alpha + ids) [384 KB] | DEFENDED for n<2000: alpha-nearness needs the 1-tree; 32 ascent iterations is the measured knee (fewer loses tour quality, more gains nothing). |
| TSP candidate graph, sparse (n≥2000) | `100 · O(n·pool)` = `O(1000n)` + grid build `O(n)` | same + `O(n)` grid | DEFENDED: replaced the dense `O(n²)` ascent at scale (measured 4–7x build speedup, same accuracy — 2026-06-16); pool=10 is the measured floor. |
| ATSP native candidates (`buildNativeCands`, k=16) | per row: select 16 of n ⇒ `≤16n` per row, `16n²` total [1.6·10⁷] | `8·16n` B [128 KB] | DEFENDED: single pass selection, no sort. |
| Helmholtz-Hodge `conservativeness` | `O(3n²)` (potential fit + curl residual) | `O(8n²)` B f64 copies | DEFENDED: the decomposition reads every arc by definition. Diagnostic tool, not hot path. |

---

## 2. CVRP SISR (`solveCvrpSisr`) — the flagship

Seed (once): native ATSP giant tour, **in-place matrix view** (stride n+1) — zero
matrix copy (this is what took n=5000 from 2 GB/412 s to 211 MB/109 s), then Split
+ education to a local optimum.

| step (per iteration unless noted) | time | memory | verdict |
|---|---|---|---|
| Seed: NN tour + descent + `T` double-bridge restarts (once) | `O(T·n·k_atsp)` + `O(n)` len per trial, k_atsp=16 [T=16 default] | `O(41n)` B ws | DEFENDED: throwaway — SISR ruins it away; quality here is worth ~0.1% at most (measured when the 2n-transform seed was replaced by native: identical final quality). |
| Seed: Split DP + education (once) | Split `O(n·L̄)` [2·10⁴]; education = granular LS to convergence `O(sweeps·n·k)` | `O(16n)` B | DEFENDED: Split examines each (start, extension) pair once; capacity monotonicity prunes exactly the infeasible tail — fewer comparisons would forfeit split optimality. |
| Ruin: non-empty count | `O(m)` scan [51] | 0 | **LEVER {`O(1)`, 0, ~1% of iter}**: maintain the count incrementally (the TW engine already does). Micro. |
| Ruin: `k_s ≤ max(1, 4c̄/(1+l̄s)−1)` anchor strings, string unlink | per string `O(l)`, `l ≤ 10`, E[removed]=`c̄` [≈10 unlinks, O(1) each on the linked rep] | `O(c̄)` undo journal (rprev/rroute) | DEFENDED: unlink is pointer surgery, distance delta tracked incrementally; you cannot remove `c̄` customers in less than `c̄` steps. |
| Recreate: sort removed by demand | `O(c̄·log c̄)` [33 cmp] | in-place | DEFENDED at c̄=10: nanoseconds; hard-first ordering is a measured quality win. |
| Recreate: per customer, gap candidates **adjacent to k neighbours only** — `2k` gap evals, each `O(1)` (2 matrix reads + delta) | `2k·c̄` evals [400], each ~3 matrix reads (the true cost is ~2 cache misses/eval on the 4 MB matrix) | 0 | DEFENDED: this is the SISR paper's granular trick — spatially useless gaps are never touched. The floor per eval is the matrix reads themselves. Cache misses, not arithmetic, are the real unit; a GPU batch (gpu.md) is the only order-of-magnitude left here. |
| Recreate: full-scan fallback (no feasible granular gap) | `O(n + m·L̄)` = `O(2n)` [2000], **rare** (tight-capacity end-game only) | 0 | DEFENDED by rarity; making it never-fire would require global gap indexing costing more than it saves. |
| Recreate: insert | `O(1)` linked splice + `O(1)` incremental distance/load | 0 | DEFENDED: floor. |
| Accept / reject | accept `O(1)`; reject = exact undo `O(c̄ + inserted)` [≈20 splices] | journal above | DEFENDED: undo replays the journal; O(removed) is the information floor for undoing removed work. This took real engineering (bit-identical) — the TW engine hasn't earned it yet, see §3. |
| Best snapshot on improvement | `O(n)` copy × improvement count | `O(16n)` B second solution | DEFENDED-ish: improvements decay geometrically; measured ≪5% of wall. |
| **Total per iteration** | `≈ O(m + c̄(2k + l))` [~450 ops, ~8–11 µs measured incl. cache misses] | live set `O(48n)` B + matrix | Wall at n=1000: 300k iters ≈ 2–3.4 s. |
| Parallel best-of-K | K independent chains, `×K` total compute, wall ≈ 1 chain | `×K` live set (arena each) | DEFENDED as a speed/quality lever (independent seeds measured ≥ cooperative schemes, which were deleted). **LEVER {EAX recombination between chains, +mem `O(n·K)`, measured +0.2–0.5% quality on TSP}** — ported to CVRP/TW chains it is untested. |

---

## 3. VRPTW SISR (`solveVrptwSisr`) — new, deliberately simpler than §2

Same skeleton; the differences are exactly where the levers live. Routes are plain
arrays (not the linked rep), rollback is route snapshots (not an exact journal).

| step (per iteration unless noted) | time | memory | verdict |
|---|---|---|---|
| Seed: native ATSP view + `splitDpTw` (once) | Split `O(n·L̄_ext)`, extension stops at first TW/cap break [≈2·10⁴] | `O(16n)` B | DEFENDED: same argument as CVRP Split; TW monotonicity does the pruning. |
| Singleton feasibility precheck (once) | `O(2n)` matrix reads | 0 | DEFENDED: it is the certificate that recreate's fallback can never fail. |
| Ruin: fleet-min branch (veh_penalty>0, rate 0.1) | `O(slots)` smallest-route scan [~52] + empty it `O(5·L̄)` | snapshot `O(L̄)` | **LEVER {`O(log m)` via size-indexed heap, +`O(m)` mem, ~1%}**: micro; scan only runs on 10% of iterations. |
| Ruin: string removal (`removeString`) | per string: `replaceRange` memmove `O(L̄)` + loc tail fix `O(L̄)` + `arcSum` recompute `O(L̄)` + load resum `O(L̄)` + first-touch snapshot `O(L̄)` ⇒ **`O(5L̄)` [~100] where CVRP pays `O(l)` [~5]** | snapshot bytes `8L̄`/route | **LEVER {`O(L̄)` total (memmove is unavoidable in array rep; make dist/load/loc incremental deltas), same mem, ~2–3x on the mutation path ≈ 15–25% of iteration}**. The honest defense of the array rep itself: L̄≈20 fits two cache lines; the CVRP linked rep saves big-O but loses locality — measured wall/iter is within 1.5x of CVRP despite the 5x op count, so the rep stays until profiling says otherwise. |
| Recreate: candidate set = **all gaps of every distinct neighbour route** — `r_c ≤ k` routes × `(L_r+1)` gaps | `Σ(L_r+1) ≈ r_c·(L̄+1)` evals [~250, vs CVRP's 40], each `O(1)` = 2 Tws merges (~20 flops) + 3 matrix reads | 0 | **LEVER {CVRP-style neighbour-adjacent gaps only: `2k` evals [40], 0 mem, ~3–6x fewer evals ≈ 30–50% of iteration} — UNMEASURED QUALITY RISK**: time windows make spatially-adjacent gaps infeasible more often than in CVRP, so the wider scan may be *why* the quality is this good. Measure before taking. This is the single biggest throughput lever in the engine. |
| Recreate: `freshen` prefix+suffix Tws per dirty candidate route | `O(2L̄)` merges [40 merges ≈ 800 flops], amortized over the customers that reuse it | `2·32·(L̄+1)` B/route ≈ `64n/m·m = 64n` B total [64 KB] | **LEVER {suffix-only rebuild from the mutation gap: `O(L̄−g)`, same mem, ÷2 on this line}**; full prefix reuse needs mutation-point tracking. Defended down to `O(L̄)`: after an arbitrary splice the downstream slacks genuinely change. |
| Recreate: empty-slot scan | `O(slots)` [52] per removed customer | 0 | **LEVER {maintain a free-slot stack: `O(1)`, `O(m)` mem, ~5–8% of iteration}**. No defense — this is lazy code. |
| Recreate: insert | `O(4L̄)` (memmove + loc + arcSum + load) [~80] | 0 | Same lever as removeString above. |
| Reject: rollback | restore each touched route `O(5L̄)` [~2–3 routes ⇒ ~300] | snapshots | **LEVER {CVRP-style exact undo journal: `O(c̄ + inserted)` [~20], drop snapshots entirely, ~2x mutation path}** — high effort: the CVRP journal took a dedicated session to make bit-exact. Take only after the cheap levers. |
| **Total per iteration** | `≈ O(c̄·(r_c·L̄ + 9L̄) + slots)` [~3–4k matrix/Tws ops, ~12 µs measured] | live `O(64n + 8nk)` B + matrix | Wall at n=1000: 300k ≈ 3.5 s. The eval line dominates: ~65%. |
| Parallel best-of-K | as CVRP §2 | as CVRP §2 | Same recombination lever, untested for TW. |

**Ceiling estimate if all TW levers land** (adjacent-gaps + incremental deltas +
free-slot stack, quality permitting): per-iteration ~4–6x fewer ops ⇒ n=1000 in
**~0.6–1 s** single-threaded, or ~3x more iterations at the current 3.5 s.

---

## 4. TSP / ATSP core

| step | time | memory | verdict |
|---|---|---|---|
| ATSP JV 2n-transform (`solveAtsp`) | build `O((2n)²)` writes | **`32n²` B** ((2n)² u32, then `initFullMatrix` dupes it) [n=1000: 32 MB; n=5000: 800 MB] | **LEVER {`0` extra via a virtual matrix view over the original + big-M arithmetic in the oracle, ~none time, all of the 32n² B}**: the CVRP seed already dodged this with `solveAtspNativeView`; the general `solveAtsp` entry still materializes. Defensible only below n≈2000. |
| ATSP native descent (Or-opt + directed 2-opt over k=16 candidates) | per pass `O(16n)`; double-bridge `O(1)`; `nativeLen` `O(n)` per trial | `O(41n)` B | DEFENDED: candidate-restricted directed moves; the `O(n)` re-length per trial is ≪ descent cost. |
| LK move step (symmetric core) | per attempt `O(D·W)` bounded by backtrack discipline (sibling alternatives only at depth ≤ 3) [≈5·24 with early exits] | `O(n)` flags | DEFENDED: depth/width/backtracking are the measured LKH-parity knobs; narrowing any of them loses tours (measured repeatedly, HANDOFF do-not-retry list). |
| Don't-look queue sweep | `O(active·W·D)`, active decays to ~0 at convergence | `O(n)` queue | DEFENDED: this replaced full `O(n)` rescans (2–4x measured win, 2026-06-21). |
| Accepted-move tour update | segment flip `O(seg)` + **re-canonicalization `O(n)` RETRACE** | `O(n)` | **LEVER {two-level doubly-linked list: `O(√n)` per accepted move, +`O(n)` mem, bounded}**: the honest history — 85% of rebuilds were doomed-multicomponent fallbacks and were eliminated (-22/-28/-31% wall, 2026-06-15); what remains is only the accepted-move path (measured 8k–134k events/run). Deferred because the bit-identical rewrite risk exceeded the residual gain. Still the only real serial TSP lever left. |
| Recombination (EAX merge, parallel islands) | `O(n)` per merge | `O(n·K)` | DEFENDED: measured +accuracy at equal wall (rl11849 0.724→0.575%); islands are the accuracy-positive parallel mode, cooperative schemes measured worse and were deleted. |

---

## 5. HGS (CVRP n≲500, VRPTW legacy education) — regime-bound, so kept coarse

| step | time | memory | verdict |
|---|---|---|---|
| Per generation: λ=40 offspring × (OX `O(n)` + Split `O(n·L̄)` + education LS `O(sweeps·n·k)`) | education dominates | population `O((μ+λ)·n)` = `O(65n)` | DEFENDED in its regime: HGS beats SISR below n≈250 (measured; ACVRP small-n 0.044% vs 0.361%). Above it, SISR is the answer — that *was* the lever and it's taken. |
| Legacy VRPTW ILS (`solveVrptw`) | `restarts·rounds` full LS convergences ⇒ `O(10·100·sweeps·n·k)` | `O(80n)` | Superseded: kept only for Solomon reproducibility and as a second-opinion engine. Its lever was §3, taken (988 s → 3.5 s). |

---

## 6. Transport layer

| step | time | memory | verdict |
|---|---|---|---|
| REST: JSON body parse (`parseFromSliceLeaky`) | `O(body bytes)`, ~10 ns/byte [n=1000 ⇒ ~8 MB ⇒ ~0.1 s] | arena ≈ 2–3× body [~20 MB] | **LEVER {binary or `.road` upload endpoint: ÷5–10 parse time and ÷3 peak mem; matters from n≈2000 (n=5000 JSON ≈ 175 MB)}**. Below that, parse ≪ solve: defended by proportion. |
| REST: flatten rows | `O((n+1)²)` memcpy | `4(n+1)²` B | DEFENDED: one pass, unavoidable row-pointer indirection removal. |
| C ABI in/out | in: zero-copy pointers; out: `O(n)` flatten | out `≈16n` B | DEFENDED: floor. |
| Python binding, list path | `O((n+1)²)` *Python-object* conversion [n=1000: ~0.3 s!] | transient list of 10⁶ PyInts | **LEVER {use the numpy path: `O((n+1)²)` memcpy, ~4 ms — already implemented}**: documented; the lever is the user reading the docs. |

---

## 7. The ranked lever table (what to attack next)

| # | lever | now → target (time) | mem | est. gain | risk / cost |
|---|---|---|---|---|---|
| 1 | TW recreate: neighbour-adjacent gaps instead of whole neighbour routes | `c̄·r_c·(L̄+1)` [2500] → `2k·c̄` [400] evals/iter | 0 | 2–4x TW iteration throughput ⇒ same wall buys 2–4x iterations | **quality unmeasured** — windows may need the wide scan; gate on Solomon + Moscow like split-string was |
| 2 | TW mutation path: incremental dist/load (+ free-slot stack, nonempty counters) | `O(5L̄)` → `O(L̄)` per mutation; scans → `O(1)` | +`O(m)` | ~25–35% TW iteration | low; mechanical |
| 3 | GPU batched delta evals (gpu.md spec exists) | `2k·c̄` serial cache misses → device-resident batch | matrix on device (`4n²` B) | ~10x at n≥2000, both CVRP and TW | high (no GPU in dev env); the only order-of-magnitude on the board |
| 4 | TW rollback: snapshots → exact undo journal | `O(5L̄·touched)` → `O(c̄)` | −snapshots | ~10–15% TW iteration | high effort (CVRP precedent: a full session for bit-exactness) |
| 5 | Chain recombination (EAX-style) for CVRP/TW parallel | quality, not time | +`O(nK)` | +0.2–0.5% quality at equal wall (TSP-measured) | medium; untested off-TSP |
| 6 | kNN build partial-selection | `n²·log n` → `n² + nk·log k` | 0 | ~10x of a one-time stage; visible at n≥5000 | trivial |
| 7 | Two-level list for LK accepted moves | `O(n)` → `O(√n)` per accepted move | +`O(n)` | bounded: 8k–134k events/run remain | high (bit-identical hot-path rewrite); full handoff in plans/commiv/14.md |
| 8 | REST binary matrix endpoint | ÷5–10 parse, ÷3 peak mem | — | matters at n≥2000 only | low |
| 9 | General-ATSP JV transform → in-place view | −`32n²` B | −32n² B | memory only; n≥2000 ATSP users | low |
| 10 | TW suffix-only freshen | `O(2L̄)` → `O(L̄−g)` | 0 | ~5% TW iteration | low |

Dead ends already paid for (do not re-attack): SMD move descriptors, cooperative
parallelism, decomposition at large n, adaptive candidate re-ranking, edge freezing,
AP-bound early stop, split-string for TW (measured +0.8% worse), hash tour cutoff.
Receipts in README "What we tried and rejected" and the memory files.
