# Benchmark experiment ledger

Append-only results for the Windows server's Ubuntu/WSL2 environment. Baselines are
frozen commit hashes; every solver run used explicit seeds, work budgets, thread
counts, and CPU affinity. Times are seconds unless marked milliseconds.

## Harness self-test

| date | harness commit | identity | sabotage | collision | budget |
|---|---|---|---|---|---|
| 2026-08-18 | `80858dc` | PASS: baseline medians 3682 / 3716 ms (0.92%) and candidate 2913 / 2872 ms (1.42%), both inside the 1.5% half-margin | PASS: doubled work produced 4750 / 4753 ms versus 2872-2913 ms normally | PASS: harness creates no output files | PASS: 20k solver-owned iterations, wall 2.87-3.74 s |
| 2026-09-01 | `887e520` (auregat, Debian 13, Ryzen 5 2600X, Zig 0.16.0, `taskset -c 2`) | PASS: baseline medians 2388 / 2383 ms (0.21%), signature 25 veh / 46264.058 / 153079.687 / 96815.629 on every run | PASS: `--iters 40000` produced 3857 / 3855 ms versus 2383-2388 ms | PASS: harness creates no output files | PASS: 20k solver-owned iterations, wall 2.37-2.55 s |
| 2026-09-02 | `f6be7bb` VRPTW harness `bench/run_fixed.py vrptw` (GH `c1_10_1`, 300k SISR iterations, seed 1, `taskset -c 2`) | PASS: `4312d28` medians 2427 / 2426 / 2423 ms, signature `100 veh / 42504.61` on every run | PASS: `--iters 600000` (at 100k/200k: 1613 vs 952 ms) changes the result and the time | PASS: no output files | PASS: solver-owned iterations, wall 2.4 s |
| 2026-09-02 | `f6be7bb` CVRP harness `bench/run_fixed.py cvrp` (Uchoa `X-n1001-k43`, 600k SISR iterations, one thread, seed 1, `taskset -c 2`) | PASS: `4312d28` medians 2486 / 2487 / 2481 ms, signature `74102 / 43 routes` on every run | PASS: `--iters 400000` at 200k gave 1585 vs 1051 ms and cost 73725 vs 74465 | PASS: no output files | PASS: solver-owned iterations, wall 2.5 s |

Primary harness: `taskset -c 2 python3 bench/run.py --build --runs 5 --iters 20000 --seed 1`.
It rejects any fixed-seed result-signature disagreement before emitting JSON.

## Experiments

| # | lever | hypothesis | commit | baseline | measurements | delta | verdict | notes |
|---:|---|---|---|---|---|---:|---|---|
| 1 | Freeing allocator for PDPTW HTTP solver work | Request arenas retain balanced solver churn until a long request ends | `86f0b61` | `18b3a5f` | `/solve/pdptw` RSS 304444 / 303808 -> 22952 / 22360 KiB; dispatch 66228 / 67092 -> 22032 / 22028 KiB; VROOM 70536 / 70864 -> 22496 / 22512 KiB | RSS -92.6% solve, -67.0% dispatch, -68.2% VROOM | WIN | Exact response SHA in both replicates: solve `ed8a1c...5709`, dispatch `c556b2...19be`, VROOM `ceb429...1993` |
| 2 | Packed retained PDPTW snapshot buffer | Per-route snapshot dupes make arena callers retain O(iterations) allocations | `1f74311` | `18b3a5f` | peak RSS 304444 / 303808 -> 25600 / 26424 KiB | -91.4% | WIN | Isolated cherry-pick; both response SHAs identical to baseline |
| 3 | Free parallel HGS worker workspaces | Worker arenas retain every offspring education | `99a63a5` | `1f74311` | time old `0.73 / 0.67 / 0.68 / 0.69 / 0.69 / 0.65 / 0.67 / 0.68 / 0.67 / 0.68 / 0.67 / 0.69 / 0.68 / 0.71 / 0.69 / 0.71 / 0.68 / 0.71 / 0.7 / 0.69`; new `0.68 / 0.7 / 0.69 / 0.67 / 0.68 / 0.67 / 0.7 / 0.65 / 0.63 / 0.65 / 0.68 / 0.69 / 0.67 / 0.68 / 0.7 / 0.68 / 0.71 / 0.71 / 0.69 / 0.67`; RSS old `34744 / 32504 / 35008 / 33116 / 32240 / 33132 / 33328 / 33300 / 31620 / 34636 / 34944 / 34408 / 32940 / 36120 / 32764 / 31684 / 36004 / 33032 / 33064 / 32612` KiB; new `5540 / 4956 / 3876 / 3180 / 5100 / 4900 / 3132 / 5492 / 5040 / 3396 / 3132 / 3308 / 4908 / 3612 / 4972 / 5164 / 5008 / 5168 / 3136 / 3252` KiB | RSS -85.2%; time +0.7% | WIN | All 40 results `75968/18/a401636f22fd9031` |
| 4 | Large native ATSP dispatch before degeneracy scaling | Large requests must honor `trials`, not silently multiply it by 100 | `386b18c` | `99a63a5` | trials=1 old 57.80 / 58.97; new 1.55 / 1.56; new trials=100 59.26 | 37.4x at requested work | WIN | Equal 100-trial work reproduces exact old cost/hash `4142/1bf8...`; 1-trial result is `4165`, the requested smaller search |
| 5 | Skip duplicate converged 3-opt cleanup | A second unchanged deterministic sweep should save >2% | `1450221` | `386b18c` | old `6.83 / 6.71 / 6.63 / 6.74 / 6.70 / 6.72 / 6.67 / 6.78 / 6.72 / 6.69 / 6.70 / 6.68 / 6.66 / 6.68 / 6.67`; new `7.17 / 6.57 / 6.58 / 6.61 / 6.54 / 6.60 / 6.51 / 6.54 / 6.51 / 6.50 / 6.53 / 6.52 / 6.55 / 6.69 / 6.52` | median +2.45%, mean +1.66%, one regression | DEAD | Below/inconsistent with 2% protocol margin; reverted by `fd16622` |
| 6 | Scalar VRPTW insertion feasibility | Scalar labels should beat two Tws merges by >2% | `1e8b327` VRPTW portion | `1450221` | old `2465.8 / 2467.4 / 2450.4 / 2460.8 / 2458.8 / 2420.1 / 2453.7 / 2487.4 / 2493.4 / 2492.6 / 2484.5 / 2462.3 / 2470.6 / 2491.9 / 2529.8 / 2490.0 / 2473.6 / 2471.6 / 2502.3 / 2472.8` ms; new `2495.1 / 2552.5 / 2478.2 / 2464.1 / 2452.9 / 2497.8 / 2455.2 / 2569.7 / 2520.3 / 2497.6 / 2501.0 / 2469.8 / 2499.9 / 2443.8 / 2516.3 / 2505.3 / 2489.8 / 2506.5 / 2467.4 / 2559.3` ms | -1.0% (regression) | DEAD | Exact route hash, but slower; reverted by `7cc58d4` |
| 7 | Scalar ordinary-PDPTW pair insertion | Avoid four segment merges per gap pair without changing scan/RNG order | `1e8b327` PDPTW portion + `ffa3d5e` | `1450221` | final baseline 3665 / 3658 / 3682 / 3688 / 3706 and 3743 / 3725 / 3666 / 3691 / 3716 ms; final candidate 2913 / 2894 / 2904 / 2924 / 2959 and 2956 / 2902 / 2867 / 2872 / 2869 ms; interleaved old 3659 / 3711 / 3669 / 3654 / 3658, new 2924 / 2989 / 2978 / 2929 / 2876 | -20.9% / -22.7% replicated | WIN | Every objective/fleet/duration/wait tuple identical; HTTP response SHA `c22c10...6d38` identical. Malformed windows explicitly fall back |
| 8 | Zero-copy contiguous NumPy inputs | `tobytes` + `from_buffer_copy` adds two matrix-sized copies | `3b2681d` | `1e8b327` helper | conversion old `76.285 / 47.256 / 82.828 / 50.528 / 49.074 / 85.720 / 48.711 / 85.256` ms; new `0.943 / 0.967 / 1.039 / 0.964 / 0.917 / 0.932 / 0.953 / 1.030` ms; peak old `324288 / 323304 / 323236 / 323464 / 323212 / 323188 / 323208 / 323868` KiB; new `128176 / 127884 / 128352 / 128328 / 128304 / 128324 / 128440 / 128308` KiB | 66.2x conversion; peak -60.3% | WIN | n=5000 uint32; all new pointers share NumPy storage; smoke solution equals list path |
| 9 | Bounded top-k native ATSP candidates | Sorting n-1 entries per row to retain 16 dominates large native startup | `003597b` | `386b18c` | old `1.50 / 1.51 / 1.55 / 1.57 / 1.59 / 1.54 / 1.58 / 1.56 / 1.53 / 1.52 / 1.57 / 1.49 / 1.51 / 1.46 / 1.45 / 1.44 / 1.43 / 1.59 / 1.57 / 1.57`; new `0.73 / 0.70 / 0.73 / 0.73 / 0.75 / 0.76 / 0.71 / 0.74 / 0.69 / 0.72 / 0.70 / 0.71 / 0.74 / 0.60 / 0.63 / 0.63 / 0.59 / 0.75 / 0.75 / 0.75` | 2.12x median | WIN | n=3000 unique-key output byte-identical. 15 real road runs: quality W/T/L 4/7/4, mean +0.049%; mean time 1038 -> 947 ms |
| 10 | Proof-bounded capped Split columns | n route-count columns allocate ~382 MiB even when only K can win | `c272171` | `1450221` Split | old `0.40 / 0.33 / 0.34 / 0.35 / 0.34 / 0.38 / 0.34 / 0.34 / 0.35 / 0.33 / 0.36 / 0.34 / 0.33 / 0.33 / 0.33 / 0.37 / 0.33 / 0.33 / 0.33 / 0.33` s and RSS `489016 / 488900 / 488728 / 487440 / 487596 / 487368 / 488048 / 487368 / 488504 / 487968 / 488332 / 487436 / 488424 / 489080 / 488108 / 487536 / 489080 / 487540 / 487368 / 489016` KiB; new `0.07 / 0.07 / 0.07 / 0.07 / 0.07 / 0.07 / 0.09 / 0.07 / 0.07 / 0.07 / 0.07 / 0.07 / 0.07 / 0.07 / 0.07 / 0.07 / 0.07 / 0.07 / 0.07 / 0.07` s and RSS `137424 / 137424 / 137424 / 137424 / 137424 / 137428 / 137428 / 137424 / 137424 / 137424 / 137424 / 137424 / 137428 / 137424 / 137488 / 137428 / 137428 / 137488 / 137420 / 137420` KiB | 4.86x; RSS -71.8% | WIN | n=5000, K=500; all 40 cost/route/hash results identical; 100-case full-DP differential test |
| 11 | Free parallel VRPTW SISR worker allocations | Worker arenas retain every improved incumbent result | `95d22a2` | `c272171` | old time `2.99 / 2.93 / 2.96 / 2.95 / 2.95 / 2.95 / 2.96 / 3.06 / 2.80 / 2.98`, RSS `56552 / 54664 / 54840 / 54704 / 54280 / 58412 / 58312 / 52888 / 54840 / 59700` KiB; new time `2.88 / 2.96 / 3.18 / 2.89 / 2.91 / 3.00 / 2.93 / 2.93 / 2.92 / 3.04`, RSS `10864 / 10752 / 10884 / 10628 / 10844 / 10900 / 10772 / 10900 / 10908 / 10948` KiB | RSS -80.2%; time +0.9% | WIN | All 20 route hashes/costs identical |
| 12 | Free parallel CVRP SISR worker allocations | CVRP workers may have the same arena-retention problem | `01f2aaa` | `95d22a2` | old `1.59 / 1.69 / 1.54 / 1.59 / 1.60 / 1.59 / 1.57 / 1.52 / 1.51 / 1.50 / 1.51 / 1.54 / 1.60 / 1.51 / 1.62`, RSS median 13380 KiB; new `1.61 / 1.65 / 1.47 / 1.66 / 1.53 / 1.63 / 1.57 / 1.64 / 1.58 / 1.62 / 1.61 / 1.61 / 1.73 / 1.50 / 1.63`, RSS median 13396 KiB | -2.55%, no memory win | DEAD | Reverted by `60bfce2` |
| 13 | Lseg-free money-mode pair insertion | Money mode still pays two Lseg concatenations per candidate gap pair that a running load/peak scalar replaces exactly | not landed | `66d73f5` | interleaved medians of 5, `lr2_10_1` money mode: old `3325 / 3329 / 3281 / 3182` ms, new `3254 / 3225 / 3198 / 3181` ms | -2.14% / -3.12% / -2.53% / -0.03%, mean of medians -1.97% | DEAD | Exact signature (23 veh, 65947.311 dist, 121213.662 dur, 45266.351 wait) on all runs. Below the 2-3% protocol margin and absent in one replicate, so the 70-line third copy of the insertion loop was not kept. The time algebra is NOT removable here: the objective prices the merged route duration, which scalar arrival labels cannot produce, so only the load half of experiment 7 transfers |
| 14 | Quadratic PDPTW seed construction | The due-sorted cheapest-pair-insertion seed rebuilt an O(L) `routeCost` per candidate gap pair (26% of the frozen run in `perf`); prefix departure/load labels plus a latest-arrival suffix label evaluate each (a, b) in O(1) with the same feasibility verdicts, deltas, and first-strict-minimum tie-break | `8dc885d` | `887e520` (auregat) | old 2371 / 2398 / 2382 / 2548 / 2388 and 2451 / 2382 / 2383 / 2381 / 2415 ms; new 1486 / 1511 / 1458 / 1468 / 1480 and 1515 / 1471 / 1496 / 1466 / 1480 ms | -38.0% / -37.9% (medians 2388 / 2383 -> 1480 / 1480) | WIN | Signature identical on all 20 runs (25 veh, 46264.058, 153079.687, 96815.629). Route-for-route differential test against the retained reference construction on 80 random/clocked instances. Reproduce: `taskset -c 2 python3 bench/run.py --build --runs 5 --iters 20000 --seed 1` |
| 15 | Direct Xoshiro blink draw | `rng.float(f64)` per candidate gap pair went through the `std.Random` fill interface (an indirect call plus a byte loop, 4.7% of the frozen run in `perf`); drawing `next()` from the engine's own `DefaultPrng` and applying std's exact f64 mapping keeps the draw sequence bit-identical | `5b6ffe0` | `8dc885d` (auregat) | first pass 1201 / 1259 / 1187 / 1290 / 1364, 1253 / 1187 / 1192 / 1236 / 1193, 1281 / 1247 / 1257 / 1287 / 1311 ms (medians 1259 / 1193 / 1281, spread above margin under load 1.9-2.1 from a parallel agent); interleaved with the `8dc885d` binary: old 1538 / 1502 / 1478, new 1230 / 1239 / 1221 ms medians | -20.0% / -17.5% / -17.4% interleaved (candidate spread 1.5%) | WIN | Signature identical on all 45 runs. Unit test draws 4M values against `std.Random.float(f64)` from the same seeds. Full candidate tree (`3f556d7`) re-measured 1187 / 1206 ms. Reproduce as experiment 14 |

| 16 | PDPTW rebench on auregat, `887e520` vs `4312d28` | Re-measure the previous agent's two levers in both objectives before building on them | `4312d28` | `887e520` | ordinary medians old 2384 / 2401 / 2392 / 2442, new 1250 / 1397 / 1190 / 1250 ms (runs 1190-1611, box otherwise idle; the spread is measurement noise on 1.2 s runs, see the quiet-window rows below); money old 2962 / 2948, new 1855 / 1854 ms | ordinary -42% to -50%; money -37.4% / -37.1% | CONFIRMED | Signatures identical: ordinary 25 veh / 46264.058 / 153079.687 / 96815.629; money 23 veh / 65947.311 / 121213.662 / 45266.351. Money mode is covered by both levers (seed construction and blink draw are objective-independent) |
| 17 | Skip the unread PDPTW suffix load labels | `suf_l` is only read by the general, break and squeeze evaluators; when every pair takes the fast path it need not be built | `f6ccf00` | `4312d28` | interleaved under the money-grid load: old 1227 / 1222, new 1251 / 1196 ms | +2.0% / -2.1% | INCONCLUSIVE alone | Kept only as the prerequisite of 18. The squeeze evaluator still needs the labels: `ensureSufL` rebuilds them on demand (`793e67b`, after the Debug suite caught an index-out-of-bounds in the eject test) |
| 18 | Incremental PDPTW `freshen` | Rebuilding six prefix/suffix arrays per touched route was 13.6% of the tip run; keeping the untouched prefix and shifting the untouched suffix halves the fold | `885695d` | `4312d28` | quiet window, medians of 5, interleaved: ordinary old 1205 / 1396 / 1223, new 1160 / 1146 / 1173 ms; money old 1860 / 1839 / 1842, new 1857 / 1834 / 1829 ms | ordinary -3.7% / -17.9% (old run 2 had three outliers) / -4.1%; money -0.2% / -0.3% / -0.7% | WIN (ordinary, marginal) | 400-edit differential test against a from-scratch rebuild in both objectives; signatures identical |
| 19 | Precompute the dropoff-plus-suffix merges once per route (money) | The general evaluator pays two Tws and two Lseg merges per gap pair; merging q into every suffix once per call leaves one merge per pair | `d74a2ca`, reverted `08bdec4` | `3160154` | money interleaved under grid load: old 1884, new 2150 ms | +14.1% | DEAD | Merge associativity holds bit for bit (test kept, 200k random triples), but the b-loop prunes early: per-call O(L) precompute work exceeds the merges it saves |
| 20 | Transposed distance rows alone | `d(prev_a, p)` and `d(last, q)` walk a matrix column per gap; a transposed copy makes them row reads | `92d7988`, `11a8f58`, `b4c6b76` | `3160154` | `perf stat` under grid load, cycles old 4.62 / 4.73 / 5.12 G, new 5.24 / 4.99 / 5.48 G (instructions -4% to -10%, cache-misses -56%) | +13.3% / +5.5% / +7.0% cycles | DEAD alone | Row `last` is still fetched for the middle-arc lookup, so the copy only adds traffic; superseded by 22 |
| 21 | Route arcs from prefix distances | `d(last, it[b])` inside the b-loop equals `pre_d[b+1] - pre_d[b]` for b > a, an exact integer difference that avoids the random-row fetch | `edf23fc` | `885695d` | quiet: ordinary 1160 / 1161 / 1160 vs 1160 / 1146 / 1173 ms; money 1813 / 1803 / 1797 vs 1857 / 1834 / 1829 ms | ordinary 0.0% / +1.3% / -1.1%; money -2.4% / -1.7% / -1.7% | INCONCLUSIVE alone | Kept as the prerequisite of 22: with it the only remaining random-row reads are the two fixed-target lookups |
| 22 | Transposed rows on top of 21 | With 21 in place the fixed-target lookups are the last column walks; a tiled transposed copy per engine instance turns them into row reads | `8c32bf3` (restores 20 on top of 21) | `edf23fc` | quiet: ordinary 1132 / 1154 / 1134 vs 1160 / 1161 / 1160 ms; money 1640 / 1657 / 1664 vs 1813 / 1803 / 1797 ms | ordinary -2.4% / -0.6% / -2.2%; money -9.5% / -8.1% / -7.4% | WIN (money) | Costs dim^2 x 4 bytes per engine instance (4 MB at 1000 customers, one per parallel fleet-min worker); signatures identical |
| 23 | Suffix arcs from prefix distances | Same trick inside `freshen`'s backward fold | `c5247c0`, reverted `b0b0779` | `edf23fc` | quiet: ordinary 1169 / 1191 / 1181 vs 1160 / 1161 / 1160 ms; money 1791 / 1775 / 1823 vs 1813 / 1803 / 1797 ms | ordinary +0.8% / +2.6% / +1.8%; money -1.2% / -1.6% / +1.4% | DEAD | The prefix fold already fetches each arc once; the second fetch hits cache |
| 24 | VRPTW direct blink draw | The two per-gap `rng.float(f64)` blink draws in SISR recreate go through the generic `std.Random` fill interface, as in experiment 15 | `b233107` (draw shared via `core/blink.zig`, `5dfd4f8`) | `4312d28` | quiet, GH `c1_10_1` 300k iterations: old 2427 / 2426 / 2423, new 2123 / 2114 / 2106 ms | -12.5% / -12.9% / -13.1% | WIN | Signature `100 veh / 42504.61` identical on all 30 runs; 4M-draw equivalence test in `core/blink.zig` |
| 25 | CVRP direct blink draw | Same draw in the CVRP SISR recreate (two per neighbour) | `9be2099` | `4312d28` | quiet, `X-n1001-k43` 600k iterations one thread: old 2486 / 2487 / 2481, new 2048 / 2037 / 2048 ms | -17.6% / -18.1% / -17.5% | WIN | Signature `74102 / 43` identical on all 30 runs |
| 26 | Branch tip `8c32bf3` vs `4312d28`, PDPTW both objectives | Combined effect of 17, 18, 21, 22 | `8c32bf3` | `4312d28` | quiet: ordinary old 1371 / 1222 / 1243, new 1130 / 1138 / 1140 ms; money old 1853 / 1831 / 1859, new 1660 / 1672 / 1682 ms | ordinary -17.6% / -6.9% / -8.3%; money -10.4% / -8.7% / -9.5% | WIN | 52-case REST corpus byte-identical to `887e520`; reproduce with the two commands in the money section below |
| 27 | Share the transposed copy per solve | One dim^2 x 4 B copy per public solve call instead of one per engine instance (fleet-min descents, parallel workers) | `f7bca03` | `8c32bf3` | design change, no timing claim: frozen signatures identical in both objectives; `parallel` test set 9/9; peak RSS via `getrusage`, `lr2_10_1`: old 11348 / 11292 KiB (1 thread), 11428 / 11204 KiB (2-thread fleet-min, 3 s); new 11232 / 11248 KiB, 11240 / 12924 KiB | n/a | KEPT | The copy is 4 MB at 1000 customers and 100 MB at 5000; workers borrow the parent's slice |

## Fixed artifacts

- Windows server: `maxgrechkov@100.123.98.112`, Ubuntu under WSL2, CPU pinned as recorded (gone; rows 1-13).
- auregat: `~/projects/stage-bins/<stage>/commiv-pdptwbench` holds every A/B binary of rows 16-26; `bench/equal-wall-quiet-timing.log` is the raw quiet-window log.
- PDPTW instance: vendored Li & Lim `vendor/pdptw/1000/lr2_10_1.txt`.
- Baseline refs are commit hashes, never moving branches.


## Post-fix opposition audit

| date | candidate | scope | result | artifacts |
|---|---|---|---|---|
| 2026-08-18 | `26c8a1c` | Fresh equal-wall sentinels against PyVRP 0.13.4, HGS-CVRP, VROOM 1.14.0, OR-Tools 9.15.6755, and LKH-3.0.14 | Road CVRP best-of-three 4/4 vs PyVRP (paired 10/0/2); road VRPTW cost-only 2/3 and fleet-first 3/3; X-n1001 wins vs PyVRP/HGS; GH c1_10_1 narrow loss; PDPTW sample 3/1/1; money sentinel 27.097% lower | `bench/OPPOSITION_POSTFIX.md`, `bench/postfix-opposition.jsonl`; server `~/postfix-opposition/raw.log` |

Commiv was pinned to CPUs 0-9 while the main competitor processes were
unpinned. This is conservative for Commiv and valid for freshly anchored
head-to-head quality, but the resulting wall times are not a before/after speed
comparison with the unpinned July campaign.

### Completed full-grid extension

| date | candidate | scope | result | artifacts |
|---|---|---|---|---|
| 2026-08-19 | `26c8a1c` | Remaining large road/road-TW/GH cells plus all 352 academic-money and all 352 ordinary PDPTW cells | Manifest exact: 1,611 rows/1,589 cells. Money: Commiv $10,970,675.92 vs VROOM $12,236,718.41, W/T/L 304/26/22. PDPTW completion+fleet+distance: 283/53/16; VROOM fixed-fleet adapter incomplete on 238 cells. Road CVRP best 6/7 vs PyVRP; road-TW cost best 5/6, fleet-first 4/6. GH family scoreboard withheld because 9/15 new PyVRP rows failed exact schedule validation. | `bench/OPPOSITION_FINAL.md`, `bench/postfix-remaining.jsonl`, `bench/postfix-remaining.cells`, `bench/postfix-final-summary.json`, `bench/postfix-opposition-combined.jsonl` |

The final service reached `CAMPAIGN_COMPLETE` after append-only recovery from
WSL restarts. The full-grid artifact supersedes 14 overlapping phase-1
PDPTW/money rows only in derived summaries; both raw sources remain immutable.
Configured time limits are soft rather than equal realized walls, especially
for money-mode VROOM (maximum 174.4 s at a nominal 90 s).

## Money mode and the ordinary-PDPTW speedup

Experiment 7's 20.9%/22.7% win comes from replacing the time-window algebra
with scalar arrival labels. Money mode (`PB_TIMEPEN=1`) is the one mode that
cannot use it: the merged route duration those labels do not produce IS its
objective, so `evalPairInsert` keeps the full Tws summary and the fast path is
gated off. Verified consequence: money-mode results are bit-identical on
`18b3a5f` (pre-branch main), on `66d73f5`, and with experiment 13 applied -
23 vehicles / 65947.311 distance / 121213.662 duration / 45266.351 wait on
`lr2_10_1` at 20k iterations, seed 1. The 352-cell money grid therefore scores
the same search on main and on this branch.

Money mode costs about 29% more wall than ordinary mode at equal iterations
(3325 vs 2581 ms medians, same instance and budget). Much of that is not
algebra overhead: money consolidates to 23 vehicles where ordinary uses 25, and
longer routes mean more gap pairs per request.

Reproduce either mode with the same harness:

```bash
taskset -c 2 python3 bench/run.py --runs 5 --iters 20000 --seed 1                      # ordinary
taskset -c 2 python3 bench/run.py --runs 5 --iters 20000 --seed 1 --time-pen 1 --veh-pen 280000   # money
```

## Equal-wall quality grids, `887e520` vs `4312d28` (auregat, 2026-09-02)

Runner `bench/equal_wall_runner.py`, scorer `bench/equal_wall_score.py`, both
binaries snapshotted with their sha256 in the first jsonl row. Money cells ran
old and new back to back on one physical core (order alternating), five cores in
parallel, cores 2/8 untouched; wall budgets 10/15/30/45/60/90 s by size.

| grid | cells | new W/T/L | old total | new total | delta | rows |
|---|---:|---|---:|---:|---:|---|
| academic money (`PB_TIMEPEN=1 PB_VEH_PEN=280000`, 1 thread) | 352 | 63/289/0 | $10,975,931.77 | $10,969,375.04 | -0.060% | `bench/equal-wall-money.jsonl` |
| ordinary PDPTW (`PB_FLEET=1 PB_EJECT=1`, 5 threads per side) | 464 (352 instances, seeds 1/2/3 at size 100) | 96/328/40 | 9642 veh / 5,090,509.0 dist | 9639 veh / 5,088,620.5 dist | -3 veh, -0.037% dist | `bench/equal-wall-pdptw.jsonl` |

Per size: 100 1/55/0, 200 7/53/0 (-0.039%), 400 7/51/0 (-0.010%), 600 16/44/0
(-0.048%), 800 15/45/0 (-0.076%), 1000 17/41/0 (-0.071%). Zero losses is
expected: at a fixed seed both binaries follow the same trajectory, so the
faster one only ever gets further along it.

The ordinary PDPTW grid (`PB_FLEET=1 PB_EJECT=1`, `PB_GRAN=2` above size 100,
seeds 1/2/3 at size 100) runs old and new at the same time with `PB_THREADS=5`
on the two SMT halves of cores 0,1,3,4,5, sides alternating per cell. That is
the same total load as the 2026-08-19 protocol (10 threads on 10 CPUs) but a
different search per binary, so the absolute numbers are not comparable with
`OPPOSITION_FINAL.md`; only the old/new pairing is. Per size (W/T/L, veh old -> new, dist old -> new): 100 1/167/0, 1207 -> 1207,
174,256.8 -> 174,252.4; 200 6/51/3, 619 -> 618, 176,634.8 -> 177,138.2; 400
18/32/8, 1149 -> 1148, 406,307.1 -> 405,625.0; 600 25/27/8, 1707 -> 1707,
843,820.2 -> 841,859.8; 800 20/30/10, 2234 -> 2233, 1,408,435.4 -> 1,407,934.5;
1000 26/21/11, 2726 -> 2726, 2,081,054.7 -> 2,081,810.5. Losses appear here
because the parallel fleet-min driver is wall-bound and thread-raced, so the two
binaries do not share a trajectory. The unit was stopped once at 14:04 for the
quiet timing window and resumed at 14:07 from the journal; the cell in flight
(`pdptw/100/lc103/s1`-area) has one duplicated old row, and the scorer takes the
last row per bin. Ran 14:07-18:35 MSK on 2026-09-02.
