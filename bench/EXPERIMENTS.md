# Benchmark experiment ledger

Append-only results for the Windows server's Ubuntu/WSL2 environment. Baselines are
frozen commit hashes; every solver run used explicit seeds, work budgets, thread
counts, and CPU affinity. Times are seconds unless marked milliseconds.

## Harness self-test

| date | harness commit | identity | sabotage | collision | budget |
|---|---|---|---|---|---|
| 2026-08-18 | `80858dc` | PASS: baseline medians 3682 / 3716 ms (0.92%) and candidate 2913 / 2872 ms (1.42%), both inside the 1.5% half-margin | PASS: doubled work produced 4750 / 4753 ms versus 2872-2913 ms normally | PASS: harness creates no output files | PASS: 20k solver-owned iterations, wall 2.87-3.74 s |

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

## Fixed artifacts

- Windows server: `maxgrechkov@100.123.98.112`, Ubuntu under WSL2, CPU pinned as recorded.
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
