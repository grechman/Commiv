# GPU lever — measured verdict (2026-07-13)

Hardware: GTX 1660 Ti 6 GB (sm_75) in WSL2 (driver 560.94, CUDA 12.6), host Ryzen 5
2600X (12 threads). Sources: `tools/gpu-probe/deltabench.cu`, `tools/gpu-probe/chainbench.cu`.
Repro: `nvcc -O3 -arch=sm_75 -Xcompiler -fopenmp -o <x> <x>.cu` then `./deltabench <n> <k>
<batches>` / `./chainbench <n> <k> <iters> <chains>`.

## Verdict: NO-GO on this hardware for this engine. Both gpu.md kernels fail their gates.

### Gate 1 — batched move-delta kernel (gpu.md kernel #2)

Device-resident matrix, per-batch tour upload + best readback (the realistic
integration cost), deterministic packed atomicMin reduction. GPU results byte-match
the CPU best on every run.

| config | cpu 1T | cpu 12T | GPU | GPU vs 12T |
|---|---|---|---|---|
| n=1001 k=16 | 288 Me/s | 929 Me/s | 105 Me/s | **0.11x** |
| n=5000 k=16 | 43 Me/s | 208 Me/s | 333 Me/s | 1.60x |
| n=5000 k=32 | 45 Me/s | 131 Me/s | 518 Me/s | 3.97x |

Fixed per-batch cost (~150 us: WSL kernel launch + 2 uploads + readback) buries the
GPU at n~1000 — the gpu.md acceptance instance itself. It only wins on big dense
sweeps (n>=5000, wide k). But the CPU engine does NOT do dense sweeps: the don't-look
queue (measured 2-4x, dlq memory) evaluates a sparse fraction of n*k per iteration
precisely because dense scanning loses. Integrating this kernel = replacing a smart
sparse scan with a brute dense one that is at best 1.6-4x faster than a dense scan
nobody runs.

### Gate 2 — massively-parallel chains (gpu.md kernel #3, proxy)

One chain per block, Or-opt with O(1) apply (an UPPER bound on SISR chain speed —
real ruin/recreate has far more state and divergence), identical start tours, same
candidate lists both sides.

| chains | GPU aggregate | CPU 12-chain aggregate | ratio | GPU per-chain vs CPU core |
|---|---|---|---|---|
| 96 | 13.4 Mit/s | 65.1 Mit/s | 0.21x | 2.6% |
| 384 | 32.3 Mit/s | 64.1 Mit/s | 0.50x | 1.6% |
| 1536 | 31.9 Mit/s | 57.7 Mit/s | **0.55x** | 0.4% |

The gate was >=5x aggregate; the ceiling measured is 0.55x — the 2600X beats the
1660 Ti outright. Per-chain rates of 0.4-2.6% of a CPU core also kill the quality
model: annealed SISR needs DEEP chains (measured repeatedly: a cold capped run that
fails in 27 s succeeds in 120 s; lc103/lc109 seeds saturate at 2 attractors, so width
does not substitute for depth), and best-of-K width was already a measured dead end
on the CPU (islands verdict: parallelism = speed lever, not accuracy).
An optimistic warp-cooperative rewrite (~8x on the candidate scan) still lands under
the gate on a proxy strictly simpler than SISR.

### Why the pieces stack against GPU here

1. WSL2 adds ~100 us per launch/transfer round-trip; the search loop is inherently a
   round-trip-per-accepted-move workload.
2. The engine's measured wins (DLQ sparsity, deep anneals, GES ejection machinery,
   granular gating) are all sequential, divergent, and sparse — the exact opposite of
   GPU-shaped work.
3. 1660 Ti bandwidth (288 GB/s) only pays on dense coalesced access; kNN candidate
   gathers are random-access.

### What would change the answer

- A data-center GPU + native Linux + a genuinely dense n>=10k workload could clear
  Gate 1's bar — not our regime (real instances are n<=5000 with sparse candidate
  structure), not this hardware.
- A ground-up GPU-native solver design (POPMUSIC-style tiling with on-device
  subproblem populations) is a research project, not a port; nothing measured tonight
  says it beats 12 CPU threads on this card.

**Recommendation: drop the GPU lever for commiv on this hardware. The Ryzen's 12
threads are the compute budget worth engineering against (the parallel wave drivers
already use them well).**
