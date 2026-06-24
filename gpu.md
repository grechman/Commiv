# GPU acceleration lever — task spec

Status: NOT STARTED. Needs hardware (CUDA/ROCm device) not present in the dev env, so
it could not be built or measured here. This is a self-contained spec for a developer
with a GPU box.

## Why GPU could help (the mathematical case)

The engine is **compute-bound on distance operations**, measured twice (allocation is
<1% of ACVRP time; the matrix is cache-local; on-the-fly distance was ~11% SLOWER than
the matrix because access is already cache-resident). The hot inner loops are:

1. **Move-delta evaluation.** For each customer/city `c` and each of its `k` candidate
   neighbours, evaluate the cost delta of a 2-opt / Or-opt / relocate move. This is
   `O(n*k)` independent additions/subtractions of matrix entries per sweep — embarrassingly
   parallel, no data dependence between candidate deltas.
2. **Best-of-K restarts.** The accuracy lever that already works (best-of-K parallel SISR
   /HGS/ATSP) is independent stochastic chains. On a CPU we run `cores-1` of them. A GPU
   runs **thousands** of lightweight chains, so best-of-2000 instead of best-of-3.

Both map to the GPU's strength: thousands of cores doing the same arithmetic over a
device-resident distance matrix. Amdahl bound: the fraction that is move-delta arithmetic
is the part that parallelises; the sequential accept/undo and tour bookkeeping do not, so
the win is largest where the search spends most of its time in delta evaluation (large n,
dense candidate sets).

## What to build, and where

**New module `src/gpu/` (backend) + a host orchestrator.** Zig's native GPU codegen
(SPIR-V/PTX) is immature; the pragmatic path is a CUDA (or HIP) kernel in C compiled
separately and called over Zig's C FFI (`@cImport`), or a Vulkan-compute backend if you
want vendor independence. Keep it behind a build flag (`-Dgpu=true`) and a runtime gate so
the CPU path stays the default and the tests keep passing without a device.

Concrete pieces:

1. **Device-resident distance matrix.** Upload the `(n+1)*(n+1)` u32 matrix once per solve;
   never transfer it per iteration. For large n where the matrix doesn't fit, tile it or
   keep candidate-only sub-rows (the candidate list is `n*k`, far smaller).
   Where it plugs in: alongside `CvrpInstance.matrix` / the ATSP transform matrix.

2. **Batched move-delta kernel.** Input: current tour as SoA arrays (next/prev/position),
   the candidate list (`n*k`), the matrix handle. Output: the best improving move per city
   (delta + the two edges). One thread per (city, candidate) pair; segmented reduction to
   the best per city, then a global reduction to the best move. This replaces the CPU
   `improveAt` scan in `src/vrp.zig` (CVRP linked-rep) and the LK candidate scan in
   `src/search.zig` (TSP core). NOTE the CVRP rep is a pointer-chasing linked list, which is
   GPU-hostile — you must mirror the tour into an **SoA array layout** for the kernel and
   reconcile after the accepted move. The accept/apply stays on the host (it's sequential).

3. **Massively-parallel best-of-K.** Run `B` independent SISR chains as `B` GPU blocks, each
   with its own RNG seed (`seed +% blockIdx`), its own working tour in shared/global memory,
   the shared read-only matrix. Periodically copy out the best chain's cost; return the best
   tour. This is the highest-ceiling use — it parallelises the whole chain, not just the
   delta scan, so it avoids the host round-trip per move. Reference CPU version:
   `solveCvrpSisrParallel` in `src/vrp.zig`.

## Acceptance / how to measure

- Correctness: GPU result must be a valid tour (`commiv.vrp.validate`) with cost == reported.
- Speed: compare GPU best-of-B vs CPU best-of-(cores-1) at **equal wall-clock** on large
  instances (X-n1001 and a synthesised n>=5000) — report gap and wall. The bench harness is
  `examples/cvrpbench.zig` (CB_* env). Add a `CB_GPU=1` path.
- The honest bar: GPU only wins where the move-delta arithmetic dominates. On small
  instances (ACVRP n<=70, Augerat) host-device transfer overhead will lose — gate by n and
  say so.

## Known risks / caveats

- **Transfer overhead** dominates at small n; this is a large-n-only lever.
- **Pointer-chasing tour reps don't vectorise** — the SoA mirror + reconcile is real work and
  a real source of bugs; validate every kernel output against the CPU path during bring-up.
- **Determinism**: GPU float/int reduction order can vary; keep the objective integer (it
  already is) and define a deterministic tie-break in the reduction so results are reproducible.
- **No device in CI** — keep the whole thing behind `-Dgpu` so `zig build test` stays green
  on CPU-only machines.

## Files to touch

- `build.zig` — `-Dgpu` flag, link the CUDA/HIP objects, a `gpu` build step.
- `src/gpu/` — new: kernels (`.cu`/`.comp`) + the Zig FFI wrapper.
- `src/vrp.zig` — gate `solveCvrpSisrParallel` to dispatch to the GPU backend when enabled.
- `examples/cvrpbench.zig` — `CB_GPU` path for measurement.
- `src/root.zig` — export the GPU entry points.
