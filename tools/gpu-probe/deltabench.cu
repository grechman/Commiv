// Phase 1 gate: batched 2-opt move-delta evaluation throughput,
// GTX 1660 Ti vs host CPU (1 thread and all threads), per commiv gpu.md.
// Realistic accounting: every GPU batch pays the tour-array upload (SoA
// next/pos) and the best-move readback; the matrix is device-resident.
// Integer objective, deterministic tie-break (packed delta|index atomicMin).
//
// Build: nvcc -O3 -arch=sm_75 -Xcompiler -fopenmp -o deltabench deltabench.cu
// Run:   ./deltabench <n> <k> <batches>
#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <cmath>
#include <chrono>
#include <climits>
#include <algorithm>
#include <vector>
#include <omp.h>

#define CUDA_CHECK(x) do { cudaError_t e = (x); if (e != cudaSuccess) { \
  printf("CUDA_FAIL %s:%d %s\n", __FILE__, __LINE__, cudaGetErrorString(e)); exit(1); } } while (0)

// delta of 2-opt(i, c): replace edges (i,next_i),(c,next_c) with (i,c),(next_i,next_c)
__global__ void deltaKernel(const uint32_t* __restrict__ mat, int n,
                            const int* __restrict__ nxt,
                            const int* __restrict__ cand, int k,
                            unsigned long long* best) {
  int t = blockIdx.x * blockDim.x + threadIdx.x;
  if (t >= n * k) return;
  int i = t / k;
  int c = cand[t];
  if (c == i || c == nxt[i]) return;
  int ni = nxt[i], nc = nxt[c];
  long long delta = (long long)mat[i * n + c] + mat[ni * n + nc]
                  - (long long)mat[i * n + ni] - (long long)mat[c * n + nc];
  // pack: (delta + bias) << 24 | t   — deterministic min by (delta, t)
  unsigned long long key = ((unsigned long long)(delta + (1LL << 30)) << 24) | (unsigned long long)t;
  atomicMin(best, key);
}

static inline long long cpuDelta(const uint32_t* mat, int n, const int* nxt, int i, int c) {
  int ni = nxt[i], nc = nxt[c];
  return (long long)mat[i * n + c] + mat[ni * n + nc]
       - (long long)mat[i * n + ni] - (long long)mat[c * n + nc];
}

int main(int argc, char** argv) {
  int n = argc > 1 ? atoi(argv[1]) : 5000;
  int k = argc > 2 ? atoi(argv[2]) : 16;
  int batches = argc > 3 ? atoi(argv[3]) : 2000;
  printf("n=%d k=%d batches=%d threads_avail=%d\n", n, k, batches, omp_get_max_threads());

  // synth instance: random coords, u32 euclid matrix (row-major n*n)
  srand(42);
  std::vector<float> xs(n), ys(n);
  for (int i = 0; i < n; i++) { xs[i] = rand() % 100000; ys[i] = rand() % 100000; }
  std::vector<uint32_t> mat((size_t)n * n);
  for (int i = 0; i < n; i++)
    for (int j = 0; j < n; j++) {
      float dx = xs[i] - xs[j], dy = ys[i] - ys[j];
      mat[(size_t)i * n + j] = (uint32_t)sqrtf(dx * dx + dy * dy);
    }

  // kNN candidates (realistic access pattern)
  std::vector<int> cand((size_t)n * k);
  {
    std::vector<std::pair<uint32_t,int>> row(n);
    for (int i = 0; i < n; i++) {
      for (int j = 0; j < n; j++) row[j] = { mat[(size_t)i * n + j], j };
      row[i].first = UINT32_MAX;
      std::partial_sort(row.begin(), row.begin() + k, row.end());
      for (int j = 0; j < k; j++) cand[(size_t)i * k + j] = row[j].second;
    }
  }

  // random tour
  std::vector<int> perm(n), nxt(n);
  for (int i = 0; i < n; i++) perm[i] = i;
  for (int i = n - 1; i > 0; i--) { int j = rand() % (i + 1); std::swap(perm[i], perm[j]); }
  for (int i = 0; i < n; i++) nxt[perm[i]] = perm[(i + 1) % n];

  const long long evals_per_batch = (long long)n * k;

  // ---------- CPU single ----------
  volatile long long sink = 0;
  auto t0 = std::chrono::steady_clock::now();
  long long best_cpu = 0;
  for (int b = 0; b < batches; b++) {
    long long best = LLONG_MAX; int bi = -1;
    for (int i = 0; i < n; i++)
      for (int j = 0; j < k; j++) {
        int c = cand[(size_t)i * k + j];
        if (c == i || c == nxt[i]) continue;
        long long d = cpuDelta(mat.data(), n, nxt.data(), i, c);
        long long t = i * k + j;
        if (d < best || (d == best && t < bi)) { best = d; bi = (int)t; }
      }
    sink += best; best_cpu = best;
  }
  double s1 = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();

  // ---------- CPU all threads ----------
  t0 = std::chrono::steady_clock::now();
  for (int b = 0; b < batches; b++) {
    long long best = LLONG_MAX; long long bi = LLONG_MAX;
#pragma omp parallel
    {
      long long lb = LLONG_MAX, lbi = LLONG_MAX;
#pragma omp for nowait
      for (int i = 0; i < n; i++)
        for (int j = 0; j < k; j++) {
          int c = cand[(size_t)i * k + j];
          if (c == i || c == nxt[i]) continue;
          long long d = cpuDelta(mat.data(), n, nxt.data(), i, c);
          long long t = (long long)i * k + j;
          if (d < lb || (d == lb && t < lbi)) { lb = d; lbi = t; }
        }
#pragma omp critical
      { if (lb < best || (lb == best && lbi < bi)) { best = lb; bi = lbi; } }
    }
    sink += best;
  }
  double sm = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();

  // ---------- GPU ----------
  uint32_t *d_mat; int *d_nxt, *d_cand; unsigned long long *d_best;
  CUDA_CHECK(cudaMalloc(&d_mat, (size_t)n * n * 4));
  CUDA_CHECK(cudaMalloc(&d_nxt, n * 4));
  CUDA_CHECK(cudaMalloc(&d_cand, (size_t)n * k * 4));
  CUDA_CHECK(cudaMalloc(&d_best, 8));
  CUDA_CHECK(cudaMemcpy(d_mat, mat.data(), (size_t)n * n * 4, cudaMemcpyHostToDevice));
  CUDA_CHECK(cudaMemcpy(d_cand, cand.data(), (size_t)n * k * 4, cudaMemcpyHostToDevice));
  int tpb = 256, blocks = (int)((evals_per_batch + tpb - 1) / tpb);
  unsigned long long init = ~0ULL, h_best = 0;
  // warmup
  CUDA_CHECK(cudaMemcpy(d_nxt, nxt.data(), n * 4, cudaMemcpyHostToDevice));
  CUDA_CHECK(cudaMemcpy(d_best, &init, 8, cudaMemcpyHostToDevice));
  deltaKernel<<<blocks, tpb>>>(d_mat, n, d_nxt, d_cand, k, d_best);
  CUDA_CHECK(cudaDeviceSynchronize());

  t0 = std::chrono::steady_clock::now();
  for (int b = 0; b < batches; b++) {
    // realistic per-batch cost: upload tour, reset best, kernel, read best back
    CUDA_CHECK(cudaMemcpy(d_nxt, nxt.data(), n * 4, cudaMemcpyHostToDevice));
    CUDA_CHECK(cudaMemcpy(d_best, &init, 8, cudaMemcpyHostToDevice));
    deltaKernel<<<blocks, tpb>>>(d_mat, n, d_nxt, d_cand, k, d_best);
    CUDA_CHECK(cudaMemcpy(&h_best, d_best, 8, cudaMemcpyDeviceToHost));
  }
  double sg = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();

  long long gpu_delta = (long long)(h_best >> 24) - (1LL << 30);
  printf("correctness: cpu_best=%lld gpu_best=%lld %s\n", best_cpu, gpu_delta,
         best_cpu == gpu_delta ? "MATCH" : "MISMATCH");
  double e = (double)evals_per_batch * batches;
  printf("cpu1:  %8.1f Mevals/s  (%.2fs)\n", e / s1 / 1e6, s1);
  printf("cpuN:  %8.1f Mevals/s  (%.2fs)\n", e / sm / 1e6, sm);
  printf("gpu:   %8.1f Mevals/s  (%.2fs)\n", e / sg / 1e6, sg);
  printf("RATIO_GPU_VS_CPUN=%.2f\n", sm / sg);
  printf("(sink=%lld)\n", (long long)sink);
  return 0;
}
