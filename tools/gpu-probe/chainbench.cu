// Phase 2 gate: independent local-search chains on-device vs host.
// Proxy chain = Or-opt (single-node relocate to best of k candidate gaps),
// O(1) apply, no segment reversal — an upper bound on what a real
// ruin-recreate chain would sustain (SISR has MORE divergence and state).
// Each GPU block runs one chain over its private tour in global memory.
// Measures per-chain and aggregate applied-iterations/second.
//
// Build: nvcc -O3 -arch=sm_75 -Xcompiler -fopenmp -o chainbench chainbench.cu
// Run:   ./chainbench <n> <k> <iters_per_chain> <chains>
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

__device__ __host__ static inline uint64_t xorshift(uint64_t* s) {
  uint64_t x = *s; x ^= x << 13; x ^= x >> 7; x ^= x << 17; *s = x; return x;
}

// One chain per block, thread 0 does the sequential chain (worst honest case:
// the chain logic is sequential; other threads idle like a real port's
// divergent warps would). Or-opt: remove node v, reinsert after best of its
// k candidates; accept if improving.
__global__ void chainKernel(const uint32_t* __restrict__ mat, int n,
                            const int* __restrict__ cand, int k,
                            int* nxts, int* prvs, long long* costs,
                            long long iters, uint64_t seed) {
  if (threadIdx.x != 0) return;
  int cid = blockIdx.x;
  int* nxt = nxts + (size_t)cid * n;
  int* prv = prvs + (size_t)cid * n;
  uint64_t rng = seed + cid * 0x9E3779B97F4A7C15ULL;
  long long cost = costs[cid];
  for (long long it = 0; it < iters; it++) {
    int v = 1 + (int)(xorshift(&rng) % (n - 1));
    int pv = prv[v], nv = nxt[v];
    long long rem = (long long)mat[pv * n + v] + mat[v * n + nv] - mat[pv * n + nv];
    long long best = 0; int bestafter = -1;
    for (int j = 0; j < k; j++) {
      int a = cand[(size_t)v * k + j];
      if (a == v || a == pv) continue;
      int na = nxt[a];
      if (na == v) continue;
      long long add = (long long)mat[a * n + v] + mat[v * n + na] - mat[a * n + na];
      long long delta = add - rem;
      if (delta < best) { best = delta; bestafter = a; }
    }
    if (bestafter >= 0) {
      nxt[pv] = nv; prv[nv] = pv;
      int na = nxt[bestafter];
      nxt[bestafter] = v; prv[v] = bestafter;
      nxt[v] = na; prv[na] = v;
      cost += best;
    }
  }
  costs[cid] = cost;
}

static void cpuChain(const uint32_t* mat, int n, const int* cand, int k,
                     int* nxt, int* prv, long long* cost, long long iters, uint64_t seed) {
  uint64_t rng = seed;
  long long c = *cost;
  for (long long it = 0; it < iters; it++) {
    int v = 1 + (int)(xorshift(&rng) % (n - 1));
    int pv = prv[v], nv = nxt[v];
    long long rem = (long long)mat[(size_t)pv * n + v] + mat[(size_t)v * n + nv] - mat[(size_t)pv * n + nv];
    long long best = 0; int bestafter = -1;
    for (int j = 0; j < k; j++) {
      int a = cand[(size_t)v * k + j];
      if (a == v || a == pv) continue;
      int na = nxt[a];
      if (na == v) continue;
      long long add = (long long)mat[(size_t)a * n + v] + mat[(size_t)v * n + na] - mat[(size_t)a * n + na];
      long long delta = add - rem;
      if (delta < best) { best = delta; bestafter = a; }
    }
    if (bestafter >= 0) {
      nxt[pv] = nv; prv[nv] = pv;
      int na = nxt[bestafter];
      nxt[bestafter] = v; prv[v] = bestafter;
      nxt[v] = na; prv[na] = v;
      c += best;
    }
  }
  *cost = c;
}

int main(int argc, char** argv) {
  int n = argc > 1 ? atoi(argv[1]) : 1001;
  int k = argc > 2 ? atoi(argv[2]) : 16;
  long long iters = argc > 3 ? atoll(argv[3]) : 200000;
  int chains = argc > 4 ? atoi(argv[4]) : 384;
  printf("n=%d k=%d iters/chain=%lld chains=%d cpu_threads=%d\n", n, k, iters, chains, omp_get_max_threads());

  srand(42);
  std::vector<float> xs(n), ys(n);
  for (int i = 0; i < n; i++) { xs[i] = rand() % 100000; ys[i] = rand() % 100000; }
  std::vector<uint32_t> mat((size_t)n * n);
  for (int i = 0; i < n; i++)
    for (int j = 0; j < n; j++) {
      float dx = xs[i] - xs[j], dy = ys[i] - ys[j];
      mat[(size_t)i * n + j] = (uint32_t)sqrtf(dx * dx + dy * dy);
    }
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

  // identical starting tour for every chain
  std::vector<int> perm(n), nxt0(n), prv0(n);
  for (int i = 0; i < n; i++) perm[i] = i;
  for (int i = n - 1; i > 0; i--) { int j = rand() % (i + 1); std::swap(perm[i], perm[j]); }
  long long cost0 = 0;
  for (int i = 0; i < n; i++) {
    int a = perm[i], b = perm[(i + 1) % n];
    nxt0[a] = b; prv0[b] = a; cost0 += mat[(size_t)a * n + b];
  }

  // ---------- CPU: 12 threads, one chain each (the real CPU deployment) ----------
  int cpu_chains = omp_get_max_threads();
  std::vector<std::vector<int>> cn(cpu_chains, nxt0), cp(cpu_chains, prv0);
  std::vector<long long> ccost(cpu_chains, cost0);
  auto t0 = std::chrono::steady_clock::now();
#pragma omp parallel for
  for (int c = 0; c < cpu_chains; c++)
    cpuChain(mat.data(), n, cand.data(), k, cn[c].data(), cp[c].data(), &ccost[c], iters, 1000 + c);
  double s_cpu = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  long long cpu_best = *std::min_element(ccost.begin(), ccost.end());

  // ---------- GPU: `chains` blocks, one chain each ----------
  uint32_t* d_mat; int *d_cand, *d_nxt, *d_prv; long long* d_cost;
  CUDA_CHECK(cudaMalloc(&d_mat, (size_t)n * n * 4));
  CUDA_CHECK(cudaMalloc(&d_cand, (size_t)n * k * 4));
  CUDA_CHECK(cudaMalloc(&d_nxt, (size_t)chains * n * 4));
  CUDA_CHECK(cudaMalloc(&d_prv, (size_t)chains * n * 4));
  CUDA_CHECK(cudaMalloc(&d_cost, (size_t)chains * 8));
  CUDA_CHECK(cudaMemcpy(d_mat, mat.data(), (size_t)n * n * 4, cudaMemcpyHostToDevice));
  CUDA_CHECK(cudaMemcpy(d_cand, cand.data(), (size_t)n * k * 4, cudaMemcpyHostToDevice));
  std::vector<long long> gcost(chains, cost0);
  for (int c = 0; c < chains; c++) {
    CUDA_CHECK(cudaMemcpy(d_nxt + (size_t)c * n, nxt0.data(), n * 4, cudaMemcpyHostToDevice));
    CUDA_CHECK(cudaMemcpy(d_prv + (size_t)c * n, prv0.data(), n * 4, cudaMemcpyHostToDevice));
  }
  CUDA_CHECK(cudaMemcpy(d_cost, gcost.data(), (size_t)chains * 8, cudaMemcpyHostToDevice));
  // warmup launch
  chainKernel<<<chains, 32>>>(d_mat, n, d_cand, k, d_nxt, d_prv, d_cost, 100, 7);
  CUDA_CHECK(cudaDeviceSynchronize());

  t0 = std::chrono::steady_clock::now();
  chainKernel<<<chains, 32>>>(d_mat, n, d_cand, k, d_nxt, d_prv, d_cost, iters, 1000);
  CUDA_CHECK(cudaDeviceSynchronize());
  double s_gpu = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  CUDA_CHECK(cudaMemcpy(gcost.data(), d_cost, (size_t)chains * 8, cudaMemcpyDeviceToHost));
  long long gpu_best = *std::min_element(gcost.begin(), gcost.end());

  double cpu_rate = (double)iters * cpu_chains / s_cpu;
  double gpu_rate = (double)iters * chains / s_gpu;
  printf("cpu: %d chains, %.2fs, per-chain %.2f Miter/s, aggregate %.2f Miter/s, best=%lld (start %lld)\n",
         cpu_chains, s_cpu, (double)iters / s_cpu / 1e6, cpu_rate / 1e6, cpu_best, cost0);
  printf("gpu: %d chains, %.2fs, per-chain %.2f Miter/s, aggregate %.2f Miter/s, best=%lld\n",
         chains, s_gpu, (double)iters / s_gpu / 1e6, gpu_rate / 1e6, gpu_best);
  printf("AGGREGATE_RATIO=%.2f PER_CHAIN_RATIO=%.3f\n", gpu_rate / cpu_rate,
         ((double)iters / s_gpu) / ((double)iters / s_cpu));
  return 0;
}
