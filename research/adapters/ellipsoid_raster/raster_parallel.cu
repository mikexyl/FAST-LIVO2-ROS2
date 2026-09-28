#include "coverage.hpp"
#include <cuda_runtime.h>
#include <algorithm>
#include <vector>

using namespace ellipsoid_raster;
namespace {
// Fixed scratch capacity, independent of snapshot size. Each reference is one
// primitive's contribution to a tile. No floating-point atomics are used.
constexpr int kReferenceBatch = 16384;

__global__ void contributions(const double* primitives, const int* ids,
    const int* reference_tiles, const double* bands, int layer, int begin,
    int count, int tiles, int tile, int size, int origin, double step,
    double radius, double* scratch) {
  int work = blockIdx.x * blockDim.x + threadIdx.x;
  int pixels = tile * tile;
  if (work >= count * pixels) return;
  int reference = begin + work / pixels, pixel = work % pixels;
  int t = reference_tiles[reference];
  int x = (t / tiles) * tile + pixel / tile;
  int y = (t % tiles) * tile + pixel % tile;
  double value = 0;
  if (x < size && y < size) {
    const double* b = bands + 6 * layer;
    Slice slice{b[0], b[1], b[2], b[3], b[4], bool(b[5])};
    value = coverage(*reinterpret_cast<const Primitive*>(primitives + 12 * ids[reference]),
        slice, (origin + x) * step, (origin + y) * step, step, radius);
  }
  scratch[work] = value;
}

__global__ void accumulate(const int* offsets, int first_tile, int tiles,
    int tile, int size, int layer, int begin, int end,
    const double* scratch, double* output) {
  int t = first_tile + blockIdx.x, pixel = threadIdx.x;
  int x = (t / tiles) * tile + pixel / tile;
  int y = (t % tiles) * tile + pixel % tile;
  if (x >= size || y >= size) return;
  size_t index = (size_t(layer) * size + x) * size + y;
  double value = output[index];
  // Same primitive order and parenthesization as the reference, including
  // across scratch batches. Kernel launches use one ordered CUDA stream.
  for (int j = max(begin, offsets[t]); j < min(end, offsets[t + 1]); ++j)
    value += scratch[size_t(j - begin) * tile * tile + pixel];
  output[index] = value;
}
}  // namespace

extern "C" const char* raster_cuda_variant() { return "parallel_contributions_v1"; }

extern "C" int raster_cuda(const double* p, int count, const int* offsets,
    const int* ids, const double* bands, int layers, double radius, double step,
    int origin, int size, int tile, double* out, size_t* peak_bytes) {
  int tiles = (size + tile - 1) / tile, nt = tiles * tiles, ni = offsets[nt];
  std::vector<int> reference_tiles(ni);
  for (int t = 0; t < nt; ++t)
    std::fill(reference_tiles.begin() + offsets[t], reference_tiles.begin() + offsets[t + 1], t);
  double *dp = nullptr, *db = nullptr, *dst = nullptr, *scratch = nullptr;
  int *dof = nullptr, *di = nullptr, *dt = nullptr;
  cudaError_t status = cudaSuccess;
  size_t ps = size_t(count) * 12 * sizeof(double), bs = size_t(layers) * 6 * sizeof(double);
  size_t os = size_t(nt + 1) * sizeof(int), is = size_t(ni) * sizeof(int);
  size_t ds = size_t(layers) * size * size * sizeof(double);
  size_t ss = size_t(std::min(ni, kReferenceBatch)) * tile * tile * sizeof(double);
  *peak_bytes = ps + bs + os + 2 * is + ds + ss;
#define CK(expr) if ((status = (expr)) != cudaSuccess) goto done
  CK(cudaMalloc(&dp, ps ? ps : 1)); CK(cudaMalloc(&db, bs));
  CK(cudaMalloc(&dof, os)); CK(cudaMalloc(&di, is ? is : 1));
  CK(cudaMalloc(&dt, is ? is : 1)); CK(cudaMalloc(&dst, ds));
  CK(cudaMalloc(&scratch, ss ? ss : 1));
  CK(cudaMemcpy(dp, p, ps, cudaMemcpyHostToDevice));
  CK(cudaMemcpy(db, bands, bs, cudaMemcpyHostToDevice));
  CK(cudaMemcpy(dof, offsets, os, cudaMemcpyHostToDevice));
  CK(cudaMemcpy(di, ids, is, cudaMemcpyHostToDevice));
  CK(cudaMemcpy(dt, reference_tiles.data(), is, cudaMemcpyHostToDevice));
  CK(cudaMemset(dst, 0, ds));
  for (int layer = 0; layer < layers; ++layer) {
    for (int begin = 0; begin < ni; begin += kReferenceBatch) {
      int end = std::min(ni, begin + kReferenceBatch), n = end - begin;
      contributions<<<(n * tile * tile + 127) / 128, 128>>>(dp, di, dt, db,
          layer, begin, n, tiles, tile, size, origin, step, radius, scratch);
      CK(cudaGetLastError());
      int first = reference_tiles[begin], last = reference_tiles[end - 1];
      accumulate<<<last - first + 1, tile * tile>>>(dof, first, tiles, tile, size,
          layer, begin, end, scratch, dst);
      CK(cudaGetLastError());
    }
  }
  CK(cudaDeviceSynchronize());
  CK(cudaMemcpy(out, dst, ds, cudaMemcpyDeviceToHost));
done:
  cudaFree(dp); cudaFree(db); cudaFree(dof); cudaFree(di);
  cudaFree(dt); cudaFree(dst); cudaFree(scratch);
  return int(status);
#undef CK
}
