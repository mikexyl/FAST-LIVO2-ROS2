#!/usr/bin/env bash
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
out="${1:?Pass isolated output directory}"
variant="${2:-parallel}"
case "$variant" in
 pixel) cuda_source=raster.cu ;;
 parallel) cuda_source=raster_parallel.cu ;;
 *) echo "Unknown CUDA variant: $variant (expected pixel or parallel)" >&2; exit 2 ;;
esac
mkdir -p "$out"
c++ -std=c++17 -O3 -fPIC -shared "$root/research/adapters/ellipsoid_raster/raster.cpp" -o "$out/libellipsoid_raster_cpu.so"
if command -v nvcc >/dev/null; then
 nvcc -std=c++17 -O3 -Xcompiler -fPIC -shared "$root/research/adapters/ellipsoid_raster/$cuda_source" -o "$out/libellipsoid_raster_cuda.so"
fi
