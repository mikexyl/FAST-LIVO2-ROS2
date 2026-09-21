#!/usr/bin/env bash
set -euo pipefail
ELLIPSOID_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
mkdir -p "$ELLIPSOID_ROOT/.ros2/ellipsoid-cuda"
/usr/local/cuda/bin/nvcc -O3 --fmad=false -arch=sm_89 -std=c++17 --shared -Xcompiler=-fPIC \
  "$ELLIPSOID_ROOT/FAST-LIVO2-ROS2/research/adapters/ellipsoid_cuda/surface.cu" \
  -o "$ELLIPSOID_ROOT/.ros2/ellipsoid-cuda/libellipsoid_surface.so"
