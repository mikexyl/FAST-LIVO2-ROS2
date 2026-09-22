#!/usr/bin/env bash
set -eo pipefail
S3E_SOURCE_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
export S3E_SOURCE_ROOT
export PYTHONPATH="$S3E_SOURCE_ROOT/FAST-LIVO2-ROS2/research${PYTHONPATH:+:$PYTHONPATH}"
export OMP_NUM_THREADS=1
export OPENBLAS_NUM_THREADS=1
export MKL_NUM_THREADS=1
export PYTHONHASHSEED=0
exec "$S3E_SOURCE_ROOT/.ros2/research-venv/bin/python" -m s3e_pipeline.snapshot "$@"
