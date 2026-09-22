#!/usr/bin/env bash
set -euo pipefail
ELLIPSOID_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ELLIPSOID_ROOT"
ELLIPSOID_WORK="${1:?provide full-run work directory}"
ELLIPSOID_RESULT="${2:?provide new report directory}"
export PYTHONPATH="$ELLIPSOID_ROOT/FAST-LIVO2-ROS2/research"
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/s3e-ellipsoid-matplotlib
while [[ ! -f "$ELLIPSOID_WORK/prepared-Carol.json" ]]; do sleep 3; done
.ros2/research-venv/bin/python -m s3e_pipeline.ellipsoid_full solve --work "$ELLIPSOID_WORK" --resume
.ros2/research-venv/bin/python -m s3e_pipeline.ellipsoid_full evaluate --work "$ELLIPSOID_WORK" --output "$ELLIPSOID_RESULT"
.ros2/research-venv/bin/python -m s3e_pipeline.ellipsoid_full_report "$ELLIPSOID_RESULT"
.ros2/rerun-venv/bin/python -m s3e_pipeline.ellipsoid_full_report "$ELLIPSOID_RESULT" --rerun
.ros2/rerun-venv/bin/rerun rrd verify "$ELLIPSOID_RESULT/trajectories.rrd"
