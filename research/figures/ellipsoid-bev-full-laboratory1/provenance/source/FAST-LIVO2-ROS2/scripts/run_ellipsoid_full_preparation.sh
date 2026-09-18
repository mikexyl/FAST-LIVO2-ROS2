#!/usr/bin/env bash
set -euo pipefail
ELLIPSOID_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ELLIPSOID_ROOT"
ELLIPSOID_WORK="${1:?provide a fresh full-run work directory}"
export PYTHONPATH="$ELLIPSOID_ROOT/FAST-LIVO2-ROS2/research"
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1
for ellipse_robot in Alpha Bob Carol; do
  while [[ ! -f "$ELLIPSOID_WORK/$ellipse_robot/export/manifest.json" ]]; do
    if [[ -f "$ELLIPSOID_WORK/$ellipse_robot/summary.json" ]]; then
      echo "Replay ended without a valid export for $ellipse_robot" >&2
      exit 1
    fi
    sleep 3
  done
  .ros2/research-venv/bin/python -m s3e_pipeline.ellipsoid_full prepare \
    --work "$ELLIPSOID_WORK" --robot "$ellipse_robot" --resume
done
