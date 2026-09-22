#!/usr/bin/env bash
set -eo pipefail
SUBMAP_SOURCE_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)"
cd "$SUBMAP_SOURCE_ROOT"
source /opt/ros/humble/setup.bash
source .ros2/temporal-submaps-revert/install/ellipselio/share/ellipselio/local_setup.bash
export PYTHONPATH="$SUBMAP_SOURCE_ROOT/FAST-LIVO2-ROS2/research:${PYTHONPATH:-}"
export LD_LIBRARY_PATH="/usr/local/cuda/lib64:${LD_LIBRARY_PATH:-}"
export OMP_NUM_THREADS=4 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
export PYTHONDONTWRITEBYTECODE=1 PYTHONUNBUFFERED=1
export ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOG_DIR=/tmp/graco-aerial-temporal-ros
export MPLCONFIGDIR="$SUBMAP_SOURCE_ROOT/.ros2/matplotlib"
exec "$@"
