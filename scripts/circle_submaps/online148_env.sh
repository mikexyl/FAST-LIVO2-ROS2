#!/usr/bin/env bash
set -eo pipefail
CIRCLE_PROJECT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
source /opt/ros/humble/setup.bash
source /workspace/.ros2/cbs-underlay/setup.bash
source /workspace/.ros2/circle-cbs148-20261001/source/.ros2/cbs-spatial-install/setup.bash
source /workspace/.ros2/upstream-area-s3e-20260921/capacity-run/install/ellipselio/share/ellipselio/local_setup.bash
export CBS_OVERLAY=/workspace/.ros2/circle-cbs148-20261001/source/.ros2/cbs-spatial-install
export CBS_UNDERLAY=/workspace/.ros2/cbs-underlay
export LD_LIBRARY_PATH="/workspace/.ros2/swarm/native/lib:/usr/local/cuda/lib64:${LD_LIBRARY_PATH:-}"
export PYTHONPATH="${CIRCLE_PROJECT}/research:/workspace/.ros2/full-ellipsoid148-20260926/native:${PYTHONPATH:-}"
# Match the previous successful 14-robot online deployment. Each native ROS
# launcher still has four callbacks; nested numerical pools must not multiply
# across fourteen robots. No host CPU affinity, memory quota or GPU cap is set.
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
export PYTHONDONTWRITEBYTECODE=1 PYTHONUNBUFFERED=1 PYTHONFAULTHANDLER=1
export ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOG_DIR=/tmp/circle-online148-ros MPLCONFIGDIR=/tmp/circle-online148-mpl
exec "$@"
