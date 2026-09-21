#!/usr/bin/env bash
set -eo pipefail
cd /workspace
source /opt/ros/humble/setup.bash
source .ros2/cbs-underlay/setup.bash
source .ros2/dpgo-install/setup.bash
source .ros2/graco-aerial-four148-20260920/install/ellipselio/share/ellipselio/local_setup.bash
export CBS_UNDERLAY=/workspace/.ros2/cbs-underlay
export LD_LIBRARY_PATH="/workspace/.ros2/swarm/native/lib:/usr/local/cuda/lib64:${LD_LIBRARY_PATH:-}"
export PYTHONPATH="/workspace/FAST-LIVO2-ROS2/research:${PYTHONPATH:-}"
export OMP_NUM_THREADS=4 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
export PYTHONDONTWRITEBYTECODE=1 PYTHONUNBUFFERED=1
export ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOG_DIR=/tmp/graco-aerial-four148-ros
export MPLCONFIGDIR=/workspace/.ros2/matplotlib
exec "$@"
