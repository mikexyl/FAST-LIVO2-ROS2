#!/usr/bin/env bash
set -eo pipefail
cd /workspace
source /opt/ros/humble/setup.bash
source /workspace/.ros2/upstream-ellipselio-20260921/install/setup.bash
export OMP_NUM_THREADS=4 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
export PYTHONDONTWRITEBYTECODE=1 PYTHONUNBUFFERED=1
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=213 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOG_DIR=/workspace/.ros2/upstream-ellipselio-20260921/ros-log
export MPLCONFIGDIR=/workspace/.ros2/upstream-ellipselio-20260921/matplotlib
exec "$@"
