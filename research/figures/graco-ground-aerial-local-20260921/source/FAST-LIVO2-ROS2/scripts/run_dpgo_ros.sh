#!/usr/bin/env bash
set -eo pipefail
S3E_SOURCE_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
CBS_UNDERLAY="${CBS_UNDERLAY:-/home/mikexyl/workspaces/sb_slam_ros2_ws/install}"
source /opt/ros/humble/setup.bash
source "$CBS_UNDERLAY/setup.bash"
source "$S3E_SOURCE_ROOT/.ros2/dpgo-install/setup.bash"
export ROS_LOG_DIR="$S3E_SOURCE_ROOT/.ros2/dpgo-ros-log"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
exec "$@"
