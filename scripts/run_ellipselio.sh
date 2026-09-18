#!/usr/bin/env bash
set -eo pipefail
S3E_ELLIPSE_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
source /opt/ros/humble/setup.bash
source "$S3E_ELLIPSE_ROOT/.ros2/ellipse-install/setup.bash"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-85}"
export ROS_LOCALHOST_ONLY=1
export ROS_LOG_DIR="$S3E_ELLIPSE_ROOT/.ros2/ellipse-ros-log"
export OMP_NUM_THREADS="${OMP_NUM_THREADS:-4}"
export OPENBLAS_NUM_THREADS=1
exec /usr/bin/python3 "$S3E_ELLIPSE_ROOT/FAST-LIVO2-ROS2/scripts/run_ellipselio.py" "$@"
