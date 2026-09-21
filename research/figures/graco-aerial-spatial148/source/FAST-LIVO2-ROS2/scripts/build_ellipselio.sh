#!/usr/bin/env bash
set -eo pipefail
S3E_ELLIPSE_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$S3E_ELLIPSE_ROOT"
source /opt/ros/humble/setup.bash
export ROS_LOG_DIR="$S3E_ELLIPSE_ROOT/.ros2/ellipse-ros-log"
export MAKEFLAGS="-j2 -l2"
colcon --log-base .ros2/ellipse-log build --base-paths ellipselio \
  --build-base .ros2/ellipse-build --install-base .ros2/ellipse-install \
  --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release \
  -DS3E_RESEARCH_SOURCE="$S3E_ELLIPSE_ROOT/FAST-LIVO2-ROS2" \
  -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
