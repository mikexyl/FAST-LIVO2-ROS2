#!/usr/bin/env bash
set -eo pipefail
S3E_SOURCE_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$S3E_SOURCE_ROOT"
# Reuse installed GTSAM 4.3 / aria / message dependencies without modifying them.
CBS_UNDERLAY="${CBS_UNDERLAY:-/home/mikexyl/workspaces/sb_slam_ros2_ws/install}"
source /opt/ros/humble/setup.bash
source "$CBS_UNDERLAY/setup.bash"
export ROS_LOG_DIR="$S3E_SOURCE_ROOT/.ros2/dpgo-ros-log"
export CMAKE_BUILD_PARALLEL_LEVEL="${CMAKE_BUILD_PARALLEL_LEVEL:-2}"
colcon --log-base .ros2/dpgo-log build --base-paths cbs cbs_ros \
  --build-base .ros2/dpgo-build --install-base .ros2/dpgo-install \
  --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release \
  -DCBS_BUILD_UTILS=OFF -DCBS_BUILD_EXAMPLES=OFF -DBUILD_TESTING=ON \
  -DCBS_ROS_WITH_REGISTRATION_FACTORS="${S3E_REGISTRATION_FACTORS:-OFF}" \
  -DS3E_REGISTRATION_ADAPTER_DIR="$S3E_SOURCE_ROOT/FAST-LIVO2-ROS2/research/adapters/gtsam_points" \
  -DGTSAM_POINTS_SOURCE_DIR="$S3E_SOURCE_ROOT/.ros2/deps/gtsam_points" \
  -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
