#!/usr/bin/env bash
set -eo pipefail
SWARM_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$SWARM_ROOT"
source /opt/ros/humble/setup.bash
export CMAKE_BUILD_PARALLEL_LEVEL=2
colcon --log-base .ros2/swarm/log build \
  --base-paths Swarm-SLAM/src/rtabmap_ros/rtabmap_msgs \
    Swarm-SLAM/src/cslam_interfaces Swarm-SLAM/src/cslam Swarm-SLAM/src/cslam_experiments \
  --build-base .ros2/swarm/build --install-base .ros2/swarm/install \
  --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
  -DCSLAM_LIDAR_ONLY=ON \
  -DGTSAM_DIR=/home/mikexyl/workspaces/sb_slam_ros2_ws/install/gtsam/lib/cmake/GTSAM \
  -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
