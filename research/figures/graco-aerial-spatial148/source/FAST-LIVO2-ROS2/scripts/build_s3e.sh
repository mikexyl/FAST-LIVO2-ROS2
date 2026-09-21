#!/usr/bin/env bash
set -eo pipefail
S3E_SOURCE_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$S3E_SOURCE_ROOT"
source /opt/ros/humble/setup.bash
export CMAKE_PREFIX_PATH="$S3E_SOURCE_ROOT/.ros2/deps:${CMAKE_PREFIX_PATH:-}"
export CMAKE_BUILD_PARALLEL_LEVEL="${CMAKE_BUILD_PARALLEL_LEVEL:-4}"
export MAKEFLAGS="-j${CMAKE_BUILD_PARALLEL_LEVEL}"
cmake -S Sophus -B .ros2/build-sophus -DCMAKE_INSTALL_PREFIX="$S3E_SOURCE_ROOT/.ros2/deps" \
  -DBUILD_SOPHUS_TESTS=OFF -DBUILD_SOPHUS_EXAMPLES=OFF -DCMAKE_EXPORT_NO_PACKAGE_REGISTRY=ON
cmake --install .ros2/build-sophus
colcon --log-base .ros2/log build \
  --base-paths livox_ros_driver2 rpg_vikit/vikit_common rpg_vikit/vikit_ros FAST-LIVO2-ROS2 \
  --build-base .ros2/build --install-base .ros2/install --symlink-install --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DLIVOX_BUILD_MESSAGES_ONLY=ON \
  -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
