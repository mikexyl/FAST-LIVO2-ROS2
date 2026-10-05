#!/usr/bin/env bash
set -eo pipefail
circle_source="$(realpath "$1")"
cd "$circle_source"
source /opt/ros/humble/setup.bash
source "${CBS_UNDERLAY:-/workspace/.ros2/cbs-underlay}/setup.bash"
export CMAKE_BUILD_PARALLEL_LEVEL="${CMAKE_BUILD_PARALLEL_LEVEL:-$(nproc)}"
circle_prefix="$circle_source/.ros2/cbs-spatial"
colcon --log-base "$circle_prefix-log" build --base-paths cbs cbs_ros \
  --build-base "$circle_prefix-build" --install-base "$circle_prefix-install" \
  --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release \
  -DCBS_BUILD_UTILS=OFF -DCBS_BUILD_EXAMPLES=OFF -DBUILD_TESTING=ON \
  -DCBS_ROS_WITH_REGISTRATION_FACTORS=ON -DS3E_REGISTRATION_CUDA=ON \
  -DCMAKE_CUDA_ARCHITECTURES="${S3E_CUDA_ARCHITECTURES:-native}" \
  -DGTSAM_DIR="${CBS_GTSAM_DIR:?Set CBS_GTSAM_DIR to the validated GTSAM 4.3 CMake directory}" \
  -DS3E_REGISTRATION_ADAPTER_DIR="$circle_source/FAST-LIVO2-ROS2/research/adapters/gtsam_points" \
  -DGTSAM_POINTS_SOURCE_DIR="$circle_source/.ros2/deps/gtsam_points" \
  -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
source "$circle_prefix-install/setup.bash"
# The ROS underlay also contains GTSAM. Match the library selected at configure
# time after all setup scripts have modified the runtime search path.
export LD_LIBRARY_PATH="${CBS_GTSAM_DIR%/cmake/GTSAM}:${LD_LIBRARY_PATH:-}"
ctest --test-dir "$circle_prefix-build/cbs" --output-on-failure
ctest --test-dir "$circle_prefix-build/cbs_ros" --output-on-failure
