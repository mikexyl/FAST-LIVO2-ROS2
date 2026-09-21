#!/usr/bin/env bash
set -eo pipefail
cd /workspace
source /opt/ros/humble/setup.bash
export PYTHONDONTWRITEBYTECODE=1
export ROS_LOG_DIR=/tmp/graco-aerial-four148-build-ros
AERIAL_BUILD_ROOT=/workspace/.ros2/graco-aerial-four148-20260920
exec >"$AERIAL_BUILD_ROOT/build.log" 2>&1
colcon --log-base "$AERIAL_BUILD_ROOT/build-log" build --base-paths /workspace/ellipselio \
    --build-base "$AERIAL_BUILD_ROOT/build" --install-base "$AERIAL_BUILD_ROOT/install" \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON \
    -DS3E_RESEARCH_SOURCE=/workspace/FAST-LIVO2-ROS2 -DPYTHON_EXECUTABLE=/usr/bin/python3
source "$AERIAL_BUILD_ROOT/install/ellipselio/share/ellipselio/local_setup.bash"
cmake -S FAST-LIVO2-ROS2/scripts/ellipselio_live -B "$AERIAL_BUILD_ROOT/launcher" \
    -DCMAKE_BUILD_TYPE=Release
cmake --build "$AERIAL_BUILD_ROOT/launcher" --parallel "$(nproc)"
ctest --test-dir "$AERIAL_BUILD_ROOT/build/ellipselio" --output-on-failure
touch "$AERIAL_BUILD_ROOT/BUILD_COMPLETE"
