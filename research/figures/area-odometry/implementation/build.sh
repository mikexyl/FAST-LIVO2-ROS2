#!/usr/bin/env bash
set -eo pipefail
cd /home/mikexyl/workspaces/fast_livo2_ws/src
source /opt/ros/humble/setup.bash
export PYTHONDONTWRITEBYTECODE=1 ROS_LOG_DIR=/tmp/ellipselio-area-odometry-build-ros
export CMAKE_BUILD_PARALLEL_LEVEL=4 MAKEFLAGS=-j4
SPATIAL_BUILD_ROOT="$PWD/.ros2/area-odometry-20260921"
exec >"$SPATIAL_BUILD_ROOT/build.log" 2>&1
colcon --log-base "$SPATIAL_BUILD_ROOT/build-log" build --base-paths ellipselio \
 --build-base "$SPATIAL_BUILD_ROOT/build" --install-base "$SPATIAL_BUILD_ROOT/install" \
 --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON \
 -DS3E_RESEARCH_SOURCE="$PWD/FAST-LIVO2-ROS2" -DPYTHON_EXECUTABLE=/usr/bin/python3
source "$SPATIAL_BUILD_ROOT/install/ellipselio/share/ellipselio/local_setup.bash"
cmake -S FAST-LIVO2-ROS2/scripts/ellipselio_live -B "$SPATIAL_BUILD_ROOT/launcher" -DCMAKE_BUILD_TYPE=Release
cmake --build "$SPATIAL_BUILD_ROOT/launcher" --parallel 4
touch "$SPATIAL_BUILD_ROOT/BUILD_COMPLETE"
