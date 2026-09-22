#!/usr/bin/env bash
set -eo pipefail
cd /home/mikexyl/workspaces/fast_livo2_ws/src
source /opt/ros/humble/setup.bash
B="$PWD/.ros2/graco-ground-aerial-local-20260921"
export PYTHONDONTWRITEBYTECODE=1 ROS_LOG_DIR=/tmp/graco-mixed-build
export CMAKE_BUILD_PARALLEL_LEVEL=4 MAKEFLAGS=-j4
colcon --log-base "$B/build-log" build --base-paths "$B/source/ellipselio" --build-base "$B/build" --install-base "$B/install" \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON -DPYTHON_EXECUTABLE=/usr/bin/python3 \
  -DS3E_RESEARCH_SOURCE="$B/source/FAST-LIVO2-ROS2"
cmake -S "$B/source/FAST-LIVO2-ROS2/scripts/ellipselio_live" -B "$B/launcher" -DCMAKE_BUILD_TYPE=Release
cmake --build "$B/launcher" --parallel 4
source "$B/install/ellipselio/share/ellipselio/local_setup.bash"
ctest --test-dir "$B/build/ellipselio" --output-on-failure
touch "$B/BUILD_COMPLETE"
