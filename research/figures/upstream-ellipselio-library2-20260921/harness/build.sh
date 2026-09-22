#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
cd /workspace
ROOT=/workspace/.ros2/upstream-ellipselio-20260921
colcon --log-base "$ROOT/log" build --base-paths "$ROOT/source/ellipselio" --build-base "$ROOT/build" --install-base "$ROOT/install" --cmake-args -DPYTHON_EXECUTABLE=/usr/bin/python3
cmake -S "$ROOT/harness/launcher" -B "$ROOT/launcher"
cmake --build "$ROOT/launcher" -j 4
