#!/usr/bin/env bash
set -eo pipefail
B=/home/mikexyl/workspaces/fast_livo2_ws/src/.ros2/graco-ground-aerial-local-20260921
ctest --test-dir "$B/build/ellipselio" --output-on-failure
.ros2/research-venv/bin/python -m pytest -q FAST-LIVO2-ROS2/research/tests/test_cbs_bridge.py FAST-LIVO2-ROS2/research/tests/test_registration_exchange.py FAST-LIVO2-ROS2/research/tests/test_vertical_initialization.py
.ros2/dpgo-build/cbs/test_inter_robot_pcm
touch "$B/BUILD_COMPLETE"
