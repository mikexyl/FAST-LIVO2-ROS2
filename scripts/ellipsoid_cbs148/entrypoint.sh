#!/usr/bin/env bash
set -eo pipefail
cd /workspace
exec bash FAST-LIVO2-ROS2/scripts/ellipsoid_cbs148/env.sh .ros2/research-venv/bin/python \
  FAST-LIVO2-ROS2/scripts/ellipsoid_cbs148/queue.py --linger "$@"
