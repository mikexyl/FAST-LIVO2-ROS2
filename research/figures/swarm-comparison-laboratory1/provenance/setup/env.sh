#!/usr/bin/env bash
# Source this file; keep Swarm dependencies separate from the other frontends.
SWARM_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
source /opt/ros/humble/setup.bash
source "$SWARM_ROOT/.ros2/swarm/install/setup.bash"
export PYTHONPATH="$SWARM_ROOT/.ros2/swarm/teaser-build:$SWARM_ROOT/Swarm-SLAM/src/cslam:$SWARM_ROOT/FAST-LIVO2-ROS2/research:$PYTHONPATH"
export LD_LIBRARY_PATH="/home/mikexyl/workspaces/sb_slam_ros2_ws/install/gtsam/lib:/home/mikexyl/workspaces/sb_slam_ros2_ws/install/teaserpp/lib:$LD_LIBRARY_PATH"
export ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-124}"
export ROS_LOG_DIR="$SWARM_ROOT/.ros2/swarm/ros-log"
export OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1
