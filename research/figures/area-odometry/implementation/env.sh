#!/usr/bin/env bash
set -eo pipefail
cd /home/mikexyl/workspaces/fast_livo2_ws/src
source /opt/ros/humble/setup.bash
source /home/mikexyl/workspaces/sb_slam_ros2_ws/install/setup.bash
source .ros2/dpgo-install/setup.bash
source .ros2/area-odometry-20260921/install/ellipselio/share/ellipselio/local_setup.bash
export CBS_UNDERLAY=/home/mikexyl/workspaces/sb_slam_ros2_ws/install
export PYTHONPATH="$PWD/.ros2/spatial-submaps-20260920/native:$PWD/FAST-LIVO2-ROS2/research:${PYTHONPATH:-}"
export LD_LIBRARY_PATH="$PWD/.ros2/swarm/native/lib:${LD_LIBRARY_PATH:-}"
export PYTHONDONTWRITEBYTECODE=1 PYTHONUNBUFFERED=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
export S3E_TEST_DDS=1
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=200 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOG_DIR=/tmp/ellipselio-area-odometry-ros
export OMP_NUM_THREADS=4 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
exec "$@"
