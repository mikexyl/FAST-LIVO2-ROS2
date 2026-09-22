#!/usr/bin/env bash
set -eo pipefail
cd /home/mikexyl/workspaces/fast_livo2_ws/src/.ros2/graco-ground-aerial-local-20260921/source
source /opt/ros/humble/setup.bash
source /home/mikexyl/workspaces/sb_slam_ros2_ws/install/setup.bash
source .ros2/dpgo-install/setup.bash
source .ros2/graco-ground-aerial-local-20260921/install/ellipselio/share/ellipselio/local_setup.bash
export CBS_UNDERLAY=/home/mikexyl/workspaces/sb_slam_ros2_ws/install
export PYTHONPATH="$PWD/../native:$PWD/FAST-LIVO2-ROS2/research:${PYTHONPATH:-}"
export LD_LIBRARY_PATH="$PWD/.ros2/graco-ground-aerial-local-20260921/install/ellipselio/lib:/home/mikexyl/workspaces/fast_livo2_ws/src/.ros2/swarm/native/lib:/usr/local/cuda/lib64:${LD_LIBRARY_PATH:-}"
export PYTHONDONTWRITEBYTECODE=1 PYTHONUNBUFFERED=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
export ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_DOMAIN_ID=180
export ROS_LOG_DIR=/tmp/graco-mixed-ros MPLCONFIGDIR=/tmp/graco-mixed-matplotlib
export OMP_NUM_THREADS=4 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
exec "$@"
