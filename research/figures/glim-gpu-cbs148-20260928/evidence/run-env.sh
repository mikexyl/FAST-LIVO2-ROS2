#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /workspace/.ros2/cbs-underlay/setup.bash
export CBS_OVERLAY=/workspace/.ros2/gpu-cbs148-20260928/source/.ros2/dpgo-install
source "$CBS_OVERLAY/setup.bash"
export CBS_UNDERLAY=/workspace/.ros2/cbs-underlay
export LD_LIBRARY_PATH=/workspace/.ros2/swarm/native/lib:${LD_LIBRARY_PATH:-}
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 S3E_TEST_GPU_DDS=1
export PYTHONPATH=/workspace/.ros2/gpu-cbs148-20260928/source/FAST-LIVO2-ROS2/research:$PYTHONPATH
export GIT_CONFIG_COUNT=2 GIT_CONFIG_KEY_0=safe.directory GIT_CONFIG_KEY_1=safe.directory
export GIT_CONFIG_VALUE_0=/workspace/.ros2/gpu-cbs148-20260928/source/cbs
export GIT_CONFIG_VALUE_1=/workspace/.ros2/gpu-cbs148-20260928/source/cbs_ros
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
cd /workspace/.ros2/gpu-cbs148-20260928/source/FAST-LIVO2-ROS2
exec "$@"
