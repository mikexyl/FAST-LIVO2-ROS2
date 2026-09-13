#!/usr/bin/env bash
set -eo pipefail
S3E_SOURCE_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
if [[ "${1:-}" == '--background' ]]; then
  shift
  exec /usr/bin/python3 "$S3E_SOURCE_ROOT/FAST-LIVO2-ROS2/scripts/s3e_session.py" start "$@"
elif [[ "${1:-}" == '--stop' ]]; then
  exec /usr/bin/python3 "$S3E_SOURCE_ROOT/FAST-LIVO2-ROS2/scripts/s3e_session.py" stop
fi
source /opt/ros/humble/setup.bash
if [[ ! -f "$S3E_SOURCE_ROOT/.ros2/install/setup.bash" ]]; then
  echo 'Build first: FAST-LIVO2-ROS2/scripts/build_s3e.sh' >&2
  exit 1
fi
source "$S3E_SOURCE_ROOT/.ros2/install/setup.bash"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-73}"
export ROS_LOCALHOST_ONLY=1
export ROS_LOG_DIR="$S3E_SOURCE_ROOT/.ros2/ros-log"
export OMP_NUM_THREADS="${OMP_NUM_THREADS:-4}"
exec /usr/bin/python3 "$S3E_SOURCE_ROOT/FAST-LIVO2-ROS2/scripts/run_s3e.py" "$@"
