#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/env.sh"
cd "$SWARM_ROOT"
exec .ros2/swarm/venv/bin/python Swarm-SLAM/s3e/run.py "$@"
