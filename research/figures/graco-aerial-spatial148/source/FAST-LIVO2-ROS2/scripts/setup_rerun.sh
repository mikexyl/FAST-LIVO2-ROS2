#!/usr/bin/env bash
set -eo pipefail
S3E_SOURCE_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
if [[ ! -x "$S3E_SOURCE_ROOT/.ros2/rerun-venv/bin/python" ]]; then
  uv venv --python /usr/bin/python3 --system-site-packages "$S3E_SOURCE_ROOT/.ros2/rerun-venv"
fi
uv pip install --python "$S3E_SOURCE_ROOT/.ros2/rerun-venv/bin/python" \
  -r "$S3E_SOURCE_ROOT/FAST-LIVO2-ROS2/scripts/rerun-requirements.txt"
