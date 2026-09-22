#!/usr/bin/env bash
set -euo pipefail
pipeline_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$pipeline_root"
export UV_CACHE_DIR="$pipeline_root/.ros2/uv-cache"
research="$pipeline_root/FAST-LIVO2-ROS2/research"
if [[ ! -x .ros2/research-venv/bin/python ]]; then
  uv venv --python /usr/bin/python3 --system-site-packages .ros2/research-venv
fi
uv pip install --python .ros2/research-venv/bin/python -r "$research/requirements.txt" pytest==9.1.1
if [[ ! -x .ros2/model-venv/bin/python ]]; then
  uv venv --python /usr/bin/python3 .ros2/model-venv
fi
uv pip install --python .ros2/model-venv/bin/python --index-url https://download.pytorch.org/whl/cu124 torch==2.5.0+cu124 torchvision==0.20.0+cu124
uv pip install --python .ros2/model-venv/bin/python -r "$research/model-requirements.lock.txt"
.ros2/research-venv/bin/python "$research/setup_assets.py"
.ros2/research-venv/bin/python "$research/setup_mapclosures.py"
.ros2/research-venv/bin/python "$research/setup_mapclosures.py" --inspection-only
FAST-LIVO2-ROS2/scripts/setup_rerun.sh
echo 'MegaLoc, MapClosures, GTSAM and Rerun are ready.'
