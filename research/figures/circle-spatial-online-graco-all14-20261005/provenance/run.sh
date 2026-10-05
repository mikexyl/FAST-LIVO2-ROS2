#!/usr/bin/env bash
set -uo pipefail
run_root=/workspace/.ros2/circle-spatial-online148-20261005
source_root=/workspace/.ros2/spatial-only148-20261005/source
export CBS_OVERLAY="$source_root/.ros2/cbs-spatial-install"
python_bin=/workspace/.ros2/research-venv/bin/python
project="$source_root/FAST-LIVO2-ROS2"
bash "$project/scripts/circle_submaps/online148_env.sh" "$python_bin" \
  "$project/scripts/circle_submaps/run_online_graco.py" \
  --config "$project/research/configs/circle_online_spatial_neighbors_graco_all14.yaml" \
  --base /workspace/.ros2/circle-mature-online148-20261001/full \
  --output "$run_root/full" --settle-iterations 30 \
  > "$run_root/full.log" 2>&1
run_status=$?
printf '%s\n' "$run_status" > "$run_root/full.exit"
exit "$run_status"
