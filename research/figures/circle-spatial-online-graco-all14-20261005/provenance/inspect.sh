#!/usr/bin/env bash
set -uo pipefail
root=/workspace/.ros2/circle-spatial-online148-20261005
project=/workspace/.ros2/spatial-only148-20261005/source/FAST-LIVO2-ROS2
export CBS_OVERLAY=/workspace/.ros2/spatial-only148-20261005/source/.ros2/cbs-spatial-install
while [[ ! -f "$root/full.exit" ]]; do sleep 10; done
if [[ "$(cat "$root/full.exit")" != 0 ]]; then
  printf 'Capture failed; inspect full/status.json before evaluation.\n' > "$root/inspection.log"
  printf '1\n' > "$root/inspection.exit"
  exit 1
fi
bash "$project/scripts/circle_submaps/online148_env.sh" /workspace/.ros2/research-venv/bin/python \
  "$root/finish_spatial_online_graco.py" --runtime-project "$project" \
  --run "$root/full" --output "$root/inspection" \
  --base /workspace/.ros2/graco-all14-multilayer148-20260922/full \
  --control /workspace/.ros2/circle-mature-online148-20261001/inspection \
  --control-timing "$root/summary.json" > "$root/inspection.log" 2>&1
result=$?
printf '%s\n' "$result" > "$root/inspection.exit"
exit "$result"
