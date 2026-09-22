#!/usr/bin/env bash
# Keep one authorized pipeline run alive outside the interactive command lifetime.
set -euo pipefail
if [[ $# -lt 3 || $1 != s3e-* || $3 != run ]]; then
  echo 'Usage: run_s3e_background.sh s3e-<name> <log-path> run ...' >&2
  exit 2
fi
pipeline_unit=$1
pipeline_log=$2
shift 2
pipeline_source="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
pipeline_log="$(realpath -m -- "$pipeline_log")"
mkdir -p -- "$(dirname -- "$pipeline_log")"
pipeline_snapshot="$(S3E_SOURCE_ROOT="$pipeline_source" "$pipeline_source/.ros2/research-venv/bin/python" "$pipeline_source/FAST-LIVO2-ROS2/research/s3e_pipeline/snapshot.py" --print-root)"
exec systemd-run --user --collect --no-block --unit="$pipeline_unit" \
  --working-directory="$pipeline_source" --service-type=exec \
  --property=KillSignal=SIGINT --property=TimeoutStopSec=30 \
  --property="StandardOutput=append:$pipeline_log" --property=StandardError=inherit \
  --setenv=PYTHONUNBUFFERED=1 --setenv="S3E_RESEARCH_SNAPSHOT=$pipeline_snapshot" \
  "$pipeline_source/FAST-LIVO2-ROS2/scripts/s3e_experiment.sh" "$@"
