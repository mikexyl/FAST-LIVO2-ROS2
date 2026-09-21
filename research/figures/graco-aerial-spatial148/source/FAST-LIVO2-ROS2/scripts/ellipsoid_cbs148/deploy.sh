#!/usr/bin/env bash
# Run on host 148 from the deployed workspace after validation.
set -euo pipefail
cd /data3/mikexyl/swarm_s3e_ws/src
RUN_IMAGE="${1:?pass the validated immutable image ID}"
docker run -d --name ellipsoid-cbs148-overnight --restart unless-stopped \
  --init --gpus all --cpus 16 --memory 48g --pids-limit 2048 --shm-size 2g \
  --log-opt max-size=10m --log-opt max-file=3 --stop-timeout 60 --user 1002:1002 \
  -v /data3/mikexyl/swarm_s3e_ws/src:/workspace \
  -v /mnt/data/s3e:/data/s3e:ro -v /usr/local/cuda-12.9:/usr/local/cuda:ro \
  -w /workspace --entrypoint bash "$RUN_IMAGE" \
  FAST-LIVO2-ROS2/scripts/ellipsoid_cbs148/entrypoint.sh
