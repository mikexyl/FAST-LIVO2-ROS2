# GRACO six-ground-robot experiment on 148

One fixed group maps `robot1`..`robot6` to `ground-01`..`ground-06`.
The expected result is one connected component. No artificial edges or GT
alignment enter detection, PCM or CBS; connectivity is measured from retained
constraints and checked against the native output reference frames.

Pipeline: EllipseLIO → native ellipsoid surface BEVs → MapClosures → six
independent DDS detection workers → distributed PCM → six native CBS optimizers,
including the existing live GICP factors. Frontend replay is serial at 1x and
ellipsoid surface preparation uses one CUDA process at a time. This is offline
multi-session processing with original absolute timestamps, not simultaneous
sensor capture. Full frontend playback totals about 36.4 minutes.

The supplied GRACO `T_Imu_Lidar` and IMU noise replace the upstream preset's
different calibration. Native point timing, negative sub-millisecond offsets,
and ground-05's two 0.2-second scans are retained. Sensor-only bags preserve CDR
bytes and record timestamps, with per-stream and file hashes; original bags are
untouched. GT is read only by evaluation. Its supplied `T_Base_Imu` reference
already matches the estimator body frame. Evo handles all trajectory metrics,
using one rigid alignment per CBS component, no scale fitting, and 50 ms
timestamp matching. A combined six-robot ATE requires all six in one component.

Keyframes, 80 m native ellipsoid crop, surface sampling, MapClosures thresholds,
registration acceptance, PCM and CBS settings match the working S3E path.
The six-robot timeout is 1800 seconds. Native analytics are enabled and saved as
`analytics.jsonl`; the message has no timestamp, so capture clock/last observed
odometry timestamps are explicitly approximate associations.

The isolated container is `ellipsoid-graco148`, using the pinned existing image
`sha256:cb89f7676e64520d3c6acfb37be9006b1b1dd52019d928b09f8ed7315ceb3ecc`,
12 CPUs, 32 GiB RAM, 2 GiB shared memory and DDS domain 190. Its package source
overlay is under `/data3/mikexyl/swarm_s3e_ws/src/.ros2/graco/source`.
Swarm-SLAM and CU-Multi use separate runtimes.

```bash
# Local sensor extraction, validation and checksums:
.ros2/research-venv/bin/python FAST-LIVO2-ROS2/scripts/run_graco.py \
  --stage pack --data /data/graco --inputs .ros2/graco148/inputs

# After transfer, the persistent controller runs a 45 s/robot smoke, then full:
ssh 148 docker exec -d ellipsoid-graco148 bash \
  /workspace/FAST-LIVO2-ROS2/scripts/graco148/env.sh \
  /workspace/.ros2/research-venv/bin/python \
  /workspace/FAST-LIVO2-ROS2/scripts/graco148/controller.py
```

The controller requires an explicitly published `inputs/TRANSFER_COMPLETE.json`
marker, validates all sensor-bag file hashes before use, freezes source hashes,
and stops with a recorded failure rather than silently changing parameters.
It survives SSH disconnection; automatic reboot recovery is not configured.
Status is `.ros2/graco/controller/status.json` and individual stages are in each
attempt's `progress.json`. Full output is
`.ros2/graco/runs/ground-01-06-full-20260917/report/REPORT.md`, with evo evidence,
individual raw/CBS ATEs, trajectory/map/BEV figures and Rerun recording.

References: [official GRACO dataset](https://github.com/SYSU-RoboticsLab/GrAco),
[sensor system](https://sites.google.com/view/graco-dataset/system).
