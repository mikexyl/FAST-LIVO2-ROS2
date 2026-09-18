# Swarm-SLAM tests on workstation 148

SSH: `ssh 148`. Host: Ubuntu 24.04, Core Ultra 9 285K (24 cores), 64 GB RAM,
RTX 5080 (16 GB). This CPU LiDAR pipeline does not require CUDA or MPS.

Workspace: `/data3/mikexyl/swarm_s3e_ws/src`, mounted as `/workspace`.
S3E v1 data: `/mnt/data/s3e/S3Ev1`, mounted **read-only** as `/data/s3e`.
The host's Jazzy/Kilted installations and unrelated containers are unchanged.

The existing `agent-swarm-slam:latest` image is pinned by image ID:

```
sha256:cf4ef4e88e7ba797ae2e59b1eae31b8947bac4c7c6720832a0a11dea03711eed
```

It provides Ubuntu 22.04, ROS Humble, PCL, OpenCV, and Open3D 0.19.0.
The older image's Swarm installation is not sourced. Current working sources,
including the documented research-export/compatibility patches, were transferred
with rsync; source commits, diffs and installed dependency versions are retained
in `provenance/` and `.ros2/swarm/provenance`.

`build.sh` builds GTSAM 4.3a2 and TEASER++ into `.ros2/swarm/native`, then the
LiDAR-only Swarm ROS packages and EllipseLIO. TEASER's PMC, Spectra and tinyply
sources are transferred from the pinned local build, avoiding floating downloads.
Research evaluation uses a separate Python environment with evo 1.36.5 and
GTSAM 4.2. Native Swarm links the isolated GTSAM 4.3 library; `pgo-ldd.txt`
records the resolved libraries. The initial remote build passed 29 tests;
the final suite, including the added frontend validity guard, passed 32 tests.

The 45-second three-robot integration replay passed: 447 scans per robot, 95
keyframes/descriptors per robot, no missing acknowledgements, complete native
optimized outputs, and no optimizer errors. An independent input audit found
zero selected timestamp/pose differences. The three identical-input trajectories
share one native origin; their maximum corresponding-position difference is
0.0612 m (RMS 0.00924 m). These fixture checks are not sequence ATE.

From the remote workspace, start the build using the pinned image:

```bash
docker run --rm --user "$(id -u):$(id -g)" --shm-size 2g \
  -v "$PWD:/workspace" -v /mnt/data/s3e/S3Ev1:/data/s3e:ro -w /workspace \
  --entrypoint bash \
  sha256:cf4ef4e88e7ba797ae2e59b1eae31b8947bac4c7c6720832a0a11dea03711eed \
  -lc 'bash Swarm-SLAM/s3e/remote/build.sh'
```

Use the same mounts/image with
`python3 Swarm-SLAM/s3e/remote/experiment.py` to run a 60-second Alpha
EllipseLIO smoke export, a 45-second duplicate-stream three-robot replay, and
Library 1 / Campus Road 1. `--smoke-only` stops after the integration test;
`--sequences library1` selects one full sequence. Existing successful frontend
exports and completed native attempts are reused; failed exports require inspection.

Frontend replay is serial at 1x. Each native test runs all nine Swarm processes
(LiDAR handler, loop detector and pose graph manager for each robot) concurrently.
This is a multistage benchmark using saved EllipseLIO inputs, not an untouched
official online launch. The user explicitly chose to retain this execution mode
on 2026-09-16 after reviewing that distinction. Upstream also supplies RTAB-Map
odometry launch examples; those frontends are not used in this experiment.
Ground truth is used only by the evaluator. Transport is reliable and exactly
acknowledged; the original Python Scan Context and registration thresholds remain.
The native stage has a one-hour wall budget and 120-second maximum final drain.
Failure is recorded and does not prevent testing the next sequence.

The input guard rejects nonfinite/nonchronological poses, interrupted captures,
and scan-to-scan speed above 20 m/s (a conservative S3E ground-robot sanity limit).
It uses no GT and makes no estimator changes. It was added after Campus Road 1's
Alpha export remained finite while diverging beyond 1000 m/s; the sequence was
stopped before Swarm-SLAM. Library 1 passes this check retrospectively. Failed
frontend attempts receive a separate report and no invented Swarm ATE.

Live state: `.ros2/swarm148/progress.json`, `experiment.log`, and each sequence's
`swarm/progress.json`. Results: `FAST-LIVO2-ROS2/research/figures/swarm148-<sequence>`.
Reports retain configurations, source patches, exact frame indices, native outputs,
evo ZIPs, GT file hashes, and PNG/PDF plots. Raw odometry ATE uses independent
robot fits; a connected Swarm result uses one shared SE(3) fit without scale.
Input replay validity does not determine whether an actual trajectory can be
measured, and incomplete native output is never filled with raw odometry.

`report.py --output <report> --rerun` uses Rerun 0.37.1 to create a compact recording
from `trajectory-view.npz`. It can run locally after copying the report back.
