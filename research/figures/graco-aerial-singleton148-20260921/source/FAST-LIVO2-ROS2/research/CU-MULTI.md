# CU-Multi setup on workstation 148

As of **2026-09-17 12:20 UTC**, the user requested a pause: the CU-Multi download on 148 is **paused**, and the automatic test watcher is suspended. About **93.3 GB** of the 197.4 GB transfer had arrived; partial files are retained. Globus Connect Personal reports `paused` / `idle`; the remote transfer task remains active and awaiting the endpoint. There are **no CU-Multi trajectory results yet**. After an explicit resume and verified transfer, the configured job runs a **120-second-per-robot smoke test**, not a full-sequence benchmark. Resume instructions and the watcher PID are recorded in the controller's `status.json`.

The pipeline is **EllipseLIO → native ellipsoid BEVs → MapClosures → distributed PCM → CBS with live GICP factors**. It supports `robot1`, `robot2`, `robot3` and `robot4` throughout odometry export, descriptor preparation, independent DDS workers, optimization, evo, plots and Rerun. Sensor replays are serial; the distributed backend runs four loop workers and four CBS processes.

## Dataset and sensor contract

The existing `/mnt/data/cu-multi/main_campus` copy contained RGB/depth, IMU/GPS, GT and semantic-label archives, but **no raw LiDAR archives**. Labels cannot replace XYZ measurements or point timestamps. The missing LiDAR archives total **197,430,380,765 bytes**; their paused destination is `/data3/mikexyl/datasets/cu-multi`. The 54.5 MB calibration archive has downloaded and its URDFs were extracted. Original archives are retained.

Sources: [official CU-Multi repository](https://github.com/arpg/CU-Multi), inspected at commit `705d497ca88348c8a80d5ab9ae7fa17d918ab348`; [dataset paper](https://arxiv.org/abs/2509.19463); [official Globus collection](https://app.globus.org/file-manager?origin_id=ae3a873e-d159-4e7b-8a57-9be2699eea52&origin_path=%2F). CU-Multi consists of separate recordings arranged as a four-robot benchmark. The published synchronized timestamps are preserved.

The adapter selects only these two streams per robot:

| Input | Topic | Configuration |
|---|---|---|
| Raw Ouster LiDAR | `/robotN/ouster/points` | Native Ouster format, 64 beams, 20 Hz; real per-point `t` in nanoseconds |
| Raw LORD IMU | `/robotN/imu/data` | 500 Hz; recorded `robotN_imu_link` frame |

GNSS, EKF-derived measurements, GT, semantic labels and camera streams are excluded from estimator inputs. The external LORD IMU is used with the published URDF calibration. For each robot, the adapter computes `T_imu_cloud` from the actual cloud and IMU frame names, following fixed joints in either direction. It rejects unknown frame chains. The downloaded calibration gives the Ouster-to-LORD transform approximately:

```text
R_imu_sensor = diag(-1, -1, 1)
t_imu_sensor = [-0.062860, 0.015570, 0.053345] m
```

The tiny nonzero off-diagonal terms in the URDF's rotation are retained. The initial IMU noise configuration is explicitly **a declared model, not a dataset-calibrated estimate**: `acc_noise=0.1`, `gyr_noise=0.01`, `acc_bias=0.0001`, `gyr_bias=0.0001`, using EllipseLIO's parameter conventions. Actual point fields and LiDAR frame names will be checked when the raw archives arrive; a format mismatch fails preparation instead of fabricating point times or assuming an identity extrinsic.

ROS2 sensor staging copies serialized LiDAR/IMU payloads unchanged, normalizes only topic leading slashes, and uses the original integer header timestamp as the bag record time. It does not rebase robot clocks. Chronology, IMU coverage, gaps, finite measurements and Ouster point times are checked. CRC-checked extraction and hashed completion markers distinguish completed inputs from partial downloads/extractions. At least 100 GiB free space must remain during extraction. The original S3E source snapshot and results are untouched by this deployment.

Keyframes retain the working 1 m / 10° / 2 s schedule, 80 m ellipsoid map crop, 0.125 m surface sampling and 0.25 m voxel centroids. Retrieval uses ellipsoid BEVs; trailing raw submaps supply geometric verification and GICP evidence. PCM uses probability 0.99 and minimum clique size 2.

## Evaluation

Evaluation reads each robot's `*_gt_utm_poses.csv`, retaining the common UTM frame and decimal timestamps at nanosecond precision. These are the dataset's **LIO-SAM2 + RTK GPS reference solutions**, rather than independently measured exact poses. The published LiDAR reference pose convention is used; the Ouster-to-LORD lever arm transforms reference positions to the estimator's IMU origin. Reference orientations are used for that calibration transform, while the metric remains translation ATE.

Evo 1.36.5 performs nearest timestamp association within 50 ms, one shared SE(3) fit per connected component, and no scale fitting. Individual CBS ATEs use the same component fit. Raw odometry gets separate robot fits and is labeled accordingly. Disconnected components and unavailable scores remain explicit. GT is read only during evaluation; it cannot initialize CBS or select loops.

## Validation completed

- **18 local adapter/regression tests passed**, covering calibration direction, rotated lever arms, integer timestamp precision, sensor allowlisting, partial caches, four-robot shared evo alignment, existing S3E evaluation and image-free exports.
- **Native four-robot DDS/PCM/GICP test passed on 148**: 30 proposed synthetic closures, 18 retained, 12 injected false closures excluded, and 18 live GICP factors. Recovered poses satisfied 2 mm translation / 0.001 rad rotation tolerances. This is a synthetic integration fixture, not a CU-Multi accuracy result.
- **Native ROS2 staging test passed**: identical serialized payloads and header timestamps, correct merged ordering, and GT absent from the staged sensor bag.
- A real-data probe read **59,998 raw robot1 IMU measurements** over the first 120 seconds; timestamps were increasing and the largest gap was **14.240 ms**. All four downloaded URDFs produced consistent sensor transforms.

[Validation summary](figures/cu-multi148-validation/validation.json), [local regression test XML](figures/cu-multi148-validation/local-tests.xml), [native/adapter test XML](figures/cu-multi148-validation/four-robot-tests.xml), [ROS2 staging test XML](figures/cu-multi148-validation/rosbag-staging-tests.xml), [synthetic PCM decisions](figures/cu-multi148-validation/synthetic-pcm.json).

The inherited Python environment emits an Axes3D import warning; the tested 2D report plots passed. This does not establish the cause of earlier S3E report crashes. Native ROS launch-testing plugins are disabled for these pytest runs because their installed hooks are incompatible with the current pytest version.

## Persistent job and inspection

The dedicated container is `ellipsoid-cu-multi148`, using the validated image `sha256:cb89f7676e64520d3c6acfb37be9006b1b1dd52019d928b09f8ed7315ceb3ecc`. It has 16 CPU cores, 48 GiB RAM, GPU access and read-only dataset mounts. The CU-Multi Python source is overlaid from `/data3/mikexyl/swarm_s3e_ws/src/.ros2/cu-multi/source/FAST-LIVO2-ROS2`, separate from the completed S3E run's source. ROS domain 187 separates the experiment.

The host launcher runs independently of SSH. It waits up to 24 hours for the checksum-verified Globus transfer, then performs one bounded test. It records failures and stops; it does not retry with changed parameters. A six-hour bound covers input extraction and the smoke pipeline. Completion of the download alone is not completion of the test.

```bash
# Current phase and download progress:
ssh 148 cat /data3/mikexyl/swarm_s3e_ws/src/.ros2/cu-multi/controller/status.json

# Once the test starts:
ssh 148 tail -40 /data3/mikexyl/swarm_s3e_ws/src/.ros2/cu-multi/controller/smoke.log
ssh 148 cat /data3/mikexyl/swarm_s3e_ws/src/.ros2/cu-multi/runs/main_campus-smoke-20260917/progress.json
```

Globus LiDAR task: `1c69efa4-b268-11f1-a606-0affd5e180af`. Calibration task: `16badbc0-b268-11f1-a0ab-0effcb3df825`. Transfer receipts are in `/data3/mikexyl/datasets/cu-multi/`.

The expected smoke report is `/data3/mikexyl/swarm_s3e_ws/src/.ros2/cu-multi/runs/main_campus-smoke-20260917/report/REPORT.md`. It is valid only when that directory also contains `COMPLETE.json`. The Rerun recording, maps, trajectories, evo artifacts and PCM decisions will be saved alongside it.

## Reproduction

Inside the dedicated container, use the existing runtime wrapper and a fresh work directory:

```bash
bash FAST-LIVO2-ROS2/scripts/ellipsoid_cbs148/env.sh \
  .ros2/research-venv/bin/python FAST-LIVO2-ROS2/scripts/run_cu_multi.py \
  --stage all --environment main_campus --duration 120 \
  --work /workspace/.ros2/cu-multi/runs/main_campus-smoke-new
```

`--stage prepare` performs extraction, sensor checks, bag staging and calibration/configuration export. `--stage run` consumes that prepared attempt after checking its hashes and runs odometry, descriptors, PCM/CBS and evaluation. `--duration 0` selects a full sequence; that will require a fresh attempt and substantially more generated disk space. The currently queued job is the 120-second smoke. Kittredge Loop is accepted by the adapter but has not been downloaded or tested.

[Dataset adapter](s3e_pipeline/cu_multi.py), [experiment runner](../scripts/run_cu_multi.py), [persistent launcher](../scripts/cu_multi148/wait_and_test.py).
