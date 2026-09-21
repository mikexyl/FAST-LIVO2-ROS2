# EllipseLIO + MapClosures + distributed CBS

The alternative frontend uses [V4RL/UCY EllipseLIO](https://github.com/v4rl-ucy/ellipselio),
pinned to `171acea502f1122e3043a460d2e6e3a30d8f8246` (MIT), on local branch
`dev/s3e-mapclosures-cbs`. The paper is Border and Chli,
[EllipseLIO: Adaptive LiDAR Inertial Odometry with an Ellipsoid Representation](https://arxiv.org/abs/2605.21150).
The project adaptation adds synchronized exports and reliable replay input;
it retains the upstream filter, adaptive scan filtering and ellipsoid map.

## Run

From the workspace `src` directory, using the existing research and CBS dependencies:

```bash
git clone https://github.com/v4rl-ucy/ellipselio.git ellipselio
# Use this project's patched EllipseLIO branch before building.
bash FAST-LIVO2-ROS2/scripts/build_ellipselio.sh
FAST-LIVO2-ROS2/scripts/s3e_experiment.sh run --stage all \
  --config FAST-LIVO2-ROS2/research/configs/square1-ellipselio-mapclosures-cbs.yaml \
  --resume
```

The clone command is only needed in a new workspace. The upstream main branch
alone does not implement this project's export interface. The current local
adaptation is in `../ellipselio`; no remote publication is implied.

Stages can run separately with `--stage odometry`, `descriptors`, `dpgo`, or
`dpgo_evaluate`. Pass the printed registry to `--input-run` for subsequent
stages. Odometry/keyframe/descriptor labels use `ellipselio`; FAST-LIVO2 uses
`livo`. Input registries with a different frontend are rejected. The existing
FAST-LIVO2 single-robot runner remains available.

## Sensor and export contract

Each robot consumes its raw `/ROBOT/velodyne_points` and `/ROBOT/imu/data`
topics. The VLP-16 GEODE configuration supplies the upstream algorithm
settings, with S3E's per-robot LiDAR-to-IMU extrinsics, IMU frequency and noise
parameters. Range filtering is 1–100 m. Point times retain their signed
FLOAT32 seconds relative to the cloud header; EllipseLIO already handles
S3E's negative offsets. Camera count is zero.

EllipseLIO normally adaptively downsamples input before deskew. The optional
export path separately retains all valid range-filtered points within each
processed time interval, deskews them using the same frozen IMU states, and
associates them with the final LiDAR-update pose before map insertion. The
reference timestamp is the exact IMU state selected by upstream deskew,
which may be slightly later than the scan-end timestamp. Failed LiDAR updates
are recorded explicitly. Initial IMU/gravity warm-up is excluded from exports.

The MCAP contains standard PointCloud2 and Odometry messages, plus a frame
index with integer nanoseconds, transforms, calibration and update status.
There are no image channels or placeholder images. Clouds are in LiDAR
coordinates and poses are in IMU/body coordinates. Keyframe construction
applies the recorded extrinsic once. Marginal covariance is diagnostic only.
The raw evidence queue is bounded at two million points, and the writer queue
at two packets; overflow or incomplete shutdown cannot produce a completed cache.
An explicit finish service serializes writer closure with mapping callbacks.
The separate IMU-rate odometry monitor uses best-effort QoS; its message count
is not a sensor-completeness count. Export completeness and gap statistics
refer to the direct scan/state frame index.

## Distributed backend

The selected path is `EllipseLIO → MapClosures → small_gicp verification →
distributed PCM → CBS pose + live GICP factors`. It keeps the working 1 m /
10° / 2 s keyframes, trailing five-second local submaps, 0.25 m submap voxels,
80 m loop-geometry crop and 30 s same-robot exclusion. MapClosures density/ORB/HBST
retrieval and native RANSAC initialization run independently on each robot.
Each query may select one LiDAR candidate, subject to the existing cooldown.
MegaLoc and CUDA inference are never started in `backend.name: mapclosures`.

Three isolated loop workers exchange descriptors and selected geometric
evidence over Fast DDS. Three CBS processes apply distributed PCM and the
existing registration-factor protocol. The experiment replays odometry
serially before these processes consume frozen artifacts. It validates an
offline distributed backend, not simultaneous live sensor operation on three
physical machines. Corrections apply to saved trajectories and maps only.

## Evaluation

`evo 1.36.5` performs timestamp association, rigid alignment and translation
APE. Raw odometry uses one independent fit per robot; connected CBS poses
use one shared fit for all robots. Scale stays one, timestamp tolerance is
0.05 s, and placeholder GT orientations are unused. These different alignment
scopes do not support interpreting raw-minus-CBS ATE as an improvement.
Rerun 0.37.1 displays robot-colored original/optimized maps, trajectories,
loop edges, registration overlays and convergence. Generated map points are
LiDAR geometry, without camera RGB.

## Ellipsoid-map BEV diagnostic

The optional `research.ellipsoid_stamps_ns` integer timestamp array requests
snapshots of the fitted native map. `research.ellipsoid_range_m` defaults to
80 m. When a schedule is supplied, export occurs after `MapIncremental` under
the map lock; the scan and final state still share their exact deskew timestamp.
Without this option, the existing export path is unchanged.

An additional PointCloud2 channel, `/ROBOT/research/ellipsoids`, stores centers
in the keyframe IMU frame, geometric semi-axis lengths `a,b,c`, the row-major
orthonormal basis `v00`–`v22` (axis directions are its columns), and native map
IDs and primitive types. Axes are **not covariance eigenvalues**. Both proper
and improper orthonormal eigenvector bases describe valid ellipsoids. Every
fitted primitive inside the crop is exported, unlike the sparse marker topic.
Initialization can legitimately have zero fitted primitives. The frame index
records requested and actual timestamps, source, count and crop.

The bounded diagnostic samples actual ellipsoid surfaces at a nominal 0.125 m
spacing, applies 0.25 m voxel centroids, and calls unchanged native MapClosures
density/ORB/HBST/RANSAC. It uses the existing 0.5 m BEV, 0.05 density threshold,
Hamming 50 and more than five RANSAC inliers. Common small_gicp verification
uses the same fresh raw submaps for both inputs. This is an offline rendering
diagnostic; the production distributed branch continues to use its raw submaps.

From workspace `src`, with fresh output paths:

```bash
export PYTHONPATH=FAST-LIVO2-ROS2/research
.ros2/research-venv/bin/python -m s3e_pipeline.ellipsoid_bev prepare \
  --reference FAST-LIVO2-ROS2/research/figures/ellipselio-square1 \
  --work .ros2/ellipse-bev-square1 --duration 43
for robot in Alpha Bob Carol; do
  OMP_NUM_THREADS=1 bash FAST-LIVO2-ROS2/scripts/run_ellipselio.sh \
    --robot "$robot" --bag /data/s3e/S3E_Square_1 \
    --mapping-config ".ros2/ellipse-bev-square1/$robot.yaml" \
    --output ".ros2/ellipse-bev-square1/$robot" --duration 43 || break
done
.ros2/research-venv/bin/python -m s3e_pipeline.ellipsoid_bev evaluate \
  --work .ros2/ellipse-bev-square1 \
  --output FAST-LIVO2-ROS2/research/figures/ellipsoid-bev-square1
.ros2/research-venv/bin/python \
  FAST-LIVO2-ROS2/research/s3e_pipeline/ellipsoid_bev_report.py \
  FAST-LIVO2-ROS2/research/figures/ellipsoid-bev-square1
.ros2/rerun-venv/bin/python \
  FAST-LIVO2-ROS2/research/s3e_pipeline/ellipsoid_bev_report.py \
  FAST-LIVO2-ROS2/research/figures/ellipsoid-bev-square1 --rerun
```

The selected snapshots and pair list are frozen before rendering. No ground
truth enters selection or registration. The small causal retrieval check uses
only selected snapshots available by query time; it is not a full distributed
replay. See [the results and images](RESULTS-ELLIPSOID-BEV.md).

## Full-sequence ellipsoid BEV comparison

The completed [Square 1 comparison](figures/ellipsoid-bev-full-square1/REPORT.md)
achieves 1.1408 m shared centralized-PGO ATE with ellipsoid BEVs and 1.1457 m
with raw BEVs. Both use the same fresh odometry and connect all three robots.
Large intermediates have been retired; poses, constraints, evo results, source
snapshots, figures and a small Rerun recording are retained in the report folder.

The second [paired run on Square 2](figures/ellipsoid-bev-full-square2/REPORT.md)
achieves 0.4641 m shared ATE with ellipsoid BEVs and 0.4703 m with raw BEVs.
Both graphs connect all three robots. Carol required half-speed replay after a
native synchronization stall; the failed attempt and successful retry are documented.

The full comparison retains the same surface sampling and native MapClosures
settings. `build_ellipsoid_cuda.sh` builds a separate CUDA sampler for this
machine's compute-8.9 GPU. It reproduces the two-stage voxel-centroid operation
with deterministic integer sums at 0.1 µm precision. Validation against three
retained real maps reproduced the original density pixels, ORB positions and
descriptor bits exactly. GPU inference/model environments are not involved.

Full map exports use `research.ellipsoid_world_frame: true` and the runner option
`research.ellipsoid_delta_export: true`. Native world coordinates make unchanged
ellipsoids bitwise stable. The writer saves changed rows and removed map IDs as
lossless NPZ deltas beside the raw-cloud/state MCAP; all delta files are included
in its completion manifest. These are still complete causal spatial snapshots,
not a sampled subset of ellipsoid centers.

`s3e_pipeline.ellipsoid_full` exposes independently rerunnable `prepare`, `solve`
and `evaluate` stages. Preparation reads each completed robot export, associates
the fixed keyframe schedule, and writes one shared raw geometry store plus
separate raw and ellipsoid descriptors. The existing isolated workers perform
causal detection for each branch. Both graphs use identical centralized GNC-TLS
settings and are evaluated on dense corrected trajectories through evo, using
one shared rigid alignment per connected component. Preparation and detection
never read position GT.

For a prepared full-run work directory:

```bash
bash FAST-LIVO2-ROS2/scripts/build_ellipsoid_cuda.sh
bash FAST-LIVO2-ROS2/scripts/run_ellipsoid_full_preparation.sh \
  .ros2/ellipse-bev-full-square1
bash FAST-LIVO2-ROS2/scripts/finish_ellipsoid_full.sh \
  .ros2/ellipse-bev-full-square1 \
  FAST-LIVO2-ROS2/research/figures/ellipsoid-bev-full-square1
```

The work directory's `schedule.json` contains all prior odometry keyframe
timestamps, selected independently of loops and GT. Its per-robot YAML files
enable the native world-frame/delta options above. New replays use the regular
`run_ellipselio.sh` runner serially. The preparation script can run alongside
replay and waits for completed exports, using one GPU job at a time.

For another sequence, initialize a new work directory with an explicit dataset
configuration and a frozen schedule from retained **raw** odometry. For example:

```bash
.ros2/research-venv/bin/python FAST-LIVO2-ROS2/scripts/prepare_ellipsoid_sequence.py \
  --dataset /data/s3e/S3E_Square_2 \
  --work .ros2/ellipse-bev-full-square2 \
  --schedule-trajectories FAST-LIVO2-ROS2/research/figures/cbs-square2/raw-odometry \
  --schedule-frontend FAST-LIVO2 --mapping-templates .ros2/ellipse-configs
```

Run each robot through `run_ellipselio.sh` with this work directory's robot YAML
and the specified dataset, then use the preparation/finishing scripts above with
the new work and output paths. All stages read `work/config.yaml`. The two BEV
branches always share the fresh EllipseLIO states and actual export timestamps.
The prior frontend supplies only the keyframe schedule; it does not supply graph
poses, loop proposals or ground truth. A first schedule entry before filter
initialization can use the first valid export within two seconds, recorded in
the audit; subsequent entries retain the 250 ms association guard.

The full report records native descriptor construction time, retained ORB counts,
CUDA sampling time and aggregate verification task time separately from detection
wall time. ORB counts are features per view, not matched correspondences. No
retrieval or acceptance threshold changes are needed for a new sequence.

For Laboratory 1, the released GT contains only endpoint records labeled 0 and
1. Evo records ATE as unavailable. The paired pipeline retains corrected maps,
separate component trajectory plots and middle-keyframe density BEVs with ORB
features. Maps use the same raw scans for both graphs, 0.15 m visualization
voxels and a 150,000-point cap per robot. The plan view is a one-metre slice
around each component's median trajectory height; Rerun retains the full height
range. Disconnected components are displayed separately, without fabricated GT
or inter-component alignment. These views and loop counts support inspection,
but do not establish a trajectory-accuracy improvement.
The completed [Laboratory 1 paired result](figures/ellipsoid-bev-full-laboratory1/REPORT.md)
connects all robots with 51 ellipsoid-BEV loops (14 inter), while raw BEVs retain
six intra-robot loops and three components after GNC. Both use the same fresh
0.5x replay, with no retries or detector tuning. Detection takes 13.12 s versus
3.32 s, excluding preparation; no ATE is reported.

`ellipsoid_full_archive` repeats frozen PGO, checks that odometry and descriptors
remain unchanged, and retains complete poses, constraints, evo evidence,
transcripts and provenance. Its explicit `--retire-large` option removes the
temporary raw MCAP, map deltas and large geometry stages only after validation.
