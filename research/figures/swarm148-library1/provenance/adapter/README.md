# Swarm-SLAM LiDAR on S3E

This adapter runs the upstream ROS2 LiDAR frontend and elected-robot pose graph
optimizer for Alpha, Bob and Carol. The observer publishes synchronized inputs,
records native outputs, and evaluates completion. It does not perform retrieval,
registration, or optimization. The original project and its license remain in
the enclosing checkout and `src/cslam`.

The benchmark uses the same fresh EllipseLIO deskewed clouds and odometry as our
raw/ellipsoid BEV MapClosures comparison. Complete scans are transformed from
LiDAR coordinates into the associated body frame before publication. GT enters
only evaluation. This is a comparison of complete methods, with different native
keyframe schedules, map history, factor noise, and loop selection.

## Pinned sources and dependencies

| Repository | Commit |
|---|---|
| MISTLab/Swarm-SLAM | af17c4b71432f750571ea699a0beccfbe9004872 |
| lajoiepy/cslam | fccf88765167431d293a1245c19411fd874f937d |
| lajoiepy/cslam_interfaces | 0e6bb91411796db593c8ce680f382a1f242ab7d9 |
| lajoiepy/cslam_experiments | 0b5fc6c9354f75ed99971aa6c6e04b6a91141689 |
| introlab/rtabmap_ros (Humble) | c25a091c2a682f0e0f4e8d19d807953e7f1a8161 |
| MIT-SPARK/TEASER-plusplus | 52a9c52ee7d4c838c5e8a75458c33178be5bfb70 |
| borglab/gtsam | 0c2049c0352e5b24a7a19f29610cf6c580e38ef8 |

The umbrella checkout and cslam use branch `dev/s3e-lidar-comparison`.
The umbrella's upstream `.gitignore` excludes `src/`; the cslam adaptation is
also retained as [patches/cslam.patch](patches/cslam.patch). It applies to the
cslam commit above. Nested checkouts are independent repositories.

This machine uses Ubuntu 22.04, ROS2 Humble, Python 3.10, GTSAM 4.3a2 for native
Swarm and TEASER++ from the existing sibling workspace. Our own PGO still uses
its separate GTSAM 4.2 environment. Swarm's Python dependencies are isolated in
`.ros2/swarm/venv`; [requirements.lock.txt](requirements.lock.txt) records its
installed packages, excluding system ROS packages. No pretrained model is used.
Scan Context and FPFH/TEASER++/ICP run on the CPU. CUDA is used only to prepare
our ellipsoid BEVs, one preparation worker at a time.

From the workspace `src` directory, after checking out the repositories above:

```bash
uv venv --python /usr/bin/python3 --system-site-packages .ros2/swarm/venv
UV_CACHE_DIR=.ros2/uv-cache uv pip install --python .ros2/swarm/venv/bin/python \
  -r Swarm-SLAM/s3e/requirements.lock.txt
cmake -S Swarm-SLAM/s3e/teaser_binding -B .ros2/swarm/teaser-build \
  -DCMAKE_BUILD_TYPE=Release \
  -DTEASER_SOURCE=/home/mikexyl/workspaces/sb_slam_ros2_ws/src/TEASER-plusplus \
  -Dteaserpp_DIR=/home/mikexyl/workspaces/sb_slam_ros2_ws/install/teaserpp/lib/cmake/teaserpp \
  -Dpybind11_DIR="$PWD/.ros2/swarm/venv/lib/python3.10/site-packages/pybind11/share/cmake/pybind11" \
  -DPYTHON_EXECUTABLE="$PWD/.ros2/swarm/venv/bin/python"
cmake --build .ros2/swarm/teaser-build -j2
bash Swarm-SLAM/s3e/build.sh
```

`build.sh` and `env.sh` prefer `.ros2/swarm/native` when present, with the original
sibling-workspace installation as a fallback. `SWARM_GTSAM_DIR` can override the
build dependency location. Only `rtabmap_msgs` is required from RTAB-Map
for this LiDAR build. The TEASER module uses the unchanged upstream binding source.

## Upstream adaptations

* `CSLAM_LIDAR_ONLY=ON` avoids the visual frontend build dependencies. The default
  full upstream build remains available.
* GTSAM 4.3 compatibility replaces old shared-pointer, quaternion, and Values
  filtering APIs. The native optimizer and factor settings are retained.
* LiDAR registration originally returned the transform that maps source cloud
  coordinates into target coordinates. For a factor with endpoints `(source,
  target)`, GTSAM requires the inverse. The adapter returns `R.T, -R.T @ t`.
  A real TEASER/ICP synthetic test checks both endpoint orders; a rotated
  synthetic test checks the complete transform convention.
* An optional `frontend.reliable_input` parameter enables reliable cloud delivery
  for offline replay; live input keeps the upstream best-effort default. Odometry
  uses a reliable queue of 100. The processing-stamp observer tracks each scan
  exactly: a later acknowledgement cannot clear an earlier missing stamp.
  Replay permits two outstanding scans and five unpublished descriptors per robot,
  waits for the native subscriptions and heartbeats, and records progress.
  The final drain is bounded at 120 seconds, independently of replay duration;
  `--max-wall-s` defaults to 3600. Interrupted runs preserve their summaries.
  These transport changes do not enter selection, retrieval or optimization.

The native `graco_lidar.yaml` provides 0.5 m distance keyframes, 0.5 m voxels,
Scan Context similarity 0.8, more than 60 registration inliers, a 20-keyframe
same-robot exclusion, and one selected inter-robot candidate per five-second
period. Spectral sparsification and vertex-cover selection remain enabled.
Exact input pairs use 1 ms synchronization tolerance. The operational backend
waiting timeout is 60 seconds; logs are enabled and the native visualizer disabled.

The original Python Scan Context implementation is retained without JIT or
algorithmic substitutions. This can make replay slower than sensor time.

## Running and evaluating

`prepare_frontends.py` regenerates Square 1, Square 2, and Laboratory 1 exports
serially at 1x, with a documented 0.5x retry only after failure. The estimator
parameters and frozen keyframe timestamp schedules come from the preceding
archived EllipseLIO experiments. Both comparison paths consume the fresh exports;
historical trajectory values must not be substituted for fresh comparisons.

```bash
.ros2/research-venv/bin/python Swarm-SLAM/s3e/prepare_frontends.py
bash Swarm-SLAM/s3e/run.sh --work .ros2/swarm/square1 \
  --output .ros2/swarm/square1/swarm --tail-s 30
```

Use fresh output directories. Independent sequences use ROS domains 124, 125,
and 126; `ROS_DOMAIN_ID=125 bash ...` overrides the default. The current runs
share CPU resources; `OMP_NUM_THREADS=2` and `OPENBLAS_NUM_THREADS=1` are recorded
in `env.sh`. Sensor replay uses timestamp order and backpressure. A run succeeds
only after every input is acknowledged, every expected keyframe/descriptor is
observed, and native optimized messages cover all robot keyframes.

After our paired BEV report and the native run finish:

```bash
PYTHONPATH=FAST-LIVO2-ROS2/research .ros2/research-venv/bin/python \
  Swarm-SLAM/s3e/evaluate.py --work .ros2/swarm/square1 \
  --output FAST-LIVO2-ROS2/research/figures/swarm-comparison-square1
PYTHONPATH=FAST-LIVO2-ROS2/research .ros2/research-venv/bin/python \
  Swarm-SLAM/s3e/report.py FAST-LIVO2-ROS2/research/figures/swarm-comparison-square1
.ros2/rerun-venv/bin/python Swarm-SLAM/s3e/report.py \
  FAST-LIVO2-ROS2/research/figures/swarm-comparison-square1 --rerun
```

Evaluation uses evo 1.36.5, nearest timestamp association within 50 ms, one shared
SE(3) alignment per output component, and no scale fitting or extra per-robot fit.
Individual ATEs use that same shared alignment. GT orientations are unused and
the antenna lever arm is uncorrected. Laboratory 1 has only untimestamped endpoint
records, so trajectory ATE and GT loop labels are unavailable. Native accepted
registrations are reported before GNC; native messages do not expose GNC weights.
Output origin IDs do not establish a graph of GNC-selected factors.

Communication counts observed serialized CDR publications once, including local
deliveries, excluding DDS transport/discovery overhead and broadcast fanout. Our
simulation uses compressed envelopes, so the byte totals differ in meaning.
Swarm wall time includes replay, backpressure, timer budgets and settling; our
detection-only time excludes preprocessing. A direct speedup ratio is invalid.

## Verification and retained evidence

```bash
bash -c 'source Swarm-SLAM/s3e/env.sh
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .ros2/swarm/venv/bin/python -m pytest \
Swarm-SLAM/src/cslam/tests Swarm-SLAM/s3e/test_registration.py \
  Swarm-SLAM/s3e/test_replay_tracking.py -q'
```

Upstream unit tests, transform checks and exact replay accounting passed on
workstation 148 (29 tests). The earlier local 30-second
identical-stream smoke run processed all 900 scans and 144 keyframes/descriptors
and aligned the three robot copies with maximum corresponding-position difference
0.081 m. Our pipeline's separate regression suite passed 31 tests.

The compact report retains native messages, input hashes, configurations, CDR byte
counts, per-process CPU/peak-memory observations, logs, evo evidence, plots and a
Rerun 0.37.1 recording. Failures retain their diagnostics and are not relabeled as
completed caches. Retire raw clouds and preparation caches only after their
consumers finish and the report has retained the corresponding compact evidence.

## Workstation 148

The separate deployment is documented in [remote/README.md](remote/README.md).
It uses the workstation's existing ROS Humble Docker image and read-only S3E
datasets, with all new code, builds and results on `/data3`. Its sequence reports
measure the actual native trajectories separately from replay input validity.
