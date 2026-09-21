# Recent-history EllipseLIO comparison

Temporal submapping remains the default. An experimental `spatial` strategy is
also available; see [SPATIAL-SUBMAPS.md](SPATIAL-SUBMAPS.md). The older motion/overlap
experiment remains retired; its source, configuration and reports remain under
`research/figures/motion-submaps` as historical evidence.
The [local rollback verification](../../research/figures/temporal-submaps-rollback/README.md)
records the successful build, temporal native tests and distributed integration checks.

Native `mapping.submaps` defaults to `{enabled: false, duration_s: 10, overlap_s: 5}`.
Two map buffers own independent points, octrees, tensors, filters, neighbours and
neighbourhood statistics. The first initialized scan fixes the sensor-time epoch.
Windows are half open: `[0,10)`, `[5,15)`, `[10,20)`, … . A boundary scan matches
against the successor before it is inserted into either applicable map. No
archived map is queried. Two buffers support `0 < overlap <= duration/2`.

The filter, covariance, trajectory and calibration persist across handovers.
Empty/one-feature and non-finite correspondence reductions fail the LiDAR update
and retain IMU propagation. Both comparison modes share these guards, the safe
short-trajectory index, and `-O3` compilation (`-Ofast` invalidates finite checks).
Thus the fresh baseline is a guarded persistent-map baseline, not the old binary.

Completed snapshots use the existing two-packet bounded writer. `index.jsonl`
contains schema version, robot/submap IDs, scan membership, sensor window,
last member time, anchor pose time, availability time and SHA-256 payload hashes.
NPZ payloads contain native processed member-scan points and fitted ellipsoids in
the last member scan's corrected IMU frame. These are **not full-resolution raw
scans**. The strict separate research deskew exporter stays disabled and its
assertion is unchanged. `/ROBOT/mapping/finish` drains snapshots and writes two
inspection-only tails when both buffers are nonempty; tails are not retrievable.
The writer creates a completion manifest only after its explicit end record.

`recent_submaps.prepare` makes one ellipsoid-surface descriptor and one evidence
cloud per completed submap using the existing projection and preprocessing.
The default `backend.mapclosures.projection_alignment: gravity` uses the native
anchor's `gravity_imu_m_s2` to make an orthographic horizontal density image.
Snapshots also record `gravity_world_m_s2` and `gravity_source` at the anchor.
The full IMU-to-level rotation is stored as the descriptor's `ground` transform,
so MapClosures loop poses are recovered in the original IMU endpoint frames.
Evidence geometry is never rotated in its saved store. Density resolution,
surface sampling, ORB extraction and registration thresholds are unchanged.

This bypasses the native sparse local-ground fit: 0.25 m voxelized surfaces
cannot meet its ten-points-per-0.5-m-voxel normal threshold. Gravity determines
roll/pitch, not terrain height; the projection transform has zero translation.
Thus a 2D loop hypothesis still lacks relative vertical displacement unless
subsequent 3D registration can recover it within the existing capture range.

Historical exports lack gravity. Preparation fails explicitly unless given a
payload/index-bound `--gravity-sidecar` reconstruction, or explicitly configured
with `projection_alignment: local_ground` to reproduce legacy behavior.
`graco_gravity148.py` reuses the four aerial captures, reconstructs gravity from
the first 126 accelerometer samples and saved poses, and labels it as an
approximation rather than the exact historical filter state. It runs the full
backend before accessing evaluation truth and preserves all original outputs.

Local consecutive keyframe IDs retain `submap_id` as provenance (including any
empty windows skipped after timestamp gaps). The DDS and deterministic workers
schedule availability timestamps; graph nodes retain anchor timestamps. They
keep the existing same-robot exclusion and additionally reject overlapping
windows. Descriptor loading checks evidence membership and source payload hash.

Exact native per-scan logs supply the trajectory and successful-update gate.
Rerun also records native analytics, clears active geometry at handover, and
shows immutable archived snapshots separately. ROS publisher captures may lose
messages; exact native logs are the evaluation authority.

## Historical isolated workstation 148 trial

Sources and outputs live under `/workspace/.ros2/recent-submaps` in the container
`ellipselio-recent-submaps-build148`, mounted from
`/data3/mikexyl/swarm_s3e_ws/src/.ros2/recent-submaps`. Builds and installation do
not replace the historical binaries. The pinned image, 12-core quota, 40 GiB
limit, four-thread launcher, OMP=4, requested noise and reliable input are shared.
These quotas describe the original experiment only. Future work on 148 must
use its available resources without CPU/RAM/GPU quotas, per the user's request.

After sourcing ROS and `.ros2/recent-submaps/install/setup.bash`, run fresh names:

```bash
/usr/bin/python3 FAST-LIVO2-ROS2/scripts/recent_submaps/run_trial.py bob-baseline
/usr/bin/python3 FAST-LIVO2-ROS2/scripts/recent_submaps/run_trial.py bob-enabled --enabled
```

The runner never launches loops or CBS. It saves source/binary/config hashes,
exact native updates, memory samples, ROS logs and a live Rerun recording.
`validate.py TRIAL` audits snapshot membership, chronology, hashes and anchors.
`evaluate.py TRIAL --truth bob_gt.txt --output REPORT` delegates association,
rigid alignment and position ATE to evo
1.36.5 with 50 ms association and no scale fitting.

Rollout requires full completion, finite chronological poses, maximum speed
≤20 m/s, no post-initialization successful-update gap ≥1 s and ATE ≤5 m. Repeat
only a passing enabled run. Alpha/Carol and the distributed backend are gated on
that confirmation. Failure ends the experiment without a parameter sweep.

After that gate, `prepare_rollout.py` generates robot configurations from the
S3Ev2 calibration files, and `run_trial.py --robot Alpha|Carol --mapping-config
CONFIG --enabled NAME` records the other robots. Run `rollout.py` through
`scripts/ellipsoid_cbs148/env.sh`, supplying `--work`, all three trial paths and
the two Bob metric JSON files as `--gate` and `--repeat-gate`. The work directory
must be new. It prepares descriptors and launches the existing isolated DDS
workers with PCM and live GICP factors. It never reads ground truth.

Once optimization finishes, `rollout_report.py --work WORK --alpha TRIAL --bob
TRIAL --carol TRIAL` evaluates the raw and corrected dense trajectories. Only
the evaluation programs read ground truth. Raw trajectories receive individual
rigid fits; CBS trajectories receive one shared fit per connected component,
with per-robot errors under that same fit. `rollout_rerun.py WORK` creates a
separate derived map/trajectory recording using the pinned Rerun environment.
Verify both live and derived files with `rerun rrd verify` before running
`retention_audit.py ROOT`. Registration transport geometry is retained through
report generation; no previous experiment is overwritten.

Native tests are `submaps_test` and `research_writer_test` in the isolated
EllipseLIO build. Python coverage is in `test_recent_submaps.py` and
`test_recent_submaps_integration.py`, alongside the existing pipeline tests.
The native integration test needs `S3E_TEST_DDS=1` and the CBS environment;
set `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` to avoid loading incompatible ROS pytest
plugins that these tests do not use.

## Previously failed frontend retries

`failed_frontends.py prepare` freezes the historical calibrated configurations
for Laboratory 4/Carol, Campus Road 1–3/Alpha, and GRACO ground-01/robot1 in a
fresh `failed-frontends-20260918` directory. S3E uses the requested noise from
the successful Library 2 run; GRACO preserves its original ground noise.
`failed_frontends.py run`, in the same ROS-sourced experiment environment,
replays them serially without loops or CBS and evaluates/validates each one.
This fixed experiment path must be changed to a fresh path for another attempt.

`run_trial.py` accepts `--bag`, `--robot`, `--mapping-config`, and `--output-root`.
`--submap-strategy spatial` enables the new spatial policy and merges the
BEV-oriented defaults from `spatial_submaps.yaml` into a fresh copy of the supplied
calibrated mapping config. Explicit spatial settings in that config take precedence.
The recorder reads the bag start time and configured IMU topic; sensor geometry,
calibration and estimator behavior remain controlled by the mapping YAML.
`sensor_coverage.py TRIAL` checks the selected sensor tail against the source
SQLite bag. `replay_status.py` reads progress without modifying the queue.
`failed_frontends_report.py` collects completed evidence and makes the trajectory
figure; `failed_frontends_audit.py` hashes recordings and checks controls.
The historical GRACO capture's separate `evaluation-graco-reference` corrects
only the reference-frame description; its numeric metrics are identical.

## Full five-group pipeline

`full_batch.py prepare --work NEW_DIRECTORY` freezes Laboratory 4, Campus Road
1–3 and GRACO ground-01–06. It reuses the five validated submap captures and
generates calibrated configurations for thirteen missing robot frontends.
`full_batch.py run --work NEW_DIRECTORY --workers N` selects frontend concurrency
at 1× in independent ROS domains 90, 92, 94, …, with four native executor
threads and OMP=4 per frontend. Three and six concurrent workers are tested;
choose concurrency for available resources. The longest replays start first.
`env.sh` sources the native submap and CBS overlays in the experiment container.

After all frontend jobs finish, each group proceeds through completed-submap
ellipsoid BEV preparation, isolated MapClosures workers, distributed PCM/CBS
with GICP factors, evo evaluation, map export and verified Rerun output.
These compute-heavy backend stages run separately from frontends. A group with
an incomplete or unstable frontend is reported and skipped; other groups
continue. Raw ATE and GT availability never gate backend admission. No estimator,
retrieval, registration or optimization parameter sweep is performed.

`status.json` records each frontend, its ROS domain, quality result and any
failure, followed by group backend results. Sources and binaries are frozen in
`source-hashes.json`; existing batch directories cannot be rerun. The original
serial batch's short interrupted Alpha capture is retained with an explicit
parallel-relaunch annotation. New outputs are in
`.ros2/recent-submaps/full-five-groups-parallel-20260919` on workstation 148.
