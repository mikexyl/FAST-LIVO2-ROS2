# Official Swarm-SLAM LiDAR on S3E — 148

The new fixed-configuration queue started **2026-09-17 09:45:35 UTC** on 148.
Square 1 is running; full-sequence results are pending. This report is separate
from the previous Swarm-SLAM tests fed by saved EllipseLIO odometry.

The runtime is **raw S3E LiDAR/IMU → RTAB-Map ICP → Swarm-SLAM Scan Context →
FPFH/TEASER++/ICP → native Swarm-SLAM GNC optimization**. All three robots run
simultaneously. There is no EllipseLIO, FAST-LIVO2, MapClosures, CBS, or PCM.

The official LiDAR components are adapted through the authors' S3E ICP launch,
upstream `graco_lidar.yaml` settings, and dataset topic remaps. The repository's
separate stereo-and-LiDAR S3E preset uses stereo loop detection; this experiment
continues the requested LiDAR-only comparison. Native cslam source and installed
Python files were checked against their pinned upstream checkout. The earlier
registration-direction patch is not applied.

[Setup, adaptations and reproduction](../../Swarm-SLAM/official_s3e/README.md),
[source and runtime provenance](figures/swarm-official148/provenance/sources.json),
[container image and mounts](figures/swarm-official148/provenance/container.json).

## Scope and timing

148 has 18 recordings: 12 in S3Ev1 and six in S3Ev2. Playground 1 is excluded as
previously requested, leaving **17 sequence attempts**. One configuration is
used per sequence; there is no parameter sweep. A sequence failure is recorded
and the next sequence is attempted.

The 17 recordings contain approximately **2 h 34 min** of sensor time. At the
official S3E example's **0.5×** replay rate, playback takes about **5 h 8 min**;
per-sequence startup and 120-second settling add approximately 37 minutes.
Expected batch duration is about six hours, not six hours per sequence.
This is online processing at a controlled playback rate, not a claim of full
real-time throughput.

## Completed startup and timing validation

All **25 upstream unit tests** and **two evo shared-alignment tests** passed.
An initial image ABI mismatch was resolved by rebuilding unchanged Swarm C++
against GTSAM 4.1.1 and isolating RTAB-Map's matching GTSAM 4.2 runtime. A missing
comma in the official launch was fixed. Fast DDS shared-memory segments were
increased from 512 KiB to 64 MiB to avoid large-cloud loss, retaining native QoS.

| Raw Square 1 validation segment | 60 seconds at 0.5× | 90 seconds at 1× |
|---|---:|---:|
| Observed raw clouds, Alpha/Bob/Carol | 589 / 575 / 575 | 889 / 873 / 873 |
| Valid ICP odometry messages | 584 / 557 / 574 | 883 / 855 / 872 |
| Native keyframes | 116 / 67 / 104 | 191 / 113 / 175 |
| Published descriptors after 45 s settling | 116 / 67 / 104 | 168 / 113 / 145 |
| Descriptors / keyframes | 287 / 287 | **426 / 479** |
| Accepted inter/intra registrations | 0 / 0 | 0 / 0 |

The 1× test ran in a separate ROS domain/container alongside the full queue.
ICP mostly kept up, while upstream Python Scan Context workers accumulated a
backlog. Its 53 unobserved descriptors after settling prevent claiming that this
configuration keeps up end-to-end at 1×. The queued experiments therefore remain
at 0.5×. This short, concurrent test does not establish an intrinsic performance
limit on every machine or sequence.

Neither short segment produced a connected three-robot optimized result. These
are startup/timing checks, not full-sequence benchmark results.

[Half-speed validation report](figures/swarm-official148/smoke-half-speed/REPORT.md),
[real-time validation report](figures/swarm-official148/smoke-real-time/REPORT.md),
[upstream tests](figures/swarm-official148/provenance/upstream-tests.xml),
[evo tests](figures/swarm-official148/provenance/evo-evaluation-tests.xml).

## Results and monitoring

The remote report updates after each sequence:

```text
/data3/mikexyl/swarm_official_s3e/REPORT.md
/data3/mikexyl/swarm_official_s3e/queue-status.json
/data3/mikexyl/swarm_official_s3e/results/<sequence>/REPORT.md
```

```bash
ssh 148 cat /data3/mikexyl/swarm_official_s3e/queue-status.json
ssh 148 cat /data3/mikexyl/swarm_official_s3e/results/S3E_Square_1/progress.json
```

Each report uses evo 1.36.5 and saves combined/per-robot ATE, raw ICP ATE,
trajectory PNG/PDF, native loop messages, coverage counts, logs and provenance.
Optimized results use one shared rigid fit per native output origin, without
scale or additional robot fits. Raw odometry uses independent fits. Missing
optimized poses are not filled using odometry, and partial runs remain marked
incomplete. GT orientations are unused; sequences without timestamped GT receive
no invented ATE. No duplicate bags or dense point-cloud exports are generated.
