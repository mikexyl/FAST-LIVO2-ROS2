# Square 1: Swarm-SLAM LiDAR comparison

All systems consume the same fresh EllipseLIO odometry and full deskewed LiDAR exports.
[Swarm-SLAM](https://github.com/MISTLab/Swarm-SLAM) runs its actual ROS2 LiDAR nodes for each robot.
Our raw and ellipsoid BEV paths use MapClosures and centralized GNC-TLS PGO.
This compares complete loop/optimization paths; keyframes, map history and factor noise differ.

Native run status: **incomplete**.

Reason: `TimeoutError('Native keyframes/descriptors did not drain within bounded wait')`. The strict all-scan replay check did not pass.

A complete three-robot native trajectory is unavailable. No missing robot estimates are replaced with odometry and no complete-run ATE is invented.

## Position ATE

RMSE in metres, evaluated entirely through evo 1.36.5 on the same dense frontend timestamps.
One shared rigid alignment per connected component, no scale fitting, no additional robot fit.

| Method | Combined | Alpha | Bob | Carol |
|---|---:|---:|---:|---:|
| Raw odometry (separate robot fits) | unavailable | 1.0970 | 0.7945 | 0.5921 |
| Our raw BEV + PGO | 1.1446 | 1.4765 | 0.9664 | 0.9070 |
| Our ellipsoid BEV + PGO | 1.2682 | 1.4425 | 1.0357 | 1.3359 |
| Swarm-SLAM LiDAR | unavailable | unavailable | unavailable | unavailable |

![Trajectories](trajectories.png)

![Individual ATE](ate.png)

## Loop and graph outcomes

| Method | Keyframes | Verification attempts | Accepted registrations | Accepted inter | Output frames |
|---|---:|---:|---:|---:|---:|
| Our raw BEV + PGO | 1548 | 123 | 31 | 31 | 1 |
| Our ellipsoid BEV + PGO | 1548 | 586 | 59 | 8 | 1 |
| Swarm-SLAM LiDAR | 2764 | 7 | 0 | 0 | unavailable |

Accepted registrations are counted before robust optimization. Swarm-SLAM does not export GNC weights in its native result messages;
its output frames reflect native origin IDs, and should not be interpreted as a verified graph of GNC-selected inter-robot factors.
Our GNC-selected loops: raw **31**, ellipsoid **59**.

Swarm GT-checkable registrations: **0/0**; endpoint-distance discrepancies above 2 m: **unavailable**.
This position-only test compares translation magnitude against GT endpoint separation. It is not a complete 6-DoF outlier test.

## Runtime and delivery


Our detection-only wall times: raw **75.5716 s**, ellipsoid **178.2231 s**.
**These wall-time definitions are different and do not support a direct speedup ratio.**

Swarm observed coordination payload: **360.58 MiB** of serialized CDR.
This counts each observed publication once on the listed topics, including local deliveries; DDS transport/discovery overhead and broadcast fanout are excluded.
Our communication figures use compressed simulated envelopes, so the byte totals have different wire semantics.

Scans published/acknowledged: **[4539, 4490, 4570] / [4538, 4490, 4569]**.
Keyframes expected/observed: **[943, 862, 959] / [943, 862, 959]**; descriptors: **[943, 862, 959]**.

## Setup and adaptations

The pinned checkout uses native Scan Context, FPFH/TEASER++ with ICP refinement, spectral candidate selection,
and elected-robot GTSAM optimization. The LiDAR pose-factor direction is corrected by inverting the
source-to-target registration transform; forward/reversed synthetic tests validate that convention.
GTSAM 4.3 compatibility changes replace removed shared-pointer, quaternion and value-filter APIs.
A LiDAR-only build avoids compiling the unrelated visual frontend. A processing acknowledgement
adds bounded replay backpressure without changing keyframe selection or registration.

Native LiDAR settings come from upstream graco_lidar.yaml: 0.5 m keyframe distance/voxel size,
similarity threshold 0.8, more than 60 registration inliers, 20-keyframe intra exclusion,
one selected inter-robot candidate per five-second detection period. Synchronization tolerance is 1 ms
for exact timestamp pairs; the operational backend wait timeout is 60 seconds. Logs are enabled.

Swarm uses a full scan per keyframe; our raw BEVs use trailing five-second submaps and ellipsoid BEVs use persistent spatial maps.
All loop selection and optimization exclude GT. GT orientations are unused; the antenna lever arm is uncorrected.

## Artifacts

[Rerun trajectories](trajectories.rrd), [machine-readable comparison](report.json),
[native run](swarm/summary.json), [native loop checks](swarm/loop-quality.jsonl), and [our paired report](ours/REPORT.md).
The project setup and adaptation details are in `Swarm-SLAM/s3e/README.md`.
