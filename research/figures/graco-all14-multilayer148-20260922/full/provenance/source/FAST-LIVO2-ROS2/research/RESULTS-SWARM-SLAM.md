# Swarm-SLAM LiDAR comparison

Three-robot S3E runs use fresh shared EllipseLIO exports. This compares the native Swarm LiDAR loop/optimizer path with our raw/ellipsoid MapClosures paths.
Strict replay failures and preprocessing failures are retained. Unavailable ATEs are not replaced by historical results.

| Sequence | Strict Swarm replay | Exact selected inputs | Complete optimized coverage | Swarm ATE (m) | Raw BEV ATE (m) | Ellipsoid BEV ATE (m) |
|---|---|---|---|---:|---:|---:|
| [square1](figures/swarm-comparison-square1/REPORT.md) | failed | False | False | unavailable | 1.1446 | 1.2682 |
| [square2](figures/swarm-comparison-square2/REPORT.md) | failed | False | True | 0.7359 | unavailable | unavailable |
| [laboratory1](figures/swarm-comparison-laboratory1/REPORT.md) | failed | False | True | unavailable | unavailable | unavailable |

ATE is evaluated with evo 1.36.5 using one shared rigid alignment per component and no scale fitting.
Laboratory 1 has no timestamped trajectory GT. Accepted native loops are recorded before GNC; native messages do not export GNC weights.
A strict all-scan failure remains a failure even if the separate audit proves every selected keyframe and descriptor was delivered. Such a case can expose a usable keyframe result while failing transport validation.
Actual recorded trajectory ATE can be measured when native optimized coverage and timestamped GT are available, even if the replay audit fails. Input validity remains a separate requirement for a controlled comparison.

Swarm uses native Scan Context and FPFH/TEASER++/ICP, with the documented transform-direction and GTSAM compatibility fixes.
Native keyframes, map history and noise models differ from our paths. The reports disclose runtime and communication accounting differences.
Setup: [Swarm-SLAM/s3e/README.md](../../Swarm-SLAM/s3e/README.md).

## Workstation 148

These tests use the user-approved multistage setup: serial EllipseLIO exports, three simultaneous native Swarm-SLAM robot instances, then evo evaluation. They are not an untouched official online launch. No fresh paired MapClosures result is claimed.

The isolated ROS Humble build and final adapter checks passed **32 tests**. The duplicate-input smoke test processed **1,341/1,341 scans** and **285/285 keyframes/descriptors**, with no optimizer errors and maximum corresponding-position disagreement **0.0612 m**. [Smoke report](figures/swarm148-smoke/REPORT.md); [remote setup](../../Swarm-SLAM/s3e/remote/README.md).

| Sequence | Native run | Exact input audit | Shared ATE [m] | Alpha | Bob | Carol |
|---|---|---|---:|---:|---:|---:|
| [Library 1](figures/swarm148-library1/REPORT.md) | complete | True | 1.5827 | 1.1024 | 1.9344 | 1.6179 |
| [Campus Road 1](figures/swarm148-campusroad1/REPORT.md) | not run: frontend failed | unavailable | unavailable | unavailable | unavailable | unavailable |

Swarm ATE uses evo 1.36.5, 50 ms timestamp association, one shared rigid alignment per output component, no scale fitting, and no extra robot alignment. Supplied GT orientations are unused. Missing native robot trajectories are not replaced.

| Sequence | Raw Alpha ATE [m] | Raw Bob | Raw Carol | Registrations accepted / inter | GT-checkable / flagged | Native time [min] |
|---|---:|---:|---:|---:|---:|---:|
| Library 1 | 1.0277 | 1.7256 | 1.3094 | 120 / 83 | 56 / 7 | 14.03 |
| Campus Road 1 | 56.0982 | unavailable | unavailable | not run | not run | not run |

Raw odometry uses independent robot fits, so its alignment scope differs from the connected Swarm result. Flagged registrations have a GT endpoint-distance discrepancy above 2 m; this is not a full 6-DoF outlier label. Accepted counts precede native GNC, whose weights are not exported. Native time includes paced replay, backpressure and settling, but excludes frontend generation.

Reports retain PNG/PDF plots, available native outputs, source provenance, evo evidence and compact verified Rerun recordings where generated. Generated clouds from completed or stopped attempts are retired after verifying the retained evidence; original S3E datasets remain intact. Remote workspace: `/data3/mikexyl/swarm_s3e_ws/src`. Live status: `ssh 148 python3 /data3/mikexyl/swarm_s3e_ws/src/Swarm-SLAM/s3e/remote/status.py`.

**Campus Road 1 failure:** Alpha EllipseLIO diverged before loop closure, with implausible motion exceeding 1000 m/s; Bob was stopped and Carol was not started. This occurred before Swarm-SLAM.
The recorded IMU stream had no gap over 50 ms around the onset; the frontend failure mechanism remains undiagnosed.
