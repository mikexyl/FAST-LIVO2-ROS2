# Full native-submap pipeline: five previously failed groups

Native EllipseLIO uses 10-second submaps with 5-second overlap. Completed native submaps supply both ellipsoid-BEV descriptors and processed member-scan registration evidence to MapClosures → distributed PCM → CBS with GICP factors.

Three independent 1× frontend replays run concurrently, with four executor threads each, separate ROS domains, and the existing shared 12-core / 40 GiB container limits. Five verified prior captures are reused. Group backends run serially after frontend completion.

ATE uses evo 1.36.5, 50 ms association, rigid alignment, and no scale fitting. Raw trajectories have independent alignments. CBS uses one alignment per measured connected component; per-robot CBS values below use that shared alignment. Ground truth is confined to evaluation. A missing value is not a zero error.

| Group | Pipeline status | Connected robots | Retained loops | PCM rejected | Backend wall [s] |
|---|---|---|---:|---:|---:|
| S3E_Laboratory_4 | Complete | Alpha+Bob+Carol | 84 | 0 | 34.72 |
| S3E_Campus_Road_1 | Complete | Alpha+Bob; Carol | 12 | 0 | 29.27 |
| S3E_Campus_Road_2 | Failed at evaluation | Unavailable | — | — | — |
| S3E_Campus_Road_3 | Complete | Alpha+Bob+Carol | 103 | 0 | 107.14 |
| GRACO_ground_01_06 | Complete | robot1+robot2+robot3+robot5+robot6; robot4 | 23 | 1 | 28.49 |

## S3E_Laboratory_4

| Robot | Raw ATE [m] | CBS ATE in shared component [m] | Maximum speed [m/s] | Maximum successful-update gap [s] | Peak mapper RSS [MiB] |
|---|---:|---:|---:|---:|---:|---:|
| Alpha | Unavailable | Unavailable | 1.013 | 0.110 | 353.5 |
| Bob | Unavailable | Unavailable | 1.132 | 0.117 | 356.9 |
| Carol | Unavailable | Unavailable | 1.355 | 0.201 | 361.4 |

Shared-component ATE: .

![Trajectories and native member-scan maps](S3E_Laboratory_4/report/trajectories-maps.png)

Full metrics, evo archives, trajectories, map arrays and verified Rerun recording: `S3E_Laboratory_4/report/`.

No evaluable raw position ATE for: Alpha, Bob, Carol. See ground-truth status in the JSON report; no reference repair or timestamp shift was applied.

## S3E_Campus_Road_1

| Robot | Raw ATE [m] | CBS ATE in shared component [m] | Maximum speed [m/s] | Maximum successful-update gap [s] | Peak mapper RSS [MiB] |
|---|---:|---:|---:|---:|---:|---:|
| Alpha | 1.5096 | 1.2198 | 2.904 | 0.102 | 556.2 |
| Bob | 4.2158 | 2.2835 | 2.427 | 0.201 | 590.2 |
| Carol | 1.5904 | 1.5904 | 3.230 | 0.201 | 601.7 |

Shared-component ATE: Alpha: 1.9743 m; Carol: 1.5904 m.

![Trajectories and native member-scan maps](S3E_Campus_Road_1/report/trajectories-maps.png)

Full metrics, evo archives, trajectories, map arrays and verified Rerun recording: `S3E_Campus_Road_1/report/`.

## S3E_Campus_Road_2

Failure: `RuntimeError('Exit 1: /workspace/.ros2/recent-submaps/full-five-groups-parallel-20260919/S3E_Campus_Road_2/evaluation.log')`. Retained evidence: `/workspace/.ros2/recent-submaps/full-five-groups-parallel-20260919/S3E_Campus_Road_2`.

## S3E_Campus_Road_3

| Robot | Raw ATE [m] | CBS ATE in shared component [m] | Maximum speed [m/s] | Maximum successful-update gap [s] | Peak mapper RSS [MiB] |
|---|---:|---:|---:|---:|---:|---:|
| Alpha | 2.8090 | 3.2512 | 2.177 | 0.205 | 528.1 |
| Bob | 3.1979 | 4.2626 | 2.711 | 0.204 | 544.6 |
| Carol | 2.5207 | 3.2425 | 2.845 | 0.209 | 623.8 |

Shared-component ATE: Alpha: 3.6144 m.

![Trajectories and native member-scan maps](S3E_Campus_Road_3/report/trajectories-maps.png)

Full metrics, evo archives, trajectories, map arrays and verified Rerun recording: `S3E_Campus_Road_3/report/`.

## GRACO_ground_01_06

| Robot | Raw ATE [m] | CBS ATE in shared component [m] | Maximum speed [m/s] | Maximum successful-update gap [s] | Peak mapper RSS [MiB] |
|---|---:|---:|---:|---:|---:|---:|
| robot1 | 3.7293 | 4.2606 | 2.243 | 0.328 | 575.6 |
| robot2 | 2.2601 | 4.1353 | 3.007 | 0.312 | 573.1 |
| robot3 | 10.8679 | 12.0275 | 3.566 | 0.320 | 536.1 |
| robot4 | 0.9279 | 0.9279 | 2.190 | 0.216 | 537.6 |
| robot5 | 1.2587 | 2.8836 | 2.483 | 0.192 | 547.9 |
| robot6 | 1.1625 | 5.6643 | 2.182 | 0.192 | 536.7 |

Shared-component ATE: robot1: 6.1543 m; robot4: 0.9279 m.

![Trajectories and native member-scan maps](GRACO_ground_01_06/report/trajectories-maps.png)

Full metrics, evo archives, trajectories, map arrays and verified Rerun recording: `GRACO_ground_01_06/report/`.

## Verification and retained evidence

The retention audit verifies live and derived Rerun recordings, descriptor/evidence membership, causal retrieval availability, same-robot loop exclusion, graph anchor timestamps, and immutable artifact hashes. Per-scan diagnostics retain handover steps, correspondence ages, successful-update status, processing time and memory measurements. Geometry is native processed member-scan data, not full-resolution raw LiDAR.

All frozen source/binary hashes were rechecked after completion. The parallel scheduler passed its five tests; the generic report reproduced Library 2 ATE results, and the additional artifact audit passed against its 280 completed submaps and 840 retrieval events.

This is one current-configuration full-pipeline trial per group, with no parameter sweep. Successful completion does not establish general robustness. The earlier serial partial capture is retained separately with its parallel-relaunch annotation. Previous experiments and the paused CU-Multi download are preserved.

Machine-readable outputs: `batch-summary.json`, `retention-audit.json`, `plan.json`, `source-hashes.json`, and each group’s `report/report.json` or `failure.json`. All bulk geometry is retained.
