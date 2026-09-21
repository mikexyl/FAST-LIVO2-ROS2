# GRACO aerial 05–08: four robots on workstation 148

Four fresh, concurrent, full-flight 1× replays using temporal-only EllipseLIO (10-second windows, 5-second overlap), native ellipsoid-BEV MapClosures, distributed PCM, and CBS with GICP registration factors. The original ROS1 bags were converted to sensor-only ROS2 input; every LiDAR/IMU measurement, header timestamp, record timestamp, and LiDAR point payload was checked against its source. No timestamps were rebased.

**Diagnostic backend result.** The frontend stability gate failed for aerial06: 2.656 s, aerial08: 1.464 s (successful-LiDAR-update gap must be less than 1 second). All captures nevertheless completed with finite chronological poses, maximum speed below 20 m/s, complete sensor coverage, and verified native exports. The failed gate is preserved in `frontend-gate.json`; no thresholds were changed to continue.

**The four robots did not form one connected map.** Measured components: aerial05; aerial06, aerial07; aerial08. No all-four shared ATE is reported.

| Flight | Raw ATE RMSE [m] | CBS ATE RMSE [m] | CBS component | Poses | Max speed [m/s] | Max update gap [s] | Frontend gate |
|---|---:|---:|---|---:|---:|---:|---|
| aerial05 | 0.8601 | 0.8601 | aerial05 | 2747 | 4.829 | 0.192 | Passed |
| aerial06 | 3.5598 | 3.5955 | aerial06 | 3103 | 3.170 | 2.656 | Failed |
| aerial07 | 0.2034 | 0.4948 | aerial06 | 3717 | 3.089 | 0.424 | Passed |
| aerial08 | 0.1668 | 0.0735 | aerial08 | 2620 | 4.735 | 1.464 | Failed |

Position ATE is evaluated entirely through **evo 1.36.5**, with 50 ms association, rigid SE(3) alignment, and no scale fitting. Raw fits are independent per robot; CBS uses one shared fit per measured connected component. These different alignment scopes matter when comparing raw and CBS errors. Ground truth was exported from `/gnss/ground_truth` only after optimization and did not influence mapping, retrieval, or loop admission.

| Connected component | Shared CBS ATE RMSE [m] | Matched poses |
|---|---:|---:|
| aerial05 | 0.8601 | 2747 |
| aerial06, aerial07 | 2.4526 | 6820 |
| aerial08 | 0.0735 | 2620 |

Accepted loops: **6** (5 inter-robot, 1 intra-robot). PCM rejected **1** of 7 proposals. CBS retained 6 GICP factors. Detection and backend execution took **14.02 seconds**, excluding sensor replay, descriptor preparation, and evaluation.

Inter-robot loop counts: {"aerial06--aerial07": 5}.

![CBS trajectories and separate component maps](report/trajectories-maps.png)

## Native submaps and runtime

| Robot | Completed submaps | Partial inspection tails | Max correspondence age [s] | Mean scan processing [ms] | P95 [ms] | Peak mapper RSS [MiB] |
|---|---:|---:|---:|---:|---:|---:|
| aerial05 | 58 | 2 | 9.984 | 16.04 | 29.93 | 483.91 |
| aerial06 | 64 | 2 | 9.992 | 17.62 | 31.89 | 501.63 |
| aerial07 | 77 | 2 | 9.991 | 16.07 | 30.50 | 501.76 |
| aerial08 | 54 | 2 | 9.980 | 17.88 | 31.40 | 503.31 |

Descriptors and registration evidence use the same native processed member scans, expressed in each completed submap’s last included IMU frame. This is not full-resolution raw geometry. Shutdown tails are retained for inspection and excluded from retrieval. Completed submaps become available causally, while graph poses retain their anchor timestamps.

The dedicated container has no CPU quota, CPU affinity restriction, or memory limit. Four-thread native launchers run four frontends concurrently; algorithm thread settings preserve the earlier temporal configuration. The source overlay and build are isolated from prior experiments.

## Verification and retained artifacts

Both native CTests and all 10 selected Python/integration tests passed, including actual four-robot PCM/CBS with GICP. The six core temporal source files match the restored temporal implementation. The final audit verified 253 descriptor/evidence memberships and 1012 causal retrieval events, along with native payload hashes, anchor poses, graph timestamps, same-robot exclusions, and frozen source hashes. Four live Rerun recordings and the derived recording passed verification.

[Numeric report](report/report.json), [native audit](retention-audit.json), [frontend gate](frontend-gate.json), [configuration](config.yaml), [source hashes](source-hashes.json), [backend summary](dpgo/summary.json), [derived Rerun recording](report/result.rrd).

Full data and live recordings remain on 148 at `/data3/mikexyl/swarm_s3e_ws/src/.ros2/graco-aerial-four148-20260920`. All bulk geometry is retained. Prior experiments and the paused CU-Multi download are unchanged. This is one fixed-configuration experiment, not a parameter sweep or a repeated-trial performance estimate.
