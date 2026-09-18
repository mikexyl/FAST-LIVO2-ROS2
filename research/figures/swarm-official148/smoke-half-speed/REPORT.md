# S3E_Square_1: official Swarm-SLAM LiDAR

Native run: **incomplete_optimized_coverage**. Full sequence: **False**. Replay: 0.5×; wall time: 172.7 s.

Three simultaneous RTAB-Map ICP odometry instances feed the unchanged upstream Swarm-SLAM Scan Context, FPFH/TEASER++/ICP and native GNC optimizer. Raw LiDAR/IMU bags are replayed online. No EllipseLIO, FAST-LIVO2, MapClosures, CBS, external pose factors or registration-direction fix is used.

| Trajectory | Shared ATE [m] | Alpha | Bob | Carol |
|---|---:|---:|---:|---:|
| Raw RTAB-Map ICP | — | 0.2127 | 0.3608 | 0.3072 |
| Swarm origin 0 | 0.1413 | 0.1413 | unavailable | unavailable |

Complete three-robot shared ATE: **unavailable m**.

Raw: independent robot SE(3) fits. Optimized: native keyframes, one shared SE(3) fit per output origin, no scale. No interpolation or external odometry. GT orientations unused; antenna lever arm uncorrected. Origin labels do not expose retained GNC factors.

Accepted registrations before GNC: 0 inter-robot, 0 intra-robot. Native GNC weights are not exposed. Partial native outputs may be measured but remain marked incomplete.

Observed scans: `{'Alpha': 589, 'Bob': 575, 'Carol': 575}`. Valid odometry: `{'Alpha': 584, 'Bob': 557, 'Carol': 574}`. Keyframe coverage: `{'Alpha': {'keyframes': 116, 'optimized': 116, 'complete': True}, 'Bob': {'keyframes': 67, 'optimized': 0, 'complete': False}, 'Carol': {'keyframes': 104, 'optimized': 0, 'complete': False}}`.

Online input is not backpressured by our recorder. Scan loss and odometry failures remain visible in the saved logs and counts.

![Trajectories](trajectories.png)

[Machine-readable results](report.json), [evo artifacts](evo/).

Reproduce a shared fit: `evo_ape kitti reference.kitti estimate.kitti -a -r trans_part`. Timestamp association is performed per robot with evo and a 50 ms tolerance before pooling.
