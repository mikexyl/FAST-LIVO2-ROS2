# Collaborative LiDAR Mapping with Multi-Layer BEVs and Belief Sharing on Robot-Decoupled Submap Graphs

The evaluated method uses fixed 40 m circles placed incrementally to cover at least 99% of occupied 1 m horizontal cells. Circle locations remain fixed while their geometry matures. Immutable point-cloud submaps feed terrain-relative multilayer MapClosures retrieval, point-GICP verification, PCM, and persistent distributed CBS with GPU VGICP factors. Optimized submap corrections reconstruct both dense trajectories and the joint map.

## GRACO: simultaneous 14-robot replay

| Measurement | Result |
|---|---:|
| Assigned / connected robots | 14 / 14 |
| Circle poses | 366 |
| Verified / PCM-retained loops | 111 / 111 |
| Admitted GPU geometry pairs with final support | 110 |
| Final joint position ATE | 0.933 m |
| Associated poses | 45,397 |
| First all-14 shared frame | 430.46 s |
| Last bag end | 520.05 s |
| Final publication | 909.19 s |
| Application traffic | 546.77 MB |
| Native points in joint map | 13,353,004 |

ATE uses evo 1.36.5, 50 ms association, one shared SE(3) alignment and no scale fitting, at the calibrated IMU reference point. The final accuracy includes post-playback settling. Independently aligned per-robot errors are separate columns in the report.

The geometry-publication ablation yields a largest component of four robots under immediate freezing, compared with 14 under maturation. It uses independent fresh odometry replays, not exactly frozen input. The disconnected ablation has no all-team joint ATE.

## Communication and computation

Retrieval queries contribute 75.1% of the 546.77 MB total; CBS beliefs contribute 0.62%. The mean is 39.05 MB per assigned robot. Separately measured connected GRACO ground-trio baselines use 32.04 MB/robot for Swarm-SLAM, 131.17 for DCL-SLAM, and 267.53 for DiSCo-SLAM. These different workloads provide context, not a matched ranking. All figures are serialized application payload, excluding transport headers, discovery and retransmissions.

Across 842 nonterminal updates, median/P95/maximum solver times are 0.351/1.003/4.499 ms; the full timer callback takes 1.629/32.230/1,188.685 ms. The maximum local graph has 61 variables. The 14 terminal callbacks and final geometry validation are outside native timing coverage. Short local updates do not imply a fully settled result every 10 seconds.

Total run time is 913.15 s. Peak simultaneous process-tree RSS is 42.34 GiB; whole-device GPU allocation peaks at 4,135 MiB. These are measurements of the simultaneous workstation workload, not per-robot or embedded-hardware requirements.

See the [technical report](output/pdf/technical-report.pdf) for the method, tables, timing figures and limitations, and [verification record](REPORT-VERIFICATION.json) for evidence checks.
