# S3E Square 2: MegaLoc + MapClosures + CBS

Completed 2026-09-14 on the requested Square 2 sequence. Components: **Alpha: Alpha, Bob: Alpha, Carol: Alpha**.
All three robots connected through **37 accepted inter-robot loops**, with **0.5340 m combined CBS ATE RMSE** under one shared rigid alignment. The earlier statement that Square 2 was unavailable was incorrect; the official release contains the complete bag and ground truth.

The official 10.09 GB download was verified against repository revision `dd2d0dccb95985468800ed858a5418005ce75cd3`. The bag spans 255.197 seconds. GT has 5 / 2 / 0 gaps over two seconds for Alpha / Bob / Carol; missing intervals are excluded from ATE. All three IMU streams have monotonic timestamps, finite measurements and no gaps over 0.2 seconds. Their largest intervals are 15.82, 15.82, and 15.94 ms. SQLite integrity and all metadata message counts pass. [Input and download audit](figures/cbs-square2/input.json).

[Official sequence files](https://huggingface.co/datasets/PengYu-Team/S3E/tree/dd2d0dccb95985468800ed858a5418005ce75cd3/S3Ev1/S3E_Square_2).

Each robot starts at bag offset zero. `vio.img_point_cov=100` is explicitly applied to all three robots, following the Playground 1 startup diagnosis. All other mapping parameters and the working MegaLoc, MapClosures, registration and CBS settings are unchanged. There is one configuration, with no sweep. Ground truth is used only for evaluation.

FAST-LIVO2 runs serially at 1× replay. The three loop front ends and three CBS optimizers then run concurrently through Fast DDS on one host using frozen odometry/descriptors. This remains offline distributed replay, not end-to-end online operation.

## CBS trajectory accuracy

Evo 1.36.5 handles timestamp association, rigid alignment and translation APE. Association uses a 0.05 s tolerance, no interpolation and no time offset. Each connected component has one shared SE(3) alignment, no scale fitting and no additional per-robot fit. Placeholder GT orientations are unused. The antenna lever arm is uncorrected.

| Robot | Component | ATE RMSE (m) | Median ATE (m) | Matched GT positions |
|---|---|---:|---:|---:|
| Alpha | Alpha | 0.6261 | 0.3749 | 195 |
| Bob | Alpha | 0.6335 | 0.4292 | 245 |
| Carol | Alpha | 0.3020 | 0.2883 | 254 |
| **Component Alpha combined** | Alpha | **0.5340** | **0.3443** | **694** |

![CBS ATE](figures/cbs-square2/position_error.png)

Native evo archives and matched trajectories are preserved under [evo](figures/cbs-square2/evo/cbs/README.md).

## Raw odometry and coverage

The raw odometry ATEs below use independent per-robot SE(3) fits for diagnosis. They cannot be subtracted directly from the shared-component CBS ATEs.

| Robot | Received / expected clouds | Exported frames | Export span (s) | Raw odometry ATE RMSE (m) |
|---|---:|---:|---:|---:|
| Alpha | 2388 / 2388 | 2372 | 237.103 | 0.5441 |
| Bob | 2483 / 2483 | 2477 | 247.602 | 0.5772 |
| Carol | 2540 / 2540 | 2534 | 253.300 | 0.2179 |

![Raw odometry](figures/cbs-square2/raw_odometry_diagnostic.png)

[Raw trajectories, evo statistics and provenance](figures/cbs-square2/raw_odometry.json).

## Loops and communication

| Measurement | Result |
|---|---:|
| Keyframes: Alpha / Bob / Carol | 317 / 274 / 270 |
| Accepted loops | 37 |
| Inter-robot / intra-robot | 37 / 0 |
| MapClosures only / both branches / MegaLoc only | 25 / 8 / 4 |
| Alpha–Bob / Alpha–Carol / Bob–Carol | 22 / 5 / 10 |
| Connected components | 1 |
| Verification attempts | 154 |
| Verification yield | 24.03% |
| Recall@1 / @5 / @20 | 75.45% / 82.04% / 87.43% |
| Proximity precision / recall | 7.90% / 47.26% |
| Accepted loops within 10 m / assessable | 23 / 33 |
| Loop exchange CDR bytes | 109519131 |
| CBS belief CDR bytes | 253672 |
| Median / p95 wall detection latency | 0.235 / 0.428 s |
| Detection + CBS wall time | 45.76 s |

Branch counts are mutually exclusive attributions within this single fused run. A constraint proposed by both retrieval branches is counted once. Recall and precision use position-proximity labels; they do not establish exact 6-DoF loop correctness. In particular, valid overlapping submaps can have origins farther than the 10 m proximity threshold. CDR accounting excludes RTPS, discovery and retransmission overhead.

Four accepted loops lack usable position GT for both endpoints and are excluded from the proximity assessment.

## Ground truth, corrected trajectories and maps

![Ground truth](figures/cbs-square2/ground_truth_trajectories.png)

![Trajectories and loops](figures/cbs-square2/trajectories_and_loops.png)

![Map top view](figures/cbs-square2/map_top_down.png)

![Map oblique](figures/cbs-square2/map_oblique.png)

Map colors show robot identity or elevation, not camera RGB. The map figures use a 0.30 m display grid and an oblique sample of at most 650,000 points. PNG and PDF versions are retained.

## Runtime and validation

| Stage | Alpha / Bob / Carol (s) |
|---|---:|
| Odometry | 249.5 / 263.1 / 265.3 |
| Keyframes / submaps | 185.0 / 160.1 / 171.6 |
| MegaLoc CUDA descriptors | 22.8 / 13.2 / 13.8 |
| MapClosures descriptors | 7.0 / 6.5 / 9.3 |

Detection/CBS took 45.76 s; evaluation, map reconstruction and Rerun export took 50.51 s. MegaLoc used one CUDA inference worker; mapping, MapClosures, GICP and CBS used CPU. Memory and convergence diagnostics are in the [complete numeric report](figures/cbs-square2/report.json).

Alpha and Bob's submap preparation overlapped subsequent robots' odometry; stage times cannot be summed into elapsed runtime. An initial sandbox attempt was stopped because ROS2 could not create its required UDP sockets. The completed run used the approved external execution environment, and the interrupted output was excluded from completed caches and the results above.

The audit found zero future-frame or same-robot-exclusion violations, unique loop endpoint pairs and exact serialized-byte accounting. It reproduced evo statistics from saved poses. The Python suite passed 39 tests with four optional ROS tests skipped. [Audit](figures/cbs-square2/audit.json) · [Figure and stage provenance](figures/cbs-square2/figures.json). Accepted constraints, verification diagnostics, effective mapping configurations and stage manifests are also retained beside the figures.

## Retention and reproduction

The Rerun 0.37.1 recording is retained locally at `.ros2/recordings/square2-cbs.rrd` (250.0 MB). The recording passed `rerun rrd verify`, including footer checks. Generated intermediate outputs totaling 7.98 GiB were removed after validation. Reports, figures, small trajectory/evo evidence and the original dataset are retained.

```bash
FAST-LIVO2-ROS2/scripts/s3e_experiment.sh run --stage all \
  --config FAST-LIVO2-ROS2/research/configs/square2-cbs.yaml --resume
```

CBS branch: `dev/fast-livo2-s3e-dpgo` at `11d84afe3397870cddecb5b6c07f4ed6866466a4`. CBS ROS branch: `dev/fast-livo2-s3e-dpgo` at `86e3e3511b9c4bb2750628c823b682b5200c34a3`.
