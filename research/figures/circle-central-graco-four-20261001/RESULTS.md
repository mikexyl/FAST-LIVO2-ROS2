# Four-robot centralized circle-submap experiment

Completed on workstation 148, 1 October 2026. GRACO A05, A07, G05 and G06 form one connected component in all three final graphs. Native final registration support and observability checks pass for every admitted geometry factor.

The incremental fixed-circle layout uses 40 m radius, 99% occupied-cell coverage, a fixed 1 m horizontal grid and a 30% greatest-overlap target. Existing centers stay fixed. The comparison keeps odometry, retained observations, point-cloud BEVs, multilayer matching, vertical initialization, verification thresholds, evidence budgets and centralized graph settings identical. Geometry is sealed once at capture end. This is a staged experiment over the union of retained snapshots, not an online scan replay.

## Main results

| Mode | Nodes | Verified / retained loops | GPU factors | Four-robot joint ATE [m] | Application traffic [MiB] | Measured backend stages [s] |
|---|---:|---:|---:|---:|---:|---:|
| Fixed circles, 40 m | 121 | 13 / 13 | 13 | 0.960 | 86.07 | 64.96 |
| Trajectory-centered, 40 m | 157 | 57 / 57 | 57 | 0.902 | 197.78 | 154.57 |
| Trajectory-centered, 80 m | 157 | 126 / 126 | 125 | 0.823 | 720.50 | 625.19 |

Circles reduce node count by 22.9% and measured application traffic by 56.5% against the matched-radius trajectory control. Their measured backend-stage total is 58.0% smaller. Joint ATE increases by 0.058 m. Against the 80 m reference, traffic is 88.1% smaller and joint ATE increases by 0.137 m. These are single-control observations; no accuracy improvement or repeated-run significance is claimed. Circle count is not a globally optimal count over unknown future growth.

The 80 m run rejects 15 verification attempts for low overlap. One retained pose loop also fails initial geometry-factor admission for low overlap; its companion pose constraint remains. This explains 125 geometry factors for 126 retained loops. Every admitted factor passes its final geometry check.

## Coverage and overlap

| Robot | Circle nodes | Circle cell coverage | 40 m trajectory cell coverage | 80 m trajectory cell coverage | Circle memberships / covered cell | Circle median greatest overlap |
|---|---:|---:|---:|---:|---:|---:|
| A05 | 30 | 99.81% | 61.51% | 99.76% | 1.84 | 30.0% |
| A07 | 21 | 99.61% | 61.76% | 99.93% | 1.88 | 30.0% |
| G05 | 43 | 99.08% | 68.68% | 99.96% | 1.76 | 30.0% |
| G06 | 27 | 99.23% | 70.00% | 99.94% | 1.76 | 30.0% |

All circles fall in the desired 20–40% greatest-overlap band for A05, A07 and G05. For G06, 26 of 27 do; the remaining deviation stays visible. Trajectory-centered 40 m disks have median greatest overlaps of 73.6–89.6% and 4.34–7.65 memberships per covered cell. The 80 m disks have 86.8–94.8% median greatest overlap and 8.86–16.73 memberships per covered cell. Circle placement therefore covers off-trajectory geometry while reducing repeated membership. The trajectory controls preserve their original footprint cadence and are not silently augmented to reach the circle coverage target.

Observed-point coverage is reported separately in the inspection tables; circles retain 99.76–99.90% of native representatives. Cells have equal weight regardless of observation density. All representatives, including those outside the footprints, remain in inspection maps with an explicit uncovered flag.

## Initialization correction and measurement scope

The first circle graph connected only A05/A07 despite verified ground bridges. The inherited centralized initializer required two distinct endpoints to support a robot-pair alignment. A07/G05 had a verified singleton bridge; G05/G06's two loops shared a G05 endpoint and likewise counted as one distinct support. Setting this experiment's `alignment_min_support` to 1 honors the user's existing singleton-loop policy. No descriptor, verification or registration threshold changed. Normal pipeline defaults remain unchanged.

Only graph solving and final-pose delivery were repeated under the corrected policy for all three controls. Sealed-map retrieval never consumes optimizer output, so descriptors, candidates, verified loop poses and geometry transfers remain valid immutable captures. The earlier partial-component ATE of 0.542 m is superseded by the final four-robot joint ATE of 0.960 m; their alignment scopes differ.

The main time column sums separately measured publication, retrieval, verification/geometry transfer and corrected graph stages. It excludes placement and BEV preparation and is **not a fresh end-to-end latency measurement**. Native GPU preparation and optimization are nested inside registration time and must not be added again.

| Mode | Publication [s] | Retrieval [s] | Verification / transfer [s] | Pose graph [s] | Registration stage [s] | Native preparation [s] | Native optimization [s] |
|---|---:|---:|---:|---:|---:|---:|---:|
| Fixed circles, 40 m | 2.37 | 48.86 | 11.87 | 0.064 | 1.80 | 0.451 | 0.015 |
| Trajectory-centered, 40 m | 3.73 | 99.78 | 45.93 | 1.514 | 3.62 | 0.773 | 0.052 |
| Trajectory-centered, 80 m | 5.31 | 180.55 | 412.44 | 15.134 | 11.76 | 1.220 | 0.121 |

Sequential preparation of the four robots takes 36.22, 81.22 and 141.97 s respectively, including placement, union/crop construction, payload writing and BEVs. Terrain estimation plus layered rendering/ORB takes 5.79, 11.21 and 30.71 s across all endpoints. Full-height rendering/ORB inspection controls take 0.085, 0.180 and 0.561 s. These combined scopes do not measure pure rendering alone.

| Mode | Fresh graph-stage wall [s] | Graph process-tree CPU [s] | Graph simultaneous RSS [MiB] | Graph whole-device GPU peak [MiB] |
|---|---:|---:|---:|---:|
| Fixed circles, 40 m | 3.25 | 13.90 | 1174 | 447 |
| Trajectory-centered, 40 m | 7.20 | 18.10 | 1680 | 645 |
| Trajectory-centered, 80 m | 29.59 | 42.17 | 2349 | 915 |

These resources cover only the fresh corrected graph and actual solution delivery. Original detection-capture resources remain separate; their peaks are not summed. RSS is a simultaneous process-tree sum sampled every 20 ms and may count shared pages twice. GPU is whole-device sampling every 100 ms, including driver overhead and unrelated activity. No artificial quotas were used. These are workstation measurements, not Jetson measurements.

## Communication

| Mode | Cached endpoints | Manifests [MiB] | Geometry [MiB] | Session / solution [MiB] | Total [MiB] |
|---|---:|---:|---:|---:|---:|
| Fixed circles, 40 m | 23 | 35.51 | 50.42 | 0.143 | 86.07 |
| Trajectory-centered, 40 m | 51 | 61.80 | 135.79 | 0.184 | 197.78 |
| Trajectory-centered, 80 m | 86 | 100.03 | 620.28 | 0.184 | 720.50 |

The ledger counts actual serialized robot/server IPC requests and replies plus framing. It combines immutable pre-graph traffic with actual corrected graph-session/solution traffic, excluding superseded solution/shutdown messages. Descriptors are uploaded once. Requested endpoint geometry is cached and reused for verification and registration. Network transport headers, odometry traffic and inspection downloads are excluded. This unoptimized centralized application protocol does not measure distributed CBS belief exchange.

## Evaluation, validation and artifacts

Position ATE uses evo 1.36.5, 50 ms association, one rigid SE(3) alignment for the complete four-robot component, and no scale fitting. GRACO's IMU-reference ground truth matches the native IMU trajectory reference. Ground truth is confined to evaluation. Each native map representative uses the optimized node for its nearest spatial footprint and is emitted once; uncovered representatives retain the nearest correction and an explicit flag. Simultaneous circle births average correction poses for dense trajectory interpolation, with disagreement diagnostics retained.

The final local suite passes **60 tests**, covering placement, replay, footprint frames/membership, actual IPC accounting, duplicate anchors, spatial map reconstruction and singleton initialization. On 148, **45 native/integration tests** passed, including CPU compatibility, GPU known-transform recovery, reversed endpoints and common-frame invariance. These suites overlap; their counts are not added. Final native results report `success: true` for all controls with no rejected final linearizations.

The [local inspection gallery](index.html) contains aligned joint maps/trajectories, footprints, full-height point-cloud BEVs, detailed tables, configuration/source/input hashes, communication ledgers, graph results and tests. The [numeric comparison](comparison.json) retains full precision and scopes. The [protocol](PROTOCOL.md) and `configs/circle_central_graco_four.yaml` describe reproduction.

Bulk geometry, full trajectories and captures remain on 148:

```text
/data3/mikexyl/swarm_s3e_ws/src/.ros2/circle-central148-20260930/
  matched-final-2/          immutable three-control detection/verification captures
  singleton-policy-graph/  corrected final graphs, evaluation and inspection
  native-install/bin/      GPU executable and hashed build manifest
  source-final/            source used for full captures
  source-graph-replay/     corrected graph-policy/evaluation source
  tests-final.log          45 native/integration checks
```

Historical attempts remain available. LiDAR odometry and existing distributed defaults are unchanged. Online geometry revision/freezing and persistent distributed submap optimization remain subsequent integration work.

## Aligned 3D trajectories and continuity inspection

The [rotatable 3D view](aligned-trajectories-3d.html) uses the same single component alignment as the joint ATE calculation and retains every saved dense sample. True metric scale is the default; optional height exaggeration affects display only. A [PNG](aligned-trajectories-3d.png) and [vector PDF](aligned-trajectories-3d.pdf) are retained.

The [continuity audit](trajectory-audit/index.html) isolates motion introduced by changing graph corrections. For circle40 its maximum is 0.404 mm per sample across all robots, and 0.132 mm for A07. The prominent A07 eastern and southern bends appear in raw odometry and GT as well. No trajectory smoothing or reconstruction change was applied; the reported ATE values remain unchanged.
