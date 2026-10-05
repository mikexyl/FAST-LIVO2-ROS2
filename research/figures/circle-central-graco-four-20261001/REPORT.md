# Centralized circle-submap comparison

GRACO A05, A07, G05 and G06. Fixed circles: 40 m, 99% occupied-cell target, 1 m grid. Placement is incremental over the growing union of retained snapshots; geometry is frozen once at each capture end. This is a staged backend test, not an online replay. Identical native odometry, input union, point-cloud BEVs, terrain layers, vertical initialization and verification thresholds are reused. The centralized solver uses GNC-TLS selected pose loops followed by joint LM with live GPU VGICP factors, instead of PCM/CBS.

| Mode | Nodes | Verified / retained loops | GPU factors | Component joint ATE [m] | Communication [MiB] | Measured backend stages [s] | Graph-stage RSS [MiB] |
|---|---:|---:|---:|---|---:|---:|---:|
| circle40 | 121 | 13 / 13 | 13 | aerial05+aerial07+ground05+ground06: 0.96 | 86.07 | 65.0 | 1174 |
| trajectory40 | 157 | 57 / 57 | 57 | aerial05+aerial07+ground05+ground06: 0.90 | 197.78 | 154.6 | 1680 |
| trajectory80 | 157 | 126 / 126 | 125 | aerial05+aerial07+ground05+ground06: 0.82 | 720.50 | 625.2 | 2349 |

The centralized initializer now admits a verified singleton bridge, matching the user’s singleton-loop policy. The original two-distinct-endpoint initializer rejected valid ground bridges in the circle run. Only graph/solution delivery was repeated after fixing this policy: BEVs, retrieval proposals, verification outcomes and transferred geometry are immutable captures. The time column is the sum of separately measured publication, retrieval, verification/transfer and corrected graph stages. It excludes placement and BEV preparation and is **not** a new end-to-end latency measurement. The memory column is actual simultaneous RSS during the corrected graph stage only. Full detection-capture resources remain in the retained original runs; their peaks are not summed with graph-stage peaks.

Communication combines actually measured pre-graph publication/retrieval/evidence streams with the actual corrected graph-session and final-pose-delivery messages. It excludes the obsolete initializer’s pose delivery/closing messages. Descriptors are uploaded once; selected endpoint geometry is cached and reused for registration. Network transport headers, odometry replay traffic and inspection-map downloads are excluded. RSS can double-count shared pages. GPU memory is whole-device sampling; native per-endpoint geometry counters omit buffers/driver overhead.

Each connected component has one evo 1.36.5 rigid alignment, 50 ms association, no scale. Disconnected components are never pooled into a fictitious joint ATE. GRACO RTK/INS IMU-reference positions are used only during evaluation. Dense trajectories average corrections from nodes born at the same timestamp before interpolation. Map points use the optimized pose of their nearest spatial footprint, so shared points are emitted once. Outside-radius points are kept using the nearest correction and explicitly marked uncovered.

The 40 m trajectory control retains original sensor-time snapshot cadence and freezes final geometry like the circle run. The 80 m reference restores the original radius under the same sealing/centralized protocol; it is not a replay of historical CBS traffic. Radius and placement effects must be distinguished.

## Coverage and overlap

| Mode | Robot | Nodes | Occupied-cell coverage | Observed-point coverage | Memberships / covered cell | Median greatest disk overlap | Circles in 20–40% overlap band |
| --- | --- | --- | --- | --- | --- | --- | --- |
| circle40 | aerial05 | 30 | 99.81% | 99.80% | 1.84 | 30.0% | 100.0% |
| circle40 | aerial07 | 21 | 99.61% | 99.90% | 1.88 | 30.0% | 100.0% |
| circle40 | ground05 | 43 | 99.08% | 99.76% | 1.76 | 30.0% | 100.0% |
| circle40 | ground06 | 27 | 99.23% | 99.86% | 1.76 | 30.0% | 96.3% |
| trajectory40 | aerial05 | 34 | 61.51% | 76.05% | 4.36 | 73.6% | 0.0% |
| trajectory40 | aerial07 | 40 | 61.76% | 75.40% | 7.65 | 89.6% | 0.0% |
| trajectory40 | ground05 | 52 | 68.68% | 83.85% | 4.53 | 77.9% | 0.0% |
| trajectory40 | ground06 | 31 | 70.00% | 83.66% | 4.34 | 77.8% | 0.0% |
| trajectory80 | aerial05 | 34 | 99.76% | 99.95% | 9.80 | 86.8% | 0.0% |
| trajectory80 | aerial07 | 40 | 99.93% | 100.00% | 16.73 | 94.8% | 0.0% |
| trajectory80 | ground05 | 52 | 99.96% | 100.00% | 9.77 | 88.9% | 0.0% |
| trajectory80 | ground06 | 31 | 99.94% | 100.00% | 8.86 | 88.9% | 0.0% |

Coverage uses equally weighted fixed-grid cells. Observed-point coverage weights native retained representatives. Disk overlap is intersection area divided by one circle’s area. Memberships count every containing disk; the final map still emits each observed representative once.

## Measured computation stages

| Mode | Preparation total [s] | Full BEV + ORB [s] | Terrain + layered BEV + ORB [s] | Publication [s] | Retrieval [s] | Verification / transfer [s] | Pose graph [s] | Registration stage [s] | Native preparation [s] | Native optimization [s] |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| circle40 | 36.22 | 0.085 | 5.79 | 2.37 | 48.86 | 11.87 | 0.064 | 1.80 | 0.451 | 0.015 |
| trajectory40 | 81.22 | 0.180 | 11.21 | 3.73 | 99.78 | 45.93 | 1.514 | 3.62 | 0.773 | 0.052 |
| trajectory80 | 141.97 | 0.561 | 30.71 | 5.31 | 180.55 | 412.44 | 15.134 | 11.76 | 1.220 | 0.121 |

Preparation totals are sequential robot preparation times including placement, union/crop construction, payload writing and BEVs. Full-height images are inspection controls. Layer time includes terrain estimation and feature extraction, so it is not pure rendering time. Native preparation and optimization are nested inside the registration stage and must not be added again. These are single measured controls, without repeated-run uncertainty estimates.

## Application communication

| Mode | Cached endpoints | Manifest [MiB] | Geometry [MiB] | Session / solution [MiB] | Total [MiB] |
| --- | --- | --- | --- | --- | --- |
| circle40 | 23 | 35.51 | 50.42 | 0.143 | 86.07 |
| trajectory40 | 51 | 61.80 | 135.79 | 0.184 | 197.78 |
| trajectory80 | 86 | 100.03 | 620.28 | 0.184 | 720.50 |

Bytes are actual serialized requests/replies plus IPC framing. On-demand geometry is transferred once per requested endpoint and reused for verification and registration. This excludes network headers, frontend traffic and inspection downloads.

## Graph-stage resources

| Mode | Observed wall [s] | Process-tree CPU [s] | Simultaneous RSS [MiB] | Whole-device GPU peak [MiB] |
| --- | --- | --- | --- | --- |
| circle40 | 3.25 | 13.90 | 1174 | 447 |
| trajectory40 | 7.20 | 18.10 | 1680 | 645 |
| trajectory80 | 29.59 | 42.17 | 2349 | 915 |

These measurements cover only the fresh corrected graph solve and solution delivery. Detection-capture resource measurements remain in the original run. No sum of historical memory peaks is reported. RSS is sampled every 20 ms and may count shared pages twice. GPU is sampled every 100 ms and includes driver overhead and any unrelated device activity. CPU seconds include ready/solution services and native child processes.

## Individual trajectory accuracy

| Mode | Robot | Raw individual ATE [m] | Corrected individual ATE [m] |
| --- | --- | --- | --- |
| circle40 | aerial05 | 0.196 | 0.213 |
| circle40 | aerial07 | 0.076 | 0.104 |
| circle40 | ground05 | 0.477 | 0.477 |
| circle40 | ground06 | 0.448 | 0.452 |
| trajectory40 | aerial05 | 0.196 | 0.230 |
| trajectory40 | aerial07 | 0.076 | 0.083 |
| trajectory40 | ground05 | 0.477 | 0.476 |
| trajectory40 | ground06 | 0.448 | 0.449 |
| trajectory80 | aerial05 | 0.196 | 0.209 |
| trajectory80 | aerial07 | 0.076 | 0.074 |
| trajectory80 | ground05 | 0.477 | 0.475 |
| trajectory80 | ground06 | 0.448 | 0.367 |

Each individual row uses its own rigid alignment. The headline joint ATE instead uses one shared alignment for all four robots; individual errors cannot substitute for joint alignment quality.

## Provenance and validation

See RESULTS.md, PROTOCOL.md, native-build.json, tests.log and tests-local.txt. Per-mode folders include configuration, source/input hashes, raw communication ledgers, native admissions, endpoint frames, placement histories and corrected trajectories. circle40 also includes saved immutable planner states. artifact-hashes.json covers this inspection package; capture-inspection-hashes.json preserves the original retrieved gallery hashes.

## Aligned 3D trajectories and continuity inspection

The [rotatable 3D view](aligned-trajectories-3d.html) uses the same single component alignment as the joint ATE calculation and retains every saved dense sample. True metric scale is the default; optional height exaggeration affects display only. A [PNG](aligned-trajectories-3d.png) and [vector PDF](aligned-trajectories-3d.pdf) are retained.

The [continuity audit](trajectory-audit/index.html) isolates motion introduced by changing graph corrections. For circle40 its maximum is 0.404 mm per sample across all robots, and 0.132 mm for A07. The prominent A07 eastern and southern bends appear in raw odometry and GT as well. No trajectory smoothing or reconstruction change was applied; the reported ATE values remain unchanged.
