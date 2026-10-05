# Distributed CBS over fixed spatial submap graphs

The fixed-circle CBS graph connects A05, A07, G05 and G06 with 121 owned submap poses and 13 live GPU registration factors. Joint ATE is 0.962 m versus 0.960 m for the centralized circle graph. Relative to 40 m trajectory-centered CBS, graph-stage communication falls by 65.5% and measured CPU time by 27.3%, while joint ATE changes from 0.951 to 0.962 m. These are single-run controls on 148.

| Submaps | Nodes | Retained/proposed loops | GPU factors | CBS joint ATE [m] | Central joint ATE [m] | CBS stage [s] | CPU [s] | Graph traffic [MiB] | RSS [GiB] |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| Fixed circles · 40 m | 121 | 13/13 | 13 | 0.962 | 0.960 | 13.11 | 14.92 | 2.13 | 2.02 |
| Trajectory centered · 40 m | 157 | 57/57 | 57 | 0.951 | 0.902 | 15.50 | 20.51 | 6.18 | 2.65 |
| Trajectory centered · 80 m | 157 | 126/126 | 125 | 0.851 | 0.823 | 24.46 | 41.66 | 12.40 | 3.50 |

A separate fresh-process fixed-circle repeat also connects all four robots and passes all 13 registration checks: joint ATE 0.961866 m versus 0.961862 m in the main run, wall time 14.09 s, graph traffic 2.134 MiB. This is a reproducibility check, not a timing distribution. See `circle40-repeat.json`.

All three controls connect all four robots. PCM rejects no verified loop in these captures. The 80 m
control retains 126 companion pose constraints but admits 125 registration factors: one geometry
factor fails initial overlap, as in the centralized control. Every admitted factor passes final support.

## Native message-order fix

A fresh-process repeat of the original CBS build lost G06's root reference. A relayed anchor had arrived
before its publishing robot's connecting separator; the receiver discarded it and unchanged subsequent
messages were suppressed. Anchor requests now acknowledge the receiver's factor-connected anchors.
When the publisher frame becomes connected but the target anchor remains missing, the sender refreshes
its bootstrap reply. Cached, unconnected pose guesses do not count as acknowledgements. The Gaussian,
producer identity, covariance and receiver's belief-update rules are unchanged. All peers must use the
matching rebuilt Keys service interface. The regression checks refresh gating, unchanged Gaussian
contents and stopping refresh after acknowledgement. All seven core tests pass. Final controls use
fresh coordinator/robot processes. The original failing run remains in `sealed-before-core-fix.json`.

An earlier attempted repair retained early anchor beliefs directly. It worsened alignment and failed
the 80 m control's final geometry checks, so it was reverted. Its results remain in
`rejected-anchor-retention.json`; they are not the implementation evaluated in the table.
No thresholds or measurement objectives were adjusted.

## Verification and shared-frame residuals

The local focused suite passes 35 tests, with six native checks skipped. The native distributed suite
passes 26 tests, with one separately gated legacy check skipped. The isolated build passes seven CBS
core and eight ROS C++ tests; one separately gated ROS distributed test is skipped. These overlapping
suites are not summed. Logs and JUnit results accompany the report.

The following diagnostics compare each verified loop measurement with the relative transform between
the two robots' published, owned poses. They use no ground truth and can expose inconsistent published
frames even when local factor support succeeds. They are residuals against noisy loop measurements,
not an independent accuracy estimate or an additional tuned acceptance threshold.

| Submaps | Maximum translation residual [m] | Maximum rotation residual [deg] |
| --- | --- | --- |
| Fixed circles · 40 m | 0.242 | 0.284 |
| Trajectory centered · 40 m | 0.295 | 0.264 |
| Trajectory centered · 80 m | 0.806 | 0.362 |

## What was implemented

Each fixed footprint owns a CBS Pose3 variable keyed by robot and submap ID. Creation-anchor IMU
frames, duplicate creation timestamps, immutable point membership and sealing availability remain
distinct. Robot-local odometry constraints connect these nodes; PCM-retained cross-robot loops add
pose constraints and the existing GPU VGICP factors. Geometry moves only after PCM, under canonical
ownership. Prepared-array hashes match the centralized controls for all 23 / 51 / 81 transferred
endpoints. Neither central optimized poses nor ground truth initializes CBS.

The dense trajectory and map reconstruction policy is unchanged: interpolate correction transforms
through creation anchors, average simultaneous-anchor corrections, and apply each submap correction
to points assigned by nearest footprint. The optimizer retains every submap even when anchors share
a timestamp. The interactive view displays all dense trajectory samples with a single shared rigid
alignment; it does not align robots individually.

## Execution and the original persistent limitation

**These successful controls use sealed CBS, with 100 local iterations at a requested 10 Hz.**
They are frozen graph replays using existing verified loops. Geometry was sealed at capture end.
No online geometry-freezing or live retrieval result is claimed.

The new transport also supports persistent prefixes, and the three-robot growing-graph test passes.
Before the repair below, the real four-robot chain exposed an existing persistent reference-propagation failure:
G06 remains in an A07 reference while the other robots reach A05. The first ten-iteration run was
rejected by the shared-frame check; keeping it running did not cure the missing multi-hop anchor
belief. The new persistent finalization barrier detects this condition before sealing and times out
instead of publishing a falsely aligned result. It does not repair the native algorithm. No silent
fallback occurred: the successful runs explicitly selected sealed mode. Failed-run status and logs
are retained in `persistent-failure.json`.

A separate regression test exposed a DDS startup race for a first prefix with no loops: a ready packet
could be sent before peer discovery. The persistent bridge now waits for subscribers before declaring
readiness. This transport fix is distinct from the original multi-hop reference defect.

## Measurement scope

The CBS-stage timer includes launch, distributed PCM, geometry preparation/exchange, optimization,
fixed settling and shutdown. It excludes reused placement, BEV/ORB preparation, retrieval, verification,
local store/hash preparation, and post-run evaluation. The 10-second settling setting masks part of the
computation difference. CPU is accumulated across the measured process tree; it can exceed wall time.
RSS is an actual simultaneous 20 ms process-tree sample, with possible shared-page double counting.
Whole-device GPU peaks are retained in comparison.json, sampled at 100 ms; they include driver overhead
and any unrelated device use. Each control uses a fresh coordinator and fresh robot processes.
No artificial CPU, memory or GPU quota was imposed on 148.

Graph communication is serialized application CDR, counting each request/response at the requester
and other sent messages per recipient. DDS discovery, network headers and retransmissions are excluded.
Reused descriptor retrieval and verification traffic is not included. Thus these values must not be
compared directly with the earlier full centralized backend totals of 86 / 198 / 720 MiB.

## Graph-stage communication breakdown

| Submaps | CBS request+response [KiB] | PCM [KiB] | Geometry [MiB] | Coordination [KiB] | Unique prepared endpoints |
| --- | --- | --- | --- | --- | --- |
| Fixed circles · 40 m | 74.1 | 16.6 | 2.045 | 1.5 | 23 |
| Trajectory centered · 40 m | 132.3 | 60.4 | 5.992 | 1.5 | 51 |
| Trajectory centered · 80 m | 216.1 | 125.9 | 12.065 | 1.5 | 81 |

Geometry is prepared once per owned/supplied endpoint and reused across factors. CBS uses the exact
same fixed 0.4 m verification preprocessing followed by the existing adaptive 0.5 m / 12,000-point
registration cap. Actual effective voxel sizes are retained in the manifests; 0.5 m is not guaranteed
after the cap. GPU voxel settings and information caps are unchanged.

## Native per-robot timing

| Submaps | Robot | Owned GPU factors | Timer update median [ms] | Timer update p95 [ms] | Belief marginalization median [ms] | Belief marginalization p95 [ms] |
| --- | --- | --- | --- | --- | --- | --- |
| Fixed circles · 40 m | aerial05 | 10 | 0.052 | 0.209 | 0.074 | 0.249 |
| Fixed circles · 40 m | aerial07 | 1 | 0.031 | 0.125 | 0.192 | 0.270 |
| Fixed circles · 40 m | ground05 | 2 | 0.051 | 0.171 | 0.095 | 0.115 |
| Fixed circles · 40 m | ground06 | 0 | 0.036 | 0.147 | 0.139 | 0.161 |
| Trajectory centered · 40 m | aerial05 | 36 | 0.066 | 0.388 | 0.210 | 0.504 |
| Trajectory centered · 40 m | aerial07 | 11 | 0.051 | 0.236 | 0.224 | 0.477 |
| Trajectory centered · 40 m | ground05 | 10 | 0.044 | 0.147 | 0.158 | 0.196 |
| Trajectory centered · 40 m | ground06 | 0 | 0.032 | 0.114 | 0.156 | 0.213 |
| Trajectory centered · 80 m | aerial05 | 86 | 0.066 | 4.072 | 0.201 | 1.010 |
| Trajectory centered · 80 m | aerial07 | 20 | 0.072 | 1.264 | 0.243 | 0.823 |
| Trajectory centered · 80 m | ground05 | 19 | 0.058 | 0.475 | 0.209 | 0.324 |
| Trajectory centered · 80 m | ground06 | 0 | 0.041 | 0.302 | 0.205 | 0.312 |

These are the logged native timer updates and belief marginalizations, not total pipeline latency.
Timer CSVs contain 99 nonterminal callbacks; the final iteration is separately marked complete.
Registration preparation, waiting, startup, transport and other callback work are outside these narrow
columns. The process-tree CPU and wall measurements above include the complete graph-stage lifetime.

## Individual ATE, each with its own rigid alignment

| Submaps | Robot | Raw ATE [m] | CBS ATE [m] |
| --- | --- | --- | --- |
| Fixed circles · 40 m | aerial05 | 0.196 | 0.204 |
| Fixed circles · 40 m | aerial07 | 0.076 | 0.080 |
| Fixed circles · 40 m | ground05 | 0.477 | 0.477 |
| Fixed circles · 40 m | ground06 | 0.448 | 0.449 |
| Trajectory centered · 40 m | aerial05 | 0.196 | 0.217 |
| Trajectory centered · 40 m | aerial07 | 0.076 | 0.072 |
| Trajectory centered · 40 m | ground05 | 0.477 | 0.476 |
| Trajectory centered · 40 m | ground06 | 0.448 | 0.446 |
| Trajectory centered · 80 m | aerial05 | 0.196 | 0.189 |
| Trajectory centered · 80 m | aerial07 | 0.076 | 0.067 |
| Trajectory centered · 80 m | ground05 | 0.477 | 0.470 |
| Trajectory centered · 80 m | ground06 | 0.448 | 0.437 |

Headline joint ATE uses one shared SE(3) alignment for all four robots. Evaluation uses evo 1.36.5,
50 ms association, no scale fitting, and the corrected GRACO IMU-reference ground truth. Ground truth
is used only after the solver has finished. Input/configuration/source/native hashes, per-robot poses,
factor manifests, PCM decisions, timing CSVs and evaluation results accompany this report.

## Persistent CBS at 10-second intervals

A separate persistent test completed 30 iterations per robot at a requested 10-second cadence. The final common-frame check failed; G06 remained in A07’s frame. [Cadence, timing, geometry checks and retained failure evidence](persistent-10s/index.html).

## Validated persistent repair

Persistent CBS is now validated on this four-robot circle graph: all four robots reach A05’s reference, with 0.955 m joint ATE after 30 independent iterations at a requested 10-second cadence. Two defects were repaired: disabled multi-hop anchor relay and a factor-slot/variable-key mix-up that discarded odometry initialization. Published loop transforms and all 13 geometry factors pass. Stages remain independently random; no coordinated stage barrier or centralized initialization was added.

[Corrected persistent report](persistent-fixed/index.html). The relay-only intermediate was rejected despite matching reference labels (87.899 m joint ATE); its [audit](persistent-anchor-relay/index.html) is retained.
