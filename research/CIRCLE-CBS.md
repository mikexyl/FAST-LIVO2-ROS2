# CBS over fixed spatial submap graphs

## Fresh online experiment (2026-10-01)

`scripts/circle_submaps/run_online_graco.py` is the fresh sensor-replay entry
point. `online148_env.sh` selects the validated native frontend, multilayer
MapClosures adapter and GPU CBS overlay. All 14 bags replay concurrently at 1×;
circle preparation, peer retrieval/verification, PCM and persistent CBS run
concurrently. The launcher imposes no host CPU, RAM or GPU quotas. Numerical
libraries use one thread per process, matching the previous successful online
frontend setup; each native frontend retains four ROS callback threads.

Unlike the frozen replay described below, this experiment publishes **immutable
creation-time geometry**. Each newly placed 40 m circle receives only the native
map representatives observed by that creation update. Centers and IDs stay fixed.
Later observations accumulate in the growing map but never revise already
published evidence. The 99% coverage target applies to spatial footprints, not
to the fraction of eventual geometry present in frozen payloads. Geometry
maturation/revision is not implemented in this experiment.

The source is an append-only stream of native 80 m accumulated-area observations
(10 s / 20 m snapshot triggers), without odometry submapping. This is a growing
union of those observations, not an export of every original scan. Sensor anchor
and seal timestamps remain separate from Unix wall publication timestamps.
Descriptors become visible only after their immutable geometry is written.
The same point evidence feeds verification and post-PCM registration exchange.
Peer workers are isolated from other robots' stores. Their spawned verifier
processes have explicit read permission for the pinned multilayer native adapter;
otherwise a legacy adapter in the Python environment can shadow it after
Landlock isolation.

Persistent CBS captures growing prefixes on a 10 s schedule and requests one
local iteration per 10 s, with asynchronous pose/anchor alternation. Busy
admission coalesces prefixes without dropping previously agreed measurements.
At end of capture it waits for the lowest-ID reference to propagate through
every retained connected component, then runs 30 settling iterations per robot.
This final drain is reported separately from playback-time publications.

`finish_online_graco.py` consumes the actual final distributed estimates. It
does not rerun detection, verification or graph optimization. It independently
reconstructs every creation-time observation prefix to check geometry membership,
hashes and absence of future points; checks that both endpoints received a loop
before graph admission; verifies finite chronological odometry, update gaps and
final geometry support; and measures when shared solutions became available
during playback. Ground truth is introduced only by this post-run evaluator.

Communication includes measured peer descriptor fan-out, geometry replies,
constraints, PCM, registration exchange and CBS requests/replies as serialized
application bytes. DDS/IP overhead, discovery, local sensor transport and
visualization/download traffic are excluded. End-to-end resources are sampled
over the actual simultaneous process tree, with whole-device GPU samples
reported separately. Historical staged IPC totals are not a matched online
communication control.

The retained short integration run is on 148 at
`.ros2/circle-online148-20261001/smoke5`; earlier failed startup/integration attempts
remain beside it. The full experiment uses the fresh `full` directory. The
inspection report records connectivity failures as well as successful execution;
disconnected maps are exported separately and are never called a global map.

The completed full replay produced 358 circles and four PCM-retained loops. Only
Ground 01–04 connected (joint ATE 1.369 m); eight aerial robots and Ground 05–06
remained singletons. All odometry streams stayed finite and chronological, with
a maximum successful-update gap of 0.212 s. The first shared alignment appeared
219.2 s after playback began, before any bag finished. The run used 20 growing
graph revisions and 49–68 iterations per robot, including final settling.

Creation-time freezing exposed a retrieval limitation: 197/358 circles lacked
terrain support and therefore had empty layer descriptors in the unchanged
legacy multilayer mode, which has no full-height fallback. Frozen payloads
contained 34.7% of final retained point IDs; only five candidates reached 3D
verification. The experiment completed but did not reproduce the staged all-14
connection. Its 128.324 MB of application traffic cannot be presented as an
equivalent-outcome communication improvement. Geometry maturation/versioning
and full-height fallback require separate follow-up controls.

The [online inspection report](figures/circle-online-graco-all14-20261001/index.html)
retains component maps, accuracy, exact native configurations, causality audits,
byte counters and verified recording metadata. Local tests: 33 passed, three
native-gated skips; actual 14-process short/full live runs exercise DDS. All 14
Rerun files passed the native footer/content validator. The first report preview
attempt hit an Agg scatter-renderer crash; only previews were regenerated, using
a deterministic raster renderer. Numerical results and PLYs were preserved.

## Frozen replay implementation

This opt-in adapter treats each sealed spatial footprint as a Pose3 variable with
key `(robot_id, submap_id)`. It uses the same odometry-chain constraints, verified
loop measurements and GLIM GPU VGICP registration configuration as the centralized
circle experiment. It does not change EllipseLIO, placement, retrieval thresholds,
geometry membership, or the default production pipeline.

The node frame is the footprint's **creation-anchor IMU frame**, not a coordinate
frame at the circle center. Circle centers and IDs stay fixed in the original
horizontal map frame. Optimizing the node re-poses its geometry. Multiple circles
created in one update have separate keys with equal timestamps and initially equal
poses. The relative odometry constraint between these nodes is identity. CBS's
keyed Estimate messages are authoritative; a timestamp-indexed TUM export cannot
represent these duplicate anchor times.

## Geometry and distributed ownership

Each robot reads only its own immutable store. The adapter validates schema 6,
creation/sealing timestamps, frame transforms, point membership, the footprint
boundary and the source NPZ hash. The transport adds a version-3 point envelope
with the immutable row hash and source hash. Both loop endpoints carry the same
provenance, including the separate geometry availability timestamp. Changed
duplicates, wrong frames, unexpected geometry and persistent cache mutations
fail closed. Legacy point-GICP and native ellipsoid envelopes remain supported.

Only PCM-retained loops request geometry. The native numeric endpoint order
chooses one factor owner, even if robot names sort differently. The owner requests
its peer's prepared geometry; other robots receive no peer filesystem access.
Unchanged evidence is cached in a persistent session. The existing ready barrier,
session/configuration fingerprint, singleton PCM policy, conservative factor
scale and final geometry-support check remain in force.

For matched controls, verification first prepares a fixed 0.4 m voxel cloud.
CBS then applies the existing 0.5 m voxel / 12,000-point adaptive budget to that
same cloud, exactly as the centralized experiment did. The runner checks every
transferred endpoint's prepared-array SHA-256 against the centralized run. The
actual registration sampling resolution can exceed 0.5 m when the point cap is
active; the manifests retain it. GPU VGICP uses the unchanged two voxel levels.

Geometry is still frozen once at the end of capture. Placement is incremental,
but this experiment is a **frozen graph replay**, not an online geometry-freezing
policy. Verified loops are reused; centralized optimized poses and ground truth
are never inputs to CBS.

## Execution and reconstruction

`circle_cbs_graco_four.yaml` selects the established **sealed** CBS execution,
distributed PCM and live GPU registration factors. It retains the existing
100-iteration settling setting and 10 Hz local timer. Each robot optimizes its own
submap variables and exchanges beliefs over ROS2/DDS. No centralized solve is
used to initialize or repair its result.

The accompanying CBS transport fix acknowledges anchors that are actually
connected by factors at the receiver. A relayed anchor arriving before its
publisher's separator could otherwise be rejected while the sender suppressed
unchanged replies. Once the publisher frame is connected but the target anchor
is still missing, an anchor-stage request refreshes that endpoint's bootstrap
reply. The Gaussian, producer identity, covariance and receiver update rules
remain unchanged. The request does not mistake a cached, unconnected pose guess
for a connected anchor. Core tests verify refresh gating, unchanged Gaussian
contents and stopping refresh once the recipient acknowledges the anchor.

An earlier attempted repair that immediately retained early anchor beliefs was
rejected after it worsened joint alignment and failed final registration checks
in the 80 m control. It is not the retained implementation; its diagnostics stay
on 148. No registration or PCM threshold was adjusted.

The adapter also accepts `dpgo.execution: persistent`. The original implementation
failed on the GRACO chain A05–A07–G05–G06: G06 obtained an A07 reference but not
A05. Anchor covariance construction excludes incoming anchor-belief factors,
while persistent responses had also disabled coordinated CBS's anchor relay.
Waiting longer could not supply a covariance for a distant anchor absent from
that cavity graph. The original failed runs remain retained, without a mode
fallback or external alignment repair.

The repair restores anchor relaying with the original producer identity, mean,
covariance and frame. Forwarded beliefs are not damped a second time or relabelled
as independent local measurements. Unchanged relays and echoes to their producer
are suppressed. Pose sharing, local-belief damping and both covariance exclusions
are unchanged. Five-robot chain and ring tests check propagation, nonidentity
frames, independently selected stages, reordered delivery, Gaussian preservation
and stable factor counts. The old core fails the new chain regression. Real DDS
tests also cover a growing five-robot chain with pose-only and GPU factors.

Real-data verification exposed a second independent defect in persistent graph
initialization. `NonlinearFactorGraph::exists` checks a factor slot, not a variable
key; using it to filter supplied initial poses discarded the entire odometry
trajectory. It is replaced by membership in `graph.keys()`, also for deciding
whether a pending belief attaches to a newly supplied factor. The regression
uses a long, rotated trajectory with a remote separator and an unconnected seed.
It verifies the first update, before transient errors can become outgoing beliefs.

The relay-only intermediate is explicitly rejected: matching reference labels
hid inconsistent transforms (87.899 m joint ATE, 208.673 m maximum published loop
mismatch). The probe now audits the published loop endpoints using the existing
2 m / 15 degree alignment tolerances; this check does not use ground truth.

The persistent adapter now checks the expected lowest native-ID reference for
every PCM-retained connected component before sealing and starting the final
settling count. A timeout is reported if the reference cannot propagate. It keeps
all optimizer processes and geometry caches alive across that final unchanged
prefix. This barrier is still a separate validation check; the native relay
repair must establish the common frame before the run can pass.

`scripts/circle_submaps/probe_persistent_cadence.py` tests this limitation at a
configurable native timer rate without modifying solver or belief settings. Its
defaults are one iteration every 10 seconds and 30 final settling iterations.
The native timer callback is mutually exclusive within each robot, and requests
retain the existing asynchronous, single-outstanding-request-per-peer policy.
The probe preserves final robot estimates, a 100 ms observation timeline, native
timing CSVs, geometry checks, source hashes and resource measurements even if the
common-frame gate fails. It evaluates joint ATE only after that gate passes.
This is still a frozen-graph test of persistent optimizer processes, not a live
frontend or incremental geometry-freezing experiment.

The original 148 cadence test completed all 30 iterations per robot in 320.9 seconds.
All 13 admitted geometry factors passed final checks, with no expired belief
requests. The common-frame gate still failed: G06 remained in A07's reference.
The maximum logged local timer callback was 435.4 ms, below the 10-second period.
The [cadence report](figures/circle-cbs-graco-four-20261001/persistent-10s/index.html)
retains the observed intervals, timing scope, final estimates and binary hashes.
The [relay-only intermediate](figures/circle-cbs-graco-four-20261001/persistent-anchor-relay/index.html)
retains the rejected result that exposed the initialization defect.

Output preserves each optimized submap separately. For dense trajectories,
corrections at equal creation times are averaged and interpolated in SE(3) over
time. For the map, each retained point uses its nearest footprint's optimized
correction; uncovered points are retained and flagged. This is the same
reconstruction policy used for the centralized controls.

## Reproduction and measurements

Run `scripts/circle_submaps/run_cbs_comparison.py` with `--central-root`,
`--config research/configs/circle_cbs_graco_four.yaml`, `--source` (the workspace
containing cbs, cbs_ros and FAST-LIVO2-ROS2), and a fresh `--output` directory.
`CBS_OVERLAY` identifies the isolated, validated GPU CBS install.
The companion checkouts are `../cbs-circle-submaps` and
`../cbs_ros-circle-submaps`, on `codex/circle-submap-graphs`. Their changes are
independent of pending original checkouts. `scripts/circle_submaps/build_cbs_spatial.sh`
builds both in an isolated overlay, including the extended Keys service. All
robots in a session must use that matching interface build. The GPU registration
adapter and upstream geometry library are unchanged.

The three controls are circle-40 m, trajectory-centered-40 m, and the previous
trajectory-centered-80 m footprints. Runs are serial on 148, with no host CPU,
memory or GPU quota. Each control starts a fresh coordinator and fresh robot
processes. Algorithm thread counts are held fixed. Geometry and large
maps remain on 148; compact diagnostics and inspection outputs are retrieved.

The graph-stage timer includes process startup, PCM, geometry preparation and
exchange, optimization and settling. It excludes reused placement, BEV generation,
retrieval, verification, local immutable-store preparation, and post-run evaluation.
Native update and belief-service CSVs provide narrower computation measurements.
Simultaneous process-tree RSS is sampled every 20 ms; shared pages can count twice.
GPU memory is a 100 ms whole-device sample, not an isolated allocation measurement.

Communication is serialized application CDR. CBS counts requests and responses at
the requester only; their sum counts those messages once. PCM, geometry and
coordination count sent bytes per recipient. DDS discovery, RTPS/IP headers and
retransmissions are excluded. Reused retrieval traffic is reported separately and
must not be mistaken for a new end-to-end distributed benchmark.

Evaluation uses evo 1.36.5, 50 ms association, one rigid alignment per connected
component and no scale fit, with GRACO's corrected IMU-reference ground truth.
Single-run timings are descriptive, not repeated-run uncertainty estimates.

## Verification

For the anchor/init repair, the local focused suite passes 21 tests (eight native-environment checks skipped).
On 148, the distributed regression suite passes 28 tests (one separately gated
legacy test skipped), and all eight CBS core C++ tests pass. The ROS package passes
eight C++ tests (one separately gated distributed test skipped). The native tests
use the isolated core and ROS interface build with the unchanged GPU adapter. These suites overlap;
the counts are not added. Coverage
includes duplicate anchor times, membership/source/frame corruption, immutable
persistent caches, reversed native robot ordering, growing GPU graphs, delayed
peers, PCM revocation, and the existing point-GICP transport. A real no-loop
initial prefix exposed and verified the fix for the DDS discovery/ready race.

All three four-robot sealed dataset runs pass the shared-frame and final geometry
checks. After the relay and initialization repairs, the four-robot persistent chain
also passes: 30 iterations per robot at 10-second intervals, common A05 reference
by iteration 8, 0.954865 m joint ATE, and 0.227704 m / 0.288652 degree maximum
published loop mismatch. All 13 GPU factors pass. The earlier failures are retained.
The [corrected persistent report](figures/circle-cbs-graco-four-20261001/persistent-fixed/index.html)
contains source/binary hashes, communication, timing and regression evidence. See the [comparison report](figures/circle-cbs-graco-four-20261001/index.html).

## All 14 GRACO robots, 2026-10-01

The 40 m / 99% circle run reuses retained validated odometry observations and
rebuilds placement, point-cloud BEVs, multilayer verification, PCM and persistent
CBS. All 14 robots join the A01 reference: 356 circles, 174 verified and
PCM-retained loops, 174 owned GPU registration factors, and 0.952042 m joint ATE.
The final native geometry checks pass for every robot. Maximum published loop
mismatch is 0.584714 m / 0.946586 degrees. The graph stage takes 441.9 s including
the 10-second cadence, reference propagation and 30 final settling iterations.

The first native retrieval attempt segfaulted after 38 accepted loops. A restart
with identical prepared inputs and thresholds completed; the cause of the first
crash remains undiagnosed. Its failure record is retained. This is a staged
backend experiment, not a fresh bag replay or an online geometry-freezing test.

The [all-robot report](figures/circle-cbs-graco-all14-20261001/index.html) records
timing and communication scopes, source hashes, per-robot results and validation.
The gravity-aligned PLY contains 12,951,872 native representatives, with dark
robot colors and individual optimized submap corrections. It is 271,989,714 bytes,
independently read by Open3D and SHA-256 verified after download. Bulk geometry
and the complete experiment remain on 148 under
`.ros2/circle-all14-20261001/recovery-1`; the requested PLY is also in the report's
local `exports/` directory and excluded from Git.
