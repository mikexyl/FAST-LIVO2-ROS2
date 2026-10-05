# CBS over fixed spatial submap graphs

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

The adapter also accepts `dpgo.execution: persistent`. Growing-prefix synthetic
tests pass, but the GRACO four-robot chain exposed a pre-existing persistent
anchor-propagation limitation: G06 obtains an A07 reference but not A05 through
the chain A05–A07–G05–G06. Persistent anchor covariance construction removes all
incoming anchor-belief factors, and persistent responses do not relay them.
Waiting more iterations did not resolve it. This is **not a successful persistent
four-robot result**, and the runner never silently switches execution modes.

The persistent adapter now checks the expected lowest native-ID reference for
every PCM-retained connected component before sealing and starting the final
settling count. A timeout is reported if the reference cannot propagate. It keeps
all optimizer processes and geometry caches alive across that final unchanged
prefix. This barrier detects the limitation; it does not change the native
belief algorithm or claim to fix it.

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

The 148 cadence test completed all 30 iterations per robot in 320.9 seconds.
All 13 admitted geometry factors passed final checks, with no expired belief
requests. The common-frame gate still failed: G06 remained in A07's reference.
The maximum logged local timer callback was 435.4 ms, below the 10-second period.
The [cadence report](figures/circle-cbs-graco-four-20261001/persistent-10s/index.html)
retains the observed intervals, timing scope, final estimates and binary hashes.

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

The local focused suite passes 35 tests (six native-environment checks skipped).
On 148, the distributed regression suite passes 26 tests (one separately gated
legacy test skipped), and all seven CBS core C++ tests pass. The ROS package passes
eight C++ tests (one separately gated distributed test skipped). The native tests
use the isolated core and ROS interface build with the unchanged GPU adapter. These suites overlap;
the counts are not added. Coverage
includes duplicate anchor times, membership/source/frame corruption, immutable
persistent caches, reversed native robot ordering, growing GPU graphs, delayed
peers, PCM revocation, and the existing point-GICP transport. A real no-loop
initial prefix exposed and verified the fix for the DDS discovery/ready race.

All three four-robot sealed dataset runs pass the shared-frame and final geometry
checks. This does not make the four-robot persistent chain successful; its failure
is explicitly retained. See the [comparison report](figures/circle-cbs-graco-four-20261001/index.html).
