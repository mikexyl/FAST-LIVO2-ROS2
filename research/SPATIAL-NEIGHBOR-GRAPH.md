# Spatial-support graphs for fixed circle submaps

Enable `pgo.spatial_neighbors.enabled: true` and `topology: spatial_only` in the
persistent CBS configuration. This mode has **no creation-order odometry chain**.
A new circle connects to older circles with validated common geometry, regardless
of their creation IDs. A05's 27--28 chronological link is therefore not retained
when those footprints do not overlap. Circle placement, publication/maturation,
point-cloud multi-layer retrieval and verified inter-robot loop measurements are
unchanged. The previous configuration remains a separate experimental control.

Build matching `cbs` and `cbs_ros` sources on `codex/spatial-neighbor-submap-graphs`
with `scripts/circle_submaps/build_cbs_spatial.sh`. Set `CBS_OVERLAY` to this
isolated build when launching. Native edge type 100 identifies a robot-local
map-consistency factor. Spatial PCM packets use protocol version 2; mismatched
peers fail closed. The historical raw-chain pipeline keeps protocol version 1.
`topology: creation_backbone` explicitly selects the previous augmentation mode;
disabling spatial neighbors preserves the original pipeline.

## Causal topology and support

Each immutable published crop can add edges to intersecting older footprints,
including its immediate predecessor only when geometry supports that edge.
Require at least 100 shared native point IDs and at least 10% of the smaller
crop's membership. Check hashes, fixed frames, footprint containment and all
shared coordinates (5 mm tolerance). Up to 2,048 deterministic shared-ID samples
must have a second principal-axis standard deviation of at least 1 m. Purely
line-like/concentrated shared observations do not qualify.

Prefer supported bridges between graph components, then cycles. Within these
classes prefer endpoints below the degree budget, then larger shared fraction,
shared count, shorter center distance and smaller ID. The normal budgets are
three spatial neighbors per endpoint and two new incident edges per publication.
A genuinely supported bridge may exceed those budgets to connect existing
components; every exception is recorded. Budgets limit extra cycles, not valid
forest connections. The total remains a spanning forest plus a bounded number of
cycle additions. Existing edges, their weights, IDs and circle locations never
change. No weak, distant or chronological fallback edge is inserted.

## What the local factors mean

Each accepted pair receives one nonlinear `BetweenFactor<Pose3>` with measurement
`inverse(T_world_i) * T_world_j`. Variables retain their creation-anchor IMU
frames; selection is based on circle footprints and shared map data. These
constraints preserve compatible corrections of crops containing the same
observations. They do not independently estimate or repair raw odometry drift,
and they do not add ICP or GPU registration factors.

Shared observations are correlated, so information does not grow with their
count. Define a regularizer at the shared geometry's centroid, expressed in
endpoint j's frame, with 3-degree rotational and 0.5-m translational scales.
For centroid transform C and right Pose3 tangent ordering [rotation, translation],

`Omega_ij = (0.25 / 3) Ad(C^-1)^T diag(sigma^-2) Ad(C^-1)`.

The scales and fraction are explicit configuration parameters, not estimated
uncertainty. This is a conservative regularization policy, not an independent
measurement or a correlation-aware posterior. Unlike the historical augmentation
mode, it uses neither creation-order path length nor odometry-chain covariance.
Supported bridge exceptions can raise incident total information above the usual
degree budget; diagnostics expose this and a matched accuracy ablation is needed.

## PCM and disconnected components

Local PCM obtains relative poses and covariance proxies through deterministic
shortest-edge-count spatial support paths. A path may move backwards through
creation IDs. A missing path is unavailable, never an identity transform.
Correlated cycles are not fused to manufacture more information. Peers exchange
direct summaries for each pair of incident separator keys, preserving the exact
chosen path covariance. They never compose those summaries into another chain.
Only verified loop proposals vote in PCM; map-consistency edges are not loop
votes or registration-geometry requests.

The present CBS interface has one anchor per physical robot. Only the local
component containing submap 0 enters that robot's solver. Other components stay
explicitly pending with their own component labels, original poses,
`optimized: false` and `reference_available: false`. Their loop endpoints are
unavailable to PCM. A later supported local bridge admits the newly connected
component into the same running process without moving or changing old factors.
Pending same-robot loops are logged separately. No extra gauge priors are added.

**Current limitation:** independent disconnected fragments are not separately
optimized, and an inter-robot loop alone cannot activate them. They need a local
support path to the robot's reference submap. This is explicit pending state,
not a claim of global alignment. `alignment_complete` is false while any fragment
is pending, and joint map export/dense trajectory interpolation refuse that
input. Supporting a distinct CBS anchor per fragment is a separate extension.
The sealed bridge rejects spatial-only mode rather than silently using chain PCM.

## Computation, communication and retained evidence

Support geometry stays local in a four-cloud LRU cache. These edges request no
new peer geometry. Total communication is not necessarily unchanged: protocol 2
sends O(s^2) pair summaries for s separator keys per robot pair, and changed graphs
can change CBS beliefs. Serialized traffic remains measured by the existing
counters. The fresh online comparison below measures both communication and
accuracy; it is not an exactly frozen-input ablation.

`local-graph.json` retains all components, admitted/pending IDs, support and
rejection decisions, construction times, cache sizes and edge diagnostics.
Published snapshots retain local constraints separately from verified loops.
`scripts/circle_submaps/inspect_spatial_graph.py` reconstructs publication-prefix
graphs from immutable saved crops. It does not replay new scans or optimize them.

Tests cover no chronological fallback, deferred components and later bridging,
causal immutable factors, deterministic restart, covariance transport, frame
invariance, corrupt geometry, weak support and historical-mode compatibility.
Native tests exercise spatial PCM summaries, missing paths, peer barriers,
serialized-byte accounting, persistent graph growth and GPU loop factors.
The new A05 inspection and verification records are in
[the spatial-only gallery](figures/spatial-only-a05-20261005/index.html).
Prior augmentation results remain in their original historical directory and
must not be cited as results of this replacement.

## GRACO all-14 online validation

The [2026-10-05 report](figures/circle-spatial-online-graco-all14-20261005/index.html)
records a fresh simultaneous 1x replay on workstation 148, without resource
quotas. It published 355 circles, admitted 439 local map-consistency factors,
and retained 113 of 114 loop proposals. All native odometry runs completed;
the largest successful-update gap was 0.232 s. All 14 recordings passed Rerun
verification. The exact source tree passed 51 Python/ROS and 17 native C++ tests.

All robot reference frames connected, but A03/19, A03/20 and A08/19 remained
pending. Consequently this is **not a complete all-14 alignment**, and no all-14
joint ATE or joint PLY is claimed. Excluding the entire A03 and A08 trajectories
in both runs, the same supported 12-robot subset achieved 0.967 m joint ATE,
versus 0.969 m for the earlier chain control. Evaluation uses evo 1.36.5,
corrected GRACO IMU-reference ground truth, 50 ms association and one shared
rigid alignment without scale fitting.

Application traffic was 526.967 MB (control: 546.767 MB); solver update
median/p95 was 0.367/1.363 ms (control: 0.351/1.003 ms). Fresh odometry,
asynchronous scheduling and publication can differ between runs. Incomplete
alignment also means these are not cost comparisons at an equivalent outcome.
The report separates application transport scope and full callback timing.

Matching native package checkpoints on `codex/spatial-neighbor-submap-graphs`:

- `cbs`: [40775cd](https://github.com/mikexyl/cbs/commit/40775cd)
- `cbs_ros`: [df0051b](https://github.com/mikexyl/cbs_ros/commit/df0051b)

Git retains the compact reports, figures, configurations, metrics and hashes.
Full capture and evaluation artifacts remain on 148 under
`/data3/mikexyl/swarm_s3e_ws/src/.ros2/circle-spatial-online148-20261005`.
The gallery's `FILES.sha256.json` inventories that complete inspection archive,
including trajectory files intentionally excluded from Git. Regenerating its
trajectory figures requires those retained files.
