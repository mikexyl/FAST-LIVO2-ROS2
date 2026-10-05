# Sparse spatial neighbors on circle submap graphs

This experimental backend retains the complete chronological odometry backbone
and adds **robot-local map-consistency factors** between supported, nonconsecutive
sealed footprints. Placement, maturation, retrieval, loop verification, PCM,
inter-robot GPU registration and trajectory/map reconstruction are unchanged.
The published mature-circle experiment remains a separate control.

Enable `pgo.spatial_neighbors.enabled` in a matched configuration. The example
`configs/circle_online_spatial_neighbors_graco_all14.yaml` differs from the mature
online configuration only in this block and its experiment label. Build the
matching `cbs_ros` branch `codex/spatial-neighbor-submap-graphs`: native wire type
100 is reserved by this adapter for map consistency. Older bridges reject it.
The CBS core requires no further change beyond the published anchor fixes.
Set `CBS_OVERLAY` to the newly built overlay when using `online148_env.sh`;
the launcher honors that override and otherwise retains the published default.

## Causal topology

Only published, immutable local rows and their owned point payloads are read.
As each new row becomes available, consider older intersecting footprints except
its chronological predecessor, which already has an odometry edge. Require at
least 100 shared native point IDs and at least 10% shared membership relative to
the smaller crop. Rank candidates by shared fraction, shared count, increasing
center distance, then older ID. Add at most two edges at creation of a graph node
and permit at most three added spatial neighbors at either endpoint. Rejected
edges and degree limits remain visible in diagnostics. A later endpoint cannot
replace or reweight an existing edge; replaying the same prefixes reconstructs
the same graph. Existing frames, circle centers and IDs never change.

Validate payload hashes, membership and footprint containment. Every common ID
must agree in the original map frame within 5 mm. Check all shared points in
bounded batches; use up to 2,048 deterministic shared-ID samples for spatial
support. The second principal-axis standard deviation must be at least 1 m:
line-like or concentrated support is rejected, while planar support is allowed.
The primitive correspondence here is exact identity, not a nearest-neighbor
surface match. Missing or changed evidence fails closed.

## Factor and correlation policy

Each accepted pair receives one nonlinear `BetweenFactor<Pose3>` with measurement
`inverse(T_world_i) * T_world_j`, in the original creation-anchor frames. The
shared geometry validates that this relation actually connects common map data.
The factor encourages compatible corrections of overlapping crops as external
loops deform the graph. It does **not** independently estimate or remove local
odometry drift. No ICP or additional GPU geometry factor is created for this
edge. Submap centers are only spatial-selection inputs, not pose-variable origins.

Shared crops reuse observations and are correlated with the raw map and chain.
Consequently, information does not grow with the number of shared points.
Compose the existing odometry covariances along the i-to-j chain using right
Pose3 tangents, transporting each edge covariance into endpoint j's frame:

`Sigma_ij = sum_k Ad(T_(k+1,j)^-1) Sigma_odom Ad(T_(k+1,j)^-1)^T`.

Then set `Omega_ij = (information_fraction / max_degree) inverse(Sigma_ij)`;
defaults are 0.25 and 3. This is an explicit, conservative regularization
heuristic, **not** a claim of statistically independent information or a fully
correlation-aware posterior. It needs a matched accuracy/deformation ablation.
Increasing degree or shared point count must not silently strengthen each factor.

The native adapter accepts the new edge only between distinct local endpoints
in metric Pose3 graphs. PCM passes it through without adding it to raw odometry
summaries, loop votes, peer packets or GPU registration manifests. Detected loops
retain their original admission and evidence policies. A same-endpoint collision
between a local constraint and a detected loop fails explicitly rather than
silently overwriting a factor. The current circle retrieval policy excludes
overlapping same-robot crops.

## Cost, output and verification

Local point payloads are cached in a four-cloud LRU; no peer cloud request is
needed for these edges. Additional geometry network bytes are zero by design.
This is **not** a claim of unchanged total communication: changed posterior
beliefs can affect CBS traffic, and added edges can increase solver work.
Persistent bridge outputs retain separate `local-graph.json`, local constraints
and map-consistency counts; detected-loop counts remain comparable. CPU time,
cache bytes, support/rejection decisions and input hashes are retained.

`scripts/circle_submaps/inspect_spatial_graph.py --store <owned-store> --output
<new-directory>` replays publication prefixes without optimization. Its A05
inspection is a graph-construction test on the retained online run, not a new
online replay or an accuracy experiment. Synthetic tests cover causality,
degree bounds, deterministic restart, covariance transport, common-frame
invariance, corrupted geometry, weak support and the disabled control. The native
growing three-robot test also enables GPU loop factors to verify that spatial
cycles survive revisions, propagate through beliefs, preserve loop counts and do
not cause geometry retransmission on an unchanged final revision.

## Retained implementation check (2026-10-05)

A05's 30 sealed submaps produce 29 unchanged backbone edges plus 33 supported
map-consistency edges (33 local cycles). Sixteen intersecting nonconsecutive
pairs fail shared membership support and eleven hit the degree budget. The
accepted pairs share 4,660–46,303 native IDs; the maximum map-frame discrepancy
is 0.0000114 m. Prefix construction on 148 took 1.318 s total, at most 81.1 ms
per prefix, with 11.66 MB of peak cached arrays and 157.7 MiB of fresh inspection
process peak RSS. These are construction measurements, not CBS solve timings.

Verification passed: 48 local Python tests (10 native-gated skips), 42 Python/ROS
checks on 148 (no skips), eight CBS-core C++ tests and nine CBS-ROS C++ tests.
The native growing three-robot GPU case includes seven new local factors per
robot, nine inter-robot registration pairs and a shared Alpha reference. Every
final geometry check passes. The unchanged last revision reuses cached geometry.

The [A05 comparison gallery](figures/spatial-neighbor-a05-20261005/index.html)
retains the actual edge list, candidate decisions, timing, input/source hashes,
JUnit records and native binary hashes. No new all-14 ATE result is claimed.
The new pipeline configuration is opt-in pending that matched dataset comparison.
Use CBS core `0a23e90` and CBS ROS `943bdbb` (or their matching descendants).
