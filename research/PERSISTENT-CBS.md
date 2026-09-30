# Persistent CBS+ backend

This opt-in mode implements Section V and Algorithms 1–2 of the supplied
`root (44).pdf` draft (SHA-256
`ca28f9411dcfa28cea53a6b18f267d0e29c5e637d3648517f9a0add3f58ad9ab`). It changes the CBS backend lifecycle and belief exchange;
EllipseLIO, accumulated-area membership, BEVs, retrieval, verification, PCM
thresholds and registration objectives remain configured independently.

## Algorithm

Each robot retains one BPSAM/iSAM2 instance, its locally anchored frame, received
pose and anchor beliefs, per-recipient outgoing belief histories, immutable
endpoint geometry and live registration factors. New admitted measurements are
inserted between optimizer updates. Neither a graph revision nor a stage switch
reinitializes the optimizer.

The draft defaults are a 1 Hz local timer, independent Bernoulli pose-stage
selection with probability 0.5, and Hellinger thresholds `Dmin=0.01`,
`Dtarget=0.1`, `Dmax=0.3`. Empty key requests ask for anchors. Responses use
current optimized means, with pose covariances from the local measurement graph
and anchor covariances from the local graph plus retained pose-belief factors.
Anchor-belief factors are excluded from both cavity graphs. Mixed requests are
partitioned by key type. Cavity factorizations are cached until the next update.

The responder compares each raw belief with its last **sent** belief to that
recipient, filters outside `[Dmin,Dmax]`, then applies the existing CBS
closed-form Hellinger step with a fixed `Dtarget`. There is no geometric target
decay and no second contraction at the recipient. A previously unseen key is
sent once without a divergence test and accepted immediately, providing the
bootstrap that the draft's previous-belief notation presupposes. A belief that
arrives before any connecting separator is retained pending that connection;
it does not create an unanchored graph island or require another bootstrap
reply. Persistent
responses originate from the responding robot's optimized estimates; they do
not relay another robot's old Gaussian as a fresh local belief.

Requests are asynchronous, with at most one outstanding request per neighbor.
Slow or unavailable peers do not block other neighbors or the local timer.
A request exceeding the configured timeout is expired; its retained beliefs
remain in the graph. Replies to expired requests are ignored. The existing
upper-triangular covariance wire format is retained.

## Graph admission and geometry

A `PersistentInput` contains a session ID, a sequential revision number, the
complete robot-local proposal prefix and an explicit final seal. Exact duplicate
inputs are idempotent; changed duplicates, replaced keyframe metadata, skipped
revisions and input after the final seal fail. Ground truth is never input.

Each revision has a separate immutable **distributed PCM transaction**, using
the same live subscriptions. One transaction's early messages are buffered with
a bounded per-peer/per-phase table. This transaction is an admission barrier,
not a barrier between optimization stages. CBS continues optimizing the last
admitted graph while PCM exchanges the next prefix. Geometry preparation is
finished before applying its graph revision.

Only PCM-retained loops can request peer geometry. Existing payloads are reused
from the robot's immutable local cache, which remains inside its Landlock read
capability. No worker receives peer filesystem access. Manifest/session/hash
checks and the supplier-ready barrier remain enabled. Each primitive/point
endpoint is prepared and uploaded only once. Unchanged companion constraints
reuse the same registration factor object and fixed scale. A replaced admitted
constraint replaces its companion geometry factor using the same cached
endpoints and rechecks support at the new measurement.

**Retraction limitation:** Algorithm 1 specifies adding measurements, not
retracting already shared evidence. If a subsequent PCM maximum clique revokes
an admitted loop, this implementation fails closed and preserves diagnostics.
It does not keep the revoked loop, silently change PCM, or restart CBS. Such
inputs currently require the historical sealed mode; a belief-retraction
protocol is separate work. Endpoint arrays and factors still grow with accepted
map content. Persistent mode is not a bounded-memory or factor-pruning method.

## Use

Use `research/configs/persistent_pipeline.yaml` with the existing live-capture
launcher, which invokes `s3e_pipeline.online_epochs`. Its `dpgo.execution:
persistent` selects the new session lifecycle. The historical default remains
`sealed` when this field is absent. GPU builds and `CBS_OVERLAY` selection use
the existing GLIM GPU setup.

There are two distinct clocks:

- `dpgo.update_interval_s: 10.0` captures a new causal input prefix.
- `dpgo.persistent_loop_rate_hz: 1.0` requests a local CBS iteration.

A timer is a requested cadence, not a real-time guarantee. If admission is busy,
the online adapter coalesces complete growing prefixes and records this fact;
measurements in their union remain present. Optimizer iterations continue on the
previous admitted prefix. Once all live workers finish, an explicit final prefix
is applied, followed by `persistent_settle_iterations` (default 10), final
registration quality checks, and a shared-frame check. Odometry-divergence
shutdown remains the responsibility of the existing capture supervisor.

For an already verified frozen graph, `s3e_pipeline.dpgo.run` accepts
`execution: persistent` with `mode: frozen`. This submits one final prefix;
it does not claim to reproduce an online arrival stream. The staged `mode: peers`
entry point fails explicitly in persistent mode: use the live `online_epochs`
route for concurrent retrieval, or produce a loops artifact first. It never
silently executes the sealed optimizer under a persistent label.

Native optional overrides in `dpgo.cbs_parameters` are `persistent_d_min`,
`persistent_d_target`, `persistent_d_max`, `pose_stage_probability` and
`belief_request_timeout_sec`. Lifecycle/PCM/registration overrides are rejected
there; use their dedicated configuration fields.

Outputs retain input/configuration/source/binary hashes, process IDs, revision
numbers, continuously published poses, PCM decisions, geometry manifests,
registration support, timing and serialized-byte counters. Process-tree memory
is sampled simultaneously. Native `NodeStats.belief_stage` and timing logs show
actual stage selection and update duration. Existing epoch visualization paths
remain available; a persistent epoch's displayed solution continues updating
until the next prefix is admitted.

## Verification

See [verification.json](figures/persistent-cbs148-20260928/verification.json) and
[the verification report](figures/persistent-cbs148-20260928/REPORT.md) for retained
run results.
The implementation tests cover fixed-target damping, both divergence gates,
bootstrap and per-recipient history, retained state across growth, cavity
exclusions/cache invalidation, geometry reuse and immutable metadata, pose-only
and GPU DDS graph growth, and the online adapter. Native GICP, GPU VGICP,
ellipsoid, covariance transport, and legacy belief tests remain regression gates.
