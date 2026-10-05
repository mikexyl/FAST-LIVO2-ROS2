# Online geometry maturation for fixed circles

The immediate-freeze all-14 GRACO trial created 358 fixed footprints but connected
only Ground 01–04. 197 snapshots had no supported terrain-relative layers, and
their immutable evidence contained only 34.7% of final retained point IDs. This
experiment changes when geometry is published, while preserving the placement,
terrain estimator, retrieval channels, matching thresholds, verification, PCM and
GPU registration settings.

## Publication policy

Select `circle_publication.mode: mature_then_freeze` using
`configs/circle_online_mature_graco_all14.yaml`. The historical
`freeze_at_creation` mode remains available and is the code default.

Circle placement still acts immediately on the growing occupied-cell map. Each
circle keeps its original ID, 40 m center/radius and creation-anchor IMU pose.
Unpublished geometry continues to accumulate all native representatives observed
inside that fixed footprint, regardless of scan membership. The source remains
the chronological union of native 80 m area observations, not every raw scan.

Normal publication requires:

- At least 30 seconds since footprint creation, measured in sensor time.
- No more than 5% growth in both unique native point IDs and occupied 1 m cells
  over a trailing interval of at least 30 seconds.
- A terrain plane that passes the unchanged support gates, and at least the
  existing five-feature retrieval minimum across the terrain-relative layers.

The feature minimum is a publication prerequisite, not a lowered correspondence
or registration threshold. Publication uses the descriptor computed from that
same sealing crop, avoiding inconsistent membership or a second descriptor fit.

Publication follows permanent circle-ID order to preserve the current append-only
store and creation-ordered odometry graph. Later circles continue accumulating
while an earlier circle waits. A 120-second sensor-time deadline bounds this
head-of-line delay. Deadline releases may remain unsupported; they are labelled
`deadline`, with their actual support status. End-of-stream tails are labelled
`end_of_stream`, not falsely called stable. A deadline is checked at the next
available native observation; no geometry is synthesized during sensor gaps.

After publication the geometry, hash, descriptor and CBS evidence remain
immutable. Geometry revisions, loop retraction and replacing factors are outside
this initial experiment. Points arriving later are retained in the growing map
for later circles and final visualization.

## Time and frame contracts

`stamp_ns`, `T_world_body`, `T_world_imu` and `created_update` identify the original
creation anchor. `sealed_update`, `seal_ns`, and `native_available_ns` identify the
latest observation included when the crop is sealed. The member-time upper bound
is that seal timestamp. All points are expressed in the original creation-anchor
IMU frame; the circle is never moved to the current robot pose.

`available_ns` remains the wall time at which both geometry and descriptor have
been written. Peers cannot query or retrieve a pending crop. The version-3 spatial
endpoint envelope binds the unchanged geometry hash and full row hash; no CBS
core change is required.

## Verification and comparison

Tests cover accumulating later observations before sealing, retaining old anchors
under sensor motion, fixed centers, spatial and point-growth stability, quality
failure, bounded FIFO publication, deadline and tail labels, immutable geometry
after publication, and reversed loop endpoint provenance. The independent audit
reconstructs each sealing prefix from native payloads and checks every published
point ID and coordinate, growth decision, source-prefix hash, descriptor hash,
and both-endpoint receipt before graph admission.

Run a short all-14 live integration test before a fresh full 1× all-14 replay on
148. Keep prior captures and source snapshots intact. The full trial uses the
same calibration and reliable sensor QoS, four ROS callback threads per frontend,
one numerical-library thread per process, no host resource quotas, and persistent
CBS at a requested 10-second iteration cadence with final reference propagation
and 30 settling iterations.

Compare actual peer application bytes, simultaneous process-tree memory, wall
time, publication delays/reasons, terrain failures, verification counts,
connectivity and component ATE against immediate freezing. Fresh odometry replay
introduces small run-to-run differences; do not claim an exactly frozen-input
ablation. Ground truth is restricted to post-run evo 1.36.5 evaluation, with 50 ms
association, rigid alignment and no scale fitting. Report final settling and
connections first achieved after playback separately.

The full-height fallback remains disabled so it does not confound this control.
Stable geometry does not guarantee semantic terrain or correct place recognition;
all existing geometric verification and final factor checks remain mandatory.

## Completed all-14 online trial, 2026-10-01

The fresh simultaneous 1× replay on 148 passed. All 14 robots shared the A01
frame at 430.46 seconds after playback release, before the last bag ended at
520.05 seconds. Final publication was at 909.19 seconds, including 389.14 seconds
of post-playback settling; the final accuracy is therefore a settled online-run
result, not an accuracy measurement at the instant playback ended.

| Measurement | Immediate freeze | Mature then freeze |
| --- | ---: | ---: |
| Fixed circles | 358 | 366 |
| Largest connected component | 4 robots | 14 robots |
| Final components | 11 | 1 |
| PCM-retained loops | 4 | 111 |
| Empty layered descriptors | 197 | 74 |
| Final observed point IDs represented in published evidence | 34.7% | 91.5% |
| Total remote application traffic | 128.32 MB | 546.77 MB |
| End-to-end wall time | 902.3 s | 913.2 s |
| Peak simultaneous process-tree RSS | 35.99 GiB | 42.34 GiB |
| All-14 joint ATE | Not defined: disconnected | 0.933 m |

The joint ATE uses 45,397 associated poses and one shared rigid alignment across
all 14 robots. Individually aligned ATEs are reported separately. The runs use
independent fresh odometry captures and are not an exactly frozen-input ablation.
Application traffic excludes DDS/IP overhead, discovery, sensor transport,
visualization and downloads. RSS may count shared pages more than once; it is
the simultaneous total for all 14 robots, not a per-robot memory requirement.

The 366 publications comprised 188 stable releases, 129 deadline releases and
49 end-of-stream tails. Median footprint-to-publication sensor age was 110.47 s;
the maximum was 129.60 s because deadlines are checked at native observations.
The 30-second minimum is not a promise of 30-second publication latency.

Of 120 candidates reaching geometric verification, 111 were accepted and all
111 survived PCM. 110 live GPU registration pairs were admitted and passed final
support checks. The remaining pair (G02/26–G04/4) failed the unchanged initial
overlap gate; its companion pose constraint remained. Persistent CBS completed
60–62 iterations per robot, randomly selecting pose or anchor exchange each iteration. No odometry
diverged; the largest gap between successful LiDAR updates was 0.472 s.

Verification retained 38 passing tests and three native-gated skips, independent
seal-time membership/frame/causality audits for every published submap, matching
runtime source hashes, and successful validation of all 14 Rerun recordings.
The exported gravity-aligned PLY contains 13,353,004 native representatives with
robot colors (280,413,486 bytes). Its SHA-256 is
`57bcbda19ee040f4db5de07e833d15c332467a4a6918ef452f087065e0931f27`.

The [inspection report](index.html)
contains the map, publication timeline, actual BEVs, trajectories, measurements,
and audit records. Bulk captures remain under
`/data3/mikexyl/swarm_s3e_ws/src/.ros2/circle-mature-online148-20261001` on 148;
the report and global PLY are also available locally. Previous runs are preserved.
