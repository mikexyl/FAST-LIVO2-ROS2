# Updated upstream odometry and accumulated-area MapClosures

This integration uses EllipseLIO upstream `6506f46f1947b4ef86cfba402f11f10a6ef520ee`.
Its registration, tensor voting, incremental map insertion, correspondence thresholds,
filter arithmetic and upstream compiler flags are retained. The `dev/upstream-ellipselio-area-mapclosures`
native branch adds reliable sensor input, exact scan diagnostics, and export-only area snapshots.
It explicitly rejects enabled odometry submapping or area-cropped odometry.

The matcher always uses the upstream persistent map. After matching and inserting a corrected
scan, the exporter selects all stored native representatives within an 80 m horizontal disk
around the corrected IMU position in the estimated gravity plane. Point age and height do not
restrict membership. Valid fitted ellipsoids share those point IDs. Tensors retain their native
persistent-map neighborhood support; export does not refit or change them. These representatives
are processed map geometry, not full-resolution raw LiDAR.

A snapshot is emitted after 20 m of horizontal movement or 10 seconds, plus a nonduplicate
shutdown tail. The trigger does not restrict membership to recent scans. Payloads use the
snapshot IMU frame and schema 5; member point/scan IDs, hashes, anchor times, gravity, and
availability times retain the existing accumulated-area endpoint contract. The bounded writer
has capacity two and fails on export errors. The separate research deskew exporter is unavailable
on this dedicated upstream branch; the original branch and its assertion remain intact.

Run `run_trial.py --area-maps --persistent-odometry` with the isolated mapper and library.
Unlike `--area-maps` alone, this explicitly disables the temporal-odometry default.
`mapping.area_maps.odometry` must be false. Diagnostics distinguish native core processing
from snapshot time and report their sum. Correspondence age is not measured by this upstream
adapter and is encoded as null, rather than fabricating a value.

`upstream_area_batch.py` prepares and runs fresh Alpha/Bob/Carol captures on Laboratory 4
(S3Ev1), Campus Road 1 (S3Ev1), and Campus Road 2–3 (S3Ev2). Its six frontend workers use
distinct ROS domains and 1× playback; the dedicated 148 container has no CPU, memory or GPU
quotas. Concurrency is an execution choice, not a user resource limit. No previous capture is reused.

Validated area snapshots feed the existing gravity-aligned ellipsoid-surface projection,
MapClosures verification, distributed isolation, PCM, and CBS registration factors using
`area_rollout.yaml`. No retrieval/verification/PCM/CBS thresholds are tuned. Same-robot endpoints
retain 30-second exclusion and reject reuse of any persistent point IDs. Ground truth is read
only for evaluation with evo 1.36.5, 50 ms association, SE(3) alignment, and scale fixed to one.
Reports distinguish independently aligned per-robot CBS ATE from shared-component CBS ATE.

The native tests check region boundaries, arbitrary heights, old-point return visits, anchor
transforms, immutable packets, unchanged filter state/covariance, and invalid configurations.
Python tests cover area evidence, causal endpoints, reversed loops, and native distributed
PCM/CBS. A source comparison records unchanged upstream estimator function bodies.
