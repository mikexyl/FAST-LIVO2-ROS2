# Online GRACO demonstration

The demo runs fresh LiDAR/IMU bags concurrently at 1×. It uses the updated
persistent-map EllipseLIO frontend, accumulated-area snapshots, terrain-relative
multilayer ellipsoid BEVs, peer MapClosures retrieval and geometric verification,
distributed PCM, and distributed CBS with GICP registration factors.

`scripts/recent_submaps/online_graco_demo.py` coordinates the processes. It reads
an earlier experiment's **sensor paths and calibrated configurations only**;
odometry, maps, descriptors, loops and optimization results are generated anew.
Each frontend retains its own ROS domain. The retrieval peers use domain 180;
the independent native backend sessions use domain 181. A startup barrier
releases all bag players after every frontend, retrieval peer and recorder is ready.

## Live data and clocks

The native bounded snapshot writer closes each compressed payload before
flushing its index row. `online_prepare` consumes complete newline-terminated
rows while capture continues. It checks payload hashes and accumulated-map
membership, and publishes a prepared row only after its evidence and multilayer
descriptor are complete. Final native manifests are checked again at shutdown.

Sensor timestamps and anchor poses remain unchanged. Because the GRACO bags
represent different recording sessions, descriptor availability uses a common
Unix wall clock. The original native availability is saved as
`native_available_ns`. A query can use only descriptors already available at
its publication time. Same-robot exclusions continue to use sensor time and
shared point identities. Registration thresholds, evidence resolution, PCM
probability and singleton handling are unchanged.

`online_worker` stays alive as its own store grows. Robots exchange descriptors
and selected verification geometry directly over DDS. Each worker and verifier
has the existing Landlock restriction to its own evidence directories and
explicit runtime dependencies. The recorder can inspect all robot outputs for
display; it does not perform retrieval or estimation.

## Periodic online backend

The current native PCM and registration exchange require immutable input seals.
The demo therefore runs **periodic distributed PCM/CBS sessions over causal
prefixes**, while frontends and retrieval continue independently. It does not
claim to support insertion into a running sealed native PCM session.

Each revision captures only rows already published and loop proposals agreed
by both incident endpoints. It runs the unchanged native PCM, GICP factor
exchange and 100-iteration CBS settling budget. It publishes a correction only
after the session completes. The next session starts after a 30-second interval,
or immediately for the final drained input. Previous revisions are retained;
late results never change the earlier part of the recording. These sessions
restart the native optimizer; they are not an incremental warm-start algorithm.

## Recording and video

`online_record` reads live post-LiDAR TF capture and descriptor publications.
It writes `demo/online.rrd` during execution and sends RGB frames to a concurrent
FFmpeg encoder for `demo/online.mp4`. The video is 1920×1080 at 5 fps. Repeated
frames preserve elapsed wall time when rendering falls behind. Arrival journals
record the actual publication, receipt and display times.

Disconnected components have separate Rerun views and separate video panels.
Every robot appears in exactly one group; the video allocates panel area by
group size. Panels merge when a published CBS revision connects their robots.
Once all robots connect, only the full-width global group view remains. On a
CBS revision, old component entities are cleared and the new shared map is
displayed. The gravity-level display uses estimated IMU
gravity, never ground truth. Between optimized anchors, the live display uses
the latest preceding available correction; future poses use the last published
correction. Display points are thinned to approximately 1 m per new point chunk;
registration retains its original evidence resolution.

The original 22 September recording used a largest-group overview with individual
trajectory side panels. `s3e_pipeline.online_revideo` produces the revised group
layout from that capture's arrival journals and original frame schedule. It checks
all 504 display-point memberships and applies each backend revision at its
recorded display event. These MP4s are labelled as re-rendered online arrivals;
the original live MP4/Rerun files remain unchanged. No estimation is rerun and
the final solution is never applied to earlier frames.

This is an online replay demonstration, not a claim that the recordings were
collected simultaneously or that every backend revision has a real-time
deadline. Measure and report acquisition coverage, odometry update gaps,
descriptor/verification delays and backend revision latency separately.

## Validation and retention

`research/tests/test_online.py` covers partial index publication, stream identity,
monotonic availability, causal backend prefixes, endpoint agreement and display
correction selection. Existing geometry and registration tests still apply.
`s3e_pipeline.online_audit` checks native completion, finite chronological poses,
successful-update gaps, immutable membership, candidate availability, optimizer
cutoffs, display timestamps and complete pose capture. A correction must be
displayed before the first bag finishes to pass its online check.

Verify the combined and per-robot recordings with `rerun rrd verify`, check MP4
frame count/duration, and decode the video to completion. Keep configuration and
source hashes, input hashes, native trajectories, arrival journals, all backend
revisions, numeric audits, Rerun recordings and video. Ground truth may be read
only after estimation for the separate evo 1.36.5 report (50 ms association,
rigid alignment, no scale fitting). The live recording is never rebuilt from
that evaluated final solution.

Workstation 148 has no CPU, memory or GPU quota. Thread counts and preparation
concurrency are throughput choices, not resource-usage limits.
