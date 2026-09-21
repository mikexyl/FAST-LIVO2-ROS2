# Motion and overlap submaps

Implemented 2026-09-19 as an opt-in `mapping.submaps.strategy: motion_overlap`.
Disabled submapping and the default `temporal` strategy retain their previous
behavior. Historical configurations and captures remain unchanged.

`motion_submaps.yaml` supplies the first experimental configuration, not tuned
results. A keyframe is selected after 1 m translation, 10 degrees rotation, or
occupied-voxel overlap below 0.6 relative to the previous selected keyframe.
Overlap measures current occupied cells supported by the previous keyframe's
cells, using a 0.5 m world grid and a one-cell neighbourhood. Only native
processed points within 80 m contribute to this overlap diagnostic. This is
our explicit geometric approximation, not GLIM's implementation or a claim
of equivalent overlap values. At least 50 occupied cells are required and,
after the first keyframe, the LiDAR update must have succeeded.

All native processed scans are still inserted after matching. Selected
keyframes control map boundaries; they do not replace the native member scans
used to fit tensors or exported as registration evidence.

A successor begins at half the nominal map budget: 10 m maximum displacement
from the map's first member pose, eight selected keyframes, or 15 seconds.
The active map requests retirement at 20 m maximum displacement or 15 selected
keyframes. These decisions use corrected, already-inserted scans and are
applied before matching the next scan. The active map remains fixed throughout
each iterated LiDAR update. Both maps own independent points, octrees, tensors,
neighbourhood statistics and scan membership; no tensors are inherited.

A normal handover also requires the successor to support the incoming scan
under its IMU-predicted pose: at least 50 sampled points and 20% of sampled
points must find a finite fitted ellipsoid within the existing 1 m maximum
search radius. The diagnostic samples the incoming native processed scan
deterministically, checking roughly 2,000 points within 80 m. It is a readiness
check, not a change to the estimator's correspondence thresholds or an
observability guarantee. Unsupported handovers wait while the active map is
fresh.

Thirty seconds is a hard **stale-geometry guard**, not the nominal splitting
period. At expiration, a ready fresh successor can take over. Otherwise the
expired maps are archived/cleared and `stale_recovery` is recorded; the current
scan seeds a fresh map without a LiDAR update, while IMU state and covariance
are retained. A timestamp gap cannot cause expired tensors to be queried.
Stationary revisits therefore retain useful geometry longer than the temporal
baseline, but do not retain it indefinitely. This explicitly preserves the
original stale-geometry concern rather than assuming spatial cropping solves it.

Schema version 2 adds the strategy, exact selected-keyframe scan IDs, observed
extent, age guard and finish reason. Sensor membership intervals remain
half-open; anchors are the last member scan's corrected IMU pose, and delivery
uses the separate availability timestamp. Completed maps need at least three
selected keyframes and 50 fitted ellipsoids to enter retrieval. Stationary or
weak completed maps and shutdown tails remain available for inspection.
Descriptors and registration evidence share exactly the exported membership.

Native diagnostics/Rerun include selected-keyframe count, geometric overlap,
extent, successor readiness and explicit handover/wait/recovery events.

The native policy tests cover speed-independent boundaries below the age
guard, stationary retention, translation/rotation/overlap selection, unsupported
handover deferral, stale recovery, no self-matching, state/covariance retention,
map-local indices and two-buffer reuse. Existing temporal and numerical-guard
tests remain enabled. Writer and distributed endpoint tests cover both schema
versions and immutable membership.

## Workstation 148 experiment

The fresh container is `ellipselio-motion-submaps148`, using all GPUs and no
CPU quota, cpuset, RAM limit or swap limit. It mounts the host directory
`.ros2/motion-submaps-20260919` at the legacy in-container build/output path
`/workspace/.ros2/recent-submaps`, so existing runner paths remain valid.
The old temporal directory is mounted read-only at `/history/recent-submaps`.
The native image, mapping source overlays, build, install and output roots are
separate from all historical experiment artifacts.

After the short frontend smoke checks pass, prepare fresh captures for the
same five authorized groups (18 robots), then run six simultaneous frontends
with four executor threads each and distinct ROS domains. This uses all 24
logical CPUs for frontend concurrency; thread counts are workload scheduling,
not enforced host/container resource quotas. Backends retain their existing
MapClosures/PCM/CBS/GICP settings and run after frontend validation.

```bash
bash FAST-LIVO2-ROS2/scripts/recent_submaps/motion_env.sh \
  .ros2/research-venv/bin/python FAST-LIVO2-ROS2/scripts/recent_submaps/full_batch.py \
  prepare --strategy motion_overlap --work /workspace/.ros2/recent-submaps/full-five-groups
bash FAST-LIVO2-ROS2/scripts/recent_submaps/motion_env.sh \
  .ros2/research-venv/bin/python FAST-LIVO2-ROS2/scripts/recent_submaps/full_batch.py \
  run --workers 6 --work /workspace/.ros2/recent-submaps/full-five-groups
```

The motion strategy always creates new captures; temporal captures cannot be
silently reused as motion-strategy results. Ground truth remains evaluation-only.
Compare against the retained temporal batch with the changed concurrency and
single-run reproducibility limitation explicitly stated. Do not attribute a
CBS common-frame failure to submapping without separate evidence.
