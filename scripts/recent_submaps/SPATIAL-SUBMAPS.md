# Spatial submaps for EllipseLIO

`mapping.submaps.strategy: spatial` is an experimental alternative to the
unchanged default `temporal` strategy. It is intended to accumulate larger
spatial coverage for ellipsoid-BEV MapClosures. It uses the same native member
scans for odometry tensors, descriptors and geometric verification. It does
not restore the retired keyframe-count/voxel-overlap scheduler.

The initial BEV-oriented profile is [spatial_submaps.yaml](spatial_submaps.yaml):

```yaml
mapping:
  submaps:
    enabled: true
    strategy: spatial
    spatial:
      radius_m: 40.0
      overlap_m: 20.0
      distance_metric: horizontal
      max_age_s: 120.0
      min_support_points: 50
      min_support_ratio: 0.2
```

Each map starts at its first corrected member pose. Its extent is the maximum
displacement from that pose, not accumulated path length. `horizontal` removes
the component parallel to gravity, using the first member's estimated gravity
as a fixed reference plane. Vertical-only flight and in-place rotation do not
reach the spatial threshold. `3d` uses Euclidean displacement instead; this is
the native parameter default when no explicit profile is supplied.

A successor begins after the active extent reaches `radius_m - overlap_m`
(20 m in this profile). A normal handover is requested at `radius_m` (40 m).
Both decisions use corrected, already-inserted poses and take effect before the
next scan's LiDAR update. Consequently thresholds can overshoot by a scan;
`overlap_m` is a nominal displacement band, not a guaranteed intersection area.
Straight paths and different speeds can produce different durations but the
same spatial handover locations. Backtracking within the start region does
not exhaust a path-length or keyframe-count budget.

Before switching, the successor must support at least 50 sampled incoming
points and 20% of the eligible sample, within the existing 1 m maximum search
distance to finite fitted ellipsoids. At most 2,000 native processed points
within 80 m are sampled. This readiness check is not an observability or accuracy
guarantee and does not change the odometry correspondence thresholds. An
unsupported spatial handover waits while the active map is still fresh.

Two explicit guards bound retention: maximum age (120 s) and the existing
10-million-point snapshot payload capacity. A successor is also started at
half either guard to allow a stationary run to build a replacement. At a hard
guard, a ready fresh successor takes over; otherwise a named recovery archives
the old map and clears geometry before the incoming scan. IMU state, covariance
and trajectory survive. That scan seeds a fresh map and has no LiDAR update.
No archived map can return to odometry. Guards and waiting events are recorded,
so a run dominated by age/capacity closures is not reported as distance-driven.

Each buffer independently owns its points, octree, tensors, filters and
neighbourhood statistics. Every processed scan is transformed once after
matching and inserted into the applicable maps. No old tensors are copied.
The matching map is fixed throughout the iterated LiDAR update.

Spatial snapshots use schema **3** (schema 2 belongs to the retired experiment).
They include the spatial origin/up direction, metric, observed extent, radius,
overlap, age guard and finish reason. The membership interval is half-open and
ends at the first excluded scan; shutdown tails end one nanosecond after their
last member. Complete maps are retrievable, while tails are inspection-only.
The gravity-aware descriptor pipeline keeps the last-member IMU anchor and the
separate availability timestamp. Same-robot time/overlap exclusions, distributed
isolation, registration, PCM and CBS settings are unchanged.

Rerun and native update logs expose active/successor extents, map age, support
counts/ratio, and handover/wait/recovery events. The artifact validator checks
actual variable-duration membership, spatial extent, anchors, payload hashes
and correspondence age against the active map's own start time.

After building and sourcing the updated native library/interfaces, use a new
output name and the sensor's calibrated config:

```bash
python3 FAST-LIVO2-ROS2/scripts/recent_submaps/run_trial.py spatial-trial \
  --robot aerial05 --bag SENSOR_ONLY_BAG --mapping-config CALIBRATED_MAPPING.yaml \
  --submap-strategy spatial --duration 120 --output-root NEW_OUTPUT_DIRECTORY \
  --mapper-executable NEW_BUILD/launcher/ellipselio_mapping_mt \
  --mapping-library NEW_BUILD/install/ellipselio/lib/libellipselio_mapping.so
```

`--submap-strategy temporal` selects the existing temporal scheduler instead.
Omitting the flag preserves the supplied config. Native submapping remains
disabled by default. Old temporal configurations and saved experiments are
not rewritten, and a spatial experiment requires fresh captures.

These settings are starting values, not tuned results. Spatial retention may
improve coverage but cannot add unseen surfaces or guarantee square BEVs.
The policy also changes odometry's map age and geometry, so it needs a full
accuracy comparison before replacing the temporal baseline. A short replay
validates operation, not accuracy or general robustness.

The full aerial-05–08 run on 148 is recorded in
[the experiment report](../../research/RESULTS-GRACO-AERIAL-SPATIAL148.md).
The 40 m radius is a handover trigger, not a strict limit on successor extent:
a growing successor can already exceed that radius before it becomes active.
Aerial-06 demonstrated this with a 60.92 m completed extent.
