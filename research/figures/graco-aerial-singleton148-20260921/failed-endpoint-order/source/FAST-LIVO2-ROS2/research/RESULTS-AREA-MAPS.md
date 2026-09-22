# Accumulated area maps for MapClosures

2026-09-21. Implemented and locally validated on two concurrent 120-second,
1× GRACO aerial-05/08 replays. MapClosures now has an opt-in source containing
**every stored native map point within 80 m horizontally of the drone**, at any
height and from any earlier scan. Odometry retains its separate recent map.
This is implementation validation, not a completed four-flight accuracy trial.

[Open the 26-image BEV gallery](figures/area-maps/implementation/smoke/bevs/index.html)
or the [overview](figures/area-maps/implementation/smoke/bevs/overview.png).
The images reproduce the cached MapClosures ORB features exactly and use
gravity-horizontal projection. See the [configuration and implementation policy](../scripts/recent_submaps/AREA-MAPS.md).

## Spatial membership

Each corrected scan updates a separate persistent native map after odometry
matching. Snapshot membership depends only on the horizontal region, with no
scan window, point-age cutoff or vertical/spherical range cutoff. Points outside
the current region remain stored and can reappear on return visits. The native
map's existing resolution rules still apply: these are stored map representatives,
not every raw LiDAR return. No additional sampling is applied to snapshot points
or prepared evidence; ellipsoid rendering and registration retain their explicitly
configured sampling.

The 20 m movement / 10 s interval settings trigger publication only. Snapshot
poses belong to the latest processed scan, independently of the ages of included
points. Descriptors and verification evidence use that same immutable selection.
Same-robot retrieval retains the 30 s exclusion and rejects selections that reuse
stored point IDs. The first validation caught a shutdown timestamp referring to a
staged, unprocessed scan; this was fixed and covered by a native regression test
before the fresh captures reported here.

## Measured results

These captures keep the preceding coverage-based recent-map odometry settings,
calibration, IMU noise, reliable input and four-thread launcher. The new accumulated
map supplies MapClosures/BEVs only. No ground truth was supplied or used, and no
ATE is reported.

| Metric | Aerial 05 | Aerial 08 |
|---|---:|---:|
| Finite, chronological native poses | 1,106 | 1,118 |
| Maximum speed | 6.09 m/s | 3.60 m/s |
| Maximum successful LiDAR update gap | 0.448 s | **3.320 s** |
| Median / maximum native scan processing | 34.4 / 164.2 ms | 40.5 / 156.4 ms |
| Mapper peak RSS | 770.4 MiB | 733.3 MiB |
| Verified area snapshots | 13 | 13 |
| Points in final selected region | 244,613 | 201,490 |
| Points in final persistent map | 407,837 | 395,054 |
| Oldest point in final selection | 112.5 s | 118.7 s |
| Final selected points at least 10 s old | 80.3% | 73.4% |
| Final selected points at least 60 s old | 15.4% | 4.0% |
| Median BEV image aspect ratio | 1.214 | 1.209 |
| Median ORB features per image | 74 | 120 |

The **update gap is sensor time between successful LiDAR corrections**.
It is separate from wall-clock computation time. Aerial-08's largest gap runs
from 28.052 to 31.372 s after initialization (successful scan IDs 267 to 298).
The 30 intervening scans were processed, but each had zero eligible features and
no accepted LiDAR correction; IMU propagation continued. Their maximum processing
time was 118.0 ms. The reason those scans had no eligible features has not been
established by this area-map implementation test.

Native scan processing includes IMU deskew, the LiDAR update, map insertion,
area accumulation and snapshot enqueue/backpressure. It excludes asynchronous
writer completion and the later descriptor/backend processing, and is not
end-to-end sensor latency. Aerial-08 fails the existing 1 s update-gap gate,
so the backend below is diagnostic.

The distributed MapClosures → PCM → CBS run completed in **12.66 s**, with 26
vertices, zero geometric verification attempts, zero proposed/retained loops,
zero PCM rejections and two disconnected components. Consequently, this capture
does not demonstrate a successful real inter-robot loop or CBS correction.
The complete capture/preparation/backend/gallery workflow took 253.70 s.
The [machine-readable report](figures/area-maps/implementation/smoke/report/report.json)
and [measurement summary](figures/area-maps/implementation/smoke/implementation-summary.json)
retain the unrounded values.

## Verification and retained evidence

The isolated native build passed **3 CTest executables**. Python checks passed
**50 unit tests plus 27 integration/regression tests**, including eight isolated
retrieval/native PCM-CBS tests across temporal, spatial, coverage and area sources.
Coverage includes independent map state, repeated odometry handovers, all-height
selection, exact boundaries, leave/return retention, immutable exports, causal
timestamps, evidence membership, and distributed registration without a spherical
range cutoff. See [test results](figures/area-maps/implementation/verification-results.json).

All 26 real area snapshots passed spatial membership, identity, anchor and hash
checks. Previously exported points were retained whenever inside a later region.
All 26 BEV images reproduced their descriptor features. Both live Rerun recordings
and the derived [result recording](figures/area-maps/implementation/smoke/report/result.rrd)
passed verification. Native per-scan diagnostics provide the complete processing
and update-gap measurements; live ROS analytics may contain fewer messages.

The [retained artifact manifest](figures/area-maps/implementation/manifest.json)
indexes copied configs, frozen sources, logs, measurements, trajectories, maps,
gallery and recordings by SHA-256. Bulk native/prepared geometry remains in
`.ros2/area-maps-20260921/smoke` and is hashed in `external-geometry.json`.
The earlier failed anchor-audit attempt remains in
`.ros2/area-maps-20260921/smoke-before-anchor-fix`.

Persistent-map memory grows with observed environment size. Geometry inherits
odometry drift and cannot fill unseen space. This short validation establishes
the requested membership behavior; full-flight accuracy, connectivity and resource
growth remain unmeasured for the new area-map source.
