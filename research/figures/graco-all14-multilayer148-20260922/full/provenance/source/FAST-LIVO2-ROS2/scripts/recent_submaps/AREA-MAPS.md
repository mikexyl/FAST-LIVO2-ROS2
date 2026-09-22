# Accumulated area maps for MapClosures

Enable `mapping.area_maps.enabled` to create MapClosures snapshots by spatial
membership in a retained map. By default this preserves the odometry map policy.
The [profile](area_maps.yaml) uses an **80 m horizontal radius around the current
drone position**, with unbounded height and no scan-age limit.

With recent-map odometry, every corrected scan updates a separate persistent native map, with its own
octree, tensor neighborhoods, filters and adaptive statistics. It receives the
same corrected world points once, after odometry has matched the scan. Odometry
does not query this archive in that mode. Points outside the current snapshot remain stored and
become available again on return visits. No area-coverage threshold or sequence
of member scans determines inclusion.

At a snapshot, every stored native map representative whose center lies inside
the horizontal disk is exported. These are the points retained by EllipseLIO's
normal map insertion/resolution rules, not every raw LiDAR return. This path adds
no further age, height, voxel or point-count filter to the saved point selection.
Valid native ellipsoids centered on those points are exported with them. Their
tensors use persistent-map neighborhoods, including neighbors across the crop
boundary; the rendered ellipsoid surfaces are clipped to the horizontal disk.

Snapshots are emitted after 20 m horizontal motion or 10 seconds, and once on
shutdown if new scans have arrived. **Those are publication triggers, not scan
windows:** a snapshot can include points observed at startup. Its pose is the
current corrected IMU pose, independently of its oldest/newest selected point.
All geometry is expressed in that snapshot's IMU frame. Shutdown snapshots are
complete area selections; snapshots without fitted ellipsoids are retained for
inspection but excluded from retrieval.

Schema 5 saves stable persistent point IDs, originating scan IDs, selected
ellipsoid point IDs, the region, and a causal availability timestamp. Validation
checks exact spatial membership, anchor transforms, point identities, and that
previously exported points still appear whenever they lie inside a later crop.
The native test also checks completeness against the entire stored map, boundary
inclusion, arbitrary height, leave/return behavior, and separation from odometry.

Use [area_rollout.yaml](area_rollout.yaml) for descriptor preparation and the
distributed backend. Prepared evidence retains every saved map point. Verification
uses fixed 0.4 m voxels; its normal geometric thresholds are unchanged. The old
80 m **spherical** range filters are disabled throughout this path, including
optional CBS registration-factor preparation. The existing separate CBS factor
point budget and voxel settings remain. Sampling is still used for ellipsoid
surfaces, registration, and visualization; it does not define snapshot membership.
The CUDA sampler encloses all selected ellipsoids before horizontal clipping;
large vertical extents use its CPU equivalent instead of silently discarding points.

Retrieval uses snapshot availability times. Graph vertices use snapshot pose
times. Same-robot candidates retain the 30-second exclusion and are additionally
rejected if their crops reuse any persistent map-point IDs. Reused stored geometry
cannot serve as independent loop evidence. Inter-robot retrieval is unaffected.
The rollout profile permits two MapClosures verification candidates per query
and sets `dpgo.pcm.minimum_clique_size: 1`. A geometrically verified singleton
can therefore connect two robots; PCM labels it `singleton_unchecked` because
there is no second loop for a pairwise consistency test. Candidate sets with
multiple loops still undergo consistency checks and clique selection, with
single-vertex cliques permitted. Earlier saved experiments retain their original
minimum clique size and verification allowance.
The direct global-area memory grows with the mapped environment. The native
octree's existing capacity is checked explicitly; the implementation fails rather
than silently evicting old geometry when that capacity is exceeded. Snapshot
publication uses a bounded writer with backpressure and immutable payloads.

Example, using an isolated build and a calibrated mapping config:

```bash
python scripts/recent_submaps/run_trial.py trial --area-maps \
  --robot aerial05 --bag /path/to/aerial05 --mapping-config /path/to/config.yaml \
  --mapper-executable /path/to/ellipselio_mapping_mt \
  --mapping-library /path/to/libellipselio_mapping.so
python -m s3e_pipeline.recent_submaps \
  --source /path/to/trial/frontend/area_maps --output /path/to/prepared \
  --config scripts/recent_submaps/area_rollout.yaml
```

`--area-maps` keeps the mapping config's submap strategy (temporal by default).
An explicit `--submap-strategy` selects the independent odometry policy. Native
odometry exports remain in `frontend/submaps`; MapClosures area exports are in
`frontend/area_maps`. Live Rerun displays them under `area_snapshots`.

This accumulates only observations available at that time, in the robot's odometry
frame. It cannot fill unseen space, and it inherits any accumulated odometry drift.
It is not an offline crop of a future completed trajectory or a GT-corrected map.

## Vertical initialization for aerial loop verification

Gravity-horizontal BEVs estimate XY translation and yaw. Enable
`backend.mapclosures.vertical_initialization.enabled: true` to estimate the
missing vertical translation from the two candidate geometry payloads before
GICP. This option defaults to false and requires descriptors explicitly marked
as gravity-horizontal. It uses neither nominal flight heights nor GNSS/GT.

The initializer votes for height differences between horizontal neighbors and
scores the strongest height hypotheses by symmetric coarse 3D overlap. Defaults
are 0.8 m voxels, eight XY neighbors within 1 m, 0.5 m height bins, five modal
hypotheses separated by more than 2 m, and a 1.5 m coarse overlap distance. The
unchanged seed is also considered. These values govern initialization only;
evidence preprocessing, GICP acceptance, PCM and CBS retain their settings.
Missing geometry or horizontal support leaves the seed unchanged for normal
verification. The verifier logs the original BEV pose, chosen height, hypothesis
scores, runtime and actual GICP initial pose. No extra file access is needed by
the isolated worker, and raw odometry and descriptors remain unchanged.

## Experimental accumulated-area odometry

Pass `--area-odometry` to `run_trial.py` to enable the same spatial membership
policy for LiDAR odometry. This implies area-map export and sets
`mapping.area_maps.odometry: true` with `mapping.submaps.enabled: false`.
The native node rejects contradictory settings. The option defaults to false.

In this mode one persistent native map supplies both odometry and MapClosures.
The odometry query is an 80 m horizontal disk (the configured area radius), with
unbounded height and no point-age cutoff. Its center and gravity plane are frozen
at the IMU-propagated pose before each iterated LiDAR update. The octree selects
the nearest candidate *inside* that area and within the existing correspondence
search radius; a closer point outside the area cannot hide an eligible point inside.
All existing tensor, residual and minimum-travel eligibility rules still apply.

There are no temporal/coverage handovers, successor buffers, or historical fallback.
Outside points stay stored for future return visits. The current scan enters the
map only after matching. Filter state and covariance are preserved. Native tensors
retain their persistent-map neighborhoods, including support across the area edge.
The existing native capacity check fails explicitly rather than evicting history.

MapClosures snapshots use the corrected pose after insertion, with their original
publication cadence. Thus the descriptor crop and the frozen odometry query can
differ slightly by the current correction; both use the same persistent geometry
and spatial policy. Per-scan diagnostics record the frozen query center/up/radius
and `odometry_map_source: accumulated_area`; `active_points` counts the whole
persistent map, not only points in the query area.
