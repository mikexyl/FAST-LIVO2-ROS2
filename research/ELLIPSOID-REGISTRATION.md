# Ellipsoid-only loop verification (experimental)

MapClosures still retrieves candidate submaps from multilayer ellipsoid BEV ORB
features. Set `backend.registration.method: ellipsoid` to verify those candidates
with the **same native fitted ellipsoid records**, rather than the processed
point cloud. Point-GICP remains the default if `method` is omitted.

For an accumulated-area run, use the existing area-map profile and add:

```yaml
backend:
  registration:
    method: ellipsoid
    ellipsoid_voxel_m: 0.4
    ellipsoid_max_count: 50000
dpgo:
  registration_factors:
    enabled: false
```

Keep the remaining registration thresholds and MapClosures settings from the
chosen profile. The `pgo.registration_factors` setting must likewise be disabled
when running the PGO variant. The existing CBS/PGO registration factors use point
GICP and are deliberately rejected in ellipsoid mode; CBS still receives the
verified ellipsoid-derived SE(3) loop measurements and odometry factors.
Prepare fresh submap stores with this setting: point-mode stores do not contain
the fitted-ellipsoid verification payload.

EllipseLIO computes tensor eigenvectors and saliency values in
[`map_processing.cpp`](../../ellipselio/src/map_processing.cpp). During odometry
it projects each scan measurement toward the nearest map representative's
weighted plane, line and point geometry. The submap verifier applies the same
projection weights recovered from each exported ellipsoid's inverse axes, but
its measurements are **source ellipsoid centers**, not scan points. It iterates
nearest target primitive association, uses a source-frame pose Jacobian,
checks support in both directions, rejects poorly constrained solutions, and
returns a loop pose with a conservative information matrix. Vertical
initialization also uses ellipsoid centers in this mode.

The 50k cap is an evidence budget: it keeps one original fitted primitive per
occupied 0.4 m center voxel, then uniformly thins if needed. It does not refit
ellipsoids. In the initial 20k trial, 24/63 candidate verifications passed; 50k
passed 42/63. This cap followed inspection of those rejections and should not be
treated as independently tuned performance. The corrected 50k mixed GRACO
trio retained 42 loops and connected G05/G06/A07; joint ATE was 0.829 m,
compared with 0.812 m for point GICP on the same frozen BEVs and submaps. See
the [paired report](figures/graco-ellipsoid-registration148-20260923/REPORT.md).

Prepared snapshots still contain the legacy point evidence for compatibility,
but ellipsoid-mode workers transmit and verify only the fitted ellipsoids. The
loop route does not load those point arrays. The source area maps, point-mode
configuration, and descriptors are unchanged.
