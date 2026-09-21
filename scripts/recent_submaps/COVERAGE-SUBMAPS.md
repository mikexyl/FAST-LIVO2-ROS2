# Observed-coverage submaps

`mapping.submaps.strategy: coverage` uses the scene observed by LiDAR to decide
submap boundaries. Temporal remains the default; `spatial` preserves the earlier
drone-displacement policy for reproducibility.

The sparse horizontal grid is centered on the first processed scan's return
centroid, not the drone. Its gravity plane and axes stay fixed across both map
buffers, so matching cell IDs refer to the same world locations. Every corrected
native processed return contributes its occupied cell. Height along that plane's
normal does not increase coverage. Repeated observations of a cell do not add area;
no empty bounding-box area is counted. Cell area describes projected occupancy,
not physical surface area, visibility, or a guarantee of good descriptors.

The experimental [profile](coverage_submaps.yaml) uses 1 m cells:

| Decision | Conditions |
|---|---|
| Start successor | At least 2,000 m² occupied, including 1,000 m² gained since the first member scan |
| Normal handover | At least 4,000 m² occupied, including 2,000 m² gained since the first member scan |
| Required geometric overlap | At least 1,000 m² shared cells and intersection/max(active area, successor area) ≥0.25 |
| Odometry readiness | At least 50 supported sampled points and a 0.20 support ratio against successor ellipsoids |
| Secondary guards | 80 m horizontal drone displacement, 120 s age, or existing snapshot point capacity |

Both area and new-area conditions must hold. A stationary drone seeing new
surfaces can trigger a normal handover. A translating drone repeatedly seeing
the same surfaces cannot do so through coverage alone. Secondary guards start a
successor at half their bound. At a hard guard, an unsupported or insufficiently
overlapping successor causes explicit recovery, retaining the IMU/filter state
and clearing old geometry. Such events are labelled separately from coverage
handovers. Area can overshoot a threshold while awaiting overlap/support.

The active and successor maps still independently fit tensors from their own
member scans. Current scans match before insertion. Changes take effect before
the next LiDAR update, never during its iterations. These native maps supply both
odometry geometry and MapClosures artifacts; this is not a separate descriptor-only
aggregation layer.

Schema 4 adds occupied grid cells and their first-observation scan IDs to the
compressed payload, plus per-member point counts, grid frame, occupied/new/shared
areas, overlap ratio, successor ID and closure reason. The validator checks cells
against saved geometry, membership/anchor times, threshold satisfaction and the
causal shared-cell intersection. Tiny float32 cell-boundary ambiguity is allowed.
Partial shutdown/recovery successors remain excluded from retrieval. Actual
per-anchor filter gravity still determines the BEV projection.

The coverage footprint counts all native processed member returns **before**
backend filtering. The existing backend 80 m slant-range filter can remove part
of that footprint; it is separate from the coverage thresholds. The grid does
not crop a sphere around the flying drone. A fixed initial gravity plane provides
consistent cells; subsequent gravity corrections can slightly change the BEV
projection relative to that scheduling plane.

Use `run_trial.py --submap-strategy coverage` with a calibrated mapping config and
the newly built launcher/library. For preparation and backend processing, use
[coverage_rollout.yaml](coverage_rollout.yaml). It explicitly keeps evidence and
registration at **0.4 m voxel resolution**, without adaptive point-count
coarsening. The RMSE/overlap/observability acceptance thresholds and PCM/CBS
settings remain unchanged. Oversized geometry is not silently made coarser;
message size and processing cost therefore grow with observed geometry. Evidence
metadata records requested/effective resolution and point counts through IPC.
A fixed-resolution verifier rejects missing or overly coarse evidence provenance.
Coverage preparation refuses the legacy adaptive configuration.

After installing a dedicated build on 148, the four-flight runner accepts:

```bash
python scripts/recent_submaps/graco_spatial148.py run --strategy coverage
```

Its default root is `.ros2/graco-aerial-coverage148-20260920`; the command refuses
to overwrite an existing result. Frozen historical experiments are unaffected.
These are initial experimental area thresholds, not tuned accuracy results.
