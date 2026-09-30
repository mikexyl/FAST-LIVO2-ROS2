# Incremental circle placement prototype

Branch: `dev/incremental-circle-submaps`. Published base:
`1eec4b98c51267398d91d8ad953cdbc472951262` on
`origin/dev/multi-robot-lidar-slam-submaps`.

This standalone prototype creates fixed spatial footprints from a growing map.
It does not modify odometry, snapshot export, MapClosures, or CBS. A published
circle is an immutable footprint, not yet a frozen geometry payload. Deciding
when to refresh or freeze its geometry belongs to later pipeline integration.

## Interface and fixed coordinates

```python
from s3e_pipeline.circle_cover import CircleCoverConfig, CircleCoverPlanner

planner = CircleCoverPlanner(CircleCoverConfig(), T_horizontal_map=fixed_transform)
result = planner.update(new_points_in_original_map_frame, update_id=0)
print(result.new_circles, result.diagnostics)
planner.save("state.json")
planner = CircleCoverPlanner.load("state.json")
```

The constructor receives one proper `T_horizontal_map` rigid transform. It
rotates map points into a gravity-horizontal frame. Input observations must
always use that original map frame; poses are not placement inputs. An optional
`T_horizontal_map=` keyword on `update` detects an attempted frame change.
The returned transform and cell arrays are copies, and circle objects and their
center tuples are immutable. Update IDs are strictly increasing nonnegative
integers. Empty updates are valid. Non-finite inputs fail before state mutation.

Grid origin is fixed at `(0, 0)` by default and retained in configuration/state.
Indices are `floor((horizontal_xy - origin) / cell_size)` and cell centers are
`origin + (index + 0.5) * cell_size`. Each occupied cell has weight one. Point
count, observation order, revisit frequency and vertical height do not change
its weight. A cell is covered when its center is inside at least one circle.
Actual point coverage is a separate diagnostic; 95% cell coverage does not
guarantee 95% point coverage.

Default configuration is `configs/circle_cover.yaml`: radius 80 m, grid 1 m,
coverage 95%, desired greatest overlap 30% ±10 percentage points, farthest-point
candidate spacing R/4, HiGHS time limit 10 seconds per triggered update.

## Placement and priority

1. Insert distinct new occupied cells. If the current circles cover at least
   95%, publish no new circles.
2. On uncovered cell centers, generate a deterministic farthest-point net,
   seeded by the lexicographically first cell. Stop when every uncovered cell
   is within R/4 of a candidate. This also guarantees a feasible radius-R cover.
3. Build a sparse candidate-to-uncovered-cell incidence matrix. Greedily select
   maximum marginal coverage until enough uncovered cells are covered; ties use
   overlap deviation and then deterministic candidate order.
4. Run HiGHS mixed-integer optimization. Binary candidate variables select
   circles; continuous cell indicators can be positive only when a selected
   circle covers that cell. Require sufficient newly covered cells, cap the
   count by the greedy count, and minimize the selected count. The 10-second
   limit applies to the solver, not the entire update. Accept any incumbent only
   after validating its integer-selected cover exactly. Otherwise use the
   feasible greedy cover. Retain status, message, primal objective, dual bound,
   gap, fallback and solver wall time.
5. Assign covered uncovered cells to their nearest new circle. Protect a
   sufficient number, selecting those with the largest margin inside the
   circles. Refine new centers with SLSQP under constraints keeping those
   assigned cells inside their circles. Existing centers stay fixed. Minimize
   squared deviation of each new circle's greatest disk overlap from 30%.
   Isolated circles carry no overlap requirement. Accept a successful proposed
   refinement only when coverage remains feasible and its cost does not rise;
   otherwise keep the original feasible centers.
6. Remove redundant new circles, then validate the complete cover and publish
   all additions atomically, with monotonic IDs and their creation update.

For equal-radius disks at separation d ≤ 2R, overlap is
`[2 acos(d/(2R)) - 2(d/(2R)) sqrt(1-(d/(2R))²)] / π`.
It is zero for tangent or separated disks. A 30% overlap corresponds to about
93.622 m separation at R=80 m. Diagnostics retain all intersecting pairs,
greatest/nearest overlap, shared occupied cells divided by the smaller pair
membership, and total cell memberships per uniquely covered cell.

Coverage takes priority over count, and count takes priority over overlap.
Refinement protects a sufficient fixed assignment, so it can stop short of
the desired overlap band even when other continuous solutions exist. We do
not tune away or hide those deviations. A count is optimal only within the
candidate set for that update when HiGHS proves it. Published past placements,
unknown future growth and subsequent continuous refinement preclude a claim
of a globally minimum circle count.

## State and verification

Saved JSON state has a schema version and SHA-256 checksum over canonical
state contents. It includes configuration, grid origin, frame, sorted occupied
cells, circles, IDs, creation updates and last update ID. Restore validates the
checksum, frame, cell uniqueness/order, ID sequence and coverage. Saving uses
an atomic temporary-file replacement.

Run:

```sh
PYTHONPATH=research python -m pytest \
  research/tests/test_circle_cover.py research/tests/test_circle_cover_replay.py
```

Tests cover expanding disks, rectangles, narrow corridors, branches,
disconnected islands, sparse observations, revisits, empty updates, height and
density/order invariance, fixed centers, restart continuation, analytic overlap,
candidate net bounds, exhaustive small candidate-set optima, accepted/rejected
timeout incumbents, refinement failure, transaction failure and 0.5/1/2 m grids.
Replay tests validate causal availability, original-anchor transforms, stable
point identity and matching native geometry/ID membership.

## GRACO A05 comparison

Execute on workstation 148 using its existing NumPy/SciPy environment, without
CPU/RAM/container quotas. Bulk payloads and full occupancy states remain there.

```sh
PYTHONPATH=research python -m s3e_pipeline.circle_cover_replay \
  --store /path/to/prepared-aerial05/store --output /new/output/grid-1m
```

Repeat in fresh processes with `--cell-size 0.5 --no-gallery` and
`--cell-size 2 --no-gallery`. The output directory must be new.

The replay is the growing union of **34 retained prepared area snapshots**,
not every original scan and not full-resolution raw geometry. Each payload's
native processed cloud is transformed by its original odometry anchor pose.
Stable geometry IDs recover membership; repeated points are checked within
float32 export precision and frozen at their first available observation to
prevent reprojection rounding from inventing occupied cells. Gravity is fixed
from the first available snapshot. Ground truth is not used.

The comparator publishes the original trajectory-centered circles at the same
availability times and radius, projected into that same fixed horizontal frame.
Its coverage and membership metrics use exactly the same growing map prefixes.
The original snapshots used per-anchor gravity; this comparator intentionally
standardizes the footprint orientation to the fixed first-snapshot plane.

Inspection outputs contain all update/solver/overlap diagnostics, immutable
placement history, configuration, source/input hashes, paired prefix images and
a timeline. Full occupied-cell states stay on 148. Planning wall time includes
grid insertion, coverage, selection, refinement and overlap diagnostics. Input
loading, point-coverage audits, baseline audits and state I/O are outside this
per-update number. Replay memory measures actual simultaneous process-tree RSS
every 20 ms and the process high-water RSS before plotting. It includes input
union, planner, solver, history and state validation; it is not a planner-only
memory measurement. No historical process peaks are summed.

The subsequent A05 controls use `configs/circle_cover_40m.yaml` (40 m / 95%)
and `configs/circle_cover_40m_99.yaml` (40 m / 99%), with all overlap, grid,
solver and refinement settings unchanged. The latest requested trial uses
40 m / 99%; the original 80 m / 95% configuration is retained for reproduction.
Trajectory-centered comparisons retain the original snapshot radius (80 m)
when the placement radius changes. The directly compared incremental trials
use identical causal map prefixes and the same fixed gravity frame.

Inspection galleries are retained under
`figures/incremental-circle-a05-r40-20260930` and
`figures/incremental-circle-a05-r40-c99-20260930`. `circle_cover_radius_report`
renders direct radius or coverage comparisons from completed states, rejecting
changes to unrelated configuration, inputs or map prefixes.

For a matched footprint-only runtime/memory control, run each mode in a fresh
process with `s3e_pipeline.circle_cover_benchmark --store ... --output ...json
--mode planner` or `--mode trajectory_centered`. These controls retain the same
causal point union and audit the same prefixes, without occupancy histories,
state I/O or plotting. Report their measurements separately from gallery-replay
memory and avoid attributing them to native LiDAR or MapClosures execution.
