# GRACO A05 accumulated-area overlap inspection

Source: `/home/mikexyl/workspaces/fast_livo2_ws/src/FAST-LIVO2-ROS2/research/figures/graco-aerial-singleton148-20260921/full`. This historical capture has 34 snapshots using the current 80 m-radius / 20 m-or-10 s accumulated-area policy. It is not a new pipeline evaluation.

- Median fraction of current points already in the immediately previous snapshot: **80.3%**.
- Median fraction of previous points retained: **87.2%**.
- Median horizontal center displacement: **20.05 m**.
- Median ideal equal-disk overlap: **84.1%**. This geometric estimate treats the two gravity planes as parallel; point-ID overlap is exact.
- 6,583,576 point occurrences across snapshots, 887,047 distinct stable point IDs: **7.42 snapshot memberships per exported point** on average.
- Triggers: {'snapshot_interval': 16, 'displacement': 17, 'shutdown': 1}. Adjacent statistics include startup and shutdown snapshots.

The global overview uses the saved raw odometry-frame map (0.25 m display downsampling). Each submap view is its original causal, hash-verified sampled-ellipsoid BEV; it is not a crop of the final map. These are historical ellipsoid images, not newly rendered point-cloud BEVs. All views share metric scale and horizontal heading; per-anchor gravity is preserved, while the map overview uses median gravity. Brightness is enhanced for inspection only. The circle indicates spatial selection, not observed coverage. Blank space can be unobserved or suppressed by the original density threshold.

Exact overlap uses immutable point-ID ranges before backend downsampling. Point-count reuse is not a measured runtime, memory, or communication multiplier. Different snapshot endpoints can repeat geometry work even when each endpoint is cached. No experiment settings were changed. Workstation 148 was unreachable, so full 3D per-snapshot payloads were not rendered; all saved 2D views are included.

[Interactive gallery](index.html) · [Overview](overview.png) · [All submaps](all-submaps.png) · [Exact overlap CSV](overlap.csv) · [Provenance](summary.json)
