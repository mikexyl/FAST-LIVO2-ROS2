# MegaLoc + MapClosures on S3E Square 1

The distributed setup also has sequence configurations and reports for
[Playground 1](RESULTS-PLAYGROUND1-CBS.md) and
[Campus Road 1](RESULTS-CAMPUS-ROAD1-CBS.md). These retain the Square 1
detection settings and use evo for trajectory evaluation.

The distributed path also supports **S3E Laboratory 1** through
`configs/laboratory1-cbs.yaml`. New trajectory evaluations use **evo 1.36.5**,
including timestamp association and one shared rigid alignment per connected
component. See [DPGO.md](DPGO.md#laboratory-1-and-evo-evaluation) for commands,
saved evo results and the Laboratory 1 ground-truth limitation.
The completed run, partial connectivity result and plots are in
[RESULTS-LABORATORY1-CBS.md](RESULTS-LABORATORY1-CBS.md).

The distributed CBS/ROS path is configured in `square1-cbs.yaml`: direct DDS
loop exchanges and one CBS optimizer per robot. See [DPGO.md](DPGO.md) for build,
run and validation commands, and [RESULTS-CBS.md](RESULTS-CBS.md) for the completed S3E result. The configuration below preserves the completed
centralized reference result.

The current setup has independent visual and LiDAR retrieval branches: FAST-LIVO2 exports → frozen keyframes and causal submaps → MegaLoc / MapClosures candidate union → native MapClosures poses → GICP verification → GTSAM GNC-TLS → corrected trajectories, maps and Rerun. Alpha, Bob and Carol have separate retrieval workers. The optimizer corrects saved artifacts; it sends no feedback to the estimator.

Run from the workspace `src` directory:

```bash
FAST-LIVO2-ROS2/scripts/s3e_experiment.sh run \
  --stage loops pgo evaluate \
  --config FAST-LIVO2-ROS2/research/configs/square1-mapclosures.yaml \
  --input-run .ros2/megaloc-mapclosures/run-b404b29e5e9327f3.json --resume
```

The input registry above is the completed successful run on this machine: 50 accepted inter-robot constraints and 1.1954 m shared-alignment position RMSE. Its odometry, keyframes and fused descriptors are reused after hash/lineage validation. Run `--stage descriptors` only when rebuilding the cached MegaLoc/MapClosures descriptors is required. Stages can run independently using the registry printed by the preceding invocation. Completed artifacts are immutable; interrupted outputs remain marked incomplete.

For a persistent run:

```bash
FAST-LIVO2-ROS2/scripts/run_s3e_background.sh s3e-mapclosures-result \
  .ros2/mapclosures-result.log run --stage loops pgo evaluate \
  --config FAST-LIVO2-ROS2/research/configs/square1-mapclosures.yaml \
  --input-run .ros2/megaloc-mapclosures/run-b404b29e5e9327f3.json --resume
```

The launcher snapshots the source once. Stop it with `systemctl --user stop s3e-mapclosures-result`. Each invocation runs one explicit configuration; there is no sweep or ablation runner.

MegaLoc searches normalized image vectors with cosine similarity ≥0.50. MapClosures independently searches its native HBST index of density-map ORB descriptors; it requires more than five native 2D RANSAC inliers. A LiDAR proposal can qualify with any visual similarity. Each requesting robot chooses at most one candidate per branch across the three replies, deduplicates their union, and applies a two-second cooldown per destination and branch. Same-robot candidates within 30 seconds and future observations are excluded.

MapClosures receives complete saved five-second trailing submaps, cropped at 80 m in body coordinates. Its default density resolution is 0.5 m. At most 20 candidates per receiving robot/query undergo native 2D RANSAC, shortlisted by binary match count. Selected MapClosures poses seed GICP directly. A visual-only proposal uses a native two-map feature match for its pose; if this fails, its failure is recorded.

MegaLoc inference uses CUDA in an isolated environment. This run reuses those saved vectors; MapClosures and GICP run on CPU. Each robot exchanges compact serialized visual vectors and density features, and requests bounded geometric evidence only for selected candidates. Landlock prevents direct access to another robot's store. Candidate records and constraints identify the proposing branches, so the result reports visual-only, LiDAR-only and jointly retrieved loops.

The output directory contains `report.json`, `metrics.csv`, `trajectories.png`, an accepted `loop-evidence.png` when available, corrected TUM trajectories, map arrays and a Rerun 0.37.1 `result.rrd`. The recording uses official ROS2 MCAP decoders for sampled sensors. See [METHODS.md](METHODS.md) for provenance and adaptations, and [RESULTS-MAPCLOSURES.md](RESULTS-MAPCLOSURES.md) for the completed experiment.

Build the new native adapter with:

```bash
.ros2/research-venv/bin/python FAST-LIVO2-ROS2/research/setup_mapclosures.py
```

This checks out pinned MapClosures, HBST and Sophus sources, requires system Eigen/OpenCV/pybind11 development packages, and installs the adapter into the research environment. `scripts/setup_research.sh` includes this step for a fresh setup; use `--stage all` to regenerate odometry only when required.

Run validation with:

```bash
OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 \
  .ros2/research-venv/bin/python -m pytest FAST-LIVO2-ROS2/research/tests -q
```

The supported stages are `odometry`, `descriptors` (including keyframes), `loops`, `pgo`, `evaluate` and native `inspect`. The only supported backend is `megaloc_mapclosures`, configured in `square1-mapclosures.yaml`.

Native MapClosures intermediates can be inspected independently of detection:

```bash
.ros2/research-venv/bin/python FAST-LIVO2-ROS2/research/setup_mapclosures.py --inspection-only
FAST-LIVO2-ROS2/scripts/s3e_experiment.sh run --stage inspect \
  --config FAST-LIVO2-ROS2/research/configs/square1-mapclosures.yaml \
  --input-run .ros2/megaloc-mapclosures/run-b404b29e5e9327f3.json --resume
```

This produces a separate `inspection-run-*.json` registry and immutable `inspect/<hash>/intermediates.rrd`. It reconstructs native density grids, displays all detected ORB features and the retained subset, and recovers exact HBST correspondences and winning RANSAC membership for every saved verification. Reconstructed descriptor bytes, keypoint coordinates, match/inlier counts and native poses must agree with the completed run. The detector binary, original registry and completed detection/PGO artifacts are preserved. See [INSPECTION.md](INSPECTION.md) for the current recording and legend.
