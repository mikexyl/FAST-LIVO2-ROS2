# Playground 1 report figures

Bob starts at bag offset 22 seconds, after its IMU gaps. Alpha and Carol use
whole-sequence replays. Joint CBS accuracy remains poor (25.6036 m RMSE).
The raw odometry diagnostic also shows large Alpha/Bob errors. Per-robot ATEs use their shared component alignment
and available timestamp-matched estimates. See the [report](../../RESULTS-PLAYGROUND1-CBS.md).

| Figure | PNG | PDF |
|---|---|---|
| Trajectory coverage | [PNG](trajectory_coverage.png) | [PDF](trajectory_coverage.pdf) |
| Raw odometry diagnostic | [PNG](raw_odometry_diagnostic.png) | [PDF](raw_odometry_diagnostic.pdf) |
| Ground-truth trajectories | [PNG](ground_truth_trajectories.png) | [PDF](ground_truth_trajectories.pdf) |
| CBS trajectories and loops | [PNG](trajectories_and_loops.png) | [PDF](trajectories_and_loops.pdf) |
| Top-down map | [PNG](map_top_down.png) | [PDF](map_top_down.pdf) |
| Oblique map | [PNG](map_oblique.png) | [PDF](map_oblique.pdf) |
| Individual CBS ATE | [PNG](position_error.png) | [PDF](position_error.pdf) |

![Trajectory coverage](trajectory_coverage.png)

![Corrected map](map_top_down.png)

Each figure has a 350-dpi PNG and PDF. Input/output hashes, complete numerical
results and retention status are in [figures.json](figures.json). Ground-truth
hashes are in [ground_truth.json](ground_truth.json); raw bag IMU gap evidence
is in [imu_gaps.json](imu_gaps.json). Map colors indicate robot or height,
not camera RGB. Bob contributes only its post-gap trajectory and map.

After regenerating the run, render its plots with the printed registry:

```bash
PYTHONPATH=FAST-LIVO2-ROS2/research OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 \
  .ros2/research-venv/bin/python -m s3e_pipeline.report_figures \
  --input-run RUN.json \
  --output FAST-LIVO2-ROS2/research/figures/cbs-playground1 --height-range -2 25
```
