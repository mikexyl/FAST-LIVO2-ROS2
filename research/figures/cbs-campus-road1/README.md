# Campus Road 1 report figures

Five figures are provided as 350-dpi PNGs and PDFs. Ground truth has gaps;
the three corrected trajectories use one shared evo SE(3) alignment, with
scale fixed to one. All three robots are connected by the saved CBS result.

| Figure | PNG | PDF |
|---|---|---|
| Ground-truth trajectories | [PNG](ground_truth_trajectories.png) | [PDF](ground_truth_trajectories.pdf) |
| CBS trajectories and loops | [PNG](trajectories_and_loops.png) | [PDF](trajectories_and_loops.pdf) |
| Top-down corrected map | [PNG](map_top_down.png) | [PDF](map_top_down.pdf) |
| Oblique corrected map | [PNG](map_oblique.png) | [PDF](map_oblique.pdf) |
| Individual CBS ATE curves | [PNG](position_error.png) | [PDF](position_error.pdf) |

![Ground-truth trajectories](ground_truth_trajectories.png)

![Corrected map](map_top_down.png)

![Individual CBS ATE](position_error.png)

| Robot | CBS ATE RMSE (m) | GT matches |
|---|---:|---:|
| Alpha | 1.5669 | 467 |
| Bob | 1.4956 | 857 |
| Carol | 1.3486 | 581 |
| Combined | 1.4707 | 1,905 |

Per-robot errors use the same shared multi-robot alignment, without individual
fits. Error curves read native evo result arrays. No GT interpolation is used
for trajectory error; gaps are displayed without connecting lines.

Map colors encode robot identity or height, not camera RGB. The 7,455,777 saved
keyframe-map points were independently voxelized at 0.25 m per robot. The
top-down display retains the highest observed point in each 0.30 m pixel;
the oblique plot uses a fixed-seed sample of 650,000 points. Height colors
saturate outside −2 to 25 m, with no height-based point removal or vertical
exaggeration. Full captions and limitations are in the
[report](../../RESULTS-CAMPUS-ROAD1-CBS.md).

Input/output hashes, alignment, numeric results, audit counts and retention
details are in [figures.json](figures.json). Ground-truth source hashes and
the common origin are in [ground_truth.json](ground_truth.json).

After regenerating the experiment, render its result (replace `RUN.json`
with the registry printed by the pipeline):

```bash
PYTHONPATH=FAST-LIVO2-ROS2/research OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 \
  .ros2/research-venv/bin/python -m s3e_pipeline.report_figures \
  --input-run RUN.json \
  --output FAST-LIVO2-ROS2/research/figures/cbs-campus-road1 --height-range -2 25
```

Render the GT-only figure without an experiment:

```bash
PYTHONPATH=FAST-LIVO2-ROS2/research OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 \
  .ros2/research-venv/bin/python -m s3e_pipeline.ground_truth_figures \
  --dataset /data/s3e/S3E_Campus_Road_1 \
  --output FAST-LIVO2-ROS2/research/figures/cbs-campus-road1
```
