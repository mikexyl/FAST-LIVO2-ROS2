# S3E Square 1 report figures

The figures below retain the original interpolated-GT evaluation. The
[updated evo ATE table](../../RESULTS-CBS.md#cbs-ate-re-evaluated-with-evo)
uses the same frozen CBS trajectories with nearest timestamp matching:
Alpha 1.5182 m, Bob 1.0740 m, Carol 0.9827 m; combined 1.2147 m.
All three robots share one SE(3) alignment, without scale fitting.
[Full-precision evo results and input hashes](evo_ate.json).

Four figures are available as 350 dpi PNGs and PDFs. PDF axes, text and
trajectory lines remain vector graphics; point-cloud layers are rasterized.
Use the PDFs in LaTeX or a print report and the PNGs in slides or Markdown.

## 1. Trajectories and loop graph

![Trajectories and verified loops](trajectories_and_loops.png)

**Caption.** Distributed CBS result on S3E Square 1. (a) Corrected trajectories
for Alpha, Bob and Carol, with position ground truth in dashed gray; circles
mark the trajectory starts. (b) The 50 verified inter-robot constraints, colored
by retrieval branch: 29 MapClosures only, 12 found by both branches and 9 MegaLoc
only. All robots share one SE(3) alignment to position ground truth, without
scale fitting. Ground-truth gaps exceeding two seconds are left unconnected.
Coordinates are relative to Alpha's first ground-truth position.

[PDF](trajectories_and_loops.pdf) · [PNG](trajectories_and_loops.png)

## 2. Top-down map

![Top-down map](map_top_down.png)

**Caption.** Top-down views of the CBS-corrected LiDAR map, rebuilt from the
1,550 keyframe scans. (a) Points colored by contributing robot, with trajectories
overlaid. (b) The same map colored by elevation relative to Alpha's initial
ground-truth elevation, with robot-colored trajectories. The three saved maps
contain 4,135,939 points in total after independent 0.25 m voxel filtering.
For display, the highest observed point in each 0.30 m XY pixel is shown.
Colors saturate below 0 m and above 25 m; no elevation-based geometry filtering
is applied. The source contains XYZ coordinates; colors encode robot identity
or elevation, rather than camera RGB.

[PDF](map_top_down.pdf) · [PNG](map_top_down.png)

## 3. Oblique map

![Oblique map](map_oblique.png)

**Caption.** Oblique view of the merged CBS map, colored by elevation with
corrected robot trajectories overlaid. A fixed-seed uniform sample of 650,000
points is used for rendering; the saved map remains unchanged. Metric proportions
are preserved with no vertical exaggeration. The viewpoint uses 52° elevation
and −65° azimuth, and the color scale matches the top-down map.

[PDF](map_oblique.pdf) · [PNG](map_oblique.png)

## 4. Position error

![Position error](position_error.png)

**Caption.** Euclidean 3D position error over time for each robot: distributed
CBS in robot colors and the frozen centralized reference in dashed gray.
Each complete three-robot solution uses one shared rigid alignment; no separate
per-robot fitting or scale correction is applied. Missing ground-truth intervals
remain blank and no smoothing is used. The plotted 11,698 matched samples
reproduce the saved overall RMSE values of 1.2112 m for CBS and 1.1954 m for
the centralized reference. S3E's placeholder orientations are unused.

[PDF](position_error.pdf) · [PNG](position_error.png)

## Reproduce

From the workspace `src` directory:

```bash
PYTHONPATH=FAST-LIVO2-ROS2/research OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 \
  .ros2/research-venv/bin/python -m s3e_pipeline.report_figures \
  --input-run .ros2/megaloc-mapclosures-cbs/run-af384eeda86f7f19.json \
  --output FAST-LIVO2-ROS2/research/figures/cbs-square1
```

This reads the completed evaluation and does not run detection or optimization.
`figures.json` records the source artifact hashes, generator hash, plotting
parameters, alignment, per-robot error statistics and output file hashes.

Example LaTeX inclusion:

```latex
\begin{figure*}[t]
  \centering
  \includegraphics[width=\textwidth]{figures/cbs-square1/map_top_down.pdf}
  \caption{Top-down views of the CBS-corrected multi-robot LiDAR map on
  S3E Square 1, colored by contributing robot (left) and elevation (right).}
  \label{fig:s3e-cbs-map}
\end{figure*}
```
