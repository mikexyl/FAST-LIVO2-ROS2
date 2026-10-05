# GRACO 14-robot teaser

[Paper PDF](graco14-circle-topview-teaser.pdf) · [600 dpi PNG](graco14-circle-topview-teaser.png) · [Preview](preview.png)

Orthographic top view in the same orientation and colors as the validated report map, with all eight aerial and six ground trajectories. Each trajectory uses its robot's exact map RGB color. Dashed lines denote aerial trajectories; solid lines denote ground trajectories. The scale bar spans exactly 100 m in the map's equal-aspect coordinates. Layout: 3.5 × 4.673 inches, suitable for one RA-L column.

The PDF preserves vector trajectories, labels, and scale bar, with a 900 dpi map layer. It uses the same deterministic 350,000-point display subset as the report; the underlying PLY retains 12,951,872 points. The corrected CBS trajectories receive the exact native gravity transform from `export.json`. No ground-truth transform, trajectory smoothing, map rotation, or manual realignment was introduced.

Suggested caption: **Joint map and trajectories of eight aerial and six ground GRACO sequences, reconstructed with fixed spatial circle submaps and persistent CBS. Matching map and trajectory colors identify each robot; dashed and solid lines show aerial and ground trajectories, respectively.**

Source hashes, dimensions, frame transform, and scale endpoints are saved in [provenance.json](provenance.json). Reproduce with the project's research Python environment and `gen_fig_teaser.py`.
