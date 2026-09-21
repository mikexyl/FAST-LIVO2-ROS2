# evo trajectory evaluation

evo 1.36.5; nearest timestamp association within 0.05 s, no interpolation or time offset. Translation APE uses one shared SE(3) alignment per connected component with scale fixed to 1. Association is performed separately for each robot. The pooled KITTI files contain matched poses in robot order. They are for component APE, not RPE or temporal analysis.

Reproduce a component result (replace `Alpha` with its component ID):

```bash
evo_ape kitti Alpha-reference.kitti Alpha-estimate.kitti -a -r trans_part --save_results check.zip
evo_res Alpha-component-ape.zip --save_plot ape.pdf --save_table statistics.csv
```

Each robot ZIP uses the shared component alignment without an additional fit. Unavailable results are recorded in `evaluation.json`; no numeric accuracy is invented. GT orientations are not used.
