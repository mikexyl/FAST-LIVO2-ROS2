# Evaluation of frozen joint multilayer BEV matches

All 48 pairs were fixed before matching. GT was loaded afterward in this separate evaluation process. 10 pairs have horizontal anchor separation greater than the sum of their area radii plus 20 m and serve as conservative non-overlap controls.

Other pairs may have intersecting footprints; that does not establish common visible surfaces. The 5 m / 5 degree diagnostic bands describe planar-seed error, not pipeline acceptance. No thresholds were tuned, no 3D registration was run, and no loops were added.

| Method | Pass >5 gate | Non-overlap pairs passing | Passing poses within 5 m / 5° | Passing poses outside bands |
|---|---:|---:|---:|---:|
| full | 3/48 | 0/10 | 2 | 1 |
| near | 6/48 | 0/10 | 6 | 0 |
| low | 14/48 | 0/10 | 14 | 0 |
| middle | 12/48 | 1/10 | 11 | 1 |
| high | 0/48 | 0/10 | 0 | 0 |
| upper | 0/48 | 0/10 | 0 | 0 |
| pooled | 16/48 | 0/10 | 16 | 0 |

## Previously illustrated pairs

| Pair | Method | Unique inliers | XY error [m] | Yaw error [deg] |
|---|---|---:|---:|---:|
| aerial06-17__ground06-7 | full | 6 | 1.242 | 1.177 |
| aerial06-17__ground06-7 | low | 25 | 1.064 | 1.077 |
| aerial06-17__ground06-7 | middle | 23 | 1.377 | 1.128 |
| aerial06-17__ground06-7 | pooled | 52 | 1.216 | 1.045 |
| aerial06-27__ground06-17 | full | 7 | 0.335 | 0.744 |
| aerial06-27__ground06-17 | low | 40 | 0.645 | 0.728 |
| aerial06-27__ground06-17 | middle | 17 | 0.400 | 0.818 |
| aerial06-27__ground06-17 | pooled | 72 | 0.576 | 0.783 |

## Conservative negative pairs

| Pair | Separation [m] | Full inliers | Pooled inliers | Pooled passes |
|---|---:|---:|---:|---|
| aerial06-7__ground06-0 | 189.2 | 0 | 3 | False |
| aerial06-7__ground06-30 | 187.1 | 0 | 0 | False |
| aerial06-12__ground06-27 | 212.8 | 0 | 0 | False |
| aerial06-12__ground06-30 | 252.0 | 0 | 3 | False |
| aerial06-17__ground06-22 | 198.6 | 0 | 3 | False |
| aerial06-17__ground06-27 | 245.8 | 0 | 0 | False |
| aerial06-17__ground06-30 | 285.9 | 0 | 0 | False |
| aerial06-22__ground06-30 | 211.4 | 0 | 3 | False |
| aerial06-27__ground06-0 | 237.3 | 0 | 3 | False |
| aerial06-32__ground06-0 | 239.3 | 0 | 3 | False |

[Gallery](index.html) · [Matching method](README.md) · [Complete evaluation](evaluation.json)
