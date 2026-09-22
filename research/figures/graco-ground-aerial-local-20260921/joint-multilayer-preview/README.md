# Joint multilayer BEV RANSAC preview

Fixed 48-pair ground–aerial Cartesian-product diagnostic. No bag replay, GICP, PCM or CBS.

Same-layer native ORB/HBST tentative matches enter a proper rigid SE(2) fitter. Raw matches, including each layer’s rejected matches, are pooled. Full-height geometry is a separate control. Native keypoints already include image crop origins; pixel coordinates are converted to meters.

Both endpoints within 0.5 m identify duplicate constraints; lowest Hamming distance wins. Final support is also one-to-one within 0.5 m at each endpoint. Hypotheses sample layers uniformly, then matches uniformly within each chosen layer. Settings are fixed: 688 draws, seed 0, strict residual <1.5 m (native 3 pixels), and strict >5 unique-inlier gate. Refinement fits a proper rotation without scale and recounts support afterward.

**This is a new consensus implementation, not bit-for-bit native RANSAC.** Every full-height and individual-layer control also uses this same fitter and uniqueness rule. Original native counts and poses are retained separately for comparison. Layer balancing changes sampling; it does not guarantee independent evidence or calibrated false-positive rates.

Each match figure uses its own joint estimate only for display. Colored lines are unique joint inliers, colored by retained layer; gray lines are rejected. A visually aligned image is not independent validation. The separate evaluation script may subsequently read GT; this matching stage never does.

| Pair | Full | Near | Low | Middle | High | Upper | Pooled |
|---|---:|---:|---:|---:|---:|---:|---:|
| aerial06-7__ground06-0 | 0 | 0 | 3 | 3 | 0 | 0 | 3 |
| aerial06-7__ground06-5 | 0 | 0 | 3 | 3 | 0 | 0 | 3 |
| aerial06-7__ground06-7 | 0 | 0 | 0 | 0 | 0 | 0 | 3 |
| aerial06-7__ground06-12 | 0 | 5 | 7 | 3 | 0 | 0 | 11 |
| aerial06-7__ground06-17 | 7 | 6 | 27 | 17 | 0 | 0 | 49 |
| aerial06-7__ground06-22 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| aerial06-7__ground06-27 | 0 | 0 | 0 | 0 | 0 | 0 | 3 |
| aerial06-7__ground06-30 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| aerial06-12__ground06-0 | 0 | 0 | 3 | 3 | 0 | 0 | 3 |
| aerial06-12__ground06-5 | 0 | 0 | 0 | 3 | 0 | 0 | 3 |
| aerial06-12__ground06-7 | 0 | 0 | 7 | 6 | 0 | 0 | 14 |
| aerial06-12__ground06-12 | 0 | 7 | 28 | 31 | 0 | 0 | 67 |
| aerial06-12__ground06-17 | 0 | 0 | 3 | 7 | 0 | 0 | 13 |
| aerial06-12__ground06-22 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| aerial06-12__ground06-27 | 0 | 0 | 0 | 3 | 0 | 0 | 0 |
| aerial06-12__ground06-30 | 0 | 0 | 0 | 3 | 3 | 0 | 3 |
| aerial06-17__ground06-0 | 0 | 0 | 3 | 0 | 0 | 0 | 3 |
| aerial06-17__ground06-5 | 3 | 0 | 11 | 9 | 0 | 0 | 22 |
| aerial06-17__ground06-7 | 6 | 0 | 25 | 23 | 3 | 0 | 52 |
| aerial06-17__ground06-12 | 0 | 5 | 14 | 19 | 0 | 0 | 37 |
| aerial06-17__ground06-17 | 0 | 0 | 0 | 0 | 0 | 0 | 3 |
| aerial06-17__ground06-22 | 0 | 0 | 0 | 6 | 0 | 0 | 3 |
| aerial06-17__ground06-27 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| aerial06-17__ground06-30 | 0 | 0 | 0 | 3 | 0 | 0 | 0 |
| aerial06-22__ground06-0 | 0 | 0 | 0 | 0 | 0 | 0 | 3 |
| aerial06-22__ground06-5 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| aerial06-22__ground06-7 | 0 | 0 | 10 | 4 | 0 | 0 | 16 |
| aerial06-22__ground06-12 | 0 | 18 | 42 | 23 | 0 | 0 | 83 |
| aerial06-22__ground06-17 | 4 | 10 | 17 | 13 | 0 | 0 | 41 |
| aerial06-22__ground06-22 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| aerial06-22__ground06-27 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| aerial06-22__ground06-30 | 0 | 0 | 0 | 0 | 0 | 0 | 3 |
| aerial06-27__ground06-0 | 0 | 0 | 0 | 3 | 0 | 0 | 3 |
| aerial06-27__ground06-5 | 0 | 0 | 0 | 3 | 0 | 0 | 0 |
| aerial06-27__ground06-7 | 0 | 0 | 0 | 3 | 0 | 0 | 3 |
| aerial06-27__ground06-12 | 0 | 5 | 15 | 3 | 0 | 0 | 20 |
| aerial06-27__ground06-17 | 7 | 15 | 40 | 17 | 3 | 0 | 72 |
| aerial06-27__ground06-22 | 0 | 0 | 3 | 4 | 0 | 0 | 3 |
| aerial06-27__ground06-27 | 0 | 0 | 0 | 0 | 0 | 0 | 3 |
| aerial06-27__ground06-30 | 0 | 3 | 0 | 0 | 0 | 0 | 3 |
| aerial06-32__ground06-0 | 0 | 0 | 3 | 0 | 0 | 0 | 3 |
| aerial06-32__ground06-5 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| aerial06-32__ground06-7 | 0 | 0 | 3 | 3 | 0 | 0 | 3 |
| aerial06-32__ground06-12 | 0 | 3 | 16 | 3 | 0 | 0 | 18 |
| aerial06-32__ground06-17 | 4 | 13 | 34 | 15 | 0 | 0 | 62 |
| aerial06-32__ground06-22 | 0 | 0 | 3 | 4 | 0 | 0 | 6 |
| aerial06-32__ground06-27 | 0 | 0 | 0 | 0 | 0 | 0 | 3 |
| aerial06-32__ground06-30 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |

[Gallery](index.html) · [Fixed design](design.json) · [Raw matching diagnostics](matching.json) · [References](REFERENCES.md)
