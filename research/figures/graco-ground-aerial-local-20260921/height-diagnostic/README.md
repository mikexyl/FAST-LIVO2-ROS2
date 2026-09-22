**Ground–aerial height diagnostic**

Offline replay of the two strongest five-inlier retrieval hypotheses. These candidates bypassed the strict >5 retrieval gate only for this diagnostic. The production pipeline, graph, retrieval settings, geometric thresholds, and PCM results remain unchanged.

The existing geometry-only initializer votes on vertical differences between horizontal neighbors after the saved BEV pose, then scores height hypotheses by symmetric coarse overlap. No ground truth or nominal flight altitude was used.

| Query ↔ candidate | Height correction (m) | Without correction | Final symmetric overlap | Final inlier RMSE (m) | 3D gate |
|---|---:|---|---:|---:|---|
| ['aerial06', 27] ↔ ['ground06', 17] | -19.5 | initial_low_overlap | 48.02% | 0.266 | accepted |
| ['aerial06', 17] ↔ ['ground06', 7] | -20.0 | initial_low_overlap | 29.38% | 0.307 | low_overlap |

Negative height correction maps ground coordinates downward in the aerial anchor’s gravity-leveled frame. It is a relative frame translation, not an estimate of either robot’s height above local terrain.

The geometric gate remains ≥30% symmetric overlap, ≤0.35 m inlier RMSE, ≥100 inliers, convergence and the existing observability/conditioning limits. The first pair passes; the second remains rejected at 29.38% overlap. No loops were sent to PCM or CBS.

After all estimator calls completed, the unchanged transforms were evaluated against the provided RTK/INS poses at their anchor timestamps:

| Query ↔ candidate | Translation error (m) | Rotation error (deg) |
|---|---:|---:|
| ['aerial06', 27] ↔ ['ground06', 17] | 1.062 | 1.067 |
| ['aerial06', 17] ↔ ['ground06', 7] | 1.077 | 1.352 |

[Complete diagnostic](summary.json) · [Evaluation-only check](evaluation-only.json) · [Script](inspect_height.py)

Recommended next experiment: a bounded fallback for spatially supported weak BEV proposals, followed by the existing geometry-only vertical initialization and unchanged full 3D verification. In a separate ablation, test terrain-relative multi-height BEVs with density normalization and confidence-weighted common structure. Sensor height alone cannot identify which surfaces are shared; accumulated submaps contain multiple observation poses.

Related approaches: [ForestLPR](https://arxiv.org/abs/2503.04475) uses terrain-normalized height slices and multi-BEV attention; [Paired-CSLiDAR](https://arxiv.org/abs/2605.00634) is a 2026 preprint investigating height-stratified aerial–ground pose refinement. Neither establishes performance for this EllipseLIO pipeline.
