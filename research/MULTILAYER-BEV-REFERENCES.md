# Reference papers used for the multilayer ellipsoid BEV preview

Recorded 21 September 2026. This document accompanies the [ground–aerial visual preview](figures/graco-ground-aerial-local-20260921/multilayer-preview/index.html).

The implementation combines **terrain-relative height slicing inspired by ForestLPR** with our existing **MapClosures feature matching**. It is a small, untrained diagnostic, not a reproduction of ForestLPR or a demonstrated aerial–ground loop-closure system. No retrieval threshold was lowered.

| Reference | Role in this preview |
|---|---|
| Gupta et al., MapClosures, ICRA 2024 | Existing density-image, ORB/HBST matching and 2D RANSAC implementation |
| Shen et al., ForestLPR, CVPR 2025 | Terrain-relative height normalization and separate BEV slices: conceptual basis |
| Rublee et al., ORB, ICCV 2011 | Existing local features used through native MapClosures/OpenCV |
| Fischler and Bolles, RANSAC, 1981 | Consensus-fitting foundation; unchanged loop fitting and a separate terrain-plane estimator |
| Hoover et al., Paired-CSLiDAR, 2026 preprint | Background motivation only; its registration algorithm is not implemented here |
| Luo et al., BVMatch, 2021 preprint | Previously discussed alternative descriptor; not implemented here |

## 1. MapClosures: implemented matching backbone

S. Gupta, T. Guadagnino, B. Mersch, I. Vizzo, and C. Stachniss. **Effectively Detecting Loop Closures using Point Cloud Density Maps.** IEEE International Conference on Robotics and Automation (ICRA), 2024. [Paper](https://www.ipb.uni-bonn.de/pdfs/gupta2024icra.pdf), [official implementation and citation](https://github.com/PRBonn/MapClosures).

We retain the experiment's pinned native implementation, revision `1710f15db000a579324e3ba045cd64a0b4d706da`. The upstream repository distinguishes its continuing development from the `ICRA2024` reproduction tag; we therefore do not claim to reproduce the paper's exact experimental version.

Each layer uses 0.5 m pixels, native linear density normalization, the existing ORB settings and self-similarity pruning, Hamming threshold 50, native 3-pixel RANSAC tolerance, and the strict **more than 5 inliers** gate. Matching is performed between the same height band in two selected maps. This pairwise HBST database differs from the original full retrieval database; the full-height control uses the same pairwise protocol for a fair comparison. No inlier counts are pooled across layers.

## 2. ForestLPR: height-slicing inspiration

Yanqing Shen, Turcan Tuna, Marco Hutter, Cesar Cadena, and Nanning Zheng. **ForestLPR: LiDAR Place Recognition in Forests Attentioning Multiple BEV Density Images.** CVPR, 2025. [Paper](https://arxiv.org/abs/2503.04475), [method, Sections 3.1–3.2](https://arxiv.org/html/2503.04475v1), [official code](https://github.com/shenyanqing1105/ForestLPR-CVPR2025).

The paper motivates ground-relative height normalization and multiple horizontal BEV slices. Its full method includes ground segmentation, spatial ground-height interpolation, logarithmic density, learned descriptors and attention across height bands.

We adopt only the **terrain-relative slicing concept**. Our diagnostic fits an independent low-surface plane in each submap and uses five fixed bands: −0.5–2, 2–5, 5–10, 10–20, and ≥20 m. These coarser urban-scene bands differ from the paper's 1–6 m forest configuration. We slice saved ellipsoid surface samples rather than raw LiDAR points, retain the full-height control, and keep native linear density and ORB. We do not use ForestLPR's cloth-simulation filtering, learned backbone, attention, training weights or feature aggregation. Its published results do not establish performance for this adaptation.

## 3. ORB: existing local descriptors

Ethan Rublee, Vincent Rabaud, Kurt Konolige, and Gary Bradski. **ORB: An Efficient Alternative to SIFT or SURF.** ICCV, 2011. [Publisher](https://ieeexplore.ieee.org/document/6126544/), DOI [10.1109/ICCV.2011.6126544](https://doi.org/10.1109/ICCV.2011.6126544).

ORB is used through the existing native MapClosures adapter. Feature detection parameters and descriptor matching thresholds are unchanged. Layering can change which keypoints are detected because each sliced density image contains different structures and has its own native density normalization.

## 4. RANSAC: consensus fitting, with two distinct uses

Martin A. Fischler and Robert C. Bolles. **Random Sample Consensus: A Paradigm for Model Fitting with Applications to Image Analysis and Automated Cartography.** Communications of the ACM, 1981. [Author institution's publication page](https://www.sri.com/publication/artificial-intelligence-pubs/random-sample-consensus-a-paradigm-for-model-fitting-with-applications-to-image-analysis-and-automated-cartography-2/), DOI [10.1145/358669.358692](https://doi.org/10.1145/358669.358692).

Loop-pose fitting uses the unchanged native MapClosures implementation. Separately, our terrain estimator uses a RANSAC-style plane fit to spatial-cell low quantiles. Its 600 trials and 0.35 m support tolerance belong only to **terrain estimation**, not to the loop acceptance gate. These terrain parameters are our experimental choices, not parameters attributed to this paper or ForestLPR.

## 5. Paired-CSLiDAR: consulted background, not implemented

Montana Hoover, Jing Liang, Tianrui Guan, and Dinesh Manocha. **Paired-CSLiDAR: Height-Stratified Registration for Cross-Source Aerial-Ground LiDAR Pose Refinement.** arXiv preprint, 1 May 2026. [Paper](https://arxiv.org/abs/2605.00634).

This work discusses the limited common geometry between aerial observations of roofs/canopy and ground observations of façades/under-canopy. It motivated considering height-dependent shared surfaces. We do not implement its Residual-Guided Stratified Registration, registration-direction reversal, or pose-selection procedure. This preview runs no 3D registration at all.

## 6. BVMatch: alternative discussed, not implemented

Lun Luo, Si-Yuan Cao, Bin Han, Hui-Liang Shen, and Junwei Li. **BVMatch: Lidar-based Place Recognition Using Bird's-eye View Images.** arXiv preprint, 2021. [Paper](https://arxiv.org/abs/2109.00317).

BVMatch was consulted as an alternative to density-image ORB matching, especially for changes in image intensity. Its Log-Gabor/BVFT descriptor and learned bag-of-words model are not used in this preview.

## Our experiment-specific implementation and limitations

- Terrain is approximated as `z = a*x + b*y + c` from the 10th-percentile heights in 4 m cells with at least 20 native map points. Cells receive equal weight. Plane support and spatial spread are checked. A supported low plane can still be a roof or a different terrain level; the method does not provide semantic certainty.
- The plane supplies height labels only. The existing IMU gravity transform continues to define the orthographic XY projection. No inter-robot pose or known flight altitude is used to build the layers.
- The two selected ground–aerial pairs are visual case studies. They do not measure database retrieval recall or false-positive rates.
- Ground truth is not accessed. A previously saved full-height RANSAC transform is used solely to place the comparison images in a common display frame. It does not guide slicing or matching.
- The original full-height controls reproduce the captured ORB features exactly. Source/payload hashes, terrain fits, per-layer descriptors, native matches, pose hypotheses and PNG hashes are retained in `summary.json` and the preview subdirectories.
- The full EllipseLIO/MapClosures/PCM/CBS pipeline has not been rerun for this experiment. This preview is the requested visual checkpoint.

Implementation: [terrain slicing](s3e_pipeline/multilayer_bev.py), [preview runner](../scripts/recent_submaps/preview_multilayer_bevs.py), [numerical checks](tests/test_multilayer_bev.py).

## Follow-up: joint multilayer RANSAC (21 September 2026)

The [joint-matching preview](figures/graco-ground-aerial-local-20260921/joint-multilayer-preview/index.html) adds **our own fusion method**, not ForestLPR's learned attention. Same-height ORB/HBST tentative matches from all five sliced images are pooled in metric gravity-level coordinates. The full-height image remains a separate control because it reuses the same underlying geometry.

The new proper rigid SE(2) estimator draws layers uniformly and then selects matches within each layer. It uses 688 trials with seed 0, the original strict 1.5 m residual tolerance (3 pixels at 0.5 m/pixel), and the strict >5-inlier gate. Endpoint pairs within 0.5 m at both ends are deduplicated, retaining the lowest-Hamming representative; consensus also enforces one-to-one endpoint support within 0.5 m. The 0.5 m deduplication radius and layer-balancing policy are our choices, not settings claimed from MapClosures or ForestLPR. Every single-layer and full-height control is also run through this same fitter. Native fits are retained separately. This is not bit-for-bit native RANSAC, and its stratified sampling does not inherit the native uniform-sampling success-probability calculation.

All 48 cross-robot pairs (six aerial snapshots × eight ground snapshots) were fixed before matching. A separate evaluation program reads GRACO RTK/INS ground truth only after matching has been saved. It labels conservative non-overlap by horizontal anchor separation greater than the sum of area radii plus 20 m, and reports horizontal translation/yaw error. GT never supplies the matches, fitted poses, display alignment, height bands, or acceptance decisions. The 5 m / 5° evaluation bands describe accuracy; they are not new loop acceptance thresholds. This sample is neither an exhaustive retrieval evaluation nor a measurement of general false-positive rate.

Implementation: [joint fitter](s3e_pipeline/joint_bev_ransac.py), [joint preview runner](../scripts/recent_submaps/preview_joint_bev_matches.py), [separate evaluation](../scripts/recent_submaps/evaluate_joint_bev_matches.py), [joint numerical tests](tests/test_joint_bev_ransac.py).

## Follow-up: geometric height initialization and 3D verification

The [3D verification report](RESULTS-GRACO-JOINT-MULTILAYER-REGISTRATION.md) runs all 16 passing joint hypotheses through the existing geometric height initializer and unchanged GICP verifier. It does not run PCM or CBS. The height initializer is our existing geometry-only method: vote for vertical offsets from horizontal nearest neighbors, score several modes using symmetric coarse overlap, and preserve the BEV yaw and horizontal translation. This is not ForestLPR's ground normalization or Paired-CSLiDAR's registration method.

**Aleksandr V. Segal, Dirk Haehnel, and Sebastian Thrun. Generalized-ICP. Robotics: Science and Systems, 2009.** [Author-hosted paper](https://www.robots.ox.ac.uk/~avsegal/resources/papers/Generalized_ICP.pdf), [author's publication list](https://www.robots.ox.ac.uk/~avsegal/research.html). This supplies the GICP algorithmic foundation, modelling local surface structure in both point clouds.

**Kenji Koide. small_gicp: Efficient and parallel algorithms for point cloud registration. Journal of Open Source Software, 9(100), 6948, 2024.** [Paper](https://joss.theoj.org/papers/10.21105/joss.06948), [official implementation](https://github.com/koide3/small_gicp). The existing verifier uses this implementation with its frozen installed native-library hash retained in the new experiment's `summary.json`.

The 0.4 m geometry voxels, 1.5 m correspondence radius, 0.6 m inlier radius, 30% minimum symmetric overlap, 0.35 m maximum RMSE, and observability/conditioning checks are our pipeline's unchanged experiment settings. They are not claimed to be prescribed by either GICP reference. GT is read only by a subsequent evaluation program; it does not select candidates, initialize height, guide GICP, set acceptance decisions, or align the displayed clouds.

## Distributed runtime integration (22 September 2026)

The all-14 GRACO experiment integrates the joint method into the peer workers. Each robot maintains five local HBST indexes, one per height band. A query carries the versioned layer packets in the existing descriptor exchange; each peer searches only its causally available maps. Raw same-band matches are pooled and deduplicated before the existing 20-hypothesis shortlist, then fitted using the joint estimator described above. The resulting IMU-frame seed is preserved through geometric height initialization and unchanged 3D verification. Geometry is requested only for selected candidates. The full-height descriptor is retained as an inspection control and does not contribute matches or rescue a failed layered query.

Terrain fitting remains independent per snapshot. A failed support check produces an explicit unavailable status and empty layer features. The existing registration, singleton-accepting PCM, and CBS settings are unchanged. This experiment uses the updated persistent-map EllipseLIO odometry and 80 m horizontal accumulated-area snapshots; it does not re-enable temporal or spatial restrictions on odometry.

A synthetic 14-worker ROS2/DDS test completed with all 28 anchor poses in one component, 53 retained PCM loops and 53 native registration factors. This validates the transport and endpoint integration; it is not evidence of dataset accuracy. Dataset results are recorded separately after the full run.

Implementation: [distributed layered matcher](s3e_pipeline/multilayer_mapclosures.py), [snapshot preparation](s3e_pipeline/recent_submaps.py), [native raw-correspondence adapter](adapters/mapclosures/adapter.cpp), [runtime tests](tests/test_multilayer_mapclosures.py).

The [completed all-14 GRACO run](RESULTS-GRACO-ALL14-MULTILAYER.md) connects eight aerial and six ground sequences with 541 retained loops, including 243 aerial–ground loops. Shared-alignment position ATE is 0.861 m. This full run has no matched full-height control and therefore does not isolate a causal benefit from layering.
