# MapClosures density / ORB / RANSAC inspection

Paths under `.ros2/` refer to local artifacts; generated data and Rerun recordings are not published in this repository.

The native inspection stage displays the working MegaLoc + MapClosures run's density images, detected and retained ORB features, descriptor bits, HBST correspondences and native/GICP pose overlays. Synchronized camera images support visual inspection. It uses the original five-second trailing submaps.

The completed density-only recording contains **211 local maps and all 175 verified pairs**: 50 accepted and 125 rejected. Reconstructed descriptor bytes and keypoint coordinates match the frozen cache. Replaying each receiver's causal HBST database reproduces the saved native match counts, RANSAC inlier counts and valid poses. Visual-only proposals use their original two-map index. An instrumented copy of the same deterministic RANSAC kernel exposes winning correspondence membership.

The upper views show native grayscale density and ORB overlays: green features survived the self-similarity filter, orange features were discarded, and arrows show orientation. Image columns correspond to local LiDAR y; image rows correspond to local LiDAR x. Density resolution is 0.5 m/pixel. In the match view, green lines are native RANSAC inliers and red lines are outliers. The descriptor tab displays retained ORB bits. Native and GICP 3D tabs display the two clouds in the query frame.

Scrub the **pair** timeline from 0 through 174. Pair 0 is a strong independent LiDAR match: **Bob 15 ↔ Alpha 14**, MegaLoc cosine **0.149**, **20 HBST matches / 15 native RANSAC inliers**, accepted GICP RMSE **0.335 m**.

Run from workspace `src`:

```bash
.ros2/research-venv/bin/python FAST-LIVO2-ROS2/research/setup_mapclosures.py --inspection-only
FAST-LIVO2-ROS2/scripts/s3e_experiment.sh run --stage inspect \
  --config FAST-LIVO2-ROS2/research/configs/square1-mapclosures.yaml \
  --input-run .ros2/megaloc-mapclosures/run-b404b29e5e9327f3.json --resume
```

Inspection writes its own immutable artifact and registry. It reads frozen keyframes, descriptors and loop results; it does not rerun retrieval, registration or PGO. The inspection extension preserves the production detector binary. No ground truth is used.

- Working density/ORB Rerun recording (local: `.ros2/megaloc-mapclosures/inspect/1fa34d56bb5b5efc6be647b12e4efea254d098b08c24d2c3725ee25b10fdd1ac/intermediates.rrd`)
- Successful detector run (local: `.ros2/megaloc-mapclosures/run-b404b29e5e9327f3.json`)
- [Results and limitations](RESULTS-MAPCLOSURES.md)
