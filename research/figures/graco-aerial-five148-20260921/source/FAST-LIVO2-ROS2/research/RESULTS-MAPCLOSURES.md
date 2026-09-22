# Completed MegaLoc + MapClosures experiment

Paths under `.ros2/` refer to local artifacts; generated data and Rerun recordings are not published in this repository.

Completed on 2026-09-11 for all three S3E Square 1 robots. MapClosures is an independent LiDAR retrieval branch in this implementation. Its proposals do not pass through MegaLoc's shortlist or visual threshold. All experiment stages finished successfully; no experiment remains running.

| Measurement | Result |
|---|---:|
| Keyframes | 1,550: Alpha 547, Bob 481, Carol 522 |
| Accepted inter-robot constraints | 50 |
| Accepted intra-robot constraints | 0 |
| LiDAR-only accepted constraints | **29**, all below MegaLoc cosine 0.50 |
| Retrieved by both branches | 12 |
| Visual-only accepted constraints | 9 |
| Geometric verifications | 175, all uncached |
| Native descriptor construction | 107.70 s |
| Loop-detection wall time | 84.21 s |
| PGO wall time | 0.654 s |
| Evaluation, maps and Rerun | 141.43 s |
| Sum of these stage times | 334.11 s (5.57 min) |
| Serialized network traffic | 180,187,723 bytes (171.8 MiB) |
| Connected components | 1: Alpha, Bob and Carol |
| Position RMSE before PGO | 2.1997 m |
| Position RMSE after PGO | **1.1954 m** |
| Position RMSE reduction within this run | 45.7% |
| Ground-truth-associated positions | 11,698 |
| Proximity Recall@1 / 5 / 20 | 78.1% / 86.1% / 88.8% |
| Accepted-loop 10 m proximity precision | 41/49 = 83.7%; one constraint lacks a GT label |
| GNC-rejected accepted constraints | 0 |

The full invocation occupied approximately 7 minutes 19 seconds, measured from the fresh log's creation to its final registry write. The stage-time sum excludes input hashing and registry/lineage checks. Saved FAST-LIVO2 odometry, keyframes and MegaLoc GPU vectors were reused. New MapClosures feature extraction, retrieval and GICP used CPU. Each retrieval worker peaked at 236–255 MiB RSS; its separate verifier peaked at 186–194 MiB. The accumulated worker verification time was 24.07 s, averaging 0.138 s per pair; it is already included in the loop-stage wall time.

Accepted robot-pair counts: Alpha–Bob 18, Alpha–Carol 10 and Bob–Carol 22. Descriptor construction produced features for every keyframe; per-robot median feature counts were 148, 247 and 329.5. Candidate selection has separate one-per-query budgets for MapClosures and MegaLoc, followed by pair deduplication. Attribution reflects the independently qualified sources of the retained proposal, rather than a run with either branch disabled.

There were 125 rejected verifications: 98 exceeded the unchanged 0.35 m registration RMSE limit, 16 failed the overlap gate and 11 lacked enough native MapClosures pose evidence. No thresholds were tuned during this experiment. A visual-only retrieval also uses MapClosures' native two-map feature alignment for initialization; failure there is reported as `mapclosures_no_pose`.


The 10 m proximity metric requires care for map matching. Eight accepted pairs have ground-truth endpoint separations between 10.47 and 24.95 m. They can still observe overlapping submaps. For seven of these pairs, the native-refined translation length agrees with the GT endpoint distance within 0.382 m. This length-only check does not establish full pose correctness. One visual-only pair, **Alpha 543 ↔ Carol 492**, has a **2.622 m translation-length discrepancy** despite passing registration and GNC; it is flagged for inspection. Its accepted registration RMSE was 0.348 m. The frozen constraints have not been edited after examining ground truth. See the per-constraint proximity audit (local: `.ros2/megaloc-mapclosures/proximity-audit.json`).

Evaluation uses one shared rigid alignment across all three robots, with no scale fitting. Before PGO means trajectories after sensor-derived initial robot alignment. S3E ground-truth orientations are placeholders and are excluded; GNSS antenna lever arms remain uncorrected. Ground truth was available only to evaluation and the subsequent read-only audit.

Validation: the native C++ adapter built successfully; 24 pipeline tests passed, including native pose direction with different ground frames, reversed endpoints, empty density features, LiDAR retrieval with orthogonal visual descriptors, independent candidate budgets, cold deterministic three-worker replay, byte accounting and PGO reuse without importing the native backend or modifying inputs. Rerun 0.37.1 verified the recording without errors. All artifact file hashes and input lineage passed verification; frozen odometry and MegaLoc artifacts remain intact.

- Trajectories (local: `.ros2/megaloc-mapclosures/evaluate/5b0c090a55209238b091e0ba1e6cb2be0cbdf86c7f9a605484cc3dd8ee44be3f/trajectories.png`)
- A LiDAR-only loop and registration overlay (local: `.ros2/megaloc-mapclosures/evaluate/5b0c090a55209238b091e0ba1e6cb2be0cbdf86c7f9a605484cc3dd8ee44be3f/loop-evidence.png`)
- Rerun recording (local: `.ros2/megaloc-mapclosures/evaluate/5b0c090a55209238b091e0ba1e6cb2be0cbdf86c7f9a605484cc3dd8ee44be3f/result.rrd`)
- Machine-readable report (local: `.ros2/megaloc-mapclosures/evaluate/5b0c090a55209238b091e0ba1e6cb2be0cbdf86c7f9a605484cc3dd8ee44be3f/report.json`)
- Exact run registry (local: `.ros2/megaloc-mapclosures/run-b404b29e5e9327f3.json`)
- [Method and adaptation report](METHODS.md)

The setup pins PRBonn MapClosures revision `1710f15db000a579324e3ba045cd64a0b4d706da`, preserves its native HBST/density-map pose pipeline, and uses saved five-second local submaps. The source snapshot, compiled extension hash, preprocessing, thresholds and input hashes are recorded with the artifacts. The supplied `local_map` configuration label says “centroids”; the frozen keyframe implementation actually retains one representative point per voxel, as documented in METHODS.md. This label has no effect on computation.

## Rollback verification

The active source now supports only MegaLoc + MapClosures, its native density/ORB inspection, and the shared odometry/PGO/evaluation stages. The spatial-map and RGB-matching experiments, alternative backend, unused coarse initializer, associated configuration files, tests and reports were removed. Existing immutable experiment artifacts remain available.

All **22 retained tests pass**. A fresh three-robot replay on all **1,550 frozen keyframes** reproduced **175 uncached verifications and 50 accepted constraints** in **68.91 seconds**. Constraint and exchange files are **byte-identical** to the successful run. Event results are identical apart from runtime fields. PGO took **0.69 seconds** and reproduced all corrected pose matrices exactly (maximum element difference **0.0**) with unchanged robust weights and one connected component. The original successful recording is open in Rerun. The 16 recorded original registry/manifest hashes remain unchanged.

See the rollback verification record (local: `.ros2/megaloc-mapclosures/rollback-verification.json`). The original **1.1954 m** trajectory result remains applicable to the identical corrected poses; evaluation and odometry were reused.
