# Ellipsoid MapClosures + distributed PCM/CBS: results on 148

Follow-up on 2026-09-17: the [requested IMU-noise/QoS trials](RESULTS-S3E-IMU-NOISE.md) on Library 2 first failed with best-effort input. Restoring reliable input yielded **1.2407 m raw Alpha ATE** over the full sequence, but Bob diverged late and the queue stopped before backend processing. There is no new combined CBS result, and these trials do not replace the frozen results below.

The overnight queue finished all **17 sequence attempts** in **9 h 21 min 5 s**, from **2026-09-16 15:52:56 UTC** to **2026-09-17 01:14:01 UTC** (04:14 Athens time). Four runs completed every stage; two more produced valid CBS trajectories but crashed during reporting. A trajectory-only audit recovered Square 2's score and exactly reproduced Square 3's saved score. Five sequences have valid three-robot ATEs. Laboratory 1 completed without usable timestamped trajectory GT.

The pipeline is **EllipseLIO → native ellipsoid-projected BEVs → MapClosures → distributed PCM → CBS**, with the existing live GICP registration factors enabled. Retrieval uses ellipsoid BEVs, and the backend uses three independent DDS loop workers and three CBS processes. Odometry is replayed serially at 1x. This is the requested fixed multistage setup; there was no parameter sweep. Playground 1 remains excluded.

The 17 attempts do **not** mean 17 successful results. The trajectory table below contains only six sequences with usable backend outputs; every remaining attempt is documented in the failure table.

| Outcome | Count | Sequences |
|---|---:|---|
| Complete with valid three-robot ATE | 3 | Square 1, Playground 2, Playground 3 |
| Valid three-robot ATE recovered after reporting crashes | 2 | Square 2, Square 3 |
| Complete, but no timestamped trajectory GT | 1 | Laboratory 1 |
| Failed before a valid combined CBS result | 11 | Library 1/2, Tunnel 1, Dormitory 1, Teaching Building 1, Laboratory 2/3/4, Campus Road 1/2/3 |
| **Total attempted** | **17** | **Five valid three-robot ATEs** |

## Trajectory results

All values below are **translation ATE RMSE in metres**, computed entirely with **evo 1.36.5**. For each connected three-robot graph, evo estimates one shared rigid alignment with scale fixed to 1. Individual CBS scores use that same alignment, with no extra robot fit. Association uses nearest timestamps within 50 ms, separately per robot. Supplied GT orientations are unused; the antenna lever arm is uncorrected.

| Sequence | Combined CBS | Alpha | Bob | Carol | Original pipeline status |
|---|---:|---:|---:|---:|---|
| Square 1 | 1.1491 | 1.3051 | 1.0144 | 1.1291 | Complete |
| Square 2 | 0.5717 | 0.4993 | 0.7157 | 0.4575 | Trajectory ATE recovered; report incomplete |
| Square 3 | 2.0994 | 2.0426 | 2.1949 | 2.0592 | Saved ATE reproduced; report incomplete |
| Playground 2 | 0.3234 | 0.2072 | 0.4232 | 0.3036 | Complete |
| Playground 3 | 0.3271 | 0.3114 | 0.2819 | 0.3849 | Complete |
| Laboratory 1 | unavailable | unavailable | unavailable | unavailable | Complete; two components, no usable trajectory GT |

Square 1/2/3 and Playground 2/3 have all three robots in the same CBS frame. Laboratory 1 has Alpha alone and a separate Bob–Carol component. Its lack of ATE is a GT limitation; its incomplete graph connectivity is a separate finding.

These are metrics on timestamp-associated GT samples, not every exported pose. The counts below expose the different GT sampling and availability across sequences; no GT interpolation is used.

| Sequence | Alpha matched / exported | Bob matched / exported | Carol matched / exported | Total matched |
|---|---:|---:|---:|---:|
| Square 1 | 384 / 4544 | 452 / 4523 | 337 / 4571 | 1173 |
| Square 2 | 195 / 2357 | 242 / 2457 | 253 / 2525 | 690 |
| Square 3 | 4079 / 4079 | 4217 / 4237 | 4611 / 4611 | 12907 |
| Playground 2 | 220 / 2191 | 218 / 2191 | 218 / 2174 | 656 |
| Playground 3 | 873 / 874 | 992 / 996 | 877 / 877 | 2742 |
| Laboratory 1 | 0 / 2906 | 0 / 2925 | 0 / 2920 | 0 |

## Raw EllipseLIO odometry

Each raw trajectory below gets its own SE(3) fit, with no scale fitting. This is a different alignment scope from the joint CBS scores above. These columns cannot be subtracted to claim an optimization improvement. A matched odometry-only graph comparison has not been performed in this run.

| Sequence | Alpha raw ATE [m] | Bob raw ATE [m] | Carol raw ATE [m] |
|---|---:|---:|---:|
| Square 1 | 1.0938 | 0.7975 | 0.5930 |
| Square 2 | 0.5021 | 0.5955 | 0.1986 |
| Square 3 | 1.9200 | 1.9444 | 1.8933 |
| Playground 2 | 0.1671 | 0.3049 | 0.2606 |
| Playground 3 | 0.3163 | 0.2894 | 0.3749 |
| Laboratory 1 | unavailable | unavailable | unavailable |

## Loops, PCM and backend runtime

PCM is enabled at probability 0.99 with minimum clique size 2. It checks inter-robot measurements; geometrically verified intra-robot loops pass unchanged. The all-loop totals therefore include loops outside PCM's inter-robot checks. A PCM exclusion is a consistency/support decision, not a ground-truth-confirmed outlier label. In Laboratory 1, both excluded inter-robot proposals lacked sufficient support.

| Sequence | Keyframes | All loops proposed → retained | Inter-robot proposed → retained | Live GICP factors | Detection + PCM + CBS [s] |
|---|---:|---:|---:|---:|---:|
| Square 1 | 1552 | 55 → 55 | 7 → 7 | 55 | 76.85 |
| Square 2 | 856 | 23 → 23 | 17 → 17 | 23 | 38.30 |
| Square 3 | 1532 | 192 → 192 | 163 → 163 | 192 | 195.84 |
| Playground 2 | 1282 | 76 → 75 | 52 → 51 | 75 | 47.17 |
| Playground 3 | 422 | 121 → 121 | 97 → 97 | 121 | 84.31 |
| Laboratory 1 | 798 | 46 → 44 | 5 → 3 | 44 | 18.64 |

These backend times exclude serial odometry playback, ellipsoid/BEV preparation and evaluation/map rendering. They are not end-to-end sequence times.

## Failed and incomplete stages

The original queue status is preserved: **4 complete and 13 failed attempts**. Two failures occurred after successful optimization, allowing trajectory evaluation from frozen artifacts. The remaining 11 attempts have no valid combined CBS ATE.

| Sequence | Stage / robot | Observed failure | Result availability |
|---|---|---|---|
| Square 2 | Evaluation / map rebuilding | Segmentation fault (exit -11), after Alpha/Bob map export | Trajectory-only evo ATE recovered for all three robots; full report remains incomplete |
| Square 3 | Report generation | Segmentation fault (exit -11), after scores, trajectories, maps and Rerun were saved | Saved ATE exactly reproduced; final report remains incomplete |
| Library 1 | CBS frame validation | Retained edges cross inconsistent output reference frames | No valid combined ATE; 563 retained loops, including 557 inter-robot loops |
| Tunnel 1 | CBS frame validation | Retained edges cross inconsistent output reference frames | No valid combined ATE; 602 retained loops, including 572 inter-robot loops |
| Dormitory 1 | Ellipsoid preparation / Carol | CUDA voxel hash capacity exceeded | No CBS result |
| Teaching Building 1 | Ellipsoid preparation / Carol | CUDA voxel hash capacity exceeded | No CBS result |
| Laboratory 2 | Descriptor preparation / Bob | Segmentation fault (exit -11) | No CBS result |
| Laboratory 3 | Descriptor preparation / Carol | Segmentation fault (exit -11) | No CBS result |
| Laboratory 4 | EllipseLIO / Carol | Implausible motion; maximum scan-derived speed 136.7 m/s | Rejected before loop detection/CBS |
| Campus Road 1 | EllipseLIO / Alpha | Implausible motion; maximum scan-derived speed 544.8 m/s | Rejected before loop detection/CBS |
| Campus Road 2 | EllipseLIO / Alpha | Implausible motion; maximum scan-derived speed 374.0 m/s | Rejected before loop detection/CBS |
| Campus Road 3 | EllipseLIO / Alpha | Native octree reached its 10,000,000-octant limit and exited, after severe divergence | No CBS result |
| Library 2 | EllipseLIO / Alpha | Implausible motion; maximum scan-derived speed 485.7 m/s | Rejected before loop detection/CBS |

The motion guard is a sensor-only ground-robot sanity check with a 20 m/s limit, not an accuracy metric. Frontend failures precede loop detection and CBS. Preparation failures and reporting crashes are implementation failures; these attempts do not establish that the underlying loop-closure method fails on those sequences. The exact causes of the native segmentation faults and CBS reference-frame failures remain unresolved. No failed sequence was automatically rerun with changed parameters.

PCM retained every proposal in Library 1 and Tunnel 1, but CBS still failed the common-frame contract. Consequently, PCM passing does not demonstrate a valid distributed solution. We do not use GT to repair those reference frames and then report an ATE.

### Native EllipseLIO error-log audit

The five failed frontends' complete `mapping.log` files were inspected separately from the wrapper summaries. Campus Road 3 / Alpha reports `Octant overflow max: 10000000 num: 10000000`; EllipseLIO's octree implementation explicitly calls `exit(1)` at this capacity limit. Its recorded odometry had already accumulated about 574 km of apparent travel. The overflow explains the termination, but does not explain what initially caused the trajectory to diverge. The subsequent export EOF error is a consequence of the mapper ending without closing its export stream.

The other four native logs have startup messages `Lidar has no new data`, `IMU has no new data`, and `Lidar start time is before IMU start time`, followed by normal shutdown without a later explanatory estimator error. The same startup messages occur in successful Square 1 / Alpha and Playground 2 / Alpha runs, and in Laboratory 4 / Alpha, whose frontend passed the motion guard. They are not sufficient evidence of a broken bag or the cause of divergence. Campus Road 1's playback log additionally reports one message-queue starvation warning near the end; its relationship to divergence has not been established.

The frozen runtime configurations set `publish.analytics: false`. Consequently, EllipseLIO's native registration residual, feature-rejection, observability and iteration diagnostics were not recorded through that topic. The exact onset and cause of the four silent divergences remain unresolved; the speed checks came from our independent trajectory validator, not an EllipseLIO divergence alarm. [Native logs, runtime configurations and wrapper summaries](figures/ellipsoid-cbs148-results/frontend-native-log-audit.json).

## Audit and retained evidence

The post-run audit read only saved dense odometry, keyframe indices and native CBS poses/constraints. It checked their hashes against original manifests, finite rigid transforms, ordered timestamps, exact keyframe/pose correspondence, endpoint validity and the common-frame contract. Dense corrections use the existing interpolation implementation. Square 2's existing Alpha/Bob corrected trajectories and all three Square 3 trajectories matched the reconstructed versions within 1e-10. Square 3's combined and individual ATEs reproduced within 1e-10. No estimator, descriptor, loop, PCM or optimization stage was rerun, and original failed completion states remain unchanged.

[Machine-readable summary](figures/ellipsoid-cbs148-results/summary.json), [frozen queue](figures/ellipsoid-cbs148-results/queue.json), [failure log excerpts](figures/ellipsoid-cbs148-results/failure-logtails.json), [audit script](figures/ellipsoid-cbs148-results/trajectory-audit.py), [Square 2 audit](figures/ellipsoid-cbs148-results/S3E_Square_2/trajectory-audit-20260917/audit.json), [Square 3 audit](figures/ellipsoid-cbs148-results/S3E_Square_3/trajectory-audit-20260917/audit.json). The 591 initially retrieved evidence files passed SHA-256 verification; native backend files and completed report hashes also matched their original markers. Local evidence includes evo ZIPs, matched trajectories, configuration, PCM decisions, quality diagnostics and figures.

| Sequence | Trajectory | Corrected map | Ellipsoid density / ORB BEVs | Numeric results |
|---|---|---|---|---|
| Square 1 | [PNG](figures/ellipsoid-cbs148-results/S3E_Square_1/report/trajectories.png) | [PNG](figures/ellipsoid-cbs148-results/S3E_Square_1/report/maps.png) | [PNG](figures/ellipsoid-cbs148-results/S3E_Square_1/report/bevs.png) | [JSON](figures/ellipsoid-cbs148-results/S3E_Square_1/report/report.json) |
| Square 2 | unavailable | unavailable | unavailable | [JSON](figures/ellipsoid-cbs148-results/S3E_Square_2/trajectory-audit-20260917/audit.json) |
| Square 3 | [PNG](figures/ellipsoid-cbs148-results/S3E_Square_3/report/trajectories.png) | [PNG](figures/ellipsoid-cbs148-results/S3E_Square_3/report/maps.png) | unavailable | [JSON](figures/ellipsoid-cbs148-results/S3E_Square_3/trajectory-audit-20260917/audit.json) |
| Playground 2 | [PNG](figures/ellipsoid-cbs148-results/S3E_Playground_2/report/trajectories.png) | [PNG](figures/ellipsoid-cbs148-results/S3E_Playground_2/report/maps.png) | [PNG](figures/ellipsoid-cbs148-results/S3E_Playground_2/report/bevs.png) | [JSON](figures/ellipsoid-cbs148-results/S3E_Playground_2/report/report.json) |
| Playground 3 | [PNG](figures/ellipsoid-cbs148-results/S3E_Playground_3/report/trajectories.png) | [PNG](figures/ellipsoid-cbs148-results/S3E_Playground_3/report/maps.png) | [PNG](figures/ellipsoid-cbs148-results/S3E_Playground_3/report/bevs.png) | [JSON](figures/ellipsoid-cbs148-results/S3E_Playground_3/report/report.json) |
| Laboratory 1 | [PNG](figures/ellipsoid-cbs148-results/S3E_Laboratory_1/report/trajectories.png) | [PNG](figures/ellipsoid-cbs148-results/S3E_Laboratory_1/report/maps.png) | [PNG](figures/ellipsoid-cbs148-results/S3E_Laboratory_1/report/bevs.png) | [JSON](figures/ellipsoid-cbs148-results/S3E_Laboratory_1/report/report.json) |

![Playground 2 corrected trajectories and GT](figures/ellipsoid-cbs148-results/S3E_Playground_2/report/trajectories.png)

![Playground 2 corrected map](figures/ellipsoid-cbs148-results/S3E_Playground_2/report/maps.png)

Rerun recordings remain on 148: Square 1 **105.4 MB**, Laboratory 1 **16.3 MB**, Playground 2 **85.1 MB**, Playground 3 **86.6 MB**, Square 3 **102.7 MB**. Square 2 did not reach Rerun generation. Exact host paths are in `summary.json`; these recordings were retained remotely rather than duplicated in the local evidence bundle.

Exact scan/pose exports from valid completed frontends remain under each attempt's `shared-odometry/` for the pending matched Swarm-SLAM comparison. Sensor bytes, integer timestamps and poses are unchanged. Large geometry caches were retired after preserving evidence; original S3E bags remain intact. Retired preparation artifacts are not reusable complete stage caches. **The matched Swarm comparison is still pending; these results are our ellipsoid MapClosures + PCM/CBS pipeline.**

## Deployment and validation before launch

The persistent Docker container is `ellipsoid-cbs148-overnight`, with restart policy `unless-stopped`, 16 CPU cores, a 48 GiB memory limit and GPU access. The image is pinned to `sha256:cb89f7676e64520d3c6acfb37be9006b1b1dd52019d928b09f8ed7315ceb3ecc`. The host workspace is `/data3/mikexyl/swarm_s3e_ws/src`; datasets are mounted read-only. Source/configuration/binary hashes were frozen at launch. At the post-run inspection the container was idle after queue completion, with zero restarts.

- All ten native CBS/cbs_ros tests passed, including the explicitly enabled distributed PCM protocol test.
- Fourteen integration tests passed, including false closures, disconnected graphs, GICP exchange and ellipsoid geometry; three queue/retention tests passed.
- CUDA surface rendering matched the CPU reference within 0.00001 m in the deployment check.
- A corrected 45-second three-robot smoke test exported and processed **136 keyframes** (53/36/47), **five PCM-retained loops** and **five live GICP factors**. All three PCM participants reached `ready`. Carol remained a separate component in this short segment; synthetic tests also cover connected graphs and unknown initial alignment.
- The smoke Rerun recording passed verification. The smoke is not a full-sequence ATE result.

The initial smoke exposed a submap metadata label mismatch at the registration-factor handoff. The producer now uses the existing coordinate/time contract; geometry was already in the correct keyframe IMU frame. The complete short test was rerun successfully before the persistent launch.

[Validation evidence](figures/ellipsoid-cbs148-validation/smoke-validation.json), [smoke report](figures/ellipsoid-cbs148-validation/smoke/REPORT.md), [integration tests](figures/ellipsoid-cbs148-validation/integration-tests.xml), [queue tests](figures/ellipsoid-cbs148-validation/queue-tests.xml), [deployment record](figures/ellipsoid-cbs148-validation/deployment.json).

[Implementation and reproduction details](../scripts/ellipsoid_cbs148/README.md).
