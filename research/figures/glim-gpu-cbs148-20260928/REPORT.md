# GLIM GPU registration path in CBS — validation on 148

The new default uses native CUDA VGICP registration factors and GLIM's relinearization/caching settings. All eight matched frozen-graph solves completed, retained one connected three-robot component, and passed the final geometry support checks. Median full-solve time improved from 42.42 to 19.91 s on GRACO (2.13×) and from 125.96 to 29.09 s on Library 1 (4.33×). Accuracy remained close across these controls; two repeats do not establish statistical equivalence.

| Group | Mode | Solve time, both repeats (s) | Joint ATE, both repeats (m) | Peak host RSS (GiB) | Whole GPU peak (MiB) |
|---|---|---:|---:|---:|---:|
| GRACO A05/A07/A08 | gicp | 42.40, 42.44 | 0.4846, 0.4852 | 1.44 | 81 |
| GRACO A05/A07/A08 | vgicp_gpu | 19.81, 20.02 | 0.4843, 0.4851 | 1.76 | 1230 |
| S3E Library 1 | gicp | 125.58, 126.33 | 1.6023, 1.6188 | 2.29 | 81 |
| S3E Library 1 | vgicp_gpu | 29.08, 29.10 | 1.6173, 1.6194 | 2.61 | 1874 |

The solve timer includes fresh-session preparation, PCM, registration exchange and the 100-iteration schedule at 10 Hz. This is a comparison of two CBS configurations, not a benchmark of GLIM itself or an isolated CUDA-kernel speedup. Odometry, frozen verified loop inputs, evidence budgets, conservative factor weighting and iteration budget are identical. Run order is CPU/GPU/GPU/CPU, serially per group. The RTX 5080/CUDA 12.9 measurements used no container resource quotas. Other authorized workstation jobs were not stopped, so these are workstation measurements rather than isolated hardware microbenchmarks.

## Accuracy and support

Dense trajectories were corrected using each optimized submap pose and evaluated after optimization with **evo 1.36.5**, 50 ms association, one shared rigid alignment per connected component and scale fixed to one. Ground truth never enters registration or CBS. GRACO uses its released IMU reference: all 8,983 dense poses associate. S3E Library 1 has only 1,031 associated samples among 12,299 dense poses; its sparse ground truth and unavailable IMU-to-RTK lever arms remain limitations. Small CPU/GPU differences should not be interpreted as a demonstrated accuracy gain or loss.

GRACO retains 106 pose loops and admits 100 geometry factors; Library 1 admits 260 geometry factors. Every admitted geometry factor passes the unchanged final inlier, overlap and observability checks. Initial rejection of unsupported geometry does not remove its PCM-retained companion pose constraint. The voxelized GPU objective differs from point GICP, so their optima need not coincide.

## What changed and why it is faster

The native `IntegratedVGICPFactorGPU` kernels from pinned gtsam_points v1.2.2 perform registration against two adaptive voxel levels (base 0.5–1.0 m, second level 2×). Endpoint points, covariances and voxel maps are uploaded once per session and shared across its factors. GPU factor linearizations are reused only when the relative pose is exactly identical. GLIM's 0.1 iSAM2 relinearization threshold and normal factor cache replace forced full relinearization in GPU mode. CPU and ellipsoid alternatives retain their old policy.

CBS still constructs outgoing cavity graphs using ordinary GTSAM. The wrapper executes native asynchronous CUDA work across each pair's voxel levels; it does not implement GLIM's whole-graph ISAM2Ext GPU hook. The information scale applies consistently to the nonlinear error and the full Hessian, gradient and constant. The adapter accounts for the pinned GPU cost's unhalved residual-sum convention; both pose gradients were checked numerically.

The remaining 20–29 s is **not ten-second online output latency**. Every fresh solve still pays startup/exchange/preparation costs and executes 100 scheduled iterations. The ten-second input-capture queue and persistent-session limitations are unchanged. Faster kernels alone do not eliminate that scheduling overhead.

## Memory scope

Host memory is the peak simultaneous sum of RSS over the experiment's unique worker/native/verifier process tree, sampled every 0.2 s; shared pages can be counted more than once. GPU memory is sampled whole-device allocation, including contexts, display and any other processes. CPU controls observed 81 MiB; GPU controls peaked at 1,230 and 1,874 MiB. These are not per-factor allocations or a Jetson memory prediction. Jetson Orin NX needs a separate ARM64/architecture-87 build and measurement; the x86/architecture-120 workstation build is not deployable there.

## Verification and activation

- Native checks: 8 passed; one standalone distributed-PCM test was environment-gated and skipped.
- Actual separate-process GPU DDS tests: normal and reversed robot order passed, including PCM rejection, geometry ownership, payload validation, common-frame poses and cleanup.
- Focused Python checks: 22 passed for GPU selection, preserved CPU settings, geometry exchange and online scheduling.
- All eight real frozen-graph controls completed; all admitted factors passed final geometry checks.
- The normal installed launcher is checked separately after pinning the exact GTSAM library used to build it. The runtime hook prevents ROS Humble's different same-SONAME GTSAM library from being selected accidentally.

The canonical pipeline now selects `vgicp_gpu`. The complete previous profile remains in `research/configs/cpu_gicp_pipeline.yaml`; existing frozen CPU communication experiments and historical results are unchanged. Selecting GPU mode without CUDA fails explicitly, without a CPU fallback. This validation covers two three-robot graphs, not a new all-14 or full-S3E rerun.

## Retained evidence

- [Raw evaluated results](evidence/matched-v2/evaluated-results.json), [table CSV](table.csv), [configuration/source/binary hashes](evidence/matched-v2/source-hashes.json), [inspection hashes](evidence/inspection-SHA256SUMS.json), [deployed source hashes](deployment-hashes.json).
- [Factor/build specification](../../GLIM-GPU-CBS.md), [default profile](../../configs/default_pipeline.yaml), [explicit CPU profile](../../configs/cpu_gicp_pipeline.yaml).
- Remote full evidence: `/data3/mikexyl/swarm_s3e_ws/src/.ros2/gpu-cbs148-20260928/`. Geometry remains on 148. The `matched-v2` directory contains all eight runs, per-robot timing CSVs, registration support diagnostics and evo artifacts.
- Upstream references: [GLIM GPU configuration](https://github.com/koide3/glim/blob/master/config/config_global_mapping_gpu.json), [global mapping implementation](https://github.com/koide3/glim/blob/master/src/glim/mapping/global_mapping.cpp), [pinned gtsam_points](https://github.com/koide3/gtsam_points/tree/9d32e7dbecf6015560d84b4901d6b0a6f483ec46).
