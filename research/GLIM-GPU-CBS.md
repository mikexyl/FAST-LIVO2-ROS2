# GLIM GPU VGICP factors in distributed CBS

The `vgicp_gpu` registration option uses the unmodified CUDA `IntegratedVGICPFactorGPU`, `GaussianVoxelMapGPU`, and asynchronous factor-set implementation from the pinned gtsam_points v1.2.2 checkout. This is the GPU VGICP family used by GLIM. It does not replace CBS with GLIM or change EllipseLIO, MapClosures, point-GICP loop verification, PCM, or snapshot membership.

The full profile is [`configs/glim_gpu_pipeline.yaml`](configs/glim_gpu_pipeline.yaml), also selected by the canonical [`default_pipeline.yaml`](configs/default_pipeline.yaml) after validation on 2026-09-28. The previous full CPU profile is retained as [`cpu_gicp_pipeline.yaml`](configs/cpu_gicp_pipeline.yaml). Historical `gicp` and `ellipsoid` modes remain available. Selecting GPU mode on a CPU-only build fails explicitly; it never silently falls back to CPU.

## Geometry and objective

- Retain the existing prepared point budget, evidence exchange, endpoint ownership, initial/final support checks, and conservative information cap. Geometry still travels between robots as prepared point payloads.
- Upload endpoint points and covariances once per native session and reuse immutable GPU arrays and voxel maps across loop factors.
- Following GLIM's GPU profile, choose a base voxel resolution between 0.5 and 1.0 m from the endpoint cloud's bounded median-range sample (5 to 20 m transition). Create two levels with a scaling factor of 2. Use all the already-prepared points, without additional random sampling.
- Each CBS loop factor aggregates the two native GPU costs. The information cap applies to their combined Hessian, gradient and constant. Scale nonlinear error consistently.
- The pinned upstream GPU cost returns the unhalved sum of squared residuals. The bridge applies the one-half convention required by GTSAM's HessianFactor, as verified against both right-perturbed pose gradients.
- Asynchronous CUDA work is batched across the voxel levels of each pair. CBS uses its existing GTSAM optimizer; it does not use GLIM's ISAM2Ext whole-graph GPU hook. Exact repeated relative poses reuse a cached GPU linearization. Changed poses invalidate that cache. No approximate pose rounding is used.
- GPU mode uses GLIM's relinearization threshold 0.1, skip 1, and normal iSAM2 factor caching. It removes the registration-specific forced full solve. CPU/ellipsoid modes retain their prior behavior.
- Cloning and rekeying retain the scale and immutable endpoint geometry and create independent native CUDA state. The same nonlinear GPU factor participates in outgoing CBS marginalization graphs.

VGICP is a voxelized objective, so its optimum need not equal point GICP's optimum. The known-transform test records the residual pose error as well as verifying correction. Dataset comparisons must report quality alongside speed.

## Build and run

Use the matching `cbs_ros` development branch at
[`584e61b`](https://github.com/mikexyl/cbs_ros/commit/584e61bd4c87577006e9c971b851a6c2a9b7a1ff)
or a compatible descendant. The core `cbs` package is unchanged by this GPU
integration (validated at `bea1d35`). The experiment manifests preserve the
original pre-commit source hashes and native binary identities.

Build in a separate prefix so existing CPU runs retain their original binaries:

```bash
export CBS_UNDERLAY=/path/to/existing/cbs-underlay
export CMAKE_PREFIX_PATH=/path/to/gtsam-4.3-prefix:${CMAKE_PREFIX_PATH:-}
export S3E_REGISTRATION_FACTORS=ON S3E_REGISTRATION_CUDA=ON
export S3E_DPGO_PREFIX="$PWD/.ros2/dpgo-gpu"
export S3E_CUDA_ARCHITECTURES=native
bash FAST-LIVO2-ROS2/scripts/build_dpgo.sh
export CBS_OVERLAY="$S3E_DPGO_PREFIX-install"
```

Use a CUDA toolkit supporting the installed device. Workstation 148 uses an RTX 5080 and CUDA 12.9, architecture 120. Jetson Orin NX requires an ARM64 build for architecture 87 on its compatible JetPack toolkit; workstation binaries are not deployable to Jetson.

`dpgo.run` selects `.ros2/dpgo-gpu-install` for GPU factors unless `CBS_OVERLAY` explicitly identifies another isolated install. The launch wrapper and worker receive that same overlay. The GPU kernel does not remove the online scheduler's fixed 100-iteration settling budget or make it a persistent incremental session; those remain separately measured integration costs.

## Verification and retained outputs

The isolated 148 source/build and captures are under `/data3/mikexyl/swarm_s3e_ws/src/.ros2/gpu-cbs148-20260928/`. Synthetic checks cover both-pose gradients, scale including the constant term, rekeying, common-frame invariance, biased-loop correction, marginalization, rejected-loop exclusion and session mismatch. Distributed tests exercise different robot orderings. The frozen-graph timing driver alternates CPU/GPU/GPU/CPU on GRACO aerial and Library 1 while retaining identical odometry, constraints, point budgets and the 100-iteration schedule.

Global GPU memory samples are labelled as whole-device measurements; geometry allocation counters omit temporary buffers and driver overhead. Simultaneous process-tree RSS is measured independently. Neither is inferred from primitive counts.

The [validation report](figures/glim-gpu-cbs148-20260928/REPORT.md) contains the paired repeated timings, evo joint ATEs, memory scopes, test results and source/configuration hashes. Both repeats completed in approximately 20 s for GRACO A05/A07/A08 (CPU: 42 s) and 29 s for S3E Library 1 (CPU: 126 s). These are fresh-session frozen-final-graph solves, not online publication latency or a benchmark of GLIM itself. The 10-second online input cadence is unchanged.

## References

- [GLIM GPU global-mapping configuration](https://github.com/koide3/glim/blob/master/config/config_global_mapping_gpu.json): voxel levels, adaptive resolution, relinearization policy.
- [GLIM global-mapping implementation](https://github.com/koide3/glim/blob/master/src/glim/mapping/global_mapping.cpp): persistent ISAM2Ext, GPU factor creation and stream resources.
- [Pinned gtsam_points v1.2.2](https://github.com/koide3/gtsam_points/tree/9d32e7dbecf6015560d84b4901d6b0a6f483ec46): native CPU/GPU registration code. No upstream source modifications are made by this adapter.
