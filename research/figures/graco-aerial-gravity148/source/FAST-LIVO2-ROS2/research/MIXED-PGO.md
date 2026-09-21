# Centralized pose and point-cloud factors

The same live factors are also available in [distributed CBS](RESULTS-CBS-REGISTRATION.md).
The commands and initialization below describe the centralized option.

The optional `pgo.registration_factors` mode adds binary
[`gtsam_points::IntegratedGICPFactor`](https://github.com/koide3/gtsam_points/blob/9d32e7dbecf6015560d84b4901d6b0a6f483ec46/include/gtsam_points/factors/integrated_gicp_factor.hpp)
costs to the centralized pose graph. Both endpoint poses remain variables.
Point correspondences and covariance combinations are refreshed at every
linearization. This is joint point-cloud/pose optimization, rather than another
pairwise registration followed by a fixed relative-pose measurement.

The existing GNC-TLS pose solver first selects loop inliers and initializes
robot alignment. Its selected odometry factors, loop pose factors, and rebuilt
component anchors remain in the mixed graph. A GICP factor is considered only
for each selected loop pair. Rejected loops cannot restore connectivity through
registration. CBS, distributed PCM and FAST-LIVO2 are unchanged by this option.

## Build and run

From the workspace `src` directory:

```bash
bash FAST-LIVO2-ROS2/scripts/build_mixed_pgo.sh
FAST-LIVO2-ROS2/scripts/s3e_experiment.sh run --stage pgo \
  --config FAST-LIVO2-ROS2/research/configs/square1-mixed-pgo.yaml \
  --input-run .ros2/megaloc-mapclosures-cbs/run-af384eeda86f7f19.json --resume
```

The build downloads upstream v1.2.2, commit
`9d32e7dbecf6015560d84b4901d6b0a6f483ec46` (MIT), into `.ros2/deps/gtsam_points`.
It compiles the unmodified upstream CPU GICP, cloud, covariance, normal and
nearest-neighbor components into the adapter executable. Upstream extended
ISAM2/LM implementations are not needed: the installed native GTSAM batch LM
optimizes this graph. Set `S3E_GTSAM_PREFIX` to a compatible GTSAM 4.3 prefix;
the default reuses the same installation as CBS.

The machine's GTSAM 4.3 development installation predates the upstream
`make_shared.h` convenience header. The adapter includes a compatibility header
that supplies the C++17 `std::make_shared` alias and Eigen allocation macro when
the real header is absent. It does not change GTSAM or point-factor code.
The native executable runs separately from Python GTSAM 4.2 to avoid mixing ABIs.
The build manifest records source, executable and loaded GTSAM library hashes;
changed adapter sources require a rebuild. This first implementation uses CPU
GICP, with four threads shared across factors, and no CUDA.

## Geometry and weighting

The inputs are the existing **full-LiDAR trailing five-second submaps**, expressed
in each keyframe's IMU/body frame. They are the same local maps used for loop
verification; the camera-visible colored cloud is not used. Stored cloud-frame
metadata is checked before loading. The adapter never uses GT.

For a factor on `(i,j)`, i is the target and j is the source:
`T_i_j = inverse(T_world_i) * T_world_j`. Source points are transformed into
the target frame. Tangent ordering is `[rx, ry, rz, tx, ty, tz]`.

Defaults: 0.5 m voxel centroids, at most 12,000 points per submap, 80 m range,
20 covariance neighbors, upstream covariance eigenvalues `[0.001, 1, 1]`,
1.5 m correspondence trimming, at least 100 inliers, bidirectional overlap
at least 0.3, normalized point-to-plane observability at least `1e-4`, and
condition number at most `1e6`. Globally planar/linear clouds also fail an
extent-rank check (relative eigenvalue threshold `1e-8`), which prevents
ambiguous normals in repeated/collinear samples from creating false support.
Covariances and search trees are reused across
incident factors. Only selected loop endpoints are loaded.

The geometric quality test runs at the pose-only initialization. A low-overlap
or degenerate pair is reported and contributes no registration factor; its
already-selected pose factor remains. Quality is checked again after mixed
optimization. A loss of final geometric support fails the stage and preserves
diagnostics instead of publishing a successful cache. No added matching factors
is reported explicitly through `registration_factor_count=0`.

Raw GICP cost grows with point count. For each pair the adapter computes the
initial source-pose Hessian `H` and its largest generalized eigenvalue against
the verified loop information `Omega`. It freezes the scalar
`s = max_information_ratio / lambda_max(H, Omega)`, so initially
`s*H <= max_information_ratio*Omega`. Default ratio is 1. Both nonlinear cost
and the entire Hessian/gradient linearization are scaled by s. The scale is
not recomputed during optimization, and the initial curvature bound is not a
guarantee about later Hessians.

The pose and GICP factors reuse LiDAR evidence, and neighboring submaps can
overlap. This weighting is conservative regularization, not a claim that all
points or both factor families are statistically independent. Native GICP
costs are not interpreted as calibrated six-dimensional chi-square statistics.
GNC selection is fixed before mixed LM; there is no point-factor GNC wrapper.

## Outputs and evaluation

`graph.json` contains corrected poses, pose residuals, registration diagnostics
and the mixed objective. `T_pose_only_body` retains the native pose-only seed
for comparison. `pose-only-graph.json` preserves the Python GNC result;
`native-result.json` records both solutions using the same native GTSAM version.
`registration.json` records factor counts, initial/final overlap, observability,
scales, linearization counts, runtime, peak RSS and input cloud hashes.
Temporary binary clouds are removed after native execution. `native-input.json`
records the exact exchange layout; its temporary payload paths are intentionally
not a standalone cache. Reproduce them from the recorded frozen NPZs and settings.

The ordinary `--stage evaluate` supports mixed graphs. For a compact comparison
with evo trajectories and sampled map figures, use the newly produced registry:

```bash
PYTHONPATH=FAST-LIVO2-ROS2/research .ros2/research-venv/bin/python \
  -m s3e_pipeline.mixed_pgo_report --input-run <mixed-run.json> \
  --output FAST-LIVO2-ROS2/research/figures/mixed-pgo-square1
```

Evaluation delegates association, rigid alignment and translation APE entirely
to evo 1.36.5. Each method uses one shared SE(3) alignment per connected
component, with no scale fitting or per-robot realignment. Supplied S3E
orientations are ignored. ATE is a position metric; sampled map plots are
qualitative and do not establish map accuracy.

Validation:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 \
  .ros2/research-venv/bin/python -m pytest \
  FAST-LIVO2-ROS2/research/tests/test_mixed_pgo.py -q
```

The tests exercise the actual native factors on known transforms in both
endpoint directions, an independently anchored component, planar degeneracy,
zero overlap, reciprocal duplicates, GNC-rejected loops, unchanged input
clouds, configuration validation and the disabled path.
