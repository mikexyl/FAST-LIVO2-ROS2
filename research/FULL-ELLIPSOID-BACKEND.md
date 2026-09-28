# Full ellipsoid backend (opt-in implementation)

Status: implementation and small-group correctness gates passed; broad benchmark
is running. No general benchmark improvement is claimed.
Historical odometry, accumulated-area membership, renderer defaults and outputs
are preserved. Source and experiment artifacts are isolated on workstation 148
under `.ros2/full-ellipsoid148-20260925` (initial pilot and renderer measurements)
and `.ros2/full-ellipsoid148-20260926` (reliable transport, uncapped controls).

The corrected build passed 8 native, 33 Python backend and 10 real DDS tests.
Seven additional native-payload fault/duplicate cases also passed. All eight
Laboratory 1 and GRACO G05/G06/A07 controls completed, including their final frame
and geometry checks. Results and limitations are retained in the
[control report](figures/full-ellipsoid148-20260926/README.md) and
[renderer measurements](figures/full-ellipsoid148-20260925/report/README.md).

## Configuration and evidence

Set `backend.ellipsoid_only: true`, registration `method: ellipsoid`, and
`backend.mapclosures.renderer: geometric_coverage` (default: `sampled_surface`).
Set `retrieval_channels: fullheight_and_layers` and enable multilayer matching.
`raster_device` selects `cpu` or `cuda`. No automatic CPU/GPU fallback is used.
The new mode reads only the native `ellipsoids` array. Points, point IDs and
member-scan clouds are never decoded. Snapshot source hashes and row membership
are retained; the existing frontend remains responsible for snapshot membership.

Renderers use all fitted primitives in the immutable area snapshot. Terrain uses
these primitive centers with the existing balanced low-surface estimator and
support gates. Verification and CBS share one saved subset: original primitives,
one per 0.4 m center voxel, capped deterministically at 50,000. Preparation stores
its policy and SHA-256; later stages validate and reuse it without preprocessing
again. Centers initialize the vertical translation. Terrain failure disables
only layered descriptors; it never disables the full-height channel.

The sampled renderer is retained as the matched experimental control. It expands
ellipsoid surfaces during rendering only. The geometric renderer and its remaining
backend stages do not construct a sampled surface or load a point cloud.

Canvas extent is a confound in the original four controls: legacy sampled density
uses tight occupied image bounds, whereas coverage uses the full area disk's
bounding grid. ORB rejects features near image borders. An additional Laboratory 1
pose-only control zero-padded the frozen sampled images to the full area canvas,
without changing intensities or thresholds. It connected all three robots with
68 loops, versus 25 loops/two components for tight sampled images and 66 loops/one
component for coverage. Therefore the original Laboratory connectivity improvement
cannot be attributed uniquely to geometric coverage. This extra control does not
establish results on other sequences; it is separate from the requested four-way
batch. Details: [canvas diagnostic](figures/full-ellipsoid148-20260926/canvas-diagnostic/README.md).

## Deployment target and performance priorities

The intended onboard target is **Jetson Orin NX 16 GB** (user clarification,
26 September 2026). Prioritize loop quality and sustainable processing latency;
a larger GPU allocation is acceptable when the complete pipeline fits with useful
headroom. Do not choose a renderer solely for its memory reduction.
The module's CPU and GPU share physical DRAM ([NVIDIA's explanation](https://forums.developer.nvidia.com/t/jetson-orin-nx-shared-memory-between-cpu-and-gpu/344025/2)).
The 16 GB is therefore not a separate graphics-memory budget. Desktop process RSS
and discrete-GPU measurements must not be added and presented as measured Jetson
usage. An onboard feasibility check must include odometry, accumulated maps,
retrieval, verification, CBS, middleware, and operating-system memory together.
Record JetPack/CUDA versions, power mode, clocks, sustained thermals, and actual
end-to-end update latency on that device; workstation 148 timings do not establish
Orin performance. No resource quota on workstation 148 is implied by this target.
The initial tiled CUDA renderer is a correctness implementation, not a demonstrated
real-time Orin implementation. On four frozen pilot snapshots, its measured median
rendering time was 0.33--17.43 s, versus 0.17--0.50 s for sampled surfaces on 148.
The CPU coverage reference was faster than this CUDA version and gave identical
image and ORB hashes on those four snapshots; it supplies broad-batch coverage
images. A subsequent implementation parallelizes individual primitive–pixel
contributions into bounded scratch storage and reduces them in the original order.
Three alternating fresh-process repeats on four frozen snapshots gave identical
image and ORB hashes. For Bob/G05/A07, total rendering fell from 18.04/9.47/12.05 s
to 3.15/3.60/4.38 s, with median sampled process GPU memory increasing from 268 to
272 MiB. Other CPU benchmarks were active; this is a preliminary engineering
comparison, not an isolated system benchmark or an Orin performance claim.
The sampled renderer remains faster in the earlier isolated measurement.
New builds default to parallel coverage; pass `pixel` as the build script's second
argument to reproduce the original implementation. Both use the same coverage
function and fixed thresholds. The broad accuracy batch keeps its frozen binaries.
Faster GPU execution remains an engineering item. Thresholds, axes, and coverage
accuracy must not be relaxed to hide that limitation.

## Coverage renderer

A native ellipsoid has center c, positive semiaxes a, and orthonormal basis B.
After the IMU gravity transform, its geometric shape matrix is
`S = R B diag(a²) Bᵀ Rᵀ`. S describes the solid boundary; it is not a noise covariance.
The projection is computed from analytic conic cross-sections. At each integration
abscissa x, the conditional yz ellipse is intersected with the terrain slab
`lo <= z - terrain(x,y) <= hi`. Its y extrema define the projected interval.
That interval is intersected with the pixel and horizontal area disk before its
length is integrated over x. No axis is enlarged. Analytic slab bounds narrow
integration support even for subpixel primitives.

The CPU reference and tiled CUDA implementation share exact conic rectangle
integrals when the primitive lies inside its height band and area disk, and
deterministic adaptive Gauss integration for the remaining intersections. Each primitive contributes its fraction of covered pixel area;
contributions add, including overlapping primitives. Each nonempty image is divided
by its maximum, thresholded using the existing normalized density threshold, and
converted to 8-bit grayscale. This is a **new additive-coverage intensity**, not an
exact reproduction of sampled point-density normalization. Empty images stay zero.
A primitive intersecting two bands contributes to both, even if its center lies in
only one band. Their projected coverage need not sum to the full-height image.

Tiles have 8×8 pixels. Fixed batches of at most 4096 primitives bound tile-reference
storage independently of snapshot size. Temporary arrays scale with the input
primitives, fixed batch size and image dimensions, never a 3-D cloud/voxel grid.
CUDA calls synchronize before returning timing data; reported allocation bytes
are renderer allocations, not a claim about total process/GPU memory.
The parallel CUDA implementation adds at most 16,384 tile references × 64 pixels
× 8 bytes = 8 MiB of scratch, plus one integer tile ID per current-batch reference.
It processes layers sequentially and preserves primitive summation order without
floating-point atomics. Empty grids, partial tiles, multiple scratch batches and
multiple primitive batches are covered by the CUDA/CPU regression tests.

`MapClosures.describe_image(image, lower_bound, resolution, ground)` shares the
existing ORB extraction, self-similarity filtering and row/column keypoint
conversion with `describe(points)`. Lower bounds are integer grid coordinates;
resolution must agree with the matcher. Metric loop recovery retains the gravity
transforms. Full-height and layered HBST channels remain distinct. Eligible
candidate endpoints are unioned; the existing MapClosures budget reserves one
slot per channel and fills remaining slots by native inliers. An endpoint is
verified once, with its strongest initialization first and alternatives tried
only after rejection. Candidate/channel and initialization outcomes are recorded.

## Live CBS primitive factor

Canonical endpoint i is target; j is source. With `T = Xi⁻¹ Xj`, the residual is
`r = Pi (T cj - ci)` for the nearest target center inside the existing radius.
Native inverse-axis saliencies recover the plane/line/point projector Pi. No
surface expansion, point covariance or Gaussian registration model is introduced.
Immutable per-endpoint arrays, projectors and center KD-trees are shared by factors.

Both poses use GTSAM's right tangent, rotation then translation. With q=T cj:
`Ji = Pi [skew(q), -I]`, `Jj = Pi R [-skew(cj), I]`.
Correspondences refresh at every error evaluation and linearization. The objective
is the sum of radial Huber costs divided by the **fixed source count**. An unmatched
primitive contributes `Huber(correspondence_radius)`, so losing correspondences
cannot turn its contribution into zero. The IRLS Gaussian contains both pose
blocks, their cross block, the gradient and the objective's constant term.

Only PCM-retained loops are considered, at the verified relative pose. Support,
bidirectional overlap, residual RMSE and observability gate factor admission.
The companion pose constraint stays in the graph. A fixed generalized-eigenvalue
curvature cap against its conservative information scales nonlinear error and
all Gaussian terms equally. Clone/rekey retains scale, source/target roles and
immutable evidence. CBS local optimization and outgoing cavity marginalization
use this nonlinear factor. Unsupported linearizations are counted; failed final
geometry rejects a successful-run verdict.

The loop measurement and primitive factor **reuse the same evidence**. They are
not independent measurements; the information cap is conservative regularization,
not an independence claim.

`dpgo.registration_factors.factor: ellipsoid` enables this factor. `gicp` keeps the
existing implementation. Verifier/factor types and preparation policies must
agree. Ellipsoid post-PCM payloads use manifest schema 2 with a
`native_ellipsoids_v1` tag, 15 float64 values per primitive (center, axes, row-major
basis), session/configuration hashes, endpoint/frame/anchor timestamps and payload
hashes. Ownership, immutable duplicates, endpoint request restrictions and the
supplier-ready barrier are retained. Centralized mixed PGO remains GICP-only;
this implementation targets distributed CBS.

The first mixed-GRACO pilot exposed an existing belief transport defect: the
client silently discarded responses taking more than one second, although the
server could already mark their incremental beliefs as sent. Native geometry
made those service times long enough to lose anchor information. The corrected
adapter retains the request until a response arrives, with an explicit 60-second
transport deadline that fails the run instead of silently dropping evidence.
This does not change the CBS objective, PCM decisions, iteration budget, or
geometric thresholds. Late-response and missing-response regressions are tested;
connected-loop output-frame consistency is also checked before success.

## Staged validation and controls

1. Analytic geometry/oracle and CPU/CUDA agreement; empty/invalid inputs;
   image origin/metric pose recovery; no-point-array preparation; channel budget.
2. Both-pose finite differences, common-frame invariance, full scaling, clone/rekey,
   correspondence loss, biased-loop correction, cavity factors and PCM exclusion.
3. Frozen Lab1 and GRACO G05/G06/A07 paired rendering/match gallery before graph runs.
4. Four matched sampled/raster × pose-only/live-factor controls on the two small
   groups, then the same controls on GRACO14 and the selected twelve S3E groups.
   Odometry, terrain, channel policy, thresholds and evidence stay fixed.

Report each failure without tuning thresholds. Evaluate trajectories only with
Evo 1.36.5, 50 ms association, rigid alignment without scale. Preserve Laboratory
GT limitations. Report component membership and joint/component ATE rather than
assigning one aligned ATE to disconnected maps. Keep bulk captures on 148.
Measure rendering, ORB, verification and CBS separately. Renderer microbenchmarks
alternate repeated runs; use synchronized GPU timing, fresh-process RSS/GPU
measurements, simultaneous process-tree memory and actual transmitted bytes.
Do not sum unrelated historical peaks or infer savings from primitive counts.

## Deferred: Gaussian splatting

Geometric coverage is the implemented renderer. Gaussian splatting is a future
research direction, with unresolved questions: which kernel width is meaningful;
how opacity should combine; how height-band integrals should be normalized; and
whether native tensor semiaxes can define a meaningful kernel at all. Native axes
currently describe fitted geometry, not a calibrated probability density.
