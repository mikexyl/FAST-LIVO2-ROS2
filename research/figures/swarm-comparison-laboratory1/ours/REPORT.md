# Laboratory 1: raw versus ellipsoid BEVs

Both branches use identical fresh EllipseLIO odometry, keyframes and raw geometric
verification evidence. MapClosures runs in three isolated robot workers with causal
simulated delivery; centralized GTSAM 4.2 GNC-TLS and a selected-inlier LM refit
optimize the saved pose graphs. These results use pose factors, without CBS or live
registration factors in the optimizer.

**Raw BEVs: 3 components, 5 selected loops (0 inter-robot). Ellipsoid BEVs: 3 components, 35 selected loops (0 inter-robot).**
Full-trajectory accuracy remains unmeasured because timestamped trajectory GT is unavailable.

## Trajectory ATE

Position RMSE in metres, evaluated entirely through evo 1.36.5.

| Configuration | Combined | Alpha | Bob | Carol |
|---|---:|---:|---:|---:|
| Raw odometry (independent robot alignment) | unavailable | unavailable | unavailable | unavailable |
| Raw BEV + PGO | unavailable | unavailable | unavailable | unavailable |
| Ellipsoid BEV + PGO | unavailable | unavailable | unavailable | unavailable |

**Full-trajectory ATE is unavailable.** Evo could not produce a trajectory
ATE; association and alignment status are saved in the evo evaluation files.
Retrieval
recall and GT-based loop accuracy are also unavailable. Connectivity, accepted
loops, graph residuals and map appearance do not establish an accuracy improvement.
The plots use independent PGO component frames, without GT alignment.

The supplied Laboratory records describe only endpoints labeled 0 and 1;
those labels are not sensor epoch timestamps. They have not been remapped
to the trajectory start/end times or interpolated into an artificial path.

With timestamped GT, connected trajectories receive one shared SE(3) fit across all robots, with no
scale fitting or subsequent per-robot realignment. Disconnected components are
independently anchored and aligned; no combined three-robot ATE is reported for
a disconnected graph. Raw-odometry diagnostics use separate robot alignments.
The timestamp tolerance is 0.05 s. GT orientations are unused and the
antenna lever arm is uncorrected. Ground truth enters only after optimization.

![Trajectories](trajectories.png)


## Loops and computation

| BEV input | Verification attempts | Accepted | Selected intra | Selected inter | Components |
|---|---:|---:|---:|---:|---:|
| raw | 7 | 7 | 5 | 0 | 3 |
| ellipsoid | 228 | 38 | 35 | 0 | 3 |

Verification attempts count candidate cloud pairs, not individual ORB matches.
Accepted registrations and GNC-selected graph factors are distinct decisions.
A larger total loop count need not imply more inter-robot matches.

| BEV input | Detection wall time | PGO wall time | Serialized exchange | Verification task time sum |
|---|---:|---:|---:|---:|
| raw | 7.32 s | 0.345 s | 1.42 MiB | 0.44 s |
| ellipsoid | 26.73 s | 0.438 s | 7.48 MiB | 12.52 s |

Detection time excludes preparation. Verification task times include evidence
packing and registration; their sum can exceed wall time because robot tasks
overlap. It is not a pure GICP-kernel timing. Delivery is reliable/unrestricted.
MapClosures and small_gicp use CPU; CUDA accelerates ellipsoid surface sampling.

### Descriptor workload

| Robot | Keyframes | Raw retained ORB | Ellipsoid retained ORB | Raw describe | Ellipsoid describe | CUDA sampling |
|---|---:|---:|---:|---:|---:|---:|
| Alpha | 295 | 1,282 | 9,956 | 0.58 s | 10.25 s | 33.66 s |
| Bob | 223 | 5,616 | 46,973 | 0.79 s | 16.21 s | 40.01 s |
| Carol | 286 | 789 | 16,555 | 0.50 s | 7.69 s | 24.11 s |

ORB counts sum retained features over all keyframes, including repeated
observations of the same structures; they are not correspondence counts.
Describe times cover native MapClosures ground alignment, density construction
and ORB extraction. They exclude raw-submap assembly, ellipsoid reconstruction,
surface sampling, debug rendering and artifact I/O.

Serial odometry replay: **14.96 min**. Preparation of both
branches: **8.92 min**, including
**1.63 min** of CUDA sampling.
Preparation overlaps later robot replays, so those stage totals are not additive.

## Fixed settings and provenance

Keyframes: {'Alpha': 295, 'Bob': 223, 'Carol': 286}. Dense frames: {'Alpha': 2902, 'Bob': 2912, 'Carol': 2919}.

Native ellipsoid centers, geometric semi-axes and axis directions define the
rendered surfaces: 0.125 m nominal sampling, 0.25 m voxel centroids, deterministic
CUDA accumulation at 0.1 micrometre resolution. Earlier validation on three real
maps reproduced CPU density pixels and ORB descriptors exactly. The ellipsoid
map is persistent and causal, cropped at 80 m. Raw BEVs use trailing 5-second
submaps with the same crop. Different map histories remain part of the comparison.

MapClosures uses 0.5 m pixels, density threshold 0.05, Hamming threshold 50 and
more than five RANSAC inliers. Retrieval keeps a top-20 shortlist with one selected
LiDAR candidate per query, a two-second cooldown and 30-second same-robot exclusion.
Common raw-evidence GICP uses the existing 0.35 m RMSE, 30% overlap and observability
gates. No thresholds were tuned on this sequence. Full settings are in inputs.json.

The fixed 1 m / 10 degree / 2 s schedule comes from saved **FAST-LIVO2**
raw trajectories, without loop labels or ground truth. Both branches use fresh
EllipseLIO states and the same actual export timestamps. A requested snapshot
before filter initialization is associated with the first valid export, with
a two-second startup allowance; later associations retain the 250 ms guard.
Observed association delays and merged schedule entries are retained in the audit.

## Retained artifacts

Full poses, constraints, evo association-status evidence, graph residuals
and weights, source/configuration provenance and figures are retained.
The compact [Rerun recording](trajectories.rrd) shows trajectories and maps in separate component frames.

## Map and BEV inspection

![Corrected component maps](maps.png)

Both maps are rebuilt from the same raw keyframe scans, using each graph's
optimized poses. The plan view shows a one-metre height slice around the
median trajectory height of each component; Rerun retains the full-height maps.
Disconnected components remain separate. No endpoint-GT
alignment is applied, and visual sharpness is not a ground-truth accuracy score.

![BEV examples and ORB features](bevs.png)

The middle keyframe of each robot is selected independently of loop results.
