# Square 2: raw versus ellipsoid BEVs

Both branches use identical fresh EllipseLIO odometry, keyframes and raw geometric
verification evidence. MapClosures runs in three isolated robot workers with causal
simulated delivery; centralized GTSAM 4.2 GNC-TLS and a selected-inlier LM refit
optimize the saved pose graphs. These results use pose factors, without CBS or live
registration factors in the optimizer.

## Trajectory ATE

Position RMSE in metres, evaluated entirely through evo 1.36.5.

| Configuration | Combined | Alpha | Bob | Carol |
|---|---:|---:|---:|---:|
| Raw odometry (independent robot alignment) | unavailable | 0.5138 | 0.5974 | 0.2014 |
| Raw BEV + PGO | 0.4703 | 0.4612 | 0.6332 | 0.2358 |
| Ellipsoid BEV + PGO | 0.4641 | 0.4583 | 0.6236 | 0.2316 |

Ellipsoid combined ATE is **0.0061 m (1.30%) lower**
than raw BEV ATE in this one paired run. This is not a repeated-trial estimate.

Connected trajectories receive one shared SE(3) fit across all robots, with no
scale fitting or subsequent per-robot realignment. Disconnected components are
independently anchored and aligned; no combined three-robot ATE is reported for
a disconnected graph. Raw-odometry diagnostics use separate robot alignments.
The timestamp tolerance is 0.05 s. Placeholder GT orientations are unused and the
antenna lever arm is uncorrected. Ground truth enters only after optimization.

![Trajectories](trajectories.png)

![Individual ATE](ate.png)

## Loops and computation

| BEV input | Verification attempts | Accepted | Selected intra | Selected inter | Components |
|---|---:|---:|---:|---:|---:|
| raw | 104 | 26 | 1 | 25 | 1 |
| ellipsoid | 313 | 30 | 6 | 24 | 1 |

Verification attempts count candidate cloud pairs, not individual ORB matches.
Accepted registrations and GNC-selected graph factors are distinct decisions.
A larger total loop count need not imply more inter-robot matches.

| BEV input | Detection wall time | PGO wall time | Serialized exchange | Verification task time sum |
|---|---:|---:|---:|---:|
| raw | 24.22 s | 0.346 s | 36.20 MiB | 11.57 s |
| ellipsoid | 51.71 s | 0.344 s | 46.08 MiB | 42.79 s |

Detection time excludes preparation. Verification task times include evidence
packing and registration; their sum can exceed wall time because robot tasks
overlap. It is not a pure GICP-kernel timing. Delivery is reliable/unrestricted.
MapClosures and small_gicp use CPU; CUDA accelerates ellipsoid surface sampling.

### Descriptor workload

| Robot | Keyframes | Raw retained ORB | Ellipsoid retained ORB | Raw describe | Ellipsoid describe | CUDA sampling |
|---|---:|---:|---:|---:|---:|---:|
| Alpha | 317 | 56,953 | 92,831 | 3.31 s | 37.53 s | 81.94 s |
| Bob | 274 | 59,307 | 94,701 | 2.46 s | 34.75 s | 89.85 s |
| Carol | 270 | 80,841 | 103,954 | 3.33 s | 48.46 s | 97.17 s |

ORB counts sum retained features over all keyframes, including repeated
observations of the same structures; they are not correspondence counts.
Describe times cover native MapClosures ground alignment, density construction
and ORB extraction. They exclude raw-submap assembly, ellipsoid reconstruction,
surface sampling, debug rendering and artifact I/O.

Serial odometry replay: **16.92 min**. Preparation of both
branches: **13.36 min**, including
**4.48 min** of CUDA sampling.
Preparation overlaps later robot replays, so those stage totals are not additive.

Native descriptor inputs average **77,811 raw points** versus
**1,327,795 sampled ellipsoid points** per keyframe. Thus construction
time includes substantially different point-processing workloads; it should
not be interpreted as feature-matching speed for equal inputs.

## Fixed settings and provenance

Keyframes: {'Alpha': 317, 'Bob': 274, 'Carol': 270}. Dense frames: {'Alpha': 2346, 'Bob': 2430, 'Carol': 2511}.

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

## Validation and loop quality

Frozen PGO was rerun for both branches. All 1,728 watched
odometry/keyframe/descriptor input hashes stayed unchanged. Pose differences:
ellipsoid: 0, raw: 0.
The regression selection passed 29 tests.

| BEV input | GT-checkable loops | Flagged distance discrepancies >2 m |
|---|---:|---:|
| ellipsoid | 23/30 | 0 |
| raw | 24/26 | 0 |

Checks compare constraint translation magnitude with GT endpoint separation.
Both endpoints need GT interpolation across gaps of at most two seconds. Flags
are position-only diagnostics, not proof of full 6-DoF outliers. These post hoc
checks did not select or remove factors. Intra/inter-robot mixtures differ.

## Replay failures and retries

Carol: the first replay failed after 34.67 s wall time.
IMU/scan synchronization stalled and raw-cloud queue safety bound stopped the first replay. Original bag maximum IMU gap was 15.94 ms; no interval exceeded 50 ms.
The retained full replay uses 0.5x playback, with estimator and
loop parameters unchanged. Both BEV branches share this same replay.
The failed attempt and original-bag IMU audit are retained in provenance.

## Retained artifacts

Full poses, constraints, evo ZIPs and shared-alignment evidence, graph residuals
and weights, source/configuration provenance and figures are retained.
The compact [Rerun recording](trajectories.rrd) shows both trajectory sets and GT.

Cleanup removed **3.10 GiB** of generated MCAP, ellipsoid deltas
and geometry stages. Frozen PGO remains reproducible from the retained evidence;
reconstructing descriptors requires replay. The original dataset is unchanged.
