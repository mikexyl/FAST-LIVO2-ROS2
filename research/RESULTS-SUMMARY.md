# Multi-robot FAST-LIVO2: results summary

Updated 2026-09-14. This document consolidates the completed **MegaLoc +
MapClosures + distributed CBS** experiments for Alpha, Bob and Carol.

The system connected all three robots on **Square 1, Square 2, Library 1,
Campus Road 1 and Playground 2**, with combined position ATE RMSE of
**1.2147 m, 0.5340 m, 1.4172 m, 1.4707 m and 0.2941 m**, respectively.
Laboratory 1 achieved partial connectivity and has
no usable full-trajectory ground truth. Playground 1 is an excluded historical
failure with documented odometry configuration and startup problems.

These are individual saved runs on different sequences. They establish the
observed outcomes; they are not repeated-trial performance estimates or a
controlled comparison of estimator settings.

**PCM update:** Current configurations add distributed PCM before CBS. The
historical table below remains unchanged. A separate
[Square 1 frozen-constraint validation](RESULTS-PCM-CBS.md) retained 50/50 loops
and produced 1.2181 m combined ATE; PCM added 65,780 CDR bytes and took
1.3–2.2 ms computation per robot. Its batch execution differs from the earlier
incremental CBS run, so the ATE difference is not attributed to PCM.

## System and evaluation

The path used for the historical experiments below was:

`FAST-LIVO2 exports → keyframes/local submaps → independent MegaLoc and MapClosures retrieval → native MapClosures pose initialization → small_gicp GICP acceptance → per-robot CBS → corrected trajectories/maps`

MegaLoc uses pretrained CUDA inference. MapClosures uses LiDAR density images,
ORB features, HBST matching and native 2D RANSAC poses. A visual-only proposal
also needs a native two-map MapClosures pose before GICP; failure to obtain
that pose is recorded explicitly. The retained setup uses 1 m / 10° / 2 s
keyframes, trailing five-second submaps, an 80 m range crop, MegaLoc cosine
similarity ≥0.50, top-20 retrieval and a 30 s same-robot exclusion. Each
requester selects at most one proposal per branch, with cooldown and pair
deduplication. [Method details](METHODS.md).

Odometry is replayed serially, with one GPU model worker at a time. Three
robot front ends and three CBS optimizers subsequently exchange ROS2 Fast DDS
messages on one host using frozen artifacts. **Offline distributed replay is
validated; simultaneous live sensor-to-model operation and deployment across
physical robots have not been demonstrated.** Corrections affect saved poses
and maps, with no feedback into FAST-LIVO2. [Distributed implementation](DPGO.md).

All CBS ATE values below use **evo 1.36.5**, nearest timestamp association
within 0.05 s, no interpolation or time offset, and **one shared SE(3)
alignment per connected component, with scale fixed to one**. Individual
robots receive no additional fit. Combined RMSE pools matched position errors;
it is not the arithmetic mean of robot RMSEs. Missing GT is excluded, supplied
placeholder orientations are unused and the GNSS antenna lever arm is
uncorrected. Ground truth is available only to evaluation.

## CBS trajectory results

All ATE values are translation RMSE in metres. “Connected” means all three
robots share one estimated component.

| Sequence | Outcome | Alpha | Bob | Carol | Combined | Matched GT positions |
|---|---|---:|---:|---:|---:|---:|
| [Square 1](RESULTS-CBS.md) | Connected | 1.5182 | 1.0740 | 0.9827 | **1.2147** | 1,176 |
| [Square 2](RESULTS-SQUARE2-CBS.md) | Connected | 0.6261 | 0.6335 | 0.3020 | **0.5340** | 694 |
| [Library 1](RESULTS-LIBRARY1-CBS.md) | Connected | 1.0239 | 1.5216 | 1.6474 | **1.4172** | 1,151 |
| [Campus Road 1](RESULTS-CAMPUS-ROAD1-CBS.md) | Connected | 1.5669 | 1.4956 | 1.3486 | **1.4707** | 1,905 |
| [Playground 2](RESULTS-PLAYGROUND2-CBS.md) | Connected | 0.2446 | 0.3438 | 0.2856 | **0.2941** | 655 |
| [Laboratory 1](RESULTS-LABORATORY1-CBS.md) | Alpha isolated; Bob–Carol connected | N/A | N/A | N/A | N/A | 0 timestamp matches |
| [Playground 1](RESULTS-PLAYGROUND1-CBS.md) | **Excluded failure**, connected graph | 28.0625 | 26.4365 | 22.0118 | **25.6036** | 862; Bob starts at 22 s |

Square 1 uses the later evo evaluation of frozen CBS trajectories. Its older
**1.2112 m** score used interpolated GT and is superseded for cross-sequence
reporting. Some archived Square 1 figures retain that older evaluation.
[Current full-precision evo results](figures/cbs-square1/evo_ate.json).

Square 2, Library 1 and Playground 2 use `vio.img_point_cov=100` for all robots, following the
Playground 1 diagnosis. Earlier sequence runs used the S3E setting of 1000.
Detection and CBS settings are unchanged. Their lower errors cannot be
attributed to this parameter alone because the sequences also changed.

**Laboratory 1 is not Library 1.** The indoor Laboratory sequence was run
after an incorrect name substitution. The requested outdoor Library 1 has
now been downloaded, verified and evaluated separately. **Correction:**
Square 2 is available in the official release;
the earlier claim of unavailability was incorrect. It has now been downloaded,
verified and evaluated.

## Raw FAST-LIVO2 odometry ATE

These are translation ATE RMSE values in metres **before loop closure and
CBS**, evaluated with **evo 1.36.5**. Each robot receives an independent
SE(3) alignment to its available position GT, with no scale fitting,
interpolation or time offset and a 0.05 s association tolerance. This differs
from the shared component alignment used for CBS above; subtracting these
raw scores from CBS scores does not measure an optimization improvement.

| Sequence | Alpha | Bob | Carol | GT matches: Alpha / Bob / Carol |
|---|---:|---:|---:|---:|
| [Square 1](figures/cbs-square1/raw_odometry.json) | 1.3058 | 0.8698 | 0.6335 | 384 / 454 / 338 |
| [Square 2](figures/cbs-square2/raw_odometry.json) | 0.5441 | 0.5772 | 0.2179 | 195 / 245 / 254 |
| [Library 1](figures/cbs-library1/raw_odometry.json) | 1.3011 | 1.9088 | 1.4021 | 397 / 378 / 376 |
| [Campus Road 1](RESULTS-CAMPUS-ROAD1-CBS.md) | Not retained | Not retained | Not retained | — |
| [Playground 2](figures/cbs-playground2/raw_odometry.json) | 0.1941 | 0.3302 | 0.2548 | 219 / 218 / 218 |
| [Laboratory 1](RESULTS-LABORATORY1-CBS.md) | N/A | N/A | N/A | No usable timestamped trajectory GT |
| [Playground 1, excluded](figures/cbs-playground1/figures.json) | 28.7455 | 25.9419 | 0.3149 | 292 / 275 / 295 |

Square 1 was evaluated from the exact saved odometry used by the reported
CBS run, without replaying SLAM or changing its input artifacts. Native evo
archives and matched trajectories are retained alongside its linked numeric
results. Campus Road 1's retained evidence does not contain raw ATE scores;
its timestamped raw poses were removed during the requested cleanup. A new
odometry replay would be needed to fill that row. Laboratory 1 has only
start/end motion-capture records, so full-path ATE remains unavailable.

The excluded Playground 1 row is the historical full run with Bob starting
at 22 s and visual variance 1000. The bounded startup diagnostics below are
separate results. Square 2, Library 1 and Playground 2 use visual variance
100; the other reported full runs use 1000.

## Loop detection and communication

Branch columns are mutually exclusive attributions from one fused run.
“Both” means the same accepted constraint was independently proposed by both
retrieval branches and counted once. These columns are not separate ablations.

| Sequence | Keyframes | Verifications | Accepted | Inter / intra | MapClosures only | Both | MegaLoc only | Yield |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| Square 1 | 1,550 | 175 | 50 | 50 / 0 | 29 | 12 | 9 | 28.57% |
| Square 2 | 861 | 154 | 37 | 37 / 0 | 25 | 8 | 4 | 24.03% |
| Library 1 | 1,474 | 1,492 | 1,049 | 1,010 / 39 | 240 | 546 | 263 | 70.31% |
| Campus Road 1 | 2,890 | 595 | 156 | 116 / 40 | 35 | 99 | 22 | 26.22% |
| Playground 2 | 1,291 | 525 | 165 | 144 / 21 | 79 | 31 | 55 | 31.43% |
| Laboratory 1 | 804 | 117 | 6 | 3 / 3 | 4 | 1 | 1 | 5.13% |
| Playground 1, excluded | 1,097 | 885 | 377 | 370 / 7 | 84 | 151 | 142 | 42.60% |

The LiDAR branch makes an independent contribution. On Square 1, all **29
MapClosures-only loops** were below MegaLoc's 0.50 similarity threshold. On
Playground 2, **79 of 165** accepted loops were MapClosures-only. The original
Square 1 native inspection reproduced density images, ORB/HBST evidence and
RANSAC results for **211 local maps and 175 verified pairs** in Rerun.
[Inspection details](INSPECTION.md).

| Sequence | Recall@1 / @5 / @20 | Accepted pairs within 10 m / assessable | Detection + CBS (s) | Loop exchange (MB) | CBS exchange (MB) |
|---|---:|---:|---:|---:|---:|
| Square 1 | 78.07% / 86.10% / 88.77% | 41 / 49 | 57.99 | 180.404 | 0.338 |
| Square 2 | 75.45% / 82.04% / 87.43% | 23 / 33 | 45.76 | 109.519 | 0.254 |
| Library 1 | 97.70% / 99.75% / 99.92% | 772 / 798 | 181.41 | 514.355 | 4.282 |
| Campus Road 1 | 88.12% / 90.20% / 91.23% | 112 / 112 | 176.64 | 396.763 | 1.087 |
| Playground 2 | 57.92% / 72.43% / 78.89% | 107 / 165 | 95.83 | 211.033 | 0.675 |
| Laboratory 1 | N/A | N/A | 23.46 | 61.674 | 0.025 |
| Playground 1, excluded | 97.90% / 100.00% / 100.00% | 362 / 377 | 113.10 | 311.532 | 5.227 |

Recall uses causal position-proximity labels where GT is available. Nearby
origins do not guarantee a correct registration, and overlapping submaps can
have origins farther than 10 m apart. Consequently these labels do not
establish exact 6-DoF loop correctness. Square 1 has one accepted loop without
a usable GT label, Square 2 has four, Library 1 has 251 and Campus Road 1 has 44. Playground 1 demonstrates that high
proximity recall and a connected graph can coexist with severe trajectory
error.

MB means 10⁶ bytes. Communication counts serialized CDR messages per directed
recipient; RTPS, discovery and retransmission overhead are excluded. Timings
cover detection and concurrent CBS, excluding odometry, descriptors and map
export. Square 1 reused saved odometry and MegaLoc features. Campus Road 1's
full pipeline took **70.0 minutes**, dominated by odometry and submap
preparation, while its detection/CBS stage took under three minutes. Low GPU
utilization outside MegaLoc inference is expected: mapping, MapClosures,
GICP and CBS use CPU.

## Loop quality and suspected outliers

A [dedicated loop-quality audit](LOOP-QUALITY.md) checks the retained accepted
measurements. A distance flag means that the measured translation length
differs from the GT endpoint distance by **more than 2 m**. This is a
diagnostic flag, not a confirmed 6-DoF outlier: GT orientations are
placeholders, antenna offsets are uncorrected, and GT may contain errors.

| Experiment | Accepted loops | Distance flags / GT-assessable | Local cycle flags / eligible loops |
|---|---:|---:|---:|
| Square 1 | 50 | **7 / 49** | 0 / 38 |
| Square 2 | 37 | **0 / 33** | 0 / 3 |
| Library 1 | 1,049 | **25 / 798** | 0 / 551 |
| Campus Road 1 | 156 | Unknown; transforms removed | Unavailable |
| Playground 2 | 165 | **0 / 165** | 0 / 11 |
| Laboratory 1 | 6 | N/A; no usable trajectory GT | Insufficient support |
| Playground 1, excluded | 377 | Unknown; transforms removed | Unavailable |

The cycle test uses short raw odometry segments and nearby inter-robot
loops, requiring at least three neighbours. A loop is flagged only when a
majority of cycles exceed 2 m translation or 5° rotation. All 18 distance
flags with sufficient cycle support pass that local test; 14 other distance
flags lack enough support. Shared registration biases can pass cycle checks,
so exact false-closure counts remain undetermined.

Accepted endpoints farther than 10 m apart are **not automatically false
closures**. Playground 2 has 58 such pairs, yet all 165 measured translation
lengths agree with available GT within 0.71 m. The detailed report includes
1/2/5 m threshold sensitivity, branch attribution, problematic endpoint IDs,
coverage limits and [all per-loop measurements](figures/loop-quality/per-loop.csv).

## Findings and limitations

**Square 1:** distributed CBS recovered the exact **50 loop measurements**
from the working centralized pipeline. Under the original shared evaluation,
CBS gave 1.2112 m versus 1.1954 m for centralized GNC-TLS. The centralized run
reduced its initial-alignment position RMSE from 2.1997 m to 1.1954 m. Those
are archived, interpolated-GT numbers and are kept separate from the evo
table above. One accepted visual-only pair was flagged by a 2.622 m
translation-length discrepancy; passing registration does not certify every
constraint. [Centralized reference and audit](RESULTS-MAPCLOSURES.md).
The expanded audit of all 49 GT-assessable accepted loops finds seven
translation-length disagreements above 2 m; the earlier single-pair
highlight came from the narrower discussion of origins beyond 10 m.
[Full loop-quality audit](LOOP-QUALITY.md).

**Square 2:** the 255.197-second bag and ground-truth files passed official
checksum verification and SQLite checks. All 7,411 expected LiDAR messages
were received; no robot had an IMU gap over 0.2 s. The 37 accepted loops
comprise 22 Alpha–Bob, five Alpha–Carol and ten Bob–Carol constraints. Every
keyframe had nonempty MapClosures density features. Combined CBS ATE was
**0.5340 m**, with **45.76 s** for detection and CBS. Raw independently aligned
odometry ATEs were **0.5441 / 0.5772 / 0.2179 m** for Alpha/Bob/Carol. GT has
five gaps over two seconds for Alpha and two for Bob, so the score covers only
available timestamp matches.

**Campus Road 1:** all three robots connected and all 23,010 expected LiDAR
messages were received. GT has substantial gaps. Of 445 selected, assessable
proposals within 10 m, 331 failed the registration RMSE gate and two lacked a
native pose. These are rejected nearby candidates, not a proven missed-loop
count.

**Library 1:** the official 17.53 GB bag passed checksum, SQLite and sensor
continuity checks. All 13,481 expected clouds were received and 13,441 frames
exported. Every keyframe had nonempty MapClosures density features. The
1,049 loops include 437 Alpha–Bob, 314 Alpha–Carol, 259 Bob–Carol and 39
intra-robot constraints. The graph connected all robots with **1.4172 m**
combined CBS ATE; detection/CBS took **181.41 s**. Raw independently aligned
odometry ATEs were **1.3011 / 1.9088 / 1.4021 m**, with no early under-motion
visible in the saved diagnostics. GT has gaps up to 21 / 43 / 29 seconds,
so evaluation covers only available timestamp matches. The high loop yield
on this sequence does not establish exact correctness of every constraint.

**Laboratory 1:** all 51 selected proposals involving Alpha failed native
MapClosures pose initialization before GICP. Only 86/295 Alpha density maps
had nonempty ORB features, with a median of zero features. MegaLoc proposals
could not bypass this pose-initialization dependency. The release provides
start/end motion-capture records, not timestamped intermediate trajectories;
full-path ATE and retrieval proximity accuracy remain unavailable.

**Playground 1:** the historical full run had large raw Alpha/Bob errors
before CBS. Bob's stored IMU stream contains gaps of approximately 0.570,
0.310 and 0.281 s. Restarting at 22 s restored output coverage but did not
restore accuracy. Database integrity checks passed; the gaps do not by
themselves prove structural bag corruption. Follow-up bounded replays found:

| Diagnostic interval | Visual variance 1000: raw ATE (m) | Visual variance 100: raw ATE (m) |
|---|---:|---:|
| Alpha, 0–105 s | 37.6765 | 0.1478 |
| Bob, 22–105 s | 34.4114 | 32.4329 |

Bob's short pre-gap segment at variance 100 achieved 0.0640 m ATE. Moving-start
initialization remains a suspect for the post-gap failure, not an isolated
cause. CBS also deformed Carol's initially accurate trajectory: 20.70 m RMS
change remained after removing the best rigid transform between raw and
corrected tracks. That change is not a GT ATE and does not isolate faulty
loop poses from optimizer behavior. No corrected full-sequence run followed
these diagnostics; the sequence was excluded at the user's request.

**Playground 2:** the official download passed checksum and SQLite checks;
all three IMU streams had no gaps over 0.2 s. All expected clouds were
received, and the early under-motion seen in Playground 1 was absent. Raw
odometry ATEs were **0.1941 / 0.3302 / 0.2548 m** for Alpha/Bob/Carol, each
with an independent rigid fit. These diagnostic scores have a different
alignment scope from joint CBS ATE and cannot be subtracted to quantify a
CBS improvement. The final shared CBS ATE was **0.2941 m**.

## Figures and retained evidence

| Sequence | Trajectories and loops | Map | Ground truth / diagnostics |
|---|---|---|---|
| Square 1 | [Trajectory PNG](figures/cbs-square1/trajectories_and_loops.png) | [Top view](figures/cbs-square1/map_top_down.png) | [Gallery and archived figure protocol](figures/cbs-square1/README.md) |
| Square 2 | [Trajectory PNG](figures/cbs-square2/trajectories_and_loops.png) | [Top view](figures/cbs-square2/map_top_down.png) | [GT trajectories](figures/cbs-square2/ground_truth_trajectories.png) |
| Library 1 | [Trajectory PNG](figures/cbs-library1/trajectories_and_loops.png) | [Top view](figures/cbs-library1/map_top_down.png) | [GT trajectories](figures/cbs-library1/ground_truth_trajectories.png) |
| Campus Road 1 | [Trajectory PNG](figures/cbs-campus-road1/trajectories_and_loops.png) | [Top view](figures/cbs-campus-road1/map_top_down.png) | [GT trajectories](figures/cbs-campus-road1/ground_truth_trajectories.png) |
| Playground 2 | [Trajectory PNG](figures/cbs-playground2/trajectories_and_loops.png) | [Top view](figures/cbs-playground2/map_top_down.png) | [GT trajectories](figures/cbs-playground2/ground_truth_trajectories.png) |
| Laboratory 1 | [Bob–Carol component](figures/cbs-laboratory1/Bob/trajectories_and_loops.png) | [Separate component maps](figures/cbs-laboratory1/README.md) | [GT endpoints only](figures/cbs-laboratory1/ground_truth/ground_truth_endpoints.png) |
| Playground 1, excluded | [Historical trajectories](figures/cbs-playground1/trajectories_and_loops.png) | [Historical map](figures/cbs-playground1/map_top_down.png) | [Startup diagnosis](figures/cbs-playground1/startup-audit/startup_motion.png) |

PNG and PDF figures are available through the individual reports. Map colors
show robot identity or elevation from LiDAR XYZ, rather than camera RGB.

![Playground 2 corrected trajectories and loop attributions](figures/cbs-playground2/trajectories_and_loops.png)

Rerun **0.37.1** recordings remain local: approximately **426.7 MB** for the
reported Square 1 run, **88.4 MB** for Laboratory 1, **471.7 MB** for Campus
Road 1, **216.1 MB** for Playground 2, **250.0 MB** for Square 2 and **313.3 MB**
for Library 1. Their locations are listed in the
sequence reports. Playground 1's recording and large generated outputs were
removed; reports and small diagnostic evidence remain. Campus Road 1 and
Playground 2 intermediates were also removed after validation, freeing
**26.08 GiB** and **5.47 GiB**, respectively. Square 2 cleanup freed another
**7.98 GiB**, including the stopped sandbox attempt. Library 1 cleanup freed
**14.08 GiB**, including its interrupted initial replay. Original datasets are retained.
Deleted caches require regeneration; retained reports are not completed stage
caches.

The sequence audits checked causal candidates, same-robot exclusion, unique
loop pairs and serialized-byte totals. Saved evo results were independently
reproduced, and retained Rerun recordings passed decoding checks. The latest
research Python suite passed **40 tests**, with four optional ROS tests
skipped; earlier native validation passed seven CBS/cbs_ros CTest cases and
three DDS synthetic cases. Native project branches are
`dev/fast-livo2-s3e-dpgo`, with CBS at `11d84af` and cbs_ros at `86e3e35`.
Full hashes, configurations, timing and validation details are linked from
each sequence report.
