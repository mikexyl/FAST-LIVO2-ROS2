# Dense trajectory continuity audit

The saved dense trajectories retain the raw sample timestamps. To isolate graph propagation, each raw step is rotated by the previous correction and subtracted from the corrected step. A fixed rigid alignment therefore contributes zero. Across these runs the maximum additional displacement is below 0.5 mm per sample. There is no evidence here of macroscopic jumps introduced by trajectory correction. This audit does not establish that the trajectory or spatial map deformation is accurate.

Trajectory-only panels omit map points and ground truth to make each line unambiguous. Close-ups show the largest raw velocity change, selected without ground truth; gray is raw odometry transformed by the first correction only, colored dots are the actual corrected samples, and dashed black is ground truth. No smoothing, sample removal or trajectory replacement is applied. ATE and graph solutions are unchanged.

## circle40 continuity audit

| Robot | Raw max step [m] | Corrected max step [m] | Raw max speed [m/s] | Corrected max speed [m/s] | Max added displacement [mm] |
|---|---:|---:|---:|---:|---:|
| aerial05 | 0.8013 | 0.8013 | 4.843 | 4.843 | 0.4039 |
| aerial07 | 0.3308 | 0.3308 | 3.446 | 3.446 | 0.1315 |
| ground05 | 0.3294 | 0.3294 | 2.499 | 2.499 | 0.0004 |
| ground06 | 0.3078 | 0.3078 | 2.067 | 2.067 | 0.0400 |

## trajectory40 continuity audit

| Robot | Raw max step [m] | Corrected max step [m] | Raw max speed [m/s] | Corrected max speed [m/s] | Max added displacement [mm] |
|---|---:|---:|---:|---:|---:|
| aerial05 | 0.8013 | 0.8010 | 4.843 | 4.841 | 0.4121 |
| aerial07 | 0.3308 | 0.3308 | 3.446 | 3.446 | 0.1375 |
| ground05 | 0.3294 | 0.3293 | 2.499 | 2.499 | 0.0912 |
| ground06 | 0.3078 | 0.3078 | 2.067 | 2.067 | 0.1838 |

## trajectory80 continuity audit

| Robot | Raw max step [m] | Corrected max step [m] | Raw max speed [m/s] | Corrected max speed [m/s] | Max added displacement [mm] |
|---|---:|---:|---:|---:|---:|
| aerial05 | 0.8013 | 0.8010 | 4.843 | 4.842 | 0.4775 |
| aerial07 | 0.3308 | 0.3308 | 3.446 | 3.446 | 0.2460 |
| ground05 | 0.3294 | 0.3293 | 2.499 | 2.499 | 0.1440 |
| ground06 | 0.3078 | 0.3077 | 2.067 | 2.066 | 0.3078 |
