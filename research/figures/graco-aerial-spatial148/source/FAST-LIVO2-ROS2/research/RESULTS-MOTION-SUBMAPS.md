# Motion/overlap submaps for EllipseLIO

**Historical experiment; implementation reverted locally at the user's request.**
Active code now retains only the 10-second / 5-second-overlap temporal strategy.
The results below and the
[retired experiment source](figures/motion-submaps/implementation/retired-source/README.md)
are preserved for reference.

The full five-group experiment completed on **2026-09-19 at 13:39:23 UTC**,
after **67 minutes 4 seconds**. All 18 fresh frontends passed the sensor-only
completion, finite-pose, speed and update-gap checks. All five distributed
MapClosures → PCM → CBS pipelines completed with valid common frames for their
connected components. The final artifact audit completed at 13:39:41 UTC.

**Accuracy is mixed, with a severe raw-odometry regression on Campus Road 2 /
Carol. This configuration should remain experimental.** Raw ATE decreased on
5/15 robots with usable ground truth and increased on 10/15, including changes
too small to establish a meaningful difference. Campus Road 2 / Carol worsened
from **3.3508 m to 59.1508 m** raw ATE, despite passing the sensor-only checks.
Those checks exclude numerical failures and large speed/update-gap excursions;
they do not exclude gradual accumulated drift.

| Group | Motion-strategy connectivity | Temporal shared CBS ATE [m] | Motion shared CBS ATE [m] | Retained loops / PCM rejected |
|---|---|---:|---:|---:|
| Laboratory 4 | All three | GT unavailable | GT unavailable | 49 / 0 |
| Campus Road 1 | Alpha + Bob; Carol separate | 1.9743 (A+B); 1.5904 (Carol) | 6.2599 (A+B); 1.2532 (Carol) | 7 / 0 |
| Campus Road 2 | All three | Invalid common-frame output | 10.9227 | 181 / 0 |
| Campus Road 3 | All three | 3.6144 | 1.0081 | 42 / 0 |
| GRACO ground-01–06 | All six | 6.1543 (five robots); 0.9279 (isolated robot4) | 4.8373 (six robots) | 43 / 1 |

GRACO's component membership changed, so its shared ATE values are not a direct
comparison over the same robot set. Campus Road 3 has a substantial CBS
improvement, while Campus Road 1's Alpha/Bob component regressed. Campus Road 2
now produces a valid shared frame, but accuracy remains poor.

The largest successful-LiDAR-update gap was **0.533 s**, maximum speed was
**3.163 m/s**, and there were **no stale recoveries**. Of 2,394 completed maps,
2,387 retired through the keyframe budget and seven through the age guard;
all were retrievable. Another 35 partial tails were retained. The audit verified
all 2,394 descriptor/evidence memberships, 8,535 causal ranked events, all 18
live Rerun recordings and all five derived recordings. Source/binary hashes
matched the frozen manifest. Bulk geometry remains on 148.

The [full per-robot comparison and figures](figures/motion-submaps/full-five-groups/REPORT.md),
[numeric summary](figures/motion-submaps/full-five-groups/batch-summary.json),
and [artifact audit](figures/motion-submaps/full-five-groups/retention-audit.json)
are copied into this workspace, together with raw/CBS trajectories, evo output,
maps and logs. No parameter sweep or repeat was performed.

## Implemented policy

The opt-in `mapping.submaps.strategy: motion_overlap` uses the configuration in
[motion_submaps.yaml](figures/motion-submaps/implementation/retired-source/FAST-LIVO2-ROS2/scripts/recent_submaps/motion_submaps.yaml):

| Decision | Initial setting |
|---|---|
| Select a keyframe | 1 m translation, 10° rotation, or occupied-voxel overlap below 0.6 |
| Begin growing a successor | 10 m extent, eight selected keyframes, or 15 s age |
| Request normal handover | 20 m extent or 15 selected keyframes |
| Accept normal handover | Successor supports at least 50 sampled points and 20% of sampled points |
| Prevent stale geometry | 30 s maximum map age; explicit recovery if no supported fresh successor exists |
| Admit a completed map to retrieval | At least three selected keyframes and 50 fitted ellipsoids |

Extent is maximum displacement from the map's first member pose. Overlap uses
a 0.5 m world voxel grid with a one-cell neighbourhood. These are initial
experimental settings, not tuned or established optimal values.

Selected keyframes control boundaries. **Every processed native scan is still
inserted after matching**, and the same exported member scans supply both
ellipsoid-BEV descriptors and registration evidence. This is processed geometry,
not full-resolution raw data. Completed maps never re-enter odometry. Handover
preserves filter state and covariance; an unsupported stale-map recovery retains
IMU propagation and seeds a fresh map.

The original persistent-map and temporal strategies remain available. Details,
frame conventions and run commands are in
[MOTION-SUBMAPS.md](figures/motion-submaps/implementation/retired-source/FAST-LIVO2-ROS2/scripts/recent_submaps/MOTION-SUBMAPS.md).

## Verification

The isolated build on 148 passed two native CTest cases, the Python snapshot,
endpoint and scheduler checks, and two actual native distributed retrieval /
PCM / CBS / GICP integration checks. The native tests include motion-dependent
boundaries, stationary retention, unsupported handovers, stale recovery,
state/covariance retention, temporal compatibility and bounded writer behaviour.
The optional DDS case skipped locally passed on 148.

Two 120-second smoke replays completed with verified Rerun recordings:

| Smoke | Native poses | Max speed [m/s] | Max successful-update gap [s] | Completed / retrievable maps | Partial tails |
|---|---:|---:|---:|---:|---:|
| Library 2 / Bob | 1,183 | 1.965 | 0.204 | 11 / 9 | 2 |
| Campus Road 2 / Bob | 1,030 | 2.072 | 0.201 | 14 / 14 | 2 |

Both passed finite/chronological pose, exact membership, anchor and payload-hash
validation; neither needed stale recovery. Library 2 used three supported
age-guard handovers and eight keyframe-budget handovers. Campus Road 2 used 14
keyframe-budget handovers. Actual ellipsoid-BEV preparation succeeded for all
nine retrievable Library 2 maps. These short checks establish pipeline operation,
not full-sequence accuracy.

Machine-readable evidence and test logs are retained in
[implementation-validation.json](figures/motion-submaps/implementation/implementation-validation.json)
and [implementation/](figures/motion-submaps/implementation/).

## Fresh full experiment

All 18 robots were fresh captures: Laboratory 4, Campus Road 1–3, and GRACO
ground-01–06. Six frontends ran concurrently at 1×, with separate ROS domains
and four executor threads each. The new container has **no CPU, RAM or swap
quota**, no CPU affinity restriction, and access to all GPUs. Thread counts
control scheduling, not enforced resource budgets.

Calibration, requested IMU noise, reliable input, MapClosures, registration,
PCM and CBS settings are retained. Successful frontend admission uses sensor
completion and stability only; ground truth remains evaluation-only. ATE uses
evo 1.36.5, 50 ms association and rigid alignment without scale. Raw trajectories
are fitted independently; CBS uses one fit per measured connected component.

The controller completed capture validation, descriptor preparation, distributed
MapClosures → PCM → CBS, evaluation, map/trajectory export and recording
verification. A separate finalizer audited source/artifact hashes and
descriptor/evidence membership, then created `REPORT.md`, `batch-summary.json`,
`retention-audit.json` and raw-ATE comparison figures. There were no backend
failures in this batch.

On 148, the experiment root is:

```text
/data3/mikexyl/swarm_s3e_ws/src/.ros2/motion-submaps-20260919
```

Full outputs are under `full-five-groups/`. Container
`ellipselio-motion-submaps148` mounts this root at
`/workspace/.ros2/recent-submaps`; historical temporal data are read-only at
`/history/recent-submaps`. The batch source manifest freezes 103 source/binary
files. Historical experiments and the paused CU-Multi download are preserved.

The comparison is against the
[retained temporal five-group batch](RESULTS-RECENT-SUBMAPS-FULL-FIVE-GROUPS.md).
Concurrency and resource allocation differ, and these are single trials. Any
observed improvement must be reported per sequence without claiming a general
accuracy cure or a controlled causal ablation.
