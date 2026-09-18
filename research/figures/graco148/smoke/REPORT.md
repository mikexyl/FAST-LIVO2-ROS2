# reference: EllipseLIO + ellipsoid MapClosures + PCM/CBS

**Smoke test only: first 45 seconds of bag time. This is not a full-sequence result.**

EllipseLIO -> native ellipsoid surface BEVs -> MapClosures -> distributed PCM -> CBS with live GICP factors.

Retrieval uses ellipsoid-projected BEVs only. 6 isolated DDS workers detect loops; distributed PCM gates inter-robot loops before native CBS optimization. Native GICP factors retain the existing CBS configuration. No centralized PGO is used.

| Robot | Raw ATE, independent fit [m] | CBS ATE, shared component fit [m] |
|---|---:|---:|
| robot1 | 0.0686 | 0.0686 |
| robot2 | 3.7203 | 3.7203 |
| robot3 | 7.5883 | 7.5883 |
| robot4 | 0.0548 | 0.0548 |
| robot5 | 0.0458 | 0.0458 |
| robot6 | 0.0403 | 0.0403 |

Combined multi-robot ATE unavailable; inspect component/GT status.

Evo 1.36.5, 50 ms timestamp association, one shared rigid fit per CBS component, no scale fitting or additional robot fit. Raw odometry uses separate fits. See the numeric report for GT frame conversion and orientation use.

PCM: **0 proposed / 0 retained / 0 excluded**. PCM applies to inter-robot measurements; verified intra-robot loops pass unchanged.

Components: `{'robot1': 'robot1', 'robot2': 'robot2', 'robot3': 'robot3', 'robot4': 'robot4', 'robot5': 'robot5', 'robot6': 'robot6'}`. Detection/PCM/CBS wall time: **19.24 s**.

The ellipsoid float bases are projected to their nearest orthogonal basis only when the maximum Gram-matrix defect is at most 0.001; larger defects fail preparation. Axes and centers are unchanged. Per-keyframe adjustment magnitudes are retained in preparation timings.

![Trajectories](trajectories.png)

![CBS corrected maps](maps.png)

![Ellipsoid density BEVs](bevs.png)

[Numeric report](report.json), [PCM decisions](dpgo/pcm.json), [evo evidence](evo/cbs/evaluation.json), [Rerun recording](result.rrd).

## Six-robot group connectivity

Expected one connected component across ground-01..06. Measured components: `[['robot1'], ['robot2'], ['robot3'], ['robot4'], ['robot5'], ['robot6']]`. All six connected in a consistent CBS frame: **False**. No edges or poses were imposed from GT.
