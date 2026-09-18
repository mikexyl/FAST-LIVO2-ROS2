# S3E_Square_1: EllipseLIO + ellipsoid MapClosures + PCM/CBS

**Smoke test only: first 45 seconds; not a full-sequence ATE result.**

EllipseLIO -> native ellipsoid surface BEVs -> MapClosures -> distributed PCM -> CBS with live GICP factors.

Retrieval uses ellipsoid-projected BEVs only. Three isolated DDS workers detect loops; distributed PCM gates inter-robot loops before native CBS optimization. Native GICP factors retain the existing CBS configuration. No centralized PGO is used.

| Robot | Raw ATE, independent fit [m] | CBS ATE, shared component fit [m] |
|---|---:|---:|
| Alpha | 0.0929 | 0.2011 |
| Bob | 0.1801 | 0.2739 |
| Carol | 0.1241 | 0.1241 |

Combined three-robot ATE unavailable; inspect component/GT status.

Evo 1.36.5, 50 ms timestamp association, one shared rigid fit per CBS component, no scale fitting or additional robot fit. Supplied GT orientations are unused. Raw odometry uses separate fits.

PCM: **5 proposed / 5 retained / 0 excluded**. PCM applies to inter-robot measurements; verified intra-robot loops pass unchanged.

Components: `{'Alpha': 'Alpha', 'Bob': 'Alpha', 'Carol': 'Carol'}`. Detection/PCM/CBS wall time: **16.22 s**.

The ellipsoid float bases are projected to their nearest orthogonal basis only when the maximum Gram-matrix defect is at most 0.001; larger defects fail preparation. Axes and centers are unchanged. Per-keyframe adjustment magnitudes are retained in preparation timings.

![Trajectories](trajectories.png)

![CBS corrected maps](maps.png)

![Ellipsoid density BEVs](bevs.png)

[Numeric report](report.json), [PCM decisions](dpgo/pcm.json), [evo evidence](evo/cbs/evaluation.json), [Rerun recording](result.rrd).
