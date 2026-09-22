# GRACO all 14: multilayer accumulated-area pipeline — 22 September 2026

**1 connected component(s), 541 retained loops, 0 PCM rejections.** All fourteen fresh full-length 1× captures passed the sensor-only stability gate.

Workstation 148 ran without CPU, memory or GPU quotas. Updated EllipseLIO matches scans against its persistent map. Every 20 m or 10 s, a snapshot retains all accumulated native map representatives inside an 80 m horizontal radius, across scan ages and heights. Snapshots feed five terrain-relative ellipsoid BEV layers and the corresponding native geometry. This is processed map geometry, not full-resolution raw scans.

Same-band ORB/HBST matches are pooled and deduplicated, then fitted with the tested joint SE(2) estimator. The full-height descriptor is an inspection control. Missing terrain support produces empty layers. The existing geometric height initializer, GICP acceptance thresholds, singleton-accepting PCM and distributed CBS/GICP factors remain unchanged.

| Robot | Raw ATE (m) | CBS individual ATE (m) | CBS shared-component ATE (m) | Component |
|---|---:|---:|---:|---|
| aerial01 | 0.1850 | 0.1676 | 0.4753 | aerial01 |
| aerial02 | 0.0602 | 0.0625 | 0.2210 | aerial01 |
| aerial03 | 0.0679 | 0.1115 | 0.6674 | aerial01 |
| aerial04 | 0.1163 | 0.1348 | 0.5074 | aerial01 |
| aerial05 | 0.1958 | 0.2076 | 1.1092 | aerial01 |
| aerial06 | 0.0632 | 0.0758 | 0.6276 | aerial01 |
| aerial07 | 0.0757 | 0.0710 | 0.4719 | aerial01 |
| aerial08 | 0.1244 | 0.1125 | 0.3049 | aerial01 |
| ground01 | 0.2883 | 0.2850 | 1.0652 | aerial01 |
| ground02 | 0.6547 | 0.6772 | 1.6079 | aerial01 |
| ground03 | 0.5857 | 0.5357 | 1.1283 | aerial01 |
| ground04 | 0.5322 | 0.4597 | 0.7766 | aerial01 |
| ground05 | 0.4769 | 0.4391 | 0.7286 | aerial01 |
| ground06 | 0.4477 | 0.4087 | 1.1111 | aerial01 |

ATE uses **evo 1.36.5**, 50 ms association, rigid SE(3) alignment and no scale fitting. Individual alignment measures trajectory shape after each robot gets its own rigid alignment; shared-component alignment also measures errors in relative placement. Separate components receive separate alignments and do not constitute a recovered shared frame. Ground truth was introduced only after optimization.

| Component | Robots | Shared ATE (m) |
|---|---|---:|
| aerial01 | aerial01, aerial02, aerial03, aerial04, aerial05, aerial06, aerial07, aerial08, ground01, ground02, ground03, ground04, ground05, ground06 | 0.8615 |

| Loop category | Geometrically accepted | PCM retained |
|---|---:|---:|
| aerial–aerial | 151 | 151 |
| ground–ground | 147 | 147 |
| aerial–ground | 243 | 243 |

CBS added 540 native registration factors. 1 retained pose constraint(s) lacked an additional registration factor; reasons and preprocessing diagnostics are preserved in `diagnostics/summary.json`.
Loop counts are submap-pair constraints, not independent places: overlapping area snapshots reuse geometry.

Network payload accounting (serialized CDR bytes, excluding RTPS/discovery/retransmission): `{"cbs_network_cdr_bytes": 4897849, "loop_network_cdr_bytes": 2484910753, "pcm_network_cdr_bytes": 684786, "registration_network_cdr_bytes": 74380094}`. Loop exchange includes both descriptors and verification geometry.

658 geometric verification attempts; outcomes: `{"low_overlap": 116, "accepted": 541, "high_rmse": 1}`. Backend wall time: 1588.4 s.

| Robot | Native poses | Failed updates after initialization | Largest update gap (s) | Mean / p95 processing (ms) | Peak mapper RSS (MiB) | Submaps | Terrain unavailable |
|---|---:|---:|---:|---:|---:|---:|---:|
| aerial01 | 3712 | 0 | 0.272 | 39.14 / 98.85 | 1388.4 | 49 | 1 |
| aerial02 | 2488 | 0 | 0.280 | 51.82 / 105.98 | 849.9 | 28 | 1 |
| aerial03 | 3595 | 1 | 0.224 | 41.15 / 100.89 | 1216.9 | 40 | 1 |
| aerial04 | 2626 | 0 | 0.263 | 46.38 / 107.12 | 875.4 | 32 | 3 |
| aerial05 | 2638 | 0 | 0.220 | 48.58 / 108.62 | 1144.8 | 34 | 2 |
| aerial06 | 3095 | 0 | 0.184 | 15.89 / 37.30 | 967.7 | 33 | 1 |
| aerial07 | 3712 | 0 | 0.186 | 13.94 / 31.82 | 1191.9 | 40 | 0 |
| aerial08 | 2510 | 0 | 0.225 | 52.57 / 108.83 | 1063.4 | 29 | 1 |
| ground01 | 3093 | 2 | 0.320 | 21.39 / 37.52 | 1244.2 | 34 | 0 |
| ground02 | 3630 | 2 | 0.332 | 21.16 / 37.38 | 1341.4 | 39 | 0 |
| ground03 | 2740 | 2 | 0.337 | 27.34 / 86.28 | 911.7 | 30 | 0 |
| ground04 | 3063 | 1 | 0.216 | 25.33 / 44.65 | 1564.1 | 33 | 0 |
| ground05 | 4830 | 0 | 0.200 | 21.68 / 40.79 | 1453.1 | 52 | 0 |
| ground06 | 2872 | 0 | 0.200 | 23.03 / 41.82 | 1278.3 | 31 | 0 |

Update gaps use sensor timestamps between successful LiDAR corrections. Processing time measures native per-scan work and is a different quantity. Stability requires complete finite chronological output, speed ≤20 m/s and successful-update gaps <1 s.

CBS uses the unchanged 100-iteration settling budget. Completion alone does not prove convergence; final pose-change and last-ten-iteration statistics are retained in `diagnostics/summary.json`.

The native adapter and Python regression checks cover disabled-mode compatibility, raw correspondence eligibility, gravity-frame pose direction, layer serialization, deduplication, and peer isolation. A separate synthetic 14-worker DDS/PCM/CBS run retained 53 loops and 53 registration factors with correct poses in one connected component. This preflight does not establish dataset accuracy.

![ATE comparison](diagnostics/ate-comparison.png)

![Recovered components](diagnostics/components-map-topdown.png)

[BEV gallery](gallery/index.html) · [Rerun recording](report/result-gravity.rrd) · [Numeric report](report/report.json) · [Detailed diagnostics](diagnostics/summary.json) · [Runtime audit](runtime-audit.json) · [Configuration](config.yaml)

Original datasets, previous experiments, and the paused CU-Multi download were preserved. Bulk geometry is retained. Source, sensor and configuration hashes accompany this run.

## Interpretation

The single shared-frame position ATE is **0.861 m**. CBS improves individually aligned ATE for **8 of 14** robots and worsens it for the remaining **6**. All 14 robots connect, including aerial04. This batch has no matched full-height BEV control, so it does not isolate the benefit of height layering.

Across the 541 retained loops, post-run RTK/INS transform errors have translation median 0.866 m, p95 2.288 m, maximum 2.960 m; rotation median 0.940 degrees, p95 2.281 degrees. These diagnostics do not change loop acceptance. See [loop errors](diagnostics/loop-errors.json) and the [interactive report](index.html).
