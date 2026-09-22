from pathlib import Path
import json,shutil,hashlib
workspace=Path(__file__).resolve().parents[3]
base=workspace/'.ros2/upstream-ellipselio-20260921'
research=workspace/'FAST-LIVO2-ROS2/research';out=research/'figures/upstream-ellipselio-library2-20260921'
first=json.loads((base/'bob-upstream/evaluation/metrics.json').read_text())
repeat=json.loads((base/'bob-upstream-repeat/evaluation/metrics.json').read_text())
old=json.loads((research/'figures/recent-submaps-library2/baseline/metrics.json').read_text())
temporal=json.loads((research/'figures/recent-submaps-library2/enabled/metrics.json').read_text())
for name in ['bob-upstream','bob-upstream-repeat']:
    shutil.copytree(base/name/'evaluation',out/name,dirs_exist_ok=True)
    for rel in ['source-hashes.json','loaded-library.txt','recording/status.json','frontend/summary.json','frontend/runtime.yaml','frontend/mapping.log']:
        dest=out/name/rel;dest.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(base/name/rel,dest)
shutil.copytree(base/'harness',out/'harness',dirs_exist_ok=True,ignore=shutil.ignore_patterns('__pycache__'))
comparison=dict(upstream_run=first,upstream_repeat=repeat,historical_persistent=old,historical_temporal=temporal)
(out/'comparison.json').write_text(json.dumps(comparison,indent=2)+'\n')
checks=[]
for key in ['install/ellipselio/lib/libellipselio_mapping.so','launcher/ellipselio_mapping_mt','harness/bob.yaml']:
    aa=json.loads((base/'bob-upstream/source-hashes.json').read_text())[key]
    bb=json.loads((base/'bob-upstream-repeat/source-hashes.json').read_text())[key]
    assert aa==bb,key
    checks.append(dict(file=key,sha256=aa))
(out/'repeat-controls.json').write_text(json.dumps(checks,indent=2)+'\n')
text=f"""# Upstream EllipseLIO: Library 2 / Bob, 2026-09-21

Upstream [6506f46](https://github.com/v4rl-ucy/ellipselio/commit/6506f46f1947b4ef86cfba402f11f10a6ef520ee), published September 18, completed the previously diverging S3E Library 2 / Bob sequence twice. Both runs {'passed' if first['gate_pass'] and repeat['gate_pass'] else 'did not both pass'} the existing stability and accuracy gate. The gate requires full completion, finite chronological poses, maximum speed at most 20 m/s, successful-update gaps below 1 second, and position ATE at most 5 m. These trials use the upstream persistent map, without our temporal or accumulated-area submapping, MapClosures, or CBS.

| Run | Position ATE RMSE | Maximum speed | Maximum successful-update gap | Full completion |
|---|---:|---:|---:|---|
| Historical guarded persistent baseline | {old['ate_rmse_m']:.4f} m | {old['max_speed_m_s']:.3f} m/s | {old['max_successful_update_gap_s']:.3f} s | Yes |
| New upstream | {first['ate_rmse_m']:.4f} m | {first['max_speed_m_s']:.3f} m/s | {first['max_successful_update_gap_s']:.3f} s | {'Yes' if first['full_completion'] else 'No'} |
| New upstream repeat | {repeat['ate_rmse_m']:.4f} m | {repeat['max_speed_m_s']:.3f} m/s | {repeat['max_successful_update_gap_s']:.3f} s | {'Yes' if repeat['full_completion'] else 'No'} |
| Historical temporal submaps, 10 s / 5 s overlap | {temporal['ate_rmse_m']:.4f} m | {temporal['max_speed_m_s']:.3f} m/s | {temporal['max_successful_update_gap_s']:.3f} s | Yes |

The new runs have {first['native_poses']} / {repeat['native_poses']} finite chronological native scan poses, and {first['failed_updates_after_initialization']} / {repeat['failed_updates_after_initialization']} failed updates after initialization. The initial scan seeds the map. Successful updates require upstream iEKF success, positive final feature count, and finite residual and pose; neither run reported success without valid final features. The final processed scan is {first['final_sensor_to_last_imu_s']*1000:.1f} / {repeat['final_sensor_to_last_imu_s']*1000:.1f} ms before Bob's final IMU sample. Other robots' messages extend the bag beyond Bob's input.

| Native mapper resource measure | New upstream | Repeat |
|---|---:|---:|
| Mean processing per scan | {first['processing_mean_ms']:.2f} ms | {repeat['processing_mean_ms']:.2f} ms |
| Median processing per scan | {first['processing_median_ms']:.2f} ms | {repeat['processing_median_ms']:.2f} ms |
| p95 processing per scan | {first['processing_p95_ms']:.2f} ms | {repeat['processing_p95_ms']:.2f} ms |
| Maximum processing per scan | {first['processing_max_ms']:.2f} ms | {repeat['processing_max_ms']:.2f} ms |
| Peak mapper RSS | {first['peak_rss_mib']:.1f} MiB | {repeat['peak_rss_mib']:.1f} MiB |
| Replay wall time including startup/drain | {first['wall_s']:.2f} s | {repeat['wall_s']:.2f} s |
| Final persistent-map points | {first['map_points_final']:,} | {repeat['map_points_final']:,} |

The runs were serial at 1× on workstation 148, in a separate container with zero CPU and memory quotas, four executor threads and OMP=4 for consistency with earlier trials. Both use identical S3Ev2 Bob calibration, map resolution 0.1 m, requested IMU noise 0.1/0.1 and bias noise 0.0001/0.0001, reliable LiDAR/IMU input, and the same publication settings. The research deskew exporter remains disabled.

The estimator, correspondence rules and compiler flags are the published upstream implementation. An isolated [instrumentation patch](figures/upstream-ellipselio-library2-20260921/instrumentation.patch) adds reliable-input configuration, observes iEKF success, and records exact completed-scan diagnostics. It does not change filter arithmetic, map geometry or numerical guards. The existing four-thread launcher is reused. Source, configuration and binary hashes, loaded-library paths, input QoS and container settings are retained. The first smoke test exposed an undeclared reliable-input parameter in the adapter; it was corrected before the validated smoke test and both full trials.

ATE is calculated entirely through evo 1.36.5, using nearest timestamp association within 50 ms and rigid SE(3) alignment with scale fixed to 1; no interpolation, time-offset fitting or pose filtering. Ground truth is used only for evaluation. The supplied S3E positions retain the earlier evaluation convention: GT orientations unused and antenna lever arm uncorrected.

The earlier persistent baseline and temporal result are historical references. The earlier guarded build used -O3; the upstream build preserves its published -Ofast flags. This trial therefore supports successful operation of the new version on this case, without attributing the improvement to one particular source change. The current development estimator branches have not been merged with upstream.

![Aligned trajectory, position error, speed and processing time](figures/upstream-ellipselio-library2-20260921/upstream-bob.png)

The dashed line marks the historical persistent baseline's first speed violation. The plot shows the first new-upstream run; both runs' numeric results and matched trajectories are retained in [the experiment directory](figures/upstream-ellipselio-library2-20260921/).

Full captures and native logs are retained locally under `.ros2/upstream-ellipselio-20260921/bob-upstream` and `bob-upstream-repeat`, and on 148 under `/data3/mikexyl/swarm_s3e_ws/src/.ros2/upstream-ellipselio-20260921/`. Each `recording/live.rrd` was checked with `rerun rrd verify`. Rerun records native published map/scan data and sparse ellipsoid markers; best-effort map chunks are partial late in the run, and a few published poses may be missed. Accuracy and update-gap results use every native logged pose rather than sampled ROS/Rerun output. No earlier experiment or paused download was changed.
"""
(research/'RESULTS-UPSTREAM-ELLIPSELIO-LIBRARY2.md').write_text(text)
