"""Summarize the frozen four-flight result without changing experiment outputs."""
import argparse
import json
from pathlib import Path


def read(path):
    return json.loads(path.read_text())


def rows(path):
    return [json.loads(line) for line in path.read_text().splitlines()]


parser = argparse.ArgumentParser()
parser.add_argument('work', type=Path)
args = parser.parse_args()
work = args.work.resolve()
state = read(work/'status.json')
assert state['phase'] == 'complete'
report = read(work/'report/report.json')
audit = read(work/'retention-audit.json')
gate = read(work/'frontend-gate.json')
robots = list(state['robots'])
loops = rows(work/'dpgo/constraints.jsonl')
inter = sum(e['i'][0] != e['j'][0] for e in loops)
groups = report['connectivity']['measured_components']
lines = ['# GRACO aerial 05–08: four robots on workstation 148', '',
    'Four fresh, concurrent, full-flight 1× replays using temporal-only EllipseLIO '
    '(10-second windows, 5-second overlap), native ellipsoid-BEV MapClosures, '
    'distributed PCM, and CBS with GICP registration factors. The original '
    'ROS1 bags were converted to sensor-only ROS2 input; every LiDAR/IMU '
    'measurement, header timestamp, record timestamp, and LiDAR point payload '
    'was checked against its source. No timestamps were rebased.', '']
if not gate['passed']:
    failures = ', '.join(f'{r}: {q["max_successful_update_gap_s"]:.3f} s' for r, q in gate['quality'].items() if not q['passed'])
    lines += [f'**Diagnostic backend result.** The frontend stability gate failed '
        f'for {failures} (successful-LiDAR-update gap must be less than 1 second). '
        'All captures nevertheless completed with finite chronological poses, '
        'maximum speed below 20 m/s, complete sensor coverage, and verified '
        'native exports. The failed gate is preserved in `frontend-gate.json`; '
        'no thresholds were changed to continue.', '']
if not report['connectivity']['all_robots_connected']:
    lines += ['**The four robots did not form one connected map.** Measured components: '
              + '; '.join(', '.join(g) for g in groups) + '. No all-four shared ATE is reported.', '']
lines += ['| Flight | Raw ATE RMSE [m] | CBS ATE RMSE [m] | CBS component | Poses | Max speed [m/s] | Max update gap [s] | Frontend gate |',
          '|---|---:|---:|---|---:|---:|---:|---|']
for robot in robots:
    q = gate['quality'][robot]
    component = report['components'][robot]
    cbs = report['cbs'][component]['per_robot'][robot]['statistics']['rmse']
    lines.append(f'| {robot} | {report["raw"][robot]["rmse_m"]:.4f} | {cbs:.4f} | {component} | '
        f'{q["native_poses"]} | {q["max_speed_m_s"]:.3f} | {q["max_successful_update_gap_s"]:.3f} | '
        f'{"Passed" if q["passed"] else "Failed"} |')
lines += ['', 'Position ATE is evaluated entirely through **evo 1.36.5**, with '
    '50 ms association, rigid SE(3) alignment, and no scale fitting. Raw fits '
    'are independent per robot; CBS uses one shared fit per measured connected '
    'component. These different alignment scopes matter when comparing raw '
    'and CBS errors. Ground truth was exported from `/gnss/ground_truth` only '
    'after optimization and did not influence mapping, retrieval, or loop admission.', '',
    '| Connected component | Shared CBS ATE RMSE [m] | Matched poses |', '|---|---:|---:|']
for component, value in report['cbs'].items():
    lines.append(f'| {", ".join(value["robots"])} | {value["rmse_m"]:.4f} | {value["samples"]} |')
lines += ['', f'Accepted loops: **{len(loops)}** ({inter} inter-robot, {len(loops)-inter} intra-robot). '
    f'PCM rejected **{report["pcm"]["excluded_loops"]}** of {report["pcm"]["proposed_loops"]} proposals. '
    f'CBS retained {report["registration"]["registration_factor_count"]} GICP factors. '
    f'Detection and backend execution took **{report["runtime"]["wall_s"]:.2f} seconds**, '
    'excluding sensor replay, descriptor preparation, and evaluation.', '',
    'Inter-robot loop counts: ' + json.dumps(report['connectivity']['inter_robot_loop_counts'], sort_keys=True) + '.', '',
    '![CBS trajectories and separate component maps](report/trajectories-maps.png)', '',
    '## Native submaps and runtime', '',
    '| Robot | Completed submaps | Partial inspection tails | Max correspondence age [s] | Mean scan processing [ms] | P95 [ms] | Peak mapper RSS [MiB] |',
    '|---|---:|---:|---:|---:|---:|---:|']
for robot in robots:
    a = audit['frontends'][robot]
    lines.append(f'| {robot} | {a["completed_submaps"]} | {a["partial_submaps"]} | {a["max_correspondence_age_s"]:.3f} | '
        f'{1000*a["processing_mean_s"]:.2f} | {1000*a["processing_p95_s"]:.2f} | {a["peak_mapper_rss_mib"]:.2f} |')
lines += ['', 'Descriptors and registration evidence use the same native processed '
    'member scans, expressed in each completed submap’s last included IMU frame. '
    'This is not full-resolution raw geometry. Shutdown tails are retained for '
    'inspection and excluded from retrieval. Completed submaps become available '
    'causally, while graph poses retain their anchor timestamps.', '',
    'The dedicated container has no CPU quota, CPU affinity restriction, or '
    'memory limit. Four-thread native launchers run four frontends concurrently; '
    'algorithm thread settings preserve the earlier temporal configuration. '
    'The source overlay and build are isolated from prior experiments.', '',
    '## Verification and retained artifacts', '',
    'Both native CTests and all 10 selected Python/integration tests passed, '
    'including actual four-robot PCM/CBS with GICP. The six core temporal '
    'source files match the restored temporal implementation. The final '
    f'audit verified {audit["descriptor_evidence_memberships"]} descriptor/evidence '
    f'memberships and {audit["causal_ranked_events"]} causal retrieval events, '
    'along with native payload hashes, anchor poses, graph timestamps, '
    'same-robot exclusions, and frozen source hashes. Four live Rerun recordings '
    'and the derived recording passed verification.', '',
    '[Numeric report](report/report.json), [native audit](retention-audit.json), '
    '[frontend gate](frontend-gate.json), [configuration](config.yaml), '
    '[source hashes](source-hashes.json), [backend summary](dpgo/summary.json), '
    '[derived Rerun recording](report/result.rrd).', '',
    'Full data and live recordings remain on 148 at '
    '`/data3/mikexyl/swarm_s3e_ws/src/.ros2/graco-aerial-four148-20260920`. '
    'All bulk geometry is retained. Prior experiments and the paused CU-Multi '
    'download are unchanged. This is one fixed-configuration experiment, '
    'not a parameter sweep or a repeated-trial performance estimate.', '']
gaps = {}
for robot in robots:
    data = rows(work/'frontends'/robot/'frontend/native_updates.jsonl')
    good = [r for r in data if r['lidar_updated']]
    gaps[robot] = [dict(duration_s=(b['sensor_stamp_ns']-a['sensor_stamp_ns'])/1e9,
        previous_scan_id=a['scan_id'], next_scan_id=b['scan_id'],
        previous_active_submap_id=a['active_submap_id'], next_active_submap_id=b['active_submap_id'],
        start_since_initialization_s=(a['sensor_stamp_ns']-data[0]['sensor_stamp_ns'])/1e9,
        rows=[r for r in data if a['scan_id'] <= r['scan_id'] <= b['scan_id']])
        for a, b in zip(good, good[1:]) if b['sensor_stamp_ns']-a['sensor_stamp_ns'] >= 10**9]
(work/'update-gap-events.json').write_text(json.dumps(gaps, indent=2)+'\n')
(work/'REPORT.md').write_text('\n'.join(lines))
print(work/'REPORT.md')
