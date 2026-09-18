"""Evaluate the fixed reliable-QoS retry using retained frontend evidence and evo."""
import gzip
import hashlib
import json
from pathlib import Path
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import yaml

HERE = Path(__file__).resolve().parent
RESEARCH = HERE.parents[2]
sys.path[:0] = [str(RESEARCH), str(RESEARCH.parent.parent/'Swarm-SLAM/s3e')]
from frontend_quality import assess
from s3e_pipeline.evaluation import ground_truth
from s3e_pipeline.evo_evaluation import evaluate


def load_rows(base, robot):
    compressed = base/'retained'/robot/'frames.jsonl.gz'
    if compressed.exists():
        with gzip.open(compressed, 'rt') as f:
            return [json.loads(line) for line in f], compressed
    path = base/robot/'export/frames.jsonl'
    return [json.loads(line) for line in path.read_text().splitlines()], path


def analytics(path):
    if not path.exists():
        return [], {}
    rows = [json.loads(line) for line in path.read_text().splitlines()]
    native = [r['native'] for r in rows[1:]]
    return rows, dict(messages=len(rows), zero_feature_updates=sum(r['num_feats']==0 for r in native),
        median_lidar_hz=float(np.median([r['lid_freq'] for r in native])),
        median_imu_hz=float(np.median([r['imu_freq'] for r in native])),
        median_native_update_ms=float(np.median([r['total_time'] for r in native]))*1000,
        time_association='Latest observed odometry stamp; native analytics has no exact timestamp')


def main():
    root = HERE/'reliable'
    retry = root/'attempt'
    old = yaml.safe_load((HERE/'attempt/Alpha/runtime.yaml').read_text())['/**']['ros__parameters']
    new = yaml.safe_load((retry/'Alpha/runtime.yaml').read_text())['/**']['ros__parameters']
    assert new.pop('input') == {'reliable': True}
    old['research'].pop('output'); new['research'].pop('output')
    assert old == new, 'Unexpected estimator parameter change'
    assert json.loads((HERE/'attempt/source-hashes.json').read_text()) == json.loads((retry/'source-hashes.json').read_text()), 'Trial pipeline sources or binaries changed'
    output = dict(sequence='S3E_Library_2', input_reliable=True,
        sequence_status=json.loads((root/'queue.json').read_text())['sequences']['S3E_Library_2']['status'],
        backend_started=(retry/'cbs.log').exists(),
        runtime_check='Only input.reliable and export path differ from best-effort trial',
        frozen_pipeline_hashes_match=True, raw_odometry={}, input_sha256={})
    for robot in ('Alpha', 'Bob', 'Carol'):
        summary_path = retry/robot/'summary.json'
        if not summary_path.exists():
            continue
        summary = json.loads(summary_path.read_text())
        rows, source = load_rows(retry, robot)
        q = assess(rows)
        if 'first_invalid_stamp_ns' in q:
            q['first_invalid_after_first_export_s'] = (q['first_invalid_stamp_ns']-rows[0]['stamp_ns'])/1e9
        with gzip.open(retry/'retained'/robot/'manifest.json.gz', 'rt') as f:
            manifest = json.load(f)
        if source.suffix == '.gz':
            with gzip.open(source, 'rb') as f:
                frame_hash = hashlib.sha256(f.read()).hexdigest()
        else:
            frame_hash = hashlib.sha256(source.read_bytes()).hexdigest()
        assert manifest['complete'] and manifest['frames'] == len(rows)
        assert frame_hash == manifest['files']['frames.jsonl']
        truth = {robot: ground_truth(HERE/f'{robot.lower()}_gt.txt')}
        metrics = evaluate(dict(poses=[dict(r, component=robot) for r in rows]), truth,
            dict(evo_max_diff_s=.05), root/'evo-full'/robot)
        native_rows, a = analytics(retry/robot/'analytics.jsonl')
        windows = {}
        for low, high in ((350, 400), (400, 430), (430, 450), (450, 485)):
            selected = [r['native'] for r in native_rows if r['last_observed_odometry_stamp_ns'] is not None
                and low <= (r['last_observed_odometry_stamp_ns']-rows[0]['stamp_ns'])/1e9 < high]
            if selected:
                windows[f'{low}-{high}s'] = dict(messages=len(selected), medians={k:float(np.median(
                    [r[k] for r in selected if r[k] is not None])) for k in ('lid_freq','imu_freq','num_feats','res_mean')})
        output['raw_odometry'][robot] = dict(replay_completed=summary['success'],
            quality=q, evo=metrics, analytics=a, wall_s=summary['wall_s'],
            late_analytics_windows=windows,
            frame_hash_verified_against_export_manifest=True,
            result_status='completed frontend; passed motion guard' if q['passed'] else 'failed frontend; ATE is diagnostic only')
        for path in (source, summary_path, HERE/f'{robot.lower()}_gt.txt', retry/robot/'runtime.yaml'):
            output['input_sha256'][str(path.relative_to(HERE))] = hashlib.sha256(path.read_bytes()).hexdigest()

    tracks = {name: load_rows(HERE/name, 'Alpha')[0] for name in ('previous', 'attempt')}
    tracks['reliable'] = load_rows(retry, 'Alpha')[0]
    start = max(rows[0]['stamp_ns'] for rows in tracks.values())
    end = min(rows[-1]['stamp_ns'] for rows in tracks.values())
    output['common_window_ns'] = [start, end]
    output['common_window_s'] = (end-start)/1e9
    output['common_window'] = {}
    for name, rows in tracks.items():
        selected = [dict(r, component='Alpha') for r in rows if start<=r['stamp_ns']<=end]
        metric = evaluate(dict(poses=selected), {'Alpha': ground_truth(HERE/'alpha_gt.txt')},
            dict(evo_max_diff_s=.05), root/'evo-common-window'/name)
        output['common_window'][name] = dict(frames=len(selected), quality=assess(selected), evo=metric)
    (root/'frontend-audit.json').write_text(json.dumps(output, indent=2)+'\n')

    origin = tracks['attempt'][0]['stamp_ns']
    fig, axes = plt.subplots(3, 1, figsize=(10, 9), layout='constrained')
    styles = [('previous', '#286fa8', 'Calibration noise + reliable'),
              ('attempt', '#d04a37', 'Requested noise + best effort'),
              ('reliable', '#22884b', 'Requested noise + reliable')]
    for name, color, label in styles:
        rows = tracks[name]
        stamps = np.array([r['stamp_ns'] for r in rows], dtype=np.int64)
        xyz = np.array([np.asarray(r['T_world_body']).reshape(4,4)[:3,3] for r in rows])
        speed = np.linalg.norm(np.diff(xyz, axis=0), axis=1)/(np.diff(stamps)/1e9)
        axes[0].plot((stamps[1:]-origin)/1e9, speed, color=color, label=label, linewidth=1)
        if name == 'previous':
            continue
        base = retry if name == 'reliable' else HERE/name
        a, _ = analytics(base/'Alpha/analytics.jsonl')
        t = [(r['last_observed_odometry_stamp_ns']-origin)/1e9 for r in a]
        axes[1].plot(t, [r['native']['num_feats'] for r in a], color=color, linewidth=.8, label=label)
        axes[2].plot(t[1:], [r['native']['lid_freq'] for r in a[1:]], color=color, linewidth=.8, label=label)
    axes[0].axhline(20, color='black', linestyle='--', label='20 m/s guard')
    axes[0].set_yscale('symlog', linthresh=1)
    axes[0].set_ylim(bottom=0)
    axes[0].set_ylabel('Scan-derived speed [m/s]')
    axes[0].legend(fontsize=9)
    axes[0].set_title('S3E Library 2 / Alpha: restoring reliable sensor input')
    axes[1].set_ylabel('Native matched features')
    axes[1].set_title('Native analytics time association is approximate', fontsize=10)
    axes[2].set_ylabel('Native LiDAR receipt rate [Hz]')
    axes[2].axhline(10, color='gray', linestyle=':', label='Configured 10 Hz')
    for ax in axes:
        ax.set_xlabel('Time from best-effort trial first exported scan [s]')
        ax.grid(alpha=.2)
    fig.savefig(root/'diagnostics.png', dpi=160)
    fig.savefig(root/'diagnostics.pdf')
    plt.close(fig)

    from evo.tools import file_interface
    robots = list(output['raw_odometry'])
    fig, axes = plt.subplots(1, len(robots), figsize=(7*len(robots), 6), layout='constrained', squeeze=False)
    for ax, robot in zip(axes[0], robots):
        path = root/'evo-full'/robot
        reference = file_interface.read_tum_trajectory_file(path/f'{robot}-reference-matched.tum')
        estimate = file_interface.read_tum_trajectory_file(path/f'{robot}-estimate-aligned.tum')
        offset = reference.positions_xyz[0]
        a = reference.positions_xyz-offset
        b = estimate.positions_xyz-offset
        result = output['raw_odometry'][robot]
        ax.plot(a[:, 0], a[:, 1], color='black', linestyle='--', label='Position GT', linewidth=1)
        ax.plot(b[:, 0], b[:, 1], color='#22884b' if result['quality']['passed'] else '#d04a37', label='Raw EllipseLIO, evo rigid fit', linewidth=1)
        status = 'passed motion guard' if result['quality']['passed'] else 'FAILED motion guard'
        ax.set_title(f'{robot}: {result["evo"][robot]["rmse_m"]:.2f} m ATE\n{status}')
        ax.set_xlabel('GT-relative x [m]'); ax.set_ylabel('GT-relative y [m]')
        ax.set_aspect('equal', adjustable='datalim'); ax.grid(alpha=.2); ax.legend(fontsize=9)
    fig.suptitle('Library 2: requested noise + reliable input; separate per-robot SE(3) fits')
    fig.savefig(root/'trajectories.png', dpi=160)
    fig.savefig(root/'trajectories.pdf')
    plt.close(fig)
    print(json.dumps(output, indent=2))


if __name__ == '__main__':
    main()
