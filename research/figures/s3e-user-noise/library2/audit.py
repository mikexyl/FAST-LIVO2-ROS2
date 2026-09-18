"""Reproduce the stopped Library 2 noise/QoS trial's diagnostics; no estimation."""
from collections import Counter
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


def load_rows(name):
    with gzip.open(HERE/name/'retained/Alpha/frames.jsonl.gz', 'rt') as f:
        return [json.loads(line) for line in f]


def motion(rows, origin):
    stamps = np.array([r['stamp_ns'] for r in rows], dtype=np.int64)
    xyz = np.array([np.asarray(r['T_world_body']).reshape(4, 4)[:3, 3] for r in rows])
    return (stamps[1:]-origin)/1e9, np.linalg.norm(np.diff(xyz, axis=0), axis=1)/(np.diff(stamps)/1e9)


def main():
    tracks = {name: load_rows(name) for name in ('previous', 'attempt')}
    config = {name: yaml.safe_load((HERE/name/'Alpha/runtime.yaml').read_text())['/**']['ros__parameters'] for name in tracks}
    assert config['previous']['lidar'] == config['attempt']['lidar']
    assert 'input' not in config['attempt']
    assert {k: config['attempt']['imu'][k] for k in ('acc_noise', 'gyr_noise', 'acc_bias', 'gyr_bias')} == dict(acc_noise=.1, gyr_noise=.1, acc_bias=.0001, gyr_bias=.0001)
    # Common timestamp bounds only. Failed-prefix evo results are diagnostics,
    # not full-sequence ATE or a controlled isolation of the parameter effects.
    start = max(rows[0]['stamp_ns'] for rows in tracks.values())
    end = min(rows[-1]['stamp_ns'] for rows in tracks.values())
    origin = tracks['attempt'][0]['stamp_ns']
    truth = {'Alpha': ground_truth(HERE/'alpha_gt.txt')}
    out = dict(status='failed; stopped after motion guard violation', robot='Alpha',
        sequence='S3E_Library_2', full_sequence=False, downstream_started=False,
        common_prefix_start_ns=start, common_prefix_end_ns=end,
        common_prefix_s=(end-start)/1e9, calibration_unchanged=True, trials={})
    for name, rows in tracks.items():
        quality = assess(rows)
        if 'first_invalid_stamp_ns' in quality:
            quality['first_invalid_after_first_export_s'] = (quality['first_invalid_stamp_ns']-rows[0]['stamp_ns'])/1e9
        prefix = [dict(r, component='Alpha') for r in rows if start <= r['stamp_ns'] <= end]
        metric = evaluate(dict(poses=prefix), truth, dict(evo_max_diff_s=.05), HERE/'evo-prefix'/name)
        errors = Counter(line.split(']: ', 1)[-1] for line in (HERE/name/'Alpha/mapping.log').read_text().splitlines() if '[ERROR]' in line)
        out['trials'][name] = dict(quality=quality, common_prefix_frames=len(prefix),
            common_prefix_quality=assess(prefix), common_prefix_evo=metric,
            log_error_messages=dict(errors), imu=config[name]['imu'], input=config[name].get('input'))
    analytics = [json.loads(line) for line in (HERE/'attempt/Alpha/analytics.jsonl').read_text().splitlines()]
    out['native_analytics'] = dict(messages=len(analytics),
        time_policy='Latest observed odometry timestamp; native analytics has no exact scan timestamp',
        median_lidar_frequency_hz=float(np.median([r['native']['lid_freq'] for r in analytics[1:]])),
        median_imu_frequency_hz=float(np.median([r['native']['imu_freq'] for r in analytics[1:]])),
        zero_feature_updates=sum(r['native']['num_feats'] == 0 for r in analytics[1:]),
        updates_excluding_initialization=len(analytics)-1,
        median_native_update_ms=1000*float(np.median([r['native']['total_time'] for r in analytics[1:]])))
    sources = [HERE/'alpha_gt.txt', HERE/'attempt/Alpha/analytics.jsonl']
    for name in tracks:
        sources += [HERE/name/'retained/Alpha/frames.jsonl.gz', HERE/name/'Alpha/runtime.yaml', HERE/name/'Alpha/mapping.log']
    out['input_sha256'] = {str(p.relative_to(HERE)): hashlib.sha256(p.read_bytes()).hexdigest() for p in sources}
    (HERE/'audit.json').write_text(json.dumps(out, indent=2)+'\n')

    fig, axes = plt.subplots(3, 1, figsize=(10, 9), layout='constrained')
    for name, color, label in [('previous', '#286fa8', 'Previous calibration noise + reliable'),
                               ('attempt', '#d04a37', 'Requested noise + native best effort')]:
        t, speed = motion(tracks[name], origin)
        axes[0].plot(t, speed, color=color, label=label, linewidth=1)
    axes[0].axhline(20, color='black', linestyle='--', label='20 m/s motion guard')
    axes[0].set_yscale('symlog', linthresh=1)
    axes[0].set_ylim(bottom=0)
    axes[0].set_ylabel('Scan-derived speed [m/s]')
    axes[0].legend(fontsize=9)
    axes[0].set_title('S3E Library 2 / Alpha: requested configuration diverged early')
    t = np.array([(r['last_observed_odometry_stamp_ns']-origin)/1e9 for r in analytics])
    axes[1].plot(t, [r['native']['num_feats'] for r in analytics], color='#d04a37')
    axes[1].set_ylabel('Native matched features')
    axes[1].set_title('Requested trial diagnostics; analytics timestamp association is approximate', fontsize=10)
    axes[2].plot(t[1:], [r['native']['lid_freq'] for r in analytics[1:]], color='#d04a37', label='Native LiDAR receipt-rate diagnostic')
    axes[2].axhline(10, color='gray', linestyle=':', label='Configured LiDAR: 10 Hz')
    axes[2].set_ylabel('LiDAR frequency [Hz]')
    axes[2].legend(fontsize=9)
    for ax in axes:
        ax.set_xlabel('Time from requested trial first exported scan [s]')
        ax.grid(alpha=.2)
    fig.savefig(HERE/'diagnostics.png', dpi=160)
    fig.savefig(HERE/'diagnostics.pdf')
    plt.close(fig)
    print(json.dumps(out, indent=2))


if __name__ == '__main__':
    main()
