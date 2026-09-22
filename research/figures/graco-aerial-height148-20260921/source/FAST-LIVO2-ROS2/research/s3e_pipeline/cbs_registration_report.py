"""Compact evo comparison of frozen CBS pose-only and live GICP runs."""
import argparse
from pathlib import Path
import shutil

import numpy as np

from .artifacts import read_json, read_jsonl, write_json, file_hash, validate_stage
from .evaluation import corrected_dense_rows, ground_truth, save_tum
from .evo_evaluation import evaluate
from .geometry import pose, transform


def collect(registry, baseline, output):
    output = Path(output); output.mkdir(parents=True, exist_ok=True)
    registries = {'pose_only': Path(baseline).resolve(), 'mixed': Path(registry).resolve()}
    runs = {mode: read_json(p) for mode, p in registries.items()}
    cfg = runs['mixed']['config']; artifacts = runs['mixed']['artifacts']
    sources = {}; tracks = {}; truth = {}; summaries = {}; stages = {}
    def record(path):
        path = Path(path); sources[str(path)] = file_hash(path); return path
    for mode, run in runs.items():
        stage = Path(run['artifacts']['dpgo']); validate_stage(stage); stages[mode] = stage
        assert run['config']['dpgo']['mode'] == 'frozen'
        assert run['config']['dpgo']['pcm']['enabled']
        assert run['config']['robots'] == cfg['robots']
        for label in (k for k in artifacts if k.startswith(('odometry.', 'keyframes.', 'loops.'))):
            assert run['artifacts'][label] == artifacts[label], 'comparison requires identical frozen inputs'
        local = output/mode; local.mkdir(exist_ok=True)
        for name in ('summary.json', 'poses.jsonl', 'constraints.jsonl', 'pcm.json'):
            shutil.copy2(record(stage/name), local/name)
        shutil.copy2(record(stage/'COMPLETE.json'), local/'stage-manifest.json')
        shutil.copy2(record(registries[mode]), local/'run.json')
        summaries[mode] = read_json(stage/'summary.json')
        poses = read_jsonl(stage/'poses.jsonl'); tracks[mode] = []
        for robot in cfg['robots']:
            keys = read_jsonl(record(Path(artifacts[f'keyframes.livo.{robot}'])/'store/keyframes.jsonl'))
            frames = read_jsonl(record(Path(artifacts[f'odometry.livo.{robot}'])/'run/export/frames.jsonl'))
            rows = [r for r in poses if r['robot_id'] == robot]
            assert [r['keyframe_id'] for r in rows] == [r['keyframe_id'] for r in keys]
            dense = list(corrected_dense_rows(frames, keys, rows))
            for row in dense: row['component'] = rows[0]['component']
            tracks[mode].extend(dense); save_tum(local/f'{robot}.tum', dense)
            truth[robot] = ground_truth(record(Path(cfg['dataset'])/f'{robot.lower()}_gt.txt'))
            shutil.copy2(record(stage/robot/'stats.jsonl'), local/f'{robot}-stats.jsonl')
    reg = read_json(record(stages['mixed']/'registration.json'))
    shutil.copy2(stages['mixed']/'registration.json', output/'mixed/registration.json')
    actual_bytes = 0; source_clouds = {}; owned_clouds = {}; wire_counts = {}
    for robot in cfg['robots']:
        stage = stages['mixed']/robot; local = output/'mixed'/robot; local.mkdir(exist_ok=True)
        for name in ('transport.json', 'manifest.json'):
            shutil.copy2(record(stage/'registration'/name), local/name)
        shutil.copy2(record(stage/'wire.jsonl'), local/'wire.jsonl')
        transport = read_json(stage/'registration/transport.json')
        for item in transport['sources']:
            assert file_hash(item['source_npz']) == item['source_sha256']
            source_clouds[item['source_npz']] = item['source_sha256']
        owned_clouds[robot] = transport['cloud_count']
        wire = [e for e in read_jsonl(stage/'wire.jsonl') if e['kind'].startswith('registration_')]
        actual_bytes += sum(e['network_bytes'] for e in wire)
        wire_counts[robot] = {kind: sum(e['kind'] == kind for e in wire)
            for kind in sorted({e['kind'] for e in wire})}
        assert not list((stage/'registration').glob('*.bin'))
    assert actual_bytes == reg['network_cdr_bytes'] == summaries['mixed']['registration_network_cdr_bytes']
    pairs = [p for r in reg['robots'].values() for p in r['pairs']]
    accepted = [p for p in pairs if p['accepted']]
    assert len(accepted) == reg['registration_factor_count']
    assert all(p['final_quality_reason'] == 'accepted' for p in accepted)
    metrics = {mode: evaluate(dict(poses=rows), truth, cfg['evaluation'], output/mode/'evo')
               for mode, rows in tracks.items()}
    report = dict(dataset=cfg['dataset'], configuration=cfg, trajectory=metrics, runtime=summaries,
        registration=reg, source_clouds=source_clouds, sources=sources,
        owned_cloud_copies=owned_clouds, registration_wire_messages=wire_counts,
        geometry=dict(unique_source_clouds=len(source_clouds),
            initial_rejected=len(pairs)-len(accepted), final_rejected=0,
            min_final_overlap=min((p['final_quality']['overlap'] for p in accepted), default=None),
            linearizations_min=min((p['linearizations'] for p in accepted), default=0),
            linearizations_max=max((p['linearizations'] for p in accepted), default=0)),
        comparison_limitation='Pose-only is the retained earlier frozen PCM/CBS run. Both use 100 asynchronous local updates; mixed CBS forces fresh GICP/iSAM2 linearizations. This is not a controlled timing or convergence comparison.',
        evaluation_scope='evo nearest timestamps within 0.05 s, one shared SE3 alignment per connected component, no scale; position-only ground truth')
    report['inputs_unchanged'] = all(file_hash(p) == h for p,h in dict(sources, **source_clouds).items())
    assert report['inputs_unchanged']
    write_json(output/'report.json', report)

    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    colors = {'Alpha': '#db504a', 'Bob': '#3b8dcc', 'Carol': '#319e65'}
    origin = truth['Alpha'][1][0]
    fig, axes = plt.subplots(1, 2, figsize=(12, 5.5), sharex=True, sharey=True)
    for ax, mode in zip(axes, ('pose_only', 'mixed')):
        for robot in cfg['robots']:
            rows = [r for r in tracks[mode] if r['robot_id'] == robot]
            alignment = pose(metrics[mode][rows[0]['component']]['alignment_SE3'])
            xyz = transform(alignment, np.array([pose(r['T_world_body'])[:3,3] for r in rows])) - origin
            ax.plot(xyz[:,0], xyz[:,1], color=colors[robot], label=robot, linewidth=1.2)
            gt = truth[robot][1] - origin
            gt = np.insert(gt, np.flatnonzero(np.diff(truth[robot][0]) > 2e9)+1, np.nan, axis=0)
            ax.plot(gt[:,0], gt[:,1], ':', color=colors[robot], alpha=.5)
        ax.set_aspect('equal'); ax.grid(alpha=.2); ax.legend()
        ax.set(xlabel='East from local origin [m]', ylabel='North from local origin [m]',
            title='PCM + CBS pose factors' if mode == 'pose_only' else 'PCM + CBS pose and live GICP factors')
    fig.suptitle('S3E Square 1 | dotted: supplied position ground truth')
    fig.tight_layout(); fig.savefig(output/'trajectories.png', dpi=180); fig.savefig(output/'trajectories.pdf'); plt.close(fig)
    write_json(output/'files.json', {str(p.relative_to(output)):file_hash(p)
        for p in sorted(output.rglob('*')) if p.is_file() and p.name != 'files.json'})
    print({mode: {c: dict(rmse=m['rmse_m'], robots={r:v['statistics']['rmse'] for r,v in m['per_robot'].items()})
        for c,m in values.items()} for mode, values in metrics.items()})
    print(dict(wall_s=summaries['mixed']['wall_s'], geometry=report['geometry'], registration_bytes=actual_bytes))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(); parser.add_argument('--input-run', required=True, type=Path)
    parser.add_argument('--baseline-run', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args(); collect(args.input_run, args.baseline_run, args.output)
