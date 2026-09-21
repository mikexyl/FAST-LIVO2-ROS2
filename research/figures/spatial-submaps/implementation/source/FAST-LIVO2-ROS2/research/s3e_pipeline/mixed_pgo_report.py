"""Compact evo comparison of a completed mixed graph and its pose-only seed."""
import argparse
from pathlib import Path
import shutil

import numpy as np

from .artifacts import read_json, read_jsonl, write_json, file_hash, validate_stage
from .evaluation import corrected_dense_rows, ground_truth, save_tum
from .evo_evaluation import evaluate
from .geometry import pose, transform, voxel_downsample


def collect(registry, output):
    registry = Path(registry); output = Path(output); output.mkdir(parents=True, exist_ok=True)
    run = read_json(registry); artifacts = run['artifacts']; cfg = run['config']
    stage = Path(artifacts[f'pgo.livo.{cfg["backend"]["name"]}'])
    validate_stage(stage)
    graph = read_json(stage/'graph.json'); native = graph['registration']
    sources = {str(registry): file_hash(registry), str(stage/'graph.json'): file_hash(stage/'graph.json')}
    truth = {}; tracks = {'pose_only': [], 'mixed': []}; keys = {}; stores = {}
    for robot in cfg['robots']:
        stores[robot] = Path(artifacts[f'keyframes.livo.{robot}'])/'store'
        keys[robot] = read_jsonl(stores[robot]/'keyframes.jsonl')
        frame_path = Path(artifacts[f'odometry.livo.{robot}'])/'run/export/frames.jsonl'
        frames = read_jsonl(frame_path); sources[str(frame_path)] = file_hash(frame_path)
        sources[str(stores[robot]/'keyframes.jsonl')] = file_hash(stores[robot]/'keyframes.jsonl')
        truth_path = Path(cfg['dataset'])/f'{robot.lower()}_gt.txt'
        truth[robot] = ground_truth(truth_path); sources[str(truth_path)] = file_hash(truth_path)
        poses = [r for r in graph['poses'] if r['robot_id'] == robot]
        for mode, field in [('pose_only', 'T_pose_only_body'), ('mixed', 'T_world_body')]:
            local = [dict(r, T_world_body=r[field]) for r in poses]
            dense = list(corrected_dense_rows(frames, keys[robot], local))
            for r in dense: r['component'] = graph['components'][robot]
            tracks[mode].extend(dense); save_tum(output/f'{robot}-{mode}.tum', dense)
    metrics = {mode: evaluate(dict(poses=rows), truth, cfg['evaluation'], output/'evo'/mode)
               for mode, rows in tracks.items()}
    report = dict(dataset=cfg['dataset'], config=cfg, trajectory=metrics, registration=native,
        stage_hash=read_json(stage/'COMPLETE.json')['stage_hash'],
        selected_pose_loops=sum(f['kind'] == 'loop' and f['solver_weight'] > 0 for f in graph['factors']),
        rejected_pose_loops=sum(f['kind'] == 'loop' and f['solver_weight'] == 0 for f in graph['factors']),
        components=graph['components'], sources=sources,
        map_figure_preprocessing='every fifth saved keyframe full scan, body coordinates; 0.5 m display voxels; not a map accuracy metric')
    for name in ('graph.json', 'registration.json', 'native-result.json', 'pose-only-graph.json'):
        shutil.copy2(stage/name, output/name)
    shutil.copy2(stage/'COMPLETE.json', output/'pgo-stage-manifest.json')
    shutil.copy2(registry, output/'run.json')

    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    colors = {'Alpha': '#db504a', 'Bob': '#3b8dcc', 'Carol': '#319e65'}
    origin = truth['Alpha'][1][0]
    fig, axes = plt.subplots(1, 2, figsize=(12, 5.5))
    for ax, mode in zip(axes, ('pose_only', 'mixed')):
        for robot in cfg['robots']:
            component = graph['components'][robot]; m = metrics[mode][component]
            xyz = transform(pose(m['alignment_SE3']), np.array([pose(r['T_world_body'])[:3, 3]
                for r in tracks[mode] if r['robot_id'] == robot])) - origin
            ax.plot(xyz[:, 0], xyz[:, 1], color=colors[robot], linewidth=1.3, label=robot)
            gt = truth[robot][1] - origin
            gt = np.insert(gt, np.flatnonzero(np.diff(truth[robot][0]) > 2e9) + 1, np.nan, axis=0)
            ax.plot(gt[:, 0], gt[:, 1], ':', color=colors[robot], alpha=.45)
        label = 'Pose factors only' if mode == 'pose_only' else 'Pose + live GICP factors'
        ax.set(title=label, xlabel='East from local origin [m]', ylabel='North from local origin [m]')
        ax.set_aspect('equal'); ax.grid(alpha=.2); ax.legend()
    fig.suptitle('S3E Square 1: centralized optimization | dotted: position ground truth')
    fig.tight_layout(); fig.savefig(output/'trajectories.png', dpi=180); fig.savefig(output/'trajectories.pdf'); plt.close(fig)

    # Display exactly the same sampled scan points under the two solutions.
    maps = {mode: {} for mode in ('pose_only', 'mixed')}
    poses = {(r['robot_id'], r['keyframe_id']): r for r in graph['poses']}
    for robot in cfg['robots']:
        parts = {mode: [] for mode in maps}
        for index, row in enumerate(keys[robot][::5]):
            path = stores[robot]/f'{row["keyframe_id"]:06d}.npz'
            sources[str(path)] = file_hash(path)
            with np.load(path, allow_pickle=False) as data: scan = voxel_downsample(data['scan'][:, :3], .5)
            solved = poses[robot, row['keyframe_id']]
            for mode, field in [('pose_only', 'T_pose_only_body'), ('mixed', 'T_world_body')]:
                alignment = pose(metrics[mode][graph['components'][robot]]['alignment_SE3'])
                parts[mode].append(transform(alignment@pose(solved[field]), scan) - origin)
                if index % 25 == 24: parts[mode] = [voxel_downsample(np.concatenate(parts[mode]), .5)]
        for mode in maps: maps[mode][robot] = voxel_downsample(np.concatenate(parts[mode]), .5)
    fig, axes = plt.subplots(1, 2, figsize=(12, 5.5), sharex=True, sharey=True)
    for ax, mode in zip(axes, maps):
        for robot, xyz in maps[mode].items(): ax.scatter(xyz[:, 0], xyz[:, 1], s=.12, color=colors[robot], alpha=.4, rasterized=True)
        ax.set_aspect('equal'); ax.set(xlabel='East from local origin [m]', ylabel='North from local origin [m]',
            title='Pose factors only' if mode == 'pose_only' else 'Pose + live GICP factors')
    fig.suptitle('Same full-LiDAR scan samples under each optimized trajectory')
    fig.tight_layout(); fig.savefig(output/'maps.png', dpi=180); fig.savefig(output/'maps.pdf'); plt.close(fig)
    report['input_files_unchanged'] = all(file_hash(p) == h for p, h in sources.items())
    if not report['input_files_unchanged']: raise ValueError('Input changed during evaluation')
    write_json(output/'report.json', report)
    write_json(output/'files.json', {str(p.relative_to(output)): file_hash(p) for p in sorted(output.rglob('*'))
        if p.is_file() and p.name != 'files.json'})
    print({mode: {c: dict(rmse=m['rmse_m'], robots={r:v['statistics']['rmse'] for r,v in m['per_robot'].items()})
                  for c,m in values.items()} for mode, values in metrics.items()})


if __name__ == '__main__':
    parser = argparse.ArgumentParser(); parser.add_argument('--input-run', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path); args = parser.parse_args()
    collect(args.input_run, args.output)
