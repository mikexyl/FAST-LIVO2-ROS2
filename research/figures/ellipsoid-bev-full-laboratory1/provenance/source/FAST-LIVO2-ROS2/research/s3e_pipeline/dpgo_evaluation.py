"""Evaluate owner-exported CBS poses. Ground truth is used only here."""
from pathlib import Path
from .frontends import frontend_name
import time
import numpy as np
from .artifacts import read_json, read_jsonl, write_json, write_jsonl
from .geometry import pose, inv
from .cbs_bridge import local_graph
from .evaluation import ground_truth, trajectory_metrics, corrected_dense_rows, save_tum, build_maps, retrieval_metrics


def evaluation_ground_truth(robots, dataset, dense, cfg):
    """Keep unavailable/out-of-sequence GT explicit, without blocking map export."""
    tracks = {}; status = {}
    for robot in robots:
        path = Path(dataset)/f'{robot.lower()}_gt.txt'
        rows = [r for r in dense if r['robot_id'] == robot]
        info = dict(path=str(path), available=False, matched_samples=0, pose_samples=len(rows))
        try:
            stamps, xyz = ground_truth(path)
            if not len(stamps) or xyz.shape != (len(stamps), 3) or not np.isfinite(xyz).all() or np.any(np.diff(stamps) <= 0):
                raise ValueError('Position GT must contain finite positions and strictly increasing timestamps')
            tracks[robot] = (stamps, xyz)
            from .evo_evaluation import associate, sync
            try:
                _, associated = associate(rows, tracks[robot], cfg.get('evo_max_diff_s', .05))
                matches = associated.num_poses
            except sync.SyncException:
                matches = 0
            info.update(rows=len(stamps), first_stamp_ns=int(stamps[0]), last_stamp_ns=int(stamps[-1]),
                        matched_samples=matches, available=matches >= 3)
            if matches < 3:
                info['reason'] = 'insufficient_timestamp_matched_positions'
                if rows and (stamps[-1] < min(r['stamp_ns'] for r in rows) or stamps[0] > max(r['stamp_ns'] for r in rows)):
                    info['reason'] = 'no_timestamp_overlap_with_sensor_trajectory'
        except (OSError, ValueError, ArithmeticError) as exc:
            tracks[robot] = (np.empty(0, dtype=np.int64), np.empty((0, 3)))
            info.update(reason='missing_or_invalid_position_ground_truth', detail=str(exc))
        status[robot] = info
    return tracks, status


def evaluate(cfg, artifacts, dataset, output):
    import gtsam
    start = time.monotonic(); output = Path(output); dpgo = Path(artifacts['dpgo'])
    summary = read_json(dpgo/'summary.json'); poses = read_jsonl(dpgo/'poses.jsonl')
    loops = read_jsonl(dpgo/'constraints.jsonl'); components = {p['robot_id']: p['component'] for p in poses}
    # No align_robots call: common-frame transforms come exclusively from CBS.
    if any(components[e['i'][0]] != components[e['j'][0]] for e in loops):
        raise ValueError('CBS failed to establish a common frame for a connected component')
    dense = []; raw_dense = []; all_keys = []; factors = list(loops); central_dense = []; events = []
    central_label = f'pgo.{frontend_name(cfg)}.{cfg["backend"]["name"]}'
    central = read_json(Path(artifacts[central_label])/'graph.json') if central_label in artifacts else None
    for robot in cfg['robots']:
        store = Path(artifacts[f'keyframes.{frontend_name(cfg)}.{robot}'])/'store'
        keys = read_jsonl(store/'keyframes.jsonl'); all_keys += keys
        rows = [p for p in poses if p['robot_id'] == robot]
        # Original map uses the first optimized pose as a display origin only.
        # Neither this transform nor evaluation alignment feeds back to CBS.
        display_alignment = pose(rows[0]['T_world_body']) @ inv(pose(keys[0]['T_world_body']))
        for row, key in zip(rows, keys): row['T_initial_body'] = (display_alignment @ pose(key['T_world_body'])).tolist()
        export = Path(artifacts[f'odometry.{frontend_name(cfg)}.{robot}'])/'run/export'
        raw_dense += [dict(r,component=robot) for r in read_jsonl(export/'frames.jsonl')]
        original, optimized, track = build_maps(export, keys, rows, cfg['evaluation']['map_voxel_m'], store)
        for r in track: r['component'] = components[robot]
        dense += track
        save_tum(output/f'{robot}-corrected.tum', track); write_jsonl(output/f'{robot}-corrected.jsonl', track)
        np.savez_compressed(output/f'{robot}-maps.npz', original=original.astype(np.float32), optimized=optimized.astype(np.float32))
        _, odom = local_graph(keys, robot, cfg['pgo']); factors += odom
        events += read_jsonl(dpgo/robot/'events.jsonl')
        if central:
            crows = [p for p in central['poses'] if p['robot_id'] == robot]
            ctrack = list(corrected_dense_rows(read_jsonl(export/'frames.jsonl'), keys, crows))
            for r in ctrack: r['component'] = central['components'][robot]
            central_dense += ctrack
        print(f'CBS evaluation: {robot}, {len(track)} poses and {len(optimized)} map points', flush=True)
    gt, gt_status = evaluation_ground_truth(cfg['robots'], dataset, dense, cfg['evaluation'])
    lookup = {(p['robot_id'], p['keyframe_id']): pose(p['T_world_body']) for p in poses}
    for edge in factors:
        error = gtsam.Pose3.Logmap(gtsam.Pose3(inv(pose(edge['T_i_j'])) @ inv(lookup[tuple(edge['i'])]) @ lookup[tuple(edge['j'])]))
        edge['squared_whitened_residual'] = float(error @ np.asarray(edge['information']) @ error)
        edge['translation_residual_m'] = float(np.linalg.norm(error[3:]))
        edge['rotation_residual_rad'] = float(np.linalg.norm(error[:3]))
    stats = {r: read_jsonl(dpgo/r/'stats.jsonl') for r in cfg['robots']}
    report = dict(runtime=summary, trajectory=trajectory_metrics(dict(poses=dense), gt, cfg['evaluation'], output/'evo/cbs'),
        raw_odometry=trajectory_metrics(dict(poses=raw_dense), gt, cfg['evaluation'], output/'evo/raw_odometry'),
        frontend=frontend_name(cfg),
        centralized_reference=trajectory_metrics(dict(poses=central_dense), gt, cfg['evaluation'], output/'evo/centralized') if central else None,
        components=components, factor_count=len(factors), loop_count=len(loops),
        gaussian_cost=.5*sum(e['squared_whitened_residual'] for e in factors),
        cost_scope='post-hoc evaluation of original measurements; excludes CBS belief factors and gauge priors',
        convergence={r:dict(final_pose_change=s[-1]['result_logmap_change'],
            last_10_max_pose_change=max(x['result_logmap_change'] for x in s[-10:]),
            last_10_received_beliefs=sum(x['num_received_beliefs'] for x in s[-10:])) for r, s in stats.items()},
        evaluation_wall_s=time.monotonic()-start, orientation_ground_truth_used=False,
        dataset=str(dataset), ground_truth=gt_status,
        trajectory_evaluation=read_json(output/'evo/cbs/evaluation.json'))
    report['trajectory_metrics_status'] = 'available' if report['trajectory'] else 'unavailable: see evo component status'
    if summary.get('pcm_enabled'):
        report['pcm'] = read_json(dpgo/'pcm.json')
    if summary.get('registration_enabled'):
        report['registration'] = read_json(dpgo/'registration.json')
        report['pose_factor_count'] = len(factors)
        report['registration_factor_count'] = report['registration']['registration_factor_count']
        report['factor_count'] += report['registration_factor_count']
        report['cost_scope'] += '; pose-only diagnostic, excludes live GICP costs recorded per owner in registration'
    if events:
        report['retrieval'] = retrieval_metrics(all_keys, events, gt, cfg['evaluation'], cfg['loops']['same_robot_exclusion_s'])
        latencies = [e['wall_detection_latency_s'] for e in events if 'wall_detection_latency_s' in e]
        if latencies:
            report['retrieval'].update(wall_detection_latency_median_s=float(np.median(latencies)),
                wall_detection_latency_p95_s=float(np.quantile(latencies,.95)))
        if not any(v['available'] for v in gt_status.values()):
            report['retrieval']['metric_label'] = 'position-proximity metrics unavailable: no usable timestamp-matched ground truth'
        from collections import Counter
        verifications = [e for e in events if e['type'] == 'verification']
        branch = lambda e: '+'.join(sorted(e.get('retrieval_sources', []))) or 'unknown'
        report['branch_attribution'] = dict(
            verification_attempts=dict(Counter(branch(e) for e in verifications)),
            accepted_constraints=dict(Counter(branch(e['diagnostics']) for e in loops)))
        report['verification_reasons'] = dict(Counter(e['reason'] for e in verifications))
    write_json(output/'report.json', report); write_jsonl(output/'factors.jsonl', factors); write_jsonl(output/'poses.jsonl', poses)
    comparison_plot(cfg, dense, central_dense, gt, report, output)
    from .cli import SOURCE
    import subprocess
    write_json(output/'visualization-input.json', dict(config=cfg, artifacts=artifacts, output='.'))
    subprocess.run([str(SOURCE/'.ros2/rerun-venv/bin/python'), '-m', 's3e_pipeline.dpgo_visualize',
                    str(output/'visualization-input.json')], check=True)
    write_json(output/'report.json', dict(report, evaluation_wall_s=time.monotonic()-start))


def comparison_plot(cfg, dense, central, gt, report, output):
    import csv
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from .geometry import transform
    COLORS={'Alpha':[235,85,80],'Bob':[70,170,245],'Carol':[95,205,130]}
    with (output/'metrics.csv').open('w') as f:
        writer = csv.writer(f); writer.writerow(['method','component','position_rmse_m','samples','status'])
        for label, metrics in [('CBS DDS',report['trajectory']),('Raw odometry (individual alignment)',report.get('raw_odometry',{})),('Centralized reference',report['centralized_reference'])]:
            for component, values in (metrics or {}).items():
                writer.writerow([label,component,values['rmse_m'],values['samples'],'available'])
        for component in sorted(set(report['components'].values()) - set(report['trajectory'])):
            writer.writerow(['CBS DDS',component,'',0,'unavailable: no usable timestamp-matched ground truth'])
    components = sorted(set(report['components'].values()))
    fig, axes = plt.subplots(1, len(components), figsize=(9*len(components),7), squeeze=False)
    for ax, component in zip(axes[0], components):
        metrics = report['trajectory'].get(component)
        robots = [r for r in cfg['robots'] if report['components'][r] == component]
        origin = gt[metrics['robots'][0]][1][0] if metrics else np.zeros(3)
        alignment = pose(metrics['alignment_SE3']) if metrics else np.eye(4)
        for robot in robots:
            rows = [r for r in dense if r['robot_id']==robot]
            xyz = transform(alignment, np.array([pose(r['T_world_body'])[:3,3] for r in rows]))-origin
            color = np.asarray(COLORS[robot])/255.
            ax.plot(xyz[:,0],xyz[:,1],color=color,label=f'{robot} CBS')
            if metrics and report['ground_truth'][robot]['available']:
                truth = np.insert(gt[robot][1]-origin, np.flatnonzero(np.diff(gt[robot][0]) > cfg['evaluation']['gt_max_gap_s']*1e9)+1, np.nan, axis=0)
                ax.plot(truth[:,0],truth[:,1],color=color,linestyle=':',alpha=.5)
            if central and metrics:
                crows=[r for r in central if r['robot_id']==robot]
                cm=report['centralized_reference'][crows[0]['component']]
                cxyz=transform(pose(cm['alignment_SE3']),np.array([pose(r['T_world_body'])[:3,3] for r in crows]))-origin
                ax.plot(cxyz[:,0],cxyz[:,1],color=color,linestyle='--',alpha=.6)
        ax.set_aspect('equal'); ax.grid(alpha=.2); ax.legend()
        ax.set_xlabel('East from local origin [m]' if metrics else 'Component frame x [m]')
        ax.set_ylabel('North from local origin [m]' if metrics else 'Component frame y [m]')
        title = Path(cfg['dataset']).name.replace('_', ' ')
        detail = f'Position RMSE {metrics["rmse_m"]:.3f} m; dotted: position GT' if metrics else 'Position error unavailable: no timestamp-matched ground truth'
        ax.set_title(f'{title} — CBS component {component}\n{detail}')
    fig.tight_layout(); fig.savefig(output/'trajectories.png',dpi=160); plt.close(fig)
