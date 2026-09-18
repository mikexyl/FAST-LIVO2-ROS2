"""Evaluate frozen CBS poses without reading retired cloud geometry or optimizing."""
from pathlib import Path
from datetime import datetime, timezone
import hashlib
import numpy as np
import yaml
from s3e_pipeline.artifacts import read_json, read_jsonl, write_json
from s3e_pipeline.geometry import pose
from s3e_pipeline.evaluation import corrected_dense_rows, save_tum
from s3e_pipeline.dpgo_evaluation import evaluation_ground_truth
from s3e_pipeline.evo_evaluation import evaluate

base = Path('/workspace/.ros2/ellipsoid-cbs148/overnight-20260916')
queue = read_json(base/'queue.json')
for sequence in ('S3E_Square_2', 'S3E_Square_3'):
    work = Path(queue['sequences'][sequence]['work'])
    output = work/'trajectory-audit-20260917'
    output.mkdir(exist_ok=False)
    cfg = yaml.safe_load((work/'config.yaml').read_text())
    artifacts = read_json(work/'artifacts.json')
    dpgo = Path(artifacts['dpgo'])
    hashes = {}
    def checked(path, marker, relative):
        digest = hashlib.sha256(path.read_bytes()).hexdigest()
        expected = read_json(marker)['files'][relative]
        assert digest == expected, str(path)
        hashes[str(path)] = dict(sha256=digest, manifest=str(marker))
    for name in ('poses.jsonl', 'constraints.jsonl', 'summary.json', 'pcm.json'):
        checked(dpgo/name, dpgo/'COMPLETE.json', name)
    poses = read_jsonl(dpgo/'poses.jsonl')
    loops = read_jsonl(dpgo/'constraints.jsonl')
    summary = read_json(dpgo/'summary.json')
    assert summary['pcm_enabled'] and summary['mode'] == 'peers'
    components = {r: next(p['component'] for p in poses if p['robot_id'] == r) for r in cfg['robots']}
    assert all(p['component'] == components[p['robot_id']] and p['reference_available'] for p in poses)
    assert all(components[e['i'][0]] == components[e['j'][0]] for e in loops)
    ids = {(p['robot_id'], p['keyframe_id']) for p in poses}
    assert len(ids) == len(poses)
    assert all(tuple(e[k]) in ids for e in loops for k in ('i', 'j'))
    dense = []; raw_dense = []; coverage = {}
    for robot in cfg['robots']:
        stage = Path(artifacts[f'keyframes.ellipselio.{robot}'])
        keys_path = stage/'store/keyframes.jsonl'
        checked(keys_path, stage/'COMPLETE.json', 'store/keyframes.jsonl')
        frames_path = work/robot/'export/frames.jsonl'
        checked(frames_path, frames_path.parent/'manifest.json', 'frames.jsonl')
        keys = read_jsonl(keys_path); frames = read_jsonl(frames_path)
        optimized = [p for p in poses if p['robot_id'] == robot]
        assert [(p['keyframe_id'], p['stamp_ns']) for p in keys] == [(p['keyframe_id'], p['stamp_ns']) for p in optimized]
        for rows in (keys, frames, optimized):
            assert np.all(np.diff([p['stamp_ns'] for p in rows]) > 0)
            transforms = np.stack([pose(p['T_world_body']) for p in rows])
            rotations = transforms[:, :3, :3]
            assert np.isfinite(transforms).all()
            assert np.max(np.abs(rotations.transpose(0, 2, 1) @ rotations - np.eye(3))) < 1e-6
            assert np.max(np.abs(np.linalg.det(rotations) - 1)) < 1e-6
            assert np.max(np.abs(transforms[:, 3] - [0, 0, 0, 1])) < 1e-8
        track = [dict(p, component=components[robot]) for p in corrected_dense_rows(frames, keys, optimized)]
        assert [p['stamp_ns'] for p in track] == [p['stamp_ns'] for p in frames]
        saved = work/'report'/f'{robot}-corrected.jsonl'
        if saved.exists():
            old = read_jsonl(saved)
            assert [p['stamp_ns'] for p in old] == [p['stamp_ns'] for p in track]
            assert np.allclose([pose(p['T_world_body']) for p in old], [pose(p['T_world_body']) for p in track], rtol=0, atol=1e-10)
        save_tum(output/f'{robot}-corrected.tum', track)
        dense += track; raw_dense += [dict(p, component=robot) for p in frames]
        coverage[robot] = dict(keyframes=len(keys), dense_poses=len(frames), first_stamp_ns=frames[0]['stamp_ns'], last_stamp_ns=frames[-1]['stamp_ns'], previous_corrected_trajectory_matched=saved.exists())
    truth, gt_status = evaluation_ground_truth(cfg['robots'], Path(cfg['dataset']), dense, cfg['evaluation'])
    for robot in cfg['robots']:
        gt_file = Path(cfg['dataset'])/f'{robot.lower()}_gt.txt'
        hashes[str(gt_file)] = dict(sha256=hashlib.sha256(gt_file.read_bytes()).hexdigest())
    metrics = evaluate(dict(poses=dense), truth, cfg['evaluation'], output/'evo/cbs')
    raw = evaluate(dict(poses=raw_dense), truth, cfg['evaluation'], output/'evo/raw_odometry')
    saved_report = work/'report/report.json'
    if saved_report.exists():
        old = read_json(saved_report)
        for component, values in metrics.items():
            assert abs(values['rmse_m'] - old['trajectory'][component]['rmse_m']) < 1e-10
            for robot, robot_values in values['per_robot'].items():
                assert abs(robot_values['statistics']['rmse'] - old['trajectory'][component]['per_robot'][robot]['statistics']['rmse']) < 1e-10
    result = dict(sequence=sequence, created_utc=datetime.now(timezone.utc).isoformat(), original_queue_status=queue['sequences'][sequence]['status'], scope='Trajectory-only recovery from unchanged CBS outputs; no frontend, retrieval, PCM, optimization or map rebuilding rerun. Original failed report status preserved.', components=components, trajectory=metrics, raw_odometry=raw, ground_truth=gt_status, coverage=coverage, input_hashes=hashes, runtime=summary, saved_report_metrics_reproduced=saved_report.exists())
    write_json(output/'audit.json', result)
    print(sequence, {c:dict(rmse_m=m['rmse_m'], per_robot={r:v['statistics']['rmse'] for r,v in m['per_robot'].items()}, samples=m['samples']) for c,m in metrics.items()}, flush=True)
    write_json(output/'COMPLETE.json', dict(scope='trajectory audit only', files={str(p.relative_to(output)):hashlib.sha256(p.read_bytes()).hexdigest() for p in output.rglob('*') if p.is_file()}))
