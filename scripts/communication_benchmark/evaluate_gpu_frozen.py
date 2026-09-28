#!/usr/bin/env python3
"""Post-hoc dense-trajectory evaluation of the frozen CPU/GPU CBS controls.

Reads no point arrays. Ground truth enters only after the distributed solves.
"""
import argparse
import json
from pathlib import Path
import sys

import numpy as np
import yaml
from scipy.spatial.transform import Rotation

SOURCE = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(SOURCE / 'FAST-LIVO2-ROS2/research'))
from s3e_pipeline.artifacts import file_hash, read_json, read_jsonl, write_json
from s3e_pipeline.dpgo_evaluation import evaluation_ground_truth
from s3e_pipeline.evaluation import corrected_dense_rows, save_tum, trajectory_metrics


def native_rows(trial, robot):
    result = []
    for row in read_jsonl(Path(trial) / 'frontend/native_updates.jsonl'):
        transform = np.eye(4)
        transform[:3, :3] = Rotation.from_quat(row['pose'][3:]).as_matrix()
        transform[:3, 3] = row['pose'][:3]
        result.append(dict(robot_id=robot, component=robot, frame_id=row['scan_id'],
            stamp_ns=row['stamp_ns'], T_world_body=transform.tolist()))
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--input', type=Path, required=True)
    args = parser.parse_args()
    progress = read_json(args.input / 'progress.json')
    if progress['phase'] != 'complete' or len(progress['entries']) != 8:
        raise ValueError('All eight matched solves must complete before final evaluation')
    result = []
    for entry in progress['entries']:
        out = Path(entry['path'])
        cfg = yaml.safe_load((out / 'config.yaml').read_text())
        frozen = read_json(out / 'input.json')
        original = Path(frozen['original']).parents[1]
        trials = read_json(original / 'trials.json')
        graph = read_jsonl(out / 'dpgo/poses.jsonl')
        components = {row['robot_id']: row['component'] for row in graph}
        evaluation = out / 'evaluation'
        evaluation.mkdir(exist_ok=False)
        dense, hashes = [], {}
        for robot in cfg['robots']:
            raw = native_rows(trials[robot], robot)
            path = Path(frozen['request']['artifacts']['keyframes.ellipselio.' + robot]) / 'store/keyframes.jsonl'
            keys = read_jsonl(path)
            opt = sorted((row for row in graph if row['robot_id'] == robot), key=lambda row: row['keyframe_id'])
            if [(r['keyframe_id'], r['stamp_ns']) for r in keys] != [(r['keyframe_id'], r['stamp_ns']) for r in opt]:
                raise ValueError('Frozen anchor IDs or timestamps differ')
            track = [dict(row, component=components[robot]) for row in corrected_dense_rows(raw, keys, opt)]
            dense.extend(track)
            save_tum(evaluation / (robot + '-corrected.tum'), track)
            for path in [path, Path(trials[robot]) / 'frontend/native_updates.jsonl']:
                hashes[str(path)] = file_hash(path)
        gt, gt_status = evaluation_ground_truth(cfg['robots'], Path(cfg['dataset']), dense, cfg['evaluation'])
        for info in gt_status.values():
            if Path(info['path']).exists():
                hashes[info['path']] = file_hash(Path(info['path']))
        metric = trajectory_metrics(dict(poses=dense), gt, cfg['evaluation'], evaluation / 'evo')
        native = read_json(out / 'dpgo/summary.json')
        pairs = [p for robot in native['robots'].values() for p in robot['registration']['pairs'] if p['accepted']]
        gpu = read_json(out / 'gpu-samples.json')
        gpu_mib = [float(row['global_device_sample'].split(',')[0]) for row in gpu['samples'] if row['global_device_sample']]
        record = dict(entry, components=components, trajectory=metric, ground_truth=gt_status,
            config_sha256=file_hash(out / 'config.yaml'), evaluation_input_hashes=hashes,
            accepted_factors=len(pairs), final_quality_passed=all(p['final_quality_reason'] == 'accepted' for p in pairs),
            final_overlap_min=min(p['final_quality']['overlap'] for p in pairs),
            final_rmse_median=float(np.median([p['final_quality']['rmse_m'] for p in pairs])),
            gpu_memory_scope=gpu['scope'], gpu_whole_device_peak_mib=max(gpu_mib),
            cbs_network_cdr_bytes=native['cbs_network_cdr_bytes'],
            registration_network_cdr_bytes=native['registration_network_cdr_bytes'])
        write_json(evaluation / 'summary.json', record)
        result.append(record)
        print(entry['group'], entry['variant'], 'wall_s', entry['wall_s'], 'metrics', metric, flush=True)
    write_json(args.input / 'evaluated-results.json', dict(entries=result,
        protocol='Frozen final graph, CPU/GPU/GPU/CPU; 100 iterations at 10 Hz. evo 1.36.5, 50 ms, shared rigid alignment without scale.'))


if __name__ == '__main__':
    main()
