"""Freeze compact loop-quality evidence from the reported, unmodified runs.

Run with the research environment. No ROS replay, model inference or SLAM.
The companion analyze.py reads inputs.json without needing deleted run caches.
"""
from pathlib import Path
import csv
import json
import sys

import numpy as np
from evo.tools.file_interface import read_tum_trajectory_file

RESEARCH = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(RESEARCH))
from s3e_pipeline.artifacts import file_hash, read_json, read_jsonl, write_json
from s3e_pipeline.evaluation import ground_truth, gt_position

OUTPUT = Path(__file__).resolve().parent
SPECS = [('Square 1', 'cbs-square1'), ('Square 2', 'cbs-square2'),
         ('Library 1', 'cbs-library1'), ('Campus Road 1', 'cbs-campus-road1'),
         ('Playground 2', 'cbs-playground2'), ('Laboratory 1', 'cbs-laboratory1'),
         ('Playground 1, excluded', 'cbs-playground1')]


def collect():
    result = {}
    sources = {}

    def record(path):
        path = Path(path).resolve()
        sources[str(path)] = file_hash(path)
        return path

    for name, folder in SPECS:
        directory = RESEARCH/'figures'/folder
        meta = read_json(record(directory/'figures.json'))
        experiment = meta.get('experiment', {})
        registry_path = Path(meta['input_run'])
        registry = read_json(record(registry_path)) if registry_path.exists() else None
        config = registry['config'] if registry else experiment['config']
        audit = experiment.get('validation', {})
        item = dict(dataset=config['dataset'], registration_config=config['backend']['registration'],
                    total_loops=audit.get('unique_loops'),
                    historical_proximity=dict(available=audit.get('accepted_gt_available'),
                                              within_10m=audit.get('accepted_within_10m')),
                    loops=None, keyframes={})
        constraints = directory/'constraints.jsonl'
        if not constraints.exists() and registry:
            constraints = Path(registry['artifacts']['dpgo'])/'constraints.jsonl'
        if not constraints.exists():
            item['status'] = 'per-loop transforms removed during prior cleanup; aggregate proximity only'
            result[name] = item
            continue
        loops = read_jsonl(record(constraints))
        item['total_loops'] = len(loops)
        distances = {}
        proximity = directory/'verification_proximity.csv'
        if proximity.exists():
            with record(proximity).open() as stream:
                for row in csv.DictReader(stream):
                    if row['accepted'] != 'True':
                        continue
                    key = tuple(sorted(((row['query_robot'], int(row['query_keyframe'])),
                                        (row['candidate_robot'], int(row['candidate_keyframe'])))))
                    assert key not in distances
                    distances[key] = float(row['gt_distance_m']) if row['gt_distance_m'] else None
        elif registry:
            keys = {}
            truth = {}
            for robot in config['robots']:
                path = Path(registry['artifacts'][f'keyframes.livo.{robot}'])/'store/keyframes.jsonl'
                for row in read_jsonl(record(path)):
                    key = (robot, row['keyframe_id'])
                    keys[key] = row
                    item['keyframes'][f'{robot}:{key[1]}'] = dict(
                        stamp_ns=row['stamp_ns'], pose=np.asarray(row['T_world_body']).reshape(4, 4).tolist())
                truth[robot] = ground_truth(record(Path(config['dataset'])/f'{robot.lower()}_gt.txt'))
            for edge in loops:
                i, j = tuple(edge['i']), tuple(edge['j'])
                a = gt_position(truth[i[0]], keys[i]['stamp_ns'], 2_000_000_000)
                b = gt_position(truth[j[0]], keys[j]['stamp_ns'], 2_000_000_000)
                distances[tuple(sorted((i, j)))] = float(np.linalg.norm(a-b)) if a is not None and b is not None else None
        # After cleanup, only query timestamps embedded in accepted constraints
        # are available. Do not invent times for candidate-only endpoints.
        if not item['keyframes']:
            stamps = {}
            for edge in loops:
                d = edge['diagnostics']; key = tuple(d['query']); stamp = d['query_stamp_ns']
                assert key not in stamps or stamps[key] == stamp
                stamps[key] = stamp
            for robot in config['robots']:
                track = read_tum_trajectory_file(record(directory/'raw-odometry'/robot/'raw.tum'))
                for key, stamp in stamps.items():
                    if key[0] != robot:
                        continue
                    index = int(np.argmin(abs(track.timestamps-stamp/1e9)))
                    assert abs(track.timestamps[index]-stamp/1e9) < 1e-5
                    item['keyframes'][f'{robot}:{key[1]}'] = dict(
                        stamp_ns=stamp, pose=track.poses_se3[index].tolist())
        item['loops'] = []
        for edge in loops:
            key = tuple(sorted((tuple(edge['i']), tuple(edge['j']))))
            assert key in distances and edge['accepted']
            d = edge['diagnostics']
            item['loops'].append(dict(i=edge['i'], j=edge['j'], T_i_j=edge['T_i_j'],
                gt_distance_m=distances[key], branch='+'.join(sorted(d['retrieval_sources'])),
                rmse_m=d['rmse_m'], overlap=d['overlap'], converged=d['converged'],
                inliers=d['inliers'], condition=d['condition'],
                min_observability=min(d['observability_eigenvalues']),
                native_inliers=d.get('native', {}).get('hypothesis', {}).get('inliers')))
        item['status'] = 'per-loop transforms available'
        result[name] = item
    assert all(file_hash(path) == value for path, value in sources.items())
    write_json(OUTPUT/'inputs.json', dict(schema_version=1, sources=sources, experiments=result,
        ground_truth_policy='existing proximity audit: linear interpolation only inside GT brackets <=2 seconds; no orientation GT',
        raw_pose_association='evo TUM parser; accepted-query timestamps match saved raw poses within 10 microseconds',
        collector_sha256=file_hash(__file__)))
    print('Saved inputs for', len(result), 'experiments; original source hashes unchanged.')


if __name__ == '__main__':
    collect()
