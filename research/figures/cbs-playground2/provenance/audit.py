import argparse
from collections import Counter
import csv
import sqlite3
from pathlib import Path
import numpy as np
import yaml
from s3e_pipeline.artifacts import read_json, read_jsonl, write_json
from s3e_pipeline.evaluation import ground_truth, gt_position
from evo.main_ape import ape
from evo.core.metrics import PoseRelation
from evo.tools.file_interface import read_kitti_poses_file, read_tum_trajectory_file, load_res_file

p = argparse.ArgumentParser()
p.add_argument('registry', type=Path)
args = p.parse_args()
registry = read_json(args.registry); cfg = registry['config']; artifacts = registry['artifacts']
root = args.registry.parent; output = root/'audit'; output.mkdir(exist_ok=True)
dpgo = Path(artifacts['dpgo']); evaluation = Path(artifacts['dpgo_evaluate'])
report = read_json(evaluation/'report.json')
keys = {tuple((r['robot_id'], r['keyframe_id'])): r for robot in cfg['robots']
        for r in read_jsonl(Path(artifacts[f'keyframes.livo.{robot}'])/'store/keyframes.jsonl')}
gt = {r: ground_truth(Path(cfg['dataset'])/f'{r.lower()}_gt.txt') for r in cfg['robots']}
positions = {k: gt_position(gt[k[0]], r['stamp_ns'], int(cfg['evaluation']['gt_max_gap_s']*1e9))
             for k,r in keys.items()}
events = [e for r in cfg['robots'] for e in read_jsonl(dpgo/r/'events.jsonl')]
ranked = [e for e in events if e['type'] == 'ranked']
verifications = [e for e in events if e['type'] == 'verification']
violations = []; candidate_count = 0
for event in ranked:
    query = tuple(event['query']); q = keys[query]
    for candidate in event['candidates']:
        candidate_count += 1
        key = (event['candidate_robot'], candidate['keyframe_id']); c = keys[key]
        dt = q['stamp_ns']-c['stamp_ns']
        if dt < 0 or (key[0] == query[0] and dt < cfg['loops']['same_robot_exclusion_s']*1e9):
            violations.append(dict(query=query, candidate=key, dt_ns=dt))
assert not violations, violations
loops = read_jsonl(dpgo/'constraints.jsonl')
canonical = [tuple(sorted((tuple(e['i']),tuple(e['j'])))) for e in loops]
assert len(canonical) == len(set(canonical))
wire = [w for robot in cfg['robots'] for w in read_jsonl(dpgo/robot/'wire.jsonl')]
wire_bytes = sum(w['network_bytes'] for w in wire)
assert wire_bytes == report['runtime']['loop_network_cdr_bytes']
for w in wire:
    recipients = len(cfg['robots'])-1 if w['dst'] == 'peers' else 1
    assert w['network_bytes'] == w['serialized_bytes']*recipients
diagnostics = []
for e in verifications:
    query,candidate = tuple(e['query']),tuple(e['candidate'])
    a,b = positions[query],positions[candidate]
    distance = float(np.linalg.norm(a-b)) if a is not None and b is not None else None
    h = (e.get('native') or {}).get('hypothesis', {})
    diagnostics.append(dict(query_robot=query[0],query_keyframe=query[1],candidate_robot=candidate[0],
        candidate_keyframe=candidate[1],accepted=e['accepted'],reason=e['reason'],
        retrieval_sources='+'.join(e.get('retrieval_sources',[])),
        gt_distance_m=distance,gt_within_10m=None if distance is None else distance<=cfg['evaluation']['proximity_m'],
        native_matches=h.get('matches'),native_inliers=h.get('inliers'),
        rmse_m=e.get('rmse_m'),overlap=e.get('overlap')))
with (output/'verification_proximity.csv').open('w',newline='') as f:
    writer=csv.DictWriter(f,fieldnames=['query_robot','query_keyframe','candidate_robot','candidate_keyframe','accepted','reason','retrieval_sources','gt_distance_m','gt_within_10m','native_matches','native_inliers','rmse_m','overlap']);writer.writeheader();writer.writerows(diagnostics)
available = [d for d in diagnostics if d['gt_distance_m'] is not None]
close = [d for d in available if d['gt_within_10m']]
meta=yaml.safe_load((Path(cfg['dataset'])/'metadata.yaml').read_text())['rosbag2_bagfile_information']
counts={v['topic_metadata']['name']:v['message_count'] for v in meta['topics_with_message_count']}
odom = {}
features = {}
for robot in cfg['robots']:
    stage=Path(artifacts[f'odometry.livo.{robot}']);summary=read_json(stage/'summary.json')
    expected_full=counts[f'/{robot}/velodyne_points']
    offset=float(cfg['odometry'].get('start_offsets_s',{}).get(robot,0.))
    bag_start=meta['starting_time']['nanoseconds_since_epoch']
    with sqlite3.connect(f"file:{next(Path(cfg['dataset']).glob('*.db3'))}?mode=ro",uri=True) as connection:
        expected=connection.execute('select count(*) from messages where topic_id=(select id from topics where name=?) and timestamp>=?',
            (f'/{robot}/velodyne_points',bag_start+round(offset*1e9))).fetchone()[0]
    assert summary['success'] and summary['finite'] and summary['input_clouds']==expected
    frames=read_jsonl(stage/'run/export/frames.jsonl')
    stamps=[x['stamp_ns'] for x in frames]
    span_s=(max(stamps)-min(stamps))/1e9 if stamps else 0.
    odom[robot]=dict(expected_clouds=expected,full_bag_clouds=expected_full,requested_start_offset_s=offset,summary=summary,
        frame_fraction=summary['exported_frames']/expected,
        first_export_stamp_ns=min(stamps), last_export_stamp_ns=max(stamps), export_span_s=span_s,
        temporal_coverage_pass=bool(span_s/((gt[robot][0][-1]-max(gt[robot][0][0],bag_start+round(offset*1e9)))/1e9) >= .95),
        elapsed_s=read_json(stage/'COMPLETE.json')['elapsed_s'])
    rows=read_jsonl(Path(artifacts[f'descriptors.livo.megaloc_mapclosures.{robot}'])/'timing.jsonl')
    n=np.array([r['features'] for r in rows])
    features[robot]=dict(keyframes=len(rows),nonempty=int(np.sum(n>0)),median=float(np.median(n)),max=int(n.max()))
reproduced = {}
for component,m in report['trajectory'].items():
    base=evaluation/'evo/cbs'
    native=load_res_file(base/f'{component}-component-ape.zip')
    direct=ape(read_kitti_poses_file(base/f'{component}-reference.kitti'),
               read_kitti_poses_file(base/f'{component}-estimate.kitti'),
               PoseRelation.translation_part,align=True,correct_scale=False)
    assert direct.stats==native.stats
    assert np.allclose(direct.np_arrays['error_array'],native.np_arrays['error_array'])
    reproduced[component]=native.stats
    for robot, metric in m['per_robot'].items():
        individual = load_res_file(base/f'{robot}-ape.zip')
        check = ape(read_tum_trajectory_file(base/f'{robot}-reference-matched.tum'),
                    read_tum_trajectory_file(base/f'{robot}-estimate-aligned.tum'),
                    PoseRelation.translation_part, align=False, correct_scale=False)
        assert np.allclose(check.np_arrays['error_array'], individual.np_arrays['error_array'], atol=1e-7)
        assert np.isclose(check.stats['rmse'], metric['statistics']['rmse'], atol=1e-7)
        reproduced[f'{component}/{robot}']=individual.stats
write_json(output/'audit.json',dict(dataset=cfg['dataset'],ranked_replies=len(ranked),
    ranked_candidates=candidate_count,causality_violations=violations,unique_loops=len(loops),
    loop_pair_counts=dict(Counter('-'.join(sorted((e['i'][0],e['j'][0]))) for e in loops)),
    wire_records=len(wire),wire_bytes=wire_bytes,wire_accounting_exact=True,
    odometry=odom,mapclosures_features=features,verification_count=len(diagnostics),
    verification_gt_available=len(available),verification_gt_unavailable=len(diagnostics)-len(available),
    within_10m_selected=len(close),within_10m_reasons=dict(Counter(d['reason'] for d in close)),
    accepted_gt_available=sum(d['accepted'] for d in available),
    accepted_within_10m=sum(d['accepted'] for d in close),
    evo_archive_reproduction=reproduced))
print('Audit passed:',output/'audit.json')
