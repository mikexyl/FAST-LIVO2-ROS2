"""Diagnostic only: unchanged production verifier on A04's one eligible skipped proposal."""
from pathlib import Path
import hashlib,json,zlib,time
import numpy as np
import yaml
from s3e_pipeline.backends import create
from s3e_pipeline.registration import bounded_cloud
B=Path('/workspace/.ros2/graco-aerial-five148-20260921');W=B/'full'
cfg=yaml.safe_load((W/'config.yaml').read_text())
def rows(p):return [json.loads(l) for l in p.read_text().splitlines()]
events=[e for p in (W/'dpgo').glob('*/events.jsonl') for e in rows(p)]
candidates=[]
for e in events:
    if e['type']!='ranked' or e['query'][0]==e['candidate_robot']:continue
    if 'aerial04' not in (e['query'][0],e['candidate_robot']):continue
    for c in e['candidates']:
        candidates.append(dict(query=e['query'],candidate=[e['candidate_robot'],c['keyframe_id']],**c))
selected=[c for c in candidates if c['eligible']]
assert len(selected)==1
c=selected[0]
rejections=[e for e in events if e['type']=='rejection' and e.get('candidate')==c['candidate'] and e.get('query')==c['query']]
assert len(rejections)==1 and rejections[0]['reason']=='verification_budget'
hashes={}
def packet(endpoint):
    r,k=endpoint;p=W/f'prepared-{r}/store/{k:06d}.npz'
    hashes[str(p)]=hashlib.sha256(p.read_bytes()).hexdigest()
    with np.load(p) as f:cloud=f['cloud'].copy()
    n=len(cloud);cloud,res=bounded_cloud(cloud,.4,max_points=None,max_range=None)
    d=W/f'prepared-{r}/ellipsoid/{k:06d}.json.zlib'
    hashes[str(d)]=hashlib.sha256(d.read_bytes()).hexdigest()
    descriptor=json.loads(zlib.decompress(d.read_bytes()))
    return dict(cloud=cloud,descriptor=descriptor,evidence_preprocessing=dict(sampling_policy='fixed',
        effective_voxel_m=res,requested_voxel_m=.4,input_points=n,output_points=len(cloud),max_range_m=None))
frozen={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in [W/'dpgo/constraints.jsonl',W/'dpgo/poses.jsonl']}
backend=create(cfg['backend']);start=time.monotonic()
try:result=backend.verify(packet(c['query']),packet(c['candidate']),dict(mapclosures_hypothesis=c['mapclosures_hypothesis'],sources=['mapclosures']))
finally:backend.close()
assert all(hashlib.sha256(Path(p).read_bytes()).hexdigest()==h for p,h in {**hashes,**frozen}.items())
from collections import Counter
decisions=[e for e in events if e['type']=='rejection' and e['query'][0]!=e['candidate'][0] and 'aerial04' in (e['query'][0],e['candidate'][0])]
summary=dict(inter_robot_candidates=len(candidates),eligible_candidates=len(selected),
    final_rejection_reasons=dict(Counter(e['reason'] for e in decisions)),
    note='Final rejection events are authoritative; ranked candidates retain a stale default rejection_reason even when eligible.',
    diagnostic=dict(query=c['query'],candidate=c['candidate'],bev_inliers=c['mapclosures_hypothesis']['inliers'],
        runtime_decision='verification_budget',result=result,wall_s=time.monotonic()-start),
    ground_truth_used=False,registration_settings_unchanged=True,production_graph_modified=False,
    input_sha256=hashes,graph_sha256=frozen)
(W/'aerial04-retrieval-diagnostic.json').write_text(json.dumps(summary,indent=2)+'\n')
print(json.dumps(dict(query=c['query'],candidate=c['candidate'],accepted=result['accepted'],reason=result['reason'],
    overlap=result.get('overlap'),rmse_m=result.get('rmse_m'),initial_overlap=result.get('initial_overlap'))))
