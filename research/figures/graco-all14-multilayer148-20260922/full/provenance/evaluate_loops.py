"""Post-run loop-transform errors only; never changes graph acceptance."""
from pathlib import Path
import sys,json,hashlib
import numpy as np
from scipy.spatial.transform import Rotation
B=Path(__file__).resolve().parent;W=B/'full'
sys.path.insert(0,'/workspace/FAST-LIVO2-ROS2/scripts/recent_submaps')
from evaluate_joint_bev_matches import track
read=lambda p:json.loads(p.read_text())
rows=lambda p:[json.loads(s) for s in p.read_text().splitlines()]
assert read(W/'status.json')['phase']=='complete'
graph_files=[W/'dpgo'/name for name in ['poses.jsonl','constraints.jsonl','proposed-constraints.jsonl']]
graph_hashes={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in graph_files}
robots=list(read(W/'trials.json'));tracks={r:track(W/f'reference/{r}_gt.txt') for r in robots}
keys={(r,p['keyframe_id']):p for r in robots for p in rows(W/f'prepared-{r}/store/keyframes.jsonl')}
retained={(tuple(e['i']),tuple(e['j'])) for e in rows(W/'dpgo/constraints.jsonl')}
records=[];skipped=[]
for edge in rows(W/'dpgo/proposed-constraints.jsonl'):
 i,j=tuple(edge['i']),tuple(edge['j'])
 try:a=tracks[i[0]](keys[i]['stamp_ns']);b=tracks[j[0]](keys[j]['stamp_ns'])
 except ValueError as e:skipped.append(dict(i=i,j=j,reason=str(e)));continue
 truth=np.linalg.inv(a)@b;estimate=np.asarray(edge['T_i_j'])
 assert estimate.shape==(4,4)
 records.append(dict(i=i,j=j,pcm_retained=(i,j) in retained,
  category='aerial-ground' if i[0].startswith('aerial')!=j[0].startswith('aerial') else 'aerial-aerial' if i[0].startswith('aerial') else 'ground-ground',
  translation_error_m=float(np.linalg.norm(estimate[:3,3]-truth[:3,3])),
  rotation_error_deg=float(np.degrees(Rotation.from_matrix(truth[:3,:3].T@estimate[:3,:3]).magnitude()))))
summary={}
for label in ['proposed','retained']:
 vals=[r for r in records if label=='proposed' or r['pcm_retained']]
 summary[label]=dict(count=len(vals))
 for metric in ['translation_error_m','rotation_error_deg']:
  v=np.array([r[metric] for r in vals]);summary[label][metric]=dict(median=float(np.median(v)),p95=float(np.quantile(v,.95)),maximum=float(v.max())) if len(v) else None
assert graph_hashes=={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in graph_files}
out=dict(evaluation_only=True,graph_unchanged=True,graph_sha256=graph_hashes,ground_truth_for_estimation=False,interpolation_max_bracket_s=.05,summary=summary,
 references={r:hashlib.sha256((W/f'reference/{r}_gt.txt').read_bytes()).hexdigest() for r in robots},records=records,skipped=skipped)
(W/'diagnostics/loop-errors.json').write_text(json.dumps(out,indent=2));print(json.dumps(summary,indent=2))
