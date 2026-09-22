"""Offline height initialization diagnostic; no GT, graph edits, or relaxed gates."""
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import hashlib,json,time
import numpy as np
import yaml
from scipy.spatial import cKDTree
from s3e_pipeline.geometry import transform
from s3e_pipeline.registration import bounded_cloud,refine

B=Path('/workspace/.ros2/upstream-area-graco-aerial148-20260921')
W=B/'full';OUT=B/'a5-height-diagnostic/exact-evidence'
OUT.mkdir(exist_ok=False)
cfg=yaml.safe_load((W/'config.yaml').read_text())['backend']['registration']
gallery=json.loads((W/'bev-gallery/manifest.json').read_text())
maps={(m['robot'],m['key']):m for m in gallery['maps']}
events=[json.loads(l) for p in sorted((W/'dpgo').glob('*/events.jsonl')) for l in p.read_text().splitlines()]
attempts=[e for e in events if e['type']=='verification' and 'aerial05' in (e['query'][0],e['candidate'][0])]
assert len(attempts)==12
controls=[e for e in events if e['type']=='verification' and e['accepted']][:2]

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def cloud(endpoint):
    r,k=endpoint;p=W/f'prepared-{r}/store/{k:06d}.npz'
    with np.load(p) as data:points=data['cloud'].copy()
    points,_=bounded_cloud(points,cfg['voxel_m'],max_points=None,max_range=cfg.get('max_range_m'))
    return points,sha(p)

def run(e):
    started=time.monotonic();target,th=cloud(e['query']);source,sh=cloud(e['candidate'])
    Gq=np.array(maps[tuple(e['query'])]['ground']);Gc=np.array(maps[tuple(e['candidate'])]['ground'])
    initial=np.array(e['initial_T_i_j']);level=Gq@initial@np.linalg.inv(Gc)
    assert len(target)==e['query_evidence']['output_points'] and len(source)==e['candidate_evidence']['output_points']
    baseline=refine(target,source,initial,cfg)
    assert baseline['reason']==e['reason'] and baseline['accepted']==e['accepted'],(e['query'],e['candidate'],baseline['reason'],e['reason'])
    for k in ('initial_overlap','overlap','rmse_m'):
        if k in baseline:assert np.isclose(baseline[k],e[k],rtol=1e-4,atol=1e-6),(k,baseline[k],e[k])
    assert abs(level[2,3])<1e-8 and np.linalg.norm(level[2,:3]-[0,0,1])<1e-8
    # Coarser geometry is used ONLY for estimating the unobserved vertical offset.
    # Actual acceptance uses the unchanged full fixed-resolution verifier below.
    t,_=bounded_cloud(transform(Gq,target),.8,max_points=None,max_range=None)
    s,_=bounded_cloud(transform(Gc,source),.8,max_points=None,max_range=None)
    s=transform(level,s)
    dist,idx=cKDTree(t[:,:2]).query(s[:,:2],k=8,workers=1)
    valid=dist<1.0;offset=(t[idx,2]-s[:,2,None])[valid]
    bins=np.rint(offset/.5).astype(np.int64);centers,counts=np.unique(bins,return_counts=True)
    order=np.argsort(-counts,kind='stable');peaks=[]
    for j in order:
        z=float(centers[j]*.5)
        if all(abs(z-x)>2.0 for x in peaks):peaks.append(z)
        if len(peaks)==5:break
    # Compare modal hypotheses using a symmetric, coarse 3D overlap score.
    # Include zero as an explicit control and deterministic tie preference.
    candidates=sorted(set([0.,*peaks]));tree_t=cKDTree(t);tree_s=cKDTree(s)
    scored=[]
    for z in candidates:
        shift=np.array([0,0,z])
        forward=float((tree_t.query(s+shift,workers=1)[0]<1.5).mean())
        reverse=float((tree_s.query(t-shift,workers=1)[0]<1.5).mean())
        scored.append(dict(dz_m=z,coarse_symmetric_overlap=min(forward,reverse)))
    best=max(scored,key=lambda x:(x['coarse_symmetric_overlap'],-abs(x['dz_m'])))
    adjusted=initial.copy();adjusted[:3,3]+=Gq[:3,:3].T@np.array([0,0,best['dz_m']])
    result=refine(target,source,adjusted,cfg)
    if 'T_i_j' in result:
        optimized=Gq@np.array(result['T_i_j'])@np.linalg.inv(Gc)
        result['final_level_dz_m']=float(optimized[2,3])
    item=dict(query=e['query'],candidate=e['candidate'],control='aerial05' not in (e['query'][0],e['candidate'][0]),
        original={k:e.get(k) for k in ('accepted','reason','initial_overlap','overlap','rmse_m','T_i_j')},
        reproduced_baseline=baseline,original_level_dz_m=float(level[2,3]),height_hypotheses=scored,selected_dz_m=best['dz_m'],
        estimated_initializer=adjusted.tolist(),result=result,
        target_sha256=th,source_sha256=sh,source_points=len(source),target_points=len(target),
        height_vote_pairs=int(valid.sum()),runtime_s=time.monotonic()-started)
    name=f"{e['query'][0]}-{e['query'][1]:03d}--{e['candidate'][0]}-{e['candidate'][1]:03d}"
    (OUT/f'{name}.json').write_text(json.dumps(item,indent=2)+'\n')
    print(json.dumps({k:item[k] for k in ('query','candidate','selected_dz_m')} | dict(
        accepted=result['accepted'],reason=result['reason'],overlap=result.get('overlap'),rmse_m=result.get('rmse_m'))),flush=True)
    return item

started=time.monotonic()
with ThreadPoolExecutor(max_workers=4) as pool:results=list(pool.map(run,attempts+controls))
summary=dict(ground_truth_used=False,production_graph_modified=False,registration_gates_unchanged=True,
    frozen_config_sha256=sha(W/'config.yaml'),script_sha256=sha(Path(__file__)),
    method='At fixed BEV XY/yaw, vote for vertical translation using XY-neighbor height differences; choose among five separated modal peaks and zero by symmetric coarse 3D overlap; run unchanged full-resolution GICP verification',
    height_estimation=dict(voxel_m=.8,xy_neighbors=8,xy_radius_m=1.,histogram_bin_m=.5,peak_separation_m=2.,modal_peaks=5,coarse_inlier_m=1.5),
    original_verification_reproduced_with_exact_evidence=True,original_a5_accepted=0,new_a5_accepted=sum(x['result']['accepted'] for x in results if not x['control']),
    a5_attempts=len(attempts),positive_controls=len(controls),results=results,runtime_s=time.monotonic()-started)
(OUT/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
print(json.dumps({k:v for k,v in summary.items() if k!='results'}),flush=True)
