"""Independent snapshot/frame and geometric residual checks. Evaluation only."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree

p=argparse.ArgumentParser()
p.add_argument('--runtime',type=Path,required=True)
p.add_argument('--capture',type=Path,required=True)
p.add_argument('--output',type=Path,required=True)
a=p.parse_args()
hashes={}
def sha(path):
    digest=hashlib.sha256(path.read_bytes()).hexdigest()
    hashes[str(path)]=digest
    return digest
def rows(path):
    sha(path)
    return [json.loads(l) for l in path.read_text().splitlines()]
def load(path):
    sha(path)
    with np.load(path) as f:return {k:f[k] for k in f.files}
def tf(T,x):return x@T[:3,:3].T+T[:3,3]
def inv(T):
    o=np.eye(4);o[:3,:3]=T[:3,:3].T;o[:3,3]=-o[:3,:3]@T[:3,3];return o
def down(x):
    _,i=np.unique(np.floor(x/.4).astype(np.int64),axis=0,return_index=True)
    return x[np.sort(i)]
def quality(target,source,T):
    moved=tf(T,source)
    f=cKDTree(target).query(moved,workers=-1)[0]
    b=cKDTree(moved).query(target,workers=-1)[0]
    return dict(forward_inliers=int((f<.6).sum()),reverse_inliers=int((b<.6).sum()),
                overlap=float(min((f<.6).mean(),(b<.6).mean())),
                forward_inlier_rmse_m=float(np.sqrt(np.mean(f[f<.6]**2))),
                truncated_symmetric_rms_1_5m=float(np.sqrt((np.mean(np.minimum(f,1.5)**2)+np.mean(np.minimum(b,1.5)**2))/2)))

sha(a.runtime/'report/report.json')
report=json.loads((a.runtime/'report/report.json').read_text())
keys={r:{x['keyframe_id']:x for x in rows(a.runtime/f'prepared-{r}/store/keyframes.jsonl')}
      for r in ['aerial05','aerial06','aerial07','aerial08']}
snapshots=[]
for r,k in [('aerial05',16),('aerial06',1),('aerial07',20),('aerial08',16)]:
    path=a.capture/f'frontends/{r}/frontend/area_maps'
    first=load(path/f'submap-{k:06d}.npz');second=load(path/f'submap-{k+1:06d}.npz')
    print(r,'payload fields',list(first),flush=True)
    # Native writer stores points, point_ids and point_scan_ids in the anchor IMU frame.
    _,i,j=np.intersect1d(first['point_ids'],second['point_ids'],return_indices=True)
    T1=np.array(keys[r][k]['T_world_body']);T2=np.array(keys[r][k+1]['T_world_body'])
    distances=np.linalg.norm(tf(T1,first['points'][i])-tf(T2,second['points'][j]),axis=1)
    evidence=load(a.runtime/f'prepared-{r}/store/{k:06d}.npz')
    nearest=cKDTree(first['points']).query(evidence['cloud'],workers=-1)[0]
    snapshots.append(dict(robot=r,keys=[k,k+1],shared_point_ids=len(i),
        shared_world_point_rms_m=float(np.sqrt(np.mean(distances**2))),shared_world_point_max_m=float(distances.max()),
        descriptor_evidence_points=len(evidence['cloud']),evidence_to_native_max_m=float(nearest.max()),
        evidence_cloud_equals_scan=bool(np.array_equal(evidence['cloud'],evidence['scan']))))
    assert distances.max()<1e-3 and nearest.max()<1e-3
    assert np.array_equal(first['point_scan_ids'][i],second['point_scan_ids'][j])

edges=rows(a.runtime/'dpgo/constraints.jsonl')
comparisons=[];seen=set()
for e in edges:
    i,j=e['i'],e['j'];pair=(i[0],j[0])
    if pair in seen:continue
    seen.add(pair)
    target=down(load(a.runtime/f'prepared-{i[0]}/store/{i[1]:06d}.npz')['cloud'])
    source=down(load(a.runtime/f'prepared-{j[0]}/store/{j[1]:06d}.npz')['cloud'])
    Gi=np.array(report['raw'][i[0]]['alignment_SE3'])@np.array(keys[i[0]][i[1]]['T_world_body'])
    Gj=np.array(report['raw'][j[0]]['alignment_SE3'])@np.array(keys[j[0]][j[1]]['T_world_body'])
    comparisons.append(dict(i=i,j=j,loop_measurement=quality(target,source,np.array(e['T_i_j'])),
        position_aligned_odometry_reference=quality(target,source,inv(Gi)@Gj)))
    print(comparisons[-1],flush=True)

assert all(hashlib.sha256(Path(p).read_bytes()).hexdigest()==h for p,h in hashes.items())
a.output.write_text(json.dumps(dict(passed=True,evaluation_only=True,ground_truth_fed_to_estimator=False,
    runtime_outputs_modified=False,snapshot_checks=snapshots,geometry_comparisons=comparisons,input_sha256=hashes,
    caveat='Four representative pairs; static nearest-neighbor quality evaluation at two fixed poses, no optimizer or GT-derived correction.'),indent=2)+'\n')
