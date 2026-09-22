"""Read-only replay of the two strongest rejected ground/aerial hypotheses."""
from pathlib import Path
import hashlib,json,zlib
import numpy as np
import yaml
from scipy.spatial.transform import Rotation,Slerp
from s3e_pipeline.backends import unpack_array
from s3e_pipeline.mapclosures_inspection import density_pixels
import s3e_mapclosures_inspection as native
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import ConnectionPatch

B=Path(__file__).resolve().parent;W=B/'source/output'
O=B.parents[1]/'FAST-LIVO2-ROS2/research/figures/graco-ground-aerial-local-20260921/viewpoint-diagnostic'
O.mkdir(exist_ok=False)
cfg=yaml.safe_load((W/'config.yaml').read_text())['backend']['mapclosures']
gallery=json.loads((W/'bev-gallery/manifest.json').read_text())['maps']
candidates=json.loads((W/'diagnostics/cross-robot-candidates.json').read_text())
def features(robot,key):
    path=W/f'prepared-{robot}/ellipsoid/{key:06d}.json.zlib'
    d=json.loads(zlib.decompress(path.read_bytes()))
    return {k:unpack_array(v) for k,v in d['mapclosures'].items()}
engine=native.Inspector(cfg['density_map_resolution'],cfg['density_threshold'],cfg['hamming_distance_threshold'])
keys={r:[json.loads(l) for l in (W/f'prepared-{r}/store/keyframes.jsonl').read_text().splitlines()] for r in ('ground06','aerial06')}
for k in keys['ground06']:engine.add(k['keyframe_id'],features('ground06',k['keyframe_id']))
results=[]
for q,c in [(27,17),(17,7)]:
    expected=next(e['mapclosures_hypothesis'] for e in candidates if e['query']==['aerial06',q] and e['candidate']==['ground06',c])
    qf=features('aerial06',q);cf=features('ground06',c)
    debug=engine.correspondences(qf,c)
    assert len(debug['query_xy'])==expected['matches']
    assert len(debug['ransac_inlier_indices'])==expected['inliers']
    assert np.allclose(debug['T_i_j'],expected['T_i_j'],atol=1e-8)
    T=qf['ground']@debug['T_i_j']@np.linalg.inv(cf['ground'])
    est=np.array(debug['candidate_xy'])@T[:2,:2].T+T[:2,3]/cfg['density_map_resolution']
    residuals=cfg['density_map_resolution']*np.linalg.norm(est-debug['query_xy'],axis=1)
    np.savez_compressed(O/f'aerial06-{q}-ground06-{c}-native.npz',**debug,xy_residuals_m=residuals)
    results.append(dict(query=['aerial06',q],candidate=['ground06',c],exact_native_replay=True,
        query_features=len(qf['xy']),candidate_features=len(cf['xy']),matches=len(debug['query_xy']),
        inliers=len(debug['ransac_inlier_indices']),inlier_indices=list(debug['ransac_inlier_indices']),
        xy_residuals_m=residuals.tolist(),hamming=np.array(debug['hamming']).tolist()))
    fig,axes=plt.subplots(1,2,figsize=(13,7),layout='constrained')
    plots=[]
    for ax,r,k,coords in [(axes[0],'ground06',c,debug['candidate_xy']),(axes[1],'aerial06',q,debug['query_xy'])]:
        m=next(m for m in gallery if m['robot']==r and m['key']==k)
        im=plt.imread(W/'bev-gallery'/m['image'])
        ax.imshow(im,cmap='gray',vmin=0,vmax=1,interpolation='nearest')
        ax.set_title(f'{r} / {k} · {m["features"]} retained ORB features');ax.axis('off')
        pixels=density_pixels(coords,m['lower_bound']);plots.append(pixels)
        for i,point in enumerate(pixels):
            color='#63ed88' if i in debug['ransac_inlier_indices'] else '#ff8862'
            ax.scatter(*point,s=35,facecolors='none',edgecolors=color,linewidths=1.3)
            ax.annotate(str(i+1),point+np.array([3,0]),color=color,fontsize=9)
    for i,(left,right) in enumerate(zip(*plots)):
        color='#63ed88' if i in debug['ransac_inlier_indices'] else '#ff8862'
        fig.add_artist(ConnectionPatch(left,right,'data','data',axesA=axes[0],axesB=axes[1],color=color,alpha=.75,lw=.9))
    fig.suptitle(f'Exact replay: {len(debug["query_xy"])} descriptor matches → {len(debug["ransac_inlier_indices"])} RANSAC inliers\nGreen: retained · orange: rejected · loop gate requires more than 5 inliers',fontsize=15)
    fig.savefig(O/f'aerial06-{q}-ground06-{c}-matches.png',dpi=160);plt.close(fig)
# Ground truth is consulted only after replay, to evaluate the already-frozen
# planar hypotheses. It is not used to match features, fit RANSAC, or build maps.
reference={r:np.loadtxt(W/f'reference/{r}_gt.txt') for r in keys}
def truth(robot,key):
    ref=reference[robot];t=keys[robot][key]['stamp_ns']/1e9
    j=int(np.searchsorted(ref[:,0],t));assert 0<j<len(ref)
    span=ref[j-1:j+1];relative=span[:,0]-span[0,0];dt=t-span[0,0]
    T=np.eye(4);T[:3,:3]=Slerp(relative,Rotation.from_quat(span[:,4:8]))([dt]).as_matrix()[0]
    T[:3,3]=span[0,1:4]+(span[1,1:4]-span[0,1:4])*dt/relative[1]
    return T
for result in results:
    q=result['query'][1];c=result['candidate'][1]
    with np.load(O/f'aerial06-{q}-ground06-{c}-native.npz') as d:T=d['T_i_j']
    qf=features('aerial06',q);cf=features('ground06',c)
    level=qf['ground']@T@np.linalg.inv(cf['ground'])
    expected=qf['ground']@np.linalg.inv(truth('aerial06',q))@truth('ground06',c)@np.linalg.inv(cf['ground'])
    yaw=lambda m:float(np.arctan2(m[1,0],m[0,0]))
    dy=yaw(level)-yaw(expected)
    result['evaluation_only']=dict(ground_truth_used=True,
        planar_translation_error_m=float(np.linalg.norm(level[:2,3]-expected[:2,3])),
        yaw_error_deg=float(abs(np.degrees(np.arctan2(np.sin(dy),np.cos(dy))))),
        reference_vertical_translation_m=float(expected[2,3]),
        limitation='GT evaluates the frozen 2D hypothesis only; no 3D verification or pipeline rerun.')
summary=dict(native_commit=native.upstream_commit,ground_truth_used_for_replay=False,
    matches_replayed_without_changing_thresholds=True,results=results,
    script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
(O/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
(O/'README.md').write_text('The original loop-retrieval result remains unchanged. These figures replay the saved descriptor matches and native RANSAC exactly. Green links are RANSAC inliers, not verified 3D loop correspondences. Ground truth is consulted only afterward to evaluate horizontal translation and yaw.\n\n[Numeric diagnostics](summary.json)\n\n![Displayed candidate](aerial06-27-ground06-17-matches.png)\n\n![Second candidate](aerial06-17-ground06-7-matches.png)\n')
print(json.dumps(summary,indent=2))
