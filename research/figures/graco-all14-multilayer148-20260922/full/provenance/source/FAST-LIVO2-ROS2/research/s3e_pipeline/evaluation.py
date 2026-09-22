"""Evaluation is the only stage granted access to S3E position ground truth."""
from collections import defaultdict
from pathlib import Path
from .frontends import frontend_name
import csv
import subprocess
import sys
import numpy as np
from scipy.spatial.transform import Rotation, Slerp

from .artifacts import read_json,read_jsonl,write_json,write_jsonl
from .geometry import pose,transform,inv,extrinsic,voxel_downsample
from .data import frames,LocalStore


def ground_truth(path):
    # Parse seconds as decimal text: do not discard precision through float epochs.
    from decimal import Decimal
    rows=[line.split() for line in Path(path).read_text().splitlines() if line.strip() and not line.startswith('#')]
    return np.array([int(Decimal(r[0])*10**9) for r in rows],dtype=np.int64),np.array([[float(v) for v in r[1:4]] for r in rows])


def gt_position(track,stamp,max_gap_ns):
    stamps,xyz=track;k=int(np.searchsorted(stamps,stamp))
    if k<len(stamps) and stamps[k]==stamp:return xyz[k]
    if k==0 or k==len(stamps) or stamps[k]-stamps[k-1]>max_gap_ns:return None
    f=(stamp-stamps[k-1])/(stamps[k]-stamps[k-1]);return xyz[k-1]*(1-f)+xyz[k]*f


def retrieval_metrics(rows,events,gt,cfg,exclusion_s):
    by_id={(r['robot_id'],r['keyframe_id']):r for r in rows}
    positions={k:gt_position(gt[k[0]],v['stamp_ns'],round(cfg['gt_max_gap_s']*1e9)) for k,v in by_id.items()}
    ranked=defaultdict(list);eligible_robots=defaultdict(set)
    for e in events:
        if e['type']!='ranked':continue
        q=tuple(e['query']);robot=e['candidate_robot'];eligible_robots[q].add(robot)
        ranked[q].extend((float(x['score']),(robot,x['keyframe_id'])) for x in e['candidates'])
    hits={k:0 for k in (1,5,20)};valid=0;tp=fp=fn=0;verification_tp=verification_fp=0
    for q,candidates in ranked.items():
        if positions[q] is None:continue
        stamp=by_id[q]['stamp_ns']
        relevant={k for k,r in by_id.items() if positions[k] is not None and r['robot_id'] in eligible_robots[q]
            and r['stamp_ns']<=stamp and (k[0]!=q[0] or stamp-r['stamp_ns']>=exclusion_s*1e9)
            and np.linalg.norm(positions[k]-positions[q])<=cfg['proximity_m']}
        selected=[k for _,k in sorted(candidates,key=lambda x:(-x[0],x[1])) if positions[k] is not None]
        selected=list(dict.fromkeys(selected))[:20]
        if relevant:
            valid+=1
            for k in hits:hits[k]+=bool(set(selected[:k])&relevant)
        tp+=len(set(selected)&relevant);fp+=len(set(selected)-relevant);fn+=len(relevant-set(selected))
    verifications=[e for e in events if e['type']=='verification']
    cached=[e for e in verifications if e.get('verification_cache_hit',False)]
    uncached=[e for e in verifications if not e.get('verification_cache_hit',False)]
    latencies=[e['simulated_detection_latency_s'] for e in verifications]
    for e in verifications:
        if not e['accepted']:continue
        a,b=positions[tuple(e['query'])],positions[tuple(e['candidate'])]
        if a is None or b is None:continue
        if np.linalg.norm(a-b)<=cfg['proximity_m']:verification_tp+=1
        else:verification_fp+=1
    return dict(metric_label='position-proximity labels; S3E orientations are placeholders',
        queries_with_causal_positive=valid,recall={str(k):hits[k]/valid if valid else None for k in hits},
        proximity_precision=tp/(tp+fp) if tp+fp else None,proximity_recall=tp/(tp+fn) if tp+fn else None,
        verification_attempts=len(verifications),verification_yield=sum(e['accepted'] for e in verifications)/len(verifications) if verifications else None,
        accepted_proximity_precision=verification_tp/(verification_tp+verification_fp) if verification_tp+verification_fp else None,
        detection_latency_median_s=float(np.median(latencies)) if latencies else None,
        detection_latency_p95_s=float(np.quantile(latencies,0.95)) if latencies else None,
        verification_runtime_s=sum(e['verification_runtime_s'] for e in verifications),
        verification_cache_hits=len(cached),verification_cache_misses=len(uncached),
        verification_cached_runtime_s=sum(e['verification_runtime_s'] for e in cached),
        verification_uncached_runtime_s=sum(e['verification_runtime_s'] for e in uncached))


def trajectory_metrics(graph,gt,cfg,output=None):
    from .evo_evaluation import evaluate
    return evaluate(graph,gt,cfg,output)


def corrected_dense_rows(odom_rows,keyrows,graph_rows):
    # Interpolate the SE(3) correction field, retaining the dense odometry motion.
    optimized={r['keyframe_id']:pose(r['T_world_body']) for r in graph_rows}
    times=np.array([r['stamp_ns'] for r in keyrows],dtype=np.int64)
    corrections=[optimized[r['keyframe_id']]@inv(pose(r['T_world_body'])) for r in keyrows]
    for row in odom_rows:
        stamp=row['stamp_ns'];j=min(max(int(np.searchsorted(times,stamp,side='right')),1),len(times)-1)
        if len(times)==1 or stamp<=times[0]:C=corrections[0]
        elif stamp>=times[-1]:C=corrections[-1]
        else:
            a,b=corrections[j-1],corrections[j];f=float((stamp-times[j-1])/(times[j]-times[j-1]))
            C=np.eye(4);C[:3,3]=(1-f)*a[:3,3]+f*b[:3,3]
            C[:3,:3]=Slerp([0.,1.],Rotation.from_matrix(np.stack([a[:3,:3],b[:3,:3]])))(f).as_matrix()
        yield dict(robot_id=row['robot_id'],frame_id=row['frame_id'],stamp_ns=stamp,
                   T_world_body=(C@pose(row['T_world_body'])).tolist())


def save_tum(path,rows):
    with Path(path).open('w') as f:
        for row in rows:
            T=pose(row['T_world_body']);q=Rotation.from_matrix(T[:3,:3]).as_quat();ns=row['stamp_ns']
            f.write(f'{ns//10**9}.{ns%10**9:09d} '+ ' '.join(f'{x:.12g}' for x in [*T[:3,3],*q])+'\n')


def build_maps(export,keyrows,graphrows,voxel,store=None):
    original=[];optimized=[];count=0
    dense=list(corrected_dense_rows(read_jsonl(Path(export)/'frames.jsonl'),keyrows,graphrows))
    alignment=pose(graphrows[0]['T_initial_body'])@inv(pose(keyrows[0]['T_world_body']))
    def measurements():
        if store is None:
            for (row,cloud,_),new in zip(frames(export,verify=False),dense):
                yield row,transform(extrinsic(row),cloud[:,:3]),new
        else:
            graph_by_id={r['keyframe_id']:r for r in graphrows}
            for row in keyrows:
                with np.load(Path(store)/f'{row["keyframe_id"]:06d}.npz') as data:
                    yield row,data['scan'][:,:3],graph_by_id[row['keyframe_id']]
    for row,local,new in measurements():
        original.append(transform(alignment@pose(row['T_world_body']),local));optimized.append(transform(pose(new['T_world_body']),local));count+=1
        if count%50==0:
            original=[voxel_downsample(np.concatenate(original),voxel)]
            optimized=[voxel_downsample(np.concatenate(optimized),voxel)]
    return voxel_downsample(np.concatenate(original),voxel),voxel_downsample(np.concatenate(optimized),voxel),dense


def evaluate(cfg,artifacts,dataset,output):
    from .cli import SOURCE
    from collections import Counter
    import time
    start=time.monotonic();output=Path(output)
    method=cfg['backend']['name'];LOOPS=f'loops.{frontend_name(cfg)}.{method}';PGO=f'pgo.{frontend_name(cfg)}.{method}'
    title='MegaLoc + MapClosures' if method=='megaloc_mapclosures' else 'MapClosures'
    gt={r:ground_truth(dataset/f'{r.lower()}_gt.txt') for r in cfg['robots']}
    graph=read_json(Path(artifacts[PGO])/'graph.json')
    if 'registration' in graph:title+=' + mixed pose/GICP PGO'
    loop=Path(artifacts[LOOPS]);events=read_jsonl(loop/'events.jsonl')
    rows=[];all_dense=[];all_original=[]
    for robot in cfg['robots']:
        keyroot=Path(artifacts[f'keyframes.{frontend_name(cfg)}.{robot}'])/'store'
        keyrows=read_jsonl(keyroot/'keyframes.jsonl');rows+=keyrows
        graphrows=[r for r in graph['poses'] if r['robot_id']==robot]
        export=Path(artifacts[f'odometry.{frontend_name(cfg)}.{robot}'])/'run/export'
        original,optimized,dense=build_maps(export,keyrows,graphrows,cfg['evaluation']['map_voxel_m'],keyroot)
        np.savez_compressed(output/f'{robot}-maps.npz',original=original.astype(np.float32),optimized=optimized.astype(np.float32))
        initial=[dict(r,T_world_body=r['T_initial_body']) for r in graphrows]
        original_dense=list(corrected_dense_rows(read_jsonl(export/'frames.jsonl'),keyrows,initial))
        for r in dense+original_dense:r['component']=graph['components'][robot]
        all_dense+=dense;all_original+=original_dense
        save_tum(output/f'{robot}-corrected.tum',dense);save_tum(output/f'{robot}-initial.tum',original_dense)
        write_jsonl(output/f'{robot}-corrected.jsonl',dense)
    retrieval=retrieval_metrics(rows,events,gt,cfg['evaluation'],cfg['loops']['same_robot_exclusion_s'])
    retrieval.update(read_json(loop/'summary.json'))
    verifications=[e for e in events if e['type']=='verification']
    constraints=read_jsonl(loop/'constraints.jsonl')
    report=dict(schema_version=2,method=title,keyframes=len(rows),
        components=graph['components'],retrieval=retrieval,
        accepted_intra_loops=sum(e['i'][0]==e['j'][0] for e in constraints),
        accepted_inter_loops=sum(e['i'][0]!=e['j'][0] for e in constraints),
        verification_reasons=dict(Counter(e['reason'] for e in verifications)),
        rejection_reasons=dict(Counter(e['reason'] for e in events if e['type']=='rejection')),
        initial_position=trajectory_metrics({'poses':all_original},gt,cfg['evaluation'],output/'evo/initial'),
        optimized_position=trajectory_metrics({'poses':all_dense},gt,cfg['evaluation'],output/'evo/optimized'),
        gnc_rejected_loops=sum(f['kind']=='loop' and f['robust_weight']<.5 for f in graph['factors']),
        orientation_ground_truth_used=False,
        thresholds=cfg['backend']['fusion'],evaluation_runtime_s=time.monotonic()-start)
    if 'registration' in graph:
        report['registration_factors']=graph['registration']
    def provenance(e):return '+'.join(sorted(e.get('retrieval_sources',[]))) or 'unknown'
    report['branch_attribution']=dict(
        verification_attempts=dict(Counter(provenance(e) for e in verifications)),
        accepted_constraints=dict(Counter(provenance(e['diagnostics']) for e in constraints)),
        lidar_only_below_visual_threshold=sum(e['diagnostics'].get('retrieval_sources')==['mapclosures']
            and e['diagnostics']['visual_similarity']<cfg['backend']['fusion']['visual_min_similarity'] for e in constraints),
        meaning='sources are independently qualified retrieval branches; these are not ablation results')
    write_json(output/'report.json',report)
    with (output/'metrics.csv').open('w',newline='') as f:
        w=csv.writer(f);w.writerow(['component','robots','samples','initial_position_rmse_m','optimized_position_rmse_m'])
        for c,v in report['optimized_position'].items():
            w.writerow([c,'+'.join(v['robots']),v['samples'],report['initial_position'][c]['rmse_m'],v['rmse_m']])
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    colors={'Alpha':'#df5752','Bob':'#388fd0','Carol':'#3faa76'}
    components=sorted(set(graph['components'].values()))
    fig,axes=plt.subplots(1,len(components),figsize=(6*len(components),6),squeeze=False)
    positions={(r['robot_id'],r['keyframe_id']):pose(r['T_world_body'])[:3,3] for r in graph['poses']}
    for ax,c in zip(axes[0],components):
        for robot in cfg['robots']:
            if graph['components'][robot]!=c:continue
            track=[r for r in graph['poses'] if r['robot_id']==robot]
            old=np.array([pose(r['T_initial_body'])[:3,3] for r in track]);new=np.array([pose(r['T_world_body'])[:3,3] for r in track])
            ax.plot(old[:,0],old[:,1],color=colors[robot],alpha=.3,linestyle='--',label=f'{robot} initial')
            ax.plot(new[:,0],new[:,1],color=colors[robot],linewidth=1.7,label=f'{robot} optimized')
        for factor in graph['factors']:
            if factor['kind']!='loop' or factor['robust_weight']<.5:continue
            if graph['components'][factor['i'][0]]!=c or graph['components'][factor['j'][0]]!=c:continue
            a,b=positions[tuple(factor['i'])],positions[tuple(factor['j'])]
            ax.plot([a[0],b[0]],[a[1],b[1]],color='#8254b0',alpha=.3,linewidth=.7)
        ax.set_title('Connected robots: '+', '.join(r for r in cfg['robots'] if graph['components'][r]==c))
        ax.set_aspect('equal');ax.set_xlabel('x (m)');ax.set_ylabel('y (m)');ax.grid(alpha=.2);ax.legend(fontsize=8)
    fig.suptitle(title+' | full S3E Square 1',fontsize=15)
    fig.tight_layout();fig.savefig(output/'trajectories.png',dpi=160);plt.close(fig)
    # Render one strongest accepted cross-robot pair (or an intra-robot pair if none).
    accepted=[e for e in verifications if e['accepted']]
    selected=sorted(accepted,key=lambda e:(e.get('retrieval_sources')!=['mapclosures'],
        e['query'][0]==e['candidate'][0],-(e.get('visual_similarity') or 0)))[:1]
    for e in selected:
        import matplotlib.image as mpimg
        fig,axes=plt.subplots(1,3,figsize=(16,5))
        clouds=[]
        for ax,endpoint in zip(axes[:2],[e['query'],e['candidate']]):
            robot,key=endpoint;p=Path(artifacts[f'keyframes.{frontend_name(cfg)}.{robot}'])/'store'
            with np.load(p/f'{key:06d}.npz') as data:clouds.append(data['cloud'][:,:3])
            if (p/f'{key:06d}.png').exists():ax.imshow(mpimg.imread(p/f'{key:06d}.png'))
            else:
                xyz=clouds[-1];ax.scatter(xyz[::10,0],xyz[::10,1],s=.3);ax.set_aspect('equal')
            ax.set_title(f'{robot} keyframe {key}');ax.axis('off')
        a,b=clouds;b=transform(pose(e['T_i_j']),b)
        axes[2].scatter(a[::10,0],a[::10,1],s=.3,label=e['query'][0]);axes[2].scatter(b[::10,0],b[::10,1],s=.3,label=e['candidate'][0])
        axes[2].set_aspect('equal');axes[2].legend();axes[2].set_title(f"Accepted registration: RMSE {e['rmse_m']:.3f} m")
        fig.tight_layout();fig.savefig(output/'loop-evidence.png',dpi=150);plt.close(fig)
    plan=dict(config=cfg,artifacts=artifacts,output='.')
    write_json(output/'visualization-input.json',plan)
    subprocess.run([str(SOURCE/'.ros2/rerun-venv/bin/python'),str(Path(__file__).with_name('visualize.py')),str(output/'visualization-input.json')],check=True)
    write_json(output/'report.json',dict(report,evaluation_runtime_s=time.monotonic()-start))
