"""Static figures and an inspectable Rerun recording for the bounded BEV test."""
import argparse
import json
from pathlib import Path
import numpy as np


def archive_inputs(work,output):
    """Retain the selected raw submaps and replay metadata before MCAP cleanup."""
    import shutil
    import sys
    sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
    from s3e_pipeline.ellipsoid_bev import extract_robot, ROBOTS, SOURCE
    from s3e_pipeline.artifacts import file_hash, write_json, write_jsonl
    spec=json.loads((work/'test.json').read_text());summary=json.loads((output/'summary.json').read_text())
    provenance=output/'provenance';provenance.mkdir(exist_ok=True)
    for relative,expected in summary['source_hashes'].items():
        source=SOURCE/relative
        if file_hash(source)!=expected:raise ValueError(f'Evaluated source changed: {source}')
        target=provenance/'source'/relative;target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(source,target)
    rows=[]
    for robot in ROBOTS:
        target=provenance/robot;target.mkdir(exist_ok=True)
        for name in ('runtime.yaml','mapping_config.yaml','summary.json'):
            shutil.copy2(work/robot/name,target/name)
        shutil.copy2(work/robot/'export/manifest.json',target/'export-manifest.json')
        for item in extract_robot(work,robot,spec['schedule'][robot]):
            key=item['keyframe_id'];row={k:v for k,v in item['row'].items() if k!='messages'}
            rows.append(dict(row,keyframe_id=key))
            np.savez_compressed(output/'maps'/f'{robot}-{key:06d}-raw-bev.npz',cloud=item['raw'])
    write_jsonl(output/'snapshots.jsonl',rows)
    for name in ('baseline-check.json','tests.log','test.json'):
        shutil.copy2(work/name,provenance/name)
    write_json(provenance/'binary-hashes.json',{
        str(p.relative_to(SOURCE)):file_hash(p) for p in [
            SOURCE/'.ros2/ellipse-install/ellipselio/lib/libellipselio_mapping.so',
            SOURCE/'.ros2/ellipse-install/ellipselio/lib/ellipselio/ellipselio_mapping_node']})


def read_rows(path):return [json.loads(line) for line in path.read_text().splitlines()]


def load(output, endpoint, branch):
    robot,key=endpoint
    with np.load(output/'maps'/f'{robot}-{key:06d}-{branch}.npz',allow_pickle=False) as f:
        return {k:f[k] for k in f.files}


def pair_image(query,candidate,matches):
    a,b=query['image'],candidate['image'];offset=a.shape[1]+24
    canvas=np.zeros((max(a.shape[0],b.shape[0]),offset+b.shape[1]),dtype=np.uint8)
    canvas[:a.shape[0],:a.shape[1]]=a;canvas[:b.shape[0],offset:]=b
    q=(matches['query_xy']-query['lower_bound'])[:,::-1]
    c=(matches['candidate_xy']-candidate['lower_bound'])[:,::-1]+[offset,0]
    lines=np.stack([q,c],axis=1)
    mask=np.zeros(len(q),dtype=bool);mask[matches['ransac_inlier_indices'].astype(int)]=True
    return canvas,lines,mask


def showcase(events):
    ellipses=[e for e in events if e['branch']=='ellipsoid']
    return max(ellipses,key=lambda e:(e['registration']['accepted'],e['native']['inliers']))['pair_index']


def figures(output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.collections import LineCollection
    from scipy.spatial.transform import Rotation
    # Convert in the research environment; keep SciPy out of the isolated
    # Rerun environment. Flipping an eigenvector preserves ellipsoid geometry.
    for path in sorted((output/'maps').glob('*-geometry.npz')):
        with np.load(path) as data:basis=data['basis'].astype(float).copy()
        basis[np.linalg.det(basis)<0,:,2]*=-1
        quaternions=Rotation.from_matrix(basis).as_quat() if len(basis) else np.empty((0,4))
        np.save(path.with_name(path.stem.replace('-geometry','-rotation')+'.npy'),quaternions.astype('f4'))
    events=read_rows(output/'events.jsonl');index=showcase(events)
    selected=[e for e in events if e['pair_index']==index]
    fig,axes=plt.subplots(2,2,figsize=(11,10),layout='constrained')
    for row,e in enumerate(selected):
        for col,endpoint in enumerate((e['i'],e['j'])):
            data=load(output,endpoint,e['branch']);ax=axes[row,col]
            ax.imshow(data['image'],cmap='gray',vmin=0,vmax=255,interpolation='nearest')
            kept=data['orb_xy'][data['kept_indices'].astype(int)]
            ax.scatter(kept[:,0],kept[:,1],s=9,facecolors='none',edgecolors='#20d987',linewidths=.6)
            name='Raw scans (5 s)' if e['branch']=='raw' else 'Native ellipsoid surfaces (spatial map)'
            ax.set_title(f'{endpoint[0]} {endpoint[1]} — {name}\n{len(kept)} retained ORB features',fontsize=10)
            ax.set_xlabel('Ground-aligned y bin (image column)');ax.set_ylabel('Ground-aligned x bin (image row)')
    fig.suptitle('Square 1 · MapClosures BEV input comparison\nFixed 0.5 m BEV resolution, density threshold 0.05',fontsize=14)
    fig.savefig(output/'bevs.png',dpi=170);fig.savefig(output/'bevs.pdf');plt.close(fig)
    fig,axes=plt.subplots(2,1,figsize=(13,11),layout='constrained')
    for ax,e in zip(axes,selected):
        q,c=[load(output,p,e['branch']) for p in (e['i'],e['j'])]
        with np.load(output/'pairs'/f'{index:03d}-{e["branch"]}.npz') as f:m={k:f[k] for k in f.files}
        canvas,lines,mask=pair_image(q,c,m)
        ax.imshow(canvas,cmap='gray',vmin=0,vmax=255)
        ax.add_collection(LineCollection(lines[~mask],colors='#f56a5b',linewidths=.4,alpha=.3))
        ax.add_collection(LineCollection(lines[mask],colors='#20d987',linewidths=.8,alpha=.8))
        reg=e['registration'];metric=f', GICP RMSE {reg["rmse_m"]:.3f} m' if reg.get('rmse_m') is not None else ''
        ax.set_title(f'{e["branch"].capitalize()}: {e["native"]["matches"]} matches, {mask.sum()} RANSAC inliers · {reg["reason"]}{metric}')
        ax.axis('off')
    fig.suptitle(f'Native MapClosures ORB/HBST matches · {selected[0]["i"]} ↔ {selected[0]["j"]}\nGreen: RANSAC inliers; red: rejected matches',fontsize=13)
    fig.savefig(output/'matches.png',dpi=170);fig.savefig(output/'matches.pdf');plt.close(fig)


def rerun(output):
    import rerun as rr
    import rerun.blueprint as rrb
    if rr.__version__!='0.37.1':raise ValueError('Rerun 0.37.1 required')
    events=read_rows(output/'events.jsonl');index=showcase(events)
    # Showcase first, then all remaining positive pairs, raw and ellipsoid side by side.
    ordered=sorted({e['pair_index'] for e in events},key=lambda i:(i!=index,i))
    rr.init('EllipseLIO ellipsoid BEVs · MapClosures');rr.save(str(output/'ellipsoid-bev.rrd'))
    rr.send_blueprint(rrb.Blueprint(rrb.Vertical(
        rrb.Horizontal(rrb.Spatial2DView(name='Raw BEV matches',origin='/raw/matches'),
                       rrb.Spatial2DView(name='Ellipsoid BEV matches',origin='/ellipsoid/matches')),
        rrb.Horizontal(rrb.Tabs(rrb.Spatial3DView(name='Query native ellipsoids',origin='/ellipsoids/query'),
                              rrb.Spatial3DView(name='Candidate native ellipsoids',origin='/ellipsoids/candidate')),
                       rrb.Tabs(rrb.Spatial3DView(name='Raw BEV → GICP',origin='/raw/registration'),
                                rrb.Spatial3DView(name='Ellipsoid BEV → GICP',origin='/ellipsoid/registration')),
                       rrb.TextDocumentView(name='Results & method',origin='/status')),row_shares=[.6,.4]),
        rrb.TimePanel(timeline='pair',play_state='Paused'),collapse_panels=True))
    for sequence,index in enumerate(ordered):
        rr.set_time('pair',sequence=sequence)
        selected=[e for e in events if e['pair_index']==index];description=[]
        for e in selected:
            branch=e['branch'];root='/'+branch
            q,c=[load(output,p,branch) for p in (e['i'],e['j'])]
            geometry=[load(output,p,'geometry') for p in (e['i'],e['j'])]
            with np.load(output/'pairs'/f'{index:03d}-{branch}.npz') as f:m={k:f[k] for k in f.files}
            canvas,lines,mask=pair_image(q,c,m)
            rr.log(root+'/matches/image',rr.Image(canvas,color_model='L'))
            rr.log(root+'/matches/query_ORB',rr.Points2D(q['orb_xy'][q['kept_indices'].astype(int)],
                colors=[85,170,255,150],radii=1.0))
            rr.log(root+'/matches/candidate_ORB',rr.Points2D(c['orb_xy'][c['kept_indices'].astype(int)]+[q['image'].shape[1]+24,0],
                colors=[85,170,255,150],radii=1.0))
            rr.log(root+'/matches/inliers',rr.LineStrips2D(lines[mask],colors=[32,217,135],radii=.5))
            rr.log(root+'/matches/outliers',rr.LineStrips2D(lines[~mask],colors=[245,106,91,80],radii=.3))
            rr.log(root+'/registration',rr.ViewCoordinates.FLU)
            rr.log(root+'/registration/query',rr.Points3D(geometry[0]['raw'],colors=[80,170,255],radii=.06))
            T=e['registration'].get('T_i_j',e['native'].get('T_i_j'))
            if T is None:rr.log(root+'/registration/candidate',rr.Clear(recursive=True))
            else:
                T=np.asarray(T);p=geometry[1]['raw']@T[:3,:3].T+T[:3,3]
                rr.log(root+'/registration/candidate',rr.Points3D(p,colors=[255,175,70],radii=.06))
            description.append(f'**{branch}:** {e["native"]["matches"]} matches, {e["native"]["inliers"]} RANSAC inliers; '+
                               e['registration']['reason']+f'; RMSE {e["registration"].get("rmse_m", "n/a")} m')
            if branch=='ellipsoid':
                for role,g,endpoint in zip(('query','candidate'),geometry,(e['i'],e['j'])):
                    path='/ellipsoids/'+role;rr.log(path,rr.ViewCoordinates.FLU)
                    step=max(1,int(np.ceil(len(g['centers'])/15000)));sl=slice(None,None,step)
                    basis=g['basis'][sl].astype(float).copy()
                    if not len(basis):
                        rr.log(path+'/native',rr.Clear(recursive=True))
                        rr.log(path+'/raw',rr.Points3D(g['raw'],colors=[200,200,200,90],radii=.035))
                        continue
                    basis[np.linalg.det(basis)<0,:,2]*=-1
                    colors=np.tile([230,175,80,150],(len(basis),1))
                    colors[g['primitive'][sl]==85]=[90,180,255,150]
                    colors[g['primitive'][sl]==170]=[100,230,155,150]
                    quaternions=np.load(output/'maps'/f'{endpoint[0]}-{endpoint[1]:06d}-rotation.npy')
                    rr.log(path+'/native',rr.Ellipsoids3D(centers=g['centers'][sl],half_sizes=g['axes'][sl],
                        quaternions=quaternions[sl],colors=colors))
                    rr.log(path+'/raw',rr.Points3D(g['raw'],colors=[200,200,200,90],radii=.035))
        text=f'# {selected[0]["i"]} ↔ {selected[0]["j"]}\n\n'+'\n\n'.join(description)+'''

BEV: native density counting, 0.5 m/pixel, threshold 0.05. Native ORB/HBST/RANSAC unchanged. Blue points are retained ORB features. Green matches are RANSAC inliers; red matches are outliers.

Ellipsoid input samples the actual fitted surfaces (centers, geometric semi-axes, orientations); 0.125 m sampling, 0.25 m voxelization, 80 m range. No Gaussian kernel or axis inflation.

The native ellipsoid map is spatial and causal. Raw reference uses trailing 5 s scans. GICP acceptance uses the same fresh raw geometry for both branches.

Blue ellipsoids: planes; green: lines; orange: balls. 3D display samples at most 15,000 ellipsoids per view; every exported ellipsoid contributes to BEV rendering.

Eight selected previously accepted pairs, not a full-sequence recall benchmark. No ground truth used. Scrub the pair timeline to inspect failures as well as successes.
'''
        rr.log('/status',rr.TextDocument(text,media_type=rr.MediaType.MARKDOWN))
    rr.get_data_recording().flush()


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('output',type=Path);p.add_argument('--rerun',action='store_true')
    p.add_argument('--archive-work',type=Path)
    args=p.parse_args()
    if args.archive_work:archive_inputs(args.archive_work,args.output)
    elif args.rerun:rerun(args.output)
    else:figures(args.output)
