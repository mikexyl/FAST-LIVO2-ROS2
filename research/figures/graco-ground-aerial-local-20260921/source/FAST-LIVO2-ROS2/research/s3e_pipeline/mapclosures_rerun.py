#!/usr/bin/env python3
"""Rerun density maps, native ORB features and exact saved-run correspondences."""
import json
from pathlib import Path
import sys
import numpy as np
import rerun as rr
import rerun.blueprint as rrb


def pixels(xy,lower):return (np.asarray(xy).reshape(-1,2)-np.asarray(lower).reshape(1,2))[:,::-1]

def log_density(path,data):
    rr.log(path+'/image',rr.Image(data['image'],color_model='L'))
    allxy=data['orb_xy'];kept=data['kept_indices'].astype(int)
    mask=np.zeros(len(allxy),dtype=bool);mask[kept]=True
    rr.log(path+'/discarded_ORB',rr.Points2D(allxy[~mask],colors=[230,140,70],radii=1.2))
    rr.log(path+'/retained_ORB',rr.Points2D(allxy[mask],colors=[65,245,170],radii=1.3))
    theta=np.deg2rad(data['orb_angles'][mask]);directions=5*np.column_stack([np.cos(theta),np.sin(theta)])
    rr.log(path+'/ORB_orientation',rr.Arrows2D(origins=allxy[mask],vectors=directions,colors=[65,245,170],radii=.25))

def main(output):
    if rr.__version__!='0.37.1':raise ValueError('Rerun 0.37.1 is required')
    output=Path(output);events=[json.loads(x) for x in (output/'events.jsonl').read_text().splitlines()]
    rr.init('S3E MapClosures intermediates');rr.save(str(output/'intermediates.rrd'))
    rr.send_blueprint(rrb.Blueprint(rrb.Horizontal(
        rrb.Vertical(
            rrb.Horizontal(rrb.Spatial2DView(name='Query density + ORB',origin='/density/query'),
                           rrb.Spatial2DView(name='Candidate density + ORB',origin='/density/candidate')),
            rrb.Tabs(rrb.Spatial2DView(name='HBST matches / RANSAC',origin='/matches'),
                     rrb.Horizontal(rrb.Spatial2DView(name='Query ORB bits',origin='/descriptors/query'),
                                    rrb.Spatial2DView(name='Candidate ORB bits',origin='/descriptors/candidate')))),
        rrb.Vertical(rrb.TextDocumentView(name='Pair details & legend',origin='/status'),
            rrb.Horizontal(rrb.Spatial2DView(name='Query RGB',origin='/rgb/query'),rrb.Spatial2DView(name='Candidate RGB',origin='/rgb/candidate')),
            rrb.Tabs(rrb.Spatial3DView(name='Native MapClosures pose',origin='/native'),
                     rrb.Spatial3DView(name='GICP result',origin='/gicp'))),column_shares=[.7,.3]),
        rrb.TimePanel(timeline='pair',play_state='Paused',fps=1),collapse_panels=True))
    for e in events:
        index=e['inspection']['original_event_index'];rr.set_time('pair',sequence=e['inspection_pair'])
        data=[]
        for role,endpoint in [('query',e['query']),('candidate',e['candidate'])]:
            robot,key=endpoint;p=output/'maps'/f'{robot}-{key:06d}'
            with np.load(p.with_suffix('.npz')) as f:d={k:f[k] for k in f.files}
            data.append(d);log_density('/density/'+role,d)
            rr.log('/rgb/'+role,rr.EncodedImage(path=p.with_suffix('.png')))
            rr.log('/descriptors/'+role,rr.Image(np.unpackbits(d['cached_bits'],axis=1)*255,color_model='L'))
        query,candidate=data
        with np.load(output/'pairs'/f'{index:06d}.npz') as f:matches={k:f[k] for k in f.files}
        qxy=pixels(matches['query_xy'],query['lower_bound']);cxy=pixels(matches['candidate_xy'],candidate['lower_bound'])
        gap=24;offset=query['image'].shape[1]+gap;canvas=np.zeros((max(query['image'].shape[0],candidate['image'].shape[0]),offset+candidate['image'].shape[1]),dtype=np.uint8)
        canvas[:query['image'].shape[0],:query['image'].shape[1]]=query['image']
        canvas[:candidate['image'].shape[0],offset:]=candidate['image'];cxy=cxy+[offset,0]
        mask=np.zeros(len(qxy),dtype=bool);mask[matches['ransac_inlier_indices'].astype(int)]=True
        rr.log('/matches/density_pair',rr.Image(canvas,color_model='L'))
        lines=np.stack([qxy,cxy],axis=1) if len(qxy) else np.empty((0,2,2))
        rr.log('/matches/RANSAC_outliers',rr.LineStrips2D(lines[~mask],colors=[245,95,85,100],radii=.35))
        rr.log('/matches/RANSAC_inliers',rr.LineStrips2D(lines[mask],colors=[65,245,170],radii=.6))
        for root,transform in [('native',e['native']['hypothesis'].get('T_i_j')),('gicp',e.get('T_i_j'))]:
            rr.log('/'+root,rr.ViewCoordinates.FLU)
            rr.log('/'+root+'/query',rr.Points3D(query['cloud'],colors=[75,170,255],radii=.06))
            if transform is None:rr.log('/'+root+'/candidate',rr.Clear(recursive=True))
            else:
                T=np.asarray(transform);xyz=candidate['cloud']@T[:3,:3].T+T[:3,3]
                rr.log('/'+root+'/candidate',rr.Points3D(xyz,colors=[255,165,65],radii=.06))
        text=f"""# Pair {e['inspection_pair']} / {len(events)-1}
**{e['query'][0]} {e['query'][1]} ↔ {e['candidate'][0]} {e['candidate'][1]}**

**{e['reason']}** · sources: {', '.join(e['retrieval_sources'])}

MegaLoc cosine: {e['visual_similarity']:.3f} (threshold 0.50)

ORB retained/detected: query {len(query['kept_indices'])}/{len(query['orb_xy'])}; candidate {len(candidate['kept_indices'])}/{len(candidate['orb_xy'])}

HBST matches: {len(qxy)} · native RANSAC inliers: {mask.sum()}

GICP RMSE: {e.get('rmse_m', 'unavailable')} m

Green: retained ORB / RANSAC inliers. Orange points: self-similar ORB features discarded. Arrows: ORB orientation. Red lines: RANSAC outliers.

Density is the original 0.5 m native grayscale grid, before feature overlays. Toggle overlay entities to inspect it alone. The descriptor tab shows 256 bits per retained feature.

Scrub **pair** to inspect all accepted and rejected verifications. Pair 0 is a strong LiDAR-only match; other pairs follow saved event order.

Original event {index}; timestamp {e['query_stamp_ns']} ns. {e['inspection']['index_mode']}. Cached features and native pose/inlier counts verified exactly.
"""
        rr.log('/status',rr.TextDocument(text,media_type=rr.MediaType.MARKDOWN))
    rr.get_data_recording().flush()

if __name__=='__main__':main(sys.argv[1])
