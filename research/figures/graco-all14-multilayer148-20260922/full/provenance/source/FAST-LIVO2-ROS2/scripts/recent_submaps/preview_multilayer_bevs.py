#!/usr/bin/env python3
"""Preview height-sliced ellipsoid BEVs and native matches, without a SLAM replay.

Run in the captured experiment's environment so its pinned native sampler and
MapClosures libraries are used. New slicing code is loaded explicitly below.
No ground truth, GICP, PCM, CBS, or changes to the original run are involved.
"""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import shutil
import time
import zlib
import cv2
import numpy as np
import yaml
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import ConnectionPatch
from matplotlib.transforms import Affine2D
from s3e_pipeline.backends import unpack_array
from s3e_pipeline.area_maps import render_area_surface
from s3e_pipeline.ellipsoid_bev import normalize_exported_basis
from s3e_pipeline.mapclosures_inspection import check_features
import s3e_mapclosures_native as native
import s3e_mapclosures_inspection as inspection

MODULE=Path(__file__).resolve().parents[2]/'research/s3e_pipeline/multilayer_bev.py'
spec=importlib.util.spec_from_file_location('preview_height_layers',MODULE)
layers=importlib.util.module_from_spec(spec);spec.loader.exec_module(layers)


def sha(path):
    h=hashlib.sha256()
    with Path(path).open('rb') as f:
        while chunk:=f.read(2**20):h.update(chunk)
    return h.hexdigest()


def rows(path):return [json.loads(l) for l in Path(path).read_text().splitlines()]
def endpoint(value):
    robot,key=value.rsplit(':',1)
    if not robot.replace('_','').isalnum():raise ValueError('Invalid robot name')
    return robot,int(key)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--work',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--pair',nargs=2,action='append',required=True,metavar=('QUERY','CANDIDATE'))
    args=parser.parse_args();W=args.work.resolve();O=args.output.resolve();O.mkdir(parents=True,exist_ok=False)
    (O/'images').mkdir();(O/'features').mkdir();(O/'pairs').mkdir();started=time.monotonic()
    pairs=[tuple(map(endpoint,p)) for p in args.pair]
    cfg=yaml.safe_load((W/'config.yaml').read_text());mc=cfg['backend']['mapclosures']
    assert native.upstream_commit==inspection.upstream_commit==mc['upstream_commit']
    assert mc['projection_alignment']=='gravity' and mc['inliers_threshold']==5
    trials=json.loads((W/'trials.json').read_text());inputs={str(W/'config.yaml'):sha(W/'config.yaml')}
    keys={r:{x['keyframe_id']:x for x in rows(W/f'prepared-{r}/store/keyframes.jsonl')} for r in trials}
    old_events=[e for r in trials for e in rows(W/f'dpgo/{r}/events.jsonl') if e['type']=='ranked']
    design=dict(bands=[dict(id=n,label=l,minimum_m=lo,maximum_m=hi) for n,l,lo,hi in layers.BANDS],
        terrain_settings=layers.TERRAIN_SETTINGS,pairs=pairs,ground_truth_used=False,
        density_normalization='unchanged native linear counts; independently normalized per layer',
        sampling='exact original native ellipsoid surface sampling; slice sampled surfaces, not ellipsoid centers',
        matching='native ORB + self-pruning + two-map HBST + native RANSAC; same layer matched to same layer',
        thresholds=mc,selection='two previously discussed candidates; positive-pair visual diagnostic, not retrieval benchmark',
        bands_fixed_before_matching=True,pooled_inlier_counts=False,run_full_pipeline=False)
    (O/'design.json').write_text(json.dumps(design,indent=2)+'\n')
    def engine(cls):return cls(mc['density_map_resolution'],mc['density_threshold'],mc['hamming_distance_threshold'])
    maps={};feature_packets={};terrain_debug={}
    for r,k in sorted({e for pair in pairs for e in pair}):
        key=keys[r][k];descriptor=W/f'prepared-{r}/ellipsoid/{k:06d}.json.zlib'
        payload=Path(trials[r])/'frontend/area_maps'/key['payload']
        assert sha(payload)==key['sha256']
        for path in (descriptor,payload):inputs[str(path)]=sha(path)
        packet=json.loads(zlib.decompress(descriptor.read_bytes()))
        frozen={n:unpack_array(v) for n,v in packet['mapclosures'].items()};G=frozen['ground']
        with np.load(payload) as data:points=data['points'].copy();e=data['ellipsoids'].copy()
        level=points@G[:3,:3].T+G[:3,3]
        coefficients,terrain,lower,support=layers.fit_terrain(level)
        terrain_debug[r,k]=(lower,support,coefficients)
        basis,_=normalize_exported_basis(e[:,6:].reshape(-1,3,3),cfg['ellipsoid_basis_roundoff_tolerance'])
        surface,sampling=render_area_surface(e[:,:3],e[:,3:6],basis,G,key['area_radius_m'])
        height=layers.relative_heights(surface@G[:3,:3].T+G[:3,3],coefficients)
        masks=dict(full=np.ones(len(surface),dtype=bool),**layers.layer_masks(height))
        membership=np.stack(list(masks.values())[1:]).sum(axis=0)
        assert np.all(membership<=1) and np.array_equal(membership==1,height>=-.5)
        entry=dict(robot=r,key=k,terrain=terrain,sampling=sampling,source_surface_points=len(surface),
            excluded_below_minus_half_meter=int((height<-.5).sum()),layers=[])
        full_debug=None
        for name,mask in masks.items():
            sliced=surface[mask];stem=f'{r}-{k:06d}-{name}'
            if len(sliced)>=30:
                feature=engine(native.MapClosures).describe(sliced,ground=G)
                debug=engine(inspection.Inspector).density(sliced,G);check_features(debug,feature)
                im=np.asarray(debug['image']);lower_bound=np.asarray(debug['lower_bound'])
            else:
                assert full_debug is not None
                feature=dict(ground=G,xy=np.empty((0,2)),bits=np.empty((0,32),dtype=np.uint8))
                im=np.zeros_like(full_debug['image']);lower_bound=np.asarray(full_debug['lower_bound'])
            if name=='full':
                full_debug=debug;check_features(debug,frozen)
                assert np.array_equal(feature['bits'],frozen['bits']) and np.array_equal(feature['xy'],frozen['xy'])
            assert cv2.imwrite(str(O/'images'/f'{stem}.png'),im)
            np.savez_compressed(O/'features'/f'{stem}.npz',**{n:feature[n] for n in ('ground','xy','bits')},
                lower_bound=lower_bound,terrain_coefficients=coefficients)
            item=dict(id=name,label='Full height (control)' if name=='full' else next(l for n,l,_,_ in layers.BANDS if n==name),
                surface_points=len(sliced),features=len(feature['xy']),image=f'images/{stem}.png',
                lower_bound=lower_bound.tolist(),shape=list(im.shape),ground=G.tolist(),
                png_sha256=sha(O/'images'/f'{stem}.png'))
            feature_packets[r,k,name]=feature;entry['layers'].append(item)
        maps[r,k]=entry
        print(r,k,'terrain height',round(terrain['sensor_height_above_fitted_plane_m'],3),
              'features',[(m['id'],m['features']) for m in entry['layers']],flush=True)
    result=[]
    for q,c in pairs:
        pair_id=f'{q[0]}-{q[1]}__{c[0]}-{c[1]}'
        original=next((x['mapclosures_hypothesis'] for ev in old_events if tuple(ev['query'])==q and ev['candidate_robot']==c[0]
            for x in ev['candidates'] if x['keyframe_id']==c[1]),None)
        # A single frozen full-height pose is used for every comparison image.
        # It never participates in height slicing, descriptor matching, or fits.
        Gq=feature_packets[(*q,'full')]['ground'];Gc=feature_packets[(*c,'full')]['ground']
        if original is None or not original.get('valid_pose'):raise ValueError('Preview needs a saved display pose')
        display_level=Gq@np.array(original['T_i_j'])@np.linalg.inv(Gc)
        display=np.eye(3);display[:2,:2]=display_level[:2,:2];display[:2,2]=display_level[:2,3]
        pair_record=dict(id=pair_id,query=q,candidate=c,original_database_hypothesis=original,
            display_source='saved original full-height RANSAC yaw and XY only; display only',layers=[])
        for name in ['full',*[b[0] for b in layers.BANDS]]:
            qf=feature_packets[(*q,name)];cf=feature_packets[(*c,name)]
            match=engine(native.MapClosures).pair(qf,cf)
            inspector=engine(inspection.Inspector);inspector.add(0,cf);debug=inspector.correspondences(qf,0)
            assert len(debug['query_xy'])==match['matches'] and len(debug['ransac_inlier_indices'])==match['inliers']
            if match['valid_pose']:assert np.allclose(debug['T_i_j'],match['T_i_j'],atol=1e-8)
            np.savez_compressed(O/'pairs'/f'{pair_id}-{name}.npz',**debug)
            hypothesis={k:(v.tolist() if isinstance(v,np.ndarray) else v) for k,v in match.items()}
            valid=bool(match['valid_pose']);passed=valid and match['inliers']>mc['inliers_threshold']
            consistency=None
            if valid:
                fit=Gq@np.array(match['T_i_j'])@np.linalg.inv(Gc)
                a=np.arctan2(fit[1,0],fit[0,0])-np.arctan2(display_level[1,0],display_level[0,0])
                consistency=dict(yaw_difference_deg=float(abs(np.degrees(np.arctan2(np.sin(a),np.cos(a))))),
                    xy_translation_difference_m=float(np.linalg.norm(fit[:2,3]-display_level[:2,3])),
                    reference='original BEV pose, not ground truth')
            item=dict(id=name,matches=match['matches'],inliers=match['inliers'],passes_unchanged_2d_gate=passed,
                hypothesis=hypothesis,consistency=consistency,figure=f'pairs/{pair_id}-{name}-matches.png')
            make_match_figure(O,maps,q,c,name,debug,display,item)
            pair_record['layers'].append(item)
        make_overview(O,maps,q,c,display,pair_record)
        result.append(pair_record)
        print(pair_id,[(i['id'],i['matches'],i['inliers'],i['passes_unchanged_2d_gate']) for i in pair_record['layers']],flush=True)
    fig,axes=plt.subplots(2,2,figsize=(11,10),layout='constrained')
    for ax,((r,k),(low,support,coef)) in zip(axes.flat,terrain_debug.items()):
        residual=layers.relative_heights(low,coef)
        ax.scatter(low[:,0],low[:,1],s=5,color='#b7bec8',label='other low cells')
        artist=ax.scatter(low[support,0],low[support,1],c=residual[support],s=14,cmap='coolwarm',vmin=-.35,vmax=.35)
        ax.set_title(f'{r} / {k}: {support.sum()} supporting cells\nFitted surface below sensor: {-coef[2]:.2f} m')
        ax.set_aspect('equal');ax.set_xlabel('Leveled X [m]');ax.set_ylabel('Leveled Y [m]')
    fig.colorbar(artist,ax=list(axes.flat),label='Height residual of plane-support cells [m]',shrink=.6)
    fig.suptitle('Independent terrain estimates · geometry only · no flight-height prior')
    fig.savefig(O/'terrain-support.png',dpi=140);plt.close(fig)
    assert all(sha(Path(p))==h for p,h in inputs.items())
    summary=dict(complete=True,design=design,maps=list(maps.values()),pairs=result,
        original_full_height_features_exactly_reproduced=True,source_artifacts_unchanged=True,
        input_sha256=inputs,source_sha256={str(p):sha(p) for p in (MODULE,Path(__file__),Path(native.__file__),Path(inspection.__file__))},
        ground_truth_used=False,registration_run=False,pcm_run=False,cbs_run=False,wall_s=time.monotonic()-started)
    (O/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    write_gallery(O,summary)
    for p in (MODULE,Path(__file__)):shutil.copy2(p,O/p.name)
    print('preview complete',round(time.monotonic()-started,2),flush=True)


def layer(maps,endpoint,name):return next(x for x in maps[endpoint]['layers'] if x['id']==name)


def show_layer(ax,O,maps,endpoint,name,registration):
    m=layer(maps,endpoint,name);im=plt.imread(O/m['image']);h,w=im.shape[:2]
    lx,ly=m['lower_bound'];resolution=.5
    pixel=np.array([[0,resolution,resolution*(lx+.5)],[resolution,0,resolution*(ly+.5)],[0,0,1.]])
    A=registration@pixel
    ax.imshow(im,cmap='gray',vmin=0,vmax=1,interpolation='nearest',origin='upper',transform=Affine2D(A)+ax.transData)
    ax.set_facecolor('black');ax.set_aspect('equal');ax.set_xlim(-95,100);ax.set_ylim(-100,110)
    return m


def make_match_figure(O,maps,q,c,name,debug,display,item):
    fig,axes=plt.subplots(1,2,figsize=(13,7),layout='constrained',sharex=True,sharey=True)
    cm=show_layer(axes[0],O,maps,c,name,display);qm=show_layer(axes[1],O,maps,q,name,np.eye(3))
    axes[0].set_title(f'{c[0]} / {c[1]} · {cm["features"]} ORB features')
    axes[1].set_title(f'{q[0]} / {q[1]} · {qm["features"]} ORB features')
    cp=np.array(debug['candidate_xy'])*.5;cp=cp@display[:2,:2].T+display[:2,2]
    qp=np.array(debug['query_xy'])*.5
    members=set(debug['ransac_inlier_indices'])
    for i,(a,b) in enumerate(zip(cp,qp)):
        color='#62ef87' if i in members else '#ff925f'
        for ax,point in zip(axes,(a,b)):
            ax.scatter(*point,s=27,facecolors='none',edgecolors=color,lw=1.)
            ax.annotate(str(i+1),point+np.array([1.3,1.3]),color=color,fontsize=7)
        fig.add_artist(ConnectionPatch(a,b,'data','data',axesA=axes[0],axesB=axes[1],color=color,alpha=.6,lw=.75))
    for ax in axes:ax.set_xlabel('Common horizontal X [m]')
    axes[0].set_ylabel('Common horizontal Y [m]')
    gate='passes >5 gate' if item['passes_unchanged_2d_gate'] else 'does not pass >5 gate'
    fig.suptitle(f'{qm["label"]} · {item["matches"]} matches → {item["inliers"]} RANSAC inliers · {gate}\n'
        'Green: this layer’s RANSAC support; orange: rejected · display uses original BEV heading · no 3D verification',fontsize=13)
    fig.savefig(O/item['figure'],dpi=160);plt.close(fig)


def make_overview(O,maps,q,c,display,pair_record):
    names=['full',*[b[0] for b in layers.BANDS]]
    fig,axes=plt.subplots(2,len(names),figsize=(20,8),layout='constrained')
    for j,name in enumerate(names):
        cm=show_layer(axes[0,j],O,maps,c,name,display);qm=show_layer(axes[1,j],O,maps,q,name,np.eye(3))
        stats=pair_record['layers'][j]
        axes[0,j].set_title(qm['label']+f'\n{stats["matches"]} matches / {stats["inliers"]} inliers',fontsize=11)
        for i,m in enumerate((cm,qm)):
            axes[i,j].set_xticks([]);axes[i,j].set_yticks([])
            axes[i,j].text(.03,.03,f'{m["features"]} features',transform=axes[i,j].transAxes,color='white',fontsize=10)
    axes[0,0].set_ylabel(f'{c[0]} / {c[1]}',fontsize=13);axes[1,0].set_ylabel(f'{q[0]} / {q[1]}',fontsize=13)
    fig.suptitle('Terrain-relative ellipsoid BEV layers · same physical scale and display heading\n'
        'Independent terrain estimates; unchanged ORB / HBST / RANSAC settings; no full pipeline',fontsize=16)
    pair_record['overview']=f'pairs/{pair_record["id"]}-layers.png'
    fig.savefig(O/pair_record['overview'],dpi=150);plt.close(fig)


def write_gallery(O,summary):
    table=[]
    for pair in summary['pairs']:
        for item in pair['layers']:
            table.append(f"| {pair['query']} ↔ {pair['candidate']} | {item['id']} | {item['matches']} | {item['inliers']} | {'Yes' if item['passes_unchanged_2d_gate'] else 'No'} |")
    lines=['**Terrain-relative multilayer ellipsoid BEV preview**','',
        'Four saved submaps, two selected pairs. No bags replayed, no ground truth accessed, no 3D registration, PCM or CBS. The original experiment remains unchanged.','',
        'Each snapshot independently estimates a low-surface plane using 4 m spatial cells and their 10th-percentile native map-point heights. A slope-constrained robust fit supplies terrain-relative heights. The plane is a local approximation, and support does not prove terrain identity when ground is obscured. Fitted planes do not rotate the gravity-aligned XY projection.','',
        'The original ellipsoids are sampled exactly as before. Their surface samples are split into fixed height bands −0.5–2, 2–5, 5–10, 10–20 and ≥20 m. This slices complete sampled surfaces, not ellipsoid centers. Samples below −0.5 m remain available in the full-height control.','',
        'All images use the original 0.5 m pixels, native linear density normalization, ORB and self-pruning, Hamming threshold 50, native 3-pixel RANSAC distance and strict >5-inlier gate. This isolates height slicing; logarithmic density and learned descriptors are not added. Inlier counts are never pooled across layers.','',
        '**These are two-map matching diagnostics, not database retrieval results.** Each band is matched to the same band. The full-height control uses the same two-map protocol, while original database counts are retained separately in summary.json. Full-height images reproduce the original cached ORB features exactly.','',
        'All comparisons use one saved full-height RANSAC pose only to set a common display heading and position. It does not guide terrain estimation, layer matching or RANSAC. Green lines indicate each layer’s own RANSAC support; they are not accepted 3D loop correspondences.','',
        '| Query ↔ candidate | Height band | Matches | Inliers | Passes unchanged 2D gate |','|---|---|---:|---:|---|',*table,'',
        '[Interactive gallery](index.html) · [Complete diagnostics](summary.json) · [Reference papers and adaptations](REFERENCES.md)','',
        '![Terrain support](terrain-support.png)']
    for pair in summary['pairs']:lines+=['',f"![Layer overview]({pair['overview']})"]
    (O/'README.md').write_text('\n'.join(lines)+'\n')
    html='''<!doctype html><html lang="en"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Ground–aerial multilayer BEV preview</title><style>body{font:16px system-ui;background:#edf1f6;color:#142237;margin:0}main{max-width:1500px;margin:auto;padding:28px}h1{margin-top:0}p{line-height:1.5}.controls{display:flex;gap:18px;flex-wrap:wrap;padding:16px;background:white;border-radius:10px}select{font:inherit;padding:7px}figure{margin:20px 0;background:white;padding:12px;border-radius:10px}img{width:100%;height:auto}a{color:#1554a0}.stats{padding:12px;background:#fff;margin-top:14px}details{margin:18px 0}summary{cursor:pointer;font-weight:bold}</style>
<main><h1>Ground–aerial multilayer ellipsoid BEVs</h1>
<p>Height bands refer to an independently estimated terrain surface, not the sensor origin. The RANSAC acceptance gate remains <strong>strictly more than five inliers</strong>. All panels share the original pair’s display heading and scale. No ground truth, registration, PCM or CBS was used in this preview.</p>
<div class="controls"><label>Submap pair <select id="pair"></select></label><label>Height band <select id="band"></select></label></div>
<div class="stats" id="stats"></div><figure><img id="matches" alt="Native descriptor matches and RANSAC inliers"><figcaption>Green: layer-specific RANSAC inliers. Orange: rejected matches. A passing 2D gate does not establish a valid 3D loop.</figcaption></figure>
<details open><summary>All height layers for this pair</summary><figure><img id="overview" alt="All BEV height layers"></figure></details>
<details><summary>Terrain estimation and limitations</summary><p>A robust plane fits spatially distributed low-height map cells independently in each snapshot. It estimates a dominant low surface; roof-only observations or nonplanar terrain can invalidate its interpretation as ground. Layers use unchanged native density images and match independently; inlier counts are not added across bands.</p><img src="terrain-support.png" alt="Terrain support cells"></details>
<p><a href="README.md">Method and complete results</a> · <a href="REFERENCES.md">Reference papers and adaptations</a> · <a href="summary.json">Numeric diagnostics</a></p></main>
<script>const DATA=__DATA__;const el=id=>document.getElementById(id);DATA.pairs.forEach((p,i)=>el('pair').add(new Option(`${p.candidate.join('/')} ↔ ${p.query.join('/')}`,i)));const labels={full:'Full height (control)',...Object.fromEntries(DATA.design.bands.map(b=>[b.id,b.label]))};Object.entries(labels).forEach(([id,label])=>el('band').add(new Option(label,id)));function show(){const p=DATA.pairs[+el('pair').value],m=p.layers.find(m=>m.id===el('band').value);el('matches').src=m.figure;el('overview').src=p.overview;let c=m.consistency;el('stats').textContent=`${m.matches} matches → ${m.inliers} inliers · ${m.passes_unchanged_2d_gate?'passes':'does not pass'} unchanged >5 gate. ${c?`Difference from original BEV pose: ${c.xy_translation_difference_m.toFixed(2)} m XY, ${c.yaw_difference_deg.toFixed(2)}° yaw (not ground-truth accuracy).`:'No valid planar pose.'}`;}el('pair').onchange=show;el('band').onchange=show;show();</script></html>'''
    (O/'index.html').write_text(html.replace('__DATA__',json.dumps(summary)))


if __name__=='__main__':main()
