#!/usr/bin/env python3
"""Reconstruct saved runtime layer images and verify their cached ORB features."""
import argparse,hashlib,json,time,zlib
from pathlib import Path
import cv2,numpy as np,yaml
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from s3e_pipeline.backends import unpack_array
from s3e_pipeline.multilayer_mapclosures import decode_layers,NAMES
from s3e_pipeline.multilayer_bev import relative_heights,layer_masks
from s3e_pipeline.area_maps import render_area_surface
from s3e_pipeline.ellipsoid_bev import normalize_exported_basis
from s3e_pipeline.mapclosures_inspection import check_features
import s3e_mapclosures_inspection as native

p=argparse.ArgumentParser(description=__doc__);p.add_argument('work',type=Path);p.add_argument('--output',type=Path,required=True);args=p.parse_args()
W=args.work;O=args.output;O.mkdir(parents=True,exist_ok=True);(O/'images').mkdir(exist_ok=True)
cfg=yaml.safe_load((W/'config.yaml').read_text());mc=cfg['backend']['mapclosures'];items=[];started=time.monotonic()
labels=['Full height (control)','Near terrain: −0.5–2 m','Low: 2–5 m','Middle: 5–10 m','High: 10–20 m','Upper: ≥20 m']
for robot in cfg['robots']:
 index=W/f'prepared-{robot}/store/keyframes.jsonl'
 if not index.exists():continue
 for row in map(json.loads,index.read_text().splitlines()):
  key=row['keyframe_id'];stem=f'{robot}-{key:04d}';saved=O/'images'/f'{stem}.json'
  if saved.exists():items.append(json.loads(saved.read_text()));continue
  dpath=W/f'prepared-{robot}/ellipsoid/{key:06d}.json.zlib'
  d=json.loads(zlib.decompress(dpath.read_bytes()));features={'full':{k:unpack_array(v) for k,v in d['mapclosures'].items()},**decode_layers(d)}
  payload=W/f'frontends/{robot}/frontend/area_maps'/row['payload']
  assert hashlib.sha256(payload.read_bytes()).hexdigest()==row['sha256']
  with np.load(payload) as data:e=data['ellipsoids'].copy()
  G=features['full']['ground'];terrain=d['mapclosures_multilayer']['terrain'];surface=np.empty((0,3))
  if len(e):
   basis,_=normalize_exported_basis(e[:,6:].reshape(-1,3,3),cfg['ellipsoid_basis_roundoff_tolerance'])
   surface,_=render_area_surface(e[:,:3],e[:,3:6],basis,G,row['area_radius_m'])
  masks={n:np.zeros(len(surface),dtype=bool) for n in NAMES}
  if terrain['available']:
   level=surface@G[:3,:3].T+G[:3,3]
   masks=layer_masks(relative_heights(level,terrain['coefficients_z_equals_ax_by_c']))
  masks={'full':np.ones(len(surface),dtype=bool),**masks}
  fig,axes=plt.subplots(2,3,figsize=(12,8),layout='constrained');details={}
  for ax,(name,mask),label in zip(axes.flat,masks.items(),labels):
   selected=surface[mask];f=features[name]
   if len(selected)>=30:
    inspector=native.Inspector(mc['density_map_resolution'],mc['density_threshold'],mc['hamming_distance_threshold'])
    debug=inspector.density(selected,G);check_features(debug,f)
    im=np.asarray(debug['image']);lo=np.asarray(debug['lower_bound'])*mc['density_map_resolution']
    hi=lo+np.asarray(im.shape)*mc['density_map_resolution']
    ax.imshow(im.T,origin='lower',extent=[lo[0],hi[0],lo[1],hi[1]],cmap='gray_r',vmin=0,vmax=255,interpolation='nearest')
   else:assert len(f['xy'])==0
   radius=row['area_radius_m'];ax.set(xlim=(-radius,radius),ylim=(-radius,radius),aspect='equal',xlabel='Level x [m]',ylabel='Level y [m]')
   ax.set_title(f'{label}\n{len(f["xy"])} ORB features');ax.plot(0,0,'+',color='#e75433',markersize=7)
   if name!='full' and not terrain['available']:ax.text(.5,.5,'Terrain unavailable',transform=ax.transAxes,ha='center')
   details[name]=dict(features=len(f['xy']),surface_points=int(mask.sum()))
  fig.suptitle(f'{robot} · area {key} · 80 m horizontal radius · gravity-aligned top-down XY',fontsize=14)
  path=O/'images'/f'{stem}.png';fig.savefig(path,dpi=125);plt.close(fig)
  item=dict(robot=robot,key=key,stamp_ns=row['stamp_ns'],available_ns=row['available_ns'],image=f'images/{stem}.png',
   terrain=terrain,layers=details,payload_sha256=row['sha256'],descriptor_sha256=hashlib.sha256(dpath.read_bytes()).hexdigest(),
   image_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),cached_orb_reproduced=True)
  saved.write_text(json.dumps(item));items.append(item)
 print(robot,'rendered',sum(x['robot']==robot for x in items),flush=True)
summary=dict(maps=items,wall_s=time.monotonic()-started,ground_truth_used=False,cached_orb_reproduced=True)
(O/'summary.json').write_text(json.dumps(summary,indent=2))
html='''<!doctype html><html lang="en"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>GRACO 14-robot multilayer submaps</title><style>body{font:16px system-ui;background:#f0f3f6;color:#172b44;margin:24px auto;max-width:1400px;padding:0 20px}select,input{font:inherit;margin:8px;padding:6px}img{width:100%;background:white}#info{line-height:1.6}a{color:#1765a7}</style><h1>GRACO multilayer ellipsoid BEVs</h1><p>Saved runtime descriptors. Each snapshot contains accumulated native map geometry inside an 80 m horizontal radius. Height layers use an independently estimated low-surface plane; odometry uses the persistent map. The orange cross is the current IMU origin. Every image reproduces its saved ORB features.</p><label>Robot <select id="robot"></select></label><label>Submap <select id="map"></select></label><div id="info"></div><img id="image" alt="Six gravity-aligned BEV layers"><p><a href="summary.json">Numeric records and hashes</a></p><script>const DATA=__DATA__;const el=id=>document.getElementById(id);[...new Set(DATA.maps.map(x=>x.robot))].forEach(r=>el('robot').add(new Option(r,r)));function show(){let m=DATA.maps.find(x=>x.robot===el('robot').value&&x.key===+el('map').value);el('image').src=m.image;el('info').textContent=m.terrain.available?`Estimated sensor height above fitted low surface: ${m.terrain.sensor_height_above_fitted_plane_m.toFixed(2)} m. Support: ${m.terrain.support_cells} cells. This is a fitted surface, not a semantic terrain label.`:`Layer features unavailable: ${m.terrain.reason}`;}function robot(){el('map').replaceChildren();DATA.maps.filter(x=>x.robot===el('robot').value).forEach(m=>el('map').add(new Option(m.key,m.key)));show()}el('robot').onchange=robot;el('map').onchange=show;robot();</script></html>'''
(O/'index.html').write_text(html.replace('__DATA__',json.dumps(summary)))
