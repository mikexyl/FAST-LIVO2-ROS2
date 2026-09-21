import json,statistics
from pathlib import Path
import numpy as np
B=Path(__file__).resolve().parent;W=B/'smoke';summary={}
def read(p):return json.loads(p.read_text())
def rows(p):return [json.loads(l) for l in p.read_text().splitlines()]
def stats(values):return dict(min=float(min(values)),median=float(statistics.median(values)),max=float(max(values)))
for robot in ['aerial05','aerial08']:
 trial=W/'frontends'/robot;updates=rows(trial/'frontend/native_updates.jsonl');maps=rows(trial/'frontend/area_maps/index.jsonl')
 stamps=np.array([r['sensor_stamp_ns'] for r in updates]);memory=rows(trial/'memory.jsonl');image_manifest=read(W/'bevs/manifest.json')
 last=maps[-1]
 with np.load(trial/'frontend/area_maps'/last['payload']) as data:
  age=(last['anchor_sensor_ns']-stamps[data['point_scan_ids']])/1e9
  old=dict(max_age_s=float(age.max()),median_age_s=float(np.median(age)),older_than_10_s_fraction=float(np.mean(age>=10)),older_than_30_s_fraction=float(np.mean(age>=30)),older_than_60_s_fraction=float(np.mean(age>=60)))
 analytic=rows(trial/'recording/analytics.jsonl')
 images=[r for r in image_manifest['maps'] if r['robot']==robot]
 summary[robot]=dict(**read(W/f'{robot}-quality.json'),artifact_validation=read(trial/'artifact-validation.json'),
  area_snapshots=len(maps),retrievable=sum(r['retrievable'] for r in maps),radius_m=last['area_radius_m'],selected_points=stats([r['geometry_count'] for r in maps]),
  last_selected_points=last['geometry_count'],last_archive_points=last['archive_point_count'],last_point_ages=old,
  ellipsoids=stats([r['ellipsoid_count'] for r in maps]),processing_s=stats([r['processing_s'] for r in updates]),mapper_peak_rss_mib=max(x['VmHWM'] for x in memory)/1024,
  live_analytics=len(analytic),native_updates=len(updates),features=stats([r['features'] for r in images]),
  image_aspect=stats([max(r['width'],r['height'])/min(r['width'],r['height']) for r in images]))
(W/'implementation-summary.json').write_text(json.dumps(summary,indent=2)+'\n');print(json.dumps(summary,indent=2))
