#!/usr/bin/env python3
"""Inspectable CBS result from verified artifacts, separate from live captures."""
import argparse
import json
from pathlib import Path
import numpy as np
import rerun as rr
import rerun.blueprint as rrb
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'research'))
from s3e_pipeline.robot_colors import robot_colors

p=argparse.ArgumentParser();p.add_argument('work',type=Path);args=p.parse_args()
if rr.__version__!='0.37.1':raise RuntimeError('Rerun 0.37.1 required')
report=args.work/'report';summary=json.loads((report/'report.json').read_text())
components=summary['components'];groups=sorted(set(components.values()))
colors=robot_colors(list(components))
rr.init(str(summary.get('dataset','Recent-history submaps'))+' / distributed CBS')
rr.save(str(report/'result.rrd'))
rr.send_blueprint(rrb.Blueprint(rrb.Horizontal(*[
    rrb.Spatial3DView(name=f'CBS component {group}',origin=f'/world/{group}') for group in groups],
    rrb.TextDocumentView(name='Evaluation and provenance',origin='/report'))))
rr.log('/report',rr.TextDocument(summary.get('map_source','Completed native submaps; not full-resolution raw LiDAR.')+'\n\n'+json.dumps(summary,indent=2)),static=True)
for robot,group in components.items():
    root=f'/world/{group}'
    rr.log(root,rr.ViewCoordinates.RIGHT_HAND_Z_UP,static=True)
    with np.load(report/f'{robot}-maps.npz',allow_pickle=False) as data:
        points=data['cbs'];points=points[::max(1,len(points)//250000)]
        rr.log(f'{root}/{robot}/map',rr.Points3D(points,colors=colors[robot],radii=.035),static=True)
    trajectory=np.loadtxt(report/f'{robot}-cbs.tum')
    rr.log(f'{root}/{robot}/trajectory',rr.LineStrips3D([trajectory[:,1:4]],colors=colors[robot],radii=.08),static=True)
poses=[json.loads(line) for line in (args.work/'dpgo/poses.jsonl').read_text().splitlines()]
lookup={(r['robot_id'],r['keyframe_id']):np.array(r['T_world_body']).reshape(4,4)[:3,3] for r in poses}
loops=[json.loads(line) for line in (args.work/'dpgo/constraints.jsonl').read_text().splitlines()]
for group in groups:
    segments=[[lookup[tuple(e['i'])],lookup[tuple(e['j'])]] for e in loops if components[e['i'][0]]==group]
    if segments:rr.log(f'/world/{group}/retained_loops',rr.LineStrips3D(segments,colors=[235,80,205],radii=.05),static=True)
rr.get_data_recording().flush();rr.disconnect()
