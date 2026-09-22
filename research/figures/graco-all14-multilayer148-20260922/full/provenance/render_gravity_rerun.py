"""Gravity-level presentation of final CBS geometry; no GT display alignment."""
from pathlib import Path
import sys,json
import numpy as np
import rerun as rr
import rerun.blueprint as rrb
sys.path.insert(0,'/workspace/FAST-LIVO2-ROS2/research')
from s3e_pipeline.robot_colors import robot_colors
from s3e_pipeline.gravity_bev import gravity_ground
B=Path(__file__).resolve().parent;W=B/'full';report=W/'report'
read=lambda p:json.loads(p.read_text())
rows=lambda p:[json.loads(s) for s in p.read_text().splitlines()]
assert read(W/'status.json')['phase']=='complete'
assert rr.__version__=='0.37.1'
summary=read(report/'report.json');components=summary['components'];groups=sorted(set(components.values()))
colors=robot_colors(list(components));poses=rows(W/'dpgo/poses.jsonl')
lookup={(r['robot_id'],r['keyframe_id']):np.asarray(r['T_world_body']) for r in poses}
rotations={}
for group in groups:
    key=rows(W/f'prepared-{group}/store/keyframes.jsonl')[0]
    R=gravity_ground(lookup[group,key['keyframe_id']][:3,:3]@np.asarray(key['gravity_imu_m_s2']))[:3,:3]
    assert np.allclose(R.T@R,np.eye(3)) and np.isclose(np.linalg.det(R),1)
    rotations[group]=R
rr.init('GRACO all 14 / gravity-level distributed CBS')
rr.save(str(report/'result-gravity.rrd'))
rr.send_blueprint(rrb.Blueprint(rrb.Horizontal(*[
    rrb.Spatial3DView(name=f'CBS component {g}',origin=f'/world/{g}') for g in groups],
    rrb.TextDocumentView(name='Evaluation and provenance',origin='/report'))))
rr.log('/report',rr.TextDocument('Gravity-level display from estimated IMU gravity. No ground-truth display alignment. Native processed accumulated-map geometry; not full-resolution raw scans.\n\n'+json.dumps(summary,indent=2)),static=True)
for robot,group in components.items():
    root=f'/world/{group}';R=rotations[group]
    rr.log(root,rr.ViewCoordinates.RIGHT_HAND_Z_UP,static=True)
    with np.load(report/f'{robot}-maps.npz',allow_pickle=False) as data:
        points=data['cbs'];points=points[::max(1,len(points)//250000)]@R.T
    rr.log(f'{root}/{robot}/map',rr.Points3D(points,colors=colors[robot],radii=.035),static=True)
    track=np.loadtxt(report/f'{robot}-cbs.tum')[:,1:4]@R.T
    rr.log(f'{root}/{robot}/trajectory',rr.LineStrips3D([track],colors=colors[robot],radii=.08),static=True)
loops=rows(W/'dpgo/constraints.jsonl')
for group in groups:
    segments=[[rotations[group]@lookup[tuple(e[k])][:3,3] for k in ('i','j')]
        for e in loops if components[e['i'][0]]==group]
    if segments:rr.log(f'/world/{group}/retained_loops',rr.LineStrips3D(segments,colors=[235,80,205],radii=.05),static=True)
rr.get_data_recording().flush();rr.disconnect()
(report/'gravity-display.json').write_text(json.dumps(dict(ground_truth_used=False,source='estimated anchor IMU gravity',
    unchanged_estimator_poses=True,R_display_component={k:v.tolist() for k,v in rotations.items()}),indent=2)+'\n')
print(report/'result-gravity.rrd')
