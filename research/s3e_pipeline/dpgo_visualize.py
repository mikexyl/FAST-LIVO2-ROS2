"""Rerun 0.37.1 export in its isolated Python environment."""
import sys
from pathlib import Path
import numpy as np
from .artifacts import read_json, read_jsonl
from .geometry import pose


def visualize(cfg, artifacts, poses, factors, stats, events, output, report):
    import rerun as rr
    import rerun.blueprint as rrb
    from .visualize import COLORS
    from .geometry import transform
    if rr.__version__ != '0.37.1': raise ValueError('Rerun 0.37.1 required')
    rr.init('S3E MegaLoc + MapClosures + CBS'); rr.save(str(output/'result.rrd'))
    rr.log('/', rr.ViewCoordinates.FLU, static=True)
    components = sorted({p['component'] for p in poses})
    rr.send_blueprint(rrb.Blueprint(rrb.Horizontal(rrb.Tabs(*[
        rrb.Spatial3DView(name=c, origin=f'/components/{c}') for c in components],
        rrb.Spatial3DView(name='Registration', origin='/registration')),
        rrb.Vertical(rrb.TimeSeriesView(name='CBS convergence', contents=['/convergence/**']),
                     rrb.Spatial2DView(name='Query', origin='/inspection/query'),
                     rrb.Spatial2DView(name='Candidate', origin='/inspection/candidate'),
                     rrb.TextDocumentView(name='Evaluation', origin='/report')))))
    import json
    rr.log('/report', rr.TextDocument(json.dumps(report, indent=2)), static=True)
    lookup = {}
    for robot in cfg['robots']:
        rows = [p for p in poses if p['robot_id'] == robot]; prefix = f'/components/{rows[0]["component"]}/{robot}'
        for label, field in [('optimized', 'T_world_body'), ('initial', 'T_initial_body')]:
            rr.log(prefix+'/'+label, rr.LineStrips3D([[pose(p[field])[:3, 3] for p in rows]],
                colors=COLORS[robot] if label == 'optimized' else [130, 130, 130]), static=True)
        with np.load(output/f'{robot}-maps.npz') as maps:
            rr.log(prefix+'/optimized_map', rr.Points3D(maps['optimized'], colors=COLORS[robot], radii=.025), static=True)
            rr.log(prefix+'/original_map', rr.Points3D(maps['original'], colors=[100,100,100], radii=.02), static=True)
        lookup.update({(robot, p['keyframe_id']): pose(p['T_world_body'])[:3,3] for p in rows})
        for stat in stats[robot]:
            rr.set_time('iteration', sequence=stat['iteration'])
            rr.log(f'/convergence/{robot}/pose_change', rr.Scalars(stat['result_logmap_change']))
        # Reuse previously sampled sensor MCAP with official ROS2 decoders.
        if 'evaluate' in artifacts:
            mcap = Path(artifacts['evaluate'])/f'{robot}-keyframes.mcap'
            if mcap.exists(): rr.log_file_from_path(mcap, entity_path_prefix='/sensors')
    for index, edge in enumerate(factors):
        if edge['kind'] != 'loop': continue
        component = next(p['component'] for p in poses if p['robot_id'] == edge['i'][0])
        rr.log(f'/components/{component}/loops/{index}', rr.LineStrips3D([[lookup[tuple(edge['i'])], lookup[tuple(edge['j'])]]],
            colors=[240,200,50]), static=True)
    verifications = [e for e in events if e['type'] == 'verification']
    for index in np.linspace(0, len(verifications)-1, min(len(verifications), 80), dtype=int):
        edge = verifications[index]; rr.set_time('event', sequence=int(index))
        for label in ('query', 'candidate'):
            robot, key = edge[label]; store = Path(artifacts[f'keyframes.livo.{robot}'])/'store'
            rr.log('/inspection/'+label, rr.EncodedImage(path=store/f'{key:06d}.png'))
            with np.load(store/f'{key:06d}.npz') as data: cloud = data['cloud'][:, :3]
            if label == 'candidate':
                T = edge.get('T_i_j', edge.get('initial_T_i_j'))
                if T is None: rr.log('/registration/candidate', rr.Clear(recursive=True)); continue
                cloud = transform(pose(T), cloud)
            rr.log('/registration/'+label, rr.Points3D(cloud, colors=COLORS[robot]))
    rr.get_data_recording().flush()


if __name__ == '__main__':
    plan_path = Path(sys.argv[1]).resolve(); plan = read_json(plan_path)
    output = plan_path.parent / plan['output']; artifacts = plan['artifacts']
    dpgo = Path(artifacts['dpgo']); robots = plan['config']['robots']
    stats = {r:read_jsonl(dpgo/r/'stats.jsonl') for r in robots}
    events = [e for r in robots for e in read_jsonl(dpgo/r/'events.jsonl')]
    visualize(plan['config'], artifacts, read_jsonl(output/'poses.jsonl'),
              read_jsonl(output/'factors.jsonl'), stats, events, output, read_json(output/'report.json'))
