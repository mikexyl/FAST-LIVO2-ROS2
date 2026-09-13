#!/usr/bin/env python3
"""One visual/LiDAR loop result with official ROS2 MCAP decoding."""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import numpy as np
import rerun as rr
import rerun.blueprint as rrb
from s3e_pipeline.artifacts import read_json,read_jsonl
from s3e_pipeline.geometry import pose,transform
COLORS={'Alpha':[235,85,80],'Bob':[70,170,245],'Carol':[95,205,130]}


def visualize(plan):
    if rr.__version__!='0.37.1':raise RuntimeError('Rerun 0.37.1 is required')
    cfg=plan['config'];artifacts=plan['artifacts'];output=Path(plan['output'])
    method=cfg['backend']['name']
    graph=read_json(Path(artifacts[f'pgo.livo.{method}'])/'graph.json')
    rr.init('S3E '+method);rr.save(str(output/'result.rrd'))
    rr.log('/',rr.ViewCoordinates.FLU,static=True)
    views=[rrb.Spatial3DView(name=f'{c} component',origin=f'/components/{c}') for c in sorted(set(graph['components'].values()))]
    rr.send_blueprint(rrb.Blueprint(rrb.Horizontal(
        rrb.Tabs(*views,rrb.Spatial3DView(name='Registration',origin='/registration')),
        rrb.Vertical(rrb.Spatial2DView(name='Query',origin='/inspection/query'),rrb.Spatial2DView(name='Candidate',origin='/inspection/candidate'),
                     rrb.TextDocumentView(name='Result',origin='/inspection/status'),rrb.TimeSeriesView(name='Residuals',contents=['/residuals/**']))),
        rrb.TimePanel(timeline='event',play_state='Paused')))
    from mcap.reader import NonSeekingReader
    from mcap.writer import Writer,CompressionType
    positions={}
    for robot in cfg['robots']:
        odom=Path(artifacts[f'odometry.livo.{robot}'])/'run/export'
        keys=read_jsonl(Path(artifacts[f'keyframes.livo.{robot}'])/'store/keyframes.jsonl')
        times={r['stamp_ns'] for r in keys[::cfg['evaluation']['rerun_sensor_stride']]}
        mcap_path=output/f'{robot}-keyframes.mcap'
        with (odom/'sensors.mcap').open('rb') as source,mcap_path.open('wb') as target:
            writer=Writer(target,compression=CompressionType.ZSTD);writer.start(profile='ros2');channels={}
            for schema,channel,message in NonSeekingReader(source).iter_messages(log_time_order=False):
                if message.log_time not in times:continue
                if channel.id not in channels:
                    sid=writer.register_schema(schema.name,schema.encoding,schema.data)
                    channels[channel.id]=writer.register_channel(channel.topic,channel.message_encoding,sid)
                writer.add_message(channels[channel.id],message.log_time,message.data,publish_time=message.publish_time)
            writer.finish()
        rr.log_file_from_path(mcap_path,entity_path_prefix='/sensors')
        prefix=f'/components/{graph["components"][robot]}/{robot}'
        track=[r for r in graph['poses'] if r['robot_id']==robot]
        xyz=np.array([pose(r['T_world_body'])[:3,3] for r in track]);raw=np.array([pose(r['T_initial_body'])[:3,3] for r in track])
        rr.log(prefix+'/optimized',rr.LineStrips3D([xyz],colors=COLORS[robot]),static=True)
        rr.log(prefix+'/initial',rr.LineStrips3D([raw],colors=[150,150,150]),static=True)
        with np.load(output/f'{robot}-maps.npz') as maps:
            rr.log(prefix+'/optimized_map',rr.Points3D(maps['optimized'],colors=COLORS[robot],radii=.025),static=True)
            rr.log(prefix+'/initial_map',rr.Points3D(maps['original'],colors=[100,100,100],radii=.02),static=True)
        positions.update({(r['robot_id'],r['keyframe_id']):pose(r['T_world_body'])[:3,3] for r in track})
    for f in graph['factors']:
        if f['kind']!='loop':continue
        a,b=graph['components'][f['i'][0]],graph['components'][f['j'][0]]
        if a!=b:continue
        color=[80,210,110] if f['robust_weight']>=.5 else [250,65,65]
        rr.log(f'/components/{a}/loops/{f["factor_index"]}',rr.LineStrips3D([[positions[tuple(f['i'])],positions[tuple(f['j'])]]],colors=color),static=True)
        rr.set_time('factor',sequence=f['factor_index']);rr.log('/residuals/graph',rr.Scalars(f['squared_whitened_residual']))
    events=[e for e in read_jsonl(Path(artifacts[f'loops.livo.{method}'])/'events.jsonl') if e['type']=='verification']
    indices=np.linspace(0,len(events)-1,min(len(events),cfg['evaluation']['rerun_verification_limit']),dtype=int)
    for index in indices:
        e=events[index];rr.set_time('event',sequence=int(index))
        for label,endpoint in [('query',e['query']),('candidate',e['candidate'])]:
            robot,key=endpoint;p=Path(artifacts[f'keyframes.livo.{robot}'])/'store'
            rr.log('/inspection/'+label,rr.EncodedImage(path=p/f'{key:06d}.png'))
            with np.load(p/f'{key:06d}.npz') as data:points=data['cloud'][:,:3]
            if label=='candidate':
                T=e.get('T_i_j',e.get('initial_T_i_j'))
                if T is None:rr.log('/registration/candidate',rr.Clear(recursive=True));continue
                points=transform(pose(T),points)
            rr.log('/registration/'+label,rr.Points3D(points,colors=COLORS[robot]))
        rr.log('/inspection/status',rr.TextDocument(f"{e['query']} → {e['candidate']}\n{e['reason']}\nRetrieval sources: {e.get('retrieval_sources')}\nVisual similarity: {e.get('visual_similarity')}\nNative initializer: {e.get('native')}"))
    rr.get_data_recording().flush()


if __name__=='__main__':
    source=Path(sys.argv[1]).resolve();plan=read_json(source)
    if not Path(plan['output']).is_absolute():plan['output']=str(source.parent/plan['output'])
    visualize(plan)
