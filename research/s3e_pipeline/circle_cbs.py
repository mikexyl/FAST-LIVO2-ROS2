"""Distributed CBS over immutable fixed spatial submap Pose3 nodes.

Replays verified measurements, never a centralized solution. Geometry is sealed
at capture end; this runner makes no online snapshot-availability claim.
"""
import argparse
import os
from pathlib import Path
import resource
import shutil
import subprocess
import time
import yaml
import numpy as np
from scipy.spatial.transform import Rotation
from .artifacts import digest, file_hash, read_json, read_jsonl, write_json, write_jsonl
from .cbs_bridge import local_graph
from .circle_cover_replay import MemorySampler
from .persistent_cbs import PersistentSession, save_snapshot, common_reference_ready
from .spatial_cbs_contract import endpoint
from .geometry import inv, pose


def prepare(capture, output, config):
    capture=Path(capture).resolve(); output=Path(output); output.mkdir(parents=True,exist_ok=False)
    original=read_json(capture/'configuration.json')
    if (config['backend'] != original['backend'] or config['robots'] != original['robots'] or
        config['circle_cover'] != original['circle_cover']):
        raise ValueError('Frozen detection/verification configuration differs')
    evidence=config['backend']['evidence']
    if evidence.get('sampling')!='fixed' or evidence.get('max_range_m') is not None:
        raise ValueError('Spatial replay requires fixed verification voxels without range cropping')
    rows={}; stores={}; lookup={}
    for robot in config['robots']:
        root=output/robot; store=root/'store'; store.mkdir(parents=True); stores[robot]=store
        rows[robot]=read_jsonl(capture/robot/'store/keyframes.jsonl')
        for row in rows[robot]:
            row['cbs_evidence_voxel_m']=evidence['voxel_m']
            lookup[robot,row['keyframe_id']]=endpoint(row)
            source=capture/robot/'store'/f"{row['keyframe_id']:06d}.npz"
            if file_hash(source)!=row['sha256']: raise ValueError('Modified frozen input')
            target=store/source.name
            # Robot-local Landlock permissions cannot follow links into other
            # stores. Immutable hardlinks (or copies across filesystems) can.
            try: os.link(source,target)
            except OSError: shutil.copy2(source,target)
        local_graph(rows[robot],robot,config['pgo'],store=store)
        write_jsonl(store/'keyframes.jsonl',rows[robot])
        for name in ('union.npz','placement.json','placement-history.jsonl','images'):
            path=capture/robot/name
            if path.exists(): (root/name).symlink_to(path)
    loops=read_jsonl(capture/'central/constraints.jsonl')
    for edge in loops:
        if not edge.get('accepted'): raise ValueError('Unverified graph input')
        edge.setdefault('diagnostics',{})['spatial_endpoints']=[lookup[tuple(edge[k])] for k in ('i','j')]
    write_json(output/'configuration.json',config)
    write_jsonl(output/'verified-input.jsonl',loops)
    write_json(output/'input-hashes.json',dict(capture=str(capture),
        source_constraints_sha256=file_hash(capture/'central/constraints.jsonl'),
        adapted_constraints_sha256=file_hash(output/'verified-input.jsonl'),
        configuration_sha256=digest(config), indexes={r:file_hash(stores[r]/'keyframes.jsonl') for r in rows}))
    write_json(output/'source-hashes.json',{p.name:file_hash(p) for p in sorted(Path(__file__).parent.glob('*.py'))})
    return stores,rows,loops


def solution_graph(snapshot, rows):
    poses=snapshot['poses']; robots=snapshot['summary']['robots']
    expected={(r,k['keyframe_id']):k['stamp_ns'] for r,keys in rows.items() for k in keys}
    actual={(p['robot_id'],p['keyframe_id']):p['stamp_ns'] for p in poses}
    if actual != expected or len(poses)!=len(expected): raise ValueError('Lost or changed submap identity')
    indexed={(p['robot_id'],p['keyframe_id']):pose(p['T_world_body']) for p in poses}
    consistency=[]
    for e in snapshot['constraints']:
        prediction=inv(indexed[tuple(e['i'])])@indexed[tuple(e['j'])]
        error=inv(pose(e['T_i_j']))@prediction
        consistency.append(dict(i=e['i'],j=e['j'],translation_m=float(np.linalg.norm(error[:3,3])),
            rotation_deg=float(np.degrees(Rotation.from_matrix(error[:3,:3]).magnitude()))))
    return dict(poses=poses,components={r:s['component'] for r,s in robots.items()},
        unconnected_submaps=[p for p in poses if not p.get('optimized',True)],
        alignment_complete=all(p.get('optimized',True) for p in poses),
        local_constraints=snapshot.get('local_constraints',[]),
        constraints=snapshot['constraints'],solver='CBS with owned submap Pose3 keys',node_type='fixed_spatial_submap',
        published_loop_residuals=consistency,
        identity='robot_id, keyframe_id; creation timestamps may repeat')


def evidence_comparison(session, central_registration):
    central=read_json(central_registration)
    expected={tuple(c['endpoint']):c['payload_sha256'] for c in central['clouds']}
    checked={}
    manifests=list(Path(session).glob('*/registration/1/manifest.json'))+list(Path(session).glob('*/registration/manifest.json'))
    for manifest in manifests:
        for c in read_json(manifest)['clouds']:
            key=(c['robot'],c['key'])
            if key not in expected or expected[key]!=c['payload_sha256']:
                raise ValueError(f'Central/CBS prepared evidence differs: {key}')
            checked[key]=c['payload_sha256']
    return dict(all_transferred_endpoints_match=True,endpoints=len(checked),
        central_endpoints=len(expected),hashes=[dict(robot=r,key=k,sha256=h) for (r,k),h in sorted(checked.items())])


def sealed(config, source, output, rows, loops):
    from .dpgo import run as optimize
    from .frontends import frontend_name
    frontend=frontend_name(config)
    inputs=output/'loop-input';inputs.mkdir();write_jsonl(inputs/'constraints.jsonl',loops)
    artifacts={f'keyframes.{frontend}.{r}':str(output/r) for r in config['robots']}
    artifacts[f'loops.{frontend}.{config["backend"]["name"]}']=str(inputs)
    optimize(config,artifacts,source,output/'cbs')
    summary=read_json(output/'cbs/summary.json')
    for r,status in summary['robots'].items():
        poses=read_jsonl(output/'cbs'/r/'poses.jsonl');status['component']=poses[0]['component']
    summary['finished']=True
    local = [e for r in config['robots'] for e in read_json(output/'cbs'/r/'local-graph.json')['constraints']]
    return dict(summary=summary,poses=read_jsonl(output/'cbs/poses.jsonl'),local_constraints=local,
                constraints=read_jsonl(output/'cbs/constraints.jsonl'))


def run(capture, config, source, output, central_registration=None):
    if config['dpgo']['execution'] not in ('sealed','persistent'):
        raise ValueError('Expected sealed or persistent CBS')
    output=Path(output); stores,rows,loops=prepare(capture,output,config)
    from .dpgo import native_overlay
    overlay=native_overlay(source,config['dpgo']);source=Path(source)
    write_json(output/'native-hashes.json',{str(p):file_hash(p) for p in (
        overlay/'cbs/lib/libcbs.so',overlay/'cbs_ros/lib/cbs_ros/cbs_ros_node',
        source/'cbs/src/bpsam/bpsam.cpp',source/'cbs/include/cbs/bpsam/bpsam.h',
        source/'cbs_ros/src/cbs_ros_node.cpp',source/'cbs_ros/srv/Keys.srv')})
    (output/'cbs').mkdir(); started=time.perf_counter(); cpu=time.process_time()
    children=resource.getrusage(resource.RUSAGE_CHILDREN); session=None
    with (output/'gpu.csv').open('w') as log:
        observer=subprocess.Popen(['nvidia-smi','--query-gpu=timestamp,memory.used,utilization.gpu',
            '--format=csv,noheader,nounits','-lms','100'],stdout=log,stderr=subprocess.DEVNULL)
        try:
            with MemorySampler() as memory:
                if config['dpgo']['execution']=='sealed':
                    snapshot=sealed(config,source,output,rows,loops)
                    ready_s=None;reference_s=None;solve_s=time.perf_counter()-started
                else:
                    session=PersistentSession(config,source,output/'cbs/session',stores)
                    while not session.ready():
                        if time.perf_counter()-started>config['dpgo']['timeout_s']: raise TimeoutError('CBS startup')
                        time.sleep(.1)
                    ready_s=time.perf_counter()-started; session.submit(rows,loops,final=False)
                    # First establish a shared frame through actual beliefs.
                    # Ten local iterations cannot guarantee it on sparse chains.
                    while True:
                        snapshot=session.snapshot()
                        if snapshot and common_reference_ready(snapshot,config['robots']): break
                        if time.perf_counter()-started-ready_s>config['dpgo']['timeout_s']:
                            raise TimeoutError('Submap component reference propagation')
                        time.sleep(.1)
                    reference_s=time.perf_counter()-started-ready_s
                    write_json(output/'reference-barrier.json',dict(elapsed_s=reference_s,
                        iterations=snapshot['summary']['iterations'],components={r:s['component'] for r,s in snapshot['summary']['robots'].items()}))
                    session.submit(rows,loops,final=True)
                    while True:
                        snapshot=session.snapshot()
                        if snapshot and snapshot['summary']['finished']: break
                        time.sleep(.1)
                    solve_s=time.perf_counter()-started-ready_s
                    save_snapshot(snapshot,output/'cbs')
        finally:
            if session: session.close()
            observer.terminate(); observer.wait(10)
    end=resource.getrusage(resource.RUSAGE_CHILDREN)
    write_json(output/'resources.json',dict(wall_s=time.perf_counter()-started,startup_s=ready_s,
        input_to_finished_s=solve_s,reference_propagation_s=reference_s,parent_cpu_s=time.process_time()-cpu,
        child_cpu_s=end.ru_utime+end.ru_stime-children.ru_utime-children.ru_stime,
        peak_simultaneous_process_tree_rss_bytes=memory.peak,
        scope=f'fresh {config["dpgo"]["execution"]} CBS processes, PCM, geometry exchange and settling; frozen retrieval reused',
        rss_scope='actual simultaneous 20 ms process-tree RSS; shared pages may count twice',
        gpu_scope='whole-device 100 ms samples; includes unrelated activity'))
    graph=solution_graph(snapshot,rows);graph['execution']=config['dpgo']['execution']; (output/'central').mkdir()
    # Reuse the identical reconstruction/evaluation adapter, not its solver.
    write_json(output/'central/graph.json',graph)
    if central_registration:
        write_json(output/'evidence-comparison.json',evidence_comparison(output/'cbs/session' if session else output/'cbs',central_registration))
    summary=snapshot['summary']; statuses=summary['robots'].values()
    sent=sum((s['cbs_last_stats'] or {}).get('bandwidth_sent_bytes',0) for s in statuses)
    recv=sum((s['cbs_last_stats'] or {}).get('bandwidth_recv_bytes',0) for s in statuses)
    coordination=summary.get('loop_network_cdr_bytes',0)
    write_json(output/'communication.json',dict(cbs_sent_cdr_bytes=sent,cbs_received_cdr_bytes=recv,
        cbs_requests_plus_responses_cdr_bytes=sent+recv,
        pcm_network_cdr_bytes=summary['pcm_network_cdr_bytes'],
        registration_sent_cdr_bytes=summary['registration_network_cdr_bytes'],
        coordination_sent_cdr_bytes=coordination,
        total_graph_application_cdr_bytes=sent+recv+summary['pcm_network_cdr_bytes']+summary['registration_network_cdr_bytes']+coordination,
        scope='graph-stage ROS2 serialized application messages; excludes DDS/IP overhead, discovery and reused retrieval',
        note='Native CBS counts requests and responses at the requester only, so their sum counts each message once; PCM and geometry count sent messages'))
    from .circle_central_evaluation import evaluate_run
    evaluate_run(output,config)
    write_json(output/'COMPLETE.json',dict(success=True,nodes=len(graph['poses']),retained_loops=len(graph['constraints']),
        config_sha256=digest(config),runner_sha256=file_hash(__file__)))


def main():
    p=argparse.ArgumentParser(); p.add_argument('--capture',type=Path,required=True)
    p.add_argument('--config',type=Path,required=True);p.add_argument('--source',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--central-registration',type=Path)
    a=p.parse_args(); config=yaml.safe_load(a.config.read_text())
    run(a.capture,config,a.source,a.output,a.central_registration)


if __name__=='__main__': main()
