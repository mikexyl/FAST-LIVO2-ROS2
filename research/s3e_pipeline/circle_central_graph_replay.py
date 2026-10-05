"""Replay only graph/solution delivery from immutable staged detection captures.

Retrieval does not consume optimizer results in this sealed experiment. Correcting
its alignment admission policy therefore needs no repeated BEV or GICP work.
Timings remain separate measured stages, not a claimed fresh end-to-end latency.
"""
import argparse
from dataclasses import asdict
from pathlib import Path
import resource
import subprocess
import time
import numpy as np
import yaml

from .artifacts import file_hash,read_json,read_jsonl,write_json,write_jsonl
from .central_circle import RobotClient,communication_summary
from .circle_cover_replay import MemorySampler
from .mixed_pgo import refine_graph
from .pgo import optimize


def replay(capture,cfg,source,output):
    capture=Path(capture).resolve();output=Path(output);output.mkdir(parents=True,exist_ok=False)
    original=read_json(capture/'configuration.json')
    if cfg['backend']!=original['backend'] or cfg['circle_cover']!=original['circle_cover']:
        raise ValueError('Captured geometry/descriptor/verification profile differs')
    write_json(output/'configuration.json',cfg)
    write_json(output/'source-hashes.json',dict(capture=read_json(capture/'source-hashes.json'),
        replay_source_sha256=file_hash(__file__),graph_policy=cfg['pgo']))
    prep=read_json(capture/'preparation-summary.json');prep.update(reused=True,original_preparation_root=str(capture))
    write_json(output/'preparation-summary.json',prep)
    roots={};stores={};rows=[]
    for robot in cfg['robots']:
        root=output/robot;root.mkdir();roots[robot]=root
        for name in ('store','descriptors','images','union.npz','placement.json','placement-history.jsonl'):
            (root/name).symlink_to(capture/robot/name)
        stores[robot]=capture/'central/server-stores'/robot
        rows.extend(read_jsonl(stores[robot]/'keyframes.jsonl'))
    central=output/'central';central.mkdir()
    loops=read_jsonl(capture/'central/constraints.jsonl')
    write_jsonl(central/'constraints.jsonl',loops)
    (central/'events.jsonl').symlink_to(capture/'central/events.jsonl')
    ledger=[];clients={};start=time.perf_counter();cpu0=time.process_time()
    u0=resource.getrusage(resource.RUSAGE_CHILDREN)
    with (output/'gpu.csv').open('w') as gpu_log:
        observer=subprocess.Popen(['nvidia-smi','--query-gpu=timestamp,memory.used,utilization.gpu',
            '--format=csv,noheader,nounits','-lms','100'],stdout=gpu_log,stderr=subprocess.DEVNULL)
        try:
            with MemorySampler() as memory:
                t=time.perf_counter();graph=optimize(rows,loops,cfg['pgo']);pose_s=time.perf_counter()-t
                write_json(central/'pose-graph.json',graph)
                t=time.perf_counter();graph=refine_graph(graph,stores,cfg['central_registration'],source,central)
                registration_s=time.perf_counter()-t;write_json(central/'graph.json',graph)
                try:
                    for robot,root in roots.items():
                        clients[robot]=RobotClient(root,robot,cfg['backend'],ledger)
                        clients[robot].call('solution',poses=[r for r in graph['poses'] if r['robot_id']==robot])
                finally:
                    for client in clients.values():client.close()
        finally:observer.terminate();observer.wait(10)
    u1=resource.getrusage(resource.RUSAGE_CHILDREN)
    resources=dict(wall_s=time.perf_counter()-start,parent_cpu_s=time.process_time()-cpu0,
        child_cpu_s=u1.ru_utime+u1.ru_stime-u0.ru_utime-u0.ru_stime,
        peak_simultaneous_process_tree_rss_bytes=memory.peak,
        scope='fresh graph solve and actual solution delivery only; frozen detection/geometry stages reused',
        rss_scope='actual simultaneous 20 ms process-tree sum; shared pages can be counted twice',
        gpu_scope='whole-device 100 ms samples during corrected graph stage')
    write_json(output/'resources.json',resources)
    # Keep only the earlier, actually measured pre-graph traffic, then append the
    # actual graph-replay session/delivery messages. Do not estimate byte counts.
    earlier=[r for r in read_jsonl(capture/'central/communication.jsonl') if r['kind'] in ('ready','manifest','geometry')]
    combined=earlier+ledger;write_jsonl(central/'communication.jsonl',combined)
    previous=read_json(capture/'central/summary.json')
    summary=dict(previous,components=graph['components'],
        retained_loops=sum(f['kind']=='loop' and f['solver_weight']>0 for f in graph['factors']),
        registration_factors=sum(f['kind']=='registration' for f in graph['factors']),
        communication=communication_summary(combined),graph_replay_resources=resources,
        capture_source=str(capture),alignment_min_support=cfg['pgo']['alignment_min_support'],
        capture_backend_wall_s=previous['wall_s'],capture_stage=previous['stage'],
        stage=dict(publication_s=previous['stage']['publication_s'],retrieval_s=previous['stage']['retrieval_s'],
            verification_and_geometry_transfer_s=previous['stage']['verification_and_geometry_transfer_s'],
            pose_graph_s=pose_s,registration_graph_s=registration_s),
        timing_scope='separately measured fixed detection capture and corrected graph stage; not fresh end-to-end latency')
    summary['wall_s']=sum(summary['stage'].values())
    write_json(central/'summary.json',summary)
    return summary


def main():
    p=argparse.ArgumentParser();p.add_argument('--config',required=True);p.add_argument('--capture',required=True)
    p.add_argument('--source',required=True);p.add_argument('--output',required=True)
    a=p.parse_args();replay(a.capture,yaml.safe_load(Path(a.config).read_text()),a.source,a.output)


if __name__=='__main__':main()
