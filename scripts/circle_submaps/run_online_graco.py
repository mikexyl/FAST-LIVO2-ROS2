#!/usr/bin/env python3
"""Fresh concurrent sensor replay -> creation-time circles -> live DDS -> CBS."""
import argparse
import json
import os
from pathlib import Path
import resource
import signal
import subprocess
import sys
import time
import traceback

import numpy as np
import psutil
import yaml

PROJECT=Path(__file__).resolve().parents[2]; SOURCE=PROJECT.parent
sys.path.insert(0,str(PROJECT/'research'))
from s3e_pipeline.artifacts import digest, file_hash, read_json, read_jsonl, write_json
from s3e_pipeline.circle_cover_replay import MemorySampler
from s3e_pipeline.online_io import JsonlTail, atomic_json


def run(args):
    cfg=yaml.safe_load(args.config.read_text()); robots=args.robots or cfg['robots'];cfg['robots']=robots
    import s3e_mapclosures_native as native
    if not hasattr(native.MapClosures,'query_correspondences'):
        raise ValueError('Pinned multilayer native adapter unavailable')
    # Spawned verifier processes inherit Landlock. Grant only the pinned binary
    # directory, rather than letting imports fall through to a legacy adapter.
    cfg['backend']['read_paths']=list(dict.fromkeys([*cfg['backend'].get('read_paths',[]),
        str(Path(native.__file__).resolve().parent)]))
    cfg.update(dataset=None,experiment_name='Live GRACO circles: fresh 1x odometry, peer retrieval and persistent CBS')
    for name in ('stores','dense_trajectories'):cfg.pop(name,None)
    cfg['dpgo'].update(execution='persistent',persistent_reference_barrier=True,
        persistent_loop_rate_hz=.1,update_interval_s=10.,persistent_settle_iterations=args.settle_iterations,
        ros_domain_id=198,timeout_s=1200.)
    cfg['dpgo']['pcm']['timeout_s']=180.
    cfg['dpgo']['registration_factors']['timeout_s']=180.
    from s3e_pipeline.circle_online import PublicationConfig
    from dataclasses import asdict
    publication=PublicationConfig(**cfg.get('circle_publication',{}))
    cfg['circle_publication']=asdict(publication)
    cfg['online']=dict(rate=1.,fresh_frontend=True,geometry_publication_policy=publication.mode,
        retrieval_transport='independent robot-to-robot DDS',resource_quotas=False,
        common_clock='unix wall descriptor availability',requested_duration_s=args.duration)
    work=args.output;work.mkdir(parents=True,exist_ok=False)
    for name in ('configs','frontends','live'):(work/name).mkdir()
    config=work/'config.yaml';config.write_text(yaml.safe_dump(cfg,sort_keys=False))
    atomic_json(work/'configuration.json',cfg)
    inputs={r:read_json(args.base/'inputs.json')[r] for r in robots};atomic_json(work/'inputs.json',inputs)
    for r in robots:
        original=args.base/f'configs/{r}.yaml';(work/f'configs/{r}.yaml').write_bytes(original.read_bytes())
        params=yaml.safe_load(original.read_text())['/**']['ros__parameters']
        assert not params['mapping']['submaps']['enabled'] and not params['mapping']['area_maps']['odometry']
        assert params['input']['reliable']
    files=[p for folder in (PROJECT/'research/s3e_pipeline',PROJECT/'scripts')
        for p in folder.rglob('*') if p.is_file() and '__pycache__' not in str(p)]
    atomic_json(work/'source-hashes.json',{str(p):file_hash(p) for p in files})
    runtime=Path('/workspace/.ros2');python=runtime/'research-venv/bin/python'
    launcher=runtime/'upstream-area-s3e-20260921/capacity-run/launcher/ellipselio_mapping_mt'
    library=runtime/'upstream-area-s3e-20260921/capacity-run/install/ellipselio/lib/libellipselio_mapping.so'
    atomic_json(work/'frontend-binary-hashes.json',{str(p):file_hash(p) for p in (launcher,library)})
    processes={};logs=[];started=time.perf_counter();cpu=time.process_time();frontend_exits={}
    gpu_log=(work/'gpu.csv').open('w')
    gpu_observer=subprocess.Popen(['nvidia-smi','--query-gpu=timestamp,memory.used,utilization.gpu',
        '--format=csv,noheader,nounits','-lms','100'],stdout=gpu_log,stderr=subprocess.DEVNULL)
    children=resource.getrusage(resource.RUSAGE_CHILDREN)
    tails={r:JsonlTail(work/f'frontends/{r}/frontend/native_updates.jsonl') for r in robots}
    native_state={r:dict(count=0,previous=None,last_success=None,max_speed=0.,max_gap=0.) for r in robots}
    def launch(name,command,domain=197):
        log=(work/f'{name}.log').open('w');logs.append(log)
        p=subprocess.Popen(list(map(str,command)),env=dict(os.environ,ROS_DOMAIN_ID=str(domain)),
            stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        processes[name]=p
        atomic_json(work/'processes.json',{k:v.pid for k,v in processes.items()})
    def check():
        for name,p in processes.items():
            if p.poll() is not None and p.returncode:raise RuntimeError(f'{name} exited {p.returncode}; see its log')
            if name.endswith('-frontend') and p.poll() is not None and name not in frontend_exits:
                frontend_exits[name]=dict(observed_exit_wall_ns=time.time_ns(),returncode=p.returncode)
                atomic_json(work/'frontend-exits.json',frontend_exits)
        for r in robots:
            state=native_state[r]
            for row in tails[r].read():
                stamp=row['sensor_stamp_ns'];pose=np.asarray(row['pose'],float)
                if not np.isfinite(pose).all():raise ValueError(f'{r}: nonfinite live odometry')
                prev=state['previous']
                if prev:
                    if stamp<=prev[0]:raise ValueError(f'{r}: nonchronological live odometry')
                    speed=np.linalg.norm(pose[:3]-prev[1][:3])/((stamp-prev[0])/1e9)
                    state['max_speed']=max(state['max_speed'],float(speed))
                    if speed>20:raise ValueError(f'{r}: live odometry diverged, speed {speed:.3f} m/s')
                if state['last_success'] is not None:
                    gap=(stamp-state['last_success'])/1e9;state['max_gap']=max(state['max_gap'],gap)
                    if gap>=1:raise ValueError(f'{r}: successful LiDAR update gap {gap:.3f} s')
                if row['lidar_updated']:state['last_success']=stamp
                state['previous']=(stamp,pose);state['count']+=1
    def wait(paths,timeout):
        end=time.monotonic()+timeout
        while not all(p.exists() for p in paths):
            check()
            if time.monotonic()>end:raise TimeoutError('Startup barrier: '+str([str(p) for p in paths if not p.exists()]))
            time.sleep(.1)
    try:
        with MemorySampler() as memory:
            for r in robots:
                launch(r+'-prepare',[python,'-m','s3e_pipeline.circle_online','--source',work/f'frontends/{r}/frontend/area_maps',
                    '--output',work/f'prepared-{r}','--config',config])
            wait([work/f'prepared-{r}/READY' for r in robots],120)
            for r in robots:
                local=work/f'live/{r}';local.mkdir()
                atomic_json(local/'spec.json',dict(robot=r,robots=robots,output=str(local),
                    store=str(work/f'prepared-{r}/store'),descriptors=str(work/f'prepared-{r}/ellipsoid'),
                    backend=cfg['backend'],loops=cfg['loops'],ros_underlay=os.environ['CBS_UNDERLAY'],
                    ros_overlay=os.environ['CBS_OVERLAY']))
                launch(r+'-live',[python,'-m','s3e_pipeline.online_worker','--spec',local/'spec.json'])
            launch('epochs',[python,'-m','s3e_pipeline.online_epochs','--work',work,'--source',SOURCE,'--config',config])
            wait([work/f'live/{r}/READY' for r in robots]+[work/'epochs/READY'],180)
            for i,r in enumerate(robots):
                command=['/usr/bin/python3',PROJECT/'scripts/recent_submaps/run_trial.py',r,'--robot',r,
                    '--area-maps','--persistent-odometry','--bag',inputs[r]['path'],
                    '--mapping-config',work/f'configs/{r}.yaml','--output-root',work/'frontends',
                    '--mapper-executable',launcher,'--mapping-library',library,'--start-barrier',work/'START']
                if args.duration:command+=['--duration',str(args.duration)]
                launch(r+'-frontend',command,110+2*i)
            wait([work/f'frontends/{r}/frontend/READY' for r in robots],180)
            atomic_json(work/'START',dict(wall_ns=time.time_ns(),robots=robots,rate=1.,duration=args.duration))
            begin=time.monotonic();last=0;limit=max(args.duration or x['sensor_duration_s'] for x in inputs.values())+1800
            while not (work/'epochs/DONE').exists():
                check()
                if time.monotonic()-begin>limit:raise TimeoutError('Live pipeline failed to drain')
                if time.monotonic()-last>5:
                    last=time.monotonic()
                    status=dict(phase='online',wall_s=last-begin,robots={r:read_json(work/f'live/{r}/progress.json')
                        for r in robots if (work/f'live/{r}/progress.json').exists()},
                        active_frontends=[r for r in robots if processes[r+'-frontend'].poll() is None],
                        native={r:{k:v for k,v in s.items() if k not in ('previous','last_success')} for r,s in native_state.items()})
                    atomic_json(work/'status.json',status);print(json.dumps(status),flush=True)
                time.sleep(.2)
            for r in robots:
                processes[r+'-frontend'].wait(180);processes[r+'-prepare'].wait(30)
                atomic_json(work/f'live/{r}/STOP',dict(wall_ns=time.time_ns()))
            for r in robots:processes[r+'-live'].wait(30)
            processes['epochs'].wait(30);check()
            atomic_json(work/'native-health.json',{r:{k:v for k,v in s.items()
                if k not in ('previous','last_success')} for r,s in native_state.items()})
            atomic_json(work/'status.json',dict(phase='capture-and-backend-complete',wall_s=time.monotonic()-begin,robots=robots,
                requested_duration_s=args.duration,finished_wall_ns=time.time_ns()))
            if {str(p):file_hash(p) for p in files}!=read_json(work/'source-hashes.json'):
                raise ValueError('Source changed during live run')
            atomic_json(work/'ONLINE_DONE',dict(success=True,wall_ns=time.time_ns(),robots=robots))
    except BaseException:
        atomic_json(work/'status.json',dict(phase='failed',traceback=traceback.format_exc()))
        raise
    finally:
        # Stop only this experiment's descendants, including detached mapper groups.
        descendants=[]
        for p in processes.values():
            try:descendants.extend(psutil.Process(p.pid).children(recursive=True))
            except psutil.NoSuchProcess:pass
        for child in reversed(descendants):
            try:child.send_signal(signal.SIGINT)
            except psutil.NoSuchProcess:pass
        for p in reversed(list(processes.values())):
            if p.poll() is None:p.send_signal(signal.SIGINT)
        for p in processes.values():
            try:p.wait(30)
            except subprocess.TimeoutExpired:p.kill();p.wait()
        _,alive=psutil.wait_procs(descendants,timeout=5)
        for child in alive:
            try:child.kill()
            except psutil.NoSuchProcess:pass
        for log in logs:log.close()
        gpu_observer.terminate();gpu_observer.wait(10);gpu_log.close()
        end=resource.getrusage(resource.RUSAGE_CHILDREN)
        atomic_json(work/'resources.json',dict(wall_s=time.perf_counter()-started,parent_cpu_s=time.process_time()-cpu,
            child_cpu_s=end.ru_utime+end.ru_stime-children.ru_utime-children.ru_stime,
            peak_simultaneous_process_tree_rss_bytes=memory.peak,
            scope='fresh simultaneous native odometry, per-robot recording, circle preparation, live DDS and persistent CBS',
            resource_quotas=False))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--config',type=Path,required=True)
    p.add_argument('--base',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--robots',nargs='+');p.add_argument('--duration',type=float,default=0.)
    p.add_argument('--settle-iterations',type=int,default=30)
    run(p.parse_args())
