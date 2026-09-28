#!/usr/bin/env python3
"""Fresh concurrent online GRACO capture, streaming retrieval and periodic CBS."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import traceback
import yaml

ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'FAST-LIVO2-ROS2/research'))
from s3e_pipeline.artifacts import file_hash, read_json
from s3e_pipeline.online_io import atomic_json


def main():
    p=argparse.ArgumentParser(); p.add_argument('--work',type=Path,required=True)
    p.add_argument('--base',type=Path,required=True,help='Existing sensor paths and calibrated configs only')
    p.add_argument('--robots',nargs='+'); p.add_argument('--duration',type=float,default=0.)
    a=p.parse_args(); work=a.work; work.mkdir(exist_ok=True)
    robots=a.robots or [f'aerial{i:02d}' for i in range(1,9)]+[f'ground{i:02d}' for i in range(1,7)]
    cfg=yaml.safe_load((a.base/'config.yaml').read_text()); cfg.update(robots=robots,dataset=None,output_root=str(work),
        experiment_name='Fresh online GRACO demo: concurrent sensors, streaming descriptors, periodic distributed PCM/CBS')
    cfg['evaluation'].pop('expected_connected_robots',None)
    cfg['dpgo']['retain_registration_transport']=False
    cfg['dpgo'].setdefault('update_interval_s',10.)
    cfg['dpgo']['pcm']['timeout_s']=180.
    cfg['dpgo']['registration_factors']['timeout_s']=180.
    cfg['online']=dict(common_clock='wall descriptor availability',backend='periodic immutable causal prefixes',
        resource_quotas=False,rate=1.,robots=len(robots),preparation_concurrency=3,
        cbs_input_interval_s=cfg['dpgo']['update_interval_s'],cbs_overrun_policy='queue every changed causal prefix')
    config=work/'config.yaml'; config.write_text(yaml.safe_dump(cfg,sort_keys=False))
    inputs={r:read_json(a.base/'inputs.json')[r] for r in robots}; atomic_json(work/'inputs.json',inputs)
    for name in ('configs','frontends','live','demo','slots'): (work/name).mkdir(exist_ok=True)
    if not (work/'demo/video.rgb').exists(): raise ValueError('Start host FFmpeg with demo/video.rgb FIFO first')
    for r in robots:
        (work/f'configs/{r}.yaml').write_bytes((a.base/f'configs/{r}.yaml').read_bytes())
        config_r=yaml.safe_load((work/f'configs/{r}.yaml').read_text())['/**']['ros__parameters']
        assert not config_r['mapping']['submaps']['enabled'] and not config_r['mapping']['area_maps']['odometry']
        assert config_r['input']['reliable']
    source_files=[q for folder in (ROOT/'FAST-LIVO2-ROS2/research/s3e_pipeline',ROOT/'FAST-LIVO2-ROS2/scripts/recent_submaps')
        for q in folder.rglob('*') if q.is_file() and '__pycache__' not in str(q)]
    source_files += [ROOT/'FAST-LIVO2-ROS2/scripts/run_ellipselio.py',ROOT/'FAST-LIVO2-ROS2/scripts/ellipselio_live_rerun.py']
    atomic_json(work/'source-hashes.json',{str(q):file_hash(q) for q in source_files})
    python=ROOT/'.ros2/research-venv/bin/python'; rrpython=ROOT/'.ros2/rerun-venv/bin/python'
    launcher=ROOT/'.ros2/upstream-area-s3e-20260921/capacity-run/launcher/ellipselio_mapping_mt'
    library=ROOT/'.ros2/upstream-area-s3e-20260921/capacity-run/install/ellipselio/lib/libellipselio_mapping.so'
    atomic_json(work/'binary-hashes.json',{str(q):file_hash(q) for q in [launcher,library,
        ROOT/'.ros2/dpgo-install/cbs/lib/libcbs.so',ROOT/'.ros2/dpgo-install/cbs_ros/lib/cbs_ros/cbs_ros_node']})
    processes={}; logs=[]
    def launch(name,args,domain=180):
        log=(work/f'{name}.log').open('w'); logs.append(log)
        process=subprocess.Popen(list(map(str,args)),env=dict(os.environ,ROS_DOMAIN_ID=str(domain)),
            stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        processes[name]=process; return process
    def check():
        for name,process in processes.items():
            if process.poll() is not None and process.returncode:
                raise RuntimeError(f'{name} failed ({process.returncode}); see {work/name}.log')
    def wait_files(paths,timeout):
        deadline=time.monotonic()+timeout
        while not all(path.exists() for path in paths):
            check()
            if time.monotonic()>deadline: raise TimeoutError('Startup barrier: '+str([str(p) for p in paths if not p.exists()]))
            time.sleep(.1)
    try:
        for r in robots:
            prepared=work/f'prepared-{r}'; local=work/f'live/{r}'; local.mkdir()
            launch(f'{r}-prepare',[python,'-m','s3e_pipeline.online_prepare','--source',work/f'frontends/{r}/frontend/area_maps',
                '--output',prepared,'--config',config,'--slots',work/'slots','--concurrency','3'])
        wait_files([work/f'prepared-{r}/READY' for r in robots],120)
        for r in robots:
            prepared=work/f'prepared-{r}'; local=work/f'live/{r}'
            spec=dict(robot=r,robots=robots,output=str(local),store=str(prepared/'store'),descriptors=str(prepared/'ellipsoid'),
                backend=cfg['backend'],loops=cfg['loops'],ros_underlay=str(ROOT/'.ros2/cbs-underlay'),
                ros_overlay=str(ROOT/'.ros2/dpgo-install'))
            atomic_json(local/'spec.json',spec)
            launch(f'{r}-live',[python,'-m','s3e_pipeline.online_worker','--spec',local/'spec.json'])
        launch('epochs',[python,'-m','s3e_pipeline.online_epochs','--work',work,'--config',config])
        launch('recorder',[rrpython,'-m','s3e_pipeline.online_record','--work',work,'--config',config])
        wait_files([work/f'live/{r}/READY' for r in robots]+[work/'epochs/READY',work/'demo/READY'],120)
        for i,r in enumerate(robots):
            args=['/usr/bin/python3',ROOT/'FAST-LIVO2-ROS2/scripts/recent_submaps/run_trial.py',r,'--robot',r,
                '--area-maps','--persistent-odometry','--bag',inputs[r]['path'],'--mapping-config',work/f'configs/{r}.yaml',
                '--output-root',work/'frontends','--mapper-executable',launcher,'--mapping-library',library,
                '--start-barrier',work/'START']
            if a.duration: args+=['--duration',str(a.duration)]
            launch(f'{r}-frontend',args,100+2*i)
        wait_files([work/f'frontends/{r}/frontend/READY' for r in robots],180)
        atomic_json(work/'START',dict(wall_ns=time.time_ns(),robots=robots,rate=1.,duration=a.duration))
        start=time.monotonic(); last=0; deadline=start+max(a.duration or x['sensor_duration_s'] for x in inputs.values())+1800
        while not (work/'epochs/DONE').exists():
            check()
            if time.monotonic()>deadline: raise TimeoutError('Online pipeline did not drain in time')
            if time.monotonic()-last>5:
                last=time.monotonic()
                status=dict(phase='online',wall_s=last-start,robots={r:read_json(work/f'live/{r}/progress.json')
                    for r in robots if (work/f'live/{r}/progress.json').exists()},
                    running_frontends=[r for r in robots if processes[f'{r}-frontend'].poll() is None])
                atomic_json(work/'status.json',status); print(json.dumps(status),flush=True)
            time.sleep(.2)
        for r in robots:
            processes[f'{r}-frontend'].wait(timeout=180)
            processes[f'{r}-prepare'].wait(timeout=30)
            atomic_json(work/f'live/{r}/STOP',dict(wall_ns=time.time_ns()))
        time.sleep(3)
        atomic_json(work/'STOP_RECORDING',dict(wall_ns=time.time_ns()))
        processes['recorder'].wait(timeout=90); check()
        if {str(q):file_hash(q) for q in source_files}!=read_json(work/'source-hashes.json'):
            raise ValueError('Source changed during run')
        atomic_json(work/'status.json',dict(phase='complete',wall_s=time.monotonic()-start,robots=robots,
            finished_wall_ns=time.time_ns(),demo=read_json(work/'demo/summary.json')))
    except BaseException as exc:
        atomic_json(work/'status.json',dict(phase='failed',error=repr(exc),traceback=traceback.format_exc()))
        if (work/'START').exists(): atomic_json(work/'STOP_RECORDING',dict(wall_ns=time.time_ns()))
        raise
    finally:
        for name,process in reversed(list(processes.items())):
            if process.poll() is None: os.killpg(process.pid,signal.SIGINT)
        for process in processes.values():
            try: process.wait(timeout=90)
            except subprocess.TimeoutExpired: os.killpg(process.pid,signal.SIGKILL); process.wait()
        for log in logs: log.close()


if __name__=='__main__': main()
