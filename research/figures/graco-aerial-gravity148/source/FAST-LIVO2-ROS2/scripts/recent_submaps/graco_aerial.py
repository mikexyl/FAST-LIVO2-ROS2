#!/usr/bin/env python3
"""Local two-flight GRACO temporal EllipseLIO / ellipsoid-BEV / PCM / CBS trial."""
import argparse
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time
import traceback
import yaml

ROOT=Path(__file__).resolve().parents[3]
SCRIPTS=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/'FAST-LIVO2-ROS2/research'))
from full_batch import quality
from s3e_pipeline.graco import stage_sensors, mapping_config

PYTHON=ROOT/'.ros2/research-venv/bin/python'
RERUN=ROOT/'.ros2/rerun-venv/bin'
LIBRARY=ROOT/'.ros2/temporal-submaps-revert/install/ellipselio/lib/libellipselio_mapping.so'
LAUNCHER=ROOT/'.ros2/temporal-submaps-revert/launcher/ellipselio_mapping_mt'
CALIBRATION=Path('/data/graco/aerial-calibration-20251121T084428Z-1-001/aerial-calibration')
FLIGHTS={'aerial05':Path('/data/graco/aerial-05-40m'),
         'aerial08':Path('/data/graco/aerial-08-25m_ros2')}


def read(path):return json.loads(path.read_text())
def now():return datetime.now(timezone.utc).isoformat()
def sha(path):
    h=hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda:stream.read(8*1024*1024),b''):h.update(chunk)
    return h.hexdigest()
def save(path,value):
    temporary=path.with_suffix('.tmp');temporary.write_text(json.dumps(value,indent=2)+'\n');temporary.replace(path)
def call(command,log,timeout,env=None):
    import psutil
    with log.open('x') as stream:
        process=subprocess.Popen(list(map(str,command)),cwd=ROOT,env=env,stdout=stream,
                                 stderr=subprocess.STDOUT,start_new_session=True)
        try:
            code=process.wait(timeout=timeout)
        except BaseException:
            try:children=psutil.Process(process.pid).children(recursive=True)
            except psutil.NoSuchProcess:children=[]
            process.send_signal(signal.SIGINT)
            try:process.wait(timeout=60)
            except subprocess.TimeoutExpired:pass
            for child in reversed(children):
                try:child.terminate()
                except psutil.NoSuchProcess:pass
            _,alive=psutil.wait_procs(children,timeout=5)
            for child in alive:
                try:child.kill()
                except psutil.NoSuchProcess:pass
            if process.poll() is None:process.kill()
            process.wait();raise
    if code:raise RuntimeError(f'Exit {code}: {log}')


def run(work):
    work=work.resolve();work.mkdir(parents=True,exist_ok=False)
    state=dict(started_utc=now(),host=os.uname().nodename,phase='preflight',robots={r:dict(status='queued',bag=str(b)) for r,b in FLIGHTS.items()})
    lock=threading.Lock()
    def mark(phase=None,robot=None,**fields):
        with lock:
            if phase:state['phase']=phase
            if robot:state['robots'][robot].update(fields)
            else:state.update(fields)
            state['updated_utc']=now();save(work/'status.json',state)
    mark()
    try:
        import s3e_mapclosures_native
        import numpy as np
        from s3e_pipeline.ellipsoid_cuda import SurfaceSampler
        sampler=SurfaceSampler()
        try:sampler.render(np.array([[0.,0.,5.]]),np.array([[.2,.2,.2]]),np.eye(3)[None])
        finally:sampler.close()
        watched={p.resolve() for directory in (ROOT/'ellipselio/src',ROOT/'ellipselio/include',ROOT/'ellipselio/msg',SCRIPTS,ROOT/'FAST-LIVO2-ROS2/research/s3e_pipeline')
                 for p in directory.rglob('*') if p.is_file() and p.suffix in ('.py','.cpp','.h','.hpp','.msg','.yaml','.sh')}
        watched.update([LIBRARY,LAUNCHER,ROOT/'ellipselio/CMakeLists.txt',ROOT/'FAST-LIVO2-ROS2/scripts/run_ellipselio.py',
            ROOT/'FAST-LIVO2-ROS2/scripts/ellipselio_live_rerun.py',ROOT/'.ros2/ellipsoid-cuda/libellipsoid_surface.so',
            Path(s3e_mapclosures_native.__file__),ROOT/'.ros2/dpgo-install/cbs/lib/libcbs.so',
            ROOT/'.ros2/dpgo-install/cbs_ros/lib/cbs_ros/cbs_ros_node'])
        frozen={str(p):sha(p) for p in sorted(watched)};save(work/'source-hashes.json',frozen)
        save(work/'calibration-hashes.json',{str(p):sha(p) for p in CALIBRATION.glob('*.yaml')})
        cfg=yaml.safe_load((SCRIPTS/'rollout.yaml').read_text())
        cfg.update(robots=list(FLIGHTS),dataset=str(work/'reference'),output_root=str(work),experiment_name='GRACO aerial-05-40m + aerial-08-25m')
        cfg['dpgo']['ros_domain_id']=116
        cfg['evaluation'].update(ground_truth_format='graco_imu_enu',expected_connected_robots=list(FLIGHTS),
            trajectory_limitation='GRACO RTK/INS T_Base_Imu positions in base-station ENU; GT orientations unused; original timestamps; rigid alignment without scale')
        (work/'config.yaml').write_text(yaml.safe_dump(cfg,sort_keys=False))
        (work/'configs').mkdir();(work/'inputs').mkdir();(work/'frontends').mkdir()
        trials={r:str(work/'frontends'/r) for r in FLIGHTS};save(work/'trials.json',trials)
        for robot in FLIGHTS:
            mapping=mapping_config(ROOT,CALIBRATION,robot,cfg['keyframes'])
            p=mapping['/**']['ros__parameters'];p.pop('research',None)
            p['mapping']['submaps']=dict(enabled=True,duration_s=10.,overlap_s=5.)
            p['publish']=dict(map=True,scan=True,markers=True,odometry=True,analytics=True,tf=True)
            (work/'configs'/f'{robot}.yaml').write_text(yaml.safe_dump(mapping,sort_keys=False))
        def frontend(item):
            index,(robot,bag)=item
            try:
                mark(robot=robot,status='staging_native_lidar_imu',ros_domain_id=110+2*index)
                sensors=work/'inputs'/robot
                stage_sensors(bag,sensors)
                metadata=yaml.safe_load((sensors/'metadata.yaml').read_text())['rosbag2_bagfile_information']
                duration=metadata['duration']['nanoseconds']/1e9
                mark(robot=robot,status='recording',sensor_duration_s=duration)
                env=dict(os.environ,ROS_DOMAIN_ID=str(110+2*index))
                call(['/usr/bin/python3',SCRIPTS/'run_trial.py',robot,'--robot',robot,'--enabled',
                    '--bag',sensors,'--mapping-config',work/'configs'/f'{robot}.yaml','--output-root',work/'frontends',
                    '--mapper-executable',LAUNCHER,'--mapping-library',LIBRARY],work/f'{robot}-frontend.log',duration+240,env)
                mark(robot=robot,status='validating')
                trial=Path(trials[robot])
                call([PYTHON,SCRIPTS/'validate.py',trial],work/f'{robot}-validation.log',300,env)
                call(['/usr/bin/python3',SCRIPTS/'sensor_coverage.py',trial],work/f'{robot}-coverage.log',120,env)
                call([RERUN/'rerun','rrd','verify',trial/'recording/live.rrd'],work/f'{robot}-recording-verification.log',300,env)
                result=quality(trial);save(work/f'{robot}-quality.json',result)
                if not result['passed']:raise ValueError(f'Frontend sensor-only quality failed: {result}')
                if not read(trial/'sensor-coverage.json')['full_selected_sensor_tail_reached']:raise ValueError('Incomplete sensor tail')
                mark(robot=robot,status='complete',quality=result)
            except Exception as error:
                save(work/f'{robot}-failure.json',dict(error=repr(error),traceback=traceback.format_exc()))
                mark(robot=robot,status='failed',error=repr(error))
        mark('frontends')
        with ThreadPoolExecutor(max_workers=2) as pool:list(pool.map(frontend,enumerate(FLIGHTS.items())))
        if any(v['status']!='complete' for v in state['robots'].values()):
            raise RuntimeError('One or more frontends failed; retained partial outputs and per-robot failure details')
        for path,expected in frozen.items():
            if sha(Path(path))!=expected:raise ValueError(f'Source changed during replay: {path}')
        artifacts={}
        for robot in FLIGHTS:
            mark('descriptors',active_robot=robot)
            output=work/f'prepared-{robot}'
            call([PYTHON,'-m','s3e_pipeline.recent_submaps','--source',Path(trials[robot])/'frontend/submaps',
                '--output',output,'--config',work/'config.yaml'],work/f'{robot}-descriptors.log',1800)
            artifacts[f'keyframes.ellipselio.{robot}']=str(output)
            artifacts[f'descriptors.ellipselio.mapclosures.{robot}']=str(output/'ellipsoid')
        save(work/'artifacts.json',artifacts)
        mark('distributed_pcm_cbs')
        call([PYTHON,Path(__file__),'backend','--work',work],work/'backend.log',cfg['dpgo']['timeout_s']+120)
        mark('evaluation')
        # Only evaluation receives GT files; timestamps and positions never influence mapping or loop admission.
        (work/'reference').mkdir()
        truth={'aerial05':Path('/data/graco/aerial-05-40m.txt'),'aerial08':Path('/data/graco/aerial-08-25m.txt')}
        for robot,path in truth.items():(work/'reference'/f'{robot}_gt.txt').symlink_to(path)
        save(work/'reference/source-hashes.json',{str(p):sha(p) for p in truth.values()})
        call([PYTHON,SCRIPTS/'rollout_report.py','--work',work,'--trials',work/'trials.json'],work/'evaluation.log',1800)
        call([RERUN/'python',SCRIPTS/'rollout_rerun.py',work],work/'rerun.log',300)
        call([RERUN/'rerun','rrd','verify',work/'report/result.rrd'],work/'recording-verification.log',300)
        report=read(work/'report/report.json')
        for path,expected in frozen.items():
            if sha(Path(path))!=expected:raise ValueError(f'Source changed during backend: {path}')
        mark('complete',finished_utc=now(),frozen_sources_verified=True,
             raw_ate={r:x['rmse_m'] for r,x in report['raw'].items()},
             shared_cbs_ate={r:x['rmse_m'] for r,x in report['cbs'].items()},
             connectivity=report['connectivity'],loops=report['runtime']['loops'],pcm_rejected=report['pcm']['excluded_loops'])
    except Exception as error:
        save(work/'failure.json',dict(stage=state['phase'],error=repr(error),traceback=traceback.format_exc()))
        mark('failed',finished_utc=now(),error=repr(error))
        raise


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('stage',choices=['run','backend']);parser.add_argument('--work',type=Path,required=True)
    args=parser.parse_args()
    if args.stage=='run':run(args.work)
    else:
        from s3e_pipeline.dpgo import run as backend
        cfg=yaml.safe_load((args.work/'config.yaml').read_text());artifacts=read(args.work/'artifacts.json')
        (args.work/'dpgo').mkdir(exist_ok=False);backend(cfg,artifacts,ROOT,args.work/'dpgo')
