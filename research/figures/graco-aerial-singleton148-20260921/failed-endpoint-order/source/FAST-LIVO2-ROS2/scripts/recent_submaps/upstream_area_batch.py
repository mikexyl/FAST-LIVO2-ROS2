#!/usr/bin/env python3
"""Fresh S3E updated-upstream persistent odometry → area BEV → MapClosures → PCM/CBS batch."""
import argparse
import copy
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import traceback
import threading
from queue import Queue
from concurrent.futures import ThreadPoolExecutor, as_completed
import numpy as np
import yaml

ROOT=Path('/workspace')
SCRIPTS=ROOT/'FAST-LIVO2-ROS2/scripts/recent_submaps'
PYTHON=ROOT/'.ros2/research-venv/bin/python'
RERUN=ROOT/'.ros2/rerun-venv/bin'
PREVIOUS=ROOT/'.ros2/recent-submaps/failed-frontends-20260918'
BASE=ROOT/'.ros2/upstream-area-s3e-20260921'
LIBRARY=BASE/'install/ellipselio/lib/libellipselio_mapping.so'
LAUNCHER=BASE/'launcher/ellipselio_mapping_mt'
STOP=threading.Event()
sys.path.insert(0,str(ROOT/'FAST-LIVO2-ROS2/research'))


def read(path):return json.loads(Path(path).read_text())


def save(path,value):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    temp=path.with_suffix('.tmp');temp.write_text(json.dumps(value,indent=2)+'\n');temp.replace(path)


def sha(path):
    h=hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda:stream.read(8*1024*1024),b''):h.update(block)
    return h.hexdigest()


def sources():
    import s3e_mapclosures_native, s3e_mapclosures_inspection
    paths=[LIBRARY,LAUNCHER,
           ROOT/'.ros2/dpgo-install/cbs/lib/libcbs.so',ROOT/'.ros2/dpgo-install/cbs_ros/lib/cbs_ros/cbs_ros_node',
           ROOT/'FAST-LIVO2-ROS2/scripts/run_ellipselio.py',ROOT/'FAST-LIVO2-ROS2/scripts/ellipselio_live_rerun.py',
           ROOT/'ellipselio/CMakeLists.txt',Path(s3e_mapclosures_native.__file__),Path(s3e_mapclosures_inspection.__file__),
           ROOT/'.ros2/ellipsoid-cuda/libellipsoid_surface.so']
    for folder in (SCRIPTS,ROOT/'FAST-LIVO2-ROS2/research/s3e_pipeline',ROOT/'ellipselio/src',ROOT/'ellipselio/include',ROOT/'ellipselio/msg'):
        paths += [p for p in folder.rglob('*') if p.suffix in ('.py','.cpp','.h','.hpp','.yaml','.msg','.sh') and p.is_file()]
    return {str(p.relative_to(ROOT)) if p.is_relative_to(ROOT) else str(p):sha(p) for p in sorted(set(paths))}


def prepare(work):
    import cv2
    work.mkdir(parents=True,exist_ok=False)
    template=yaml.safe_load((SCRIPTS/'area_rollout.yaml').read_text())
    groups=[]
    for name,version in [('S3E_Laboratory_4','S3Ev1'),('S3E_Campus_Road_1','S3Ev1'),
                         ('S3E_Campus_Road_2','S3Ev2'),('S3E_Campus_Road_3','S3Ev2')]:
        folder=work/name;folder.mkdir();(folder/'configs').mkdir()
        robots=['Alpha','Bob','Carol'];dataset=Path('/data/s3e')/version/name
        cfg=copy.deepcopy(template);cfg.update(robots=robots,dataset=str(dataset),experiment_name=name,output_root=str(folder))
        cfg['dpgo']['ros_domain_id']=211
        cfg['odometry']['native_sensor_qos']=False
        (folder/'config.yaml').write_text(yaml.safe_dump(cfg,sort_keys=False))
        group=dict(name=name,robots=robots,folder=str(folder),dataset=str(dataset),graco=False,trials={})
        for robot in robots:
            config=yaml.safe_load((SCRIPTS/'baseline.yaml').read_text())
            params=config['/**']['ros__parameters'];params['mapping']['namespace']=robot
            calibration=dataset.parent/'Calibration'/f'{robot.lower()}.yaml'
            fs=cv2.FileStorage(str(calibration),cv2.FILE_STORAGE_READ)
            if not fs.isOpened():raise ValueError(f'Missing calibration {calibration}')
            T=fs.getNode('Tic').mat()@np.linalg.inv(fs.getNode('Tlc').mat())
            if not np.isfinite(T).all() or not np.allclose(T[:3,:3].T@T[:3,:3],np.eye(3),atol=1e-5):raise ValueError('Invalid calibration')
            params['lidar'].update(topic=f'/{robot}/velodyne_points',t_imu_lidar=T[:3,3].tolist(),r_imu_lidar=T[:3,:3].ravel().tolist())
            params['imu'].update(topic=f'/{robot}/imu/data',rate=int(fs.getNode('IMU.Frequency').real()))
            fs.release()
            params.pop('research',None);params.pop('use_sim_time',None)
            params['mapping']['submaps']=dict(enabled=False)
            params['mapping']['area_maps']=dict(yaml.safe_load((SCRIPTS/'area_maps.yaml').read_text()),odometry=False)
            params['publish']=dict(map=True,scan=True,markers=True,odometry=True,analytics=True,tf=True)
            params['input']=dict(reliable=True)
            config_path=folder/'configs'/f'{robot}.yaml';config_path.write_text(yaml.safe_dump(config,sort_keys=False))
            metadata=yaml.safe_load((dataset/'metadata.yaml').read_text())['rosbag2_bagfile_information']
            group['trials'][robot]=dict(path=str(folder/'frontends'/robot),reused=False,bag=str(dataset),config=str(config_path),
                duration_s=metadata['duration']['nanoseconds']/1e9,config_sha256=sha(config_path),calibration_sha256=sha(calibration))
        save(folder/'trials.json',{robot:row['path'] for robot,row in group['trials'].items()});groups.append(group)
    save(work/'plan.json',groups);save(work/'source-hashes.json',sources())
    return groups


def quality(trial):
    summary=read(trial/'frontend/summary.json')
    rows=[json.loads(line) for line in (trial/'frontend/native_updates.jsonl').read_text().splitlines()]
    if len(rows)<2:raise ValueError('Too few native poses')
    stamps=np.array([r['stamp_ns'] for r in rows],dtype=np.int64)
    sensor=np.array([r['sensor_stamp_ns'] for r in rows],dtype=np.int64)
    poses=np.array([r['pose'] for r in rows]);success=sensor[[r['lidar_updated'] for r in rows]]
    chronological=bool(np.all(np.diff(stamps)>0) and np.all(np.diff(sensor)>0))
    speed=float(np.max(np.linalg.norm(np.diff(poses[:,:3],axis=0),axis=1)/(np.diff(stamps)/1e9)))
    gap=float(np.max(np.diff(np.r_[sensor[0],success,sensor[-1]]))/1e9)
    result=dict(full_completion=bool(summary['success'] and summary['requested_duration']==0),
                finite=bool(np.isfinite(poses).all()),chronological=chronological,
                max_speed_m_s=speed,max_successful_update_gap_s=gap,native_poses=len(rows),ground_truth_used=False)
    result['passed']=bool(result['full_completion'] and result['finite'] and chronological and speed<=20 and gap<1)
    return result


def call(command,log,timeout,env=None):
    import psutil
    with Path(log).open('w') as stream:
        process=subprocess.Popen(list(map(str,command)),cwd=ROOT,env=env,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
        try:
            deadline=time.monotonic()+timeout
            while process.poll() is None:
                if STOP.is_set():raise InterruptedError('Batch stop requested')
                if time.monotonic()>deadline:raise subprocess.TimeoutExpired(command,timeout)
                try:process.wait(timeout=1)
                except subprocess.TimeoutExpired:pass
            code=process.returncode
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


def run(work,workers=6):
    import fcntl
    lock=(work/'controller.lock').open('w');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    if (work/'status.json').exists():raise ValueError('Refusing to rerun an existing batch')
    frozen=read(work/'source-hashes.json');groups=read(work/'plan.json')
    if sources()!=frozen:raise ValueError('Frozen source or binaries changed')
    mutex=threading.RLock();domains=Queue()
    if workers<1 or 90+2*(workers-1)>230:raise ValueError('Worker count exceeds available distinct ROS domains')
    frontend_domains=[90+2*i for i in range(workers)]
    for domain in frontend_domains:domains.put(domain)
    state=dict(pid=os.getpid(),started_utc=time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime()),
               frontend_workers=workers,frontend_ros_domains=frontend_domains,phase='frontends',groups=[])
    def persist():
        state['updated_utc']=time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime());save(work/'status.json',state)
    def interrupted():
        if STOP.is_set():
            with mutex:state['phase']='interrupted';persist()
            raise KeyboardInterrupt('Batch stop requested')
    for sig in (signal.SIGINT,signal.SIGTERM):signal.signal(sig,lambda *_:STOP.set())
    tasks=[]
    for group in groups:
        row=dict(name=group['name'],status='running',stage='frontends',frontends={});state['groups'].append(row)
        for robot,item in group['trials'].items():
            row['frontends'][robot]=dict(status='queued',reused=item['reused'],path=item['path'])
            tasks.append((group,row,robot,item))
    persist()
    def frontend(task):
        group,row,robot,item=task;folder=Path(group['folder']);trial=Path(item['path']);domain=domains.get()
        env=dict(os.environ,ROS_DOMAIN_ID=str(domain));current=row['frontends'][robot]
        def mark(status,**values):
            with mutex:current.update(status=status,ros_domain_id=domain,**values);persist()
        try:
            if STOP.is_set():raise InterruptedError('Batch stop requested')
            mark('checking_reuse' if item['reused'] else 'recording')
            if not item['reused']:
                (folder/'frontends').mkdir(exist_ok=True)
                call(['/usr/bin/python3',SCRIPTS/'run_trial.py',robot,'--area-maps','--persistent-odometry','--robot',robot,
                      '--mapper-executable',LAUNCHER,'--mapping-library',LIBRARY,
                      '--bag',item['bag'],'--mapping-config',item['config'],'--output-root',folder/'frontends'],
                     folder/f'{robot}-frontend.log',item['duration_s']+240,env)
                mark('validating')
                call([PYTHON,SCRIPTS/'validate.py',trial],folder/f'{robot}-validation.log',300,env)
                call(['/usr/bin/python3',SCRIPTS/'sensor_coverage.py',trial],folder/f'{robot}-coverage.log',120,env)
                call([RERUN/'rerun','rrd','verify',trial/'recording/live.rrd'],folder/f'{robot}-recording-verification.log',300,env)
            result=quality(trial);save(folder/f'{robot}-quality.json',result)
            if not result['passed']:raise ValueError(f'{robot} failed sensor-only frontend stability: {result}')
            if not read(trial/'sensor-coverage.json')['full_selected_sensor_tail_reached']:raise ValueError('Incomplete source sensor tail')
            mark('complete',quality=result)
        except Exception as exc:
            save(folder/f'{robot}-frontend-failure.json',dict(error=repr(exc),traceback=traceback.format_exc()))
            mark('interrupted' if STOP.is_set() else 'failed',error=repr(exc))
        finally:domains.put(domain)
    # Validate cached captures first, then start the longest new replays first.
    tasks.sort(key=lambda t:(not t[3]['reused'],-t[3]['duration_s'],t[0]['name'],t[2]))
    with ThreadPoolExecutor(max_workers=workers) as pool:
        for future in as_completed([pool.submit(frontend,task) for task in tasks]):future.result()
    interrupted();state['phase']='backends';persist()
    for group,row in zip(groups,state['groups']):
        interrupted();folder=Path(group['folder'])
        def update(stage,**values):
            row.update(stage=stage,**values);persist()
        try:
            if sources()!=frozen:raise ValueError('Frozen source or binaries changed')
            if any(r['status']!='complete' for r in row['frontends'].values()):raise ValueError('One or more frontends failed; see per-robot results')
            update('descriptors')
            cfg=yaml.safe_load((folder/'config.yaml').read_text());artifacts={}
            for robot,item in group['trials'].items():
                update('descriptors',robot=robot)
                call([PYTHON,'-m','s3e_pipeline.recent_submaps','--source',Path(item['path'])/'frontend/area_maps',
                      '--output',folder/f'prepared-{robot}','--config',folder/'config.yaml'],folder/f'{robot}-descriptors.log',1800)
                artifacts[f'keyframes.ellipselio.{robot}']=str(folder/f'prepared-{robot}')
                artifacts[f'descriptors.ellipselio.mapclosures.{robot}']=str(folder/f'prepared-{robot}/ellipsoid')
            save(folder/'artifacts.json',artifacts);update('distributed_pcm_cbs')
            call([PYTHON,Path(__file__),'backend','--work',folder],folder/'backend.log',cfg['dpgo']['timeout_s']+120)
            update('evaluation')
            call([PYTHON,SCRIPTS/'rollout_report.py','--work',folder,'--trials',folder/'trials.json'],folder/'evaluation.log',3600)
            call([RERUN/'python',SCRIPTS/'rollout_rerun.py',folder],folder/'rerun.log',300)
            call([RERUN/'rerun','rrd','verify',folder/'report/result.rrd'],folder/'recording-verification.log',300)
            report=read(folder/'report/report.json')
            update('complete',status='complete',components=report['components'],connectivity=report['connectivity'],
                   raw_ate={r:x['rmse_m'] for r,x in report['raw'].items()},
                   cbs_component_ate={c:x['rmse_m'] for c,x in report['cbs'].items()},
                   loops=report['runtime']['loops'],pcm_rejected=report['pcm']['excluded_loops'])
        except Exception as exc:
            save(folder/'failure.json',dict(stage=row['stage'],error=repr(exc),traceback=traceback.format_exc()))
            update('failed',status='failed',failed_stage=row['stage'],error=repr(exc))
    interrupted();state['phase']='finished';state['finished_utc']=time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime());persist()


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('stage',choices=['prepare','run','backend']);parser.add_argument('--work',type=Path,required=True)
    parser.add_argument('--workers',type=int,default=6)
    args=parser.parse_args()
    if args.stage=='prepare':print(json.dumps(prepare(args.work),indent=2))
    elif args.stage=='run':run(args.work,args.workers)
    else:
        from s3e_pipeline.dpgo import run as backend
        cfg=yaml.safe_load((args.work/'config.yaml').read_text());artifacts=read(args.work/'artifacts.json')
        (args.work/'dpgo').mkdir(exist_ok=False);backend(cfg,artifacts,ROOT,args.work/'dpgo')
        artifacts['dpgo']=str(args.work/'dpgo');save(args.work/'artifacts.json',artifacts)
