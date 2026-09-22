#!/usr/bin/env python3
"""Persistent bounded sequence queue; no sweeps, no GT in estimation, no silent retries."""
import argparse
import fcntl
import gzip
import hashlib
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time
import traceback
import uuid
import yaml

ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'FAST-LIVO2-ROS2/research'))
sys.path.insert(0,str(ROOT/'Swarm-SLAM/s3e'))
from s3e_pipeline.artifacts import file_hash,read_json,read_jsonl,write_json
from frontend_quality import check_export
ROBOTS=('Alpha','Bob','Carol')
PYTHON=ROOT/'.ros2/research-venv/bin/python'
BASE=ROOT/'.ros2/ellipsoid-cbs148'
BASE.mkdir(parents=True,exist_ok=True)


def atomic(path,obj):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    temp=path.with_suffix('.tmp');temp.write_text(json.dumps(obj,indent=2)+'\n');temp.replace(path)


def utc():return time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime())


def terminate(proc):
    import psutil
    try:children=psutil.Process(proc.pid).children(recursive=True)
    except psutil.NoSuchProcess:children=[]
    # Let wrappers flush capture summaries before killing any surviving descendants.
    try:proc.send_signal(signal.SIGINT);proc.wait(timeout=20)
    except (ProcessLookupError,subprocess.TimeoutExpired):pass
    for child in reversed(children):
        try:child.terminate()
        except psutil.NoSuchProcess:pass
    _,alive=psutil.wait_procs(children,timeout=5)
    for child in alive:
        try:child.kill()
        except psutil.NoSuchProcess:pass
    if proc.poll() is None:proc.kill()
    proc.wait()


def call(cmd,log,timeout):
    print(utc(),'RUN',*map(str,cmd),flush=True)
    with Path(log).open('w') as stream:
        proc=subprocess.Popen(list(map(str,cmd)),cwd=ROOT,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
        try:code=proc.wait(timeout=timeout)
        except BaseException:
            terminate(proc);raise
    if code:raise RuntimeError(f'Command exited {code}; see {log}')


def discover(root):
    priority={'S3E_Square_1':0,'S3E_Square_2':1,'S3E_Library_1':2,'S3E_Laboratory_1':3,
              'S3E_Playground_2':4,'S3E_Campus_Road_1':20}
    bags=sorted(root.glob('S3Ev*/S3E_*/metadata.yaml'),key=lambda p:(priority.get(p.parent.name,10),str(p)))
    return [p.parent for p in bags if p.parent.name!='S3E_Playground_1']


def mapping(dataset,robot,cfg):
    import cv2
    import numpy as np
    mapping=yaml.safe_load((ROOT/f'.ros2/ellipse-configs/{robot}.yaml').read_text())
    p=mapping['/**']['ros__parameters'];cal=dataset.parent/'Calibration'/f'{robot.lower()}.yaml'
    fs=cv2.FileStorage(str(cal),cv2.FILE_STORAGE_READ)
    if not fs.isOpened():raise ValueError(f'Calibration unavailable: {cal}')
    T=fs.getNode('Tic').mat()@np.linalg.inv(fs.getNode('Tlc').mat())
    if not np.allclose(T[:3,:3].T@T[:3,:3],np.eye(3),atol=1e-5):raise ValueError('Invalid LiDAR/IMU calibration')
    p['lidar'].update(t_imu_lidar=T[:3,3].tolist(),r_imu_lidar=T[:3,:3].reshape(-1).tolist())
    p['imu'].update(rate=int(fs.getNode('IMU.Frequency').real()),acc_noise=fs.getNode('IMU.NoiseAcc').real(),
        gyr_noise=fs.getNode('IMU.NoiseGyro').real(),acc_bias=fs.getNode('IMU.AccWalk').real(),gyr_bias=fs.getNode('IMU.GyroWalk').real())
    fs.release()
    # Apply explicit experiment noise after loading the sensor calibration;
    # dataset noise must not silently overwrite a requested experiment setting.
    noise=cfg.get('odometry',{}).get('imu_noise')
    if noise is not None:
        if set(noise)!={'acc_noise','gyr_noise','acc_bias','gyr_bias'} or any(
            not np.isfinite(v) or v<0 for v in noise.values()):
            raise ValueError('Expected four finite nonnegative EllipseLIO IMU noise parameters')
        p['imu'].update(noise)
    if cfg.get('odometry',{}).get('native_sensor_qos',False):
        p.get('input',{}).pop('reliable',None)
        if not p.get('input'):p.pop('input',None)
    if cfg.get('odometry',{}).get('record_analytics',False):
        p['publish']['analytics']=True
    p['research']=dict(ellipsoid_keyframes=True,ellipsoid_range_m=80.,ellipsoid_world_frame=True,
        ellipsoid_delta_export=True,keyframe_translation_m=cfg['keyframes']['translation_m'],
        keyframe_rotation_deg=cfg['keyframes']['rotation_deg'],keyframe_interval_s=cfg['keyframes']['max_interval_s'])
    return mapping


def source_hashes():
    result={}
    for relative in ('FAST-LIVO2-ROS2/research/s3e_pipeline','FAST-LIVO2-ROS2/research/adapters',
                     'FAST-LIVO2-ROS2/scripts/ellipsoid_cbs148','ellipselio/src','ellipselio/include',
                     'cbs/src','cbs/include','cbs_ros/src','cbs_ros/include'):
        for p in sorted((ROOT/relative).rglob('*')):
            if p.is_file() and '__pycache__' not in p.parts:result[str(p.relative_to(ROOT))]=file_hash(p)
    for relative in ('.ros2/dpgo-install/cbs/lib/libcbs.so','.ros2/dpgo-install/cbs_ros/lib/cbs_ros/cbs_ros_node',
                     '.ros2/ellipse-install/ellipselio/lib/libellipselio_mapping.so','.ros2/ellipsoid-cuda/libellipsoid_surface.so',
                     'FAST-LIVO2-ROS2/research/configs/square1-ellipselio-mapclosures-cbs.yaml',
                     *[f'.ros2/ellipse-configs/{r}.yaml' for r in ROBOTS]):
        result[relative]=file_hash(ROOT/relative)
    for module in ('s3e_mapclosures_native','s3e_mapclosures_inspection'):
        for p in (ROOT/'.ros2/research-venv/lib/python3.10/site-packages').glob(module+'*.so'):
            result[str(p.relative_to(ROOT))]=file_hash(p)
    return result


def sequence(dataset,work,duration=0.,config_path=None):
    cfg=yaml.safe_load((config_path or ROOT/'FAST-LIVO2-ROS2/research/configs/square1-ellipselio-mapclosures-cbs.yaml').read_text())
    cfg.update(dataset=str(dataset),output_root=str(work),descriptor_branches=['ellipsoid'],ellipsoid_basis_roundoff_tolerance=.001)
    cfg['backend']['mapclosures']['local_map']='Causal native persistent ellipsoid map; 80 m crop; 0.125 m surface sampling and 0.25 m voxel centroids'
    cfg['dpgo'].update(ros_domain_id=cfg['dpgo'].get('ros_domain_id',186) if config_path else 186,timeout_s=1200)
    cfg['dpgo']['pcm']['enabled']=True
    # Registration factors remain enabled exactly as in the working CBS configuration.
    work.mkdir(parents=True,exist_ok=False)
    (work/'config.yaml').write_text(yaml.safe_dump(cfg,sort_keys=False));write_json(work/'source-hashes.json',source_hashes())
    meta=yaml.safe_load((dataset/'metadata.yaml').read_text())['rosbag2_bagfile_information']
    topics={t['topic_metadata']['name']:t for t in meta['topics_with_message_count']}
    info=dict(dataset=str(dataset),metadata_sha256=file_hash(dataset/'metadata.yaml'),
              duration_s=meta['duration']['nanoseconds']/1e9,files={},calibration={})
    for name in meta['relative_file_paths']:
        p=dataset/name
        if not p.is_file() or not p.resolve().is_relative_to(dataset.resolve()):raise ValueError('Missing/unsafe bag file')
        info['files'][name]=dict(bytes=p.stat().st_size,mtime_ns=p.stat().st_mtime_ns)
    for robot in ROBOTS:
        for topic in (f'/{robot}/velodyne_points',f'/{robot}/imu/data'):
            if topic not in topics or topics[topic]['message_count']<1:raise ValueError('Missing sensor stream '+topic)
        info['calibration'][robot]=file_hash(dataset.parent/'Calibration'/f'{robot.lower()}.yaml')
    write_json(work/'dataset.json',info)
    schedules={}
    def progress(stage,robot=None):atomic(work/'progress.json',dict(stage=stage,robot=robot,updated_utc=utc()))
    for robot in ROBOTS:
        progress('odometry',robot)
        config=work/f'{robot}.yaml';config.write_text(yaml.safe_dump(mapping(dataset,robot,cfg),sort_keys=False))
        cmd=['bash',ROOT/'FAST-LIVO2-ROS2/scripts/run_ellipselio.sh','--robot',robot,'--bag',dataset,
             '--output',work/robot,'--mapping-config',config,'--rate','1.0']
        if duration:cmd+=['--duration',duration]
        call(cmd,work/f'{robot}-odometry.log',max(300,(duration or info['duration_s'])*2+240))
        q=check_export(work/robot/'export');write_json(work/robot/'quality.json',q)
        if not q['passed']:raise RuntimeError(f'{robot} frontend failed: {q}')
        rows=read_jsonl(work/robot/'export/frames.jsonl')
        # Independently verify native keyframe export against the Python selector.
        from s3e_pipeline.data import select
        selected=[];last=None
        for row in rows:
            wanted=select(row,last,cfg['keyframes'])
            if wanted!=('ellipsoid_delta' in row):raise ValueError('Native/Python keyframe selection mismatch')
            if wanted:
                last=row
                if row['ellipsoids']['requested_stamps_ns']!=[row['stamp_ns']]:raise ValueError('Snapshot association mismatch')
                selected.append(dict(keyframe_id=len(selected),stamp_ns=row['stamp_ns']))
        if not selected:raise ValueError('No exported keyframes')
        schedules[robot]=selected
        link=work/'odometry'/robot;link.mkdir(parents=True);(link/'run').symlink_to(work/robot,target_is_directory=True)
    write_json(work/'schedule.json',dict(schedule=schedules,selection=cfg['keyframes'],ground_truth_used=False,
        source='Exact causal keyframes selected by the current EllipseLIO export; no second frontend replay'))
    for robot in ROBOTS:
        progress('ellipsoid_descriptors',robot)
        call([PYTHON,'-m','s3e_pipeline.ellipsoid_full','prepare','--work',work,'--robot',robot],
             work/f'{robot}-descriptors.log',3600)
    progress('distributed_pcm_cbs')
    call([PYTHON,'-m','s3e_pipeline.ellipsoid_cbs','solve','--work',work],work/'cbs.log',1500)
    progress('evo_maps_report')
    call([PYTHON,'-m','s3e_pipeline.ellipsoid_cbs','evaluate','--work',work],work/'evaluation.log',1800)
    progress('complete')


def cleanup(work,success):
    """Retire only this attempt's regenerable geometry after compact evidence exists."""
    if (work/'cleanup.json').exists() and read_json(work/'cleanup.json').get('complete'):
        return
    archive=work/'retained';archive.mkdir(exist_ok=True);entries=[]
    if success and not (work/'report/COMPLETE.json').is_file():raise ValueError('Missing complete report')
    # Preserve exact sensor/pose inputs for the still-pending matched Swarm
    # comparison. Hard links avoid a second MCAP copy; report this retention.
    preserve = success or all((work/r/'quality.json').exists() and
        read_json(work/r/'quality.json').get('passed') for r in ROBOTS)
    if preserve:preserve_shared_odometry(work)
    for robot in ROBOTS:
        export=work/robot/'export'
        if not export.exists():continue
        dst=archive/robot;dst.mkdir(exist_ok=True)
        for name in ('frames.jsonl','manifest.json'):
            p=export/name
            if p.is_file():
                with p.open('rb') as i,gzip.open(dst/(name+'.gz'),'wb') as o:shutil.copyfileobj(i,o)
                with gzip.open(dst/(name+'.gz'),'rb') as f:
                    if hashlib.sha256(f.read()).hexdigest()!=file_hash(p):raise ValueError('Archive verification failed')
        for p in [export/'sensors.mcap',*sorted((export/'ellipsoids').glob('*.npz'))]:
            if not p.exists():continue
            if p.is_symlink() or not p.resolve().is_relative_to(work.resolve()):raise ValueError('Unsafe cleanup path')
            entries.append(dict(path=str(p.relative_to(work)),bytes=p.stat().st_size,sha256=file_hash(p)))
    # Keep exact descriptors and compact metadata; selected BEVs are in the report.
    for marker in work.glob('prepared-*.json'):
        prepared=Path(read_json(marker)['path'])
        if not prepared.resolve().is_relative_to(work.resolve()):raise ValueError('Foreign prepared stage')
        for p in [*sorted((prepared/'store').glob('*.npz')),*sorted((prepared/'bevs').glob('*.npz'))]:
            if p.is_symlink():raise ValueError('Symlink in cleanup')
            entries.append(dict(path=str(p.relative_to(work)),bytes=p.stat().st_size,sha256=file_hash(p)))
    retained=sum(e['bytes'] for e in entries if e['path'].endswith('/sensors.mcap')) if preserve else 0
    record=dict(complete=False,bytes=sum(e['bytes'] for e in entries),files=entries,
        shared_sensor_bytes_retained=retained,bytes_released=sum(e['bytes'] for e in entries)-retained,
        scope='Generated sensor clouds, ellipsoid deltas, verification submaps and full BEV image cache only; descriptors, selected BEVs, reports, poses and original S3E bags retained')
    atomic(work/'cleanup.json',record)
    for entry in entries:(work/entry['path']).unlink()
    for marker in work.glob('prepared-*.json'):
        prepared=Path(read_json(marker)['path'])
        atomic(prepared/'RETIRED.json',dict(reason='Regenerable geometry retired; stage is no longer a complete cache',archive=str(archive)))
    record['complete']=True;atomic(work/'cleanup.json',record)
    if success:shutil.copy2(work/'cleanup.json',work/'report/cleanup.json')
    (archive/'README.md').write_text('Generated geometry retired after saving compact evidence. Immutable preparation directories are deliberately no longer reusable geometry caches; regenerate from the original bags.\n')


def preserve_shared_odometry(work):
    shared=work/'shared-odometry';saved={}
    for robot in ROBOTS:
        source=work/robot/'export';target=shared/'frontend'/robot/'export'
        if (target/'manifest.json').exists():
            saved[robot]=read_json(target/'manifest.json')['files'];continue
        target.mkdir(parents=True,exist_ok=True)
        rows=read_jsonl(source/'frames.jsonl')
        for row in rows:
            row.pop('ellipsoids',None);row.pop('ellipsoid_delta',None)
        from s3e_pipeline.artifacts import write_jsonl
        write_jsonl(target/'frames.jsonl',rows)
        os.link(source/'sensors.mcap',target/'sensors.mcap')
        original=read_json(source/'manifest.json')
        files={n:file_hash(target/n) for n in ('frames.jsonl','sensors.mcap')}
        if files['sensors.mcap']!=original['files']['sensors.mcap']:raise ValueError('Shared sensor payload changed')
        write_json(target/'manifest.json',dict(original,files=files,
            derivation=dict(source_manifest_sha256=file_hash(source/'manifest.json'),
                change='Removed optional ellipsoid-delta references only; sensor bytes, integer timestamps and poses unchanged')))
        shutil.copy2(work/robot/'summary.json',target.parent/'summary.json')
        saved[robot]=files
    write_json(shared/'inputs.json',dict(reason='Retained until the paired Swarm-SLAM comparison is finished',files=saved))


def summary(base,state):
    lines=['# Overnight EllipseLIO + ellipsoid MapClosures + PCM/CBS','',
        'Fixed configuration, one sequence at a time. Playground 1 remains excluded by the earlier user instruction. Failures are recorded and the queue continues.','',
        '| Sequence | Status | Shared combined ATE [m] | PCM proposed / retained | Report |','|---|---|---:|---:|---|']
    for name,item in state['sequences'].items():
        value='—';loops='—';link='—'
        if item.get('work'):
            work=Path(item['work']);p=work/'report/report.json'
            if p.exists():
                r=read_json(p);m=list(r['trajectory'].values())
                if len(m)==1 and len(m[0]['robots'])==3:value=f'{m[0]["rmse_m"]:.4f}'
                loops=f'{r["pcm"]["proposed_loops"]} / {r["pcm"]["retained_loops"]}'
                link=f'[report]({work.relative_to(base)}/report/REPORT.md)'
            elif (work/'failure.json').exists():link=f'[failure]({work.relative_to(base)}/failure.json)'
        lines.append(f'| {name} | {item["status"]} | {value} | {loops} | {link} |')
    (base/'RESULTS.md').write_text('\n'.join(lines)+'\n')


def queue(args):
    base=args.base.resolve();base.mkdir(parents=True,exist_ok=True)
    lock=(base/'queue.lock').open('w');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    bags=discover(args.data)
    if args.sequences:bags=[p for p in bags if p.name in args.sequences]
    if not bags:raise ValueError('No selected datasets')
    state=read_json(base/'queue.json') if (base/'queue.json').exists() else dict(started_utc=utc(),sequences={})
    code=source_hashes()
    if args.config:code['experiment_config']=file_hash(args.config)
    if 'source_hashes' in state and state['source_hashes']!=code:
        state.update(status='stopped_source_changed',updated_utc=utc())
        atomic(base/'queue.json',state);return
    state['source_hashes']=code
    state.update(pid=os.getpid(),pipeline='EllipseLIO + ellipsoid BEV MapClosures + distributed PCM/CBS + GICP',configuration='fixed; no sweep')
    for p in bags:state['sequences'].setdefault(p.name,dict(status='pending',dataset=str(p)))
    def save():state['updated_utc']=utc();atomic(base/'queue.json',state);summary(base,state)
    save()
    for dataset in bags:
        name=dataset.name;item=state['sequences'][name]
        if item['status'] in ('complete','failed'):continue
        current_code=source_hashes()
        if args.config:current_code['experiment_config']=file_hash(args.config)
        if current_code!=state['source_hashes']:
            state['status']='stopped_source_changed';save();return
        if shutil.disk_usage(base).free<100*2**30:
            state['status']='stopped_low_disk';save();return
        # After an external restart, a partial attempt is retained, never used as a completed cache.
        if item.get('work'):
            old=Path(item['work'])
            if (old/'report/COMPLETE.json').exists():
                cleanup(old,True);item.update(status='complete',finished_utc=utc());save();continue
            item.setdefault('interrupted_attempts',[]).append(item['work'])
        work=base/name/(time.strftime('%Y%m%dT%H%M%SZ',time.gmtime())+'-'+uuid.uuid4().hex[:6])
        item.update(status='running',work=str(work),started_utc=utc());state.update(status='running',current=name);save()
        try:
            meta=yaml.safe_load((dataset/'metadata.yaml').read_text())['rosbag2_bagfile_information']
            limit=min(14400,3*(args.duration or meta['duration']['nanoseconds']/1e9)*2+6600)
            command=[PYTHON,__file__,'--sequence',dataset,'--work',work,'--duration',args.duration]
            if args.config:command+=['--config',args.config]
            call(command,base/f'{name}.log',limit)
            if not (work/'report/COMPLETE.json').is_file():raise ValueError('Sequence exited without a completed report')
            item.update(status='complete',finished_utc=utc())
        except Exception as exc:
            item.update(status='failed',error=repr(exc),finished_utc=utc());work.mkdir(parents=True,exist_ok=True)
            atomic(work/'failure.json',dict(error=repr(exc),traceback=traceback.format_exc(),
                progress=read_json(work/'progress.json') if (work/'progress.json').exists() else None,utc=utc()))
        save()
        try:cleanup(work,item['status']=='complete')
        except Exception as exc:item['cleanup_error']=repr(exc);save()
    state.update(status='finished',current=None,finished_utc=utc());save()


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--data',type=Path,default=Path('/data/s3e'))
    p.add_argument('--base',type=Path,default=BASE/'overnight-20260916')
    p.add_argument('--sequences',nargs='+');p.add_argument('--duration',type=float,default=0.)
    p.add_argument('--linger',action='store_true',help='Remain inspectable after the queue finishes; accept Docker stop signals as PID 1')
    p.add_argument('--sequence',type=Path);p.add_argument('--work',type=Path)
    p.add_argument('--config',type=Path,help='Explicit fixed experiment config; existing default stays unchanged')
    args=p.parse_args()
    def interrupted(*_):raise KeyboardInterrupt('Persistent worker interrupted')
    signal.signal(signal.SIGTERM,interrupted)
    if args.config:args.config=args.config.resolve()
    if args.sequence:sequence(args.sequence.resolve(),args.work.resolve(),args.duration,args.config)
    else:
        queue(args)
        if args.linger:
            print(utc(),'Queue stopped/finished; inspect queue.json and RESULTS.md.',flush=True)
            while True:time.sleep(60)
