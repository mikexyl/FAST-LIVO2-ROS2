#!/usr/bin/env python3
"""Actual DDS communication replay of frozen point-cloud frontend artifacts at 1x.

Only availability is remapped to a common replay clock. Sensor anchors, geometry,
descriptors and calibration stay frozen. No ground truth is opened by workers.
"""
import argparse,copy,hashlib,json,os,signal,subprocess,sys,time,traceback
from pathlib import Path
import numpy as np
import yaml

ROOT=Path('/workspace'); BASE=ROOT/'.ros2/communication-benchmark148-20260928'
SOURCE=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(SOURCE/'research'))
from s3e_pipeline.artifacts import canonical,file_hash,read_json,read_jsonl
from s3e_pipeline.online_io import atomic_json


def merge(dst,src):
    for key,value in src.items():
        if isinstance(value,dict):merge(dst.setdefault(key,{}),value)
        else:dst[key]=copy.deepcopy(value)


def retained_odometry(source, robot):
    """Use native poses even when a previous backend produced no report.

    The native update timestamp is the same IMU pose timestamp used by the
    trajectory exporter, not the scan-start timestamp or ground truth.
    """
    raw_path=source/'report'/(robot+'-raw.tum')
    if raw_path.exists():
        raw=np.loadtxt(raw_path,ndmin=2)
    else:
        trial=Path(read_json(source/'trials.json')[robot])
        raw_path=trial/'frontend/native_updates.jsonl'
        rows=read_jsonl(raw_path)
        raw=np.array([[row['stamp_ns']/1e9,*row['pose']] for row in rows],dtype=float)
    if raw.ndim!=2 or len(raw)<2 or raw.shape[1]!=8 or not np.isfinite(raw).all() or np.any(np.diff(raw[:,0])<=0):
        raise ValueError('Invalid or nonchronological retained raw odometry: '+robot)
    return raw,raw_path


def run(group,attempt,duration=0.,domain=194):
    source=ROOT/'.ros2/point-bev-ablation148-20260925/groups'/group
    work=BASE/'runs'/('ours-'+group+'-'+attempt);work.mkdir(parents=True,exist_ok=False)
    status=BASE/'status'/('ours-'+group+'-'+attempt+'.json')
    state=dict(method='ours',group=group,phase='validating_inputs',output=str(work),started=time.time(),
        replay='frozen native odometry and point-cloud descriptors, common relative sensor-time 1x replay',resource_quotas=False)
    atomic_json(status,state)
    processes={};logs=[];streams={};started=time.monotonic()
    try:
        cfg=yaml.safe_load((source/'config.yaml').read_text())
        canonical_cfg=yaml.safe_load((SOURCE/'research/configs/default_pipeline.yaml').read_text())
        for key in ['backend','loops','pgo','dpgo']:merge(cfg[key],canonical_cfg[key])
        cfg.update(output_root=str(work),descriptor_branches=['point_cloud'],experiment_name='Communication benchmark: point-cloud backend, ten-second causal CBS inputs')
        cfg['dpgo'].update(ros_domain_id=domain+1,retain_registration_transport=False)
        cfg['evaluation'].pop('expected_connected_robots',None)
        config=work/'config.yaml';config.write_text(yaml.safe_dump(cfg,sort_keys=False))
        robots=cfg['robots'];state['robots']=robots
        events=[];audit={};hashes={str(source/'config.yaml'):file_hash(source/'config.yaml')}
        for robot in robots:
            raw,raw_path=retained_odometry(source,robot)
            speeds=np.linalg.norm(np.diff(raw[:,1:4],axis=0),axis=1)/np.diff(raw[:,0])
            audit[robot]=dict(poses=len(raw),max_speed_m_s=float(speeds.max()),duration_s=float(raw[-1,0]-raw[0,0]),path=str(raw_path),sha256=file_hash(raw_path))
            hashes[str(raw_path)]=file_hash(raw_path)
            if speeds.max()>20:
                atomic_json(work/'divergence.json',dict(robot=robot,reason='retained raw odometry exceeds 20 m/s',**audit[robot]))
                raise ValueError('Diverged retained odometry; entire group excluded: '+robot)
            old=source/f'prepared-{robot}';prepared=work/f'prepared-{robot}'
            store=prepared/'store';store.mkdir(parents=True);desc=prepared/'ellipsoid';desc.mkdir()
            index=old/'store/keyframes.jsonl';hashes[str(index)]=file_hash(index)
            rows=read_jsonl(index)
            for row in rows:
                key=row['keyframe_id'];delay=max(0.,row['available_ns']/1e9-raw[0,0])
                if duration and delay>duration:continue
                name=f'{key:06d}.npz';packet=f'{key:06d}.json.zlib'
                os.link(old/'store'/name,store/name);os.link(old/'ellipsoid'/packet,desc/packet)
                hashes[str(old/'store'/name)]=file_hash(old/'store'/name)
                hashes[str(old/'ellipsoid'/packet)]=file_hash(old/'ellipsoid'/packet)
                events.append((delay,robot,row))
            streams[robot]=(store/'keyframes.jsonl').open('xb',buffering=0)
            (work/f'live/{robot}').mkdir(parents=True)
        atomic_json(work/'input-hashes.json',hashes);atomic_json(work/'raw-odometry-audit.json',audit)
        atomic_json(work/'trials.json',read_json(source/'trials.json'))
        atomic_json(work/'source-hashes.json',{str(p):file_hash(p) for p in SOURCE.rglob('*') if p.is_file() and '__pycache__' not in str(p)})
        def launch(name,args,channel):
            log=(work/(name+'.log')).open('w');logs.append(log)
            env=dict(os.environ,ROS_DOMAIN_ID=str(channel),ROS_LOCALHOST_ONLY='1',CBS_UNDERLAY=str(ROOT/'.ros2/cbs-underlay'),PYTHONPATH=str(SOURCE/'research')+os.pathsep+str(SOURCE/'research/native')+os.pathsep+os.environ.get('PYTHONPATH',''))
            p=subprocess.Popen([sys.executable,*map(str,args)],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            processes[name]=p
        def check():
            for name,p in processes.items():
                if p.poll() is not None and (p.returncode or name.endswith('-live')):
                    raise RuntimeError(name+' exited '+str(p.returncode))
        for robot in robots:
            local=work/f'live/{robot}';prepared=work/f'prepared-{robot}'
            spec=dict(robot=robot,robots=robots,output=str(local),store=str(prepared/'store'),descriptors=str(prepared/'ellipsoid'),
                backend=cfg['backend'],loops=cfg['loops'],ros_underlay=str(ROOT/'.ros2/cbs-underlay'),ros_overlay=str(ROOT/'.ros2/dpgo-install'))
            atomic_json(local/'spec.json',spec)
            launch(robot+'-live',['-m','s3e_pipeline.online_worker','--spec',local/'spec.json'],domain)
        launch('epochs',['-m','s3e_pipeline.online_epochs','--work',work,'--source',ROOT,'--config',config],domain+1)
        deadline=time.monotonic()+120
        while not all((work/f'live/{r}/READY').exists() for r in robots) or not (work/'epochs/READY').exists():
            check()
            if time.monotonic()>deadline:raise TimeoutError('Online peer startup barrier')
            time.sleep(.1)
        start=time.monotonic();state.update(phase='replaying',replay_started=time.time(),target_cbs_input_interval_s=10.)
        atomic_json(status,state)
        for delay,robot,row in sorted(events,key=lambda e:(e[0],e[1],e[2]['keyframe_id'])):
            while time.monotonic()-start<delay:check();time.sleep(.05)
            item=copy.deepcopy(row);item.update(native_available_ns=row['available_ns'],available_ns=time.time_ns(),availability_clock='unix_wall_frozen_descriptor_release')
            streams[robot].write(canonical(item)+b'\n')
        replay_span=min(duration,max(x['duration_s'] for x in audit.values())) if duration else max(x['duration_s'] for x in audit.values())
        while time.monotonic()-start<replay_span:check();time.sleep(.05)
        for robot in robots:
            streams[robot].close()
            atomic_json(work/f'prepared-{robot}/summary.json',dict(complete=True,replay=True,source=str(source/f'prepared-{robot}')))
        state.update(phase='draining_cbs_queue',replay_wall_s=time.monotonic()-start);atomic_json(status,state)
        while not (work/'epochs/DONE').exists():check();time.sleep(.25)
        check()
        final=read_json(work/'epochs/latest.json') if (work/'epochs/latest.json').exists() else None
        if final:
            (work/'dpgo').symlink_to(Path(final['path']),target_is_directory=True)
        state.update(phase='complete',wall_s=time.monotonic()-started,epochs=read_json(work/'epochs/DONE'))
        if not all(file_hash(p)==h for p,h in hashes.items()):raise ValueError('Frozen source artifacts changed')
    except Exception as e:
        state.update(phase='failed',error=repr(e),traceback=traceback.format_exc(),wall_s=time.monotonic()-started)
    finally:
        # Ask persistent peers to flush journals before stopping their process groups.
        for robot in state.get('robots',[]):
            local=work/f'live/{robot}'
            if local.exists():(local/'STOP').touch()
        for name,p in processes.items():
            if p.poll() is None:os.killpg(p.pid,signal.SIGINT)
        for p in processes.values():
            try:p.wait(timeout=20)
            except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
        for f in [*streams.values(),*logs]:f.close()
        atomic_json(status,state)
    print(group,state['phase'],state.get('error'),flush=True)
    return 0 if state['phase']=='complete' else 1


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--group',required=True);p.add_argument('--attempt',default='full-v2');p.add_argument('--duration',type=float,default=0.);p.add_argument('--domain',type=int,default=194)
    a=p.parse_args();raise SystemExit(run(a.group,a.attempt,a.duration,a.domain))
