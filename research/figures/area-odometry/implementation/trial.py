import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import numpy as np

root=Path(__file__).resolve().parents[2]
base=Path(__file__).resolve().parent
work=base/'comparison'
scripts=root/'FAST-LIVO2-ROS2/scripts/recent_submaps'
sys.path[:0]=[str(scripts),str(root/'FAST-LIVO2-ROS2/research')]
from graco_aerial import call, save, sha

def rows(path):return [json.loads(line) for line in path.read_text().splitlines()]

def measure(path):
    data=rows(path/'frontend/native_updates.jsonl')
    times=np.array([r['sensor_stamp_ns'] for r in data],dtype=np.int64)
    pose_times=np.array([r['stamp_ns'] for r in data],dtype=np.int64)
    poses=np.array([r['pose'] for r in data])
    assert np.isfinite(poses).all() and np.all(np.diff(times)>0) and np.all(np.diff(pose_times)>0)
    accepted=[i for i,r in enumerate(data) if r['lidar_updated']]
    gap=float(np.diff(np.r_[times[0],times[accepted],times[-1]]).max()/1e9)
    speed=float((np.linalg.norm(np.diff(poses[:,:3],axis=0),axis=1)/(np.diff(pose_times)/1e9)).max())
    elapsed=(times-times[0])/1e9
    interval=[r for r,t in zip(data,elapsed) if 27<=t<=33]
    processing=np.array([r['processing_s'] for r in data])
    memory=rows(path/'memory.jsonl')
    areas=rows(path/'frontend/area_maps/index.jsonl')
    return dict(native_poses=len(data),finite_chronological=True,full_sequence=False,ground_truth_used=False,
        passed=gap<1 and speed<20,max_successful_update_gap_s=gap,max_speed_m_s=speed,
        successful_updates=len(accepted),unsuccessful_updates=len(data)-len(accepted),
        interval_27_to_33_s=dict(scans=len(interval),failed=sum(not r['lidar_updated'] for r in interval),
            zero_features=sum(r['features']==0 for r in interval)),
        processing_ms=dict(median=float(np.median(processing)*1000),p95=float(np.percentile(processing,95)*1000),max=float(processing.max()*1000)),
        max_correspondence_age_s=max(r['age_max_s'] for r in data if r['lidar_updated']),
        handovers=data[-1]['handovers'],odometry_map_source=data[-1]['odometry_map_source'],
        mapper_peak_rss_mib=max(r['VmHWM'] for r in memory)/1024,area_snapshots=len(areas),
        final_archive_points=areas[-1]['archive_point_count'],pose_duration_s=float(elapsed[-1]))

work.mkdir(exist_ok=False)
started=time.monotonic();state=dict(phase='starting',ground_truth_used=False,duration_s=120,results={})
save(work/'status.json',state)
try:
    watched={p.resolve() for directory in (root/'ellipselio/src',root/'ellipselio/include',root/'ellipselio/msg',scripts,root/'FAST-LIVO2-ROS2/research/s3e_pipeline') for p in directory.rglob('*') if p.is_file() and p.suffix in ('.py','.cpp','.h','.hpp','.msg','.yaml','.sh')}
    watched.update([base/'install/ellipselio/lib/libellipselio_mapping.so',base/'launcher/ellipselio_mapping_mt',root/'ellipselio/CMakeLists.txt',root/'FAST-LIVO2-ROS2/scripts/ellipselio_live_rerun.py',root/'FAST-LIVO2-ROS2/scripts/run_ellipselio.py'])
    frozen={str(p):sha(p) for p in sorted(watched)};save(work/'source-hashes.json',frozen)
    cases=[('coverage-control',False),('area-odometry',True),('area-odometry-repeat',True)]
    for index,(name,enabled) in enumerate(cases):
        if name.endswith('repeat') and not state['results']['area-odometry']['passed']:break
        state['phase']=name;save(work/'status.json',state)
        env=dict(os.environ,ROS_DOMAIN_ID=str(230+index))
        command=['/usr/bin/python3',scripts/'run_trial.py',name,'--robot','aerial08','--duration','120',
            '--bag',root/'.ros2/graco-aerial-temporal-20260920/inputs/aerial08',
            '--mapping-config',root/'.ros2/graco-aerial-temporal-20260920/configs/aerial08.yaml',
            '--output-root',work,'--mapper-executable',base/'launcher/ellipselio_mapping_mt',
            '--mapping-library',base/'install/ellipselio/lib/libellipselio_mapping.so']
        command+=['--area-odometry'] if enabled else ['--area-maps','--submap-strategy','coverage']
        call(command,work/f'{name}.log',360,env)
        call([sys.executable,scripts/'validate.py',work/name],work/f'{name}-validation.log',180,env)
        call([root/'.ros2/rerun-venv/bin/rerun','rrd','verify',work/name/'recording/live.rrd'],work/f'{name}-rrd.log',120,env)
        result=measure(work/name)
        if enabled:
            assert result['handovers']==0 and result['odometry_map_source']=='accumulated_area'
            assert not (work/name/'frontend/submaps').exists()
            for row in rows(work/name/'frontend/native_updates.jsonl'):
                assert row['query_area_radius_m']==80 and np.isfinite(row['query_area_center_world']).all()
                assert np.isclose(np.linalg.norm(row['query_area_up_world']),1)
        save(work/f'{name}-quality.json',result)
        state['results'][name]=result;save(work/'status.json',state)
        print(name,json.dumps(result),flush=True)
    assert all(sha(Path(name))==value for name,value in frozen.items())
    state.update(phase='complete',wall_s=time.monotonic()-started,frozen_sources_verified=True)
    save(work/'status.json',state)
except Exception as error:
    import traceback
    state.update(phase='failed',error=repr(error),traceback=traceback.format_exc());save(work/'status.json',state);raise
