import json
import os
from pathlib import Path
import sys
import time
import numpy as np
import yaml

root=Path(__file__).resolve().parents[2];base=Path(__file__).resolve().parent;work=base/'full'
scripts=root/'FAST-LIVO2-ROS2/scripts/recent_submaps'
sys.path[:0]=[str(scripts),str(root/'FAST-LIVO2-ROS2/research')]
from graco_aerial import call,save,sha
short=json.loads((base/'comparison/status.json').read_text())
assert short['phase']=='complete' and short['results']['area-odometry']['passed'] and short['results']['area-odometry-repeat']['passed']
frozen=json.loads((base/'comparison/source-hashes.json').read_text());assert all(sha(Path(p))==h for p,h in frozen.items())
resume=sys.argv[1:]==['--analyze-existing']
if resume:
    previous=json.loads((work/'status.json').read_text())
    assert previous['phase']=='failed' and 'bool_' in previous['error']
    assert (work/'aerial08/artifact-validation.json').exists() and (work/'rrd.log').exists()
    save(work/'quality-generation-failed-status.json',previous)
else:
    work.mkdir(exist_ok=False);save(work/'source-hashes.json',frozen)
state=dict(phase='capture',ground_truth_used_by_frontend=False,full_sequence=True);save(work/'status.json',state)
start=time.monotonic();env=dict(os.environ,ROS_DOMAIN_ID='226')
trial=work/'aerial08';bag=root/'.ros2/graco-aerial-temporal-20260920/inputs/aerial08'
try:
    if not resume:
      call(['/usr/bin/python3',scripts/'run_trial.py','aerial08','--robot','aerial08','--area-odometry',
          '--bag',bag,'--mapping-config',root/'.ros2/graco-aerial-temporal-20260920/configs/aerial08.yaml',
          '--output-root',work,'--mapper-executable',base/'launcher/ellipselio_mapping_mt',
          '--mapping-library',base/'install/ellipselio/lib/libellipselio_mapping.so'],work/'capture.log',600,env)
      state['phase']='validation';save(work/'status.json',state)
      call([sys.executable,scripts/'validate.py',trial],work/'validation.log',240,env)
      call([root/'.ros2/rerun-venv/bin/rerun','rrd','verify',trial/'recording/live.rrd'],work/'rrd.log',180,env)
    data=[json.loads(l) for l in (trial/'frontend/native_updates.jsonl').read_text().splitlines()]
    stamp=np.array([r['sensor_stamp_ns'] for r in data],dtype=np.int64);pose_stamp=np.array([r['stamp_ns'] for r in data],dtype=np.int64)
    pose=np.array([r['pose'] for r in data]);ok=np.array([r['lidar_updated'] for r in data])
    assert np.isfinite(pose).all() and np.all(np.diff(stamp)>0) and np.all(np.diff(pose_stamp)>0)
    gap=float(np.diff(np.r_[stamp[0],stamp[ok],stamp[-1]]).max()/1e9)
    speed=float((np.linalg.norm(np.diff(pose[:,:3],axis=0),axis=1)/(np.diff(pose_stamp)/1e9)).max())
    processing=np.array([r['processing_s'] for r in data])*1000
    memory=[json.loads(l) for l in (trial/'memory.jsonl').read_text().splitlines()]
    maps=[json.loads(l) for l in (trial/'frontend/area_maps/index.jsonl').read_text().splitlines()]
    metadata=yaml.safe_load((bag/'metadata.yaml').read_text())['rosbag2_bagfile_information']
    remaining=float((metadata['starting_time']['nanoseconds_since_epoch']+metadata['duration']['nanoseconds']-stamp[-1])/1e9)
    quality=dict(native_poses=len(data),finite_chronological=True,full_sequence=True,passed=gap<1 and speed<20 and remaining<1,
        max_successful_update_gap_s=gap,max_speed_m_s=speed,successful_updates=int(ok.sum()),unsuccessful_updates=int((~ok).sum()),
        processing_ms=dict(median=float(np.median(processing)),p95=float(np.percentile(processing,95)),max=float(processing.max())),
        mapper_peak_rss_mib=max(r['VmHWM'] for r in memory)/1024,area_snapshots=len(maps),
        handovers=data[-1]['handovers'],max_correspondence_age_s=max(r['age_max_s'] for r in data if r['lidar_updated']),
        pose_duration_s=float((stamp[-1]-stamp[0])/1e9),last_pose_before_bag_end_s=remaining,
        final_archive_points=maps[-1]['archive_point_count'],odometry_map_source=data[-1]['odometry_map_source'])
    assert quality['handovers']==0 and quality['odometry_map_source']=='accumulated_area'
    save(work/'quality.json',quality);print(json.dumps(quality),flush=True)
    state['phase']='evaluation';save(work/'status.json',state)
    call([sys.executable,base/'evaluate_full.py'],work/'evaluation.log',180,env)
    assert all(sha(Path(p))==h for p,h in frozen.items())
    state.update(phase='complete',frozen_sources_verified=True,frontend_passed=quality['passed'])
    state['post_capture_recovery_wall_s' if resume else 'wall_s']=time.monotonic()-start
    save(work/'status.json',state)
except Exception as error:
    import traceback
    state.update(phase='failed',error=repr(error),traceback=traceback.format_exc());save(work/'status.json',state);raise
