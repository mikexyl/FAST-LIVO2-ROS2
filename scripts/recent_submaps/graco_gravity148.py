#!/usr/bin/env python3
"""Isolated gravity-BEV reprocessing of the immutable four-flight aerial capture."""
import argparse
import hashlib
import json
import shutil
import sqlite3
import sys
import time
import traceback
from pathlib import Path

import numpy as np
import yaml

ROOT = Path('/workspace')
BASE = ROOT/'.ros2/graco-aerial-gravity148-20260920'
OLD = ROOT/'.ros2/graco-aerial-four148-20260920'
SCRIPTS = ROOT/'FAST-LIVO2-ROS2/scripts/recent_submaps'
PYTHON = ROOT/'.ros2/research-venv/bin/python'
RERUN = ROOT/'.ros2/rerun-venv/bin'
sys.path.insert(0, str(ROOT/'FAST-LIVO2-ROS2/research'))
from graco_aerial148 import audit, call, now, read, rows, save, sha


def startup_gravity(robot, source, output):
    """Causal approximation for old captures, not a recovered ESKF gravity state.

    The native initializer averages >125 samples in the initial IMU/world frame.
    Replay callback timing and later ESKF gravity corrections were not recorded.
    Use the first 126 measurements and retain that limitation explicitly.
    """
    from rosbags.typesys import Stores, get_typestore
    types = get_typestore(Stores.ROS2_HUMBLE)
    database = OLD/'inputs'/robot/'sensors.db3'
    with sqlite3.connect(f'file:{database}?mode=ro', uri=True) as db:
        messages = db.execute('SELECT timestamp,data FROM messages WHERE topic_id='
                             '(SELECT id FROM topics WHERE name=?) ORDER BY timestamp LIMIT 126',
                             ('/gnss/imu',)).fetchall()
    acc, gyro, stamps = [], [], []
    for _, raw in messages:
        msg = types.deserialize_cdr(raw, 'sensor_msgs/msg/Imu')
        a, g = msg.linear_acceleration, msg.angular_velocity
        acc.append([a.x,a.y,a.z]);gyro.append([g.x,g.y,g.z])
        stamps.append(int(msg.header.stamp.sec)*10**9+int(msg.header.stamp.nanosec))
    acc, gyro = np.asarray(acc), np.asarray(gyro)
    assert len(acc)==126 and np.isfinite(acc).all() and np.isfinite(gyro).all()
    assert np.all(np.diff(stamps)>0)
    mean = acc.mean(axis=0)
    assert 8 < np.linalg.norm(mean) < 12
    gravity_world = -mean/np.linalg.norm(mean)*9.81
    first, second = acc[:63].mean(axis=0), acc[63:].mean(axis=0)
    angle = np.degrees(np.arccos(np.clip(first@second/np.linalg.norm(first)/np.linalg.norm(second),-1,1)))
    manifest=read(source/'manifest.json'); submaps={}
    for row in rows(source/'index.jsonl'):
        assert row['begin_ns']>max(stamps)
        rotation=np.asarray(row['T_world_imu']).reshape(4,4)[:3,:3]
        submaps[str(row['submap_id'])]=dict(stamp_ns=row['stamp_ns'], payload_sha256=row['sha256'],
                                          gravity_imu_m_s2=(rotation.T@gravity_world).tolist())
    save(output, dict(schema_version=1,robot_id=robot,ground_truth_used=False,
        source_index_sha256=manifest['index_sha256'],source_database=str(database),database_sha256=sha(database),
        topic='/gnss/imu',samples=126,first_sample_ns=min(stamps),available_ns=max(stamps),
        method='startup_accelerometer_reconstruction',
        limitation='First 126 sensor measurements approximate the native initialization average. '
                   'Callback-specific initialization membership and later ESKF gravity corrections were not exported. '
                   'This is not the exact per-anchor filter gravity.',
        mean_acceleration_m_s2=mean.tolist(),acceleration_std_m_s2=acc.std(axis=0).tolist(),
        mean_gyro_rad_s=gyro.mean(axis=0).tolist(),startup_half_direction_difference_deg=float(angle),
        gravity_world_m_s2=gravity_world.tolist(),submaps=submaps))


def verify_original():
    for path, expected in read(OLD/'full/source-hashes.json').items():
        actual=Path(path)
        for name in ('FAST-LIVO2-ROS2','ellipselio'):
            prefix=ROOT/name
            if actual.is_relative_to(prefix):
                actual=OLD/'source'/name/actual.relative_to(prefix)
                break
        assert sha(actual)==expected, str(actual)


def run(work):
    work.mkdir(parents=True,exist_ok=False)
    started=time.monotonic()
    state=dict(phase='preflight',started_utc=now(),original_work=str(OLD/'full'),
               frontend_reused=True,gravity_reconstructed=True,diagnostic_only=True)
    def mark(phase, **values):
        state.update(phase=phase,updated_utc=now(),**values);save(work/'status.json',state)
    try:
        assert (BASE/'BUILD_COMPLETE').exists()
        verify_original()
        import s3e_mapclosures_native
        assert Path(s3e_mapclosures_native.__file__).is_relative_to('/opt/graco-gravity-mapclosures')
        assert sha(Path(s3e_mapclosures_native.__file__))==sha(BASE/'mapclosures'/Path(s3e_mapclosures_native.__file__).name)
        watched={p for root in (ROOT/'FAST-LIVO2-ROS2/research/s3e_pipeline',SCRIPTS,
                    ROOT/'FAST-LIVO2-ROS2/research/adapters/mapclosures',ROOT/'ellipselio')
                 for p in root.rglob('*') if p.is_file()}
        watched.update([Path(s3e_mapclosures_native.__file__),ROOT/'.ros2/ellipsoid-cuda/libellipsoid_surface.so',
                        ROOT/'.ros2/dpgo-install/cbs/lib/libcbs.so',
                        ROOT/'.ros2/dpgo-install/cbs_ros/lib/cbs_ros/cbs_ros_node'])
        frozen={str(p):sha(p) for p in sorted(watched)}
        save(work/'source-hashes.json',frozen)
        cfg=yaml.safe_load((OLD/'full/config.yaml').read_text())
        cfg.update(dataset=str(work/'reference'),output_root=str(work),
                   experiment_name='GRACO aerial 05/06/07/08, temporal submaps, reconstructed IMU gravity BEVs')
        cfg['backend']['mapclosures']['projection_alignment']='gravity'
        (work/'config.yaml').write_text(yaml.safe_dump(cfg,sort_keys=False))
        for name in ['trials.json','frontend-gate.json']+[r+'-quality.json' for r in cfg['robots']]+[
                     r+'-recording-verification.log' for r in cfg['robots']]:
            shutil.copy2(OLD/'full'/name,work/name)
        trials=read(work/'trials.json');artifacts={}
        (work/'gravity').mkdir()
        mark('descriptors')
        for robot in cfg['robots']:
            mark('descriptors',active_robot=robot)
            source=Path(trials[robot])/'frontend/submaps'
            sidecar=work/'gravity'/f'{robot}.json'
            startup_gravity(robot,source,sidecar)
            output=work/f'prepared-{robot}'
            call([PYTHON,'-m','s3e_pipeline.recent_submaps','--source',source,'--output',output,
                  '--config',work/'config.yaml','--gravity-sidecar',sidecar],work/f'{robot}-descriptors.log',1800)
            artifacts[f'keyframes.ellipselio.{robot}']=str(output)
            artifacts[f'descriptors.ellipselio.mapclosures.{robot}']=str(output/'ellipsoid')
        save(work/'artifacts.json',artifacts)
        mark('distributed_pcm_cbs')
        call([PYTHON,SCRIPTS/'graco_aerial148.py','backend','--work',work],work/'backend.log',1920)
        mark('evaluation')
        # First access to GT is after the full distributed backend has finished.
        shutil.copytree(OLD/'full/reference',work/'reference')
        call([PYTHON,SCRIPTS/'rollout_report.py','--work',work,'--trials',work/'trials.json'],work/'evaluation.log',1800)
        report=read(work/'report/report.json')
        report.update(diagnostic_only=True,gravity_reconstructed=True,
                      frontend_quality=read(work/'frontend-gate.json')['quality'])
        save(work/'report/report.json',report)
        call([RERUN/'python',SCRIPTS/'rollout_rerun.py',work],work/'rerun.log',300)
        call([RERUN/'rerun','rrd','verify',work/'report/result.rrd'],work/'recording-verification.log',300)
        mark('auditing')
        audit(work,frozen)
        evidence={}
        for robot in cfg['robots']:
            files=list((work/f'prepared-{robot}/store').glob('*.npz'))
            assert all(sha(p)==sha(OLD/f'full/prepared-{robot}/store'/p.name) for p in files)
            assert sha(work/f'report/{robot}-raw.tum')==sha(OLD/f'full/report/{robot}-raw.tum')
            evidence[robot]=dict(identical_evidence_files=len(files),identical_raw_trajectory=True)
        original=read(OLD/'full/report/report.json')
        assert report['raw']==original['raw']
        verify_original()
        save(work/'comparison.json',dict(original_work=str(OLD/'full'),evidence=evidence,
            original_sources_verified=True,original=dict(cbs=original['cbs'],connectivity=original['connectivity'],
                runtime=original['runtime'],pcm=original['pcm']),
            gravity=dict(cbs=report['cbs'],connectivity=report['connectivity'],runtime=report['runtime'],pcm=report['pcm'])))
        mark('complete',finished_utc=now(),wall_s=time.monotonic()-started,
             loops=report['runtime']['loops'],connectivity=report['connectivity'])
    except Exception as error:
        save(work/'failure.json',dict(stage=state['phase'],error=repr(error),traceback=traceback.format_exc()))
        mark('failed',error=repr(error))
        raise


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--work',type=Path,default=BASE/'full')
    run(parser.parse_args().work)
