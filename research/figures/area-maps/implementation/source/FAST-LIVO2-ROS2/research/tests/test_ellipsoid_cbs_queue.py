"""Overnight queue bounds and conservative cleanup of a completed/failed attempt."""
import importlib.util
import json
from pathlib import Path
import pytest

path=Path(__file__).resolve().parents[2]/'scripts/ellipsoid_cbs148/queue.py'
spec=importlib.util.spec_from_file_location('ellipsoid_cbs_queue',path)
queue=importlib.util.module_from_spec(spec);spec.loader.exec_module(queue)


def test_discovery_keeps_user_exclusion_and_prioritizes_known_sequences(tmp_path):
    names=['S3E_Playground_1','S3E_Tunnel_1','S3E_Square_2','S3E_Square_1']
    for name in names:
        p=tmp_path/'S3Ev1'/name;p.mkdir(parents=True);(p/'metadata.yaml').write_text('')
    assert [p.name for p in queue.discover(tmp_path)]==['S3E_Square_1','S3E_Square_2','S3E_Tunnel_1']


def test_requested_noise_survives_calibration_and_native_qos_is_omitted(tmp_path,monkeypatch):
    import cv2
    import numpy as np
    import yaml
    root=tmp_path/'source';configs=root/'.ros2/ellipse-configs';configs.mkdir(parents=True)
    (configs/'Alpha.yaml').write_text(yaml.safe_dump({'/**':{'ros__parameters':dict(
        lidar={},imu={},publish={'analytics':False},input={'reliable':True})}}))
    calibration=tmp_path/'S3Ev2/Calibration';calibration.mkdir(parents=True)
    T=np.eye(4);T[:3,3]=[.1,-.2,.3]
    fs=cv2.FileStorage(str(calibration/'alpha.yaml'),cv2.FILE_STORAGE_WRITE)
    fs.write('Tic',T);fs.write('Tlc',np.eye(4))
    fs.release()
    # OpenCV reads S3E's dotted keys but its writer refuses to create them.
    with (calibration/'alpha.yaml').open('a') as f:
        for name,value in {'IMU.Frequency':100.,'IMU.NoiseAcc':.01,'IMU.NoiseGyro':.02,
                           'IMU.AccWalk':.003,'IMU.GyroWalk':.004}.items():f.write(f'{name}: {value}\n')
    monkeypatch.setattr(queue,'ROOT',root)
    cfg=dict(keyframes=dict(translation_m=1.,rotation_deg=10.,max_interval_s=2.),
             odometry=dict(imu_noise=dict(acc_noise=.1,gyr_noise=.1,acc_bias=.0001,gyr_bias=.0001),
                           native_sensor_qos=True,record_analytics=True))
    p=queue.mapping(tmp_path/'S3Ev2/sequence','Alpha',cfg)['/**']['ros__parameters']
    assert p['imu']==dict(rate=100,acc_noise=.1,gyr_noise=.1,acc_bias=.0001,gyr_bias=.0001)
    assert p['lidar']['t_imu_lidar']==[.1,-.2,.3]
    assert 'input' not in p and p['publish']['analytics']
    # Without the explicit experiment settings, the original run remains reproducible.
    cfg.pop('odometry');p=queue.mapping(tmp_path/'S3Ev2/sequence','Alpha',cfg)['/**']['ros__parameters']
    assert p['imu']['acc_noise']==.01 and p['input']['reliable'] is True


def test_cleanup_requires_completed_report_and_preserves_original_data(tmp_path):
    work=tmp_path/'attempt';export=work/'Alpha/export';export.mkdir(parents=True)
    (export/'frames.jsonl').write_text('{"stamp_ns": 1}\n')
    (export/'sensors.mcap').write_bytes(b'generated fixture')
    original=tmp_path/'original.db3';original.write_bytes(b'original dataset')
    with pytest.raises(ValueError,match='Missing complete report'):queue.cleanup(work,True)
    assert (export/'sensors.mcap').exists()
    queue.cleanup(work,False)
    assert not (export/'sensors.mcap').exists()
    assert (export/'frames.jsonl').exists() and (work/'retained/Alpha/frames.jsonl.gz').exists()
    assert original.read_bytes()==b'original dataset'
    assert json.loads((work/'cleanup.json').read_text())['complete']
    record=(work/'cleanup.json').read_bytes()
    queue.cleanup(work,False)
    assert (work/'cleanup.json').read_bytes()==record


def test_shared_odometry_keeps_identical_payload_timestamps_and_poses(tmp_path):
    import hashlib
    work=tmp_path/'work';payload=b'exact serialized cloud and pose bytes'
    for robot in queue.ROBOTS:
        source=work/robot/'export';source.mkdir(parents=True)
        row=dict(stamp_ns=1700000000000000017,T_world_body=list(range(16)),ellipsoids={'count':1},ellipsoid_delta={'file':'absent'})
        (source/'frames.jsonl').write_text(json.dumps(row)+'\n');(source/'sensors.mcap').write_bytes(payload)
        (source/'manifest.json').write_text(json.dumps(dict(schema_version=1,complete=True,files={'sensors.mcap':hashlib.sha256(payload).hexdigest()})))
        (source.parent/'summary.json').write_text('{"success":true}')
    queue.preserve_shared_odometry(work)
    for robot in queue.ROBOTS:
        source=work/robot/'export';target=work/'shared-odometry/frontend'/robot/'export'
        assert (source/'sensors.mcap').stat().st_ino==(target/'sensors.mcap').stat().st_ino
        derived=json.loads((target/'frames.jsonl').read_text())
        assert derived==dict(stamp_ns=1700000000000000017,T_world_body=list(range(16)))
        (source/'sensors.mcap').unlink()
        assert (target/'sensors.mcap').read_bytes()==payload
