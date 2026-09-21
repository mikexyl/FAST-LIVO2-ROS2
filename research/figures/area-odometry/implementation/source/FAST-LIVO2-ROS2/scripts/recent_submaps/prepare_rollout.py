#!/usr/bin/env python3
"""Freeze S3Ev2 Alpha/Carol calibration with the confirmed Bob settings."""
import copy
import hashlib
import json
from pathlib import Path
import cv2
import numpy as np
import yaml
root=Path('/workspace');out=root/'.ros2/recent-submaps/configs';out.mkdir(exist_ok=True)
base=yaml.safe_load((Path(__file__).parent/'enabled.yaml').read_text())
calibrations={}
for robot in ('Alpha','Bob','Carol'):
    calibration=Path('/data/s3e/S3Ev2/Calibration')/f'{robot.lower()}.yaml'
    fs=cv2.FileStorage(str(calibration),cv2.FILE_STORAGE_READ)
    if not fs.isOpened():raise ValueError(f'Calibration unavailable: {calibration}')
    T=fs.getNode('Tic').mat()@np.linalg.inv(fs.getNode('Tlc').mat())
    if not np.isfinite(T).all() or not np.allclose(T[:3,:3].T@T[:3,:3],np.eye(3),atol=1e-5):
        raise ValueError('Invalid calibration rotation')
    rate=int(fs.getNode('IMU.Frequency').real());fs.release()
    cfg=copy.deepcopy(base);params=cfg['/**']['ros__parameters']
    if robot=='Bob':
        if not np.allclose(T[:3,3],params['lidar']['t_imu_lidar'],atol=1e-12) or not np.allclose(T[:3,:3].ravel(),params['lidar']['r_imu_lidar'],atol=1e-12):
            raise ValueError('Bob comparison calibration differs from S3Ev2 source')
    params['mapping']['namespace']=robot
    params['lidar'].update(topic=f'/{robot}/velodyne_points',t_imu_lidar=T[:3,3].tolist(),r_imu_lidar=T[:3,:3].ravel().tolist())
    params['imu'].update(topic=f'/{robot}/imu/data',rate=rate)
    (out/f'{robot}.yaml').write_text(yaml.safe_dump(cfg,sort_keys=False))
    calibrations[robot]=dict(path=str(calibration),sha256=hashlib.sha256(calibration.read_bytes()).hexdigest())
(out/'calibration.json').write_text(json.dumps(calibrations,indent=2)+'\n')
