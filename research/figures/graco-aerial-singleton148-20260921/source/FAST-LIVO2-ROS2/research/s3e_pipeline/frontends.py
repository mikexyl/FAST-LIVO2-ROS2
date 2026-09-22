"""Odometry identity and sensor-specific EllipseLIO configuration."""
from pathlib import Path
import re
import subprocess
import yaml
from .artifacts import file_hash


def frontend_name(cfg):
    name=cfg.get('odometry',{}).get('frontend','fast_livo2')
    if name not in ('fast_livo2','ellipselio'):raise ValueError('Unsupported odometry frontend')
    return 'livo' if name=='fast_livo2' else name


def ellipse_mapping(source, robot):
    source=Path(source)
    base=yaml.safe_load((source/'ellipselio/config/vlp16_geode.yaml').read_text())
    p=base['/**']['ros__parameters']
    prior=yaml.safe_load((source/f'FAST-LIVO2-ROS2/config/s3e/{robot.lower()}.yaml').read_text())['/**']['ros__parameters']
    calibration=(Path('/data/s3e/Calibration')/f'{robot.lower()}.yaml').read_text()
    def value(name):return float(re.search(r'^'+re.escape(name)+r':\s*([\deE+.-]+)',calibration,re.M)[1])
    p['mapping'].update(namespace=robot)
    p['publish']={key:False for key in p['publish']};p['publish']['odometry']=True
    p['cameras']=dict(num_cams=0);p['input']=dict(reliable=True)
    p['lidar'].update(topic=f'/{robot}/velodyne_points',min_range=1.,max_range=100.,
        t_imu_lidar=prior['extrin_calib']['extrinsic_T'],r_imu_lidar=prior['extrin_calib']['extrinsic_R'])
    p['imu'].update(topic=f'/{robot}/imu/data',rate=int(value('IMU.Frequency')),
        acc_noise=value('IMU.NoiseAcc'),gyr_noise=value('IMU.NoiseGyro'),
        acc_bias=value('IMU.AccWalk'),gyr_bias=value('IMU.GyroWalk'))
    return base


def ellipse_provenance(source):
    source=Path(source);repo=source/'ellipselio'
    binary=source/'.ros2/ellipse-install/ellipselio/lib/libellipselio_mapping.so'
    if not binary.exists():raise FileNotFoundError('Build EllipseLIO with scripts/build_ellipselio.sh')
    return dict(upstream_commit='171acea502f1122e3043a460d2e6e3a30d8f8246',
        commit=subprocess.check_output(['git','-C',str(repo),'rev-parse','HEAD'],text=True).strip(),
        sources={str(p.relative_to(repo)):file_hash(p) for pattern in ('src/**/*','include/**/*','CMakeLists.txt','package.xml')
            for p in sorted(repo.glob(pattern)) if p.is_file()},binary_sha256=file_hash(binary))
