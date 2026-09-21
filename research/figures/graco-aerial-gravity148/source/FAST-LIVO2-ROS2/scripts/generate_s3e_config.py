#!/usr/bin/env python3
"""Translate the published S3E OpenCV calibration into FAST-LIVO2 parameters."""
import argparse
from pathlib import Path
import cv2
import numpy as np
import yaml


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--calibration-dir', type=Path, default=Path('/data/s3e/Calibration'))
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    for robot in ('Alpha', 'Bob', 'Carol'):
        source = args.calibration_dir / f'{robot.lower()}.yaml'
        fs = cv2.FileStorage(str(source), cv2.FILE_STORAGE_READ)
        if not fs.isOpened():
            raise FileNotFoundError(source)
        # Dataset convention: Tic maps camera -> IMU; Tlc maps camera -> LiDAR.
        tic, tlc = (fs.getNode(key).mat() for key in ('Tic', 'Tlc'))
        tcl = np.linalg.inv(tlc)
        til = tic @ tcl
        for transform in (tic, tlc, til):
            assert np.allclose(transform[:3, :3].T @ transform[:3, :3], np.eye(3), atol=1e-5)
            assert np.isclose(np.linalg.det(transform[:3, :3]), 1, atol=1e-5)
        base = yaml.safe_load((root / 'config/avia.yaml').read_text())
        p = base['/**']['ros__parameters']
        p['common'].update(img_topic=f'/{robot}/left_camera/image',
                           lid_topic=f'/{robot}/velodyne_points_start',
                           imu_topic=f'/{robot}/imu/data')
        p['extrin_calib'] = dict(extrinsic_T=til[:3, 3].tolist(),
                                extrinsic_R=til[:3, :3].ravel().tolist(),
                                Rcl=tcl[:3, :3].ravel().tolist(), Pcl=tcl[:3, 3].tolist())
        p['time_offset'].update(img_time_offset=0.0, imu_time_offset=0.0)
        p['preprocess'].update(lidar_type=2, scan_line=16, blind=1.0,
                               filter_size_surf=0.2, point_filter_num=1,
                               velodyne_time_scale=1000.0)
        # Sparse 16-beam scans need larger outer voxels for distant surfaces.
        # Repeated paving patterns should not dominate the geometric update.
        p['lio']['voxel_size'] = 2.0
        p['vio']['img_point_cov'] = 1000
        p['evo'].update(seq_name=f'S3E_Square_1_{robot.lower()}', pose_output_en=True)
        p['uav']['gravity_align_en'] = True
        p['publish']['dense_map_en'] = False
        p['local_map']['map_sliding_en'] = True
        # FAST-LIVO2 covariance settings retained as an initial tuning baseline.
        # Its discrete covariance parameters are not the dataset noise densities.
        p['imu'].pop('b_acc_cov', None)
        p['imu'].pop('b_gyr_cov', None)
        camera = dict(cam_model='Pinhole', scale=0.5)
        for dst, src in [('cam_width','width'), ('cam_height','height'),
                         ('cam_fx','fx'), ('cam_fy','fy'), ('cam_cx','cx'), ('cam_cy','cy'),
                         ('cam_d0','k1'), ('cam_d1','k2'), ('cam_d2','p1'), ('cam_d3','p2')]:
            v = fs.getNode(f'Camera.{src}').real()
            camera[dst] = int(v) if src in ('width', 'height') else v
        dest = root / 'config/s3e'
        dest.mkdir(exist_ok=True)
        header = (f'# Source: {source}\n'
                  '# T_imu_lidar = Tic @ inv(Tlc); T_camera_lidar = inv(Tlc).\n'
                  '# Point time is seconds relative to the adapted scan START header.\n')
        (dest / f'{robot.lower()}.yaml').write_text(header + yaml.safe_dump(base, sort_keys=False))
        (dest / f'{robot.lower()}_camera.yaml').write_text(yaml.safe_dump({'/**': {'ros__parameters': camera}}, sort_keys=False))
        fs.release()
        print(f'{robot}: generated calibrated LiDAR/IMU and left-camera parameters')


if __name__ == '__main__':
    main()
