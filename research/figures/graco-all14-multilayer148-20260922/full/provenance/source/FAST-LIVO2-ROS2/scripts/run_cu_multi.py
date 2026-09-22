#!/usr/bin/env python3
"""Bounded CU-Multi EllipseLIO/ellipsoid MapClosures/PCM/CBS experiment."""
import argparse
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time
import traceback

import yaml

SOURCE = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(SOURCE/'FAST-LIVO2-ROS2/research'))
sys.path.insert(0, str(SOURCE/'Swarm-SLAM/s3e'))
from s3e_pipeline.artifacts import file_hash, read_json, read_jsonl, write_json
from s3e_pipeline.cu_multi import ROBOTS, extract_sensor_archive, stage_sensors, mapping_config


def progress(work, stage, **extra):
    write_json(work/'progress.json', dict(stage=stage, updated_utc=time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()), **extra))
    print(stage, extra, flush=True)


def call(command, log, timeout):
    with log.open('w') as stream:
        proc = subprocess.Popen([str(x) for x in command], cwd=SOURCE, stdout=stream,
                                stderr=subprocess.STDOUT, start_new_session=True)
        try:
            result = proc.wait(timeout=timeout)
        except BaseException:
            import signal
            os.killpg(proc.pid, signal.SIGINT)
            try:
                proc.wait(timeout=60)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL); proc.wait()
            raise
        if result:
            raise RuntimeError(f'Command exited {result}; see {log}')


def source_hashes():
    result = {}
    for folder in ('FAST-LIVO2-ROS2/research/s3e_pipeline', 'FAST-LIVO2-ROS2/scripts/cu_multi148',
                   'ellipselio/src', 'ellipselio/include'):
        for path in sorted((SOURCE/folder).rglob('*')):
            if path.is_file() and '__pycache__' not in path.parts:
                result[str(path.relative_to(SOURCE))] = file_hash(path)
    for name in ('FAST-LIVO2-ROS2/scripts/run_cu_multi.py', 'FAST-LIVO2-ROS2/scripts/run_ellipselio.py',
                 'FAST-LIVO2-ROS2/scripts/run_ellipselio.sh', 'ellipselio/config/os64_ncd.yaml',
                 '.ros2/ellipse-install/ellipselio/lib/libellipselio_mapping.so',
                 '.ros2/ellipsoid-cuda/libellipsoid_surface.so', '.ros2/dpgo-install/cbs/lib/libcbs.so',
                 '.ros2/dpgo-install/cbs_ros/lib/cbs_ros/cbs_ros_node'):
        result[name] = file_hash(SOURCE/name)
    return result


def prepare(args):
    work = args.work
    work.mkdir(parents=True, exist_ok=False)
    cfg = yaml.safe_load((SOURCE/'FAST-LIVO2-ROS2/research/configs/square1-ellipselio-mapclosures-cbs.yaml').read_text())
    cfg.update(dataset=str(args.archives/args.environment), robots=list(ROBOTS), output_root=str(work),
               descriptor_branches=['ellipsoid'], ellipsoid_basis_roundoff_tolerance=.001)
    cfg['backend']['mapclosures']['local_map'] = 'Causal native persistent ellipsoid map; 80 m crop; 0.125 m surface sampling and 0.25 m voxel centroids'
    cfg['dpgo'].update(mode='peers', ros_domain_id=187, timeout_s=1800)
    cfg['dpgo']['pcm']['enabled'] = True
    cfg['evaluation'].update(ground_truth_format='cu_multi_utm', T_reference_body={},
                             gt_orientation_used_for_lever_arm=True,
                             trajectory_limitation='Translation APE against CU-Multi LIO-SAM2 + RTK GPS reference; reference orientation used only for calibrated Ouster-to-LORD IMU lever arm; no scale fit')
    write_json(work/'source-hashes.json', source_hashes())
    info = dict(dataset='CU-Multi', environment=args.environment, robots={},
                duration_s=args.duration, start_offset_s=args.start_offset,
                time_policy='Preserve published synchronized sensor timestamps; no per-robot time rebasing',
                reference_access='Ground truth is read only during evaluation')
    for robot in ROBOTS:
        progress(work, 'extract_sensors', robot=robot)
        cache = args.inputs/args.environment/robot
        lidar = extract_sensor_archive(args.data/args.environment/robot/f'{robot}_{args.environment}_lidar.zip', cache/'lidar')
        imu = extract_sensor_archive(args.archives/args.environment/robot/f'{robot}_{args.environment}_imu_gps.zip', cache/'lord-imu')
        progress(work, 'validate_and_merge_sensors', robot=robot)
        sensors = stage_sensors(lidar, imu, work/'sensor-bags'/robot, robot, args.duration, args.start_offset)
        urdf = args.data/'calib/robot_description/urdf'/f'{robot}.urdf'
        mapping, reference_body = mapping_config(SOURCE, urdf, robot, sensors, cfg['keyframes'])
        (work/f'{robot}.yaml').write_text(yaml.safe_dump(mapping, sort_keys=False))
        cfg['evaluation']['T_reference_body'][robot] = reference_body.tolist()
        info['robots'][robot] = dict(sensors=sensors, urdf=str(urdf), urdf_sha256=file_hash(urdf),
                                    body_frame=robot+'_imu_link', reference_frame=robot+'_os_sensor',
                                    imu_noise_provenance='Initial declared noise model; not estimated from GT or calibrated by the dataset')
        write_json(work/'dataset.json', info)
    (work/'config.yaml').write_text(yaml.safe_dump(cfg, sort_keys=False))
    write_json(work/'PREPARED.json', dict(config_sha256=file_hash(work/'config.yaml'),
                                        source_hashes_sha256=file_hash(work/'source-hashes.json'),
                                        dataset_sha256=file_hash(work/'dataset.json'),
                                        mapping_sha256={r:file_hash(work/f'{r}.yaml') for r in ROBOTS}))
    progress(work, 'prepared')


def run(work):
    from frontend_quality import check_export
    from s3e_pipeline.data import select
    cfg = yaml.safe_load((work/'config.yaml').read_text()); info = read_json(work/'dataset.json')
    prepared = read_json(work/'PREPARED.json')
    for filename, key in [('config.yaml','config_sha256'), ('dataset.json','dataset_sha256'),
                          ('source-hashes.json','source_hashes_sha256')]:
        if file_hash(work/filename) != prepared[key]:
            raise ValueError(f'Prepared input changed: {filename}')
    if source_hashes() != read_json(work/'source-hashes.json'):
        raise ValueError('Source changed after preparation; create a fresh attempt')
    schedules = {}
    for robot in cfg['robots']:
        if file_hash(work/f'{robot}.yaml') != prepared['mapping_sha256'][robot]:
            raise ValueError('Prepared mapping configuration changed')
        sensor_bag = work/'sensor-bags'/robot
        marker = read_json(sensor_bag/'COMPLETE.json')
        if any(file_hash(sensor_bag/name) != sha for name, sha in marker['files'].items()):
            raise ValueError('Prepared sensor bag changed')
        progress(work, 'odometry', robot=robot)
        command = ['bash', SOURCE/'FAST-LIVO2-ROS2/scripts/run_ellipselio.sh', '--robot', robot,
                   '--bag', work/'sensor-bags'/robot, '--output', work/robot,
                   '--mapping-config', work/f'{robot}.yaml', '--rate', '1.0']
        # Staging already bounds the interval. Keep this label in the native
        # replay/report so a smoke test cannot be mistaken for a full run.
        if info['duration_s']:
            command += ['--duration', info['duration_s']]
        metadata = yaml.safe_load((work/'sensor-bags'/robot/'metadata.yaml').read_text())['rosbag2_bagfile_information']
        call(command, work/f'{robot}-odometry.log', metadata['duration']['nanoseconds']/1e9*2+240)
        quality = check_export(work/robot/'export')
        quality['purpose'] = 'CU-Multi ground-robot sanity bound; sensor-only, not an accuracy metric'
        write_json(work/robot/'quality.json', quality)
        if not quality['passed']:
            raise RuntimeError(f'{robot} frontend failed: {quality}')
        selected = []; last = None
        for row in read_jsonl(work/robot/'export/frames.jsonl'):
            wanted = select(row, last, cfg['keyframes'])
            if wanted != ('ellipsoid_delta' in row):
                raise ValueError('Native/Python keyframe selection mismatch')
            if wanted:
                if row['ellipsoids']['requested_stamps_ns'] != [row['stamp_ns']]:
                    raise ValueError('Ellipsoid snapshot timestamp mismatch')
                selected.append(dict(keyframe_id=len(selected), stamp_ns=row['stamp_ns'])); last = row
        if not selected:
            raise ValueError('No native ellipsoid keyframes')
        schedules[robot] = selected
        link = work/'odometry'/robot; link.mkdir(parents=True)
        (link/'run').symlink_to(work/robot, target_is_directory=True)
    write_json(work/'schedule.json', dict(schedule=schedules, selection=cfg['keyframes'], ground_truth_used=False))
    for robot in cfg['robots']:
        progress(work, 'ellipsoid_descriptors', robot=robot)
        call([sys.executable, '-m', 's3e_pipeline.ellipsoid_full', 'prepare', '--work', work, '--robot', robot],
             work/f'{robot}-descriptors.log', 7200)
    progress(work, 'distributed_pcm_cbs')
    call([sys.executable, '-m', 's3e_pipeline.ellipsoid_cbs', 'solve', '--work', work], work/'cbs.log', 2100)
    progress(work, 'evo_maps_report')
    call([sys.executable, '-m', 's3e_pipeline.ellipsoid_cbs', 'evaluate', '--work', work], work/'evaluation.log', 3600)
    progress(work, 'complete')


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--stage', choices=['prepare', 'run', 'all'], default='all')
    p.add_argument('--data', type=Path, default=Path('/data/cu-multi'))
    p.add_argument('--archives', type=Path, default=Path('/data/cu-multi-archives'))
    p.add_argument('--inputs', type=Path, default=SOURCE/'.ros2/cu-multi/inputs')
    p.add_argument('--environment', choices=['main_campus', 'kittredge_loop'], default='main_campus')
    p.add_argument('--work', type=Path, required=True)
    p.add_argument('--duration', type=float, default=120., help='Bounded smoke interval; 0 selects the full sequence')
    p.add_argument('--start-offset', type=float, default=0.)
    args = p.parse_args(); args.work = args.work.resolve()
    owns_attempt = args.stage == 'run' or not args.work.exists()
    if any(not math.isfinite(x) or x < 0 for x in (args.duration, args.start_offset)):
        p.error('Replay duration and offset must be finite and nonnegative')
    os.environ['ROS_DOMAIN_ID'] = '187'
    try:
        if args.stage in ('prepare', 'all'):
            prepare(args)
        if args.stage in ('run', 'all'):
            run(args.work)
    except BaseException as exc:
        if owns_attempt and args.work.exists():
            write_json(args.work/'failure.json', dict(error=str(exc), traceback=traceback.format_exc(),
                                                     progress=read_json(args.work/'progress.json') if (args.work/'progress.json').exists() else None))
        raise


if __name__ == '__main__':
    main()
