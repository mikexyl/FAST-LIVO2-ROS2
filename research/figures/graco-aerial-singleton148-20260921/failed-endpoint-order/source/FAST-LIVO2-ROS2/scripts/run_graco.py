#!/usr/bin/env python3
"""One fixed six-session GRACO ground experiment with distributed PCM/CBS."""
import argparse
import os
from pathlib import Path
import shutil
import sys
import traceback

import yaml

SOURCE = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(SOURCE/'FAST-LIVO2-ROS2/research'))
sys.path.insert(0, str(SOURCE/'Swarm-SLAM/s3e'))
from s3e_pipeline.artifacts import file_hash, read_json, read_jsonl, write_json
from s3e_pipeline.graco import ROBOTS, stage_sensors, mapping_config, connectivity
from run_cu_multi import call, progress


def pack(args):
    args.inputs.mkdir(parents=True, exist_ok=True)
    for i, robot in enumerate(ROBOTS, 1):
        bag = args.data/(f'ground-{i:02d}'+('_ros2' if i>2 else ''))
        progress(args.inputs, 'copy_sensor_topics', robot=robot)
        stage_sensors(bag, args.inputs/'sensors'/robot)
    shutil.copytree(args.data/'ground-calibration', args.inputs/'calibration', dirs_exist_ok=True)
    gt = args.inputs/'reference'; gt.mkdir(exist_ok=True)
    for i, robot in enumerate(ROBOTS, 1):
        shutil.copy2(args.data/f'ground-{i:02d}.txt', gt/f'{robot}_gt.txt')
    progress(args.inputs, 'complete')


def source_hashes():
    hashes = {}
    for folder in ('FAST-LIVO2-ROS2/research/s3e_pipeline', 'ellipselio/src', 'ellipselio/include'):
        for p in sorted((SOURCE/folder).rglob('*')):
            if p.is_file() and '__pycache__' not in p.parts:
                hashes[str(p.relative_to(SOURCE))] = file_hash(p)
    for name in ('FAST-LIVO2-ROS2/scripts/run_graco.py', 'FAST-LIVO2-ROS2/scripts/run_cu_multi.py',
                 'FAST-LIVO2-ROS2/scripts/run_ellipselio.py', 'FAST-LIVO2-ROS2/scripts/run_ellipselio.sh',
                 'ellipselio/config/vlp16_graco.yaml', '.ros2/ellipse-install/ellipselio/lib/libellipselio_mapping.so',
                 '.ros2/ellipsoid-cuda/libellipsoid_surface.so', '.ros2/dpgo-install/cbs/lib/libcbs.so',
                 '.ros2/dpgo-install/cbs_ros/lib/cbs_ros/cbs_ros_node'):
        hashes[name] = file_hash(SOURCE/name)
    for module in ('s3e_mapclosures_native', 's3e_mapclosures_inspection'):
        for p in (SOURCE/'.ros2/research-venv/lib/python3.10/site-packages').glob(module+'*.so'):
            hashes[str(p.relative_to(SOURCE))] = file_hash(p)
    return hashes


def experiment(args):
    from frontend_quality import check_export
    from s3e_pipeline.data import select
    work = args.work.resolve(); work.mkdir(parents=True, exist_ok=False)
    cfg = yaml.safe_load((SOURCE/'FAST-LIVO2-ROS2/research/configs/square1-ellipselio-mapclosures-cbs.yaml').read_text())
    cfg.update(dataset=str(args.inputs/'reference'), robots=list(ROBOTS), output_root=str(work),
               descriptor_branches=['ellipsoid'], ellipsoid_basis_roundoff_tolerance=.001)
    cfg['backend']['mapclosures']['local_map'] = 'Causal persistent native ellipsoid map; 80 m crop; 0.125 m surface sampling and 0.25 m voxel centroids'
    cfg['dpgo'].update(mode='peers', ros_domain_id=190, timeout_s=1800)
    cfg['dpgo']['pcm']['enabled'] = True
    cfg['evaluation'].update(ground_truth_format='graco_imu_enu', expected_connected_robots=list(ROBOTS),
                             trajectory_limitation='GRACO RTK/INS T_Base_Imu reference; original timestamps; shared rigid alignment without scale')
    (work/'config.yaml').write_text(yaml.safe_dump(cfg, sort_keys=False))
    write_json(work/'source-hashes.json', source_hashes())
    info = dict(dataset='GRACO ground-01..06', robots={}, duration_s=args.duration,
                calibration={p.name:file_hash(p) for p in (args.inputs/'calibration').iterdir() if p.is_file()},
                expected_components=1, time_policy='Six distinct recording sessions; preserve absolute timestamps, chronological causal retrieval, no time rebasing',
                reference_access='Ground truth is read only during evaluation')
    schedules = {}
    for i, robot in enumerate(ROBOTS, 1):
        bag = args.inputs/'sensors'/robot
        progress(work, 'validate_sensors', robot=robot)
        marker = read_json(bag/'COMPLETE.json')
        if any(file_hash(bag/n) != h for n,h in marker['files'].items()):
            raise ValueError(f'{robot} staged sensor hash mismatch')
        mapping = mapping_config(SOURCE, args.inputs/'calibration', robot, cfg['keyframes'])
        (work/f'{robot}.yaml').write_text(yaml.safe_dump(mapping, sort_keys=False))
        info['robots'][robot] = dict(sequence=f'ground-{i:02d}', sensors=read_json(bag/'SENSORS.json'),
                                    mapping_sha256=file_hash(work/f'{robot}.yaml'))
        write_json(work/'dataset.json', info)
        progress(work, 'odometry', robot=robot)
        metadata = yaml.safe_load((bag/'metadata.yaml').read_text())['rosbag2_bagfile_information']
        call(['bash', SOURCE/'FAST-LIVO2-ROS2/scripts/run_ellipselio.sh', '--robot', robot, '--bag', bag,
              '--output', work/robot, '--mapping-config', work/f'{robot}.yaml', '--rate', '1', '--duration', args.duration],
             work/f'{robot}-odometry.log', (args.duration or metadata['duration']['nanoseconds']/1e9)*2+240)
        quality = check_export(work/robot/'export'); quality['purpose'] = 'Sensor-only GRACO ground-robot sanity check; not an accuracy metric'
        write_json(work/robot/'quality.json', quality)
        if not quality['passed']:
            raise RuntimeError(f'{robot} odometry failed validity: {quality}')
        summary = read_json(work/robot/'summary.json')
        if summary.get('analytics_messages', 0) < 1:
            raise RuntimeError(f'{robot} native estimator analytics were not recorded')
        last = None; selected = []
        for row in read_jsonl(work/robot/'export/frames.jsonl'):
            wanted = select(row, last, cfg['keyframes'])
            if wanted != ('ellipsoid_delta' in row):
                raise ValueError('Native/Python keyframe selection mismatch')
            if wanted:
                if row['ellipsoids']['requested_stamps_ns'] != [row['stamp_ns']]:
                    raise ValueError('Ellipsoid snapshot timestamp mismatch')
                selected.append(dict(keyframe_id=len(selected), stamp_ns=row['stamp_ns'])); last = row
        if not selected: raise ValueError('No native ellipsoid keyframes')
        schedules[robot] = selected
        link = work/'odometry'/robot; link.mkdir(parents=True)
        (link/'run').symlink_to(work/robot, target_is_directory=True)
    write_json(work/'schedule.json', dict(schedule=schedules, selection=cfg['keyframes'], ground_truth_used=False))
    for robot in ROBOTS:
        progress(work, 'ellipsoid_descriptors', robot=robot)
        call([sys.executable, '-m', 's3e_pipeline.ellipsoid_full', 'prepare', '--work', work, '--robot', robot],
             work/f'{robot}-descriptors.log', 3600)
    progress(work, 'distributed_pcm_cbs')
    call([sys.executable, '-m', 's3e_pipeline.ellipsoid_cbs', 'solve', '--work', work], work/'cbs.log', 2100)
    dpgo = Path(read_json(work/'artifacts.json')['dpgo'])
    connected = connectivity(ROBOTS, read_jsonl(dpgo/'constraints.jsonl'), read_jsonl(dpgo/'poses.jsonl'))
    write_json(work/'connectivity.json', connected)
    progress(work, 'evo_maps_report', all_six_connected=connected['all_six_connected'])
    call([sys.executable, '-m', 's3e_pipeline.ellipsoid_cbs', 'evaluate', '--work', work], work/'evaluation.log', 3600)
    shutil.copy2(work/'connectivity.json', work/'report/connectivity.json')
    with (work/'report/REPORT.md').open('a') as stream:
        stream.write('\n## Six-robot group connectivity\n\n')
        stream.write(f'Expected one connected component across ground-01..06. Measured components: `{connected["measured_components"]}`. ')
        stream.write(f'All six connected in a consistent CBS frame: **{connected["all_six_connected"]}**. No edges or poses were imposed from GT.\n')
    progress(work, 'complete', all_six_connected=connected['all_six_connected'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--stage', choices=['pack','run'], required=True)
    parser.add_argument('--data', type=Path, default=Path('/data/graco'))
    parser.add_argument('--inputs', type=Path, required=True)
    parser.add_argument('--work', type=Path)
    parser.add_argument('--duration', type=float, default=0., help='0 for full sessions; positive value is a smoke interval per robot')
    args = parser.parse_args()
    import math
    if not math.isfinite(args.duration) or args.duration<0: parser.error('Invalid duration')
    if args.stage=='run' and args.work is None: parser.error('--work is required for run')
    os.environ['ROS_DOMAIN_ID']='190'
    owns_work = args.work is not None and not args.work.exists()
    try:
        pack(args) if args.stage=='pack' else experiment(args)
    except BaseException as exc:
        if owns_work and args.work.exists():
            write_json(args.work/'failure.json', dict(error=str(exc), traceback=traceback.format_exc()))
            progress(args.work, 'failed', error=str(exc))
        raise


if __name__ == '__main__': main()
