#!/usr/bin/env python3
"""Run one S3E robot, wait for mapper readiness, replay the bag, save evidence."""
import argparse
from datetime import datetime
import json
import math
import os
import re
from pathlib import Path
import shutil
import signal
import socket
import subprocess
import time
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.utilities import get_rmw_implementation_identifier
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2, Image


def stop(proc, timeout=15):
    if proc is None or proc.poll() is not None:
        return
    # ros2 launch forwards SIGINT to its children itself. Signalling the whole
    # group here sends it twice and can interrupt otherwise clean shutdown.
    proc.send_signal(signal.SIGINT)
    try:
        proc.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        os.killpg(proc.pid, signal.SIGTERM)
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)
            proc.wait()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--bag', type=Path, default=Path('/data/s3e/S3E_Square_1'))
    p.add_argument('--robot', choices=['Alpha', 'Bob', 'Carol'], default='Alpha')
    p.add_argument('--rate', type=float, default=1.0)
    p.add_argument('--start-offset', type=float, default=0.0,
                   help='Start this many seconds into the bag; preserve sensor timestamps')
    p.add_argument('--duration', type=float, default=0.0, help='Bag seconds to replay; 0 = entire bag')
    p.add_argument('--rviz', action='store_true')
    p.add_argument('--rerun', action='store_true', help='Use the official Rerun 0.37.1 MCAP integration')
    p.add_argument('--rerun-headless', action='store_true', help='Record Rerun/MCAP without opening a viewer')
    p.add_argument('--rerun-port', type=int, default=9877)
    p.add_argument('--gdb', action='store_true', help='Capture a native crash backtrace in mapping.log')
    p.add_argument('--output', type=Path, help='Explicit fresh output directory for an experiment stage')
    p.add_argument('--mapping-config', type=Path,
                   help='Override the robot mapping YAML; save the exact configuration in the run')
    p.add_argument('--namespace', default='')
    p.add_argument('--export', action='store_true', help='Direct synchronized MCAP export')
    p.add_argument('--export-python', type=Path)
    args = p.parse_args()
    if args.gdb:
        os.environ['FAST_LIVO_GDB'] = '1'
    if not all(math.isfinite(v) for v in (args.rate, args.duration, args.start_offset)) or args.rate <= 0 or args.duration < 0 or args.start_offset < 0:
        p.error('rate must be positive; duration and start offset must be finite and nonnegative')
    if not (args.bag / 'metadata.yaml').is_file():
        p.error(f'No metadata.yaml in {args.bag}')
    if args.mapping_config and not args.mapping_config.is_file():
        p.error(f'No mapping configuration at {args.mapping_config}')
    source = Path(__file__).resolve().parents[2]
    use_rerun = args.rerun or args.rerun_headless
    rerun_bin = source / '.ros2/rerun-venv/bin'
    if use_rerun and not (rerun_bin / 'rerun').is_file():
        p.error('Install Rerun first: FAST-LIVO2-ROS2/scripts/setup_rerun.sh')
    seq = f'{args.bag.resolve().name}_{args.robot.lower()}_{datetime.now():%Y%m%d_%H%M%S_%f}'
    output = args.output.resolve() if args.output else source / '.ros2/runs' / seq
    output.mkdir(parents=True)
    config_dir = source / 'FAST-LIVO2-ROS2/config/s3e'
    mapping_config = args.mapping_config or config_dir / f'{args.robot.lower()}.yaml'
    shutil.copy2(mapping_config, output / 'mapping_config.yaml')
    shutil.copy2(config_dir / f'{args.robot.lower()}_camera.yaml', output / 'camera_config.yaml')
    # Upstream writes its debug and trajectory files below the source directory.
    for directory in ['result', 'pcd', 'image']:
        (source / 'FAST-LIVO2-ROS2/Log' / directory).mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = rclpy.create_node('s3e_run_monitor')
    stats = dict(robot=args.robot, bag=str(args.bag), rate=args.rate, requested_duration=args.duration,
                 start_offset_s=args.start_offset,
                 rmw_implementation=get_rmw_implementation_identifier(),
                 odometry_messages=0, cloud_messages=0, visual_messages=0, finite=True,
                 first_pose_stamp=None, last_pose_stamp=None, first_clock=None, last_clock=None,
                 max_pose_step_m=0.0, path_length_m=0.0)
    previous = None
    last_progress = time.monotonic()

    def odometry(msg):
        nonlocal previous
        q, v = msg.pose.pose.orientation, msg.pose.pose.position
        stats['finite'] &= all(math.isfinite(x) for x in [v.x,v.y,v.z,q.x,q.y,q.z,q.w])
        xyz = (v.x, v.y, v.z)
        if previous is not None:
            step = math.dist(xyz, previous)
            stats['max_pose_step_m'] = max(stats['max_pose_step_m'], step)
            stats['path_length_m'] += step
        previous = xyz
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        if stats['first_pose_stamp'] is None:
            stats['first_pose_stamp'] = stamp
        stats['last_pose_stamp'] = stamp
        stats['odometry_messages'] += 1

    def clock(msg):
        nonlocal last_progress
        stamp = msg.clock.sec + msg.clock.nanosec / 1e9
        if stats['first_clock'] is None:
            stats['first_clock'] = stamp
        if stamp != stats['last_clock']:
            last_progress = time.monotonic()
        stats['last_clock'] = stamp

    def count(key):
        def callback(_):
            stats[key] += 1
        return callback

    prefix = '/' + args.namespace.strip('/') if args.namespace.strip('/') else ''
    node.create_subscription(Odometry, prefix + '/aft_mapped_to_init', odometry, 100)
    node.create_subscription(Clock, '/clock', clock, qos_profile_sensor_data)
    node.create_subscription(PointCloud2, prefix + '/cloud_registered', count('cloud_messages'), 10)
    node.create_subscription(Image, prefix + '/rgb_img', count('visual_messages'), 10)
    launch_proc = player = bridge = viewer = None
    extra_logs = []

    def save_processes():
        (output / 'processes.json').write_text(json.dumps(dict(
            runner=os.getpid(), mapper_launch=launch_proc.pid if launch_proc else None,
            player=player.pid if player else None, bridge=bridge.pid if bridge else None,
            viewer=viewer.pid if viewer else None), indent=2) + '\n')

    error = None
    try:
        with (output / 'mapping.log').open('w') as mapping_log, (output / 'playback.log').open('w') as playback_log:
            if use_rerun:
                bridge_args = [str(rerun_bin / 'python'), str(source / 'FAST-LIVO2-ROS2/scripts/s3e_rerun.py'),
                               '--robot', args.robot, '--output', str(output)]
                if not args.rerun_headless:
                    viewer_log = (output / 'rerun_viewer.log').open('w')
                    extra_logs.append(viewer_log)
                    with socket.socket() as port_check:
                        port_check.bind(('127.0.0.1', args.rerun_port))
                    viewer = subprocess.Popen([str(rerun_bin / 'rerun'), '--bind', '127.0.0.1',
                        '--port', str(args.rerun_port), '--memory-limit', '8GB', '--threads', '4',
                        '--window-size', '1600x950', '--hide-welcome-screen'],
                        stdout=viewer_log, stderr=subprocess.STDOUT, start_new_session=True)
                    bridge_args += ['--url', f'rerun+http://127.0.0.1:{args.rerun_port}/proxy']
                bridge_log = (output / 'rerun_bridge.log').open('w')
                extra_logs.append(bridge_log)
                bridge = subprocess.Popen(bridge_args, stdout=bridge_log, stderr=subprocess.STDOUT,
                                          start_new_session=True)
                save_processes()
                deadline = time.monotonic() + 45
                while not (output / 'rerun_ready').exists():
                    if bridge.poll() is not None or (viewer and viewer.poll() is not None) or time.monotonic() > deadline:
                        raise RuntimeError(f'Rerun did not become ready; inspect logs in {output}')
                    rclpy.spin_once(node, timeout_sec=0.1)
            launch_proc = subprocess.Popen(['ros2','launch','fast_livo','mapping_s3e.launch.py',
                f'robot:={args.robot}', f'use_rviz:={str(args.rviz).lower()}',
                f'use_rerun:={str(use_rerun).lower()}', f'seq_name:={seq}',
                f'namespace:={args.namespace}',
                f'mapping_config:={output / "mapping_config.yaml"}',
                f'output_directory:={output / "mapper" if args.export else ""}',
                f'export_directory:={output / "export" if args.export else ""}',
                f'export_python:={args.export_python or source / ".ros2/research-venv/bin/python"}',
                f'export_writer:={source / "FAST-LIVO2-ROS2/research/s3e_pipeline/mcap_writer.py"}'],
                stdout=mapping_log, stderr=subprocess.STDOUT, start_new_session=True)
            print(f'Run directory: {output}', flush=True)
            deadline = time.monotonic() + 60
            topics = [f'/{args.robot}/velodyne_points_start', f'/{args.robot}/left_camera/image', f'/{args.robot}/imu/data']
            save_processes()
            while not all(node.count_subscribers(t) for t in topics):
                if launch_proc.poll() is not None or time.monotonic() > deadline:
                    raise RuntimeError(f'Mapper did not become ready; see {output / "mapping.log"}')
                rclpy.spin_once(node, timeout_sec=0.1)
            playback_topics = [f'/{args.robot}/velodyne_points', f'/{args.robot}/imu/data',
                               f'/{args.robot}/left_camera/compressed']
            if use_rerun:
                playback_topics.append(f'/{args.robot}/fix')
            player = subprocess.Popen(['ros2','bag','play',str(args.bag), '--clock','100',
                '--rate',str(args.rate),'--read-ahead-queue-size','1000','--disable-keyboard-controls',
                '--start-offset',str(args.start_offset),
                '--topics', *playback_topics],
                stdout=playback_log, stderr=subprocess.STDOUT, start_new_session=True)
            save_processes()
            print(f'Mapper ready; replaying {args.robot} at {args.rate}x from bag offset {args.start_offset:g}s', flush=True)
            last_progress = time.monotonic()
            next_report = last_progress + 30
            while player.poll() is None:
                rclpy.spin_once(node, timeout_sec=0.1)
                if bridge and bridge.poll() is not None:
                    raise RuntimeError('Rerun bridge exited during playback; inspect rerun_bridge.log')
                if viewer and viewer.poll() is not None:
                    raise RuntimeError('Rerun viewer was closed')
                if launch_proc.poll() is not None or not all(node.count_subscribers(t) for t in topics):
                    raise RuntimeError('Mapping node exited during playback; inspect mapping.log')
                if not stats['finite']:
                    raise RuntimeError('Non-finite odometry')
                if time.monotonic() - last_progress > 60:
                    raise RuntimeError('No playback clock progress for 60 seconds')
                elapsed = (stats['last_clock'] or 0) - (stats['first_clock'] or 0)
                if time.monotonic() >= next_report:
                    print(f'{elapsed:.1f}s bag time; {stats["odometry_messages"]} poses, '
                          f'{stats["visual_messages"]} visual frames', flush=True)
                    next_report = time.monotonic() + 30
                if args.duration and elapsed >= args.duration:
                    stats['stopped_at_duration'] = True
                    stop(player)
                    break
            if player.returncode and not stats.get('stopped_at_duration'):
                raise RuntimeError(f'Bag player exited with {player.returncode}')
            # Let callbacks already received by the mapper finish.
            drain_until = time.monotonic() + (120 if args.export else 5)
            drain_last_count = stats['odometry_messages']
            drain_last_progress = time.monotonic()
            while time.monotonic() < drain_until:
                rclpy.spin_once(node, timeout_sec=0.1)
                if stats['odometry_messages'] != drain_last_count:
                    drain_last_count = stats['odometry_messages']
                    drain_last_progress = time.monotonic()
                if args.export and time.monotonic() - drain_last_progress > 5:
                    break
            if stats['odometry_messages'] < 10 or stats['cloud_messages'] == 0 or stats['visual_messages'] == 0:
                raise RuntimeError('Replay produced insufficient odometry/map/visual output')
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        error = 'Interrupted'
    except Exception as exc:
        error = str(exc)
    finally:
        stop(player)
        stop(launch_proc, timeout=180 if args.export else 15)
        stop(bridge)
        if error:
            stop(viewer)
        if bridge and bridge.returncode not in (0, -2):
            error = error or f'Rerun bridge exited with {bridge.returncode}; inspect rerun_bridge.log'
        if use_rerun:
            stats['rerun_version'] = '0.37.1'
            stats['visualization_recording'] = str(output / 'visualization.rrd')
            stats['ros2_mcap_recording'] = str(output / 'visualization.mcap')
            stats['rerun_viewer_exit_code'] = viewer.poll() if viewer else None
            stats['rerun_bridge_exit_code'] = bridge.returncode if bridge else None
        for log_file in extra_logs:
            log_file.close()
        trajectory = (output / 'mapper/Log/result' if args.export else source / 'FAST-LIVO2-ROS2/Log/result') / f'{seq}.txt'
        if trajectory.exists():
            shutil.copy2(trajectory, output / 'trajectory.tum')
        mapping_log_path = output / 'mapping.log'
        if mapping_log_path.exists():
            log_text = mapping_log_path.read_text()
            fatal_exits = [int(code) for code in re.findall(r'process has died.*exit code (-?\d+)', log_text)
                           if int(code) not in (0, -2, -15)]
            if fatal_exits or re.search(r'received signal SIG(?:SEGV|ABRT|BUS)', log_text):
                error = f'Child process crashed; see {mapping_log_path}'
            counts = re.search(r'S3E adapter received (\d+) clouds and (\d+) images', log_text)
            if counts:
                stats['input_clouds'] = int(counts[1])
                stats['input_images'] = int(counts[2])
        if args.export:
            exported = output / 'export/manifest.json'
            if not exported.is_file():
                error = error or 'Direct export did not complete'
            else:
                export_stats = json.loads(exported.read_text())
                stats['exported_frames'] = export_stats['frames']
                stats['export_complete'] = export_stats['complete']
        stats['namespace'] = args.namespace
        stats['error'] = error
        stats['success'] = error is None
        (output / 'summary.json').write_text(json.dumps(stats, indent=2) + '\n')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    print(json.dumps(stats, indent=2), flush=True)
    if error:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
