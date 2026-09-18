#!/usr/bin/env python3
"""Bounded one-robot replay with direct scan/state exports, no camera input."""
import argparse
import json
import math
import re
from pathlib import Path
import shutil
import signal
import subprocess
import time

import rclpy
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from std_srvs.srv import Trigger
import yaml
from run_s3e import stop


def main():
    p=argparse.ArgumentParser();p.add_argument('--robot',required=True)
    p.add_argument('--bag',required=True,type=Path);p.add_argument('--output',required=True,type=Path)
    p.add_argument('--mapping-config',required=True,type=Path);p.add_argument('--rate',type=float,default=1.)
    p.add_argument('--mapper-executable',type=Path,help='Optional isolated launcher; the default native executable is unchanged')
    p.add_argument('--no-research-export',action='store_true',help='Run native ROS publishers only, without synchronized research export or its completion service')
    p.add_argument('--start-offset',type=float,default=0.);p.add_argument('--duration',type=float,default=0.)
    args=p.parse_args()
    if not re.fullmatch(r'[A-Za-z][A-Za-z0-9_]*',args.robot):p.error('Invalid robot identifier')
    if not all(math.isfinite(x) for x in (args.rate,args.start_offset,args.duration)) or args.rate<=0 or min(args.start_offset,args.duration)<0:
        raise ValueError('Invalid replay timing')
    source=Path(__file__).resolve().parents[2];out=args.output.resolve();out.mkdir(parents=True)
    mapper_executable=(args.mapper_executable or source/'.ros2/ellipse-install/ellipselio/lib/ellipselio/ellipselio_mapping_node').resolve()
    shutil.copy2(args.mapping_config,out/'mapping_config.yaml')
    config=yaml.safe_load(args.mapping_config.read_text());params=config['/**']['ros__parameters']
    params['use_sim_time']=True
    research_options=params.get('research',{})
    writer_name='ellipsoid_delta_writer.py' if research_options.pop('ellipsoid_delta_export',False) else 'mcap_writer.py'
    params['research']=dict(research_options,enabled=not args.no_research_export,robot=args.robot,python=str(source/'.ros2/research-venv/bin/python'),
        writer=str(source/'FAST-LIVO2-ROS2/research/s3e_pipeline'/writer_name),output=str(out/'export'))
    (out/'runtime.yaml').write_text(yaml.safe_dump(config,sort_keys=False))
    metadata=yaml.safe_load((args.bag/'metadata.yaml').read_text())['rosbag2_bagfile_information']
    start_ns=metadata['starting_time']['nanoseconds_since_epoch']+round(args.start_offset*1e9)
    rclpy.init();node=rclpy.create_node(f'{args.robot.lower()}_ellipse_monitor')
    clock_pub=node.create_publisher(Clock,'/clock',10)
    stats=dict(frontend='ellipselio',robot=args.robot,bag=str(args.bag.resolve()),rate=args.rate,mapper_executable=str(mapper_executable),
        start_offset_s=args.start_offset,requested_duration=args.duration,research_export_enabled=not args.no_research_export,odometry_messages=0,
        finite=True,path_length_m=0.,max_pose_step_m=0.,first_pose_stamp_ns=None,last_pose_stamp_ns=None)
    previous=None;last_clock=None
    def odometry(msg):
        nonlocal previous
        v=msg.pose.pose.position;q=msg.pose.pose.orientation;xyz=(v.x,v.y,v.z)
        stats['finite'] &= all(math.isfinite(x) for x in (*xyz,q.x,q.y,q.z,q.w))
        if previous is not None:
            step=math.dist(xyz,previous);stats['path_length_m']+=step;stats['max_pose_step_m']=max(stats['max_pose_step_m'],step)
        previous=xyz;stats['odometry_messages']+=1
        stamp=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        if stats['first_pose_stamp_ns'] is None:stats['first_pose_stamp_ns']=stamp
        stats['last_pose_stamp_ns']=stamp
    def clock(msg):
        nonlocal last_clock
        last_clock=msg.clock.sec*10**9+msg.clock.nanosec
    node.create_subscription(Odometry,f'/{args.robot}/ellipselio_odom',odometry,rclpy.qos.qos_profile_sensor_data)
    node.create_subscription(Clock,'/clock',clock,rclpy.qos.qos_profile_sensor_data)
    analytics_stream=None
    if params.get('publish',{}).get('analytics',False):
        from ellipselio.msg import EllipseLioAnalytics
        from rosidl_runtime_py.convert import message_to_ordereddict
        analytics_stream=(out/'analytics.jsonl').open('w')
        stats['analytics_messages']=0
        stats['analytics_time_policy']='Native message has no header; capture records latest observed /clock and odometry stamp, not an exact scan association'
        def analytics(msg):
            values=message_to_ordereddict(msg)
            nonfinite=[k for k,v in values.items() if isinstance(v,float) and not math.isfinite(v)]
            for k in nonfinite:values[k]=None
            record=dict(observed_clock_ns=last_clock,last_observed_odometry_stamp_ns=stats['last_pose_stamp_ns'],
                        native=values,nonfinite_fields=nonfinite)
            analytics_stream.write(json.dumps(record,allow_nan=False)+'\n')
            stats['analytics_messages']+=1
            if stats['analytics_messages']%100==0:analytics_stream.flush()
        node.create_subscription(EllipseLioAnalytics,f'/{args.robot}/analytics',analytics,rclpy.qos.qos_profile_sensor_data)
    mapper=None;player=None;error=None;begin=time.monotonic()
    try:
        with (out/'mapping.log').open('w') as mapping_log,(out/'playback.log').open('w') as playback_log:
            mapper=subprocess.Popen([str(mapper_executable),
                '--ros-args','-r',f'__node:={args.robot.lower()}_ellipselio','--params-file',str(out/'runtime.yaml')],
                stdout=mapping_log,stderr=subprocess.STDOUT,start_new_session=True)
            # Advance a pre-bag clock until the node's initialization timer creates subscribers.
            deadline=time.monotonic()+60
            while not all(node.count_subscribers(t) for t in (params['lidar']['topic'],params['imu']['topic'])):
                if mapper.poll() is not None or time.monotonic()>deadline:raise RuntimeError('EllipseLIO did not initialize; inspect mapping.log')
                ns=start_ns-60*10**9+round((time.monotonic()-begin)*1e9)
                msg=Clock();msg.clock.sec,msg.clock.nanosec=divmod(ns,10**9);clock_pub.publish(msg)
                rclpy.spin_once(node,timeout_sec=.05)
            player=subprocess.Popen(['ros2','bag','play',str(args.bag),'--clock','100','--rate',str(args.rate),
                '--start-offset',str(args.start_offset),'--read-ahead-queue-size','1000','--disable-keyboard-controls',
                '--topics',params['lidar']['topic'],params['imu']['topic']],
                stdout=playback_log,stderr=subprocess.STDOUT,start_new_session=True)
            (out/'processes.json').write_text(json.dumps(dict(mapper=mapper.pid,player=player.pid)))
            replay_start=time.monotonic();next_report=replay_start;bounded_stop=False
            max_wall=(args.duration or metadata['duration']['nanoseconds']/1e9)/args.rate+180
            while player.poll() is None:
                rclpy.spin_once(node,timeout_sec=.1)
                if mapper.poll() is not None:raise RuntimeError('EllipseLIO exited during replay')
                if not stats['finite']:raise RuntimeError('Non-finite EllipseLIO state')
                if time.monotonic()-replay_start>max_wall:raise TimeoutError('EllipseLIO replay exceeded time budget')
                elapsed=max(0.,((last_clock or start_ns)-start_ns)/1e9)
                if time.monotonic()>=next_report:
                    print(f'{args.robot}: {elapsed:.1f}s bag time, {stats["odometry_messages"]} IMU-rate poses',flush=True)
                    (out/'progress.json').write_text(json.dumps(dict(stats,elapsed_bag_s=elapsed)))
                    next_report=time.monotonic()+30
                if args.duration and elapsed>=args.duration:bounded_stop=True;stop(player);break
            if player.returncode not in (0,-signal.SIGINT) and not bounded_stop:raise RuntimeError(f'Bag player failed: {player.returncode}; inspect playback.log')
            drain=time.monotonic()+3
            while time.monotonic()<drain:rclpy.spin_once(node,timeout_sec=.1)
            if stats['odometry_messages']<10:raise RuntimeError('Insufficient EllipseLIO output')
            if not args.no_research_export:
                finish=node.create_client(Trigger,f'/{args.robot}/research/finish')
                if not finish.wait_for_service(timeout_sec=5):raise RuntimeError('Missing export completion service')
                future=finish.call_async(Trigger.Request())
                rclpy.spin_until_future_complete(node,future,timeout_sec=60)
                if not future.done() or not future.result().success:raise RuntimeError('Native export failed to finish')
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):error='Interrupted'
    except Exception as exc:error=str(exc)
    finally:
        stop(player);stop(mapper,timeout=60)
        if analytics_stream is not None:analytics_stream.close()
        manifest=out/'export/manifest.json'
        if args.no_research_export:stats['export_complete']=None
        elif not manifest.exists():error=error or 'Synchronized export did not complete'
        else:
            exported=json.loads(manifest.read_text());stats.update(exported_frames=exported['frames'],export_complete=exported['complete'])
            if not exported['complete']:error=error or 'Incomplete export'
            rows=[json.loads(line) for line in (out/'export/frames.jsonl').read_text().splitlines()]
            stamps=[row['stamp_ns'] for row in rows]
            gaps=[(b-a)/1e9 for a,b in zip(stamps,stamps[1:])]
            stats['synchronized_export']=dict(first_stamp_ns=stamps[0],last_stamp_ns=stamps[-1],
                span_s=(stamps[-1]-stamps[0])/1e9,max_gap_s=max(gaps,default=0),
                gaps_over_200ms=sum(g>.2 for g in gaps),lidar_updates=sum(row['lidar_updated'] for row in rows),
                min_cloud_points=min(row['cloud_points'] for row in rows),
                max_cloud_points=max(row['cloud_points'] for row in rows),
                image_frames=sum(row['image_available'] for row in rows))
            if any(b<=a for a,b in zip(stamps,stamps[1:])):error=error or 'Nonmonotonic synchronized export'
        if mapper:
            stats['native_mapper_returncode']=mapper.returncode
            if mapper.returncode not in (0,-signal.SIGINT):error=error or f'Native mapper exited with {mapper.returncode}'
        stats.update(success=error is None,error=error,wall_s=time.monotonic()-begin)
        (out/'summary.json').write_text(json.dumps(stats,indent=2)+'\n')
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
    print(json.dumps(stats),flush=True)
    if error:raise SystemExit(1)


if __name__=='__main__':main()
