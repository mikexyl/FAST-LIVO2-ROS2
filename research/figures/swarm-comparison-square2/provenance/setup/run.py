#!/usr/bin/env python3
"""Run three actual Swarm-SLAM robot processes on shared, saved LiDAR odometry.

The replay/observer does not retrieve, register or optimize. It publishes only
sensor/odometry inputs, records native messages, and checks completion.
"""
import argparse
from collections import Counter,deque
import heapq
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import numpy as np
import yaml
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.serialization import serialize_message
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosidl_runtime_py.convert import message_to_ordereddict
from sensor_msgs.msg import PointCloud2,PointField
from nav_msgs.msg import Odometry
from std_msgs.msg import UInt32,Int64
from cslam_common_interfaces import msg as cm
from s3e_pipeline.data import frames
from s3e_pipeline.geometry import extrinsic,transform
from s3e_pipeline.artifacts import read_json,file_hash,write_json,write_jsonl

ROOT=Path(__file__).resolve().parents[2]
ROBOTS=('Alpha','Bob','Carol')

def matrix(p):
    T=np.eye(4);q=p.orientation
    T[:3,:3]=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix()
    T[:3,3]=[p.position.x,p.position.y,p.position.z]
    return T.tolist()

def messages(row,cloud,robot):
    points=transform(extrinsic(row),cloud[:,:3]).astype('<f4')
    pc=PointCloud2();pc.header.frame_id=f'r{robot}_body'
    pc.header.stamp.sec,pc.header.stamp.nanosec=divmod(row['stamp_ns'],10**9)
    pc.height=1;pc.width=len(points);pc.point_step=12;pc.row_step=12*len(points)
    pc.fields=[PointField(name=n,offset=4*i,datatype=PointField.FLOAT32,count=1) for i,n in enumerate('xyz')]
    pc.is_dense=True;pc.data=points.tobytes()
    odom=Odometry();odom.header.stamp=pc.header.stamp;odom.header.frame_id=f'r{robot}_odom';odom.child_frame_id=pc.header.frame_id
    T=np.asarray(row['T_world_body']).reshape(4,4);q=Rotation.from_matrix(T[:3,:3]).as_quat()
    odom.pose.pose.position.x,odom.pose.pose.position.y,odom.pose.pose.position.z=map(float,T[:3,3])
    odom.pose.pose.orientation.x,odom.pose.pose.orientation.y,odom.pose.pose.orientation.z,odom.pose.pose.orientation.w=map(float,q)
    # ROS zeros indicate unspecified input covariance. Upstream uses its own
    # fixed factor noise (0.01 rad, 0.1 m), not the filter marginal covariance.
    return pc,odom

class Observer:
    def __init__(self,node,output):
        self.node=node;self.output=output;self.started=time.monotonic()
        self.keys={r:{} for r in range(3)};self.descriptors={r:set() for r in range(3)}
        self.optimized={};self.debug={};self.loops=[];self.states={};self.bytes=Counter();self.counts=Counter()
        self.processed={r:0 for r in range(3)};self.processed_count=Counter()
        self.subs=[];self.events=(output/'events.jsonl').open('w')
        for r in range(3):
            self.subscribe(Int64,f'/r{r}/cslam/input_processed_stamp',lambda m,r=r:self.ack(r,m))
            self.subscribe(cm.KeyframeOdom,f'/r{r}/cslam/keyframe_odom',lambda m,r=r:self.keyframe(r,m))
            self.subscribe(cm.IntraRobotLoopClosure,f'/r{r}/cslam/intra_robot_loop_closure',lambda m,r=r:self.loop(r,m))
            self.subscribe(cm.OptimizationResult,f'/r{r}/cslam/optimized_estimates',lambda m,r=r:self.result(r,m),True)
            self.subscribe(cm.OptimizationResult,f'/r{r}/cslam/debug_optimization_result',lambda m,r=r:self.graph(r,m))
            self.subscribe(cm.OptimizerState,f'/r{r}/cslam/optimizer_state',lambda m,r=r:self.states.update({r:m.state}))
            self.subscribe(UInt32,f'/r{r}/cslam/heartbeat',None,True)
            self.subscribe(cm.RobotIds,f'/r{r}/cslam/get_pose_graph',None,True)
            self.subscribe(cm.LocalDescriptorsRequest,f'/r{r}/cslam/local_descriptors_request',None,True)
        self.subscribe(cm.InterRobotLoopClosure,'/cslam/inter_robot_loop_closure',lambda m:self.loop(None,m),True)
        self.subscribe(cm.GlobalDescriptors,'/cslam/global_descriptors',self.descriptor,True)
        self.subscribe(cm.InterRobotMatches,'/cslam/inter_robot_matches',None,True)
        self.subscribe(cm.LocalPointCloudDescriptors,'/cslam/local_descriptors',None,True)
        self.subscribe(cm.PoseGraph,'/cslam/pose_graph',None,True)

    def event(self,kind,**data):
        self.events.write(json.dumps(dict(type=kind,wall_s=time.monotonic()-self.started,**data))+'\n');self.events.flush()

    def ack(self,robot,msg):
        self.processed[robot]=msg.data;self.processed_count[robot]+=1

    def subscribe(self,typ,topic,callback,communication=False):
        def receive(message):
            if communication:
                self.bytes[topic]+=len(serialize_message(message));self.counts[topic]+=1
            if callback:callback(message)
        self.subs.append(self.node.create_subscription(typ,topic,receive,1000))

    def keyframe(self,robot,msg):
        stamp=msg.odom.header.stamp.sec*10**9+msg.odom.header.stamp.nanosec
        self.keys[robot][msg.id]=dict(robot_id=ROBOTS[robot],keyframe_id=msg.id,frame_id=msg.id,stamp_ns=stamp,T_world_body=matrix(msg.odom.pose.pose))
        self.event('keyframe',robot=robot,keyframe=msg.id,stamp_ns=stamp)

    def descriptor(self,msg):
        for d in msg.descriptors:self.descriptors[d.robot_id].add(d.keyframe_id)

    def loop(self,robot,msg):
        data=dict(message_to_ordereddict(msg));data['robot_id']=robot
        self.loops.append(data);self.event('verification',**data)

    def result(self,robot,msg):
        if msg.success and len(msg.estimates):
            self.optimized[robot]=dict(message_to_ordereddict(msg))
            self.event('optimization_received',robot=robot,origin=msg.origin_robot_id,poses=len(msg.estimates))

    def graph(self,robot,msg):
        if msg.success:
            self.debug[robot]=dict(message_to_ordereddict(msg))
            self.event('optimization_finished',robot=robot,poses=len(msg.estimates),factors=len(msg.factors))

    def save(self):
        for r in range(3):
            write_jsonl(self.output/f'{ROBOTS[r]}-keyframes.jsonl',[v for _,v in sorted(self.keys[r].items())])
        write_json(self.output/'optimized.json',self.optimized);write_json(self.output/'graphs.json',self.debug)
        write_jsonl(self.output/'loops.jsonl',self.loops)
        write_json(self.output/'communication.json',dict(serialized_cdr_bytes=dict(self.bytes),messages=dict(self.counts),
            semantics='Observed application CDR payloads once per publication on listed coordination topics; includes local deliveries; excludes DDS transport/discovery overhead and does not multiply broadcast fanout.'))

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--work',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--rate',type=float,default=1.);p.add_argument('--duration',type=float,default=0.)
    p.add_argument('--smoke-duplicate-alpha',action='store_true');p.add_argument('--tail-s',type=float,default=30.)
    args=p.parse_args();work=args.work.resolve();out=args.output.resolve();out.mkdir(parents=True,exist_ok=False)
    cfg=yaml.safe_load((ROOT/'Swarm-SLAM/src/cslam_experiments/config/graco_lidar.yaml').read_text())
    params=cfg['/**']['ros__parameters'];params['evaluation'].update(enable_logs=True,enable_pose_timestamps_recording=True,log_folder=str(out/'native-logs'))
    params['visualization']['enable']=False;params['backend']['max_waiting_time_sec']=60
    params['frontend']['pointcloud_odom_approx_time_sync_s']=.001
    config=out/'config.yaml';config.write_text(yaml.safe_dump(cfg,sort_keys=False))
    write_json(out/'inputs.json',dict(frontend=str(work/'frontend'),rate=args.rate,duration=args.duration,
        smoke_duplicate_alpha=args.smoke_duplicate_alpha,post_replay_settle_s=args.tail_s,
        config_sha256=file_hash(config),clouds='Complete deskewed scan transformed into the associated body frame.',
        replay_backpressure='At most five unacknowledged scans and five unpublished keyframe descriptors per robot; causal timestamp order.',
        scripts={str(f.relative_to(ROOT)):file_hash(f) for f in [Path(__file__),ROOT/'Swarm-SLAM/src/cslam/cslam/lidar_pr/icp_utils.py',ROOT/'Swarm-SLAM/src/cslam/cslam/lidar_handler_node.py']}))
    rclpy.init();node=rclpy.create_node('s3e_replay_observer');observer=Observer(node,out)
    inflight=[deque() for _ in range(3)];backpressure_s=0.
    children=[];logs=[];summary=dict(complete=False,success=False);expected=[0,0,0];published=[0,0,0];last=[None]*3
    resource_samples={}
    def check():
        for child in children:
            if child.poll() is not None:raise RuntimeError(f'Swarm process exited: {child.args}, status {child.returncode}')
    def resources():
        for child in children:
            try:
                status=dict(line.split(':',1) for line in Path(f'/proc/{child.pid}/status').read_text().splitlines() if ':' in line)
                stat=Path(f'/proc/{child.pid}/stat').read_text().split(') ',1)[1].split()
                resource_samples[str(child.pid)]=dict(command=child.args,
                    cpu_s=(int(stat[11])+int(stat[12]))/os.sysconf('SC_CLK_TCK'),
                    peak_rss_kib=int(status['VmHWM'].split()[0]))
            except (FileNotFoundError,KeyError):pass
    def spin(seconds):
        end=time.monotonic()+max(0.,seconds)
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=min(.05,max(0.,end-time.monotonic())));check()
    try:
        pubs=[(node.create_publisher(PointCloud2,f'/r{r}/pointcloud',100),node.create_publisher(Odometry,f'/r{r}/odom',100)) for r in range(3)]
        for r in range(3):
            for name in ('lidar_handler_node.py','loop_closure_detection_node.py','pose_graph_manager'):
                path=(ROOT/'Swarm-SLAM/src/cslam/cslam'/name if name.endswith('.py') else ROOT/'.ros2/swarm/install/cslam/lib/cslam'/name)
                cmd=([sys.executable,str(path)] if name.endswith('.py') else [str(path)])+['--ros-args','--params-file',str(config),'-r',f'__ns:=/r{r}','-p',f'robot_id:={r}','-p','max_nb_robots:=3']
                log=(out/f'r{r}-{name}.log').open('w');logs.append(log)
                children.append(subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT,start_new_session=True))
        deadline=time.monotonic()+90
        while not all(pc.get_subscription_count()>0 and od.get_subscription_count()>0 for pc,od in pubs):
            if time.monotonic()>deadline:raise TimeoutError('Swarm input subscriptions did not become ready')
            spin(.2)
        spin(7.)
        iterators=[iter(frames(work/'frontend'/('Alpha' if args.smoke_duplicate_alpha else r)/'export')) for r in ROBOTS]
        queue=[]
        for r,it in enumerate(iterators):
            row,cloud,_=next(it);heapq.heappush(queue,(row['stamp_ns'],r,row,cloud))
        first=queue[0][0];started=time.monotonic();next_report=started
        while queue:
            stamp,r,row,cloud=heapq.heappop(queue);elapsed=(stamp-first)/1e9
            if args.duration and elapsed>args.duration:break
            blocked=time.monotonic()
            while True:
                while inflight[r] and inflight[r][0]<=observer.processed[r]:inflight[r].popleft()
                if len(inflight[r])<5 and len(observer.keys[r])-len(observer.descriptors[r])<5:break
                if time.monotonic()-blocked>120:raise TimeoutError(f'Robot {r} stalled while processing inputs/descriptors')
                spin(.02)
            backpressure_s+=time.monotonic()-blocked
            spin(started+elapsed/args.rate-time.monotonic());check()
            pc,odom=messages(row,cloud,r);pubs[r][1].publish(odom);pubs[r][0].publish(pc)
            inflight[r].append(stamp)
            published[r]+=1;xyz=np.asarray(row['T_world_body']).reshape(4,4)[:3,3]
            if last[r] is None or np.sum((xyz-last[r])**2)>params['frontend']['keyframe_generation_ratio_distance']**2:
                expected[r]+=1;last[r]=xyz.copy()
            rclpy.spin_once(node,timeout_sec=0.)
            if time.monotonic()>=next_report:
                resources()
                print(f'{elapsed:.1f}s input; keyframes expected/received {expected}/'+str([len(observer.keys[r]) for r in range(3)]),flush=True)
                next_report=time.monotonic()+20
            try:
                nxt,pts,_=next(iterators[r]);heapq.heappush(queue,(nxt['stamp_ns'],r,nxt,pts))
            except StopIteration:pass
        summary['sensor_replay_wall_s']=time.monotonic()-started
        print('Sensor replay finished; waiting for queued descriptors.',flush=True)
        deadline=time.monotonic()+max(180.,summary['sensor_replay_wall_s'])
        while not all(observer.processed_count[r]==published[r] and len(observer.keys[r])==expected[r] and len(observer.descriptors[r])==expected[r] for r in range(3)):
            if time.monotonic()>deadline:raise TimeoutError('Native keyframes/descriptors did not drain within bounded wait')
            spin(.1)
        print('All descriptors received; final optimization interval.',flush=True);spin(args.tail_s)
        if not all(r in observer.optimized and len(observer.optimized[r]['estimates'])==expected[r] for r in range(3)):
            raise RuntimeError('Final native optimized estimates do not cover every keyframe')
        summary.update(complete=True,success=True,total_wall_s=time.monotonic()-started,backpressure_s=backpressure_s)
    except Exception as exc:
        summary['error']=repr(exc);print('FAILED',repr(exc),flush=True)
    finally:
        resources();write_json(out/'resources.json',resource_samples)
        observer.save();summary.update(expected_keyframes=expected,received_keyframes=[len(observer.keys[r]) for r in range(3)],
            received_descriptors=[len(observer.descriptors[r]) for r in range(3)],published_scans=published)
        summary['processed_scans']=[observer.processed_count[r] for r in range(3)]
        for child in children:
            if child.poll() is None:os.killpg(child.pid,signal.SIGINT)
        for child in children:
            try:child.wait(timeout=15)
            except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL);child.wait()
        for log in logs:log.close()
        summary['native_optimizer_error_count']=sum(p.read_text().count('Optimization failed:') for p in out.glob('*pose_graph_manager.log'))
        observer.events.close();node.destroy_node();rclpy.shutdown();write_json(out/'summary.json',summary)
    if not summary['success']:raise SystemExit(1)
    print(json.dumps(summary),flush=True)

if __name__=='__main__':main()
