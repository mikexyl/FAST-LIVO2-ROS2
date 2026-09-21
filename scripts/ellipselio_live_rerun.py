#!/usr/bin/env python3
"""Record live EllipseLIO ROS outputs directly to Rerun, without bag reconstruction."""
import argparse
from collections import Counter
import json
import math
from pathlib import Path
import signal
import time

import numpy as np


def stamp_ns(stamp):
    return stamp.sec*10**9 + stamp.nanosec


def cloud_xyz(msg, limit):
    fields = {f.name: f for f in msg.fields}
    types = {1:'i1', 2:'u1', 3:'i2', 4:'u2', 5:'i4', 6:'u4', 7:'f4', 8:'f8'}
    xyz = np.column_stack([np.ndarray((msg.height, msg.width),
        dtype=np.dtype(('>' if msg.is_bigendian else '<')+types[fields[k].datatype]),
        buffer=msg.data, offset=fields[k].offset,
        strides=(msg.row_step, msg.point_step)).ravel() for k in ('x','y','z')])
    xyz = xyz[np.isfinite(xyz).all(axis=1)]
    if len(xyz)>limit:
        xyz = xyz[np.linspace(0, len(xyz)-1, limit, dtype=int)]
    return xyz.astype(np.float32)


def main():
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import PointCloud2, Imu
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock
    from tf2_msgs.msg import TFMessage
    from visualization_msgs.msg import MarkerArray
    from ellipselio.msg import EllipseLioAnalytics
    import rerun as rr
    import rerun.blueprint as rrb

    p=argparse.ArgumentParser()
    p.add_argument('--robot',required=True)
    p.add_argument('--sequence',default='Library 2',help='Dataset label for the recording')
    p.add_argument('--imu-topic',help='Raw IMU topic; defaults to /ROBOT/imu/data')
    p.add_argument('--submap-dir',type=Path,help='Live immutable native submap index, when enabled')
    p.add_argument('--start-ns',required=True,type=int,help='Original bag start timestamp')
    p.add_argument('--output',required=True,type=Path)
    p.add_argument('--grpc-port',type=int,default=9876)
    p.add_argument('--map-chunks',type=int,default=100,help='Native SplitMap chunk count')
    p.add_argument('--max-map-points',type=int,default=100000)
    p.add_argument('--max-scan-points',type=int,default=6000)
    args=p.parse_args()
    if rr.__version__!='0.37.1':raise RuntimeError('Rerun 0.37.1 is required')
    args.output.mkdir(parents=True,exist_ok=False)
    rr.init(f'{args.robot} live EllipseLIO / {args.sequence}')
    sinks=[rr.FileSink(str(args.output/'live.rrd'))]
    if args.grpc_port:
        sinks.append(rr.GrpcServerSink(port=args.grpc_port,server_memory_limit='512MiB'))
    rr.set_sinks(*sinks)
    rr.log('/',rr.ViewCoordinates.RIGHT_HAND_Z_UP,static=True)
    rr.send_blueprint(rrb.Blueprint(rrb.Horizontal(
        rrb.Tabs(
            rrb.Spatial3DView(name='Live native map + scan',origin='/world',contents=['/world/**','-/world/ellipsoids/**']),
            rrb.Spatial3DView(name='Native ellipsoid markers',origin='/world',contents=['/world/ellipsoids/**','/world/trajectory','/world/body/**']),
            rrb.Spatial3DView(name='Native processed scan',origin='/scan')),
        rrb.Vertical(
            rrb.TimeSeriesView(name='Post-LiDAR pose speed [m/s]',origin='/diagnostics/speed'),
            rrb.TimeSeriesView(name='Native registration residual',origin='/diagnostics/residual'),
            rrb.Tabs(
                rrb.TimeSeriesView(name='Matched / rejected features',origin='/diagnostics/features'),
                rrb.TimeSeriesView(name='Sensor receipt rates [Hz]',origin='/diagnostics/rates'),
                rrb.TimeSeriesView(name='IMU',origin='/diagnostics/imu'),
                rrb.TimeSeriesView(name='Native pose covariance diagonal',origin='/diagnostics/covariance'),
                rrb.TimeSeriesView(name='Observability scores',origin='/diagnostics/observability'),
                rrb.TimeSeriesView(name='Native update time [ms]',origin='/diagnostics/timing')),
            rrb.TextDocumentView(name='Live status',origin='/status')),column_shares=[.65,.35]),
        rrb.TimePanel(timeline='elapsed',play_state='Following'),collapse_panels=True))
    description=(f'# {args.robot}: live EllipseLIO capture\n\n'
        'Recorded directly from the running native ROS publishers. No saved poses or original-bag clouds are reconstructed.\n\n'
        'Map/scan coordinates are the publisher\'s odom_ellipselio frame. '
        'Map chunks retain their common native snapshot stamp; receipt time is logged separately. '
        'Native map is published every 10 s in up to 100 chunks over roughly 10 s. '
        'A snapshot may be partial if best-effort output messages are lost or playback ends while chunks are in flight.\n\n'
        f'Display sampling: at most {args.max_scan_points} points per scan and {args.max_map_points} per full map. '
        'Ellipsoids are the upstream sparse MarkerArray output (approximately one per 100 newly added map points), not the complete fitted map.\n\n'
        'TF supplies exact post-LiDAR-update poses. Odometry supplies IMU-propagated velocity/covariance diagnostics. '
        'Analytics uses native sensor_stamp_ns when available; legacy publishers use approximate /clock timing. '
        'Covariance diagonals are shown in native published order; they are not relative-edge uncertainty.\n\n'
        'Timeline elapsed = seconds from original bag start. The earlier run first exceeded 20 m/s near 444.9 s on this timeline; this rerun may differ.\n')
    rr.log('/about',rr.TextDocument(description),static=True)
    rclpy.init()
    node=rclpy.create_node(args.robot.lower()+'_live_rerun')
    qos=QoSProfile(depth=100,reliability=ReliabilityPolicy.BEST_EFFORT)
    counts=Counter(); rejected=Counter(); first={}; last={}; clock_ns=None
    last_pose=None; trajectory=[]; trajectory_time=-1.; map_stamp=None; map_chunk=0
    last_odom_log=-1; last_imu_log=-1; first_violation=None; max_speed=0.; map_snapshots=[]
    started=time.monotonic(); stopped=False; fault=None
    world_frame=args.robot+'/odom_ellipselio'
    (args.output/'ABOUT.md').write_text(description)
    poses=(args.output/'post_lidar_poses.tum').open('w',buffering=1)
    poses.write('# Native post-LiDAR TF: timestamp tx ty tz qx qy qz qw\n')

    def set_time(ns):
        rr.set_time('elapsed',duration=np.timedelta64(int(ns)-args.start_ns,'ns'))
        rr.set_time('sensor_time',timestamp=np.datetime64(int(ns),'ns'))
        receipt=clock_ns if clock_ns is not None else ns
        rr.set_time('received_elapsed',duration=np.timedelta64(int(receipt)-args.start_ns,'ns'))

    def seen(topic,ns):
        counts[topic]+=1;first.setdefault(topic,int(ns));last[topic]=int(ns)

    def clock(msg):
        nonlocal clock_ns
        clock_ns=stamp_ns(msg.clock)

    def transforms(msg):
        nonlocal last_pose,trajectory_time,first_violation,max_speed
        for t in msg.transforms:
            if t.child_frame_id!=args.robot+'/imu_ellipselio':continue
            ns=stamp_ns(t.header.stamp)
            if t.header.frame_id!=world_frame:rejected['tf_frame']+=1;continue
            if ns<args.start_ns or (last_pose and ns<=last_pose[0]):continue
            v=t.transform.translation;q=t.transform.rotation
            xyz=np.array([v.x,v.y,v.z]);quat=[q.x,q.y,q.z,q.w]
            if not np.isfinite([*xyz,*quat]).all():rejected['nonfinite_pose']+=1;continue
            seconds,nanoseconds=divmod(ns,10**9)
            poses.write(f'{seconds}.{nanoseconds:09d} '+ ' '.join(f'{x:.12g}' for x in [*xyz,*quat])+'\n')
            seen('post_lidar_tf',ns);set_time(ns)
            if last_pose:
                speed=float(np.linalg.norm(xyz-last_pose[1])/((ns-last_pose[0])/1e9))
                max_speed=max(max_speed,speed)
                rr.log('/diagnostics/speed/scan_pose',rr.Scalars(speed))
                rr.log('/diagnostics/speed/guard_20m_s',rr.Scalars(20.))
                if speed>20 and first_violation is None:
                    first_violation=dict(stamp_ns=ns,elapsed_s=(ns-args.start_ns)/1e9,speed_m_s=speed)
                    rr.log('/events',rr.TextLog('First motion-guard violation: '+json.dumps(first_violation),level='WARN'))
            last_pose=(ns,xyz);trajectory.append(xyz.tolist())
            rr.log('/world/body',rr.Transform3D(translation=xyz,quaternion=rr.Quaternion(xyzw=quat)))
            rr.log('/world/body/axes',rr.Arrows3D(origins=np.zeros((3,3)),vectors=np.eye(3)*2,
                colors=[[255,70,70],[70,230,90],[70,140,255]],radii=.04))
            if ns/1e9-trajectory_time>=.5:
                rr.log('/world/trajectory',rr.LineStrips3D([trajectory],colors=[60,180,255],radii=.07))
                trajectory_time=ns/1e9

    def scan(msg):
        ns=stamp_ns(msg.header.stamp)
        if msg.header.frame_id!=world_frame:rejected['scan_frame']+=1;return
        seen('native_scan',ns);set_time(ns)
        xyz=cloud_xyz(msg,args.max_scan_points)
        rr.log('/world/scan',rr.Points3D(xyz,colors=[255,185,70],radii=.035))
        rr.log('/scan/points',rr.Points3D(xyz,colors=[255,185,70],radii=.04))

    def map_cloud(msg):
        nonlocal map_stamp,map_chunk
        ns=stamp_ns(msg.header.stamp)
        if msg.header.frame_id!=world_frame:rejected['map_frame']+=1;return
        if ns<handover_ns:
            rejected['archived_map_chunk']+=1;return
        seen('native_map_chunk',ns);set_time(ns)
        if ns!=map_stamp or map_chunk>=args.map_chunks:
            if map_stamp is not None:map_snapshots.append(dict(stamp_ns=map_stamp,received_chunks=map_chunk))
            map_stamp=ns;map_chunk=0
            rr.log('/world/native_map',rr.Clear(recursive=True))
        xyz=cloud_xyz(msg,max(1,args.max_map_points//args.map_chunks))
        rr.log(f'/world/native_map/chunk_{map_chunk:03d}',rr.Points3D(xyz,colors=[155,170,190],radii=.035))
        map_chunk+=1

    def markers(msg):
        # Preserve native marker IDs and lifetime. Upstream only publishes new markers.
        for m in msg.markers:
            if m.action==3:
                rr.log('/world/ellipsoids',rr.Clear(recursive=True));continue
            ns=stamp_ns(m.header.stamp)
            if m.header.frame_id!=world_frame:rejected['marker_frame']+=1;continue
            seen('native_ellipsoid_marker',ns);set_time(ns)
            path=f'/world/ellipsoids/{m.ns}/{m.id}'
            if m.action==3:rr.log('/world/ellipsoids',rr.Clear(recursive=True));continue
            if m.action==2:rr.log(path,rr.Clear(recursive=True));continue
            if m.type!=2:rejected['unsupported_marker']+=1;continue
            v=m.pose.position;q=m.pose.orientation;s=m.scale;c=m.color
            values=[v.x,v.y,v.z,q.x,q.y,q.z,q.w,s.x,s.y,s.z]
            if not np.isfinite(values).all() or min(s.x,s.y,s.z)<=0:
                rejected['invalid_marker']+=1;continue
            rr.log(path,rr.Ellipsoids3D(centers=[[v.x,v.y,v.z]],half_sizes=[[s.x/2,s.y/2,s.z/2]],
                quaternions=[[q.x,q.y,q.z,q.w]],colors=[[round(c.r*255),round(c.g*255),round(c.b*255),150]],fill_mode='Solid'))

    def scalar(path,value):
        if math.isfinite(value):rr.log(path,rr.Scalars(float(value)))

    diagnostic_rows=(args.output/'analytics.jsonl').open('w',buffering=1)
    active_id=-1
    handover_ns=0
    archived=set()

    def analytics(msg):
        nonlocal active_id,handover_ns
        ns=getattr(msg,'stamp_ns',0) or clock_ns
        if ns is None or ns<args.start_ns:return
        seen('native_analytics',ns);set_time(ns)
        fields=msg.get_fields_and_field_types()
        diagnostic_rows.write(json.dumps({k:getattr(msg,k) for k in fields},default=list)+'\n')
        current=getattr(msg,'active_submap_id',-1)
        if current != active_id:
            rr.log('/events',rr.TextLog(f'Submap handover: {active_id} -> {current}'))
            rr.log('/world/ellipsoids',rr.Clear(recursive=True))
            rr.log('/world/native_map',rr.Clear(recursive=True))
            active_id=current
            handover_ns=ns
        for field in ('active_submap_id','successor_submap_id','handovers','active_features',
                      'successor_features','correspondence_age_mean','correspondence_age_max','lidar_updated',
                      'active_extent_m','successor_extent_m','active_age_s','successor_support','successor_support_ratio',
                      'active_area_m2','active_new_area_m2','successor_area_m2','shared_area_m2','coverage_overlap_ratio'):
            if hasattr(msg,field):scalar('/diagnostics/submaps/'+field,getattr(msg,field))
        if getattr(msg,'submap_event',''):
            rr.log('/events',rr.TextLog(msg.submap_event))
        for field in ('num_feats','num_reject','num_planes','num_lines','num_balls'):
            scalar('/diagnostics/features/'+field,getattr(msg,field))
        scalar('/diagnostics/residual/res_mean',msg.res_mean)
        for field in ('imu_freq','lid_freq','odom_freq'):scalar('/diagnostics/rates/'+field,getattr(msg,field))
        for field in ('obs_score','vert_score','bin_score'):scalar('/diagnostics/observability/'+field,getattr(msg,field))
        for field in ('imu_time','state_time','map_time','total_time'):scalar('/diagnostics/timing/'+field,1000*getattr(msg,field))
        for field in ('map_size','oct_num','scan_size','buffer_size','kf_iterations'):
            scalar('/diagnostics/native/'+field,getattr(msg,field))

    def odometry(msg):
        nonlocal last_odom_log
        ns=stamp_ns(msg.header.stamp);seen('imu_propagated_odometry',ns)
        if ns-last_odom_log<50000000:return
        last_odom_log=ns;set_time(ns)
        for i in range(6):scalar(f'/diagnostics/covariance/native_diagonal_{i}',msg.pose.covariance[7*i])
        v=msg.twist.twist.linear
        scalar('/diagnostics/speed/imu_propagated_twist',math.sqrt(v.x*v.x+v.y*v.y+v.z*v.z))

    def imu(msg):
        nonlocal last_imu_log
        ns=stamp_ns(msg.header.stamp);seen('raw_imu',ns)
        if ns-last_imu_log<50000000:return
        last_imu_log=ns;set_time(ns)
        for label,v in [('acc',msg.linear_acceleration),('gyro',msg.angular_velocity)]:
            for axis in ('x','y','z'):scalar(f'/diagnostics/imu/{label}_{axis}',getattr(v,axis))

    def status(complete=False):
        if args.submap_dir and (args.submap_dir/'index.jsonl').exists():
            import hashlib
            for line in (args.submap_dir/'index.jsonl').read_text().splitlines(keepends=True):
                if not line.endswith('\n'):continue
                row=json.loads(line);key=row['submap_id']
                if key in archived:continue
                payload=args.submap_dir/row['payload']
                if hashlib.sha256(payload.read_bytes()).hexdigest()!=row['sha256']:
                    raise ValueError('Live submap payload hash mismatch')
                with np.load(payload,allow_pickle=False) as data:
                    points=data['points'];stride=max(1,len(points)//args.max_map_points)
                    T=np.array(row['T_world_imu']).reshape(4,4)
                    points=points[::stride]@T[:3,:3].T+T[:3,3]
                set_time(row['available_ns'])
                entity='area_snapshots' if row.get('strategy')=='area' else 'archived_submaps'
                rr.log(f'/world/{entity}/{key}',rr.Points3D(points,colors=[90,115,155],radii=.025))
                rr.log(f'/world/{entity}/{key}/metadata',rr.TextDocument(json.dumps(row,indent=2)))
                archived.add(key)
        value=dict(robot=args.robot,capture='live ROS subscriptions during a fresh EllipseLIO replay',
            complete=complete,wall_s=time.monotonic()-started,counts=dict(counts),rejected=dict(rejected),
            first_stamp_ns=first,last_stamp_ns=last,first_motion_violation=first_violation,
            maximum_scan_pose_speed_m_s=max_speed,clock_ns=clock_ns,
            map_snapshots=map_snapshots+([dict(stamp_ns=map_stamp,received_chunks=map_chunk)] if map_stamp else []),
            recording_bytes=(args.output/'live.rrd').stat().st_size,error=fault)
        temp=args.output/'status.tmp';temp.write_text(json.dumps(value,indent=2)+'\n');temp.replace(args.output/'status.json')
        if clock_ns is not None and clock_ns>=args.start_ns:
            set_time(clock_ns)
            rr.log('/status',rr.TextDocument(description+'\n\n## Capture status\n```json\n'+json.dumps({k:v for k,v in value.items() if k!='map_snapshots'},indent=2)+'\n```'))
        rr.get_data_recording().flush(timeout_sec=0.)

    subscriptions=[(Clock,'/clock',clock),(TFMessage,'/tf',transforms),
        (PointCloud2,f'/{args.robot}/cloud_scan',scan),(PointCloud2,f'/{args.robot}/cloud_map',map_cloud),
        (MarkerArray,f'/{args.robot}/visualization_marker',markers),
        (EllipseLioAnalytics,f'/{args.robot}/analytics',analytics),
        (Odometry,f'/{args.robot}/ellipselio_odom',odometry),(Imu,args.imu_topic or f'/{args.robot}/imu/data',imu)]
    for msg_type,topic,callback in subscriptions:node.create_subscription(msg_type,topic,callback,qos)
    node.create_timer(5.,status)
    def stop(*_):
        nonlocal stopped
        stopped=True
    signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
    (args.output/'READY').write_text(str(time.time_ns())+'\n')
    try:
        while not stopped and rclpy.ok():rclpy.spin_once(node,timeout_sec=.1)
    except BaseException as exc:
        fault=repr(exc);raise
    finally:
        status(complete=fault is None)
        rr.get_data_recording().flush()
        rr.disconnect()
        poses.close()
        diagnostic_rows.close()
        # The closed file includes its footer; the live status size did not.
        final_status=json.loads((args.output/'status.json').read_text())
        final_status['recording_bytes']=(args.output/'live.rrd').stat().st_size
        temp=args.output/'status.tmp';temp.write_text(json.dumps(final_status,indent=2)+'\n');temp.replace(args.output/'status.json')
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
