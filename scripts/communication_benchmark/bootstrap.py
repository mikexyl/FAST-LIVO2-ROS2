#!/usr/bin/env python3
"""Wrap only the experiment harness with a passive monitor and divergence guard."""
import argparse,os,runpy,signal,subprocess,sys,time,math
from pathlib import Path
from meter import ROS1Meter,ROS2Meter,write

ap=argparse.ArgumentParser();ap.add_argument('--method',required=True);ap.add_argument('--runner',required=True);a,rest=ap.parse_known_args()
out=Path(rest[rest.index('--output')+1]);probes=[];processes={};previous={}
original_killpg=os.killpg
original_popen=subprocess.Popen
class ObservedPopen(original_popen):
    def __init__(self,*args,**kwargs):
        command=args[0] if args else kwargs.get('args',[])
        command_text=' '.join(str(x) for x in command)
        if a.method=='swarm' and ('cslam/' in command_text or 'pose_graph_manager' in command_text):
            trace=out/'tx';trace.mkdir(parents=True,exist_ok=True)
            env=dict(kwargs.get('env') or os.environ)
            env['LD_PRELOAD']=str(Path(__file__).with_name('libcommunication_probe.so'))+(':'+env['LD_PRELOAD'] if env.get('LD_PRELOAD') else '')
            env['COMM_TRACE_DIR']=str(trace);kwargs['env']=env
        super().__init__(*args,**kwargs)
        processes[self.pid]=args[0] if args else kwargs.get('args',[])
subprocess.Popen=ObservedPopen

def finish():
    while probes:
        p=probes.pop()
        try:p.finish()
        except Exception as e:
            write(out/'communication-monitor-failure.json',dict(error=repr(e)))

def killpg(pid,sig):
    command=' '.join(str(x) for x in processes.get(pid,[]))
    if command and ' bag ' not in command and 'rosbag' not in command:finish()
    return original_killpg(pid,sig)
os.killpg=killpg

def diverged(signum,frame):raise RuntimeError('Odometry divergence: entire group stopped; see divergence.json')
signal.signal(signal.SIGUSR1,diverged)

def check(topic,msg):
    # Observe native odometry, never optimized paths or interrobot transforms.
    stamp=msg.header.stamp
    t=stamp.to_sec() if hasattr(stamp,'to_sec') else stamp.sec+stamp.nanosec*1e-9
    p=msg.pose.pose.position;q=msg.pose.pose.orientation
    xyz=[p.x,p.y,p.z];values=xyz+[q.x,q.y,q.z,q.w]
    if not all(math.isfinite(v) for v in values):
        fail=dict(topic=topic,stamp=t,reason='nonfinite native odometry')
    else:
        old=previous.get(topic);previous[topic]=(t,xyz)
        if old is None or t<=old[0]:return
        speed=math.sqrt(sum((x-y)**2 for x,y in zip(xyz,old[1])))/(t-old[0])
        if speed<=20:return
        fail=dict(topic=topic,stamp=t,reason='native odometry speed exceeds 20 m/s',speed_m_s=speed,previous=old,current_xyz=xyz)
    if not (out/'divergence.json').exists():
        write(out/'divergence.json',fail);os.kill(os.getpid(),signal.SIGUSR1)

if a.method!='swarm':
    import rospy
    original_init=rospy.init_node;original_sub=rospy.Subscriber
    def init(*args,**kwargs):
        result=original_init(*args,**kwargs);probes.append(ROS1Meter(out,a.method));return result
    rospy.init_node=init
    def sub(topic,typ,callback=None,*args,**kwargs):
        if a.method in ['dcl','disco'] and topic.endswith('/lio_sam/mapping/odometry') and callback:
            original=callback
            def callback(msg):check(topic,msg);return original(msg)
        elif a.method=='gac' and topic=='/laser_odom_to_init' and callback:
            original=callback
            closed=dict(zip(original.__code__.co_freevars,[x.cell_contents for x in original.__closure__]))
            if 'current' in closed and 'intervals' in closed:
                def callback(msg):
                    robot=closed['current'][0]
                    if robot is not None:
                        lo,hi=closed['intervals'][robot]
                        if lo-1 <= msg.header.stamp.to_sec() <= hi+1:check(topic+'/'+robot,msg)
                    return original(msg)
        return original_sub(topic,typ,callback,*args,**kwargs)
    rospy.Subscriber=sub
    if a.method=='gac':
        import pexpect
        original_terminate=pexpect.spawn.terminate
        def terminate(child,*args,**kwargs):
            finish();return original_terminate(child,*args,**kwargs)
        pexpect.spawn.terminate=terminate
else:
    import rclpy
    original_create=rclpy.create_node
    def create(*args,**kwargs):
        node=original_create(*args,**kwargs);probes.append(ROS2Meter(node,out,a.method))
        orig=node.create_subscription
        def sub(typ,topic,callback,*args,**kwargs):
            if topic.endswith('/odom'):
                cb=callback
                def callback(msg):check(topic,msg);return cb(msg)
            return orig(typ,topic,callback,*args,**kwargs)
        node.create_subscription=sub
        return node
    rclpy.create_node=create
sys.argv=[a.runner]+rest
sys.path.insert(0,str(Path(a.runner).parent))
try:runpy.run_path(a.runner,run_name='__main__')
finally:finish()
