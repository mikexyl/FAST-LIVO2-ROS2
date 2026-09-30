"""Long-lived, robot-local CBS prefix/geometry bridge. No peer filesystem reads."""
import argparse
import json
import os
from pathlib import Path
import time
import zlib
import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message
from std_msgs.msg import UInt8MultiArray
from cbs_ros.msg import Estimate, NodeStats, PcmStatus, PersistentInput
from .artifacts import canonical, digest, read_json, write_jsonl
from .online_io import atomic_json, Journal
from .cbs_bridge import local_graph, loop_for_robot, ros_graph, matrix_pose, named_pcm_verdicts
from .ros_dpgo_worker import reliable
from .registration_exchange import RegistrationExchange


class PersistentRobot(Node):
    def __init__(self, spec):
        self.spec = spec; self.robot = spec['robot']; self.robots = spec['robots']
        super().__init__('persistent_cbs_bridge', namespace=self.robot)
        self.output = Path(spec['output']); self.revision = 0
        self.rows = []; self.edges = {}; self.pcm = None; self.registration = None
        self.cache = {}; self.inbox = []; self.current = None; self.finished = False
        self.wire_bytes = 0; self.stats = None; self.last_estimate = None
        self.pcm_bytes = 0; self.pcm_counted = set(); self.started = time.monotonic()
        self.stats_log = Journal(self.output/'stats.jsonl')
        self.graph = self.create_publisher(PersistentInput, f'/{self.robot}/cbs/persistent_input', reliable(1, True))
        index = self.robots.index(self.robot)
        self.subs = [self.create_subscription(Estimate, f'/{self.robot}/cbs/estimate', self.estimate, reliable(1, True)),
            self.create_subscription(NodeStats, f'/cbs_ros_{index}/stats', self.statistics, reliable()),
            self.create_subscription(PcmStatus, f'/cbs_ros_{index}/pcm/status', self.pcm_update, reliable(1, True))]
        self.peer_publishers = {}
        for peer in self.robots:
            if peer == self.robot: continue
            self.peer_publishers[peer] = self.create_publisher(UInt8MultiArray,
                f'/s3e/persistent/{self.robot}/to/{peer}', reliable())
            self.subs.append(self.create_subscription(UInt8MultiArray,
                f'/s3e/persistent/{peer}/to/{self.robot}', self.receive, reliable()))
        from .isolation import restrict_reads, worker_paths
        restrict_reads(worker_paths(spec['store'], spec['store'], {}) +
                       [spec['output'], spec['ros_underlay'], spec['ros_overlay']])
        self.timer = self.create_timer(.05, self.tick)
        atomic_json(self.output/'READY', dict(pid=os.getpid()))

    def send(self, event):
        msg = UInt8MultiArray(data=zlib.compress(canonical(event)))
        self.peer_publishers[event['dst']].publish(msg)
        self.wire_bytes += len(serialize_message(msg))

    def receive(self, msg):
        event = json.loads(zlib.decompress(bytes(msg.data)))
        if event['dst'] != self.robot or event['src'] not in self.peer_publishers:
            raise ValueError('Misrouted persistent registration message')
        prefix = self.spec['session']+'/'
        session = event['body'].get('session_id', '')
        if not session.startswith(prefix): raise ValueError('Persistent peer session mismatch')
        revision = int(session[len(prefix):])
        if revision < self.revision: return
        if revision > self.revision+1: raise ValueError('Peer advanced beyond next prefix')
        self.inbox.append((revision, event))

    def pcm_update(self, msg):
        if msg.session_id != f"{self.spec['session']}/{self.revision}": return
        if msg.robot_id != self.robots.index(self.robot): raise ValueError('Foreign PCM status')
        if msg.state == 'error': raise RuntimeError(msg.error)
        self.pcm = {k:getattr(msg, k) for k in ('session_id', 'robot_id', 'state', 'graph_messages',
                    'own_nodes', 'network_cdr_bytes', 'compute_s', 'elapsed_s')}
        self.pcm['verdicts'] = [{k:getattr(v,k) for k in v.get_fields_and_field_types()} for v in msg.verdicts]
        if msg.state == 'ready' and self.revision not in self.pcm_counted:
            self.pcm_bytes += msg.network_cdr_bytes; self.pcm_counted.add(self.revision)

    def statistics(self, msg):
        self.stats = {k:getattr(msg,k) for k in msg.get_fields_and_field_types() if k != 'header'}
        self.stats_log.append(dict(self.stats,graph_revision=0 if self.last_estimate is None else self.last_estimate.graph_revision))
        if self.last_estimate is not None: self.estimate(self.last_estimate)

    def estimate(self, msg):
        self.last_estimate = msg
        if msg.robot_id != self.robots.index(self.robot): raise ValueError('Foreign CBS estimate')
        if not self.current or msg.graph_revision != self.revision: return
        if list(msg.keyframe_ids) != [r['keyframe_id'] for r in self.rows] or list(msg.stamp_ns) != [r['stamp_ns'] for r in self.rows]:
            raise ValueError('Incomplete/mistimestamped persistent estimate')
        if not self.pcm or self.pcm['state'] != 'ready': return
        poses = [dict(robot_id=self.robot, keyframe_id=int(k), stamp_ns=int(t),
            T_world_body=matrix_pose(p).tolist(), reference_available=msg.reference_available,
            component=self.robots[msg.reference_robot_id]) for k,t,p in zip(msg.keyframe_ids,msg.stamp_ns,msg.poses)]
        decisions = named_pcm_verdicts(self.pcm, self.robots)
        proposed = list(self.edges.values())
        retained = [e for e in proposed if e['i'][0] == e['j'][0] or decisions[tuple(e['i']),tuple(e['j'])]['retained']]
        registration = None
        if self.registration:
            path = self.output/'native'/self.robot/'cbs_online/registration.json'
            if not path.exists(): return
            registration = read_json(path)
            if registration['session_id'] != f"{self.spec['session']}/{self.revision}": return
            if msg.finished and (not registration['final'] or not registration['success']):
                raise RuntimeError('Final persistent geometry validation failed')
        status = dict(revision=self.revision, iteration=msg.iteration, finished=msg.finished,
            keyframes=len(poses), loops=len(retained), proposed_loops=len(proposed),
            pcm=self.pcm, registration=registration, component=self.robots[msg.reference_robot_id],
            reference_available=msg.reference_available, process_id=os.getpid(),
            registration_network_cdr_bytes=self.wire_bytes, pcm_network_cdr_bytes=self.pcm_bytes,
            cbs_last_stats=self.stats, wall_s=time.monotonic()-self.started)
        # One atomic object couples its trajectory and metadata to this revision.
        atomic_json(self.output/'estimate.json', dict(status=status, poses=poses, constraints=retained))
        if msg.finished and self.stats and self.stats['iteration'] >= msg.iteration:
            write_jsonl(self.output/'poses.jsonl',poses)
            write_jsonl(self.output/'constraints.jsonl',retained)
            atomic_json(self.output/'summary.json',status); self.finished = True

    def tick(self):
        path = self.output/'input'/f'{self.revision+1}.json'
        if path.exists():
            if self.current and (self.last_estimate is None or self.last_estimate.graph_revision != self.revision):
                raise ValueError('Next prefix arrived before current prefix was applied')
            value = read_json(path)
            if value['revision'] != self.revision+1: raise ValueError('Invalid prefix revision')
            rows = value['rows']
            if rows[:len(self.rows)] != self.rows: raise ValueError('Changed persistent keyframe prefix')
            self.rows = rows; self.current = value; self.revision += 1; self.pcm = None
            nodes, odometry = local_graph(rows, self.robot, self.spec['pgo'])
            edges = [loop_for_robot(e,self.robot) for e in value['edges']]
            self.edges = {(tuple(e['i']),tuple(e['j'])):e for e in edges}
            if len(self.edges) != len(edges): raise ValueError('Duplicate persistent loop')
            if self.spec['registration_factors']['enabled']:
                self.registration = RegistrationExchange(self.robot,self.robots,rows,self.spec['store'],
                    self.output/'registration'/str(self.revision),f"{self.spec['session']}/{self.revision}",
                    self.spec['registration_factors'],self.send,self.cache)
            self.graph.publish(PersistentInput(session_id=self.spec['session'],revision=self.revision,
                final=value['final'],graph=ros_graph(nodes,odometry+edges,self.robots)))
        pending = []
        for revision,event in self.inbox:
            if revision == self.revision and self.registration: self.registration.receive(event)
            elif revision > self.revision: pending.append((revision,event))
        self.inbox = pending
        if self.registration: self.registration.advance(self.pcm,self.edges)


def main():
    parser=argparse.ArgumentParser(); parser.add_argument('--spec',type=Path,required=True)
    spec=read_json(parser.parse_args().spec); rclpy.init(); node=PersistentRobot(spec)
    try:
        while rclpy.ok() and not node.finished: rclpy.spin_once(node,timeout_sec=.1)
    finally:
        node.stats_log.close(); node.destroy_node(); rclpy.shutdown()


if __name__=='__main__': main()
