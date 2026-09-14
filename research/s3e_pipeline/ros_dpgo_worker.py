"""One robot's offline DDS front end and CBS input/output bridge.

Peer watermarks order frozen observations; they contain timestamps, never poses.
The launch process has no routing, retrieval, registration or optimization role.
"""
import argparse
from collections import deque
import hashlib
import json
from pathlib import Path
import resource
import time
import zlib
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.serialization import serialize_message
from std_msgs.msg import Bool, String, UInt8MultiArray
from cbs_ros.msg import Estimate, NodeStats, PcmInputSeal, PcmStatus
from pose_graph_tools_msgs.msg import PoseGraph
from .artifacts import canonical, read_json, read_jsonl, write_json, write_jsonl
from .cbs_bridge import local_graph, loop_for_robot, ros_graph, matrix_pose


def reliable(depth=1000, durable=False):
    return QoSProfile(depth=depth, reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL if durable else DurabilityPolicy.VOLATILE)


class Robot(Node):
    def __init__(self, spec):
        self.spec = spec; self.robot = spec['robot']; self.robots = spec['robots']
        super().__init__('s3e_frontend', namespace=self.robot)
        self.output = Path(spec['output']); self.start = time.monotonic()
        self.rows = read_jsonl(Path(spec['store']) / 'keyframes.jsonl')
        self.nodes, self.odometry = local_graph(self.rows, self.robot, spec['pgo'])
        self.next_index = 0; self.statuses = {}; self.mailbox = deque()
        self.query = None; self.remaining_replies = set(); self.payloads = set()
        self.counter = 0; self.done_sent = False; self.done_peers = set()
        self.graph_sent = False; self.input_complete = False; self.finished = False
        self.edges = {}; self.events = []; self.wire = []; self.stats = []
        self.last_estimate = None; self.last_progress = self.start
        self.graph_messages = 0; self.pcm_status = None
        self.query_wall_start = {}
        self.graph_pub = self.create_publisher(PoseGraph, f'/{self.robot}/cbs/pose_graph', reliable(durable=True))
        self.complete_pub = self.create_publisher(Bool, f'/{self.robot}/cbs/input_complete', reliable(1, True))
        pcm_prefix = f'/cbs_ros_{self.robots.index(self.robot)}/pcm'
        self.seal_pub = self.create_publisher(PcmInputSeal, f'{pcm_prefix}/input_seal', reliable(1, True))
        self.subscriptions_ = [self.create_subscription(Estimate, f'/{self.robot}/cbs/estimate', self.estimate,
                                                       reliable(1, True)),
            self.create_subscription(NodeStats, f'/cbs_ros_{self.robots.index(self.robot)}/stats', self.statistics, reliable())]
        if spec.get('pcm_enabled', False):
            self.subscriptions_.append(self.create_subscription(PcmStatus, f'{pcm_prefix}/status',
                                                               self.pcm_update, reliable(1, True)))
        self.peer_publishers = {}
        for peer in self.robots:
            if peer == self.robot: continue
            # Separate directed links preserve message order up to terminal DONE.
            self.peer_publishers[peer] = self.create_publisher(UInt8MultiArray,
                f'/s3e/loops/{self.robot}/to/{peer}', reliable())
            self.subscriptions_.append(self.create_subscription(UInt8MultiArray,
                f'/s3e/loops/{peer}/to/{self.robot}', self.receive, reliable()))
        self.status_pub = self.create_publisher(String, f'/s3e/loops/{self.robot}/watermark', reliable(1, True))
        for peer in self.robots:
            self.subscriptions_.append(self.create_subscription(String, f'/s3e/loops/{peer}/watermark',
                self.status, reliable(1, True)))
        self.worker = None
        if spec['mode'] == 'peers':
            from .distributed import Worker
            self.worker = Worker(self.robot, spec['store'], spec['descriptors'], spec['backend'],
                                 dict(spec['loops'], robots=self.robots))
        self.frozen = read_jsonl(spec['constraints']) if spec['mode'] == 'frozen' else []
        # ROS middleware loads plugins lazily, so initialize its interfaces first.
        # Restrict filesystem reads for this process and its verifier subprocess.
        from .isolation import restrict_reads, worker_paths
        allowed = worker_paths(spec['store'], spec.get('descriptors', spec['store']), spec.get('backend', {}))
        allowed += [spec['output'], spec['ros_underlay'], spec['ros_overlay']]
        restrict_reads(allowed)
        self.timer = self.create_timer(.002, self.tick)

    def status(self, msg):
        value = json.loads(msg.data)
        if value['robot'] not in self.robots: raise ValueError('Unknown peer')
        prior = self.statuses.get(value['robot'])
        if prior is None or value['index'] >= prior['index']:
            self.statuses[value['robot']] = value

    def publish_status(self):
        row = self.rows[self.next_index] if self.next_index < len(self.rows) else None
        value = dict(robot=self.robot, index=self.next_index, next=None if row is None else row['stamp_ns'])
        self.statuses[self.robot] = value
        msg = String(data=canonical(value).decode()); self.status_pub.publish(msg)
        self.wire.append(dict(kind='watermark', src=self.robot, dst='peers',
            serialized_bytes=len(serialize_message(msg)), network_bytes=len(serialize_message(msg))*(len(self.robots)-1)))

    def receive(self, msg):
        value = json.loads(zlib.decompress(bytes(msg.data)))
        if value['dst'] != self.robot or value['src'] not in self.robots:
            raise ValueError('Misrouted DDS message')
        self.mailbox.append(value)

    def send(self, value):
        if value['dst'] == 'optimizer':
            edge = value['body']
            for peer in sorted({edge['i'][0], edge['j'][0]}):
                self.send(dict(src=self.robot, dst=peer, kind='constraint', body=edge))
            return
        if value['dst'] == self.robot:
            self.mailbox.append(value); return
        encoded = zlib.compress(canonical(value)); msg = UInt8MultiArray(data=encoded)
        self.peer_publishers[value['dst']].publish(msg)
        self.wire.append(dict(src=self.robot, dst=value['dst'], kind=value['kind'],
            serialized_bytes=len(serialize_message(msg)), network_bytes=len(serialize_message(msg)),
            sha256=hashlib.sha256(encoded).hexdigest(), wall_s=time.monotonic()-self.start))

    def accept(self, edge):
        edge = loop_for_robot(edge, self.robot)
        key = (tuple(edge['i']), tuple(edge['j']))
        # Reciprocal proposals choose the earliest query, regardless of verifier
        # completion order. CBS replaces a factor if an earlier proposal arrives.
        rank = lambda e: (e.get('diagnostics', {}).get('query_stamp_ns', 0),
                          tuple(e.get('diagnostics', {}).get('query', e['i'])))
        if key in self.edges and rank(self.edges[key]) <= rank(edge): return
        self.edges[key] = edge
        self.publish_graph([], [edge])

    def publish_graph(self, nodes, edges):
        self.graph_pub.publish(ros_graph(nodes, edges, self.robots))
        self.graph_messages += 1

    def pcm_update(self, msg):
        if msg.session_id != self.spec['pcm_session_id']: return
        if msg.robot_id != self.robots.index(self.robot): raise ValueError('Foreign PCM status')
        if msg.state == 'error': raise RuntimeError(f'Distributed PCM failed: {msg.error}')
        self.pcm_status = dict(session_id=msg.session_id, robot_id=msg.robot_id, state=msg.state,
            graph_messages=msg.graph_messages, own_nodes=msg.own_nodes,
            network_cdr_bytes=msg.network_cdr_bytes,
            compute_s=msg.compute_s, elapsed_s=msg.elapsed_s,
            verdicts=[{k: getattr(v, k) for k in v.get_fields_and_field_types()} for v in msg.verdicts])
        if self.last_estimate and self.last_estimate.finished: self.estimate(self.last_estimate)

    def result(self, outgoing, records):
        for record in records:
            if record.get('type') == 'verification':
                record['wall_detection_latency_s'] = time.monotonic()-self.query_wall_start[record['query'][1]]
        self.events.extend(records)
        for message in outgoing:
            if message['kind'] == 'payload_request':
                self.payloads.add((message['body']['query_id'], message['dst'], message['body']['candidate_id']))
            self.send(message)

    def tick(self):
        if self.finished: return
        if not self.graph_sent:
            if self.graph_pub.get_subscription_count() < 1 or any(p.get_subscription_count() < 1 for p in self.peer_publishers.values()): return
            self.graph_sent = True
            if self.spec['mode'] == 'frozen':
                self.publish_graph(self.nodes, self.odometry)
                for edge in self.frozen: self.accept(edge)
                self.next_index = len(self.rows)
            self.publish_status()
        while self.mailbox:
            event = self.mailbox.popleft(); kind = event['kind']
            if kind == 'constraint': self.accept(event['body']); continue
            if kind == 'done': self.done_peers.add(event['src']); continue
            if self.worker is None: raise ValueError('Retrieval message in frozen mode')
            body = event['body']; now = body.get('query_stamp_ns', body.get('stamp_ns'))
            if kind == 'candidates': self.remaining_replies.discard(event['src'])
            if kind == 'payload_response':
                self.payloads.remove((body['query_id'], event['src'], body['candidate_id']))
                event['_verification_id'] = self.counter; self.counter += 1
            outgoing, records = self.worker.handle(event, now)
            self.result(outgoing, records)
        if self.worker:
            for result in self.worker.collect(): self.result(result['outgoing'], result['records'])
            if self.query is not None and not self.remaining_replies and not self.payloads:
                self.query = None; self.next_index += 1; self.publish_status()
            if self.query is None and len(self.statuses) == len(self.robots):
                candidates = [(s['next'], r, s['index']) for r, s in self.statuses.items() if s['next'] is not None]
                if candidates and min(candidates)[1] == self.robot:
                    stamp, _, key = min(candidates)
                    if key != self.next_index: raise ValueError('Invalid peer watermark')
                    self.publish_graph([self.nodes[key]], self.odometry[key-1:key] if key else [])
                    self.query = key; self.remaining_replies = set(self.robots)
                    self.query_wall_start[key] = time.monotonic()
                    outgoing, records = self.worker.handle(dict(kind='observe', src=self.robot,
                        body=dict(keyframe_id=key)), stamp)
                    self.result(outgoing, records)
        exhausted = len(self.statuses) == len(self.robots) and all(s['next'] is None for s in self.statuses.values())
        if exhausted and not self.done_sent and (self.worker is None or not self.worker.pending):
            self.done_sent = True
            for peer in self.robots: self.send(dict(src=self.robot, dst=peer, kind='done', body={}))
        # DONE is FIFO behind the sender's final constraints on every link.
        if self.done_peers == set(self.robots) and not self.input_complete:
            # Wait for own graph to reach CBS before starting its settling budget.
            if self.spec.get('pcm_enabled', False):
                # Seal names the exact number of graph messages. CBS waits for
                # their receipt before exchanging PCM evidence, across topics.
                self.seal_pub.publish(PcmInputSeal(session_id=self.spec['pcm_session_id'],
                    graph_messages=self.graph_messages, own_nodes=len(self.rows)))
                self.input_complete = True; self.detection_s = time.monotonic()-self.start
            elif self.last_estimate and len(self.last_estimate.keyframe_ids) == len(self.rows):
                self.complete_pub.publish(Bool(data=True)); self.input_complete = True
                self.detection_s = time.monotonic()-self.start
        now = time.monotonic()
        if now-self.last_progress > 5:
            self.last_progress = now
            write_json(self.output/'progress.json', dict(robot=self.robot, observed=self.next_index,
                total=len(self.rows), loops=len(self.edges), input_complete=self.input_complete,
                iteration=None if not self.last_estimate else self.last_estimate.iteration, wall_s=now-self.start))

    def statistics(self, msg):
        self.stats.append({name: getattr(msg, name) for name in msg.get_fields_and_field_types() if name != 'header'})
        if self.last_estimate and self.last_estimate.finished:
            self.estimate(self.last_estimate)

    def estimate(self, msg):
        if msg.robot_id != self.robots.index(self.robot): raise ValueError('Foreign estimate')
        self.last_estimate = msg
        if not msg.finished: return
        # Estimates and statistics have independent DDS topics. Wait for the
        # matching final statistics so the last request/response is counted.
        if not self.stats or self.stats[-1]['iteration'] < msg.iteration: return
        pcm_enabled = self.spec.get('pcm_enabled', False)
        if pcm_enabled and (not self.pcm_status or self.pcm_status['state'] != 'ready'): return
        if list(msg.keyframe_ids) != [r['keyframe_id'] for r in self.rows] or list(msg.stamp_ns) != [r['stamp_ns'] for r in self.rows]:
            raise ValueError('Incomplete or mistimestamped CBS estimate')
        poses = [dict(robot_id=self.robot, keyframe_id=int(key), stamp_ns=int(stamp),
            T_world_body=matrix_pose(p).tolist(), reference_available=msg.reference_available,
            component=self.robots[msg.reference_robot_id])
            for key, stamp, p in zip(msg.keyframe_ids, msg.stamp_ns, msg.poses)]
        write_jsonl(self.output/'poses.jsonl', poses)
        proposals = [self.edges[k] for k in sorted(self.edges)]
        retained = proposals
        if pcm_enabled:
            decisions = {((self.robots[v['robot_from']], v['key_from']),
                          (self.robots[v['robot_to']], v['key_to'])): v for v in self.pcm_status['verdicts']}
            expected = {k for k in self.edges if k[0][0] != k[1][0]}
            if set(decisions) != expected: raise ValueError('PCM verdicts do not cover incident inter-robot loops')
            retained = [e for e in proposals if e['i'][0] == e['j'][0] or
                        decisions[tuple(e['i']), tuple(e['j'])]['retained']]
            write_json(self.output/'pcm.json', self.pcm_status)
        write_jsonl(self.output/'proposed-constraints.jsonl', proposals)
        write_jsonl(self.output/'constraints.jsonl', retained)
        write_jsonl(self.output/'events.jsonl', self.events); write_jsonl(self.output/'wire.jsonl', self.wire)
        write_jsonl(self.output/'stats.jsonl', self.stats)
        write_json(self.output/'summary.json', dict(robot=self.robot, poses=len(poses), loops=len(retained),
            proposed_loops=len(proposals), pcm_enabled=pcm_enabled,
            pcm_network_cdr_bytes=self.pcm_status['network_cdr_bytes'] if pcm_enabled else 0,
            finished=True, reference_available=msg.reference_available, reference_robot_id=msg.reference_robot_id,
            iterations=msg.iteration,
            detection_s=self.detection_s, wall_s=time.monotonic()-self.start,
            max_rss_kib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss,
            verifier_max_rss_kib=0 if self.worker is None else self.worker.verifier_rss,
            loop_network_cdr_bytes=sum(m['network_bytes'] for m in self.wire),
            cbs_last_stats=self.stats[-1] if self.stats else None))
        self.finished = True


def main():
    parser = argparse.ArgumentParser(); parser.add_argument('--spec', type=Path, required=True)
    args = parser.parse_args(); spec = read_json(args.spec)
    rclpy.init(); node = Robot(spec)
    try:
        while rclpy.ok() and not node.finished: rclpy.spin_once(node, timeout_sec=.1)
    finally:
        if node.worker: node.worker.close()
        node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__': main()
