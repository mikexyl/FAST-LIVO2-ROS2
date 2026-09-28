"""Persistent owner-isolated, live DDS MapClosures and verification worker."""
import argparse
from collections import deque
import hashlib
import json
from pathlib import Path
import time
import zlib
import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message
from std_msgs.msg import UInt8MultiArray
from .artifacts import canonical, read_json
from .distributed import Worker
from .online_io import Journal, atomic_json
from .ros_dpgo_worker import reliable
from .cbs_bridge import loop_for_robot
from .isolation import restrict_reads, worker_paths


class OnlineRobot(Node):
    def __init__(self,spec):
        self.robot=spec['robot']; self.robots=spec['robots']; self.spec=spec
        super().__init__('online_mapclosures',namespace=self.robot)
        self.output=Path(spec['output']); self.prepared=Path(spec['store']).parent
        self.mailbox=deque(); self.peer_publishers={}; self.subscriptions_=[]
        for peer in self.robots:
            if peer==self.robot: continue
            self.peer_publishers[peer]=self.create_publisher(UInt8MultiArray,
                f'/s3e/live/{self.robot}/to/{peer}',reliable())
            self.subscriptions_.append(self.create_subscription(UInt8MultiArray,
                f'/s3e/live/{peer}/to/{self.robot}',self.receive,reliable()))
        self.worker=Worker(self.robot,spec['store'],spec['descriptors'],spec['backend'],
                           dict(spec['loops'],robots=self.robots,online=True))
        allowed=worker_paths(spec['store'],spec['descriptors'],spec['backend'])
        allowed += [self.output,self.prepared,spec['ros_underlay'],spec['ros_overlay']]
        restrict_reads(allowed)
        self.events=Journal(self.output/'events.jsonl'); self.wire=Journal(self.output/'wire.jsonl')
        self.next=0; self.query=None; self.replies=set(); self.payloads=set()
        self.query_start={}; self.counter=0; self.edges={}; self.done_peers=set()
        self.done_sent=False; self.last_progress=0; self.ready=False
        self.timer=self.create_timer(.005,self.tick)

    def receive(self,msg):
        value=json.loads(zlib.decompress(bytes(msg.data)))
        if value['dst']!=self.robot or value['src'] not in self.robots:
            raise ValueError('Misrouted live message')
        self.mailbox.append(value)

    def send(self,value):
        if value['dst']=='optimizer':
            for peer in sorted({value['body']['i'][0],value['body']['j'][0]}):
                self.send(dict(src=self.robot,dst=peer,kind='constraint',body=value['body']))
            return
        if value['dst']==self.robot: self.mailbox.append(value); return
        encoded=zlib.compress(canonical(value)); msg=UInt8MultiArray(data=encoded)
        self.peer_publishers[value['dst']].publish(msg)
        self.wire.append(dict(src=self.robot,dst=value['dst'],kind=value['kind'],
            network_cdr_bytes=len(serialize_message(msg)),sha256=hashlib.sha256(encoded).hexdigest()))

    def accept(self,edge):
        edge=loop_for_robot(edge,self.robot); key=(tuple(edge['i']),tuple(edge['j']))
        rank=lambda e:(e['diagnostics']['query_stamp_ns'],tuple(e['diagnostics']['query']))
        if key in self.edges and rank(self.edges[key])<=rank(edge): return
        self.edges[key]=edge
        atomic_json(self.output/'constraints.json',dict(available_wall_ns=time.time_ns(),
            constraints=[self.edges[k] for k in sorted(self.edges)]))
        self.events.append(dict(type='constraint_received',i=edge['i'],j=edge['j']))

    def result(self,outgoing,records):
        for record in records:
            if record.get('type')=='verification':
                record.pop('simulated_detection_latency_s',None)
                record['verification_submitted_wall_ns']=record['delivery_ns']
                record['delivery_ns']=time.time_ns()
                record['wall_detection_latency_s']=(record['delivery_ns']-record['query_stamp_ns'])/1e9
            self.events.append(record)
        for message in outgoing:
            if message['kind']=='payload_request':
                self.payloads.add((message['body']['query_id'],message['dst'],message['body']['candidate_id']))
            self.send(message)

    def tick(self):
        self.worker.store.refresh()
        if not self.ready:
            if any(p.get_subscription_count()<1 for p in self.peer_publishers.values()): return
            self.ready=True; atomic_json(self.output/'READY',dict(wall_ns=time.time_ns()))
        # Bounded callback batches leave DDS and local observation polling responsive.
        for _ in range(min(8,len(self.mailbox))):
            event=self.mailbox.popleft(); kind=event['kind']; body=event['body']
            if kind=='constraint': self.accept(body); continue
            if kind=='done': self.done_peers.add(event['src']); continue
            if kind=='candidates': self.replies.discard(event['src'])
            if kind=='payload_response':
                self.payloads.remove((body['query_id'],event['src'],body['candidate_id']))
                event['_verification_id']=self.counter; self.counter+=1
            self.result(*self.worker.handle(event,time.time_ns()))
        for result in self.worker.collect(): self.result(result['outgoing'],result['records'])
        if self.query is not None and not self.replies and not self.payloads:
            self.query=None; self.next+=1
        if self.query is None and self.next<len(self.worker.store.rows):
            row=self.worker.store.row(self.next); self.query=self.next; self.replies=set(self.robots)
            self.query_start[self.next]=time.time_ns()
            self.events.append(dict(type='observe',keyframe_id=self.next,available_ns=row['available_ns'],
                                    anchor_ns=row['stamp_ns']))
            self.result(*self.worker.handle(dict(kind='observe',src=self.robot,body=dict(keyframe_id=self.next)),row['available_ns']))
        if ((self.prepared/'summary.json').exists() and self.next==len(self.worker.store.rows)
                and self.query is None and not self.worker.pending and not self.done_sent):
            self.done_sent=True
            for peer in self.robots: self.send(dict(src=self.robot,dst=peer,kind='done',body={}))
        if self.done_peers==set(self.robots) and not (self.output/'DONE').exists():
            atomic_json(self.output/'DONE',dict(wall_ns=time.time_ns(),observed=self.next,loops=len(self.edges)))
        if time.monotonic()-self.last_progress>1:
            self.last_progress=time.monotonic()
            atomic_json(self.output/'progress.json',dict(observed=len(self.worker.seen),queried=self.next,
                prepared=len(self.worker.store.rows),loops=len(self.edges),mailbox=len(self.mailbox),
                pending_verifications=len(self.worker.pending),done=self.done_sent,wall_ns=time.time_ns()))

    def close(self):
        self.worker.close(); self.events.close(); self.wire.close()


if __name__=='__main__':
    p=argparse.ArgumentParser(); p.add_argument('--spec',type=Path,required=True); a=p.parse_args()
    spec=read_json(a.spec); rclpy.init(); node=OnlineRobot(spec)
    try:
        while rclpy.ok() and not (node.output/'STOP').exists(): rclpy.spin_once(node,timeout_sec=.1)
    finally:
        node.close(); node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
