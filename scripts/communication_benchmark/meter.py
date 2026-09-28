"""Passive serialized-message metering; no estimator or message contents are changed."""
import collections
import io
import json
import math
import os
import re
import threading
import time
from pathlib import Path

def wall_ns():
    return time.time_ns() if hasattr(time,'time_ns') else int(time.time()*1e9)


def write(path, data):
    path = Path(path)
    tmp = path.with_suffix(path.suffix + '.tmp')
    tmp.write_text(json.dumps(data, indent=2, allow_nan=False) + '\n')
    tmp.replace(path)


def owner(node, method):
    if method == 'dcl':
        m = re.match(r'^/([a-z])/', node)
        return ord(m[1]) - ord('a') if m else None
    if method == 'disco':
        m = re.match(r'^/?jackal(\d+)(?:_|/)', node)
        return int(m[1]) if m else None
    if method == 'swarm':
        m = re.match(r'^/r(\d+)(?:/|$)', node)
        return int(m[1]) if m else None
    if method == 'gac':
        if node == '/plOdometryNode': return 'frontend'
        if node == '/submapManagerNode': return 'backend'
    return None


def category(topic):
    low = topic.lower()
    if any(x in low for x in ['descriptor', 'context_info', 'matches']): return 'retrieval_and_geometry'
    if 'loop' in low: return 'verification_and_loops'
    if any(x in low for x in ['estimate', 'pose_graph', 'optimization', 'optimizer']): return 'optimization'
    if any(x in low for x in ['laser_cloud', 'laser_full', 'laser_odom']): return 'centralized_backend_input'
    return 'control'


def publisher_bus_rows(response):
    """roscpp XMLRPC getBusStats differs from the documented rospy envelope."""
    stats = response[2] if len(response) == 3 and isinstance(response[0], int) else response
    for entry in stats[0]:
        topic, connections = (entry[0], entry[-1])
        for link in connections:
            if len(link) == 5:  # roscpp: id, bytesSent, messageDataSent, numMessages, connected
                yield topic, link[0], link[2], link[3]
            elif len(link) == 4:  # rospy documented connection statistics
                yield topic, link[0], link[1], link[2]
            else:
                raise ValueError('Unsupported native bus statistics layout')


class Totals:
    def __init__(self, out, method):
        self.out, self.method = Path(out), method
        self.start = time.monotonic()
        self.start_wall_ns = wall_ns()
        self.rows = {}
        self.bins = collections.Counter()
        self.lock = threading.RLock()
        self.graph_history = []
        self.errors = collections.Counter()
        self.closed = False
        self.finished = False
        self.lost_events = 0

    def add(self, topic, publisher, peers, size, source_override=None):
        src = source_override if source_override is not None else owner(publisher, self.method)
        if src is None: return
        receivers = sorted(set(x for x in peers if x is not None and x != src), key=str)
        if not receivers: return
        with self.lock:
            key = (topic, publisher, tuple(receivers))
            if key not in self.rows:
                self.rows[key] = dict(topic=topic, publisher=publisher, source_owner=src,
                    recipient_owners=receivers, category=category(topic), messages=0,
                    serialized_publication_bytes=0, peer_payload_bytes=0, max_message_bytes=0)
            r = self.rows[key]
            r['messages'] += 1
            r['serialized_publication_bytes'] += size
            r['peer_payload_bytes'] += size * len(receivers)
            r['max_message_bytes'] = max(r['max_message_bytes'], size)
            self.bins[int(time.monotonic()-self.start)] += size*len(receivers)

    def save(self):
        with self.lock:
            rows = list(self.rows.values())
            data = dict(schema_version=1, method=self.method, finished=self.finished,
                scope='Serialized ROS application publications multiplied by distinct observed remote robot subscribers; observer and same-robot copies excluded. Excludes transport/discovery/retransmission; not NIC bytes or confirmed peer delivery.',
                boundary='centralized frontend-to-backend proxy; not native interrobot traffic' if self.method=='gac' else 'interrobot',
                total_peer_payload_bytes=sum(x['peer_payload_bytes'] for x in rows),
                total_serialized_publication_bytes=sum(x['serialized_publication_bytes'] for x in rows),
                by_category={k:sum(r['peer_payload_bytes'] for r in rows if r['category']==k) for k in sorted(set(x['category'] for x in rows))},
                topics=rows, wall_s=time.monotonic()-self.start,
                one_second_peer_bytes=dict(self.bins), observer_errors=dict(self.errors),
                observer_lost_message_events=self.lost_events, graph_samples=len(self.graph_history))
            write(self.out/'communication-measured.json', data)
            write(self.out/'communication-graphs.json', self.graph_history)
        return data


class ROS1Meter(Totals):
    def __init__(self, out, method):
        super().__init__(out, method)
        import rospy, rosgraph
        self.rospy, self.master = rospy, rosgraph.Master('/communication_audit')
        self.subs = {}
        self.peer_map = {}
        self.publisher_nodes = set()
        self.bus_max = {}
        self.bus_samples=[]
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def refresh(self):
        pubs, subs, services = self.master.getSystemState()
        self.peer_map = {t:set(owner(n,self.method) for n in ns) - {None} for t,ns in subs}
        eligible = []
        for topic,nodes in pubs:
            native = [n for n in nodes if owner(n,self.method) is not None]
            if not native: continue
            if self.method == 'gac' and topic not in ['/laser_cloud_corner_last','/laser_cloud_surf_last','/laser_full_3','/laser_odom_to_init']: continue
            # Subscribe also before peer connections are established; callbacks use live graph.
            if self.method in ['dcl','disco'] and not any(
                s != owner(n,self.method) for n in native for s in self.peer_map.get(topic,set())): continue
            eligible.append((topic,native,sorted(self.peer_map.get(topic,set()),key=str)))
            self.publisher_nodes.update(native)
            if topic not in self.subs:
                def receive(msg, topic=topic):
                    node = msg._connection_header.get('callerid','')
                    if hasattr(msg,'_buff'):size=len(msg._buff)
                    else:
                        buffer=io.BytesIO();msg.serialize(buffer);size=len(buffer.getvalue())
                    self.add(topic,node,self.peer_map.get(topic,set()),size)
                self.subs[topic] = self.rospy.Subscriber(topic,self.rospy.AnyMsg,receive,queue_size=None,buff_size=2**26)
        graph = dict(wall_s=time.monotonic()-self.start, wall_ns=wall_ns(), topics=eligible)
        if not self.graph_history or self.graph_history[-1]['topics'] != eligible: self.graph_history.append(graph)
        self.audit_bus()

    def audit_bus(self):
        # Independent native TCPROS counters, preserving maxima through disconnects.
        import xmlrpc.client
        debug=[]
        uris={}
        for n in self.publisher_nodes:
            try:uris[self.master.lookupNode(n)]=n
            except Exception:pass
        for node in list(self.publisher_nodes):
            try:
                uri = self.master.lookupNode(node)
                proxy = xmlrpc.client.ServerProxy(uri)
                info_response=proxy.getBusInfo('/communication_audit')
                stats_response=proxy.getBusStats('/communication_audit')
                debug.append(dict(node=node,info_response=info_response,stats_response=stats_response))
                info=info_response[2]
                links = {str(x[0]):x for x in info if len(x)>=2}
                for topic, connection, size, count in publisher_bus_rows(stats_response):
                        metadata = links.get(str(connection))
                        if metadata is None: continue
                        peer = uris.get(metadata[1],metadata[1])
                        if owner(peer,self.method) is None or owner(peer,self.method)==owner(node,self.method): continue
                        key = (node,uri,topic,str(connection),peer)
                        old = self.bus_max.get(key,{})
                        self.bus_max[key] = dict(publisher=node,topic=topic,peer_node=peer,
                            native_bytes=max(size,old.get('native_bytes',0)),
                            native_messages=max(count,old.get('native_messages',0)),
                            serialized_body_bytes=max(size-4*count,old.get('serialized_body_bytes',0)))
            except Exception as e:
                self.errors['bus_poll_'+type(e).__name__] += 1
        write(self.out/'communication-native-bus.json',dict(
            scope='Independent publisher TCPROS connection counters. Native bytes include ROS transport framing. Observer/local links excluded. Polling may miss final disconnected links; not substituted for main payload metric.',
            connections=list(self.bus_max.values()),last_native_samples=debug))
        self.bus_samples.append(dict(wall_ns=wall_ns(),serialized_body_bytes=native_logical_totals(self.bus_max.values(),self.method)['total_peer_payload_bytes']))
        write(self.out/'communication-native-timeline.json',self.bus_samples)

    def run(self):
        while not self.closed:
            try:
                self.refresh();self.save()
            except Exception as e: self.errors['poll_'+type(e).__name__] += 1
            for _ in range(5):
                if self.closed: break
                time.sleep(.1)

    def finish(self):
        self.closed=True
        self.thread.join(timeout=4)
        try:self.refresh()
        except Exception as e:self.errors['final_'+type(e).__name__]+=1
        self.finished=True
        self.save()


class ROS2Meter(Totals):
    def __init__(self, node, out, method):
        super().__init__(out,method)
        self.node=node;self.subs={};self.peer_map={};self.publishers={};self.topic_publishers={}
        self.timer=node.create_timer(.5,self.refresh)

    def refresh(self):
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
        from rosidl_runtime_py.utilities import get_message
        graph=[]
        for topic,types in self.node.get_topic_names_and_types():
            if '/cslam/' not in topic or len(types)!=1:continue
            pubs=self.node.get_publishers_info_by_topic(topic)
            subs=self.node.get_subscriptions_info_by_topic(topic)
            node_name=lambda e:e.node_namespace.rstrip('/')+'/'+e.node_name
            peers=set(owner(node_name(e),self.method) for e in subs)-{None}
            self.peer_map[topic]=peers
            for e in pubs:self.publishers[tuple(e.endpoint_gid)]=node_name(e)
            native=[e for e in pubs if owner(node_name(e),self.method) is not None]
            if not native:continue
            self.topic_publishers[topic]=sorted(set(node_name(e) for e in native))
            graph.append((topic,[node_name(e) for e in native],sorted(peers)))
            if topic not in self.subs:
                reliability=ReliabilityPolicy.BEST_EFFORT if any(e.qos_profile.reliability==ReliabilityPolicy.BEST_EFFORT for e in native) else ReliabilityPolicy.RELIABLE
                qos=QoSProfile(history=HistoryPolicy.KEEP_ALL,reliability=reliability,durability=DurabilityPolicy.VOLATILE)
                def receive(msg,topic=topic):
                    native_names=self.topic_publishers.get(topic,[])
                    peers=self.peer_map.get(topic,set())
                    if len(native_names)==1:
                        self.add(topic,native_names[0],peers,len(msg));return
                    fanouts=set(len(peers-{owner(n,self.method)}) for n in native_names)
                    if len(fanouts)==1:
                        count=next(iter(fanouts))
                        self.add(topic,'shared-topic publisher unavailable in installed Humble',
                            ['remote-copy-'+str(i) for i in range(count)],len(msg),source_override='unknown native sender')
                    else:self.errors['ambiguous_shared_topic_fanout']+=1
                kwargs={}
                try:
                    from rclpy.qos_event import SubscriptionEventCallbacks
                    def lost(event):self.lost_events+=event.total_count_change
                    kwargs['event_callbacks']=SubscriptionEventCallbacks(message_lost=lost)
                    self.subs[topic]=self.node.create_subscription(get_message(types[0]),topic,receive,qos,raw=True,**kwargs)
                except Exception as e:
                    self.errors['subscription_'+type(e).__name__]+=1
                    self.subs[topic]=self.node.create_subscription(get_message(types[0]),topic,receive,qos,raw=True)
        if not self.graph_history or self.graph_history[-1]['topics'] != graph:self.graph_history.append(dict(wall_s=time.monotonic()-self.start,wall_ns=wall_ns(),topics=graph))
        self.save()

    def finish(self):
        self.node.destroy_timer(self.timer)
        self.finished=True;self.save()
        aggregate_probe(self.out)


def aggregate_probe(out):
    """Publisher sizes with exact node identity, using observed native recipient topology.

    Counts logical transmissions, including early publications before discovery;
    neither this nor subscriber observation is a confirmed network-delivery metric.
    """
    out=Path(out); peers={}; first={}
    for sample in json.loads((out/'communication-graphs.json').read_text()):
        for topic,publishers,receivers in sample['topics']:
            peers.setdefault(topic,set()).update(receivers)
            first.setdefault(topic,sample.get('wall_ns',0))
    totals=Totals(out,'swarm'); records=[]; serialization_errors=[]
    for file in sorted((out/'tx').glob('*.tsv')):
        for line in file.read_text().splitlines():
            stamp,node,topic,size,error=line.split('\t')
            record=(int(stamp),node,topic,int(size),int(error));records.append(record)
            if record[-1]:serialization_errors.append(dict(file=file.name,line=line))
    if not records: raise ValueError('No native ROS2 publication trace')
    earliest=min(x[0] for x in records); bins=collections.Counter();before_discovery=0;unmapped=collections.Counter()
    for stamp,node,topic,size,error in records:
        if error:continue
        if topic not in peers:unmapped[topic]+=1;continue
        remote=peers[topic]-{owner(node,'swarm')}
        totals.add(topic,node,peers[topic],size)
        bins[(stamp-earliest)//10**9]+=size*len(remote)
        if stamp<first[topic]:before_discovery+=size*len(remote)
    observer=json.loads((out/'communication-measured.json').read_text())
    with totals.lock:
        rows=list(totals.rows.values())
        result=dict(schema_version=1,method='swarm',measurement='native RCL publication probe',
            scope='Actual serialized CDR publication sizes times distinct remote robot owners in the observed native subscription topology. Includes startup publications before discovery; excludes same-robot/observer copies, DDS/IP headers and retransmissions. Not confirmed delivery or NIC bytes.',
            topics=rows,total_peer_payload_bytes=sum(x['peer_payload_bytes'] for x in rows),
            by_category={k:sum(r['peer_payload_bytes'] for r in rows if r['category']==k) for k in sorted(set(x['category'] for x in rows))},
            one_second_peer_bytes=dict(bins),serialization_errors=serialization_errors,
            unmapped_topics=dict(unmapped),before_first_graph_discovery_bytes=before_discovery,
            observer_payload_bytes=observer['total_peer_payload_bytes'],
            observer_errors=observer['observer_errors'],publication_records=len(records),
            complete_probe=not serialization_errors and not unmapped)
    write(out/'communication-publications.json',result)
    return result


def native_logical_totals(connections,method):
    """Sum reconnects for each subscriber node, then de-duplicate robot recipients.

    A ROS1 publisher can have multiple processes subscribed on one robot. Retain
    the largest node counter per remote robot; additionally expose the unmerged
    native TCPROS total, rather than pretending both metrics are identical.
    """
    per_node={}
    for row in connections:
        key=(row['publisher'],row['topic'],row['peer_node'])
        v=per_node.setdefault(key,dict(bytes=0,messages=0))
        v['bytes']+=row['serialized_body_bytes'];v['messages']+=row['native_messages']
    directed={}
    for (publisher,topic,node),value in per_node.items():
        key=(publisher,topic,owner(node,method))
        if key not in directed or directed[key]['bytes']<value['bytes']:directed[key]=value
    rows=[dict(publisher=pub,topic=topic,recipient=peer,category=category(topic),serialized_body_bytes=v['bytes'],messages=v['messages']) for (pub,topic,peer),v in sorted(directed.items(),key=lambda x:str(x[0]))]
    return dict(total_peer_payload_bytes=sum(x['serialized_body_bytes'] for x in rows),
        native_all_peer_connection_bytes=sum(v['bytes'] for v in per_node.values()),topics=rows,
        by_category={c:sum(x['serialized_body_bytes'] for x in rows if x['category']==c) for c in sorted(set(x['category'] for x in rows))})
