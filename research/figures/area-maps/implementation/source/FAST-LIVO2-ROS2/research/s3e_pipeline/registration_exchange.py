"""Peer-requested GICP submaps after PCM; one immutable local native batch."""
import hashlib
from pathlib import Path
import time

import numpy as np

from .artifacts import digest, file_hash, write_json
from .backends import pack_array, unpack_array
from .mixed_pgo import settings
from .registration import bounded_cloud


class RegistrationExchange:
    def __init__(self, robot, robots, rows, store, output, session, cfg, send):
        self.robot = robot; self.robots = robots; self.rows = rows
        self.store = Path(store); self.output = Path(output); self.output.mkdir(exist_ok=True)
        self.session = session; self.cfg = settings(cfg); self.send_message = send
        self.fingerprint = digest(self.cfg); self.started = None
        self.selected = None; self.owned = []; self.clouds = {}; self.local = {}
        self.requested = set(); self.needed = set(); self.ready = set()
        self.announced = False; self.published = False; self.pending = []
        self.source_records = {}

    def send(self, peer, kind, **body):
        self.send_message(dict(src=self.robot, dst=peer, kind='registration_'+kind,
            body=dict(body, session_id=self.session, configuration=self.fingerprint)))

    def receive(self, event):
        body = event['body']
        if body.get('session_id') != self.session: return
        if event['src'] not in self.robots or event['src'] == self.robot:
            raise ValueError('Invalid registration peer')
        if body.get('configuration') != self.fingerprint:
            raise ValueError('Registration peer configuration mismatch')
        self.pending.append(event)

    def payload(self, key):
        if type(key) is not int or not 0 <= key < len(self.rows):
            raise ValueError('Invalid requested local geometry ID')
        if key not in self.local:
            row = self.rows[key]
            if (row['robot_id'] != self.robot or row['keyframe_id'] != key or
                row.get('cloud_frame') != row.get('body_frame') or not row.get('body_frame') or
                row.get('submap_end_ns') != row['stamp_ns'] or
                row.get('geometry_preprocessing') not in ('causal trailing submap in keyframe IMU frame',
                    'causal completed native submap in anchor IMU frame','causal accumulated area map in snapshot IMU frame')):
                raise ValueError('Invalid registration submap coordinates or time')
            if row.get('strategy')=='area' and self.cfg['max_range_m'] is not None:
                raise ValueError('Area registration geometry must not have a 3D range crop')
            path = self.store/f'{key:06d}.npz'
            with np.load(path, allow_pickle=False) as data:
                cloud, voxel = bounded_cloud(data['cloud'], self.cfg['voxel_m'], self.cfg['max_points'], self.cfg['max_range_m'])
            cloud = np.asarray(cloud, dtype='<f8')
            if len(cloud) < self.cfg['covariance_neighbors']: raise ValueError('Insufficient submap points')
            payload = dict(robot=self.robot, key=key, stamp_ns=row['stamp_ns'], cloud_frame=row['cloud_frame'],
                points=len(cloud), cloud=pack_array(cloud), effective_voxel_m=voxel,
                payload_sha256=hashlib.sha256(cloud.tobytes()).hexdigest())
            self.local[key] = payload
            self.source_records[key] = dict(source_npz=str(path), source_sha256=file_hash(path),
                                           **{k:v for k,v in payload.items() if k != 'cloud'})
        return self.local[key]

    def advance(self, pcm, edges):
        if self.published: return
        if not pcm or pcm['state'] != 'ready': return
        if self.selected is None:
            self.started = time.monotonic()
            decisions = {((self.robots[v['robot_from']], v['key_from']), (self.robots[v['robot_to']], v['key_to'])):
                         v['retained'] for v in pcm['verdicts']}
            self.selected = {k:e for k,e in edges.items() if k[0][0] == k[1][0] or decisions[k]}
            self.owned = [k for k in sorted(self.selected) if k[0][0] == self.robot]
            self.needed = {e for pair in self.owned for e in pair}
            for robot, key in sorted(self.needed):
                if robot == self.robot: self.clouds[robot, key] = self.payload(key)
                else: self.requested.add((robot, key))
            for peer in self.robots:
                keys = sorted(k for r,k in self.requested if r == peer)
                if keys: self.send(peer, 'request', keys=keys)
        for event in self.pending:
            peer = event['src']; body = event['body']; kind = event['kind']
            if kind == 'registration_request':
                keys = body['keys']
                allowed = {j[1] for i,j in self.selected if i[0] == peer and j[0] == self.robot}
                if not isinstance(keys, list) or any(type(k) is not int or k not in allowed for k in keys):
                    raise ValueError('Peer requested geometry outside PCM-retained incident loops')
                for key in sorted(set(keys)): self.send(peer, 'response', payload=self.payload(key))
            elif kind == 'registration_response':
                payload = body['payload']; endpoint = (peer, payload['key'])
                if endpoint not in self.requested or payload['robot'] != peer:
                    raise ValueError('Unrequested or misidentified registration cloud')
                cloud = unpack_array(payload['cloud'])
                if (cloud.dtype != np.dtype('<f8') or cloud.shape != (payload['points'], 3) or
                    not self.cfg['covariance_neighbors'] <= len(cloud) <= self.cfg['max_points'] or
                    not np.isfinite(cloud).all() or type(payload['stamp_ns']) is not int or
                    not payload['cloud_frame'].startswith(peer+'/') or
                    hashlib.sha256(cloud.tobytes()).hexdigest() != payload['payload_sha256']):
                    raise ValueError('Invalid serialized registration cloud')
                if endpoint in self.clouds and self.clouds[endpoint] != payload:
                    raise ValueError('Changed duplicate registration cloud')
                self.clouds[endpoint] = payload
            elif kind == 'registration_ready': self.ready.add(peer)
            else: raise ValueError('Unknown registration message')
        self.pending.clear()
        if not self.announced and self.needed == set(self.clouds):
            self.announced = True; self.ready.add(self.robot)
            for peer in self.robots:
                if peer != self.robot: self.send(peer, 'ready')
        if time.monotonic()-self.started > self.cfg['timeout_s']:
            raise TimeoutError('Registration peer evidence/ready timeout')
        # All suppliers stay alive until every owner has received its evidence.
        # No central coordinator prepares or routes any submap.
        if self.ready != set(self.robots): return
        native_clouds = []
        for index, (endpoint, payload) in enumerate(sorted(self.clouds.items())):
            filename = f'{index:06d}.bin'
            unpack_array(payload['cloud']).tofile(self.output/filename)
            native_clouds.append(dict(endpoint=[self.robots.index(endpoint[0]), endpoint[1]], file=filename,
                                      **{k:v for k,v in payload.items() if k != 'cloud'}))
        spec = dict(schema_version=1, session_id=self.session, robot_id=self.robots.index(self.robot),
            settings=self.cfg, pairs=[dict(i=[self.robots.index(i[0]), i[1]], j=[self.robots.index(j[0]), j[1]]) for i,j in self.owned],
            clouds=native_clouds)
        write_json(self.output/'transport.json', dict(sources=list(self.source_records.values()),
            requested=[list(e) for e in sorted(self.requested)], owned_pairs=len(self.owned),
            cloud_count=len(native_clouds), elapsed_s=time.monotonic()-self.started,
            supplier_completion_barrier=True))
        write_json(self.output/'manifest.json.tmp', spec)
        (self.output/'manifest.json.tmp').replace(self.output/'manifest.json')
        self.published = True
