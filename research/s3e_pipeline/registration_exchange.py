"""Peer-requested GICP submaps after PCM; one immutable local native batch."""
import hashlib
from pathlib import Path
import time

import numpy as np

from .artifacts import digest, file_hash, write_json
from .backends import pack_array, unpack_array
from .cbs_bridge import named_pcm_verdicts, native_pair
from .mixed_pgo import settings
from .registration import bounded_cloud


class RegistrationExchange:
    def __init__(self, robot, robots, rows, store, output, session, cfg, send, cache=None):
        self.robot = robot; self.robots = robots; self.rows = rows
        self.store = Path(store); self.output = Path(output); self.output.mkdir(exist_ok=True)
        self.session = session; self.cfg = settings(cfg); self.send_message = send
        self.fingerprint = digest(self.cfg); self.started = None
        self.selected = None; self.owned = []; self.clouds = {}; self.local = {}
        self.requested = set(); self.needed = set(); self.ready = set()
        self.announced = False; self.published = False; self.pending = []
        self.source_records = {}
        self.ellipsoid = self.cfg['factor']=='ellipsoid'
        self.expected = {}
        self.cache = cache
        if cache is not None:
            self.local = cache.setdefault('local', {})
            self.source_records = cache.setdefault('source_records', {})
            cache.setdefault('remote', {})

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
            if not self.ellipsoid and row.get('strategy')=='area' and self.cfg['max_range_m'] is not None:
                raise ValueError('Area registration geometry must not have a 3D range crop')
            path = self.store/f'{key:06d}.npz'
            with np.load(path, allow_pickle=False) as data:
                if self.ellipsoid:
                    from .ellipsoid_backend import prepared_evidence
                    cloud=prepared_evidence(data,row,self.cfg);voxel=self.cfg['ellipsoid_voxel_m']
                else:cloud, voxel = bounded_cloud(data['cloud'], self.cfg['voxel_m'], self.cfg['max_points'], self.cfg['max_range_m'])
            cloud = np.asarray(cloud, dtype='<f8')
            if len(cloud) < (3 if self.ellipsoid else self.cfg['covariance_neighbors']): raise ValueError('Insufficient submap points')
            payload = dict(robot=self.robot, key=key, stamp_ns=row['stamp_ns'], cloud_frame=row['cloud_frame'],
                points=len(cloud), cloud=pack_array(cloud), effective_voxel_m=voxel,
                payload_sha256=hashlib.sha256(cloud.tobytes()).hexdigest())
            if self.ellipsoid:
                payload.update(schema_version=2,geometry_type='native_ellipsoids_v1',
                               evidence_sha256=row['ellipsoid_evidence_sha256'])
                self.validate_endpoint(payload)
            self.local[key] = payload
            self.source_records[key] = dict(source_npz=str(path), source_sha256=file_hash(path),
                                           **{k:v for k,v in payload.items() if k != 'cloud'})
        return self.local[key]

    def validate_endpoint(self,payload):
        endpoint=(payload['robot'],payload['key'])
        expected=self.expected.get(endpoint)
        if (payload.get('schema_version')!=2 or payload.get('geometry_type')!='native_ellipsoids_v1' or
            payload['cloud_frame']!=payload['robot']+'/imu' or
            payload.get('evidence_sha256')!=payload['payload_sha256']):
            raise ValueError('Invalid ellipsoid payload type/frame/hash')
        if expected is not None and any(payload[k]!=expected[k] for k in ('stamp_ns','cloud_frame','payload_sha256')):
            raise ValueError('Ellipsoid evidence differs from verified loop endpoint')

    def advance(self, pcm, edges):
        if self.published: return
        if not pcm or pcm['state'] != 'ready': return
        if self.selected is None:
            self.started = time.monotonic()
            decisions = named_pcm_verdicts(pcm, self.robots)
            self.selected = {native_pair(k, self.robots):e for k,e in edges.items()
                             if k[0][0] == k[1][0] or decisions[k]['retained']}
            if self.ellipsoid:
                for edge in self.selected.values():
                    evidence=edge.get('diagnostics',{}).get('ellipsoid_endpoints',[])
                    if len(evidence)!=2:raise ValueError('Verified loop lacks primitive endpoint provenance')
                    for meta in evidence:
                        endpoint=(meta['robot'],meta['key'])
                        if endpoint in self.expected and self.expected[endpoint]!=meta:
                            raise ValueError('Changed verified endpoint evidence')
                        self.expected[endpoint]=meta
            self.owned = [k for k in sorted(self.selected) if k[0][0] == self.robot]
            self.needed = {e for pair in self.owned for e in pair}
            for robot, key in sorted(self.needed):
                if robot == self.robot: self.clouds[robot, key] = self.payload(key)
                elif self.cache is not None and (robot, key) in self.cache['remote']:
                    self.clouds[robot, key] = self.cache['remote'][robot, key]
                    if self.ellipsoid: self.validate_endpoint(self.clouds[robot, key])
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
                if (cloud.dtype != np.dtype('<f8') or cloud.shape != (payload['points'], 15 if self.ellipsoid else 3) or
                    not (3 if self.ellipsoid else self.cfg['covariance_neighbors']) <= len(cloud) <= self.cfg['ellipsoid_max_count' if self.ellipsoid else 'max_points'] or
                    not np.isfinite(cloud).all() or type(payload['stamp_ns']) is not int or
                    not payload['cloud_frame'].startswith(peer+'/') or
                    hashlib.sha256(cloud.tobytes()).hexdigest() != payload['payload_sha256']):
                    raise ValueError('Invalid serialized registration cloud')
                if self.ellipsoid:
                    from .ellipsoid_registration import validate,projectors
                    projectors(validate(cloud));self.validate_endpoint(payload)
                    if endpoint not in self.expected:raise ValueError('Missing verified endpoint metadata')
                if endpoint in self.clouds and self.clouds[endpoint] != payload:
                    raise ValueError('Changed duplicate registration cloud')
                self.clouds[endpoint] = payload
                if self.cache is not None:
                    old = self.cache['remote'].get(endpoint)
                    if old is not None and old != payload: raise ValueError('Changed persistent geometry')
                    self.cache['remote'][endpoint] = payload
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
            if self.cache is None:
                unpack_array(payload['cloud']).tofile(self.output/filename)
            else:
                cache_dir = self.output.parent/'cache'; cache_dir.mkdir(exist_ok=True)
                filename = payload['payload_sha256']+'.bin'
                cached = cache_dir/filename
                if not cached.exists(): unpack_array(payload['cloud']).tofile(cached)

            native_clouds.append(dict(endpoint=[self.robots.index(endpoint[0]), endpoint[1]], file=filename,
                                      **{k:v for k,v in payload.items() if k != 'cloud'}))
            if self.cache is not None: native_clouds[-1]['storage'] = 'persistent_cache'
        spec = dict(schema_version=2 if self.ellipsoid else 1, session_id=self.session, robot_id=self.robots.index(self.robot),
            settings=self.cfg, pairs=[dict(i=[self.robots.index(i[0]), i[1]], j=[self.robots.index(j[0]), j[1]]) for i,j in self.owned],
            clouds=native_clouds)
        write_json(self.output/'transport.json', dict(sources=list(self.source_records.values()),
            requested=[list(e) for e in sorted(self.requested)], owned_pairs=len(self.owned),
            cloud_count=len(native_clouds), elapsed_s=time.monotonic()-self.started,
            supplier_completion_barrier=True))
        write_json(self.output/'manifest.json.tmp', spec)
        (self.output/'manifest.json.tmp').replace(self.output/'manifest.json')
        self.published = True
