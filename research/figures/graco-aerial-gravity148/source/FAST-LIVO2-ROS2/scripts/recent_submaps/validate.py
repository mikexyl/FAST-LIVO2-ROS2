#!/usr/bin/env python3
"""Sensor-only artifact audit; never reads ground truth."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation


def validate(path):
    path=Path(path)
    updates=[json.loads(line) for line in (path/'frontend/native_updates.jsonl').read_text().splitlines()]
    if not updates:raise ValueError('No exact native updates')
    for i,row in enumerate(updates):
        if row['scan_id']!=i or not np.isfinite(row['pose']).all():raise ValueError('Invalid native trajectory')
        if i and (row['stamp_ns']<=updates[i-1]['stamp_ns'] or row['sensor_stamp_ns']<=updates[i-1]['sensor_stamp_ns']):
            raise ValueError('Nonchronological native updates')
    source=path/'frontend/submaps'; count=0;complete=0
    if source.exists():
        manifest=json.loads((source/'manifest.json').read_text())
        if not manifest['complete'] or hashlib.sha256((source/'index.jsonl').read_bytes()).hexdigest()!=manifest['index_sha256']:
            raise ValueError('Invalid snapshot manifest')
        rows=[json.loads(line) for line in (source/'index.jsonl').read_text().splitlines()]
        if manifest['submaps']!=len(rows):raise ValueError('Snapshot count mismatch')
        for row in rows:
            members=[x for x in updates if row['begin_ns']<=x['sensor_stamp_ns']<row['end_ns']]
            if [x['scan_id'] for x in members]!=row['member_scan_ids']:raise ValueError('Window membership mismatch')
            last=members[-1]
            if row['last_member_ns']!=last['sensor_stamp_ns'] or row['stamp_ns']!=last['stamp_ns']:
                raise ValueError('Anchor timestamp mismatch')
            T=np.eye(4);T[:3,:3]=Rotation.from_quat(last['pose'][3:]).as_matrix();T[:3,3]=last['pose'][:3]
            if not np.allclose(T,np.array(row['T_world_imu']).reshape(4,4),atol=1e-10):raise ValueError('Anchor pose mismatch')
            if row['available_ns']<row['last_member_ns'] or (row['retrievable'] and not row['complete']):
                raise ValueError('Noncausal snapshot')
            if row['schema_version']!=1:raise ValueError('Expected a temporal submap snapshot')
            if row['retrievable']!=row['complete']:
                raise ValueError('Incorrect temporal-submap retrieval admission')
            if row['complete'] and row['available_ns']<row['end_ns']:raise ValueError('Premature completion')
            payload=source/row['payload']
            if hashlib.sha256(payload.read_bytes()).hexdigest()!=row['sha256']:raise ValueError('Payload hash mismatch')
            with np.load(payload,allow_pickle=False) as data:
                if data['points'].shape!=(row['geometry_count'],3) or data['ellipsoids'].shape!=(row['ellipsoid_count'],15):
                    raise ValueError('Geometry size mismatch')
                if not all(np.isfinite(data[k]).all() for k in data.files):raise ValueError('Nonfinite geometry')
            count+=1;complete+=row['complete']
        if any(row['age_max_s']>10.+1e-6 for row in updates if row['lidar_updated']):
            raise ValueError('Archived correspondence age')
    result=dict(native_updates=len(updates),snapshots=count,completed_snapshots=complete,inspection_tails=count-complete,
                retrievable_snapshots=sum(r['retrievable'] for r in rows) if source.exists() else 0,
                finite_chronological=True,exact_membership=True,anchor_poses_verified=True,payload_hashes_verified=True)
    (path/'artifact-validation.json').write_text(json.dumps(result,indent=2)+'\n')
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('trial',type=Path)
    print(json.dumps(validate(parser.parse_args().trial),indent=2))
