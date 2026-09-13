"""ROS-free artifact decoding and strictly trailing local submaps."""
from collections import deque
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from mcap.reader import NonSeekingReader
from rosbags.typesys import Stores, get_typestore

from .artifacts import read_json, read_jsonl, file_hash, write_jsonl
from .geometry import pose, inv, transform, extrinsic, voxel_downsample


def cloud_array(msg):
    names=['x','y','z','intensity']
    types={1:'i1',2:'u1',3:'i2',4:'u2',5:'i4',6:'u4',7:'f4',8:'f8'}
    fields={f.name:f for f in msg.fields}
    columns=[]
    for name in names:
        field=fields.get(name)
        if field is None:
            if name != 'intensity':
                raise ValueError(f'Cloud missing {name}')
            columns.append(np.zeros(msg.width*msg.height)); continue
        dtype=np.dtype(('>' if msg.is_bigendian else '<')+types[field.datatype])
        columns.append(np.ndarray((msg.height,msg.width),dtype=dtype,buffer=msg.data,
            offset=field.offset,strides=(msg.row_step,msg.point_step)).ravel())
    points=np.column_stack(columns).astype(np.float32)
    return points[np.isfinite(points).all(1)]


def frames(export, verify=True):
    export=Path(export); manifest=read_json(export/'manifest.json')
    if manifest.get('schema_version') != 1 or not manifest.get('complete'):
        raise ValueError('Incomplete or unsupported export')
    if verify and any(file_hash(export/k)!=v for k,v in manifest['files'].items()):
        raise ValueError('Modified odometry export')
    rows=read_jsonl(export/'frames.jsonl')
    types=get_typestore(Stores.ROS2_HUMBLE)
    with (export/'sensors.mcap').open('rb') as f:
        # mcap 1.4's indexed reader with insertion-order queuing expands all
        # chunks before yielding messages. The non-seeking API streams them.
        messages=iter(NonSeekingReader(f).iter_messages(log_time_order=False))
        for row in rows:
            payload={}
            for entry in row['messages']:
                schema, channel, msg=next(messages)
                if channel.topic != entry['topic'] or msg.log_time != row['stamp_ns']:
                    raise ValueError('MCAP/frame index mismatch')
                payload[channel.topic.rsplit('/',1)[-1]]=types.deserialize_cdr(msg.data,schema.name)
            yield row, cloud_array(payload['cloud']), bytes(payload['image'].data)
        if next(messages,None) is not None:
            raise ValueError('Unindexed MCAP messages')


def select(row, last, cfg):
    if last is None:
        return True
    T=inv(pose(last['T_world_body']))@pose(row['T_world_body'])
    return (np.linalg.norm(T[:3,3]) >= cfg['translation_m'] or
        Rotation.from_matrix(T[:3,:3]).magnitude() >= np.deg2rad(cfg['rotation_deg']) or
        row['stamp_ns']-last['stamp_ns'] >= round(cfg['max_interval_s']*1e9))


def keyframes(export, output, cfg, camera_config):
    output=Path(output); output.mkdir(parents=True)
    trailing=deque(); last=None; rows=[]
    for row,cloud,image in frames(export):
        T=pose(row['T_world_body']); B=extrinsic(row)
        body=np.column_stack([transform(B,cloud[:,:3]),cloud[:,3]])
        # Downsample each scan before assembling; retained measurements all precede the query.
        trailing.append((row['stamp_ns'],T,voxel_downsample(body,cfg['voxel_m'])))
        cutoff=row['stamp_ns']-round(cfg['submap_s']*1e9)
        while trailing and trailing[0][0] < cutoff:
            trailing.popleft()
        if not select(row,last,cfg):
            continue
        key=len(rows); cloud_parts=[]
        for stamp,oldT,points in trailing:
            assert stamp <= row['stamp_ns']
            cloud_parts.append(np.column_stack([transform(inv(T)@oldT,points[:,:3]),points[:,3]]))
        submap=voxel_downsample(np.concatenate(cloud_parts),cfg['voxel_m'])
        np.savez_compressed(output/f'{key:06d}.npz',cloud=submap.astype(np.float32),
                            scan=body.astype(np.float32))
        (output/f'{key:06d}.png').write_bytes(image)
        item={k:v for k,v in row.items() if k!='messages'}
        scaled_camera=dict(camera_config)
        sx=row.get('image_width',camera_config['cam_width'])/camera_config['cam_width']
        sy=row.get('image_height',camera_config['cam_height'])/camera_config['cam_height']
        for field in ('cam_fx','cam_cx'):scaled_camera[field]*=sx
        for field in ('cam_fy','cam_cy'):scaled_camera[field]*=sy
        scaled_camera.update(cam_width=round(camera_config['cam_width']*sx),cam_height=round(camera_config['cam_height']*sy),scale=1.0)
        item.update(keyframe_id=key, submap_start_ns=trailing[0][0], submap_end_ns=row['stamp_ns'],
                    submap_points=len(submap), camera=scaled_camera,
                    cloud_frame=row['body_frame'], geometry_preprocessing='causal trailing submap in keyframe IMU frame')
        rows.append(item); last=row
    write_jsonl(output/'keyframes.jsonl',rows)
    if not rows:
        raise ValueError('No keyframes')
    return rows


class LocalStore:
    """One worker's capability: validated local integer IDs, never remote filesystem paths."""
    def __init__(self, root, robot):
        self.root=Path(root).resolve(); self.robot=robot
        self.rows=read_jsonl(self.root/'keyframes.jsonl')
        if any(r['robot_id']!=robot or r['keyframe_id']!=i for i,r in enumerate(self.rows)):
            raise ValueError('Robot store identity mismatch')

    def row(self,key):
        if type(key) is not int or not 0 <= key < len(self.rows):
            raise ValueError('Invalid local keyframe ID')
        return self.rows[key]

    def payload(self,key):
        row=self.row(key)
        with np.load(self.root/f'{key:06d}.npz',allow_pickle=False) as f:
            cloud=f['cloud']
        return row,cloud,(self.root/f'{key:06d}.png').read_bytes()
