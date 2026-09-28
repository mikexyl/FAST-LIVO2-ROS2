"""Causal display transforms shared by the live Rerun and video recorder."""
import numpy as np
from .geometry import inv
from .gravity_bev import gravity_ground


def corrections(rows, estimates):
    selected=[(row,np.asarray(estimates[row['keyframe_id']])) for row in rows if row['keyframe_id'] in estimates]
    if not selected: return np.empty(0,dtype=np.int64),np.empty((0,4,4))
    stamps=np.array([r['stamp_ns'] for r,_ in selected],dtype=np.int64)
    matrices=np.array([T@inv(np.asarray(r['T_world_body'])) for r,T in selected])
    return stamps,matrices


def corrected(points,stamps,anchors,transforms):
    points=np.asarray(points)
    if not len(anchors): return points.copy()
    indices=np.clip(np.searchsorted(anchors,stamps,side='right')-1,0,len(anchors)-1)
    selected=transforms[indices]
    return np.einsum('nij,nj->ni',selected[:,:3,:3],points)+selected[:,:3,3]


def component_level(rows,estimates):
    if not rows: return np.eye(4)
    row=next((r for r in rows if r['keyframe_id'] in estimates),rows[0])
    T=np.asarray(estimates.get(row['keyframe_id'],row['T_world_body']))
    return gravity_ground(T[:3,:3]@np.asarray(row['gravity_imu_m_s2']))
