import numpy as np

TANGENT_ORDER = ['rx', 'ry', 'rz', 'tx', 'ty', 'tz']


def pose(value):
    T = np.asarray(value, dtype=float).reshape(4,4)
    if not np.isfinite(T).all() or not np.allclose(T[3], [0,0,0,1]):
        raise ValueError('Invalid homogeneous transform')
    if not np.allclose(T[:3,:3].T @ T[:3,:3], np.eye(3), atol=1e-5) or np.linalg.det(T[:3,:3]) < 0.999:
        raise ValueError('Invalid SO(3) rotation')
    return T


def inv(T):
    result = np.eye(4)
    result[:3,:3] = T[:3,:3].T
    result[:3,3] = -T[:3,:3].T @ T[:3,3]
    return result


def transform(T, xyz):
    return np.asarray(xyz) @ T[:3,:3].T + T[:3,3]


def extrinsic(row, destination='body'):
    c = row['calibration']
    T = np.eye(4)
    T[:3,:3] = np.array(c[f'R_{destination}_lidar']).reshape(3,3)
    T[:3,3] = c[f't_{destination}_lidar']
    return pose(T)


def voxel_downsample(xyz, size):
    xyz = np.asarray(xyz)
    xyz = xyz[np.isfinite(xyz[:,:3]).all(axis=1)]
    if not len(xyz):
        return xyz
    _, idx = np.unique(np.floor(xyz[:,:3]/size).astype(np.int64), axis=0, return_index=True)
    return xyz[np.sort(idx)]


def skew(v):
    x,y,z=v
    return np.array([[0,-z,y],[z,0,-x],[-y,x,0]])


def adjoint(T):
    A=np.zeros((6,6)); R=T[:3,:3]
    A[:3,:3]=R; A[3:,3:]=R; A[3:,:3]=skew(T[:3,3]) @ R
    return A


def constraint(i, j, T, information, **metadata):
    T=pose(T); information=np.asarray(information).reshape(6,6)
    if not np.isfinite(information).all() or not np.allclose(information,information.T) or np.linalg.eigvalsh(information).min() <= 0:
        raise ValueError('Information must be positive definite')
    if tuple(i)==tuple(j):
        raise ValueError('Self-edge')
    return dict(schema_version=1, i=list(i), j=list(j), T_i_j=T.tolist(),
        information=information.tolist(), tangent_order=TANGENT_ORDER,
        perturbation='right: T_measured * Exp([rotation, translation]); GTSAM Pose3 Logmap residual',
        **metadata)


def reverse_constraint(edge):
    out=dict(edge); T=pose(edge['T_i_j']); A=adjoint(inv(T))
    out.update(i=edge['j'], j=edge['i'], T_i_j=inv(T).tolist(),
               information=(A.T @ np.asarray(edge['information']) @ A).tolist())
    return out


def canonical_edge(edge):
    return edge if tuple(edge['i']) < tuple(edge['j']) else reverse_constraint(edge)


def rigid_align(source, target):
    """One proper SE(3) alignment, with no scale fit."""
    a,b=np.asarray(source),np.asarray(target)
    ac,bc=a.mean(0),b.mean(0)
    U,_,Vt=np.linalg.svd((a-ac).T@(b-bc))
    R=Vt.T @ np.diag([1,1,np.linalg.det(Vt.T@U.T)]) @ U.T
    T=np.eye(4); T[:3,:3]=R; T[:3,3]=bc-R@ac
    return T
