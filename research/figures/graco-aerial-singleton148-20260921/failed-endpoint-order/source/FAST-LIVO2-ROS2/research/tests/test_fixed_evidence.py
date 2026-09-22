"""Keep a metric verification gate independent of adaptive transport coarsening."""
import copy
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from s3e_pipeline.registration import bounded_cloud, refine, FeatureCache
from s3e_pipeline.backends import pack_payload, unpack_payload, create
from s3e_pipeline.geometry import transform, inv
from test_pipeline import CFG
from test_mapclosures import features, descriptor


def large_scene():
    rng=np.random.default_rng(134)
    parts=[]
    for axis in range(3):
        p=rng.uniform(-35,35,(60000,3));p[:,axis]=0;parts.append(p)
    return np.vstack(parts)


def test_large_identical_geometry_retains_resolution_and_passes():
    target=large_scene();T=np.eye(4)
    T[:3,:3]=Rotation.from_euler('zyx',[27,5,-3],degrees=True).as_matrix();T[:3,3]=[7,-4,3]
    source=transform(inv(T),target)
    coarse,coarse_voxel=bounded_cloud(target,.4,16000)
    t,tv=bounded_cloud(target,.4,None);s,sv=bounded_cloud(source,.4,None)
    assert len(t)>16000 and tv==sv==.4 and coarse_voxel>.4 and len(coarse)<=16000
    cfg=dict(CFG['backend']['registration'],sampling='fixed',voxel_m=.4,max_points=100)
    result=refine(t,s,T,cfg,FeatureCache())
    assert result['accepted'] and result['rmse_m']<.2 and result['overlap']>.99,result
    assert result['target_voxel_m']==.4 and result['source_voxel_m']==.4
    assert result['target_points']>16000 and result['sampling_policy']=='fixed'


def test_evidence_resolution_survives_transport_and_is_checked():
    cloud=large_scene()[:2000]
    info=dict(sampling_policy='adaptive',requested_voxel_m=.5,effective_voxel_m=1.5,input_points=40000,output_points=len(cloud),max_range_m=80.)
    f=features();d=descriptor(f,[1,0]);d.pop('visual')
    payload=dict(row={},cloud=cloud,image=b'',descriptor=d,evidence_preprocessing=info)
    decoded=unpack_payload(pack_payload(payload))
    assert decoded['evidence_preprocessing']==info
    np.testing.assert_array_equal(decoded['cloud'],cloud)
    cfg=copy.deepcopy(CFG['backend']);cfg.update(name='mapclosures');cfg['registration'].update(sampling='fixed',voxel_m=.4)
    backend=create(cfg)
    try:
        with pytest.raises(ValueError,match='requires evidence'):
            backend.verify(decoded,decoded)
        decoded.pop('evidence_preprocessing')
        with pytest.raises(ValueError,match='requires evidence'):
            backend.verify(decoded,decoded)
    finally:backend.close()
