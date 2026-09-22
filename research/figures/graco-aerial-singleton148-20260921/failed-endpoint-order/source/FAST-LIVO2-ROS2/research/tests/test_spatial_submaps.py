"""Spatial export, causal endpoints, and variable-duration artifact validation."""
import hashlib
import importlib.util
import io
import json
from pathlib import Path
import struct
import numpy as np
import pytest
from s3e_pipeline.submap_writer import consume


def validator():
    path=Path(__file__).resolve().parents[2]/'scripts/recent_submaps/validate.py'
    spec=importlib.util.spec_from_file_location('spatial_validate',path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module.validate


def fixture(root):
    frontend=root/'frontend';frontend.mkdir()
    updates=[];epoch=100*10**9
    for i,x in enumerate([0,10,20,40]):
        stamp=epoch+i*15*10**9
        updates.append(dict(scan_id=i,sensor_stamp_ns=stamp,stamp_ns=stamp,pose=[x,0,0,0,0,0,1],
                            lidar_updated=i>0,active_submap_id=0,age_max_s=i*15.))
    (frontend/'native_updates.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in updates))
    T=np.eye(4);T[0,3]=40
    row=dict(schema_version=3,strategy='spatial',submap_id=0,keyframe_id=0,robot_id='A',
             complete=True,retrievable=True,member_scan_ids=[0,1,2,3],geometry_count=4,ellipsoid_count=0,
             begin_ns=epoch,end_ns=epoch+60*10**9,last_member_ns=epoch+45*10**9,
             stamp_ns=epoch+45*10**9,available_ns=epoch+60*10**9,T_world_imu=T.tolist(),
             origin_world=[0,0,0],up_world=[0,0,1],extent_m=40,radius_m=40,overlap_m=20,
             max_age_s=120,distance_metric='horizontal',finish_reason='radius',
             gravity_imu_m_s2=[0,0,-9.81],gravity_world_m_s2=[0,0,-9.81],gravity_source='ellipselio_filter_at_anchor')
    data=json.dumps(row).encode()
    stream=struct.pack('<I',len(data))+data+np.arange(12,dtype='<f4').tobytes()+b'\0'*4
    consume(io.BytesIO(stream),frontend/'submaps')
    return frontend


def rewrite_index(frontend,change):
    source=frontend/'submaps';row=json.loads((source/'index.jsonl').read_text());change(row)
    (source/'index.jsonl').write_text(json.dumps(row)+'\n')
    manifest=json.loads((source/'manifest.json').read_text())
    manifest['index_sha256']=hashlib.sha256((source/'index.jsonl').read_bytes()).hexdigest()
    (source/'manifest.json').write_text(json.dumps(manifest))


def test_spatial_writer_and_variable_age_validation(tmp_path):
    frontend=fixture(tmp_path)
    result=validator()(tmp_path)
    assert result['completed_snapshots']==result['retrievable_snapshots']==1
    row=json.loads((frontend/'submaps/index.jsonl').read_text())
    assert row['strategy']=='spatial' and row['gravity_source']=='ellipselio_filter_at_anchor'
    with pytest.raises(FileExistsError):consume(io.BytesIO(b'\0'*4),frontend/'submaps')


@pytest.mark.parametrize('field,value,reason',[
    ('extent_m',12,'extent mismatch'),('radius_m',80,'Premature spatial'),
    ('max_age_s',30,'Expired spatial'),('origin_world',[1,0,0],'origin/up'),
    ('retrievable',False,'retrieval admission'),('member_scan_ids',[0,1,2],'membership mismatch')])
def test_spatial_invalid_exports_are_rejected(tmp_path,field,value,reason):
    frontend=fixture(tmp_path);rewrite_index(frontend,lambda r:r.update({field:value}))
    with pytest.raises(ValueError,match=reason):validator()(tmp_path)


def test_spatial_archived_correspondence_is_rejected(tmp_path):
    frontend=fixture(tmp_path);path=frontend/'native_updates.jsonl'
    updates=[json.loads(s) for s in path.read_text().splitlines()]
    updates[1]['age_max_s']=16
    path.write_text(''.join(json.dumps(r)+'\n' for r in updates))
    with pytest.raises(ValueError,match='Archived correspondence'):validator()(tmp_path)


def test_spatial_tail_is_inspection_only(tmp_path):
    frontend=fixture(tmp_path)
    rewrite_index(frontend,lambda r:r.update(complete=False,retrievable=False,
        end_ns=r['last_member_ns']+1,finish_reason='shutdown_or_recovery_tail'))
    result=validator()(tmp_path)
    assert result['inspection_tails']==1 and result['retrievable_snapshots']==0


def test_schema_three_requires_spatial():
    row=dict(schema_version=3,submap_id=0)
    header=json.dumps(row).encode()
    import tempfile
    with tempfile.TemporaryDirectory() as tmp:
        with pytest.raises(ValueError,match='Schema 3'):
            consume(io.BytesIO(struct.pack('<I',len(header))+header),Path(tmp)/'invalid')


def test_spatial_preparation_preserves_membership_gravity_and_causality(tmp_path,monkeypatch):
    from s3e_pipeline import ellipsoid_cuda
    from s3e_pipeline.recent_submaps import prepare
    from s3e_pipeline.backends import unpack_array
    import zlib
    class Sampler:
        def close(self):pass
    monkeypatch.setattr(ellipsoid_cuda,'SurfaceSampler',Sampler)
    frontend=fixture(tmp_path)
    cfg=dict(backend=dict(mapclosures=dict(density_map_resolution=.5,density_threshold=.05,hamming_distance_threshold=50)))
    rows=prepare(frontend/'submaps',tmp_path/'prepared',cfg)
    row=rows[0]
    assert row['stamp_ns']<row['available_ns'] and row['strategy']=='spatial'
    packet=json.loads(zlib.decompress((tmp_path/'prepared/ellipsoid/000000.json.zlib').read_bytes()))
    assert packet['member_scan_ids']==row['member_scan_ids'] and packet['payload_sha256']==row['sha256']
    np.testing.assert_array_equal(unpack_array(packet['mapclosures']['ground']),np.eye(4))
