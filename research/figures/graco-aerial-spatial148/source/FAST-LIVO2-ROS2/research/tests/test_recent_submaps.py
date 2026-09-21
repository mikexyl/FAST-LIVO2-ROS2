import io
import json
import struct
import tempfile
import unittest
from pathlib import Path
import numpy as np
from s3e_pipeline.submap_writer import consume
from s3e_pipeline.distributed import eligible_observation

class RecentSubmaps(unittest.TestCase):
    def stream(self, complete=True):
        row=dict(schema_version=1,submap_id=0,geometry_count=2,ellipsoid_count=0,
                 complete=complete,retrievable=complete,member_scan_ids=[0,1])
        header=json.dumps(row).encode()
        return struct.pack('<I',len(header))+header+np.arange(6,dtype='<f4').tobytes()+b'\0'*4

    def test_snapshot_immutable_and_tail(self):
        with tempfile.TemporaryDirectory() as root:
            out=Path(root)/'snapshots'
            consume(io.BytesIO(self.stream(False)),out)
            row=json.loads((out/'index.jsonl').read_text())
            self.assertFalse(row['retrievable'])
            with np.load(out/row['payload']) as f:
                np.testing.assert_array_equal(f['points'],np.arange(6).reshape(2,3))
            with self.assertRaises(FileExistsError):consume(io.BytesIO(self.stream()),out)

    def test_truncated_stream_never_complete(self):
        with tempfile.TemporaryDirectory() as root:
            out=Path(root)/'snapshots'
            with self.assertRaises(EOFError):consume(io.BytesIO(self.stream()[:-4]),out)
            self.assertFalse((out/'manifest.json').exists())

    def test_availability_and_same_robot_exclusion(self):
        row=dict(stamp_ns=9,available_ns=10,begin_ns=0,end_ns=10)
        self.assertFalse(eligible_observation(row,dict(stamp_ns=9),False,30))
        self.assertTrue(eligible_observation(row,dict(stamp_ns=10),False,30))
        self.assertFalse(eligible_observation(row,dict(stamp_ns=45,anchor_ns=38),True,30))
        self.assertTrue(eligible_observation(row,dict(stamp_ns=45,anchor_ns=39),True,30))
        self.assertFalse(eligible_observation(row,dict(stamp_ns=45,anchor_ns=39,begin_ns=5,end_ns=15),True,30))
        self.assertFalse(eligible_observation(dict(row,retrievable=False),dict(stamp_ns=50),False,30))


def test_submap_graph_uses_anchor_not_availability_and_reversed_endpoints():
    from s3e_pipeline.cbs_bridge import local_graph, loop_for_robot
    from s3e_pipeline.geometry import constraint, reverse_constraint
    rows=[]
    for key in range(3):
        T=np.eye(4);T[0,3]=key
        rows.append(dict(robot_id='Bob',keyframe_id=key,submap_id=key,
                         stamp_ns=100+5*key,available_ns=102+5*key,
                         T_world_body=T.tolist(),member_scan_ids=list(range(key*5,key*5+10))))
    nodes,edges=local_graph(rows,'Bob',dict(odometry_rotation_sigma_deg=1,odometry_translation_sigma_m=.15))
    assert [r['stamp_ns'] for r in nodes]==[100,105,110]
    assert len(edges)==2
    edge=constraint(['Alpha',0],['Bob',2],np.eye(4),np.eye(6),kind='loop',accepted=True)
    reverse=loop_for_robot(reverse_constraint(edge),'Bob')
    assert reverse['i']==edge['i'] and reverse['j']==edge['j']


def test_worker_observation_requires_availability(tmp_path,monkeypatch):
    from s3e_pipeline.distributed import Worker
    from s3e_pipeline.artifacts import write_jsonl
    row=dict(robot_id='Bob',keyframe_id=0,stamp_ns=100,available_ns=105,
             begin_ns=90,end_ns=101,retrievable=True)
    write_jsonl(tmp_path/'keyframes.jsonl',[row])
    import yaml
    backend=yaml.safe_load((Path(__file__).resolve().parents[1]/'configs/square1-ellipselio-mapclosures-cbs.yaml').read_text())['backend']
    worker=Worker('Bob',tmp_path,tmp_path,backend,dict(robots=['Bob'],verification_queue_size=0))
    monkeypatch.setattr(worker,'descriptor',lambda key:dict(dummy=True))
    event=dict(kind='observe',body=dict(keyframe_id=0))
    import pytest
    with pytest.raises(ValueError):worker.handle(event,100)
    outgoing,_=worker.handle(event,105)
    assert outgoing[0]['body']['anchor_ns']==100
    assert outgoing[0]['body']['stamp_ns']==105


def test_descriptor_and_evidence_membership_must_agree(tmp_path):
    from s3e_pipeline.distributed import Worker
    from s3e_pipeline.data import LocalStore
    from s3e_pipeline.artifacts import write_jsonl,write_json
    row=dict(robot_id='Bob',keyframe_id=0,submap_id=9,member_scan_ids=[50,51],sha256='abc')
    write_jsonl(tmp_path/'keyframes.jsonl',[row])
    # Exercise the actual descriptor loader without starting a native retrieval index.
    worker=object.__new__(Worker);worker.store=LocalStore(tmp_path,'Bob');worker.descriptors=tmp_path
    write_json(tmp_path/'000000.json',dict(submap_id=9,member_scan_ids=[50,51],payload_sha256='abc'))
    assert worker.descriptor(0)['member_scan_ids']==[50,51]
    write_json(tmp_path/'000000.json',dict(submap_id=9,member_scan_ids=[51,52],payload_sha256='abc'))
    import pytest
    with pytest.raises(ValueError,match='membership mismatch'):worker.descriptor(0)


def test_prepared_evidence_keeps_anchor_time_and_membership(tmp_path,monkeypatch):
    import hashlib
    import yaml
    from s3e_pipeline.recent_submaps import prepare
    from s3e_pipeline import ellipsoid_cuda
    class Sampler:
        def render(self,centers,axes,basis):return centers,dict(surface_samples=len(centers))
        def close(self):pass
    monkeypatch.setattr(ellipsoid_cuda,'SurfaceSampler',Sampler)
    source=tmp_path/'source';source.mkdir()
    np.savez_compressed(source/'submap-000000.npz',points=np.arange(90,dtype=np.float32).reshape(-1,3)/10,
                        ellipsoids=np.empty((0,15),dtype=np.float32))
    row=dict(schema_version=1,robot_id='Bob',submap_id=0,keyframe_id=0,complete=True,retrievable=True,
             member_scan_ids=[0,1],begin_ns=90,end_ns=110,last_member_ns=100,stamp_ns=102,available_ns=111,
             T_world_imu=np.eye(4).ravel().tolist(),payload='submap-000000.npz',
             gravity_imu_m_s2=[0,0,-9.81],gravity_source='ellipselio_filter_at_anchor',
             sha256=hashlib.sha256((source/'submap-000000.npz').read_bytes()).hexdigest())
    (source/'index.jsonl').write_text(json.dumps(row)+'\n')
    (source/'manifest.json').write_text(json.dumps(dict(complete=True,index_sha256=hashlib.sha256((source/'index.jsonl').read_bytes()).hexdigest())))
    cfg=yaml.safe_load((Path(__file__).resolve().parents[1]/'configs/square1-ellipselio-mapclosures-cbs.yaml').read_text())
    rows=prepare(source,tmp_path/'prepared',cfg)
    assert rows[0]['submap_end_ns']==rows[0]['stamp_ns']==102
    assert rows[0]['last_member_ns']==100 and rows[0]['available_ns']==111
    assert rows[0]['member_scan_ids']==[0,1]
