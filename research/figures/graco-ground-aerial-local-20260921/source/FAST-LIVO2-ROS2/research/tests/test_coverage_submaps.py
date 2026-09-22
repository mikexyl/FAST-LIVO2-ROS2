"""Observed footprints, immutable schema-4 geometry, and shared-cell evidence."""
import copy
import io
import json
from pathlib import Path
import struct
import numpy as np
import pytest
from s3e_pipeline.submap_writer import consume
from s3e_pipeline.coverage import validate_coverage, validate_overlaps
from test_spatial_submaps import validator, rewrite_index


def fixture(root):
    frontend=root/'frontend';frontend.mkdir()
    epoch=100*10**9
    updates=[dict(scan_id=i,sensor_stamp_ns=epoch+i*15*10**9,stamp_ns=epoch+i*15*10**9,
                  pose=[i,0,30,0,0,0,1],lidar_updated=i>0,active_submap_id=0,age_max_s=i*15.) for i in range(4)]
    (frontend/'native_updates.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in updates))
    T=np.eye(4);T[:3,3]=[3,0,30]
    row=dict(schema_version=4,strategy='coverage',robot_id='A',submap_id=0,keyframe_id=0,
             complete=True,retrievable=True,member_scan_ids=[0,1,2,3],member_point_counts=[2,3,3,3],
             geometry_count=11,ellipsoid_count=0,begin_ns=epoch,end_ns=epoch+60*10**9,
             last_member_ns=epoch+45*10**9,stamp_ns=epoch+45*10**9,available_ns=epoch+60*10**9,
             T_world_imu=T.tolist(),origin_world=[0,0,30],up_world=[0,0,1],extent_m=3,
             distance_metric='horizontal',max_displacement_m=80,max_age_s=120,finish_reason='coverage',
             gravity_imu_m_s2=[0,0,-9.81],gravity_world_m_s2=[0,0,-9.81],gravity_source='ellipselio_filter_at_anchor',
             coverage_origin_world=[50.7,10.2,0],coverage_x_world=[1,0,0],coverage_y_world=[0,1,0],
             coverage_cell_size_m=1,coverage_cell_count=5,coverage_seed_cells=2,occupied_area_m2=5,new_area_m2=3,
             target_area_m2=5,required_new_area_m2=3,min_shared_area_m2=4,min_overlap_ratio=.5,
             shared_area_m2=4,overlap_ratio=.8,overlap_successor_id=1)
    world=np.array([[x+.2,10.2,0] for x in [50,51,50,51,52,51,52,53,52,53,54]])
    pts=(world-[3,0,30]).astype('<f4');cells=np.array([[-1,0,0],[0,0,0],[1,0,1],[2,0,2],[3,0,3]],dtype='<i4')
    tail=dict(row,submap_id=1,keyframe_id=1,complete=False,retrievable=False,member_scan_ids=[2,3],
              member_point_counts=[3,3],geometry_count=6,begin_ns=epoch+30*10**9,end_ns=epoch+45*10**9+1,
              origin_world=[2,0,30],extent_m=1,finish_reason='shutdown_or_recovery_tail',coverage_cell_count=4,
              coverage_seed_cells=3,occupied_area_m2=4,new_area_m2=1,shared_area_m2=0,overlap_ratio=0,overlap_successor_id=-1)
    tail_cells=np.array([[0,0,2],[1,0,2],[2,0,2],[3,0,3]],dtype='<i4')
    packets=[]
    for meta,cloud,grid in [(row,pts,cells),(tail,pts[5:],tail_cells)]:
        header=json.dumps(meta).encode();packets.append(struct.pack('<I',len(header))+header+cloud.tobytes()+grid.tobytes())
    consume(io.BytesIO(b''.join(packets)+b'\0'*4),frontend/'submaps')
    return row,dict(points=pts,coverage_cells=cells),tail,dict(points=pts[5:],coverage_cells=tail_cells)


def test_coverage_export_and_measured_overlap(tmp_path):
    row,data,tail,other=fixture(tmp_path)
    result=validator()(tmp_path)
    assert result['completed_snapshots']==result['inspection_tails']==1
    assert result['retrievable_snapshots']==1
    assert row['coverage_origin_world'][0]>row['origin_world'][0]+40
    cells={0:validate_coverage(row,data),1:validate_coverage(tail,other)}
    validate_overlaps([row,tail],cells)
    with pytest.raises(FileExistsError):consume(io.BytesIO(b'\0'*4),tmp_path/'frontend/submaps')


@pytest.mark.parametrize('field,value,reason',[
    ('occupied_area_m2',100,'area mismatch'),('new_area_m2',100,'area mismatch'),
    ('coverage_seed_cells',4,'seed mismatch'),('required_new_area_m2',4,'Premature coverage'),
    ('member_point_counts',[1,3,3,3],'point counts'),('coverage_x_world',[0,0,0],'frame')])
def test_coverage_rejects_invalid_metadata(tmp_path,field,value,reason):
    row,data,_,_=fixture(tmp_path);row[field]=value
    with pytest.raises(ValueError,match=reason):validate_coverage(row,data)


def test_coverage_cells_must_come_from_returns(tmp_path):
    row,data,_,_=fixture(tmp_path)
    wrong=copy.deepcopy(data);wrong['coverage_cells'][0,0]=100
    with pytest.raises(ValueError,match='observed geometry'):validate_coverage(row,wrong)
    wrong=copy.deepcopy(data);wrong['coverage_cells'][3,2]=0
    with pytest.raises(ValueError,match='first-observation'):validate_coverage(row,wrong)


def test_overlap_uses_only_cells_available_at_retirement(tmp_path):
    row,data,tail,other=fixture(tmp_path)
    cells={0:validate_coverage(row,data),1:validate_coverage(tail,other)}
    cells[1][(200,200)]=99  # Future observation cannot reduce or increase historical overlap.
    validate_overlaps([row,tail],cells)
    row['shared_area_m2']=5
    with pytest.raises(ValueError,match='overlap mismatch'):validate_overlaps([row,tail],cells)


def test_coverage_preparation_preserves_payload_and_frame(tmp_path,monkeypatch):
    from s3e_pipeline import ellipsoid_cuda
    from s3e_pipeline.recent_submaps import prepare
    class Sampler:
        def close(self):pass
    monkeypatch.setattr(ellipsoid_cuda,'SurfaceSampler',Sampler)
    fixture(tmp_path)
    cfg=dict(backend=dict(mapclosures=dict(density_map_resolution=.5,density_threshold=.05,hamming_distance_threshold=50),
                          evidence=dict(sampling='fixed',voxel_m=.4),registration=dict(sampling='fixed',voxel_m=.4)))
    prepared=prepare(tmp_path/'frontend/submaps',tmp_path/'prepared',cfg)
    assert len(prepared)==1 and prepared[0]['strategy']=='coverage'
    assert prepared[0]['coverage_cell_count']==5 and prepared[0]['member_scan_ids']==[0,1,2,3]
