import json
import numpy as np
from s3e_pipeline.artifacts import write_json,write_jsonl,read_json
from s3e_pipeline.ellipsoid_full_report import figures
from s3e_pipeline.evo_evaluation import evaluate


def test_missing_gt_does_not_align_or_overlay_disconnected_components(tmp_path):
    report=dict(dataset='/nonexistent/S3E_Laboratory_1',branches={})
    for branch,components in [('raw',{'Alpha':'Alpha','Bob':'Bob','Carol':'Bob'}),
                               ('ellipsoid',{'Alpha':'Alpha','Bob':'Alpha','Carol':'Alpha'})]:
        report['branches'][branch]=dict(trajectory={},components=components)
        path=tmp_path/branch;path.mkdir();records=[]
        for i,robot in enumerate(components):
            for frame in range(4):
                T=np.eye(4);T[:3,3]=[100*i+frame,frame*.5,0]
                records.append(dict(robot_id=robot,component=components[robot],T_world_body=T.tolist()))
        write_jsonl(path/'poses.jsonl',records)
    write_json(tmp_path/'report.json',report);write_json(tmp_path/'inputs.json',{})
    figures(tmp_path)
    view=read_json(tmp_path/'visualization.json')
    assert view['gt_available'] is False
    assert view['components']['raw']=={'Alpha':['Alpha'],'Bob':['Bob','Carol']}
    assert view['components']['ellipsoid']=={'Alpha':['Alpha','Bob','Carol']}
    with np.load(tmp_path/'trajectory-view.npz') as f:
        assert not any(k.startswith('gt_') for k in f.files)
        np.testing.assert_array_equal(f['raw_Bob'][0],[100,0,0])
    assert (tmp_path/'trajectories.png').exists()
    assert not (tmp_path/'ate.png').exists()


def test_endpoint_ids_cannot_be_used_as_trajectory_timestamps(tmp_path):
    poses=[];truth={}
    for robot in ('Alpha','Bob','Carol'):
        truth[robot]=(np.array([0,10**9],dtype=np.int64),np.array([[0.,0.,0.],[1.,2.,3.]]))
        for i in range(4):
            T=np.eye(4);T[0,3]=i
            poses.append(dict(robot_id=robot,component='Alpha',stamp_ns=(1662375655+i)*10**9,T_world_body=T.tolist()))
    assert evaluate(dict(poses=poses),truth,dict(evo_max_diff_s=.05),tmp_path)=={}
    status=read_json(tmp_path/'evaluation.json')['components']['Alpha']
    assert status['available'] is False
    assert all(r['matched_samples']==0 for r in status['robots'].values())
