from pathlib import Path
import sys
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from s3e_pipeline.circle_central_evaluation import correction_anchors,corrected_union


def test_duplicate_births_have_finite_mean_trajectory_correction():
    keys=[];poses=[]
    for k,(stamp,shift) in enumerate([(1,1),(1,3),(2,4)]):
        T=np.eye(4);corrected=np.eye(4);corrected[0,3]=shift
        keys.append(dict(stamp_ns=stamp,keyframe_id=k,T_world_body=T.tolist()))
        poses.append(dict(keyframe_id=k,T_world_body=corrected.tolist()))
    a,p,d=correction_anchors(keys,poses)
    assert [r['stamp_ns'] for r in a]==[1,2]
    np.testing.assert_allclose(np.asarray(p[0]['T_world_body'])[:3,3],[2,0,0])
    assert d[0]['max_translation_correction_deviation_m']==1


def test_map_uses_individual_spatial_nodes_once_and_flags_uncovered():
    keys=[];poses=[]
    for k,x in enumerate([0,4]):
        T=np.eye(4);corrected=np.eye(4);corrected[2,3]=k+1
        keys.append(dict(keyframe_id=k,T_world_body=T.tolist(),T_horizontal_map=T.tolist(),
            footprint_center_xy=[x,0],footprint_radius_m=3))
        poses.append(dict(keyframe_id=k,T_world_body=corrected.tolist()))
    points=np.array([[0,0,0],[3,0,20],[20,0,-5]],float)
    result,owner,uncovered=corrected_union(points,keys,poses)
    assert len(result)==len(points)
    assert owner.tolist()==[0,1,1] and uncovered.tolist()==[False,False,True]
    np.testing.assert_allclose(result[:,2],[1,22,-3])


def test_verified_singleton_bridges_initialize_central_graph():
    from s3e_pipeline.geometry import constraint
    from s3e_pipeline.pgo import optimize
    rows=[dict(robot_id=r,keyframe_id=0,stamp_ns=1,T_world_body=np.eye(4).tolist()) for r in ('A','B','C')]
    T=np.eye(4);T[0,3]=10
    loops=[constraint([a,0],[b,0],T,np.eye(6)*10,accepted=True,kind='loop') for a,b in [('A','B'),('B','C')]]
    cfg=dict(odometry_rotation_sigma_deg=1.,odometry_translation_sigma_m=.15,
        alignment_translation_m=2.,alignment_rotation_deg=15.,alignment_min_support=1,gnc_probability=.99)
    result=optimize(rows,loops,cfg)
    assert result['components']==dict(A='A',B='A',C='A')
    assert sum(f['kind']=='loop' and f['solver_weight']>0 for f in result['factors'])==2
