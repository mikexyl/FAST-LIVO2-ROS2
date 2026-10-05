import copy
from pathlib import Path
import sys
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from s3e_pipeline.trajectory_continuity import continuity_summary


def trajectory():
    result=[]
    for k in range(5):
        T=np.eye(4); T[:3,:3]=Rotation.from_rotvec([.1*k,.02*k,0]).as_matrix()
        T[:3,3]=[.2*k,.01*k*k,2.]
        result.append(dict(robot_id='A',stamp_ns=1600000000000000000+k*100000000,T_world_body=T.tolist()))
    return result


def test_rigid_alignment_adds_no_dense_motion():
    raw=trajectory(); H=np.eye(4); H[:3,:3]=Rotation.from_rotvec([.3,.7,-.8]).as_matrix(); H[:3,3]=[100,-20,8]
    new=[dict(r,T_world_body=(H@np.asarray(r['T_world_body'])).tolist()) for r in raw]
    d=continuity_summary(raw,new)
    assert d['max_correction_added_step_m'] < 1e-12
    assert d['raw_max_speed_m_s'] == pytest.approx(d['corrected_max_speed_m_s'])


def test_abrupt_submap_correction_is_detected():
    raw=trajectory();new=copy.deepcopy(raw)
    for r in new[2:]:r['T_world_body'][0][3]+=1.
    d=continuity_summary(raw,new)
    assert d['max_correction_added_step_m']==pytest.approx(1.)
    assert d['largest_added_step_elapsed_s']==pytest.approx(.1)


def test_mismatched_or_nonchronological_rows_are_rejected():
    raw=trajectory();new=copy.deepcopy(raw);new[2]['stamp_ns']+=1
    with pytest.raises(ValueError,match='timestamps'):continuity_summary(raw,new)
    raw[2]['stamp_ns']=raw[1]['stamp_ns']
    with pytest.raises(ValueError,match='chronological'):continuity_summary(raw,raw)
