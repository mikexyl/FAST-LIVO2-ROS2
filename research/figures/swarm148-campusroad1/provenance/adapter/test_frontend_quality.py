import json
import numpy as np
from frontend_quality import assess,check_export


def rows(positions):
    result=[]
    for i,x in enumerate(positions):
        pose=np.eye(4);pose[0,3]=x
        result.append(dict(stamp_ns=1_600_000_000_000_000_000+i*100_000_000,T_world_body=pose.tolist()))
    return result


def test_smooth_motion_and_finite_divergence_are_distinguished():
    assert assess(rows([0,.1,.2,.3]))['passed']
    result=assess(rows([0,.1,.2,100]))
    assert not result['passed']
    assert result['max_speed_m_s']>900
    assert result['first_invalid_stamp_ns']==1_600_000_000_300_000_000


def test_invalid_timestamps_and_nonfinite_positions_fail():
    data=rows([0,.1]);data[1]['stamp_ns']=data[0]['stamp_ns']
    assert not assess(data)['passed']
    assert not assess(rows([0,float('nan')]))['passed']


def test_interrupted_capture_is_not_a_complete_frontend(tmp_path):
    export=tmp_path/'export';export.mkdir()
    (tmp_path/'summary.json').write_text(json.dumps(dict(success=False,error='Interrupted')))
    assert not check_export(export)['passed']
