"""Exercise the new batch use of the unchanged, tested CBS GPU factor wrapper."""
import copy
import subprocess
import numpy as np
import pytest
from test_mixed_pgo import invoke, BINARY, native
from test_pipeline import T
from s3e_pipeline.artifacts import read_json, write_json
from s3e_pipeline.geometry import inv
from s3e_pipeline.mixed_pgo import settings


@pytest.mark.parametrize('reverse',[False,True])
def test_gpu_batch_correction_gauge_and_reversed_pair(native,tmp_path,reverse):
    version=read_json(BINARY.parent/'build.json')['versions']
    if 'vgicp_gpu' not in version.get('supported_factors',[]):pytest.skip('CUDA batch adapter required')
    _,truth,nodes=invoke(tmp_path,reverse=reverse)
    spec=read_json(tmp_path/'input.json')
    spec['settings']=settings(dict(enabled=True,factor='vgicp_gpu',max_information_ratio=100.,num_threads=2))
    write_json(tmp_path/'input.json',spec)
    subprocess.run([str(BINARY),str(tmp_path/'input.json'),str(tmp_path/'gpu.json')],check=True,timeout=60)
    result=read_json(tmp_path/'gpu.json')
    assert result['success'] and result['device']=='CUDA' and result['registration_factor_count']==1
    final={r['key']:np.array(r['T_world_body']) for r in result['poses']}
    baseline={r['key']:np.array(r['T_world_body']) for r in result['baseline_poses']}
    after=np.linalg.norm(inv(final[0])@final[1]-truth)
    before=np.linalg.norm(inv(baseline[0])@baseline[1]-truth)
    assert after<before,(before,after)
    np.testing.assert_allclose(final[2],nodes[2]['T_world_body'],atol=1e-8)
    assert result['registrations'][0]['gpu']['gpu_linearization_batches']>1
    changed=copy.deepcopy(spec); gauge=T(100,-30,7,.2)
    for row in changed['nodes']:row['T_world_body']=(gauge@np.array(row['T_world_body'])).tolist()
    for row in changed['pose_factors']:
        if row['kind']=='anchor':row['measurement']=(gauge@np.array(row['measurement'])).tolist()
    write_json(tmp_path/'gauge.json',changed)
    subprocess.run([str(BINARY),str(tmp_path/'gauge.json'),str(tmp_path/'gauge-out.json')],check=True,timeout=60)
    shifted={r['key']:np.array(r['T_world_body']) for r in read_json(tmp_path/'gauge-out.json')['poses']}
    np.testing.assert_allclose(inv(shifted[0])@shifted[1],inv(final[0])@final[1],atol=1e-5)
