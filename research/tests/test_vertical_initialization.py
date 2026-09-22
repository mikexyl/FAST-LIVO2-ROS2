import copy
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from s3e_pipeline.geometry import transform
from s3e_pipeline.gravity_bev import gravity_ground
from s3e_pipeline.vertical_initialization import initialize_vertical


def scene():
    rng=np.random.default_rng(241)
    points=rng.uniform([-10,-8,-2],[11,9,2],(5000,3))
    points[:,2]=.03*points[:,0]+.02*points[:,1]
    return points


@pytest.mark.parametrize('dz',[-17.,0.,12.5])
def test_height_with_tilt_yaw_translation_and_reverse(dz):
    source_level=scene();Gq=gravity_ground([.3,7,-7]);Gc=gravity_ground([1,-6,-8])
    planar=np.eye(4);planar[:3,:3]=Rotation.from_euler('z',.4).as_matrix();planar[:2,3]=[4,-3]
    truth=planar.copy();truth[2,3]=dz
    source=transform(np.linalg.inv(Gc),source_level)
    target=transform(np.linalg.inv(Gq),transform(truth,source_level))
    initial=np.linalg.inv(Gq)@planar@Gc;expected=np.linalg.inv(Gq)@truth@Gc
    before=(target.copy(),source.copy(),initial.copy())
    actual,d=initialize_vertical(target,source,initial,Gq,Gc)
    reverse,_=initialize_vertical(source,target,np.linalg.inv(initial),Gc,Gq)
    np.testing.assert_allclose(actual,expected,atol=.26)
    np.testing.assert_allclose(reverse,np.linalg.inv(expected),atol=.26)
    np.testing.assert_allclose(actual[:3,:3],initial[:3,:3],atol=1e-14)
    np.testing.assert_allclose((Gq@actual@np.linalg.inv(Gc))[:2,3],planar[:2,3],atol=1e-12)
    for a,b in zip((target,source,initial),before):np.testing.assert_array_equal(a,b)
    assert not d['ground_truth_used'] and d['status']=='estimated'


@pytest.mark.parametrize('source',[np.empty((0,3)),np.array([[np.nan,0,0]]),np.array([[1e3,1e3,0]])])
def test_missing_geometry_or_horizontal_support_keeps_seed(source):
    seed=np.eye(4);seed[2,3]=5
    actual,d=initialize_vertical(scene(),source,seed,np.eye(4),np.eye(4))
    np.testing.assert_array_equal(actual,seed)
    assert d['status'] in ('insufficient_geometry','no_horizontal_support')


def test_one_point_and_invalid_parameters_are_safe():
    actual,d=initialize_vertical([[0,0,9]],[[0,0,0]],np.eye(4),np.eye(4),np.eye(4))
    assert actual[2,3]==9 and d['height_vote_pairs']==1
    for options in ({'voxel_m':0},{'xy_neighbors':0},{'modal_peaks':1.5},{'unknown':True}):
        with pytest.raises(ValueError):initialize_vertical(scene(),scene(),np.eye(4),np.eye(4),np.eye(4),options)
    invalid=np.eye(4);invalid[0,0]=np.nan
    with pytest.raises(ValueError):initialize_vertical(scene(),scene(),invalid,np.eye(4),np.eye(4))


def test_runtime_wiring_preserves_disabled_seed_and_requires_provenance(monkeypatch):
    from s3e_pipeline import mapclosures as module
    from s3e_pipeline.backends import pack_array
    backend=object.__new__(module.MegaLocMapClosures)
    backend.visual_enabled=False;backend.name='mapclosures';backend.options={'inliers_threshold':5}
    backend.cfg={'registration':{}};backend.geometry_cache=None;backend.multilayer=None
    f=dict(ground=np.eye(4),xy=np.zeros((0,2)),bits=np.zeros((0,32),dtype=np.uint8))
    descriptor=dict(mapclosures={k:pack_array(v) for k,v in f.items()},
                    projection_alignment={'projection':'orthographic gravity-horizontal'})
    source=scene();target=source+[0,0,12]
    q=dict(descriptor=descriptor,cloud=target);c=dict(descriptor=descriptor,cloud=source)
    hypothesis=dict(valid_pose=True,inliers=8,T_i_j=np.eye(4).tolist())
    seen=[]
    def refine(t,s,seed,*_):seen.append(seed.copy());return dict(accepted=False,reason='sentinel')
    monkeypatch.setattr(module,'refine',refine)
    result=backend.verify(q,c,{'mapclosures_hypothesis':hypothesis})
    np.testing.assert_array_equal(seen[-1],np.eye(4));assert 'vertical_initialization' not in result
    backend.options['vertical_initialization']={'enabled':True}
    result=backend.verify(q,c,{'mapclosures_hypothesis':hypothesis})
    assert seen[-1][2,3]==12 and result['vertical_initialization']['correction_m']==12
    assert result['reason']=='sentinel' and not result['accepted']
    missing=copy.deepcopy(q);missing['descriptor'].pop('projection_alignment')
    with pytest.raises(ValueError,match='gravity-horizontal'):backend.verify(missing,c,{'mapclosures_hypothesis':hypothesis})
