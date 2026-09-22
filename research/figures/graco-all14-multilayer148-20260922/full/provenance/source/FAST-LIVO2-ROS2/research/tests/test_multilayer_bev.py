import numpy as np
import pytest
from s3e_pipeline.multilayer_bev import fit_terrain,relative_heights,layer_masks


def scene():
    rng=np.random.default_rng(17)
    xy=rng.uniform(-45,45,(50000,2));z=.025*xy[:,0]-.01*xy[:,1]-1.2+rng.normal(0,.025,len(xy))
    ground=np.c_[xy,z]
    # A very dense upper surface occupies a much smaller horizontal area.
    roofxy=rng.uniform(-12,12,(90000,2));roof=np.c_[roofxy,np.full(len(roofxy),8.)]
    return np.r_[ground,roof]


def test_sparse_ground_survives_dense_roof_and_vertical_origin_change():
    p=scene();coef,diag,_,_=fit_terrain(p)
    assert np.allclose(coef[:2],[.025,-.01],atol=.002)
    assert abs(coef[2]+1.2)<.12
    shifted=p+[0,0,-27.3];other,_,_,_=fit_terrain(shifted)
    assert np.allclose(other[:2],coef[:2],atol=1e-10)
    assert abs(other[2]-coef[2]+27.3)<1e-10
    assert np.allclose(relative_heights(p,coef),relative_heights(shifted,other),atol=1e-10)
    assert diag['independent_per_submap']


def test_layer_boundaries_are_disjoint_and_missing_support_is_not_fabricated():
    h=np.array([-1.,-.5,1.999,2.,4.999,5.,9.999,10.,19.999,20.,100.])
    masks=layer_masks(h);counts=np.stack(list(masks.values())).sum(axis=0)
    assert np.array_equal(counts,[0,1,1,1,1,1,1,1,1,1,1])
    assert masks['low'][3] and masks['middle'][5] and masks['upper'][9]


def test_narrow_or_nonfinite_ground_is_rejected():
    rng=np.random.default_rng(2);x=rng.uniform(-80,80,20000)
    p=np.c_[x,rng.uniform(-.1,.1,len(x)),np.zeros(len(x))]
    with pytest.raises(ValueError):fit_terrain(p)
    p[0,0]=np.nan
    with pytest.raises(ValueError):fit_terrain(p)
