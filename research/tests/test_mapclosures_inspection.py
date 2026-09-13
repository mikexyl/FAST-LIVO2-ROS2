from pathlib import Path
import sys
import numpy as np
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import s3e_mapclosures_native as detector
import s3e_mapclosures_inspection as inspection
from s3e_pipeline.mapclosures_inspection import check_features,density_pixels


def test_density_grid_and_orb_equal_detector_cache():
    rng=np.random.default_rng(18);cloud=rng.uniform([-55,-55,-1],[55,55,8],(30000,3))
    production=detector.MapClosures(.5,.05,50).describe(cloud)
    debug=inspection.Inspector(.5,.05,50).density(cloud,production['ground'])
    check_features(debug,production)
    assert debug['image'].dtype==np.uint8 and debug['image'].ndim==2
    assert debug['image'].shape==(production['density_rows'],production['density_cols'])
    assert len(debug['orb_xy'])>=len(debug['kept_indices'])>0
    assert np.allclose(density_pixels([[-30,8]],[-32,4]),[[4,2]])
    broken=dict(production,bits=production['bits'].copy());broken['bits'][0,0]^=1
    with pytest.raises(ValueError,match='ORB descriptors'):check_features(debug,broken)


def test_recovered_ransac_membership_equals_original_pose_and_count():
    rng=np.random.default_rng(14);xy=rng.uniform(-80,80,(60,2));bits=rng.integers(0,256,(60,32),dtype=np.uint8)
    R=np.array([[np.cos(.4),-np.sin(.4)],[np.sin(.4),np.cos(.4)]])
    transformed=xy@R.T+[12,-18];transformed[-12:]+=[35,25]
    candidate=dict(xy=xy,bits=bits,ground=np.eye(4));query=dict(candidate,xy=transformed)
    expected=detector.MapClosures(.5,.05,50).pair(query,candidate)
    engine=inspection.Inspector(.5,.05,50);engine.add(0,candidate);actual=engine.correspondences(query,0)
    assert len(actual['query_xy'])==expected['matches']==60
    assert len(actual['ransac_inlier_indices'])==expected['inliers']==48
    assert set(actual['ransac_inlier_indices'])==set(range(48))
    assert np.allclose(actual['T_i_j'],expected['T_i_j'],atol=1e-10)
