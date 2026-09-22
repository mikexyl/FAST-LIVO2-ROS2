import numpy as np
import pytest
from s3e_pipeline.joint_bev_ransac import pool_correspondences, rigid_fit, fit_consensus, unique_support


def packet(c, q, hamming=10):
    return dict(candidate_xy=np.asarray(c)/.5, query_xy=np.asarray(q)/.5,
                hamming=np.full(len(c), hamming))


def scene():
    rng = np.random.default_rng(21)
    c = rng.uniform(-40, 40, (50, 2)); a = .7
    R = np.array([[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]])
    q = c@R.T+[12, -7]
    return c, q, R


def test_recovers_shared_rigid_pose_amid_large_bad_layer_and_reversed_endpoints():
    c, q, R = scene(); rng = np.random.default_rng(4)
    data = {'low': packet(c[:25], q[:25]), 'middle': packet(c[25:], q[25:]),
            'high': packet(rng.uniform(-60, 60, (200, 2)), rng.uniform(-60, 60, (200, 2)), 40)}
    pool = pool_correspondences(data); result = fit_consensus(pool)
    T = np.asarray(result['T_query_candidate_level_2d'])
    assert result['inliers'] >= 49
    assert np.allclose(T[:2, :2], R, atol=.003)
    assert np.allclose(T[:2, 2], [12, -7], atol=.1)
    assert set(result['layer_support']) == set(data)
    backwards = pool_correspondences({'low': packet(q, c)})
    reverse = np.asarray(fit_consensus(backwards)['T_query_candidate_level_2d'])
    forward = rigid_fit(c, q)
    assert np.allclose(reverse, np.linalg.inv(forward), atol=1e-9)


def test_repeated_height_features_do_not_inflate_support_and_full_control_is_separate():
    c, q, _ = scene()
    pool = pool_correspondences({name: packet(c, q, 10+i) for i, name in enumerate(['near', 'low', 'middle'])})
    assert pool['input_matches'] == 150 and len(pool['layer']) == 50
    assert pool['duplicates_removed'] == 100
    assert all(len(x) == 3 for x in pool['duplicate_groups'])
    assert fit_consensus(pool)['inliers'] == 50
    with pytest.raises(ValueError):
        pool_correspondences({'full': packet(c, q), 'low': packet(c, q)})


def test_many_to_one_support_and_strict_distance_boundary():
    pool = dict(query_xy_m=np.array([[0., 0.], [2., 0.], [5., 0.]]),
                candidate_xy_m=np.array([[0., 0.], [.1, 0.], [5., 0.]]), hamming=np.ones(3))
    assert np.array_equal(unique_support(pool, np.array([0., .1, 1.5]), 1.5, .5), [0])


def test_empty_nonfinite_and_degenerate_inputs():
    empty = packet(np.empty((0, 2)), np.empty((0, 2)))
    assert not fit_consensus(pool_correspondences({'low': empty}))['valid_pose']
    p = packet([[1., 2.]], [[3., 4.]])
    assert not fit_consensus(pool_correspondences({'low': p}))['valid_pose']
    p['query_xy'][0, 0] = np.nan
    with pytest.raises(ValueError):
        pool_correspondences({'low': p})
    with pytest.raises(ValueError):
        rigid_fit(np.zeros((3, 2)), np.ones((3, 2)))


def test_proper_rotation_no_scale_and_final_support_is_rechecked():
    c, q, _ = scene(); rng = np.random.default_rng(8)
    noisy = q+rng.normal(0, .4, q.shape)
    pool = pool_correspondences({'low': packet(c, noisy)})
    result = fit_consensus(pool); T = np.asarray(result['T_query_candidate_level_2d'])
    assert np.isclose(np.linalg.det(T[:2, :2]), 1.)
    errors = np.linalg.norm(pool['candidate_xy_m']@T[:2, :2].T+T[:2, 2]-pool['query_xy_m'], axis=1)
    assert np.all(errors[result['inlier_indices']] < 1.5)
    assert result['layer_support']['low'] == result['inliers']


def test_more_than_five_unique_inliers_gate_is_strict():
    c, q, _ = scene()
    for n, passes in [(5, False), (6, True)]:
        result = fit_consensus(pool_correspondences({'low': packet(c[:n], q[:n])}))
        assert result['inliers'] == n
        assert result['passes_2d_gate'] is passes
