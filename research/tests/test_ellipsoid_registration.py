"""Native fitted primitives are the loop registration evidence."""
import numpy as np
import pytest
from types import SimpleNamespace
from scipy.spatial.transform import Rotation

from s3e_pipeline.backends import create, pack_array, pack_payload, unpack_payload
from s3e_pipeline.ellipsoid_registration import bounded_ellipsoids, projectors, refine_ellipsoids, right_jacobian


CFG = dict(min_inliers=100, correspondence_m=1.5, inlier_m=.6,
           min_initial_overlap=.1, min_overlap=.3, max_rmse_m=.35,
           min_observability=1e-4, max_condition=1e6,
           translation_sigma_floor_m=.2, rotation_sigma_floor_deg=2.,
           max_iterations=40)


def scene():
    rng = np.random.default_rng(42)
    centers = rng.uniform(-8, 8, (1300, 3))
    axes = np.array([[1., .7, .16], [1., .18, .15], [.2, .18, .16]])
    choice = rng.integers(0, 3, len(centers))
    basis = np.tile(np.eye(3).reshape(1, 9), (len(centers), 1))
    target = np.column_stack((centers, axes[choice], basis))
    T = np.eye(4)
    T[:3, :3] = Rotation.from_euler('xyz', [.02, -.03, .08]).as_matrix()
    T[:3, 3] = [.35, -.18, .4]
    source = target.copy()
    source[:, :3] = (centers - T[:3, 3]) @ T[:3, :3]
    source[:, 6:] = np.einsum('ij,njk->nik', T[:3, :3].T,
                               target[:, 6:].reshape(-1, 3, 3)).reshape(-1, 9)
    return target, source, T


def test_native_projection_weights():
    e = np.array([[0, 0, 0, 1., 1., .1, *np.eye(3).ravel()]])
    np.testing.assert_allclose(projectors(e)[0], np.diag([.1, .1, 1.]), atol=1e-12)
    e[0, 3:6] = [1., .1, .1]
    np.testing.assert_allclose(projectors(e)[0], np.diag([.1, 1., 1.]), atol=1e-12)
    e[0, 3:6] = [1., 1., 1.]
    np.testing.assert_allclose(projectors(e)[0], np.eye(3))


def test_primitive_jacobian_uses_right_pose_tangent():
    p=np.array([[2.,-1.,.7]])
    R=Rotation.from_euler('xyz',[.1,-.2,.3]).as_matrix()
    P=np.diag([.2,.7,1.]).reshape(1,3,3)
    J=right_jacobian(p,R,P)[0]
    for axis in range(6):
        delta=np.zeros(6);delta[axis]=1e-6
        moved=R@(Rotation.from_rotvec(delta[:3]).apply(p[0])+delta[3:])
        base=R@p[0]
        np.testing.assert_allclose((P[0]@(moved-base))/1e-6,J[:,axis],atol=2e-6)


def test_refines_both_directions_and_exposes_weak_geometry():
    target, source, truth = scene()
    initial = truth.copy(); initial[:3, 3] += [.12, -.08, .09]
    result = refine_ellipsoids(target, source, initial, CFG)
    assert result['accepted'], result['reason']
    np.testing.assert_allclose(result['T_i_j'], truth, atol=1e-5)
    assert result['inliers'] >= 1200 and result['reverse_inliers'] >= 1200
    assert result['registration_method'] == 'native_ellipsoid_primitives'
    np.linalg.cholesky(result['information'])
    reverse = refine_ellipsoids(source, target, np.linalg.inv(initial), CFG)
    assert reverse['accepted'], reverse['reason']
    np.testing.assert_allclose(reverse['T_i_j'], np.linalg.inv(truth), atol=1e-5)
    flat = target.copy(); flat[:, 2] = 0.; flat[:, 3:6] = [1e8, 1e8, .01]
    result = refine_ellipsoids(flat, flat, np.eye(4), CFG)
    assert not result['accepted'] and result['reason'] == 'unobservable'


def test_low_overlap_invalid_geometry_and_evidence_transport():
    target, source, truth = scene()
    far = source.copy(); far[:, :3] += 50
    assert refine_ellipsoids(target, far, truth, CFG)['reason'] == 'initial_low_overlap'
    invalid = target.copy(); invalid[0, 3] = 0
    with pytest.raises(ValueError, match='axes'):bounded_ellipsoids(invalid)
    bounded = bounded_ellipsoids(target, .4, 200)
    assert len(bounded) == 200
    assert all(any(np.array_equal(row, native) for native in target) for row in bounded)
    packet = dict(row={'robot_id':'A'}, cloud=np.empty((0, 3)), image=b'',
                  descriptor={}, ellipsoids=bounded)
    restored = unpack_payload(pack_payload(packet))
    np.testing.assert_array_equal(restored['ellipsoids'], bounded)
    assert len(restored['cloud']) == 0


def test_mapclosures_verifies_with_primitives_and_no_point_cloud():
    import s3e_mapclosures_native as native
    target, source, truth = scene()
    descriptor = dict(mapclosures={k:pack_array(v) for k,v in dict(
        ground=np.eye(4), xy=np.empty((0,2)), bits=np.empty((0,32),dtype=np.uint8)).items()})
    backend = create(dict(name='mapclosures', registration=dict(CFG, method='ellipsoid'),
        mapclosures=dict(upstream_commit=native.upstream_commit, density_map_resolution=.5,
                         density_threshold=.05, hamming_distance_threshold=50,
                         inliers_threshold=5, max_hypotheses_per_query=20)))
    initial=truth.copy();initial[:3,3]+=[.12,-.08,.09]
    payload=lambda e:dict(descriptor=descriptor,ellipsoids=e,cloud=np.empty((0,3)))
    result=backend.verify(payload(target),payload(source),
        dict(mapclosures_hypothesis=dict(valid_pose=True,inliers=10,T_i_j=initial.tolist()),sources=['mapclosures']))
    assert result['accepted'] and result['registration_method']=='native_ellipsoid_primitives'
    np.testing.assert_allclose(result['T_i_j'],truth,atol=1e-5)


def test_worker_ellipsoid_payload_does_not_load_point_geometry():
    from s3e_pipeline.distributed import Worker
    ellipsoids,_,_=scene()
    def forbidden(_):raise AssertionError('Point evidence was loaded')
    worker=object.__new__(Worker)
    worker.seen={0}
    worker.store=SimpleNamespace(row=lambda key:{'robot_id':'A','keyframe_id':key},
        ellipsoids=lambda key:ellipsoids,payload=forbidden)
    worker.backend_config=dict(registration=dict(method='ellipsoid',ellipsoid_max_count=200))
    worker.descriptor=lambda key:{}
    result=worker.payload(0)
    assert result['ellipsoids'].shape==(200,15) and result['cloud'].shape==(0,3)
