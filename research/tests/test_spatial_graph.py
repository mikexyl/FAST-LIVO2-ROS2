import copy
import os
from pathlib import Path
import sys
import time

import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from scipy.linalg import expm, logm

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from s3e_pipeline.artifacts import file_hash, read_json, write_jsonl
from s3e_pipeline.cbs_bridge import local_graph, ros_graph
from s3e_pipeline.geometry import inv, transform, adjoint
from s3e_pipeline.spatial_graph import SpatialGraph, SpatialGraphConfig, shared_count, path_covariance, active_component
from test_circle_cbs import spatialize


def fixture(tmp_path, count=8, line=False):
    rng = np.random.default_rng(18)
    world = rng.uniform(-8, 8, (500, 3))
    if line:
        world[:, 1:] = 0
    rows = []
    for k in range(count):
        T = np.eye(4)
        T[:3, :3] = Rotation.from_euler('xyz', [.01*k, -.03*k, .1*k]).as_matrix()
        T[:3, 3] = [k, 2*np.sin(k), 0]
        row = dict(robot_id='A', keyframe_id=k, stamp_ns=10**9*k,
                   T_world_body=T.tolist(), cloud_frame='A/imu', body_frame='A/imu')
        path = tmp_path / f'{k:06d}.npz'
        np.savez_compressed(path, cloud=transform(inv(T), world).astype(np.float32))
        spatialize(row, path)
        rows.append(row)
    pgo = dict(odometry_rotation_sigma_deg=1., odometry_translation_sigma_m=.15,
               spatial_neighbors=dict(enabled=True))
    return rows, pgo


def test_prefix_causality_immutable_edges_and_restart(tmp_path, monkeypatch):
    rows, pgo = fixture(tmp_path)
    graph = SpatialGraph('A', tmp_path, pgo)
    old = []
    for n in range(1, len(rows)+1):
        edges = graph.update(rows[:n])
        assert edges[:len(old)] == old
        assert all(e['j'][1] < n and e['j'][1] > e['i'][1] for e in edges)
        assert all(not e['diagnostics']['degree_budget_exception'] or e['diagnostics']['joins_components'] for e in edges)
        assert all(sum(e['j'][1] == j for e in edges) <= 2 for j in range(n))
        old = copy.deepcopy(edges)
    assert len(edges) > 0
    replay = SpatialGraph('A', tmp_path, pgo)
    assert replay.update(rows) == edges
    disabled = dict(pgo, spatial_neighbors=dict(enabled=False))
    _, chain = local_graph(rows, 'A', disabled)
    legacy = copy.deepcopy(pgo)
    legacy['spatial_neighbors']['topology'] = 'creation_backbone'
    _, augmented = local_graph(rows, 'A', legacy, store=tmp_path)
    assert augmented[:len(chain)] == chain
    assert all(e['j'][1]-e['i'][1] > 1 for e in augmented[len(chain):])
    reads = graph.payload_reads
    monkeypatch.setattr(np, 'load', lambda *a, **k: pytest.fail('Unchanged prefix reloaded geometry'))
    assert graph.update(rows) == edges and graph.payload_reads == reads
    _, all_edges = local_graph(rows, 'A', pgo, spatial_graph=graph)
    assert all_edges == edges
    assert all(e['kind'] == 'map_consistency' for e in all_edges)



@pytest.mark.parametrize('fault', ['row', 'hash', 'frame', 'robot', 'coordinates', 'prefix'])
def test_invalid_or_changed_input_fails_closed(tmp_path, fault):
    rows, pgo = fixture(tmp_path)
    graph = SpatialGraph('A', tmp_path, pgo)
    graph.update(rows[:2])
    if fault == 'row':
        rows[0]['footprint_center_xy'] = [1., 0.]
    elif fault == 'hash':
        rows[2]['sha256'] = '0'*64
    elif fault == 'frame':
        rows[2]['T_horizontal_map'][0][3] = 1.
    elif fault == 'robot':
        rows[2]['robot_id'] = 'B'
    elif fault == 'prefix':
        rows = rows[:1]
    else:
        path = tmp_path/'000002.npz'
        with np.load(path) as data:
            cloud = data['cloud'].copy(); ids = data['point_ids'].copy()
        cloud[0, 0] += .1
        np.savez_compressed(path, cloud=cloud, point_ids=ids)
        rows[2]['sha256'] = file_hash(path)
    with pytest.raises(ValueError):
        graph.update(rows)


def test_weak_or_unshared_overlap_is_not_an_edge(tmp_path):
    rows, pgo = fixture(tmp_path, line=True)
    graph = SpatialGraph('A', tmp_path, pgo)
    assert graph.update(rows) == []
    assert any(x['reason'] == 'weak_shared_geometry' for x in graph.decisions)
    # Coincident circle footprints alone cannot create a constraint.
    for k, row in enumerate(rows):
        row['geometry_id_ranges'] = [[k*1000, k*1000+500]]
    graph = SpatialGraph('A', tmp_path, pgo)
    assert graph.update(rows) == [] and graph.payload_reads == 0


@pytest.mark.parametrize('topology',['spatial_only','creation_backbone'])
def test_composed_covariance_and_information_budget(tmp_path, topology):
    rows, pgo = fixture(tmp_path)
    pgo['spatial_neighbors']['topology'] = topology
    graph = SpatialGraph('A', tmp_path, pgo)
    for e in graph.update(rows):
        i, j = e['i'][1], e['j'][1]
        if topology == 'creation_backbone':
            cov = path_covariance(rows, i, j, graph.edge_covariance)
        else:
            C = np.eye(4); C[:3,3] = e['diagnostics']['shared_centroid_in_j']
            sigmas = [np.deg2rad(3.)]*3+[.5]*3
            cov = adjoint(C)@np.diag(np.square(sigmas))@adjoint(C).T
        np.testing.assert_allclose(np.asarray(e['information']) @ cov, np.eye(6)*.25/3, atol=1e-12)
        np.testing.assert_allclose(transform(np.asarray(e['T_i_j']), [1., 2., 3.]),
            transform(inv(np.asarray(rows[i]['T_world_body'])) @ np.asarray(rows[j]['T_world_body']), [1.,2.,3.]))
    # A common world transform leaves factors (and their tangent covariances) unchanged.
    old = graph.edges
    T = np.eye(4); T[:3,:3] = Rotation.from_euler('xyz',[.3,-.2,.5]).as_matrix(); T[:3,3] = [12.,-9.,2.]
    moved = copy.deepcopy(rows)
    for row in moved:
        row['T_world_body'] = (T@np.asarray(row['T_world_body'])).tolist()
        row['T_world_imu'] = np.asarray(row['T_world_body']).ravel().tolist()
        row['T_horizontal_map'] = inv(T).tolist()
        row['gravity_world_m_s2'] = (T[:3,:3]@[0,0,-9.81]).tolist()
    new = SpatialGraph('A',tmp_path,pgo).update(moved)
    assert [(e['i'],e['j']) for e in old] == [(e['i'],e['j']) for e in new]
    for a,b in zip(old,new):
        np.testing.assert_allclose(a['T_i_j'],b['T_i_j'],atol=1e-12)
        np.testing.assert_allclose(a['information'],b['information'],atol=1e-10)


def test_ranges_and_config():
    assert shared_count([[1,5],[9,20]], [[3,8],[10,15],[19,30]]) == 8
    assert shared_count([], [[1,5]]) == 0
    for config in (dict(max_degree=0), dict(max_new_edges=4), dict(enabled='true'),
                   dict(min_shared_fraction=2), dict(information_fraction=float('nan'))):
        with pytest.raises(ValueError): SpatialGraphConfig(**config)


def test_path_covariance_against_numerical_right_perturbations(tmp_path):
    from s3e_pipeline.geometry import skew
    rows, pgo = fixture(tmp_path, count=5)
    transforms = [inv(np.asarray(a['T_world_body']))@np.asarray(b['T_world_body'])
                  for a,b in zip(rows,rows[1:])]
    def product(items):
        value = np.eye(4)
        for item in items: value = value@item
        return value
    truth = product(transforms); numerical = np.zeros((6,6)); eps = 1e-6
    covariance = SpatialGraph('A',tmp_path,pgo).edge_covariance
    for k in range(len(transforms)):
        jacobian = np.empty((6,6))
        for axis in range(6):
            v = np.eye(6)[axis]*eps; hat = np.zeros((4,4))
            hat[:3,:3] = skew(v[:3]); hat[:3,3] = v[3:]
            changed = list(transforms); changed[k] = changed[k]@expm(hat)
            tangent = np.real(logm(inv(truth)@product(changed)))
            jacobian[:,axis] = np.r_[tangent[2,1],tangent[0,2],tangent[1,0],tangent[:3,3]]/eps
        numerical += jacobian@covariance@jacobian.T
    np.testing.assert_allclose(path_covariance(rows,0,4,covariance),numerical,atol=1e-9)


def test_ros_distinct_edge_type(tmp_path):
    pytest.importorskip('rclpy')
    rows, cfg = fixture(tmp_path)
    nodes, edges = local_graph(rows, 'A', cfg, store=tmp_path)
    message = ros_graph(nodes, edges, ['A'])
    assert all(e.type == 100 for e in message.edges)


@pytest.mark.skipif(os.environ.get('S3E_TEST_PERSISTENT_DDS')!='1', reason='Requires native spatial CBS overlay')
def test_native_growing_graph_with_gpu_and_spatial_cycles(tmp_path):
    from test_persistent_cbs import fixture as native_fixture
    from s3e_pipeline.persistent_cbs import PersistentSession, common_reference_ready
    from s3e_pipeline.spatial_cbs_contract import endpoint
    cfg, stores, rows, loops, truth = native_fixture(tmp_path, True)
    cfg['pgo']['spatial_neighbors'] = dict(enabled=True, min_second_axis_std_m=.1)
    cfg['dpgo']['registration_factors']['max_range_m'] = None
    cfg['dpgo']['ros_domain_id'] = 198
    for r, keys in rows.items():
        for row in keys:
            spatialize(row, stores[r]/f"{row['keyframe_id']:06d}.npz", .01)
        write_jsonl(stores[r]/'keyframes.jsonl', keys)
    for edge in loops:
        edge['diagnostics'] = dict(spatial_endpoints=[endpoint(rows[r][k]) for r,k in (edge['i'],edge['j'])])
    session = PersistentSession(cfg, Path(__file__).resolve().parents[3], tmp_path/'session', stores)
    try:
        deadline = time.monotonic()+30
        while not session.ready():
            assert time.monotonic() < deadline; time.sleep(.1)
        pids = [p.pid for _,p in session.processes]
        previous = []
        for count, selected, final in [(2,loops[:1],False),(6,loops,False),(6,loops,True)]:
            session.submit({r:v[:count] for r,v in rows.items()}, selected, final)
            snap = None; start = time.monotonic()
            while snap is None or (final and not snap['summary']['finished']) or time.monotonic()-start<2:
                snap = session.snapshot(); time.sleep(.1); assert time.monotonic()-start<65
            current = snap['local_constraints']
            assert all(e in current for e in previous); previous = current
            assert [p.pid for _,p in session.processes] == pids
        assert common_reference_ready(snap, cfg['robots'])
        assert snap['summary']['loops'] == len(loops)
        assert snap['summary']['map_consistency_factors'] > 0
        for row in snap['poses']:
            np.testing.assert_allclose(row['T_world_body'], inv(truth['Alpha',0])@truth[row['robot_id'],row['keyframe_id']],atol=.01)
        for r in rows:
            status = snap['summary']['robots'][r]
            assert status['registration']['success']
            assert all(p['i'][0] != p['j'][0] for p in status['registration']['pairs'])
            assert not read_json(session.output/r/'registration/3/transport.json')['requested']
    finally:
        session.close()


def disconnect_then_bridge(rows, store, bridge_key=2):
    """0 and 1 share no identities; 2 genuinely contains both observation sets."""
    from s3e_pipeline.circle_submaps import id_ranges
    for k in range(1,bridge_key+1):
        path = store/f'{k:06d}.npz'
        with np.load(path) as data:
            xyz, ids = data['cloud'].copy(), data['point_ids'].copy()
        if k < bridge_key: ids += 10000000
        else:
            xyz = np.concatenate([xyz,xyz]); ids = np.r_[ids,ids+10000000]
        np.savez_compressed(path, cloud=xyz, point_ids=ids)
        rows[k].update(sha256=file_hash(path), geometry_count=len(ids),
                      geometry_id_ranges=id_ranges(ids))


def test_no_chain_disconnected_component_and_later_spatial_bridge(tmp_path):
    rows, pgo = fixture(tmp_path,count=3)
    disconnect_then_bridge(rows,tmp_path)
    graph = SpatialGraph('A',tmp_path,pgo)
    nodes, edges = local_graph(rows[:2],'A',pgo,spatial_graph=graph)
    active, admitted, pending = active_component(nodes,edges,pgo)
    assert not edges and [r['keyframe_id'] for r in active] == [0]
    assert [r['keyframe_id'] for r in pending] == [1]
    nodes, edges = local_graph(rows,'A',pgo,spatial_graph=graph)
    assert {(e['i'][1],e['j'][1]) for e in edges} == {(0,2),(1,2)}
    assert active_component(nodes,edges,pgo) == (nodes,edges,[])
    assert graph.diagnostics()['component_count'] == 1


def test_pending_submaps_cannot_be_exported_as_a_joint_map():
    from s3e_pipeline.circle_central_evaluation import correction_anchors, corrected_union
    pending = [dict(keyframe_id=1, optimized=False)]
    with pytest.raises(ValueError, match='unconnected'):
        correction_anchors([], pending)
    with pytest.raises(ValueError, match='Unconnected'):
        corrected_union(np.empty((0,3)), [], pending)


@pytest.mark.skipif(os.environ.get('S3E_TEST_PERSISTENT_DDS')!='1', reason='Requires native spatial CBS overlay')
def test_native_pending_component_joins_through_new_overlap(tmp_path):
    from test_persistent_cbs import fixture as native_fixture
    from s3e_pipeline.persistent_cbs import PersistentSession
    from s3e_pipeline.geometry import constraint
    cfg, stores, rows, _, truth = native_fixture(tmp_path, True, robots=['Alpha','Bob'],pairs=[('Alpha','Bob')])
    cfg['pgo']['spatial_neighbors'] = dict(enabled=True,min_second_axis_std_m=.1)
    cfg['dpgo']['registration_factors']['max_range_m'] = None
    cfg['dpgo']['ros_domain_id'] = 198
    for r in rows:
        for row in rows[r]: spatialize(row,stores[r]/f"{row['keyframe_id']:06d}.npz",.01)
        disconnect_then_bridge(rows[r],stores[r],bridge_key=3)
        write_jsonl(stores[r]/'keyframes.jsonl',rows[r])
    loops = [constraint(['Alpha',k],['Bob',k],inv(truth['Alpha',k])@truth['Bob',k],np.eye(6)*100,kind='loop',accepted=True) for k in (0,1)]
    from s3e_pipeline.spatial_cbs_contract import endpoint
    for e in loops: e['diagnostics'] = dict(spatial_endpoints=[endpoint(rows[r][k]) for r,k in (e['i'],e['j'])])
    session = PersistentSession(cfg,Path(__file__).resolve().parents[3],tmp_path/'session',stores)
    try:
        deadline = time.monotonic()+30
        while not session.ready():
            assert time.monotonic()<deadline; time.sleep(.1)
        pids=[p.pid for _,p in session.processes]
        for count,final in [(3,False),(4,True)]:
            session.submit({r:v[:count] for r,v in rows.items()},loops,final)
            snap=None; start=time.monotonic()
            while snap is None or time.monotonic()-start<2 or (final and not snap['summary']['finished']):
                snap=session.snapshot(); time.sleep(.1); assert time.monotonic()-start<65
            assert [p.pid for _,p in session.processes]==pids
            assert snap['summary']['alignment_complete'] == final
            if not final:
                assert snap['summary']['map_consistency_factors'] == 0
                assert snap['summary']['pending_map_consistency_factors'] == 2
            assert snap['summary']['loops'] == (2 if final else 1)
            for row in snap['poses']:
                assert row['optimized'] == (final or row['keyframe_id']==0)
                if row['optimized']:
                    np.testing.assert_allclose(row['T_world_body'], inv(truth['Alpha',0])@truth[row['robot_id'],row['keyframe_id']],atol=.01)
                else:
                    assert not row['reference_available'] and '/unconnected/' in row['component']
    finally: session.close()


def test_spatial_only_cannot_silently_use_sealed_chain_pcm(tmp_path):
    from s3e_pipeline.dpgo import run
    cfg=dict(dpgo=dict(execution='sealed'),pgo=dict(spatial_neighbors=dict(enabled=True)))
    with pytest.raises(ValueError,match='persistent execution'):
        run(cfg,{},tmp_path,tmp_path/'out')
    assert not (tmp_path/'out').exists()
