"""Acceptance tests for the standalone footprint planner, without a ROS runtime."""
import itertools
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np
import pytest
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from s3e_pipeline import circle_cover as cc


def points(xy):
    xy = np.asarray(xy).reshape(-1, 2)
    return np.c_[xy, np.zeros(len(xy))]


def grid(x0, x1, y0, y1):
    return points(np.array(list(itertools.product(np.arange(x0, x1)+.2, np.arange(y0, y1)+.2))))


@pytest.mark.parametrize('shape', ['disk', 'rectangle', 'corridor', 'branches', 'islands', 'sparse'])
def test_incremental_shapes(shape):
    cfg = cc.CircleCoverConfig(radius_m=8, solver_time_limit_s=1)
    base = grid(-24, 25, -24, 25)
    if shape == 'disk':
        steps = [base[np.linalg.norm(base[:, :2], axis=1) < r] for r in (3, 9, 15, 23)]
    elif shape == 'rectangle':
        steps = [grid(0, x, -8, 8) for x in (6, 16, 28, 44)]
    elif shape == 'corridor':
        steps = [grid(0, x, 0, 2) for x in (5, 20, 40, 65)]
    elif shape == 'branches':
        steps = [grid(0, 30, 0, 2), grid(14, 16, -22, 24), grid(14, 42, 22, 24)]
    elif shape == 'islands':
        steps = [grid(x, x+5, 0, 5) for x in (0, 50, -60)]
    else:
        steps = [points([[0, 0], [50, 12], [-70, 0]]), points([[200, 200]])]
    planner = cc.CircleCoverPlanner(cfg)
    for k, observation in enumerate(steps):
        old = planner.circles
        result = planner.update(observation, k)
        assert result.diagnostics['coverage'] >= .95
        assert planner.circles[:len(old)] == old
        assert [c.id for c in planner.circles] == list(range(len(planner.circles)))
        assert all(c.created_update == k for c in result.new_circles)
        if result.new_circles:
            assert result.diagnostics['solver'] is not None
    old = planner.state()
    result = planner.update(np.vstack(steps)[::-1].repeat(2, axis=0), len(steps))
    assert result.new_circles == () and result.diagnostics['added_cells'] == 0
    assert planner.state()['occupied_cells'] == old['occupied_cells']


def test_order_density_height_and_restart(tmp_path):
    batches = [grid(0, x, -3, 4) for x in (8, 18, 32)]
    cfg = cc.CircleCoverConfig(radius_m=8)
    normal, permuted = cc.CircleCoverPlanner(cfg), cc.CircleCoverPlanner(cfg)
    for k, batch in enumerate(batches):
        changed = np.repeat(batch[::-1], 3, axis=0)
        changed[:, 2] = np.arange(len(changed))*2
        normal.update(batch, k); permuted.update(changed, k)
        assert normal.state() == permuted.state()
        saved = tmp_path/'state.json'; normal.save(saved)
        restored = cc.CircleCoverPlanner.load(saved)
        assert restored.state() == normal.state()
        if k+1 < len(batches):
            restored.update(batches[k+1], k+1)
            reference = cc.CircleCoverPlanner.load(saved)
            reference.update(batches[k+1], k+1)
            assert restored.state() == reference.state()
    envelope = json.loads(saved.read_text()); envelope['state']['config']['radius_m'] += 1
    saved.write_text(json.dumps(envelope))
    with pytest.raises(ValueError):
        cc.CircleCoverPlanner.load(saved)


def test_restart_matches_uninterrupted_continuation(tmp_path):
    cfg = cc.CircleCoverConfig(radius_m=8)
    live = cc.CircleCoverPlanner(cfg)
    resumed = cc.CircleCoverPlanner(cfg)
    for k in range(4):
        batch = grid(k*13, k*13+15, -3, 4)
        live.update(batch, k); resumed.update(batch, k)
        assert live.state() == resumed.state()
        resumed.save(tmp_path/'restart.json')
        resumed = cc.CircleCoverPlanner.load(tmp_path/'restart.json')


def test_cell_density_invariance():
    batch = grid(-10, 20, -4, 5)
    a, b = cc.CircleCoverPlanner(cc.CircleCoverConfig(radius_m=8)), cc.CircleCoverPlanner(cc.CircleCoverConfig(radius_m=8))
    changed = np.repeat(batch, 4, axis=0); changed[:, :2] += .5
    a.update(batch, 0); b.update(changed, 0)
    assert a.state() == b.state()


def test_gravity_frame_height_and_origin():
    R = Rotation.from_euler('xyz', [.3, -.2, .1]).as_matrix()
    T = np.eye(4); T[:3, :3] = R
    cfg = cc.CircleCoverConfig(radius_m=8, grid_origin_xy=(.3, -.4))
    xy = grid(-12, 13, -6, 7)
    first, second = cc.CircleCoverPlanner(cfg, T), cc.CircleCoverPlanner(cfg, T)
    elevated = xy.copy(); elevated[:, 2] = 40
    first.update(xy@R, 0); second.update(elevated@R, 0)
    assert first.state() == second.state()
    frame_copy = first.T_horizontal_map; frame_copy[0, 3] += 1
    with pytest.raises(ValueError, match='cannot change'):
        first.update(points([]), 1, T_horizontal_map=frame_copy)
    assert np.array_equal(first.T_horizontal_map, T)


def test_empty_invalid_inputs_and_atomicity():
    planner = cc.CircleCoverPlanner()
    assert planner.update(points([]), 0).new_circles == ()
    assert planner.update(points([[1, 1]]), 2).diagnostics['coverage'] == 1
    state = planner.state()
    for update_id in (2, 1, -1, True, 2.5):
        with pytest.raises(ValueError):
            planner.update(points([]), update_id)
    for batch in (np.array([[0, np.nan, 0]]), [[np.inf, 0, 0]], [[1, 2]], [[1e100, 0, 0]]):
        with pytest.raises(ValueError):
            planner.update(batch, 3)
        assert planner.state() == state
    for T in (np.zeros((4, 4)), np.diag([-1, 1, 1, 1])):
        with pytest.raises(ValueError):
            cc.CircleCoverPlanner(T_horizontal_map=T)


def test_analytic_overlap():
    assert cc.disk_overlap(0, 80) == 1
    assert cc.disk_overlap(160, 80) == 0
    assert cc.disk_overlap(300, 80) == 0
    assert cc.disk_overlap(80, 80) == pytest.approx(2/3-np.sqrt(3)/(2*np.pi))
    assert cc.disk_overlap(93.622, 80) == pytest.approx(.3, abs=2e-6)


def test_candidate_net_and_exhaustive_optimum():
    rng = np.random.default_rng(7)
    for trial in range(8):
        xy = np.unique(rng.integers(0, 7, (9, 2)), axis=0).astype(float)+.5
        candidates = cc.candidate_centers(xy, .5)
        assert np.array_equal(candidates, cc.candidate_centers(xy[::-1].repeat(2, axis=0), .5))
        assert cKDTree(candidates).query(xy)[0].max() <= .5+1e-12
        table = cc.coverage_table(xy, candidates, 2.)
        needed = int(np.ceil(.8*len(xy)))
        best = next(k for k in range(1, len(candidates)+1)
                    if any(np.count_nonzero(np.asarray(table[:, ids].sum(axis=1)).ravel()) >= needed
                           for ids in itertools.combinations(range(len(candidates)), k)))
        planner = cc.CircleCoverPlanner(cc.CircleCoverConfig(radius_m=2, coverage_target=.8))
        greedy = planner._greedy(table, needed, candidates, np.empty((0, 2)))
        selected, diagnostics = planner._solve(table, needed, greedy)
        assert diagnostics['optimal_candidate_count'], diagnostics
        assert not diagnostics['fallback'] and len(selected) == best


@pytest.mark.parametrize('feasible_incumbent', [False, True])
def test_solver_timeout_fallback(monkeypatch, feasible_incumbent):
    def timeout(c, **kwargs):
        m = int(np.count_nonzero(kwargs['integrality']))
        x = np.r_[np.ones(m), np.ones(len(c)-m)] if feasible_incumbent else None
        return SimpleNamespace(x=x, status=1, message='Time limit', fun=None, mip_gap=.7)
    monkeypatch.setattr(cc, 'milp', timeout)
    planner = cc.CircleCoverPlanner(cc.CircleCoverConfig(radius_m=8))
    result = planner.update(grid(0, 30, 0, 4), 0)
    assert result.diagnostics['coverage'] >= .95
    assert result.diagnostics['solver']['status'] == 1
    assert result.diagnostics['solver']['mip_gap'] == .7
    # All-one incumbent exceeds the greedy upper bound, so it must be rejected.
    assert result.diagnostics['solver']['fallback']


def test_feasible_timeout_incumbent_is_used(monkeypatch):
    def timeout(c, **kwargs):
        return SimpleNamespace(x=np.ones(len(c)), status=1, message='Time limit', fun=1, mip_gap=.2)
    monkeypatch.setattr(cc, 'milp', timeout)
    result = cc.CircleCoverPlanner().update(points([[0, 0]]), 0)
    assert result.diagnostics['coverage'] == 1
    assert not result.diagnostics['solver']['fallback']
    assert not result.diagnostics['solver']['optimal_candidate_count']


def test_refinement_failure_and_candidate_loss(monkeypatch):
    def fail(*args, **kwargs):
        raise RuntimeError('Injected refinement failure')
    monkeypatch.setattr(cc, 'minimize', fail)
    planner = cc.CircleCoverPlanner(cc.CircleCoverConfig(radius_m=8))
    first = planner.update(grid(0, 10, 0, 4), 0)
    old = planner.circles
    result = planner.update(grid(10, 23, 0, 4), 1)
    assert result.diagnostics['coverage'] >= .95
    assert planner.circles[:len(old)] == old
    assert not result.diagnostics['refinement']['success']
    state = planner.state()
    monkeypatch.setattr(planner, '_refine', lambda new, *args: (new+1e6, {}))
    with pytest.raises(RuntimeError, match='state not committed'):
        planner.update(grid(30, 50, 0, 4), 2)
    assert planner.state() == state


@pytest.mark.parametrize('resolution', [.5, 1., 2.])
def test_grid_resolution_and_actual_point_coverage(resolution):
    planner = cc.CircleCoverPlanner(cc.CircleCoverConfig(radius_m=8, cell_size_m=resolution))
    observation = grid(-12, 13, -8, 9)
    result = planner.update(observation, 0)
    assert result.diagnostics['coverage'] >= .95
    direct = planner.point_coverage(observation)
    assert direct['points'] == len(observation) and 0 <= direct['coverage'] <= 1
    assert np.array_equal(planner.cell_centers, (planner.cells+.5)*resolution)


def test_99_percent_incremental_coverage():
    planner = cc.CircleCoverPlanner(cc.CircleCoverConfig(radius_m=8, coverage_target=.99))
    for k in range(4):
        old = planner.circles
        result = planner.update(grid(k*10, k*10+16, -7, 8), k)
        assert result.diagnostics['coverage'] >= .99
        assert planner.circles[:len(old)] == old
