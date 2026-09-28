"""Native point preparation matches the frozen point-BEV ablation contract."""
import io
import json
import zlib

import numpy as np
import pytest
import yaml

from s3e_pipeline.backends import pack_array
from s3e_pipeline.recent_submaps import DEFAULT_CONFIG, bev_representation, prepare
from s3e_pipeline.submap_writer import consume


def test_native_point_preparation_matches_ablation_without_surface_sampling(tmp_path, monkeypatch):
    import s3e_mapclosures_native as native
    from s3e_pipeline import area_maps, ellipsoid_cuda
    from s3e_pipeline.multilayer_mapclosures import describe_layers, decode_layers
    from s3e_pipeline.gravity_bev import submap_gravity
    from test_area_maps import fixture, packet

    def forbidden(*args, **kwargs):
        raise AssertionError('Point BEVs must not sample ellipsoids')
    monkeypatch.setattr(area_maps, 'render_area_surface', forbidden)
    monkeypatch.setattr(ellipsoid_cuda, 'SurfaceSampler', forbidden)
    row, data = fixture()
    rng = np.random.default_rng(37)
    points = rng.uniform(-50, 50, (30000, 3)).astype('<f4')
    points[:, 2] = rng.choice([0., 3., 7., 15., 25.], len(points), p=[.6, .1, .1, .1, .1])
    ids = np.arange(len(points), dtype='<i4')
    data.update(points=points, point_ids=ids, point_scan_ids=np.zeros_like(ids),
                ellipsoid_point_ids=ids[:3].copy())
    data['ellipsoids'][:, :3] = points[:3]
    row.update(geometry_count=len(points), archive_point_count=len(points),
               geometry_id_ranges=[[0, len(points)]], member_scan_ids=[0], area_radius_m=80.)
    source = tmp_path/'source'
    consume(io.BytesIO(packet(row, data) + b'\0'*4), source)
    before = {p.name: p.read_bytes() for p in source.iterdir()}
    cfg = yaml.safe_load(DEFAULT_CONFIG.read_text())
    out = tmp_path/'prepared'
    rows = prepare(source, out, cfg)

    # Same computation as the frozen point_bev_ablation.make_descriptor path.
    mc = cfg['backend']['mapclosures']
    engine = native.MapClosures(mc['density_map_resolution'], mc['density_threshold'], mc['hamming_distance_threshold'])
    ground, _ = submap_gravity(row)
    features = engine.describe(points, ground=ground)
    layers = describe_layers(engine, points, points, ground)
    result = json.loads(zlib.decompress((out/'ellipsoid/000000.json.zlib').read_bytes()))
    assert len(features['xy']) > 0
    assert result['representation'] == 'point_cloud'
    assert result['mapclosures'] == {k: pack_array(features[k]) for k in ('ground', 'xy', 'bits')}
    assert result['mapclosures_multilayer'] == layers
    assert layers['terrain']['available']
    assert all(n > 0 for n in layers['surface_points'].values())
    decode_layers(result)
    with np.load(out/'store/000000.npz') as evidence:
        assert set(evidence.files) == {'cloud', 'scan'}
        np.testing.assert_array_equal(evidence['cloud'], points)
        np.testing.assert_array_equal(evidence['scan'], points)
    assert rows[0]['stamp_ns'] == row['stamp_ns']
    assert rows[0]['available_ns'] == row['available_ns']
    assert rows[0]['member_scan_ids'] == result['member_scan_ids'] == [0]
    assert json.loads((out/'summary.json').read_text())['representation'] == 'point_cloud'
    assert json.loads((out/'timings.jsonl').read_text())['sampling'] is None
    assert before == {p.name: p.read_bytes() for p in source.iterdir()}


def test_legacy_and_explicit_ellipsoid_profiles_remain_selectable():
    historical = yaml.safe_load((DEFAULT_CONFIG.parents[2]/'scripts/recent_submaps/area_rollout.yaml').read_text())
    assert bev_representation(historical) == 'ellipsoid'
    cfg = yaml.safe_load(DEFAULT_CONFIG.read_text())
    assert bev_representation(cfg) == 'point_cloud'
    full = yaml.safe_load((DEFAULT_CONFIG.parent/'full_ellipsoid_backend.yaml').read_text())
    def merge(target, values):
        for key, value in values.items():
            if isinstance(value, dict): merge(target.setdefault(key, {}), value)
            else: target[key] = value
    merge(cfg, full)
    assert bev_representation(cfg) == 'ellipsoid'
    assert cfg['dpgo']['registration_factors']['factor'] == 'ellipsoid'


@pytest.mark.parametrize('change', [dict(representation='invalid'), dict(renderer='geometric_coverage')])
def test_mixed_or_invalid_point_renderer_is_rejected(change):
    cfg = yaml.safe_load(DEFAULT_CONFIG.read_text())
    cfg['backend']['mapclosures'].update(change)
    with pytest.raises(ValueError): bev_representation(cfg)


def test_explicit_point_profile_rejects_old_ellipsoid_packets():
    from s3e_pipeline.mapclosures import MegaLocMapClosures
    backend = object.__new__(MegaLocMapClosures)
    backend.options = dict(representation='point_cloud')
    with pytest.raises(ValueError, match='representation'):
        backend.decode(dict(representation='ellipsoid'))
