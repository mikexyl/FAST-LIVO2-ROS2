#!/usr/bin/env python3
"""Verify every eligible frozen joint BEV hypothesis using existing height/GICP.

Run in the captured experiment environment. This stage never reads ground truth,
never changes retrieval/registration thresholds, and does not run PCM or CBS.
"""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import shutil
import time
import zlib
from collections import Counter

import numpy as np
import yaml
import small_gicp
from scipy.spatial import cKDTree
from s3e_pipeline.backends import unpack_array
from s3e_pipeline.geometry import transform
from s3e_pipeline.registration import bounded_cloud, refine, FeatureCache
from s3e_pipeline.vertical_initialization import initialize_vertical
import s3e_pipeline.registration as registration_module
import s3e_pipeline.vertical_initialization as height_module
import s3e_pipeline.geometry as geometry_module

ROOT = Path(__file__).resolve().parents[2]
MODULE = ROOT/'research/s3e_pipeline/joint_bev_verification.py'
spec = importlib.util.spec_from_file_location('s3e_pipeline.joint_bev_verification', MODULE)
conversion = importlib.util.module_from_spec(spec); spec.loader.exec_module(conversion)


def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        while part := f.read(2**20):
            h.update(part)
    return h.hexdigest()


def save(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False)+'\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--work', type=Path, required=True)
    parser.add_argument('--preview', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(); work, preview, out = args.work.resolve(), args.preview.resolve(), args.output.resolve()
    out.mkdir(exist_ok=False)
    for folder in ['pairs', 'geometry', 'source']:
        (out/folder).mkdir()
    started = time.monotonic(); config_path = work/'config.yaml'; match_path = preview/'matching.json'
    cfg = yaml.safe_load(config_path.read_text())['backend']
    registration, vertical, evidence = cfg['registration'], cfg['mapclosures']['vertical_initialization'], cfg['evidence']
    assert registration['sampling'] == evidence['sampling'] == 'fixed'
    assert registration['voxel_m'] == evidence['voxel_m'] == .4
    assert registration['max_range_m'] is None and evidence['max_range_m'] is None
    assert vertical['enabled'] and cfg['mapclosures']['inliers_threshold'] == 5
    matching = json.loads(match_path.read_text()); assert matching['complete'] and not matching['ground_truth_used']
    verification = json.loads((preview/'verification.json').read_text())
    assert verification['matching_sha256'] == sha(match_path)
    pairs = [p for p in matching['pairs'] if p['methods']['pooled']['passes_2d_gate']]
    assert all(p['methods']['pooled']['valid_pose'] and p['methods']['pooled']['inliers'] > 5 for p in pairs)
    inputs = {str(p): sha(p) for p in [config_path, match_path, preview/'verification.json']}
    design = dict(candidate_ids=[p['id'] for p in pairs], selection='every passing pooled hypothesis; no GT filtering',
        matching_sha256=sha(match_path), registration=registration, vertical_initialization=vertical,
        evidence=evidence, initial_level_vertical_translation_m=0.,
        frame_contract='T_query_IMU_candidate_IMU = inverse(G_query) @ T_query_level_candidate_level @ G_candidate',
        no_height_layer_filtering_of_registration_geometry=True,
        geometry='native processed points from the same accumulated-area snapshots; not raw full-resolution scans',
        ground_truth_used=False, run_pcm=False, run_cbs=False, loops_added_to_graph=0)
    save(out/'design.json', design)
    sources = [Path(__file__), MODULE, Path(registration_module.__file__), Path(height_module.__file__),
               Path(geometry_module.__file__), Path(small_gicp.__file__)]
    source_hashes = {str(p): sha(p) for p in sources}
    for p in sources:
        if p.suffix == '.py':
            shutil.copy2(p, out/'source'/p.name)
    maps = {(m['robot'], m['key']): m for m in matching['maps']}
    endpoints = sorted({tuple(p[n]) for p in pairs for n in ['query', 'candidate']})
    clouds, grounds, metadata = {}, {}, []
    for r, k in endpoints:
        geometry = work/f'prepared-{r}/store/{k:06d}.npz'
        descriptor = work/f'prepared-{r}/ellipsoid/{k:06d}.json.zlib'
        feature_path = preview/'features'/f'{r}-{k:06d}-full.npz'
        for p in [geometry, descriptor, feature_path]:
            inputs[str(p)] = sha(p)
        with np.load(geometry) as data:
            original = data['cloud'].copy()
        packet = json.loads(zlib.decompress(descriptor.read_bytes()))
        assert packet['projection_alignment']['projection'] == 'orthographic gravity-horizontal'
        G = unpack_array(packet['mapclosures']['ground'])
        with np.load(feature_path) as data:
            assert np.array_equal(G, data['ground'])
        cloud, resolution = bounded_cloud(original, evidence['voxel_m'], None, None)
        assert resolution == evidence['voxel_m'] and np.isfinite(cloud).all()
        clouds[r, k], grounds[r, k] = cloud, G
        name = f'geometry/{r}-{k:06d}.npz'
        np.savez_compressed(out/name, cloud=cloud, ground=G)
        metadata.append(dict(robot=r, key=k, stamp_ns=maps[r, k]['stamp_ns'],
            source_points=len(original), evidence_points=len(cloud), evidence_voxel_m=resolution,
            geometry=name, sha256=sha(out/name)))
        print('evidence', r, k, len(original), '->', len(cloud), flush=True)
    cache = FeatureCache(registration.get('feature_cache_mib', 128))
    results = []
    for pair in pairs:
        start = time.monotonic(); q, c = tuple(pair['query']), tuple(pair['candidate'])
        fit = pair['methods']['pooled']; target, source = clouds[q], clouds[c]; Gq, Gc = grounds[q], grounds[c]
        initial = conversion.lift_planar_pose(fit['T_query_candidate_level_2d'], Gq, Gc)
        height_seed, diagnostic = initialize_vertical(target, source, initial, Gq, Gc, vertical)
        zero_level = Gq@initial@np.linalg.inv(Gc); height_level = Gq@height_seed@np.linalg.inv(Gc)
        assert np.allclose(zero_level[:3, :3], height_level[:3, :3], atol=1e-10)
        assert np.allclose(zero_level[:2, 3], height_level[:2, 3], atol=1e-10)
        refine_start = time.monotonic()
        result = refine(target, source, height_seed, registration, cache)
        refine_wall = time.monotonic()-refine_start
        row = dict(id=pair['id'], query=q, candidate=c, pooled_inliers=fit['inliers'],
            bev_initial_T_i_j=initial.tolist(), height_seed_T_i_j=height_seed.tolist(),
            vertical_initialization=diagnostic, verification=result, refinement_wall_s=refine_wall,
            wall_s=time.monotonic()-start, ground_truth_used=False)
        save(out/'pairs'/f'{pair["id"]}.json', row); results.append(row)
        save(out/'progress.json', dict(completed=len(results), total=len(pairs),
            accepted=sum(r['verification']['accepted'] for r in results), last=pair['id']))
        print(pair['id'], diagnostic['correction_m'], result['reason'],
              {k: result.get(k) for k in ['initial_overlap', 'overlap', 'rmse_m', 'condition']}, flush=True)
    assert all(sha(Path(p)) == h for p, h in inputs.items())
    assert all(sha(Path(p)) == h for p, h in source_hashes.items())
    summary = dict(complete=True, design=design, maps=metadata, results=results,
        accepted=sum(r['verification']['accepted'] for r in results),
        reasons=dict(Counter(r['verification']['reason'] for r in results)),
        input_sha256=inputs, source_sha256=source_hashes, source_artifacts_unchanged=True,
        ground_truth_used=False, thresholds_unchanged=True, pcm_run=False, cbs_run=False,
        loops_added_to_graph=0, wall_s=time.monotonic()-started, native_geometry_cache_hits=cache.hits)
    save(out/'summary.json', summary)
    print('verification complete', summary['accepted'], '/', len(results), summary['reasons'], summary['wall_s'], flush=True)


if __name__ == '__main__':
    main()
