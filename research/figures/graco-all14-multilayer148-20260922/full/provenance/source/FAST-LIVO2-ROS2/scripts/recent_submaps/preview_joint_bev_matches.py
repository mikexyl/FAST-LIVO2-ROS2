#!/usr/bin/env python3
"""Fixed offline joint-layer matching experiment. No ground-truth reads.

Use the frozen experiment environment, as for preview_multilayer_bevs.py.
Evaluate the immutable matching output separately with evaluate_joint_bev_matches.py.
"""
import argparse
import importlib.util
import json
from pathlib import Path
import shutil
import time
import zlib

import cv2
import numpy as np
import yaml
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import ConnectionPatch, Patch
from matplotlib.transforms import Affine2D


def load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    return module


HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
preview = load_module('layer_preview', HERE/'preview_multilayer_bevs.py')
joint = load_module('joint_consensus', ROOT/'research/s3e_pipeline/joint_bev_ransac.py')
sha = preview.sha
NAMES = ['full', *[b[0] for b in preview.layers.BANDS]]
COLORS = dict(near='#cf7cf6', low='#50e1dc', middle='#ffd066', high='#ee7788', upper='#819cff')


def cache_map(work, out, trials, key, mc, basis_tolerance):
    r, k = key['robot_id'], key['keyframe_id']
    descriptor = work/f'prepared-{r}/ellipsoid/{k:06d}.json.zlib'
    payload = Path(trials[r])/'frontend/area_maps'/key['payload']
    assert sha(payload) == key['sha256']
    packet = json.loads(zlib.decompress(descriptor.read_bytes()))
    frozen = {n: preview.unpack_array(v) for n, v in packet['mapclosures'].items()}
    G = frozen['ground']
    with np.load(payload) as data:
        points, ellipsoids = data['points'].copy(), data['ellipsoids'].copy()
    coefficients, terrain, _, _ = preview.layers.fit_terrain(points@G[:3, :3].T+G[:3, 3])
    basis, _ = preview.normalize_exported_basis(ellipsoids[:, 6:].reshape(-1, 3, 3), basis_tolerance)
    surface, sampling = preview.render_area_surface(ellipsoids[:, :3], ellipsoids[:, 3:6], basis, G, key['area_radius_m'])
    heights = preview.layers.relative_heights(surface@G[:3, :3].T+G[:3, 3], coefficients)
    masks = dict(full=np.ones(len(surface), dtype=bool), **preview.layers.layer_masks(heights))
    entry = dict(robot=r, key=k, terrain=terrain, sampling=sampling, layers=[])
    feature_packets = {}
    def engine(cls):
        return cls(mc['density_map_resolution'], mc['density_threshold'], mc['hamming_distance_threshold'])
    for name, mask in masks.items():
        sliced = surface[mask]
        if len(sliced) >= 30:
            feature = engine(preview.native.MapClosures).describe(sliced, ground=G)
            debug = engine(preview.inspection.Inspector).density(sliced, G)
            preview.check_features(debug, feature)
            image, lower = np.asarray(debug['image']), np.asarray(debug['lower_bound'])
        else:
            feature = dict(ground=G, xy=np.empty((0, 2)), bits=np.empty((0, 32), dtype=np.uint8))
            image, lower = np.zeros_like(full_image), full_lower
        if name == 'full':
            preview.check_features(debug, frozen)
            assert np.array_equal(feature['bits'], frozen['bits']) and np.array_equal(feature['xy'], frozen['xy'])
            full_image, full_lower = image, lower
        stem = f'{r}-{k:06d}-{name}'
        assert cv2.imwrite(str(out/'images'/f'{stem}.png'), image)
        np.savez_compressed(out/'features'/f'{stem}.npz', **{n: feature[n] for n in ('ground', 'xy', 'bits')},
                            lower_bound=lower, terrain_coefficients=coefficients)
        entry['layers'].append(dict(id=name, image=f'images/{stem}.png', lower_bound=lower.tolist(),
            shape=list(image.shape), features=len(feature['xy']), surface_points=len(sliced),
            png_sha256=sha(out/'images'/f'{stem}.png')))
        feature_packets[name] = feature
    return entry, feature_packets, {str(p): sha(p) for p in (descriptor, payload)}


def composite(out, entry):
    from matplotlib.colors import to_rgb
    full = entry['layers'][0]; lower = np.asarray(full['lower_bound'])
    rgb = np.zeros((*full['shape'], 3), dtype=float)
    for layer in entry['layers'][1:]:
        im = cv2.imread(str(out/layer['image']), cv2.IMREAD_GRAYSCALE).astype(float)/255
        x, y = (np.asarray(layer['lower_bound'])-lower).astype(int)
        h, w = im.shape
        rgb[x:x+h, y:y+w] += im[:, :, None]*np.asarray(to_rgb(COLORS[layer['id']]))
    return np.clip(rgb, 0, 1), lower


def match_figure(out, maps, pair, pool):
    result = pair['methods']['pooled']; T = result['T_query_candidate_level_2d']
    T = np.eye(3) if T is None else np.asarray(T)
    fig, axes = plt.subplots(1, 2, figsize=(13, 7), layout='constrained', sharex=True, sharey=True)
    for ax, endpoint, transform in zip(axes, [tuple(pair['candidate']), tuple(pair['query'])], [T, np.eye(3)]):
        im, lower = composite(out, maps[endpoint]); res = .5
        pixel = np.array([[0, res, res*(lower[0]+.5)], [res, 0, res*(lower[1]+.5)], [0, 0, 1]])
        ax.imshow(im, origin='upper', interpolation='nearest', transform=Affine2D(transform@pixel)+ax.transData)
        ax.set_facecolor('#0e121b'); ax.set_aspect('equal'); ax.set_xlim(-100, 100); ax.set_ylim(-100, 100)
        ax.set_title(f'{endpoint[0]} / {endpoint[1]}'); ax.set_xlabel('Horizontal X [m]')
    axes[0].set_ylabel('Horizontal Y [m]')
    q = pool['query_xy_m']; c = pool['candidate_xy_m']@T[:2, :2].T+T[:2, 2]
    members = set(result['inlier_indices'])
    for i, (a, b, layer) in enumerate(zip(c, q, pool['layer'])):
        accepted = i in members; color = COLORS[str(layer)] if accepted else '#adb5c1'
        for ax, point in zip(axes, (a, b)):
            ax.scatter(*point, s=19 if accepted else 8, facecolors='none', edgecolors=color,
                       linewidths=.8 if accepted else .4, alpha=1 if accepted else .35)
        fig.add_artist(ConnectionPatch(a, b, 'data', 'data', axesA=axes[0], axesB=axes[1],
                                      color=color, lw=.65 if accepted else .3, alpha=.7 if accepted else .1))
    support = ' · '.join(f'{name}: {result["layer_support"].get(name, 0)}' for name in COLORS)
    fig.suptitle(f'Joint RANSAC: {result["inliers"]} unique inliers / {result["matches"]} tentative matches\n'
                 f'{support}\nDisplay aligned by this joint estimate; no GT alignment or 3D verification', fontsize=12)
    axes[0].legend(handles=[Patch(color=COLORS[n], label=l) for n, l, _, _ in preview.layers.BANDS],
                   loc='lower left', fontsize=8, framealpha=.85)
    fig.savefig(out/pair['figure'], dpi=130, bbox_inches='tight', pad_inches=.12); plt.close(fig)


def write_gallery(out, summary):
    rows = []
    for p in summary['pairs']:
        m = p['methods']
        rows.append('| '+p['id']+' | '+' | '.join(str(m[n]['inliers']) for n in [*NAMES, 'pooled'])+' |')
    text = ['# Joint multilayer BEV RANSAC preview', '',
        'Fixed 48-pair ground–aerial Cartesian-product diagnostic. No bag replay, GICP, PCM or CBS.', '',
        'Same-layer native ORB/HBST tentative matches enter a proper rigid SE(2) fitter. Raw matches, including each layer’s rejected matches, are pooled. Full-height geometry is a separate control. Native keypoints already include image crop origins; pixel coordinates are converted to meters.', '',
        'Both endpoints within 0.5 m identify duplicate constraints; lowest Hamming distance wins. Final support is also one-to-one within 0.5 m at each endpoint. Hypotheses sample layers uniformly, then matches uniformly within each chosen layer. Settings are fixed: 688 draws, seed 0, strict residual <1.5 m (native 3 pixels), and strict >5 unique-inlier gate. Refinement fits a proper rotation without scale and recounts support afterward.', '',
        '**This is a new consensus implementation, not bit-for-bit native RANSAC.** Every full-height and individual-layer control also uses this same fitter and uniqueness rule. Original native counts and poses are retained separately for comparison. Layer balancing changes sampling; it does not guarantee independent evidence or calibrated false-positive rates.', '',
        'Each match figure uses its own joint estimate only for display. Colored lines are unique joint inliers, colored by retained layer; gray lines are rejected. A visually aligned image is not independent validation. The separate evaluation script may subsequently read GT; this matching stage never does.', '',
        '| Pair | Full | Near | Low | Middle | High | Upper | Pooled |',
        '|---|---:|---:|---:|---:|---:|---:|---:|', *rows, '',
        '[Gallery](index.html) · [Fixed design](design.json) · [Raw matching diagnostics](matching.json) · [References](REFERENCES.md)', '']
    (out/'README.md').write_text('\n'.join(text))
    html = '''<!doctype html><html lang="en"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>Joint multilayer BEV matches</title>
<style>body{margin:0;background:#edf1f6;color:#142237;font:16px system-ui}main{max-width:1450px;margin:auto;padding:24px}select{font:inherit;padding:9px}img{width:100%}figure{margin:15px 0;background:white;padding:8px;border-radius:10px}table{border-collapse:collapse;background:white;margin:14px 0}td,th{padding:9px 14px;border:1px solid #d7deea}p{line-height:1.5}a{color:#175b9c}</style>
<main><h1>Joint multilayer BEV matching</h1><p>48 fixed ground–aerial pairs. Unchanged 1.5 m residual and >5-inlier gate. Shared rigid 2D fit with duplicate removal and layer-balanced sampling. No 3D registration or graph optimization.</p>
<label>Pair <select id="pair"></select></label><div id="stats"></div><figure><img id="plot" alt="Joint matches, colored by layer"><figcaption>Colored: unique joint inliers. Gray: rejected matches. Images are placed using the joint estimate, not ground truth; visual alignment is not independent verification.</figcaption></figure>
<p><a href="README.md">Method and all counts</a> · <a href="matching.json">Matching diagnostics</a> · <a href="evaluation-report.md">Separate evaluation</a> · <a href="REFERENCES.md">Reference papers</a></p></main>
<script>const DATA=__DATA__;const select=document.getElementById('pair');DATA.pairs.forEach((p,i)=>select.add(new Option(p.id,i)));function show(){const p=DATA.pairs[+select.value];document.getElementById('plot').src=p.figure;let rows=Object.entries(p.methods).map(([name,m])=>`<tr><td>${name}</td><td>${m.input_matches}</td><td>${m.matches}</td><td>${m.inliers}</td><td>${m.passes_2d_gate?'Pass':'Reject'}</td></tr>`).join('');document.getElementById('stats').innerHTML='<table><tr><th>Method</th><th>Raw matches</th><th>Unique tentative</th><th>Unique inliers</th><th>2D gate</th></tr>'+rows+'</table>';}select.onchange=show;select.value=DATA.pairs.findIndex(p=>p.id==='aerial06-27__ground06-17');show();</script></html>'''
    (out/'index.html').write_text(html.replace('__DATA__', json.dumps(summary)))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--work', type=Path, required=True)
    parser.add_argument('--cached-preview', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(); work, out, old = args.work.resolve(), args.output.resolve(), args.cached_preview.resolve()
    out.mkdir(exist_ok=False)
    for name in ['images', 'features', 'pairs']:
        (out/name).mkdir()
    started = time.monotonic()
    config = yaml.safe_load((work/'config.yaml').read_text()); mc = config['backend']['mapclosures']
    assert mc['density_map_resolution'] == .5 and mc['inliers_threshold'] == 5
    assert mc['upstream_commit'] == preview.native.upstream_commit == preview.inspection.upstream_commit
    # Frozen before descriptor generation or matching; no GT-based pair selection.
    ids = {'aerial06': [7, 12, 17, 22, 27, 32], 'ground06': [0, 5, 7, 12, 17, 22, 27, 30]}
    design = dict(endpoint_ids=ids, pair_selection='fixed Cartesian product before matching or evaluation',
        comparisons=[*NAMES, 'pooled'], settings=joint.SETTINGS,
        sampling='uniform layer, then uniform match for each of two draws',
        same_layer_matching=True, full_height_excluded_from_pool=True,
        deduplication='both endpoints within 0.5 m; final one-to-one support within 0.5 m',
        controls='same new rigid fitter on each single layer and full height; native fits retained separately',
        ground_truth_used_for_matching=False, subsequent_evaluation_only=True,
        negative_label='evaluation-only GT horizontal center separation > radius sum + 20 m',
        pose_error_reporting='horizontal translation and yaw; diagnostic 5 m / 5 degree bands, not acceptance gates',
        repeats=dict(pairs=[[['aerial06', 27], ['ground06', 17]], [['aerial06', 17], ['ground06', 7]]], seeds=[0, 1, 2]),
        run_registration=False, run_pcm=False, run_cbs=False)
    (out/'design.json').write_text(json.dumps(design, indent=2)+'\n')
    trials = json.loads((work/'trials.json').read_text())
    old_summary = json.loads((old/'summary.json').read_text())
    old_maps = {(m['robot'], m['key']): m for m in old_summary['maps']}
    inputs = {str(work/'config.yaml'): sha(work/'config.yaml'), str(old/'summary.json'): sha(old/'summary.json')}
    inputs.update(old_summary['input_sha256'])
    maps, packets = {}, {}
    for r, keys in ids.items():
        key_path = work/f'prepared-{r}/store/keyframes.jsonl'; inputs[str(key_path)] = sha(key_path)
        keyframes = {k['keyframe_id']: k for k in preview.rows(key_path)}
        for k in keys:
            key = keyframes[k]
            if (r, k) in old_maps:
                entry = json.loads(json.dumps(old_maps[r, k])); features = {}
                for layer in entry['layers']:
                    image = old/layer['image']; assert sha(image) == layer['png_sha256']
                    feature = old/'features'/f'{r}-{k:06d}-{layer["id"]}.npz'
                    for src, dst in [(image, out/layer['image']), (feature, out/'features'/feature.name)]:
                        inputs[str(src)] = sha(src); shutil.copy2(src, dst)
                    with np.load(feature) as data:
                        features[layer['id']] = {name: data[name].copy() for name in ['ground', 'xy', 'bits']}
            else:
                entry, features, hashes = cache_map(work, out, trials, key, mc, config['ellipsoid_basis_roundoff_tolerance'])
                inputs.update(hashes)
            entry.update(stamp_ns=key['stamp_ns'], area_radius_m=key['area_radius_m'])
            maps[r, k] = entry
            for name, feature in features.items():
                packets[r, k, name] = feature
            print('cached', r, k, 'features', [m['features'] for m in entry['layers']], flush=True)
    result = []
    def engine(cls):
        return cls(mc['density_map_resolution'], mc['density_threshold'], mc['hamming_distance_threshold'])
    for qk in ids['aerial06']:
        for ck in ids['ground06']:
            q, c = ('aerial06', qk), ('ground06', ck)
            pair_id = f'aerial06-{qk}__ground06-{ck}'
            record = dict(id=pair_id, query=q, candidate=c, methods={}, native={},
                          figure=f'pairs/{pair_id}-joint.png')
            debug_packets = {}
            for name in NAMES:
                qf, cf = packets[(*q, name)], packets[(*c, name)]
                native = engine(preview.native.MapClosures).pair(qf, cf)
                inspector = engine(preview.inspection.Inspector); inspector.add(0, cf)
                debug = inspector.correspondences(qf, 0)
                assert len(debug['query_xy']) == native['matches']
                assert len(debug['ransac_inlier_indices']) == native['inliers']
                debug_packets[name] = debug
                np.savez_compressed(out/'pairs'/f'{pair_id}-{name}-native.npz', **debug)
                record['native'][name] = {n: v.tolist() if isinstance(v, np.ndarray) else v for n, v in native.items()}
                pool = joint.pool_correspondences({name: debug})
                record['methods'][name] = joint.fit_consensus(pool)
            pool = joint.pool_correspondences({n: debug_packets[n] for n in NAMES if n != 'full'})
            record['methods']['pooled'] = joint.fit_consensus(pool)
            record['duplicate_groups'] = pool['duplicate_groups']
            np.savez_compressed(out/'pairs'/f'{pair_id}-pooled.npz', **{n: pool[n] for n in
                ['query_xy_m', 'candidate_xy_m', 'layer', 'hamming']})
            if (qk, ck) in [(27, 17), (17, 7)]:
                record['seed_repeats'] = [record['methods']['pooled'], *[joint.fit_consensus(pool, {'seed': seed}) for seed in [1, 2]]]
            match_figure(out, maps, record, pool)
            result.append(record)
            print(pair_id, 'native full', record['native']['full']['inliers'], 'unique',
                  {name: m['inliers'] for name, m in record['methods'].items()}, flush=True)
    assert all(sha(Path(p)) == h for p, h in inputs.items())
    source_paths = [Path(__file__), HERE/'preview_multilayer_bevs.py',
                    ROOT/'research/s3e_pipeline/joint_bev_ransac.py', preview.MODULE,
                    Path(preview.native.__file__), Path(preview.inspection.__file__)]
    summary = dict(complete=True, design=design, maps=list(maps.values()), pairs=result,
        input_sha256=inputs, source_sha256={str(p): sha(p) for p in source_paths},
        ground_truth_used=False, original_full_height_features_exactly_reproduced=True,
        source_artifacts_unchanged=True, registration_run=False, pcm_run=False, cbs_run=False,
        wall_s=time.monotonic()-started)
    (out/'matching.json').write_text(json.dumps(summary, indent=2)+'\n')
    write_gallery(out, summary)
    for p in source_paths[:4]:
        shutil.copy2(p, out/p.name)
    print('joint preview complete', summary['wall_s'], flush=True)


if __name__ == '__main__':
    main()
