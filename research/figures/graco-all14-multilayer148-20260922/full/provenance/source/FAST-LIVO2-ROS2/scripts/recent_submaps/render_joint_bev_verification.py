#!/usr/bin/env python3
"""Render frozen verification geometry and a gallery without reading GT."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from s3e_pipeline.registration import bounded_cloud
from s3e_pipeline.geometry import transform


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path)
    args = parser.parse_args(); out = args.output.resolve()
    summary_path = out/'summary.json'; frozen = sha(summary_path)
    data = json.loads(summary_path.read_text()); assert data['complete']
    (out/'views').mkdir(exist_ok=False)
    (out/'visual_geometry').mkdir(exist_ok=False)
    points, grounds, files = {}, {}, []
    for m in data['maps']:
        key = (m['robot'], m['key'])
        assert sha(out/m['geometry']) == m['sha256']
        with np.load(out/m['geometry']) as z:
            # Visualization-only voxel size; verification used 0.4 m unchanged.
            cloud, _ = bounded_cloud(z['cloud'], .8, None, None)
            G = z['ground'].copy()
        name = f'visual_geometry/{key[0]}-{key[1]:06d}.npz'
        np.savez_compressed(out/name, cloud=cloud.astype(np.float32), ground=G)
        files.append(dict(robot=key[0], key=key[1], path=name, points=len(cloud), sha256=sha(out/name)))
        points[key], grounds[key] = cloud, G
    views = []
    for row in data['results']:
        q, c = tuple(row['query']), tuple(row['candidate']); result = row['verification']
        target = transform(grounds[q], points[q])
        before = transform(grounds[q]@np.asarray(row['height_seed_T_i_j']), points[c])
        T = result.get('T_i_j', row['height_seed_T_i_j'])
        after = transform(grounds[q]@np.asarray(T), points[c])
        all_points = np.concatenate([target, before, after])
        lo, hi = all_points.min(axis=0)-3, all_points.max(axis=0)+3
        fig, axes = plt.subplots(2, 2, figsize=(13, 10), layout='constrained')
        for col, (cloud, label) in enumerate([(before, 'Joint BEV + height initialization'),
                (after, 'GICP result' if 'T_i_j' in result else 'No refinement; unchanged seed')]):
            for line, dimensions in enumerate([(0, 1), (0, 2)]):
                ax = axes[line, col]; i, j = dimensions
                ax.set_facecolor('#101824')
                ax.scatter(target[:, i], target[:, j], s=.5, c='#4ea9ff', alpha=.65,
                           linewidths=0, rasterized=True, label='Aerial query')
                ax.scatter(cloud[:, i], cloud[:, j], s=.5, c='#ffa95b', alpha=.65,
                           linewidths=0, rasterized=True, label='Ground candidate')
                ax.set_xlim(lo[i], hi[i]); ax.set_ylim(lo[j], hi[j]); ax.set_aspect('equal')
                ax.set_xlabel('Horizontal X [m]'); ax.set_ylabel('Horizontal Y [m]' if j == 1 else 'Gravity-level Z [m]')
                ax.set_title(label+(' · top view' if j == 1 else ' · side view'))
        axes[0, 0].legend(loc='lower left', markerscale=5, framealpha=.85)
        overlap = result.get('overlap'); rmse = result.get('rmse_m')
        values = f'overlap {overlap:.1%} · RMSE {rmse:.3f} m' if overlap is not None and rmse is not None else result['reason']
        fig.suptitle(f'{row["id"]} · {"ACCEPTED" if result["accepted"] else "REJECTED"}: {result["reason"]}\n'
            f'{row["pooled_inliers"]} BEV inliers · height correction {row["vertical_initialization"]["correction_m"]:+.1f} m · {values}\n'
            'Query gravity frame; no GT display alignment · 0.8 m voxels for visualization only', fontsize=12)
        path = f'views/{row["id"]}.png'; fig.savefig(out/path, dpi=125, bbox_inches='tight'); plt.close(fig)
        views.append(dict(id=row['id'], accepted=result['accepted'], reason=result['reason'],
                          path=path, sha256=sha(out/path)))
        print('rendered', row['id'], flush=True)
    html = '''<!doctype html><html lang="en"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>Joint BEV 3D verification</title>
<style>body{margin:0;background:#edf1f6;color:#142237;font:16px system-ui}main{max-width:1450px;margin:auto;padding:25px}p{line-height:1.5}select{padding:9px;font:inherit}img{width:100%}figure{background:white;padding:10px;margin:18px 0;border-radius:10px}table{border-collapse:collapse;background:white}td,th{padding:8px 14px;border:1px solid #d5dde8}a{color:#155ba3}</style>
<main><h1>Joint multilayer BEV → height → GICP</h1><p>Every one of the 16 passing joint hypotheses was verified using the existing settings. Blue: aerial query. Orange: ground candidate. Left: height-initialized seed. Right: GICP output. Coordinates come from estimated transforms; no GT alignment is used to draw these figures.</p>
<label>Candidate <select id="pair"></select></label><div id="stats"></div><figure><img id="view" alt="Top and side views before and after GICP"></figure>
<p><a href="REPORT.md">Results and evaluation</a> · <a href="verification.rrd">Interactive 3D Rerun recording</a> · <a href="summary.json">Frozen verification diagnostics</a> · <a href="evaluation.json">Separate GT evaluation</a> · <a href="REFERENCES.md">References</a></p></main>
<script>const DATA=__DATA__;const select=document.getElementById('pair');DATA.results.forEach((p,i)=>select.add(new Option(`${p.id} — ${p.verification.accepted?'ACCEPTED':'REJECTED'} (${p.verification.reason})`,i)));function show(){const p=DATA.results[+select.value],r=p.verification;document.getElementById('view').src=`views/${p.id}.png`;const entries=[['BEV unique inliers',p.pooled_inliers],['Height correction',p.vertical_initialization.correction_m+' m'],['Initial overlap',r.initial_overlap==null?'—':(100*r.initial_overlap).toFixed(1)+'%'],['Final symmetric overlap',r.overlap==null?'—':(100*r.overlap).toFixed(1)+'%'],['RMSE',r.rmse_m==null?'—':r.rmse_m.toFixed(3)+' m'],['Verdict',r.reason]];document.getElementById('stats').innerHTML='<table>'+entries.map(([k,v])=>`<tr><th>${k}</th><td>${v}</td></tr>`).join('')+'</table>';}select.onchange=show;select.value=DATA.results.findIndex(p=>p.id==='aerial06-27__ground06-17');show();</script></html>'''
    (out/'index.html').write_text(html.replace('__DATA__', json.dumps(data)))
    assert sha(summary_path) == frozen
    record = dict(ground_truth_used=False, visualization_voxel_m=.8, verification_voxel_m=.4,
                  frozen_verification_sha256=frozen, maps=files, views=views, script_sha256=sha(Path(__file__)))
    (out/'rendering.json').write_text(json.dumps(record, indent=2)+'\n')
    shutil.copy2(Path(__file__), out/'source'/Path(__file__).name)


if __name__ == '__main__':
    main()
