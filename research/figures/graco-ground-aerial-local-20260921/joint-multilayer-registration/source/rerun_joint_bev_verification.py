#!/usr/bin/env python3
"""Record frozen before/after verification geometry, without GT or recomputation."""
import json
from pathlib import Path
import sys
import hashlib
import shutil
import numpy as np
import rerun as rr
import rerun.blueprint as rrb


def transform(T, points):
    return points@T[:3, :3].T+T[:3, 3]


def main(out):
    out = Path(out).resolve()
    assert rr.__version__ == '0.37.1'
    data = json.loads((out/'summary.json').read_text())
    rendering = json.loads((out/'rendering.json').read_text())
    clouds, grounds = {}, {}
    for m in rendering['maps']:
        with np.load(out/m['path']) as z:
            clouds[m['robot'], m['key']] = z['cloud'].copy()
            grounds[m['robot'], m['key']] = z['ground'].copy()
    rr.init('GRACO joint multilayer BEV verification'); rr.save(str(out/'verification.rrd'))
    rr.send_blueprint(rrb.Blueprint(rrb.Vertical(
        rrb.Horizontal(rrb.Spatial3DView(name='Joint BEV + height seed', origin='/seed'),
                       rrb.Spatial3DView(name='GICP result', origin='/result')),
        rrb.TextDocumentView(name='Verdict and thresholds', origin='/status'), row_shares=[.8, .2]),
        rrb.TimePanel(timeline='pair', play_state='Paused', fps=1), collapse_panels=True))
    for i, row in enumerate(data['results']):
        rr.set_time('pair', sequence=i)
        q, c = tuple(row['query']), tuple(row['candidate']); result = row['verification']
        G = grounds[q]; query = transform(G, clouds[q])
        for root, T in [('seed', row['height_seed_T_i_j']),
                        ('result', result.get('T_i_j', row['height_seed_T_i_j']))]:
            rr.log('/'+root, rr.Clear(recursive=True))
            rr.log('/'+root, rr.ViewCoordinates.RIGHT_HAND_Z_UP)
            rr.log('/'+root+'/aerial_query', rr.Points3D(query, colors=[78, 169, 255], radii=.09))
            rr.log('/'+root+'/ground_candidate', rr.Points3D(transform(G@np.asarray(T), clouds[c]),
                                                            colors=[255, 169, 91], radii=.09))
        text = f'''# Pair {i}: {row['id']}
**{'ACCEPTED' if result['accepted'] else 'REJECTED'} — {result['reason']}**

Joint BEV unique inliers: {row['pooled_inliers']}; geometric height correction: {row['vertical_initialization']['correction_m']:+.1f} m.

Initial overlap: {result.get('initial_overlap')}; final symmetric overlap: {result.get('overlap')}; RMSE: {result.get('rmse_m')} m.

Unchanged acceptance: convergence, ≥100 inliers, ≥30% symmetric overlap within 0.6 m, RMSE ≤0.35 m, observability and conditioning checks.

Blue is aerial query; orange is ground candidate. Left is the height-initialized seed; right is the returned GICP transform (or unchanged seed if refinement did not run). Scrub the pair timeline to inspect all 16 candidates.

Geometry displayed at 0.8 m voxels; verification used fixed 0.4 m voxels without range/point-count cropping. No GT is used for these displays. No PCM/CBS was run and no loop was added to the graph.
'''
        rr.log('/status', rr.TextDocument(text, media_type=rr.MediaType.MARKDOWN))
    rr.get_data_recording().flush()
    shutil.copy2(Path(__file__), out/'source'/Path(__file__).name)
    print('Recorded', len(data['results']), 'verification pairs')


if __name__ == '__main__':
    main(sys.argv[1])
