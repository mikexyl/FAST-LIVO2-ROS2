"""Layout-only redraw from frozen matches; does not rerun matching/evaluation."""
import importlib.util
import json
from pathlib import Path
import numpy as np

out = Path(__file__).resolve().parent
root = out.parents[3]
path = root/'scripts/recent_submaps/preview_joint_bev_matches.py'
spec = importlib.util.spec_from_file_location('joint_preview_render', path)
module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
matching_hash = module.sha(out/'matching.json')
evaluation_hash = module.sha(out/'evaluation.json')
summary = json.loads((out/'matching.json').read_text())
maps = {(m['robot'], m['key']): m for m in summary['maps']}
for pair in summary['pairs']:
    with np.load(out/'pairs'/f'{pair["id"]}-pooled.npz') as archive:
        pool = {name: archive[name].copy() for name in archive.files}
    module.match_figure(out, maps, pair, pool)
assert module.sha(out/'matching.json') == matching_hash
assert module.sha(out/'evaluation.json') == evaluation_hash
record = dict(layout_only=True, change='tight bounding box preserves complete axis labels',
              matching_sha256=matching_hash, evaluation_sha256=evaluation_hash,
              current_renderer_sha256=module.sha(path), redraw_script_sha256=module.sha(Path(__file__)),
              matching_runner_snapshot_sha256=module.sha(out/'preview_joint_bev_matches.py'),
              figures={p['figure']: module.sha(out/p['figure']) for p in summary['pairs']})
(out/'figure-refresh.json').write_text(json.dumps(record, indent=2)+'\n')
(out/'preview_joint_bev_matches_render.py').write_bytes(path.read_bytes())
print('Refreshed', len(summary['pairs']), 'figures; matching and evaluation hashes unchanged')
