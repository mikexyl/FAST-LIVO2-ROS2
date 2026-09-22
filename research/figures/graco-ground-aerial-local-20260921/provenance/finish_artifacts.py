"""Preserve a reviewable local experiment, without duplicating bulk evidence."""
import hashlib
import json
import shutil
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.image as mpimg

B = Path(__file__).resolve().parent
HOST = B.parents[1]
W = B / 'source/output'
DEST = HOST / 'FAST-LIVO2-ROS2/research/figures/graco-ground-aerial-local-20260921'
assert json.loads((W/'status.json').read_text())['phase'] == 'complete'
assert not DEST.exists()
manifest = json.loads((W/'bev-gallery/manifest.json').read_text())
assert manifest['complete'] and manifest['exact_cached_features']
candidates = json.loads((W/'diagnostics/cross-robot-candidates.json').read_text())
best = max(candidates, key=lambda c: (c['mapclosures_hypothesis']['inliers'], c['mapclosures_hypothesis']['matches']))
fig, axes = plt.subplots(1, 2, figsize=(12, 6), layout='constrained')
for ax, endpoint in zip(axes, (best['candidate'], best['query'])):
    m = next(m for m in manifest['maps'] if [m['robot'], m['key']] == endpoint)
    im = mpimg.imread(W/'bev-gallery'/m['image'])
    ax.imshow(im, cmap='gray', vmin=0, vmax=1, interpolation='nearest')
    ax.set_title(f"{endpoint[0]} / {endpoint[1]} · snapshot {m['end_s']:.0f} s\n{m['features']} ORB features")
    ax.axis('off')
    x, y, length = 10, im.shape[0]-12, 20/manifest['density_map_resolution_m']
    ax.plot([x, x+length], [y, y], color='#ffd75e', lw=3)
    ax.text(x+length/2, y-5, '20 m', color='#ffd75e', ha='center', fontsize=10)
fig.suptitle('Best rejected cross-robot candidate — NOT a verified loop\n5 RANSAC inliers; unchanged acceptance gate requires >5', fontsize=15)
fig.savefig(W/'diagnostics/best-rejected-bev-pair.png', dpi=170)
plt.close(fig)
gallery = W/'bev-gallery/index.html'
html = gallery.read_text().replace('<select id="pair">', '<p><strong>No loop pairs passed retrieval in this experiment.</strong> The best cross-robot candidate had five RANSAC inliers; the configured gate requires more than five. No candidate reached 3D verification.</p><select id="pair">')
html = html.replace('<p class="meta">0.5 m per image pixel.', '<h2>Best rejected candidate</h2><p>This is a retrieval diagnostic, not a recovered alignment. Each image keeps its native orientation.</p><img style="max-width:100%" src="../diagnostics/best-rejected-bev-pair.png" alt="Best rejected ground–aerial BEV candidate"><p class="meta">0.5 m per image pixel.')
gallery.write_text(html)
(W/'REPORT.md').write_text((W/'REPORT.md').read_text().replace('![Trajectories and maps]', '![Best rejected BEV pair](diagnostics/best-rejected-bev-pair.png)\n\n![Trajectories and maps]'))

def sha(p):
    h=hashlib.sha256()
    with p.open('rb') as f:
        while chunk:=f.read(2**20): h.update(chunk)
    return h.hexdigest()

copied = {}
def copy(src, dst):
    dst.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(src, dst)
    expected = sha(src)
    assert sha(dst) == expected, str(dst)
    copied[str(dst.relative_to(DEST))] = dict(source=str(src), sha256=expected, bytes=src.stat().st_size)

DEST.mkdir(parents=True)
excluded = []
for p in sorted(W.rglob('*')):
    if not p.is_file(): continue
    relative = p.relative_to(W)
    # Keep map products and all recordings, but leave original evidence in W.
    if p.suffix == '.npz' and ('area_maps' in relative.parts or relative.parts[0].startswith('prepared-')):
        excluded.append(dict(path=str(p), bytes=p.stat().st_size, sha256=sha(p)))
        continue
    copy(p, DEST/'full'/relative)
for root in (B/'source/FAST-LIVO2-ROS2', B/'source/ellipselio'):
    for p in sorted(root.rglob('*')):
        if p.is_file() and '__pycache__' not in p.parts:
            copy(p, DEST/'source'/root.name/p.relative_to(root))
for p in sorted(B.iterdir()):
    if p.is_file() and (p.suffix in ('.py','.sh','.log','.json','.png') or p.name == 'BUILD_COMPLETE'):
        copy(p, DEST/'provenance'/p.name)
(DEST/'README.md').write_text('[Full report](full/REPORT.md) · [BEV gallery](full/bev-gallery/index.html) · [Rerun](full/report/result.rrd)\n\n'
    f'Run on the local laptop. Original full output and bulk evidence remain at `{W}`. '
    'No original or intermediate geometry was deleted. Absolute paths in frozen metadata identify original runtime locations. '
    'The retained full/ directory contains trajectories, maps, all three verified Rerun recordings, descriptor packets, logs, configuration, audit, and evaluation evidence.\n')
copy_manifest=dict(complete=True, original_output=str(W), copied_files=copied,
    bulk_evidence_retained_in_original_location=excluded, deleted_files=[],
    gallery_density_images_unchanged=True, report_postprocessor_sha256=sha(Path(__file__)))
(DEST/'copy-verification.json').write_text(json.dumps(copy_manifest,indent=2)+'\n')
top = HOST/'FAST-LIVO2-ROS2/research/RESULTS-GRACO-GROUND06-AERIAL06-LOCAL.md'
assert not top.exists()
prefix='figures/graco-ground-aerial-local-20260921/full/'
report=(W/'REPORT.md').read_text()
for directory in ('diagnostics/','bev-gallery/','report/'):
    report=report.replace(']('+directory,']('+prefix+directory)
report=report.replace('](retention-audit.json)',']('+prefix+'retention-audit.json)')
top.write_text(report)
print(json.dumps(dict(report=str(top),artifacts=str(DEST),files=len(copied),
    retained_bytes=sum(v['bytes'] for v in copied.values()),
    omitted_bulk_files=len(excluded),bulk_deleted=False),indent=2))
