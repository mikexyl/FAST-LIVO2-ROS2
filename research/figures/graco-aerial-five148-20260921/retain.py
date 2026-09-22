"""Keep portable results; bulk area payloads and staged inputs remain on 148."""
import hashlib,json
from pathlib import Path
B=Path('/workspace/.ros2/graco-aerial-five148-20260921');W=B/'full'
assert json.loads((W/'status.json').read_text())['phase']=='complete'
assert (W/'REPORT.md').exists() and (W/'bev-gallery/manifest.json').exists()
def sha(p):
    h=hashlib.sha256()
    with p.open('rb') as f:
        for x in iter(lambda:f.read(8*1024*1024),b''):h.update(x)
    return h.hexdigest()
files={}
for p in B.rglob('*'):
    if not p.is_file() or p.is_symlink():continue
    rel=p.relative_to(B)
    if rel.parts[0] in ('inputs','converted','preparation-before-transfer-completed','gallery-before-backend-completed'):continue
    if '__pycache__' in rel.parts or '.pytest_cache' in rel.parts:continue
    if p.name in ('retained-files.json','retained-files.txt','local-copy-verification.json'):continue
    if p.suffix=='.npz' and ('area_maps' in rel.parts or 'prepared-aerial04' in rel.parts):continue
    files[str(rel)]=dict(bytes=p.stat().st_size,sha256=sha(p))
for r in ('aerial05','aerial06','aerial07','aerial08'):
    prepared=W/f'prepared-{r}'
    for p in [prepared/'store/keyframes.jsonl',prepared/'summary.json',prepared/'timings.jsonl']:
        files[str(p.relative_to(B))]=dict(bytes=p.stat().st_size,sha256=sha(p))
manifest=dict(files=files,remote_root='/data3/mikexyl/swarm_s3e_ws/src/.ros2/graco-aerial-five148-20260921',
    all_bulk_geometry_retained_on_148=True,bulk_deleted=False,prior_outputs_preserved=True,
    reused_capture_root='/data3/mikexyl/swarm_s3e_ws/src/.ros2/upstream-area-graco-aerial148-20260921/full')
(B/'retained-files.json').write_text(json.dumps(manifest,indent=2)+'\n')
(B/'retained-files.txt').write_text('\n'.join([*files,'retained-files.json','retained-files.txt'])+'\n')
print(json.dumps(dict(files=len(files),bytes=sum(v['bytes'] for v in files.values()))))
