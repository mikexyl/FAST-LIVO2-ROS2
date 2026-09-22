"""Retain new outputs and source; leave the reused large geometry on 148."""
import hashlib,json
from pathlib import Path
B=Path('/workspace/.ros2/graco-aerial-height148-20260921');W=B/'full'
assert json.loads((W/'status.json').read_text())['phase']=='complete'
assert (W/'REPORT.md').exists()
def sha(p):
    h=hashlib.sha256()
    with p.open('rb') as f:
        for chunk in iter(lambda:f.read(8*1024*1024),b''):h.update(chunk)
    return h.hexdigest()
files={}
paths=[p for p in B.rglob('*') if p.is_file() and not p.is_symlink()]
# Resolve only the compact prepared metadata, not geometry/descriptor payloads.
for robot in ('aerial05','aerial06','aerial07','aerial08'):
    prepared=W/f'prepared-{robot}'
    paths.extend([prepared/'store/keyframes.jsonl',prepared/'summary.json',prepared/'timings.jsonl'])
for p in sorted(set(paths)):
    rel=p.relative_to(B)
    if '__pycache__' in rel.parts or '.pytest_cache' in rel.parts or p.name in ('retained-files.json','retained-files.txt'):continue
    files[str(rel)]=dict(bytes=p.stat().st_size,sha256=sha(p))
manifest=dict(files=files,remote_root='/data3/mikexyl/swarm_s3e_ws/src/.ros2/graco-aerial-height148-20260921',
    reused_geometry_root='/data3/mikexyl/swarm_s3e_ws/src/.ros2/upstream-area-graco-aerial148-20260921/full',
    source_and_original_results_preserved=True,bulk_deleted=False)
(B/'retained-files.json').write_text(json.dumps(manifest,indent=2)+'\n')
(B/'retained-files.txt').write_text('\n'.join([*files,'retained-files.json','retained-files.txt'])+'\n')
print(json.dumps(dict(files=len(files),bytes=sum(v['bytes'] for v in files.values()))))
