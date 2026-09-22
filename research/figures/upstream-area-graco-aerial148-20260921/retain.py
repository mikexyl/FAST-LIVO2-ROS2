"""Select review artifacts for local retention; preserve all bulk files on 148."""
import hashlib
import json
from pathlib import Path

B=Path('/workspace/.ros2/upstream-area-graco-aerial148-20260921')
W=B/'full'
assert json.loads((W/'status.json').read_text())['phase']=='complete'
assert (W/'REPORT.md').exists()
gallery=json.loads((W/'bev-gallery/manifest.json').read_text())
assert gallery['complete'] and gallery['exact_cached_features']
audit=json.loads((W/'retention-audit.json').read_text())
for robot in audit['frontends']:
    loaded=(W/'frontends'/robot/'loaded-library.txt').read_text().splitlines()
    assert loaded and all('upstream-area-s3e-20260921/capacity-run/install' in s for s in loaded)

selected={};bulk=[]
def sha(path):
    digest=hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda:stream.read(8*1024*1024),b''):digest.update(chunk)
    return digest.hexdigest()
for path in sorted(B.rglob('*')):
    if not path.is_file():continue
    rel=path.relative_to(B)
    if '__pycache__' in rel.parts or rel.name in {'retained-files.json','retained-files.txt','retain.log'}:continue
    excluded=(rel.name=='live.rrd' or
              ('frontends' in rel.parts and path.suffix=='.npz') or
              (any(p.startswith('prepared-') for p in rel.parts) and
               (path.suffix=='.npz' or path.suffix=='.zlib')))
    if excluded:
        bulk.append(dict(path=str(rel),bytes=path.stat().st_size))
        continue
    selected[str(rel)]=dict(bytes=path.stat().st_size,sha256=sha(path))
manifest=dict(files=selected,bulk_retained_on_148=bulk,bulk_deleted=False,
    remote_root='/data3/mikexyl/swarm_s3e_ws/src/.ros2/upstream-area-graco-aerial148-20260921',
    source_build_and_prior_tests='/data3/mikexyl/swarm_s3e_ws/src/.ros2/upstream-area-s3e-20260921',
    loaded_capacity_library_verified_for_all_robots=True)
(B/'retained-files.json').write_text(json.dumps(manifest,indent=2)+'\n')
(B/'retained-files.txt').write_text('\n'.join([*selected,'retained-files.json','retained-files.txt'])+'\n')
print(json.dumps(dict(files=len(selected),bytes=sum(v['bytes'] for v in selected.values()),
                     remote_bulk_files=len(bulk),remote_bulk_bytes=sum(v['bytes'] for v in bulk))))
