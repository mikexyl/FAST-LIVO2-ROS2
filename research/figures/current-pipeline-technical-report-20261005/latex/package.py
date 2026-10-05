#!/usr/bin/env python3
"""Package only the current manuscript and its compact evidence/figure assets."""
import hashlib
import json
import zipfile
from pathlib import Path

root = Path(__file__).resolve().parent.parent
files = []
for dirname in ['latex', 'figures', 'evidence']:
    files += [p for p in (root / dirname).rglob('*') if p.is_file() and '__pycache__' not in p.parts]
files += list((root / 'output/pdf').glob('*.pdf'))
files += [root / name for name in ['README.md', 'REPORT.md', 'EDITORIAL-NOTES.md',
                                   'REPORT-VERIFICATION.json', 'review/table-checks.json']]
files = sorted(set(files))
assert all(p.is_file() for p in files)
manifest = {str(p.relative_to(root)): hashlib.sha256(p.read_bytes()).hexdigest() for p in files}
(root / 'SHA256SUMS.json').write_text(json.dumps(manifest, indent=2) + '\n')
files.append(root / 'SHA256SUMS.json')
out = root / 'technical-report-source.zip'
with zipfile.ZipFile(out, 'w', zipfile.ZIP_DEFLATED, compresslevel=9) as z:
    for p in files:
        z.write(p, 'technical-report/' + str(p.relative_to(root)))
with zipfile.ZipFile(out) as z:
    assert z.testzip() is None
print(json.dumps({'zip': str(out), 'files': len(files), 'bytes': out.stat().st_size,
                  'sha256': hashlib.sha256(out.read_bytes()).hexdigest()}, indent=2))
