from pathlib import Path
import hashlib
import json
import shutil

root = Path(__file__).resolve().parents[2]
base = Path(__file__).resolve().parent
work = base / 'smoke'
dest = root / 'FAST-LIVO2-ROS2/research/figures/area-maps/implementation'

def digest(path):
    hasher = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            hasher.update(block)
    return hasher.hexdigest()

def copy(source, target):
    target.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source, target)
    assert digest(source) == digest(target), str(source)

def bulk_geometry(path):
    return path.suffix == '.npz' and any(part in ('submaps', 'area_maps', 'store') for part in path.relative_to(work).parts)

frozen = json.loads((work / 'source-hashes.json').read_text())
assert all(digest(Path(name)) == sha for name, sha in frozen.items())
dest.mkdir(parents=True, exist_ok=False)
for name in ('verification-results.json', 'build.log', 'build.sh', 'env.sh', 'smoke.py', 'analyze.py', 'analyze_gap.py', 'retain.py'):
    copy(base / name, dest / name)
copy(base / 'build/ellipselio/Testing/Temporary/LastTest.log', dest / 'native-tests.log')
copy(base / 'smoke-before-anchor-fix/status.json', dest / 'before-anchor-fix-status.json')
external_geometry = {}
for path in sorted(work.rglob('*')):
    if not path.is_file():
        continue
    if bulk_geometry(path):
        external_geometry[str(path)] = digest(path)
    else:
        copy(path, dest / 'smoke' / path.relative_to(work))
external_binaries = {}
for name, sha in frozen.items():
    path = Path(name)
    if path.suffix in ('.py', '.cpp', '.h', '.hpp', '.msg', '.yaml', '.sh', '.txt') or path.name == 'CMakeLists.txt':
        copy(path, dest / 'source' / path.relative_to(root))
    else:
        external_binaries[name] = sha
for directory in (root / 'ellipselio/tests', root / 'FAST-LIVO2-ROS2/research/tests'):
    for path in directory.glob('*'):
        if path.is_file() and path.suffix in ('.py', '.cpp', '.h'):
            copy(path, dest / 'source' / path.relative_to(root))
for name in ('scripts/recent_submaps/AREA-MAPS.md', 'research/RESULTS-AREA-MAPS.md'):
    path = root / 'FAST-LIVO2-ROS2' / name
    copy(path, dest / 'source' / path.relative_to(root))
(dest / 'external-binaries.json').write_text(json.dumps(external_binaries, indent=2) + '\n')
(dest / 'external-geometry.json').write_text(json.dumps(external_geometry, indent=2) + '\n')
files = {str(path.relative_to(dest)): digest(path) for path in sorted(dest.rglob('*')) if path.is_file()}
manifest = {
    'files': files, 'files_verified': len(files), 'source_root': str(base),
    'ground_truth_used': False, 'full_sequence': False, 'frontend_gate_passed': False,
    'external_geometry_files_verified': len(external_geometry),
    'frozen_sources_verified': True,
}
(dest / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
assert all(digest(dest / name) == sha for name, sha in files.items())
print(f'Retained and verified {len(files)} files; {len(external_geometry)} bulk geometry files remain at {work}')
