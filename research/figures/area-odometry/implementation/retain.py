from pathlib import Path
import hashlib
import json
import shutil

root=Path(__file__).resolve().parents[2];base=Path(__file__).resolve().parent;work=base/'comparison'
dest=root/'FAST-LIVO2-ROS2/research/figures/area-odometry/implementation'
def digest(path):
    h=hashlib.sha256()
    with path.open('rb') as f:
        for block in iter(lambda:f.read(1024*1024),b''):h.update(block)
    return h.hexdigest()
def copy(source,target):
    target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(source,target)
    assert digest(source)==digest(target)

state=json.loads((work/'status.json').read_text());assert state['phase']=='complete'
full=base/'full';assert json.loads((full/'status.json').read_text())['phase']=='complete'
frozen=json.loads((work/'source-hashes.json').read_text())
assert all(digest(Path(p))==h for p,h in frozen.items())
dest.mkdir(parents=True,exist_ok=False)
for name in ['build.sh','env.sh','build.log','trial.py','full.py','evaluate_full.py','plot.py','retain.py','verification-results.json']:
    copy(base/name,dest/name)
copy(base/'build/ellipselio/Testing/Temporary/LastTest.log',dest/'native-tests.log')
copy(base/'build/ellipselio/CMakeCache.txt',dest/'CMakeCache.txt')
for name in ('status.json','capture.log','aerial08/recorder.log'):
    copy(base/'full-dds-domain-failed'/name,dest/'full-dds-domain-failed'/name)
geometry={};binaries={}
for folder in (work,full):
    for path in sorted(folder.rglob('*')):
        if not path.is_file():continue
        if path.suffix=='.npz' and any(p in ('area_maps','submaps') for p in path.relative_to(folder).parts):
            geometry[str(path)]=digest(path)
        else:copy(path,dest/folder.name/path.relative_to(folder))
for name,sha in frozen.items():
    path=Path(name)
    if path.suffix in ('.py','.cpp','.h','.hpp','.msg','.yaml','.sh','.txt'):
        copy(path,dest/'source'/path.relative_to(root))
    else:binaries[name]=sha
for path in (root/'ellipselio/tests').glob('*'):
    if path.is_file():copy(path,dest/'source'/path.relative_to(root))
for name in ['scripts/recent_submaps/AREA-MAPS.md','research/RESULTS-AREA-ODOMETRY.md']:
    path=root/'FAST-LIVO2-ROS2'/name;copy(path,dest/'source'/path.relative_to(root))
(dest/'external-geometry.json').write_text(json.dumps(geometry,indent=2)+'\n')
(dest/'external-binaries.json').write_text(json.dumps(binaries,indent=2)+'\n')
files={str(p.relative_to(dest)):digest(p) for p in sorted(dest.rglob('*')) if p.is_file()}
(dest/'manifest.json').write_text(json.dumps(dict(files=files,files_verified=len(files),external_geometry_files_verified=len(geometry),
    source_root=str(base),ground_truth_used_by_frontend=False,ground_truth_used_only_for_full_evaluation=True,
    full_sequence_robots=['aerial08'],frozen_sources_verified=True),indent=2)+'\n')
assert all(digest(dest/p)==h for p,h in files.items())
print('Retained and verified',len(files),'files;',len(geometry),'external geometry payloads')
