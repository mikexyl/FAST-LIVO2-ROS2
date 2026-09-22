"""Stage evaluation-only references after the recorded backend completion."""
from pathlib import Path
import hashlib,json,shutil,time
B=Path(__file__).resolve().parent
state=json.loads((B/'backend-complete.json').read_text())
assert state['phase']=='backend_complete'
dest=B/'reference';dest.mkdir(exist_ok=False)
names={f'aerial{i:02}':f'aerial-{i:02}-{h}m.txt' for i,h in enumerate([40,20,20,40,40,20,25,25],1)}
names.update({f'ground{i:02}':f'ground-{i:02}.txt' for i in range(1,7)})
out=dict(ground_truth_for_estimation=False,evaluation_only=True,
    backend_completed_utc=state['updated_utc'],copied_utc=time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime()),files={})
for robot,name in names.items():
    source=Path('/data/graco')/name;target=dest/f'{robot}_gt.txt'
    shutil.copy2(source,target)
    value=hashlib.sha256(source.read_bytes()).hexdigest()
    assert hashlib.sha256(target.read_bytes()).hexdigest()==value
    out['files'][robot]=dict(source=str(source),sha256=value)
(dest/'source-hashes.json').write_text(json.dumps(out,indent=2)+'\n')
print('Staged 14 references for evaluation only')
