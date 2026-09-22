"""Make the new run's prepared-data links work on both host and container."""
from pathlib import Path
import json,os
B=Path('/workspace/.ros2/graco-aerial-height148-20260921');W=B/'full'
assert json.loads((W/'status.json').read_text())['phase']=='complete'
changed={}
for p in sorted(W.glob('prepared-*')):
    assert p.is_symlink()
    target=p.resolve(strict=True);old=os.readlink(p)
    relative=os.path.relpath(target,p.parent)
    tmp=p.with_name(p.name+'.relative')
    tmp.symlink_to(relative,target_is_directory=True);tmp.replace(p)
    assert p.resolve(strict=True)==target
    changed[p.name]=dict(before=old,after=relative,payload_changed=False)
(B/'portable-links.json').write_text(json.dumps(changed,indent=2)+'\n')
print(json.dumps(changed))
