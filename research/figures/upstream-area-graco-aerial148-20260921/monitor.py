import json
from pathlib import Path
import numpy as np
B=Path('/workspace/.ros2/upstream-area-graco-aerial148-20260921');W=B/'full'
state=json.loads((W/'status.json').read_text());result=dict(phase=state['phase'],robots={})
for robot,status in state['robots'].items():
 t=W/'frontends'/robot;p=t/'frontend/native_updates.jsonl';row=dict(status=status['status'])
 procs=t/'frontend/processes.json'
 if procs.exists() and not (t/'loaded-library.txt').exists():
  pid=json.loads(procs.read_text())['mapper'];maps=Path(f'/proc/{pid}/maps')
  if maps.exists():
   hits=[s for s in maps.read_text().splitlines() if 'libellipselio_mapping.so' in s]
   if hits:
    assert all('upstream-area-s3e-20260921/capacity-run/install' in s for s in hits),hits
    (t/'loaded-library.txt').write_text('\n'.join(hits)+'\n')
 if p.exists():
  rows=[json.loads(l) for l in p.read_text().splitlines() if l.endswith('}')]
  if rows:
   stamps=np.array([x['sensor_stamp_ns'] for x in rows],dtype=np.int64);success=stamps[[x['lidar_updated'] for x in rows]]
   last=rows[-1];row.update(poses=len(rows),sensor_elapsed_s=(int(stamps[-1])-int(stamps[0]))/1e9,
     points=last['active_points'],features=last['features'],last_update_success=last['lidar_updated'],
     failed_after_init=sum(not x['lidar_updated'] for x in rows[1:]),
     max_update_gap_s=float(np.diff(np.r_[stamps[0],success,stamps[-1]]).max(initial=0)/1e9),
     pose_finite=bool(np.isfinite([x['pose'] for x in rows]).all()))
 result['robots'][robot]=row
(B/'progress.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
