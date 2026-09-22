import json
from pathlib import Path
import numpy as np
B=Path(__file__).resolve().parent;W=B/'source/output'
for r in ('ground06','aerial06'):
    trial=W/'frontends'/r;p=trial/'frontend/native_updates.jsonl'
    if not p.exists():continue
    rows=[json.loads(s) for s in p.read_text().splitlines() if s.endswith('}')]
    if len(rows)<2:continue
    stamps=np.array([v['stamp_ns'] for v in rows],dtype=np.int64)
    poses=np.array([v['pose'][:3] for v in rows])
    successes=np.array([v['sensor_stamp_ns'] for v in rows if v['lidar_updated']],dtype=np.int64)
    print(json.dumps(dict(robot=r,poses=len(rows),elapsed_sensor_s=(stamps[-1]-stamps[0])/1e9,
        failed_updates_after_init=sum(not v['lidar_updated'] for v in rows[1:]),
        max_speed_m_s=float(np.max(np.linalg.norm(np.diff(poses,axis=0),axis=1)/(np.diff(stamps)/1e9))),
        max_success_gap_s=float(np.max(np.diff(successes))/1e9) if len(successes)>1 else None,
        latest_features=rows[-1]['features'],processing_mean_ms=float(np.mean([v['processing_s'] for v in rows])*1000))))
