import json, math, re, sys
from pathlib import Path
import numpy as np
root=Path(sys.argv[1]);p=root/'frontend/native_updates.jsonl'
lines=p.read_text().splitlines();rows=[]
for line in lines:
    try:rows.append(json.loads(re.sub(r'(?<![A-Za-z])[-+]?nan','NaN',line,flags=re.I)))
    except json.JSONDecodeError:pass
stamp=np.array([r['stamp_ns'] for r in rows],dtype=np.int64)
sensor=np.array([r['sensor_stamp_ns'] for r in rows],dtype=np.int64)
pose=np.array([r['pose'] for r in rows]);speed=np.linalg.norm(np.diff(pose[:,:3],axis=0),axis=1)/(np.diff(stamp)*1e-9)
ok=np.array([r['lidar_updated'] and r['num_feats']>0 and math.isfinite(r['residual']) for r in rows])
gaps=np.diff(np.r_[sensor[0],sensor[ok],sensor[-1]])*1e-9
print(json.dumps(dict(poses=len(rows),elapsed_s=(int(sensor[-1])-int(sensor[0]))/1e9,max_speed_m_s=float(speed.max()),max_update_gap_s=float(gaps.max()),failures_after_init=int((~ok[1:]).sum()),map_points=rows[-1]['map_size'],processing_mean_ms=float(np.mean([r['processing_s'] for r in rows])*1000),last_pose=rows[-1]['pose'][:3]),indent=2))
