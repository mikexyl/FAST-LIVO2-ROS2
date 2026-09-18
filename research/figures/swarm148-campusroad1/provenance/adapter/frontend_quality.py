"""Ground-truth-free sanity guard for S3E ground-robot odometry exports."""
import hashlib
import json
from pathlib import Path
import numpy as np


def assess(rows, max_speed_m_s=20.):
    result=dict(passed=False,max_speed_limit_m_s=max_speed_m_s,frames=len(rows),
                purpose='S3E ground-robot sanity check only; not an accuracy metric; no GT used.')
    if len(rows)<2:return dict(result,reason='Fewer than two synchronized poses')
    stamps=np.array([r['stamp_ns'] for r in rows],dtype=np.int64)
    xyz=np.array([np.asarray(r['T_world_body']).reshape(4,4)[:3,3] for r in rows])
    if not np.isfinite(xyz).all():return dict(result,reason='Nonfinite odometry positions')
    dt=np.diff(stamps)/1e9
    if np.any(dt<=0):return dict(result,reason='Non-increasing pose timestamps')
    steps=np.linalg.norm(np.diff(xyz,axis=0),axis=1);speeds=steps/dt
    bad=np.flatnonzero(speeds>max_speed_m_s)
    result.update(path_length_m=float(steps.sum()),max_scan_step_m=float(steps.max()),
                  max_speed_m_s=float(speeds.max()),intervals_over_speed_limit=len(bad))
    if len(bad):
        return dict(result,reason='Implausible ground-robot odometry speed',
                    first_invalid_stamp_ns=int(stamps[bad[0]+1]))
    return dict(result,passed=True,reason='Finite, chronological poses within the declared motion bound')


def check_export(export):
    export=Path(export)
    summary=export.parent/'summary.json'
    if summary.exists() and not json.loads(summary.read_text()).get('success'):
        return dict(passed=False,reason='Frontend replay did not complete successfully')
    path=export/'frames.jsonl';data=path.read_bytes()
    result=assess([json.loads(line) for line in data.splitlines()])
    result['frames_sha256']=hashlib.sha256(data).hexdigest()
    return result
