#!/usr/bin/env python3
"""Read-only progress and sensor-only diagnostics for the failed-case queue."""
import json
from pathlib import Path
import numpy as np

root = Path('/workspace/.ros2/recent-submaps/failed-frontends-20260918')
state = json.loads((root / 'status.json').read_text())
cases = {r['name']: r for r in json.loads((root / 'cases.json').read_text())}
for item in state['cases']:
    result = {k: item[k] for k in ('name', 'phase', 'passed', 'error') if k in item}
    trial = root / item['name']
    path = trial / 'frontend/native_updates.jsonl'
    if path.exists():
        data = path.read_bytes()
        rows = [json.loads(line) for line in data[:data.rfind(b'\n')].splitlines()]
        if len(rows) > 1:
            xyz = np.array([r['pose'][:3] for r in rows])
            stamps = np.array([r['stamp_ns'] for r in rows], dtype=np.int64)
            sensor = np.array([r['sensor_stamp_ns'] for r in rows], dtype=np.int64)
            successful = sensor[[r['lidar_updated'] for r in rows]]
            result.update(sensor_elapsed_s=(sensor[-1]-cases[item['name']]['start_ns'])/1e9,
                          duration_s=cases[item['name']]['bag_duration_s'], poses=len(rows),
                          max_speed_m_s=float(np.max(np.linalg.norm(np.diff(xyz, axis=0), axis=1)/(np.diff(stamps)/1e9))),
                          max_success_gap_s=float(np.max(np.diff(np.r_[sensor[0], successful, sensor[-1]])) / 1e9),
                          last_features=rows[-1]['features'], handovers=rows[-1]['active_submap_id'])
    if 'metrics' in item:
        result.update({k: item['metrics'].get(k) for k in ('ate_rmse_m', 'gate_pass', 'full_completion')})
    print(json.dumps(result))
print(json.dumps(dict(finished_utc=state.get('finished_utc'))))
