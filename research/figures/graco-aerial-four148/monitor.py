import json
import math
from pathlib import Path

base = Path(__file__).resolve().parent
work = base/'full'
state = json.loads((work/'status.json').read_text())
print('phase:', state['phase'], 'updated:', state['updated_utc'])
for robot, info in state['robots'].items():
    path = work/'frontends'/robot/'frontend/native_updates.jsonl'
    if not path.exists():
        print(robot, info)
        continue
    data = [json.loads(line) for line in path.read_text().splitlines() if line.endswith('}')]
    if not data:
        print(robot, 'initializing')
        continue
    sensor = json.loads((base/'inputs'/robot/'SENSORS.json').read_text())
    start = sensor['first_stamp_ns']['/velodyne/points']
    last = sensor['last_stamp_ns']['/velodyne/points']
    current = data[-1]['sensor_stamp_ns']
    good = [r['sensor_stamp_ns'] for r in data if r['lidar_updated']]
    gap = max((b-a for a,b in zip(good,good[1:])), default=0)/1e9
    finite = all(math.isfinite(x) for r in data for x in r['pose'])
    speed = max((math.dist(a['pose'][:3], b['pose'][:3])*1e9/(b['stamp_ns']-a['stamp_ns'])
                 for a,b in zip(data, data[1:])), default=0)
    print(robot, info['status'], f'{(current-start)/1e9:.1f}/{(last-start)/1e9:.1f} s',
          f'poses={len(data)}', f'max successful-update gap={gap:.3f}s',
          f'finite={finite}', f'max speed={speed:.2f}m/s')
    if 'error' in info:
        print(info['error'])
if (work/'failure.json').exists():
    print((work/'failure.json').read_text())
