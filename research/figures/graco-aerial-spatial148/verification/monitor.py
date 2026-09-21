from pathlib import Path
import json,math
B=Path('/data3/mikexyl/swarm_s3e_ws/src/.ros2/graco-aerial-spatial148-20260920')
state=json.loads((B/'full/status.json').read_text());print('Phase:',state['phase'])
for robot,info in state['robots'].items():
    p=B/'full/frontends'/robot/'frontend/native_updates.jsonl'
    if not p.exists():print(robot,info);continue
    lines=p.read_text().splitlines();rows=[]
    for l in lines:
        try:rows.append(json.loads(l))
        except json.JSONDecodeError:pass
    good=[r for r in rows if r['lidar_updated']]
    if not rows:continue
    gap=max(((b['stamp_ns']-a['stamp_ns'])/1e9 for a,b in zip(good,good[1:])),default=0)
    dt=(rows[-1]['sensor_stamp_ns']-rows[0]['sensor_stamp_ns'])/1e9
    speed=max((math.dist(a['pose'][:3],b['pose'][:3])/((b['stamp_ns']-a['stamp_ns'])/1e9) for a,b in zip(rows,rows[1:]) if b['stamp_ns']>a['stamp_ns']),default=0)
    print(robot,info['status'],dict(sensor_elapsed_s=round(dt,1),poses=len(rows),handovers=rows[-1]['handovers'],successful_gap_s=round(gap,3),max_speed_m_s=round(speed,3),last_update=rows[-1]['lidar_updated']))
if state['phase']=='failed':print((B/'full/failure.json').read_text())
