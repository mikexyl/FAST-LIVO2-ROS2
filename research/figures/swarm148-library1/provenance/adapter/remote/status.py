#!/usr/bin/env python3
"""Read current remote progress without ROS dependencies or process mutation."""
import json
from pathlib import Path

root=Path(__file__).resolve().parents[3]
base=root/'.ros2/swarm148'
def read(path):return json.loads(path.read_text()) if path.exists() else None
controller=read(base/'progress.json')
result={'controller':{k:controller.get(k) for k in ('stage','sequence','robot')} if controller else None}
if controller and controller.get('sequence'):
    work=base/controller['sequence']
    result['frontend']={robot:read(work/f'frontend/{robot}/summary.json') or
                       read(work/f'frontend/{robot}/progress.json') for robot in ('Alpha','Bob','Carol')}
    result['frontend']={robot:{k:v.get(k) for k in ('success','exported_frames','elapsed_bag_s','error')}
                       if v else None for robot,v in result['frontend'].items()}
    result['native']=read(work/'swarm/summary.json') or read(work/'swarm/progress.json')
    events=work/'swarm/events.jsonl'
    if events.exists():
        attempts=accepted=inter=0
        for line in events.read_text().splitlines():
            try:event=json.loads(line)
            except json.JSONDecodeError:continue
            if event['type']!='verification':continue
            attempts+=1
            if event['success']:
                accepted+=1;inter+=event['robot_id'] is None
        result['native_registrations']={'attempts':attempts,'accepted':accepted,'accepted_inter':inter}
print(json.dumps(result,indent=2))
