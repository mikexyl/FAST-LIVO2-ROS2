#!/usr/bin/env python3
"""Lightweight host-side status; no ROS or virtual environment needed."""
import argparse,json
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--base',type=Path,default=Path(__file__).resolve().parents[3]/'.ros2/ellipsoid-cbs148/overnight-20260916')
a=p.parse_args();s=json.loads((a.base/'queue.json').read_text())
print(json.dumps({k:v for k,v in s.items() if k not in ('sequences','source_hashes')},indent=2))
for name,item in s['sequences'].items():
    print(name,item['status'],item.get('error',''))
    if item['status']=='running':
        work=Path(item['work'])
        # Stored paths are container paths; translate for host-side inspection.
        if str(work).startswith('/workspace/'):work=Path(__file__).resolve().parents[3]/work.relative_to('/workspace')
        q=work/'progress.json'
        if q.exists():print(' ',q.read_text().strip())
        for q in work.glob('prepare-*-progress.json'):
            value=json.loads(q.read_text());print(' ',q.name,value.get('keyframes'), '/',value.get('scheduled'))
