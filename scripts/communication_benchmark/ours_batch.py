#!/usr/bin/env python3
"""Run independent online backend replays on separate ROS domains on 148."""
import argparse,concurrent.futures,json,subprocess,time
from pathlib import Path
from batch import GROUPS
from launch import ROOT,save

def lane(groups,index,attempt,source):
 status=ROOT/'status'/('lane-ours-'+str(index)+'-'+attempt+'.json')
 state=dict(method='ours',groups=groups,completed=[],phase='running',started=time.time());save(status,state)
 for group in groups:
  state['active']=group;save(status,state)
  tag='ours-'+group+'-'+attempt;existing=ROOT/'status'/(tag+'.json')
  if existing.exists():raise RuntimeError('Refusing to overwrite '+str(existing))
  command=['docker','exec','-u','0','ellipsoid-gpu148-unlimited-20260926','bash','-lc',
   'source /opt/ros/humble/setup.bash\nsource /workspace/.ros2/cbs-underlay/setup.bash\nsource /workspace/.ros2/dpgo-install/setup.bash\n'
   'export CBS_UNDERLAY=/workspace/.ros2/cbs-underlay\nexport LD_LIBRARY_PATH=/workspace/.ros2/swarm/native/lib:${LD_LIBRARY_PATH:-}\n'
   'export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1\n'
   'exec /workspace/.ros2/research-venv/bin/python '+str(source/'scripts/communication_benchmark/ours.py')
   +' --group '+group+' --attempt '+attempt+' --domain '+str(194+index*2)]
  with (ROOT/'logs'/(tag+'.log')).open('x') as log:
   result=subprocess.run(command,stdout=log,stderr=subprocess.STDOUT)
  state['completed'].append(dict(group=group,exit_code=result.returncode));save(status,state)
 state.update(phase='finished',finished=time.time(),active=None);save(status,state)

if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--lanes',type=int,default=2);p.add_argument('--attempt',default='full-v2')
 p.add_argument('--groups',nargs='+',default=GROUPS)
 p.add_argument('--source',type=Path,default=Path('/workspace/.ros2/communication-benchmark148-20260928/source/FAST-LIVO2-ROS2'))
 a=p.parse_args()
 with concurrent.futures.ThreadPoolExecutor(max_workers=a.lanes) as pool:
  list(pool.map(lambda i:lane(a.groups[i::a.lanes],i,a.attempt,a.source),range(a.lanes)))
