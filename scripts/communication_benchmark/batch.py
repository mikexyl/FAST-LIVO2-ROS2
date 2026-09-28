#!/usr/bin/env python3
"""One sequential lane per native method; lanes may run concurrently on 148."""
import argparse,json,subprocess,sys,time
from pathlib import Path
from launch import ROOT,save
GROUPS=['GRACO_ground','GRACO_aerial','GRACO_mixed']+[
 'S3E_'+name+'_'+str(i) for name,n in [('Laboratory',3),('Library',2),('Playground',3),('Square',3),('Tunnel',1)] for i in range(1,n+1)]
def main():
 p=argparse.ArgumentParser();p.add_argument('--method',required=True);p.add_argument('--attempt',default='full-v1');p.add_argument('--groups',nargs='+',default=GROUPS);a=p.parse_args()
 ledger=ROOT/'status'/('lane-'+a.method+'-'+a.attempt+'.json')
 state=dict(method=a.method,groups=a.groups,completed=[],started=time.time(),phase='running')
 save(ledger,state)
 for group in a.groups:
  status=ROOT/'status'/(a.method+'-'+group+'-'+a.attempt+'.json')
  if status.exists():
   previous=json.loads(status.read_text())
   if previous['phase'] not in ['finished','excluded','setup_failed']:raise RuntimeError('Existing active job '+str(status))
  else:
   state['active']=group;save(ledger,state)
   subprocess.run([sys.executable,str(Path(__file__).with_name('launch.py')),'--method',a.method,'--group',group,'--attempt',a.attempt],check=True)
  state['completed'].append(group);save(ledger,state)
  measured=json.loads(status.read_text())
  if measured['phase']=='setup_failed' or 'AttributeError' in str(measured.get('native_error')) or 'ModuleNotFoundError' in str(measured.get('native_error')):
   state.update(phase='instrumentation_or_setup_failure',failure=measured);save(ledger,state);raise RuntimeError('Stop lane after setup failure')
 state.update(phase='finished',finished=time.time(),active=None);save(ledger,state)
if __name__=='__main__':main()
