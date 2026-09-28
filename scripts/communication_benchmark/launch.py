#!/usr/bin/env python3
"""Fresh, isolated native benchmark jobs; reuse immutable bags and calibrated runners."""
import argparse,json,os,shlex,subprocess,time,shutil,hashlib
from pathlib import Path
H=Path('/data3/mikexyl/swarm_s3e_ws/src')
ROOT=H/'.ros2/communication-benchmark148-20260928'
BASE=H/'.ros2/graco-baselines148-20260922'
CAL=H/'.ros2/calibration-corrected148-20260926'
S3E=H/'.ros2/s3e-native-baselines148-20260923'
GROUPS={'GRACO_ground':['ground01','ground02','ground03'],'GRACO_aerial':['aerial05','aerial07','aerial08'],'GRACO_mixed':['ground05','ground06','aerial07']}
def save(p,v):
 p.parent.mkdir(parents=True,exist_ok=True);t=p.with_suffix('.tmp');t.write_text(json.dumps(v,indent=2)+'\n');t.replace(p)
def main():
 ap=argparse.ArgumentParser();ap.add_argument('--method',required=True,choices=['dcl','disco','swarm','gac']);ap.add_argument('--group',required=True);ap.add_argument('--duration',type=float,default=0);ap.add_argument('--tail',type=float,default=120);ap.add_argument('--attempt',default='full');a=ap.parse_args()
 graco=a.group.startswith('GRACO');robots=GROUPS[a.group] if graco else ['Alpha','Bob','Carol']
 tag=a.method+'-'+a.group+'-'+a.attempt;out=ROOT/'runs'/tag
 assert not out.exists(),out
 state=dict(method=a.method,group=a.group,robots=robots,attempt=a.attempt,phase='starting',start_unix=time.time(),resource_quotas=False,output=str(out),duration_s=a.duration,tail_s=a.tail)
 status=ROOT/'status'/(tag+'.json');assert not status.exists(),status
 name='communication-'+tag.lower().replace('_','-')+'-0928';state['container']=name
 if a.method=='gac' and graco and a.group!='GRACO_ground':
  state.update(phase='excluded',reason='Native GAC requires camera data absent from GRACO aerial sequences; no method substitution.');save(status,state);return
 version=ROOT/'code-versions'/tag;version.mkdir(parents=True)
 for f in (ROOT/'code').iterdir():
  if f.is_file():shutil.copy2(f,version/f.name)
 state['instrumentation_sha256']={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in version.iterdir() if p.is_file()}
 template_name=('graco-calibrated-swarm-ground-20260926' if graco else 's3e-native-swarm-20260923') if a.method=='swarm' else ('graco-calibrated-dcl-ground-20260926-r2' if graco else 's3e-native-dcl-20260923')
 template=json.loads(subprocess.check_output(['docker','inspect',template_name]))[0]
 create=['docker','run','-d','--name',name,'--entrypoint','/bin/sleep']
 if a.method=='swarm':create+=['--ipc','host']
 mounts={m['Destination']:m for m in template['Mounts']}
 for m in mounts.values():
  spec='type=bind,src='+m['Source']+',dst='+m['Destination']
  # Frozen baseline artifacts are read only. Native GAC caches may require /baseline writes.
  if not m['RW'] or (m['Destination']=='/experiment'):spec+=',readonly'
  create+=['--mount',spec]
 create+=['--mount','type=bind,src='+str(ROOT)+',dst=/comm',template['Image'],'infinity']
 save(status,state)
 try:
  subprocess.run(create,check=True,stdout=subprocess.DEVNULL)
  state['image']=template['Image'];state['mounts']=create
  setup=['set -eo pipefail']
  if a.method=='swarm':
   setup+=['source /opt/ros/humble/setup.bash','source /ros2_ws/install/setup.bash','export PYTHONPATH=/teaser:${PYTHONPATH:-}','export LD_LIBRARY_PATH=/usr/local/lib:${LD_LIBRARY_PATH:-}:/native/lib','export ROS_LOCALHOST_ONLY=1','export ROS_DOMAIN_ID=181','export OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1','export FASTRTPS_DEFAULT_PROFILES_FILE=/baseline/code/fastdds.xml']
  else:
   setup+=['source /opt/ros/melodic/setup.bash','source /baseline/'+{'dcl':'dcl_melodic_ws','disco':'disco_ws','gac':'gac_ws'}[a.method]+'/devel/setup.bash']
   if a.method=='disco':setup+=['export LD_LIBRARY_PATH=/baseline/install/gtsam/lib:${LD_LIBRARY_PATH:-}']
  runner='/comm/runners/'+('graco' if graco else 's3e')+'/run_'+a.method+'.py'
  command=['python3','/comm/code-versions/'+tag+'/bootstrap.py','--method',a.method,'--runner',runner,'--output','/comm/runs/'+tag,'--robots']+robots+['--duration',str(a.duration)]
  if a.method!='gac':command+=['--tail',str(a.tail)]
  if a.method!='swarm':command+=['--port','19731']
  if graco and a.method!='gac':command+=['--calibration-dir','/experiment/calibration']
  if not graco:command+=['--group',a.group]
  if a.method=='dcl':
   command+=['--binary-root','/baseline/dcl_melodic_ws/devel/lib/dcl_lio_sam']
   setup+=['mkdir -p /root/log /comm/runs/'+tag+'-pcm','ln -s /comm/runs/'+tag+'-pcm /root/dcl_output']
  if a.method=='gac':
   setup+=['mkdir -p /comm/runs/'+tag+'-native/data/testSavemap /comm/runs/'+tag+'-native/cache','ln -s /comm/runs/'+tag+'-native /root/gacm_output']
  full=['docker','exec',name,'bash','-lc','\n'.join(setup)+'\nexec '+shlex.join(command)]
  state.update(phase='running',command=full);save(status,state)
  (ROOT/'logs').mkdir(exist_ok=True)
  if graco:duration=max({'ground01':335,'ground02':388,'ground03':298,'ground05':521,'ground06':312,'aerial05':298,'aerial07':396,'aerial08':280}[r] for r in robots)
  else:duration=next(x['duration_s'] for x in json.loads((S3E/'manifest.json').read_text()) if x['group']==a.group)
  # Stop a stuck native run, not a resource cap. GAC retains its prior one-hour finishing allowance.
  deadline=(a.duration or duration)*(3 if a.method=='gac' else 1)+(3600 if a.method=='gac' else 360)
  with (ROOT/'logs'/(tag+'.log')).open('x') as f:
   p=subprocess.Popen(full,stdout=f,stderr=subprocess.STDOUT);state['pid']=p.pid;save(status,state)
   try:code=p.wait(timeout=deadline)
   except subprocess.TimeoutExpired:
    state['watchdog']='Full sequence plus declared native finishing allowance exceeded';subprocess.run(['docker','stop','-t','10',name],stdout=subprocess.DEVNULL);code=p.wait();state['timed_out']=True
  state.update(phase='finished',exit_code=code,elapsed_s=time.time()-state['start_unix'])
  if (out/'summary.json').exists():
   native=json.loads((out/'summary.json').read_text());state['native_phase']=native.get('phase');state['native_error']=native.get('error')
  if (out/'divergence.json').exists():state['verdict']='stopped_entire_group_on_odometry_divergence'
 except Exception as e:state.update(phase='setup_failed',error=repr(e))
 finally:
  subprocess.run(['docker','stop','-t','15',name],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
  save(status,state)
 print(tag,state['phase'],state.get('exit_code'),flush=True)
if __name__=='__main__':main()
