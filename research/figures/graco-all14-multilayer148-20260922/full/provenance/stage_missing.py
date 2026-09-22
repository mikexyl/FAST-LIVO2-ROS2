"""Validate native sensor conversions for aerial01–03 without accessing references."""
from pathlib import Path
import sys,json,subprocess,traceback,threading
from concurrent.futures import ThreadPoolExecutor
B=Path(__file__).resolve().parent;R=B.parents[1];S=B/'source/FAST-LIVO2-ROS2'
sys.path[:0]=[str(S/'scripts/recent_submaps'),str(S/'research')]
from graco_aerial148 import verify_conversion
from s3e_pipeline.graco import stage_sensors,TOPICS
from upstream_area_batch import save,sha
state={};lock=threading.Lock()
def mark(r,**v):
 with lock:state.setdefault(r,{}).update(v);save(B/'staging-missing.json',state);print(r,v,flush=True)
def one(i):
 r=f'aerial{i:02}';name=f'aerial-{i:02}-'+('40m' if i==1 else '20m');bag=Path('/data3/graco')/(name+'.bag')
 converted=Path('/data3/graco')/(name+'_full_ros2')
 try:
  audit=None
  if (converted/'metadata.yaml').exists():
   mark(r,phase='verifying_existing_conversion')
   try:audit=verify_conversion(bag,converted)
   except Exception as e:mark(r,existing_conversion_rejected=repr(e));audit=None
  if audit is None:
   converted=B/'converted'/r;converted.parent.mkdir(exist_ok=True)
   mark(r,phase='converting_sensors')
   with (B/f'{r}-conversion.log').open('w') as log:
    subprocess.run([str(Path(sys.executable).parent/'rosbags-convert'),'--src',str(bag),'--dst',str(converted),
      '--dst-storage','sqlite3','--dst-version','5','--include-topic',*TOPICS],stdout=log,stderr=subprocess.STDOUT,check=True)
   mark(r,phase='verifying_conversion');audit=verify_conversion(bag,converted)
  mark(r,phase='hashing_source');audit.update(source=str(bag),source_bytes=bag.stat().st_size,source_sha256=sha(bag))
  save(B/f'{r}-conversion-audit.json',audit)
  mark(r,phase='staging_sensors');info=stage_sensors(converted,B/'inputs'/r)
  mark(r,phase='complete',counts=info['counts'],source_sha256=audit['source_sha256'])
 except Exception as e:mark(r,phase='failed',error=repr(e),traceback=traceback.format_exc())
with ThreadPoolExecutor(max_workers=3) as pool:list(pool.map(one,[1,2,3]))
assert all(x['phase']=='complete' for x in state.values())
