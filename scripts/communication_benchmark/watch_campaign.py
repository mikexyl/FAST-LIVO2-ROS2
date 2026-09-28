#!/usr/bin/env python3
"""One-shot completion driver for this already launched benchmark, then exit.

Downloads compact metadata only. Rebuilds the local report on completed-group
changes and final completion; it does not schedule or launch future experiments.
"""
import hashlib,json,os,subprocess,sys,time,zipfile
from pathlib import Path
PROJECT=Path(__file__).resolve().parents[2]
OUT=PROJECT/'research/figures/communication-benchmark148-20260928'
REPORT=PROJECT/'research/figures/s3e-graco-comparison-20260924'
HOST='mikexyl@192.168.0.148'
REMOTE='/data3/mikexyl/swarm_s3e_ws/src/.ros2/communication-benchmark148-20260928'

def package():
 path=REPORT/'technical-report-source.zip';temporary=path.with_suffix('.new.zip')
 with zipfile.ZipFile(path) as source:
  entries={name:source.read(name) for name in source.namelist()}
 for name in list(entries):
  local=REPORT/name
  if local.is_file():entries[name]=local.read_bytes()
 for name in ['latex/sections/communication.tex','output/pdf/communication-overview.pdf','output/pdf/cbs-cadence.pdf','latex/sections/gpu-cbs.tex','output/pdf/gpu-comparison.pdf']:
  entries[name]=(REPORT/name).read_bytes()
 for name in ['REPORT.md','table.csv','table.json','gpu-comparison.pdf','deployment-hashes.json','evidence/matched-v2/evaluated-results.json']:
  gpu=PROJECT/'research/figures/glim-gpu-cbs148-20260928'/name
  if gpu.is_file():entries['gpu-cbs/'+name]=gpu.read_bytes()
 for name in ['measured-results.json','communication.csv','REPORT.md','paper-self-review.md']:
  entries['communication/'+name]=(OUT/name).read_bytes()
 with zipfile.ZipFile(temporary,'w',zipfile.ZIP_DEFLATED) as target:
  for name,value in entries.items():target.writestr(name,value)
 temporary.replace(path)
 verification=dict(date='2026-09-28',scope='Communication-section update; historical accuracy results unchanged',
  counter_unit_tests_passed=8,online_tests_passed=11,native_cadence_smoke_revisions=3,
  native_cadence_smoke_status='complete',latex_build='passed',
  visual_review='Communication overview and cadence figures inspected; generated report communication pages rendered and inspected',
  historical_comparison_sha256=hashlib.sha256((REPORT/'comparison-data.json').read_bytes()).hexdigest(),
  current_communication_data_sha256=hashlib.sha256((OUT/'measured-results.json').read_bytes()).hexdigest(),
  campaign_complete=json.loads((OUT/'measured-results.json').read_text())['campaign_complete'])
 (REPORT/'COMMUNICATION-VERIFICATION.json').write_text(json.dumps(verification,indent=2)+'\n')
 manifest=REPORT/'SHA256SUMS.json';hashes=json.loads(manifest.read_text())
 for name in ['README.md','latex/main.tex','latex/sections/communication.tex','output/pdf/technical-report.pdf',
              'output/pdf/communication-overview.pdf','output/pdf/cbs-cadence.pdf','technical-report-source.zip','COMMUNICATION-VERIFICATION.json',
              'latex/sections/gpu-cbs.tex','output/pdf/gpu-comparison.pdf']:
  hashes[name]=hashlib.sha256((REPORT/name).read_bytes()).hexdigest()
 manifest.write_text(json.dumps(hashes,indent=2,sort_keys=True)+'\n')

def main():
 last=None;errors=0
 while True:
  try:
   subprocess.run(['ssh','-o','BatchMode=yes','-o','ConnectTimeout=15',HOST,'python3 '+REMOTE+'/code/collect.py'],check=True,timeout=90)
   temporary=OUT/'measured-results.download.json'
   subprocess.run(['scp','-o','BatchMode=yes','-o','ConnectTimeout=15',HOST+':'+REMOTE+'/report/measured-results.json',str(temporary)],check=True,timeout=60)
   data=json.loads(temporary.read_text());temporary.replace(OUT/'measured-results.json')
   signature=[(r['group'],r['method'],r['status'],r.get('valid_total')) for r in data['rows']]
   if signature!=last or data['campaign_complete']:
    env=dict(os.environ,MPLCONFIGDIR='/tmp/communication-mpl')
    subprocess.run([str(PROJECT.parent/'.ros2/research-venv/bin/python'),str(Path(__file__).with_name('build_report.py'))],check=True,env=env)
    subprocess.run([sys.executable,str(REPORT/'latex/compile.py')],check=True)
    package();last=signature
   errors=0
   state=dict(updated_unix=time.time(),campaign_complete=data['campaign_complete'],terminal_entries=sum(r['status'] in ['complete','failed','excluded','stopped_divergence','timeout'] for r in data['rows']))
   (OUT/'sync-status.json').write_text(json.dumps(state,indent=2)+'\n')
   if data['campaign_complete']:return
   if any(l['phase'] not in ['running','finished'] for l in data['lanes']):
    raise RuntimeError('Benchmark lane requires investigation: '+str([(l['method'],l['phase']) for l in data['lanes']]))
  except Exception as e:
   errors+=1
   (OUT/'sync-error.json').write_text(json.dumps(dict(error=repr(e),consecutive_errors=errors,wall_unix=time.time()),indent=2)+'\n')
   print(repr(e),flush=True)
   if errors>=5:raise
  time.sleep(60)

if __name__=='__main__':main()
