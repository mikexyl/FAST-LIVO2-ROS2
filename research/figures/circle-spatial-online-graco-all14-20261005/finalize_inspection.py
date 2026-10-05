from pathlib import Path
import importlib.util,json,shutil,csv
root=Path('/workspace/.ros2/circle-spatial-online148-20261005')
assert (root/'full.exit').read_text().strip()=='0'
assert (root/'inspection.exit').read_text().strip()=='0'
out=root/'inspection'; provenance=out/'provenance';provenance.mkdir(exist_ok=True)
for name in ['source-hashes.json','inputs.json','frontend-binary-hashes.json','ONLINE_DONE','START','native-health.json','frontend-exits.json','config.yaml','gpu.csv']:
 shutil.copy2(root/'full'/name,provenance/name)
for name in ['preflight.json','frontend-source-hashes.json','run.sh','inspect.sh']:
 shutil.copy2(root/name,provenance/name)
shutil.copy2('/workspace/.ros2/spatial-only148-20261005/verification.json',provenance/'build-verification.json')
spec=importlib.util.spec_from_file_location('finish_spatial',root/'finish_spatial_online_graco.py');m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
robot='aerial03'
keys=[json.loads(s) for s in (root/f'full/prepared-{robot}/store/keyframes.jsonl').read_text().splitlines() if s]
local=json.loads((root/f'full/epochs/persistent/{robot}/local-graph.json').read_text())
m.topology_preview(root/'full',out,robot,keys,local)
shutil.copy2(root/'finish_spatial_online_graco.py',out/'spatial-auditor.py')
audit=json.loads((out/'audit.json').read_text());audit['alignment_complete']=False;audit['success_scope']='Independent data and runtime audit; full alignment is reported separately.'
(out/'audit.json').write_text(json.dumps(audit,sort_keys=True)+'\n')
rows=list(csv.DictReader((out/'cbs-updates.csv').open()))
for row in rows:
 for k in ('progress_s','update_ms','active_callback_ms'):row[k]=float(row[k])
m.timing_plot(rows,audit,out)
shutil.copy2(__file__,out/'finalize_inspection.py')
print('Final graph colors and capture provenance saved')
