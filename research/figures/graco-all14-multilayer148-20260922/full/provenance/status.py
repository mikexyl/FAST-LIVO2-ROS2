from pathlib import Path
import json
B=Path(__file__).resolve().parent;W=B/'full'
for n in ['frontend-status.json','extra-frontend-status.json']:
 if (W/n).exists():
  d=json.loads((W/n).read_text());print(n,d['phase'],{r:s['status'] for r,s in d['robots'].items()})
  for r,s in d['robots'].items():
   if s['status']=='failed':print(r,s.get('error'))
for p in sorted(W.glob('prepared-*/summary.json')):
 t=p.parent/'timings.jsonl';rows=[json.loads(l) for l in t.read_text().splitlines()];print(p.parent.name,len(rows),'terrain_unavailable',sum(not x['multilayer']['terrain']['available'] for x in rows),'features',sum(sum(x['multilayer']['feature_counts'].values()) for x in rows))
for n in ['status.json','dpgo/progress.json']:
 p=W/n
 if p.exists():
  d=json.loads(p.read_text())
  if n=='dpgo/progress.json':print(n,round(d['wall_s'],1),{r:(v['observed'],v['total'],v['loops']) for r,v in d['robots'].items()})
  else:print(n,d['phase'],d.get('error',''))
