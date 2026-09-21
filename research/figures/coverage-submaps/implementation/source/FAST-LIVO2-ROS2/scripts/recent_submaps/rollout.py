#!/usr/bin/env python3
"""Run the authorized three-robot backend after a confirmed Bob frontend gate."""
import argparse
import json
from pathlib import Path
import sys
import yaml
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'research'))
from s3e_pipeline.recent_submaps import prepare
from s3e_pipeline.dpgo import run

root=Path('/workspace')
p=argparse.ArgumentParser();p.add_argument('--work',type=Path,required=True)
p.add_argument('--bob',type=Path,required=True);p.add_argument('--alpha',type=Path,required=True);p.add_argument('--carol',type=Path,required=True)
p.add_argument('--gate',type=Path,required=True);p.add_argument('--repeat-gate',type=Path,required=True)
args=p.parse_args()
if not all(json.loads(path.read_text())['gate_pass'] for path in (args.gate,args.repeat_gate)):
    raise ValueError('Bob gate and reproducibility confirmation are required')
args.work.mkdir(parents=True,exist_ok=False)
cfg=yaml.safe_load((Path(__file__).parent/'rollout.yaml').read_text())
(args.work/'config.yaml').write_text(yaml.safe_dump(cfg,sort_keys=False))
artifacts={}
for robot,trial in [('Alpha',args.alpha),('Bob',args.bob),('Carol',args.carol)]:
    summary=json.loads((trial/'frontend/summary.json').read_text())
    if not summary['success'] or summary['requested_duration']!=0:raise ValueError('Incomplete frontend')
    out=args.work/f'prepared-{robot}'
    prepare(trial/'frontend/submaps',out,cfg)
    artifacts[f'keyframes.ellipselio.{robot}']=str(out.resolve())
    artifacts[f'descriptors.ellipselio.mapclosures.{robot}']=str((out/'ellipsoid').resolve())
(args.work/'dpgo').mkdir()
run(cfg,artifacts,root,args.work/'dpgo')
artifacts['dpgo']=str((args.work/'dpgo').resolve())
(args.work/'artifacts.json').write_text(json.dumps(artifacts,indent=2)+'\n')
