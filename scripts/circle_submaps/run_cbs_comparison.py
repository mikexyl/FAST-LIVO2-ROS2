#!/usr/bin/env python3
"""Serial matched CBS graph controls from frozen, verified GRACO captures."""
import argparse
import copy
import os
from pathlib import Path
import subprocess
import sys
import yaml
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'research'))
from s3e_pipeline.artifacts import read_json, write_json


def main():
    p=argparse.ArgumentParser();p.add_argument('--central-root',type=Path,required=True)
    p.add_argument('--source',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--config',type=Path,required=True);a=p.parse_args()
    a.output.mkdir(parents=True,exist_ok=False);base=yaml.safe_load(a.config.read_text())
    for mode in ('circle40','trajectory40','trajectory80'):
        capture=a.central_root/'matched-final-2'/f'{mode}-final'
        cfg=copy.deepcopy(base);cfg['circle_cover']=read_json(capture/'configuration.json')['circle_cover']
        cfg['experiment_name']=f'{mode}: distributed CBS over frozen spatial submap graph'
        config=a.output/f'{mode}.yaml';config.write_text(yaml.safe_dump(cfg,sort_keys=False))
        print('Starting',mode,flush=True)
        try:
            # Include a fresh coordinator, not just fresh robot processes, so
            # allocator retention cannot contaminate later memory controls.
            env=dict(os.environ,PYTHONPATH=str(Path(__file__).resolve().parents[2]/'research')+
                     os.pathsep+os.environ.get('PYTHONPATH',''))
            subprocess.run([sys.executable,'-m','s3e_pipeline.circle_cbs','--capture',str(capture),
                '--config',str(config),'--source',str(a.source),'--output',str(a.output/mode),
                '--central-registration',str(a.central_root/'singleton-policy-graph'/mode/'central/registration.json')],
                env=env,check=True)
        except BaseException as error:
            write_json(a.output/'FAILED.json',dict(mode=mode,error=str(error)));raise
        print('Completed',mode,flush=True)


if __name__=='__main__': main()
