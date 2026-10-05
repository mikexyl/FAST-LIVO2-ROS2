#!/usr/bin/env python3
"""Serial fresh-process circle40 / trajectory40 / trajectory80 controls.

Run in the existing research environment with the compatible native adapter on
PYTHONPATH. The driver can be detached on 148; its status/logs survive reconnects.
"""
import argparse
import json
from pathlib import Path
import subprocess
import sys
import time
import yaml


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--config',required=True)
    parser.add_argument('--source',required=True);parser.add_argument('--output',required=True)
    parser.add_argument('--controls-first',action='store_true',help='Run trajectory controls before the circle repeat')
    parser.add_argument('--prepared-root',help='Reuse completed immutable preparations from another campaign')
    args=parser.parse_args();output=Path(args.output);output.mkdir(parents=True,exist_ok=True)
    status_path=output/'campaign-status.json'
    if status_path.exists():raise FileExistsError('Campaign status exists; inspect before choosing fresh outputs')
    cfg=yaml.safe_load(Path(args.config).read_text());status=dict(started=time.time(),runs={})
    def save():
        temporary=status_path.with_suffix('.partial');temporary.write_text(json.dumps(status,indent=2));temporary.replace(status_path)
    completed=[]
    jobs=[('circle40-final','circle',40.),('trajectory40-final','trajectory',40.),('trajectory80-final','trajectory',80.)]
    if args.controls_first:jobs=jobs[1:]+jobs[:1]
    for label,mode,radius in jobs:
        current=dict(cfg);current['circle_cover']=dict(cfg['circle_cover'],radius_m=radius)
        config=output/(label+'.yaml');config.write_text(yaml.safe_dump(current,sort_keys=False))
        status['runs'][label]=dict(phase='running',started=time.time(),configuration=str(config));save()
        with (output/(label+'.log')).open('w') as log:
            command=[sys.executable,'-m','s3e_pipeline.central_circle','--config',str(config),
                '--source',args.source,'--output',str(output/label),'--mode',mode]
            if args.prepared_root:command+=['--prepared',str(Path(args.prepared_root)/label)]
            result=subprocess.run(command,stdout=log,stderr=subprocess.STDOUT)
        status['runs'][label].update(phase='completed' if result.returncode==0 else 'failed',
                                    exit_code=result.returncode,finished=time.time());save()
        if result.returncode==0:completed.append(label)
    if completed:
        with (output/'report.log').open('w') as log:
            result=subprocess.run([sys.executable,'-m','s3e_pipeline.circle_central_evaluation',
                '--config',args.config,'--output',str(output/'inspection'),
                *[arg for label in completed for arg in ('--run',label+'='+str(output/label))]],
                stdout=log,stderr=subprocess.STDOUT)
        status['report_exit_code']=result.returncode
    status['finished']=time.time();status['phase']='completed' if len(completed)==3 and status.get('report_exit_code')==0 else 'partial_or_failed';save()
    return 0 if status['phase']=='completed' else 1


if __name__=='__main__':sys.exit(main())
