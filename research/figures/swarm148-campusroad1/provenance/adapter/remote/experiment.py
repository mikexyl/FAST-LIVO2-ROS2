#!/usr/bin/env python3
"""Run bounded native Swarm tests on 148, preserving diagnostics on failure."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import yaml

ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'Swarm-SLAM/s3e'))
from frontend_quality import check_export
ROBOTS=('Alpha','Bob','Carol')
SEQUENCES={'library1':'Library_1','campusroad1':'Campus_Road_1'}


def write(path,value):
    path.parent.mkdir(parents=True,exist_ok=True)
    temporary=path.with_suffix('.tmp')
    temporary.write_text(json.dumps(value,indent=2)+'\n');temporary.replace(path)


def call(cmd,log,domain=124):
    print('RUN',*map(str,cmd),flush=True)
    env=dict(os.environ,ROS_DOMAIN_ID=str(domain),OMP_NUM_THREADS='4',OPENBLAS_NUM_THREADS='1',
             PYTHONPATH=str(ROOT/'FAST-LIVO2-ROS2/research'),MPLCONFIGDIR=str(ROOT/'.ros2/matplotlib'))
    with log.open('w') as stream:
        result=subprocess.run(list(map(str,cmd)),cwd=ROOT,env=env,stdout=stream,stderr=subprocess.STDOUT)
    print('EXIT',result.returncode,log,flush=True)
    return result.returncode


def frontend(work,sequence,robot,rate,duration=0.):
    out=work/'frontend'/robot
    def valid():
        quality=check_export(out/'export');write(out/'quality.json',quality)
        if not quality['passed']:raise RuntimeError(f'{robot} frontend quality failure: {quality["reason"]}; max speed {quality.get("max_speed_m_s")} m/s')
        return True
    if out.exists():
        if (out/'summary.json').exists() and json.loads((out/'summary.json').read_text())['success']:
            return valid()
        raise RuntimeError(f'Inspect incomplete frontend before restarting: {out}')
    command=['bash','FAST-LIVO2-ROS2/scripts/run_ellipselio.sh','--robot',robot,
             '--bag',f'/data/s3e/S3E_{sequence}','--output',out,
             '--mapping-config',ROOT/f'.ros2/ellipse-configs/{robot}.yaml','--rate',rate]
    if duration:command+=['--duration',duration]
    return call(command,work/f'{robot}-frontend.log',domain=185)==0 and valid()


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--sequences',nargs='+',choices=list(SEQUENCES),default=list(SEQUENCES))
    p.add_argument('--smoke-only',action='store_true')
    p.add_argument('--rate',type=float,default=1.)
    p.add_argument('--skip-smoke',action='store_true')
    args=p.parse_args();os.chdir(ROOT)
    base=ROOT/'.ros2/swarm148';base.mkdir(parents=True,exist_ok=True)
    progress=dict(started_utc=time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime()),host='148',sequences={})
    def update(**kwargs):
        progress.update(kwargs);write(base/'progress.json',progress)
    if not args.skip_smoke:
        work=base/'smoke';work.mkdir(exist_ok=True)
        update(stage='frontend_smoke')
        if not frontend(work,'Library_1','Alpha',1.,60.):
            update(stage='failed',reason='EllipseLIO smoke failed');raise SystemExit(1)
        update(stage='swarm_smoke')
        if not (work/'swarm/summary.json').exists():
            call(['bash','Swarm-SLAM/s3e/run.sh','--work',work,'--output',work/'swarm',
                  '--duration',45.,'--smoke-duplicate-alpha','--tail-s',30.,'--max-wall-s',600.],work/'swarm.log',184)
        summary=json.loads((work/'swarm/summary.json').read_text())
        progress['smoke']=summary
        # Transport must pass before a full run. No GT is used in this check.
        if not summary.get('input_replay_complete'):
            update(stage='failed',reason='Lossless replay smoke failed');raise SystemExit(1)
        if call([ROOT/'.ros2/research-venv/bin/python','Swarm-SLAM/s3e/remote/check_smoke.py'],
                work/'integration-check.log'):
            update(stage='failed',reason='Native duplicate-stream geometry/input audit failed');raise SystemExit(1)
        update(stage='smoke_complete')
    if args.smoke_only:return
    for name in args.sequences:
        work=base/name;work.mkdir(exist_ok=True)
        f=work/'frontend';f.mkdir(exist_ok=True)
        (f/'config.yaml').write_text(yaml.safe_dump(dict(dataset=f'/data/s3e/S3E_{SEQUENCES[name]}',
            robots=list(ROBOTS),evaluation=dict(evo_max_diff_s=.05)),sort_keys=False))
        record=progress['sequences'].setdefault(name,{})
        try:
            for robot in ROBOTS:
                update(stage='frontend',sequence=name,robot=robot)
                if not frontend(work,SEQUENCES[name],robot,args.rate):
                    raise RuntimeError(f'{robot} frontend failed; inspect retained diagnostics')
            update(stage='swarm',sequence=name,robot=None)
            if not (work/'swarm/summary.json').exists():
                record['swarm_returncode']=call(['bash','Swarm-SLAM/s3e/run.sh','--work',work,
                    '--output',work/'swarm','--tail-s',60.,'--max-wall-s',3600.],work/'swarm.log')
            update(stage='evaluation',sequence=name)
            output=ROOT/f'FAST-LIVO2-ROS2/research/figures/swarm148-{name}'
            record['report']=str(output)
            rc=call([ROOT/'.ros2/research-venv/bin/python','Swarm-SLAM/s3e/evaluate.py',
                     '--work',work,'--output',output],work/'evaluation.log')
            if rc:raise RuntimeError('Native evaluation failed')
            rc=call([ROOT/'.ros2/research-venv/bin/python','Swarm-SLAM/s3e/remote/report.py',
                     '--work',work,'--output',output],work/'report.log')
            if rc:raise RuntimeError('Remote report failed')
            record['status']='finished';record['swarm']=json.loads((work/'swarm/summary.json').read_text())
        except Exception as exc:
            record.update(status='failed',error=repr(exc))
            print('FAILED',name,repr(exc),flush=True)
            if not (work/'swarm/summary.json').exists():
                output=ROOT/f'FAST-LIVO2-ROS2/research/figures/swarm148-{name}'
                record['failure_report_returncode']=call([ROOT/'.ros2/research-venv/bin/python',
                    'Swarm-SLAM/s3e/remote/failure_report.py','--work',work,'--output',output,
                    '--reason',str(exc)],work/'failure-report.log')
                record['report']=str(output)
        update(stage='sequence_finished',sequence=name)
    update(stage='finished',finished_utc=time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime()))


if __name__=='__main__':main()
