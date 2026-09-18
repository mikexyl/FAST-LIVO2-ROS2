#!/usr/bin/env python3
"""Regenerate shared EllipseLIO exports and paired BEV caches, once per sequence."""
import concurrent.futures
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import time
import yaml

ROOT=Path(__file__).resolve().parents[2]
ROBOTS=('Alpha','Bob','Carol')
SEQUENCES={'square1':'Square_1','square2':'Square_2','laboratory1':'Laboratory_1'}

def write(path,data):
    path.write_text(json.dumps(data,indent=2)+'\n')

def prepare(work,robot):
    env=dict(os.environ,PYTHONPATH=str(ROOT/'FAST-LIVO2-ROS2/research'),
             OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1')
    with (work/f'prepare-{robot}.log').open('w') as log:
        subprocess.run([str(ROOT/'.ros2/research-venv/bin/python'),'-m','s3e_pipeline.ellipsoid_full',
            'prepare','--work',str(work),'--robot',robot,'--resume'],env=env,stdout=log,stderr=subprocess.STDOUT,check=True)
    print(work.parent.name,robot,'descriptor preparation complete',flush=True)

def main():
    os.chdir(ROOT)
    tasks=[]
    with concurrent.futures.ThreadPoolExecutor(max_workers=1) as gpu:
        for sequence,dataset in SEQUENCES.items():
            archive=ROOT/f'FAST-LIVO2-ROS2/research/figures/ellipsoid-bev-full-{sequence}'
            work=ROOT/f'.ros2/swarm/{sequence}/frontend'
            if not work.exists():
                work.mkdir(parents=True)
                cfg=json.loads((archive/'inputs.json').read_text())['config']
                cfg.update(dataset=f'/data/s3e/S3E_{dataset}',output_root=str(work))
                cfg['odometry']['rate']=1.0
                (work/'config.yaml').write_text(yaml.safe_dump(cfg,sort_keys=False))
                shutil.copy2(archive/'provenance/schedule.json',work/'schedule.json')
                schedule=json.loads((work/'schedule.json').read_text())['schedule']
                for robot in ROBOTS:
                    mapping=yaml.safe_load((ROOT/f'.ros2/ellipse-configs/{robot}.yaml').read_text())
                    mapping['/**']['ros__parameters']['research']=dict(
                        ellipsoid_stamps_ns=[r['stamp_ns'] for r in schedule[robot]],
                        ellipsoid_range_m=80.,ellipsoid_world_frame=True,ellipsoid_delta_export=True)
                    (work/f'{robot}.yaml').write_text(yaml.safe_dump(mapping,sort_keys=False))
                write(work/'comparison.json',dict(baseline_archive=str(archive),
                    shared_inputs='Both systems consume this same fresh EllipseLIO export; Swarm uses its native keyframe selector and single scans.',
                    replay_policy='1x serial replay; one documented 0.5x retry on failure; single CUDA preparation job.',
                    ground_truth_used=False))
            for robot in ROBOTS:
                out=work/robot
                if (out/'summary.json').exists() and json.loads((out/'summary.json').read_text()).get('success'):
                    print(sequence,robot,'reuse complete export',flush=True)
                else:
                    for attempt,rate in enumerate((1.,.5),1):
                        if out.exists():
                            raise RuntimeError(f'Inspect incomplete export before restarting: {out}')
                        print(sequence,robot,'replay',rate,flush=True)
                        with (work/f'{robot}-attempt{attempt}.log').open('w') as log:
                            result=subprocess.run(['bash','FAST-LIVO2-ROS2/scripts/run_ellipselio.sh',
                                '--robot',robot,'--bag',f'/data/s3e/S3E_{dataset}',
                                '--output',str(out),'--mapping-config',str(work/f'{robot}.yaml'),
                                '--rate',str(rate)],stdout=log,stderr=subprocess.STDOUT)
                        if result.returncode==0:break
                        if attempt==2:raise RuntimeError(f'{sequence} {robot} failed both replay attempts')
                        if out.exists():shutil.move(out,work/f'{robot}-attempt1-failed')
                    print(sequence,robot,'replay complete',flush=True)
                tasks.append(gpu.submit(prepare,work,robot))
                # Surface previous preparation failures promptly.
                for task in tasks:
                    if task.done():task.result()
        for task in tasks:task.result()
    print('All shared frontends and descriptor stores complete.',flush=True)

if __name__=='__main__':main()
