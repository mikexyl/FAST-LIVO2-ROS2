#!/usr/bin/env python3
"""One fresh 1x frontend capture in the isolated ROS-sourced container."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import yaml

root=Path(__file__).resolve().parents[3]
p=argparse.ArgumentParser()
p.add_argument('--robot',default='Bob')
p.add_argument('--bag',type=Path,default=Path('/data/s3e/S3Ev2/S3E_Library_2'))
p.add_argument('--output-root',type=Path,default=root/'.ros2/recent-submaps')
p.add_argument('--mapping-config',type=Path)
p.add_argument('--mapper-executable',type=Path,default=root/'.ros2/recent-submaps/launcher/ellipselio_mapping_mt')
p.add_argument('--mapping-library',type=Path,default=root/'.ros2/recent-submaps/install/ellipselio/lib/libellipselio_mapping.so')
p.add_argument('name');p.add_argument('--enabled',action='store_true');p.add_argument('--duration',type=float,default=0)
args=p.parse_args()
out=args.output_root/args.name
out.mkdir(parents=True,exist_ok=False)
config=args.mapping_config or root/'FAST-LIVO2-ROS2/scripts/recent_submaps'/('enabled.yaml' if args.enabled else 'baseline.yaml')
if (args.robot!='Bob' or args.bag!=Path('/data/s3e/S3Ev2/S3E_Library_2')) and args.mapping_config is None:
    raise ValueError('Supply a dataset/robot calibrated mapping config')
metadata=yaml.safe_load((args.bag/'metadata.yaml').read_text())['rosbag2_bagfile_information']
start_ns=metadata['starting_time']['nanoseconds_since_epoch']
mapping_params=yaml.safe_load(config.read_text())['/**']['ros__parameters']
provenance={}
for directory in (root/'ellipselio/src',root/'ellipselio/include',root/'ellipselio/msg',root/'FAST-LIVO2-ROS2/scripts/recent_submaps'):
    for path in directory.rglob('*'):
        if path.is_file() and '__pycache__' not in str(path):
            provenance[str(path.relative_to(root))]=hashlib.sha256(path.read_bytes()).hexdigest()
for path in (config,root/'ellipselio/CMakeLists.txt',root/'FAST-LIVO2-ROS2/scripts/run_ellipselio.py',
             root/'FAST-LIVO2-ROS2/scripts/ellipselio_live_rerun.py',
             root/'FAST-LIVO2-ROS2/research/s3e_pipeline/submap_writer.py',
             args.mapping_library,args.mapper_executable):
    provenance[str(path.relative_to(root))]=hashlib.sha256(path.read_bytes()).hexdigest()
(out/'source-hashes.json').write_text(json.dumps(provenance,indent=2)+'\n')
recorder=None
try:
    with (out/'recorder.log').open('w') as log:
        recorder=subprocess.Popen([str(root/'.ros2/rerun-venv/bin/python'),
            str(root/'FAST-LIVO2-ROS2/scripts/ellipselio_live_rerun.py'),'--robot',args.robot,'--start-ns',str(start_ns),
            '--sequence',args.bag.name,'--imu-topic',mapping_params['imu']['topic'],
            '--output',str(out/'recording'),'--grpc-port','0',
            '--submap-dir',str(out/'frontend/submaps')],stdout=log,stderr=subprocess.STDOUT)
        deadline=time.monotonic()+60
        while not (out/'recording/READY').exists():
            if recorder.poll() is not None or time.monotonic()>deadline:raise RuntimeError('Recorder did not initialize')
            time.sleep(.1)
        command=['/usr/bin/python3',str(root/'FAST-LIVO2-ROS2/scripts/run_ellipselio.py'),
            '--robot',args.robot,'--bag',str(args.bag),'--mapping-config',str(config),
            '--output',str(out/'frontend'),'--rate','1','--no-research-export',
            '--mapper-executable',str(args.mapper_executable)]
        if args.duration:command+=['--duration',str(args.duration)]
        with (out/'trial.log').open('w') as trial_log, (out/'memory.jsonl').open('w') as memory:
            trial=subprocess.Popen(command,stdout=trial_log,stderr=subprocess.STDOUT)
            while trial.poll() is None:
                if recorder.poll() is not None:
                    trial.send_signal(signal.SIGINT);trial.wait(timeout=90)
                    raise RuntimeError('Recorder exited during replay')
                processes=out/'frontend/processes.json'
                if processes.exists():
                    pid=json.loads(processes.read_text())['mapper']
                    try:
                        status=Path(f'/proc/{pid}/status').read_text()
                        fields={k:int(v.split()[0]) for k,v in (line.split(':',1) for line in status.splitlines() if line.startswith(('VmRSS:','VmHWM:')))}
                        memory.write(json.dumps(dict(wall_ns=time.time_ns(),**fields))+'\n');memory.flush()
                    except FileNotFoundError:pass
                time.sleep(1)
            if trial.returncode:raise RuntimeError(f'Trial failed: {trial.returncode}')
finally:
    if recorder is not None and recorder.poll() is None:
        recorder.send_signal(signal.SIGINT);recorder.wait(timeout=90)
