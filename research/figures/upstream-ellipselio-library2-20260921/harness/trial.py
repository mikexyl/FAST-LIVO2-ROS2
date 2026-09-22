#!/usr/bin/env python3
"""Isolated upstream replay with a direct ROS Rerun recording and native diagnostics."""
import argparse, hashlib, json, os, signal, subprocess, time
from pathlib import Path
import yaml
root=Path('/workspace/.ros2/upstream-ellipselio-20260921')
p=argparse.ArgumentParser();p.add_argument('name');p.add_argument('--duration',type=float,default=0);a=p.parse_args()
out=root/a.name;out.mkdir(exist_ok=False)
bag=Path('/data/s3e/S3Ev2/S3E_Library_2')
metadata=yaml.safe_load((bag/'metadata.yaml').read_text())['rosbag2_bagfile_information']
start=metadata['starting_time']['nanoseconds_since_epoch']
source=json.loads((root/'source-hashes.json').read_text())
for f in [root/'install/ellipselio/lib/libellipselio_mapping.so',root/'launcher/ellipselio_mapping_mt',*list((root/'harness').glob('*.py'))]:
    source[str(f.relative_to(root))]=hashlib.sha256(f.read_bytes()).hexdigest()
(out/'source-hashes.json').write_text(json.dumps(source,indent=2)+'\n')
(out/'bag-metadata.yaml').write_text((bag/'metadata.yaml').read_text())
recorder=trial=None
try:
    with (out/'recorder.log').open('w') as log:
        recorder=subprocess.Popen(['/workspace/.ros2/rerun-venv/bin/python',str(root/'harness/ellipselio_live_rerun.py'),'--robot','Bob','--sequence','Library 2 — upstream 6506f46','--start-ns',str(start),'--output',str(out/'recording'),'--grpc-port','0'],stdout=log,stderr=subprocess.STDOUT)
        deadline=time.monotonic()+60
        while not (out/'recording/READY').exists():
            if recorder.poll() is not None or time.monotonic()>deadline:raise RuntimeError('Recorder initialization failed')
            time.sleep(.1)
        env=dict(os.environ,ELLIPSELIO_TRIAL_LOG=str(out/'frontend/native_updates.jsonl'))
        cmd=['/usr/bin/python3',str(root/'harness/run_upstream.py'),'--robot','Bob','--bag',str(bag),'--output',str(out/'frontend'),'--mapping-config',str(root/'harness/bob.yaml'),'--mapper-executable',str(root/'launcher/ellipselio_mapping_mt'),'--no-research-export']
        if a.duration:cmd+=['--duration',str(a.duration)]
        with (out/'trial.log').open('w') as log,(out/'memory.jsonl').open('w') as memory:
            trial=subprocess.Popen(cmd,env=env,stdout=log,stderr=subprocess.STDOUT)
            while trial.poll() is None:
                if recorder.poll() is not None:raise RuntimeError('Recorder exited during replay')
                process=out/'frontend/processes.json'
                if process.exists():
                    pid=json.loads(process.read_text())['mapper']
                    try:
                        status=Path(f'/proc/{pid}/status').read_text()
                        fields={k:int(v.split()[0]) for k,v in (line.split(':',1) for line in status.splitlines() if line.startswith(('VmRSS:','VmHWM:')))}
                        memory.write(json.dumps(dict(wall_ns=time.time_ns(),**fields))+'\n');memory.flush()
                        if not (out/'loaded-library.txt').exists():
                            maps=Path(f'/proc/{pid}/maps').read_text()
                            (out/'loaded-library.txt').write_text('\n'.join(s for s in maps.splitlines() if 'ellipselio_mapping.so' in s)+'\n')
                    except FileNotFoundError:pass
                time.sleep(1)
            if trial.returncode:raise RuntimeError(f'Trial failed: {trial.returncode}')
finally:
    for proc in [trial,recorder]:
        if proc is not None and proc.poll() is None:
            proc.send_signal(signal.SIGINT);proc.wait(timeout=90)
