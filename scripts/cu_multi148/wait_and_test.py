#!/usr/bin/env python3
"""Persistent host-side download gate followed by one bounded four-robot smoke."""
import argparse
import fcntl
import json
import os
from pathlib import Path
import subprocess
import time
import traceback


def save(path, value):
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(value, indent=2)+'\n'); temporary.replace(path)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--globus', type=Path, default=Path('/mnt/data/cu-multi/.pixi/envs/default/bin/globus'))
    p.add_argument('--transfer', type=Path, default=Path('/data3/mikexyl/datasets/cu-multi/lidar-transfer.json'))
    p.add_argument('--container', default='ellipsoid-cu-multi148')
    p.add_argument('--work', default='/workspace/.ros2/cu-multi/runs/main_campus-smoke-20260917')
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args(); args.output.mkdir(parents=True, exist_ok=True)
    lock = (args.output/'worker.lock').open('w'); fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    task = json.loads(args.transfer.read_text())['task_id']
    status = dict(task_id=task, container=args.container, work=args.work,
                  duration_s=120, robots=['robot1','robot2','robot3','robot4'], pid=os.getpid())
    status_path = args.output/'status.json'
    def update(phase, **fields):
        status.update(phase=phase, updated_utc=time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()), **fields)
        save(status_path, status); print(json.dumps(status), flush=True)
    try:
        deadline = time.monotonic()+24*3600
        while True:
            result = subprocess.run([str(args.globus), 'task', 'show', task, '--format', 'json'],
                                    capture_output=True, text=True, timeout=45)
            if result.returncode:
                raise RuntimeError('Globus status query failed: '+result.stderr[-2000:])
            transfer = json.loads(result.stdout)
            update('waiting_for_download', transfer={k: transfer.get(k) for k in
                   ('status','nice_status','bytes_transferred','files_transferred','total_estimated_bytes','effective_bytes_per_second','faults')})
            if transfer['status'] == 'SUCCEEDED':
                save(args.output/'verified-transfer.json', transfer)
                break
            if transfer['status'] != 'ACTIVE' or time.monotonic() >= deadline:
                raise RuntimeError('LiDAR download failed or exceeded the 24-hour deadline')
            time.sleep(30)
        command = ['docker', 'exec', args.container, 'bash',
                   'FAST-LIVO2-ROS2/scripts/ellipsoid_cbs148/env.sh',
                   '.ros2/research-venv/bin/python', 'FAST-LIVO2-ROS2/scripts/run_cu_multi.py',
                   '--stage', 'all', '--duration', '120', '--work', args.work]
        save(args.output/'command.json', command)
        update('testing')
        with (args.output/'smoke.log').open('w') as log:
            proc = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)
            try:
                code = proc.wait(timeout=6*3600)
            except BaseException:
                # This container is dedicated to this attempt. Stop native
                # children too; killing only the docker client leaves them live.
                subprocess.run(['docker','stop','--time','60',args.container], timeout=90)
                proc.wait(timeout=30)
                raise
        if code:
            raise RuntimeError(f'Smoke test exited {code}; see smoke.log and attempt failure.json')
        update('complete')
    except BaseException as exc:
        update('failed', error=str(exc), traceback=traceback.format_exc())
        raise


if __name__ == '__main__':
    main()
