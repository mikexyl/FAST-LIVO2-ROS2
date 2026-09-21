#!/usr/bin/env python3
"""Persistent, fixed GRACO six-robot smoke then full run; no parameter sweep."""
import fcntl
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import traceback

ROOT = Path('/workspace/.ros2/graco')
SOURCE = Path('/workspace')
sys.path.insert(0, str(SOURCE/'FAST-LIVO2-ROS2/scripts'))
from run_graco import source_hashes


def save(path, value):
    temp = path.with_suffix('.tmp'); temp.write_text(json.dumps(value, indent=2)+'\n'); temp.replace(path)


def stop(proc):
    import psutil
    try: descendants = psutil.Process(proc.pid).children(recursive=True)
    except psutil.NoSuchProcess: descendants = []
    if proc.poll() is None:
        proc.send_signal(signal.SIGINT)
        try: proc.wait(timeout=60)
        except subprocess.TimeoutExpired: pass
    for child in reversed(descendants):
        try: child.terminate()
        except psutil.NoSuchProcess: pass
    _, alive = psutil.wait_procs(descendants, timeout=5)
    for child in alive:
        try: child.kill()
        except psutil.NoSuchProcess: pass
    if proc.poll() is None: proc.kill()
    proc.wait()


def main():
    control = ROOT/'controller'; control.mkdir(parents=True, exist_ok=False)
    lock = (control/'lock').open('w'); fcntl.flock(lock, fcntl.LOCK_EX|fcntl.LOCK_NB)
    frozen = source_hashes(); save(control/'source-hashes.json', frozen)
    status = dict(pid=os.getpid(), robots=[f'robot{i}' for i in range(1,7)], expected_components=1,
                  started_utc=time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime()), attempts=[])
    def update(phase, **values):
        status.update(phase=phase, updated_utc=time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime()), **values)
        save(control/'status.json', status)
    proc = None
    try:
        update('waiting_for_verified_transfer')
        while not (ROOT/'inputs/TRANSFER_COMPLETE.json').exists(): time.sleep(10)
        for label, duration in [('smoke',45),('full',0)]:
            if source_hashes()!=frozen: raise ValueError('Frozen source changed before experiment')
            work = ROOT/'runs'/f'ground-01-06-{label}-20260917'
            command = [sys.executable, str(SOURCE/'FAST-LIVO2-ROS2/scripts/run_graco.py'),
                       '--stage','run','--inputs','/data/graco-inputs','--work',str(work),'--duration',str(duration)]
            status['attempts'].append(dict(label=label, work=str(work), command=command))
            update(label, work=str(work))
            with (control/f'{label}.log').open('w') as log:
                proc = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
                deadline = time.monotonic()+6*3600
                while proc.poll() is None:
                    if time.monotonic()>deadline: raise TimeoutError('GRACO attempt exceeded six-hour wall limit')
                    update(label, work=str(work), subprocess_pid=proc.pid)
                    time.sleep(10)
                if proc.returncode: raise RuntimeError(f'{label} exited {proc.returncode}; see {work}/failure.json')
            status['attempts'][-1]['status']='complete'
        connected = json.loads((work/'connectivity.json').read_text())
        update('complete', all_six_connected=connected['all_six_connected'])
    except BaseException as exc:
        if proc is not None: stop(proc)
        update('failed', error=str(exc), traceback=traceback.format_exc())
        raise


if __name__=='__main__': main()
