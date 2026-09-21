#!/usr/bin/env python3
"""Start a persistent S3E supervisor or stop the session started by this helper."""
from datetime import datetime
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
REGISTRY = ROOT / '.ros2/active_s3e.json'


def alive(pid):
    try:
        return Path(f'/proc/{pid}/stat').read_text().split(') ', 1)[1][0] != 'Z'
    except FileNotFoundError:
        return False


def stop_owned(pid, marker):
    if not pid or not alive(pid):
        return
    if marker not in Path(f'/proc/{pid}/cmdline').read_bytes().replace(b'\0', b' ').decode():
        raise RuntimeError(f'PID {pid} no longer belongs to this S3E session')
    os.kill(pid, signal.SIGINT)
    deadline = time.monotonic() + 20
    while alive(pid) and time.monotonic() < deadline:
        time.sleep(0.1)
    if alive(pid):
        os.kill(pid, signal.SIGTERM)


def main():
    command, *args = sys.argv[1:]
    old = json.loads(REGISTRY.read_text()) if REGISTRY.exists() else None
    if command == 'start':
        if old and alive(old['pid']):
            raise SystemExit('An S3E session is already running. Stop it with run_s3e.sh --stop.')
        directory = ROOT / '.ros2/session_logs'
        directory.mkdir(parents=True, exist_ok=True)
        log = directory / f'{datetime.now():%Y%m%d_%H%M%S_%f}.log'
        with log.open('w') as stream:
            process = subprocess.Popen([str(ROOT / 'FAST-LIVO2-ROS2/scripts/run_s3e.sh'), *args],
                stdin=subprocess.DEVNULL, stdout=stream, stderr=subprocess.STDOUT,
                start_new_session=True, env={**os.environ, 'PYTHONUNBUFFERED': '1'})
        REGISTRY.write_text(json.dumps(dict(pid=process.pid, log=str(log)), indent=2) + '\n')
        print(f'S3E supervisor PID: {process.pid}\nSession log: {log}')
    elif command == 'stop':
        if not old:
            print('No managed S3E session.')
            return
        stop_owned(old['pid'], 'run_s3e.py')
        # A completed replay intentionally leaves the Rerun viewer available.
        for line in Path(old['log']).read_text().splitlines():
            if line.startswith('Run directory: '):
                path = Path(line.removeprefix('Run directory: ')) / 'processes.json'
                if path.exists():
                    processes = json.loads(path.read_text())
                    for key, marker in [('player', 'ros2 bag play'), ('mapper_launch', 'mapping_s3e.launch.py'),
                                        ('bridge', 's3e_rerun.py'), ('viewer', '/rerun-venv/bin/rerun')]:
                        stop_owned(processes.get(key), marker)
        print('S3E replay and visualization stopped.')
    else:
        raise SystemExit('Expected start or stop')


if __name__ == '__main__':
    main()
