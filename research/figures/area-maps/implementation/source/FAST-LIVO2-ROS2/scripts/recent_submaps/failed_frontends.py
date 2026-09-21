#!/usr/bin/env python3
"""Fresh, serial submap retries of the previously failed native frontends."""
import argparse
import copy
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import yaml

ROOT = Path('/workspace')
SCRIPTS = ROOT / 'FAST-LIVO2-ROS2/scripts/recent_submaps'
WORK = ROOT / '.ros2/recent-submaps/failed-frontends-20260918'
RESEARCH = ROOT / '.ros2/research-venv/bin/python'


def save(path, value):
    temp = path.with_suffix('.tmp')
    temp.write_text(json.dumps(value, indent=2) + '\n')
    temp.replace(path)


def prepare():
    WORK.mkdir(exist_ok=False)
    (WORK / 'configs').mkdir()
    previous = json.loads((ROOT / '.ros2/ellipsoid-cbs148/overnight-20260916/queue.json').read_text())['sequences']
    cases = []
    for sequence, robot in [('S3E_Laboratory_4', 'Carol'), ('S3E_Campus_Road_1', 'Alpha'),
                            ('S3E_Campus_Road_2', 'Alpha'), ('S3E_Campus_Road_3', 'Alpha')]:
        entry = previous[sequence]
        cases.append(dict(name=sequence.lower() + '-' + robot.lower(), robot=robot,
                          bag=entry['dataset'], original_config=str(Path(entry['work']) / f'{robot}.yaml'),
                          truth=str(Path(entry['dataset']) / f'{robot.lower()}_gt.txt'), dataset='S3E'))
    cases.append(dict(name='graco-ground01-robot1', robot='robot1', dataset='GRACO',
                      bag=str(ROOT / '.ros2/graco/inputs/sensors/robot1'),
                      original_config=str(ROOT / '.ros2/graco/runs/ground-01-06-full-20260917/robot1.yaml'),
                      truth=str(ROOT / '.ros2/graco/inputs/reference/robot1_gt.txt')))
    for case in cases:
        original = Path(case['original_config'])
        cfg = yaml.safe_load(original.read_text())
        params = cfg['/**']['ros__parameters']
        case['original_config_sha256'] = hashlib.sha256(original.read_bytes()).hexdigest()
        case['original_imu'] = copy.deepcopy(params['imu'])
        params.pop('research', None)
        params['mapping']['submaps'] = dict(enabled=True, duration_s=10., overlap_s=5.)
        params['publish'] = dict(map=True, scan=True, markers=True, odometry=True, analytics=True, tf=True)
        params['input'] = dict(reliable=True)
        if case['dataset'] == 'S3E':
            params['imu'].update(acc_noise=.1, gyr_noise=.1, acc_bias=.0001, gyr_bias=.0001)
        path = WORK / 'configs' / (case['name'] + '.yaml')
        path.write_text(yaml.safe_dump(cfg, sort_keys=False))
        case['config'] = str(path)
        case['config_sha256'] = hashlib.sha256(path.read_bytes()).hexdigest()
        metadata = yaml.safe_load((Path(case['bag']) / 'metadata.yaml').read_text())['rosbag2_bagfile_information']
        case['bag_duration_s'] = metadata['duration']['nanoseconds'] / 1e9
        case['start_ns'] = metadata['starting_time']['nanoseconds_since_epoch']
    save(WORK / 'cases.json', cases)
    return cases


def execute(command, log_path, timeout):
    with log_path.open('w') as log:
        proc = subprocess.Popen([str(x) for x in command], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            return proc.wait(timeout=timeout)
        except BaseException:
            os.killpg(proc.pid, signal.SIGINT)
            try:
                proc.wait(timeout=90)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
                proc.wait()
            raise


def run():
    cases = json.loads((WORK / 'cases.json').read_text())
    status = dict(started_utc=time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()), cases=[])
    if (WORK / 'status.json').exists():
        raise ValueError('Use a fresh queue; existing attempts are immutable')
    for case in cases:
        trial = WORK / case['name']
        row = dict(name=case['name'], robot=case['robot'], phase='frontend')
        status['cases'].append(row)
        save(WORK / 'status.json', status)
        try:
            command = ['/usr/bin/python3', SCRIPTS / 'run_trial.py', case['name'], '--enabled',
                       '--robot', case['robot'], '--bag', case['bag'], '--mapping-config', case['config'],
                       '--output-root', WORK]
            row['frontend_exit'] = execute(command, WORK / (case['name'] + '.log'), case['bag_duration_s'] + 240)
            row['phase'] = 'evaluation'
            save(WORK / 'status.json', status)
            # Evaluation failures remain explicit; never turn partial output into a pass.
            row['evaluation_exit'] = execute([RESEARCH, SCRIPTS / 'evaluate.py', trial, '--robot', case['robot'],
                '--truth', case['truth'], '--gt-reference', case['dataset'].lower(),
                '--output', trial / 'evaluation'], trial / 'evaluation.log', 120)
            if (trial / 'evaluation/metrics.json').exists():
                row['metrics'] = json.loads((trial / 'evaluation/metrics.json').read_text())
                row['metrics'].pop('handovers', None)
            row['validation_exit'] = execute([RESEARCH, SCRIPTS / 'validate.py', trial], trial / 'validation.log', 180)
            row['recording_verification_exit'] = execute([ROOT / '.ros2/rerun-venv/bin/rerun', 'rrd', 'verify',
                trial / 'recording/live.rrd'], trial / 'recording-verification.log', 180)
            row['phase'] = 'complete'
            row['passed'] = all(row[k] == 0 for k in ('frontend_exit', 'evaluation_exit', 'validation_exit',
                'recording_verification_exit')) and row.get('metrics', {}).get('gate_pass', False)
        except Exception as exc:
            row.update(phase='error', passed=False, error=repr(exc))
        row['finished_utc'] = time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime())
        save(WORK / 'status.json', status)
    status['finished_utc'] = time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime())
    save(WORK / 'status.json', status)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('stage', choices=['prepare', 'run'])
    args = parser.parse_args()
    if args.stage == 'prepare':
        print(json.dumps(prepare(), indent=2))
    else:
        run()
