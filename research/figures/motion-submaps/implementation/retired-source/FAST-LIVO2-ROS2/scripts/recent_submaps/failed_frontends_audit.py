#!/usr/bin/env python3
"""Retain hashes and check controls for the completed failed-frontend retries."""
import argparse
import hashlib
import json
from pathlib import Path
import yaml


def sha(path):
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(8*1024*1024), b''):
            h.update(block)
    return h.hexdigest()


def audit(root):
    state = json.loads((root/'status.json').read_text())
    assert state.get('finished_utc'), 'Queue incomplete'
    cases = json.loads((root/'cases.json').read_text())
    records = {}; binaries = set()
    for case, outcome in zip(cases, state['cases']):
        assert case['name'] == outcome['name']
        assert all(outcome[k] == 0 for k in ('frontend_exit','evaluation_exit','validation_exit','recording_verification_exit'))
        trial = root/case['name']
        original = yaml.safe_load(Path(case['original_config']).read_text())['/**']['ros__parameters']
        current = yaml.safe_load(Path(case['config']).read_text())['/**']['ros__parameters']
        assert original['lidar'] == current['lidar']
        assert original['imu']['topic'] == current['imu']['topic']
        assert original['imu']['rate'] == current['imu']['rate']
        if case['dataset'] == 'GRACO':
            assert original['imu'] == current['imu']
        assert current['input']['reliable'] and current['mapping']['submaps'] == dict(enabled=True,duration_s=10.,overlap_s=5.)
        coverage = json.loads((trial/'sensor-coverage.json').read_text())
        assert coverage['full_selected_sensor_tail_reached']
        validation = json.loads((trial/'artifact-validation.json').read_text())
        log = (trial/'recording-verification.log').read_text()
        assert 'verified without error' in log
        sources = json.loads((trial/'source-hashes.json').read_text())
        binaries.add(sources['.ros2/recent-submaps/install/ellipselio/lib/libellipselio_mapping.so'])
        files = [trial/'recording/live.rrd',trial/'frontend/native_updates.jsonl',trial/'frontend/submaps/index.jsonl',
                 trial/'frontend/runtime.yaml',trial/'source-hashes.json']
        records[case['name']] = dict(coverage=coverage, snapshot_validation=validation,
            source_sensor_settings_preserved=True, original_imu=original['imu'], current_imu=current['imu'],
            recording_verified=True, files={str(p.relative_to(root)):dict(bytes=p.stat().st_size,sha256=sha(p)) for p in files})
    assert len(binaries)==1
    before=json.loads((root/'graco-ground01-robot1/evaluation/metrics.json').read_text())
    after=json.loads((root/'graco-ground01-robot1/evaluation-graco-reference/metrics.json').read_text())
    assert all(after[k]==v for k,v in before.items() if k!='gt_reference')
    result=dict(schema_version=1, identical_native_binary=True, native_binary_sha256=next(iter(binaries)),
                graco_reference_label_correction_only=True, all_bulk_geometry_retained=True, trials=records)
    (root/'retention-audit.json').write_text(json.dumps(result,indent=2)+'\n')


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('root',type=Path)
    audit(parser.parse_args().root)
