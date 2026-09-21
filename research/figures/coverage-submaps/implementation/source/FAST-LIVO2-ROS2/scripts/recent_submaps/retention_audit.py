#!/usr/bin/env python3
"""Hash retained experiment outputs after recording verification and reporting."""
import argparse
import hashlib
import json
from pathlib import Path


def sha(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(8 * 1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def audit(root):
    verification = (root / 'recording-verification.log').read_text()
    if '5 files verified without error' not in verification:
        raise ValueError('Live recordings must pass rerun rrd verify first')
    trials = ['bob-baseline', 'bob-enabled', 'bob-enabled-repeat', 'alpha-submaps', 'carol-submaps']
    records = {}
    binaries = set()
    for trial in trials:
        folder = root / trial
        hashes = json.loads((folder / 'source-hashes.json').read_text())
        binaries.add(hashes['.ros2/recent-submaps/install/ellipselio/lib/libellipselio_mapping.so'])
        path = folder / 'recording/live.rrd'
        records[trial] = dict(path=str(path), bytes=path.stat().st_size, sha256=sha(path),
                              verified=True, source_hashes=hashes)
        if trial != 'bob-baseline':
            records[trial]['snapshot_validation'] = json.loads((folder / 'artifact-validation.json').read_text())
    if len(binaries) != 1:
        raise ValueError('Frontend binary changed across trials')
    work = root / 'multi-robot'
    derived_log = (root / 'derived-recording-verification.log').read_text()
    if '1 file verified without error' not in derived_log and '1 files verified without error' not in derived_log:
        raise ValueError('Derived recording must pass verification')
    outputs = {}
    for path in sorted((work / 'report').rglob('*')):
        if path.is_file():
            outputs[str(path.relative_to(work))] = dict(bytes=path.stat().st_size, sha256=sha(path))
    source = Path('/workspace/FAST-LIVO2-ROS2')
    files = [*source.glob('research/s3e_pipeline/*.py'), *source.glob('scripts/recent_submaps/*')]
    result = dict(schema_version=1, trials=records, identical_frontend_binary=True,
                  native_binary_sha256=next(iter(binaries)), outputs=outputs,
                  backend_source_hashes={str(p.relative_to(source)): sha(p) for p in files if p.is_file()},
                  config_sha256=sha(work / 'config.yaml'),
                  bulk_geometry_retained=True, live_recording_verification=verification,
                  derived_recording_verification=derived_log)
    (root / 'retention-audit.json').write_text(json.dumps(result, indent=2) + '\n')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('root', type=Path)
    audit(parser.parse_args().root)
