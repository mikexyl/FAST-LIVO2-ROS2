#!/usr/bin/env python3
"""Read-only collector for retained online CBS timings. Emit JSON to stdout.

Run on the capture host; no geometry, optimization, or replay is required.
"""
import csv
import hashlib
import io
import json
from pathlib import Path
import sys


def collect(root):
    sources = {}

    def read(path):
        raw = path.read_bytes()
        sources[str(path.relative_to(root))] = hashlib.sha256(raw).hexdigest()
        return raw.decode()

    def obj(path):
        return json.loads(read(path))

    def lines(path):
        return [json.loads(s) for s in read(path).splitlines() if s.strip()]

    start = obj(root / 'START')
    robots = {}
    for robot in start['robots']:
        p = root / 'epochs/persistent' / robot
        native = p / 'native' / robot / 'cbs_online'
        timing = list(native.glob('timing_robot_*.csv'))
        service = list(native.glob('belief_service_timing_robot_*.csv'))
        assert len(timing) == len(service) == 1
        robots[robot] = dict(
            stats=lines(p / 'stats.jsonl'),
            timing=list(csv.DictReader(io.StringIO(read(timing[0])))),
            service=list(csv.DictReader(io.StringIO(read(service[0])))),
            cbs_yaml=read(p / 'cbs.yaml'),
        )
    revisions = []
    for p in sorted((root / 'epochs').glob('[0-9][0-9][0-9]')):
        inp = obj(p / 'input.json')
        summary = obj(p / 'dpgo/summary.json')
        registration = obj(p / 'dpgo/registration.json')
        revisions.append(dict(
            revision=inp['revision'], cutoff_wall_ns=inp['cutoff_wall_ns'],
            local_submaps=inp['rows'], global_submaps=summary['keyframes'],
            global_loops=summary['loops'], final=inp['final'],
            global_registration_pairs=registration['registration_factor_count'],
            local_registration_pairs={r: sum(bool(e['accepted']) for e in v['pairs'])
                                      for r, v in registration['robots'].items()},
        ))
    return dict(schema=1, root=str(root), start=start, robots=robots,
                revisions=revisions, events=lines(root / 'epochs/events.jsonl'),
                source_sha256=sources)


if __name__ == '__main__':
    json.dump(collect(Path(sys.argv[1])), sys.stdout, separators=(',', ':'))
    print()
