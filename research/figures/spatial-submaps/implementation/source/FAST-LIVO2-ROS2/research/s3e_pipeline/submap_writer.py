#!/usr/bin/env python3
"""Immutable recent-history snapshots over the bounded ResearchExport transport."""
import argparse
import hashlib
import json
from pathlib import Path
import struct
import sys
import numpy as np


def exact(stream, count):
    data = bytearray()
    while len(data) < count:
        block = stream.read(count - len(data))
        if not block:
            raise EOFError('Submap writer missing explicit end record')
        data.extend(block)
    return bytes(data)


def consume(stream, output):
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    rows = []
    with (output/'index.jsonl').open('x') as index:
        while True:
            size = struct.unpack('<I', exact(stream, 4))[0]
            if size == 0:
                break
            if size > 4*1024*1024:
                raise ValueError('Oversize submap header')
            row = json.loads(exact(stream, size))
            if row['schema_version'] not in (1, 3) or any(r['submap_id'] == row['submap_id'] for r in rows):
                raise ValueError('Invalid or repeated submap')
            if row['schema_version'] == 3 and row.get('strategy') != 'spatial':
                raise ValueError('Schema 3 requires the spatial strategy')
            arrays = {}
            for name, width, key in [('points', 3, 'geometry_count'), ('ellipsoids', 15, 'ellipsoid_count')]:
                count = row[key]
                if not 0 <= count <= 10000000:
                    raise ValueError('Invalid geometry count')
                arrays[name] = np.frombuffer(exact(stream, count*width*4), dtype='<f4').reshape(-1, width)
                if not np.isfinite(arrays[name]).all():
                    raise ValueError('Nonfinite snapshot')
            name = f"submap-{row['submap_id']:06d}.npz"
            with (output/name).open('xb') as payload:
                np.savez_compressed(payload, **arrays)
            row.update(payload=name, sha256=hashlib.sha256((output/name).read_bytes()).hexdigest())
            index.write(json.dumps(row, sort_keys=True)+'\n')
            index.flush()
            rows.append(row)
    manifest = dict(schema_version=1, complete=True, submaps=len(rows),
                    index_sha256=hashlib.sha256((output/'index.jsonl').read_bytes()).hexdigest())
    (output/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True)
    consume(sys.stdin.buffer, parser.parse_args().output)
