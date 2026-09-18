"""Content-addressed stages; COMPLETE is written only after all outputs are hashed."""
from contextlib import contextmanager
import hashlib
import json
import os
from pathlib import Path
import time
import uuid

SCHEMA_VERSION = 1


def canonical(value):
    return json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()


def digest(value):
    return hashlib.sha256(canonical(value)).hexdigest()


def file_hash(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        for block in iter(lambda: f.read(8 * 1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def write_json(path, value):
    Path(path).write_bytes(canonical(value) + b'\n')


def read_json(path):
    return json.loads(Path(path).read_text())


def read_jsonl(path):
    with Path(path).open() as f:
        return [json.loads(line) for line in f if line.strip()]


def write_jsonl(path, rows):
    with Path(path).open('wb') as f:
        for row in rows:
            f.write(canonical(row) + b'\n')


def tree_hashes(path, exclude=()):
    return {str(p.relative_to(path)): file_hash(p) for p in sorted(Path(path).rglob('*'))
            if p.is_file() and str(p.relative_to(path)) not in exclude}


def validate_stage(path, verify_files=True):
    path = Path(path)
    manifest = read_json(path / 'COMPLETE.json')
    if manifest['schema_version'] != SCHEMA_VERSION:
        raise ValueError(f'Unsupported artifact schema: {path}')
    if digest(manifest['identity']) != manifest['stage_hash']:
        raise ValueError(f'Invalid stage identity: {path}')
    if verify_files and tree_hashes(path, ('COMPLETE.json',)) != manifest['files']:
        raise ValueError(f'Modified or incomplete stage: {path}')
    return manifest


def invalidate_dependents(artifacts,replaced_hash):
    """Remove stale registry references without modifying any immutable artifact."""
    obsolete={replaced_hash};removed=[]
    def references(value):
        if isinstance(value,dict):return any(references(v) for v in value.values())
        if isinstance(value,list):return any(references(v) for v in value)
        return isinstance(value,str) and value in obsolete
    while True:
        changed=False
        for label,path in list(artifacts.items()):
            manifest=validate_stage(path,verify_files=False)
            if references(manifest['identity']['inputs']):
                obsolete.add(manifest['stage_hash']);removed.append(label);del artifacts[label];changed=True
        if not changed:return removed


@contextmanager
def stage_output(root, stage, config, inputs, code_hash, resume=False):
    identity = dict(stage=stage, config=config, inputs=inputs, code_hash=code_hash,
                    schema_version=SCHEMA_VERSION)
    key = digest(identity)
    parent = Path(root) / stage
    parent.mkdir(parents=True, exist_ok=True)
    final = parent / key
    if final.exists():
        validate_stage(final)
        if not resume:
            raise FileExistsError(f'Immutable stage exists; use --resume: {final}')
        yield final, True
        return
    lock = parent / (key + '.lock')
    lock_fd = os.open(lock, os.O_CREAT | os.O_EXCL | os.O_WRONLY, 0o644)
    os.write(lock_fd, canonical({'pid': os.getpid(), 'created_unix_ns': time.time_ns()}))
    os.close(lock_fd)
    work = parent / (key + '.incomplete-' + uuid.uuid4().hex)
    work.mkdir()
    start = time.monotonic()
    try:
        yield work, False
        write_json(work / 'COMPLETE.json', dict(schema_version=SCHEMA_VERSION, stage_hash=key,
            identity=identity, files=tree_hashes(work), elapsed_s=time.monotonic() - start))
        work.rename(final)
    except BaseException:
        # Preserve evidence for inspection. Incomplete directories are never cache hits.
        raise
    finally:
        lock.unlink()


def stage_path(root, stage, config, inputs, code_hash):
    return Path(root) / stage / digest(dict(stage=stage, config=config, inputs=inputs,
                                          code_hash=code_hash, schema_version=SCHEMA_VERSION))
