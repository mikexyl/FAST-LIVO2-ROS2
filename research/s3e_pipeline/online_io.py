"""Append-only handoffs for live capture. A trailing partial line is not published."""
import json
import os
from pathlib import Path
import time
from .artifacts import canonical


def atomic_json(path, value):
    path=Path(path); temporary=path.with_suffix(path.suffix+'.tmp')
    temporary.write_bytes(canonical(value)+b'\n'); temporary.replace(path)


class JsonlTail:
    def __init__(self, path):
        self.path=Path(path); self.offset=0; self.pending=b''; self.identity=None

    def read(self):
        if not self.path.exists(): return []
        with self.path.open('rb') as stream:
            stat=os.fstat(stream.fileno()); identity=(stat.st_dev,stat.st_ino)
            if self.identity is not None and (identity!=self.identity or stat.st_size<self.offset):
                raise ValueError('Append-only input was replaced or truncated')
            self.identity=identity; stream.seek(self.offset)
            data=stream.read(); self.offset+=len(data)
        lines=(self.pending+data).split(b'\n'); self.pending=lines.pop()
        return [json.loads(line) for line in lines if line.strip()]

    def finish(self):
        rows=self.read()
        if self.pending.strip(): raise ValueError('Incomplete final JSON line')
        return rows


class Journal:
    def __init__(self,path):
        self.stream=Path(path).open('xb',buffering=0)

    def append(self,record):
        self.stream.write(canonical(dict(record,logged_wall_ns=time.time_ns()))+b'\n')

    def close(self): self.stream.close()
