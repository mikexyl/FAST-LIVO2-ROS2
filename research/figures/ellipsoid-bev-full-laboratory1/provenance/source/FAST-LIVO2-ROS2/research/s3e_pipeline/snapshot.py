"""Archive research sources, then execute every worker from that immutable copy."""
import hashlib
import json
import os
from pathlib import Path
import sys
import tempfile


def freeze(source,destination):
    source=Path(source);destination=Path(destination);destination.mkdir(parents=True,exist_ok=True)
    files={str(p.relative_to(source)):p.read_bytes() for p in sorted(source.rglob('*'))
        if p.is_file() and '__pycache__' not in p.parts and '.pytest_cache' not in p.parts
        and p.suffix not in ('.pyc','.pyo')}
    hashes={name:hashlib.sha256(data).hexdigest() for name,data in files.items()}
    key=hashlib.sha256(json.dumps(hashes,sort_keys=True,separators=(',',':')).encode()).hexdigest()
    final=destination/key
    if final.exists():
        if json.loads((final/'SOURCE.json').read_text())!=hashes:raise ValueError('Modified code snapshot manifest')
        for name,expected in hashes.items():
            if hashlib.sha256((final/'research'/name).read_bytes()).hexdigest()!=expected:
                raise ValueError('Modified code snapshot')
        return final/'research'
    with tempfile.TemporaryDirectory(prefix='.source-',dir=destination) as temporary:
        temp=Path(temporary)
        for name,data in files.items():
            out=temp/'research'/name;out.parent.mkdir(parents=True,exist_ok=True);out.write_bytes(data)
            if (source/name).read_bytes()!=data:raise RuntimeError('Source changed while taking snapshot; retry the invocation')
        (temp/'SOURCE.json').write_text(json.dumps(hashes,sort_keys=True,indent=2)+'\n')
        # Other invocations may publish the same content concurrently.
        try:temp.rename(final)
        except OSError:
            if not final.exists():raise
    return final/'research'


def main():
    workspace=Path(os.environ['S3E_SOURCE_ROOT']).resolve()
    source=Path(os.environ.get('S3E_RESEARCH_SNAPSHOT') or workspace/'FAST-LIVO2-ROS2/research')
    frozen=freeze(source,workspace/'.ros2/code-snapshots')
    if sys.argv[1:]==['--print-root']:
        print(frozen);return
    env=dict(os.environ,PYTHONPATH=str(frozen)+os.pathsep+os.environ.get('PYTHONPATH',''),
             S3E_RESEARCH_SNAPSHOT=str(frozen),PYTHONDONTWRITEBYTECODE='1')
    os.execve(sys.executable,[sys.executable,'-m','s3e_pipeline.cli',*sys.argv[1:]],env)


if __name__=='__main__':main()
