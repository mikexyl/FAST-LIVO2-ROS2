#!/usr/bin/env python3
"""Install only the pinned MegaLoc source and checkpoint."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import tarfile
ROOT=Path(__file__).resolve().parents[2]


def sha(path):
    h=hashlib.sha256()
    with path.open('rb') as f:
        for block in iter(lambda:f.read(8*1024*1024),b''):h.update(block)
    return h.hexdigest()


def main():
    p=argparse.ArgumentParser();p.add_argument('--verify-only',action='store_true');args=p.parse_args()
    lock=json.loads(Path(__file__).with_name('assets.lock.json').read_text())
    destination=ROOT/'.ros2/research-models/megaloc';destination.mkdir(parents=True,exist_ok=True)
    if not args.verify_only:
        if not (destination/'megaloc_model.py').exists():
            rev=lock['repositories']['megaloc']['commit'];archive=destination/'source.tar.gz'
            subprocess.run(['curl','-L','--fail',f'https://codeload.github.com/gmberton/MegaLoc/tar.gz/{rev}','-o',str(archive)],check=True)
            with tarfile.open(archive) as tar:
                for member in tar.getmembers():
                    relative=Path(*Path(member.name).parts[1:])
                    if not relative.parts:continue
                    if '..' in relative.parts or member.issym() or member.islnk():raise ValueError('Unsafe source archive')
                    member.name=str(relative);tar.extract(member,destination)
        checkpoint=destination/'model.safetensors'
        if not checkpoint.exists():
            partial=checkpoint.with_suffix('.partial')
            subprocess.run(['curl','-L','--fail',lock['checkpoint_urls']['megaloc/model.safetensors'],'-o',str(partial)],check=True)
            partial.rename(checkpoint)
    for name,expected in lock['checkpoints'].items():
        if sha(ROOT/name)!=expected:raise ValueError('MegaLoc checkpoint hash mismatch')
    (destination.parent/'gpu.lock').touch(exist_ok=True)
    print('Pinned MegaLoc checkpoint verified.')


if __name__=='__main__':main()
