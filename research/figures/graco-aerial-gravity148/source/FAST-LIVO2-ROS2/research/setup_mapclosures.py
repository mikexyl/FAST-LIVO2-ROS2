#!/usr/bin/env python3
"""Build the pinned MapClosures adapter in the CPU research environment."""
import json
from pathlib import Path
import subprocess
import sys
import sysconfig
import hashlib
import argparse

ROOT=Path(__file__).resolve().parents[2]
RESEARCH=Path(__file__).resolve().parent

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--inspection-only',action='store_true');args=parser.parse_args()
    lock=json.loads((RESEARCH/'assets.lock.json').read_text())
    paths={}
    for name in ('mapclosures','mapclosures-hbst','mapclosures-sophus'):
        spec=lock['repositories'][name];path=ROOT/'.ros2/research-models'/name;paths[name]=path
        if not (path/'.git').exists():
            subprocess.run(['git','clone',spec['url'],str(path)],check=True)
            subprocess.run(['git','-C',str(path),'checkout','--detach',spec['commit']],check=True)
        actual=subprocess.check_output(['git','-C',str(path),'rev-parse','HEAD'],text=True).strip()
        if actual!=spec['commit']:raise ValueError(f'{name} source revision mismatch')
        if subprocess.check_output(['git','-C',str(path),'status','--porcelain'],text=True).strip():
            raise ValueError(f'{name} source checkout is modified')
    build=ROOT/('.ros2/mapclosures-inspection-build' if args.inspection_only else '.ros2/mapclosures-build')
    subprocess.run(['cmake','-S',str(RESEARCH/'adapters/mapclosures'),'-B',str(build),
        '-DCMAKE_BUILD_TYPE=Release',f'-DPYTHON_EXECUTABLE={sys.executable}',
        f'-DS3E_INSPECTION_ONLY={"ON" if args.inspection_only else "OFF"}',
        f'-DPYTHON_MODULE_DIR={sysconfig.get_path("platlib")}',
        f'-DMAPCLOSURES_SOURCE={paths["mapclosures"]}',f'-DHBST_SOURCE={paths["mapclosures-hbst"]}',
        f'-DSOPHUS_SOURCE={paths["mapclosures-sophus"]}'],check=True)
    subprocess.run(['cmake','--build',str(build),'--parallel','2'],check=True)
    subprocess.run(['cmake','--install',str(build)],check=True)
    module='s3e_mapclosures_inspection' if args.inspection_only else 's3e_mapclosures_native'
    binary=next(Path(sysconfig.get_path('platlib')).glob(module+'*.so'))
    print(json.dumps(dict(binary=str(binary),sha256=hashlib.sha256(binary.read_bytes()).hexdigest())))

if __name__=='__main__':main()
