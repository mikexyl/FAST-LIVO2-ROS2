#!/usr/bin/env python3
"""Retain compact comparison evidence before retiring completed-run cloud caches."""
import argparse
import gzip
import json
from pathlib import Path
import shutil
import subprocess
import tarfile
from s3e_pipeline.artifacts import file_hash,read_json,write_json

ROOT=Path(__file__).resolve().parents[2]
ROBOTS=('Alpha','Bob','Carol')

def archive(work,output,retire=False):
    report=read_json(output/'report.json')
    if not (work/'swarm/summary.json').exists():raise ValueError('Native replay is still active')
    for name in ('REPORT.md','trajectories.png','trajectories.rrd','swarm/summary.json'):
        if not (output/name).is_file():raise ValueError('Missing report artifact: '+name)
    subprocess.run([str(ROOT/'.ros2/rerun-venv/bin/rerun'),'rrd','verify',str(output/'trajectories.rrd')],check=True)
    provenance=output/'provenance';provenance.mkdir(exist_ok=True)
    shutil.copytree(ROOT/'.ros2/swarm/provenance',provenance/'setup',dirs_exist_ok=True)
    shutil.copytree(ROOT/'Swarm-SLAM/s3e',provenance/'adapter',dirs_exist_ok=True,
                    ignore=shutil.ignore_patterns('__pycache__','.pytest_cache'))
    for name in ('execution-policy.json','tests.log','our-regression.log','build.log',
                 'registration-direction-check.log','evaluation-smoke.log','report-smoke.log'):
        p=ROOT/'.ros2/swarm'/name
        if p.is_file():shutil.copy2(p,provenance/name)
    for name in ('comparison-failure.json','ours.log','swarm.log'):
        p=work/name
        if p.is_file():shutil.copy2(p,provenance/name)
    if (work/'basis-failure').exists():shutil.copytree(work/'basis-failure',provenance/'basis-failure',dirs_exist_ok=True)
    for p in (work/'frontend').glob('*.json'):shutil.copy2(p,provenance/p.name)
    for p in (work/'frontend').glob('*.yaml'):shutil.copy2(p,provenance/p.name)
    hashes={}
    for robot in ROBOTS:
        source=work/f'frontend/{robot}';target=provenance/robot;target.mkdir(exist_ok=True)
        for name in ('summary.json','mapping_config.yaml','runtime.yaml','mapping.log','playback.log'):
            if (source/name).is_file():shutil.copy2(source/name,target/name)
        for name in ('manifest.json','frames.jsonl'):
            p=source/'export'/name;hashes[f'{robot}/{name}']=file_hash(p)
            with p.open('rb') as src,gzip.open(target/(name+'.gz'),'wb') as dst:shutil.copyfileobj(src,dst)
        p=Path(report['dataset'])/f'{robot.lower()}_gt.txt'
        hashes[f'{robot}/ground_truth']=file_hash(p)
    write_json(provenance/'input-sha256.json',hashes)
    native_logs=work/'swarm/native-logs'
    if native_logs.is_dir():
        with tarfile.open(output/'swarm/native-logs.tar.gz','w:gz') as tf:tf.add(native_logs,arcname='native-logs')
        # Read the compressed stream through EOF before retiring its source.
        with gzip.open(output/'swarm/native-logs.tar.gz','rb') as stream:
            while stream.read(1024**2):pass
    if 'failure' not in report['ours']:
        from s3e_pipeline.ellipsoid_full_archive import archive as archive_ours
        archive_ours(work/'frontend',output/'ours',retire)
    elif retire:
        # Retain the preparation failure; no successful BEV cache is fabricated.
        shutil.copy2(work/'frontend/prepare-Alpha.log',provenance/'prepare-Alpha-failure.log')
        paths=[work/f'frontend/{r}/export' for r in ROBOTS]+[work/'frontend/stages']
        entries=[]
        for p in paths:
            if p.is_symlink() or not p.resolve().is_relative_to(work.resolve()):raise ValueError('Unsafe cache path')
            entries.append(dict(path=str(p),bytes=sum(f.stat().st_size for f in p.rglob('*') if f.is_file())))
        write_json(output/'cleanup.json',dict(complete=False,retired=entries))
        for p in paths:shutil.rmtree(p)
        write_json(output/'cleanup.json',dict(complete=True,retired=entries,total_bytes=sum(e['bytes'] for e in entries)))
        write_json(work/'frontend/RETIRED.json',dict(archive=str(output),status='native results retained; BEV preparation failure retained'))
    if retire and native_logs.is_dir():shutil.rmtree(native_logs)
    write_json(output/'files.json',{str(p.relative_to(output)):dict(bytes=p.stat().st_size,sha256=file_hash(p))
               for p in sorted(output.rglob('*')) if p.is_file() and p.name!='files.json'})
    print('Native messages, evo evidence, figures, Rerun, inputs and failure diagnostics archived.',flush=True)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--work',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--retire-large',action='store_true');a=p.parse_args();archive(a.work.resolve(),a.output.resolve(),a.retire_large)
