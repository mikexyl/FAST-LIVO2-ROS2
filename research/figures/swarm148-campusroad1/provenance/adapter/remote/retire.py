#!/usr/bin/env python3
"""Retire only completed generated MCAPs after preserving exact compact evidence."""
import argparse
import gzip
import hashlib
import json
from pathlib import Path

ROOT=Path(__file__).resolve().parents[3]
def digest(path):
    h=hashlib.sha256()
    with path.open('rb') as f:
        for block in iter(lambda:f.read(2**20),b''):h.update(block)
    return h.hexdigest()


def main():
    p=argparse.ArgumentParser();p.add_argument('sequence',choices=['library1','campusroad1','smoke'])
    p.add_argument('--apply',action='store_true');args=p.parse_args()
    work=ROOT/'.ros2/swarm148'/args.sequence
    report=ROOT/f'FAST-LIVO2-ROS2/research/figures/swarm148-{args.sequence}'
    smoke=args.sequence=='smoke'
    data=json.loads((report/'report.json').read_text()) if not smoke else {}
    failure=bool(data.get('frontend_failure'))
    required=(('REPORT.md','smoke.png','smoke.pdf','smoke.rrd','native/summary.json') if smoke else
              ('REPORT.md','report.json','Alpha-divergence.png','Alpha-divergence.pdf') if failure else
              ('REPORT.md','trajectories.png','trajectories.pdf','trajectories.rrd','report.json','swarm/summary.json'))
    for name in required:
        if not (report/name).is_file():raise ValueError(f'Missing retained artifact: {name}')
    if failure:
        disposition=json.loads((work/'pipeline-disposition.json').read_text())
        if disposition['failure_stage']!='frontend':raise ValueError('Unconfirmed failed attempt')
        robots=[r for r in ('Alpha','Bob','Carol') if (work/f'frontend/{r}/export/sensors.mcap').exists()]
        for robot in robots:
            if json.loads((work/f'frontend/{robot}/summary.json').read_text())!=json.loads((report/f'provenance/{robot}/summary.json').read_text()):
                raise ValueError('Archived frontend summary differs from source')
    else:
        summary=json.loads((work/'swarm/summary.json').read_text())
        if summary!=json.loads((report/('native' if smoke else 'swarm')/'summary.json').read_text()):
            raise ValueError('Archived native summary differs from completed run')
        robots=('Alpha',) if smoke else ('Alpha','Bob','Carol')
    entries=[]
    for robot in robots:
        export=work/f'frontend/{robot}/export';evidence=report/f'provenance/{robot}'
        interrupted=failure and not (export/'manifest.json').exists()
        if interrupted and json.loads((export.parent/'summary.json').read_text()).get('success'):
            raise ValueError('Successful capture is unexpectedly missing its manifest')
        for name in ('manifest.json','frames.jsonl'):
            if name=='manifest.json' and interrupted:continue
            with gzip.open(evidence/(name+'.gz'),'rb') as stream:archived=stream.read()
            if hashlib.sha256(archived).hexdigest()!=digest(export/name):
                raise ValueError(f'Archived {robot}/{name} differs from source')
        cloud=export/'sensors.mcap'
        if cloud.is_symlink() or not cloud.resolve().is_relative_to(work.resolve()):
            raise ValueError('Unsafe generated-cloud path')
        manifest=json.loads((evidence/'incomplete-export.json' if interrupted else export/'manifest.json').read_text())
        actual=digest(cloud)
        if actual!=manifest['files']['sensors.mcap']:raise ValueError('MCAP hash mismatch')
        entries.append(dict(robot=robot,path=str(cloud),bytes=cloud.stat().st_size,sha256=actual))
    result=dict(complete=False,retired=entries,total_bytes=sum(e['bytes'] for e in entries),
                policy='Only generated sensor MCAPs retired. Exact frame indices, original manifests, poses, constraints, reports, figures and Rerun remain. Original S3E bags are untouched.')
    print(json.dumps(result,indent=2),flush=True)
    if not args.apply:return
    (report/'cleanup.json').write_text(json.dumps(result,indent=2)+'\n')
    for entry in entries:
        path=Path(entry['path']);path.unlink()
        (path.parent/'RETIRED.json').write_text(json.dumps(dict(archive=str(report),retired=entry,
            warning='MCAP retired after validated report; this export is no longer a reusable sensor cache.'),indent=2)+'\n')
    result['complete']=True
    (report/'cleanup.json').write_text(json.dumps(result,indent=2)+'\n')
    print(f'Retired {result["total_bytes"]/2**30:.3f} GiB of generated MCAPs.',flush=True)


if __name__=='__main__':main()
