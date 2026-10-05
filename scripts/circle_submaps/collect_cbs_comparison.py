#!/usr/bin/env python3
"""Copy compact verified results; keep geometry and reconstructed maps on 148."""
import argparse
import csv
from pathlib import Path
import shutil
import sys
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'research'))
from s3e_pipeline.artifacts import file_hash, read_json, read_jsonl, write_json


def main():
    p=argparse.ArgumentParser();p.add_argument('--runs',type=Path,required=True)
    p.add_argument('--central',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False);entries={}
    for mode in ('circle40','trajectory40','trajectory80'):
        run=a.runs/mode;out=a.output/mode;out.mkdir();read_json(run/'COMPLETE.json')
        entry={k:read_json(run/path) for k,path in dict(summary='cbs/summary.json',
            registration='cbs/registration.json',resources='resources.json',communication='communication.json',
            evidence='evidence-comparison.json',evaluation='evaluation/result.json').items()}
        entry['published_loop_residuals']=read_json(run/'central/graph.json')['published_loop_residuals']
        central=a.central/mode
        entry['central']={k:read_json(central/path) for k,path in dict(summary='central/summary.json',
            resources='resources.json',evaluation='evaluation/result.json').items()}
        gpu=[float(line.split(',')[1]) for line in (run/'gpu.csv').read_text().splitlines() if len(line.split(','))==3]
        entry['resources']['whole_device_peak_gpu_mib']=max(gpu) if gpu else None
        entry['native_timing']={};entry['input']=str(run)
        for name in ('COMPLETE.json','configuration.json','source-hashes.json','input-hashes.json','native-hashes.json',
            'resources.json','communication.json','evidence-comparison.json','gpu.csv'):
            shutil.copy2(run/name,out/name)
        for name in ('summary.json','registration.json','pcm.json','constraints.jsonl','poses.jsonl'):
            shutil.copy2(run/'cbs'/name,out/name)
        shutil.copy2(run/'evaluation/result.json',out/'evaluation.json')
        shutil.copy2(run/'central/graph.json',out/'published-submap-graph.json')
        for robot in entry['summary']['robots']:
            dest=out/robot;dest.mkdir()
            shutil.copy2(run/'evaluation'/f'{robot}-corrected.tum',dest/'corrected.tum')
            shutil.copy2(run/'cbs'/robot/'stats.jsonl',dest/'stats.jsonl')
            for name in ('manifest.json','transport.json'):
                shutil.copy2(run/'cbs'/robot/'registration'/name,dest/name)
            timing={}
            for pattern,column,label in [('timing_robot_*.csv','update_ms','timer_update'),
                ('belief_service_timing_robot_*.csv','get_beliefs_ms','belief_marginalization')]:
                path=next((run/'cbs'/robot/'native'/robot/'cbs_online').glob(pattern))
                shutil.copy2(path,dest/path.name)
                with path.open() as f:records=list(csv.DictReader(f))
                x=np.asarray([float(r[column]) for r in records])
                timing[label]=dict(count=len(x),sum_s=float(x.sum()/1000),median_ms=float(np.median(x)),
                    p95_ms=float(np.percentile(x,95)),max_ms=float(x.max()))
            entry['native_timing'][robot]=timing
        entries[mode]=entry
    write_json(a.output/'comparison.json',entries)
    root=a.runs.parent;failure={}
    repeat=root/'circle40-bootstrap-repeat'
    if (repeat/'COMPLETE.json').exists():
        write_json(a.output/'circle40-repeat.json',dict(input=str(repeat),**{
            k:read_json(repeat/path) for k,path in dict(
                complete='COMPLETE.json',native_hashes='native-hashes.json',
                summary='cbs/summary.json',registration='cbs/registration.json',
                resources='resources.json',communication='communication.json',
                evidence='evidence-comparison.json',evaluation='evaluation/result.json',
                graph='central/graph.json').items()}))
    for name in ('circle40','circle40-shared'):
        session=root/name/'cbs/session';statuses={}
        for robot in entries['circle40']['summary']['robots']:
            statuses[robot]=read_json(session/robot/'estimate.json')['status']
        failure[name]=dict(statuses=statuses,log=(root/f'{name}.log').read_text())
    write_json(a.output/'persistent-failure.json',failure)
    fresh=root/'matched-fresh/circle40/cbs'
    if fresh.exists():
        write_json(a.output/'sealed-before-core-fix.json',dict(
            statuses={r:read_json(fresh/r/'summary.json') for r in entries['circle40']['summary']['robots']},
            log=(root/'matched-fresh.log').read_text(),
            reason='message-order dependent dropped anchor before its connecting separator; repaired with receiver-aware bootstrap refresh'))
    rejected=root/'matched-corefixed'
    if rejected.exists():
        write_json(a.output/'rejected-anchor-retention.json',dict(
            retained_implementation=False,
            reason='Retaining early anchors altered alignment and failed final geometry in trajectory80; change reverted.',
            log=(root/'matched-corefixed.log').read_text(),
            completed_evaluations={m:read_json(rejected/m/'evaluation/result.json') for m in entries
                if (rejected/m/'evaluation/result.json').exists()}))
    write_json(a.output/'FILES.sha256.json',{str(p.relative_to(a.output)):file_hash(p) for p in sorted(a.output.rglob('*')) if p.is_file()})


if __name__=='__main__':main()
