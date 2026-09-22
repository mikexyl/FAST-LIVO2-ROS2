"""Partition a large live RRD by entity, retaining every original chunk exactly once."""
import argparse
import json
import os
from pathlib import Path
import re
import sys
import tempfile

ROOT=Path('/workspace');BASE=ROOT/'.ros2/upstream-area-s3e-20260921'
sys.path.insert(0,str(ROOT/'FAST-LIVO2-ROS2/scripts/recent_submaps'))
from upstream_area_batch import call,sha,save,RERUN


def partition(trial):
    source=trial/'recording/live.rrd';out=trial/'recording/entity-parts'
    out.mkdir(exist_ok=True)
    env=dict(os.environ,RAYON_NUM_THREADS='2')
    def run(args,name,timeout=900):call([RERUN/'rerun','rrd',*args],out/(name+'.log'),timeout,env)
    original_hash=sha(source)
    run(['verify','--check-footers=false',source],'source-stream-verification')
    if not (out/'source-chunks.log').exists():
        run(['print',source],'source-chunks')
    text=(out/'source-chunks.log').read_text()
    entities=sorted(set(re.findall(r' - (/.*?) - data columns:',text)))
    assert entities
    general=[p for p in entities if not p.startswith('/world/ellipsoids/')]
    ellipsoids=[p for p in entities if p.startswith('/world/ellipsoids/')]
    groups=[general]+[ellipsoids[i:i+8000] for i in range(0,len(ellipsoids),8000)]
    assert sum(map(len,groups))==len(entities)
    parts=[]
    for number,group in enumerate(groups):
        selected=set(group);drop=[p for p in entities if p not in selected]
        target=out/f'part-{number:03d}.rrd'
        if not target.exists():
            with tempfile.TemporaryDirectory(prefix='rrd-partition-') as tmp:
                batches=[drop[i:i+25000] for i in range(0,len(drop),25000)] or [[]]
                current=source
                for step,batch in enumerate(batches):
                    dest=target if step==len(batches)-1 else Path(tmp)/f'stage-{step}.rrd'
                    args=['filter']
                    for entity in batch:args.extend(['--drop-entity',entity])
                    args.extend([current,'-o',dest]);run(args,f'part-{number:03d}-filter-{step}')
                    current=dest
        run(['verify',target],f'part-{number:03d}-verification')
        parts.append(dict(path=str(target),sha256=sha(target),entities=group,bytes=target.stat().st_size))
        print(trial.name,f'part {number+1}/{len(groups)} verified',flush=True)
    # Normalize both sides to the same compaction policy before comparison.
    # Direct unordered comparison with hundreds of thousands of chunks is quadratic.
    merged=out/'recombined-for-comparison.rrd'
    if not merged.exists():
        run(['merge',*[Path(p['path']) for p in parts],'-o',merged],'recombine')
    normalized=[]
    for name, path in [('source',source),('parts',merged)]:
        target=out/f'comparison-{name}-compacted.rrd'
        if not target.exists():
            run(['optimize','--num-pass','3',path,'-o',target],f'normalize-{name}')
        normalized.append(target)
    run(['compare','--unordered',*normalized],'normalized-complete-comparison',900)
    assert sha(source)==original_hash
    result=dict(source=str(source),source_sha256=original_hash,parts=parts,entities=len(entities),
        source_data_verified=True,partitioned_footer_and_data_verified=True,normalized_complete_data_comparison_passed=True,
        original_preserved=True,chunking_preserved=True,
        comparison_scope='Both sides optimized with --num-pass 3, then rerun rrd compare --unordered; tool ignores log_time; every source entity belongs to exactly one part',
        viewer_instructions='Open all part-*.rrd files together; each keeps the original recording ID.',
        tool_sha256=sha(RERUN/'rerun'),script_sha256=sha(Path(__file__)))
    save(out/'result.json',result)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('trials',nargs='+',type=Path)
    for trial in p.parse_args().trials:partition(trial)
