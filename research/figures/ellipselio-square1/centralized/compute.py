"""Reproduce centralized pose-factor PGO from the retained EllipseLIO archive."""
from decimal import Decimal
from pathlib import Path
import sys
import time

import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0,str(Path(__file__).resolve().parents[3]))
from s3e_pipeline.artifacts import read_json,read_jsonl,write_json,file_hash
from s3e_pipeline.pgo import optimize
from s3e_pipeline.evaluation import corrected_dense_rows,ground_truth,save_tum
from s3e_pipeline.evo_evaluation import evaluate


def main():
    output=Path(__file__).resolve().parent;archive=output.parent
    sources={}
    def source(path):
        path=Path(path);sources[str(path)]=file_hash(path);return path
    run=read_json(source(archive/'run.json'));cfg=run['config'];robots=cfg['robots']
    keys={r:read_jsonl(source(archive/r/'keyframes.jsonl')) for r in robots}
    loops=read_jsonl(source(archive/'dpgo/constraints.jsonl'))
    for name in ('pgo.py','geometry.py','evaluation.py','evo_evaluation.py','artifacts.py'):
        source(Path(__file__).resolve().parents[3]/'s3e_pipeline'/name)
    source(__file__)
    start=time.monotonic()
    graph=optimize([x for r in robots for x in keys[r]],loops,cfg['pgo'])
    optimization_s=time.monotonic()-start
    write_json(output/'graph.json',graph)
    raw={}
    for robot in robots:
        raw[robot]=[]
        for line in source(archive/robot/'raw.tum').read_text().splitlines():
            if not line.strip() or line.startswith('#'):continue
            fields=line.split();values=np.array([float(x) for x in fields[1:]])
            T=np.eye(4);T[:3,3]=values[:3];T[:3,:3]=Rotation.from_quat(values[3:]).as_matrix()
            raw[robot].append(dict(robot_id=robot,frame_id=len(raw[robot]),
                stamp_ns=int(Decimal(fields[0])*10**9),T_world_body=T.tolist()))
    def dense(poses,save=False):
        result=[]
        for robot in robots:
            selected=[p for p in poses if p['robot_id']==robot]
            rows=[dict(p,component=selected[0]['component'])
                for p in corrected_dense_rows(raw[robot],keys[robot],selected)]
            if save:save_tum(output/f'{robot}-corrected.tum',rows)
            result.extend(rows)
        return result
    # Ground truth is introduced only after optimization.
    gt={r:ground_truth(source(Path(cfg['dataset'])/(r.lower()+'_gt.txt'))) for r in robots}
    metrics=evaluate(dict(poses=dense(graph['poses'],True)),gt,cfg['evaluation'],output/'evo')
    prior=read_json(source(archive/'report.json'))
    reconstructed=evaluate(dict(poses=dense(read_jsonl(source(archive/'dpgo/poses.jsonl')))),gt,cfg['evaluation'])
    for component,old in prior['trajectory'].items():
        new=reconstructed[component]
        assert new['samples']==old['samples'] and abs(new['rmse_m']-old['rmse_m'])<1e-7
        for robot in old['per_robot']:
            assert abs(new['per_robot'][robot]['statistics']['rmse']-old['per_robot'][robot]['statistics']['rmse'])<1e-7
    report=dict(trajectory=metrics,optimization_s=optimization_s,configuration=cfg['pgo'],
        method=graph['optimization_method'],factor_scope='odometry and PCM-retained loop pose factors; no live point-cloud factors',
        input_loops=len(loops),selected_loops=sum(f['kind']=='loop' and f['solver_weight']>0 for f in graph['factors']),
        components=graph['components'],factors=len(graph['factors']),
        reconstruction_check='retained TUM precision reproduces saved CBS ATE and sample counts within 1e-7 m',
        comparison_limitation='Earlier CBS includes live GICP factors and uses a different optimization procedure; not an identical-objective solver comparison',
        sources=sources,inputs_unchanged=all(file_hash(p)==h for p,h in sources.items()))
    assert report['inputs_unchanged']
    write_json(output/'report.json',report)
    for component,m in metrics.items():
        print(dict(component=component,combined_ATE=m['rmse_m'],samples=m['samples'],
            per_robot={r:v['statistics']['rmse'] for r,v in m['per_robot'].items()},
            optimization_s=optimization_s,selected_loops=report['selected_loops']))


if __name__=='__main__':main()
