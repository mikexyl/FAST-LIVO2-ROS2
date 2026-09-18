#!/usr/bin/env python3
"""Freeze a paired BEV experiment from saved sensor-only odometry schedules."""
import argparse
from decimal import Decimal
from pathlib import Path
import shutil
import sys
import numpy as np
from scipy.spatial.transform import Rotation
import yaml

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'FAST-LIVO2-ROS2/research'))
from s3e_pipeline.artifacts import file_hash,write_json
from s3e_pipeline.data import select
from s3e_pipeline.ellipsoid_full import config,ROBOTS


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--dataset',type=Path,required=True)
    p.add_argument('--work',type=Path,required=True)
    p.add_argument('--schedule-trajectories',type=Path,required=True)
    p.add_argument('--schedule-frontend',required=True)
    p.add_argument('--mapping-templates',type=Path,required=True)
    args=p.parse_args()
    work=args.work.resolve();dataset=args.dataset.resolve()
    if not (dataset/'metadata.yaml').exists():raise ValueError('Missing ROS2 bag metadata')
    cfg=config();cfg.update(dataset=str(dataset),output_root=str(work))
    schedules={};sources={};configs={}
    for robot in ROBOTS:
        trajectory=args.schedule_trajectories/robot/'raw.tum';last=None;selected=[]
        for line in trajectory.read_text().splitlines():
            if not line.strip() or line.startswith('#'):continue
            f=line.split();T=np.eye(4);T[:3,3]=list(map(float,f[1:4]));T[:3,:3]=Rotation.from_quat(list(map(float,f[4:8]))).as_matrix()
            row=dict(stamp_ns=int(Decimal(f[0])*10**9),T_world_body=T.tolist())
            if select(row,last,cfg['keyframes']):
                selected.append(dict(keyframe_id=len(selected),stamp_ns=row['stamp_ns']));last=row
        if not selected:raise ValueError(f'No schedule for {robot}')
        sources[robot]=dict(path=str(trajectory.resolve()),sha256=file_hash(trajectory))
        schedules[robot]=selected
        mapping=yaml.safe_load((args.mapping_templates/f'{robot}.yaml').read_text())
        mapping['/**']['ros__parameters']['research']=dict(ellipsoid_stamps_ns=[r['stamp_ns'] for r in selected],
            ellipsoid_range_m=80.,ellipsoid_world_frame=True,ellipsoid_delta_export=True)
        configs[robot]=mapping
    work.mkdir(parents=True,exist_ok=False)
    (work/'config.yaml').write_text(yaml.safe_dump(cfg,sort_keys=False))
    for robot,mapping in configs.items():
        (work/f'{robot}.yaml').write_text(yaml.safe_dump(mapping,sort_keys=False))
    write_json(work/'schedule.json',dict(schedule=schedules,sources=sources,
        source_frontend=args.schedule_frontend,selection=cfg['keyframes'],ground_truth_used=False,
        note='Frozen from prior raw odometry; both BEV branches use identical fresh EllipseLIO exports at these times.'))
    provenance=work/'provenance';provenance.mkdir()
    shutil.copy2(__file__,provenance/Path(__file__).name)
    shutil.copy2(work/'config.yaml',provenance/'config.yaml')
    shutil.copy2(dataset/'metadata.yaml',provenance/'bag-metadata.yaml')
    write_json(provenance/'bag-hashes.json',{p.name:file_hash(p) for p in sorted(dataset.glob('*.db3'))})
    for robot in ROBOTS:shutil.copy2(args.schedule_trajectories/robot/'raw.tum',provenance/f'{robot}-schedule-source.tum')
    print({r:len(v) for r,v in schedules.items()},flush=True)


if __name__=='__main__':main()
