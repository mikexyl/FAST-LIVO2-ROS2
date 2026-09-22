"""Fixed validation of the accelerated sampler on retained real map snapshots."""
import argparse,json,sys,time
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from s3e_pipeline.ellipsoid_cuda import SurfaceSampler
from s3e_pipeline.artifacts import write_json


def main():
    import s3e_mapclosures_native as native
    import s3e_mapclosures_inspection as inspection
    p=argparse.ArgumentParser();p.add_argument('--archive',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    args=p.parse_args();sampler=SurfaceSampler();results=[]
    for stem in ['Alpha-000002','Alpha-000026','Bob-000031']:
        with np.load(args.archive/'maps'/f'{stem}-geometry.npz') as f:
            c,a,b=[f[k].astype(float) for k in ('centers','axes','basis')]
        started=time.monotonic();cloud,meta=sampler.render(c,a,b);wall=time.monotonic()-started
        other,_=sampler.render(c,a,b)
        if not np.array_equal(cloud,other):raise ValueError('CUDA sampler is nondeterministic')
        engine=native.MapClosures(.5,.05,50);desc=engine.describe(cloud)
        debug=inspection.Inspector(.5,.05,50).density(cloud,desc['ground'])
        with np.load(args.archive/'maps'/f'{stem}-ellipsoid.npz') as f:
            pixels=(float(np.mean(debug['image']==f['image'])) if debug['image'].shape==f['image'].shape else None)
            record=dict(snapshot=stem,ellipsoids=len(c),cuda_s=wall,**meta,
                orb_original=len(f['cached_xy']),orb_cuda=len(desc['xy']),
                exact_descriptors=np.array_equal(desc['bits'],f['cached_bits']),
                exact_keypoints=np.array_equal(desc['xy'],f['cached_xy']),
                ground_matrix_max_difference=float(np.max(np.abs(desc['ground']-f['ground']))),
                identical_density_pixel_fraction=pixels,deterministic=True)
        results.append(record);print(json.dumps(record),flush=True)
    sampler.close();write_json(args.output,results)


if __name__=='__main__':main()
