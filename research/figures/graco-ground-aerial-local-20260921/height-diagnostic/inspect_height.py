"""Offline diagnostic only: run existing height/GICP stages on frozen weak seeds."""
from pathlib import Path
import hashlib,json,shutil,time,zlib
import numpy as np
import yaml
from s3e_pipeline.backends import unpack_array
from s3e_pipeline.registration import bounded_cloud,refine
from s3e_pipeline.vertical_initialization import initialize_vertical

B=Path(__file__).resolve().parent;W=B/'source/output'
O=B.parents[1]/'FAST-LIVO2-ROS2/research/figures/graco-ground-aerial-local-20260921/height-diagnostic'
O.mkdir(exist_ok=False)
cfg=yaml.safe_load((W/'config.yaml').read_text())['backend']
hashes={};results=[];start=time.monotonic()
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def payload(r,k):
    geometry=W/f'prepared-{r}/store/{k:06d}.npz';descriptor=W/f'prepared-{r}/ellipsoid/{k:06d}.json.zlib'
    for p in (geometry,descriptor):hashes[str(p)]=sha(p)
    with np.load(geometry) as d:cloud=d['cloud'].copy()
    cloud,res=bounded_cloud(cloud,cfg['evidence']['voxel_m'],None,None)
    packet=json.loads(zlib.decompress(descriptor.read_bytes()))
    G=unpack_array(packet['mapclosures']['ground'])
    return cloud,G
for q,c in [(27,17),(17,7)]:
    target,Gq=payload('aerial06',q);source,Gc=payload('ground06',c)
    p=O.parent/f'viewpoint-diagnostic/aerial06-{q}-ground06-{c}-native.npz';hashes[str(p)]=sha(p)
    with np.load(p) as data:seed=data['T_i_j'].copy()
    row=dict(query=['aerial06',q],candidate=['ground06',c],original_retrieval_eligible=False,
        descriptor_inliers=5,diagnostic_only=True,geometry_preprocessing='unchanged fixed 0.4 m evidence voxels; no range or point-count crop')
    row['without_height']=refine(target,source,seed,cfg['registration'])
    T,diagnostic=initialize_vertical(target,source,seed,Gq,Gc,cfg['mapclosures']['vertical_initialization'])
    row['vertical_initialization']=diagnostic
    row['with_height']=refine(target,source,T,cfg['registration'])
    row['height_seed_T_i_j']=T.tolist()
    results.append(row)
    (O/f'aerial06-{q}-ground06-{c}.json').write_text(json.dumps(row,indent=2)+'\n')
    print(json.dumps(dict(query=q,candidate=c,height_correction_m=diagnostic['correction_m'],
        without_height={k:row['without_height'].get(k) for k in ('accepted','reason','initial_overlap','overlap','rmse_m')},
        with_height={k:row['with_height'].get(k) for k in ('accepted','reason','initial_overlap','overlap','rmse_m','inliers','condition')})),flush=True)
assert all(sha(Path(p))==h for p,h in hashes.items())
summary=dict(diagnostic_only=True,ground_truth_used=False,original_pipeline_results_unchanged=True,
    unchanged_registration_settings=cfg['registration'],unchanged_height_settings=cfg['mapclosures']['vertical_initialization'],
    original_retrieval_gate_bypassed_only_for_these_saved_diagnostic_pairs=True,
    loops_added_to_graph=0,geometry_and_seeds_sha256=hashes,results=results,wall_s=time.monotonic()-start)
(O/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
shutil.copy2(Path(__file__),O/'inspect_height.py')
print('diagnostic complete',time.monotonic()-start,flush=True)
