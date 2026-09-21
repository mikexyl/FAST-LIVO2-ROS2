import hashlib
import json
from pathlib import Path
import zlib
import cv2
import numpy as np
import s3e_mapclosures_inspection as native
from s3e_pipeline.backends import unpack_array
from s3e_pipeline.ellipsoid_bev import normalize_exported_basis
from s3e_pipeline.ellipsoid_cuda import SurfaceSampler
from s3e_pipeline.mapclosures_inspection import check_features

base=Path(__file__).resolve().parent
work=base/'verification';source=base/'trials/aerial05-spatial-smoke/frontend/submaps'
out=work/'bevs';out.mkdir(exist_ok=False)
keys=[json.loads(s) for s in (work/'prepared-aerial05/store/keyframes.jsonl').read_text().splitlines()]
sampler=SurfaceSampler();inspector=native.Inspector(.5,.05,50);maps=[]
try:
    for key in keys:
        k=key['keyframe_id']
        packet=json.loads(zlib.decompress((work/f'prepared-aerial05/ellipsoid/{k:06d}.json.zlib').read_bytes()))
        features={k:unpack_array(v) for k,v in packet['mapclosures'].items()}
        with np.load(source/key['payload']) as data:e=data['ellipsoids']
        basis,_=normalize_exported_basis(e[:,6:].reshape(-1,3,3),.001)
        points,_=sampler.render(e[:,:3],e[:,3:6],basis)
        debug=inspector.density(points,features['ground']);check_features(debug,features)
        image=out/f'{k:06d}.png';assert cv2.imwrite(str(image),debug['image'])
        maps.append(dict(key=k,png=image.name,sha256=hashlib.sha256(image.read_bytes()).hexdigest(),
            features=len(features['xy']),width=int(debug['image'].shape[1]),height=int(debug['image'].shape[0]),
            elapsed_s=(key['last_member_ns']-key['begin_ns'])/1e9,extent_m=key['extent_m'],
            payload_sha256=key['sha256'],member_scans=len(key['member_scan_ids'])))
finally:sampler.close()
(out/'manifest.json').write_text(json.dumps(dict(maps=maps,exact_cached_features=True,ground_truth_used=False,
    projection='actual native per-anchor filter gravity',resolution_m=.5),indent=2)+'\n')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
fig,axes=plt.subplots(2,3,figsize=(13,10),layout='constrained')
for ax,m in zip(axes.flat,maps):
    im=cv2.imread(str(out/m['png']),cv2.IMREAD_GRAYSCALE)
    ax.imshow(im,cmap='gray',vmin=0,vmax=255,interpolation='nearest')
    ax.set_title(f"Submap {m['key']} · {m['elapsed_s']:.1f} s · {m['extent_m']:.1f} m extent\n"
                 f"{m['width']*.5:.1f} × {m['height']*.5:.1f} m footprint · {m['features']} ORB features")
    ax.axis('off')
fig.suptitle('Aerial 05 · spatial submaps · gravity-horizontal ellipsoid BEVs',fontsize=17)
fig.savefig(out/'overview.png',dpi=160);plt.close(fig)
print(json.dumps(maps,indent=2))
