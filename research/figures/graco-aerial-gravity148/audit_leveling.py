import json
from pathlib import Path
import zlib
import hashlib
import numpy as np
from s3e_pipeline.backends import unpack_array

BASE=Path('/workspace/.ros2/graco-aerial-gravity148-20260920')
WORK=BASE/'full'
OLD=Path('/workspace/.ros2/graco-aerial-four148-20260920/full')
results={}
for robot in ('aerial05','aerial06','aerial07','aerial08'):
    sidecar=json.loads((WORK/f'gravity/{robot}.json').read_text())
    angles=[];vertical_leaks=[];features=[];before_features=[]
    for path in sorted((WORK/f'prepared-{robot}/ellipsoid').glob('*.json.zlib')):
        desc=json.loads(zlib.decompress(path.read_bytes()))
        before=json.loads(zlib.decompress((OLD/f'prepared-{robot}/ellipsoid'/path.name).read_bytes()))
        gravity=np.array(sidecar['submaps'][str(desc['submap_id'])]['gravity_imu_m_s2'])
        up=-gravity/np.linalg.norm(gravity)
        R=unpack_array(desc['mapclosures']['ground'])[:3,:3]
        old_R=unpack_array(before['mapclosures']['ground'])[:3,:3]
        np.testing.assert_allclose(R@up,[0,0,1],atol=1e-12)
        angles.append(float(np.degrees(np.arccos(np.clip(abs(old_R[2]@up),0,1)))))
        vertical_leaks.append(float(np.linalg.norm((R@up)[:2])))
        features.append(desc['mapclosures_features']);before_features.append(before['mapclosures_features'])
    results[robot]=dict(submaps=len(features),old_plane_tilt_deg=dict(min=min(angles),max=max(angles),submap8=angles[8]),
        max_horizontal_projection_of_unit_up=max(vertical_leaks),
        features_before=dict(total=sum(before_features),median=float(np.median(before_features)),submap8=before_features[8]),
        features_after=dict(total=sum(features),median=float(np.median(features)),submap8=features[8]),
        startup_acceleration_std_m_s2=sidecar['acceleration_std_m_s2'],
        startup_half_direction_difference_deg=sidecar['startup_half_direction_difference_deg'])
out=dict(ground_truth_used=False,projection='orthographic, aligned to reconstructed startup IMU gravity',
         limitation='Historical per-anchor filter gravity and absolute terrain elevation were not recorded.',
         robots=results,script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
(WORK/'projection-audit.json').write_text(json.dumps(out,indent=2)+'\n')
print(json.dumps(out,indent=2))
