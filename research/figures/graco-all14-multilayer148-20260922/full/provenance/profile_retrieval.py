from pathlib import Path
import json,zlib,time,yaml
from s3e_pipeline.multilayer_mapclosures import MultilayerMatcher
B=Path(__file__).resolve().parent;W=B/'full';cfg=yaml.safe_load((W/'config.yaml').read_text())
def d(r,k):return json.loads(zlib.decompress((W/f'prepared-{r}/ellipsoid/{k:06d}.json.zlib').read_bytes()))
m=MultilayerMatcher(cfg['backend']['mapclosures']);keys=list(range(34));s=time.monotonic()
for k in keys:m.add(k,d('ground01',k))
print('index_s',time.monotonic()-s,flush=True)
s=time.monotonic();h=m.query(d('aerial07',27),keys)
print('query_s',time.monotonic()-s,[(x['keyframe_id'],x['matches'],x['inliers']) for x in h],flush=True)
