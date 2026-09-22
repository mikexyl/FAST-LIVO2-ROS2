from pathlib import Path
import json,yaml
B=Path(__file__).resolve().parent;W=B/'full'
read=lambda p:json.loads(p.read_text())
a=read(W/'frontend-status.json');b=read(W/'extra-frontend-status.json')
assert a['phase']=='pending_or_failed' and b['phase']=='complete'
a['robots'].update(b['robots']);assert len(a['robots'])==14 and all(v['status']=='complete' for v in a['robots'].values())
a.update(phase='complete',capture_groups=[dict(robots=11,workers=4),dict(robots=3,workers=3)],resource_quotas=False)
(W/'frontend-status.json').write_text(json.dumps(a,indent=2))
for n in ['inputs.json','calibration-hashes.json']:
 d=read(W/n);d.update(read(W/('extra-'+n)));assert len(d)==14;(W/n).write_text(json.dumps(d,indent=2))
(W/'frontend-gate.json').write_text(json.dumps(dict(passed=True,quality={r:v['quality'] for r,v in a['robots'].items()}),indent=2))
p=W/'config.yaml';cfg=yaml.safe_load(p.read_text());cfg['odometry'].pop('imu_noise',None)
cfg['odometry']['imu_noise_by_robot']={r:{k:yaml.safe_load((W/'configs'/f'{r}.yaml').read_text())['/**']['ros__parameters']['imu'][k]
 for k in ['acc_noise','gyr_noise','acc_bias','gyr_bias']} for r in cfg['robots']}
p.write_text(yaml.safe_dump(cfg,sort_keys=False))
print('Fourteen complete, finite chronological captures passed the sensor-only stability gate.')
