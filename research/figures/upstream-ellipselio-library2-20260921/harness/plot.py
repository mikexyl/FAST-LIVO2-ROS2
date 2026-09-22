import json,sys
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from evo.tools import file_interface
trial=Path(sys.argv[1]);out=Path(sys.argv[2]);out.mkdir(parents=True,exist_ok=True)
evaldir=trial/'evaluation';metrics=json.loads((evaldir/'metrics.json').read_text())
rows=[json.loads(s) for s in (trial/'frontend/native_updates.jsonl').read_text().splitlines()]
ref=np.loadtxt(evaldir/'evo/Bob-reference-matched.tum');est=np.loadtxt(evaldir/'evo/Bob-estimate-aligned.tum')
result=file_interface.load_res_file(evaldir/'evo/Bob-ape.zip')
error=result.np_arrays['error_array'];pose=np.array([r['pose'] for r in rows]);stamp=np.array([r['stamp_ns'] for r in rows],dtype=np.int64)
t=(stamp-stamp[0])*1e-9;et=est[:,0]-stamp[0]*1e-9
fig,ax=plt.subplots(2,2,figsize=(12,8),layout='constrained')
origin=ref[0,1:4]
ax[0,0].plot(ref[:,1]-origin[0],ref[:,2]-origin[1],color='black',lw=1.4,label='Ground truth')
ax[0,0].plot(est[:,1]-origin[0],est[:,2]-origin[1],color='#1675B8',lw=1,label='Upstream persistent map')
ax[0,0].set(xlabel='x from initial GT position [m]',ylabel='y from initial GT position [m]',title=f'Aligned trajectory; ATE RMSE {metrics["ate_rmse_m"]:.3f} m');ax[0,0].axis('equal');ax[0,0].legend(fontsize=8)
ax[0,1].plot(et,error,color='#1675B8',lw=.8);ax[0,1].set(xlabel='Time from first native pose [s]',ylabel='Position APE [m]',title='evo 1.36.5; SE(3), 50 ms, scale fixed')
speed=np.linalg.norm(np.diff(pose[:,:3],axis=0),axis=1)/(np.diff(stamp)*1e-9)
ax[1,0].plot(t[1:],speed,color='#067849',lw=.7);ax[1,0].set(xlabel='Time [s]',ylabel='Estimated speed [m/s]',title=f'Maximum {metrics["max_speed_m_s"]:.2f} m/s')
ax[1,1].plot(t,[r['processing_s']*1000 for r in rows],color='#8251A8',lw=.65);ax[1,1].set(xlabel='Time [s]',ylabel='Processing time [ms]',title=f'Median {metrics["processing_median_ms"]:.1f} ms; p95 {metrics["processing_p95_ms"]:.1f} ms')
for a in ax.flat:a.grid(alpha=.2)
for a in [ax[0,1],ax[1,0],ax[1,1]]:a.axvline(429.1926,color='#BC6B28',ls='--',lw=1,label='Historical divergence time')
ax[0,1].legend(fontsize=8,loc='lower left')
fig.suptitle('S3E Library 2 / Bob — upstream EllipseLIO 6506f46',fontsize=14)
fig.savefig(out/'upstream-bob.png',dpi=180)
fig.savefig(out/'upstream-bob.pdf')
