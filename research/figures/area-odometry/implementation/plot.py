import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

work=Path(__file__).resolve().parent/'comparison'
state=json.loads((work/'status.json').read_text())
assert state['phase']=='complete'
fig,axes=plt.subplots(1,3,figsize=(15,4.3),constrained_layout=True)
colors=['#ad451b','#1978a5','#34954b']
labels=['Recent-map control','Accumulated area','Area repeat']
runtimes=[]
for index,name in enumerate(state['results']):
    rows=[json.loads(line) for line in (work/name/'frontend/native_updates.jsonl').read_text().splitlines()]
    t=(np.array([r['sensor_stamp_ns'] for r in rows],dtype=np.int64)-rows[0]['sensor_stamp_ns'])/1e9
    f=np.array([r['features'] for r in rows]);p=np.array([r['pose'][:3] for r in rows])
    focus=(t>=24)&(t<=35)
    axes[0].plot(t[focus],f[focus],label=labels[index],color=colors[index],linewidth=1)
    failures=focus&np.array([not r['lidar_updated'] for r in rows])
    axes[0].scatter(t[failures],f[failures],marker='x',s=18,color=colors[index])
    axes[1].plot(p[:,0],p[:,1],color=colors[index],linewidth=1.3,label=labels[index])
    runtimes.append(np.array([r['processing_s'] for r in rows])*1000)
axes[0].set(title='Aerial 08: previous failure interval',xlabel='Sensor time since initialization (s)',ylabel='Eligible correspondences')
axes[0].set_yscale('symlog',linthresh=10);axes[0].legend(fontsize=8)
axes[1].set(title='Raw odometry; no ground truth',xlabel='Odometry X (m)',ylabel='Odometry Y (m)');axes[1].axis('equal')
axes[2].boxplot(runtimes,tick_labels=labels[:len(runtimes)],showfliers=False)
axes[2].set(title='Per-scan computation (whiskers: 1.5 IQR)',ylabel='Wall time (ms)');axes[2].tick_params(axis='x',rotation=20)
for ax in axes:ax.grid(alpha=.2)
fig.savefig(work/'comparison.png',dpi=180)
print(work/'comparison.png')
