from pathlib import Path
import json
import numpy as np
WORK=Path('/workspace/.ros2/graco-aerial-spatial148-20260920/full')
comparison=json.loads((WORK/'comparison.json').read_text())
cfg=dict(robots=list(comparison['robots']))
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
names=list(cfg['robots']);x=np.arange(len(names));w=.36
fig,axes=plt.subplots(1,3,figsize=(16,4.8),layout='constrained')
for ax,label,a,b in [(axes[0],'Raw position ATE RMSE [m]','temporal_raw_ate_m','spatial_raw_ate_m'),
                      (axes[1],'CBS position ATE RMSE [m] — component fits','temporal_cbs_component_fit_ate_m','spatial_cbs_component_fit_ate_m')]:
    aa=[comparison['robots'][r][a] for r in names];bb=[comparison['robots'][r][b] for r in names]
    ax.bar(x-w/2,aa,w,label='Temporal 10 s / 5 s',color='#64748b');ax.bar(x+w/2,bb,w,label='Spatial 40 m / 20 m',color='#0d9488')
    ax.set_title(label);ax.set_xticks(x,names,rotation=15);ax.set_ylim(bottom=0);ax.grid(axis='y',alpha=.2);ax.legend()
    for xx,v in zip(x-w/2,aa):ax.text(xx,v,f'{v:.2f}',ha='center',va='bottom',fontsize=9)
    for xx,v in zip(x+w/2,bb):ax.text(xx,v,f'{v:.2f}',ha='center',va='bottom',fontsize=9)
ax=axes[2]
a=[comparison['robots'][r]['temporal_quality']['max_successful_update_gap_s'] for r in names]
b=[comparison['robots'][r]['spatial_quality']['max_successful_update_gap_s'] for r in names]
ax.bar(x-w/2,a,w,color='#64748b');ax.bar(x+w/2,b,w,color='#0d9488');ax.axhline(1,color='#b91c1c',ls='--',label='Failure threshold')
ax.set_title('Largest successful LiDAR update gap [s]');ax.set_xticks(x,names,rotation=15);ax.grid(axis='y',alpha=.2);ax.legend()
fig.suptitle('GRACO aerial 05–08 on 148 · retained temporal vs fresh spatial run',fontsize=16)
fig.supxlabel('CBS scopes differ: temporal aerial-06/07 share one fit; spatial has four separate fits and zero accepted loops.', fontsize=10)
fig.savefig(WORK/'report/temporal-spatial-comparison.png',dpi=170)
