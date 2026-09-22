"""Read-only evaluation of frozen runtime outputs; never modifies estimator inputs."""
import os
os.environ.setdefault('MPLCONFIGDIR', '/tmp/graco-cbs-diagnostic-mpl')
import copy
import hashlib
import json
from collections import defaultdict
from importlib.metadata import version
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from evo.core import metrics
from evo.core.trajectory import PosePath3D
from evo.main_ape import ape
from evo.tools import file_interface
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

OUT = Path(__file__).resolve().parent
BASE = OUT.parent / 'graco-aerial-height148-20260921/full'
ROBOTS = [f'aerial{i:02d}' for i in range(5, 9)]
assert version('evo') == '1.36.5'
inputs = {}

def read(p):
    p = BASE / p
    inputs[str(p.relative_to(BASE))] = hashlib.sha256(p.read_bytes()).hexdigest()
    return json.loads(p.read_text())

def rows(p):
    p = BASE / p
    inputs[str(p.relative_to(BASE))] = hashlib.sha256(p.read_bytes()).hexdigest()
    return [json.loads(l) for l in p.read_text().splitlines()]

def tum(p):
    p = BASE / p
    inputs[str(p.relative_to(BASE))] = hashlib.sha256(p.read_bytes()).hexdigest()
    return file_interface.read_tum_trajectory_file(p)

def inv(T):
    o = np.eye(4)
    o[:3,:3] = T[:3,:3].T
    o[:3,3] = -o[:3,:3] @ T[:3,3]
    return o

def err(T):
    return dict(translation_m=float(np.linalg.norm(T[:3,3])),
                rotation_deg=float(np.rad2deg(Rotation.from_matrix(T[:3,:3]).magnitude())))

def stats(x):
    x = np.array(x)
    return dict(mean=float(x.mean()), median=float(np.median(x)), max=float(x.max()), min=float(x.min()))

def metric(ref, est, align=True):
    return ape(copy.deepcopy(ref), copy.deepcopy(est), metrics.PoseRelation.translation_part,
               align=align, correct_scale=False)

report = read('report/report.json')
keys = {r: {x['keyframe_id']: np.array(x['T_world_body']) for x in
            rows(f'prepared-{r}/store/keyframes.jsonl')} for r in ROBOTS}
optimized = {(x['robot_id'], x['keyframe_id']): np.array(x['T_world_body']) for x in rows('dpgo/poses.jsonl')}
raw_align = {r: np.array(report['raw'][r]['alignment_SE3']) for r in ROBOTS}
edges = rows('dpgo/constraints.jsonl')
loops = []
for e in edges:
    i,j = tuple(e['i']),tuple(e['j'])
    Z = np.array(e['T_i_j'])
    E = inv(optimized[i]) @ optimized[j]
    Gi,Gj = raw_align[i[0]] @ keys[i[0]][i[1]], raw_align[j[0]] @ keys[j[0]][j[1]]
    reference = inv(Gi) @ Gj
    delta = (Gi @ Z)[:3,3] - Gj[:3,3]
    diag = e['diagnostics']
    expected = np.array(diag['T_i_j'])
    if tuple(diag['query']) != i:
        assert tuple(diag['candidate']) == i and tuple(diag['query']) == j
        expected = inv(expected)
    else:
        assert tuple(diag['candidate']) == j
    loops.append(dict(i=list(i), j=list(j), pair=i[0]+'--'+j[0],
        cbs_vs_measurement=err(inv(Z) @ E),
        measurement_vs_position_aligned_odometry=err(inv(reference) @ Z),
        estimated_j_anchor_error_enu_m=delta.tolist(),
        optimized_vs_position_aligned_odometry=err(inv(reference) @ E),
        canonicalization_max_abs_error=float(np.abs(Z-expected).max()),
        verification_overlap=diag['overlap'], verification_rmse_m=diag['rmse_m']))

pair_summary = {}
for pair in sorted({l['pair'] for l in loops}):
    subset = [l for l in loops if l['pair'] == pair]
    pair_summary[pair] = dict(count=len(subset),
        cbs_vs_measurement_translation_m=stats([l['cbs_vs_measurement']['translation_m'] for l in subset]),
        cbs_vs_measurement_rotation_deg=stats([l['cbs_vs_measurement']['rotation_deg'] for l in subset]),
        measurement_vs_position_aligned_odometry_translation_m=stats([l['measurement_vs_position_aligned_odometry']['translation_m'] for l in subset]),
        measurement_vs_position_aligned_odometry_rotation_deg=stats([l['measurement_vs_position_aligned_odometry']['rotation_deg'] for l in subset]),
        mean_estimated_j_anchor_error_enu_m=np.mean([l['estimated_j_anchor_error_enu_m'] for l in subset],axis=0).tolist())

robot_summary = {}
refs, cbs_paths = {}, {}
for r in ROBOTS:
    refs[r] = tum(f'report/evo/cbs/{r}-reference-matched.tum')
    cbs_paths[r] = tum(f'report/evo/cbs/{r}-estimate-aligned.tum')
    individual = tum(f'report/evo/cbs-individual/{r}-estimate-aligned.tum')
    raw = tum(f'report/evo/raw/{r}-estimate-aligned.tum')
    assert np.array_equal(raw.timestamps,cbs_paths[r].timestamps)
    residual = cbs_paths[r].positions_xyz - refs[r].positions_xyz
    mean = residual.mean(0)
    centered = residual-mean
    shape = metric(raw, individual)
    file_interface.save_res_file(OUT/f'{r}-raw-vs-cbs-shape.zip',shape)
    odom_res = [err(inv(inv(keys[r][k]) @ keys[r][k+1]) @ (inv(optimized[r,k]) @ optimized[r,k+1]))
                for k in range(len(keys[r])-1)]
    robot_summary[r] = dict(raw_ate_m=report['raw'][r]['rmse_m'],
        cbs_individual_ate_m=report['cbs_individual'][r]['rmse_m'],
        cbs_shared_ate_m=report['cbs']['aerial05']['per_robot'][r]['statistics']['rmse'],
        mean_shared_residual_enu_m=mean.tolist(),
        centered_shared_residual_rms_m=float(np.sqrt(np.mean(np.sum(centered**2,axis=1)))),
        raw_vs_cbs_shape_evo_rms_m=shape.stats['rmse'],
        odometry_edge_change_translation_m=stats([x['translation_m'] for x in odom_res]),
        odometry_edge_change_rotation_deg=stats([x['rotation_deg'] for x in odom_res]))

subsets = {}
for group in [ROBOTS, ROBOTS[1:], ROBOTS[:3], ['aerial05','aerial07']]:
    reference = PosePath3D(poses_se3=np.concatenate([refs[r].poses_se3 for r in group]))
    estimate = PosePath3D(poses_se3=np.concatenate([cbs_paths[r].poses_se3 for r in group]))
    result = metric(reference, estimate)
    name = '--'.join(group)
    file_interface.save_res_file(OUT/f'{name}-subset-ape.zip', result)
    subsets[name] = dict(statistics=result.stats, robots=group, optimized_again=False,
                        alignment='one new rigid alignment of this subset only')

summary = dict(engine='evo 1.36.5', input_base=str(BASE), evaluation_only=True,
    fed_back_to_estimation=False, caveats=[
    'Loop reference uses raw odometry orientation and independently position-GT-fitted SE3; it is not exact 6-DoF ground truth.',
    'Loop consistency can localize an error upstream of CBS but cannot identify registration, calibration or dataset reference bias by itself.',
    'Subset ATE changes evaluation alignment only; no new optimization is run.'],
    robots=robot_summary, pairs=pair_summary, subsets=subsets, loops=loops,
    max_canonicalization_error=max(l['canonicalization_max_abs_error'] for l in loops),
    input_sha256=inputs)
(OUT/'analysis.json').write_text(json.dumps(summary,indent=2)+'\n')

fig,axs = plt.subplots(1,3,figsize=(16,4.5))
x=np.arange(4); w=.25
for offset,label,field in [(-w,'Raw: individual alignment','raw_ate_m'),(0,'CBS: individual alignment','cbs_individual_ate_m'),(w,'CBS: shared alignment','cbs_shared_ate_m')]:
    axs[0].bar(x+offset,[robot_summary[r][field] for r in ROBOTS],w,label=label)
axs[0].set_xticks(x,['A05','A06','A07','A08']); axs[0].set_ylabel('Position ATE (m)'); axs[0].legend(fontsize=8)
axs[0].set_title('Trajectory shape versus shared placement')
pairs=list(pair_summary)
axs[1].bar(np.arange(len(pairs))-.18,[pair_summary[p]['measurement_vs_position_aligned_odometry_translation_m']['mean'] for p in pairs],.36,label='Incoming loop vs reference*')
axs[1].bar(np.arange(len(pairs))+.18,[pair_summary[p]['cbs_vs_measurement_translation_m']['mean'] for p in pairs],.36,label='CBS vs incoming loop')
axs[1].set_xticks(np.arange(len(pairs)),[p.replace('aerial','A') for p in pairs],rotation=20)
axs[1].set_ylabel('Mean translation discrepancy (m)'); axs[1].set_title('Where relative-placement discrepancy enters'); axs[1].legend(fontsize=8)
for k,(label,color) in enumerate(zip(['East','North','Up'],['tab:blue','tab:orange','tab:green'])):
    axs[2].bar(x+(k-1)*w,[robot_summary[r]['mean_shared_residual_enu_m'][k] for r in ROBOTS],w,label=label,color=color)
axs[2].set_xticks(x,['A05','A06','A07','A08']); axs[2].axhline(0,color='black',lw=.5)
axs[2].set_ylabel('Mean aligned position residual (m)'); axs[2].set_title('Shared-alignment error direction'); axs[2].legend(fontsize=8)
fig.text(.5,.01,'*Reference: independently position-GT-aligned raw odometry; diagnostic only, not exact six-DoF ground truth.',ha='center',fontsize=9)
fig.tight_layout(rect=[0,.05,1,1]);fig.savefig(OUT/'cbs-error-diagnosis.png',dpi=180);plt.close(fig)
print(json.dumps({k:v for k,v in summary.items() if k not in ['loops','input_sha256']},indent=2))
