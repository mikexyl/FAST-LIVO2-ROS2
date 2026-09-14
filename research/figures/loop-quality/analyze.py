"""Reproduce loop-quality tables and plots using only retained inputs.json."""
from pathlib import Path
from collections import Counter
import csv
import json
import sys

import numpy as np
from scipy.spatial.transform import Rotation

HERE = Path(__file__).resolve().parent
RESEARCH = HERE.parents[1]
sys.path.insert(0, str(RESEARCH))
from s3e_pipeline.artifacts import file_hash, read_json, write_json
from s3e_pipeline.geometry import inv
from s3e_pipeline.report_figures import configure, plt, save

DISTANCE_THRESHOLD_M = 2.0
CYCLE_WINDOW_S = 10.0
CYCLE_ROTATION_DEG = 5.0
MIN_NEIGHBORS = 3


def cycle_error(T, U, A, B):
    """Two loops T_i_j/U_k_l and short odometry A=T_i_k/B=T_j_l.

    Max over all four cycle origins makes the translation diagnostic
    invariant to swapping loops or reversing both constraint endpoints.
    """
    cycles = [T@B@inv(U)@inv(A), inv(T)@A@U@inv(B),
              U@inv(B)@inv(T)@A, inv(U)@inv(A)@T@B]
    return (max(float(np.linalg.norm(E[:3, 3])) for E in cycles),
            max(float(np.rad2deg(Rotation.from_matrix(E[:3, :3]).magnitude())) for E in cycles))


def check_algebra():
    rng = np.random.default_rng(17)
    P = []
    for _ in range(4):
        T = np.eye(4); T[:3, :3] = Rotation.random(random_state=rng).as_matrix()
        T[:3, 3] = rng.normal(size=3)*3; P.append(T)
    i, j, k, l = P
    T, U, A, B = inv(i)@j, inv(k)@l, inv(i)@k, inv(j)@l
    assert np.allclose(cycle_error(T, U, A, B), [0, 0], atol=1e-10)
    U = U.copy(); U[0, 3] += 5
    error = cycle_error(T, U, A, B)
    assert error[0] > 4.99
    assert np.allclose(error, cycle_error(U, T, inv(A), inv(B)))
    assert np.allclose(error, cycle_error(inv(T), inv(U), B, A))
    assert np.isclose(np.linalg.norm(T[:3, 3]), np.linalg.norm(inv(T)[:3, 3]))


def key(endpoint):
    return f'{endpoint[0]}:{endpoint[1]}'


def audit(name, experiment):
    loops = experiment['loops']; keys = experiment['keyframes']
    result = dict(total=experiment['total_loops'], status=experiment['status'],
                  historical_proximity=experiment['historical_proximity'])
    if loops is None:
        result.update(distance_available=None, distance_flags=None, cycle_eligible=None,
                      cycle_flags=None, reason='per-loop measurements not retained')
        return result, []
    endpoints = [tuple(sorted((tuple(e['i']), tuple(e['j'])))) for e in loops]
    assert len(endpoints) == len(set(endpoints))
    matrices = [np.asarray(e['T_i_j']).reshape(4, 4) for e in loops]
    for T in matrices:
        assert np.isfinite(T).all() and np.allclose(T[3], [0, 0, 0, 1])
        assert np.allclose(T[:3, :3].T@T[:3, :3], np.eye(3), atol=1e-6)
        assert np.isclose(np.linalg.det(T[:3, :3]), 1, atol=1e-6)
    neighbours = [[] for _ in loops]
    cycles = []
    for a, edge in enumerate(loops):
        i, j = edge['i'], edge['j']
        if i[0] == j[0] or key(i) not in keys or key(j) not in keys:
            continue
        pi, pj = keys[key(i)], keys[key(j)]
        for b in range(a+1, len(loops)):
            other = loops[b]; k, l = other['i'], other['j']
            if (i[0], j[0]) != (k[0], l[0]) or key(k) not in keys or key(l) not in keys:
                continue
            pk, pl = keys[key(k)], keys[key(l)]
            if max(abs(pi['stamp_ns']-pk['stamp_ns']), abs(pj['stamp_ns']-pl['stamp_ns'])) > CYCLE_WINDOW_S*1e9:
                continue
            A = inv(np.asarray(pi['pose']))@np.asarray(pk['pose'])
            B = inv(np.asarray(pj['pose']))@np.asarray(pl['pose'])
            error = cycle_error(matrices[a], matrices[b], A, B)
            neighbours[a].append(error); neighbours[b].append(error); cycles.append(error)
    rows = []
    cfg = experiment['registration_config']
    for index, (edge, T) in enumerate(zip(loops, matrices)):
        gt = edge['gt_distance_m']; measured = float(np.linalg.norm(T[:3, 3]))
        error = abs(measured-gt) if gt is not None else None
        values = neighbours[index]
        bad = sum(t > DISTANCE_THRESHOLD_M or r > CYCLE_ROTATION_DEG for t, r in values)
        eligible = len(values) >= MIN_NEIGHBORS
        gate_failures = []
        for field, violation in [('converged', not edge['converged']),
                ('rmse', edge['rmse_m'] > cfg['max_rmse_m']+1e-9),
                ('overlap', edge['overlap'] < cfg['min_overlap']-1e-9),
                ('inliers', edge['inliers'] < cfg['min_inliers']),
                ('condition', edge['condition'] > cfg['max_condition']),
                ('observability', edge['min_observability'] < cfg['min_observability'])]:
            if violation: gate_failures.append(field)
        rows.append(dict(sequence=name, i=key(edge['i']), j=key(edge['j']),
            i_stamp_ns=keys.get(key(edge['i']), {}).get('stamp_ns'),
            j_stamp_ns=keys.get(key(edge['j']), {}).get('stamp_ns'),
            branch=edge['branch'], inter_robot=edge['i'][0] != edge['j'][0],
            measured_distance_m=measured, gt_distance_m=gt, distance_error_m=error,
            distance_flag=None if error is None else error > DISTANCE_THRESHOLD_M,
            rmse_m=edge['rmse_m'], overlap=edge['overlap'], native_inliers=edge['native_inliers'],
            acceptance_gate_violations='+'.join(gate_failures),
            cycle_neighbours=len(values), cycle_inconsistent_neighbours=bad,
            cycle_eligible=eligible, cycle_flag=bad > len(values)/2 if eligible else None,
            cycle_translation_median_m=float(np.median([v[0] for v in values])) if values else None,
            cycle_rotation_median_deg=float(np.median([v[1] for v in values])) if values else None))
    available = [x for x in rows if x['distance_error_m'] is not None]
    flags = [x for x in available if x['distance_flag']]
    result.update(distance_available=len(available), distance_missing=len(rows)-len(available),
        distance_flags=len(flags) if available else None,
        threshold_counts={str(t):sum(x['distance_error_m'] > t for x in available) if available else None for t in [1, 2, 5]},
        distance_median_m=float(np.median([x['distance_error_m'] for x in available])) if available else None,
        distance_max_m=max((x['distance_error_m'] for x in available), default=None),
        within_10m=sum(x['gt_distance_m'] <= 10 for x in available),
        beyond_10m=sum(x['gt_distance_m'] > 10 for x in available),
        cycle_pairs=len(cycles), cycle_eligible=sum(x['cycle_eligible'] for x in rows),
        cycle_flags=sum(x['cycle_flag'] is True for x in rows) if any(x['cycle_eligible'] for x in rows) else None,
        cycle_translation_max_m=max((x[0] for x in cycles), default=None),
        cycle_rotation_max_deg=max((x[1] for x in cycles), default=None),
        distance_flags_with_cycle_coverage=sum(x['cycle_eligible'] for x in flags),
        distance_flags_also_cycle_flagged=sum(x['cycle_flag'] is True for x in flags),
        distance_flags_inter_robot=sum(x['inter_robot'] for x in flags),
        flags_by_branch=dict(Counter(x['branch'] for x in flags)),
        assessed_by_branch=dict(Counter(x['branch'] for x in available)),
        acceptance_gate_violation_count=sum(bool(x['acceptance_gate_violations']) for x in rows))
    historical = result['historical_proximity']
    if historical['available'] is not None:
        assert historical['available'] == len(available)
        assert historical['within_10m'] == result['within_10m']
    return result, rows


def main():
    check_algebra()
    inputs = read_json(HERE/'inputs.json'); results = {}; all_rows = []
    for name, experiment in inputs['experiments'].items():
        results[name], rows = audit(name, experiment); all_rows += rows
    # Main thresholds are fixed for every experiment; the table includes
    # 1/2/5 m sensitivity rather than choosing thresholds to fit each run.
    output = dict(schema_version=1, distance_threshold_m=DISTANCE_THRESHOLD_M,
        distance_metric='abs(norm(T_i_j.translation) - GT endpoint distance)',
        cycle_window_s=CYCLE_WINDOW_S, cycle_translation_threshold_m=DISTANCE_THRESHOLD_M,
        cycle_rotation_threshold_deg=CYCLE_ROTATION_DEG, cycle_min_neighbours=MIN_NEIGHBORS,
        cycle_rule='strict majority of at least three short inter-robot cycles fail either threshold',
        ground_truth_policy=inputs['ground_truth_policy'],
        limitations=['Distance check ignores rotation and translation direction.',
            'GT uncertainty, missing orientations and uncorrected antenna lever arms prevent exact 6-DoF labels.',
            'Nearby loops and short odometry may share errors; cycle consistency is not independent ground truth.',
            'Intra-robot loops are excluded from the short-cycle check.',
            'Missing per-loop transforms or endpoint timestamps reduce audit coverage.'],
        experiments=results, input_sha256=file_hash(HERE/'inputs.json'),
        analyzer_sha256=file_hash(__file__), synthetic_checks='exact cycle, injected 5 m error, loop swap, endpoint reversal passed')
    write_json(HERE/'results.json', output)
    for filename, rows in [('per-loop.csv', all_rows), ('distance-flags.csv', [r for r in all_rows if r['distance_flag']])]:
        with (HERE/filename).open('w', newline='') as stream:
            writer=csv.DictWriter(stream, fieldnames=list(all_rows[0]));writer.writeheader();writer.writerows(rows)
    configure();fig, axes = plt.subplots(1, 2, figsize=(10, 4.4), layout='constrained')
    labels=[]; counts=[]
    palette={'Square 1':'#64748b','Square 2':'#2684bc','Library 1':'#d17b0f','Playground 2':'#269b69'}
    for name, result in results.items():
        errors=sorted(r['distance_error_m'] for r in all_rows if r['sequence']==name and r['distance_error_m'] is not None)
        if not errors:continue
        axes[0].step(errors,np.arange(1,len(errors)+1)/len(errors),where='post',color=palette[name],label=f'{name} (n={len(errors)})')
        labels.append(name);counts.append(result['distance_flags'])
    axes[0].axvline(DISTANCE_THRESHOLD_M,color='.3',ls='--',lw=1,label='2 m diagnostic threshold')
    axes[0].set(xlabel='Translation-length disagreement with position GT (m)',ylabel='Fraction of assessable accepted loops',xlim=(0,None),ylim=(0,1.03));axes[0].grid();axes[0].legend(fontsize=7)
    bars=axes[1].bar(labels,counts,color=[palette[name] for name in labels])
    for b, name in zip(bars,labels):
        result=results[name];axes[1].text(b.get_x()+b.get_width()/2,b.get_height()+.4,f"{result['distance_flags']} / {result['distance_available']}",ha='center',fontsize=9)
    axes[1].set(ylabel='Accepted loops with disagreement > 2 m',ylim=(0,max(counts)*1.2+1));axes[1].tick_params(axis='x',labelrotation=20);axes[1].grid(axis='y')
    fig.suptitle('Loop measurement audit — distance flags are not confirmed full-pose outliers')
    save(fig,HERE,'loop_quality',300)
    write_json(HERE/'files.json',{p.name:file_hash(p) for p in HERE.iterdir() if p.is_file() and p.name!='files.json'})
    for name, result in results.items():
        print(name, json.dumps(result,sort_keys=True))


if __name__ == '__main__':
    main()
