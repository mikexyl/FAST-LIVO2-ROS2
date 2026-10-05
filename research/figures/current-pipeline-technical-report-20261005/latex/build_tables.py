#!/usr/bin/env python3
"""Build current-method report tables from retained numerical evidence (stdlib only)."""
import json
import math
from collections import Counter
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
EVIDENCE = ROOT / 'evidence'
OUT = ROOT / 'latex' / 'generated'
OUT.mkdir(exist_ok=True)


def read(name):
    return json.loads((EVIDENCE / name).read_text())


def table(name, caption, label, columns, header, rows, note=''):
    text = [r'\begin{table}[htbp]', r'\centering\small',
            r'\caption{' + caption + '}', r'\label{' + label + '}',
            r'\begin{tabular}{' + columns + '}', r'\toprule',
            header + r' \\', r'\midrule']
    text += [' & '.join(str(cell) for cell in row) + r' \\' for row in rows]
    text += [r'\bottomrule', r'\end{tabular}']
    if note:
        text += [r'\par\vspace{5pt}\begin{minipage}{0.97\linewidth}\footnotesize ' + note + r'\end{minipage}']
    text += [r'\end{table}', '']
    (OUT / (name + '.tex')).write_text('\n'.join(text))


audit = read('audit.json')
evaluation = read('evaluation.json')
timing = read('cbs-timing/summary.json')
config = read('configuration.json')
freeze = read('immediate-freeze/audit.json')
freeze_eval = read('immediate-freeze/evaluation.json')
freeze_cbs = read('immediate-freeze/cbs-summary.json')
joint = evaluation['joint_ate']['aerial01']
robots = sorted(audit['geometry'])
assert len(robots) == 14 and len(set(evaluation['components'].values())) == 1
assert joint['samples'] == sum(v['samples'] for v in joint['per_robot'].values())
derived_joint = math.sqrt(sum(v['samples'] * v['statistics']['rmse'] ** 2
                             for v in joint['per_robot'].values()) / joint['samples'])
assert math.isclose(derived_joint, joint['rmse_m'], rel_tol=1e-12)
assert all(v['minimum_prefix_coverage'] >= config['circle_cover']['coverage_target']
           for v in audit['geometry'].values())

rows = []
for robot in robots:
    name = ('A' if robot.startswith('aerial') else 'G') + robot[-2:]
    rj = joint['per_robot'][robot]
    rows.append([name, audit['geometry'][robot]['circles'], f"{rj['samples']:,}",
                 f"{evaluation['raw_ate'][robot]['rmse_m']:.3f}",
                 f"{evaluation['individual_ate'][robot]['rmse_m']:.3f}",
                 f"{rj['statistics']['rmse']:.3f}"])
rows.append([r'\textbf{All 14}', sum(v['circles'] for v in audit['geometry'].values()),
             f"{joint['samples']:,}", '---', '---', r'\textbf{' + f"{joint['rmse_m']:.3f}" + '}'])
table('robot-accuracy', r'GRACO position ATE (RMSE, metres). Raw and CBS individual columns each fit a separate rigid transform per robot. The shared column applies one transform to the entire connected team.',
      'tab:robot-accuracy', 'lrrrrr', r'Robot & Circles & Matched poses & Raw, individual & CBS, individual & CBS, shared', rows,
      r'A01--A08 are aerial; G01--G06 are ground. The all-14 entry is pose-weighted joint RMSE, not the mean of row RMSEs. Independent alignment can hide relative placement error.')


def evidence_fraction(a):
    g = a['geometry'].values()
    return sum(v['published_evidence_unique_points'] for v in g) / sum(v['points'] for v in g)


components_before = Counter(freeze_eval['components'].values())
table('maturation-ablation', r'Geometry-publication ablation on simultaneous all-14 replays. All placement, retrieval and verification thresholds are retained. Each column is a separate fresh execution.',
      'tab:maturation-ablation', 'lrr', r'Measure & Immediate freezing & Maturation (current)', [
          ['Circle nodes', freeze['descriptors']['total'], audit['descriptors']['total']],
          ['Connected components', len(components_before), len(set(evaluation['components'].values()))],
          ['Largest component (robots)', max(components_before.values()), len(robots)],
          ['Retained loops', freeze_cbs['loops'], audit['verification_accepted']],
          ['Empty layered descriptors', freeze['descriptors']['empty_layered_features'], audit['descriptors']['empty_layered_features']],
          ['Final observed IDs in published evidence', f"{100*evidence_fraction(freeze):.1f}\\%", f"{100*evidence_fraction(audit):.1f}\\%"],
          ['All-team joint ATE (m)', 'Unavailable', f"{joint['rmse_m']:.3f}"],
          ['Application traffic (MB)', f"{freeze['communication']['total_application_cdr_bytes']/1e6:.2f}", f"{audit['communication']['total_application_cdr_bytes']/1e6:.2f}"]],
      r'The disconnected immediate-freeze run has no all-team joint ATE. Native-ID evidence coverage is different from occupied-cell footprint coverage. Independent odometry and scheduling introduce small differences in circle placement; this is not a frozen-input or repeated-trial estimate.')

comm = audit['communication']
peer = comm['peer_retrieval_by_kind_cdr_bytes']
assert sum(peer.values()) == comm['peer_retrieval_cdr_bytes']
traffic = [('Retrieval queries', peer['query']), ('Verification geometry replies', peer['payload_response']),
           ('Candidate replies', peer['candidates']), ('Verified constraints', peer['constraint']),
           ('Geometry requests', peer['payload_request']), ('Peer completion messages', peer['done']),
           ('PCM exchange', comm['pcm_network_cdr_bytes']),
           ('CBS registration geometry', comm['registration_network_cdr_bytes']),
           ('CBS belief requests and replies', comm['cbs_network_cdr_bytes'])]
total = comm['total_application_cdr_bytes']
assert sum(value for _, value in traffic) == total
rows = [[name, f'{value/1e6:.3f}', f'{100*value/total:.2f}'] for name, value in traffic]
rows.append([r'\textbf{Total}', r'\textbf{' + f'{total/1e6:.3f}' + '}', r'\textbf{100.00}'])
table('traffic', r'Measured remote application payload for the current GRACO all-14 run. One MB is $10^6$ bytes.',
      'tab:traffic', 'lrr', r'Exchange & MB & Share (\%)', rows,
      r'Recipient fan-out is counted. Middleware/transport headers, discovery and retransmissions are excluded; these are not physical link-byte measurements.')

comparators = read('baseline-communication.json')['rows']
display = {'swarm': r'Swarm-SLAM~\cite{swarm}', 'dcl': r'DCL-SLAM~\cite{dcl}', 'disco': r'DiSCo-SLAM~\cite{disco}'}
rows = []
for row in comparators:
    assert row['status'] == 'complete' and row['valid_total']
    assert len(row['components']) == 1 and len(row['components'][0]) == 3
    rows.append([display[row['method']], 'G01--G03', '3/3', f"{row['bytes']/1e6:.2f}", f"{row['bytes']/3e6:.2f}"])
rows.append([r'\textbf{Current pipeline}', 'A01--A08, G01--G06', '14/14', f'{total/1e6:.2f}', f'{total/14e6:.2f}'])
table('baseline-communication', r'Communication context from completed native-baseline GRACO ground-trio runs and the current 14-robot mixed run. The workloads are different; per-robot normalization is descriptive.',
      'tab:baseline-traffic', 'llrrr', r'Method & Assigned team & Connected & Total MB & MB/robot', rows,
      r'All rows use serialized inter-robot application payload accounting. Different team size, trajectory length, observations and constraints preclude a matched efficiency or accuracy ranking. No incomplete measurement is included.')

rows = []
for label, key in [('Local solver update', 'update_ms'), ('Full nonterminal callback', 'active_callback_ms'), ('Peer-belief service', 'service_callback_ms')]:
    d = timing[key]
    rows.append([label, f"{d['n']:,}", f"{d['median']:.3f}", f"{d['p95']:.3f}", f"{d['max']:.3f}"])
table('timing', r'Native elapsed wall-time measurements. All duration columns are in milliseconds.',
      'tab:timing', 'lrrrr', r'Timed region & Samples & Median (ms) & P95 (ms) & Maximum (ms)', rows,
      r'The full callback contains the local solver update, so rows must not be added. Peer services run separately. Fourteen terminal callbacks, final geometry validation and pre-update early returns are outside the timing coverage.')

c = config['circle_cover']; p = config['circle_publication']; b = config['backend']; rf = config['dpgo']['registration_factors']
settings = [
    ['Odometry', r'Persistent native EllipseLIO; calibrated IMU frame'],
    ['Circle footprint', f"Radius {c['radius_m']:g}\\,m; {c['cell_size_m']:g}\\,m occupied-cell grid; {100*c['coverage_target']:g}\\% coverage"],
    ['Placement priorities', r'Coverage, circle count, then 30\% overlap target'],
    ['Placement solve limit', f"{c['solver_time_limit_s']:g}\\,s per update; validated greedy fallback"],
    ['Normal geometry publication', f"Age $\\geq${p['minimum_age_s']:g}\\,s; growth $\\leq${100*p['maximum_growth_fraction']:g}\\% over $\\geq${p['stability_window_s']:g}\\,s; terrain/feature gates"],
    ['Forced publication', f"{p['maximum_wait_s']:g}\\,s deadline at next observation, or end of stream"],
    ['BEV retrieval', r'Point density, 0.5\,m pixels; five terrain-relative bands'],
    ['Point-GICP verification', r'0.4\,m evidence voxels; 1.5\,m correspondence radius'],
    ['PCM', r'Probability 0.99; minimum clique size one'],
    ['Registration factors', f"Native GPU VGICP; 0.5\\,m preprocessing; {rf['max_points']:,} points/endpoint"],
    ['Factor voxel levels', r'Two adaptive levels; recorded maps at 1 and 2\,m'],
    ['Persistent CBS', r'0.1\,Hz requested; random outgoing pose/anchor exchange'],
    ['Final settling', r'Common-reference barrier, then 30 iterations'],
    ['Evaluation', r'evo 1.36.5; 50\,ms association; shared rigid alignment; no scale'],
]
table('settings', r'Effective configuration of the reported online run.', 'tab:settings',
      r'lp{0.68\linewidth}', r'Component & Setting', settings)

checks = {'robot_count': len(robots), 'circles': sum(v['circles'] for v in audit['geometry'].values()),
          'matched_poses': joint['samples'], 'joint_ate_m': joint['rmse_m'],
          'joint_ate_recomputed_from_robot_sse_m': derived_joint,
          'communication_bytes': total, 'timed_updates': timing['timed_updates'],
          'peer_service_requests': timing['service_requests'],
          'mature_evidence_coverage': evidence_fraction(audit),
          'freeze_evidence_coverage': evidence_fraction(freeze),
          'all_nonempty_prefixes_meet_coverage': True,
          'traffic_breakdown_sums_exactly': True,
          'baseline_rows_complete_and_connected': True,
          'generated_tables': sorted(p.name for p in OUT.glob('*.tex'))}
(ROOT / 'review' / 'table-checks.json').write_text(json.dumps(checks, indent=2) + '\n')
print(json.dumps(checks, indent=2))
