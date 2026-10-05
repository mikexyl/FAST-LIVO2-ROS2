#!/usr/bin/env python3
"""Collect a small persistent cadence report without copying geometry."""
import argparse
import html
import json
from pathlib import Path
import shutil
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'research'))
from s3e_pipeline.artifacts import file_hash, read_json, write_json


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--run', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--before', type=Path, help='Earlier cadence run for a repair comparison')
    a = p.parse_args()
    a.output.mkdir(parents=True, exist_ok=False)
    result = read_json(a.run / 'result.json')
    if not ((a.run / 'COMPLETE.json').exists() or (a.run / 'FAILED.json').exists()):
        raise ValueError('Wait for a completed test or retained failure before collecting')
    repaired = a.before is not None
    for name in ('result.json', 'configuration.json', 'input-hashes.json', 'source-hashes.json',
                 'probe-source.json', 'iteration-timeline.jsonl', 'final-robot-estimates.json',
                 'published-transform-audit.json', 'FAILED.json', 'COMPLETE.json'):
        if (a.run / name).exists():
            shutil.copy2(a.run / name, a.output / name)
    session = a.run / 'cbs/session'
    shutil.copy2(session / 'session.json', a.output / 'session.json')
    if (a.run / 'evaluation/result.json').exists():
        shutil.copy2(a.run / 'evaluation/result.json', a.output / 'evaluation.json')
        shutil.copy2(a.run / 'central/graph.json', a.output / 'published-submap-graph.json')
    if repaired:
        for name in ('result.json', 'iteration-timeline.jsonl', 'configuration.json'):
            shutil.copy2(a.before / name, a.output / ('before-' + name))
        shutil.copy2(a.before / 'cbs/session/session.json', a.output / 'before-session.json')
    rows = []
    expired = {}
    for robot, status in result['robots'].items():
        dest = a.output / robot
        dest.mkdir()
        for name in ('cbs.yaml', 'estimate.json', 'stats.jsonl'):
            shutil.copy2(session / robot / name, dest / name)
        trajectory = a.run / 'evaluation' / f'{robot}-corrected.tum'
        if trajectory.exists(): shutil.copy2(trajectory, dest / 'corrected.tum')
        log = session / f'{robot}-cbs.log'
        shutil.copy2(log, dest / 'cbs.log')
        expired[robot] = log.read_text().count('Expired belief request')
        native = session / robot / 'native' / robot / 'cbs_online'
        for pattern in ('timing_robot_*.csv', 'belief_service_timing_robot_*.csv', 'registration.json'):
            for path in native.glob(pattern):
                shutil.copy2(path, dest / path.name)
        timing = result['timing'][robot]
        rows.append([robot, str(status['iteration']),
                     f"{timing['observed_iteration_interval_s']['median']:.3f}",
                     f"{timing['update_ms']['maximum']:.3f}",
                     f"{timing['active_callback_ms']['maximum']:.3f}",
                     f"{timing['callback_ms']['maximum']:.3f}", status['component'],
                     str(status['registration']['registration_factor_count']),
                     'pass' if status['registration']['final'] and status['registration']['success'] else 'fail'])
    communication = dict(
        cbs_cdr_bytes=sum((s['cbs_last_stats'] or {}).get('bandwidth_sent_bytes', 0) +
                          (s['cbs_last_stats'] or {}).get('bandwidth_recv_bytes', 0)
                          for s in result['robots'].values()),
        pcm_cdr_bytes=sum(s['pcm_network_cdr_bytes'] for s in result['robots'].values()),
        geometry_cdr_bytes=sum(s['registration_network_cdr_bytes'] for s in result['robots'].values()),
        expired_belief_requests=expired)
    communication['total_measured_cdr_bytes'] = sum(v for v in communication.values() if isinstance(v, int))
    write_json(a.output / 'communication.json', communication)
    headers = ['Robot', 'Iterations', 'Median interval [s]', 'Max solve [ms]',
               'Max timer callback [ms]', 'Max belief service [ms]', 'Final reference',
               'Owned GPU factors', 'Final geometry']
    table = '\n'.join(['| ' + ' | '.join(headers) + ' |',
                       '| ' + ' | '.join(['---'] * len(headers)) + ' |'] +
                      ['| ' + ' | '.join(row) + ' |' for row in rows])
    audit_path = a.run / 'published-transform-audit.json'
    audited = audit_path.exists() and read_json(audit_path)['success']
    verdict = ('PASS: common reference, published transforms and final geometry checks passed.'
               if result['success'] and audited else
               'INCOMPLETE VALIDATION: common reference and local geometry alone do not prove consistent published transforms.'
               if result['success'] else
               'FAIL: ' + result['failure']['message'] + '.')
    resources = result['resources']
    summary = (f"Tested persistent CBS on the frozen 121-submap, 13-loop GRACO A05/A07/G05/G06 graph "
               f"on 148. Each robot was configured for one iteration every {result['interval_requested_s']:g} "
               f"seconds and {result['iterations_requested']} iterations. The run took {resources['wall_s']:.1f} "
               "seconds, including startup, PCM, geometry preparation, scheduled waiting and shutdown.")
    details = ("""This rerun repairs persistent anchor relay and graph initialization, using the same configuration
as the earlier 10-second test. PCM, GPU registration settings, verified loops and point geometry are
unchanged. Both runs retain independent random stage selection; their random draws can differ.
""" if repaired else """This test changes only the execution mode, cadence, settling count and timeout. The native
optimizer binaries, PCM policy, GPU registration settings, verified loops and point geometry match
the sealed experiment.
""") + """Every transferred prepared endpoint matches the centralized evidence hashes.
No artificial host CPU, memory or GPU quota was imposed.

Each robot's timer callback is mutually exclusive. Peer belief requests remain asynchronous, with
one outstanding request per peer. The requested 10 seconds is a scheduling interval, not solve time.
Observed intervals are measured by 100 ms polling. The solve column times the native optimizer
update. Timer callbacks include local update and request dispatch, but exclude asynchronous response
latency. Belief-service timing includes lock waiting, marginalization and response packing. The
terminal timer callback is not present in the native timing CSV.

The same frozen graph is submitted once to optimizers that remain alive throughout the test. This
isolates cadence and persistent belief behavior; it is not a live frontend or growing-map replay.
"""
    interpretation = ''
    if repaired:
        interpretation = """The old persistent response path disabled the original CBS anchor relay. Incoming
anchor factors are excluded from the GLOBAL covariance graph, so anchors learned only through those
factors could not produce a local marginal and were silently omitted. Sparse chains exposed the bug;
the earlier fully connected three-robot tests did not.

The repair forwards cached anchor Gaussians with their original producer, mean, covariance and frame.
It does not label forwarded evidence as an independent local measurement or damp it a second time.
Repeated unchanged relays and echoes to the original producer are suppressed. Local-belief damping,
the two covariance exclusions and random stage selection remain unchanged.

A second bug discarded all supplied odometry initial poses in persistent mode: GTSAM's
FactorGraph::exists argument is a factor slot, not a pose key. The repair checks the graph's
variable-key set. The whole map no longer starts at identity, and new remote separators initialize
from their connected local measurement. Unconnected seeds remain excluded. The new long, rotated
trajectory regression fails before this correction and passes after it.

The relay-only intermediate was rejected: its labels reached A05, but the bad initialization
produced 87.899 m joint ATE and a 208.673 m published loop mismatch. This probe now also audits
published endpoint transforms using the existing configured alignment tolerances (2 m, 15 degrees),
independently of ground truth. Local geometry checks alone could not expose that error.

The new native five-robot chain test fails on the old core and passes after the repair. Tests also
cover independent stages, reordered delivery, a ring, producer/covariance preservation, duplicate
suppression and graph-size stability. Real DDS tests cover a growing five-robot chain with both
pose-only graphs and GPU registration factors.
"""
        if (a.run / 'evaluation/result.json').exists():
            evaluation = read_json(a.run / 'evaluation/result.json')
            ate = next(iter(evaluation['joint_ate'].values()))['rmse_m']
            residuals = read_json(a.run / 'central/graph.json')['published_loop_residuals']
            max_t = max(r['translation_m'] for r in residuals)
            max_r = max(r['rotation_deg'] for r in residuals)
            interpretation += f"\nJoint ATE is {ate:.6f} m, versus 0.961862 m for the retained coordinated run and 0.959933 m for the centralized control. Maximum published loop mismatch is {max_t:.6f} m / {max_r:.6f} degrees. These small ATE differences are descriptive, not an accuracy-improvement claim. Evaluation uses evo 1.36.5, 50 ms association, one shared rigid alignment, no scale fit and corrected GRACO IMU-reference ground truth. Ground truth is used only after optimization.\n"
    if not result['success'] and result['failure']['message'] == 'Persistent CBS final common-frame contract failed':
        interpretation += """Slowing the cadence did not establish a common frame across all four robots within
30 iterations. A05, A07 and G05 publish in A05's frame, while G06 still publishes in A07's frame.
The local geometry checks can pass despite this global inconsistency. No four-robot joint ATE is
reported because the common-frame gate failed; no external alignment repairs the result.

This reproduces the reference-propagation failure at a slower cadence. The logged computation fits
comfortably within 10 seconds, so optimizer callback overrun does not explain this run's failure.
Code inspection shows that the persistent GLOBAL marginal removes incoming anchor-belief factors
and persistent responses do not relay those anchors. This is consistent with failure to carry A05's
reference through the A05–A07–G05–G06 chain; repairing that mechanism is separate from this cadence test.
"""
    measurements = (f"Measured process-tree CPU: {resources['parent_cpu_s'] + resources['child_cpu_s']:.2f} s. "
        f"Peak simultaneous process-tree RSS: {resources['peak_simultaneous_process_tree_rss_bytes']/2**30:.2f} GiB "
        "(20 ms sampling; shared pages may count twice). "
        f"CBS requests/responses: {communication['cbs_cdr_bytes']/1024:.2f} KiB; "
        f"PCM: {communication['pcm_cdr_bytes']/1024:.2f} KiB; "
        f"geometry exchange: {communication['geometry_cdr_bytes']/2**20:.3f} MiB. "
        f"Expired belief requests: {sum(expired.values())}. "
        "Communication counts serialized application messages at the requester/sender once; it excludes "
        "discovery, network headers, retransmissions, reused retrieval and coordinator-local input/status traffic.")
    report = '# Persistent CBS at 10-second intervals\n\n' + verdict + '\n\n' + summary + '\n\n' + table
    report += '\n\n' + interpretation + '\n\n' + details + '\n\n' + measurements + '\n'
    (a.output / 'REPORT.md').write_text(report)
    page = '<!doctype html><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">'
    page += '<title>Persistent CBS · 10-second cadence</title><style>body{font:16px/1.6 system-ui;color:#22333d;background:#f5f7fa;max-width:1150px;margin:40px auto;padding:0 24px}table{border-collapse:collapse;background:white;font-size:13px}td,th{padding:10px;border-bottom:1px solid #ddd;text-align:right}td:first-child,th:first-child{text-align:left}.scroll{overflow:auto}a{color:#207584}</style>'
    page += '<h1>Persistent CBS at 10-second intervals</h1><p><strong>' + html.escape(verdict) + '</strong></p><p>' + html.escape(summary) + '</p>'
    page += '<div class="scroll"><table><tr>' + ''.join('<th>' + html.escape(h) + '</th>' for h in headers) + '</tr>'
    page += ''.join('<tr>' + ''.join('<td>' + html.escape(v) + '</td>' for v in row) + '</tr>' for row in rows) + '</table></div>'
    for paragraph in (interpretation + '\n\n' + details + '\n\n' + measurements).split('\n\n'):
        if paragraph.strip(): page += '<p>' + html.escape(paragraph) + '</p>'
    page += '<p><a href="../index.html">Sealed comparison</a> · <a href="REPORT.md">Report</a> · <a href="result.json">Numerical results</a> · <a href="iteration-timeline.jsonl">Iteration timeline</a></p>'
    (a.output / 'index.html').write_text(page)
    write_json(a.output / 'FILES.sha256.json', {str(f.relative_to(a.output)):file_hash(f)
        for f in sorted(a.output.rglob('*')) if f.is_file()})


if __name__ == '__main__':
    main()
