#!/usr/bin/env python3
"""Reproduce CBS timing figures from saved counters; no experiment is rerun."""
import csv
import hashlib
import html
import json
import os
from pathlib import Path

os.environ.setdefault('MPLCONFIGDIR', '/tmp/circle-cbs-timing-matplotlib')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.ticker import MaxNLocator
import numpy as np

ROOT = Path(__file__).resolve().parent
BLUE, ORANGE, GREEN = '#0072B2', '#D55E00', '#009E73'
INK, GRAY = '#23333F', '#8B949E'
plt.rcParams.update({
    'font.family': 'serif', 'font.serif': ['DejaVu Serif'], 'font.size': 8,
    'axes.titlesize': 9, 'axes.titleweight': 'bold', 'axes.labelsize': 8,
    'legend.fontsize': 7, 'legend.frameon': False,
    'axes.spines.top': False, 'axes.spines.right': False,
    'axes.grid': True, 'grid.alpha': .16, 'grid.linewidth': .5,
    'pdf.fonttype': 42, 'ps.fonttype': 42, 'savefig.dpi': 300,
})


def read(path):
    return json.loads(path.read_text())


def distribution(values):
    x = np.asarray(values, float)
    assert len(x) and np.isfinite(x).all()
    return dict(n=len(x), min=float(x.min()), median=float(np.median(x)),
                p95=float(np.percentile(x, 95)), max=float(x.max()))


def write_csv(path, rows):
    with path.open('w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def build_data(raw, audit):
    start = raw['start']['wall_ns']
    revisions = {r['revision']: r for r in raw['revisions']}
    updates, services, omissions, checks = [], [], [], []
    for robot, data in raw['robots'].items():
        stats = {s['iteration']: s for s in data['stats']}
        assert len(stats) == len(data['stats'])
        assert 'belief_stage_switch_strategy: random' in data['cbs_yaml']
        counts, admitted_factors = 0, 0
        timed = set()
        for t in data['timing']:
            iteration = int(t['iteration'])
            assert iteration not in timed
            timed.add(iteration)
            s = stats[iteration]
            rev = revisions[s['graph_revision']]
            for k in ('num_new_values', 'num_new_factors', 'num_removed_factors',
                      'num_accepted_beliefs'):
                assert int(t[k]) == s[k], (robot, iteration, k)
            counts += int(t['num_new_values'])
            assert counts == rev['local_submaps'][robot], (robot, iteration)
            admitted_factors += s['num_new_factors'] - s['num_removed_factors']
            row = dict(robot=robot, iteration=iteration,
                       progress_s=(s['logged_wall_ns'] - start) / 1e9,
                       graph_revision=s['graph_revision'],
                       outgoing_exchange_stage=s['belief_stage'],
                       owned_submaps=counts, local_optimizer_variables=s['num_values'],
                       cumulative_admitted_factors=admitted_factors,
                       local_registration_pairs=rev['local_registration_pairs'][robot],
                       global_submaps=rev['global_submaps'],
                       global_registration_pairs=rev['global_registration_pairs'],
                       new_factors=s['num_new_factors'],
                       meaningful_update=s['meaningful_update'],
                       accepted_beliefs=s['num_accepted_beliefs'])
            for k in ('update_ms', 'stats_ms', 'belief_request_ms',
                      'visualization_ms', 'active_callback_ms'):
                row[k] = float(t[k])
                assert np.isfinite(row[k]) and row[k] >= 0
            assert row['active_callback_ms'] >= row['update_ms']
            updates.append(row)
        missing = sorted(set(stats) - timed)
        # The native terminal path returns before logTiming. Never invent zeros.
        assert missing == [max(stats)], (robot, missing)
        omissions.append(dict(robot=robot, iteration=missing[0], reason='terminal callback has no timing row'))
        assert all(stats[b]['logged_wall_ns'] > stats[a]['logged_wall_ns']
                   for a, b in zip(sorted(stats), sorted(stats)[1:]))
        for t in data['service']:
            iteration = int(t['iteration'])
            assert iteration in stats or iteration == 0, (robot, iteration)
            s = stats.get(iteration)
            next_stats = stats.get(iteration + 1)
            row = dict(robot=robot, request=int(t['request']), iteration=iteration,
                       mapped_iteration_progress_s=((s['logged_wall_ns'] - start) / 1e9) if s else None,
                       next_iteration_progress_s=((next_stats['logged_wall_ns'] - start) / 1e9)
                           if next_stats else None,
                       request_kind='pose+anchor' if int(t['include_agent_keys']) else 'anchor',
                       returned_beliefs=int(t['returned_belief_count']))
            for k in ('get_beliefs_ms', 'lock_wait_ms', 'prepare_request_ms',
                      'pack_response_ms', 'callback_ms'):
                row[k] = float(t[k])
                assert np.isfinite(row[k]) and row[k] >= 0
            assert abs(sum(row[k] for k in ('get_beliefs_ms', 'lock_wait_ms',
                       'prepare_request_ms', 'pack_response_ms')) - row['callback_ms']) < .00001
            services.append(row)
        checks.append(dict(robot=robot, timing_rows=len(timed), stats_rows=len(stats),
                           owned_nodes_agree_with_revision=True,
                           final_variables=stats[max(stats)]['num_values']))
    assert len(checks) == 14
    last_bag = (audit['timeline']['last_bag_expected_end_wall_ns'] - start) / 1e9
    last_graph = min(r['progress_s'] for r in updates if r['graph_revision'] == max(revisions))
    by_revision = []
    for rev_id, rev in revisions.items():
        group = [r for r in updates if r['graph_revision'] == rev_id]
        meaningful = [r for r in group if r['meaningful_update']]
        by_revision.append(dict(revision=rev_id, global_submaps=rev['global_submaps'],
            global_registration_pairs=rev['global_registration_pairs'],
            first_update_s=min(r['progress_s'] for r in group),
            last_update_s=max(r['progress_s'] for r in group),
            update_ms=distribution([r['update_ms'] for r in group]),
            active_callback_ms=distribution([r['active_callback_ms'] for r in group]),
            meaningful_count=len(meaningful),
            meaningful_update_ms=distribution([r['update_ms'] for r in meaningful])))
    def metric(rows, key):
        return distribution([r[key] for r in rows])
    summary = dict(
        experiment='circle-mature-online148-20261001/full', analysis_date='2026-10-05',
        timed_updates=len(updates), service_requests=len(services), terminal_timing_omissions=omissions,
        service_requests_before_first_iteration=sum(r['iteration'] == 0 for r in services),
        validation=checks, update_ms=metric(updates, 'update_ms'),
        active_callback_ms=metric(updates, 'active_callback_ms'),
        stats_ms=metric(updates, 'stats_ms'), service_callback_ms=metric(services, 'callback_ms'),
        service_compute_ms=metric(services, 'get_beliefs_ms'),
        service_lock_wait_ms=metric(services, 'lock_wait_ms'),
        by_revision=by_revision,
        meaningful_update_ms=metric([r for r in updates if r['meaningful_update']], 'update_ms'),
        no_new_information_update_ms=metric([r for r in updates if not r['meaningful_update']], 'update_ms'),
        last_bag_s=last_bag, first_final_graph_update_s=last_graph,
        global_submaps_final=revisions[max(revisions)]['global_submaps'],
        global_registration_pairs_final=revisions[max(revisions)]['global_registration_pairs'],
        maximum_local_variables=max(r['local_optimizer_variables'] for r in updates),
        stage_policy='Random outgoing pose/anchor exchange, not strict alternation; both optimize the local graph.',
        timing_scope='Elapsed wall time from native steady clock, not CPU time or synchronized GPU kernel timing.',
        native_timer_source_sha256='eed0bb254106852048f0aee44b90d470e09d40767aa1b56e2a37f24dd9918bc1',
        raw_sha256=hashlib.sha256((ROOT / 'raw-timing.json').read_bytes()).hexdigest(),
    )
    return updates, services, summary


def bins(rows, xkey, ykey, width=30):
    grouped = {}
    for r in rows:
        grouped.setdefault(int(r[xkey] // width), []).append(r)
    x, p50, p95 = [], [], []
    for k in sorted(grouped):
        values = grouped[k]
        x.append(np.median([r[xkey] for r in values]) / 60)
        p50.append(np.median([r[ykey] for r in values]))
        p95.append(np.percentile([r[ykey] for r in values], 95))
    return x, p50, p95


def quantile_lines(ax, rows, xkey, ykey):
    x, p50, p95 = bins(rows, xkey, ykey)
    ax.plot(x, p50, '-', color=INK, lw=1.35, label='Median (30 s bins)', zorder=4)
    ax.plot(x, p95, '--', color=INK, lw=1.1, label='95th percentile', zorder=4)


def finish(fig, name):
    for suffix in ('pdf', 'png'):
        fig.savefig(ROOT / f'{name}.{suffix}', bbox_inches='tight', dpi=300)
    plt.close(fig)


def plots(raw, updates, services, summary):
    final_s = max(r['progress_s'] for r in updates)
    last_bag = summary['last_bag_s'] / 60
    stages = [('pose', BLUE, 'o'), ('anchor', ORANGE, '^')]
    fig, axes = plt.subplots(2, 2, figsize=(7.2, 6.0), sharex=True)
    for ax in axes.flat:
        ax.axvspan(last_bag, final_s / 60 + .2, color='#ECEFF1', alpha=.65, zorder=0)
        ax.axvline(last_bag, color='#687782', lw=.8, ls=':')
        ax.set_xlim(0, final_s / 60 + .2)
        ax.xaxis.set_major_locator(MaxNLocator(6))
    for ax, key in zip(axes[0], ['update_ms', 'active_callback_ms']):
        for stage, color, marker in stages:
            rows = [r for r in updates if r['outgoing_exchange_stage'] == stage]
            ax.scatter([r['progress_s'] / 60 for r in rows], [r[key] for r in rows],
                       s=8, marker=marker, color=color, alpha=.40, linewidths=0)
        quantile_lines(ax, updates, 'progress_s', key)
        ax.set_ylabel('Elapsed time (ms)')
    axes[0, 0].set_title('(a) Solver update only', loc='left')
    axes[0, 0].set_ylim(0, 5)
    axes[0, 1].set_title('(b) Full timer callback', loc='left')
    axes[0, 1].set_yscale('log')
    axes[0, 1].set_ylim(.2, 2000)
    axes[0, 1].annotate('Startup / geometry admission', xy=(3.0, 1100), xytext=(5.0, 500),
                        fontsize=6.4, arrowprops=dict(arrowstyle='-', color=INK, lw=.6))
    ax = axes[1, 0]
    mapped_services = [r for r in services if r['mapped_iteration_progress_s'] is not None]
    ax.scatter([r['mapped_iteration_progress_s'] / 60 for r in mapped_services],
               [r['callback_ms'] for r in mapped_services], s=4, color=GRAY, alpha=.20, linewidths=0)
    quantile_lines(ax, mapped_services, 'mapped_iteration_progress_s', 'callback_ms')
    ax.set_title('(c) Answering one peer request', loc='left')
    ax.set_ylabel('Service elapsed time (ms)')
    ax.set_yscale('log'); ax.set_ylim(.0008, 300)
    ax.text(.03, .96, 'Time assigned to serving iteration*', transform=ax.transAxes,
            va='top', fontsize=6.3, color=INK)
    ax = axes[1, 1]
    events = [e for e in raw['events'] if e['type'] == 'epoch_published']
    x = [(e['published_wall_ns'] - raw['start']['wall_ns']) / 60e9 for e in events]
    ax.step(x, [e['keyframes'] for e in events], where='post', color=BLUE, lw=1.6, label='Submap poses')
    ax.step(x, [e['loops'] for e in events], where='post', color=ORANGE, lw=1.5, label='Retained loops')
    ax.set_title('(d) Published global graph', loc='left')
    ax.set_ylabel('Count'); ax.set_ylim(0, 420)
    ax.legend(loc='upper left', fontsize=6.8)
    ax.text(.97, .56, '366 submaps\n111 loops\n110 geometry pairs', transform=ax.transAxes,
            ha='right', fontsize=7, color=INK)
    for ax in axes[1]:
        ax.set_xlabel('Minutes after common playback release')
    handles = [Line2D([], [], color=c, marker=m, ls='', ms=3.6,
                      label=f'{s.capitalize()} exchange selected') for s, c, m in stages]
    handles += [Line2D([], [], color=INK, lw=1.35, label='Median (30 s bins)'),
                Line2D([], [], color=INK, ls='--', lw=1.1, label='95th percentile')]
    fig.legend(handles=handles, loc='upper center', bbox_to_anchor=(.52, .965), ncol=2,
               columnspacing=1.6, fontsize=7)
    fig.suptitle('Online GRACO · CBS computation as the graph grows', y=1.015, fontsize=11)
    fig.subplots_adjust(top=.85, bottom=.12, left=.09, right=.98, hspace=.36, wspace=.31)
    fig.text(.09, .012, '842 timed updates across 14 robots; gray region: after last bag ends.\n'
             '*Peer requests have no wall timestamps; their serving-iteration timestamps locate them approximately.',
             fontsize=6.7, color=INK)
    finish(fig, 'cbs-update-progress')

    fig, axes = plt.subplots(1, 2, figsize=(7.2, 3.15))
    categories = [
        ('New factors admitted', ORANGE, '^', lambda r: r['new_factors'] > 0),
        ('Beliefs only', BLUE, 'o', lambda r: not r['new_factors'] and r['meaningful_update']),
        ('No new information', GRAY, '.', lambda r: not r['meaningful_update']),
    ]
    for ax, key, title in zip(axes, ['update_ms', 'active_callback_ms'],
                              ['Solver update only', 'Full timer callback']):
        for label, color, marker, keep in categories:
            rows = [r for r in updates if keep(r)]
            ax.scatter([r['local_optimizer_variables'] for r in rows], [r[key] for r in rows],
                       s=13, marker=marker, color=color, alpha=.45, linewidths=0, label=label)
        ax.set_xlabel('Variables in the local optimizer')
        ax.set_ylabel('Elapsed time (ms)'); ax.set_yscale('log'); ax.set_title(title, loc='left')
        ax.set_xlim(0, 65)
    fig.legend(*axes[0].get_legend_handles_labels(), loc='upper center', bbox_to_anchor=(.52, 1.04), ncol=3)
    fig.subplots_adjust(top=.81, bottom=.25, left=.08, right=.98, wspace=.30)
    fig.text(.08, .04, 'Variables include owned submaps, peer separator poses and anchors.\n'
             'One concurrent run; caching, new evidence and GPU contention vary. These points are not a complexity bound.',
             fontsize=6.7, color=INK)
    finish(fig, 'cbs-local-graph-scaling')

    fig, axes = plt.subplots(4, 4, figsize=(10, 7.6), sharex=True, sharey=True)
    for ax, (robot, data) in zip(axes.flat, raw['robots'].items()):
        rows = [r for r in updates if r['robot'] == robot]
        x = [r['progress_s'] / 60 for r in rows]
        ax.plot(x, [r['active_callback_ms'] for r in rows], color=ORANGE, lw=.8, alpha=.8)
        ax.plot(x, [r['update_ms'] for r in rows], color=BLUE, lw=.8)
        ax.axvline(last_bag, color=GRAY, lw=.7, ls=':')
        ax.set_yscale('log'); ax.set_ylim(.03, 2000); ax.set_xlim(2.5, 15.4)
        ax.set_title(robot.replace('aerial', 'A').replace('ground', 'G'), loc='left', fontsize=8)
        ax.text(.98, .96, f"{summary['validation'][list(raw['robots']).index(robot)]['final_variables']} final variables",
                transform=ax.transAxes, fontsize=6.5, ha='right', va='top')
    for ax in list(axes.flat)[14:]: ax.axis('off')
    handles = [Line2D([], [], color=BLUE, label='Solver update'),
               Line2D([], [], color=ORANGE, label='Full timer callback')]
    fig.legend(handles=handles, loc='lower right', bbox_to_anchor=(.98, .10), fontsize=8)
    fig.supxlabel('Minutes after playback release', fontsize=9)
    fig.supylabel('Elapsed time per update (ms, logarithmic scale)', fontsize=9)
    fig.suptitle('Per-robot CBS timings · all recorded nonterminal updates', fontsize=12)
    fig.subplots_adjust(left=.075, right=.985, top=.91, bottom=.075, hspace=.34, wspace=.18)
    finish(fig, 'cbs-per-robot-progress')


def report(raw, updates, services, summary):
    rows = []
    for robot in raw['robots']:
        group = [r for r in updates if r['robot'] == robot]
        u = distribution([r['update_ms'] for r in group])
        c = distribution([r['active_callback_ms'] for r in group])
        rows.append(dict(robot=robot, owned_submaps=group[-1]['owned_submaps'],
            local_variables=group[-1]['local_optimizer_variables'],
            owned_registration_pairs=group[-1]['local_registration_pairs'],
            timed_updates=len(group), update_median_ms=u['median'], update_p95_ms=u['p95'],
            update_max_ms=u['max'], callback_p95_ms=c['p95'], callback_max_ms=c['max']))
    write_csv(ROOT / 'per-robot.csv', rows)
    u, c, s = (summary[k] for k in ('update_ms', 'active_callback_ms', 'service_callback_ms'))
    paragraphs = [
        'The local solver remained inexpensive as the published global graph grew from 72 to 366 submap poses '
        'and from 22 to 110 admitted GPU registration pairs. These are 14 concurrent local optimizers, '
        'not one solver containing the whole global graph. The largest recorded local estimate has 61 variables.',
        f"Across {len(updates)} recorded nonterminal updates, solver wall time was {u['median']:.3f} ms median, "
        f"{u['p95']:.3f} ms at the 95th percentile and {u['max']:.3f} ms maximum. "
        f"Full timer callbacks were {c['median']:.3f} / {c['p95']:.3f} / {c['max']:.3f} ms respectively. "
        'The callback includes lock waiting, successful input preparation/admission, registration preparation, '
        'the solver update, residual/statistics evaluation, asynchronous request submission and visualization. '
        'A callback returning before performing an update is not in this CSV.',
        f"The {len(services):,} logged peer-service responses took {s['median']:.3f} ms median, "
        f"{s['p95']:.3f} ms at the 95th percentile and {s['max']:.3f} ms maximum. "
        'These include request-lock waiting, cavity marginalization and response packing. They are separate '
        'from the update callback and are not summed into a fictitious synchronized global cycle.',
        'All times are elapsed wall time, not CPU time or a GPU kernel microbenchmark. GPU work can be '
        'charged to different enclosing calls; the full callback and peer-service measurements are the '
        'broader scopes. Waiting for asynchronous peer replies, DDS delivery, separate PCM/exchange callbacks '
        'and Python bridge preparation are not a complete end-to-end latency measurement here.',
        'Update timestamps come from the matching robot/iteration statistics receipt. Peer-service CSV rows '
        'contain only an iteration number, so their plotted times are assigned to the serving robot’s iteration '
        'timestamp, approximately locating each request within that iteration. Requests before iteration 1 '
        'have no matching timestamp and appear only in the retained data and duration statistics. Duration measurements are native '
        'steady-clock measurements. One terminal timer callback per robot is missing by design and is never '
        'filled with zero; the final geometry-validation cost is therefore not covered by these update maxima.',
        'The actual recorded configuration selects outgoing pose or anchor exchange randomly at each iteration. '
        'It does not strictly alternate. Both categories run the local optimizer; the labels describe the '
        'exchange selected after the update, not two independent solver kernels. Earlier report wording has been corrected.',
        'Gray shading marks time after the last bag ended. The late decrease in cost occurs while the graph is '
        'fixed and fewer new beliefs/factors arrive; it does not imply that larger graphs are intrinsically cheaper. '
        'The recorded solver updates show no growth-related bottleneck at this tested size, but this is not a controlled asymptotic '
        'scalability experiment or a Jetson benchmark.',
    ]
    titles = ['Progress and graph growth', 'Cost versus local graph size', 'Every robot']
    names = ['cbs-update-progress', 'cbs-local-graph-scaling', 'cbs-per-robot-progress']
    body = '<!doctype html><meta charset="utf-8"><title>CBS computation and graph growth</title>'
    body += '<style>body{font:16px/1.6 system-ui;color:#23333f;background:#f6f8fa;max-width:1200px;margin:35px auto;padding:0 22px}img{width:100%;background:white}table{border-collapse:collapse;width:100%;font-size:13px}td,th{padding:7px;text-align:right;border-bottom:1px solid #dce3e7}td:first-child,th:first-child{text-align:left}a{color:#0072b2}.lead{background:#eaf2f7;padding:15px}</style>'
    body += '<p><a href="../index.html">← Full online experiment</a></p><h1>CBS computation as the graph grows</h1>'
    body += '<p class="lead">'+html.escape(paragraphs[0])+'</p>'
    body += '<p>'+html.escape(paragraphs[1])+'</p>'
    for name, title in zip(names, titles):
        body += f'<h2>{title}</h2><p><a href="{name}.pdf">Vector PDF</a> · <a href="{name}.png">PNG</a></p><img src="{name}.png" alt="{title}">'
    body += '<h2>Measurement scope and interpretation</h2>'
    body += ''.join('<p>'+html.escape(p)+'</p>' for p in paragraphs[2:])
    headers = ['Robot', 'Submaps', 'Variables', 'Geometry pairs', 'Update P50 / P95 / max (ms)', 'Callback P95 / max (ms)']
    body += '<h2>Per-robot summary</h2><table><tr>'+''.join('<th>'+h+'</th>' for h in headers)+'</tr>'
    for r in rows:
        cells = [r['robot'], str(r['owned_submaps']), str(r['local_variables']), str(r['owned_registration_pairs']),
                 ' / '.join(f"{r[k]:.3f}" for k in ('update_median_ms', 'update_p95_ms', 'update_max_ms')),
                 f"{r['callback_p95_ms']:.3f} / {r['callback_max_ms']:.3f}"]
        body += '<tr>'+''.join('<td>'+html.escape(v)+'</td>' for v in cells)+'</tr>'
    body += '</table><p>Geometry-pair counts are the accepted pairs reported by each native adapter, not all pairs touching that robot.</p>'
    body += '<p><a href="updates.csv">All update rows</a> · <a href="peer-services.csv">Peer-service rows</a> · <a href="per-robot.csv">Per-robot CSV</a> · <a href="summary.json">Summary and validation</a> · <a href="raw-timing.json">Original counters and hashes</a> · <a href="gen_fig_cbs_timing.py">Reproduction script</a></p>'
    (ROOT / 'index.html').write_text(body)
    (ROOT / 'REPORT.md').write_text('# CBS computation and graph growth\n\n'+'\n\n'.join(paragraphs)+'\n')


def main():
    raw = read(ROOT / 'raw-timing.json')
    audit = read(ROOT.parent / 'audit.json')
    updates, services, summary = build_data(raw, audit)
    write_csv(ROOT / 'updates.csv', updates)
    write_csv(ROOT / 'peer-services.csv', services)
    (ROOT / 'summary.json').write_text(json.dumps(summary, indent=2)+'\n')
    plots(raw, updates, services, summary)
    report(raw, updates, services, summary)
    print(json.dumps({k: summary[k] for k in ('timed_updates', 'service_requests', 'update_ms',
                     'active_callback_ms', 'service_callback_ms', 'maximum_local_variables')}, indent=2))


if __name__ == '__main__':
    main()
