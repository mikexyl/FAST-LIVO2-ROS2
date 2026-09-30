"""Compare radius/coverage trials on identical retained map prefixes, after replay."""
import argparse
import html
from pathlib import Path

import numpy as np

from .circle_cover import CircleCoverPlanner, covered_mask
from .circle_cover_replay import write_json, sha256
from .circle_cover_report import read, table


def compare(reference, trial, reference_state, trial_state):
    reference, trial = Path(reference), Path(trial)
    a, b = read(reference/'provenance.json'), read(trial/'provenance.json')
    ah, bh = read(reference/'history.json'), read(trial/'history.json')
    if (a['metadata'] != b['metadata'] or a['payloads'] != b['payloads'] or
            a['T_horizontal_map'] != b['T_horizontal_map'] or
            a['source_sha256']['circle_cover.py'] != b['source_sha256']['circle_cover.py'] or
            {k:v for k,v in a['config'].items() if k not in ('radius_m', 'coverage_target')} !=
            {k:v for k,v in b['config'].items() if k not in ('radius_m', 'coverage_target')} or
            [(h['available_ns'], h['planner']['occupied_cells']) for h in ah] !=
            [(h['available_ns'], h['planner']['occupied_cells']) for h in bh]):
        raise ValueError('Controls differ in inputs, map prefixes, frame, planner or other settings')
    planners = [CircleCoverPlanner.load(reference_state), CircleCoverPlanner.load(trial_state)]
    if not np.array_equal(planners[0].cells, planners[1].cells):
        raise ValueError('Final occupied grids differ')
    results = []
    rows = []
    for path, p, provenance in zip([reference, trial], planners, [a, b]):
        summary = read(path/'summary.json'); final = summary['final']['planner']
        results.append(dict(radius_m=p.config.radius_m, coverage_target=p.config.coverage_target,
                            circle_count=final['circle_count'],
                            minimum_prefix_coverage=summary['minimum_prefix_coverage'],
                            final_cell_coverage=final['coverage'], final_point_coverage=final['point_coverage']['coverage'],
                            greatest_overlap_median=final['nearest_overlap_median'],
                            overlap_in_band_fraction=final['nearest_overlap_in_band_fraction'],
                            mean_memberships_per_covered_cell=final['mean_memberships_per_covered_cell'],
                            mean_update_s=summary['mean_update_s'], max_update_s=summary['max_update_s'],
                            solver_updates=summary['solver_updates'], proven_updates=summary['proven_candidate_minimum_updates'],
                            greedy_fallback_updates=summary['greedy_fallback_updates'],
                            replay_memory=provenance['memory']))
        r = results[-1]
        rows.append([f"{r['radius_m']:g} m", f"{100*r['coverage_target']:g}%", r['circle_count'], f"{100*r['minimum_prefix_coverage']:.3f}%",
                     f"{100*r['final_cell_coverage']:.2f}%", f"{100*r['final_point_coverage']:.2f}%",
                     f"{r['mean_memberships_per_covered_cell']:.3f}", f"{100*r['greatest_overlap_median']:.2f}%",
                     f"{1000*r['mean_update_s']:.1f}", f"{1000*r['max_update_s']:.1f}"])
    fixed = read(trial/'placement-history.json')
    for prev, current in zip(fixed, fixed[1:]):
        if current['circles'][:len(prev['circles'])] != prev['circles']:
            raise ValueError('Published center or ID changed')
    if any(h['planner']['coverage'] < planners[1].config.coverage_target for h in bh):
        raise ValueError('Coverage gate failed')
    artifact = 'coverage-comparison' if results[0]['radius_m'] == results[1]['radius_m'] else 'radius-comparison'
    write_json(trial/f'{artifact}.json', dict(results=results, identical_causal_map_prefixes=True,
        immutable_trial_centers_verified=True, report_source_sha256=sha256(__file__)))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle
    xy = planners[0].cell_centers
    limits = [xy]
    for p in planners:
        centers = np.asarray([c.center_xy for c in p.circles])
        limits.extend([centers-p.config.radius_m, centers+p.config.radius_m])
    limits = np.vstack(limits); lo = limits.min(axis=0)-8; hi = limits.max(axis=0)+8
    fig, axes = plt.subplots(1, 2, figsize=(13, 6.3), constrained_layout=True)
    for ax, p, color, r in zip(axes, planners, ['#375875', '#168a76'], results):
        centers = [c.center_xy for c in p.circles]
        mask = covered_mask(xy, centers, p.config.radius_m)
        ax.scatter(xy[mask, 0], xy[mask, 1], s=.6, color='#8997a3')
        ax.scatter(xy[~mask, 0], xy[~mask, 1], s=3, color='#e07025')
        for c in p.circles:
            ax.add_patch(Circle(c.center_xy, c.radius_m, fill=False, ec=color, lw=1.1, alpha=.9))
            ax.plot(*c.center_xy, '+', color=color, ms=5)
            ax.text(c.center_xy[0]+2, c.center_xy[1]+2, str(c.id), color=color, fontsize=8)
        ax.set(xlim=(lo[0], hi[0]), ylim=(lo[1], hi[1]), aspect='equal', xlabel='Horizontal x [m]', ylabel='Horizontal y [m]',
               title=f"R={p.config.radius_m:g} m · target {100*p.config.coverage_target:g}% · {len(p.circles)} circles\n{100*r['final_cell_coverage']:.2f}% cell coverage · {r['mean_memberships_per_covered_cell']:.2f} memberships/cell")
        ax.grid(alpha=.12)
    fig.suptitle('GRACO A05 · incremental footprints on identical growing-map observations\nOrange: uncovered cells · 1 m grid · 30% greatest-overlap target', fontsize=12)
    fig.savefig(trial/f'{artifact}.png', dpi=145); fig.savefig(trial/f'{artifact}.pdf'); plt.close(fig)
    new = results[-1]
    stats = table(['Radius', 'Coverage target', 'Circles', 'Lowest prefix coverage', 'Final cell coverage', 'Point coverage',
                   'Memberships / covered cell', 'Median greatest overlap', 'Mean update [ms]', 'Max update [ms]'], rows)
    page = '''<!doctype html><html lang="en"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>A05 circle-placement comparison</title><style>body{font:17px system-ui;color:#213442;background:#f5f7f9;max-width:1300px;margin:35px auto;padding:0 20px}img{width:100%;border-radius:8px;background:white}p{line-height:1.5}a{color:#087a67}table{width:100%;border-collapse:collapse;background:white;font-size:14px}th,td{padding:10px;border-bottom:1px solid #dbe3e7;text-align:left}th{background:#e5edee}.hint{color:#526576;font-size:15px}</style>
<h1>GRACO A05 · COMPARISON_TITLE</h1><p>Same 34 causal retained snapshot prefixes, fixed first-snapshot gravity frame, 1 m grid, and 30% ±10 pp greatest-overlap band. CHANGED_SETTING</p>
<img src="ARTIFACT.png" alt="Incremental footprints under two settings on the same final observed map">TABLE
<p>VERDICT</p>
<p><a href="index.html">Inspect all 34 chronological 40 m prefixes →</a></p>
<p class="hint">The chronological gallery compares the new 40 m footprints with the original trajectory-centered footprints, which retain their native 80 m radius. The figure above instead compares the two incremental planners.</p>
<p class="hint">Times are per-update placement plus cell/overlap diagnostics in one replay each, excluding input I/O, point audits, state I/O and figure rendering. They are not full LiDAR or MapClosures processing times. Bulk geometry and occupancy states remain on 148.</p>
<p class="hint">The observed map is the growing union of retained native processed area snapshots, not every original scan. Circle count is certified only within each update’s candidate set when the solver proves it. Old centers remain fixed. Geometry refresh/freeze policy and LiDAR integration are deferred.</p>
<p><a href="ARTIFACT.json">Comparison numbers</a> · <a href="summary.json">40 m summary</a> · <a href="history.json">All solver/overlap diagnostics</a> · <a href="provenance.json">Source and input hashes</a> · <a href="requested-config.yaml">40 m configuration</a></p></html>'''
    verdict = (f"Every 40 m prefix meets the coverage gate. The final cover uses {new['circle_count']} circles, "
               f"with {100*new['final_cell_coverage']:.2f}% cell coverage and "
               f"{100*new['overlap_in_band_fraction']:.0f}% of circles inside the requested overlap band. "
               f"The solver proves the candidate-set count on {new['proven_updates']}/{new['solver_updates']} triggered updates; "
               f"{new['greedy_fallback_updates']} greedy fallbacks.")
    if artifact == 'coverage-comparison':
        title = f"{100*results[0]['coverage_target']:g}% versus {100*new['coverage_target']:g}% coverage at {new['radius_m']:g} m"
        changed = 'Only the required coverage changes.'
    else:
        title = f"{results[0]['radius_m']:g} m versus {new['radius_m']:g} m circles"
        changed = 'Only the placement radius changes.'
    (trial/'report.html').write_text(page.replace('TABLE', stats).replace('VERDICT', html.escape(verdict))
                                   .replace('ARTIFACT', artifact).replace('COMPARISON_TITLE', title)
                                   .replace('CHANGED_SETTING', changed))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--reference', type=Path, required=True)
    parser.add_argument('--trial', type=Path, required=True)
    parser.add_argument('--reference-state', type=Path, required=True)
    parser.add_argument('--trial-state', type=Path, required=True)
    args = parser.parse_args()
    compare(args.reference, args.trial, args.reference_state, args.trial_state)
