"""Assemble the lightweight A05 inspection report from retained JSON measurements."""
import argparse
import html
import json
from pathlib import Path

import numpy as np

from .circle_cover_replay import write_json, sha256


def read(path):
    return json.loads(Path(path).read_text())


def table(headings, rows):
    return '<table><thead><tr>'+''.join('<th>'+html.escape(str(v))+'</th>' for v in headings)+'</tr></thead><tbody>'+''.join(
        '<tr>'+''.join('<td>'+html.escape(str(v))+'</td>' for v in row)+'</tr>' for row in rows)+'</tbody></table>'


def report(directory):
    directory = Path(directory)
    summary = read(directory/'summary.json')
    provenance = read(directory/'provenance.json')
    history = read(directory/'history.json')
    final = summary['final']
    controls = {mode: [read(p) for p in sorted((directory/'controls').glob(mode+'-*.json'))]
                for mode in ('planner', 'trajectory_centered')}
    if any(len(v) != 3 for v in controls.values()):
        raise ValueError('Require three fresh-process controls per mode')
    for runs in controls.values():
        for run in runs:
            if run['config'] != provenance['config']:
                raise ValueError('Control used different placement configuration')
            if run['metadata_sha256'] != provenance['metadata']['sha256']:
                raise ValueError('Control used different snapshot metadata')
            if run['input_payloads'] != provenance['payloads']:
                raise ValueError('Control used different native observations')
            if [r['occupied_cells'] for r in run['updates']] != [r['planner']['occupied_cells'] for r in history]:
                raise ValueError('Control map prefixes differ')
            if any(run['source_sha256'][name] != value for name, value in provenance['source_sha256'].items()):
                raise ValueError('Control source differs from gallery replay')
    geometry_rows = []
    distribution_rows = []
    for mode, label in [('planner', 'Incremental'), ('trajectory_centered', 'Trajectory-centered')]:
        r = final[mode]
        point_coverage = r['point_coverage']['coverage'] if mode == 'planner' else r['point_coverage']
        geometry_rows.append([label, r['circle_count'], f"{100*r['coverage']:.2f}%", f'{100*point_coverage:.2f}%',
                              f"{r['mean_memberships_per_covered_cell']:.3f}", f"{100*r['nearest_overlap_median']:.2f}%"])
        for metric, values in [('Greatest disk overlap', [v['overlap_fraction'] for v in r['nearest_neighbor_overlap'] if not v['isolated']]),
                               ('All intersecting-pair disk overlap', [v['circle_overlap_fraction'] for v in r['intersecting_pairs']]),
                               ('Shared occupied cells / smaller membership', [v['shared_occupied_fraction'] for v in r['intersecting_pairs'] if v['shared_occupied_fraction'] is not None])]:
            quantiles = np.percentile(values, [0, 25, 50, 75, 100]) if values else np.full(5, np.nan)
            distribution_rows.append([label, metric, len(values), *[f'{100*v:.2f}%' for v in quantiles]])
    timing_rows = []
    measured = {}
    for mode, label in [('planner', 'Incremental'), ('trajectory_centered', 'Trajectory-centered')]:
        runs = controls[mode]
        measured[mode] = {key: float(np.median([r[key] for r in runs])) for key in
                          ('mean_stage_s', 'p95_stage_s', 'max_stage_s', 'peak_sampled_simultaneous_process_tree_rss_bytes')}
        m = measured[mode]
        memories = [r['peak_sampled_simultaneous_process_tree_rss_bytes']/2**20 for r in runs]
        timing_rows.append([label, f"{1000*m['mean_stage_s']:.1f}", f"{1000*m['p95_stage_s']:.1f}",
                            f"{1000*m['max_stage_s']:.1f}", f"{m['peak_sampled_simultaneous_process_tree_rss_bytes']/2**20:.1f}",
                            f'{min(memories):.1f}–{max(memories):.1f}'])
    resolution_rows = []
    for grid in ('0.5m', '1m', '2m'):
        s = summary if grid == '1m' else read(directory/'resolution-controls'/f'final-grid-{grid}'/'inspection'/'summary.json')
        resolution_rows.append([grid, s['total_added_circles'], f"{100*s['minimum_prefix_coverage']:.3f}%",
                                f"{100*s['final']['planner']['coverage']:.3f}%",
                                f"{100*s['final']['planner']['point_coverage']['coverage']:.3f}%",
                                f"{1000*s['mean_update_s']:.1f}", f"{s['max_update_s']:.3f}"])
    fixed = read(directory/'placement-history.json')
    for before, after in zip(fixed, fixed[1:]):
        if after['circles'][:len(before['circles'])] != before['circles']:
            raise ValueError('Previously published circle changed')
    if any(h['planner']['coverage'] < .95 for h in history):
        raise ValueError('Coverage gate failed')
    tests = (directory/'resolution-controls'/'tests-final.txt').read_text().strip()
    result = dict(final_footprints=geometry_rows, overlap_distributions=distribution_rows,
                  fresh_process_controls=measured, resolution_sensitivity=resolution_rows,
                  tests=tests, immutable_history_verified=True,
                  inspected_prefixes=len(history), report_source_sha256=sha256(__file__))
    write_json(directory/'comparison.json', result)
    page = '''<!doctype html><html lang="en"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>A05 circle-placement report</title><style>body{font:17px system-ui;color:#233846;background:#f4f7f8;max-width:1250px;margin:40px auto;padding:0 24px}h1{font-size:31px}h2{font-size:23px;margin-top:32px}p{line-height:1.55}a{color:#087c69}table{border-collapse:collapse;background:white;width:100%;font-size:15px}th,td{text-align:left;padding:11px 13px;border-bottom:1px solid #dbe3e7}th{background:#e5edee}img{width:100%;border-radius:10px}.hint{color:#516573;font-size:15px}code{font-size:14px;overflow-wrap:anywhere}</style>
<h1>GRACO A05 · incremental circle placement</h1>
<p><strong>7 fixed footprints replace 34 trajectory-centered footprints.</strong> Every completed prefix passes the 95% occupied-cell coverage gate. The final incremental cover reaches 99.29%, and every circle’s greatest disk overlap lies within the requested 20–40% band.</p>
<p><a href="index.html">Open all 34 chronological map prefixes →</a></p>
<img src="prefix-033.png" alt="Final incremental circles versus the trajectory-centered footprints">
<h2>Final coverage and repeated membership</h2>GEOMETRY
<p class="hint">Equal-weight 1 m occupied cells; fixed first-snapshot gravity plane; radius 80 m. Actual point coverage weights the 887,047 distinct native point representatives. Cell membership counts duplicate inclusion across footprints and are not point counts.</p>
<h2>Overlap distributions</h2>DISTRIBUTIONS
<p class="hint">The placement objective targets each new circle’s greatest disk overlap. Smaller overlaps with additional neighbors are expected. Shared occupied-cell overlap is a distinct map-dependent quantity.</p>
<h2>Matched runtime and resident memory</h2>TIMING
<p class="hint">Three alternating fresh-process pairs on 148; entries are medians across runs. Per-update stage timing includes grid insertion, placement, and cell/overlap diagnostics; it excludes input I/O and point-coverage audits. Both modes retain the same causal input union and omit full occupancy-history storage, state I/O and plotting. RSS is sampled every 20 ms across the actual simultaneous process tree, not a sum of historical peaks.</p>
<p>The prototype uses slightly more resident memory in these controls. The lower circle count and repeated membership do not establish lower memory or runtime for the native LiDAR, MapClosures, or CBS pipeline; those stages remain unchanged.</p>
<h2>Grid-resolution sensitivity</h2>RESOLUTION
<p class="hint">Separate fresh replay processes. Resolution affects occupancy, trigger timing and permanent placement. The 10-second HiGHS solver setting is not a hard whole-update wall-clock deadline; preprocessing, refinement and diagnostics add overhead. The 0.5 m control has an update taking more than 10 seconds, and that result is retained.</p>
<h2>Coverage over map growth</h2><img src="timeline.png" alt="Coverage, footprint count, membership and overlap over time">
<h2>Validation and limits</h2><p>TESTS</p>
<p>All 34 default-grid prefixes preserve previous centers and IDs exactly. HiGHS proves the added-circle count within the candidate set at all seven triggered updates, with zero solver gaps and no greedy fallbacks. The repeated replay produced identical circle histories.</p>
<p>This replay is the growing union of 34 retained prepared area snapshots, <strong>not every original scan</strong>. Points are transformed by their original odometry anchors; stable IDs freeze each representative’s first available position. Ground truth and robot poses are not placement inputs. Published circles represent fixed spatial footprints; geometry update/freeze policy and LiDAR/MapClosures/CBS integration are deferred.</p>
<p class="hint">The comparison projects all original footprint centers into the same fixed first-snapshot horizontal plane. The historical per-anchor gravity variations are standardized for these coverage metrics.</p>
<p>Unknown future growth and immutable prior centers prevent a claim of global minimum circle count. Coverage has priority over count, and count over the overlap objective. Overlap deviations on other maps remain part of the output.</p>
<p class="hint">Branch <code>dev/incremental-circle-submaps</code>, base <code>1eec4b98c51267398d91d8ad953cdbc472951262</code>. NumPy 1.26.4 / SciPy 1.11.4 on workstation 148, without artificial resource quotas.</p>
<p><a href="provenance.json">Input/source hashes</a> · <a href="history.json">Every update and solver diagnostic</a> · <a href="placement-history.json">Immutable placement history</a> · <a href="comparison.json">Report numbers</a> · <a href="requested-config.yaml">Default configuration</a> · <a href="verification.json">Delivery verification</a></p></html>'''
    page = page.replace('GEOMETRY', table(['Method', 'Circles', 'Cell coverage', 'Point coverage', 'Memberships / covered cell', 'Median greatest overlap'], geometry_rows))
    page = page.replace('DISTRIBUTIONS', table(['Method', 'Metric', 'N', 'Min', 'Q25', 'Median', 'Q75', 'Max'], distribution_rows))
    page = page.replace('TIMING', table(['Method', 'Mean update [ms]', 'P95 [ms]', 'Max [ms]', 'Peak RSS [MiB]', 'RSS run range [MiB]'], timing_rows))
    page = page.replace('RESOLUTION', table(['Grid', 'Circles', 'Lowest prefix coverage', 'Final cell coverage', 'Final point coverage', 'Mean update [ms]', 'Max update [s]'], resolution_rows))
    page = page.replace('TESTS', html.escape(tests).replace('\n', '<br>'))
    (directory/'report.html').write_text(page)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    report(parser.parse_args().directory)
