"""Reproduce the measured CBS submap comparison and its compact gallery."""
from pathlib import Path
import json
import os
import numpy as np
os.environ.setdefault('MPLCONFIGDIR','/tmp/circle-matplotlib')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT=Path(__file__).resolve().parent
NAMES={'circle40':'Fixed circles · 40 m','trajectory40':'Trajectory centered · 40 m',
       'trajectory80':'Trajectory centered · 80 m'}
COLORS=['#347f89','#b79b69','#818b9f']


def table(headers,rows):
    return '\n'.join(['| '+' | '.join(headers)+' |','| '+' | '.join(['---']*len(headers))+' |']+
        ['| '+' | '.join(map(str,r))+' |' for r in rows])


def main():
    data=json.loads((ROOT/'comparison.json').read_text());rows=[];comm=[];native=[];individual=[]
    repair_path=ROOT/'persistent-fixed/result.json'
    repair_note=None
    if repair_path.exists():
        repair=json.loads(repair_path.read_text())
        audit=json.loads((repair_path.parent/'published-transform-audit.json').read_text())
        if repair['success'] and audit['success']:
            evaluation=json.loads((repair_path.parent/'evaluation.json').read_text())
            ate=next(iter(evaluation['joint_ate'].values()))['rmse_m']
            repair_note=(f"Persistent CBS is now validated on this four-robot circle graph: all four robots "
                f"reach A05's reference, with {ate:.3f} m joint ATE after 30 independent iterations "
                "at a requested 10-second cadence. Two defects were repaired: disabled multi-hop anchor "
                "relay and a factor-slot/variable-key mix-up that discarded odometry initialization. "
                "Published loop transforms and all 13 geometry factors pass. Stages remain independently "
                "random; no coordinated stage barrier or centralized initialization was added.")
    for mode,e in data.items():
        s=e['summary'];r=e['resources'];c=e['communication'];ev=e['evaluation']
        assert len(ev['joint_ate'])==1 and len(set(ev['components'].values()))==1
        ate=next(iter(ev['joint_ate'].values()))['rmse_m']
        central=next(iter(e['central']['evaluation']['joint_ate'].values()))['rmse_m']
        rows.append([NAMES[mode],s['keyframes'],f"{s['loops']}/{s['proposed_loops']}",
            e['registration']['registration_factor_count'],f'{ate:.3f}',f'{central:.3f}',
            f"{r['wall_s']:.2f}",f"{r['parent_cpu_s']+r['child_cpu_s']:.2f}",
            f"{c['total_graph_application_cdr_bytes']/2**20:.2f}",
            f"{r['peak_simultaneous_process_tree_rss_bytes']/2**30:.2f}"])
        comm.append([NAMES[mode],f"{c['cbs_requests_plus_responses_cdr_bytes']/1024:.1f}",
            f"{c['pcm_network_cdr_bytes']/1024:.1f}",f"{c['registration_sent_cdr_bytes']/2**20:.3f}",
            f"{c['coordination_sent_cdr_bytes']/1024:.1f}",e['evidence']['endpoints']])
        for robot,t in e['native_timing'].items():
            a=t['timer_update'];b=t['belief_marginalization']
            native.append([NAMES[mode],robot,e['registration']['robots'][robot]['registration_factor_count'],
                f"{a['median_ms']:.3f}",f"{a['p95_ms']:.3f}",f"{b['median_ms']:.3f}",f"{b['p95_ms']:.3f}"])
            individual.append([NAMES[mode],robot,f"{ev['raw_ate'][robot]['rmse_m']:.3f}",
                f"{ev['individual_ate'][robot]['rmse_m']:.3f}"])
    circle=data['circle40'];control=data['trajectory40']
    reduction=1-circle['communication']['total_graph_application_cdr_bytes']/control['communication']['total_graph_application_cdr_bytes']
    cpu=lambda e:e['resources']['parent_cpu_s']+e['resources']['child_cpu_s']
    joint=lambda e:next(iter(e['evaluation']['joint_ate'].values()))['rmse_m']
    central_joint=next(iter(circle['central']['evaluation']['joint_ate'].values()))['rmse_m']
    summary=(f"The fixed-circle CBS graph connects A05, A07, G05 and G06 with 121 owned submap poses and 13 live GPU registration factors. "
        f"Joint ATE is {joint(circle):.3f} m versus {central_joint:.3f} m for the centralized circle graph. Relative to 40 m trajectory-centered CBS, "
        f"graph-stage communication falls by {reduction:.1%} and measured CPU time by {1-cpu(circle)/cpu(control):.1%}, "
        f"while joint ATE changes from {joint(control):.3f} to {joint(circle):.3f} m. These are single-run controls on 148.")
    main_table=table(['Submaps','Nodes','Retained/proposed loops','GPU factors','CBS joint ATE [m]',
        'Central joint ATE [m]','CBS stage [s]','CPU [s]','Graph traffic [MiB]','RSS [GiB]'],rows)
    repeated=json.loads((ROOT/'circle40-repeat.json').read_text())
    repeat_note=(f"A separate fresh-process fixed-circle repeat also connects all four robots and passes all 13 "
        f"registration checks: joint ATE {joint(repeated):.6f} m versus {joint(circle):.6f} m in the main run, "
        f"wall time {repeated['resources']['wall_s']:.2f} s, graph traffic "
        f"{repeated['communication']['total_graph_application_cdr_bytes']/2**20:.3f} MiB. "
        "This is a reproducibility check, not a timing distribution. See `circle40-repeat.json`.")
    residual_rows=[[NAMES[m],f"{max(v['translation_m'] for v in e['published_loop_residuals']):.3f}",
        f"{max(v['rotation_deg'] for v in e['published_loop_residuals']):.3f}"] for m,e in data.items()]
    body='# Distributed CBS over fixed spatial submap graphs\n\n'+summary+'\n\n'+main_table+'\n\n'+repeat_note+'''\n
All three controls connect all four robots. PCM rejects no verified loop in these captures. The 80 m
control retains 126 companion pose constraints but admits 125 registration factors: one geometry
factor fails initial overlap, as in the centralized control. Every admitted factor passes final support.

## Native message-order fix

A fresh-process repeat of the original CBS build lost G06's root reference. A relayed anchor had arrived
before its publishing robot's connecting separator; the receiver discarded it and unchanged subsequent
messages were suppressed. Anchor requests now acknowledge the receiver's factor-connected anchors.
When the publisher frame becomes connected but the target anchor remains missing, the sender refreshes
its bootstrap reply. Cached, unconnected pose guesses do not count as acknowledgements. The Gaussian,
producer identity, covariance and receiver's belief-update rules are unchanged. All peers must use the
matching rebuilt Keys service interface. The regression checks refresh gating, unchanged Gaussian
contents and stopping refresh after acknowledgement. All seven core tests pass. Final controls use
fresh coordinator/robot processes. The original failing run remains in `sealed-before-core-fix.json`.

An earlier attempted repair retained early anchor beliefs directly. It worsened alignment and failed
the 80 m control's final geometry checks, so it was reverted. Its results remain in
`rejected-anchor-retention.json`; they are not the implementation evaluated in the table.
No thresholds or measurement objectives were adjusted.

## Verification and shared-frame residuals

The local focused suite passes 35 tests, with six native checks skipped. The native distributed suite
passes 26 tests, with one separately gated legacy check skipped. The isolated build passes seven CBS
core and eight ROS C++ tests; one separately gated ROS distributed test is skipped. These overlapping
suites are not summed. Logs and JUnit results accompany the report.

The following diagnostics compare each verified loop measurement with the relative transform between
the two robots' published, owned poses. They use no ground truth and can expose inconsistent published
frames even when local factor support succeeds. They are residuals against noisy loop measurements,
not an independent accuracy estimate or an additional tuned acceptance threshold.

'''+table(['Submaps','Maximum translation residual [m]','Maximum rotation residual [deg]'],residual_rows)+'''

## What was implemented

Each fixed footprint owns a CBS Pose3 variable keyed by robot and submap ID. Creation-anchor IMU
frames, duplicate creation timestamps, immutable point membership and sealing availability remain
distinct. Robot-local odometry constraints connect these nodes; PCM-retained cross-robot loops add
pose constraints and the existing GPU VGICP factors. Geometry moves only after PCM, under canonical
ownership. Prepared-array hashes match the centralized controls for all 23 / 51 / 81 transferred
endpoints. Neither central optimized poses nor ground truth initializes CBS.

The dense trajectory and map reconstruction policy is unchanged: interpolate correction transforms
through creation anchors, average simultaneous-anchor corrections, and apply each submap correction
to points assigned by nearest footprint. The optimizer retains every submap even when anchors share
a timestamp. The interactive view displays all dense trajectory samples with a single shared rigid
alignment; it does not align robots individually.

## Execution and the persistent limitation

**These successful controls use sealed CBS, with 100 local iterations at a requested 10 Hz.**
They are frozen graph replays using existing verified loops. Geometry was sealed at capture end.
No online geometry-freezing or live retrieval result is claimed.

The new transport also supports persistent prefixes, and the three-robot growing-graph test passes.
However, the real four-robot chain exposed an existing persistent reference-propagation failure:
G06 remains in an A07 reference while the other robots reach A05. The first ten-iteration run was
rejected by the shared-frame check; keeping it running did not cure the missing multi-hop anchor
belief. The new persistent finalization barrier detects this condition before sealing and times out
instead of publishing a falsely aligned result. It does not repair the native algorithm. No silent
fallback occurred: the successful runs explicitly selected sealed mode. Failed-run status and logs
are retained in `persistent-failure.json`.

A separate regression test exposed a DDS startup race for a first prefix with no loops: a ready packet
could be sent before peer discovery. The persistent bridge now waits for subscribers before declaring
readiness. This transport fix is distinct from the unresolved multi-hop reference limitation.

## Measurement scope

The CBS-stage timer includes launch, distributed PCM, geometry preparation/exchange, optimization,
fixed settling and shutdown. It excludes reused placement, BEV/ORB preparation, retrieval, verification,
local store/hash preparation, and post-run evaluation. The 10-second settling setting masks part of the
computation difference. CPU is accumulated across the measured process tree; it can exceed wall time.
RSS is an actual simultaneous 20 ms process-tree sample, with possible shared-page double counting.
Whole-device GPU peaks are retained in comparison.json, sampled at 100 ms; they include driver overhead
and any unrelated device use. Each control uses a fresh coordinator and fresh robot processes.
No artificial CPU, memory or GPU quota was imposed on 148.

Graph communication is serialized application CDR, counting each request/response at the requester
and other sent messages per recipient. DDS discovery, network headers and retransmissions are excluded.
Reused descriptor retrieval and verification traffic is not included. Thus these values must not be
compared directly with the earlier full centralized backend totals of 86 / 198 / 720 MiB.

## Graph-stage communication breakdown

'''+table(['Submaps','CBS request+response [KiB]','PCM [KiB]','Geometry [MiB]','Coordination [KiB]','Unique prepared endpoints'],comm)+'''\n
Geometry is prepared once per owned/supplied endpoint and reused across factors. CBS uses the exact
same fixed 0.4 m verification preprocessing followed by the existing adaptive 0.5 m / 12,000-point
registration cap. Actual effective voxel sizes are retained in the manifests; 0.5 m is not guaranteed
after the cap. GPU voxel settings and information caps are unchanged.

## Native per-robot timing

'''+table(['Submaps','Robot','Owned GPU factors','Timer update median [ms]','Timer update p95 [ms]',
    'Belief marginalization median [ms]','Belief marginalization p95 [ms]'],native)+'''\n
These are the logged native timer updates and belief marginalizations, not total pipeline latency.
Timer CSVs contain 99 nonterminal callbacks; the final iteration is separately marked complete.
Registration preparation, waiting, startup, transport and other callback work are outside these narrow
columns. The process-tree CPU and wall measurements above include the complete graph-stage lifetime.

## Individual ATE, each with its own rigid alignment

'''+table(['Submaps','Robot','Raw ATE [m]','CBS ATE [m]'],individual)+'''\n
Headline joint ATE uses one shared SE(3) alignment for all four robots. Evaluation uses evo 1.36.5,
50 ms association, no scale fitting, and the corrected GRACO IMU-reference ground truth. Ground truth
is used only after the solver has finished. Input/configuration/source/native hashes, per-robot poses,
factor manifests, PCM decisions, timing CSVs and evaluation results accompany this report.
'''
    cadence=ROOT/'persistent-10s/result.json'
    if cadence.exists():
        probe=json.loads(cadence.read_text())
        body+='\n## Persistent CBS at 10-second intervals\n\n'
        body+=(f"A separate persistent test completed {probe['iterations_requested']} iterations per robot "
            f"at a requested {probe['interval_requested_s']:g}-second cadence. "
            + ('The final validation passed. ' if probe['success'] else
               'The final common-frame check failed; G06 remained in A07’s frame. ')
            + '[Cadence, timing, geometry checks and retained failure evidence](persistent-10s/index.html).\n')
    if repair_note:
        body=body.replace('## Execution and the persistent limitation', '## Execution and the original persistent limitation')
        body=body.replace('However, the real four-robot chain exposed', 'Before the repair below, the real four-robot chain exposed')
        body=body.replace('unresolved multi-hop reference limitation', 'original multi-hop reference defect')
        body+='\n## Validated persistent repair\n\n'+repair_note+'\n\n[Corrected persistent report](persistent-fixed/index.html). The relay-only intermediate was rejected despite matching reference labels: 87.899 m joint ATE. Its [audit](persistent-anchor-relay/index.html) is retained.\n'
    (ROOT/'REPORT.md').write_text(body)
    plt.rcParams.update({'font.family':'serif','font.serif':['DejaVu Serif'],'font.size':9,
        'axes.spines.top':False,'axes.spines.right':False,'pdf.fonttype':42,'savefig.dpi':300})
    fig,axes=plt.subplots(1,4,figsize=(11,3.1));es=[data[k] for k in NAMES]
    plots=[('Joint ATE [m]',[next(iter(e['evaluation']['joint_ate'].values()))['rmse_m'] for e in es]),
        ('Graph-stage wall [s]',[e['resources']['wall_s'] for e in es]),
        ('Graph traffic [MiB]',[e['communication']['total_graph_application_cdr_bytes']/2**20 for e in es]),
        ('Simultaneous RSS [GiB]',[e['resources']['peak_simultaneous_process_tree_rss_bytes']/2**30 for e in es])]
    for ax,(title,values) in zip(axes,plots):
        bars=ax.bar(range(3),values,color=COLORS,width=.64)
        ax.set_xticks(range(3),['Circle\n40 m','Trajectory\n40 m','Trajectory\n80 m']);ax.set_title(title)
        ax.set_ylim(0,max(values)*1.22);ax.grid(axis='y',alpha=.17);ax.set_axisbelow(True)
        ax.bar_label(bars,labels=[f'{v:.3f}' if title.startswith('Joint') else f'{v:.2f}' for v in values],padding=3,fontsize=8)
    fig.suptitle('GRACO A05 / A07 / G05 / G06 · distributed CBS submap graphs',fontsize=12)
    fig.tight_layout()
    for ext in ('png','pdf'):fig.savefig(ROOT/f'comparison.{ext}')
    plt.close(fig)
    htmlrows=''.join('<tr>'+''.join(f'<td>{x}</td>' for x in row)+'</tr>' for row in rows)
    headers=['Submaps','Nodes','Loops','GPU factors','CBS ATE (m)','Central ATE (m)','Wall (s)','CPU (s)','Traffic (MiB)','RSS (GiB)']
    page='''<!doctype html><html lang="en"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>CBS spatial submap graphs · GRACO four robots</title><style>
body{margin:0;background:#f5f7fa;color:#22333d;font:16px/1.55 system-ui}main{max-width:1160px;margin:auto;padding:35px 24px}
h1{font-size:32px;line-height:1.2;margin:0 0 15px}h2{font-size:22px;margin-top:30px}a{color:#207584}
.card{padding:22px;background:white;border-radius:12px;margin:20px 0}.note{border-left:4px solid #b58b36;background:#fff7e7;padding:15px 20px}
img{width:100%;height:auto}table{border-collapse:collapse;width:100%;font-size:13px}td,th{padding:9px 7px;border-bottom:1px solid #dde4e8;text-align:right}
td:first-child,th:first-child{text-align:left}.scroll{overflow:auto}.links{display:flex;gap:24px;flex-wrap:wrap}
</style><main><h1>CBS over spatial submap graphs</h1><p>GRACO · A05, A07, G05, G06 · workstation 148 · 1 October 2026</p>'''
    page+='<p>'+summary+'</p><div class="links"><a href="aligned-trajectories-3d.html">Explore aligned trajectories in 3D</a><a href="REPORT.md">Detailed report</a><a href="comparison.pdf">Figure PDF</a><a href="comparison.json">Numerical results</a></div>'
    if cadence.exists():
        page+='<p><a href="persistent-10s/index.html">New: persistent CBS tested at one iteration every 10 seconds — timing and final validation</a></p>'
    page+='<div class="card"><img src="comparison.png" alt="Joint ATE, graph wall time, communication and memory for the three submapping controls"></div>'
    page+='<div class="card scroll"><table><thead><tr>'+''.join('<th>'+h+'</th>' for h in headers)+'</tr></thead><tbody>'+htmlrows+'</tbody></table></div>'
    page+='''<p>All four robots connect in each successful control. All admitted registration factors pass final checks.
    Frozen detection and verification are reused; the table measures the new peer-to-peer graph stage. Traffic excludes
    retrieval, network headers and discovery. Each CBS run retains the same 100-iteration settling setting.</p>
    <div class="note"><strong>Sealed CBS validated; persistent four-robot propagation remains unresolved.</strong>
    The persistent run reached A05’s frame on three robots but left G06 in A07’s frame. It was rejected and retained for
    diagnosis. The successful results explicitly use sealed CBS; no automatic fallback or hidden alignment repair was used.
    <a href="persistent-failure.json">Failure evidence</a>.</div>
    <h2>One shared alignment</h2><a href="aligned-trajectories-3d.html"><img src="aligned-trajectories-3d.png" alt="All four CBS-corrected trajectories in a shared 3D frame"></a>
    <p>Every dense trajectory sample is retained. Circle centers remain fixed in the original map; CBS optimizes each
    submap’s creation-anchor pose, then the same reconstruction policy is applied to trajectories and map points.</p>
    <p><a href="../circle-central-graco-four-20261001/index.html">Centralized comparison and BEVs</a> ·
    <a href="../../CIRCLE-CBS.md">Implementation and measurement specification</a></p></main></html>'''
    if repair_note:
        begin=page.index('<div class="note"><strong>Sealed CBS validated;')
        end=page.index('</div>',begin)+len('</div>')
        page=page[:begin]+'<div class="note"><strong>Persistent propagation repaired and validated.</strong><p>'+repair_note+'</p><a href="persistent-fixed/index.html">Corrected persistent report, timing and evidence</a></div>'+page[end:]
    (ROOT/'index.html').write_text(page)
    print('Saved report, gallery, comparison PNG/PDF')


if __name__=='__main__':main()
