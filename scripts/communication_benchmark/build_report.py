#!/usr/bin/env python3
"""Publication figures and report section from measured, status-aware counters."""
import collections,csv,datetime,html,json,math,shutil
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import numpy as np

PROJECT=Path(__file__).resolve().parents[2]
ROOT=PROJECT/'research/figures/communication-benchmark148-20260928'
REPORT=PROJECT/'research/figures/s3e-graco-comparison-20260924'
NAMES={'ours':'Ours (10 s inputs)','swarm':'Swarm-SLAM','dcl':'DCL-SLAM','disco':'DiSCo-SLAM','gac':'GAC proxy'}
METHODS=list(NAMES)
plt.rcParams.update({'font.family':'serif','font.serif':['DejaVu Serif'],'font.size':8,
 'axes.titlesize':9,'axes.labelsize':8,'legend.fontsize':7,'legend.frameon':False,
 'pdf.fonttype':42,'ps.fonttype':42,'axes.spines.top':False,'axes.spines.right':False,
 'savefig.dpi':300})

def group_name(group):return group.replace('S3E_','').replace('GRACO_','GRACO ').replace('_',' ')
def components(row):return '+'.join(str(len(g)) for g in row['components']) if row.get('components') else 'unverified'
def completed(row):return row['status']=='complete' and row.get('valid_total')
def cell(row):
 if row['status']=='excluded':return 'I'
 if row['bytes'] is None:return 'pending' if row['status']=='pending' else row['status']
 if row['status'] in ['running','replaying','draining_cbs_queue','starting']:return 'running'
 suffix='' if completed(row) else (' F' if row['status'] in ['failed','stopped_divergence','timeout'] else ' ?')
 bound='' if row.get('valid_total') else '≥'
 return bound+f"{row['bytes']/1e6:,.1f}"+suffix

def main():
 data=json.loads((ROOT/'measured-results.json').read_text());rows=data['rows'];groups=list(dict.fromkeys(x['group'] for x in rows));lookup={(r['group'],r['method']):r for r in rows}
 assert len(lookup)==len(rows)
 counts=collections.Counter(r['status'] for r in rows)
 stamp=datetime.datetime.fromtimestamp(data['collected_unix'],datetime.timezone.utc).strftime('%d %B %Y, %H:%M UTC')
 successful=sum(completed(r) for r in rows);terminal=sum(r['status'] in ['complete','excluded','failed','stopped_divergence','timeout'] for r in rows)
 progress=f"Snapshot {stamp}: {terminal}/{len(rows)} entries finished or excluded; {successful} completed runs have complete byte accounting."
 repairs=[r for r in rows if r.get('attempt_selection')]
 repair_note=('Fresh harness-repair attempts are selected for '+', '.join(group_name(r['group']) for r in repairs)+
  '. Library 2 lacked a prior exported report trajectory; it now reads the retained native odometry. Playground 1 was interrupted by a concurrent progress-file write; progress publication is now atomic. The original attempts and their traffic remain in the measured JSON and on workstation 148. Algorithm settings and thresholds are unchanged.') if repairs else ''
 # A table-shaped figure keeps failures and different connectivity visible.
 fig,ax=plt.subplots(figsize=(7.05,5.5));fig.subplots_adjust(left=.23,right=.995,top=.84,bottom=.09)
 ax.set_xlim(-.5,4.5);ax.set_ylim(len(groups)-.5,-.5)
 for i,group in enumerate(groups):
  for j,method in enumerate(METHODS):
   row=lookup[group,method];valid=completed(row)
   face='#edf6f4' if method=='ours' else '#f4f5f6'
   if valid:
    strength=np.clip((math.log10(max(row['bytes']/1e6,1))-1)/3,0,1)
    face=plt.colormaps['Blues'](.08+.32*strength)
   patch=Rectangle((j-.47,i-.45),.94,.9,facecolor=face,edgecolor='white',lw=.6)
   if row['status'] in ['stopped_divergence','failed','timeout']:patch.set_hatch('///');patch.set_edgecolor('#c6b5ac')
   ax.add_patch(patch)
   text=cell(row)
   if valid and method!='gac':text+='\n'+components(row)
   elif valid:text+='\ncentralized'
   ax.text(j,i,text,ha='center',va='center',fontsize=7.2,fontweight='bold' if method=='ours' else 'normal',color='#173541' if valid else '#69777c')
 ax.set_xticks(range(5),['Ours\n10 s inputs','Swarm-\nSLAM','DCL-\nSLAM','DiSCo-\nSLAM','GAC\nproxy'])
 ax.tick_params(top=True,labeltop=True,bottom=False,labelbottom=False,length=0,pad=7)
 ax.set_yticks(range(len(groups)),[group_name(g) for g in groups]);ax.tick_params(axis='y',labelsize=7.8)
 for spine in ax.spines.values():spine.set_visible(False)
 ax.axhline(2.5,color='#bac7ca',lw=.8);ax.axvline(3.5,color='#bac7ca',lw=.8)
 fig.text(.23,.975,'Measured communication · MB per assigned trio',fontsize=11,fontweight='bold',color='#173541')
 fig.text(.23,.937,'Second line: sizes of verified output components',fontsize=8,color='#56646b')
 fig.text(.02,.026,'F: partial failed/stopped run     I: required camera absent     ≥: incomplete accounting\nGAC: centralized frontend-to-backend proxy. Pending/running entries are not zeros.',fontsize=7.3,color='#56646b')
 for suffix in ['pdf','png']:fig.savefig(ROOT/('communication-overview.'+suffix),dpi=300)
 plt.close(fig)
 # Show capture and publication separately; avoid presenting input cadence as latency.
 ours=[r for r in rows if r['method']=='ours' and r.get('cadence',{}).get('publications')]
 fig,axes=plt.subplots(1,2,figsize=(7.05,2.7),layout='constrained')
 palette=['#0072B2','#D55E00','#009E73','#CC79A7','#E69F00','#56B4E9']
 for i,row in enumerate(ours):
  p=row['cadence']['publications'];color=palette[i%len(palette)]
  x=np.array([v['revision'] for v in p]);delay=np.array([v['delay_s'] for v in p])
  style=['-','--',':'][i//len(palette)]
  axes[0].plot(x,delay,lw=1.1,color=color,ls=style,label=group_name(row['group']))
  intervals=row['cadence']['capture_intervals_s']
  if intervals:axes[1].plot(np.arange(2,len(intervals)+2),intervals,lw=.9,color=color,ls=style)
 axes[0].set(xlabel='Published CBS revision',ylabel='Input-to-publication delay (s)')
 axes[1].axhline(10,color='#3a454c',lw=.8,ls='--');axes[1].set(xlabel='Captured CBS revision',ylabel='Input capture interval (s)')
 for ax in axes:ax.grid(alpha=.16)
 if ours:axes[0].legend(fontsize=5.7,ncol=2)
 else:axes[0].text(.5,.5,'No CBS publication yet',ha='center',transform=axes[0].transAxes)
 for suffix in ['pdf','png']:fig.savefig(ROOT/('cbs-cadence.'+suffix),dpi=300)
 plt.close(fig)
 with (ROOT/'communication.csv').open('w') as f:
  writer=csv.writer(f);writer.writerow(['group','method','status','bytes','complete_accounting','components','captured_epochs','published_epochs'])
  for row in rows:writer.writerow([row['group'],row['method'],row['status'],row['bytes'],row.get('valid_total'),components(row),row.get('captured_epochs'),row.get('complete_epochs')])
 text=['# Measured communication benchmark','',progress,'',
  'The frozen CPU point-cloud pipeline uses ten-second causal CBS input capture. Full native solves can lag; every queued revision is charged, including repeated PCM and registration exchanges. These runs are distinct from the retained single-final-solve byte audit.','',
  'Values below are decimal MB of serialized application payload per remote robot. They exclude transport/discovery/retransmissions and local sensor traffic. GAC is a centralized input proxy, shown separately. A partial failed run is not a full-sequence saving.','',
  '| Group | '+' | '.join(NAMES.values())+' |','|---|'+'---:|'*5]
 for group in groups:text.append('| '+group_name(group)+' | '+' | '.join(cell(lookup[group,m]) for m in METHODS)+' |')
 text+=['','F: failed, timed out or stopped on odometry divergence; I: native camera input unavailable; ≥: incomplete accounting. Successful cells are paired with components in the figure and JSON.','',
  '![Measured communication](communication-overview.png)','![CBS input cadence and publication delay](cbs-cadence.png)','',
  'Native publisher counters are primary for ROS1. The first DiSCo ground run recorded 802.58 MB at native publishers versus 741.08 MB at the passive observer; those missing observation bursts are not treated as savings. The first DCL ground cross-check differed by 0.005%. Swarm uses source-attributed RCL publication sizes because this Humble version does not expose publisher IDs to Python subscriber callbacks.','',
  'Our frontend evidence is frozen and replayed at 1x; baseline frontends run natively. This is a communication benchmark, not a matched computational-runtime benchmark. No quality-normalized efficiency conclusion is made without considering output connectivity, trajectory coverage and failures.','',
  repair_note,'',
  '[Detailed protocol](../../../scripts/communication_benchmark/README.md) · [Measured data](measured-results.json) · [CSV](communication.csv)']
 (ROOT/'REPORT.md').write_text('\n'.join(text)+'\n')
 table='<table><thead><tr><th>Group</th>'+''.join('<th>'+html.escape(NAMES[m])+'</th>' for m in METHODS)+'</tr></thead><tbody>'
 for group in groups:table+='<tr><td>'+group_name(group)+'</td>'+''.join('<td>'+html.escape(cell(lookup[group,m]))+'</td>' for m in METHODS)+'</tr>'
 table+='</tbody></table>'
 (ROOT/'index.html').write_text('<!doctype html><meta charset="utf-8"><title>Measured communication benchmark</title><style>body{max-width:1080px;margin:40px auto;font:16px/1.55 system-ui;color:#173541;padding:0 20px}h1{font-size:30px}table{border-collapse:collapse;width:100%;font-size:14px}th,td{padding:8px;text-align:right;border-bottom:1px solid #ddd}td:first-child,th:first-child{text-align:left}img{max-width:100%}.note{padding:15px;background:#eef5f5}a{color:#087f8c}</style><h1>Measured communication benchmark</h1><p>'+html.escape(progress)+'</p><p class="note">Ten-second CBS graph inputs are queued while prior solves run. The achieved publication delay is shown below. Baselines retain their native methods. GAC is a centralized input proxy; failed partial runs are not full-sequence savings.</p>'+table+'<p>F: failed/stopped partial run; I: camera input absent; ≥: incomplete accounting. MB are decimal. Pending entries are not zeros.</p><img src="communication-overview.png"><img src="cbs-cadence.png"><p><a href="REPORT.md">Detailed report</a> · <a href="measured-results.json">Measured results</a> · <a href="communication.csv">CSV</a> · <a href="../s3e-graco-comparison-20260924/output/pdf/technical-report.pdf">Technical report PDF</a></p>')
 if repair_note:
  page=ROOT/'index.html';page.write_text(page.read_text()+'<p>'+html.escape(repair_note)+'</p>')
 for name in ['communication-overview.pdf','cbs-cadence.pdf']:shutil.copy2(ROOT/name,REPORT/'output/pdf'/name)
 tex=r'''\clearpage
\section{Measured communication and CBS update frequency}
\label{sec:communication}
This campaign measures the frozen CPU point-cloud pipeline and four native baselines on three matched GRACO trios and the twelve selected S3E groups. It is separate from the historical ellipsoid accuracy experiments above. '''+progress.replace('%',r'\%')+r'''

\paragraph{Protocol and scope.}
Each baseline uses its own calibrated native frontend, retrieval, registration and optimizer. Standard runs replay at $1\times$ with a 120-second finishing window. GAC uses its released sequential-session interface and existing one-hour finishing allowance. Our method replays immutable native odometry, point-cloud descriptors and accumulated-area evidence at $1\times$, then executes live distributed retrieval, PCM and CBS with GICP factors. Thus the experiment measures backend communication, not a fresh matched frontend-runtime trial. All source/configuration identities and failures are retained on workstation 148, without resource quotas.

We count serialized application payload per remote robot, excluding local sensor/visualization copies, transport headers, discovery and retransmissions. These values are not NIC bytes or confirmed wireless delivery. ROS1 measurements use native TCPROS publisher counters with four-byte message framing removed; duplicate receiving processes on one robot are merged and the unmerged count is also retained. Passive observations provide an independent cross-check. Swarm-SLAM uses a transparent RCL publication probe to attribute shared topics exactly; its native subscriber topology determines recipient multiplicity. Our counters cover retrieval, verification, registration exchange, PCM and CBS service requests/responses. The released GAC backend is centralized, so its frontend-to-backend cloud/odometry payload is reported only as a separately labelled proxy.

\paragraph{Ten-second CBS inputs.}
The previous online scheduler completed a sealed distributed solve and then waited another 30 seconds. Its five graph revisions each contained many CBS iterations. The revised scheduler captures changed causal graph inputs every 10 seconds on the common descriptor-availability clock, independently of solver completion. It queues every captured revision and flushes the final tail, preserving the existing 100-iteration settling budget, PCM and live registration factors. Geometry is hard-linked locally; each fresh native session's actual peer exchanges are charged again. Queue wait and publication latency are measured explicitly: a ten-second input cadence does not establish ten-second output latency.

\begin{figure}[tb]
\centering\includegraphics[width=\linewidth]{../output/pdf/communication-overview.pdf}
\caption{Measured serialized communication in decimal MB per assigned three-robot team. Successful cells show component sizes below the byte total. F denotes traffic from a partial failed/stopped run; I denotes missing required camera input. Incomplete accounting is a lower bound. Running and pending entries are not zeros. GAC is a centralized input proxy rather than native peer traffic. Read byte totals jointly with connectivity and completion; smaller failed-run totals do not demonstrate communication efficiency.}
\label{fig:communication}
\end{figure}

\begin{figure}[tb]
\centering\includegraphics[width=\linewidth]{../output/pdf/cbs-cadence.pdf}
\caption{Actual CBS input-to-publication delay (left) and intervals between captured causal graph revisions (right). The dashed line marks the ten-second input target; final-tail flushes can be shorter, while unchanged or absent verified graph inputs need not create a revision. Slow native solves accumulate queueing delay rather than silently discarding graph updates.}
\label{fig:cbs-cadence}
\end{figure}

\paragraph{Accounting checks and limitations.}
The first completed GRACO ground DiSCo run recorded 802.58\,MB in native publisher counters versus 741.08\,MB in the passive observer. The missed observation bursts illustrate why subscriber-only counting is insufficient. DCL's corresponding independent cross-check differed by approximately 0.005\%. Monitoring serializes messages or polls counters and is not assumed to have zero runtime overhead. Any robot exceeding the 20\,m/s odometry guard stops its whole group; the partial traffic is preserved. Failed or in-flight native CBS revisions without complete counters remain explicitly lower bounds. No equal-quality radio-efficiency claim is made from different output components, incomplete trajectories, or early failures.
\FloatBarrier
'''
 if repair_note:tex=tex.replace(r'\FloatBarrier',repair_note+'\n'+r'\FloatBarrier')
 (REPORT/'latex/sections/communication.tex').write_text(tex)
 main=REPORT/'latex/main.tex';content=main.read_text()
 if r'\input{sections/communication}' not in content:content=content.replace(r'\input{sections/discussion}',r'\input{sections/communication}'+'\n'+r'\input{sections/discussion}');main.write_text(content)
 (ROOT/'paper-self-review.md').write_text('# Communication section review\n\nOutline: measurement scope → CBS scheduling → measured outcomes → limitations.\n\n- Contribution: the section supplies measured engineering evidence, not a new algorithm claim.\n- Clarity: payload bytes, per-robot copies, native publisher counters and GAC proxy are distinguished.\n- Experimental strength: failures and component sizes remain visible; no superiority claim from smaller failed runs.\n- Completeness: pending runs and incomplete native counters remain labelled. Physical radio tests and repeated trials are outside the evidence.\n- Design soundness: fresh sealed sessions resend geometry; ten-second capture is separated from achieved latency. This limitation is stated rather than concealed.\n')
 print(progress)

if __name__=='__main__':main()
