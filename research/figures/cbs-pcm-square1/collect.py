"""Compact PCM/CBS validation report from a frozen-constraint run; no replay.

Run with the research venv, passing the completed registry. Reads only frame
indices for dense correction/evo; no clouds, images, descriptors or GT enter PCM.
"""
import argparse
import csv
from pathlib import Path
import shutil
import sys
from collections import Counter

import numpy as np

RESEARCH = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(RESEARCH))
from s3e_pipeline.artifacts import read_json, read_jsonl, write_json, file_hash
from s3e_pipeline.evaluation import corrected_dense_rows, ground_truth, save_tum
from s3e_pipeline.evo_evaluation import evaluate


def collect(registry):
    output = Path(__file__).resolve().parent
    registry = Path(registry).resolve()
    run = read_json(registry); cfg = run['config']; artifacts = run['artifacts']
    dpgo = Path(artifacts['dpgo']); poses = read_jsonl(dpgo/'poses.jsonl')
    pcm = read_json(dpgo/'pcm.json'); summary = read_json(dpgo/'summary.json')
    assert cfg['dpgo']['mode'] == 'frozen' and cfg['dpgo']['pcm']['enabled']
    assert Path(cfg['dataset']).name == 'S3E_Square_1'
    loops = read_jsonl(dpgo/'constraints.jsonl')
    proposals = read_jsonl(dpgo/'proposed-constraints.jsonl')
    original = read_jsonl(Path(artifacts['loops.livo.megaloc_mapclosures'])/'constraints.jsonl')
    edge_key = lambda e: (tuple(e['i']), tuple(e['j']))
    assert sorted(original, key=edge_key) == sorted(proposals, key=edge_key)
    accepted = {(cfg['robots'][v['robot_from']], v['key_from'], cfg['robots'][v['robot_to']], v['key_to'])
                for v in pcm['verdicts'] if v['retained']}
    assert accepted == {(e['i'][0], e['i'][1], e['j'][0], e['j'][1]) for e in loops}
    inputs = {}
    def record(path):
        path = Path(path).resolve(); inputs[str(path)] = file_hash(path); return path
    for name in ('COMPLETE.json', 'summary.json', 'pcm.json', 'constraints.jsonl',
                 'proposed-constraints.jsonl', 'poses.jsonl'):
        shutil.copy2(record(dpgo/name), output/name)
    # This is a retained copy of the source stage manifest, not a claim that
    # this compact report folder itself is a pipeline stage/cache.
    (output/'COMPLETE.json').rename(output/'dpgo-stage-manifest.json')
    shutil.copy2(record(registry), output/'run.json')
    tracks = []; truth = {}; robot_pcm = {}; checks = {}; cdr = 0
    for robot in cfg['robots']:
        robot_output = output/robot; robot_output.mkdir(exist_ok=True)
        local = dpgo/robot
        p = read_json(record(local/'pcm.json')); robot_pcm[robot] = p
        shutil.copy2(local/'pcm.json', robot_output/'pcm.json')
        shutil.copy2(record(local/'stats.jsonl'), robot_output/'stats.jsonl')
        native = local/'native'/robot/'cbs_online'
        for name in ('pcm_checks.csv', 'pcm_decisions.csv', 'pcm_wire.csv'):
            shutil.copy2(record(native/name), robot_output/name)
        rows = list(csv.DictReader((native/'pcm_checks.csv').open()))
        assert all(row['available'] == '1' for row in rows)
        checks[robot] = dict(tests=len(rows), inconsistent=sum(r['consistent'] == '0' for r in rows),
            max_squared_mahalanobis=max((float(r['squared_mahalanobis']) for r in rows), default=0.))
        wire = list(csv.DictReader((native/'pcm_wire.csv').open()))
        assert sum(int(r['serialized_bytes']) for r in wire) == p['network_cdr_bytes']
        cdr += p['network_cdr_bytes']
        key_stage = Path(artifacts[f'keyframes.livo.{robot}'])
        odom_stage = Path(artifacts[f'odometry.livo.{robot}'])
        record(key_stage/'COMPLETE.json'); record(odom_stage/'COMPLETE.json')
        for descriptor in (f'descriptors.livo.megaloc.{robot}', f'descriptors.livo.megaloc_mapclosures.{robot}'):
            if descriptor in artifacts: record(Path(artifacts[descriptor])/'COMPLETE.json')
        keys = read_jsonl(record(key_stage/'store/keyframes.jsonl'))
        frames = read_jsonl(record(odom_stage/'run/export/frames.jsonl'))
        corrected = [p for p in poses if p['robot_id'] == robot]
        assert [r['keyframe_id'] for r in keys] == [r['keyframe_id'] for r in corrected]
        dense = list(corrected_dense_rows(frames, keys, corrected))
        component = {p['component'] for p in corrected}; assert len(component) == 1
        for row in dense: row['component'] = next(iter(component))
        tracks.extend(dense); save_tum(output/f'{robot}-corrected.tum', dense)
        truth[robot] = ground_truth(record(Path(cfg['dataset'])/f'{robot.lower()}_gt.txt'))
    assert cdr == summary['pcm_network_cdr_bytes']
    metrics = evaluate(dict(poses=tracks), truth, cfg['evaluation'], output/'evo')
    counts = Counter((e['i'][0], e['j'][0]) for e in proposals)
    pair_counts = [{'robots': list(pair), 'proposed': number,
                   'retained': sum((e['i'][0], e['j'][0]) == pair for e in loops)}
                  for pair, number in sorted(counts.items())]
    report = dict(dataset=cfg['dataset'], configuration=cfg, pcm=pcm, runtime=summary,
        per_robot_pcm=robot_pcm, pair_counts=pair_counts, cycle_checks=checks, trajectory=metrics,
        input_measurements_unchanged=True,
        input_indices_and_manifests_unchanged=all(file_hash(p) == h for p, h in inputs.items()),
        sources=inputs,
        scope='Frozen 50-loop Square 1 set; PCM and CBS rerun, odometry/retrieval/registration not rerun',
        comparison_limitation='Earlier CBS result used incremental input; this run gates one frozen batch before optimization. No ATE change is attributed solely to PCM when its retained set is unchanged.')
    assert report['input_indices_and_manifests_unchanged']
    write_json(output/'report.json', report)

    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from evo.tools.file_interface import read_tum_trajectory_file
    fig, axes = plt.subplots(1, 2, figsize=(12, 5))
    colors = {'Alpha': '#df5550', 'Bob': '#439bea', 'Carol': '#49a869'}
    origin = truth['Alpha'][1][0]
    for robot in cfg['robots']:
        est = read_tum_trajectory_file(output/f'evo/{robot}-estimate-aligned.tum')
        xyz = est.positions_xyz - origin
        axes[0].plot(xyz[:, 0], xyz[:, 1], color=colors[robot], label=robot)
        gt = truth[robot][1] - origin
        # Do not visually join across large GNSS gaps.
        gt = np.insert(gt, np.flatnonzero(np.diff(truth[robot][0]) > 2e9) + 1, np.nan, axis=0)
        axes[0].plot(gt[:, 0], gt[:, 1], ':', color=colors[robot], alpha=.5)
    axes[0].set_aspect('equal'); axes[0].legend(); axes[0].grid(alpha=.2)
    axes[0].set(xlabel='East from local origin [m]', ylabel='North from local origin [m]',
                title='Square 1: distributed PCM + CBS\nDotted: supplied position GT')
    values = [float(row['squared_mahalanobis']) for robot in cfg['robots']
              for row in csv.DictReader((output/robot/'pcm_checks.csv').open())
              if cfg['robots'].index(robot) < int(row['peer']) and row['available'] == '1']
    threshold = float(next(csv.DictReader((output/'Alpha/pcm_checks.csv').open()))['threshold'])
    assert all(v > 0 for v in values)
    axes[1].hist(values, bins=np.geomspace(min(values)*.8, max(threshold, max(values))*1.2, 36), color='#64768b')
    axes[1].set_xscale('log')
    axes[1].axvline(threshold, color='#c2473e', linestyle='--', label=f'99% chi-square: {threshold:.2f}')
    axes[1].set(xlabel='Squared Mahalanobis cycle statistic (log scale)', ylabel='Unique loop-pair checks',
                title=f'{len(loops)} / {len(proposals)} loops retained; {len(values)} checks')
    axes[1].legend(); axes[1].grid(axis='y', alpha=.2)
    fig.tight_layout(); fig.savefig(output/'pcm_square1.png', dpi=170); fig.savefig(output/'pcm_square1.pdf')
    plt.close(fig)
    write_json(output/'files.json', {str(p.relative_to(output)): file_hash(p)
        for p in sorted(output.rglob('*')) if p.is_file() and p.name != 'files.json' and '__pycache__' not in p.parts})
    print({component: dict(rmse_m=m['rmse_m'], per_robot={r: v['statistics']['rmse']
        for r,v in m['per_robot'].items()}) for component,m in metrics.items()})
    print({'pair_counts': pair_counts, 'pcm_cdr_bytes': cdr, 'checks': checks})


if __name__ == '__main__':
    parser = argparse.ArgumentParser(); parser.add_argument('registry', type=Path)
    collect(parser.parse_args().registry)
