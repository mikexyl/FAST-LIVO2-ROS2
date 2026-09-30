#!/usr/bin/env python3
"""Inspect frozen accumulated-area membership and saved BEVs; never recrop future geometry."""
import argparse
from collections import Counter
import csv
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import PowerNorm
from matplotlib.patches import Circle
from matplotlib.transforms import Affine2D
import numpy as np
from PIL import Image
import yaml


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def intersection(a, b):
    i = j = total = 0
    while i < len(a) and j < len(b):
        total += max(0, min(a[i][1], b[j][1]) - max(a[i][0], b[j][0]))
        if a[i][1] < b[j][1]:
            i += 1
        else:
            j += 1
    return total


def union_size(ranges):
    total = 0
    lo = hi = None
    for a, b in sorted(ranges):
        if hi is None:
            lo, hi = a, b
        elif a > hi:
            total += hi - lo
            lo, hi = a, b
        else:
            hi = max(hi, b)
    return total + (hi - lo if hi is not None else 0)


def horizontal_basis(up):
    up = np.asarray(up, dtype=float)
    up /= np.linalg.norm(up)
    x = np.array([1., 0., 0.])
    x -= up * np.dot(up, x)
    x /= np.linalg.norm(x)
    return np.stack([x, np.cross(up, x), up])


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--work', type=Path, required=True)
    ap.add_argument('--robot', default='aerial05')
    ap.add_argument('--output', type=Path, required=True)
    args = ap.parse_args()
    work, out, robot = args.work.resolve(), args.output.resolve(), args.robot
    out.mkdir(parents=True, exist_ok=True)
    (out / 'submaps').mkdir(exist_ok=True)
    keyfile = work / f'prepared-{robot}/store/keyframes.jsonl'
    rows = [json.loads(s) for s in keyfile.read_text().splitlines()]
    manifest_file = work / 'bev-gallery/manifest.json'
    manifest = json.loads(manifest_file.read_text())
    images = {x['key']: x for x in manifest['maps'] if x['robot'] == robot}
    assert set(images) == {r['keyframe_id'] for r in rows}
    assert all(r['strategy'] == 'area' for r in rows)
    for r in rows:
        ranges = r['geometry_id_ranges']
        assert all(a < b for a, b in ranges)
        assert all(a[1] <= b[0] for a, b in zip(ranges, ranges[1:]))
        assert sum(b - a for a, b in ranges) == r['geometry_count']
        m = images[r['keyframe_id']]
        assert m['payload_sha256'] == r['sha256']
        assert digest(work / 'bev-gallery' / m['image']) == m['png_sha256']
        im = Image.open(work / 'bev-gallery' / m['image'])
        assert im.size == (m['width'], m['height'])
    counts = np.array([r['geometry_count'] for r in rows])
    common = np.array([[intersection(a['geometry_id_ranges'], b['geometry_id_ranges'])
                        for b in rows] for a in rows], dtype=np.int64)
    assert np.array_equal(common, common.T)
    assert np.array_equal(np.diag(common), counts)
    fractions = common / counts[:, None]
    up = np.median([r['area_up_world'] for r in rows], axis=0)
    H = horizontal_basis(up)
    centers = np.array([r['area_center_world'] for r in rows]) @ H.T
    trajectory_file = work / f'report/{robot}-raw.tum'
    trajectory = np.loadtxt(trajectory_file, ndmin=2)[:, 1:4] @ H.T
    mapfile = work / f'report/{robot}-maps.npz'
    with np.load(mapfile) as data:
        # rollout_report.py saves 'raw' directly in the odometry world frame.
        points = data['raw'] @ H.T
    assert np.isfinite(points).all() and np.isfinite(trajectory).all()
    t0 = rows[0]['begin_ns']
    stats = []
    for i, r in enumerate(rows):
        radius = r['area_radius_m']
        distance = area = reused = retained = iou = None
        if i:
            delta = np.array(r['area_center_world']) - rows[i-1]['area_center_world']
            normal = np.asarray(r['area_up_world'])
            distance = float(np.linalg.norm(delta - normal * np.dot(normal, delta)))
            x = min(distance / (2 * radius), 1.)
            area = float((2 * np.arccos(x) - 2 * x * np.sqrt(1 - x*x)) / np.pi)
            reused = float(fractions[i, i-1])
            retained = float(fractions[i-1, i])
            iou = float(common[i, i-1] / (counts[i] + counts[i-1] - common[i, i-1]))
        stats.append(dict(key=r['keyframe_id'], time_s=(r['stamp_ns']-t0)/1e9,
                          trigger=r['finish_reason'], points=int(counts[i]),
                          center_step_m=distance, disk_overlap_fraction=area,
                          shared_previous_points=int(common[i, i-1]) if i else None,
                          fraction_current_from_previous=reused,
                          fraction_previous_retained=retained, point_jaccard=iou,
                          figure=f'submaps/{i:03d}.png'))
    unique = union_size([p for r in rows for p in r['geometry_id_ranges']])
    median = lambda field: float(np.median([s[field] for s in stats[1:]]))
    summary = dict(robot=robot, snapshots=len(rows), radius_m=rows[0]['area_radius_m'],
                   median_reused_fraction=median('fraction_current_from_previous'),
                   median_previous_retained=median('fraction_previous_retained'),
                   median_center_step_m=median('center_step_m'),
                   median_disk_overlap=median('disk_overlap_fraction'),
                   total_point_occurrences=int(counts.sum()), unique_point_ids=unique,
                   mean_snapshot_memberships_per_unique_point=float(counts.sum()/unique),
                   triggers=dict(Counter(r['finish_reason'] for r in rows)),
                   source_run=str(work), source_variant='historical sampled-ellipsoid BEVs; accumulated-area membership',
                   ground_truth_used=False, point_membership_exact=True,
                   visual_geometry='Retained raw-map overview; saved causal BEV per snapshot. No final-map recropping.',
                   display_frame='One median-gravity horizontal frame for overview; BEVs retain per-anchor gravity and share horizontal heading.',
                   source_hashes={str(p): digest(p) for p in [keyfile, manifest_file, trajectory_file, mapfile, Path(__file__)]},
                   snapshots_detail=stats)
    (out/'summary.json').write_text(json.dumps(summary, indent=2)+'\n')
    with (out/'overlap.csv').open('w') as f:
        writer = csv.DictWriter(f, fieldnames=list(stats[0])); writer.writeheader(); writer.writerows(stats)
    plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 10,
                         'axes.spines.top': False, 'axes.spines.right': False})
    # Histogram is only a display raster of the retained map, not a new backend BEV.
    lo = np.minimum(points[:, :2].min(axis=0), centers[:, :2].min(axis=0)-82)
    hi = np.maximum(points[:, :2].max(axis=0), centers[:, :2].max(axis=0)+82)
    bins = [np.linspace(lo[k], hi[k], int(np.ceil((hi[k]-lo[k])/.4))+1) for k in range(2)]
    hist, xb, yb = np.histogram2d(points[:, 0], points[:, 1], bins=bins)
    density = np.log1p(hist.T)
    extent = [xb[0], xb[-1], yb[0], yb[-1]]
    colors = plt.colormaps['viridis'](np.linspace(.12, .92, len(rows)))
    def base_map(ax):
        ax.imshow(density, origin='lower', extent=extent, cmap='Greys',
                  vmax=np.quantile(density[density > 0], .99), interpolation='nearest')
        ax.plot(trajectory[:, 0], trajectory[:, 1], color='#087e8b', lw=1.25)
        ax.set(xlabel='Horizontal x (m)', ylabel='Horizontal y (m)', aspect='equal', xlim=(lo[0], hi[0]), ylim=(lo[1], hi[1]))
    def footprint(ax, i, **kwargs):
        u = horizontal_basis(rows[i]['area_up_world'])
        angle = np.linspace(0, 2*np.pi, 240)
        p = (np.cos(angle)[:, None]*u[0] + np.sin(angle)[:, None]*u[1]) * rows[i]['area_radius_m']
        p = (p + rows[i]['area_center_world']) @ H.T
        ax.plot(p[:, 0], p[:, 1], **kwargs)
    resolution = yaml.safe_load((work/'config.yaml').read_text())['backend']['mapclosures']['density_map_resolution']
    def bev(ax, i, small=False):
        m = images[rows[i]['keyframe_id']]
        image = np.array(Image.open(work/'bev-gallery'/m['image']))
        G = np.asarray(m['ground'])[:3, :3]
        T = np.asarray(rows[i]['T_world_body'])
        level_to_display = H @ T[:3, :3] @ G.T
        # Use only heading: a 2D image has no heights with which to change its gravity plane.
        U, _, V = np.linalg.svd(level_to_display[:2, :2])
        A = np.eye(3); A[:2, :2] = U @ V
        assert np.linalg.det(A[:2, :2]) > 0
        lower = np.asarray(m['lower_bound']) * resolution
        upper = lower + np.array(image.shape) * resolution
        ax.imshow(image.T, origin='lower', extent=[lower[0], upper[0], lower[1], upper[1]],
                  cmap='gray', norm=PowerNorm(.6, vmin=0, vmax=255), interpolation='nearest',
                  transform=Affine2D(A)+ax.transData)
        ax.add_patch(Circle((0, 0), rows[i]['area_radius_m'], fill=False, ec='#53bfcb', lw=.9, alpha=.8))
        ax.scatter([0], [0], c='#ffc857', marker='+', s=45, linewidths=1.6)
        ax.set(aspect='equal', xlim=(-84, 84), ylim=(-84, 84), facecolor='#080d14')
        ax.plot([-70, -50], [-70, -70], color='white', lw=2)
        ax.text(-60, -65, '20 m', color='white', ha='center', fontsize=8)
        if small:
            ax.set_xticks([]); ax.set_yticks([])
        else:
            ax.set(xlabel='Horizontal x from snapshot center (m)', ylabel='Horizontal y from snapshot center (m)')
    fig, axes = plt.subplots(1, 3, figsize=(17, 5.8), layout='constrained')
    base_map(axes[0])
    axes[0].scatter(centers[:,0], centers[:,1], c=colors, s=24, zorder=3)
    for i in range(0, len(rows), 4):
        axes[0].annotate(str(i), centers[i,:2], xytext=(4,5), textcoords='offset points', fontsize=9)
    axes[0].set_title('A05 accumulated map + trajectory\nSnapshot centers numbered in time order')
    base_map(axes[1])
    for i in range(13, 18):
        footprint(axes[1], i, color=colors[i], lw=1.5)
        axes[1].scatter(*centers[i,:2], color=colors[i], s=25)
        axes[1].annotate(str(i), centers[i,:2], xytext=(4,4), textcoords='offset points')
    axes[1].set_title('Five consecutive 80 m-radius footprints\nSnapshots 13–17')
    im = axes[2].imshow(fractions*100, vmin=0, vmax=100, cmap='magma', origin='upper')
    axes[2].set(xlabel='Other snapshot', ylabel='Current snapshot', title='Exact shared point membership\nShared points ÷ current snapshot points')
    fig.colorbar(im, ax=axes[2], shrink=.65, label='% of current snapshot')
    fig.suptitle(f'GRACO A05 · {len(rows)} accumulated-area snapshots · median adjacent point reuse {100*summary["median_reused_fraction"]:.1f}%', fontsize=16)
    fig.savefig(out/'overview.png', dpi=165); plt.close(fig)
    for i, s in enumerate(stats):
        fig, axes = plt.subplots(1, 2, figsize=(12, 5.7), layout='constrained')
        base_map(axes[0])
        if i:
            footprint(axes[0], i-1, color='#d57b20', lw=1.5, ls='--', label=f'Previous: {i-1}')
        footprint(axes[0], i, color='#1769aa', lw=2, label=f'Selected: {i}')
        axes[0].scatter(*centers[i,:2], color='#1769aa', marker='o', s=40, zorder=4)
        axes[0].legend(loc='upper right', fontsize=9)
        axes[0].set_title('Full retained map and snapshot footprints')
        bev(axes[1], i)
        axes[1].set_title('Actual saved submap BEV\nSampled ellipsoids · display contrast enhanced')
        reused = f'{100*s["fraction_current_from_previous"]:.1f}% shared with previous' if i else 'First snapshot'
        fig.suptitle(f'A05 / {i:02d} · {s["time_s"]:.1f} s · {s["points"]:,} native points · {reused}', fontsize=14)
        fig.savefig(out/s['figure'], dpi=140); plt.close(fig)
    fig, axes = plt.subplots(9, 4, figsize=(14, 29), layout='constrained')
    for i, ax in enumerate(axes.flat):
        if i >= len(rows): ax.axis('off'); continue
        bev(ax, i, small=True)
        s = stats[i]
        reuse = f'{100*s["fraction_current_from_previous"]:.0f}% reused' if i else 'first snapshot'
        ax.set_title(f'{i:02d} · {s["time_s"]:.0f} s · {reuse}\n{s["points"]/1000:.1f}k native points', fontsize=11)
    fig.suptitle('A05 · every saved submap at the same scale\nHistorical sampled-ellipsoid BEVs · circle radius 80 m · + snapshot center', fontsize=17)
    fig.savefig(out/'all-submaps.png', dpi=130); plt.close(fig)
    report = f'''# GRACO A05 accumulated-area overlap inspection

Source: `{work}`. This historical capture has {len(rows)} snapshots using the current 80 m-radius / 20 m-or-10 s accumulated-area policy. It is not a new pipeline evaluation.

- Median fraction of current points already in the immediately previous snapshot: **{summary['median_reused_fraction']:.1%}**.
- Median fraction of previous points retained: **{summary['median_previous_retained']:.1%}**.
- Median horizontal center displacement: **{summary['median_center_step_m']:.2f} m**.
- Median ideal equal-disk overlap: **{summary['median_disk_overlap']:.1%}**. This geometric estimate treats the two gravity planes as parallel; point-ID overlap is exact.
- {counts.sum():,} point occurrences across snapshots, {unique:,} distinct stable point IDs: **{counts.sum()/unique:.2f} snapshot memberships per exported point** on average.
- Triggers: {summary['triggers']}. Adjacent statistics include startup and shutdown snapshots.

The global overview uses the saved raw odometry-frame map (0.25 m display downsampling). Each submap view is its original causal, hash-verified sampled-ellipsoid BEV; it is not a crop of the final map. These are historical ellipsoid images, not newly rendered point-cloud BEVs. All views share metric scale and horizontal heading; per-anchor gravity is preserved, while the map overview uses median gravity. Brightness is enhanced for inspection only. The circle indicates spatial selection, not observed coverage. Blank space can be unobserved or suppressed by the original density threshold.

Exact overlap uses immutable point-ID ranges before backend downsampling. Point-count reuse is not a measured runtime, memory, or communication multiplier. Different snapshot endpoints can repeat geometry work even when each endpoint is cached. No experiment settings were changed. Workstation 148 was unreachable, so full 3D per-snapshot payloads were not rendered; all saved 2D views are included.

[Interactive gallery](index.html) · [Overview](overview.png) · [All submaps](all-submaps.png) · [Exact overlap CSV](overlap.csv) · [Provenance](summary.json)
'''
    (out/'REPORT.md').write_text(report)
    template = Path(__file__).with_suffix('.html').read_text()
    payload = json.dumps(dict(stats=stats, median=summary['median_reused_fraction'], multiplicity=counts.sum()/unique))
    (out/'index.html').write_text(template.replace('__DATA__', payload))
    print(json.dumps({k:v for k,v in summary.items() if k not in ('source_hashes','snapshots_detail')}, indent=2))


if __name__ == '__main__':
    main()
