"""Render report figures from a completed CBS evaluation, without rerunning SLAM."""
import argparse
import os
from pathlib import Path
import tempfile

os.environ.setdefault('MPLCONFIGDIR', str(Path(tempfile.gettempdir()) / 's3e-report-matplotlib'))
import matplotlib
import mpl_toolkits

# A ROS underlay can preload the system namespace. Use mplot3d from the same
# installation as pyplot, without changing either Python environment on disk.
local_toolkits = Path(matplotlib.__file__).parent.parent / 'mpl_toolkits'
if local_toolkits.exists():
    mpl_toolkits.__path__ = [str(local_toolkits), *mpl_toolkits.__path__]
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.colors as colors
import matplotlib.patheffects as effects
from matplotlib.collections import LineCollection
from matplotlib.lines import Line2D
from matplotlib.ticker import MaxNLocator
import numpy as np

from .artifacts import read_json, read_jsonl, write_json, file_hash, validate_stage
from .evaluation import ground_truth
from .geometry import transform

ROBOTS = ['Alpha', 'Bob', 'Carol']
COLORS = {'Alpha': '#dc514b', 'Bob': '#2684bc', 'Carol': '#269b69'}
BRANCHES = {
    ('mapclosures',): ('MapClosures only', '#d17b0f'),
    ('mapclosures', 'megaloc'): ('Both branches', '#8856a7'),
    ('megaloc',): ('MegaLoc only', '#158b8d'),
}


def configure():
    plt.rcParams.update({
        'font.family': 'DejaVu Sans', 'font.size': 9, 'axes.titlesize': 10.5,
        'axes.labelsize': 9, 'legend.fontsize': 8, 'xtick.labelsize': 8,
        'ytick.labelsize': 8, 'axes.spines.top': False, 'axes.spines.right': False,
        'axes.linewidth': .7, 'grid.linewidth': .5, 'grid.alpha': .25,
        'pdf.fonttype': 42, 'ps.fonttype': 42, 'savefig.facecolor': 'white',
    })


def save(fig, output, name, dpi):
    for extension in ('png', 'pdf'):
        fig.savefig(output / f'{name}.{extension}', dpi=dpi, bbox_inches='tight', pad_inches=.06)
    plt.close(fig)
    print(f'Saved {name}.png / .pdf', flush=True)


def metric_axes(ax, limits):
    ax.set(xlim=limits[:2], ylim=limits[2:], xlabel='x (m)', ylabel='y (m)')
    ax.set_aspect('equal', adjustable='box')
    ax.grid(True, zorder=0)
    ax.xaxis.set_major_locator(MaxNLocator(nbins=5))
    ax.yaxis.set_major_locator(MaxNLocator(nbins=7))


def paths(rows, alignment, origin):
    xyz = np.asarray([np.asarray(row['T_world_body'])[:3, 3] for row in rows])
    return transform(alignment, xyz) - origin


def trajectory_figure(tracks, truth, keys, factors, output, dpi):
    bounds = np.concatenate([*tracks.values(), *truth.values()])
    bounds = bounds[np.isfinite(bounds).all(axis=1)]
    lower, upper = bounds[:, :2].min(0) - 12, bounds[:, :2].max(0) + 12
    limits = [lower[0], upper[0], lower[1], upper[1]]
    fig, axes = plt.subplots(1, 2, figsize=(7.6, 6.5))
    for ax in axes:
        metric_axes(ax, limits)
    axes[0].set_title('(a) Distributed CBS trajectories', loc='left', pad=10)
    axes[1].set_title('(b) Verified loop closures', loc='left', pad=10)
    for robot in tracks:
        xyz, gt = tracks[robot], truth[robot]
        axes[0].plot(gt[:, 0], gt[:, 1], color='#555c63', lw=1, ls=(0, (3, 2)), alpha=.75)
        axes[0].plot(xyz[:, 0], xyz[:, 1], color=COLORS[robot], lw=1.5)
        axes[0].scatter(*xyz[0, :2], s=25, marker='o', c=COLORS[robot], edgecolors='white', linewidths=.6, zorder=5)
        axes[1].plot(xyz[:, 0], xyz[:, 1], color='#bac1c7', lw=1)
    handles = [Line2D([], [], color=COLORS[r], lw=2, label=r) for r in tracks]
    if any(len(x) for x in truth.values()):
        handles.append(Line2D([], [], color='#555c63', ls='--', label='Position ground truth'))
    axes[0].legend(handles=handles, loc='upper left', bbox_to_anchor=(0, -.30), frameon=False, ncol=2)
    branch_handles = []
    for branch, (label, color) in BRANCHES.items():
        selected = [e for e in factors if e['kind'] == 'loop' and
                    tuple(sorted(e['diagnostics']['retrieval_sources'])) == branch]
        if not selected:
            continue
        segments = np.asarray([[keys[tuple(e['i'])][:2], keys[tuple(e['j'])][:2]] for e in selected])
        axes[1].add_collection(LineCollection(segments, colors=color, linewidths=1, alpha=.9, zorder=4))
        endpoints = segments.reshape(-1, 2)
        axes[1].scatter(endpoints[:, 0], endpoints[:, 1], s=5, c=color, zorder=5)
        branch_handles.append(Line2D([], [], color=color, marker='.', lw=1.2, label=f'{label} ({len(selected)})'))
    axes[1].legend(handles=branch_handles, loc='upper left', bbox_to_anchor=(0, -.30), frameon=False)
    fig.subplots_adjust(wspace=.30, bottom=.19, top=.93)
    save(fig, output, 'trajectories_and_loops', dpi)


def bev_projection(points, owners, resolution, norm):
    """Orthographic depth buffer: highest observed point in each XY display pixel."""
    lower = np.floor(points[:, :2].min(0) / resolution) * resolution
    pixels = np.floor((points[:, :2] - lower) / resolution).astype(np.int64)
    nx, ny = pixels.max(0) + 1
    ids = pixels[:, 1] * nx + pixels[:, 0]
    order = np.lexsort((points[:, 2], ids))
    ordered_ids = ids[order]
    selected = order[np.r_[ordered_ids[:-1] != ordered_ids[1:], True]]
    robot_image = np.full((ny, nx, 3), 255, dtype=np.uint8)
    height_image = robot_image.copy()
    palette = np.asarray([colors.to_rgb(COLORS[r]) for r in ROBOTS]) * 255
    row, col = pixels[selected, 1], pixels[selected, 0]
    robot_image[row, col] = palette[owners[selected]].astype(np.uint8)
    height_image[row, col] = (matplotlib.colormaps['viridis'](norm(points[selected, 2]))[:, :3] * 255).astype(np.uint8)
    extent = [float(lower[0]), float(lower[0] + nx * resolution),
              float(lower[1]), float(lower[1] + ny * resolution)]
    return robot_image, height_image, extent, len(selected)


def bev_figure(points, owners, tracks, norm, output, dpi, resolution, view_limits=None):
    robot_image, height_image, extent, count = bev_projection(points, owners, resolution, norm)
    fig = plt.figure(figsize=(9.1, 6.8))
    grid = fig.add_gridspec(1, 3, width_ratios=[1, 1, .04])
    axes = [fig.add_subplot(grid[0]), fig.add_subplot(grid[1])]
    cax = fig.add_subplot(grid[2])
    titles = ['(a) Map contributions by robot', '(b) Merged map elevation']
    for ax, image, title in zip(axes, [robot_image, height_image], titles):
        ax.imshow(image, origin='lower', extent=extent, interpolation='none', zorder=1)
        metric_axes(ax, view_limits if view_limits is not None else extent)
        ax.set_title(title, loc='left', pad=9)
        for robot, xyz in tracks.items():
            line, = ax.plot(xyz[:, 0], xyz[:, 1], lw=.8,
                            color='#202830' if ax is axes[0] else COLORS[robot], zorder=4)
            line.set_path_effects([effects.Stroke(linewidth=1.8, foreground='white'), effects.Normal()])
    handles = [Line2D([], [], color=COLORS[r], lw=4, label=r) for r in tracks]
    axes[0].legend(handles=handles, frameon=False, ncol=3, loc='upper center', bbox_to_anchor=(.5, -.20))
    colorbar = fig.colorbar(matplotlib.cm.ScalarMappable(norm=norm, cmap='viridis'), cax=cax, extend='both')
    colorbar.set_label('Elevation relative to start (m)')
    fig.subplots_adjust(left=.075, right=.92, top=.94, bottom=.12, wspace=.28)
    box = cax.get_position()
    cax.set_position([box.x0, box.y0 + .2*box.height, box.width, .6*box.height])
    save(fig, output, 'map_top_down', dpi)
    return dict(display_resolution_m=resolution, occupied_pixels=count, extent_m=extent, view_limits_m=view_limits)


def oblique_figure(points, tracks, norm, output, dpi, max_points, view_limits=None):
    # Fixed-seed sampling is for rendering only. The saved map is unchanged.
    if view_limits is not None:
        xmin,xmax,ymin,ymax = view_limits
        points = points[(points[:,0]>=xmin)&(points[:,0]<=xmax)&(points[:,1]>=ymin)&(points[:,1]<=ymax)]
    rng = np.random.default_rng(0)
    indices = np.sort(rng.choice(len(points), min(len(points), max_points), replace=False))
    sample = points[indices]
    fig = plt.figure(figsize=(9.1, 6.5))
    ax = fig.add_subplot(111, projection='3d', computed_zorder=False)
    ax.scatter(sample[:, 0], sample[:, 1], sample[:, 2], c=sample[:, 2], cmap='viridis', norm=norm,
               s=.10, linewidths=0, depthshade=False, rasterized=True, zorder=1)
    for robot, xyz in tracks.items():
        ax.plot(*xyz.T, lw=1.1, color=COLORS[robot], zorder=3, label=robot)
    lower, upper = points.min(0), points.max(0)
    ax.set(xlim=(lower[0], upper[0]), ylim=(lower[1], upper[1]), zlim=(lower[2], upper[2]),
           xlabel='x (m)', ylabel='y (m)', zlabel='Elevation (m)')
    ax.set_box_aspect(upper - lower)
    ax.view_init(elev=52, azim=-65)
    for axis in (ax.xaxis, ax.yaxis, ax.zaxis):
        axis.pane.fill = False
        axis.set_major_locator(MaxNLocator(nbins=5))
        axis._axinfo['grid']['color'] = (.7, .73, .76, .35)
    ax.zaxis.set_major_locator(MaxNLocator(nbins=3))
    ax.zaxis.set_tick_params(labelsize=7, pad=1)
    ax.set_title('Merged CBS map — oblique view', loc='left', pad=8)
    ax.legend(loc='lower center', bbox_to_anchor=(.45, -.025), frameon=False, ncol=3)
    cax = fig.add_axes([.88, .28, .018, .43])
    cax.set_zorder(10)
    colorbar = fig.colorbar(matplotlib.cm.ScalarMappable(norm=norm, cmap='viridis'), cax=cax, extend='both')
    colorbar.set_label('Elevation relative to start (m)')
    fig.subplots_adjust(left=0, right=.89, top=.94, bottom=.05)
    save(fig, output, 'map_oblique', dpi)
    return dict(rendered_points=len(sample), sampling_seed=0, elevation_deg=52, azimuth_deg=-65, vertical_exaggeration=1, view_limits_m=view_limits)


def error_figure(evaluation, report, output, dpi):
    """Plot native evo arrays and statistics; never recalculate trajectory errors."""
    from evo.tools.file_interface import load_res_file
    saved = {}
    for robot in ROBOTS:
        for method in ('cbs', 'centralized'):
            path = evaluation/'evo'/method/f'{robot}-ape.zip'
            if path.is_file():
                saved[robot, method] = load_res_file(path)
    robots = [r for r in ROBOTS if (r, 'cbs') in saved]
    if not robots:
        return None
    epoch = min(x.np_arrays['timestamps'][0] for x in saved.values())
    fig, axes = plt.subplots(len(robots), 1, figsize=(7.6, 1.8*len(robots)+.4), sharex=True, squeeze=False)
    values = {}
    for ax, robot in zip(axes[:,0], robots):
        values[robot] = {}
        for method, color, style in [('cbs', COLORS[robot], '-'), ('centralized', '#5b6168', '--')]:
            if (robot, method) not in saved:
                continue
            result = saved[robot, method]
            times = result.np_arrays['timestamps'] - epoch
            errors = result.np_arrays['error_array']
            gaps = np.flatnonzero(np.diff(times) > 2)+1
            ax.plot(np.insert(times, gaps, np.nan), np.insert(errors, gaps, np.nan), color=color, ls=style, lw=1)
            values[robot][method] = dict(samples=len(errors), statistics=result.stats)
        ax.set_title(f'{robot}  |  evo APE RMSE {saved[robot,"cbs"].stats["rmse"]:.2f} m', loc='left', fontsize=9.5)
        ax.set(ylabel='Translation APE (m)', ylim=(0, None)); ax.grid(True)
        ax.yaxis.set_major_locator(MaxNLocator(nbins=4))
    axes[-1,0].set_xlabel('Time from first associated estimate (s)')
    fig.tight_layout(h_pad=1)
    save(fig, output, 'position_error', dpi)
    return values


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input-run', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--dpi', type=int, default=350)
    parser.add_argument('--bev-resolution', type=float, default=.30)
    parser.add_argument('--oblique-points', type=int, default=650000)
    parser.add_argument('--height-range', type=float, nargs=2, default=[0,25], metavar=('MIN','MAX'))
    parser.add_argument('--map-view-margin', type=float, help='Display only: frame maps around trajectories plus this XY margin in metres')
    args = parser.parse_args()
    registry = read_json(args.input_run); artifacts = registry['artifacts']
    evaluation = Path(artifacts['dpgo_evaluate']); output = args.output.resolve()
    if output == evaluation or evaluation in output.parents:
        raise ValueError('Figures must be written outside the immutable evaluation artifact')
    output.mkdir(parents=True, exist_ok=True)
    configure()
    complete = validate_stage(evaluation)
    report = read_json(evaluation/'report.json')
    rows = {r: read_jsonl(evaluation/f'{r}-corrected.jsonl') for r in ROBOTS}
    key_rows = read_jsonl(evaluation/'poses.jsonl')
    factors = read_jsonl(evaluation/'factors.jsonl')
    components = sorted(set(report['components'].values()))
    gap_ns = round(registry['config']['evaluation']['gt_max_gap_s'] * 10**9)
    component_info = {}
    for component in components:
        robots = [r for r in ROBOTS if report['components'][r] == component]
        destination = output if len(components) == 1 else output/component
        destination.mkdir(parents=True, exist_ok=True)
        metrics = report['trajectory'].get(component)
        alignment = np.asarray(metrics['alignment_SE3']) if metrics else np.eye(4)
        gt = {r: ground_truth(Path(registry['config']['dataset'])/f'{r.lower()}_gt.txt')
              for r in (metrics['robots'] if metrics else [])}
        origin = gt[metrics['robots'][0]][1][0] if metrics else np.zeros(3)
        tracks = {r: paths(rows[r], alignment, origin) for r in robots}
        truth = {r: np.insert(gt[r][1]-origin, np.flatnonzero(np.diff(gt[r][0]) > gap_ns)+1, np.nan, axis=0)
                 if r in gt else np.empty((0,3)) for r in robots}
        local_keys = [r for r in key_rows if r['robot_id'] in robots]
        keys = {(r['robot_id'], r['keyframe_id']): xyz for r, xyz in zip(local_keys, paths(local_keys, alignment, origin))}
        local_factors = [e for e in factors if tuple(e['i']) in keys and tuple(e['j']) in keys]
        trajectory_figure(tracks, truth, keys, local_factors, destination, args.dpi)
        clouds = []
        for robot in robots:
            with np.load(evaluation/f'{robot}-maps.npz') as data:
                cloud = transform(alignment, data['optimized']) - origin
            if not np.isfinite(cloud).all():
                raise ValueError('Nonfinite map point')
            clouds.append(cloud.astype(np.float32))
        owners = np.concatenate([np.full(len(c), ROBOTS.index(r), dtype=np.uint8) for r,c in zip(robots,clouds)])
        points = np.concatenate(clouds)
        norm = colors.Normalize(vmin=args.height_range[0], vmax=args.height_range[1], clip=True)
        view_limits = None
        if args.map_view_margin is not None:
            if args.map_view_margin <= 0: raise ValueError('Map view margin must be positive')
            xy = np.concatenate(list(tracks.values()))[:,:2]
            low,high = xy.min(0)-args.map_view_margin,xy.max(0)+args.map_view_margin
            view_limits = [float(low[0]),float(high[0]),float(low[1]),float(high[1])]
        bev = bev_figure(points, owners, tracks, norm, destination, args.dpi, args.bev_resolution, view_limits)
        oblique = oblique_figure(points, tracks, norm, destination, args.dpi, args.oblique_points, view_limits)
        component_info[component] = dict(robots=robots, alignment_SE3=alignment.tolist(), origin_world_m=origin.tolist(),
            alignment_scope=metrics['alignment_scope'] if metrics else 'CBS component frame; no ground-truth alignment',
            map_point_counts={r:len(c) for r,c in zip(robots,clouds)}, map_point_total=len(points), bev=bev, oblique=oblique)
    errors = error_figure(evaluation, report, output, args.dpi)
    metadata = dict(schema_version=2, input_run=str(args.input_run.resolve()),
        evaluation_stage=complete['stage_hash'], evaluation_files=complete['files'],
        generator_sha256=file_hash(Path(__file__)), matplotlib_version=matplotlib.__version__,
        dpi=args.dpi, components=component_info,
        map_color='robot identity or elevation; XYZ source contains no RGB',
        elevation_color_limits_m=args.height_range, position_error=errors,
        trajectory_error_source='saved evo result arrays and statistics',
        files={str(p.relative_to(output)):file_hash(p) for p in sorted(output.rglob('*')) if p.suffix in ('.png','.pdf')})
    write_json(output/'figures.json', metadata)
    print(f'Figures: {output}', flush=True)


if __name__ == '__main__':
    main()
