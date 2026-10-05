"""Published reference frames at each robot's local iteration, before/after."""
import json
import os
from pathlib import Path

import numpy as np
os.environ.setdefault('MPLCONFIGDIR', '/tmp/circle-matplotlib')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap, BoundaryNorm
from matplotlib.patches import Patch

ROOT = Path(__file__).resolve().parent
ROBOTS = ['aerial05', 'aerial07', 'ground05', 'ground06']
NAMES = ['A05', 'A07', 'G05', 'G06']
COLORS = ['#277b83', '#dba850', '#a7aec1', '#b16f7b']


def main():
    plt.rcParams.update({'font.family':'serif', 'font.serif':['DejaVu Serif'],
                         'font.size':9, 'pdf.fonttype':42, 'savefig.dpi':300})
    fig, axes = plt.subplots(2, 1, figsize=(7.1, 3.5), sharex=True)
    for ax, path, title in zip(axes,
        ['before-iteration-timeline.jsonl', 'iteration-timeline.jsonl'],
        ['Before: anchor relay disabled', 'After: anchor relay preserves the original producer']):
        rows = [json.loads(line) for line in (ROOT/path).read_text().splitlines()]
        data = np.full((4, 30), np.nan)
        for row in rows:
            if 1 <= row['iteration'] <= 30:
                data[ROBOTS.index(row['robot']), row['iteration']-1] = ROBOTS.index(row['component'])
        assert np.isfinite(data).all(), 'Missing iteration observations'
        ax.imshow(data, cmap=ListedColormap(COLORS), norm=BoundaryNorm(np.arange(5)-.5, 4),
                  aspect='auto', interpolation='nearest', extent=(.5, 30.5, 3.5, -.5))
        ax.set_yticks(range(4), NAMES)
        ax.set_title(title, loc='left', fontsize=10, pad=6)
        ax.set_xticks([1, 5, 10, 15, 20, 25, 30])
        for spine in ax.spines.values(): spine.set_visible(False)
        ax.tick_params(length=0)
    axes[-1].set_xlabel('Local iteration (requested interval: 10 s)')
    fig.legend(handles=[Patch(facecolor=c, label=n+' frame') for c,n in zip(COLORS,NAMES)],
               loc='lower center', bbox_to_anchor=(.53, .01), ncol=4, frameon=False, fontsize=8)
    fig.subplots_adjust(left=.08, right=.98, top=.92, bottom=.23, hspace=.63)
    for ext in ('pdf', 'png'): fig.savefig(ROOT/f'anchor-propagation.{ext}')
    plt.close(fig)


if __name__ == '__main__': main()
