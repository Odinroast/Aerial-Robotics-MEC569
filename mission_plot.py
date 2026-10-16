"""Plots for mission.py logs: live while flying, and a PNG at the end.

    python3 mission_plot.py mission_log_XXXX.csv                     # save mission_log_XXXX.png
    python3 mission_plot.py mission_log_XXXX.csv --goal 3.25 1.14   # also mark B and the sweep areas
    python3 mission_plot.py mission_log_XXXX.csv --live             # follow a log that is being written

Left: map with trajectory per leg, wall hits from the side rangers, sweep areas, box
detections ('enter'), center estimates and landing spots. Right: zrange over time with
box-finding periods shaded.
"""
import argparse
import csv
import math
import time
from pathlib import Path

SENSOR_ANGLES = (0., 180., 90., -90.)
WALL_BEAMS = ('front', 'back', 'left', 'right')
LEG_COLORS = {'1': 'tab:blue', '2': 'tab:orange'}
HEIGHT = 0.5


def number(row, key):
    try:
        return float(row.get(key) or 'nan')
    except ValueError:
        return math.nan


class Track:
    """Rows of a mission log, read incrementally; time continues across legs."""

    def __init__(self):
        self.rows, self.events, self.result = [], [], None
        self._offset, self._leg, self._last_t = 0., None, 0.

    def add(self, row):
        if row.get('state') == 'LANDED':
            self.events.append(('landed', row.get('leg'), self.rows[-1] if self.rows else None, row.get('event')))
            return
        if row.get('state') == 'END':
            self.result = row.get('event')
            return
        if not row.get('time'):
            return
        if row['leg'] != self._leg:
            self._offset, self._leg = self._last_t + (2. if self._leg else 0.), row['leg']
        row['t'] = self._offset + number(row, 'time')
        self._last_t = row['t']
        self.rows.append(row)


def sweep_areas(meta):
    """[(leg, x_lines, (y0, y1), window)] from the mission parameters, or [] without them."""
    if not meta or meta.get('goal') is None:
        return []
    import mission
    start, goal = meta.get('start') or mission.START, meta['goal']
    sweep, window = meta.get('sweep') or mission.SWEEP, meta.get('window') or mission.X_WINDOW
    box = meta.get('box_size') or mission.bf.BOX_SIZE
    return [(str(k), mission.sweep_lines(x, window, box), y_range, window, x)
            for k, (_, x, y_range, _) in enumerate(mission.mission_legs(start, goal, sweep), 1)]


def draw(fig, track, meta=None, title=None, live=False):
    from matplotlib.patches import Rectangle
    fig.clf()
    ax, bx = fig.subplots(1, 2, gridspec_kw={'width_ratios': (1.3, 1)})
    rows = track.rows
    hits = []
    for r in rows[::4]:
        for beam, offset in zip(WALL_BEAMS, SENSOR_ANGLES):
            d = number(r, beam)
            if math.isfinite(d) and d < 2.:
                a = math.radians(number(r, 'yaw') + offset)
                hits.append((number(r, 'x') + d*math.cos(a), number(r, 'y') + d*math.sin(a)))
    if hits:
        ax.scatter(*zip(*hits), s=2, c='0.4', alpha=.35, label='wall hits')
    for leg, x_lines, (y0, y1), window, x_target in sweep_areas(meta):
        color = LEG_COLORS.get(leg, 'k')
        ax.add_patch(Rectangle((x_target - window, min(y0, y1)), 2*window, abs(y1 - y0),
                               fill=False, ls='--', ec=color, alpha=.6))
        for x in x_lines:
            ax.plot([x, x], [y0, y1], ':', color=color, lw=1, alpha=.7)
    for leg in sorted({r['leg'] for r in rows}):
        part = [r for r in rows if r['leg'] == leg]
        ax.plot([number(r, 'x') for r in part], [number(r, 'y') for r in part], '-',
                color=LEG_COLORS.get(leg, 'k'), lw=1.5, label=f'leg {leg}')
        centers = [(number(r, 'center_x'), number(r, 'center_y')) for r in part
                   if math.isfinite(number(r, 'center_x'))]
        if centers:
            ax.plot(*centers[-1], '+', color='tab:green', ms=14, mew=2.5)
    enters = [r for r in rows if r.get('event') == 'enter' and r['state'].startswith('BOX')]
    if enters:
        ax.plot([number(r, 'x') for r in enters], [number(r, 'y') for r in enters], 'x', color='tab:red',
                ms=8, mew=2, label='box edge seen')
    for _, leg, r, text in track.events:
        if r is not None and text and text.startswith('landed'):
            ax.plot(number(r, 'x'), number(r, 'y'), '*', color='gold', mec='k', ms=16, label='landed')
    if meta and meta.get('start'):
        ax.plot(*meta['start'], 's', color='tab:blue', mec='k', ms=9, label='A')
    if meta and meta.get('goal'):
        ax.plot(*meta['goal'], '^', color='tab:orange', mec='k', ms=10, label='B')
    if live and rows:
        ax.plot(number(rows[-1], 'x'), number(rows[-1], 'y'), 'o', color='k', ms=8)
    ax.set_aspect('equal', adjustable='datalim')
    ax.grid(alpha=.3)
    ax.set(xlabel='x (m)', ylabel='y (m)')
    handles, labels = ax.get_legend_handles_labels()
    unique = dict(zip(labels, handles))
    ax.legend(unique.values(), unique.keys(), loc='best', fontsize=7)

    t = [r['t'] for r in rows]
    for leg in sorted({r['leg'] for r in rows}):
        part = [r for r in rows if r['leg'] == leg]
        bx.plot([r['t'] for r in part], [number(r, 'zrange') for r in part], lw=.8,
                color=LEG_COLORS.get(leg, 'k'), label=f'zrange leg {leg}')
    if rows:
        boxing = [r['state'].startswith('BOX') for r in rows]
        bx.fill_between(t, 0, 1, where=boxing, transform=bx.get_xaxis_transform(), color='tab:green',
                        alpha=.08, label='box finding')
        bx.axhline(HEIGHT, color='k', lw=.6, ls=':')
        for r in rows:
            if r.get('event') == 'enter':
                bx.axvline(r['t'], color='tab:red', lw=.6, alpha=.6)
    bx.set(xlabel='mission time (s)', ylabel='zrange (m)', ylim=(.2, .8))
    bx.grid(alpha=.3)
    bx.legend(loc='upper right', fontsize=7)
    state = rows[-1]['state'] if rows else 'waiting for data'
    fig.suptitle(title or (f'{track.result}' if track.result else state), fontsize=11)
    fig.tight_layout()


def load(path):
    track = Track()
    with open(path, newline='') as f:
        for row in csv.DictReader(f):
            track.add(row)
    return track


def save(path, meta=None, out=None):
    """Draw a finished (or partial) log to PNG without needing a display."""
    from matplotlib.figure import Figure
    from matplotlib.backends.backend_agg import FigureCanvasAgg
    out = out or str(Path(path).with_suffix('.png'))
    track = load(path)
    fig = Figure(figsize=(15, 6.5))
    FigureCanvasAgg(fig)
    draw(fig, track, meta, title=f'{Path(path).name}: {track.result or "no result"}')
    fig.savefig(out, dpi=110)
    return out


def live(path, meta=None, period=0.7):
    """Follow a log as mission.py writes it. Runs until the window is closed."""
    import matplotlib.pyplot as plt
    if plt.get_backend().lower() in ('agg', 'pdf', 'svg', 'ps', 'cairo', 'template'):
        print('Live plot: no display available, skipping (the PNG is still saved at the end)')
        return
    while not Path(path).exists():
        time.sleep(.2)
    track, header, pending = Track(), None, ''
    plt.ion()
    fig = plt.figure('mission', figsize=(14, 6.5))
    with open(path, newline='') as f:
        while plt.fignum_exists(fig.number):
            pending += f.read()
            lines = pending.split('\n')
            pending = lines.pop()               # keep a partly written line for later
            for line in lines:
                if not line.strip():
                    continue
                values = next(csv.reader([line]))
                if header is None:
                    header = values
                    continue
                track.add(dict(zip(header, values)))
            draw(fig, track, meta, live=track.result is None)
            fig.canvas.draw_idle()
            plt.pause(period)


def start_live(path, meta=None):
    """Start the live plot in its own process (started before cflib so nothing is forked)."""
    import multiprocessing
    process = multiprocessing.get_context('spawn').Process(target=live, args=(path, meta), daemon=False)
    process.start()
    return process


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('log')
    parser.add_argument('--start', nargs=2, type=float, metavar=('X', 'Y'))
    parser.add_argument('--goal', nargs=2, type=float, metavar=('X', 'Y'))
    parser.add_argument('--sweep', nargs=2, type=float, metavar=('DY_MIN', 'DY_MAX'))
    parser.add_argument('--live', action='store_true')
    args = parser.parse_args()
    meta = {'start': args.start, 'goal': args.goal, 'sweep': args.sweep}
    if args.live:
        live(args.log, meta)
    else:
        print(save(args.log, meta))
