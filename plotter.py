"""Post-flight plot: python3 plotter.py flight_log_XXXX.csv"""
import csv
import math
import sys
from pathlib import Path
from planner import SENSOR_ANGLES


def plot_flight(csv_path, goal=None):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    with open(csv_path, newline='') as f:
        rows = list(csv.DictReader(f))
    if not rows:
        raise ValueError('Log has no flight rows')
    num = lambda r, k: float(r[k]) if r[k] not in ('', 'nan') else math.nan
    xs, ys = [num(r, 'x') for r in rows], [num(r, 'y') for r in rows]
    hits = []
    for r in rows:
        for name, offset in zip(('front', 'back', 'left', 'right'), SENSOR_ANGLES):
            d = num(r, name)
            if math.isfinite(d) and d < 2.:
                a = math.radians(num(r, 'yaw')+offset)
                hits.append((num(r, 'x')+d*math.cos(a), num(r, 'y')+d*math.sin(a)))

    fig, (ax, bx) = plt.subplots(1, 2, figsize=(14, 6), gridspec_kw={'width_ratios': (2, 1)})
    if hits:
        ax.scatter(*zip(*hits), s=2, c='k', alpha=.4, label='Ranger hits')
    ax.plot(xs, ys, 'r-', lw=2, label='Trajectory')
    ax.plot(xs[0], ys[0], 'bs', label='Start')
    if goal:
        ax.plot(*goal, 'g*', ms=15, label='Goal')
    ax.set(xlabel='X (m)', ylabel='Y (m)', title=f'{Path(csv_path).name}: {rows[-1]["reason"] if rows[-1]["state"] == "END" else rows[-1]["state"]}')
    ax.set_aspect('equal')
    ax.grid(alpha=.3)
    ax.legend(loc='best', fontsize=8)
    t = [num(r, 'time') for r in rows]
    bx.plot(t, [num(r, 'cmd_vx') for r in rows], label='cmd vx')
    bx.plot(t, [num(r, 'cmd_vy') for r in rows], label='cmd vy')
    bx.plot(t, [num(r, 'goal_distance') for r in rows], label='goal distance (m)')
    bx.set(xlabel='Time (s)', title='Commands')
    bx.grid(alpha=.3)
    bx.legend(fontsize=8)
    out = str(Path(csv_path).with_suffix('.png'))
    fig.tight_layout()
    fig.savefig(out, dpi=120)
    plt.close(fig)
    return out


if __name__ == '__main__':
    print(plot_flight(sys.argv[1]))
