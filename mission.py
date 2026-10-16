"""Two-box mission: from box A fly to B, find and land on the box there, then come back
and land on the box at A.

    python3 mission.py --start 0 0 --goal 3.0 0.5 --sweep -1.0 1.0
    python3 mission.py --box-size 0.40 --box-height 0.29 --x-window 0.3 --pause 5

Coordinates are map meters; START (point A) is where the drone sits at takeoff, on top
of box 1, nose along map +X. Each leg:

1. NAV: fly to the first sweep line with the obstacle-avoiding navigator (planner.py).
2. SWEEP: fly parallel lines along Y across the --sweep range, near the target x. The box
   center may be up to --x-window from it; the lines are close enough that one of them
   crosses the box well inside its edges. Each line is a box_finder search; on the first
   dip it finds the center (one pass each way along Y and X) and the leg ends.
3. LAND on the center. After --pause seconds take off again for the next leg.

Leg 1 sweeps at the goal's x. Leg 2 first sweeps a short Y range around the remembered
box 1 (START, after drift: ~15-30 cm off), then the whole --sweep range at START's x.

Takeoff resets the position estimate on the ground, so each leg anchors its odometry to
the map pose where the drone last landed. The nose stays at +X the whole time.
"""
import argparse
import csv
from datetime import datetime
import math
import threading
import time
import box_finder as bf
from box_finder import BoxFinder, Failed
from planner import Navigator, guard, wrap

START = (0.0, 0.0)          # Point A: map position of box 1, where the drone takes off, m.
GOAL = (3.25, 1.14)           # Point B: the box there is near this x, m.
SWEEP = (-0.5, 0.5)         # Y range swept at each end, map m. Start/end it off the box.
X_WINDOW = 0.30             # m; the box center is within this of the target x.
LINE_MARGIN = 0.05          # m; a line must pass this far inside the box edge to see it well.
LOCAL_UNCERTAINTY = 0.30    # m; how far box 1 may be from where we remember it.
PAUSE = 5.                  # s on box 2 before taking off again.
NAV_MIN = 0.50              # m; shorter hops fly straight instead of using the navigator.
HOP_SPEED = 0.20            # m/s
HOP_SECONDS = 15.
HEIGHT_TOLERANCE = 0.15     # While navigating, land if zrange is off HEIGHT by more than this
HEIGHT_SECONDS = 1.5        # ...for this long. Box edges cause ~0.6 s spikes.
LEG_SECONDS = 300.
DEFAULT_URI = 'radio://0/98/2M/E7E7E7E7E8'


def sweep_lines(x_target, window=X_WINDOW, box_size=bf.BOX_SIZE):
    """X positions of parallel sweep lines so a box centered within `window` of x_target
    has a line at least LINE_MARGIN inside its edges."""
    reach = box_size/2 - LINE_MARGIN
    if reach <= 0:
        raise ValueError('box too small for LINE_MARGIN')
    n = max(1, math.ceil(window/reach - 1e-9))
    spacing = 2*window/n
    return [x_target - window + spacing*(k + .5) for k in range(n)]


class Leg:
    """Pure logic for one flight: from wherever the drone is, find the box near x_target in
    the Y range and stop over its center. Feed step() map-frame poses."""

    def __init__(self, x_target, y_range, box_size=bf.BOX_SIZE, box_height=bf.BOX_HEIGHT,
                 window=X_WINDOW, remembered=None, arena=None):
        self.box_size, self.box_height, self.arena = box_size, box_height, arena
        self.plans = []
        if remembered is not None:
            half = box_size/2 + LOCAL_UNCERTAINTY + .1
            self.plans.append(('local', remembered[0], (remembered[1] - half, remembered[1] + half)))
        self.plans.append(('full', x_target, tuple(y_range)))
        self.window = window
        self.pos, self.yaw, self.now, self.zrange = (0., 0.), 0., 0., math.nan
        self.ranges = (math.nan,)*4
        self.state, self.done, self.fault = 'START', False, None
        self.finder, self.nav, self.center, self.notes = None, None, None, []
        self.task = self._run()

    def _body(self, wx, wy):
        c, s = math.cos(math.radians(self.yaw)), math.sin(math.radians(self.yaw))
        vx, vy = guard(c*wx + s*wy, -s*wx + c*wy, self.ranges)
        return vx, vy, max(-20., min(20., -2.*self.yaw))

    def _travel(self, point):
        if math.dist(self.pos, point) > NAV_MIN:
            self.nav = Navigator(point, self.arena)
            while True:
                command = self.nav.step(self.pos[0], self.pos[1], self.yaw, self.ranges, self.now)
                if self.nav.fault:
                    raise Failed(f'navigation: {self.nav.fault}')
                if self.nav.done:
                    break
                self.state = 'NAV ' + self.nav.state
                yield command
            self.nav = None
        self.state, start = 'HOP', self.now
        while math.dist(self.pos, point) > .03:
            if self.now - start > HOP_SECONDS:
                raise Failed(f'could not reach ({point[0]:.2f}, {point[1]:.2f})')
            vx, vy = bf.GAIN*(point[0]-self.pos[0]), bf.GAIN*(point[1]-self.pos[1])
            n = math.hypot(vx, vy)
            if n > HOP_SPEED:
                vx, vy = vx*HOP_SPEED/n, vy*HOP_SPEED/n
            yield self._body(vx, vy)

    def _run(self):
        for name, x_target, (y_a, y_b) in self.plans:
            lines = sorted(sweep_lines(x_target, self.window, self.box_size), key=lambda x: abs(x - self.pos[0]))
            if abs(self.pos[1] - y_b) < abs(self.pos[1] - y_a):
                y_a, y_b = y_b, y_a
            for k, x_line in enumerate(lines):
                y0, y1 = (y_a, y_b) if k % 2 == 0 else (y_b, y_a)
                yield from self._travel((x_line, y0))
                self.finder = BoxFinder(search_direction=(0., 1. if y1 > y0 else -1.),
                                        search_distance=abs(y1 - y0), box_height=self.box_height,
                                        box_size=self.box_size)
                while True:
                    command = self.finder.step(self.now, self.pos[0], self.pos[1], self.yaw,
                                               self.zrange, self.ranges)
                    self.state = 'BOX ' + self.finder.state
                    if self.finder.done:
                        self.center, self.state, self.done = self.finder.center, 'LAND', True
                        return
                    if self.finder.fault:
                        if 'no box found' not in self.finder.fault:
                            raise Failed(self.finder.fault)
                        self.notes.append(f'{name} sweep, line x={x_line:.2f}: {self.finder.fault}')
                        break
                    if self.finder.center is not None:
                        self.center = self.finder.center
                    yield command
        raise Failed('no box found on any sweep line')

    def step(self, t, x, y, yaw, zrange, ranges=(math.nan,)*4):
        """Return body-frame (vx, vy, yaw_rate) for MotionCommander.start_linear_motion."""
        if self.done or self.fault:
            return 0., 0., 0.
        self.pos, self.yaw, self.now, self.zrange, self.ranges = (x, y), yaw, t, zrange, tuple(ranges)
        try:
            return next(self.task)
        except Failed as error:
            self.fault, self.state = str(error), 'FAILED'
        except StopIteration:
            pass
        return 0., 0., 0.


class Anchor:
    """Map pose of the estimator origin. take_off() resets the estimator while the drone is
    still on the ground, so odometry (0, 0, 0) is the spot where it sat."""

    def __init__(self, x, y, yaw=0.):
        self.x, self.y, self.yaw = x, y, yaw

    def pose(self, ox, oy, oyaw):
        c, s = math.cos(math.radians(self.yaw)), math.sin(math.radians(self.yaw))
        return self.x + c*ox - s*oy, self.y + s*ox + c*oy, wrap(oyaw + self.yaw)


# ------------------------------------------------------------------- hardware
WALL_BEAMS = bf.WALL_BEAMS
FIELDS = ['time', 'leg', 'state', 'event', 'x', 'y', 'yaw', 'odom_x', 'odom_y', 'z', 'zrange',
          *WALL_BEAMS, 'up', 'deviation', 'cmd_vx', 'cmd_vy', 'cmd_yaw_rate', 'center_x', 'center_y']


def fly(uri, output, start=START, goal=GOAL, sweep=SWEEP, window=X_WINDOW,
        box_size=bf.BOX_SIZE, box_height=bf.BOX_HEIGHT, pause=PAUSE, arena=None):
    import cflib.crtp
    from cflib.crazyflie import Crazyflie
    from cflib.crazyflie.log import LogConfig
    from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
    from cflib.positioning.motion_commander import MotionCommander

    latest, lock = {}, threading.Lock()

    def on_log(timestamp, data, config):
        with lock:
            latest[config.name] = (time.monotonic(), dict(data))

    def odom():
        with lock:
            d = latest['pose'][1]
        return d['stateEstimate.x'], d['stateEstimate.y'], d['stateEstimate.yaw']

    legs = [('to B', goal[0], None), ('back to A', start[0], start)]
    anchor, results = Anchor(*start), []
    cflib.crtp.init_drivers()
    with open(output, 'w', newline='') as file, \
            SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        log = csv.DictWriter(file, fieldnames=FIELDS)
        log.writeheader()
        # Two blocks: one log packet holds at most 26 bytes.
        pose = LogConfig(name='pose', period_in_ms=20)
        pose.add_variable('range.zrange', 'uint16_t')
        for axis in ('x', 'y', 'z', 'yaw'):
            pose.add_variable('stateEstimate.'+axis, 'float')
        walls = LogConfig(name='walls', period_in_ms=20)
        for beam in (*WALL_BEAMS, 'up'):
            walls.add_variable('range.'+beam, 'uint16_t')
        configs = (pose, walls)
        for config in configs:
            scf.cf.log.add_config(config)
            config.data_received_cb.add_callback(on_log)
            config.start()
        motion = MotionCommander(scf, default_height=bf.HEIGHT)
        try:
            deadline = time.monotonic() + 5.
            while len(latest) < len(configs):
                if time.monotonic() > deadline:
                    raise RuntimeError('No telemetry')
                time.sleep(.05)
            for number, (name, x_target, remembered) in enumerate(legs, 1):
                if number > 1:
                    print(f'On the box; taking off again in {pause:.0f} s')
                    time.sleep(pause)
                leg = Leg(x_target, sweep, box_size, box_height, window, remembered, arena)
                scf.cf.platform.send_arming_request(True)
                time.sleep(1.)
                print(f'Leg {number} ({name}): taking off at map ({anchor.x:.2f}, {anchor.y:.2f}); '
                      f'sweep lines x = {", ".join(f"{x:.2f}" for x in sweep_lines(x_target, window, box_size))}')
                motion.take_off()
                motion.start_linear_motion(0., 0., 0., 0.)
                time.sleep(1.)
                result, t0 = None, time.monotonic()
                height_ok = last_print = t0
                printed, printed_finder, printed_finder_notes = 0, None, 0
                while result is None:
                    now = time.monotonic()
                    with lock:
                        (got_pose, d), (got_walls, w) = latest['pose'], latest['walls']
                    if now - min(got_pose, got_walls) > .5:
                        result = 'telemetry lost'; break
                    if now - t0 > LEG_SECONDS:
                        result = 'leg time limit'; break
                    up = bf.meters(w['range.up'])
                    if up < .2:
                        result = 'object above the drone'; break
                    zrange = bf.meters(d['range.zrange'])
                    if not leg.state.startswith('NAV') or (math.isfinite(zrange) and abs(zrange - bf.HEIGHT) < HEIGHT_TOLERANCE):
                        height_ok = now
                    if now - height_ok > HEIGHT_SECONDS:
                        result = f'height lost while navigating (z-range {zrange:.2f} m)'; break
                    ranges = tuple(bf.meters(w['range.'+beam]) for beam in WALL_BEAMS)
                    ox, oy, oyaw = d['stateEstimate.x'], d['stateEstimate.y'], d['stateEstimate.yaw']
                    x, y, yaw = anchor.pose(ox, oy, oyaw)
                    command = leg.step(now - t0, x, y, yaw, zrange, ranges)
                    finder = leg.finder
                    c = leg.center or (math.nan, math.nan)
                    log.writerow({'time': now - t0, 'leg': number, 'state': leg.state,
                                  'event': (finder.event if finder else None) or '',
                                  'x': x, 'y': y, 'yaw': yaw, 'odom_x': ox, 'odom_y': oy,
                                  'z': d['stateEstimate.z'], 'zrange': zrange,
                                  **dict(zip(WALL_BEAMS, ranges)), 'up': up,
                                  'deviation': finder.detector.deviation if finder else '',
                                  **dict(zip(('cmd_vx', 'cmd_vy', 'cmd_yaw_rate'), command)),
                                  'center_x': c[0], 'center_y': c[1]})
                    if finder is not printed_finder:
                        printed_finder, printed_finder_notes = finder, 0
                    while finder and len(finder.notes) > printed_finder_notes:
                        print(finder.notes[printed_finder_notes]); printed_finder_notes += 1
                    while len(leg.notes) > printed:
                        print(leg.notes[printed]); printed += 1
                    if finder and finder.event == 'enter' and finder.state == 'SEARCH':
                        print(f'Box found at ({x:.2f}, {y:.2f})')
                    if leg.fault:
                        result = leg.fault; break
                    if leg.done:
                        result = f'landed on box at ({c[0]:.2f}, {c[1]:.2f})'; break
                    if now - last_print > 1.:
                        print(f'{leg.state:14s} ({x:5.2f}, {y:5.2f})')
                        last_print = now
                    motion.start_linear_motion(command[0], command[1], 0., command[2])
                    time.sleep(max(0., .02 - (time.monotonic() - now)))
                motion.land(velocity=bf.LAND_SPEED)
                print(f'Leg {number}: {result}')
                results.append(result)
                log.writerow({'leg': number, 'state': 'LANDED', 'event': result})
                if not leg.done:
                    break
                time.sleep(.5)
                # The estimate stays valid on the ground until the next takeoff resets it.
                anchor = Anchor(*anchor.pose(*odom()))
                print(f'Landed at map ({anchor.x:.2f}, {anchor.y:.2f})')
        except KeyboardInterrupt:
            results.append('interrupted')
            print('Interrupted; landing')
        finally:
            motion.land(velocity=bf.LAND_SPEED)   # no-op when already on the ground
            for config in configs:
                config.stop()
            log.writerow({'state': 'END', 'event': ' | '.join(results)})
    return results


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('uri', nargs='?', default=DEFAULT_URI)
    parser.add_argument('--start', nargs=2, type=float, metavar=('X', 'Y'), default=START,
                        help='point A, map m: box 1, where the drone takes off')
    parser.add_argument('--goal', nargs=2, type=float, metavar=('X', 'Y'), default=GOAL,
                        help='point B, map m: the second box is near this x')
    parser.add_argument('--sweep', nargs=2, type=float, metavar=('Y_MIN', 'Y_MAX'), default=SWEEP,
                        help='map Y range swept at both ends')
    parser.add_argument('--x-window', type=float, default=X_WINDOW,
                        help='m; box center is within this of the target x')
    parser.add_argument('--box-size', type=float, default=bf.BOX_SIZE, help='side of the square box, m')
    parser.add_argument('--box-height', type=float, default=bf.BOX_HEIGHT, help='m')
    parser.add_argument('--pause', type=float, default=PAUSE, help='s on box 2 before taking off again')
    parser.add_argument('--arena', nargs=4, type=float, metavar=('X_MIN', 'X_MAX', 'Y_MIN', 'Y_MAX'),
                        help='virtual fence for the navigator, map m (default: none)')
    parser.add_argument('--output', default=datetime.now().strftime('mission_log_%Y%m%d_%H%M%S.csv'))
    args = parser.parse_args()
    outcome = fly(args.uri, args.output, tuple(args.start), tuple(args.goal), tuple(args.sweep),
                  args.x_window, args.box_size, args.box_height, args.pause,
                  tuple(args.arena) if args.arena else None)
    print('Result:', ' | '.join(outcome), '| log:', args.output)
    raise SystemExit(0 if len(outcome) == 2 and all(r.startswith('landed') for r in outcome) else 1)
