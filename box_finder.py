"""Find a box on the floor with the down ranger, locate its center, and land on it.

Standalone test script, separate from the navigation stack (main.py / planner.py).

    python3 box_finder.py                          # take off, search along +X, land on the box
    python3 box_finder.py --box-height 0.26 --box-size 0.4   # box height and largest side, m
    python3 box_finder.py --search 0 1 --distance 1.5   # search direction (x y) and length
    python3 box_finder.py --replay flight_log.csv  # mark box edges in a ranger log

Place the drone on the floor, nose along +X, with the box somewhere ahead on the search
line. Ctrl+C lands.

How the box shows up: the firmware holds zrange (height above whatever is underneath) at
HEIGHT. Flying onto the box, zrange suddenly reads up to BOX_HEIGHT less until the height
controller climbs; flying off, it reads more until it descends. So the box is a dip and a
spike in zrange - HEIGHT. How big they are depends on speed, where the line crosses the
box, and how much the controller compensates (real logs: often only partly, ~0.4 m on top).

1. SEARCH + X PASS: fly along the search direction recording zrange against position;
   once it dips (over the box), keep going past the far edge, then fly back past the near
   edge. Controller lag and the ranger cone shift the profile along the direction of
   travel (~13 cm in real flights), so the way-back profile is the way-out profile
   mirrored about the box center. The center is the mirror point where they match best;
   this uses the whole profile and needs no edge thresholds.
2. Y PASS: the same, across, through the x center.
3. LAND at the result right away. Over the box the estimator measures height from the box
   top, so a normal landing touches down on it.

One pass per axis on purpose: the Flow-deck position drifts 2-18 cm between repeated
passes in real logs (the same edge moved that much), so repeats only re-measure in a
drifted frame. What counts is little time between measuring and landing.

Walls: every search, pass and go-to stops when the ranger facing the direction of travel
reads under WALL_STOP; the center fit works with passes of unequal length. On top of
that, planner.guard limits speed toward anything within 25 cm and backs away inside it.
The box edges need ~0.45 m of free floor to the nearest wall so a pass can get past them.
"""
import argparse
import collections
import csv
from datetime import datetime
import math
import threading
import time
import numpy as np
from planner import guard   # Body-frame range guard shared with the navigation stack.

HEIGHT = 0.5              # Flight height above the surface underneath, m.
BOX_HEIGHT = 0.16         # m; thresholds scale with it.
BOX_SIZE = 0.45           # m; largest side of the box (or a bit more). Sets the pass length.
# Thresholds as fractions of the box height, from the push log with a 26 cm box: hover
# zrange 0.515 +/- 0.010 m (max 0.534); edge dips to 0.19-0.38; on top of the box the
# controller only half recovers (0.38-0.41); spikes off the box reach 0.57-0.94.
# The dip shrinks with speed because the height controller keeps up better: real searches
# dipped 8-10 cm at 0.25 m/s but only 3.9-5.6 cm at 0.15 m/s, against floor noise within
# +/-1.9 cm (0.1 s average).
# The 16 cm box at 0.15 m/s dips only 2.3-4 cm below the floor level measured in flight
# (box log 190123: 2.3 cm for ~0.4 s). Against that level, no search over bare floor in the
# logs reached 1 cm for ENTER_HOLD, and 2 cm missed 190123.
ENTER_FRACTION = 0.09     # zrange this far below the floor level: over the box (2.6 cm @ 29 cm box)
LEAVE_FRACTION = 0.12     # zrange this far above it, after an enter: off the box.
MIN_LEVEL = 0.015         # m; enter/leave levels are never smaller than this (16 cm box: 1.5 cm).
ENTER_HOLD = 0.20         # s the dip must stay below MIN_LEVEL to count as an enter.
SIGNAL_FRACTION = 0.15    # a pass whose profile varies less than this missed the box (4.4 cm).
SMOOTH = 0.10             # s; average fresh zrange readings over this window.
# The hover right after takeoff read ~2 cm higher than the same floor in steady flight
# (0.52 vs 0.50 m), which alone crossed a 2 cm level. So the search measures the floor
# level while flying, and can't report an enter until that level is known.
FLOOR_SETTLE = 0.9        # s of search before an enter can count.
FLOOR_WINDOW = (1.5, 0.4) # floor level = median of search readings from this many s ago to this many.
SEARCH_DISTANCE = 2.0     # m to fly looking for the box before giving up.
PASS_SPEED = 0.15         # m/s; search and passes, the same both ways.
PAST_EDGE = 0.20          # m to fly past a box edge: real profiles settle ~0.4 m after it
                          # (13 cm lag, then the height controller's overshoot).
CALIBRATE = 1.5           # s of hover to measure the floor zrange after takeoff.
SETTLE = 1.0              # s to hold still so speed and height settle between passes.
WALL_STOP = 0.35          # m; end a search, pass or go-to when the ranger ahead reads less.
SENSOR_ANGLES = (0., 180., 90., -90.)   # front, back, left, right (body frame, degrees)
GAIN = 1.2                # 1/s, position hold / go-to gain.
LAND_SPEED = 0.15         # m/s
MISSION_SECONDS = 240.
DEFAULT_URI = 'radio://0/98/2M/E7E7E7E7E8'


class Failed(Exception):
    pass


class EdgeDetector:
    """Turns zrange readings into 'enter' / 'leave' box events (search and log replay).

    `height` is the hover zrange over the floor; set it from a measured hover when possible
    (the real hover sat at 0.515 m with HEIGHT 0.5)."""

    def __init__(self, height=HEIGHT, box_height=BOX_HEIGHT):
        self.height = height
        self.enter_level = max(MIN_LEVEL, ENTER_FRACTION*box_height)
        self.leave_level = max(MIN_LEVEL, LEAVE_FRACTION*box_height)
        self.samples = collections.deque()
        self.on_box = False
        self.deviation = 0.
        self.last = None
        self.below_since = None

    def update(self, t, z):
        """Feed one zrange reading (m). Returns 'enter', 'leave' or None."""
        # The ranger refreshes at ~20 Hz; repeated values are the same reading.
        if not math.isfinite(z) or z == self.last:
            return None
        self.last = z
        self.samples.append((t, z))
        while self.samples[0][0] < t - SMOOTH:
            self.samples.popleft()
        self.deviation = sum(v for _, v in self.samples)/len(self.samples) - self.height
        # The hold counts from MIN_LEVEL, so a deep box's dip isn't delayed past its own level.
        if self.deviation < -min(MIN_LEVEL, self.enter_level):
            self.below_since = t if self.below_since is None else self.below_since
        else:
            self.below_since = None
        if (not self.on_box and self.deviation < -self.enter_level
                and self.below_since is not None and t - self.below_since >= ENTER_HOLD):
            self.on_box = True
            return 'enter'
        if self.on_box and self.deviation > self.leave_level:
            self.on_box = False
            return 'leave'
        return None


def mirror_center(out, back, guess, span, min_signal=SIGNAL_FRACTION*BOX_HEIGHT):
    """Box center along a line from two zrange profiles [(position, zrange), ...] recorded
    flying out (+) and back (-) along it: the point c where back(2c - s) best matches out(s),
    searched within guess +/- span.

    Profiles are relative to their median (the floor level) and count as floor level
    outside the flown range, so a candidate that lines up floor with floor but leaves
    the box signatures unmatched scores badly."""
    out, back = np.array(out), np.array(back)
    out, back = out[np.argsort(out[:, 0])], back[np.argsort(back[:, 0])]
    for profile in (out, back):
        if len(profile) < 10 or np.ptp(profile[:, 1]) < min_signal:
            raise Failed(f'box not seen on a pass (profile varies {np.ptp(profile[:, 1]):.3f} m)')
        profile[:, 1] -= np.median(profile[:, 1])
    s = np.arange(guess - 3*span - 1., guess + 3*span + 1., .005)
    f = np.interp(s, out[:, 0], out[:, 1], left=0., right=0.)
    candidates = np.arange(guess-span, guess+span, .0025)
    scores = np.array([np.mean((f - np.interp(2*c - s, back[:, 0], back[:, 1], left=0., right=0.))**2)
                       for c in candidates])
    i = int(np.argmin(scores))
    if 0 < i < len(scores)-1:
        a, b, c = scores[i-1:i+2]
        if a - 2*b + c > 0:
            return float(candidates[i] + .0025*.5*(a - c)/(a - 2*b + c))
    return float(candidates[i])


class BoxFinder:
    """Pure logic: feed step() the pose and zrange, get body velocities back."""

    def __init__(self, search_direction=(1., 0.), search_distance=SEARCH_DISTANCE,
                 box_height=BOX_HEIGHT, box_size=BOX_SIZE, height=HEIGHT):
        n = math.hypot(*search_direction)
        ux, uy = search_direction[0]/n, search_direction[1]/n
        self.frame = ((ux, uy), (-uy, ux))      # search axis, perpendicular axis
        self.search_distance = search_distance
        self.box_height, self.box_size = box_height, box_size
        self.detector = EdgeDetector(height, box_height)
        self.pos, self.now, self.event, self.zrange = (0., 0.), 0., None, math.nan
        self.yaw, self.ranges = 0., (math.nan,)*4
        self.state, self.done, self.fault = 'START', False, None
        self.center, self.history, self.notes = None, [], []
        self.task = self._mission()

    # ----------------------------------------------------------------- tasks
    def _hold(self, point, seconds, state='SETTLE'):
        self.state, until = state, self.now + seconds
        while self.now < until:
            yield self._toward(point, PASS_SPEED)

    def _goto(self, point):
        """Fly to `point`, or stop short when a wall is ahead."""
        self.state, start = 'GOTO', self.now
        while math.dist(self.pos, point) > .03:
            if self.now - start > 20.:
                raise Failed('could not reach a waypoint')
            heading = (point[0]-self.pos[0], point[1]-self.pos[1])
            if self._ahead(heading) < WALL_STOP:
                self.notes.append(f'go-to stopped {math.dist(self.pos, point):.2f} m short: wall')
                point = self.pos
                break
            yield self._toward(point, PASS_SPEED)
        yield from self._hold(point, SETTLE)

    def _line(self, u, line, speed):
        """Velocity along unit vector u plus a correction back onto the line through `line`."""
        off = (self.pos[0]-line[0])*-u[1] + (self.pos[1]-line[1])*u[0]
        return speed*u[0] + GAIN*off*u[1], speed*u[1] - GAIN*off*u[0]

    def _pass(self, u, line, stop, profile=None, search=False):
        """Fly along u until the projection on u reaches `stop` (or a wall is ahead), appending
        (projection, zrange) to `profile`. With search=True, stop instead at the first 'enter'
        and fail if there is none within the search distance."""
        self.state, start, last = ('SEARCH' if search else 'PASS'), self.pos, None
        profile = [] if profile is None else profile
        began, floor = self.now, []
        while search or (self.pos[0]*u[0] + self.pos[1]*u[1]) < stop:
            if search:
                older = [z for t, z in floor if self.now - FLOOR_WINDOW[0] <= t < self.now - FLOOR_WINDOW[1]]
                if self.now - began < FLOOR_SETTLE or len(older) < 5:
                    self.detector.on_box = False
                elif self.event == 'enter':
                    break
                else:
                    self.detector.height = sorted(older)[len(older)//2]
                if self.zrange != last and math.isfinite(self.zrange):
                    floor.append((self.now, self.zrange))
            if search and math.dist(self.pos, start) > self.search_distance:
                raise Failed(f'no box found within {self.search_distance:.1f} m')
            if self._ahead(u) < WALL_STOP:
                if search:
                    raise Failed(f'wall ahead after {math.dist(self.pos, start):.2f} m of search, no box found')
                self.notes.append(f'pass along ({u[0]:+.2f}, {u[1]:+.2f}) stopped '
                                  f'{stop - (self.pos[0]*u[0] + self.pos[1]*u[1]):.2f} m short: wall')
                break
            if self.zrange != last and math.isfinite(self.zrange):
                profile.append((self.pos[0]*u[0] + self.pos[1]*u[1], self.zrange))
                last = self.zrange
            yield self._line(u, line, PASS_SPEED)
        return profile

    def _back(self, u, line, stop, out, guess, span):
        """After an out pass along u (profile `out`), hold, fly back to projection `stop`,
        and return the box center along u from the two profiles."""
        yield from self._hold(self.pos, SETTLE)
        back = yield from self._pass((-u[0], -u[1]), line, -stop)
        try:
            found = mirror_center(out, [(-s, z) for s, z in back], guess, span=span,
                                  min_signal=SIGNAL_FRACTION*self.box_height)
        except Failed as error:
            if any('wall' in note for note in self.notes):
                raise Failed(f'{error}; a wall cut a pass short, so the box is probably too '
                             f'close to it (needs ~{WALL_STOP + .15:.2f} m between box edge and wall)')
            raise
        self.history.append((u, guess, found))
        return found

    def _mission(self):
        # Hover over the floor first and use the measured zrange as the reference level.
        self.state, until, readings = 'SETTLE', self.now + CALIBRATE, []
        while self.now < until:
            if math.isfinite(self.zrange):
                readings.append(self.zrange)
            yield self._toward(self.pos, PASS_SPEED)
        if readings:
            self.detector.height = sorted(readings)[len(readings)//2]
        # Events during the hover were judged against the default level; start clean.
        self.detector.on_box = False
        (u, v), start = self.frame, self.pos
        along = lambda p, w: p[0]*w[0] + p[1]*w[1]

        # X: the search is the out pass. It stops at the near edge (plus lag); keep going
        # without stopping so speed stays constant, then come back past the near edge.
        out = yield from self._pass(u, start, None, search=True)
        hit = along(self.pos, u)
        self.center = self.pos
        yield from self._pass(u, start, hit + self.box_size + PAST_EDGE, out)
        # The enter lags the near edge, so the center is past `hit`. With a low box the dip
        # after leaving it looks like the one onto it, and an unbounded fit paired them
        # (16 cm box: center 0.37 instead of ~0.6). In odometry the boxes measured up to
        # ~20% longer than they are, hence the generous far bound.
        lo, hi = hit + .02, hit + .75*self.box_size
        cu = yield from self._back(u, start, hit - PAST_EDGE, out,
                                   guess=(lo + hi)/2, span=(hi - lo)/2)
        cv = along(start, v)
        self.center = (cu*u[0] + cv*v[0], cu*u[1] + cv*v[1])

        # Y: across the box through the x center, from one side to the other and back.
        half = self.box_size + PAST_EDGE
        yield from self._goto((self.center[0] - half*v[0], self.center[1] - half*v[1]))
        out = yield from self._pass(v, self.center, cv + half)
        cv = yield from self._back(v, self.center, cv - half, out,
                                   guess=cv, span=self.box_size/2 + .15)
        self.center = (cu*u[0] + cv*v[0], cu*u[1] + cv*v[1])
        yield from self._goto(self.center)
        self.state, self.done = 'LAND', True

    # --------------------------------------------------------------- control
    def _toward(self, point, speed):
        vx, vy = GAIN*(point[0]-self.pos[0]), GAIN*(point[1]-self.pos[1])
        n = math.hypot(vx, vy)
        return (vx*speed/n, vy*speed/n) if n > speed else (vx, vy)

    def _ahead(self, direction):
        """Closest horizontal range among beams within 50 degrees of a world direction."""
        heading = math.degrees(math.atan2(direction[1], direction[0])) - self.yaw
        near = [r for r, a in zip(self.ranges, SENSOR_ANGLES)
                if math.isfinite(r) and abs((heading - a + 180.) % 360. - 180.) < 50.]
        return min(near, default=math.inf)

    def step(self, t, x, y, yaw, zrange, ranges=(math.nan,)*4):
        """Return body-frame (vx, vy, yaw_rate) for MotionCommander.start_linear_motion.

        x, y in m and yaw in degrees from the state estimate; zrange in m; ranges are the
        (front, back, left, right) rangers in m, NaN for no return."""
        if self.done or self.fault:
            return 0., 0., 0.
        self.pos, self.now, self.zrange, self.yaw, self.ranges = (x, y), t, zrange, yaw, tuple(ranges)
        self.event = self.detector.update(t, zrange)
        try:
            wx, wy = next(self.task)
        except Failed as error:
            self.fault, self.state = str(error), 'FAILED'
            return 0., 0., 0.
        except StopIteration:
            return 0., 0., 0.
        c, s = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
        # The guard works on raw ranges, so it holds even if the plan or odometry is wrong.
        vx, vy = guard(c*wx + s*wy, -s*wx + c*wy, self.ranges)
        return vx, vy, max(-20., min(20., -2.*yaw))


# ------------------------------------------------------------------- hardware
WALL_BEAMS = ('front', 'back', 'left', 'right')
FIELDS = ['time', 'state', 'event', 'x', 'y', 'z', 'yaw', 'zrange', *WALL_BEAMS, 'up',
          'deviation', 'on_box', 'cmd_vx', 'cmd_vy', 'center_x', 'center_y']


def meters(mm):
    """Ranger value in mm -> meters; NaN for no return / out of range."""
    return mm/1000. if 20 < mm < 4000 else math.nan


def fly(uri, output, finder):
    import cflib.crtp
    from cflib.crazyflie import Crazyflie
    from cflib.crazyflie.log import LogConfig
    from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
    from cflib.positioning.motion_commander import MotionCommander

    latest, lock = {}, threading.Lock()

    def on_log(timestamp, data, config):
        with lock:
            latest[config.name] = (time.monotonic(), dict(data))

    result = 'not started'
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
        try:
            deadline = time.monotonic() + 5.
            while len(latest) < len(configs):
                if time.monotonic() > deadline:
                    raise RuntimeError('No telemetry')
                time.sleep(.05)
            scf.cf.platform.send_arming_request(True)
            time.sleep(1.)
            # MotionCommander takes off on enter and lands on exit (also on errors / Ctrl+C).
            with MotionCommander(scf, default_height=HEIGHT) as motion:
                motion.start_linear_motion(0., 0., 0., 0.)
                time.sleep(2.)
                start, printed_notes = time.monotonic(), 0
                while True:
                    now = time.monotonic()
                    with lock:
                        (got_pose, d), (got_walls, w) = latest['pose'], latest['walls']
                    if now - min(got_pose, got_walls) > .5:
                        result = 'telemetry lost'; break
                    if now - start > MISSION_SECONDS:
                        result = 'mission time limit'; break
                    up = meters(w['range.up'])
                    if up < .2:
                        result = 'object above the drone'; break
                    zrange = meters(d['range.zrange'])
                    ranges = tuple(meters(w['range.'+beam]) for beam in WALL_BEAMS)
                    x, y, yaw = d['stateEstimate.x'], d['stateEstimate.y'], d['stateEstimate.yaw']
                    vx, vy, rate = finder.step(now - start, x, y, yaw, zrange, ranges)
                    c = finder.center or (math.nan, math.nan)
                    log.writerow({'time': now-start, 'state': finder.state, 'event': finder.event or '',
                                  'x': x, 'y': y, 'z': d['stateEstimate.z'], 'yaw': yaw, 'zrange': zrange,
                                  **dict(zip(WALL_BEAMS, ranges)), 'up': up,
                                  'deviation': finder.detector.deviation, 'on_box': int(finder.detector.on_box),
                                  'cmd_vx': vx, 'cmd_vy': vy, 'center_x': c[0], 'center_y': c[1]})
                    while len(finder.notes) > printed_notes:
                        print(finder.notes[printed_notes]); printed_notes += 1
                    if finder.fault:
                        result = finder.fault; break
                    if finder.event == 'enter' and finder.state == 'SEARCH':
                        print(f'Box found at ({x:.2f}, {y:.2f})')
                    if finder.done:
                        result = f'landed on box at ({c[0]:.2f}, {c[1]:.2f})'
                        motion.land(velocity=LAND_SPEED)
                        break
                    motion.start_linear_motion(vx, vy, 0., rate)
                    time.sleep(max(0., .02-(time.monotonic()-now)))
                for u, before, after in finder.history:
                    print(f'pass along ({u[0]:+.2f}, {u[1]:+.2f}): center {before:.3f} -> {after:.3f} m')
                print('Result:', result)
        except KeyboardInterrupt:
            result = 'interrupted'
            print('Interrupted; landing')
        finally:
            for config in configs:
                config.stop()
            log.writerow({'state': 'END', 'event': result})
    return result


# --------------------------------------------------------------------- replay
def replay(path, box_height=BOX_HEIGHT, plot=True):
    """Run the edge detector over a log with a 'zrange' column; returns [(t, event)].

    The reference level is the median zrange over the first 2 s at flight height."""
    with open(path) as f:
        rows = list(csv.DictReader(f))
    time_key = next(k for k in rows[0] if k.startswith('time'))
    t0 = float(rows[0][time_key])
    scale = 1000. if '(ms)' in time_key or '(us)' in time_key else 1.   # flight_log.csv stamps are ms
    data = [((float(r[time_key])-t0)/scale, float(r['zrange'])) for r in rows if r['zrange']]
    # Skip takeoff: start once zrange is within 2 cm of HEIGHT.
    first = next(i for i, (_, zi) in enumerate(data) if abs(zi-HEIGHT) < .02)
    window = 4. if first else CALIBRATE
    hover = sorted(zi for ti, zi in data[first:] if ti < data[first][0] + window)
    height = hover[len(hover)//2]
    detector, t, z, events = EdgeDetector(height, box_height), [], [], []
    for ti, zi in data[first:]:
        event = detector.update(ti, zi)
        t.append(ti); z.append(zi)
        if event:
            events.append((ti, event))
    print(f'Hover reference {height:.3f} m; box height {box_height:.3f} m: '
          f'enter below {height-detector.enter_level:.3f}, leave above {height+detector.leave_level:.3f}')
    if plot:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(14, 4))
        ax.plot(t, z, lw=.8, label='zrange')
        for ti, e in events:
            ax.axvline(ti, color='tab:green' if e == 'enter' else 'tab:red', lw=1)
        ax.axhline(height-detector.enter_level, color='tab:green', ls=':', lw=.8)
        ax.axhline(height+detector.leave_level, color='tab:red', ls=':', lw=.8)
        ax.set(xlabel='time (s)', ylabel='zrange (m)', title=f'{path}: green = onto box, red = off box')
        ax.grid()
        out = path.rsplit('.', 1)[0] + '_edges.png'
        fig.savefig(out, dpi=80, bbox_inches='tight')
        print('Plot:', out)
    return events


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('uri', nargs='?', default=DEFAULT_URI)
    parser.add_argument('--search', nargs=2, type=float, default=(1., 0.), metavar=('X', 'Y'),
                        help='search direction in the takeoff frame (default +X)')
    parser.add_argument('--distance', type=float, default=SEARCH_DISTANCE, help='search length, m')
    parser.add_argument('--box-height', type=float, default=BOX_HEIGHT, help='box height, m')
    parser.add_argument('--box-size', type=float, default=BOX_SIZE, help='largest box side, m')
    parser.add_argument('--replay', metavar='CSV', help='only mark box edges in a log')
    parser.add_argument('--output', default=datetime.now().strftime('box_log_%Y%m%d_%H%M%S.csv'))
    args = parser.parse_args()
    if args.replay:
        for ti, e in replay(args.replay, args.box_height):
            print(f'{ti:7.2f} s  {e}')
    else:
        finder = BoxFinder(args.search, args.distance, args.box_height, args.box_size)
        print('Result:', fly(args.uri, args.output, finder), '| log:', args.output)
