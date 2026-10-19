"""Two-box mission: from box A fly to B, find and land on the box there, then come back
and land on the box at A.

    python3 mission.py --start 0 0 --goal 3.0 0.5 --sweep -1.0 1.0
    python3 mission.py --box-size 0.45 --box-height 0.16 --x-window 0.3 --pause 5

Coordinates are map meters; START (point A) is where the drone sits at takeoff, on top
of box 1, nose along map +X. Each leg:

1. APPROACH: the obstacle-avoiding navigator (planner.py) flies at the target and a
   little past it, at NAV_SPEED outside the box area and at the pass speed inside it,
   where the box detector watches zrange. On an edge it finds the center from there
   (one pass each way along the box axis it was flying, then across). Otherwise:
2. NAV: fly to the first sweep line with the same navigator.
3. SWEEP: fly parallel lines along Y near the target x. --sweep is the Y range of the box
   center around the target's y (e.g. -0.5 0.5: goal_y - 0.5 .. goal_y + 0.5); each line
   runs FLOOR_MARGIN + half a box past it on both sides, so it starts over bare floor.
   The box center may be up to --x-window from the target x; the lines are close enough
   that one of them crosses the box well inside its edges. Each line is a box_finder
   search; on the first dip it finds the center (one pass each way along Y and X).
4. LAND on the center. After --pause seconds take off again for the next leg.

Cluttered rooms: all hops and sweeps share one obstacle memory, line starts too close to
something seen are moved along the line, a start that can't be reached is replaced by
one further along (or the line is skipped), and a line where the box was seen but its
center wasn't found is flown again the other way. Only safety stops end a leg early.

Leg 1 sweeps at the goal's x. Leg 2 first sweeps a short Y range around the remembered
box 1 (START, after drift: ~15-30 cm off), then the whole --sweep range at START's x.

Takeoff resets the position estimate on the ground, so each leg anchors its odometry to
the map pose where the drone last landed. The nose stays at +X the whole time.
"""
import argparse
import collections
import csv
from datetime import datetime
import math
import threading
import time
import numpy as np
import box_finder as bf
from box_finder import BoxFinder, Failed
from planner import Navigator, guard, wrap

START = (0.0, 0.0)          # Point A: map position of box 1, where the drone takes off, m.
#GOAL = (3.60, 1.24)           # Point B: the box there is near this x, m.
GOAL = (3.30, 0.40)           # Point B: the box there is near this x, m.
SWEEP = (-0.2, 0.2)         # Y range of the box center at each end, relative to that point's y, m.
X_WINDOW = 0.10             # m; the box center is within this of the target x.
LINE_MARGIN = 0.05          # m; a line must pass this far inside the box edge to see it well.
FLOOR_MARGIN = 0.30         # m of floor flown before the nearest possible box edge, so the
                            # search measures the floor first (111021 started on the box edge).
LOCAL_UNCERTAINTY = 0.30    # m; how far box 1 may be from where we remember it, in X and Y
                            # (mission 114850: one line at the remembered x only grazed it).
DRIFT_MARGIN = 0.05         # m added to how far a box center may be from its sweep line.
PAUSE = 5.                  # s on box 2 before taking off again.
NAV_MIN = 0.50              # m; shorter hops fly straight instead of using the navigator.
NAV_SPEED = 0.30            # m/s; navigator top speed outside the box area. Inside it the
                            # navigator flies at the pass speed and watches for the box.
REGION_MARGIN = 0.45        # m of floor around every possible box position where the box is
                            # watched for: ~1.1 s at 0.2-0.3 m/s to learn the floor level first.
TAKEOFF_EXCLUDE = 0.40      # m past the box we took off from where nothing is watched for.
APPROACH_OVERSHOOT = 0.20   # m; the approach aims this far past the far edge of a box at the
                            # target, so it crosses the box at a steady speed.
APPROACH_MIN = 0.30         # m; closer than this the approach is skipped.
WATCH_MIN_SPEED = 0.8       # x pass speed; slower (braking, scanning) restarts the watch...
WATCH_GRACE = 0.3           # ...after this many s (the speed estimate from odometry is noisy).
STRAIGHT_SECONDS = 1.0      # s; if the navigator flew this long before the edge within
STRAIGHT_OFFSET = 0.03      # this many m of a line along a box axis, that was the out pass.
HOP_SPEED = 0.25            # m/s
HOP_SECONDS = 15.
HOP_STUCK = 3.              # s without getting closer: the range guard is holding the drone back.
HOP_ACCEPT = (0.05, 0.15)   # m off the line start (x, y) still good enough to start the line there.
START_CLEARANCE = 0.30      # m from remembered obstacles for a line start (the guard pushes back inside 0.25).
START_STEP = 0.10           # m; move a blocked line start this far along the line and try again.
START_TRIES = 3             # line starts tried before skipping the line.
MIN_LINE = 0.30             # m; shorter lines aren't worth flying.
LINE_ATTEMPTS = 2           # box-finding attempts per line (the second goes the other way).
HEIGHT_TOLERANCE = 0.15     # While navigating, land if zrange is off HEIGHT by more than this
HEIGHT_SECONDS = 1.5        # ...for this long. Box edges cause ~0.6 s spikes.
UP_STOP = 0.20              # m; land if the up ranger reads less than this...
UP_SECONDS = 0.15           # ...for this long (one bad reading isn't enough).
ZRANGE_FROZEN = 1.0         # s; land if the down ranger repeats the same value this long.
                            # Normal: <=0.3 s. One flight logged 0.501 m for 52 s.
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


def line_extent(y_range, box_size=bf.BOX_SIZE):
    """Y span of the sweep lines for a box whose center is somewhere in y_range."""
    reach = box_size/2 + FLOOR_MARGIN
    return min(y_range) - reach, max(y_range) + reach


class Unreachable(Failed):
    """A travel target that the navigator or the range guard won't let the drone reach."""


class Leg:
    """Pure logic for one flight: from wherever the drone is, find the box near x_target
    with its center in the Y range and stop over that center. Feed step() map-frame poses."""

    def __init__(self, x_target, y_range, box_size=bf.BOX_SIZE, box_height=bf.BOX_HEIGHT,
                 window=X_WINDOW, remembered=None, arena=None, approach=True):
        """approach: first fly straight at the target (and past it), watching for the box
        on the way; the sweep only runs if that didn't land."""
        self.box_size, self.box_height, self.arena = box_size, box_height, arena
        self.approach_target = None
        if approach:
            self.approach_target = remembered if remembered is not None else \
                (x_target, (min(y_range) + max(y_range))/2)
        self.plans = []
        local = None
        if remembered is not None:
            local = (remembered[1] - LOCAL_UNCERTAINTY, remembered[1] + LOCAL_UNCERTAINTY)
            self.plans.append(('local', remembered[0], local, LOCAL_UNCERTAINTY))
        covered = local is not None and abs(x_target - remembered[0]) + window <= LOCAL_UNCERTAINTY + 1e-9 \
            and local[0] <= min(y_range) and max(y_range) <= local[1]
        if not covered:
            self.plans.append(('full', x_target, tuple(y_range), window))
        self.pos, self.yaw, self.now, self.zrange = (0., 0.), 0., 0., math.nan
        self.ranges = (math.nan,)*4
        self.state, self.done, self.fault = 'START', False, None
        self.finder, self.nav, self.center, self.notes = None, None, None, []
        self.event, self.deviation = None, math.nan     # current box detector, for the log
        self.found_by = None                            # 'approach' or 'sweep'
        # One obstacle memory for the whole leg; every navigator and every sweep adds to it.
        self.memory = Navigator((0., 0.), arena)
        self.trips, self.takeoff = 0, None
        self.task = self._run()

    def _in_region(self, p):
        """Inside the area where the box may be (plus floor margin), and not over the box
        the leg took off from."""
        if self.takeoff is not None and math.dist(p, self.takeoff) < self.box_size/2 + TAKEOFF_EXCLUDE:
            return False
        reach = self.box_size/2 + REGION_MARGIN
        return any(abs(p[0] - x) <= window + reach and min(c) - reach <= p[1] <= max(c) + reach
                   for _, x, c, window in self.plans)

    def _run_finder(self, finder):
        """Run a BoxFinder to the end. True when it is over the center, ready to land."""
        self.finder = finder
        while True:
            command = finder.step(self.now, self.pos[0], self.pos[1], self.yaw, self.zrange, self.ranges)
            self.state = 'BOX ' + finder.state
            self.event, self.deviation = finder.event, finder.detector.deviation
            if finder.done:
                self.center, self.state, self.done = finder.center, 'LAND', True
                return True
            if finder.fault:
                self.center = None
                return False
            if finder.center is not None:
                self.center = finder.center
            yield command

    def _approach(self, target):
        """Fly at the target and a little past it. Inside the box area the navigator slows
        to the pass speed and the box detector watches the floor; on an edge, find the
        center from there. True when it is over the center."""
        dx, dy = target[0] - self.pos[0], target[1] - self.pos[1]
        n = math.hypot(dx, dy)
        if n < APPROACH_MIN:
            return False
        over = self.box_size/2 + bf.PAST_EDGE + APPROACH_OVERSHOOT
        goal = (target[0] + dx/n*over, target[1] + dy/n*over)
        self.nav = Navigator(goal, self.arena, scan_every=math.inf, startup_scan=self.trips == 0,
                             max_speed=NAV_SPEED)
        self.nav.points = self.memory.points
        self.trips += 1
        detector = bf.EdgeDetector(bf.HEIGHT, self.box_height)
        watch = bf.SearchDetector(detector, blind_extra=None)   # no hover level to compare with
        history, trace, hit, steady_at = collections.deque(), [], None, -math.inf
        try:
            while True:
                history.append((self.now, self.pos))
                while history[0][0] < self.now - .3:
                    history.popleft()
                (t0, p0), (t1, p1) = history[0], history[-1]
                velocity = ((p1[0] - p0[0])/(t1 - t0), (p1[1] - p0[1])/(t1 - t0)) if t1 > t0 else (0., 0.)
                inside = self._in_region(self.pos)
                self.nav.max_speed = bf.PASS_SPEED if inside else NAV_SPEED
                event = detector.update(self.now, self.zrange)
                if inside and self.nav.state in ('DIRECT', 'DETOUR', 'BLOCKED') \
                        and math.hypot(*velocity) >= WATCH_MIN_SPEED*bf.PASS_SPEED:
                    steady_at = self.now
                steady = inside and self.now - steady_at < WATCH_GRACE
                if steady:
                    if math.isfinite(self.zrange) and (not trace or trace[-1][3] != self.zrange):
                        trace.append((self.now, *self.pos, self.zrange))
                    if watch.check(self.now, self.zrange, event):
                        hit = (self.pos, velocity)
                        break
                    self.event, self.deviation = (None if watch.blind else event), detector.deviation
                else:
                    if watch.began is not None:
                        watch.restart()
                    trace = []
                    self.event, self.deviation = None, math.nan
                command = self.nav.step(self.pos[0], self.pos[1], self.yaw, self.ranges, self.now)
                if self.nav.fault:
                    if self.nav.fault.startswith(('No path', 'No progress')):
                        self.notes.append(f'approach: {self.nav.fault}; sweeping instead')
                        return False
                    raise Failed(f'navigation: {self.nav.fault}')
                if self.nav.done:
                    self.notes.append('approach: no box edge seen on the way; sweeping')
                    return False
                self.state = 'NAV ' + self.nav.state + (' WATCH' if steady else '')
                yield command
        finally:
            self.nav = None
        point, (vx, vy) = hit
        u = (math.copysign(1., vx), 0.) if abs(vx) >= abs(vy) else (0., math.copysign(1., vy))
        # Flown straight along u for the last STRAIGHT_SECONDS: that flight is the out pass.
        recent = [(x, y) for t, x, y, _ in trace if t >= self.now - STRAIGHT_SECONDS]
        straight = bool(trace) and trace[0][0] <= self.now - STRAIGHT_SECONDS and \
            max((abs((x - point[0])*u[1] - (y - point[1])*u[0]) for x, y in recent),
                default=math.inf) <= STRAIGHT_OFFSET
        self.event = 'enter'
        self.notes.append(f'approach: box edge seen at ({point[0]:.2f}, {point[1]:.2f}) flying '
                          f'({u[0]:+.0f}, {u[1]:+.0f}); finding its center from '
                          + ('here' if straight else 'a pass along that axis'))
        finder = bf.BoxFinder(search_direction=u, box_height=self.box_height, box_size=self.box_size,
                              height=detector.height, approach=point,
                              approach_profile=[(x, y, z) for _, x, y, z in trace] if straight else None)
        if (yield from self._run_finder(finder)):
            self.found_by = 'approach'
            return True
        self.notes.append(f'approach: center not found ({finder.fault}); sweeping')
        return False

    def _clearance(self, point):
        points = self.memory.obstacle_points()
        d = float(np.min(np.hypot(*(points - point).T))) if len(points) else math.inf
        if self.arena is not None:
            x0, x1, y0, y1 = self.arena
            d = min(d, point[0] - x0, x1 - point[0], point[1] - y0, y1 - point[1])
        return d

    def _body(self, wx, wy):
        c, s = math.cos(math.radians(self.yaw)), math.sin(math.radians(self.yaw))
        vx, vy = guard(c*wx + s*wy, -s*wx + c*wy, self.ranges)
        return vx, vy, max(-20., min(20., -2.*self.yaw))

    def _travel(self, point):
        if math.dist(self.pos, point) > NAV_MIN:
            # Scan only before the leg's first trip (and whenever blocked): later trips stay
            # in the sweep area, which the memory already holds.
            self.nav = Navigator(point, self.arena, scan_every=math.inf, startup_scan=self.trips == 0)
            self.nav.points = self.memory.points
            self.trips += 1
            try:
                while True:
                    command = self.nav.step(self.pos[0], self.pos[1], self.yaw, self.ranges, self.now)
                    if self.nav.fault:
                        error = Unreachable if self.nav.fault.startswith(('No path', 'No progress')) else Failed
                        raise error(f'navigation: {self.nav.fault}')
                    if self.nav.done:
                        break
                    self.state = 'NAV ' + self.nav.state
                    yield command
            finally:
                self.nav = None
        self.state, start = 'HOP', self.now
        best, best_at = math.inf, self.now
        while math.dist(self.pos, point) > .03:
            d = math.dist(self.pos, point)
            if d < best - .01:
                best, best_at = d, self.now
            if self.now - best_at > HOP_STUCK or self.now - start > HOP_SECONDS:
                if abs(self.pos[0] - point[0]) <= HOP_ACCEPT[0] and abs(self.pos[1] - point[1]) <= HOP_ACCEPT[1]:
                    break
                raise Unreachable(f'could not reach ({point[0]:.2f}, {point[1]:.2f})')
            vx, vy = bf.GAIN*(point[0]-self.pos[0]), bf.GAIN*(point[1]-self.pos[1])
            n = math.hypot(vx, vy)
            if n > HOP_SPEED:
                vx, vy = vx*HOP_SPEED/n, vy*HOP_SPEED/n
            yield self._body(vx, vy)

    def _start_line(self, x_line, y_from, y_to):
        """Fly to the line x = x_line, starting as close to y_from as is clear and reachable.
        Returns the start y, or None when no start leaves a line worth flying."""
        step = START_STEP if y_to > y_from else -START_STEP
        y, tries = y_from, 0
        while abs(y_to - y) >= MIN_LINE and tries < START_TRIES:
            if self._clearance((x_line, y)) < START_CLEARANCE:
                y += step
                continue
            tries += 1
            try:
                yield from self._travel((x_line, y))
                return self.pos[1]
            except Unreachable as error:
                self.notes.append(f'line x={x_line:.2f}: start y={y:.2f} unreachable ({error}), trying further along')
                y += step
        return None

    def _run(self):
        if self.approach_target is not None and (yield from self._approach(self.approach_target)):
            return
        seen = False
        for name, x_target, centers, window in self.plans:
            lo, hi = line_extent(centers, self.box_size)
            lines = sweep_lines(x_target, window, self.box_size)
            # Most likely line first (the middle one), then the nearer of the rest.
            lines.sort(key=lambda x: (round(abs(x - x_target), 3), abs(x - self.pos[0])))
            cross = min(self.box_size/2, window/len(lines) + DRIFT_MARGIN)
            for x_line in lines:
                for attempt in range(LINE_ATTEMPTS):
                    # From the end nearer the drone; after a failed attempt that's usually
                    # the other end, so the retry crosses the box the other way.
                    y_from, y_to = (lo, hi) if abs(self.pos[1] - lo) <= abs(self.pos[1] - hi) else (hi, lo)
                    y0 = yield from self._start_line(x_line, y_from, y_to)
                    if y0 is None:
                        self.notes.append(f'{name} sweep, line x={x_line:.2f}: no clear, reachable start; skipped')
                        break
                    finder = BoxFinder(search_direction=(0., 1. if y_to > y0 else -1.),
                                       search_distance=abs(y_to - y0), box_height=self.box_height,
                                       box_size=self.box_size, cross_offset=cross)
                    if (yield from self._run_finder(finder)):
                        self.found_by = 'sweep'
                        return
                    self.notes.append(f'{name} sweep, line x={x_line:.2f}, attempt {attempt + 1}: {finder.fault}')
                    if 'no box found' in finder.fault:
                        break
                    seen = True
        raise Failed('box seen but its center not found' if seen else 'no box found on any sweep line')

    def step(self, t, x, y, yaw, zrange, ranges=(math.nan,)*4):
        """Return body-frame (vx, vy, yaw_rate) for MotionCommander.start_linear_motion."""
        if self.done or self.fault:
            return 0., 0., 0.
        self.pos, self.yaw, self.now, self.zrange, self.ranges = (x, y), yaw, t, zrange, tuple(ranges)
        self.takeoff = self.takeoff or self.pos
        if self.nav is None:   # a navigator maps into the same memory itself
            self.memory.observe(x, y, yaw, self.ranges, t)
        try:
            return next(self.task)
        except Failed as error:
            self.fault, self.state = str(error), 'FAILED'
        except StopIteration:
            pass
        return 0., 0., 0.


def mission_legs(start=START, goal=GOAL, sweep=SWEEP):
    """[(name, x_target, y_range, remembered box)] for the two legs. `sweep` is relative to
    the target's y."""
    return [('to B', goal[0], (goal[1] + sweep[0], goal[1] + sweep[1]), None),
            ('back to A', start[0], (start[1] + sweep[0], start[1] + sweep[1]), start)]


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

    legs = mission_legs(start, goal, sweep)
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
            for number, (name, x_target, y_range, remembered) in enumerate(legs, 1):
                if number > 1:
                    print(f'On the box; taking off again in {pause:.0f} s')
                    time.sleep(pause)
                leg = Leg(x_target, y_range, box_size, box_height, window, remembered, arena)
                scf.cf.platform.send_arming_request(True)
                time.sleep(1.)
                print(f'Leg {number} ({name}): taking off at map ({anchor.x:.2f}, {anchor.y:.2f}); '
                      f'sweep lines x = {", ".join(f"{x:.2f}" for x in sweep_lines(x_target, window, box_size))}, '
                      f'y {y_range[0]:.2f} .. {y_range[1]:.2f}')
                motion.take_off()
                motion.start_linear_motion(0., 0., 0., 0.)
                time.sleep(1.)
                result, t0 = None, time.monotonic()
                height_ok = last_print = last_flush = t0
                up_low_since, last_zrange, zrange_changed = None, None, t0
                printed, printed_finder, printed_finder_notes = 0, None, 0
                while result is None:
                    now = time.monotonic()
                    with lock:
                        (got_pose, d), (got_walls, w) = latest['pose'], latest['walls']
                    up = bf.meters(w['range.up'])
                    zrange = bf.meters(d['range.zrange'])
                    ranges = tuple(bf.meters(w['range.'+beam]) for beam in WALL_BEAMS)
                    ox, oy, oyaw = d['stateEstimate.x'], d['stateEstimate.y'], d['stateEstimate.yaw']
                    x, y, yaw = anchor.pose(ox, oy, oyaw)
                    # A single spurious up reading stopped two flights, so it must persist.
                    up_low_since = (up_low_since or now) if up < UP_STOP else None
                    # Not while watching for the box: crossing it at the pass speed is a long dip.
                    if not leg.state.startswith('NAV') or leg.state.endswith('WATCH') or \
                            (math.isfinite(zrange) and abs(zrange - bf.HEIGHT) < HEIGHT_TOLERANCE):
                        height_ok = now
                    if d['range.zrange'] != last_zrange:
                        last_zrange, zrange_changed = d['range.zrange'], now
                    if now - min(got_pose, got_walls) > .5:
                        result = 'telemetry lost'
                    elif now - zrange_changed > ZRANGE_FROZEN:
                        result = f'down ranger frozen at {zrange:.3f} m'
                    elif now - t0 > LEG_SECONDS:
                        result = 'leg time limit'
                    elif up_low_since is not None and now - up_low_since >= UP_SECONDS:
                        result = f'object above the drone ({up:.2f} m)'
                    elif now - height_ok > HEIGHT_SECONDS:
                        result = f'height lost while navigating (z-range {zrange:.2f} m)'
                    command = (0., 0., 0.) if result else leg.step(now - t0, x, y, yaw, zrange, ranges)
                    finder = leg.finder
                    c = leg.center or (math.nan, math.nan)
                    log.writerow({'time': now - t0, 'leg': number, 'state': leg.state,
                                  'event': leg.event or '',
                                  'x': x, 'y': y, 'yaw': yaw, 'odom_x': ox, 'odom_y': oy,
                                  'z': d['stateEstimate.z'], 'zrange': zrange,
                                  **dict(zip(WALL_BEAMS, ranges)), 'up': up,
                                  'deviation': leg.deviation,
                                  **dict(zip(('cmd_vx', 'cmd_vy', 'cmd_yaw_rate'), command)),
                                  'center_x': c[0], 'center_y': c[1]})
                    if now - last_flush > .25:
                        file.flush(); last_flush = now
                    if result:
                        break
                    if finder is not printed_finder:
                        printed_finder, printed_finder_notes = finder, 0
                    while finder and len(finder.notes) > printed_finder_notes:
                        print(finder.notes[printed_finder_notes]); printed_finder_notes += 1
                    while len(leg.notes) > printed:
                        print(leg.notes[printed]); printed += 1
                    if leg.event == 'enter' and leg.state == 'BOX SEARCH':
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
    parser.add_argument('--sweep', nargs=2, type=float, metavar=('DY_MIN', 'DY_MAX'), default=SWEEP,
                        help="Y range of the box center at each end, relative to that point's y "
                             "(lines run half a box + FLOOR_MARGIN past it)")
    parser.add_argument('--x-window', type=float, default=X_WINDOW,
                        help='m; box center is within this of the target x')
    parser.add_argument('--box-size', type=float, default=bf.BOX_SIZE, help='side of the square box, m')
    parser.add_argument('--box-height', type=float, default=bf.BOX_HEIGHT, help='m')
    parser.add_argument('--pause', type=float, default=PAUSE, help='s on box 2 before taking off again')
    parser.add_argument('--arena', nargs=4, type=float, metavar=('X_MIN', 'X_MAX', 'Y_MIN', 'Y_MAX'),
                        help='virtual fence for the navigator, map m (default: none)')
    parser.add_argument('--output', default=datetime.now().strftime('mission_log_%Y%m%d_%H%M%S.csv'))
    parser.add_argument('--no-live', action='store_true', help='no live plot window')
    args = parser.parse_args()
    import mission_plot
    meta = {'start': tuple(args.start), 'goal': tuple(args.goal), 'sweep': tuple(args.sweep),
            'window': args.x_window, 'box_size': args.box_size}
    viewer = None if args.no_live else mission_plot.start_live(args.output, meta)
    outcome = []
    try:
        outcome = fly(args.uri, args.output, tuple(args.start), tuple(args.goal), tuple(args.sweep),
                      args.x_window, args.box_size, args.box_height, args.pause,
                      tuple(args.arena) if args.arena else None)
    finally:
        from pathlib import Path
        if Path(args.output).exists():
            print('Plot:', mission_plot.save(args.output, meta))
    print('Result:', ' | '.join(outcome), '| log:', args.output)
    if viewer is not None and viewer.is_alive():
        print('Close the live plot window to exit.')
    raise SystemExit(0 if len(outcome) == 2 and all(r.startswith('landed') for r in outcome) else 1)
