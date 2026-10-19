"""Point-A-to-point-B navigation for a Crazyflie with Flow + Multi-ranger decks.

The drone never turns and translates at the same time; in flight that smeared the
ranger map by 20-50 cm and bent the actual direction of travel by up to 40 degrees.
  SCAN   Hover and turn slowly to +45, -45 and back to 0 degrees. The four beams are
         90 degrees apart, so this sees in every direction.
  MOVE   Hold the nose at 0 degrees and fly up to SCAN_EVERY meters, then BRAKE (fixed
         time) and SCAN again. Rescan early when the planned direction is blocked.
Every step:
  - Ranger hits become obstacle points in the map frame (short-term memory).
  - A* plans on a 5 cm grid from those points (unknown space counts as free). It keeps
    MIN_CLEARANCE as a hard limit and prefers CLEARANCE and more, so it fits ~60 cm doors
    and flies down their middle. The current path is kept until it is blocked or a
    clearly cheaper one appears. The drone flies straight at the goal when the whole
    line is clear with CLEARANCE, otherwise toward a point 40 cm along the path.
  - The chosen heading must have a corridor free of remembered points.
  - A body-frame guard limits velocity toward any beam that reads closer than CLEARANCE
    and backs away inside it. It uses raw ranges, so it works even if odometry is wrong.
  - Position jumps (collision or estimator failure) end the flight.
"""
import heapq
import math
import numpy as np
from scipy.ndimage import distance_transform_edt

DRONE_RADIUS = 0.10
CLEARANCE = 0.25          # Preferred distance from drone center to an obstacle (open space, guard).
MIN_CLEARANCE = 0.18      # Hard minimum, used only where needed (doors): ~60 cm openings fit.
FAR_CLEARANCE = 0.12      # Hard minimum for planning around points farther than NEAR_POINTS:
NEAR_POINTS = 0.80        # the ranger cone smears far edges ~10 cm into openings.
MAX_SPEED = 0.20          # m/s
LOOKAHEAD = 1.0           # Corridor length checked per heading, m.
MIN_FREE = 0.10           # A heading needs at least this much free corridor, m.
PREFERRED_FREE = 0.30     # Headings with this much free corridor are preferred.
GOAL_TOLERANCE = 0.08     # m
SCAN_EVERY = 1.00         # Rescan after flying this far, m.
SCAN_ANGLES = (45., -45., 0.)
SCAN_RATE = 60.           # deg/s, slow enough that ranger lag barely smears the map.
BRAKE_SECONDS = 0.8       # Zero velocity before each scan.
RANGE_AGE = 0.10          # s; rangers lag the pose by about this much.
GUARD_GAIN = 1.0          # 1/s; allowed speed toward a beam = gain*(range - CLEARANCE).
MAX_PUSH = 0.15           # Fastest back-away speed of the guard, m/s.
MAX_POSE_JUMP = 0.10      # m between 20 Hz samples; more means the estimate broke.
SENSOR_ANGLES = (0., 180., 90., -90.)  # front, back, left, right (body frame, degrees)
MEMORY_RANGE = 1.5        # Hits farther than this are not stored.
MEMORY_SECONDS = 90.      # Forget points after this. Shorter makes long flights re-explore dead ends.
CELL = 0.03               # One remembered point per 3 cm cell.
HEADINGS = np.radians(np.arange(-180., 180., 10.))
GRID = 0.05               # Planning grid cell size, m.
REPLAN_SECONDS = 0.25
PATH_LOOKAHEAD = 0.40     # Steer toward the path point this far ahead, m.
SWITCH_RATIO = 1.25       # Keep the current path unless a new one is this much cheaper.
CONE = math.radians(2.)   # Half-angle treated as seen-through in front of a reading.
NEIGHBORS = [(dx, dy, math.hypot(dx, dy)) for dx in (-1, 0, 1) for dy in (-1, 0, 1) if dx or dy]


def wrap(degrees):
    return (degrees + 180.) % 360. - 180.


def plan_path(start, goal, points, bounds, previous=None):
    """A* on a GRID-sized grid; unknown space counts as free.

    `previous` (the path being flown) is kept while it stays unblocked and costs less
    than SWITCH_RATIO times the new path: replanning from scratch every 0.25 s flips
    between near-equal routes and the drone dithers in place.

    Cells closer than MIN_CLEARANCE to an obstacle point or the bounds are blocked, and
    cells closer than CLEARANCE + 25 cm cost more, so paths keep a wide margin where
    there is room and run down the middle of narrow openings. Points farther than
    NEAR_POINTS from the start only block within FAR_CLEARANCE: far door edges are
    measured too wide, and they are re-measured on approach. A drone that drifted
    into a blocked cell may still leave it toward more clearance. Returns world
    waypoints from start to goal, or None when the goal is unreachable.
    """
    x0, x1, y0, y1 = bounds
    nx, ny = math.ceil((x1-x0)/GRID), math.ceil((y1-y0)/GRID)
    points = np.asarray(points, dtype=float).reshape(-1, 2)

    def distance_to(subset):
        occupied = np.zeros((nx, ny), bool)
        cells = np.floor((subset - (x0, y0))/GRID).astype(int)
        inside = (cells[:, 0] >= 0) & (cells[:, 0] < nx) & (cells[:, 1] >= 0) & (cells[:, 1] < ny)
        occupied[cells[inside, 0], cells[inside, 1]] = True
        return distance_transform_edt(~occupied)*GRID if occupied.any() else np.full((nx, ny), np.inf)
    near = np.hypot(*(points - start).T) < NEAR_POINTS
    near_clearance, far_clearance = distance_to(points[near]), distance_to(points[~near])
    cx, cy = x0 + (np.arange(nx)+.5)*GRID, y0 + (np.arange(ny)+.5)*GRID
    edge = np.minimum.outer(np.minimum(cx-x0, x1-cx), np.minimum(cy-y0, y1-cy))
    clearance = np.minimum(np.minimum(near_clearance, far_clearance), edge)
    blocked = (np.minimum(near_clearance, edge) < MIN_CLEARANCE) | (far_clearance < FAR_CLEARANCE)
    penalty = 1. + 3.*np.clip((CLEARANCE + .25 - clearance)/.25, 0., 1.) \
        + 5.*np.clip((CLEARANCE - clearance)/(CLEARANCE - MIN_CLEARANCE), 0., 1.)

    def cell(p):
        return (min(nx-1, max(0, int((p[0]-x0)/GRID))), min(ny-1, max(0, int((p[1]-y0)/GRID))))
    source, target = cell(start), cell(goal)
    if blocked[target]:
        return None
    cost, parent = {source: 0.}, {source: None}
    heap = [(0., 0., source)]
    while heap:
        _, g, current = heapq.heappop(heap)
        if current == target:
            break
        if g > cost[current]:
            continue
        for dx, dy, step in NEIGHBORS:
            n = (current[0]+dx, current[1]+dy)
            if not (0 <= n[0] < nx and 0 <= n[1] < ny):
                continue
            if blocked[n] and not (blocked[current] and clearance[n] > clearance[current]):
                continue
            new = g + step*penalty[n]
            if new < cost.get(n, math.inf):
                cost[n], parent[n] = new, current
                heapq.heappush(heap, (new + math.hypot(n[0]-target[0], n[1]-target[1]), new, n))
    if target not in parent:
        return None
    path, c = [], target
    while c is not None:
        path.append((x0 + (c[0]+.5)*GRID, y0 + (c[1]+.5)*GRID))
        c = parent[c]
    path.reverse()
    path = [tuple(start)] + path[1:-1] + [tuple(goal)]
    if previous:
        # Rejoin the old path at its closest point and price it on the new map.
        i = min(range(len(previous)), key=lambda k: math.dist(start, previous[k]))
        old = [tuple(start)] + list(previous[i+1:])
        old_cost = 0.
        for a, b in zip(old, old[1:]):
            n = max(1, math.ceil(math.dist(a, b)/(GRID/2)))
            for k in range(1, n+1):
                c = cell((a[0]+(b[0]-a[0])*k/n, a[1]+(b[1]-a[1])*k/n))
                if blocked[c] and math.dist(start, a) > 2*GRID:
                    return path
                old_cost += math.dist(a, b)/n/GRID*penalty[c]
        if old_cost < SWITCH_RATIO*cost[target]:
            return old
    return path


def guard(vx, vy, ranges):
    """Limit body velocity using the raw ranges: slow toward close beams, back away inside CLEARANCE."""
    front, back, left, right = ranges
    limit = lambda r: math.inf if not math.isfinite(r) else max(-MAX_PUSH, GUARD_GAIN*(r - CLEARANCE))
    hi_x, lo_x = limit(front), -limit(back)
    hi_y, lo_y = limit(left), -limit(right)
    vx = (hi_x+lo_x)/2 if lo_x > hi_x else min(hi_x, max(lo_x, vx))
    vy = (hi_y+lo_y)/2 if lo_y > hi_y else min(hi_y, max(lo_y, vy))
    return vx, vy


class Navigator:
    def __init__(self, goal, arena=None, scan_every=SCAN_EVERY, startup_scan=True, max_speed=MAX_SPEED):
        """goal: (x, y) in meters. arena: optional (x_min, x_max, y_min, y_max) virtual fence.
        scan_every: rescan after flying this far (math.inf: only when blocked).
        startup_scan: scan before moving; skip it when the obstacle memory is already filled.
        max_speed: m/s; may be changed between steps."""
        self.goal = (float(goal[0]), float(goal[1]))
        self.scan_every, self.max_speed = scan_every, max_speed
        self.arena = self.bounds = arena
        self.points = {}               # cell -> (x, y, time)
        self.last_ranges = (math.nan,)*4
        self.last_pose = None          # (x, y, yaw, time)
        self.heading = None            # Chosen travel heading, radians.
        self.path, self.planned_at = None, -math.inf
        self.no_path_since = self.stuck_since = self.arrived_since = None
        self.state, self.reason, self.fault = ('BRAKE' if startup_scan else 'MOVE'), 'startup', None
        self.phase_start, self.scan_targets, self.scan_xy = None, [], None
        self.done = False
        self.free = float('nan')
        self.best_distance, self.progress_time = math.inf, None

    # ---------------------------------------------------------------- mapping
    def observe(self, x, y, yaw, ranges, now):
        """ranges: (front, back, left, right) in meters, NaN when there is no valid return.

        Only beams whose value changed are used: the rangers refresh slower than the log,
        and a held old value paired with a newer pose would be misplaced.
        """
        fresh = [math.isfinite(r) and r != old for r, old in zip(ranges, self.last_ranges)]
        self.last_ranges = tuple(ranges)
        if self.points:
            keys = list(self.points)
            xy = np.array([self.points[k][:2] for k in keys])
            stamps = np.array([self.points[k][2] for k in keys])
            remove = now - stamps > MEMORY_SECONDS
            for r, offset, new in zip(ranges, SENSOR_ANGLES, fresh):
                if not new or r < .05:
                    continue
                a = math.radians(yaw + offset)
                ux, uy = math.cos(a), math.sin(a)
                rel = xy - (x, y)
                along = rel[:, 0]*ux + rel[:, 1]*uy
                perp = np.abs(rel[:, 1]*ux - rel[:, 0]*uy)
                # The ranger reports the nearest target in its cone, so nothing is in
                # front of the reading. The cone used here is much narrower than the
                # real one, so a beam grazing past a thin pole does not erase it, but it
                # erases far door edges that the cone smeared into the opening.
                remove |= (along > .05) & (along < min(r, MEMORY_RANGE) - .08) \
                    & (perp < .015 + along*math.tan(CONE))
            for key in np.asarray(keys, dtype=object)[remove]:
                del self.points[tuple(key)]
        for r, offset, new in zip(ranges, SENSOR_ANGLES, fresh):
            if new and .05 <= r <= MEMORY_RANGE:
                a = math.radians(yaw + offset)
                px, py = x + r*math.cos(a), y + r*math.sin(a)
                self.points[(math.floor(px/CELL), math.floor(py/CELL))] = (px, py, now)

    def obstacle_points(self):
        return np.array([v[:2] for v in self.points.values()]).reshape(-1, 2)

    # ------------------------------------------------------------- planning
    def free_distance(self, x, y, headings, clearance=CLEARANCE, length=LOOKAHEAD):
        """How far (up to `length`) the drone center can travel along each heading keeping `clearance`."""
        headings = np.asarray(headings, dtype=float)
        free = np.full(headings.shape, length)
        ux, uy = np.cos(headings), np.sin(headings)
        pts = self.obstacle_points()
        if len(pts):
            rel = pts - (x, y)
            along = np.outer(ux, rel[:, 0]) + np.outer(uy, rel[:, 1])
            perp = np.abs(np.outer(ux, rel[:, 1]) - np.outer(uy, rel[:, 0]))
            hit = (perp < clearance) & (along >= 0.)
            entry = along - np.sqrt(np.maximum(0., clearance**2 - perp**2))
            free = np.minimum(free, np.where(hit, entry, np.inf).min(axis=1))
        if self.arena is not None:
            x0, x1, y0, y1 = self.arena
            m = clearance
            with np.errstate(divide='ignore', invalid='ignore'):
                for u, p, lo, hi in ((ux, x, x0+m, x1-m), (uy, y, y0+m, y1-m)):
                    limit = np.where(u > 1e-9, (hi-p)/u, np.where(u < -1e-9, (lo-p)/u, np.inf))
                    free = np.minimum(free, np.maximum(limit, 0.))
        return free

    def path_heading(self, x, y):
        """Heading toward the path point PATH_LOOKAHEAD ahead of the closest path point."""
        path = self.path
        i = min(range(len(path)), key=lambda k: math.dist((x, y), path[k]))
        target, remaining = path[i], PATH_LOOKAHEAD
        for a, b in zip(path[i:], path[i+1:]):
            step = math.dist(a, b)
            target = b
            if step >= remaining:
                target = (a[0]+(b[0]-a[0])*remaining/step, a[1]+(b[1]-a[1])*remaining/step)
                break
            remaining -= step
        return math.atan2(target[1]-y, target[0]-x)

    # --------------------------------------------------------------- control
    def _brake(self, now, reason):
        self.state, self.reason, self.phase_start = 'BRAKE', reason, now

    def _hold_yaw(self, yaw, check=True):
        if check and abs(yaw) > 30.:
            self.fault = f'Yaw drifted to {yaw:.0f} deg while holding 0 (yaw rate sign or estimator problem)'
        return max(-20., min(20., -2.*yaw))

    def _output(self, vx, vy, yaw_rate, ranges):
        if self.fault:
            return 0., 0., 0.
        vx, vy = guard(vx, vy, ranges)
        return vx, vy, yaw_rate

    def step(self, x, y, yaw, ranges, now):
        """Return body-frame (vx, vy, yaw_rate) for MotionCommander.start_linear_motion.

        x, y in meters and yaw in degrees, all in the map frame (yaw 0 = start heading).
        """
        if self.fault or self.done:
            return 0., 0., 0.
        yaw_rate_now = 0.
        if self.last_pose is not None:
            px, py, pyaw, pt = self.last_pose
            if math.dist((x, y), (px, py)) > MAX_POSE_JUMP:
                self.fault = 'Position estimate jumped (collision or Flow-deck failure)'
                return 0., 0., 0.
            if now > pt:
                yaw_rate_now = wrap(yaw - pyaw)/(now - pt)
        self.last_pose = (x, y, yaw, now)
        self.observe(x, y, yaw - yaw_rate_now*RANGE_AGE, ranges, now)
        if self.phase_start is None:
            self.phase_start = now

        dx, dy = self.goal[0]-x, self.goal[1]-y
        distance = math.hypot(dx, dy)
        if distance < self.best_distance - .05 or self.progress_time is None:
            self.best_distance, self.progress_time = distance, now
        if now - self.progress_time > 60.:
            self.fault = 'No progress toward goal for 60 s'
            return 0., 0., 0.

        # Arrival: settle inside the tolerance for one second, then report done.
        if distance < GOAL_TOLERANCE or (self.arrived_since is not None and distance < 2*GOAL_TOLERANCE):
            self.arrived_since = now if self.arrived_since is None else self.arrived_since
            self.state, self.reason = 'ARRIVING', 'holding at goal'
            if now - self.arrived_since >= 1.:
                self.state, self.done = 'ARRIVED', True
            # Arrival can interrupt a scan, so turn back to 0 without the yaw-drift check.
            return self._output(0., 0., self._hold_yaw(yaw, check=False), ranges)
        self.arrived_since = None

        if self.state == 'BRAKE':
            if now - self.phase_start >= BRAKE_SECONDS:
                self.state, self.phase_start, self.scan_targets = 'SCAN', now, list(SCAN_ANGLES)
            return self._output(0., 0., 0., ranges)

        if self.state == 'SCAN':
            error = wrap(self.scan_targets[0] - yaw)
            if abs(error) < 4.:
                self.scan_targets.pop(0)
                self.phase_start = now
                if not self.scan_targets:
                    self.state, self.scan_xy, self.planned_at = 'MOVE', (x, y), -math.inf
                    return self._output(0., 0., 0., ranges)
                error = wrap(self.scan_targets[0] - yaw)
            if abs(yaw) > 90. or now - self.phase_start > 6.:
                self.fault = 'Yaw does not follow scan commands (yaw rate sign or estimator problem)'
                return 0., 0., 0.
            self.reason = f'turning to {self.scan_targets[0]:.0f} deg'
            return self._output(0., 0., max(-SCAN_RATE, min(SCAN_RATE, 4.*error)), ranges)

        # MOVE: hold yaw at 0 and translate.
        yaw_rate = self._hold_yaw(yaw)
        if self.fault:
            return 0., 0., 0.
        if self.bounds is None:
            m = 1.5
            self.bounds = (min(x, self.goal[0])-m, max(x, self.goal[0])+m,
                           min(y, self.goal[1])-m, max(y, self.goal[1])+m)
        if now - self.planned_at >= REPLAN_SECONDS:
            self.path = plan_path((x, y), self.goal, self.obstacle_points(), self.bounds, self.path)
            self.planned_at = now
            self.no_path_since = None if self.path else (self.no_path_since or now)
        if self.no_path_since is not None and now - self.no_path_since > 10.:
            self.fault = 'No path to the goal for 10 s (blocked, or goal too close to an obstacle)'
            return 0., 0., 0.
        goal_heading = math.atan2(dy, dx)
        # Only fly straight when the whole line to the goal is clear; checking just the next
        # meter steers into dead ends that the planner then steers out of, every tick.
        direct = self.free_distance(x, y, [goal_heading], length=distance)[0] >= distance
        desired = goal_heading if direct or not self.path else self.path_heading(x, y)

        candidates = np.concatenate(([desired], HEADINGS))
        # The path already keeps CLEARANCE wherever there is room; in a door it runs down
        # the middle, so following it only needs MIN_CLEARANCE.
        free = self.free_distance(x, y, candidates, CLEARANCE if direct else MIN_CLEARANCE)
        # Prefer turning early over creeping up to an obstacle; fall back to tight gaps.
        roomy = free >= min(PREFERRED_FREE, distance)
        valid = roomy if roomy.any() else free >= MIN_FREE
        # The path is the plan; the corridor check is the safety layer on top of it.
        deviation = np.abs(np.angle(np.exp(1j*(candidates - desired))))
        deviation[~valid] = np.inf
        best = int(np.argmin(deviation))
        self.scan_xy = self.scan_xy or (x, y)
        travelled = math.dist((x, y), self.scan_xy)

        if not math.isfinite(deviation[best]):
            if travelled > .2:
                self._brake(now, 'no free heading; look around')
                return self._output(0., 0., 0., ranges)
            self.stuck_since = now if self.stuck_since is None else self.stuck_since
            self.state, self.reason, self.free = 'STUCK', 'no free heading', 0.
            if now - self.stuck_since > 10.:
                self.fault = 'No free heading for 10 s'
                return 0., 0., 0.
            return self._output(0., 0., yaw_rate, ranges)
        self.stuck_since = None
        if travelled >= self.scan_every or (best and travelled > .2):
            self._brake(now, 'periodic scan' if travelled >= self.scan_every else 'planned direction blocked')
            return self._output(0., 0., 0., ranges)
        self.heading, self.free = float(candidates[best]), float(free[best])
        self.state = ('BLOCKED' if best else 'DIRECT' if direct else
                      'DETOUR' if self.path else 'NO_PATH')
        self.reason = f'heading {math.degrees(self.heading):.0f} deg, {self.free:.2f} m free'

        speed = min(self.max_speed, .5*(self.free - .05), .8*distance + .02)
        wx, wy = speed*math.cos(self.heading), speed*math.sin(self.heading)
        c, s = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
        return self._output(c*wx + s*wy, -s*wx + c*wy, yaw_rate, ranges)
