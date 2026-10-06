"""Stable global route + continuous, drift-tolerant velocity following."""
import heapq
import math
import threading
import time
from slam import CLEARANCE, wrap


def projection(p, a, b):
    """Progress along an edge, and sideways distance from its supporting line."""
    dx, dy = b[0]-a[0], b[1]-a[1]
    length = math.hypot(dx, dy)
    if length < 1e-9:
        return 1., math.dist(p, b)
    t = ((p[0]-a[0])*dx+(p[1]-a[1])*dy)/length**2
    return t, abs((p[0]-a[0])*dy-(p[1]-a[1])*dx)/length


def plan_route(start, goal, grid):
    """A* over the inflated occupancy grid; unknown cells remain traversable."""
    blocked = grid.costmap()
    def valid(cell):
        c, r = cell
        return 0 <= c < grid.cols and 0 <= r < grid.rows and not blocked[c, r]
    def clear(a, b):
        return grid.segment_clear(a, b, blocked)
    source, target = grid.world_to_grid(*start), grid.world_to_grid(*goal)
    if not valid(source) or not valid(target):
        return []
    if clear(start, goal):
        points = [start, goal]
    else:
        heap, costs, parents = [(math.dist(source, target), 0., source)], {source: 0.}, {}
        neighbors = [(x, y) for x in (-1, 0, 1) for y in (-1, 0, 1) if x or y]
        while heap:
            _, cost, cell = heapq.heappop(heap)
            if cost > costs[cell]:
                continue
            if cell == target:
                break
            for dx, dy in neighbors:
                nxt = (cell[0]+dx, cell[1]+dy)
                if not valid(nxt):
                    continue
                if dx and dy and (not valid((cell[0]+dx, cell[1])) or
                                  not valid((cell[0], cell[1]+dy))):
                    continue
                new = cost+math.hypot(dx, dy)
                if new < costs.get(nxt, float('inf')):
                    costs[nxt], parents[nxt] = new, cell
                    heapq.heappush(heap, (new+math.dist(nxt, target), new, nxt))
        if target not in costs:
            return []
        cells, cell = [target], target
        while cell != source:
            cell = parents[cell]
            cells.append(cell)
        points = [start]+[grid.grid_to_world(*c) for c in reversed(cells)][1:-1]+[goal]
        # Remove redundant cell-by-cell dots without cutting obstacle corners.
        simple, index = [start], 0
        while index < len(points)-1:
            nxt = len(points)-1
            while nxt > index+1 and not clear(points[index], points[nxt]):
                nxt -= 1
            if not clear(points[index], points[nxt]):
                return []
            simple.append(points[nxt])
            index = nxt
        points = simple
    route = [start]
    for a, b in zip(points, points[1:]):
        count = max(1, math.ceil(math.dist(a, b)/.40))
        route.extend((a[0]+(b[0]-a[0])*i/count, a[1]+(b[1]-a[1])*i/count)
                     for i in range(1, count+1))
    return route


class Navigator:
    def __init__(self, grid, goal, turn_every=3):
        self.grid, self.goal = grid, goal
        if not isinstance(turn_every, int) or isinstance(turn_every, bool) or turn_every < 1:
            raise ValueError("turn_every must be a positive integer")
        self.cruise_speed = .25
        self.lookahead, self.arrival_tolerance, self.pass_corridor = .55, .15, .30
        self.turn_every, self.max_yaw_rate = turn_every, 45.
        self.yaw_tolerance = 3.  # Resume translation once the scheduled turn is within 3 degrees.
        self.lock = threading.Lock()
        self.route, self.index, self.revision = [], 1, 0
        self.reached, self.pending_turns, self.turn_sign = 0, 0, 1
        self.desired_yaw, self.turn_started = None, None
        self.state, self.reason, self.target, self.fault = 'WAIT_PATH', '', None, None
        self.last_plan, self.no_path_since = -float('inf'), None
        self.progress_anchor, self.progress_time = None, None

    def update_route(self, pose, now=None):
        """Worker call. Preserve the route unless blocked or substantially off it."""
        now = time.monotonic() if now is None else now
        if now-self.last_plan < .7:
            return
        p, blocked = (pose['x'], pose['y']), self.grid.costmap()
        with self.lock:
            if self.turn_started is not None:
                return  # Keep waypoint identities stable during an in-place turn.
            route, index, revision = list(self.route), self.index, self.revision
        replace = len(route) < 2
        if index < len(route):
            t, cross = projection(p, route[index-1], route[index])
            deviation = cross if 0 <= t <= 1 else math.dist(p, route[index-1] if t < 0 else route[index])
            remaining = [p]+route[index:]
            replace |= deviation > .45 or any(not self.grid.segment_clear(a, b, blocked)
                                             for a, b in zip(remaining, remaining[1:]))
        if replace:
            new_route = plan_route(p, self.goal, self.grid)
            with self.lock:
                if revision == self.revision and self.turn_started is None:
                    self.route, self.index = new_route, 1
                    self.revision += 1
        self.last_plan = now

    @staticmethod
    def velocity_limit(distance):
        """Limit only the component toward a nearby surface. Unknown: creep speed.
        Model: v*reaction_time + v²/(2*deceleration) <= surface_range-clearance.
        """
        if distance is None or not math.isfinite(distance) or distance <= 0:
            return .05
        room = max(0., distance-CLEARANCE)
        acceleration, delay = .4, .20
        return max(0., math.sqrt((acceleration*delay)**2+2*acceleration*room)-acceleration*delay)

    def command(self, pose, ranges, now=None):
        now = time.monotonic() if now is None else now
        # The worker can replace the route only between complete command updates.
        with self.lock:
            return self._command(pose, ranges, now)

    def _rotation_command(self, yaw, now):
        """Return yaw-only motion, or None when this scheduled turn is complete."""
        error = wrap(self.desired_yaw-yaw)
        if abs(error) <= self.yaw_tolerance:
            self.turn_started = None
            return None
        if now-self.turn_started > 8.:
            self.fault = 'Yaw turn timed out'
            return 0., 0., 0.
        self.state, self.reason = 'ROTATING', 'scheduled_turn'
        rate = max(-self.max_yaw_rate, min(self.max_yaw_rate, 2.*error))
        return 0., 0., rate  # Never send translation while rotating.

    def _command(self, pose, ranges, now):
        if self.fault:
            return 0., 0., 0.
        p = (pose['x'], pose['y'])
        yaw = pose['yaw']
        if not all(math.isfinite(v) for v in (*p, yaw)):
            self.fault = 'Invalid pose'
            return 0., 0., 0.
        # Watch real odometry displacement, not changes caused by SLAM corrections.
        odom = (pose.get('odom_x', p[0]), pose.get('odom_y', p[1]))
        if self.desired_yaw is None:
            self.desired_yaw = yaw
            self.progress_anchor, self.progress_time = odom, now
        if self.turn_started is not None:
            rotation = self._rotation_command(yaw, now)
            if rotation is not None:
                return rotation  # Do not count waypoints or chase XY drift during yaw.
            # Rotation is intentionally stationary; restart the progress watchdog.
            self.progress_anchor, self.progress_time = odom, now
        if len(self.route) < 2:
            self.state, self.reason = 'WAIT_PATH', 'no_route'
            self.no_path_since = now if self.no_path_since is None else self.no_path_since
            if now-self.no_path_since > 10.:
                self.fault = 'No map route for 10 seconds'
            return 0., 0., 0.
        self.no_path_since = None
        while self.index < len(self.route):
            t, cross = projection(p, self.route[self.index-1], self.route[self.index])
            if not (math.dist(p, self.route[self.index]) <= self.arrival_tolerance or
                    (self.index < len(self.route)-1 and t >= 1. and cross <= self.pass_corridor)):
                break
            self.index += 1
            self.reached += 1
            if self.reached % self.turn_every == 0:
                self.pending_turns += 1
        if self.index == len(self.route):
            self.state, self.reason = 'PATH_END', 'route_complete'
            return 0., 0., 0.
        if self.pending_turns:
            self.desired_yaw = wrap(self.desired_yaw+90.*self.turn_sign)
            self.turn_sign *= -1
            self.pending_turns -= 1
            self.turn_started = now
            rotation = self._rotation_command(yaw, now)
            if rotation is not None:
                return rotation
            self.progress_anchor, self.progress_time = odom, now
        # Between scheduled turns, yaw rate is zero as well. Translation and yaw
        # commands never overlap, including the final few degrees of alignment.
        yaw_rate = 0.
        # Aim ahead, instead of stopping to center over each dot.
        a, b = self.route[self.index-1], self.route[self.index]
        t, _ = projection(p, a, b)
        t = min(1., max(0., t))
        cursor = (a[0]+t*(b[0]-a[0]), a[1]+t*(b[1]-a[1]))
        remaining, target = self.lookahead, b
        for point in self.route[self.index:]:
            distance = math.dist(cursor, point)
            if distance >= remaining and distance > 1e-9:
                target = (cursor[0]+(point[0]-cursor[0])*remaining/distance,
                          cursor[1]+(point[1]-cursor[1])*remaining/distance)
                break
            remaining -= distance
            cursor = target = point
        blocked = self.grid.costmap()
        if not self.grid.segment_clear(p, target, blocked):
            target = b                    # Shorten lookahead to avoid cutting a corner.
        self.target = target
        dx, dy = target[0]-p[0], target[1]-p[1]
        distance = math.hypot(dx, dy)
        if distance < 1e-9:
            return 0., 0., yaw_rate
        speed = self.cruise_speed
        speed = min(speed, max(.05, 1.2*math.dist(p, self.goal)))
        wx, wy = speed*dx/distance, speed*dy/distance
        # MotionCommander takes BODY velocities: +X forward, +Y left.
        angle = math.radians(yaw)
        c, s = math.cos(angle), math.sin(angle)
        bx, by = wx*c+wy*s, -wx*s+wy*c
        front, back, left, right = ranges
        bx = max(-self.velocity_limit(back), min(self.velocity_limit(front), bx))
        by = max(-self.velocity_limit(right), min(self.velocity_limit(left), by))
        # Check the combined velocity too; separate axes must not cut a mapped corner.
        wx, wy = bx*c-by*s, bx*s+by*c
        self.reason = 'clear'
        if not self.grid.segment_clear(p, (p[0]+.35*wx, p[1]+.35*wy), blocked):
            bx = by = 0.
            self.reason = 'map_blocks_motion'
        elif math.hypot(bx, by) < .01:
            self.reason = 'range_blocks_motion'
        self.state = 'NAVIGATING'
        if math.hypot(bx, by) < .01:
            self.state = 'BLOCKED'
        if math.dist(odom, self.progress_anchor) >= .08:
            self.progress_anchor, self.progress_time = odom, now
        elif now-self.progress_time > 12.:
            self.fault = 'No odometry displacement for 12 seconds'
            return 0., 0., 0.
        return bx, by, yaw_rate
