"""Closed-loop 2D simulation of the navigator. Run: python3 -m unittest -v"""
import math
import random
import unittest
from planner import Navigator, DRONE_RADIUS, SENSOR_ANGLES, guard

ARENA = (0., 4.6, -1.475, 1.475)


def ray_box(ox, oy, ux, uy, box):
    """Distance along a ray to an axis-aligned box (x0, x1, y0, y1), or inf."""
    x0, x1, y0, y1 = box
    t0, t1 = -math.inf, math.inf
    for o, u, lo, hi in ((ox, ux, x0, x1), (oy, uy, y0, y1)):
        if abs(u) < 1e-12:
            if not lo <= o <= hi:
                return math.inf
            continue
        a, b = (lo-o)/u, (hi-o)/u
        t0, t1 = max(t0, min(a, b)), min(t1, max(a, b))
    return t0 if t1 >= t0 >= 0 else math.inf


def ray_circle(ox, oy, ux, uy, circle):
    cx, cy, r = circle
    fx, fy = ox-cx, oy-cy
    b = fx*ux+fy*uy
    disc = b*b-(fx*fx+fy*fy-r*r)
    if disc < 0:
        return math.inf
    t = -b-math.sqrt(disc)
    return t if t >= 0 else math.inf


def box_distance(x, y, box):
    x0, x1, y0, y1 = box
    return math.hypot(max(x0-x, 0, x-x1), max(y0-y, 0, y-y1))


class Sim:
    """Point drone with Crazyflie-like dynamics, Flow-deck odometry and four rangers.

    Matched to real logs: body-frame velocity setpoints (the firmware re-rotates them
    with the current yaw, while velocity lags ~0.35 s), lagged yaw rate (the bang-bang
    sweep overshoots ~20 degrees as in flight), odometry that drifts while yawing, and
    rangers that refresh at ~10 Hz and arrive 50-150 ms old.
    """
    def __init__(self, start, boxes=(), circles=(), yaw_sign=1., seed=0):
        self.x, self.y, self.yaw = *start, 0.
        self.vx = self.vy = self.yaw_rate = 0.
        self.boxes, self.circles, self.yaw_sign = list(boxes), list(circles), yaw_sign
        self.rng = random.Random(seed)
        self.min_clearance = math.inf
        self.ox, self.oy = start
        a = self.rng.uniform(0, 2*math.pi)
        self.spin_drift = (.01*math.cos(a), .01*math.sin(a))  # m/s of odometry error per rad/s of yaw
        self.history = []

    def ranges(self, age=.05):
        """Ranges measured `age` seconds ago, along the true beams at that time."""
        x, y, yaw = self.history[-1-round(age/.01)] if len(self.history) > round(age/.01) else (self.x, self.y, self.yaw)
        out = []
        for offset in SENSOR_ANGLES:
            d = math.inf
            # The VL53L1x sees a cone (27 deg in the datasheet) and reports the nearest
            # target in it. Model a conservative 12 deg cone.
            for spread in (-6., -3., 0., 3., 6.):
                a = math.radians(yaw+offset+spread)
                ux, uy = math.cos(a), math.sin(a)
                # Arena walls: distance to leave the arena box from inside.
                x0, x1, y0, y1 = ARENA
                walls = min((x1-x)/ux if ux > 1e-9 else (x0-x)/ux if ux < -1e-9 else math.inf,
                            (y1-y)/uy if uy > 1e-9 else (y0-y)/uy if uy < -1e-9 else math.inf)
                d = min([d, walls]+[ray_box(x, y, ux, uy, b) for b in self.boxes]
                        + [ray_circle(x, y, ux, uy, c) for c in self.circles])
            out.append(d+self.rng.gauss(0, .01) if d < 4. else float('nan'))
        return tuple(out)

    def odometry(self):
        j = lambda: self.rng.gauss(0, .004)  # Flow-deck jitter seen in real hover logs.
        return self.ox+j(), self.oy+j(), self.yaw+self.rng.gauss(0, .3)

    def advance(self, command, dt):
        bx, by, yaw_rate = command
        for _ in range(round(dt/.01)):
            h = .01
            self.yaw_rate += h/.12*(self.yaw_sign*yaw_rate - self.yaw_rate)
            self.yaw += self.yaw_rate*h
            c, s = math.cos(math.radians(self.yaw)), math.sin(math.radians(self.yaw))
            wx, wy = c*bx-s*by, s*bx+c*by
            self.vx += h/.35*(wx-self.vx)
            self.vy += h/.35*(wy-self.vy)
            self.x += self.vx*h
            self.y += self.vy*h
            spin = abs(math.radians(self.yaw_rate))
            self.ox += (self.vx*1.02 + self.spin_drift[0]*spin)*h
            self.oy += (self.vy*1.02 + self.spin_drift[1]*spin)*h
            self.history.append((self.x, self.y, self.yaw))
            x0, x1, y0, y1 = ARENA
            walls = min(self.x-x0, x1-self.x, self.y-y0, y1-self.y)
            self.min_clearance = min([self.min_clearance, walls]
                                     + [box_distance(self.x, self.y, b) for b in self.boxes]
                                     + [math.hypot(self.x-cx, self.y-cy)-r for cx, cy, r in self.circles])
        del self.history[:-30]


def fly(start, goal, boxes=(), circles=(), seconds=120., yaw_sign=1., seed=0, fence=None):
    """The ARENA walls are physical in the simulation; `fence` is the navigator's virtual fence."""
    sim = Sim(start, boxes, circles, yaw_sign, seed)
    nav = Navigator(goal, fence)
    t, dt, ticks = 0., .05, 0
    ranges = sim.ranges()
    measured = sim.odometry(), ranges
    while t < seconds and not nav.done and not nav.fault:
        command = nav.step(*measured[0], measured[1], t)
        ticks += 1
        if ticks % 2 == 0:
            ranges = sim.ranges()  # The multi-ranger refreshes at ~10 Hz and lags, as in real logs.
        measured = sim.odometry(), ranges  # Delivered one control cycle late.
        sim.advance(command, dt)
        t += dt
    return nav, sim, t


class NavigationTests(unittest.TestCase):
    def assert_reached(self, start, goal, tolerance=.30, **world):
        for seed in range(3):
            nav, sim, t = fly(start, goal, seed=seed, **world)
            self.assertIsNone(nav.fault, f'seed {seed}')
            self.assertTrue(nav.done, f'seed {seed}: timed out at ({sim.x:.2f}, {sim.y:.2f})')
            # Odometry error (2% scale plus drift while turning) is included in the true error.
            self.assertLess(math.dist((sim.x, sim.y), goal), tolerance, f'seed {seed}')
            self.assertLess(t, 75., f'seed {seed}: took too long')
            self.assertGreater(sim.min_clearance, DRONE_RADIUS+.03, f'seed {seed}: too close to obstacle')

    def test_short_hop_in_open_space(self):
        # The failing real flight: 30 cm forward, nothing in the way.
        nav, sim, t = fly((.3, 0.), (.6, 0.))
        self.assertTrue(nav.done)
        self.assertLess(t, 12.)

    def test_long_diagonal_open_space(self):
        self.assert_reached((.3, 0.), (4., 1.))

    def test_wall_straight_ahead(self):
        self.assert_reached((.3, 0.), (3., 0.), boxes=[(1.4, 1.5, -.6, .6)])

    def test_box_offset_from_path(self):
        self.assert_reached((.3, 0.), (3.5, .2), boxes=[(1.5, 1.9, -.2, .5)])

    def test_thin_pole_near_diagonal(self):
        self.assert_reached((.3, 0.), (2., 1.), circles=[(1.2, .55, .03)])

    def test_gap_in_wall(self):
        self.assert_reached((.3, .8), (3.5, 0.), boxes=[(2., 2.1, -1.475, -.35), (2., 2.1, .35, 1.475)])

    def test_door_60cm(self):
        # The real door: a ~60 cm opening in a 10 cm thick wall.
        self.assert_reached((.3, 0.), (4., -.6), boxes=[(2.5, 2.6, -1.475, -.9), (2.5, 2.6, -.3, 1.475)])

    def test_door_55cm_approached_at_an_angle(self):
        self.assert_reached((.3, 1.), (4., -.4), boxes=[(2.5, 2.6, -1.475, -.675), (2.5, 2.6, -.125, 1.475)])

    def test_door_in_room_corner(self):
        # As in the real room: the opening runs from the end of a wall to the side wall.
        # Detours make this path ~5 m, so the simulated odometry drifts further.
        self.assert_reached((.3, 0.), (4., -1.1), tolerance=.35, boxes=[(2.5, 2.6, -.85, 1.475)])

    def test_virtual_fence_is_respected(self):
        self.assert_reached((.3, 0.), (3., 1.), fence=ARENA)

    def test_two_obstacles(self):
        self.assert_reached((.3, 0.), (4., 0.), boxes=[(1.2, 1.4, -.5, .3), (2.6, 2.8, -.3, .6)])

    def test_blocked_arena_lands_instead_of_wandering(self):
        nav, sim, t = fly((.3, 0.), (3., 0.), boxes=[(1.5, 1.6, -1.475, 1.475)])
        self.assertFalse(nav.done)
        self.assertIn('No path', nav.fault)
        self.assertLess(t, 60.)
        self.assertGreater(sim.min_clearance, DRONE_RADIUS+.03)

    def test_wrong_yaw_sign_faults_without_crashing(self):
        nav, sim, _ = fly((.3, 0.), (3., 1.), yaw_sign=-1.)
        self.assertFalse(nav.done)
        self.assertIsNotNone(nav.fault)
        self.assertGreater(sim.min_clearance, DRONE_RADIUS)

    def test_never_turns_and_translates_together(self):
        nav, _, _ = fly((.3, 0.), (3., 0.), boxes=[(1.4, 1.5, -.6, .6)])
        self.assertTrue(nav.done)
        sim = Sim((.3, 0.), boxes=[(1.4, 1.5, -.6, .6)])
        nav, t = Navigator((3., 0.), ARENA), 0.
        while not nav.done and t < 60.:
            vx, vy, rate = nav.step(*sim.odometry(), sim.ranges(), t)
            self.assertFalse(abs(rate) > 20.5 and math.hypot(vx, vy) > .05, f'{nav.state} at {t:.2f}')
            sim.advance((vx, vy, rate), .05)
            t += .05

    def test_guard_backs_away_from_close_obstacle_even_with_wrong_map(self):
        nan = float('nan')
        vx, vy = guard(.2, 0., (.15, nan, nan, nan))
        self.assertLess(vx, 0.)
        vx, vy = guard(.2, .2, (.40, nan, .30, nan))
        self.assertLess(vx, .2)
        self.assertLess(vy, .2)
        self.assertEqual(guard(.2, -.1, (nan,)*4), (.2, -.1))

    def test_position_jump_ends_flight(self):
        nav = Navigator((2., 0.))
        nan = float('nan')
        nav.step(1., 0., 0., (nan,)*4, 0.)
        nav.step(1.25, 0., 0., (nan,)*4, .05)
        self.assertIn('jumped', nav.fault)


if __name__ == '__main__':
    unittest.main()
