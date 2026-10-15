"""Closed-loop simulation of box_finder.py. Run: python3 -m unittest test_box_finder"""
import math
import random
import unittest
from pathlib import Path
import box_finder as bf
from test_navigation import box_distance, ray_box

CONE = math.radians(13.5)   # VL53L1x half field of view.
DISK = [(r*math.cos(a), r*math.sin(a)) for r in (0., .33, .67, 1.)
        for a in [2*math.pi*k/max(1, round(8*r)) for k in range(max(1, round(8*r)))]]


class Sim:
    """Drone over a floor with one box.

    - Horizontal: velocity setpoints with ~0.35 s lag, Flow-deck odometry with 2% scale
      error and jitter.
    - Vertical: the estimator follows height above whatever is underneath, but only
      `chase` of each surface change (real logs plateau at ~0.4 m over the box, i.e. a
      partial chase); the controller drives that estimate to HEIGHT with ~0.4 s lag.
    - zrange: area average over the 27 degree cone (or the nearest return), 20 Hz,
      1 mm quantized, a few mm of noise.
    """
    def __init__(self, box, start=(0., 0.), chase=1., nearest=False, bias=0., walls=(), seed=0, drift=0.):
        self.box, self.chase, self.nearest = box, chase, nearest   # box: (cx, cy, length, width, angle_deg)
        self.drift, self.drift_rng = drift, random.Random(1000 + seed)   # odometry random walk, m/sqrt(s)
        self.walls = list(walls)                 # axis-aligned (x0, x1, y0, y1) at ranger height
        self.wall_clearance = math.inf
        self.bias = bias                         # zrange offset; the real hover read 0.515 at HEIGHT 0.5
        self.rng = random.Random(seed)
        self.x, self.y = start
        self.ox, self.oy = start
        self.vx = self.vy = 0.
        self.z, self.vz = bf.HEIGHT, 0.          # absolute altitude
        self.surface_seen = self.surface()       # what the estimator believes the floor is
        self.t, self.next_range, self.zrange = 0., 0., bf.HEIGHT

    def on_box(self, x, y):
        cx, cy, length, width, angle = self.box
        a = math.radians(angle)
        dx, dy = x-cx, y-cy
        return abs(dx*math.cos(a)+dy*math.sin(a)) <= length/2 and abs(-dx*math.sin(a)+dy*math.cos(a)) <= width/2

    def surface(self):
        return bf.BOX_HEIGHT if self.on_box(self.x, self.y) else 0.

    def measure(self):
        r = max(.05, self.z)*math.tan(CONE)
        hits = [self.on_box(self.x+r*px, self.y+r*py) for px, py in DISK]
        if self.nearest:
            top = bf.BOX_HEIGHT if any(hits) else 0.
        else:
            top = bf.BOX_HEIGHT*sum(hits)/len(hits)
        return round(self.z - top + self.bias + self.rng.gauss(0, .005), 3)

    def advance(self, command, dt):
        bx, by, _ = command
        for _ in range(round(dt/.005)):
            h = .005
            self.vx += h/.35*(bx-self.vx)
            self.vy += h/.35*(by-self.vy)
            self.x += self.vx*h
            self.y += self.vy*h
            self.ox += self.vx*1.02*h + self.drift*math.sqrt(h)*self.drift_rng.gauss(0, 1)
            self.oy += self.vy*1.02*h + self.drift*math.sqrt(h)*self.drift_rng.gauss(0, 1)
            # Estimator: height = altitude - believed floor; the belief moves `chase` of the way.
            target = self.chase*self.surface()
            self.surface_seen += h/.08*(target - self.surface_seen)
            estimate = self.z - self.surface_seen
            self.vz += h/.15*(max(-.6, min(.6, 2.5*(bf.HEIGHT-estimate))) - self.vz)
            self.z += self.vz*h
            self.t += h
            if self.walls:
                self.wall_clearance = min([self.wall_clearance] + [box_distance(self.x, self.y, w) for w in self.walls])
            if self.t >= self.next_range:
                self.zrange, self.next_range = self.measure(), self.next_range + .05

    def ranges(self):
        """(front, back, left, right) with the nose along +x, NaN beyond 4 m."""
        out = []
        for a in (0., 180., 90., -90.):
            ux, uy = math.cos(math.radians(a)), math.sin(math.radians(a))
            d = min([ray_box(self.x, self.y, ux, uy, w) for w in self.walls], default=math.inf)
            out.append(d + self.rng.gauss(0, .005) if d < 4. else math.nan)
        return tuple(out)

    def odometry(self):
        j = lambda: self.rng.gauss(0, .003)
        return self.ox + j(), self.oy + j()


def run(box, seconds=240., box_size=bf.BOX_SIZE, **kw):
    sim = Sim(box, **kw)
    finder = bf.BoxFinder(box_size=box_size)
    t = 0.
    while t < seconds and not finder.done and not finder.fault:
        x, y = sim.odometry()
        command = finder.step(t, x, y, 0., sim.zrange, sim.ranges())
        sim.advance(command, .02)
        t += .02
    return finder, sim, t


class BoxFinderTests(unittest.TestCase):
    def assert_lands_on_center(self, box, tolerance=.05, **kw):
        for seed in range(3):
            finder, sim, t = run(box, seed=seed, **kw)
            self.assertIsNone(finder.fault, f'seed {seed}')
            self.assertTrue(finder.done, f'seed {seed}: timed out in {finder.state}')
            error = math.dist((sim.x, sim.y), box[:2])
            self.assertLess(error, tolerance, f'seed {seed}: {error:.3f} m from center after {t:.0f} s')

    def test_square_box_ahead(self):
        self.assert_lands_on_center((1.0, .10, .30, .30, 0.))

    def test_box_hit_near_its_corner(self):
        self.assert_lands_on_center((1.2, -.12, .35, .30, 0.))

    def test_rotated_rectangle(self):
        self.assert_lands_on_center((1.0, .05, .40, .25, 25.), tolerance=.06)

    def test_like_real_log(self):
        # Push log, 26 cm box: hover reads 0.515, and on top the controller only makes up
        # about half the box height (zrange 0.38-0.41).
        self.assert_lands_on_center((1.0, .10, .30, .30, 0.), chase=.55, bias=.015)

    def test_one_pass_per_axis_and_quick(self):
        for seed in range(3):
            finder, sim, t = run((1.0, .10, .30, .30, 0.), chase=.55, bias=.015, seed=seed)
            self.assertTrue(finder.done)
            self.assertEqual(len(finder.history), 2)
            self.assertLess(t, 55., f'seed {seed}')

    def test_odometry_drift(self):
        # Real logs: the same edge moved 2-18 cm between passes ~40 s apart.
        self.assert_lands_on_center((1.0, .10, .30, .30, 0.), chase=.55, bias=.015,
                                    drift=.005, tolerance=.07)

    def test_nearest_return_ranger(self):
        self.assert_lands_on_center((1.0, .10, .30, .30, 0.), nearest=True)
        self.assert_lands_on_center((1.0, .05, .40, .25, 25.), nearest=True, chase=.55, tolerance=.06)

    def test_large_box(self):
        self.assert_lands_on_center((1.0, .10, .50, .50, 10.), box_size=.5, nearest=True, chase=.55)
        self.assert_lands_on_center((1.0, .10, .50, .50, 10.), box_size=.5)

    def test_wall_beside_box_shortens_passes(self):
        # The full side pass would reach y = +0.8; the wall is at y = 0.75.
        wall = (-1., 3., .75, .85)
        for seed in range(3):
            finder, sim, t = run((1.0, .10, .30, .30, 0.), walls=[wall], chase=.55, seed=seed)
            self.assertIsNone(finder.fault, f'seed {seed}')
            self.assertTrue(finder.done)
            self.assertTrue(any('wall' in n for n in finder.notes))
            self.assertLess(math.dist((sim.x, sim.y), (1.0, .10)), .05, f'seed {seed}')
            self.assertGreater(sim.wall_clearance, .15, f'seed {seed}')

    def test_box_in_a_corner(self):
        # Box edges 0.5 m from two walls: every first pass is cut short.
        walls = [(1.75, 1.85, -2., 2.), (-1., 3., -.85, -.75)]
        for seed in range(3):
            finder, sim, t = run((1.1, -.1, .30, .30, 0.), walls=walls, chase=.55, seed=seed)
            self.assertIsNone(finder.fault, f'seed {seed}')
            self.assertLess(math.dist((sim.x, sim.y), (1.1, -.1)), .05, f'seed {seed}')
            self.assertGreater(sim.wall_clearance, .15, f'seed {seed}')

    def test_box_against_a_wall_fails_with_reason(self):
        finder, sim, t = run((1.1, -.1, .30, .30, 0.), walls=[(1.55, 1.65, -2., 2.)])
        self.assertIn('too close', finder.fault)
        self.assertGreater(sim.wall_clearance, .15)

    def test_wall_ahead_without_box_stops_search(self):
        finder, sim, t = run((5., 5., .3, .3, 0.), walls=[(1.2, 1.3, -2., 2.)])
        self.assertIn('wall ahead', finder.fault)
        self.assertGreater(sim.wall_clearance, .2)

    def test_guard_backs_off_a_wall_that_appears(self):
        finder = bf.BoxFinder()
        nan = float('nan')
        for k in range(100):
            vx, vy, _ = finder.step(k*.02, 0., 0., 0., .5, (nan, nan, .12, nan))
        self.assertLess(vy, 0.)

    def test_no_box_fails_cleanly(self):
        finder, sim, t = run((5., 5., .3, .3, 0.))
        self.assertIn('no box found', finder.fault)
        self.assertLess(abs(sim.x - bf.SEARCH_DISTANCE), .2)

    def test_hover_noise_gives_no_edges(self):
        # Real hover: 0.515 +/- 0.010 m, extremes 0.499-0.534.
        sim, detector = Sim((5., 5., .3, .3, 0.)), bf.EdgeDetector(.515)
        for k in range(1000):
            sim.advance((0., 0., 0.), .02)
            z = sim.zrange + .015 + max(-.025, min(.025, random.Random(k).gauss(0, .01)))
            self.assertIsNone(detector.update(k*.02, z))

    def test_detector_on_real_push_log(self):
        log = Path(__file__).with_name('flight_log.csv')
        if not log.exists():
            self.skipTest('flight_log.csv not present')
        events = bf.replay(str(log), plot=False)
        kinds = [e for _, e in events]
        self.assertGreaterEqual(len(kinds), 10)
        self.assertEqual(kinds[0], 'enter')
        self.assertTrue(all(a != b for a, b in zip(kinds, kinds[1:])))
        self.assertAlmostEqual(events[0][0], 16.9, delta=.2)
        self.assertFalse([t for t, _ in events if t < 16.5], 'false edge during the steady hover')

    def test_detector_on_slow_search_log(self):
        log = Path(__file__).with_name('box_log_20261015_111352.csv')
        if not log.exists():
            self.skipTest('box_log_20261015_111352.csv not present')
        events = bf.replay(str(log), plot=False)
        self.assertGreaterEqual(len(events), 2)
        self.assertEqual(events[0][1], 'enter')
        self.assertAlmostEqual(events[0][0], 4.0, delta=.2)


if __name__ == '__main__':
    unittest.main()
