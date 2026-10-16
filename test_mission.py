"""Closed-loop simulation of mission.py. Run: python3 -m unittest test_mission"""
import math
import unittest
import box_finder as bf
import mission as m
from test_box_finder import Sim


class Field(Sim):
    """Sim with several boxes; the drone may start on top of one."""
    def __init__(self, boxes, start, **kw):
        self.boxes = list(boxes)
        super().__init__(self.boxes[0], start=start, **kw)
        self.z = bf.HEIGHT + self.surface()
        self.surface_seen = self.surface()
        self.ox = self.oy = self.yaw = 0.

    def advance(self, command, dt):
        """Body-frame command; the navigator turns to scan, so yaw is modelled here."""
        vx, vy, rate = command
        c, s = math.cos(math.radians(self.yaw)), math.sin(math.radians(self.yaw))
        self.yaw = (self.yaw + rate*dt + 180.) % 360. - 180.
        super().advance((c*vx - s*vy, s*vx + c*vy, 0.), dt)

    def on_box(self, x, y):
        for self.box in self.boxes:
            if Sim.on_box(self, x, y):
                return True
        return False

    def take_off(self):
        """Estimator reset on the ground: odometry restarts at 0 where the drone sits."""
        self.ox = self.oy = self.vx = self.vy = 0.
        self.z, self.vz = bf.HEIGHT + self.surface(), 0.
        self.surface_seen = self.surface()


def fly_leg(field, leg, anchor, seconds=m.LEG_SECONDS):
    t = 0.
    while t < seconds and not leg.done and not leg.fault:
        x, y, yaw = anchor.pose(*field.odometry(), field.yaw)
        command = leg.step(t, x, y, yaw, field.zrange, field.ranges())
        field.advance(command, .02)
        t += .02
    return t


def mission(box1, box2, start, goal, sweep=(-1., 1.), drift=0., seed=0, box_size=.4, **kw):
    """Both legs; returns [(leg, true landing spot, seconds)]."""
    field = Field([box1, box2], start, drift=drift, seed=seed, **kw)
    anchor, out = m.Anchor(*start), []
    for x_target, remembered in ((goal[0], None), (start[0], start)):
        field.take_off()
        leg = m.Leg(x_target, sweep, box_size=box_size, remembered=remembered)
        t = fly_leg(field, leg, anchor)
        out.append((leg, (field.x, field.y), t))
        if not leg.done:
            break
        anchor = m.Anchor(*anchor.pose(*field.odometry(), field.yaw))
        field.yaw = 0.
    return out


class SweepLineTests(unittest.TestCase):
    def test_two_lines_cover_window(self):
        lines = m.sweep_lines(2.0, .3, .4)
        self.assertEqual(len(lines), 2)
        self.assertAlmostEqual(lines[0], 1.85)
        self.assertAlmostEqual(lines[1], 2.15)

    def test_every_offset_has_a_line_inside_the_box(self):
        for window, size in ((.3, .4), (.5, .4), (.3, .3), (.1, .4)):
            lines = m.sweep_lines(0., window, size)
            for k in range(101):
                c = -window + 2*window*k/100
                self.assertLessEqual(min(abs(c - x) for x in lines), size/2 - m.LINE_MARGIN + 1e-9)


class MissionTests(unittest.TestCase):
    def assert_lands_on_both(self, box1, box2, start, goal, tolerance=.06, seeds=range(2), **kw):
        for seed in seeds:
            legs = mission(box1, box2, start, goal, seed=seed, **kw)
            self.assertEqual(len(legs), 2, f'seed {seed}: leg 1 failed: {legs[0][0].fault}')
            for (leg, spot, t), box, name in zip(legs, (box2, box1), ('box at B', 'box at A')):
                self.assertTrue(leg.done, f'seed {seed}, {name}: {leg.fault} ({leg.notes})')
                error = math.dist(spot, box[:2])
                self.assertLess(error, tolerance, f'seed {seed}, {name}: {error:.3f} m off after {t:.0f} s')

    def test_round_trip(self):
        # Drone placed 5 cm off box 1's center; box 2 is 0.2 m off the goal x.
        self.assert_lands_on_both((.05, .0, .4, .4, 0.), (2.2, .4, .4, .4, 0.), start=(0., 0.), goal=(2.0, 0.))

    def test_box_at_the_edge_of_the_x_window(self):
        self.assert_lands_on_both((0., 0., .4, .4, 0.), (1.72, -.5, .4, .4, 0.), start=(0., 0.), goal=(2.0, 0.))

    def test_with_odometry_drift(self):
        self.assert_lands_on_both((0., 0., .4, .4, 0.), (2.1, .3, .4, .4, 0.), start=(0., 0.), goal=(2.0, 0.),
                                  drift=.005, chase=.55, bias=.015, tolerance=.08)

    def test_return_uses_the_local_sweep(self):
        legs = mission((0., 0., .4, .4, 0.), (2.0, .5, .4, .4, 0.), start=(0., 0.), goal=(2.0, 0.))
        self.assertTrue(legs[1][0].done)
        self.assertFalse([n for n in legs[1][0].notes if 'full' in n])

    def test_falls_back_to_full_sweep(self):
        # Remembered box 1 is 1.5 m off: the local sweep misses, the full sweep finds it.
        field = Field([(0., -.6, .4, .4, 0.)], (0., .9))
        field.take_off()
        leg = m.Leg(0., (-1., 1.), remembered=(0., .9))
        fly_leg(field, leg, m.Anchor(0., .9))
        self.assertTrue(leg.done, leg.fault)
        self.assertTrue(any('local' in n for n in leg.notes))
        self.assertLess(math.dist((field.x, field.y), (0., -.6)), .06)

    def test_no_box_fails_cleanly(self):
        field = Field([(5., 5., .4, .4, 0.)], (0., 0.))
        field.take_off()
        leg = m.Leg(2.0, (-1., 1.))
        fly_leg(field, leg, m.Anchor(0., 0.))
        self.assertIn('no box found on any sweep line', leg.fault)


if __name__ == '__main__':
    unittest.main()
