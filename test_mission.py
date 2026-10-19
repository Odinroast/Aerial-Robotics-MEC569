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


def mission(box1, box2, start, goal, sweep=(-.5, .5), drift=0., seed=0, box_size=.4, walls=(),
            window=.3, approach=True, **kw):
    """Both legs; returns [(leg, true landing spot, seconds)]."""
    field = Field([box1, box2], start, drift=drift, seed=seed, walls=walls, **kw)
    anchor, out = m.Anchor(*start), []
    for _, x_target, y_range, remembered in m.mission_legs(start, goal, sweep):
        field.take_off()
        leg = m.Leg(x_target, y_range, box_size=box_size, box_height=bf.BOX_HEIGHT, window=window,
                    remembered=remembered, approach=approach)
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
    def assert_lands_on_both(self, box1, box2, start, goal, tolerance=.06, seeds=(0, 1), **kw):
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

    def test_sweep_is_centered_on_each_points_y(self):
        legs = m.mission_legs((0., .2), (3.25, 1.14), (-.5, .5))
        self.assertEqual(legs[0][1], 3.25)
        self.assertAlmostEqual(legs[0][2][0], .64)
        self.assertAlmostEqual(legs[0][2][1], 1.64)
        self.assertAlmostEqual(legs[1][2][0], -.3)
        self.assertAlmostEqual(legs[1][2][1], .7)

    def test_box_far_from_y_zero(self):
        # Like the last flights: goal y 1.14, so a sweep around y = 0 never reaches it.
        self.assert_lands_on_both((0., 0., .4, .4, 0.), (3.3, 1.25, .4, .4, 0.), start=(0., 0.),
                                  goal=(3.25, 1.14), sweep=(-.6, .6), seeds=[0])

    def test_wall_cuts_a_sweep_line_short(self):
        # Box on the second line; the first line runs into a wall at y = 0.75 before its
        # range end (1.0). The second line must start where the first stopped.
        wall = (1.0, 3.0, .75, .85)
        legs = mission((0., 0., .4, .4, 0.), (2.18, -.3, .4, .4, 0.), start=(0., 0.), goal=(2.0, 0.),
                       walls=[wall], approach=False)
        leg = legs[0][0]
        self.assertTrue(leg.done, f'{leg.fault} {leg.notes}')
        self.assertLess(math.dist(legs[0][1], (2.18, -.3)), .06)

    def test_line_start_next_to_clutter(self):
        # Like mission 110535: the planned start of the first line is 8 cm from something,
        # so the navigator has no path to it. The start moves along the line instead.
        clutter = (1.0, 1.7, -.85, -.75)
        legs = mission((0., 0., .35, .35, 0.), (1.35, .05, .35, .35, 0.), start=(0., 0.), goal=(1.35, 0.),
                       sweep=(-.2, .2), box_size=.35, walls=[clutter], window=.1, approach=False)
        leg = legs[0][0]
        self.assertTrue(leg.done, f'{leg.fault} {leg.notes}')
        self.assertLess(math.dist(legs[0][1], (1.35, .05)), .06)

    def test_tight_sweep_starts_on_the_floor(self):
        # Like mission 111021: --sweep -0.2 0.2 with a 35 cm, 16 cm high box. The lines run
        # past the range, so the search sees floor before the box.
        saved, bf.BOX_HEIGHT = bf.BOX_HEIGHT, .16
        try:
            for seed in (0, 1):
                legs = mission((0., 0., .35, .35, 0.), (1.35, .1, .35, .35, 0.), start=(0., 0.), goal=(1.35, 0.),
                               sweep=(-.2, .2), box_size=.35, window=.1, chase=.55, hover_bias=.02, seed=seed)
                for (leg, spot, t), box in zip(legs, ((1.35, .1), (0., 0.))):
                    self.assertTrue(leg.done, f'seed {seed}: {leg.fault} {leg.notes}')
                    self.assertLess(math.dist(spot, box), .06, f'seed {seed}')
        finally:
            bf.BOX_HEIGHT = saved

    def test_retries_a_line_when_the_center_fails(self):
        # Like mission 111021: the box was seen but a pass didn't show it. The line is flown
        # again (the other way) instead of ending the mission.
        real, calls = bf.mirror_center, []

        def flaky(*args, **kw):
            calls.append(1)
            if len(calls) == 1:
                raise bf.Failed('box not seen on a pass (profile varies 0.021 m)')
            return real(*args, **kw)
        bf.mirror_center = flaky
        try:
            field = Field([(2.0, .1, .4, .4, 0.)], (0., 0.))
            field.take_off()
            leg = m.Leg(2.0, (-.3, .3), box_size=.4, window=.3, approach=False)
            fly_leg(field, leg, m.Anchor(0., 0.))
        finally:
            bf.mirror_center = real
        self.assertTrue(leg.done, f'{leg.fault} {leg.notes}')
        self.assertTrue(any('attempt 1: box not seen' in n for n in leg.notes), leg.notes)
        self.assertLess(math.dist((field.x, field.y), (2.0, .1)), .06)

    def test_return_finds_box_1_drifted_in_x(self):
        # Like mission 114850: after drift box 1 isn't on the line through A's x. The local
        # sweep covers +/-0.3 m in X too (3 lines for a 35 cm box), middle line first.
        legs = mission((.3, .05, .35, .35, 0.), (1.35, -.1, .35, .35, 0.), start=(0., 0.), goal=(1.35, 0.),
                       sweep=(-.2, .2), box_size=.35, window=.1, approach=False)
        self.assertEqual(len(legs), 2, legs[0][0].fault)
        leg, spot, t = legs[1]
        self.assertTrue(leg.done, f'{leg.fault} {leg.notes}')
        self.assertLess(math.dist(spot, (.3, .05)), .06)
        self.assertEqual([p[0] for p in leg.plans], ['local'], 'the full sweep repeats the local one')

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

    def approach_mission(self, box2, goal, seed=0, approach=True, **kw):
        saved, bf.BOX_HEIGHT = bf.BOX_HEIGHT, .16
        try:
            return mission((0., 0., .35, .35, 0.), box2, start=(0., 0.), goal=goal, sweep=(-.2, .2),
                           box_size=.35, window=.1, chase=.55, hover_bias=.02, seed=seed,
                           approach=approach, **kw)
        finally:
            bf.BOX_HEIGHT = saved

    def assert_found_on_approach(self, legs, boxes):
        self.assertEqual(len(legs), 2, f'{legs[0][0].fault} {legs[0][0].notes}')
        for (leg, spot, t), box in zip(legs, boxes):
            self.assertTrue(leg.done, f'{leg.fault} {leg.notes}')
            self.assertEqual(leg.found_by, 'approach', leg.notes)
            self.assertLess(math.dist(spot, box), .06, f'{box}: {leg.notes}')

    def test_box_found_while_navigating(self):
        # Box 2 near B: the navigator crosses it on the way, and both legs land from there,
        # faster than sweeping.
        for seed in (0, 1):
            legs = self.approach_mission((1.7, .08, .35, .35, 0.), (1.65, 0.), seed=seed)
            self.assert_found_on_approach(legs, ((1.7, .08), (0., 0.)))
            swept = self.approach_mission((1.7, .08, .35, .35, 0.), (1.65, 0.), seed=seed, approach=False)
            self.assertLess(sum(t for *_, t in legs), sum(t for *_, t in swept) - 10.,
                            f'seed {seed}: {[round(t) for *_, t in legs]} vs {[round(t) for *_, t in swept]}')

    def test_box_found_while_navigating_along_y(self):
        legs = self.approach_mission((.06, 1.5, .35, .35, 0.), (0., 1.5))
        self.assert_found_on_approach(legs, ((.06, 1.5), (0., 0.)))

    def test_box_found_while_navigating_diagonally(self):
        # The route crosses the box at an angle; the passes still run along the box sides.
        legs = self.approach_mission((1.5, .8, .35, .35, 0.), (1.5, .8))
        self.assert_found_on_approach(legs, ((1.5, .8), (0., 0.)))

    def test_box_off_the_route_falls_back_to_the_sweep(self):
        legs = mission((0., 0., .4, .4, 0.), (2.0, .45, .4, .4, 0.), start=(0., 0.), goal=(2.0, 0.))
        leg = legs[0][0]
        self.assertTrue(leg.done, f'{leg.fault} {leg.notes}')
        self.assertEqual(leg.found_by, 'sweep', leg.notes)
        self.assertIn('approach: no box edge seen on the way; sweeping', leg.notes)
        self.assertLess(math.dist(legs[0][1], (2.0, .45)), .06)

    def test_no_box_fails_cleanly(self):
        field = Field([(5., 5., .4, .4, 0.)], (0., 0.))
        field.take_off()
        leg = m.Leg(2.0, (-1., 1.))
        fly_leg(field, leg, m.Anchor(0., 0.))
        self.assertIn('no box found on any sweep line', leg.fault)


if __name__ == '__main__':
    unittest.main()
