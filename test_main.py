"""main.fly() against a fake cflib driven by the simulator (real time, ~10 s)."""
import csv
import math
import sys
import tempfile
import threading
import time
import types
import unittest
from pathlib import Path
from unittest.mock import patch
import main
from test_navigation import Sim


class FakeCrazyflie:
    """Estimator frame is offset and rotated from the map, like after a Kalman reset."""
    YAW_OFFSET, ORIGIN = 30., (1., 2.)

    def __init__(self, sim):
        self.sim, self.command, self.callbacks = sim, (0., 0., 0.), []
        self.flying = self.landed = False
        self.height_mm = 500
        self.stop = threading.Event()

    def emit(self):
        while not self.stop.wait(.05):
            if self.flying:
                self.sim.advance(self.command, .05)
            a = math.radians(self.YAW_OFFSET)
            dx, dy = self.sim.x-main.START[0], self.sim.y-main.START[1]
            ranges = [int(r*1000) if math.isfinite(r) else 8190 for r in self.sim.ranges()]
            data = {'stateEstimate.x': self.ORIGIN[0]+math.cos(a)*dx-math.sin(a)*dy,
                    'stateEstimate.y': self.ORIGIN[1]+math.sin(a)*dx+math.cos(a)*dy,
                    'stateEstimate.yaw': self.sim.yaw+self.YAW_OFFSET,
                    **{'range.'+d: v for d, v in zip(main.DIRECTIONS, ranges+[8190, self.height_mm])}}
            for f in self.callbacks:
                f(0, data, None)

    def modules(self):
        fake = self

        class Callbacks:
            def add_callback(self, f): fake.callbacks.append(f)

        class LogConfig:
            def __init__(self, **_): self.data_received_cb = Callbacks()
            def add_variable(self, *_): pass
            def start(self): threading.Thread(target=fake.emit, daemon=True).start()
            def stop(self): fake.stop.set()

        class SyncCrazyflie:
            def __init__(self, uri, cf=None):
                self.cf = types.SimpleNamespace(log=types.SimpleNamespace(add_config=lambda c: None),
                                                platform=types.SimpleNamespace(send_arming_request=lambda a: None))
            def __enter__(self): return self
            def __exit__(self, *a): pass

        class MotionCommander:
            def __init__(self, scf, default_height): pass
            def __enter__(self):
                fake.flying = True
                return self
            def __exit__(self, *a):
                fake.command, fake.flying, fake.landed = (0., 0., 0.), False, True
            def start_linear_motion(self, vx, vy, vz, yaw_rate): fake.command = (vx, vy, yaw_rate)
            def stop(self): fake.command = (0., 0., 0.)

        names = {'cflib': {}, 'cflib.crtp': {'init_drivers': lambda: None},
                 'cflib.crazyflie': {'Crazyflie': lambda **k: None},
                 'cflib.crazyflie.log': {'LogConfig': LogConfig},
                 'cflib.crazyflie.syncCrazyflie': {'SyncCrazyflie': SyncCrazyflie},
                 'cflib.positioning': {},
                 'cflib.positioning.motion_commander': {'MotionCommander': MotionCommander}}
        modules = {}
        for name, attrs in names.items():
            module = types.ModuleType(name)
            module.__dict__.update(attrs)
            modules[name] = module
        modules['cflib'].crtp = modules['cflib.crtp']
        return modules


class MainLoopTest(unittest.TestCase):
    def test_flies_to_goal_around_wall_and_lands(self):
        sim = Sim(main.START, boxes=[(.9, 1.0, -.4, .4)])
        fake = FakeCrazyflie(sim)
        with tempfile.TemporaryDirectory() as tmp, patch.dict(sys.modules, fake.modules()), \
                patch.object(main, 'GOAL', (1.5, 0.)):
            output = str(Path(tmp)/'log.csv')
            result = main.fly('radio://fake', output)
            with open(output) as f:
                rows = list(csv.DictReader(f))
        self.assertEqual(result, 'goal reached')
        self.assertTrue(fake.landed)
        self.assertLess(math.dist((sim.x, sim.y), (1.5, 0.)), .15)
        self.assertGreater(sim.min_clearance, .13)
        self.assertAlmostEqual(float(rows[0]['x']), main.START[0], delta=.05)
        self.assertIn('DETOUR', {r['state'] for r in rows})
        self.assertEqual((rows[-1]['state'], rows[-1]['reason']), ('END', 'goal reached'))

    def test_lands_when_height_is_lost(self):
        sim = Sim(main.START)
        fake = FakeCrazyflie(sim)
        threading.Timer(3., lambda: setattr(fake, 'height_mm', 300)).start()
        with tempfile.TemporaryDirectory() as tmp, patch.dict(sys.modules, fake.modules()), \
                patch.object(main, 'GOAL', (3., 0.)):
            result = main.fly('radio://fake', str(Path(tmp)/'log.csv'))
        self.assertTrue(result.startswith('height lost'), result)
        self.assertTrue(fake.landed)


if __name__ == '__main__':
    unittest.main()
