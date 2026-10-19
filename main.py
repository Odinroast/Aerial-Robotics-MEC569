"""Fly a Crazyflie (Flow deck + Multi-ranger deck) from START to GOAL, avoiding obstacles.

    python3 main.py                      # default radio URI
    python3 main.py radio://0/80/2M/E7E7E7E7E7 --goal 2.0 0.5

Place the drone at START facing map +X. It takes off, then repeats: stop, turn in place
to +45/-45/0 degrees so the four rangers map the surroundings, and fly up to 1 m with the
nose held at 0 (it never turns and moves at the same time). A range guard backs away from
anything closer than 25 cm. It lands on arrival, on Ctrl+C, and on any fault: no path,
no progress, pose jump, yaw drift, lost telemetry, or lost height. A CSV log and a PNG
plot are saved.
"""
import argparse
import csv
from datetime import datetime
import math
import threading
import time
from planner import MIN_CLEARANCE, Navigator

START = (0.3, 0.0)                    # Map position of the drone at takeoff, m.
GOAL = (3.5, -1.5)                     # Target position, m.
HEIGHT = 0.5                          # Flight height, m.
# Optional virtual fence (x_min, x_max, y_min, y_max) in map meters. Keep it None unless
# the space really is that box: the drone keeps clear of the fence like a wall, so a
# fence that cuts through a doorway closes it.
ARENA = None
MISSION_SECONDS = 180.
HEIGHT_TOLERANCE = 0.15               # Land if the z-ranger stays off HEIGHT by more than this...
HEIGHT_SECONDS = 0.3                  # ...for this long (contact, prop strike, or drift).
DEFAULT_URI = 'radio://0/98/2M/E7E7E7E7E8'
DIRECTIONS = ('front', 'back', 'left', 'right', 'up', 'zrange')
FIELDS = ['time', 'state', 'reason', 'x', 'y', 'yaw', 'odom_x', 'odom_y', 'odom_yaw',
          *DIRECTIONS, 'cmd_vx', 'cmd_vy', 'cmd_yaw_rate', 'heading', 'free', 'goal_distance']


def meters(mm):
    """Ranger value in mm -> meters; NaN for no return / out of range."""
    value = mm/1000.
    return value if 0.02 < value < 4.0 else float('nan')


class Frame:
    """Map frame anchored at START: the first post-takeoff pose becomes START, heading 0."""
    def __init__(self, odom):
        self.ox, self.oy, self.oyaw = odom
        self.c, self.s = math.cos(math.radians(-self.oyaw)), math.sin(math.radians(-self.oyaw))

    def pose(self, odom):
        dx, dy = odom[0]-self.ox, odom[1]-self.oy
        return (START[0] + self.c*dx - self.s*dy, START[1] + self.s*dx + self.c*dy,
                (odom[2]-self.oyaw+180.) % 360. - 180.)


def fly(uri, output):
    import cflib.crtp
    from cflib.crazyflie import Crazyflie
    from cflib.crazyflie.log import LogConfig
    from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
    from cflib.positioning.motion_commander import MotionCommander

    latest, lock = {}, threading.Lock()

    def on_log(timestamp, data, _):
        with lock:
            latest.update(received=time.monotonic(),
                          odom=(data['stateEstimate.x'], data['stateEstimate.y'], data['stateEstimate.yaw']),
                          ranges=tuple(meters(data['range.'+d]) for d in DIRECTIONS))

    def sample():
        with lock:
            return dict(latest)

    def fresh_sample(after, timeout=1.):
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            s = sample()
            if s and s['received'] > after and all(map(math.isfinite, s['odom'])):
                return s
            time.sleep(.01)
        raise RuntimeError('No fresh telemetry')

    if ARENA is not None:
        x0, x1, y0, y1 = ARENA
        m = MIN_CLEARANCE
        for name, (px, py) in (('START', START), ('GOAL', GOAL)):
            if not (x0+m <= px <= x1-m and y0+m <= py <= y1-m):
                raise SystemExit(f'{name} {(px, py)} is not at least {m} m inside ARENA {ARENA}; '
                                 'move it or set ARENA = None')
    nav = Navigator(GOAL, ARENA)
    result = 'not started'
    cflib.crtp.init_drivers()
    with open(output, 'w', newline='') as file, \
            SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        log = csv.DictWriter(file, fieldnames=FIELDS)
        log.writeheader()
        config = LogConfig(name='nav', period_in_ms=50)
        for d in DIRECTIONS:
            config.add_variable('range.'+d, 'uint16_t')
        for axis in ('x', 'y', 'yaw'):
            config.add_variable('stateEstimate.'+axis, 'float')
        scf.cf.log.add_config(config)
        config.data_received_cb.add_callback(on_log)
        config.start()
        try:
            fresh_sample(0., timeout=5.)
            scf.cf.platform.send_arming_request(True)
            time.sleep(1.)
            # MotionCommander takes off on enter (resetting the estimator) and lands on exit,
            # including on exceptions and Ctrl+C.
            with MotionCommander(scf, default_height=HEIGHT) as motion:
                motion.start_linear_motion(0., 0., 0., 0.)
                time.sleep(1.5)
                frame = Frame(fresh_sample(time.monotonic())['odom'])
                print(f'Airborne. Flying {START} -> {GOAL}')
                start = last_print = height_ok = time.monotonic()
                while True:
                    now = time.monotonic()
                    s = sample()
                    if now - s['received'] > .5:
                        result = 'telemetry lost'; break
                    if now - start > MISSION_SECONDS:
                        result = 'mission time limit'; break
                    if math.isfinite(s['ranges'][4]) and s['ranges'][4] < .2:
                        result = 'object above the drone'; break
                    height = s['ranges'][5]
                    if math.isfinite(height) and abs(height-HEIGHT) < HEIGHT_TOLERANCE:
                        height_ok = now
                    if now - height_ok > HEIGHT_SECONDS:
                        result = f'height lost (z-range {height:.2f} m, wanted {HEIGHT:.2f} m)'; break
                    x, y, yaw = frame.pose(s['odom'])
                    command = nav.step(x, y, yaw, s['ranges'][:4], now)
                    if nav.done:
                        result = 'goal reached'; break
                    if nav.fault:
                        result = nav.fault; break
                    motion.start_linear_motion(command[0], command[1], 0., command[2])
                    distance = math.dist((x, y), GOAL)
                    log.writerow({'time': now-start, 'state': nav.state, 'reason': nav.reason,
                                  'x': x, 'y': y, 'yaw': yaw,
                                  **dict(zip(('odom_x', 'odom_y', 'odom_yaw'), s['odom'])),
                                  **dict(zip(DIRECTIONS, s['ranges'])),
                                  **dict(zip(('cmd_vx', 'cmd_vy', 'cmd_yaw_rate'), command)),
                                  'heading': math.degrees(nav.heading or 0.), 'free': nav.free,
                                  'goal_distance': distance})
                    if now - last_print > 1.:
                        print(f'{nav.state:9s} ({x:5.2f}, {y:5.2f}) yaw {yaw:6.1f}  '
                              f'{distance:.2f} m to goal  {nav.reason}')
                        last_print = now
                    time.sleep(max(0., .05-(time.monotonic()-now)))
                motion.stop()
                print('Landing:', result)
        except KeyboardInterrupt:
            result = 'interrupted'
            print('Interrupted; landing')
        finally:
            config.stop()
            log.writerow({'state': 'END', 'reason': result})
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('uri', nargs='?', default=DEFAULT_URI)
    parser.add_argument('--goal', nargs=2, type=float, metavar=('X', 'Y'))
    parser.add_argument('--start', nargs=2, type=float, metavar=('X', 'Y'))
    parser.add_argument('--height', type=float)
    parser.add_argument('--arena', nargs=4, type=float, metavar=('X_MIN', 'X_MAX', 'Y_MIN', 'Y_MAX'),
                        help='virtual fence in map meters (default: none)')
    parser.add_argument('--output', default=datetime.now().strftime('flight_log_%Y%m%d_%H%M%S.csv'))
    args = parser.parse_args()
    if args.goal: GOAL = tuple(args.goal)
    if args.start: START = tuple(args.start)
    if args.height: HEIGHT = args.height
    if args.arena: ARENA = tuple(args.arena)
    outcome = fly(args.uri, args.output)
    print('Result:', outcome, '| log:', args.output)
    try:
        from plotter import plot_flight
        print('Plot:', plot_flight(args.output, GOAL))
    except Exception as error:
        print('Plot failed:', error)
    raise SystemExit(0 if outcome == 'goal reached' else 1)
