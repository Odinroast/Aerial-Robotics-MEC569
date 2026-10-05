"""
Crazyflie collision-avoidance flight (Flow deck v2 + Multi-ranger deck).

Modes
    python crazyflie_avoid.py            # 'goal' (default): fly to a point, steering around obstacles
    python crazyflie_avoid.py push       # hover in place and move away from anything that approaches
    python crazyflie_avoid.py takeoff    # plain take-off / hover / land test

Safety behaviour in every mode
    - aborts if either deck is missing, the battery is low, or the estimator does not converge
    - lands immediately if something is detected above the drone
    - lands after MAX_FLIGHT_TIME seconds or on Ctrl-C
"""

import logging
import math
import sys
import time
from threading import Event, Lock

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils import uri_helper
from cflib.utils.multiranger import Multiranger

# ----------------------------------------------------------------------------
URI = uri_helper.uri_from_env(default='radio://0/98/2M/E7E7E7E7E8')

DEFAULT_HEIGHT = 0.4       # flight height [m]
MIN_BATTERY_V = 3.6        # refuse to fly below this [V]
MAX_FLIGHT_TIME = 30.0     # hard limit per flight [s]
LOOP_DT = 0.05             # control period [s]

# goal mode
GOAL_X = 2.0               # target, relative to the take-off point [m]
GOAL_Y = 0.0
GOAL_TOL = 0.10            # arrival radius [m]
V_MAX = 0.3                # max horizontal speed [m/s]
K_ATTRACT = 0.8            # proportional gain toward the goal [1/s]

# obstacle handling
SAFE_DIST = 0.50           # start reacting at this range [m]
STOP_DIST = 0.25           # never move toward an obstacle closer than this [m]
K_REPEL = 0.6              # repulsion speed at STOP_DIST [m/s]
CEILING_DIST = 0.20        # land if something is this close above [m]

logging.basicConfig(level=logging.ERROR)

# ----------------------------------------------------------------------------
# Deck detection
flow_event = Event()
ranger_event = Event()


def _deck_cb(event, name):
    def cb(_, value_str):
        if int(value_str):
            event.set()
            print(f'{name} attached.')
        else:
            print(f'{name} NOT attached.')
    return cb


# ----------------------------------------------------------------------------
# Pre-flight checks
def battery_voltage(scf):
    conf = LogConfig(name='Battery', period_in_ms=100)
    conf.add_variable('pm.vbat', 'float')
    with SyncLogger(scf, conf) as logger:
        for entry in logger:
            return entry[1]['pm.vbat']


def wait_for_position_estimator(scf, threshold=0.001, timeout=10.0):
    conf = LogConfig(name='Kalman Variance', period_in_ms=100)
    for a in ('X', 'Y', 'Z'):
        conf.add_variable(f'kalman.var{a}', 'float')
    hist = {a: [1000.0] * 10 for a in ('X', 'Y', 'Z')}
    start = time.time()
    with SyncLogger(scf, conf) as logger:
        for entry in logger:
            for a in ('X', 'Y', 'Z'):
                hist[a].append(entry[1][f'kalman.var{a}'])
                hist[a].pop(0)
            if all(max(h) - min(h) < threshold for h in hist.values()):
                return True
            if time.time() - start > timeout:
                return False


def reset_estimator(scf):
    scf.cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    scf.cf.param.set_value('kalman.resetEstimation', '0')
    return wait_for_position_estimator(scf)


def arm(scf, state=True):
    try:
        scf.cf.supervisor.send_arming_request(state)
        time.sleep(1.0 if state else 0.1)
    except AttributeError:        # firmware without the supervisor arming API
        pass


# ----------------------------------------------------------------------------
# Position feedback (asynchronous log)
class PositionTracker:
    def __init__(self, scf):
        self._lock = Lock()
        self._pos = (0.0, 0.0, 0.0)
        self._conf = LogConfig(name='Position', period_in_ms=50)
        for v in ('stateEstimate.x', 'stateEstimate.y', 'stateEstimate.z'):
            self._conf.add_variable(v, 'float')
        scf.cf.log.add_config(self._conf)
        self._conf.data_received_cb.add_callback(self._cb)

    def _cb(self, _ts, data, _conf):
        with self._lock:
            self._pos = (data['stateEstimate.x'],
                         data['stateEstimate.y'],
                         data['stateEstimate.z'])

    def start(self):
        self._conf.start()

    def stop(self):
        self._conf.stop()

    @property
    def position(self):
        with self._lock:
            return self._pos


# ----------------------------------------------------------------------------
# Avoidance logic
def _clip(v, lim):
    return max(-lim, min(lim, v))


def _free(d):
    """Multi-ranger returns None when nothing is in range."""
    return d is None or d > SAFE_DIST


def _repel(d):
    """Speed pushing away from an obstacle at distance d (0 when clear)."""
    if _free(d):
        return 0.0
    d = max(d, 1e-3)
    return K_REPEL * (SAFE_DIST - d) / (SAFE_DIST - STOP_DIST) if d > STOP_DIST else K_REPEL


def avoidance_velocity(mr, vx_des, vy_des):
    """
    Combine a desired body-frame velocity with repulsion from the four side
    sensors. Motion toward any obstacle closer than STOP_DIST is blocked, and
    if the front is blocked while the goal is ahead the drone sidesteps toward
    the clearer side so it can get around.
    """
    front, back, left, right = mr.front, mr.back, mr.left, mr.right

    vx = vx_des - _repel(front) + _repel(back)
    vy = vy_des - _repel(left) + _repel(right)

    # sidestep around a frontal obstacle
    if not _free(front) and vx_des > 0:
        clear_l = left if left is not None else 4.0
        clear_r = right if right is not None else 4.0
        vy += V_MAX if clear_l >= clear_r else -V_MAX

    # hard block: never close on something already too near
    if front is not None and front < STOP_DIST: vx = min(vx, 0.0)
    if back  is not None and back  < STOP_DIST: vx = max(vx, 0.0)
    if left  is not None and left  < STOP_DIST: vy = min(vy, 0.0)
    if right is not None and right < STOP_DIST: vy = max(vy, 0.0)

    # keep total speed under V_MAX
    speed = math.hypot(vx, vy)
    if speed > V_MAX:
        vx, vy = vx * V_MAX / speed, vy * V_MAX / speed
    return vx, vy


def ceiling_close(mr):
    return mr.up is not None and mr.up < CEILING_DIST


# ----------------------------------------------------------------------------
# Flight modes
def fly_to_goal(scf, tracker):
    with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc, Multiranger(scf) as mr:
        time.sleep(1.0)
        x0, y0, _ = tracker.position
        gx, gy = x0 + GOAL_X, y0 + GOAL_Y
        t0 = time.time()

        last_mr_down = mr.down
        box_detected = False
        while True:
            if ceiling_close(mr):
                print('Obstacle above - landing.')
                break
            if time.time() - t0 > MAX_FLIGHT_TIME:
                print('Time limit reached - landing.')
                break

            if last_mr_down - mr.down > 0.3: #Lowkey forgot what units lol
                print('Box detected below - locating.')
                box_detected = True
                break

            x, y, _ = tracker.position
            ex, ey = gx - x, gy - y
            if math.hypot(ex, ey) < GOAL_TOL:
                print('Goal reached.')
                break

            vx_des = _clip(K_ATTRACT * ex, V_MAX)
            vy_des = _clip(K_ATTRACT * ey, V_MAX)
            vx, vy = avoidance_velocity(mr, vx_des, vy_des)
            mc.start_linear_motion(vx, vy, 0.0)
            last_mr_down = mr.down
            time.sleep(LOOP_DT)

        if box_detected: # Simple option
            print('Box detected below - landing.')
            mc.forward(0.2)
            while True:
                mc.start_linear_motion(0.5, 0.0, 0.0) # I believe X is to the right
                if mr.down - last_mr_down  < 0.3:
                    mc.left(0.2)
                    break

        mc.stop()
        time.sleep(0.5)


def push_away(scf):
    with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc, Multiranger(scf) as mr:
        time.sleep(1.0)
        t0 = time.time()
        while time.time() - t0 < MAX_FLIGHT_TIME:
            if ceiling_close(mr):
                print('Hand above - landing.')
                break
            vx, vy = avoidance_velocity(mr, 0.0, 0.0)
            mc.start_linear_motion(vx, vy, 0.0)
            time.sleep(LOOP_DT)
        mc.stop()
        time.sleep(0.5)


def take_off_simple(scf):
    with MotionCommander(scf, default_height=DEFAULT_HEIGHT):
        time.sleep(3)


# ----------------------------------------------------------------------------
if __name__ == '__main__':
    mode = sys.argv[1] if len(sys.argv) > 1 else 'goal'
    if mode not in ('goal', 'push', 'takeoff'):
        print(__doc__)
        sys.exit(1)

    cflib.crtp.init_drivers()
    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        scf.cf.param.add_update_callback(group='deck', name='bcFlow2',
                                         cb=_deck_cb(flow_event, 'Flow deck'))
        scf.cf.param.add_update_callback(group='deck', name='bcMultiranger',
                                         cb=_deck_cb(ranger_event, 'Multi-ranger deck'))

        if not flow_event.wait(timeout=5):
            print('No flow deck - aborting.')
            sys.exit(1)
        if mode != 'takeoff' and not ranger_event.wait(timeout=5):
            print('No Multi-ranger deck - aborting.')
            sys.exit(1)

        vbat = battery_voltage(scf)
        print(f'Battery: {vbat:.2f} V')
        if vbat < MIN_BATTERY_V:
            print('Battery too low - aborting.')
            sys.exit(1)

        if not reset_estimator(scf):
            print('Position estimate did not converge - aborting.')
            sys.exit(1)

        tracker = PositionTracker(scf)
        tracker.start()
        arm(scf, True)
        try:
            if mode == 'goal':
                fly_to_goal(scf, tracker)
            elif mode == 'push':
                push_away(scf)
            else:
                take_off_simple(scf)
        except KeyboardInterrupt:
            print('Interrupted - landing.')      # MotionCommander lands on exit
        finally:
            tracker.stop()
            scf.cf.commander.send_stop_setpoint()
            arm(scf, False)
