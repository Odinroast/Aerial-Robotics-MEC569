"""Run: python3 main.py [radio URI]
Offline replay: python3 main.py --replay old_flight_log.csv --output replay.csv
Telemetry callback only copies data. One worker maps/plans; the main loop flies.
"""
import argparse
import csv
import math
import queue
import threading
import time
from collections import Counter
import numpy as np
from slam import GlobalOccupancyGrid, GridSLAM, MAX_RANGE
from planner import Navigator

START = (.3, 0.)
GOAL = (4.35, 0.)
GOAL_AREA = (4.1, 4.6, -.25, .25)
HEIGHT = .8
TURN_EVERY_WAYPOINTS = 3  # Set to 3, 4, etc.; CLI --turn-every overrides this.
DEFAULT_URI = 'radio://0/98/2M/E7E7E7E7E8'
DIRECTIONS = ('front', 'back', 'left', 'right', 'up', 'zrange')


def meters(mm):
    """Firmware range.* values are millimeters. Convert ONCE; keep unknown unknown."""
    value = float(mm)/1000.
    return value if math.isfinite(value) and 0. < value < MAX_RANGE else float('nan')


def save_rows(path, rows):
    if rows:
        with open(path, 'w', newline='') as file:
            writer = csv.DictWriter(file, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)


def replay(path, output):
    """Replay recorded odometry/ranges. This cannot validate true position accuracy."""
    grid, rows, counts = GlobalOccupancyGrid(), [], Counter()
    slam = GridSLAM(grid, START)
    last = -float('inf')
    with open(path, newline='') as file:
        reader = csv.DictReader(line for line in file if not line.startswith('#'))
        for row in reader:
            stamp = float(row['timestamp (ms)'])/1000.
            odom = {'x': float(row.get('raw_X') or row['X']),
                    'y': float(row.get('raw_Y') or row['Y']), 'yaw': float(row.get('raw_Yaw') or row['Yaw'])}
            slam.corrected_pose(odom)
            if row.get('controller_state') == 'GROUND' or stamp-last < .049:
                continue
            # Old CSV range columns are already meters, not millimeters.
            ranges = tuple(float(row[d]) if 0 < float(row[d]) < MAX_RANGE else float('nan')
                           for d in DIRECTIONS[:4])
            slam.update(stamp, odom, ranges)
            pose, diag = slam.corrected_pose(odom), slam.diagnostics()
            counts[diag['slam_state']] += 1
            rows.append({'timestamp (ms)': row['timestamp (ms)'], 'raw_X': odom['x'],
                         'raw_Y': odom['y'], 'raw_Yaw': odom['yaw'],
                         'X': pose['x'], 'Y': pose['y'], 'Yaw': pose['yaw'], **diag})
            last = stamp
    save_rows(output, rows)
    print('Replay SLAM states:', dict(counts))
    print('Final map-to-odometry transform:', slam.diagnostics())
    print('Saved', output)


def fly(uri, output, turn_every=TURN_EVERY_WAYPOINTS, show_plot=True, plot_hz=5.):
    # Import radio dependencies only for flight. Offline tests/replay need no cflib.
    import cflib.crtp
    from cflib.crazyflie import Crazyflie
    from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
    from cflib.crazyflie.log import LogConfig
    from cflib.positioning.motion_commander import MotionCommander

    grid = GlobalOccupancyGrid()
    slam, nav = GridSLAM(grid, START), Navigator(grid, GOAL, turn_every=turn_every)
    samples, stop = queue.Queue(maxsize=1), threading.Event()
    data_lock = threading.Lock()
    latest, worker_state, history = {}, {'heartbeat': time.monotonic(), 'error': None}, []
    live_state = {'command': (0., 0., 0.), 'phase': 'GROUND'}
    plotter = None

    def get_live_sample():
        with data_lock:
            sample = dict(latest)
            if sample:
                sample.update(live_state)
            return sample

    def callback(timestamp, data, _):
        odom = {'x': data['stateEstimate.x'], 'y': data['stateEstimate.y'],
                'yaw': data['stateEstimate.yaw']}
        if not all(math.isfinite(v) for v in odom.values()):
            return                         # Existing telemetry-age check will stop flight.
        sample = {'timestamp': timestamp, 'received': time.monotonic(), 'odom': odom,
                  'ranges': tuple(meters(data['range.'+d]) for d in DIRECTIONS),
                  'raw_ranges': tuple(data['range.'+d] for d in DIRECTIONS)}
        with data_lock:
            latest.clear()
            latest.update(sample)
        # Overwrite old queued samples: never build a backlog behind live flight.
        try:
            samples.get_nowait()
        except queue.Empty:
            pass
        try:
            samples.put_nowait(sample)
        except queue.Full:
            pass

    def worker():
        last_timestamp = None
        try:
            while not stop.is_set():
                try:
                    sample = samples.get(timeout=.1)
                except queue.Empty:
                    continue
                if sample['timestamp'] == last_timestamp:
                    continue
                last_timestamp = sample['timestamp']
                slam.update(sample['received'], sample['odom'], sample['ranges'][:4])
                pose = slam.corrected_pose(sample['odom'])
                nav.update_route(pose)
                with data_lock:
                    worker_state['heartbeat'] = time.monotonic()
        except Exception as error:
            with data_lock:
                worker_state['error'] = str(error)

    cflib.crtp.init_drivers()
    log = LogConfig(name='Navigation', period_in_ms=50)  # 20 Hz, synchronized telemetry.
    for direction in DIRECTIONS:
        log.add_variable('range.'+direction, 'uint16_t')
    for axis in ('x', 'y', 'yaw'):
        log.add_variable('stateEstimate.'+axis, 'float')
    thread = None
    try:
        with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
            scf.cf.log.add_config(log)
            log.data_received_cb.add_callback(callback)
            log.start()
            deadline = time.monotonic()+5.
            while not latest and time.monotonic() < deadline:
                time.sleep(.05)
            with data_lock:
                initial = dict(latest)
            if not initial:
                raise RuntimeError('No telemetry received; flight not started')
            slam.corrected_pose(initial['odom'])  # Anchor map at START while grounded.
            if show_plot:
                from plotter import LivePlotter
                try:
                    plotter = LivePlotter(grid, slam, nav, get_live_sample, START, GOAL, GOAL_AREA,
                                          update_hz=plot_hz)
                    plotter.start()
                except Exception as error:
                    print('Could not start dashboard; flight continues:', error)
            with data_lock:
                live_state['phase'] = 'TAKEOFF'
            scf.cf.platform.send_arming_request(True)
            time.sleep(1.)
            try:
                with MotionCommander(scf, default_height=HEIGHT) as motion:
                    # Start mapping only after takeoff: ground readings do not enter the map.
                    with data_lock:
                        worker_state['heartbeat'] = time.monotonic()
                        live_state['phase'] = 'NAVIGATING'
                    thread = threading.Thread(target=worker, daemon=True)
                    thread.start()
                    mission_start = last_print = time.monotonic()
                    print(f'Navigation started: radius 0.10 m, margin 0.04 m, turn every {turn_every} waypoints')
                    try:
                        while True:
                            now = time.monotonic()
                            with data_lock:
                                sample, health = dict(latest), dict(worker_state)
                            if now-sample['received'] > .5 or now-health['heartbeat'] > 2.:
                                print('Telemetry or SLAM worker stale; landing')
                                motion.stop()
                                break
                            if health['error']:
                                print('Mapping/planning failed:', health['error'])
                                motion.stop()
                                break
                            if now-mission_start > 180.:
                                print('Mission time limit reached; landing')
                                motion.stop()
                                break
                            pose = slam.corrected_pose(sample['odom'])
                            ranges = sample['ranges']
                            if math.isfinite(ranges[4]) and ranges[4] < .20:
                                print('Up-sensor stop; landing')
                                motion.stop()
                                break
                            x0, x1, y0, y1 = GOAL_AREA
                            if x0 <= pose['x'] <= x1 and y0 <= pose['y'] <= y1:
                                print('Entered goal area; landing')
                                motion.stop()
                                break
                            vx, vy, yaw_rate = nav.command(pose, ranges[:4], now)
                            if nav.fault:
                                print(nav.fault+'; landing')
                                motion.stop()
                                break
                            motion.start_linear_motion(vx, vy, 0., yaw_rate)
                            with data_lock:
                                live_state['command'] = (vx, vy, yaw_rate)
                            diag = slam.diagnostics()
                            with nav.lock:
                                state, reason, target = nav.state, nav.reason, nav.target or (None, None)
                                reached, revision, desired = nav.reached, nav.revision, nav.desired_yaw
                            history.append({'timestamp (ms)': sample['timestamp'],
                                            **dict(zip(DIRECTIONS, ranges)),
                                            **dict(zip(('raw_'+d+'_mm' for d in DIRECTIONS), sample['raw_ranges'])),
                                            'X': pose['x'], 'Y': pose['y'], 'Yaw': pose['yaw'],
                                            'raw_X': sample['odom']['x'], 'raw_Y': sample['odom']['y'],
                                            'raw_Yaw': sample['odom']['yaw'], 'controller_state': state,
                                            'stop_reason': reason, 'cmd_vx': vx, 'cmd_vy': vy,
                                            'cmd_yaw_rate': yaw_rate, 'target_X': target[0], 'target_Y': target[1],
                                            'waypoints_reached': reached, 'plan_revision': revision,
                                            'desired_yaw': desired, **diag})
                            if now-last_print > 2.:
                                print(f"{state}: XY=({pose['x']:.2f}, {pose['y']:.2f}), "
                                      f"waypoints={reached}, SLAM={diag['slam_state']}, reason={reason}")
                                last_print = now
                            time.sleep(max(.001, .02-(time.monotonic()-now)))
                    finally:
                        with data_lock:
                            live_state.update(command=(0., 0., 0.), phase='LANDING')
            finally:
                stop.set()
                if thread:
                    thread.join(timeout=3.)
                log.stop()
                with data_lock:
                    live_state['phase'] = 'FINISHED'
    finally:
        if plotter is not None:
            plotter.close()
        save_rows(output, history)            # Save useful diagnostics even after an error.
        np.savez_compressed('occupancy_map.npz', grid=grid.grid, resolution=grid.resolution,
                            bounds=[grid.x_min, grid.x_max, grid.y_min, grid.y_max])
        print('Saved '+output+' and occupancy_map.npz' if history else 'No flight rows; saved occupancy_map.npz')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('uri', nargs='?', default=DEFAULT_URI)
    parser.add_argument('--turn-every', type=int, default=TURN_EVERY_WAYPOINTS,
                        help='Rotate in place every N reached waypoints (positive integer)')
    parser.add_argument('--no-plot', action='store_true', help='Disable the live dashboard')
    parser.add_argument('--plot-hz', type=float, default=5., help='Dashboard refresh rate (default: 5 Hz)')
    parser.add_argument('--replay', help='Existing CSV; no drone/radio connection')
    parser.add_argument('--output', default='flight_log_slam.csv')
    args = parser.parse_args()
    if not math.isfinite(args.plot_hz) or not 0. < args.plot_hz <= 30.:
        parser.error("--plot-hz must be greater than 0 and at most 30")
    if args.turn_every < 1:
        parser.error("--turn-every must be at least 1")
    if args.replay:
        replay(args.replay, args.output)
    else:
        fly(args.uri, args.output, turn_every=args.turn_every,
            show_plot=not args.no_plot, plot_hz=args.plot_hz)
