"""Live dashboard, isolated from flight commands.
A background thread copies snapshots; a separate process owns Matplotlib's GUI
main thread. Closing the dashboard does not stop the drone or its controller.
"""
from collections import deque
import math
import multiprocessing as mp
import queue
import threading
import time
import numpy as np
from slam import DRONE_RADIUS, CLEARANCE, OCCUPIED, ANGLES, rotate


class LivePlotter:
    def __init__(self, grid, slam, navigator, get_sample, start, goal, goal_area,
                 update_hz=5., save_path='flight_live_plot.png'):
        self.grid, self.slam, self.nav, self.get_sample = grid, slam, navigator, get_sample
        self.period = 1./update_hz
        self.config = {'bounds': (grid.x_min, grid.x_max, grid.y_min, grid.y_max),
                       'resolution': grid.resolution, 'start': start, 'goal': goal,
                       'goal_area': goal_area, 'save_path': save_path, 'update_hz': update_hz}
        context = mp.get_context('spawn')  # Do not fork a running radio/logging thread.
        self.messages, self.finished = context.Queue(maxsize=1), context.Event()
        self.process = context.Process(target=_plot_window,
                                       args=(self.messages, self.finished, self.config), daemon=True)
        self.stopped, self.thread = threading.Event(), None
        self.odom_anchor = None
        self.started = time.monotonic()

    def start(self):
        self.process.start()
        self.thread = threading.Thread(target=self._publish, name='plot-snapshots', daemon=True)
        self.thread.start()

    def snapshot(self):
        """Only the display thread copies the map; the flight loop never renders."""
        sample = self.get_sample()
        pose = raw_aligned = None
        if sample:
            odom = sample['odom']
            pose = self.slam.corrected_pose(odom)
            if self.odom_anchor is None:
                with self.slam.lock:
                    self.odom_anchor = self.slam.angle, self.slam.translation.copy()
            angle, translation = self.odom_anchor
            xy = rotate([odom['x'], odom['y']], angle)+translation
            raw_aligned = (float(xy[0]), float(xy[1]))
        with self.grid.lock:
            logodds, blocked = self.grid.grid.copy(), self.grid.costmap().copy()
        with self.nav.lock:
            navigation = {'route': list(self.nav.route), 'index': self.nav.index,
                          'target': self.nav.target, 'state': self.nav.state,
                          'reason': self.nav.reason, 'reached': self.nav.reached,
                          'revision': self.nav.revision, 'turn_every': self.nav.turn_every,
                          'desired_yaw': self.nav.desired_yaw, 'fault': self.nav.fault}
        return {'time': time.monotonic()-self.started, 'pose': pose, 'odom_aligned': raw_aligned,
                'sample': sample, 'grid': logodds, 'blocked': blocked,
                'navigation': navigation, 'slam': self.slam.diagnostics()}

    def _publish(self):
        try:
            while not self.stopped.is_set() and self.process.is_alive():
                snapshot = self.snapshot()
                # A full queue means the GUI is slow. Drop that update, never wait.
                try:
                    self.messages.put_nowait(snapshot)
                except queue.Full:
                    pass
                self.stopped.wait(self.period)
        except Exception as error:
            print('Dashboard updates disabled:', error)

    def close(self):
        self.stopped.set()
        if self.thread:
            self.thread.join(timeout=1.)
        if self.process.is_alive():
            try:
                while True:
                    self.messages.get_nowait()
            except queue.Empty:
                pass
            try:
                self.messages.put(self.snapshot(), timeout=.2)  # Final frame; flight already ended.
            except (queue.Full, ValueError, OSError):
                pass
        self.finished.set()
        if self.process.pid is not None:
            self.process.join(timeout=3.)
            if self.process.is_alive():
                self.process.terminate()  # Only the display process, after flight cleanup.
                self.process.join(timeout=1.)
        self.messages.cancel_join_thread()
        self.messages.close()


class Dashboard:
    """All artists are created/updated in ONE rendering process/thread."""
    def __init__(self, config):
        import matplotlib.pyplot as plt
        from matplotlib.colors import ListedColormap
        from matplotlib.collections import LineCollection
        from matplotlib.patches import Circle, FancyArrowPatch, Rectangle
        self.plt, self.config = plt, config
        self.figure = plt.figure(figsize=(14, 8), layout='constrained')
        spec = self.figure.add_gridspec(2, 2, width_ratios=(2.1, 1.), height_ratios=(3., 1.))
        self.map_ax = self.figure.add_subplot(spec[0, 0])
        self.info_ax = self.figure.add_subplot(spec[0, 1])
        self.motion_ax = self.figure.add_subplot(spec[1, 0])
        self.range_ax = self.figure.add_subplot(spec[1, 1])
        x0, x1, y0, y1 = config['bounds']
        self.map_ax.set(xlim=(x0, x1), ylim=(y0, y1), xlabel='X (m)', ylabel='Y (m)',
                        title='Live map and global route')
        self.map_ax.set_aspect('equal', adjustable='box')
        self.map_ax.axhline((y0+y1)/2., color='#a1a1aa', linestyle=':', linewidth=1.)
        self.map_ax.grid(alpha=.18)
        self.map_image = self.map_ax.imshow(np.ones((2, 2)), origin='lower',
                                           extent=(x0, x1, y0, y1), vmin=0, vmax=2,
                                           cmap=ListedColormap(['#ffffff', '#d4d4d8', '#20242c']), zorder=0)
        self.clearance_image = self.map_ax.imshow(np.zeros((2, 2)), origin='lower',
                                                extent=(x0, x1, y0, y1), vmin=0, vmax=1,
                                                cmap=ListedColormap(['#00000000', '#f59e0b50']), zorder=1)
        gx0, gx1, gy0, gy1 = config['goal_area']
        self.map_ax.add_patch(Rectangle((gx0, gy0), gx1-gx0, gy1-gy0,
                                       facecolor='#22c55e25', edgecolor='#16a34a', linewidth=2., label='Landing region'))
        self.map_ax.plot(*config['start'], marker='s', color='#64748b', label='Start')
        self.map_ax.plot(*config['goal'], marker='*', markersize=13, color='#16a34a', label='Goal')
        self.sensor_rays = LineCollection([], colors='#475569', linewidths=1., linestyles=':', alpha=.5)
        self.map_ax.add_collection(self.sensor_rays)
        self.route_line, = self.map_ax.plot([], [], '--o', color='#16a34a', markersize=3, label='Remaining route')
        self.trail_line, = self.map_ax.plot([], [], color='#dc2626', linewidth=1.5, label='SLAM trajectory')
        self.odom_line, = self.map_ax.plot([], [], ':', color='#6366f1', label='Odometry, initially aligned')
        self.target_dot, = self.map_ax.plot([], [], 'x', color='#0284c7', markersize=9, label='Lookahead target')
        self.drone = Circle(config['start'], DRONE_RADIUS, edgecolor='#dc2626', facecolor='#dc262630', linewidth=2., zorder=5)
        self.margin = Circle(config['start'], CLEARANCE, edgecolor='#f59e0b', fill=False, linestyle=':', zorder=4)
        self.heading = FancyArrowPatch(config['start'], config['start'], arrowstyle='->', mutation_scale=14,
                                      color='#dc2626', linewidth=2., zorder=6)
        self.velocity = FancyArrowPatch(config['start'], config['start'], arrowstyle='->', mutation_scale=14,
                                       color='#0284c7', linewidth=2., zorder=6)
        for artist in (self.drone, self.margin, self.heading, self.velocity):
            self.map_ax.add_patch(artist)
        self.map_ax.legend(loc='upper left', fontsize=7, ncol=2)
        self.info_ax.axis('off')
        self.info_ax.set_title('Navigation and localization', loc='left')
        self.info_text = self.info_ax.text(0., 1., 'Waiting for telemetry...', va='top',
                                          family='monospace', fontsize=9, transform=self.info_ax.transAxes)
        self.motion_ax.set(xlabel='Elapsed time (s)', ylabel='Body velocity (m/s)', ylim=(-.3, .3))
        self.motion_ax.grid(alpha=.25)
        self.yaw_ax = self.motion_ax.twinx()
        self.yaw_ax.set(ylabel='Yaw command (deg/s)', ylim=(-50, 50))
        self.vx_line, = self.motion_ax.plot([], [], color='#0284c7', label='vx')
        self.vy_line, = self.motion_ax.plot([], [], color='#16a34a', label='vy')
        self.yaw_line, = self.yaw_ax.plot([], [], color='#d97706', label='yaw rate')
        self.motion_ax.legend(handles=[self.vx_line, self.vy_line, self.yaw_line], loc='upper left', fontsize=8)
        self.range_ax.set(title='Ranges (m); ? = unknown', ylim=(0., 4.))
        self.bars = self.range_ax.bar(['Front', 'Back', 'Left', 'Right', 'Up', 'Down'], [0.]*6, color='#0284c7')
        self.range_labels = [self.range_ax.text(i, .08, '?', ha='center', fontsize=8) for i in range(6)]
        self.range_ax.tick_params(axis='x', labelsize=8)
        self.trail, self.odom_trail, self.commands = deque(maxlen=5000), deque(maxlen=5000), deque(maxlen=600)
        self.last_timestamp = None

    def update(self, snapshot):
        grid, blocked = snapshot['grid'], snapshot['blocked']
        x0, _, y0, _ = self.config['bounds']
        res = self.config['resolution']
        extent = (x0, x0+grid.shape[0]*res, y0, y0+grid.shape[1]*res)
        cells = np.where(grid >= OCCUPIED, 2, np.where(grid < 0., 0, 1))
        self.map_image.set_data(cells.T)
        self.map_image.set_extent(extent)
        self.clearance_image.set_data((blocked & (grid < OCCUPIED)).T)
        self.clearance_image.set_extent(extent)
        nav, diag, sample, pose = snapshot['navigation'], snapshot['slam'], snapshot['sample'], snapshot['pose']
        route = nav['route'][max(0, nav['index']-1):]
        self.route_line.set_data(*zip(*route)) if route else self.route_line.set_data([], [])
        target = nav['target']
        self.target_dot.set_data([target[0]], [target[1]]) if target else self.target_dot.set_data([], [])
        command = sample.get('command', (0., 0., 0.)) if sample else (0., 0., 0.)
        ranges = sample.get('ranges', (float('nan'),)*6) if sample else (float('nan'),)*6
        if pose:
            position = (pose['x'], pose['y'])
            self.drone.center = self.margin.center = position
            angle = math.radians(pose['yaw'])
            c, s = math.cos(angle), math.sin(angle)
            rays = []
            for distance, offset in zip(ranges[:4], ANGLES):
                if distance is not None and math.isfinite(distance):
                    rays.append([position, (position[0]+distance*math.cos(angle+offset),
                                            position[1]+distance*math.sin(angle+offset))])
            self.sensor_rays.set_segments(rays)
            self.heading.set_positions(position, (position[0]+.3*c, position[1]+.3*s))
            vx, vy, _ = command
            self.velocity.set_positions(position, (position[0]+.8*(vx*c-vy*s), position[1]+.8*(vx*s+vy*c)))
            if sample['timestamp'] != self.last_timestamp:
                self.trail.append(position)
                self.odom_trail.append(snapshot['odom_aligned'])
                self.last_timestamp = sample['timestamp']
            self.trail_line.set_data(*zip(*self.trail))
            self.odom_line.set_data(*zip(*self.odom_trail))
        self.commands.append((snapshot['time'], *command))
        t, vx, vy, yaw = np.array(self.commands).T
        self.vx_line.set_data(t, vx)
        self.vy_line.set_data(t, vy)
        self.yaw_line.set_data(t, yaw)
        self.motion_ax.set_xlim(max(0., t[-1]-30.), max(30., t[-1]+.1))
        for bar, label, value in zip(self.bars, self.range_labels, ranges):
            known = value is not None and math.isfinite(value)
            bar.set_height(value if known else 0.)
            label.set_position((bar.get_x()+bar.get_width()/2, min(3.85, (value if known else 0.)+.08)))
            label.set_text(f'{value:.2f}' if known else '?')
        fmt = lambda value: '—' if value is None else f'{value:.2f}'
        age = time.monotonic()-sample['received'] if sample else float('nan')
        text = [f"Phase: {sample.get('phase', 'WAITING') if sample else 'WAITING'}",
                f"Controller: {nav['state']}", f"Reason: {nav['reason'] or '—'}",
                f"Pose X/Y: {fmt(pose['x'] if pose else None)} / {fmt(pose['y'] if pose else None)} m",
                f"Yaw: {fmt(pose['yaw'] if pose else None)} deg",
                f"Desired yaw: {fmt(nav['desired_yaw'])} deg", '',
                f"Reached waypoints: {nav['reached']}", f"Turn every: {nav['turn_every']} waypoints",
                f"Route revision: {nav['revision']}",
                f"Target: ({target[0]:.2f}, {target[1]:.2f}) m" if target else 'Target: —', '',
                f"Command vx/vy: {command[0]:.2f} / {command[1]:.2f} m/s",
                f"Command yaw: {command[2]:.1f} deg/s", '',
                f"SLAM: {diag['slam_state']}", f"Inliers: {100*diag['inliers']:.0f}%",
                f"Match residual: {diag['match_error']:.3f} m" if math.isfinite(diag['match_error']) else 'Match residual: —',
                f"Accepted full/partial: {diag['full_updates']}/{diag['partial_updates']}",
                f"Map/odom XY: {diag['tx']:.3f} / {diag['ty']:.3f} m",
                f"Yaw offset: {diag['yaw_offset']:.2f} deg", '',
                f"Telemetry age: {age:.2f} s", f"Radius/margin: {DRONE_RADIUS:.2f} / {CLEARANCE-DRONE_RADIUS:.2f} m",
                'White=free; gray=unknown; black=occupied', 'Amber=footprint clearance',
                f"Fault: {nav['fault'] or '—'}"]
        self.info_text.set_text('\n'.join(text))
        self.figure.canvas.draw_idle()


def _plot_window(messages, finished, config):
    """Spawned process entry point: its main thread owns every GUI call."""
    import signal
    signal.signal(signal.SIGINT, signal.SIG_IGN)  # Parent handles Ctrl+C and landing.
    dashboard = None
    try:
        import matplotlib
        dashboard = Dashboard(config)
        def refresh():
            latest = None
            try:
                while True:
                    latest = messages.get_nowait()
            except queue.Empty:
                pass
            if latest is None and finished.is_set():
                try:
                    latest = messages.get(timeout=.2)  # Allow final queued bytes to arrive.
                except queue.Empty:
                    pass
            if latest is not None:
                dashboard.update(latest)
            if finished.is_set():
                dashboard.figure.savefig(config['save_path'], dpi=150)
                dashboard.plt.close(dashboard.figure)
                return False
            return True
        backend = matplotlib.get_backend().lower()
        if backend == 'agg':
            print('No interactive Matplotlib backend; saving dashboard image at shutdown.')
            while not finished.wait(1./config['update_hz']):
                refresh()
            refresh()
        else:
            timer = dashboard.figure.canvas.new_timer(interval=round(1000/config['update_hz']))
            timer.add_callback(refresh)
            timer.start()
            dashboard.plt.show()            # GUI event loop, isolated from flight.
    except Exception as error:
        print('Dashboard disabled; flight continues:', error)
    finally:
        if dashboard is not None:
            try:
                dashboard.figure.savefig(config['save_path'], dpi=150)
                dashboard.plt.close(dashboard.figure)
            except Exception as error:
                print('Could not save dashboard:', error)
