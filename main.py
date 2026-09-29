"""
Main entry point for Crazyflie autonomous navigation and goal-area sweep.
Uses a 10 Hz background global planner and 50 Hz reactive local controller from planner.py.
"""
import csv  
import logging
import sys
import time
import math
import queue
from enum import Enum, auto
from collections import deque
from math import sin, cos, radians

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils.multiranger import Multiranger
from cflib.utils import uri_helper
from cflib.crazyflie.log import LogConfig
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch
from matplotlib.patches import Rectangle

from slam import GlobalOccupancyGrid
from planner import NavigationManager

# --- Global Variables & Parameters ---
log_data = deque(maxlen=100)
csv_history = []
map_len, map_width = (2, 2)
goal_len = 0.25  # Length of goal spaces

plot_queue = queue.Queue()
URI = uri_helper.uri_from_env(default='radio://0/98/2M/E7E7E7E7E8')
if len(sys.argv) > 1:
    URI = sys.argv[1]

logging.basicConfig(level=logging.ERROR)
start_x, start_y = 0.3, 0.0  # Drone starts inside Start Area

# Goal Region Definitions [x_min, x_max, y_min, y_max]
START_GOAL_AREA = [0.0, goal_len, -map_width / 2, map_width / 2]
TARGET_GOAL_AREA = [map_len - goal_len, map_len, -goal_len / 2, goal_len / 2]
TARGET_CENTER = (map_len - (goal_len / 2), 0.0)

# Thread-safe dictionary storing latest pose
latest_pose = {'x': start_x, 'y': start_y, 'yaw': 0.0}

class MissionState(Enum):
    NAVIGATING = auto()
    SWEEPING = auto()
    FINISHED = auto()

# --- Initialize Global Occupancy Grid Map ---
global_map = GlobalOccupancyGrid(
    map_x_min=0.0, 
    map_x_max=map_len, 
    map_y_min=-map_width / 2, 
    map_y_max=map_width / 2, 
    resolution=0.02
)

# --- Matplotlib Setup ---
# --- Matplotlib Setup ---
plt.ion()  
fig, ax = plt.subplots(figsize=(8, 8))

drone_path_line, = ax.plot([], [], 'ro-', label='Drone Path', markersize=4)
lidar_scatter, = ax.plot([], [], 'bx', label='Lidar Hits', markersize=3)

# Plot line for high-level global path dots
path_dots_line, = ax.plot([], [], 'go--', label='Global Path Dots', markersize=5, linewidth=1.5)

# Draw Start Area and Target Goal Area Rectangles
start_box = Rectangle(
    (START_GOAL_AREA[0], START_GOAL_AREA[2]),
    START_GOAL_AREA[1] - START_GOAL_AREA[0],
    START_GOAL_AREA[3] - START_GOAL_AREA[2],
    linewidth=1.5, edgecolor='gray', facecolor='lightgray', alpha=0.3, label='Start Zone'
)
target_box = Rectangle(
    (TARGET_GOAL_AREA[0], TARGET_GOAL_AREA[2]),
    TARGET_GOAL_AREA[1] - TARGET_GOAL_AREA[0],
    TARGET_GOAL_AREA[3] - TARGET_GOAL_AREA[2],
    linewidth=1.5, edgecolor='green', facecolor='lightgreen', alpha=0.3, label='Target Goal Zone'
)
ax.add_patch(start_box)
ax.add_patch(target_box)

heading_arrow = FancyArrowPatch(
    (0, 0), (0, 0),
    color='red',
    arrowstyle='->,head_width=4,head_length=8',
    mutation_scale=10,
    linewidth=2,
    label='Heading'
)
ax.add_patch(heading_arrow)

ax.set_xlim(0, map_len)
ax.set_ylim(-map_width/2, map_width/2)
ax.set_xlabel('X Position (m)')
ax.set_ylabel('Y Position (m)')
ax.set_title('Real-Time Hardware Mapping & Autonomous Planning')
ax.grid(True)
ax.legend(loc='upper right')

drone_x_hist, drone_y_hist = [], []
ARROW_LENGTH = 0.4 

def process_plot_updates(current_dots=None):
    updated = False
    latest_x, latest_y, latest_yaw = None, None, None

    # Empty queue to get latest drone pose
    while not plot_queue.empty():
        x, y, yaw = plot_queue.get_nowait()
        drone_x_hist.append(x)
        drone_y_hist.append(y)
        latest_x, latest_y, latest_yaw = x, y, yaw
        updated = True

    # NEW: Update the visual global path line whenever dots exist
    if current_dots:
        dots_x = [pt[0] for pt in current_dots]
        dots_y = [pt[1] for pt in current_dots]
        path_dots_line.set_data(dots_x, dots_y)
    else:
        path_dots_line.set_data([], [])

    if updated and latest_x is not None:
        # Update drone path history
        drone_path_line.set_data(drone_x_hist, drone_y_hist)
        
        # Extract obstacle coordinates from grid
        obstacle_pts = global_map.get_occupied_points(threshold=1.0)
        if len(obstacle_pts) > 0:
            lidar_scatter.set_data(obstacle_pts[:, 0], obstacle_pts[:, 1])
        else:
            lidar_scatter.set_data([], [])

        # Update heading arrow position
        rad = radians(latest_yaw)
        dx = ARROW_LENGTH * cos(rad)
        dy = ARROW_LENGTH * sin(rad)
        heading_arrow.set_positions((latest_x, latest_y), (latest_x + dx, latest_y + dy))
        
        fig.canvas.draw_idle()
        fig.canvas.flush_events()

def generate_concentric_sweep(area_bounds, step_margin=0.2):
    """Generates sequential waypoints sweeping a rectangle from outside to inside."""
    x_min, x_max, y_min, y_max = area_bounds
    waypoints = []
    
    while (x_min < x_max) and (y_min < y_max):
        waypoints.extend([
            (x_min, y_min),
            (x_max, y_min),
            (x_max, y_max),
            (x_min, y_max),
            (x_min, y_min + step_margin)
        ])
        x_min += step_margin
        x_max -= step_margin
        y_min += step_margin
        y_max -= step_margin
        
    return waypoints


def log_stab_callback(timestamp, data, logconf):
    F_d = data['range.front'] / 1000
    B_d = data['range.back'] / 1000
    L_d = data['range.left'] / 1000
    R_d = data['range.right'] / 1000
    U_d = data['range.up'] / 1000
    D_d = data['range.zrange'] / 1000
    x = data['stateEstimate.x'] + start_x
    y = data['stateEstimate.y'] + start_y
    deg = data['stateEstimate.yaw']

    # Update local memory cache for planner thread
    latest_pose['x'] = x
    latest_pose['y'] = y
    latest_pose['yaw'] = deg

    parsed_data = [F_d, B_d, L_d, R_d, U_d, D_d, x, y, deg]
    log_data.append(parsed_data)
    csv_history.append([timestamp, F_d, B_d, L_d, R_d, U_d, D_d, x, y, deg])

    global_map.update_map(x, y, deg, ranges=(F_d, B_d, L_d, R_d))
    plot_queue.put((x, y, deg))

# Read raw ranger distances (meters)
def to_meters(mm_val):
    """Converts raw Multiranger readings (mm) to meters. Defaults to 1.0m if None or 0."""
    if mm_val is None or mm_val == 0:
        return 1.0  # Open space / out-of-range default
    return mm_val / 1000.0

if __name__ == '__main__':
    cflib.crtp.init_drivers(enable_debug_driver=False)

    log_config = LogConfig(name='LogData', period_in_ms=10)
    log_config.add_variable('range.front', 'FP16')
    log_config.add_variable('range.back', 'FP16')
    log_config.add_variable('range.left', 'FP16')
    log_config.add_variable('range.right', 'FP16')
    log_config.add_variable('range.zrange', 'FP16')
    log_config.add_variable('range.up', 'FP16')
    log_config.add_variable('stateEstimate.x', 'FP16')
    log_config.add_variable('stateEstimate.y', 'FP16')
    log_config.add_variable('stateEstimate.yaw', 'FP16')
    
    cf = Crazyflie(rw_cache='./cache')
    with SyncCrazyflie(URI, cf=cf) as scf:
        cf = scf.cf
        cf.log.add_config(log_config)
        log_config.data_received_cb.add_callback(log_stab_callback)
        log_config.start()

        scf.cf.platform.send_arming_request(True)
        time.sleep(1.0)
        
        # Instantiate Navigation Manager Thread
        nav_manager = NavigationManager(
            get_pose_func=lambda: dict(latest_pose),
            get_grid_func=lambda: global_map,
            goal=TARGET_CENTER
        )
        nav_manager.start_planner_thread()

        mission_state = MissionState.NAVIGATING
        sweep_waypoints = generate_concentric_sweep(TARGET_GOAL_AREA)
        active_sweep_target = None

        with MotionCommander(scf, default_height=0.3) as motion_commander:
            with Multiranger(scf) as multi_ranger:

                print("Autonomous Flight System Online!")

                while mission_state != MissionState.FINISHED:
                    loop_start = time.time()
                    
                    # Read current path dots from navigation manager
                    with nav_manager.path_lock:
                        dots_to_draw = list(nav_manager.current_dots)

                    # Pass dots to visualization renderer
                    process_plot_updates(current_dots=dots_to_draw)

                    curr_x = latest_pose['x']
                    curr_y = latest_pose['y']

                    # Clean, readable usage in your main loop:
                    ranger_data = (
                        to_meters(multi_ranger.front),
                        to_meters(multi_ranger.back),
                        to_meters(multi_ranger.left),
                        to_meters(multi_ranger.right),
                    )

                    # Hand off flight phase management
                    if mission_state == MissionState.NAVIGATING:
                        # Check if inside target goal region
                        if (TARGET_GOAL_AREA[0] <= curr_x <= TARGET_GOAL_AREA[1] and 
                            TARGET_GOAL_AREA[2] <= curr_y <= TARGET_GOAL_AREA[3]):
                            print("Entered Target Goal Region! Switching to Sweep Mode...")
                            mission_state = MissionState.SWEEPING
                            continue

                        # Compute 50 Hz velocity command toward goal
                        vx, vy, yaw_rate = nav_manager.get_control_command(latest_pose, ranger_data)
                        motion_commander.start_linear_motion(vx, vy, 0.0, yaw_rate)

                    elif mission_state == MissionState.SWEEPING:
                        if active_sweep_target is None and sweep_waypoints:
                            active_sweep_target = sweep_waypoints.pop(0)

                        if active_sweep_target:
                            # Update planner goal to current sweep point
                            nav_manager.goal = active_sweep_target
                            dist_to_sweep_pt = math.hypot(active_sweep_target[0] - curr_x, active_sweep_target[1] - curr_y)

                            if dist_to_sweep_pt < 0.15:
                                active_sweep_target = None  # Reached, advance to next point
                            else:
                                vx, vy, yaw_rate = nav_manager.get_control_command(latest_pose, ranger_data)
                                motion_commander.start_linear_motion(vx, vy, 0.0, yaw_rate)
                        else:
                            print("Area Sweep Complete! Landing...")
                            motion_commander.stop()
                            mission_state = MissionState.FINISHED

                    # Emergency kill switch check (hand directly over top sensor)
                    if multi_ranger.up and multi_ranger.up < 0.2:
                        print("Emergency Up-sensor triggered. Aborting mission!")
                        motion_commander.stop()
                        break

                    # Maintain 50 Hz loop execution
                    elapsed = time.time() - loop_start
                    time.sleep(max(0.005, 0.02 - elapsed))

            nav_manager.stop()
            log_config.stop()
            print('Demo terminated!')

    # --- CSV Export ---
    filename = "flight_log.csv"
    headers = ["timestamp (us)", "front", "back", "left", "right", "up", "zrange", "X", "Y", "Yaw"]
    
    with open(filename, mode="w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["# Metadata"])
        writer.writerow(["# Map Length (m)", map_len])
        writer.writerow(["# Map Width (m)", map_width])
        writer.writerow(["# Start X (m)", start_x])
        writer.writerow(["# Start Y (m)", start_y])
        writer.writerow(["# --- Data Start ---"])
        writer.writerow(headers)
        writer.writerows(csv_history)
        
    print(f"Successfully saved {len(csv_history)} data rows to '{filename}'")

    plt.ioff()
    plt.show()