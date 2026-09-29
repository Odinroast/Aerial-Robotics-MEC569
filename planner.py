"""
planner.py - Two-Tier Navigation Stack for Crazyflie SLAM

Contains:
1. FastRaycasterPlanner: High-level 10 Hz path generator with deadlock recovery.
2. LocalReactiveController: High-rate (50 Hz) motion controller with anti-oscillation
   and 90-degree heading yaw scanning for improved sensor coverage.
"""

import time
import math
import threading
import numpy as np


class FastRaycasterPlanner:
    """
    High-Level Global Planner (10 Hz).
    Generates a list of path 'dots' from current location to goal.
    """
    def __init__(self, step_len=0.25, max_steps=30):
        self.step_len = step_len
        self.max_steps = max_steps
        self.blocked_timer = None
        self.recovery_active = False

    def generate_path_dots(self, start, goal, occupancy_grid):
        """
        Calculates path dots spaced at `step_len` meters toward goal.
        Includes deadlock detection for triggering recovery rotations.
        """
        curr_x, curr_y = start
        goal_x, goal_y = goal
        dots = [(curr_x, curr_y)]

        reached_or_progressed = False

        for _ in range(self.max_steps):
            dist_to_goal = math.hypot(goal_x - curr_x, goal_y - curr_y)
            
            if dist_to_goal <= self.step_len:
                dots.append((goal_x, goal_y))
                reached_or_progressed = True
                break

            angle = math.atan2(goal_y - curr_y, goal_x - curr_x)
            next_x = curr_x + self.step_len * math.cos(angle)
            next_y = curr_y + self.step_len * math.sin(angle)

            # Check for grid collision (threshold 0.7)
            if occupancy_grid.is_occupied(next_x, next_y, threshold=0.7):
                found_detour = False
                # Search surrounding detour angles
                for offset_deg in [45, -45, 90, -90, 135, -135]:
                    alt_angle = angle + math.radians(offset_deg)
                    alt_x = curr_x + self.step_len * math.cos(alt_angle)
                    alt_y = curr_y + self.step_len * math.sin(alt_angle)

                    if not occupancy_grid.is_occupied(alt_x, alt_y, threshold=0.7):
                        next_x, next_y = alt_x, alt_y
                        found_detour = True
                        break

                if not found_detour:
                    # Path is completely blocked
                    break

            dots.append((next_x, next_y))
            curr_x, curr_y = next_x, next_y
            reached_or_progressed = True

        # --- Problem 3 Fix: Deadlock Recovery Logic ---
        if not reached_or_progressed or len(dots) <= 1:
            if self.blocked_timer is None:
                self.blocked_timer = time.time()
            elif time.time() - self.blocked_timer > 3.0: # Blocked > 3 seconds
                self.recovery_active = True
        else:
            self.blocked_timer = None
            self.recovery_active = False

        return dots, self.recovery_active


class LocalReactiveController:
    """
    Low-Level Reactive Controller (50 Hz).
    Implements a strict 'Stop -> Scan Yaw -> Realign -> Resume' sequence at discrete waypoints.
    """
    def __init__(self, target_speed=0.20, min_safety_dist=0.25, lookahead_dist=0.35, 
                 scan_interval=2, yaw_tolerance_deg=8.0):
        self.target_speed = target_speed
        self.min_safety_dist = min_safety_dist
        self.lookahead_dist = lookahead_dist
        self.scan_interval = scan_interval          # 1 = Every waypoint, 2 = Every second
        self.yaw_tolerance_deg = yaw_tolerance_deg  # Alignment threshold in degrees

        # Hysteresis / Anti-Oscillation Memory
        self.last_avoid_dir = 0
        self.avoid_lock_time = 0.0

        # State Machine Flags
        self.last_waypoint_idx = -1
        self.waypoint_visit_count = 0
        self.scan_phase = 1.0  # Alternates +1 (+90 deg) and -1 (-90 deg)
        
        # Internal States: 'NAVIGATING', 'SCANNING', 'REALIGNING'
        self.state = 'NAVIGATING'
        self.target_yaw_angle = 0.0

    def compute_motion(self, current_pose, path_dots, ranger_data, recovery_flag=False):
        cx, cy, cyaw = current_pose['x'], current_pose['y'], current_pose['yaw']
        now = time.time()

        # Handle emergency recovery spin
        if recovery_flag:
            self.state = 'NAVIGATING'
            return 0.0, 0.0, 30.0

        if len(path_dots) < 2:
            return 0.0, 0.0, 0.0

        # --- Dynamic Lookahead Target Selection ---
        target_dot = path_dots[-1]
        active_idx = len(path_dots) - 1

        for i, dot in enumerate(path_dots):
            dist = math.hypot(dot[0] - cx, dot[1] - cy)
            if dist >= self.lookahead_dist:
                target_dot = dot
                active_idx = i
                break

        # Line-of-flight heading toward lookahead dot
        travel_angle_deg = math.degrees(math.atan2(target_dot[1] - cy, target_dot[0] - cx))

        # --- Waypoint Arrival Detector ---
        if active_idx != self.last_waypoint_idx:
            self.last_waypoint_idx = active_idx
            self.waypoint_visit_count += 1

            # Check if this waypoint triggers a scan pause
            if self.waypoint_visit_count % self.scan_interval == 0:
                self.state = 'SCANNING'
                self.scan_phase *= -1.0  # Alternate scan side
                # Freeze target scan heading relative to initial travel angle
                self.target_yaw_angle = (travel_angle_deg + (90.0 * self.scan_phase)) % 360.0

        # =========================================================================
        # STATE 1: SCANNING (Rotate to +90 / -90 degrees in place)
        # =========================================================================
        if self.state == 'SCANNING':
            yaw_err = (self.target_yaw_angle - cyaw + 180) % 360 - 180
            yaw_rate = float(np.clip(yaw_err * 1.5, -45.0, 45.0))

            if abs(yaw_err) > self.yaw_tolerance_deg:
                return 0.0, 0.0, yaw_rate  # Zero translation while scanning
            else:
                # Target scan angle reached -> Transition to realigning phase
                self.state = 'REALIGNING'

        # =========================================================================
        # STATE 2: REALIGNING (Rotate back to face line-of-flight)
        # =========================================================================
        if self.state == 'REALIGNING':
            flight_yaw_err = (travel_angle_deg - cyaw + 180) % 360 - 180
            yaw_rate = float(np.clip(flight_yaw_err * 1.5, -45.0, 45.0))

            if abs(flight_yaw_err) > self.yaw_tolerance_deg:
                return 0.0, 0.0, yaw_rate  # Zero translation while re-aligning
            else:
                # Fully aligned with flight path -> Resume normal flight
                self.state = 'NAVIGATING'

        # =========================================================================
        # STATE 3: NAVIGATING (Translate forward along flight path)
        # =========================================================================
        flight_yaw_err = (travel_angle_deg - cyaw + 180) % 360 - 180
        yaw_rate = float(np.clip(flight_yaw_err * 1.5, -45.0, 45.0))

        # Rotate-first check for flight direction alignment
        if abs(flight_yaw_err) > self.yaw_tolerance_deg:
            return 0.0, 0.0, yaw_rate

        # Compute translation velocity
        angle_to_target = math.atan2(target_dot[1] - cy, target_dot[0] - cx)
        vx_global = self.target_speed * math.cos(angle_to_target)
        vy_global = self.target_speed * math.sin(angle_to_target)

        # Convert to Body Frame
        yaw_rad = math.radians(cyaw)
        vx_body =  vx_global * math.cos(yaw_rad) + vy_global * math.sin(yaw_rad)
        vy_body = -vx_global * math.sin(yaw_rad) + vy_global * math.cos(yaw_rad)

        # Reactive Obstacle Avoidance
        front, back, left, right = ranger_data
        repulsion_x, repulsion_y = 0.0, 0.0

        if front < self.min_safety_dist:
            repulsion_x -= (self.min_safety_dist - front) * 2.5
        if back < self.min_safety_dist:
            repulsion_x += (self.min_safety_dist - back) * 2.5

        if left < self.min_safety_dist or right < self.min_safety_dist:
            if now > self.avoid_lock_time:
                self.last_avoid_dir = -1 if left < right else 1
                self.avoid_lock_time = now + 0.8

            if self.last_avoid_dir == -1:
                repulsion_y -= 0.15
            else:
                repulsion_y += 0.15
        else:
            if now > self.avoid_lock_time:
                self.last_avoid_dir = 0

        final_vx = float(np.clip(vx_body + repulsion_x, -0.3, 0.3))
        final_vy = float(np.clip(vy_body + repulsion_y, -0.3, 0.3))

        return final_vx, final_vy, yaw_rate

    
class NavigationManager:
    """
    Thread-safe thread launcher for the dual-loop setup.
    """
    def __init__(self, get_pose_func, get_grid_func, goal):
        self.get_pose_func = get_pose_func
        self.get_grid_func = get_grid_func
        self.goal = goal

        self.planner = FastRaycasterPlanner(step_len=0.25)
        self.controller = LocalReactiveController(scan_interval=2)

        self.path_lock = threading.Lock()
        self.current_dots = []
        self.recovery_flag = False
        self.running = True

    def start_planner_thread(self):
        thread = threading.Thread(target=self._global_loop, daemon=True)
        thread.start()

    def _global_loop(self):
        """10 Hz Thread for updates."""
        while self.running:
            loop_start = time.time()
            pose = self.get_pose_func()
            grid = self.get_grid_func()

            if pose and grid:
                start_pt = (pose['x'], pose['y'])
                dots, recovery = self.planner.generate_path_dots(start_pt, self.goal, grid)

                with self.path_lock:
                    self.current_dots = dots
                    self.recovery_flag = recovery

            elapsed = time.time() - loop_start
            time.sleep(max(0.01, 0.10 - elapsed))

    def get_control_command(self, current_pose, ranger_data):
        """Called inside your 50 Hz main thread hardware loop."""
        with self.path_lock:
            dots = list(self.current_dots)
            recovery = self.recovery_flag

        return self.controller.compute_motion(current_pose, dots, ranger_data, recovery)

    def stop(self):
        self.running = False