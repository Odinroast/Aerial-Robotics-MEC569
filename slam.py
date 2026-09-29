import numpy as np

# Occumpancy grid based SLAM
class GlobalOccupancyGrid:
    def __init__(self, map_x_min=0.0, map_x_max=2.0, map_y_min=-1.0, map_y_max=1.0, resolution=0.02):
        """
        Fixed Global Occupancy Grid Map.
        - Keeps full history of the environment.
        - Raycasting dynamically clears phantom points and sensor noise.
        """
        self.x_min = map_x_min
        self.x_max = map_x_max
        self.y_min = map_y_min
        self.y_max = map_y_max
        self.resolution = resolution

        self.cols = int(np.ceil((map_x_max - map_x_min) / resolution))
        self.rows = int(np.ceil((map_y_max - map_y_min) / resolution))

        # Log-odds array: 0 = Unknown, >0 = Occupied, <0 = Free Space
        self.grid = np.zeros((self.cols, self.rows), dtype=np.float32)

        # Probabilistic Update Constants
        self.L_OCCUPIED = 0.85   # Confidence boost on obstacle hit
        self.L_FREE = -0.35      # Confidence penalty along clear laser path
        self.L_MAX = 4.0
        self.L_MIN = -4.0

    def world_to_grid(self, x, y):
        """Converts global (x, y) meters into grid matrix indices (col, row)."""
        col = int((x - self.x_min) / self.resolution)
        row = int((y - self.y_min) / self.resolution)
        return col, row

    def grid_to_world(self, col, row):
        """Converts grid matrix indices back to global (x, y) meters."""
        x = self.x_min + (col + 0.5) * self.resolution
        y = self.y_min + (row + 0.5) * self.resolution
        return x, y

    def update_map(self, drone_x, drone_y, drone_yaw_deg, ranges):
        """
        Updates the global persistent map using current drone pose and Multiranger distance readings.
        ranges = (Front, Back, Left, Right) in meters.
        """
        # Sensor angles relative to drone body frame (Front, Back, Left, Right)
        sensor_angles_deg = [0.0, 180.0, 90.0, 270.0]
        
        # Drone position in grid space
        d_col, d_row = self.world_to_grid(drone_x, drone_y)

        for dist, angle_offset in zip(ranges, sensor_angles_deg):
            # Filter out invalid / out-of-range sensor readings
            if dist is None or dist <= 0.05 or dist >= 2.0:
                continue

            # Calculate global angle of laser beam
            beam_yaw_rad = np.radians(drone_yaw_deg + angle_offset)

            # Hit location in global meters
            hit_x = drone_x + dist * np.cos(beam_yaw_rad)
            hit_y = drone_y + dist * np.sin(beam_yaw_rad)

            hit_col, hit_row = self.world_to_grid(hit_x, hit_y)

            # Raycast line from drone position to target hit cell
            line_cells = self._bresenham_line(d_col, d_row, hit_col, hit_row)

            # 1. Clear free space along the laser path
            for c, r in line_cells[:-1]:
                if 0 <= c < self.cols and 0 <= r < self.rows:
                    self.grid[c, r] = max(self.L_MIN, self.grid[c, r] + self.L_FREE)

            # 2. Mark obstacle at the impact endpoint
            ec, er = line_cells[-1]
            if 0 <= ec < self.cols and 0 <= er < self.rows:
                self.grid[ec, er] = min(self.L_MAX, self.grid[ec, er] + self.L_OCCUPIED)

    def get_binary_costmap(self, threshold=1.0):
        """
        Returns a 2D boolean array (True = Obstacle, False = Passable)
        Ready for A* or Dijkstra Path Planning algorithms.
        """
        return self.grid >= threshold

    def get_occupied_points(self, threshold=1.0):
        """Returns global (X, Y) coordinates of obstacles for plotting."""
        occupied_indices = np.argwhere(self.grid >= threshold)
        if len(occupied_indices) == 0:
            return np.empty((0, 2))

        pts_x = self.x_min + (occupied_indices[:, 0] + 0.5) * self.resolution
        pts_y = self.y_min + (occupied_indices[:, 1] + 0.5) * self.resolution
        return np.column_stack((pts_x, pts_y))

    def is_occupied(self, x: float, y: float, threshold: float = 0.7) -> bool:
        """
        Checks if world coordinates (x, y) fall inside an occupied grid cell.
        Converts log-odds values to probability for standard threshold comparison.
        """
        col, row = self.world_to_grid(x, y)

        # Check bounds: Out-of-bounds cells treated as occupied for drone safety
        if not (0 <= col < self.cols and 0 <= row < self.rows):
            return True

        # Convert log-odds L back to probability P: P = 1 / (1 + exp(-L))
        log_odds = self.grid[col, row]
        prob = 1.0 / (1.0 + np.exp(-log_odds))

        return prob >= threshold
    
    def _bresenham_line(self, x0, y0, x1, y1):
        """Bresenham's Line Algorithm for discrete grid raycasting."""
        points = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy

        curr_x, curr_y = x0, y0
        while True:
            points.append((curr_x, curr_y))
            if curr_x == x1 and curr_y == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                curr_x += sx
            if e2 < dx:
                err += dx
                curr_y += sy
        return points