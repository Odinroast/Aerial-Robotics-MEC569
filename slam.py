"""Online occupancy-grid SLAM: odometry prediction + gated scan-to-map matching.
All geometry is in meters. Public yaw angles are degrees.
This local SLAM front end has no pose graph or loop-closure optimization.
"""
from collections import deque
import math
import threading
import numpy as np
from scipy.ndimage import distance_transform_edt, map_coordinates
from scipy.spatial import cKDTree

# One footprint definition for BOTH the planner and the reactive controller.
DRONE_RADIUS = 0.100                  # Your 100 mm radius, measured from drone center.
SAFETY_MARGIN = 0.040                 # Extra 40 mm beyond the propeller envelope.
CLEARANCE = DRONE_RADIUS + SAFETY_MARGIN
MAX_RANGE = 4.0                       # Ignore no-return/sentinel values above this.
OCCUPIED = math.log(0.7 / 0.3)        # Same occupancy cutoff everywhere.
ANGLES = np.radians([0., 180., 90., -90.])  # front, back, left, right


def wrap(degrees):
    return (degrees + 180.) % 360. - 180.


def rotate(points, angle):
    c, s = math.cos(angle), math.sin(angle)
    return np.asarray(points) @ np.array([[c, s], [-s, c]])


class GlobalOccupancyGrid:
    def __init__(self, map_x_min=0., map_x_max=4.6,
                 map_y_min=-1.475, map_y_max=1.475, resolution=.02):
        self.x_min, self.x_max = map_x_min, map_x_max
        self.y_min, self.y_max, self.resolution = map_y_min, map_y_max, resolution
        self.cols = math.ceil((map_x_max-map_x_min)/resolution)
        self.rows = math.ceil((map_y_max-map_y_min)/resolution)
        self.grid = np.zeros((self.cols, self.rows), np.float32)
        self.lock = threading.RLock()
        self._version, self._cached = 0, None

    def world_to_grid(self, x, y):
        return math.floor((x-self.x_min)/self.resolution), math.floor((y-self.y_min)/self.resolution)

    def grid_to_world(self, c, r):
        return self.x_min+(c+.5)*self.resolution, self.y_min+(r+.5)*self.resolution

    @staticmethod
    def line(c0, r0, c1, r1):
        """Cells crossed by a ray. Used for mapping and collision checking."""
        dc, dr = abs(c1-c0), abs(r1-r0)
        sc, sr, error = (1 if c0 < c1 else -1), (1 if r0 < r1 else -1), dc-dr
        while True:
            yield c0, r0
            if (c0, r0) == (c1, r1):
                break
            twice = 2*error
            if twice > -dr:
                error -= dr
                c0 += sc
            if twice < dc:
                error += dc
                r0 += sr

    def update_map(self, x, y, yaw, ranges):
        """A valid hit frees cells before it and adds evidence at the endpoint.
        Unknown/no-return measurements add neither obstacles nor free space.
        """
        if not all(math.isfinite(v) for v in (x, y, yaw)):
            return
        start = self.world_to_grid(x, y)
        with self.lock:
            for distance, offset in zip(ranges, ANGLES):
                if distance is None or not math.isfinite(distance) or not .05 < distance < MAX_RANGE:
                    continue
                angle = math.radians(yaw)+offset
                end = self.world_to_grid(x+distance*math.cos(angle), y+distance*math.sin(angle))
                cells = list(self.line(*start, *end))
                for c, r in cells[:-1]:
                    if 0 <= c < self.cols and 0 <= r < self.rows:
                        self.grid[c, r] = max(-4., self.grid[c, r]-.20)
                c, r = cells[-1]
                if 0 <= c < self.cols and 0 <= r < self.rows:
                    self.grid[c, r] = min(4., self.grid[c, r]+.65)
            self._version += 1

    def costmap(self):
        """Inflate occupied cells by radius + margin. Cache until the map changes."""
        with self.lock:
            if self._cached is not None and self._cached[0] == self._version:
                return self._cached[1]
            occupied = self.grid >= OCCUPIED
            if occupied.any():
                # Half a cell diagonal accounts for the area of occupied cells.
                blocked = distance_transform_edt(~occupied)*self.resolution <= CLEARANCE+self.resolution/math.sqrt(2)
            else:
                blocked = np.zeros_like(occupied)
            xs = self.x_min+(np.arange(self.cols)+.5)*self.resolution
            ys = self.y_min+(np.arange(self.rows)+.5)*self.resolution
            blocked |= ((xs[:, None] < self.x_min+CLEARANCE) |
                        (xs[:, None] >= self.x_max-CLEARANCE) |
                        (ys[None, :] < self.y_min+CLEARANCE) |
                        (ys[None, :] >= self.y_max-CLEARANCE))
            # Never mutate this array after publishing it to another thread.
            self._cached = self._version, blocked
            return blocked

    def segment_clear(self, a, b, blocked=None):
        blocked = self.costmap() if blocked is None else blocked
        cells = list(self.line(*self.world_to_grid(*a), *self.world_to_grid(*b)))
        for c, r in cells:
            if not 0 <= c < self.cols or not 0 <= r < self.rows or blocked[c, r]:
                return False
        # Check both neighboring cells when crossing diagonally: no corner cutting.
        for (c0, r0), (c1, r1) in zip(cells, cells[1:]):
            if c0 != c1 and r0 != r1 and (blocked[c0, r1] or blocked[c1, r0]):
                return False
        return True

    def occupied_points(self, threshold=OCCUPIED):
        with self.lock:
            cells = np.argwhere(self.grid >= threshold)
        return np.column_stack((self.x_min+(cells[:, 0]+.5)*self.resolution,
                                self.y_min+(cells[:, 1]+.5)*self.resolution))


class GridSLAM:
    def __init__(self, grid, start=(.3, 0.)):
        self.grid, self.start = grid, np.array(start)
        self.lock = threading.Lock()
        self.translation, self.angle = None, 0.   # map <- odometry transform
        self.status = 'BOOTSTRAP'
        self.inliers, self.match_error = 0., float('nan')
        self.full_updates, self.partial_updates = 0, 0
        self.frames = deque(maxlen=12)           # Recent synchronized poses/ranges.
        self.reference, self.reference_time = None, -float('inf')
        self.last_match, self.last_map, self.last_sample = -float('inf'), -float('inf'), None
        self.last_seen = None

    def corrected_pose(self, odom):
        """Transform the newest odometry without waiting for scan matching."""
        if not all(math.isfinite(odom[k]) for k in ('x', 'y', 'yaw')):
            raise ValueError('Non-finite odometry')
        with self.lock:
            if self.translation is None:
                # Place the first logged pose at START, initially pointing along map +X.
                self.angle = -math.radians(odom['yaw'])
                self.translation = self.start-rotate([odom['x'], odom['y']], self.angle)
            xy = rotate([odom['x'], odom['y']], self.angle)+self.translation
            return {'x': float(xy[0]), 'y': float(xy[1]),
                    'yaw': wrap(odom['yaw']+math.degrees(self.angle)),
                    'odom_x': odom['x'], 'odom_y': odom['y']}

    def diagnostics(self):
        with self.lock:
            return {'slam_state': self.status, 'inliers': self.inliers,
                    'match_error': self.match_error, 'tx': float(self.translation[0]) if self.translation is not None else 0.,
                    'ty': float(self.translation[1]) if self.translation is not None else 0.,
                    'yaw_offset': math.degrees(self.angle),
                    'full_updates': self.full_updates, 'partial_updates': self.partial_updates}

    def _freeze_reference(self, stamp):
        """Match against OLD evidence, never the scans currently being matched."""
        points = self.grid.occupied_points(threshold=1.4)  # At least ~3 agreeing hits.
        if len(points) < 25:
            return
        mask = np.zeros((self.grid.cols, self.grid.rows), bool)
        c = ((points[:, 0]-self.grid.x_min)/self.grid.resolution).astype(int)
        r = ((points[:, 1]-self.grid.y_min)/self.grid.resolution).astype(int)
        mask[c, r] = True
        self.reference = (distance_transform_edt(~mask)*self.grid.resolution, cKDTree(points))
        self.reference_time = stamp

    def _match(self, odom, stamp):
        if self.reference is None:
            return 'BOOTSTRAP'
        # Samples already inserted into this reference MUST NOT match themselves.
        usable = [(points, origins) for t, points, origins in self.frames
                  if t > self.reference_time+.001]
        if not usable or sum(len(p) for p, _ in usable) < 12:
            return 'ODOMETRY_ONLY'
        with self.lock:
            angle, translation = self.angle, self.translation.copy()
        endpoints = rotate(np.concatenate([p for p, _ in usable]), angle)+translation
        origins = rotate(np.concatenate([o for _, o in usable]), angle)+translation
        pivot = rotate([odom['x'], odom['y']], angle)+translation
        relative = endpoints-pivot
        field, tree = self.reference

        def residual(points):
            indices = np.array([(points[..., 0]-self.grid.x_min)/self.grid.resolution-.5,
                                (points[..., 1]-self.grid.y_min)/self.grid.resolution-.5])
            return map_coordinates(field, indices, order=1, mode='constant', cval=.30)

        def search(offsets):
            a = offsets[:, 2]
            c, s = np.cos(a)[:, None], np.sin(a)[:, None]
            px = c*relative[:, 0]-s*relative[:, 1]+pivot[0]+offsets[:, 0, None]
            py = s*relative[:, 0]+c*relative[:, 1]+pivot[1]+offsets[:, 1, None]
            points = np.stack((px, py), axis=-1)
            distances = residual(points)
            # Robust endpoint loss plus a small odometry prior prevents large jumps.
            loss = np.mean(np.minimum(distances, .20)**2, axis=1)
            loss += .03*(offsets[:, 0]**2+offsets[:, 1]**2+(.5*offsets[:, 2])**2)
            i = int(np.argmin(loss))
            return offsets[i], distances[i], points[i]

        # Bounded coarse search followed by a finer search (no neural network).
        mesh = np.meshgrid(np.arange(-.10, .101, .025), np.arange(-.10, .101, .025),
                           np.radians(np.arange(-4., 4.01, 1.)), indexing='ij')
        coarse = np.stack(mesh, axis=-1).reshape(-1, 3)
        best, distances, points = search(coarse)
        if np.any(np.abs(best[:2]) >= .099) or abs(best[2]) >= math.radians(3.99):
            return 'SEARCH_LIMIT'             # True solution may be outside the search.
        mesh = np.meshgrid(np.arange(-.02, .021, .01), np.arange(-.02, .021, .01),
                           np.radians(np.arange(-.8, .81, .4)), indexing='ij')
        best, distances, points = search(best+np.stack(mesh, axis=-1).reshape(-1, 3))
        good = distances < .065
        fraction = float(np.mean(good))
        rms = float(np.sqrt(np.mean(distances[good]**2))) if good.any() else float('inf')
        with self.lock:
            self.inliers, self.match_error = fraction, rms
        if fraction < .65 or rms > .045:
            return 'LOW_MATCH'

        # Estimate local surface normals. One wall cannot determine all x/y/yaw.
        # Correct only observable modes; a wall does not fix unconstrained drift.
        jacobian = []
        for point in points[good]:
            distance, indices = tree.query(point, k=min(10, len(tree.data)))
            neighbors = tree.data[indices[distance < .18]]
            if len(neighbors) < 4:
                continue
            values, vectors = np.linalg.eigh(np.cov(neighbors.T))
            if values[1] < .0002 or values[0] > .2*values[1]:
                continue
            normal = vectors[:, 0]
            arm = point-pivot-best[:2]
            jacobian.append([normal[0], normal[1], np.dot(normal, [-arm[1], arm[0]])/.5])
        if len(jacobian) < 10:
            return 'WEAK_GEOMETRY'
        j = np.array(jacobian)
        eig = np.linalg.eigvalsh(j.T@j/len(j))
        supported = eig > max(.05, eig[-1]/150.)
        if not supported.any():
            return 'AMBIGUOUS'
        full_match = bool(supported.all())
        if not full_match:
            # Without full pose geometry, keep odometry yaw. Search translation only,
            # then remove translation along directions unsupported by wall normals.
            best, _, _ = search(coarse[np.abs(coarse[:, 2]) < 1e-9])
            values, directions = np.linalg.eigh(j[:, :2].T@j[:, :2]/len(j))
            observed = directions[:, values > .05]
            best[:2] = observed@(observed.T@best[:2])
            best[2] = 0.
            check = residual(endpoints+best[:2])
            good = check < .065
            if np.mean(good) < .65 or np.sqrt(np.mean(check[good]**2)) > .045:
                return 'AMBIGUOUS'
            if np.any(np.abs(best[:2]) >= .099):
                return 'SEARCH_LIMIT'

        # A good endpoint match must not put a wall through the observed free ray.
        ray_origins = rotate(origins-pivot, best[2])+pivot+best[:2]
        points = rotate(relative, best[2])+pivot+best[:2]
        conflicts = 0
        for origin, endpoint in zip(ray_origins, points):
            length = np.linalg.norm(endpoint-origin)
            if length <= .16:
                continue
            fractions = np.arange(.06, length-.08, .04)/length
            probes = origin+fractions[:, None]*(endpoint-origin)
            conflicts += bool(np.any(residual(probes) < .025))
        if conflicts/max(1, len(points)) > .20:
            return 'RAY_CONFLICT'

        # Smooth the accepted correction, limiting each update to 2 cm / 0.6 degrees.
        shift = best[:2]*.4
        length = np.linalg.norm(shift)
        if length > .02:
            shift *= .02/length
        turn = float(np.clip(best[2]*.4, -math.radians(.6), math.radians(.6)))
        with self.lock:
            self.translation = pivot+shift+rotate(self.translation-pivot, turn)
            self.angle += turn
            self.full_updates += int(full_match)
            self.partial_updates += int(not full_match)
        return 'MATCHED' if full_match else 'PARTIAL_MATCH'

    def update(self, stamp, odom, ranges):
        """Called by ONE worker. Main/control thread only reads the transform."""
        if not all(math.isfinite(v) for v in (odom['x'], odom['y'], odom['yaw'])):
            return
        self.corrected_pose(odom)           # Initialize the transform if needed.
        if self.last_sample is not None and stamp <= self.last_sample:
            return                         # Duplicate/stale packet: no repeated evidence.
        self.last_sample = stamp
        # Points expressed in odometry; the same SE(2) correction moves every scan.
        points = []
        for distance, offset in zip(ranges, ANGLES):
            if distance is not None and math.isfinite(distance) and .05 < distance < MAX_RANGE:
                angle = math.radians(odom['yaw'])+offset
                points.append([odom['x']+distance*math.cos(angle),
                               odom['y']+distance*math.sin(angle)])
        if points:
            self.frames.append((stamp, np.array(points),
                                np.tile([odom['x'], odom['y']], (len(points), 1))))
        # Drop old scan data after gaps so it cannot dominate a new match.
        while self.frames and stamp-self.frames[0][0] > .8:
            self.frames.popleft()
        if stamp-self.last_match >= .20:
            status = self._match(odom, stamp)
            with self.lock:
                self.status = status
            self.last_match = stamp
        if stamp-self.reference_time >= 2.0:
            self._freeze_reference(stamp)   # BEFORE inserting the current observation.
        if stamp-self.last_map >= .10:
            pose = self.corrected_pose(odom)
            self.grid.update_map(pose['x'], pose['y'], pose['yaw'], ranges)
            self.last_map = stamp
        self.last_seen = stamp
