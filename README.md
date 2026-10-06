# Simple Crazyflie navigation with grid SLAM

Use all four runtime files together: `main.py`, `slam.py`, `planner.py`, and `plotter.py`.
Run from this folder so Python imports the matching versions.

```bash
python3 -m pip install -r requirements.txt
python3 -m unittest -v
python3 main.py --turn-every 3
```

Choose the turn interval on the command line:

```bash
python3 main.py --turn-every 3   # Waypoints 3, 6, 9, 12...
python3 main.py --turn-every 4   # Waypoints 4, 8, 12, 16...
```

Or edit `TURN_EVERY_WAYPOINTS = 3` near the top of `main.py` for a persistent default.
The command-line setting overrides that default. Values must be positive integers.

Your existing radio URI is the default. To change it:

```bash
python3 main.py radio://0/98/2M/E7E7E7E7E8 --turn-every 4
```

Point the drone along the arena's +X direction at the specified start position
before starting. The first logged ground pose anchors the map at `(0.3, 0.0)`
with heading zero. Arena bounds and goal match your previous code.

## What the files do

- `main.py`: synchronized telemetry, one mapping/planning worker, the 50 Hz flight
  loop, goal landing, shutdown, and CSV logging. No GUI rendering or median-position
  delay in the flight loop. Radio libraries are imported only when flying.
- `slam.py`: occupancy mapping and a local scan-to-map SLAM front end. It estimates
  a map-to-odometry transform; that transform corrects the pose used by navigation.
- `planner.py`: A* global planning, stable route waypoints, lookahead tracking,
  alternating yaw, and simple directional obstacle avoidance.

The code comments explain the coordinate transforms, matching gates, footprint
inflation, braking model, and waypoint progression. Dependencies are NumPy, SciPy,
cflib, and Matplotlib for the optional live dashboard.

## Live dashboard

`main.py` opens the dashboard automatically. There is no separate plotter command:

```bash
python3 main.py --turn-every 4
python3 main.py --turn-every 4 --plot-hz 10
python3 main.py --turn-every 4 --no-plot
```

The display refreshes at 5 Hz by default; flight commands continue at 50 Hz.
`--plot-hz` changes the display rate only. The plot uses the actual grid bounds,
resolution, START, GOAL, and GOAL_AREA, so changing those settings updates the axes
and landing region automatically. With symmetric Y bounds, Y=0 remains the center.

The dashboard shows:
- free/unknown/occupied cells, and the inflated footprint clearance;
- goal point, landing region, start point, route, and lookahead target;
- current drone footprint and heading, commanded motion vector, and horizontal range rays;
- corrected trajectory and odometry trajectory aligned to the initial map frame;
- X/Y/yaw, desired yaw, waypoint count, turn interval, route revision, and stop reason;
- horizontal body velocity and yaw command history over the latest 30 seconds;
- six range readings, with `?` for unknown;
- SLAM status, matching residual, inliers, accepted corrections, and map-to-odometry transform;
- telemetry age, mission phase, and faults.

`plotter.py` runs a snapshot-publishing background thread. It uses a separate
process for rendering because interactive Matplotlib GUI backends require a main
thread. That process owns all GUI objects and receives snapshots through a bounded
queue. Slow plotting drops display updates instead of blocking flight commands.
The display reads state only; it never sends drone commands or changes navigation.

Closing the window leaves the drone running. To interrupt flight, use Ctrl+C in
its terminal; the existing MotionCommander context handles landing. A final
`flight_live_plot.png` is saved when the window closes or the flight ends. Keep
using the CSV for the complete history; display history is bounded.

A desktop GUI backend is required for an interactive window. Without one, the
plotter falls back to headless rendering and saves the PNG. Display failures do
not change navigation. Offline tests cover rendering, custom map dimensions,
unknown readings, read-only state access, and plot-process cleanup; an interactive
window on your flight computer and live hardware integration remain untested.

Matplotlib threading reference:
https://matplotlib.org/stable/users/faq.html#work-with-threads

## Your 100 mm drone radius

These constants are at the top of `slam.py`:

```python
DRONE_RADIUS = 0.100
SAFETY_MARGIN = 0.040
CLEARANCE = DRONE_RADIUS + SAFETY_MARGIN
```

The drone's physical radius is exactly 100 mm. The separate 40 mm margin makes
nominal center-to-obstacle clearance 140 mm. Grid inflation also accounts for half
a cell diagonal so occupied-cell area does not shrink the effective clearance.
The same footprint governs A*, segment checks, and the directional speed limits.
Ranges approximate sensor origins at the drone center; calibrating each sensor's
actual position can improve mapping precision.

The speed limit uses a simple stopping-distance model with assumed 0.4 m/s²
braking acceleration and 0.20 s reaction time. These are starting parameters,
not measured guarantees for your vehicle. Set them in `Navigator.velocity_limit()`.

## Localization: what is implemented

1. Predict the latest pose from odometry and the current map-to-odometry transform.
2. Accumulate up to 12 synchronized scans, no older than 0.8 s.
3. Every 0.2 s, match scan endpoints against an older map snapshot using a bounded
   search: +/-10 cm in XY and +/-4 degrees in yaw, followed by refinement.
4. Require endpoint agreement and reject solutions at the search boundary.
5. Estimate surface geometry. If the full pose is supported, correct XY and yaw.
   If only some translation directions are supported, keep odometry yaw and
   correct only those translation directions. A straight wall alone cannot fix
   drift along its length.
6. Reject matches whose observed free rays pass through known mapped obstacles.
7. Smooth accepted corrections; each update is limited to 2 cm and 0.6 degrees.
8. Insert the current scan only AFTER matching. Scans already included in a
   reference are excluded from matching that reference.

This is lightweight online grid SLAM with local localization. It has no pose
graph, global loop-closure optimization, relocalization after a large jump, or
retroactive rebuilding of old scans. Initial mapping necessarily uses odometry.
Four sparse horizontal range readings cannot always constrain the full pose.
Weak matches retain the current transform and continue using odometry; they do
not automatically stop flight or force a settling sequence. No drift correction
is guaranteed where the map and geometry provide too little information.

Status values in the console/CSV:

| Status | Meaning |
|---|---|
| `BOOTSTRAP` | Not enough established map evidence yet |
| `MATCHED` | Accepted XY and yaw correction |
| `PARTIAL_MATCH` | Accepted correction in supported translation directions; yaw retained |
| `ODOMETRY_ONLY` | Too few new valid endpoints |
| `LOW_MATCH` | Poor overlap with the reference |
| `WEAK_GEOMETRY` / `AMBIGUOUS` | Insufficient surface constraints |
| `SEARCH_LIMIT` | Best solution reaches the local search limit |
| `RAY_CONFLICT` | Candidate contradicts observed free rays |

The matcher intentionally does not invent corrections merely to report success.

## Motion behavior

Defaults in `Navigator.__init__`:

| Parameter | Value |
|---|---|
| Cruise speed | 0.25 m/s |
| Translation during scheduled yaw | exactly 0 m/s |
| Lookahead | 0.55 m |
| Intermediate waypoint acceptance | within 0.15 m, or passed within a 0.30 m corridor |
| Maximum waypoint spacing | 0.40 m, preserving corners |
| Replan deviation | over 0.45 m, or blocked remaining route |
| Yaw interval | every 3 reached waypoints by default; configurable |
| Maximum yaw rate | 45 degrees/s |

With `--turn-every 3`, waypoint 3 adds +90 degrees, waypoint 6 adds -90,
waypoint 9 adds +90, etc. With `--turn-every 4`, those turns occur at waypoints
4, 8, 12, etc. The drone keeps the new heading after each turn.

Each scheduled turn enters `ROTATING` and returns only `(0, 0, yaw_rate)`.
The main loop also commands vertical velocity zero. No XY correction, obstacle
repulsion, or waypoint chase can add translation during rotation. Waypoint
counting and route replacement are paused during the turn; SLAM mapping continues.

Translation resumes immediately when heading error is at most 3 degrees, without
waiting for precise XY settling. Navigation commands have yaw rate zero, so
translation and commanded yaw never overlap. Physical inertia or estimator drift
can still change the measured XY position while translation commands are zero.
The no-progress watchdog is reset after an intentionally stationary turn.

Replans preserve the reached-waypoint counter. Cancelled dots are not counted,
and new dots come from the newly planned route. Numbering is therefore traversal
order, not permanent arena landmarks. The final waypoint uses the tighter arrival
test; a wide sideways pass cannot incorrectly finish outside the goal area.

Missing ranges limit the affected body-velocity component to 0.05 m/s. They do
not reduce every velocity component to 0.05 m/s. Valid nearby returns apply the
braking limit; the inflated map checks the combined motion too. Unknown map cells
remain traversable, but are not certified clear. Four rays can miss narrow or
unmapped diagonal obstacles.

## Changes motivated by your log/code

Your old log has 13,331 samples over 133.30 s. During `NAVIGATING`, about 14.54 s
had zero commanded translation; the median nonzero command speed was 0.0836 m/s.
Only two scan sequences occurred. The code required 6 cm arrival or a 5 cm passing
corridor, and scans turned out and then back.

The replacement removes:

- tight intermediate waypoint centering;
- pre-turn and post-turn settling states;
- returning to the old heading after every scan;
- stop-to-align behavior before translation;
- constant regeneration of route dots;
- the blanket missing-range speed cap;
- GUI rendering and expensive repeated circular-offset queries in the flight loop;
- mapping ground readings and substituting 1 m for missing ranges.

The old log cannot identify the reason for each pause or separate actual
movement from estimator drift. These are changes to plausible causes, not proof
that each cause was active in the recorded flight.

Telemetry uses firmware `range.*` values (uint16 millimeters), converted once
into meters. No second Multiranger polling/conversion stream is used. Scans and
pose come from the same 20 Hz telemetry packet; mapping runs at most 10 Hz.
Sensor values may still update internally at different rates. Duplicate telemetry
packets are excluded; identical numeric ranges are not assumed to be new independent
measurements merely because a callback ran faster.

The occupied threshold is consistent for matching/map planning. Two agreeing
endpoint updates are needed before a previously unknown cell blocks navigation.
The code caches footprint inflation until the map changes.

## Saved flight diagnostics

`flight_log_slam.csv` includes raw firmware ranges in mm, filtered ranges in m,
raw odometry, corrected pose, waypoint count, plan revision, desired yaw,
controller state, stop reason, SLAM status, map-to-odometry transform, inlier
fraction, candidate residual, and counts of accepted full/partial corrections.
Raw odometry is saved separately so it can be compared with the corrected pose.

`occupancy_map.npz` saves the final grid, resolution, and bounds. The map is not
loaded on the next flight; each run anchors and builds its own map.

The up-sensor stop, telemetry-age check, worker-health check, bounded yaw/no-route
and no-progress failures, and a 180 s mission timeout lead to landing. There are
no arbitrary delays while waiting for tiny XY errors to settle.

## Offline replay and validation

```bash
python3 main.py --replay /path/to/flight_log_working2.csv --output replay.csv
```

Old CSV range columns are already meters. Replay does not convert them again.
Replay uses raw odometry columns when available. It runs mapping/localization
only; it cannot predict how the drone would fly under new velocity commands.

Seventeen offline tests passed, covering footprint clearance, invalid/duplicate
measurements, A* detours, alternating in-place yaw with zero translation, component-specific
unknown-range limits, goal handling, scan-to-map correction, partial constraints,
rejection of rays passing through a known wall, configurable intervals of 3 and 4,
rotation timeouts, and frozen waypoint/route updates while rotating.

In the synthetic known-room test, an injected XY bias of 7.21 cm reduced to
2.74 cm after several bounded updates. This is an ideal test, not real-flight
accuracy. The supplied recorded log has no independent ground-truth position,
so its replay cannot prove that the corrected position is closer to reality.
See `validation.txt` for the final recorded-log replay results.

Sources for firmware units and sparse-ranging limitations:
- https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/api/logs/#range
- https://www.bitcraze.io/tag/mapping/
- https://github.com/SteveMacenski/slam_toolbox
