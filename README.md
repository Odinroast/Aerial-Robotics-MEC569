# Crazyflie: fly from A to B around obstacles, and land on boxes

Two programs share the same navigator:

- `main.py`: fly from a start point to a goal around obstacles (this page).
- `mission.py`: the two-box mission. Take off from a box at A, find the box near B while
  flying there, land on its center, then come back and land on the box at A. See
  [MISSION.md](MISSION.md). A full run through a cluttered room is in
  `working_mission_log_20261019_155143.csv` / `.png` (3.7 m each way, both boxes landed).

Hardware: Crazyflie 2.x with Flow deck and Multi-ranger deck.

```bash
python3 -m pip install -r requirements.txt
python3 -m unittest discover -p 'test_*.py' # all simulator tests, no drone needed (~45 s)
python3 main.py                              # fly START -> GOAL from main.py
python3 main.py --goal 2.0 0.5               # override the goal (meters)
python3 main.py radio://0/80/2M/E7E7E7E7E7   # other radio URI
python3 main.py --arena 0 4.6 -1.475 1.475   # optional virtual fence (off by default)
```

Edit `START`, `GOAL`, `HEIGHT` and `ARENA` at the top of `main.py`. Place the drone at
`START` with its nose along map +X. Ctrl+C lands. Only use a fence that matches the real
space: the drone keeps clear of it like a wall, so a fence through a doorway closes it.
The goal needs at least ~20 cm of free space around it.

## What it does

1. Takes off, hovers 1.5 s, anchors the map frame at `START`.
2. Stops, then turns in place to +45°, −45° and back to 0° (60°/s) so the four rangers
   cover 360°. It never turns and moves at the same time: doing both smeared the map
   and bent the flight path, which caused the crash in `flight_log_20261015_090531.csv`.
3. Flies with the nose held at 0°, at up to 0.2 m/s. A* replans on a 5 cm grid four
   times a second from the obstacles seen so far (unknown space counts as free). The
   path keeps 25 cm or more from obstacles where there is room, and down to 18 cm (8 cm
   of air past the props) in doors, flying down their middle. It sticks to its current
   route instead of flipping between alternatives. Each heading is checked against a
   corridor of remembered obstacles. It rescans after every 1 m, or when blocked.
4. A range guard on the raw rangers limits speed toward anything within 25 cm and backs
   away from anything closer, even if the map is wrong.
5. Lands when within 8 cm of the goal for 1 s, or on any fault: no path for 10 s, no
   progress for 60 s, pose jump > 10 cm, yaw drift, telemetry lost, height lost
   (z-ranger off by > 15 cm for 0.3 s), object above, 180 s limit.

Each flight writes `flight_log_<time>.csv` (the last row, state `END`, says why it
landed) and a `.png` plot of the trajectory and ranger hits. Re-plot with `python3 plotter.py flight_log_<time>.csv`.

## Files

Navigation (A to B):

- `main.py`: radio, takeoff/landing, 20 Hz control loop, CSV log.
- `planner.py`: obstacle memory, A* planning, corridor check, scan/move state machine, range
  guard. The mission uses it too, with fewer scans and a speed it changes in flight.
- `plotter.py`: post-flight plot.
- `test_navigation.py`: closed-loop 2D simulator (noisy odometry, 10 Hz rangers, delay).
- `test_main.py`: runs `main.fly()` against a fake cflib connected to the simulator.

Box mission (details in [MISSION.md](MISSION.md)):

- `mission.py`: the two legs: approach while watching for the box, sweep lines as a
  fallback, obstacle handling, map anchoring between legs, safety landings, CSV log.
- `box_finder.py`: box detection from the down ranger and the center estimate from
  out-and-back passes. Also runs on its own for a single box.
- `mission_plot.py`: live plot while flying and the saved `.png`.
- `test_mission.py`, `test_box_finder.py`: closed-loop simulations of both, plus replays
  of real logs.

Other:

- `log_collector.py`: hover and push the drone away by hand, logging all rangers (front,
  back, left, right, up, z) to CSV.
- `flight_log.csv`: a sample navigation flight.

## Limits

- Openings narrower than about 50 cm (as measured by the rangers) are treated as blocked.
- Stopping to scan makes flights slower (about 40 s for 3 m with obstacles in simulation).
- Obstacles must be at ranger height (they see horizontally only).
- Position comes from the Flow deck alone and drifts a few cm per meter; the goal is
  reached in that estimated frame. Use a textured floor and good lighting.
