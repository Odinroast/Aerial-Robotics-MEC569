# Two-box mission (`mission.py`)

The drone takes off from box 1 at point A, flies to point B, finds box 2 there and lands
on its center. After a short pause it takes off again, flies back to A, finds box 1 and
lands on it.

Hardware: Crazyflie 2.x with Flow deck and Multi-ranger deck (down ranger for the boxes,
side rangers for walls, up ranger for overhead objects).

```bash
venv/bin/python mission.py                                  # defaults from the top of mission.py
venv/bin/python mission.py --goal 3.25 1.14                 # point B, map meters
venv/bin/python mission.py --sweep -0.5 0.5 --x-window 0.3  # where to look for each box
venv/bin/python mission.py --box-size 0.45 --box-height 0.16 --pause 5
venv/bin/python mission.py radio://0/80/2M/E7E7E7E7E7       # other radio URI
venv/bin/python -m unittest test_mission test_box_finder    # simulator tests, no drone needed
```

Ctrl+C lands. The program exits with 0 only if it landed on both boxes.

## Setting up

- Place the drone **on top of box 1**, roughly centered, with its nose along map +X.
  That spot is point A (`--start`, default `0 0`). The nose stays at +X for the whole
  mission; the drone never turns around.
- Put box 2 so that its center is within `--x-window` (default 0.10 m) of B's x, and
  inside B's Y range.
- `--sweep DY_MIN DY_MAX` (default `-0.2 0.2`) is where the **box center** may be in Y,
  relative to each point's own y. The sweep lines run half a box plus 0.30 m
  (`FLOOR_MARGIN`) past it on both sides, so they start over bare floor: with a 0.35 m
  box and `-0.2 0.2`, B's lines run from goal_y - 0.675 to goal_y + 0.675.
- Clutter near the lines is fine: line starts that are too close to something seen are
  moved along the line. Keep about 0.5 m between a box edge and walls if you can; a
  pass cut short by a wall can make the center estimate fail (the line is then retried).
- Both boxes are square and the same size: `--box-size` (side, default 0.35 m) and
  `--box-height` (default 0.16 m).

## Options

| Option | Default | Meaning |
|---|---|---|
| `uri` | `radio://0/98/2M/E7E7E7E7E8` | Crazyflie radio address |
| `--start X Y` | `0 0` | Point A: box 1, where the drone takes off |
| `--goal X Y` | `1.35 0` | Point B: box 2 is near this x |
| `--sweep DY_MIN DY_MAX` | `-0.2 0.2` | Y range of the box center at each end, relative to that point's y |
| `--x-window` | `0.10` | The box center is within this of the target x, m |
| `--box-size` | `0.35` | Side of the square box, m |
| `--box-height` | `0.16` | Box height, m (sets the detection levels) |
| `--pause` | `5` | Seconds on box 2 before taking off again |
| `--arena X_MIN X_MAX Y_MIN Y_MAX` | none | Virtual fence for the navigator |
| `--output` | `mission_log_<time>.csv` | Log file name |
| `--no-live` | off | Don't open the live plot window |

The defaults come from the constants at the top of `mission.py` (mission geometry and
safety limits) and `box_finder.py` (box size and height, flight height, speeds,
detection levels).

## What it does

Each of the two legs runs the same steps:

1. **Take off** to 0.5 m. Takeoff resets the position estimate on the ground, so the leg
   maps its odometry onto the map spot where the drone last landed (A for leg 1).
2. **Approach and watch.** The obstacle-avoiding navigator from `planner.py` (the same
   one `main.py` uses) flies straight at the target and 0.55 m past it (half a box +
   0.40 m), so it crosses a box at the target at a steady speed. The target is the middle
   of the sweep range at B, or the remembered spot of box 1. To save battery the
   navigator scans (turns to ±45°) only before the leg's first trip and whenever its way
   is blocked, not every meter.
   - It flies at 0.30 m/s away from the box area. Within 0.45 m of any possible box
     position (the sweep area) it slows to the 0.20 m/s pass speed and the box detector
     watches zrange, like a sweep line does (see below). The box it took off from is
     ignored.
   - Braking, scanning or turning slower than 0.16 m/s for more than 0.3 s restarts the
     watch, which then needs about 1 s of floor again.
   - On a box edge it finds the center from there (see below). If the last 1 s was a
     straight line along X or Y (within 3 cm), that flight already is the out pass and
     it just keeps going. On a diagonal route it backs up 0.40 m and flies a pass along
     the box axis nearest to its heading. Box edges must be square to the map X/Y axes.
   - If nothing is seen on the way, the navigator can't get there, or the center isn't
     found, the leg falls back to the sweep.
   In simulation this saves about 7 s per leg on a straight route (38 s instead of 45 s).
3. **Travel** to the start of the first sweep line with the same navigator. Later trips
   inside the sweep area use the obstacle memory instead of scanning. Hops under 0.5 m
   fly straight at 0.25 m/s.
4. **Sweep.** Parallel lines along Y, near the target x. They are spaced so that a box
   anywhere within `--x-window` has a line crossing it at least 5 cm inside its edges.
   With a 0.35 m box and a 0.10 m window that's one line at the target x; with a 0.45 m
   box and a 0.30 m window, two lines at target x ± 0.15 m. Each line starts at the end
   nearest the drone.
5. **Find the center** as soon as a line crosses the box (see below).
6. **Land** on the center at 0.15 m/s.

### In a cluttered room

The drone keeps trying instead of giving up at the first obstacle:

- All travel and all sweeps of a leg share one obstacle memory (side rangers), so later
  hops already know about things seen earlier.
- A line start closer than 0.30 m to anything remembered is moved 10 cm along the line
  until it's clear.
- If the navigator finds no path to a start, or the range guard keeps the drone from
  reaching it, the start is moved 10 cm further along (up to 3 tries). Getting within
  5 cm in x and 15 cm in y of it is good enough. If no start works, that line is skipped.
- If a line sees the box but the center estimate fails (a pass that didn't show the box,
  a pass cut short by a wall), the line is flown again the other way before moving on.
- The leg only fails once every line has been tried.

Leg 1 sweeps at B's x. Leg 2 sweeps around where it remembers box 1 (A). After the trip
out and back the position estimate can be 15–30 cm off, in X as well as Y, so this
sweep allows ±0.30 m in both (`LOCAL_UNCERTAINTY`): three lines for a 35 cm box, at A's
x and ±0.20 m, the middle one first. In mission 114850 a single line at A's x only
grazed box 1. The full `--sweep` range is only flown afterwards if it covers more than
that.

### Finding a box

`box_finder.py` does this part. The Flow deck holds the height above whatever is
underneath, so flying onto a box makes the down ranger (zrange) dip briefly before the
controller climbs, and flying off it makes it spike.

- At the start of each line the drone hovers 1 s, then flies the line at 0.20 m/s.
  All passes use that speed; moves between passes fly at 0.25 m/s, with a 0.6 s hold
  before each pass.
- While flying it measures the floor level (median zrange from 1.5 s to 0.4 s ago,
  skipping the first 0.4 s of the line). A dip of 9 % of the box height, and at least
  1.5 cm, held for 0.2 s, counts as finding the box. For the 16 cm box that's 1.5 cm.
- Until the floor level is known (about the first 1 s, 15 cm) only a deep dip counts:
  4 cm below the hover level for the 16 cm box. The hover reads about 2 cm higher than
  steady flight, so a normal dip there would fire falsely.
- **Along Y:** the line keeps going past the box at the same speed, then the drone flies
  back. The center is where the out and back zrange profiles mirror each other best,
  which cancels the sensor and controller lag.
- **Along X:** one pass across the box through that point, out and back, the same way.
  It only extends as far as the box center can be from the sweep line (half the line
  spacing + 5 cm), plus half a box and 0.20 m.
- After an edge seen while navigating, the same two axes are measured: first along the
  direction it was flying, then across it. The cross pass allows the center to be up to
  half a box off the route, so it is 1.1 m long for a 35 cm box.
- Then it flies to the center and lands.

## Safety landings

The drone lands (and the mission stops) on any of these:

- telemetry older than 0.5 s;
- down ranger stuck on the same value for 1 s (one flight logged 0.501 m for 52 s);
- object above: up ranger under 0.20 m for 0.15 s (a single bad reading is ignored);
- height lost while navigating: zrange off 0.5 m by more than 0.15 m for 1.5 s (not
  while watching for the box, where crossing it is a long dip);
- navigator faults that mean something broke (pose jump, yaw not following);
- every sweep line tried without landing (no box found, or its center not found);
- 300 s on one leg;
- Ctrl+C.

Passes and searches also stop 0.35 m from a wall seen by the side rangers.

## Output

- `mission_log_<time>.csv`: one row per control step (20 Hz) with time, leg, state,
  detector event, map and odometry pose, zrange, wall and up ranges, detector deviation,
  command and current center estimate. A `LANDED` row after each leg and an `END` row at
  the end say what happened.
- `mission_log_<time>.png`: saved at the end, even after a failure. Left: the map with
  each leg's path, wall hits, sweep areas and lines, box detections, the center estimate
  and landing spots. Right: zrange over time with the box-finding periods shaded.
- A live version of the same plot opens in its own window while flying (skipped when no
  display is available; `--no-live` turns it off).

Re-plot a log:

```bash
venv/bin/python mission_plot.py mission_log_<time>.csv --goal 3.25 1.14 --sweep -0.5 0.5
venv/bin/python mission_plot.py mission_log_<time>.csv --live   # follow a log being written
```

## Files

- `mission.py`: mission logic (`Leg`, sweep lines, map anchoring) and the radio loop.
- `box_finder.py`: box detection, center estimate and landing on one box. Also runs on
  its own: `venv/bin/python box_finder.py`.
- `planner.py`: obstacle-avoiding navigator used for the travel between the points.
- `mission_plot.py`: live and saved plots.
- `test_mission.py`: closed-loop simulation of whole missions (two boxes, drift, walls,
  box at the edge of the window, box found while navigating straight, along Y and
  diagonally, fallback to the sweep and to the full sweep, no box).
- `test_box_finder.py`: box finding in simulation, plus replays of real logs.

## Limits

- The 16 cm box only made 2–4 cm dips at 0.15 m/s (more at the current 0.20 m/s),
  against 1–1.5 cm of floor noise. Cluttered floors, cables or other low objects in the
  sweep range can be mistaken for the box. Raising `PASS_SPEED` in `box_finder.py`
  makes the dips deeper (both directions must use the same speed).
- Something low on the floor near the target can trigger the watch during the approach.
  The center search then fails ("box not seen on a pass") and the leg sweeps, which
  costs about 20 s.
- With fewer navigator scans, an obstacle that appears beside the path between scans is
  only caught by the range guard (which slows and backs away within 25 cm).
- Position comes from the Flow deck alone and drifts 2–18 cm between passes. Each axis
  of the center uses one out-and-back pass, so drift during that pass ends up in the
  estimate. Use a textured floor and good lighting.
- Objects are only seen at ranger height, and only in four directions (the navigator
  turns ±45° to scan; the box finding doesn't turn).
- Box 2 must actually be within `--x-window` of B's x and inside the sweep range; the
  drone doesn't look anywhere else.

## Known issues

Both are from mission 20261018_175000 (B at (1.35, 0), wall at x ≈ 1.8) and are not
fixed yet:

- **Approach goal inside a wall.** The approach aims 0.55 m past the target. If that point
  is in or next to a wall, the navigator has no path and reports `NO_PATH`. It still
  flies toward the goal, but the box watch only runs in `DIRECT`, `DETOUR` and `BLOCKED`,
  so it crossed the box without looking. After 10 s it gave up and the sweep found the
  box (55 s for the leg). The planned fix: watch in `NO_PATH` too, and pull the goal back
  to the target when obstacles show up near it.
- **Radio link lost between legs.** "Too many packets lost" while sitting on box 2 left
  the Crazyflie disconnected, and leg 2's `take_off()` raised "Crazyflie is not
  connected". The log and plot were still saved. The planned fix: check the link before
  each takeoff and reconnect (the drone is on the ground then), or stop with a clear
  message.
