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
- Put box 2 so that its center is within `--x-window` (default 0.30 m) of B's x, and
  inside B's Y sweep range.
- The Y sweep range (`--sweep DY_MIN DY_MAX`, default `-0.5 0.5`) is relative to each
  point's own y: with B at y = 1.14 it sweeps y = 0.64 .. 1.64 at B, and y = -0.5 .. 0.5
  at A. Both ends should be off the box, so each sweep line starts over bare floor.
- Keep about 0.5 m between a box edge and any wall. The passes stop 0.35 m from a wall,
  and a pass cut short can make the center estimate fail.
- Both boxes are square and the same size: `--box-size` (side, default 0.45 m) and
  `--box-height` (default 0.16 m).

## Options

| Option | Default | Meaning |
|---|---|---|
| `uri` | `radio://0/98/2M/E7E7E7E7E8` | Crazyflie radio address |
| `--start X Y` | `0 0` | Point A: box 1, where the drone takes off |
| `--goal X Y` | `3.25 1.14` | Point B: box 2 is near this x |
| `--sweep DY_MIN DY_MAX` | `-0.5 0.5` | Y range swept at each end, relative to that point's y |
| `--x-window` | `0.30` | The box center is within this of the target x, m |
| `--box-size` | `0.45` | Side of the square box, m |
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
2. **Travel** to the start of the first sweep line with the obstacle-avoiding navigator
   from `planner.py` (the same one `main.py` uses). Hops under 0.5 m fly straight.
3. **Sweep.** Parallel lines along Y, near the target x. They are spaced so that a box
   anywhere within `--x-window` has a line crossing it at least 5 cm inside its edges.
   With a 0.45 m box and a 0.30 m window that's two lines, at target x ± 0.15 m.
   Each line starts at the end of the range nearest the drone; later lines start where
   the previous one ended, in case a wall cut it short.
4. **Find the center** as soon as a line crosses the box (see below).
5. **Land** on the center at 0.15 m/s.

Leg 1 sweeps at B's x. Leg 2 first sweeps a short Y range around where it remembers
box 1 (A, which can be 15–30 cm off after drift), and only falls back to the full sweep
at A's x if that finds nothing.

### Finding a box

`box_finder.py` does this part. The Flow deck holds the height above whatever is
underneath, so flying onto a box makes the down ranger (zrange) dip briefly before the
controller climbs, and flying off it makes it spike.

- At the start of each line the drone hovers 1.5 s, then flies the line at 0.15 m/s.
- While flying it measures the floor level (median zrange from 1.5 s to 0.4 s ago). A
  dip of 9 % of the box height, and at least 1.5 cm, held for 0.2 s, counts as finding
  the box. For the 16 cm box that's 1.5 cm. The first 0.9 s of a line (about 13 cm) can't
  trigger, because the hover reads about 2 cm higher than steady flight.
- **Along Y:** the line keeps going past the box at the same speed, then the drone flies
  back. The center is where the out and back zrange profiles mirror each other best,
  which cancels the sensor and controller lag.
- **Along X:** one pass across the box through that point, out and back, the same way.
- Then it flies to the center and lands.

## Safety landings

The drone lands (and the mission stops) on any of these:

- telemetry older than 0.5 s;
- down ranger stuck on the same value for 1 s (one flight logged 0.501 m for 52 s);
- object above: up ranger under 0.20 m for 0.15 s (a single bad reading is ignored);
- height lost while navigating: zrange off 0.5 m by more than 0.15 m for 1.5 s;
- navigator faults (no path, pose jump) or a hop it can't finish in 15 s;
- no box on any sweep line, or a center estimate that fails;
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
  box at the edge of the window, fallback to the full sweep, no box).
- `test_box_finder.py`: box finding in simulation, plus replays of real logs.

## Limits

- The 16 cm box only makes a 2–4 cm dip at 0.15 m/s, against 1–1.5 cm of floor noise.
  Cluttered floors, cables or other low objects in the sweep range can be mistaken for
  the box. If it misses the box, raising `PASS_SPEED` in `box_finder.py` makes the dips
  deeper (both directions must use the same speed).
- Position comes from the Flow deck alone and drifts 2–18 cm between passes. Each axis
  of the center uses one out-and-back pass, so drift during that pass ends up in the
  estimate. Use a textured floor and good lighting.
- Objects are only seen at ranger height, and only in four directions (the navigator
  turns ±45° to scan; the box finding doesn't turn).
- Box 2 must actually be within `--x-window` of B's x and inside the sweep range; the
  drone doesn't look anywhere else.
