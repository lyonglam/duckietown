# Duckiebot Grid Navigation - Project Summary

Internal summary for the team. Not part of the repository.

Robot: `duck4` (shared TUM bot). Branch: `person1-navigation` on
`github.com/lyonglam/duckietown`.

---

## What it does

Pick a start tile, the direction the bot is facing, and a destination on a grid map.
A* produces the list of turns. The robot lane-follows between intersections, stops at
each red line, executes the next turn, and repeats until the route is finished. If an
object blocks the lane it slides into the oncoming lane, passes it, and comes back.

---

## Files (all pushed to the branch)

| File | Runs on | Purpose |
|---|---|---|
| `a_star.py` | laptop / VM | A* over the tile graph, outputs the turn list |
| `navigator.py` | the robot | lane following, stop lines, turns, obstacle avoidance, HTTP API |
| `lane_follower.py` | the robot | standalone lane following, for tuning |
| `hsv_calibrator.py` | VM (needs a screen) | slider tool to tune colours for the lab lighting |
| `map_ui.html` | any browser | click-to-plan grid UI, sends the route to the robot |

Kept out of the repo: this summary, the lab checklist, and the working notes.

---

## How the pieces connect

    map_ui.html  --(A* in browser)-->  POST /navigate {"commands":[...]}
                                              |
                                              v
                                        navigator.py on duck4
                                              |
                        camera --> lane following --> wheels
                        ToF    --> obstacle check

`navigator.py` serves the same `/navigate` contract Person 2 already defined in
`duckiebot_server.py`, so the app needed no changes. It also serves `map_ui.html`
itself, so the UI is reachable at `http://duck4.local:8083/` from anywhere on the
network including the VM.

Endpoints: `POST /navigate`, `GET /status`, `POST /abort`, `POST /follow`
(lane-follow with no route, useful for testing).

---

## The three decisions that mattered

**1. A* searches (row, col, FACING), not just tiles.**
A route without heading cannot tell left from right. With heading in the state, the
turn at each step falls out of comparing the incoming and outgoing direction.
Commands are emitted ONLY at the five intersections; curves are driven by lane
following, so emitting a command for one would desync the whole plan.

**2. Turns end when the bot sees a lane again, not after a fixed time.**
Timed turns could not be tuned: a tight right and a wide left need different amounts
of rotation. Rotating until a lane is visible ahead is self-correcting and removed
the tuning problem entirely.

**3. Obstacle detection uses range, not colour.**
See below - this was the hardest part.

---

## Obstacle avoidance: why the camera approach failed

The duckies are the same colour family as the yellow centre line, so colour alone
finds the centre line. Two shape-based detectors were built and tested on the robot:

- Upright bounding box aspect ratio plus fill ratio. Failed: a lane dash is a solid
  rectangle of tape, so it fills its box as completely as a duckie fills its own.
- Rotated bounding box elongation (`cv2.minAreaRect`). Failed as well. Logged on the
  robot, the SAME dashes measured elongation 1.1, 1.4, 1.7, 2.2 and 2.4 on
  consecutive frames. Perspective foreshortening squashes a nearby dash into a
  near-square blob, so elongation is not a stable property.

A position test (the blob must sit clear of the centre line) did not rescue it
either: dashes measured 44 to 84 px right of the detected line while the bot sat
offset in its lane.

Worse, misclassified dashes were being erased from the yellow mask to protect the
steering, which left nothing to steer by, so lane following itself broke.

**What previous TUM batches did:** Safe-Navigation trained a neural object detector.
Duckietown-Btown used the time-of-flight range sensor
(`packages/obstacle_detection/src/tof_obstacle_detection_node.py`). Nobody solved it
with classical colour or shape vision.

**Our solution:** the ToF sensor. A duckie stands up and returns a short range; flat
tape on the road does not. Detect at 0.35 m, clear at 0.50 m - the gap is hysteresis
so a reading sitting on the limit cannot chatter the manoeuvre on and off. Crucially
this never touches the vision pipeline, so a bad reading cannot break lane following.

---

## Tuning knobs

Lane following (`lane_follower.py` and the top of `navigator.py`):

- `KP` wanders -> raise; snakes side to side -> lower
- `KD` raise to damp oscillation
- `V_BAR` base speed
- `RED_BAND_TOP` raise to stop closer to the red line

Intersections:

- `CREEP_LEFT_S` / `CREEP_RIGHT_S` how far into the junction before rotating.
  Right is tighter so it rotates earlier.
- `MIN_/MAX_LEFT_TURN_S`, `MIN_/MAX_RIGHT_TURN_S` guards on the rotation

Obstacle avoidance:

- `AVOID_SHIFT_FRAC` HOW FAR IT SWERVES. 0.50 aggressive, 0.35 default, 0.15 barely.
- `TOF_DETECT_M` / `TOF_CLEAR_M` when it reacts and when it counts as clear again
- `AVOIDANCE_ENABLED = False` turns the whole feature off in one line

---

## Known limits

- **No AprilTags.** The robot stays in sync with the plan by counting red stop lines.
  One missed or double-counted line makes every later turn wrong. Guards are in place
  (minimum red area, cooldown after a turn) but this is the main fragility.
- **Stop lines are ignored during a swerve**, so an obstacle placed right before an
  intersection could cause a missed count. Do not set the demo course up that way.
- **Colour thresholds are lighting dependent.** Re-run `hsv_calibrator.py` if the lab
  lighting changes.
- **The robot's base image is newer than daffy**, so the official
  `indefinite_navigation` demo does not run on it (`led_emitter_node` is missing and
  `fsm_node` dies). Our stack bypasses the demo entirely, which is why this was never
  a blocker.

---

## Running it

On the robot:

    ssh duckie@duck4.local
    docker exec -it duckiebot-interface bash
    source /environment.sh && export VEHICLE_NAME=duck4
    python3 /data/navigator.py

Then open `http://duck4.local:8083/` in a browser, click a start tile, set the
facing, click a destination, press Send. Ctrl-C or `POST /abort` stops the robot.
