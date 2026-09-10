# Duckiebot Grid Navigation

Autonomous grid navigation for a Duckiebot (DB21, robot name `duck4`). Give it a
destination tile; it works out where it is from an AprilTag, plans a route with A\*,
and drives there — following its lane by camera, stopping at every red line, turning
the right way at each intersection, and swerving around obstacles it sees with the
time-of-flight range sensor.

TUM Duckietown Practical Course, Campus Heilbronn.

## Why this is a custom stack

`duck4` runs a newer base image than the `daffy` release the official
`indefinite_navigation` demo targets. That demo's `fsm_node` is a required node and
calls `led_emitter_node/set_pattern` on startup — a service the newer
`dt-duckiebot-interface` no longer provides. `fsm_node` dies, and roslaunch tears the
whole demo down with it.

The drivers underneath are fine, so this project talks only to those:

```
camera_node/image/compressed  ─┐
front_center_tof_driver/range ─┤→ navigator.py →  car_cmd_switch_node/cmd
                               │                  → kinematics_node → wheels
browser (map_ui.html) ─HTTP────┘
```

Publishing to `car_cmd_switch_node/cmd` injects commands *downstream* of the
fsm-driven switch, so nothing depends on `fsm_node` or `led_emitter_node`. Commands
still pass through `kinematics_node`, so wheel trim/gain calibration is applied.

## Running it

Everything runs on the robot — streaming video to a VM adds too much latency.

```bash
ssh duckie@duck4.local
docker exec -it duckiebot-interface bash
source /environment.sh          # if ROS_MASTER_URI is empty
export VEHICLE_NAME=duck4
python3 /data/navigator.py
```

Then open `http://duck4.local:8083/` in a browser. Ctrl-C stops the robot — it
publishes zero velocity on shutdown.

**Before the first run:** take the lens cap off, and tune the HSV thresholds for your
lab's lighting with `hsv_calibrator.py` (writes `/data/hsv_thresholds.json`).
Without it the built-in defaults are used and it will warn loudly.

### Environment variables

| Variable | Default | Purpose |
|---|---|---|
| `VEHICLE_NAME` | `duck4` | ROS topic namespace |
| `NAV_PORT` | `8083` | HTTP server port |
| `HSV_FILE` | `/data/hsv_thresholds.json` | calibrated colour thresholds |
| `TOF_TOPIC` | `/<vehicle>/front_center_tof_driver_node/range` | range sensor |
| `LIVE_TRACKING` | `0` | set `1` for live AprilTag position on the map |

### HTTP API

| Endpoint | Does |
|---|---|
| `POST /navigate` | run a plan: `{"commands": ["right","straight","left","stop"]}` |
| `GET /status` | state, remaining commands, ToF range, live tile |
| `POST /abort` | stop and clear the plan |
| `POST /follow` | lane-follow only, no route |
| `POST /localize` | read the current tile's AprilTag (robot must be idle) |

## Planning a route

`a_star.py` runs anywhere — it has no ROS dependency.

```bash
python3 a_star.py                 # self-tests
python3 a_star.py 6 0 N 0 5       # plan (6,0) facing North to (0,5)
```

It searches over `(row, col, facing)` rather than tile position, because a turn
direction depends on which way you arrive. Manhattan distance is the heuristic
(admissible — every edge is one tile step), with a small turn penalty to break ties
toward straighter routes.

Commands are emitted **only at the five real intersections**. Curves and straights are
driven by lane following and get no command — an extra command would desynchronise
every later turn from the road.

The self-test plans all 1980 `(start tile, start facing, destination)` combinations and
asserts each is well-formed and U-turn-free.

## How the robot stays in sync

The navigator is a state machine:

```
LANE_FOLLOW  -> normal driving, watching for a red stop line
AT_STOP_LINE -> red seen: brake, pause, pop the next command
TURNING      -> creep in, then rotate until a lane is visible ahead again
STUCK        -> lane lost too long: stop, watch for the lane, warn the operator
DONE         -> plan finished: sit still
```

**Route progress is tracked by counting red stop lines.** One missed line, or one
counted twice, and every later turn is wrong. Two guards exist — `RED_COOLDOWN_S`
ignores red for a while after a turn, and `RED_MIN_AREA` must be exceeded — but if you
keep desynchronising, that's the argument for using the AprilTags for correction (see
Known limitations).

## Files

**On the robot**

| File | Does |
|---|---|
| `navigator.py` | the whole robot-side stack: lane following, stop lines, turns, obstacle avoidance, localisation, HTTP server |
| `lane_follower.py` | standalone lane follower — same gains, for tuning without a route |
| `led_emitter_shim.py` | stub for the missing `set_pattern` service, if you want the stock demo to start |
| `run_logger.py` | append-only event log to `/data/run_log.jsonl` |
| `apriltag_map.json` | tag ID to tile lookup, 30 tiles |

**Off the robot**

| File | Does |
|---|---|
| `a_star.py` | route planner + self-tests |
| `map_ui.html` | browser UI — pick a destination, watch the route and live position |
| `hsv_calibrator.py` | interactive HSV threshold tuning |
| `apriltag_scan_test.py` | standalone tag detection check, before trusting `/localize` |
| `analyze_runs.py` | `run_log.jsonl` to `commands_by_type.png` + `routes.csv` |
| `fake_log_generator.py` | synthetic log so the analyser can be tested with no robot |

Lane following is duplicated in `navigator.py` rather than imported from
`lane_follower.py`, so the navigator is one file you can paste onto the bot and run as
one process. Gains and thresholds transfer directly between the two.

### Earlier work kept for reference

`map_graph_prototype.py` (the original map graph — `a_star.py`'s map data comes from
it), `duckiebot_server.py` and `mock_duckiebot_server.py` (the laptop-side server
whose HTTP contract `navigator.py` matches), `map_editor_v2.html`,
`obstacle_detector_node.py` and `src/lane_following/` (early camera-based obstacle
work, superseded by the range sensor — see below).

## Obstacle avoidance, and why it isn't camera-based

Rubber duckies are the same colour family as the yellow centreline, so colour alone
finds the centreline. Two shape-based detectors were built and tested on the robot, and
both failed:

- **Bounding-box aspect ratio + extent.** A lane dash is a solid rectangle of tape, so
  it fills its own box about as completely as a duckie fills its own (~0.95 vs ~0.85).
- **Rotated bounding-box elongation** (`cv2.minAreaRect`). Should read 3:1+ for a dash
  and ~1:1 for a duckie. The *same* dashes measured 1.1 to 2.4 across consecutive
  frames: perspective foreshortening squashes a near dash into a near-square blob.

Worse, misclassified dashes were being erased from the yellow mask to protect the
steering — which left nothing to steer by, so lane following itself broke.

The time-of-flight sensor sidesteps all of it: a duckie stands up and returns a short
range, flat tape on the road returns nothing. It's also a signal independent of the
camera, so a bad reading cannot corrupt the lane-centering measurement. Detection
triggers below 0.35 m and clears past 0.50 m (hysteresis, so a reading hovering at the
limit can't chatter), and five consecutive short readings are required before
believing it — a printed AprilTag reflects similarly to a duckie, but only for the one
frame you drive over it.

Avoidance stays in closed-loop lane following throughout; only the steering *target* is
ramped across the centreline and back, rather than running an open-loop manoeuvre.

## Known limitations

- **The AprilTags are not physically placed yet.** `apriltag_map.json` and `/localize`
  are complete in software; validate with `apriltag_scan_test.py` once the tags are up.
- **Tag position is displayed, not used.** Route progress is still stop-line counting.
  Using the tag to correct a disagreement is the obvious next step.
- **Turn completion ends on lane visibility**, not wheel encoders.
- **Obstacles are swerved around, never waited for.** No stop-and-wait behaviour.

## Attribution

Lane following uses HSV masking, `cv2.moments` centroid, then PD steering — a standard
approach. Two things were taken from the TUM QuackQuack team's `simple_lane_follower`
and are credited in `lane_follower.py`: publishing to `car_cmd_switch_node/cmd` to
inject commands downstream of the FSM switch, and initial HSV values (since re-tuned).
The range-sensor approach follows Duckietown-Btown's `tof_obstacle_detection_node`,
credited in `navigator.py` — their thresholds are for stopping, ours for swerving.

Map data originates from `map_graph_prototype.py`.
