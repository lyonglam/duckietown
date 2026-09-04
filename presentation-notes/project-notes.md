# Project Doc / Handoff — Duckiebot Grid Navigation (A* + Lane Following)

## Quick status (read this first)
- All THREE calibrations DONE: camera intrinsic (0.06), camera extrinsic (0.18), wheel/kinematics
  (saved as /data/config/calibrations/kinematics/duck4.yaml, dated today).
- `indefinite_navigation` demo RUNS and the bot HAS lane-followed (briefly) — the calibrated
  pipeline works. Lane following + red-line stop is the foundation; turning at intersections
  is the next phase (needs AprilTags placed).
- AprilTags: APPROVED by tutor, NOT yet printed/placed.
- OPEN PROBLEM (root cause FOUND 2026-06-30): the demo crashes because the bot's base image is
  version-INCOMPATIBLE with the daffy demo. duckiebot-interface v4.3.2 dropped led_emitter_node
  (set_pattern API) in favor of led_driver_node (switch API), but daffy indefinite_navigation's
  fsm_node REQUIRES /duck4/led_emitter_node/set_pattern. Service missing -> fsm_node dies ->
  whole demo collapses. NOT OOM, NOT network. duck4 is a SHARED bot -> waiting on tutor before
  any reflash. See "Open problem / ROOT CAUSE" below.

## Environment
- Ubuntu VM in VirtualBox. Bridged Adapter -> Windows Mobile Hotspot adapter (192.168.137.x).
- VM sometimes needs: `sudo ip link set enp0s3 up` then `sudo dhclient enp0s3`
  (if dhclient hangs forever -> hotspot is OFF or bridged to wrong/stale adapter; can use a
  static IP fallback: `sudo ip addr add 192.168.137.50/24 dev enp0s3` + default route via .137.1).
- Distro: daffy. Robot: duck4 (use `duck4`, NO `.local`, in dts commands).
- Robot IP seen this session: 192.168.137.97. SSH: `ssh duckie@duck4.local` (or the IP).
- duckietown-shell 6.2.23 (6.2.26 available; upgrade later with `pipx upgrade duckietown-shell`).

## LESSONS LEARNED THIS SESSION (avoid repeating these)
1. LENS CAP must be OFF before lane following — with it on, the camera sees black and the bot
   won't drive even though all nodes are up. (Calibration is stored, so cap-on doesn't break cal.)
2. The keyboard GUI window DROPS the 'a' button press inside the VM (arrows/axes work, buttons
   don't). FIX: use CLI mode -> `dts duckiebot keyboard_control duck4 --cli`, keep that terminal
   focused, press keys there.
3. There is NO `--stop` flag on the demo command (my earlier error). To STOP the demo, stop its
   Docker container on the robot:
     ssh duckie@duck4.local  ->  docker ps  ->  docker stop demo_indefinite_navigation
   (or use Dashboard -> Portainer -> stop the container).
4. Keyboard `e` (e-stop) / `s` (stop) are UNRELIABLE for stopping — the reliable brake is
   `docker stop` on the demo container. Keep an SSH session with `docker ps` ready BEFORE
   pressing 'a'.
5. Do NOT re-run the demo command repeatedly — it runs ON THE BOT and hands the prompt back
   immediately. Re-running causes stuck-container 409 conflicts ("removal already in progress").
6. Web-API reboot (port 80) can fail with "Connection refused" when services are degraded.
   Reboot via SSH instead: `ssh duckie@duck4.local` then `sudo reboot`.
7. The Jetson Nano fan is temperature-controlled (spins ~>51C). A still fan is NORMAL, not a fault.

## Keyboard control keys (daffy)
- Arrow keys = manual drive.   'a' = start autonomous lane following.   's' = stop autonomous.
- 'e' = emergency stop (unreliable here — prefer docker stop).
- 'a' makes it AUTO-drive; arrows are ignored once in autonomous mode.

## Standard run sequence (current best-known-good)
1. `ping duck4.local` (confirm on network).
2. Clean slate: SSH in -> `docker ps` -> `docker stop <demo container>` if one is running.
3. Start demo: `dts duckiebot demo --demo_name indefinite_navigation --duckiebot_name duck4`
   (prompt returns fast; demo runs on bot). WAIT 3-5 min. Do NOT re-run.
4. Verify up: in `dts start_gui_tools duck4` -> `rosnode list` shows fsm_node,
   lane_controller_node, stop_line_filter_node, line/lane nodes, unicorn_intersection_node,
   apriltag_detector_node, random_april_tag_turns_node.
5. LENS CAP OFF. Place bot in a lane (straight/curve, both lines visible, good lighting).
6. `dts duckiebot keyboard_control duck4 --cli`, focus that terminal, press 'a'.
7. Brake = `docker stop` the demo container (have SSH+`docker ps` ready first).

## Open problem / ROOT CAUSE (found 2026-06-30)
Symptom: indefinite_navigation crashes; fsm_node dies and takes the whole demo down (big
"killing on exit" cascade). Often shows up right when keyboard control is opened.

ROOT CAUSE (confirmed at every layer): base-software VERSION MISMATCH.
- fsm_node calls service /duck4/led_emitter_node/set_pattern on startup / on state change
  (e.g. keyboard control switching to NORMAL_JOYSTICK_CONTROL). fsm_node is a REQUIRED roslaunch
  node, so when it dies roslaunch shuts down ALL demo nodes -> that's the cascade, not a crash
  of individual nodes.
- That service does NOT exist on the bot. There is NO set_pattern service anywhere.
- The bot runs dt-duckiebot-interface:v4.3.2, which provides led_driver_node (services: switch,
  get/request_parameters, set_logger_level, tests/...) and has NO led_emitter_node. The
  led_emitter code is not even in the image (find /code -iname 'led_emitter*' = empty).
- v4.3.2 is NEWER than daffy and replaced the daffy led_emitter (set_pattern API) with
  led_driver (switch API). Per daffy docs, led_emitter_node belongs in dt-duckiebot-interface,
  so the daffy demo and this bot's interface are on INCOMPATIBLE versions.
- Version soup confirms drift: duckiebot-interface v4.3.2, car-interface v4.1.0, ros v4.3.0
  (NOT a matched daffy set). car-interface all.launch only starts joy_mapper, kinematics,
  velocity_to_pose, car_cmd_switch (no led_emitter).

Ruled out (do NOT chase these again):
- NOT OOM: no Exited (137); fsm dies exit code 1 from a ServiceException, clean cascade.
- NOT network: the service is genuinely absent even on a stable local check.
- NOT a stopped/wrong core container: exactly one car-interface and one duckiebot-interface exist.
- `dts duckiebot update duck4` does NOTHING here (core containers are pinned to version tags,
  not the rolling daffy tag) -> reports "Nothing to do".

FIX (decided 2026-06-30): duck4 is a SHARED TUM bot -> check with tutor BEFORE changing base
software. Real fix = get the base image onto a consistent daffy version (most reliable =
reflash SD card with daffy). BEFORE any reflash, BACK UP /data/config/calibrations/ (camera
intrinsic + extrinsic + kinematics/duck4.yaml) and restore after, so the 3 calibrations aren't
lost. Questions for tutor: was duck4 recently reflashed/updated? should it be on daffy? do
other course bots have the same problem? who is allowed to reflash?

## PLAN B (probably now PLAN A): our OWN lane follower, skip the demo entirely
Decided 2026-08-12. Other TUM teams wrote their own lane followers instead of using the
demo - see the DuckietownTUM org (https://github.com/DuckietownTUM). Survey done:
- QuackQuack = BEST MODEL. packages/my_lane_following/src/simple_lane_follower.py is a
  self-contained ROS node: bottom-35% ROI -> HSV masks yellow+white -> cv2.moments
  centroids -> lane center -> PD (kp=5.0, kd=2.5, v_bar=0.12) -> Twist2DStamped.
  KEY TRICK: it publishes to /VEHICLE/car_cmd_switch_node/cmd, which injects commands
  DOWNSTREAM of the fsm-driven switch -> does NOT need fsm_node or led_emitter, but
  STILL goes through kinematics_node so wheel calibration applies. Perfect for our
  broken stack.
- Safe-Navigation = NOT useful (it's a fork of Duckietown's full complete_image_pipeline,
  not custom code). dt-meow (traffic signs), Self-Navigation, DuckiePilot, QuackSquad
  not reviewed yet - look there later for INTERSECTION/turn handling if needed.

OUR FILES (in this folder, ready to copy to the bot's /data):
- lane_follower.py  -> custom follower. camera -> HSV -> PD -> car_cmd_switch_node/cmd.
                       Also stops at red lines. Bypasses fsm_node/led_emitter entirely.
- hsv_calibrator.py -> live trackbar HSV tuner (like the labyrinth project's tuner).
                       Saves /data/hsv_thresholds.json, which lane_follower.py auto-loads.

RUN ORDER NEXT SESSION:
1. Boot bot. Confirm core containers up (ros, duckiebot-interface, car-interface).
   NO demo needed - do NOT start indefinite_navigation.
2. Copy both .py files to the bot's /data (scp or cat > /data/... << 'EOF').
3. LENS CAP OFF. Tune colors first (needs GUI, so run in the VM):
     dts start_gui_tools duck4   ->   python3 hsv_calibrator.py
   keys: 1=yellow 2=white 3=red, s=save, q=quit. Tune until mask shows ONLY that color.
4. Run the follower ON THE BOT:
     ssh duckie@duck4.local -> docker exec -it duckiebot-interface bash
     source /environment.sh ; export VEHICLE_NAME=duck4 ; python3 /data/lane_follower.py
   Ctrl-C = brake (it publishes zero velocity on shutdown).
5. Tune gains: wanders->raise KP; snakes/oscillates->lower KP or raise KD; too fast->lower V_BAR.
VERIFY FIRST on the bot: `rostopic list | grep car_cmd` and `rostopic info
/duck4/car_cmd_switch_node/cmd` to confirm the topic name + that kinematics_node subscribes.
If that injection point doesn't drive, fall back to publishing WheelsCmdStamped directly to
/duck4/wheels_driver_node/wheels_cmd (loses calibration, but always works).
NEXT PHASE after this works: red-line stop is already in; add AprilTag intersection turns +
A* turn list on top (the actual project goal).

## PHASE 2: A* NAVIGATION (built 2026-08-12, untested on hardware)
Teammate's repo github.com/lyonglam/duckietown is now PUBLIC. It already fixes the
interfaces, so we build to them:
- map_graph_prototype.py: tile_connections[(row,col)][facing_in] -> facing_out or [list].
  DIRECTIONS N=(-1,0) S=(1,0) E=(0,1) W=(0,-1). get_neighbors() only - NO pathfinding.
- duckiebot_server.py: Flask :8083, POST /navigate, {"commands":["straight","left",
  "right","stop"]} -> {"status":"executed","commands_processed":N}. execute_bot_command()
  is a stub.
- map_editor_v2.html = the grid UI. behavior_logic.py = lane-following behavior.
Map verified: 7x6, fully connected (1980/1980 routes), 5 intersections
(2,5)(4,0)(4,3)(4,5)(6,3) - matches our map notes exactly.

DECIDED: sequence-counting first, NO AprilTags for v1 (see below).

a_star.py (runs anywhere, no ROS):
- KEY IDEA: search state is (row, col, FACING), not just the tile. A route without
  heading can't tell you left from right. Turn = compare facing_in vs facing_out
  (clockwise N->E->S->W, so +1 = right, -1 = left).
- Emits a command ONLY for tiles in INTERSECTIONS. Curves/straights emit NOTHING because
  lane following just drives them - an extra command would desync the whole plan.
- Small TURN_PENALTY (0.3) prefers straighter routes; each open-loop turn is a drift risk.
- Rejects invalid start facings with a clear message (e.g. corner tile (6,0) has no 'N'
  entry, so "at (6,0) facing N" is not a real state - the UI should grey these out).
- Self-tested: python3 a_star.py  (turn logic, map sanity, 1980 routes well-formed).
  CLI: python3 a_star.py 6 0 W 0 5  -> prints route + ready-to-POST JSON.

navigator.py (runs ON the bot, replaces the demo entirely):
- State machine LANE_FOLLOW -> AT_STOP_LINE -> TURNING -> back, driven by red-line
  detection. Includes lane following inline (copied from lane_follower.py) so it's ONE
  file / ONE process to paste and run.
- Serves the SAME contract as the teammate's server, so the app needs NO changes:
  POST http://duck4.local:8083/navigate, GET /status, POST /abort. Uses built-in
  http.server (NOT Flask - the bot's containers may not have Flask).
- Turns are OPEN-LOOP TIMED (creep into the intersection, then arc). CREEP_S,
  LEFT_TURN_S, RIGHT_TURN_S, TURN_OMEGA etc MUST be tuned on the real bot with a
  stopwatch. Expect this to be the fiddliest part.
- BIGGEST RISK: with no AprilTags the robot stays in sync ONLY by counting red stop
  lines in order. One missed or double-counted line makes every later turn wrong.
  Guards: RED_MIN_AREA (ignore specks) + RED_COOLDOWN_S (can't retrigger on the line
  just crossed). If desync keeps happening -> that is the argument for AprilTags.

APRILTAGS - deliberately deferred, and a warning:
Do NOT let Claude generate AprilTag images. tag36h11 is a fixed codebook of 587 exact
bit patterns; a fabricated pattern won't detect or will decode as the WRONG id, and you
only find out after printing and mounting. Get official images from
github.com/AprilRobotics/apriltag-imgs (or Duckietown's own signage) and verify the
printed size against the Duckietown spec before printing a whole sheet.
Also note: the demo's apriltag_detector_node is part of the BROKEN demo, so going the
AprilTag route also means running our own detector (dt_apriltags / pupil_apriltags).

## PHASE 3: OBSTACLE AVOIDANCE -- ATTEMPTED AND REVERTED (documented negative result)
STATUS: removed from navigator.py. Lane following + A* navigation are UNAFFECTED and
remain the working deliverable. This is written up because the negative result is worth
reporting -- we have measured evidence, not a guess.

Goal: detect the rubber duckies and swerve into the oncoming lane around them.
Blocker: duckies are the SAME COLOUR FAMILY as the yellow centre line, so colour alone
finds the centre line. Two shape-based detectors were built and tested on the real bot:

  attempt 1 -- upright bounding box aspect ratio + extent (contour area / box area).
     FAILED. A lane dash is a SOLID RECTANGLE of tape, so it fills its bounding box as
     completely as a duckie fills its own (extent ~0.95 vs ~0.85). Raising extent to 0.85
     changed nothing, because the test never separated the two classes in the first place.

  attempt 2 -- rotated bounding box (cv2.minAreaRect) elongation = long side / short side.
     Should hug a dash at any angle and read 3:1+, vs ~1:1 for a duckie.
     FAILED. Logged from the bot, the SAME dashes measured elongation
        1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7, 2.2, 2.3, 2.4
     across consecutive frames. Perspective foreshortening squashes a near dash into a
     near-square blob. Elongation is not a stable property of these objects.

  position fallback (blob must sit clear of the centre line) ALSO failed: dashes measured
  44-84 px right of the detected line (line_x=307, blobs at x=351 and x=391) while the bot
  sat offset in its lane -- well past the 25 px margin.

Worse failure mode: misclassified dashes were being erased from the yellow mask (to stop
an obstacle dragging the steering), which left nothing to steer by. Lane following itself
broke -- "lane lost -- holding turn" -- and the bot swerved before ever reaching a duckie.

CONCLUSION for the report: reliable duckie detection needs information this pipeline does
not have. Options if picked up again:
  - DEPTH. The bot has a front ToF sensor (front_center_tof_driver_node) -- a standing
    object returns a range, flat tape does not. Probably the cheapest real fix.
  - Size-vs-image-position geometry: with the extrinsic calibration, a blob's expected
    size at its ground position is known; a standing object breaks that relation.
  - A trained detector (the course's own duckie datasets exist).
Not solvable by thresholding a single ROI, which is what we had.

## (superseded) original avoidance design notes
Problem: duckies are the SAME COLOUR FAMILY as the centre line, so a colour-only detector
just finds the centre line. Worse, a duckie in our lane drags the yellow centroid toward
itself and steers us INTO it.
Solution = SHAPE, not colour:
- candidate blobs from a wider/darker yellow ("duckie" threshold, optional to tune)
- keep only compact SOLID blobs: aspect ratio 0.45-2.20 AND extent (area/bbox) > 0.45.
  A long dash fails on aspect; a DIAGONAL dash has a square bbox but fails on extent.
  That extent test is the one that matters -- verified in simulation.
- red beak in the blob's box relaxes the trigger area by DUCKIE_RED_BONUS
- detected blobs are ERASED from the yellow mask BEFORE the centre line is measured
- only blobs RIGHT of the centre line (our lane) and big enough (close) trigger a swerve
Swerve = NOT a scripted manoeuvre. We stay in closed-loop lane following and slide the
TARGET across the centre line via `ramp` 0->1:
    lane_center = yc + width*HALF_LANE_FRAC*(1 - 2*ramp)
    ramp 0 = our lane, 0.5 = on the line, 1 = oncoming lane. ~0.6s to slide out.
White-edge logic is skipped mid-swerve (geometry mirrors once past the centre line).
Stop lines are IGNORED mid-swerve to protect the plan counter -- so do NOT put a duckie
right before an intersection.
Test without a route: POST /follow (lane-follow only, no plan).
New in /status: "avoiding" = current ramp value.

## STILL AVAILABLE: led_emitter shim (only if we want the ORIGINAL demo back)
Idea: fsm_node doesn't need LEDs to WORK, only needs the set_pattern service to EXIST and
return success. So run a tiny fake led_emitter_node that answers set_pattern with "OK". Then
the daffy demo stops crashing and lane following can run. Keeps calibrations, no brick risk.
Script: led_emitter_shim.py (in this project folder, next to this doc).

Run procedure (run the shim ON THE BOT so there are no VM<->bot network issues):
1. Put the script on the bot. SSH in and paste it into /data (which is mounted into the
   containers). From the VM:
     ssh duckie@duck4.local
   then on the bot create the file (paste script body between the EOF markers):
     cat > /data/led_emitter_shim.py << 'EOF'
     ...paste contents of led_emitter_shim.py...
     EOF
2. Enter a container that has ROS + duckietown_msgs and is on the bot's master:
     docker exec -it duckiebot-interface bash
   Inside, make sure ROS env is set (if `echo $ROS_MASTER_URI` is empty, run
   `source /environment.sh`), then:
     export ROS_NAMESPACE=duck4
     python3 /data/led_emitter_shim.py
   Leave this running. It should print "READY - advertising /duck4/led_emitter_node/set_pattern".
3. In a NEW VM terminal, start the demo:
     dts duckiebot demo --demo_name indefinite_navigation --duckiebot_name duck4
   Wait 3-5 min. Do NOT re-run.
4. Verify: dts start_gui_tools duck4 -> `rosservice list | grep set_pattern` should now show
   /duck4/led_emitter_node/set_pattern, and the demo should NOT collapse. rosnode list should
   keep fsm_node, lane_controller_node, etc.
5. LENS CAP OFF, place in lane, then keyboard control (--cli), press 'a'. Brake = docker stop
   the demo container.

Outcomes:
- Demo stays up + lane-follows -> led_emitter was the only mismatch. Unblocked, NO reflash needed.
- Demo still crashes on a DIFFERENT missing service/topic -> more version drift behind it ->
  fall back to the reflash plan below.
Notes: if the demo errors about set_pattern TYPE (not "unavailable"), the service type differs
from ChangePattern - check `rossrv show duckietown_msgs/ChangePattern` and adjust the script.
For a permanent fix later, bundle this node into the team's own package so it launches with A*.

## NEXT SESSION: reflash plan (tutor APPROVED 2026-06-30)
Tutor OK'd reflashing BUT warned: many people reflashed and then the bot wouldn't boot again.
So go slow and careful. Order of operations:
1. FIRST, before wiping anything: back up calibrations off the bot. SSH in and copy
   /data/config/calibrations/ to the laptop/VM (camera_intrinsic/duck4.yaml,
   camera_extrinsic/duck4.yaml, kinematics/duck4.yaml). e.g. from the VM:
     scp -r duckie@duck4.local:/data/config/calibrations ./duck4_calib_backup
   Verify the 3 yaml files are actually in the backup before proceeding.
2. Flash a CONSISTENT daffy image (init_sd_card / current Duckietown daffy flasher). VERIFY the
   exact procedure against current Duckietown docs at flash time - do NOT trust old commands.
   Use a known-good SD card; bad/slow cards are a common "won't boot again" cause.
3. First boot can take a LONG time (downloads containers) - be patient, good power, stable net.
4. After it boots: restore the 3 calibration yamls to /data/config/calibrations/ (same paths).
5. Re-verify: dts start_gui_tools duck4 -> rosservice list | grep set_pattern should now show
   /duck4/led_emitter_node/set_pattern. THEN run indefinite_navigation.
Brick-avoidance notes: never pull power mid-flash or mid-first-boot; keep the bot on stable
power; if it won't boot after flash, suspect the SD card / image version before the hardware.

## The plan (for context)
Tell the bot a START tile + facing and an END tile on a grid UI; A* over the Duckietown tile
graph -> ordered list of turns at intersections (left/straight/right). Bot lane-follows between
intersections and turns at each per the plan. Reuse `indefinite_navigation`; replace the random
turn picker (`random_april_tag_turns_node`) with our A* decision. AprilTags give accurate
intersection turns + (bonus) which-intersection position checks.

## Map (tum_map.yaml)
7x6 grid, tile_size 0.585 m. 5 decision points: 4-way at (row4,col3); 3-way at (2,5),(4,0),(4,5),(6,3).
Everything else is curves/straights handled by lane following.

## Team roles
- Person 1 (me): Robot + ROS integration / on-bot navigation (calibration done; running bot;
  lane follow + red-line stop; intersection turn execution; integration; testing/recording).
- Person 2: App + path planner (extend map_editor.html grid UI; A*; turn-list; send to robot
  via robot_http_api_node / ros_http_api_node or ROS bridge).
- Person 3: Map modeling + perception + evaluation (map yaml -> tile graph; OpenCV obstacle
  detector -> stop; metrics/plots).
- Teammate code branch (private repo lyonglam/duckietown, branch person2-behaviour-tests-docs)
  not yet reviewed — repo is private; need pasted code or public access before running on bot.

## Key daffy commands
- ROS terminal:          dts start_gui_tools duck4
- Camera view:           (in that terminal) rqt_image_view -> /duck4/camera_node/image/compressed
- Manual drive:          dts duckiebot keyboard_control duck4   (use --cli in the VM)
- Intrinsic cal:         dts duckiebot calibrate_intrinsics duck4
- Extrinsic cal:         dts duckiebot calibrate_extrinsics duck4
- Wheel trim set/save:   rosparam set /duck4/kinematics_node/trim VALUE   (SET, not additive)
                         rosservice call /duck4/kinematics_node/save_calibration
- Lane following:        dts duckiebot demo --demo_name lane_following --duckiebot_name duck4
- Indefinite nav:        dts duckiebot demo --demo_name indefinite_navigation --duckiebot_name duck4
- Stop a demo:           ssh duckie@duck4.local -> docker stop <demo container>   (no --stop flag!)
- Reboot:                ssh duckie@duck4.local -> sudo reboot

## How I (Person 1) like help
Simple, direct, step-by-step. Small concrete actions, explicit detail, no assumed knowledge.
Show commands before running and say what each does. I'm not very experienced.
