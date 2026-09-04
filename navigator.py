#!/usr/bin/env python3
"""
navigator.py -- runs a planned route on duck4. Lane-follows, stops at red lines,
executes the next turn from the plan, repeats until the plan is done.

This is the robot-side half of the A* project. a_star.py produces the command list;
this node executes it.

HOW THE PLAN ARRIVES
--------------------
This node serves the SAME contract our teammate's duckiebot_server.py already defines,
so the web app needs no changes -- it just points at the robot instead of the laptop:

    POST http://duck4.local:8083/navigate
         {"commands": ["right", "straight", "left", "stop"]}
      -> {"status": "executed", "commands_processed": 4}

    GET  http://duck4.local:8083/status
      -> {"state": "LANE_FOLLOW", "remaining": ["left","stop"], "done": false, ...}

    POST http://duck4.local:8083/abort      -> stops the robot and clears the plan

    POST http://duck4.local:8083/localize   -> reads the tile's AprilTag (robot must be
         idle), returns {"localized": true, "row": 2, "col": 0, "tag_id": 7,
         "confidence": 54.0} or {"localized": false, "reason": "..."}

It uses Python's built-in http.server (no Flask dependency, which the bot's containers
may not have). The server runs in a background thread; ROS work stays on the main thread.

WHY ONE FILE INSTEAD OF SEPARATE NODES
--------------------------------------
Lane following is duplicated from lane_follower.py rather than imported, so this is a
single file you can paste onto the bot and run as one process. Easier to start, easier
to Ctrl-C, and one place to look when debugging. Keep lane_follower.py around for
standalone tuning -- gains and thresholds transfer directly.

HOW THE STATE MACHINE WORKS
---------------------------
    LANE_FOLLOW -- normal driving, watching for a red stop line
    AT_STOP_LINE -- red seen: brake, pause, pop the next command
    TURNING      -- creep in, then rotate until a lane is visible ahead again
    STUCK        -- lane lost too long: stop, watch for the lane, warn the operator
    DONE         -- plan finished (or no plan): sit still

IMPORTANT -- THIS COUNTS STOP LINES
-----------------------------------
With no AprilTags, the ONLY thing keeping the robot in sync with the plan is counting red
stop lines in order. One missed line, or one line counted twice, and every later turn is
wrong. Two guards are in place: RED_COOLDOWN_S ignores red for a while after a turn (so
one line can't fire twice), and RED_MIN_AREA must be exceeded (so a fleck of red doesn't
trigger). Tune both. If you keep desyncing, that's the argument for adding AprilTags.

RUN (on the bot):
    ssh duckie@duck4.local
    docker exec -it duckiebot-interface bash
    source /environment.sh
    export VEHICLE_NAME=duck4
    python3 /data/navigator.py

Ctrl-C stops the robot (it publishes zero velocity on shutdown).
"""

import json
import os
import signal
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np
import rospy
from duckietown_msgs.msg import Twist2DStamped
from sensor_msgs.msg import CompressedImage, Range

# Run logging is OPTIONAL. This file's whole point is "one file you can paste onto the
# bot" -- it must still run if run_logger.py never made the trip. All logging goes
# through self.log_event(), which no-ops when the module is missing.
try:
    from run_logger import RunLogger
except ImportError:
    RunLogger = None

# AprilTag localization is OPTIONAL, same deal as run logging: the node must still run
# on a bot where no tag library was ever installed. See localize().
#
# Two interchangeable builds of the same AprilTag3 bindings, tried in order:
#   dt_apriltags     - Duckietown's fork. Prebuilt aarch64 wheels, works with the old
#                      numpy (1.17) these robots ship. This is the one that installs.
#   pupil_apriltags  - no aarch64 wheel, so pip tries to compile it and the build fails
#                      on this image; recent releases also need numpy >= 1.20.
# Both expose Detector(families=...) and .detect(gray) returning objects with .tag_id,
# .decision_margin and .center, which is all localize() below touches.
Detector = None
APRILTAG_LIB = None
for _lib in ("dt_apriltags", "pupil_apriltags"):
    try:
        Detector = __import__(_lib, fromlist=["Detector"]).Detector
        APRILTAG_LIB = _lib
        break
    except ImportError:
        pass

VEHICLE = os.environ.get("VEHICLE_NAME", "duck4")
THRESHOLD_FILE = os.environ.get("HSV_FILE", "/data/hsv_thresholds.json")
HTTP_PORT = int(os.environ.get("NAV_PORT", "8083"))
# Optional planner UI served straight off the robot. Put map_ui.html here and browse to
# http://duck4.local:8083/ from anywhere on the network (the VM included). Serving it
# from the bot means the page and the robot share an origin, so nothing to configure.
UI_FILE = os.environ.get("NAV_UI", "/data/map_ui.html")

# ---- lane following (same meaning as in lane_follower.py; tune there first) ----
KP = 4.0
KD = 2.0
V_BAR = 0.10
OMEGA_MAX = 6.0
V_MIN = 0.05
SLOWDOWN_STRENGTH = 0.8
ROI_FRACTION = 0.40

# ---- lane geometry gating (see lane_follower.py for the full reasoning) ----
# Road layout: [white edge][oncoming lane][YELLOW center][our lane][white edge].
# Ungated searches let the FAR white edge hijack steering on curves.
YELLOW_SEARCH_MAX = 0.70
WHITE_SEARCH_MIN = 0.30
MIN_LANE_WIDTH_PX = 20
HALF_LANE_FRAC = 0.25
RED_LANE_MIN = 0.30
LANE_MEMORY_DECAY = 0.85
LANE_LOST_MAX_FRAMES = 15
# After LANE_LOST_MAX_FRAMES the bot used to stop SILENTLY -- mid-route that looked
# like a dead robot, with no hint in the logs or /status. Now it enters STUCK instead:
# stays stopped, keeps watching for the lane, recovers on its own if the lane comes
# back, and nags the terminal once the wait gets long.
STUCK_RECOVER_FRAMES = 5  # consecutive frames with a lane before trusting recovery
STUCK_TIMEOUT_S = 30.0    # STUCK longer than this -> periodic operator warnings

# ---- stop line detection ----
RED_MIN_AREA = 3000      # red pixels needed to call it a stop line
RED_COOLDOWN_S = 4.0     # ignore red for this long AFTER FINISHING A TURN only
STOP_PAUSE_S = 1.0       # how long to sit still at the line before moving
# Blindness when a plan STARTS must be near zero. Anything longer and a bot parked just
# short of a stop line drives straight over it before it can see it. We only need long
# enough for one fresh camera frame.
STARTUP_GRACE_S = 0.3
# Only look for red in the BOTTOM part of the ROI. Red spotted far ahead makes the bot
# brake early, and where it stops decides where every maneuver starts -- so stopping
# consistently close to the line is what makes turns repeatable. RAISE this to stop
# closer to the line, LOWER it to stop earlier.
RED_BAND_TOP = 0.45

# ---- intersection maneuvers ----
# Each maneuver is: creep into the intersection, then rotate, then drive out.
#
# The ROTATION IS NOT TIMED. Timing can't work here -- a tight right and a wide left
# need different amounts of rotation, and it varies by intersection. Instead the bot
# turns until it SEES A LANE AHEAD again, which is self-correcting. The min/max times
# are only guards: don't check too early (we'd re-detect the lane we're leaving), and
# give up eventually if nothing is found.
# NOTE: these are YOUR tuned values from the lab, not the original defaults.
CREEP_V = 0.10           # speed while entering the intersection
CREEP_STRAIGHT_S = 1.0   # straight already works -- left alone
CREEP_LEFT_S = 2.5       # wide turn: go DEEPER into the intersection before rotating
CREEP_RIGHT_S = 1.5      # tight turn: rotate EARLY or you overshoot the corner

STRAIGHT_V = 0.12        # speed crossing straight through
STRAIGHT_S = 1.0         # seconds to clear the intersection going straight

TURN_V = 0.10            # forward speed during a turn (0 = spin in place)
TURN_OMEGA = 2.0         # rad/s; POSITIVE is LEFT in Duckietown convention
MIN_LEFT_TURN_S = 0.5    # rotate at least this long before looking for a lane
MAX_LEFT_TURN_S = 1.7    # ...and give up looking after this long
MIN_RIGHT_TURN_S = 0.2
MAX_RIGHT_TURN_S = 1.5
LANE_REACQUIRE_TOL = 0.28  # lane counts as "ahead" if within this fraction of center

# There is NO blind "exit" phase any more. Driving straight for a fixed time after a turn
# means deliberately ignoring the camera, which looked exactly like "it won't follow the
# lane". Instead we hand straight back to lane following with red suppressed for
# RED_COOLDOWN_S -- so the bot drives out of the intersection by actually following the
# lane, and carries on to the NEXT red line on its own.
#
# FINISH_S only applies after the LAST command: keep lane-following this long so the bot
# clears the intersection and settles in a lane before stopping.
FINISH_S = 3.0

# ---- obstacle avoidance, using the TIME-OF-FLIGHT range sensor ----
# Set False to disable entirely and get plain navigation back.
AVOIDANCE_ENABLED = True

# WHY DISTANCE AND NOT THE CAMERA: a rubber duckie is the same colour family as the
# yellow centre line, and shape does not separate them either -- perspective squashes a
# near dash into the same compact blob a duckie makes, so the same markings measured
# elongation 1.1 to 2.4 across consecutive frames. The ToF sensor sidesteps all of it:
# a duckie STANDS UP and returns a short range, flat tape on the road returns nothing.
#
# Just as important, this detector never touches the vision pipeline. It cannot corrupt
# the yellow mask or the centre line, so a false reading can no longer break lane
# following the way the camera-based attempt did.
#
# Approach follows the previous TUM batch (Duckietown-Btown,
# packages/obstacle_detection/src/tof_obstacle_detection_node.py): read sensor_msgs/Range
# and apply hysteresis so a reading hovering at the limit cannot chatter. Their numbers
# (detect 0.12 m) are for STOPPING; we swerve, so we need to see it further out.
TOF_TOPIC = os.environ.get(
    "TOF_TOPIC", "/{}/front_center_tof_driver_node/range".format(VEHICLE))
TOF_DETECT_M = 0.35      # closer than this = something is in the way
TOF_CLEAR_M = 0.50       # must get beyond this to count as clear again (hysteresis)
TOF_MIN_VALID_M = 0.02   # below this the reading is noise, not an object
TOF_STALE_S = 2.0        # no messages for this long = sensor is not working
# How many consecutive short readings before we believe an obstacle is really there.
# The sensor runs at ~15 Hz, so 5 frames is about a third of a second. This is what
# stops bright flat things on the road -- AprilTags above all, since white reflects
# strongly where dark tape returns almost nothing -- from tripping a swerve as the bot
# drives over them. RAISE it if tags still trigger; LOWER it if real obstacles are
# noticed too late.
TOF_CONFIRM_FRAMES = 5
TOF_DEBUG = True         # log the live range about once a second, to tune the above

# The swerve itself: no scripted manoeuvre. We stay in closed-loop lane following and
# slide the TARGET across the centre line.
#   ramp 0.0 = our lane, 0.5 = straddling the line, 1.0 = oncoming lane
AVOID_RAMP_RATE = 0.055  # how fast the target slides per frame
AVOID_HOLD_S = 2.0       # stay out this long after the obstacle stops being seen
AVOID_MAX_S = 8.0        # never sit in the oncoming lane longer than this

# ---- HOW FAR THE SWERVE GOES. This is the knob to turn. ----
# Fraction of image width the target shifts LEFT at full swerve:
#   0.50 = full mirror, ends up a half-lane past the centre line (aggressive)
#   0.35 = sits just left of the centre line (default, enough to clear a duckie)
#   0.25 = rides exactly on the centre line (very gentle)
#   0.15 = barely moves over
# Raise it to swerve wider, lower it if the bot swings too far.
AVOID_SHIFT_FRAC = 0.35

# Steering is CLAMPED during a swerve. Without this the large lateral error commands
# near-full lock and the bot pirouettes instead of easing across.
AVOID_OMEGA_MAX = 2.2

# While swerving, the centre line ends up on our RIGHT. The normal left-hand search
# window would miss it completely, yc would go None, and the lane-lost handler would
# hold the last hard turn -- which is exactly how a swerve becomes a 180. Widen it.
YELLOW_SEARCH_SWERVE = 0.95

# ---- AprilTag self-localization ----
# Tags (tag36h11) are mounted flat on the ROAD tiles of the 7x6 grid -- 30 of them; the
# rest of the grid is empty space with no tag. Before a route starts, POST /localize
# reads the tag under the camera and reports which tile the bot is on, so the planner
# no longer has to be told the start tile. Facing is deliberately OUT OF SCOPE: only
# tag_id / decision_margin / center are read, no pose estimation.
# Set False to disable; /localize then answers "disabled" instead of detecting.
APRILTAG_ENABLED = True
APRILTAG_MAP_FILE = os.environ.get(
    "APRILTAG_MAP",
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "apriltag_map.json"))
# One frame can misread; sample several and take a majority vote on the tag id.
LOCALIZE_SAMPLE_FRAMES = 5
LOCALIZE_MIN_MARGIN = 20.0    # decision_margin below ~20 is borderline (measured with
                              # apriltag_scan_test.py) -- discard those samples
LOCALIZE_MIN_VALID = 2        # need at least this many confident samples to answer
LOCALIZE_FRAME_TIMEOUT_S = 2.0  # max wait for ONE fresh camera frame

# ---- LIVE TILE TRACKING (optional, for the map in the browser) ----
# /localize is a one-shot answer taken while the robot is parked. This is the other
# thing: keep reading tags WHILE driving so the map can show where the robot is.
#
# It is deliberately OFF by default. Tag detection on every frame would compete with
# lane following for CPU on the Jetson, and lane following is the part that must never
# stutter. When on, we detect on every Nth frame only, and the whole thing is wrapped so
# that a detector failure can never take down the driving loop.
#
# Turn it on here, or without editing the file:  export LIVE_TRACKING=1
LIVE_TRACKING = os.environ.get("LIVE_TRACKING", "0") not in ("0", "", "false", "False")
LIVE_EVERY_N_FRAMES = 6       # camera is ~30 fps, so this samples at ~5 Hz
LIVE_MIN_MARGIN = 25.0        # stricter than LOCALIZE_MIN_MARGIN: showing the WRONG
                              # tile on the map is worse than showing none
LIVE_STALE_S = 5.0            # older than this and the map should stop trusting it

DEFAULT_THRESHOLDS = {
    "yellow": {"lower": [15, 80, 80], "upper": [40, 255, 255]},
    "white": {"lower": [0, 0, 170], "upper": [180, 70, 255]},
    "red": {"lower": [0, 120, 120], "upper": [10, 255, 255]},
    "red2": {"lower": [170, 120, 120], "upper": [180, 255, 255]},
}

VALID_COMMANDS = {"straight", "left", "right", "stop"}


def load_thresholds():
    try:
        with open(THRESHOLD_FILE) as f:
            data = json.load(f)
        for key in DEFAULT_THRESHOLDS:
            data.setdefault(key, DEFAULT_THRESHOLDS[key])
        rospy.loginfo("[navigator] thresholds loaded from %s", THRESHOLD_FILE)
        return data
    except Exception:
        rospy.logwarn("[navigator] no %s -- using DEFAULTS. Run hsv_calibrator.py!",
                      THRESHOLD_FILE)
        return DEFAULT_THRESHOLDS


def load_tag_map():
    """
    tag id (as a STRING -- JSON keys) -> {"row", "col"} for the 30 road tiles.
    Returns None if the file is missing/broken: the node still starts and /localize
    reports a clear failure, instead of the whole navigator dying for a side feature.
    """
    try:
        with open(APRILTAG_MAP_FILE) as f:
            data = json.load(f)
        rospy.loginfo("[navigator] apriltag map: %d tags from %s",
                      len(data), APRILTAG_MAP_FILE)
        return data
    except Exception as e:
        rospy.logerr("[navigator] cannot load %s (%s) -- /localize will not work",
                     APRILTAG_MAP_FILE, e)
        return None


def centroid_x(mask, x_lo=None, x_hi=None):
    """
    Center of mass in x, or None. x_lo/x_hi restrict the search to a horizontal band so
    the far side of the road can't hijack a line (blanking, not cropping, so the result
    stays in full-image coordinates).
    """
    if x_lo is not None or x_hi is not None:
        gated = np.zeros_like(mask)
        lo = 0 if x_lo is None else max(0, int(x_lo))
        hi = mask.shape[1] if x_hi is None else min(mask.shape[1], int(x_hi))
        if hi <= lo:
            return None
        gated[:, lo:hi] = mask[:, lo:hi]
        mask = gated
    m = cv2.moments(mask)
    if m["m00"] < 500:
        return None
    return m["m10"] / m["m00"]


class Navigator:
    def __init__(self):
        # disable_signals so OUR Ctrl-C handler runs and can brake before rospy tears
        # the publisher down.
        rospy.init_node("navigator", anonymous=False, disable_signals=True)
        self.th = load_thresholds()
        self.prev_error = 0.0
        self.last_omega = 0.0       # steering memory for when the lines drop out
        self.lost_frames = 0
        self.recover_frames = 0     # consecutive lane sightings while STUCK

        self.lock = threading.Lock()
        self.plan = []              # commands still to execute
        self.plan_total = 0
        self.state = "DONE"         # start idle -- nothing happens until a plan arrives
        self.maneuver = None        # (phase_list, index, phase_started_at)
        self.state_since = rospy.Time.now()
        self.ignore_red_until = rospy.Time.now()
        self.last_command = None
        self.finish_at = None       # set after the last command: settle, then stop
        self.ramp = 0.0             # 0 = our lane, 1 = oncoming lane
        self.avoid_until = None
        self.avoid_started = None
        self.obstacle = False       # latched by the ToF callback, with hysteresis
        self.tof_near_count = 0     # consecutive short readings, see cb_tof
        self.live_tile = None       # last tile seen while driving (live tracking)
        # The AprilTag detector is a C library and is NOT thread safe. Two callers
        # reach it: update_live_tile() from the ROS camera thread, and localize()
        # from an HTTP request thread. Running both at once corrupts the heap and
        # segfaults the process. Every detect() call must hold this lock.
        self.detector_lock = threading.Lock()
        self.tof_range = None
        self.tof_stamp = None

        # Evidence file for the report: success rates, timings, stuck counts. See
        # run_logger.py (writer) and analyze_runs.py (reader).
        self.runlog = RunLogger() if RunLogger is not None else None
        if self.runlog is None:
            rospy.logwarn("[navigator] run_logger.py not found -- run logging OFF")
        self.stuck_timeout_logged = False   # one stuck_timeout event per episode

        # AprilTag self-localization (POST /localize). latest_frame is the ONLY frame
        # cache: cb_image overwrites it every frame, localize() reads it. frame_seq
        # lets localize() wait for genuinely NEW frames instead of re-reading one.
        self.latest_frame = None
        self.frame_seq = 0
        self.localized_tile = None      # {"row","col"} after a good localize
        self.last_localization = None   # full last result (success or failure), for /status
        self.localize_lock = threading.Lock()   # one detection run at a time
        self.tag_map = load_tag_map() if APRILTAG_ENABLED else None
        if APRILTAG_ENABLED and Detector is not None:
            # Same setup as apriltag_scan_test.py: tag36h11 ONLY.
            self.tag_detector = Detector(families="tag36h11")
        else:
            self.tag_detector = None
            if APRILTAG_ENABLED:
                rospy.logwarn("[navigator] pupil_apriltags not installed -- /localize "
                              "disabled. pip install pupil-apriltags")

        self.pub = rospy.Publisher(
            "/{}/car_cmd_switch_node/cmd".format(VEHICLE), Twist2DStamped, queue_size=1)
        rospy.Subscriber(
            "/{}/camera_node/image/compressed".format(VEHICLE), CompressedImage,
            self.cb_image, queue_size=1, buff_size=2 ** 24)
        if AVOIDANCE_ENABLED:
            rospy.Subscriber(TOF_TOPIC, Range, self.cb_tof, queue_size=1)
            rospy.loginfo("[navigator] obstacle avoidance ON, listening on %s", TOF_TOPIC)

        self.start_http()
        rospy.on_shutdown(self.stop)
        signal.signal(signal.SIGINT, self.handle_sigint)
        signal.signal(signal.SIGTERM, self.handle_sigint)
        rospy.loginfo("[navigator] ready. POST a plan to http://%s.local:%d/navigate",
                      VEHICLE, HTTP_PORT)

    # ================= plan handling =================

    def set_plan(self, commands):
        """Validate and install a new plan. Returns (ok, message)."""
        bad = [c for c in commands if c not in VALID_COMMANDS]
        if bad:
            return False, "unknown commands: %s" % bad
        with self.lock:
            # A trailing "stop" is the planner's end marker, not a maneuver.
            self.plan = [c for c in commands if c != "stop"]
            self.plan_total = len(self.plan)
            self.state = "LANE_FOLLOW" if self.plan else "DONE"
            self.state_since = rospy.Time.now()
            # NOT RED_COOLDOWN_S -- see STARTUP_GRACE_S. Suppressing red here made the
            # bot miss a stop line it was already parked in front of.
            self.ignore_red_until = rospy.Time.now() + rospy.Duration(STARTUP_GRACE_S)
            self.maneuver = None
            self.last_command = None
            self.finish_at = None
        rospy.loginfo("[navigator] new plan (%d turns): %s", self.plan_total, commands)
        self.log_event("plan_started", commands=list(commands), count=self.plan_total)
        return True, "ok"

    def abort(self):
        with self.lock:
            self.plan = []
            self.state = "DONE"
            self.maneuver = None
        self.drive(0.0, 0.0)
        rospy.logwarn("[navigator] ABORTED")

    def status(self):
        with self.lock:
            return {
                "state": self.state,
                "remaining": list(self.plan),
                "completed": self.plan_total - len(self.plan),
                "total": self.plan_total,
                "last_command": self.last_command,
                "done": self.state == "DONE" and not self.plan,
                "stuck": self.state == "STUCK",
                "avoiding": round(self.ramp, 2),
                "obstacle": self.obstacle,
                "tof_m": None if self.tof_range is None else round(self.tof_range, 3),
                "last_localization": self.last_localization,
                "live_tile": self.live_tile_status(),
                "live_tracking": LIVE_TRACKING,
            }

    def follow_only(self):
        """Lane-follow with no route. Useful for testing lane following on its own."""
        with self.lock:
            self.plan = []
            self.plan_total = 0
            self.state = "LANE_FOLLOW"
            self.state_since = rospy.Time.now()
            self.ignore_red_until = rospy.Time.now() + rospy.Duration(STARTUP_GRACE_S)
            self.maneuver = None
            self.finish_at = None
        rospy.loginfo("[navigator] lane-follow only (no route) -- will stop at red")

    # ================= perception =================

    def mask_for(self, hsv, name):
        lo = np.array(self.th[name]["lower"])
        hi = np.array(self.th[name]["upper"])
        mask = cv2.inRange(hsv, lo, hi)
        if name == "red":
            mask = cv2.bitwise_or(mask, cv2.inRange(
                hsv, np.array(self.th["red2"]["lower"]),
                np.array(self.th["red2"]["upper"])))
        k = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)
        return mask

    def cb_image(self, msg):
        frame = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            return
        # Cache for localize() -- newest frame wins, nothing else uses this.
        self.latest_frame = frame
        self.frame_seq += 1

        # Live tile tracking, if enabled. Every Nth frame only, and never allowed to
        # raise: driving must not stop because a tag lookup went wrong.
        if (LIVE_TRACKING and self.tag_detector is not None
                and self.frame_seq % LIVE_EVERY_N_FRAMES == 0):
            try:
                self.update_live_tile(frame)
            except Exception as e:
                rospy.logwarn_throttle(10.0, "[navigator] live tracking error: %s", e)
        h, w = frame.shape[:2]
        roi = frame[int(h * (1.0 - ROI_FRACTION)):, :]
        hsv = cv2.GaussianBlur(cv2.cvtColor(roi, cv2.COLOR_BGR2HSV), (5, 5), 0)

        with self.lock:
            state = self.state

        if state == "DONE":
            self.drive(0.0, 0.0)
        elif state == "LANE_FOLLOW":
            self.do_lane_follow(hsv, w)
        elif state == "AT_STOP_LINE":
            self.do_stop_line()
        elif state == "TURNING":
            self.do_turn(hsv, w)
        elif state == "STUCK":
            self.do_stuck(hsv, w)

    # ================= states =================

    def cb_tof(self, msg):
        """
        Front range sensor. Hysteresis (detect near, clear further out) stops a reading
        that sits right on the limit from flickering the swerve on and off.

        Readings below TOF_MIN_VALID_M are sensor noise. The VL53L0X also reports
        out-of-range as a large value, which simply fails the detect test, so "nothing
        there" needs no special case.
        """
        self.tof_range = msg.range
        self.tof_stamp = rospy.Time.now()
        if TOF_DEBUG:
            rospy.loginfo_throttle(1.0, "[navigator] tof %.3f m  near_count=%d  obstacle=%s",
                                   msg.range, self.tof_near_count, self.obstacle)
        if msg.range < TOF_MIN_VALID_M:
            return

        # Three bands, not two. Between DETECT and CLEAR we hold whatever we already
        # decided -- that gap is the hysteresis.
        if msg.range < TOF_DETECT_M:
            # A single short reading is NOT enough. Anything bright and flat that the
            # beam clips -- an AprilTag on the road especially, since white reflects far
            # better than dark tape and returns a strong signal where asphalt returns
            # none -- produces a brief spike as the bot drives past. A real obstacle
            # stands in front of us and keeps reading short. So require the reading to
            # persist before believing it.
            self.tof_near_count += 1
            if not self.obstacle and self.tof_near_count >= TOF_CONFIRM_FRAMES:
                self.obstacle = True
                rospy.loginfo("[navigator] obstacle at %.2f m (%d consecutive readings)",
                              msg.range, self.tof_near_count)
        elif msg.range > TOF_CLEAR_M:
            self.tof_near_count = 0
            if self.obstacle:
                self.obstacle = False
                rospy.loginfo("[navigator] obstacle cleared (%.2f m)", msg.range)

    def update_live_tile(self, frame):
        """
        Read whatever tag is in view and remember which tile it means.

        Unlike localize(), this takes no vote across frames -- it runs continuously, so
        a single bad frame is corrected by the next one a fifth of a second later. The
        protection instead is a HIGHER margin threshold: we would rather show nothing
        than show the robot on the wrong tile.
        """
        gray = np.ascontiguousarray(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
        # Non-blocking: if localize() already holds the detector, skip this frame
        # rather than wait. Another one arrives in a fifth of a second.
        if not self.detector_lock.acquire(False):
            return
        try:
            tags = self.tag_detector.detect(gray)
        finally:
            self.detector_lock.release()
        if not tags:
            return
        best = max(tags, key=lambda t: t.decision_margin)
        if best.decision_margin < LIVE_MIN_MARGIN:
            return
        info = self.tag_map.get(str(int(best.tag_id))) if self.tag_map else None
        if not info:
            return
        prev = self.live_tile
        self.live_tile = {"row": info["row"], "col": info["col"],
                          "tag_id": int(best.tag_id),
                          "confidence": round(float(best.decision_margin), 1),
                          "age_s": 0.0, "t": time.time()}
        if not prev or (prev["row"], prev["col"]) != (info["row"], info["col"]):
            rospy.loginfo("[navigator] live tile -> (%d,%d) tag %d",
                          info["row"], info["col"], int(best.tag_id))

    def live_tile_status(self):
        """live_tile with a freshness stamp, or None once it goes stale."""
        lt = self.live_tile
        if not lt:
            return None
        age = time.time() - lt["t"]
        if age > LIVE_STALE_S:
            return None
        out = dict(lt)
        out["age_s"] = round(age, 1)
        return out

    def tof_alive(self):
        """False if the sensor has gone quiet -- better to say so than silently coast."""
        if self.tof_stamp is None:
            return False
        return (rospy.Time.now() - self.tof_stamp).to_sec() < TOF_STALE_S

    def update_avoidance(self, yc):
        """
        Move `ramp` toward where we want to be, one step per frame. Ramping rather than
        switching is what makes the swerve smooth instead of a lurch.
        """
        if not AVOIDANCE_ENABLED:
            return
        now = rospy.Time.now()

        if not self.tof_alive():
            rospy.logwarn_throttle(
                5.0, "[navigator] no ToF data on %s -- avoidance inactive", TOF_TOPIC)
            self.obstacle = False

        if self.obstacle:
            if self.avoid_started is None:
                rospy.loginfo("[navigator] swerving into the other lane")
                self.log_event("avoidance_start", tof_m=None if self.tof_range is None
                               else round(self.tof_range, 3))
                self.avoid_started = now
            self.avoid_until = now + rospy.Duration(AVOID_HOLD_S)

        # Safety valve: never sit in the oncoming lane indefinitely.
        if (self.avoid_started is not None
                and (now - self.avoid_started).to_sec() > AVOID_MAX_S):
            rospy.logwarn_throttle(2.0,
                                   "[navigator] avoidance timed out -- returning to lane")
            self.avoid_until = now
            self.obstacle = False

        want = 1.0 if (self.avoid_until is not None and now < self.avoid_until) else 0.0
        if yc is None:
            want = self.ramp     # no centre line to measure against: hold, don't lurch

        if want > self.ramp:
            self.ramp = min(1.0, self.ramp + AVOID_RAMP_RATE)
        elif want < self.ramp:
            self.ramp = max(0.0, self.ramp - AVOID_RAMP_RATE)

        if self.ramp <= 0.001 and self.avoid_started is not None and want == 0.0:
            rospy.loginfo("[navigator] back in our lane")
            self.log_event("avoidance_end",
                           duration_s=round((now - self.avoid_started).to_sec(), 1))
            self.avoid_started = None
            self.avoid_until = None

    def do_lane_follow(self, hsv, width):
        # Route finished: we've been settling in the lane, now actually stop.
        if self.finish_at is not None and rospy.Time.now() > self.finish_at:
            rospy.loginfo("[navigator] route complete -- stopping")
            self.log_event("route_complete")
            self.finish_at = None
            self.enter("DONE")
            self.drive(0.0, 0.0)
            return

        # Find each line only where it can legitimately be: yellow centerline on our
        # left, our white edge on our right.
        swerving = self.ramp > 0.001
        y_gate = YELLOW_SEARCH_SWERVE if swerving else YELLOW_SEARCH_MAX
        yc = centroid_x(self.mask_for(hsv, "yellow"), x_hi=width * y_gate)
        wc = centroid_x(self.mask_for(hsv, "white"), x_lo=width * WHITE_SEARCH_MIN)
        center = width / 2.0

        # If "white" landed left of yellow it's the far edge line, not ours.
        if yc is not None and wc is not None and wc <= yc + MIN_LANE_WIDTH_PX:
            rospy.logwarn_throttle(2.0, "[navigator] white line looks wrong -- ignoring")
            wc = None

        self.update_avoidance(yc)

        # Stop line, but only in OUR lane -- a red line across the oncoming lane is not
        # ours to obey, and stopping for it would also miscount the plan.
        # Suppressed mid-swerve: we are on the wrong side of the road, so the lane-relative
        # reasoning does not hold and a bogus trigger would desync the whole plan.
        if self.ramp > 0.001:
            rospy.logwarn_throttle(3.0, "[navigator] swerving -- stop lines ignored")
        elif rospy.Time.now() > self.ignore_red_until:
            red = self.mask_for(hsv, "red")
            red_cut = int(yc) if yc is not None else int(width * RED_LANE_MIN)
            red[:, :max(0, red_cut)] = 0
            # ignore red high up in the ROI -- that's a line still far ahead
            red[:int(red.shape[0] * RED_BAND_TOP), :] = 0
            if cv2.countNonZero(red) > RED_MIN_AREA:
                rospy.loginfo("[navigator] stop line detected")
                self.enter("AT_STOP_LINE")
                self.drive(0.0, 0.0)
                return

        if self.ramp > 0.001 and yc is not None:
            # Swerving. Steer relative to the CENTRE LINE and slide the target from its
            # right side to its left:
            #   ramp 0 -> yc + half lane (our lane)
            #   ramp .5 -> yc            (straddling the line)
            #   ramp 1 -> yc - half lane (oncoming lane)
            # White-edge logic is skipped: past the centre line the left/right geometry
            # is mirrored and would fight the swerve.
            lane_center = yc + width * (HALF_LANE_FRAC - AVOID_SHIFT_FRAC * self.ramp)
        elif yc is not None and wc is not None:
            lane_center = (yc + wc) / 2.0
        elif yc is not None:
            lane_center = yc + width * HALF_LANE_FRAC
        elif wc is not None:
            lane_center = wc - width * HALF_LANE_FRAC
        else:
            # Hold the turn rather than straightening -- straightening mid-curve is what
            # carries the bot out of its lane.
            self.lost_frames += 1
            if self.ramp > 0.001:
                # Mid-swerve: do NOT hold the turn. Holding a hard turn with no reference
                # is what spins the bot right around. Straighten up and coast instead.
                rospy.logwarn_throttle(2.0, "[navigator] lane lost mid-swerve -- coasting")
                self.last_omega = 0.0
                self.drive(V_MIN, 0.0)
                return
            rospy.logwarn_throttle(2.0, "[navigator] lane lost -- holding turn")
            if self.lost_frames <= LANE_LOST_MAX_FRAMES:
                self.last_omega *= LANE_MEMORY_DECAY
                self.drive(V_MIN, self.last_omega)
            else:
                rospy.logwarn("[navigator] lane lost for %d frames -- giving up and "
                              "entering STUCK (stopped, watching for the lane)",
                              self.lost_frames)
                self.log_event("stuck_entered", lost_frames=self.lost_frames)
                self.recover_frames = 0
                self.stuck_timeout_logged = False
                self.enter("STUCK")
                self.drive(0.0, 0.0)
            return

        self.lost_frames = 0
        error = (lane_center - center) / center
        d_error = error - self.prev_error
        self.prev_error = error
        cap = AVOID_OMEGA_MAX if self.ramp > 0.001 else OMEGA_MAX
        omega = float(np.clip(-(KP * error + KD * d_error), -cap, cap))
        v = max(V_MIN, V_BAR * (1.0 - SLOWDOWN_STRENGTH * error * error))
        self.last_omega = omega
        self.drive(v, omega)

    def do_stop_line(self):
        """Sit still briefly, then pop the next command and start the maneuver."""
        self.drive(0.0, 0.0)
        if (rospy.Time.now() - self.state_since).to_sec() < STOP_PAUSE_S:
            return

        with self.lock:
            if not self.plan:
                rospy.loginfo("[navigator] plan complete -- stopping")
                self.state = "DONE"
                return
            command = self.plan.pop(0)
            self.last_command = command

        rospy.loginfo("[navigator] executing '%s' (%d left)", command, len(self.plan))
        self.log_event("command_executed", command=command, remaining=len(self.plan))
        self.maneuver = (self.phases_for(command), 0, rospy.Time.now())
        self.enter("TURNING")

    def do_stuck(self, hsv, width):
        """Lane lost for too long: sit still, watch for the lane, recover if it returns."""
        self.drive(0.0, 0.0)

        # Same detection lane following uses -- we just look, we don't steer.
        yc = centroid_x(self.mask_for(hsv, "yellow"), x_hi=width * YELLOW_SEARCH_MAX)
        wc = centroid_x(self.mask_for(hsv, "white"), x_lo=width * WHITE_SEARCH_MIN)
        if yc is not None and wc is not None and wc <= yc + MIN_LANE_WIDTH_PX:
            wc = None

        if yc is not None or wc is not None:
            # One lucky frame is not a recovery -- require a streak.
            self.recover_frames += 1
            if self.recover_frames >= STUCK_RECOVER_FRAMES:
                stuck_s = (rospy.Time.now() - self.state_since).to_sec()
                rospy.loginfo("[navigator] lane visible again after %.1fs STUCK -- "
                              "resuming lane follow", stuck_s)
                self.log_event("stuck_recovered", stuck_s=round(stuck_s, 1))
                self.prev_error = 0.0
                self.last_omega = 0.0
                self.lost_frames = 0
                self.recover_frames = 0
                self.enter("LANE_FOLLOW")
            return
        self.recover_frames = 0

        stuck_s = (rospy.Time.now() - self.state_since).to_sec()
        if stuck_s > STUCK_TIMEOUT_S:
            if not self.stuck_timeout_logged:   # ONE event per episode, not per frame
                self.stuck_timeout_logged = True
                self.log_event("stuck_timeout", stuck_s=round(stuck_s, 1))
            rospy.logwarn_throttle(
                5.0, "[navigator] STUCK for %.0fs -- no lane in sight, robot needs help",
                stuck_s)

    # ================= apriltag localization =================

    def grab_fresh_frame(self, last_seq):
        """Wait for a frame NEWER than last_seq. Returns (frame, seq) or (None, seq)."""
        deadline = time.time() + LOCALIZE_FRAME_TIMEOUT_S
        while time.time() < deadline:
            if self.frame_seq != last_seq and self.latest_frame is not None:
                return self.latest_frame, self.frame_seq
            time.sleep(0.02)
        return None, last_seq

    def localize(self):
        """
        Which tile is the bot parked on? Reads the tile's AprilTag and answers from
        apriltag_map.json. Only meaningful while IDLE -- mid-route the camera points
        wherever the route put it, and we must not fight the lane pipeline for frames.
        Returns the response dict for the HTTP layer; also stored for /status.
        """
        if not APRILTAG_ENABLED:
            return {"localized": False, "reason": "apriltag localization disabled"}
        if self.tag_detector is None:
            return {"localized": False,
                    "reason": "no apriltag library on the bot "
                              "(pip install dt-apriltags)"}
        if self.tag_map is None:
            return {"localized": False,
                    "reason": "apriltag_map.json missing or unreadable"}
        with self.lock:
            if self.state != "DONE":
                return {"localized": False,
                        "reason": "route active (state %s) -- localize only while idle"
                                  % self.state}
        if not self.localize_lock.acquire(False):
            return {"localized": False, "reason": "localization already in progress"}
        try:
            result = self.sample_and_vote()
        finally:
            self.localize_lock.release()
        with self.lock:
            self.last_localization = result
            self.localized_tile = ({"row": result["row"], "col": result["col"]}
                                   if result["localized"] else None)
        return result

    def sample_and_vote(self):
        """
        The actual detection: up to LOCALIZE_SAMPLE_FRAMES fresh frames, majority vote
        on tag id. Distinguishes the three ways it can fail -- nothing seen, seen but
        low confidence, frames disagreeing -- because each needs a different fix
        (tag placement / distance / duplicate tags in view).
        """
        votes = {}              # tag_id -> [decision_margins]
        frames_seen = 0
        frames_with_tag = 0
        seq = self.frame_seq
        for _ in range(LOCALIZE_SAMPLE_FRAMES):
            frame, seq = self.grab_fresh_frame(seq)
            if frame is None:
                break           # camera went quiet: judge from what we already have
            frames_seen += 1
            gray = np.ascontiguousarray(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
            with self.detector_lock:
                tags = self.tag_detector.detect(gray)
            if not tags:
                continue
            frames_with_tag += 1
            # More than one tag in view: the clearest one is the one we're on.
            best = max(tags, key=lambda t: t.decision_margin)
            if best.decision_margin < LOCALIZE_MIN_MARGIN:
                continue
            votes.setdefault(int(best.tag_id), []).append(best.decision_margin)

        if frames_seen == 0:
            rospy.logwarn("[navigator] localize: no camera frames -- camera down?")
            return {"localized": False, "reason": "no camera frames"}
        if not votes:
            if frames_with_tag:
                rospy.logwarn("[navigator] localize: tag seen in %d/%d frames but every "
                              "margin was below %.0f -- too far or bad angle",
                              frames_with_tag, frames_seen, LOCALIZE_MIN_MARGIN)
                return {"localized": False,
                        "reason": "tag visible but below confidence threshold"}
            rospy.logwarn("[navigator] localize: no tag detected in %d frames",
                          frames_seen)
            return {"localized": False, "reason": "no tag detected"}

        total_valid = sum(len(m) for m in votes.values())
        tag_id, margins = max(votes.items(), key=lambda kv: len(kv[1]))
        if len(margins) < LOCALIZE_MIN_VALID:
            rospy.logwarn("[navigator] localize: only %d confident sample(s) of %d "
                          "frames -- not enough to trust", len(margins), frames_seen)
            return {"localized": False, "reason": "too few confident samples"}
        if len(margins) * 2 <= total_valid:
            counts = {t: len(m) for t, m in votes.items()}
            rospy.logwarn("[navigator] localize: frames disagree on the tag id %s -- "
                          "two tags in view?", counts)
            return {"localized": False,
                    "reason": "frames disagree on tag id: %s" % counts}

        info = self.tag_map.get(str(tag_id))
        if info is None:
            rospy.logwarn("[navigator] localize: tag %d is not in %s -- map out of "
                          "date?", tag_id, APRILTAG_MAP_FILE)
            return {"localized": False,
                    "reason": "tag %d not in apriltag_map.json" % tag_id}

        confidence = round(max(margins), 1)
        result = {"localized": True, "row": info["row"], "col": info["col"],
                  "tag_id": tag_id, "confidence": confidence}
        rospy.loginfo("[navigator] localized: tag %d -> tile (%d,%d) "
                      "(margin %.0f, %d/%d frames)", tag_id, info["row"], info["col"],
                      confidence, len(margins), frames_seen)
        return result

    def phases_for(self, command):
        """
        A maneuver is a list of phases. Two kinds:
            ("timed", duration, _,     v, omega)  -- drive for a fixed time
            ("until_lane", min_s, max_s, v, omega)  -- drive until a lane is ahead again

        Every maneuver creeps first (the stop line is painted BEFORE the intersection),
        and every turn drives out at the end so we never finish sitting in the middle.
        """
        if command == "straight":
            return [("timed", CREEP_STRAIGHT_S, 0.0, CREEP_V, 0.0),
                    ("timed", STRAIGHT_S, 0.0, STRAIGHT_V, 0.0)]
        if command == "left":
            return [("timed", CREEP_LEFT_S, 0.0, CREEP_V, 0.0),
                    ("until_lane", MIN_LEFT_TURN_S, MAX_LEFT_TURN_S, TURN_V, TURN_OMEGA)]
        if command == "right":
            return [("timed", CREEP_RIGHT_S, 0.0, CREEP_V, 0.0),
                    ("until_lane", MIN_RIGHT_TURN_S, MAX_RIGHT_TURN_S,
                     TURN_V, -TURN_OMEGA)]
        return [("timed", 0.0, 0.0, 0.0, 0.0)]

    def lane_ahead(self, hsv, width):
        """
        Is there a usable lane roughly in FRONT of us? This is how a turn knows it's
        finished. Requiring the lane to be near the middle of the view stops us from
        latching onto the lane we're still leaving.
        """
        yc = centroid_x(self.mask_for(hsv, "yellow"), x_hi=width * YELLOW_SEARCH_MAX)
        wc = centroid_x(self.mask_for(hsv, "white"), x_lo=width * WHITE_SEARCH_MIN)
        if yc is not None and wc is not None and wc <= yc + MIN_LANE_WIDTH_PX:
            wc = None
        if yc is None and wc is None:
            return False
        if yc is not None and wc is not None:
            lane_center = (yc + wc) / 2.0
        elif yc is not None:
            lane_center = yc + width * HALF_LANE_FRAC
        else:
            lane_center = wc - width * HALF_LANE_FRAC
        return abs(lane_center - width / 2.0) < width * LANE_REACQUIRE_TOL

    def do_turn(self, hsv, width):
        phases, idx, started = self.maneuver
        kind, a, b, v, omega = phases[idx]
        elapsed = (rospy.Time.now() - started).to_sec()

        if kind == "timed":
            if elapsed < a:
                self.drive(v, omega)
                return
        else:  # until_lane
            if elapsed < a:                       # too early to trust what we see
                self.drive(v, omega)
                return
            if elapsed < b:
                if not self.lane_ahead(hsv, width):
                    self.drive(v, omega)
                    return
                rospy.loginfo("[navigator] lane reacquired after %.1fs of turning",
                              elapsed)
            else:
                rospy.logwarn("[navigator] turn timed out after %.1fs -- no lane found, "
                              "continuing anyway", elapsed)

        idx += 1
        if idx < len(phases):
            self.maneuver = (phases, idx, rospy.Time.now())
            return

        # Maneuver finished. ALWAYS go back to lane following -- that is what drives us
        # out of the intersection and on to the next red line. Stay deaf to red for a
        # moment so we don't re-trigger on the line we just crossed.
        self.ignore_red_until = rospy.Time.now() + rospy.Duration(RED_COOLDOWN_S)
        self.prev_error = 0.0
        self.last_omega = 0.0
        self.lost_frames = 0
        with self.lock:
            self.state = "LANE_FOLLOW"
            self.state_since = rospy.Time.now()
            if self.plan:
                self.finish_at = None
                rospy.loginfo("[navigator] maneuver done -> following lane to next "
                              "stop line (%d command(s) left)", len(self.plan))
            else:
                # Last command: keep following the lane briefly so we clear the
                # intersection and settle straight, THEN stop.
                self.finish_at = rospy.Time.now() + rospy.Duration(FINISH_S)
                rospy.loginfo("[navigator] final maneuver done -> settling for %.1fs",
                              FINISH_S)
        self.log_event("maneuver_done", command=self.last_command,
                       remaining=len(self.plan))

    def enter(self, state):
        with self.lock:
            self.state = state
            self.state_since = rospy.Time.now()

    def log_event(self, event, **data):
        """Append one event to the run log file. Safe to call with logging disabled."""
        if self.runlog is not None:
            self.runlog.log(event, **data)

    # ================= motion =================

    def drive(self, v, omega):
        msg = Twist2DStamped()
        msg.header.stamp = rospy.Time.now()
        msg.v = v
        msg.omega = omega
        self.pub.publish(msg)

    def stop(self):
        """
        Get a zero-velocity command through, reliably.

        This MUST NOT use rospy.sleep(): once shutdown starts rospy disables sleeping and
        raises, so the old version bailed out before the stop command was ever sent -- the
        wheels driver then just held the last command and the bot drove on. Plain
        time.sleep() keeps working during shutdown, and we publish repeatedly because a
        single message can be dropped while the node is tearing down.
        """
        msg = Twist2DStamped()
        msg.v = 0.0
        msg.omega = 0.0
        for _ in range(10):
            try:
                msg.header.stamp = rospy.Time.now()
                self.pub.publish(msg)
            except Exception:
                pass
            time.sleep(0.05)
        print("[navigator] STOPPED (zero velocity sent).")

    def handle_sigint(self, signum, frame):
        """Ctrl-C: stop the wheels FIRST, then shut the node down."""
        print("\n[navigator] interrupt -- stopping robot...")
        with self.lock:
            self.plan = []
            self.state = "DONE"
            self.maneuver = None
        self.stop()
        rospy.signal_shutdown("user interrupt")

    # ================= http =================

    def start_http(self):
        nav = self

        class Handler(BaseHTTPRequestHandler):
            def _send(self, code, payload):
                body = json.dumps(payload).encode()
                self.send_response(code)
                self.send_header("Content-Type", "application/json")
                # the web app is served from elsewhere, so allow cross-origin calls
                self.send_header("Access-Control-Allow-Origin", "*")
                self.send_header("Access-Control-Allow-Headers", "Content-Type")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def do_OPTIONS(self):
                self._send(200, {})

            def _send_html(self, code, html):
                body = html.encode("utf-8")
                self.send_response(code)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def do_GET(self):
                if self.path.startswith("/status"):
                    self._send(200, nav.status())
                    return
                if self.path.split("?")[0] in ("/", "/index.html", "/ui"):
                    try:
                        with open(UI_FILE, encoding="utf-8") as f:
                            self._send_html(200, f.read())
                    except Exception:
                        self._send_html(404,
                            "<h1>No planner UI installed</h1><p>Copy <code>map_ui.html"
                            "</code> to <code>%s</code> on the robot, then reload."
                            "</p><p>The robot API still works: "
                            "<code>GET /status</code>, <code>POST /navigate</code>, "
                            "<code>POST /abort</code>.</p>" % UI_FILE)
                    return
                self._send(404, {"error": "not found"})

            def do_POST(self):
                if self.path.startswith("/abort"):
                    nav.abort()
                    self._send(200, {"status": "aborted"})
                    return
                if self.path.startswith("/follow"):
                    nav.follow_only()
                    self._send(200, {"status": "following"})
                    return
                if self.path.startswith("/localize"):
                    self._send(200, nav.localize())
                    return
                if not self.path.startswith("/navigate"):
                    self._send(404, {"error": "not found"})
                    return
                try:
                    n = int(self.headers.get("Content-Length", 0))
                    data = json.loads(self.rfile.read(n).decode() or "{}")
                except Exception as e:
                    self._send(400, {"error": "bad JSON: %s" % e})
                    return
                commands = data.get("commands")
                if not isinstance(commands, list):
                    self._send(400, {"error": "Missing 'commands' array"})
                    return
                ok, message = nav.set_plan(commands)
                if not ok:
                    self._send(400, {"error": message})
                    return
                self._send(200, {"status": "executed",
                                 "commands_processed": len(commands)})

            def log_message(self, *args):
                pass  # keep ROS logs readable

        server = ThreadingHTTPServer(("0.0.0.0", HTTP_PORT), Handler)
        threading.Thread(target=server.serve_forever, daemon=True).start()
        rospy.loginfo("[navigator] HTTP listening on port %d", HTTP_PORT)


if __name__ == "__main__":
    try:
        Navigator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
