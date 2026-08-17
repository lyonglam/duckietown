#!/usr/bin/env python3
"""
lane_follower.py -- custom lane follower for duck4. Replaces indefinite_navigation.

WHY THIS EXISTS
---------------
duck4's base image is newer than daffy and is MISSING led_emitter_node, which the
daffy demo's fsm_node requires -> fsm_node dies -> the whole demo collapses. But the
bot's DRIVERS are fine (camera publishes, wheels_driver + kinematics_node run). This
node talks only to the working parts, so it sidesteps the version mess completely:

    camera_node/image/compressed  ->  THIS NODE  ->  car_cmd_switch_node/cmd
                                                     -> kinematics_node -> wheels

Publishing to car_cmd_switch_node/cmd injects commands DOWNSTREAM of the fsm-driven
switch, so nothing depends on fsm_node / led_emitter. It still goes through
kinematics_node, so your wheel calibration (trim/gain) is applied. No demo needed.

ATTRIBUTION
-----------
Method: HSV color masking -> cv2.moments centroid -> proportional-derivative steering.
This is a standard, widely-taught lane-detection approach, not specific to any one team.

Prior work consulted: the TUM QuackQuack team's `simple_lane_follower`
(https://github.com/DuckietownTUM/QuackQuack). We independently implemented this node,
but took two things from their solution and credit them here:
  (1) the idea of publishing to car_cmd_switch_node/cmd to inject drive commands
      downstream of the fsm-driven switch (this is what lets us run without fsm_node), and
  (2) initial HSV starting values, which we then re-tuned for our own lab lighting
      (see hsv_calibrator.py -- our tuned values live in hsv_thresholds.json).
All control gains, the ROI size, the speed-reduction law, the single-line-visible
fallback, the lane-lost behavior and the shutdown brake are our own.

BEFORE RUNNING
--------------
1. LENS CAP OFF.
2. Tune thresholds with hsv_calibrator.py -> writes /data/hsv_thresholds.json.
   This node loads that file if present, else falls back to built-in defaults.
3. Have a brake ready: Ctrl-C stops this node and it publishes a zero command on exit.

RUN (on the bot, so there is no VM<->bot latency on the video stream):
    ssh duckie@duck4.local
    docker exec -it duckiebot-interface bash
    source /environment.sh          # if ROS_MASTER_URI is empty
    export VEHICLE_NAME=duck4
    python3 /data/lane_follower.py

TUNING ORDER IF IT MISBEHAVES
-----------------------------
- Wanders / doesn't react enough      -> raise KP
- Oscillates / snakes left-right      -> lower KP, or raise KD
- Too fast to control                 -> lower V_BAR
- Sees lane where there is none       -> re-tune HSV (hsv_calibrator.py)
"""

import json
import os

import cv2
import numpy as np
import rospy
from duckietown_msgs.msg import Twist2DStamped
from sensor_msgs.msg import CompressedImage

VEHICLE = os.environ.get("VEHICLE_NAME", "duck4")
THRESHOLD_FILE = os.environ.get("HSV_FILE", "/data/hsv_thresholds.json")

# ---- control gains (start conservative; tune on the bot, then these become OURS) ----
KP = 4.0            # how hard it steers per unit of error
KD = 2.0            # damping; fights oscillation
V_BAR = 0.10        # base forward speed (m/s). Keep LOW while testing.
OMEGA_MAX = 6.0     # clamp on turn rate (rad/s)
V_MIN = 0.05        # never crawl below this while following

# Slow down when off-center. We scale speed by how straight we are (cosine-ish taper)
# rather than a linear penalty -- gentler near center, firmer at large errors.
SLOWDOWN_STRENGTH = 0.8

ROI_FRACTION = 0.40     # use bottom 40% of the image

# ---- lane geometry gating (fixes curves + wrong-lane stop lines) ----
# A Duckietown road is: [white edge][oncoming lane][YELLOW center][our lane][white edge].
# So from our lane, yellow is on the left and our white edge is on the right. Searching
# the whole width lets the FAR white edge win on curves and steer us the wrong way.
YELLOW_SEARCH_MAX = 0.70    # look for yellow only in the left 70% of the image
WHITE_SEARCH_MIN = 0.30     # look for white only in the right 70%
MIN_LANE_WIDTH_PX = 20      # white must be at least this far right of yellow to be ours
HALF_LANE_FRAC = 0.25       # when only one line is visible, aim this far off it
RED_LANE_MIN = 0.30         # if yellow is missing, ignore red left of this fraction

# ---- behavior when both lines vanish (happens mid-curve) ----
LANE_MEMORY_DECAY = 0.85    # keep turning the way we were, fading out
LANE_LOST_MAX_FRAMES = 15   # after this many blind frames, stop instead of guessing
RED_STOP_AREA = 3000    # red pixel count in ROI that counts as "at a stop line"

DEFAULT_THRESHOLDS = {
    "yellow": {"lower": [15, 80, 80], "upper": [40, 255, 255]},
    "white": {"lower": [0, 0, 170], "upper": [180, 70, 255]},
    "red": {"lower": [0, 120, 120], "upper": [10, 255, 255]},
    "red2": {"lower": [170, 120, 120], "upper": [180, 255, 255]},
}


def load_thresholds():
    """Prefer calibrated values; fall back to defaults with a loud warning."""
    try:
        with open(THRESHOLD_FILE) as f:
            data = json.load(f)
        rospy.loginfo("[lane_follower] loaded thresholds from %s", THRESHOLD_FILE)
        for key in DEFAULT_THRESHOLDS:
            data.setdefault(key, DEFAULT_THRESHOLDS[key])
        return data
    except Exception:
        rospy.logwarn("[lane_follower] no %s -- using DEFAULT thresholds. "
                      "Run hsv_calibrator.py for your lighting!", THRESHOLD_FILE)
        return DEFAULT_THRESHOLDS


def centroid_x(mask, x_lo=None, x_hi=None):
    """
    X position of the mask's center of mass, or None if the color isn't there.

    x_lo/x_hi restrict the search to a horizontal band (in pixels). This matters: a
    Duckietown road has white edge lines on BOTH sides, so an ungated white search can
    lock onto the far edge of the ONCOMING lane. Blanking outside the band (rather than
    cropping) keeps the returned x in full-image coordinates.
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
    if m["m00"] < 500:          # too few pixels to trust
        return None
    return m["m10"] / m["m00"]


class LaneFollower:
    def __init__(self):
        rospy.init_node("lane_follower", anonymous=False)
        self.th = load_thresholds()
        self.prev_error = 0.0
        self.have_lane = False
        self.last_omega = 0.0       # steering memory for when the lines drop out
        self.lost_frames = 0

        # Stop-line behavior: this node only SLOWS/STOPS at red lines for now.
        # Intersection turning + A* plug in here later (that's the next phase).
        self.stopped_on_red = False

        self.pub = rospy.Publisher(
            "/{}/car_cmd_switch_node/cmd".format(VEHICLE), Twist2DStamped, queue_size=1)
        rospy.Subscriber(
            "/{}/camera_node/image/compressed".format(VEHICLE), CompressedImage,
            self.cb_image, queue_size=1, buff_size=2 ** 24)

        rospy.on_shutdown(self.stop)
        rospy.loginfo("[lane_follower] running. Ctrl-C to stop.")

    def mask_for(self, hsv, name):
        lo = np.array(self.th[name]["lower"])
        hi = np.array(self.th[name]["upper"])
        mask = cv2.inRange(hsv, lo, hi)
        if name == "red":       # red wraps the hue circle -> second range
            lo2 = np.array(self.th["red2"]["lower"])
            hi2 = np.array(self.th["red2"]["upper"])
            mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lo2, hi2))
        # clean up specks and close small gaps in the lines
        k = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)
        return mask

    def cb_image(self, msg):
        arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if frame is None:
            return

        h, w = frame.shape[:2]
        roi = frame[int(h * (1.0 - ROI_FRACTION)):, :]
        hsv = cv2.GaussianBlur(cv2.cvtColor(roi, cv2.COLOR_BGR2HSV), (5, 5), 0)

        # --- find the two lines, each only where it can legitimately be ---
        # Yellow is the centerline, so it lives on our LEFT. White is our lane's right
        # edge, so it lives on our RIGHT. Searching the full width lets the far side of
        # the road hijack either one -- which is what sent us into the oncoming lane on
        # curves.
        yc = centroid_x(self.mask_for(hsv, "yellow"), x_hi=w * YELLOW_SEARCH_MAX)
        wc = centroid_x(self.mask_for(hsv, "white"), x_lo=w * WHITE_SEARCH_MIN)
        center = w / 2.0

        # Geometry check: our lane is bounded yellow-on-left, white-on-right. If white
        # came out LEFT of yellow we've locked onto the far edge line, so drop it and
        # navigate off the centerline alone.
        if yc is not None and wc is not None and wc <= yc + MIN_LANE_WIDTH_PX:
            rospy.logwarn_throttle(
                2.0, "[lane_follower] white line looks wrong (left of yellow) -- ignoring")
            wc = None

        # --- red stop line, but only in OUR lane ---
        # A stop line on the far side of the road is not ours to obey. Our lane is to the
        # right of the yellow centerline, so ignore red left of it.
        red = self.mask_for(hsv, "red")
        red_cut = int(yc) if yc is not None else int(w * RED_LANE_MIN)
        red[:, :max(0, red_cut)] = 0
        if cv2.countNonZero(red) > RED_STOP_AREA:
            if not self.stopped_on_red:
                rospy.loginfo("[lane_follower] RED LINE -> stopping")
                self.stopped_on_red = True
            self.drive(0.0, 0.0)
            return
        self.stopped_on_red = False

        if yc is not None and wc is not None:
            lane_center = (yc + wc) / 2.0
        elif yc is not None:
            # only yellow visible: it should sit LEFT of us, so aim right of it
            lane_center = yc + w * HALF_LANE_FRAC
        elif wc is not None:
            # only white visible: it should sit RIGHT of us, so aim left of it
            lane_center = wc - w * HALF_LANE_FRAC
        else:
            # Lost both lines. Do NOT straighten out -- on a tight curve both lines leave
            # the view for a moment, and driving straight there is exactly what carries us
            # out of the lane. Keep the turn we were already making, fading it out, and
            # stop if the lane doesn't come back.
            if self.have_lane:
                rospy.logwarn_throttle(2.0, "[lane_follower] lane lost -- holding turn")
            self.have_lane = False
            self.lost_frames += 1
            if self.lost_frames <= LANE_LOST_MAX_FRAMES:
                self.last_omega *= LANE_MEMORY_DECAY
                self.drive(V_MIN, self.last_omega)
            else:
                rospy.logwarn_throttle(2.0, "[lane_follower] lane still lost -- stopped")
                self.drive(0.0, 0.0)
            return

        self.have_lane = True
        self.lost_frames = 0

        # normalized error in [-1, 1]; negative = lane is left of us
        error = (lane_center - center) / center
        d_error = error - self.prev_error
        self.prev_error = error

        # negative sign: lane to the LEFT (negative error) must produce a LEFT
        # turn, which is POSITIVE omega in Duckietown's convention.
        omega = -(KP * error + KD * d_error)
        omega = float(np.clip(omega, -OMEGA_MAX, OMEGA_MAX))

        # Slow down when far off-center -- big corrections at speed cause overshoot.
        # Quadratic taper: barely slows near center, drops off harder as error grows.
        v = max(V_MIN, V_BAR * (1.0 - SLOWDOWN_STRENGTH * error * error))
        self.last_omega = omega     # remembered in case we lose the lines next frame
        self.drive(v, omega)

    def drive(self, v, omega):
        msg = Twist2DStamped()
        msg.header.stamp = rospy.Time.now()
        msg.v = v
        msg.omega = omega
        self.pub.publish(msg)

    def stop(self):
        """Always leave the wheels stopped, even on Ctrl-C / crash."""
        for _ in range(5):
            self.drive(0.0, 0.0)
            rospy.sleep(0.05)
        rospy.loginfo("[lane_follower] stopped.")


if __name__ == "__main__":
    try:
        LaneFollower()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
