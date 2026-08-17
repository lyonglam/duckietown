#!/usr/bin/env python3
"""
hsv_calibrator.py -- interactive HSV threshold tuner for duck4's camera.

Same idea as the labyrinth-project color tuner: live video + trackbars, you drag
sliders until ONLY the thing you care about shows up white in the mask window,
then press 's' to save. lane_follower.py reads the saved file automatically.

WHY YOU NEED THIS
-----------------
HSV thresholds are lighting-dependent. Values that worked in someone else's lab
will NOT work in yours. Tuning these on YOUR track under YOUR lighting is the
single biggest factor in whether lane following works. Budget ~10 minutes.

WHERE TO RUN IT
---------------
Needs a GUI window, so run it in the VM inside gui-tools (which has X + ROS):
    dts start_gui_tools duck4
then inside that shell:
    python3 /code/catkin_ws/hsv_calibrator.py       (or wherever you put it)

LENS CAP OFF. Point the bot at a real lane with yellow + white lines visible.

CONTROLS
--------
  1 / 2 / 3 : switch which color you are tuning (1=yellow, 2=white, 3=red)
  s         : save ALL colors to hsv_thresholds.json
  q         : quit without saving

TIP: tune so the mask is mostly-clean. A few stray dots are fine (the follower
blurs + does morphology). Big blobs of floor/wall showing up = keep tightening.
"""

import json
import os

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import CompressedImage

VEHICLE = os.environ.get("VEHICLE_NAME", "duck4")
CAMERA_TOPIC = "/{}/camera_node/image/compressed".format(VEHICLE)

# Where the thresholds get written. lane_follower.py looks here first.
# /data is shared into the bot's containers, which makes it easy to copy over.
SAVE_PATH = os.environ.get("HSV_SAVE_PATH", "/data/hsv_thresholds.json")

# Starting points, taken from a previous TUM team's working values.
# These are a STARTING GUESS ONLY -- expect to change them for your lighting.
DEFAULTS = {
    "yellow": {"lower": [15, 80, 80], "upper": [40, 255, 255]},
    "white": {"lower": [0, 0, 170], "upper": [180, 70, 255]},
    # Red wraps around the hue circle, so it needs TWO ranges. We tune the
    # first one here and mirror it to the second on save.
    "red": {"lower": [0, 120, 120], "upper": [10, 255, 255]},
}

COLOR_ORDER = ["yellow", "white", "red"]
WIN = "controls"


class HSVCalibrator:
    def __init__(self):
        self.frame = None
        self.current = "yellow"
        self.values = {k: {kk: list(vv) for kk, vv in v.items()} for k, v in DEFAULTS.items()}

        rospy.init_node("hsv_calibrator", anonymous=True)
        rospy.Subscriber(CAMERA_TOPIC, CompressedImage, self.cb_image, queue_size=1,
                         buff_size=2 ** 24)

        cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(WIN, 500, 300)
        for i, name in enumerate(["H min", "S min", "V min", "H max", "S max", "V max"]):
            maxval = 179 if name.startswith("H") else 255
            cv2.createTrackbar(name, WIN, 0, maxval, lambda _v: None)
        self.load_into_trackbars(self.current)

        rospy.loginfo("[hsv_calibrator] listening on %s", CAMERA_TOPIC)

    # ---------- trackbar <-> values ----------
    def load_into_trackbars(self, color):
        lo = self.values[color]["lower"]
        hi = self.values[color]["upper"]
        for name, val in zip(["H min", "S min", "V min"], lo):
            cv2.setTrackbarPos(name, WIN, int(val))
        for name, val in zip(["H max", "S max", "V max"], hi):
            cv2.setTrackbarPos(name, WIN, int(val))

    def read_trackbars(self):
        lo = [cv2.getTrackbarPos(n, WIN) for n in ["H min", "S min", "V min"]]
        hi = [cv2.getTrackbarPos(n, WIN) for n in ["H max", "S max", "V max"]]
        return lo, hi

    def cb_image(self, msg):
        arr = np.frombuffer(msg.data, np.uint8)
        self.frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)

    # ---------- main loop ----------
    def run(self):
        rate = rospy.Rate(15)
        while not rospy.is_shutdown():
            if self.frame is not None:
                frame = self.frame.copy()
                h, w = frame.shape[:2]

                # Show the SAME region of interest the follower uses: bottom 35%.
                # Tuning on the full image is misleading -- the sky/horizon isn't
                # what the follower looks at.
                roi_top = int(h * 0.65)
                roi = frame[roi_top:, :]

                lo, hi = self.read_trackbars()
                self.values[self.current]["lower"] = lo
                self.values[self.current]["upper"] = hi

                hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
                hsv = cv2.GaussianBlur(hsv, (5, 5), 0)
                mask = cv2.inRange(hsv, np.array(lo), np.array(hi))
                if self.current == "red":
                    # mirror the low-hue range to the high end of the hue circle
                    lo2 = [180 - hi[0], lo[1], lo[2]]
                    hi2 = [179, hi[1], hi[2]]
                    mask = cv2.bitwise_or(mask, cv2.inRange(hsv, np.array(lo2), np.array(hi2)))

                cv2.putText(roi, "tuning: %s  (1=yellow 2=white 3=red, s=save, q=quit)"
                            % self.current, (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                            (0, 255, 0), 1)
                cv2.imshow("roi (what the follower sees)", roi)
                cv2.imshow("mask (want ONLY your color white)", mask)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                rospy.loginfo("[hsv_calibrator] quit without saving")
                break
            elif key == ord("s"):
                self.save()
            elif key in (ord("1"), ord("2"), ord("3")):
                self.current = COLOR_ORDER[key - ord("1")]
                self.load_into_trackbars(self.current)
                rospy.loginfo("[hsv_calibrator] now tuning: %s", self.current)
            rate.sleep()

        cv2.destroyAllWindows()

    def save(self):
        out = {k: {kk: list(vv) for kk, vv in v.items()} for k, v in self.values.items()}
        # store red's mirrored second range explicitly so the follower doesn't guess
        r = out["red"]
        out["red2"] = {"lower": [180 - r["upper"][0], r["lower"][1], r["lower"][2]],
                       "upper": [179, r["upper"][1], r["upper"][2]]}
        try:
            with open(SAVE_PATH, "w") as f:
                json.dump(out, f, indent=2)
            rospy.loginfo("[hsv_calibrator] SAVED -> %s", SAVE_PATH)
            print("\nSAVED to %s:\n%s\n" % (SAVE_PATH, json.dumps(out, indent=2)))
        except Exception as e:
            rospy.logerr("[hsv_calibrator] could not save to %s: %s", SAVE_PATH, e)
            print("\nCOPY THESE VALUES MANUALLY:\n%s\n" % json.dumps(out, indent=2))


if __name__ == "__main__":
    HSVCalibrator().run()
