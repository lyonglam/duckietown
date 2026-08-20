#!/usr/bin/env python3
"""
apriltag_scan_test.py -- ONE-OFF PLACEMENT TEST. Not part of the production pipeline.

WHY THIS EXISTS
---------------
Before committing to a tag mounting/placement design for all 42 tiles, we need to know
whether a FLAT tag36h11 tag lying on the floor is detectable and decodable by duck4's
camera at all -- and between which distances it is detectable RELIABLY. A flat tag is
viewed at a shallow, perspective-squashed angle, which is the worst case for the
decoder, so: measure first, glue later.

HOW TO USE IT
-------------
Live, against the robot's camera (needs a ROS shell on/against the bot):

    python3 apriltag_scan_test.py --live

    Put the tag on the floor, place the bot at different distances, watch the prints.
    Every frame with a detection is saved ANNOTATED into ./apriltag_test_captures/ --
    that folder is the deliverable: it lets us compare "detected at what distance"
    side by side afterwards without re-running anything. Ctrl-C prints a summary.

Offline, against a saved photo (no ROS needed -- rospy is only imported for --live):

    python3 apriltag_scan_test.py --file photo.jpg

READING THE NUMBERS
-------------------
decision_margin is the decoder's confidence (how far the bit samples landed from the
decision boundary). Higher is better; detections below ~20 are marginal and will
flicker frame to frame. The min/max range in the summary is the answer to "was that
distance solid or borderline".

Dependencies: cv2 + numpy (already on the bot), pupil_apriltags
(pip install pupil-apriltags), rospy only for --live.
"""

import argparse
import os
import signal
import sys
import time

import cv2
import numpy as np

try:
    from pupil_apriltags import Detector
except ImportError:
    sys.exit("pupil_apriltags is not installed. Run:  pip install pupil-apriltags")

CAPTURE_DIR = "apriltag_test_captures"
PROCESS_EVERY_S = 0.5   # ~2 Hz in --live mode; a test script needn't hammer the CPU
VEHICLE = os.environ.get("VEHICLE_NAME", "duck4")
TOPIC = "/{}/camera_node/image/compressed".format(VEHICLE)


class TagScanner:
    def __init__(self):
        # This project uses tag36h11 ONLY (per project notes). Restricting the family
        # keeps the decoder from wasting time on -- or false-matching -- other families.
        self.detector = Detector(families="tag36h11")
        self.frames = 0
        self.frames_with_tag = 0
        self.margin_min = None
        self.margin_max = None
        self.last_processed = 0.0

    def process(self, frame, label):
        """Detect on one BGR frame: print per-tag lines, save annotated hits."""
        self.frames += 1
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        tags = self.detector.detect(gray)
        stamp = time.strftime("%H:%M:%S")
        if not tags:
            print("[no tag detected] frame %d @ %s (%s)" % (self.frames, stamp, label))
            return
        self.frames_with_tag += 1
        for t in tags:
            m = t.decision_margin
            self.margin_min = m if self.margin_min is None else min(self.margin_min, m)
            self.margin_max = m if self.margin_max is None else max(self.margin_max, m)
            print("frame %d @ %s: tag id=%d  decision_margin=%.1f  center=(%.0f, %.0f)"
                  % (self.frames, stamp, t.tag_id, m, t.center[0], t.center[1]))
        self.save_annotated(frame, tags)

    def save_annotated(self, frame, tags):
        os.makedirs(CAPTURE_DIR, exist_ok=True)
        out = frame.copy()
        for t in tags:
            cv2.polylines(out, [t.corners.astype(int)], True, (0, 255, 0), 2)
            cx, cy = int(t.center[0]), int(t.center[1])
            text = "id=%d m=%.0f" % (t.tag_id, t.decision_margin)
            # dark outline first, green on top -- readable on any background
            for color, thick in (((0, 0, 0), 4), ((0, 255, 0), 1)):
                cv2.putText(out, text, (max(5, cx - 40), max(20, cy - 12)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, thick, cv2.LINE_AA)
        now = time.time()
        ids = "-".join(str(t.tag_id) for t in tags)
        name = "%s-%03d_id%s.jpg" % (
            time.strftime("%Y%m%d-%H%M%S", time.localtime(now)),
            int(now * 1000) % 1000, ids)
        path = os.path.join(CAPTURE_DIR, name)
        cv2.imwrite(path, out)
        print("    saved %s" % path)

    def summary(self):
        print("\n---- summary ----")
        print("frames processed : %d" % self.frames)
        print("with detection   : %d" % self.frames_with_tag)
        if self.margin_min is None:
            print("decision_margin  : no detections at all")
        else:
            print("decision_margin  : min %.1f, max %.1f  (below ~20 = borderline)"
                  % (self.margin_min, self.margin_max))
        print("-----------------")


def run_live(scanner):
    # rospy is imported HERE, not at the top, so --file works on a laptop with no ROS.
    try:
        import rospy
        from sensor_msgs.msg import CompressedImage
    except ImportError:
        sys.exit("rospy not available -- use --file <image> instead, or run this "
                 "inside the robot's ROS container.")

    # disable_signals so OUR Ctrl-C handler runs and can print the summary first.
    rospy.init_node("apriltag_scan_test", anonymous=True, disable_signals=True)

    def cb(msg):
        now = time.time()
        if now - scanner.last_processed < PROCESS_EVERY_S:
            return
        scanner.last_processed = now
        frame = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            return
        scanner.process(frame, "live")

    def on_sigint(signum, stackframe):
        scanner.summary()
        rospy.signal_shutdown("user interrupt")

    signal.signal(signal.SIGINT, on_sigint)
    signal.signal(signal.SIGTERM, on_sigint)
    rospy.Subscriber(TOPIC, CompressedImage, cb, queue_size=1, buff_size=2 ** 24)
    print("listening on %s at ~%.0f Hz -- Ctrl-C for the summary"
          % (TOPIC, 1.0 / PROCESS_EVERY_S))
    rospy.spin()


def run_file(scanner, path):
    frame = cv2.imread(path, cv2.IMREAD_COLOR)
    if frame is None:
        sys.exit("could not read image: %s" % path)
    scanner.process(frame, os.path.basename(path))


def main():
    ap = argparse.ArgumentParser(
        description="tag36h11 floor-placement test: detect via the robot camera or "
                    "a saved photo, save annotated captures for comparison.")
    mode = ap.add_mutually_exclusive_group(required=True)
    mode.add_argument("--live", action="store_true",
                      help="subscribe to %s and scan at ~2 Hz" % TOPIC)
    mode.add_argument("--file", metavar="PATH",
                      help="run once on a saved image (no ROS needed)")
    args = ap.parse_args()

    scanner = TagScanner()
    if args.live:
        run_live(scanner)
    else:
        run_file(scanner, args.file)


if __name__ == "__main__":
    main()
