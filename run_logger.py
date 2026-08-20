#!/usr/bin/env python3
"""
run_logger.py -- append-only JSON-lines event log for navigator runs.

WHY THIS EXISTS
---------------
The report needs EVIDENCE: success rate per route, timings, how often the bot got
confused. rospy's console log scrolls away when the terminal closes and is painful to
parse afterwards. This writes one JSON object per line to a file instead, which
survives the run and loads line-by-line on a laptop (see analyze_runs.py).

WHY JSON LINES AND NOT ONE JSON DOCUMENT
----------------------------------------
The node gets Ctrl-C'd, crashes, and power-cycles. A single JSON array needs a closing
bracket that a killed process never writes; JSONL stays valid up to the last complete
line no matter how the process died. Each write opens, appends, and closes the file for
the same reason -- nothing is buffered when the power goes.

Standard library only -- the bot's containers may have nothing else installed.

USAGE
-----
    from run_logger import RunLogger
    log = RunLogger()                     # /data/run_log.jsonl, or $RUN_LOG_FILE
    log.log("plan_started", commands=["left", "straight"], count=2)
"""

import json
import os
import threading
import time


class RunLogger:
    def __init__(self, path=None):
        self.path = path or os.environ.get("RUN_LOG_FILE", "/data/run_log.jsonl")
        # ROS callbacks and the HTTP thread can both log -- keep lines whole.
        self._lock = threading.Lock()
        self._warned = False

    def log(self, event, **data):
        """Append one event. NEVER raises -- a full disk must not stop the robot."""
        now = time.time()
        record = {
            "t": round(now, 3),
            "iso": time.strftime("%Y-%m-%dT%H:%M:%S", time.localtime(now)),
            "event": event,
            "data": data,
        }
        try:
            line = json.dumps(record)
            with self._lock:
                with open(self.path, "a") as f:
                    f.write(line + "\n")
        except Exception as e:
            if not self._warned:      # say it once, not once per frame
                self._warned = True
                print("[run_logger] cannot write %s: %s -- logging disabled"
                      % (self.path, e))
