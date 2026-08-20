#!/usr/bin/env python3
"""
fake_log_generator.py -- write a made-up run_log.jsonl so analyze_runs.py can be
developed and tested before the robot has produced any real data.

Event shapes match exactly what navigator.py + run_logger.py emit. The DATA IS FAKE:
never mix this output into a real log used for the report.

Usage: python3 fake_log_generator.py [out_path]     (default ./run_log.jsonl)
"""

import json
import random
import sys
import time

COMMANDS = ["left", "right", "straight"]
N_ROUTES = 8


def event(t, name, **data):
    return {"t": round(t, 3),
            "iso": time.strftime("%Y-%m-%dT%H:%M:%S", time.localtime(t)),
            "event": name, "data": data}


def fake_route(t, events):
    """Append one route's events starting at t; return the next route's start time."""
    plan = [random.choice(COMMANDS) for _ in range(random.randint(2, 5))]
    events.append(event(t, "plan_started", commands=plan + ["stop"], count=len(plan)))
    remaining = len(plan)
    for cmd in plan:
        t += random.uniform(8.0, 20.0)            # lane-follow to the stop line
        remaining -= 1
        events.append(event(t, "command_executed", command=cmd, remaining=remaining))
        t += random.uniform(2.0, 5.0)             # the maneuver itself
        events.append(event(t, "maneuver_done", command=cmd, remaining=remaining))
        if random.random() < 0.15:                # occasional duckie on the road
            t += random.uniform(1.0, 4.0)
            events.append(event(t, "avoidance_start",
                                tof_m=round(random.uniform(0.15, 0.34), 3)))
            dur = random.uniform(3.0, 7.0)
            t += dur
            events.append(event(t, "avoidance_end", duration_s=round(dur, 1)))
        if random.random() < 0.12:                # sometimes the lane drops out
            t += random.uniform(1.0, 6.0)
            events.append(event(t, "stuck_entered", lost_frames=16))
            if random.random() < 0.5:             # ...and sometimes it comes back
                stuck = random.uniform(2.0, 10.0)
                t += stuck
                events.append(event(t, "stuck_recovered", stuck_s=round(stuck, 1)))
            else:                                 # ...or it doesn't: route dies here
                t += 31.0
                events.append(event(t, "stuck_timeout", stuck_s=31.0))
                return t + random.uniform(20.0, 60.0)
    t += random.uniform(3.0, 6.0)                 # FINISH_S settle before stopping
    events.append(event(t, "route_complete"))
    return t + random.uniform(20.0, 60.0)         # gap before someone posts a new plan


def main(path):
    random.seed(4)    # same fake data every run -- easier to eyeball the analyzer
    events, t = [], time.time() - 3600.0
    for _ in range(N_ROUTES):
        t = fake_route(t, events)
    with open(path, "w") as f:
        for e in events:
            f.write(json.dumps(e) + "\n")
    print("wrote %d events (%d routes) to %s -- FAKE data, for testing only"
          % (len(events), N_ROUTES, path))


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "run_log.jsonl")
