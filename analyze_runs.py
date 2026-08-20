#!/usr/bin/env python3
"""
analyze_runs.py -- turn a run_log.jsonl from the robot into report material.

Runs on the LAPTOP, not the robot: matplotlib is not on the bot and doesn't need to
be. Copy the log off (scp duckie@duck4.local:/data/run_log.jsonl .) and run:

    python3 analyze_runs.py run_log.jsonl

Prints a summary + per-route table, and writes next to the log file:
    commands_by_type.png   bar chart of executed commands
    routes.csv             the per-route table, for the report

A "route" is everything from one plan_started event to the next. There is no route id
in the log -- the robot runs one plan at a time, so arrival order IS the grouping.
"""

import csv
import json
import os
import sys
from collections import Counter


def load_events(path):
    events = []
    with open(path) as f:
        for lineno, line in enumerate(f, 1):
            line = line.strip()
            if not line:
                continue
            try:
                events.append(json.loads(line))
            except ValueError:
                # a killed process can leave a torn final line -- skip it, don't die
                print("skipping unparseable line %d" % lineno)
    events.sort(key=lambda e: e.get("t", 0))
    return events


def split_routes(events):
    """Group events into routes; events before the first plan_started are dropped."""
    routes = []
    current = None
    for e in events:
        if e["event"] == "plan_started":
            current = {"start": e, "events": []}
            routes.append(current)
        elif current is not None:
            current["events"].append(e)
    return routes


def summarize_route(i, route):
    start, evs = route["start"], route["events"]
    complete = next((e for e in evs if e["event"] == "route_complete"), None)
    # Incomplete routes have no natural end; the last event seen is the honest bound.
    end_t = complete["t"] if complete else (evs[-1]["t"] if evs else start["t"])
    return {
        "route": i,
        "started": start.get("iso", ""),
        "planned": " ".join(start["data"].get("commands", [])),
        "n_planned": start["data"].get("count", 0),
        "n_executed": sum(1 for e in evs if e["event"] == "command_executed"),
        "duration_s": round(end_t - start["t"], 1),
        "completed": bool(complete),
        "stuck": sum(1 for e in evs if e["event"] == "stuck_entered"),
        "stuck_timeout": sum(1 for e in evs if e["event"] == "stuck_timeout"),
    }


def save_chart(events, out_dir):
    counts = Counter(e["data"].get("command", "?")
                     for e in events if e["event"] == "command_executed")
    if not counts:
        print("no command_executed events -- skipping the chart")
        return
    try:
        import matplotlib
        matplotlib.use("Agg")   # save-to-file only; works without a display
        import matplotlib.pyplot as plt
        from matplotlib.ticker import MaxNLocator
    except ImportError:
        print("matplotlib not installed -- skipping the chart (pip install matplotlib)")
        return

    names = sorted(counts)
    values = [counts[n] for n in names]
    fig, ax = plt.subplots(figsize=(5.0, 3.2), dpi=150)
    fig.patch.set_facecolor("#fcfcfb")
    ax.set_facecolor("#fcfcfb")
    ax.bar(names, values, color="#2a78d6", width=0.55, zorder=3)
    for i, v in enumerate(values):    # direct labels beat squinting at gridlines
        ax.text(i, v, " %d" % v, ha="center", va="bottom", fontsize=9, color="#0b0b0b")
    ax.set_title("Commands executed by type", loc="left", fontsize=11, color="#0b0b0b")
    ax.set_ylabel("times executed", fontsize=9, color="#52514e")
    ax.tick_params(colors="#898781", labelsize=9)
    ax.yaxis.grid(True, color="#e1e0d9", linewidth=0.8)
    ax.set_axisbelow(True)
    ax.yaxis.set_major_locator(MaxNLocator(integer=True))  # counts: no 2.5 ticks
    for side in ("top", "right", "left"):
        ax.spines[side].set_visible(False)
    ax.spines["bottom"].set_color("#c3c2b7")
    fig.tight_layout()
    png = os.path.join(out_dir, "commands_by_type.png")
    fig.savefig(png)
    print("wrote %s" % png)


def main(path):
    events = load_events(path)
    routes = split_routes(events)
    rows = [summarize_route(i + 1, r) for i, r in enumerate(routes)]
    out_dir = os.path.dirname(os.path.abspath(path))

    completed = [r for r in rows if r["completed"]]
    print("routes attempted : %d" % len(rows))
    print("routes completed : %d (%.0f%%)"
          % (len(completed), 100.0 * len(completed) / len(rows) if rows else 0.0))
    if completed:
        print("avg time, completed routes: %.1f s"
              % (sum(r["duration_s"] for r in completed) / len(completed)))

    # Failures worth a paragraph in the report -- list them, don't just count them.
    for name in ("stuck_entered", "stuck_timeout"):
        matching = [e for e in events if e["event"] == name]
        print("%-16s: %d" % (name, len(matching)))
        for e in matching:
            print("    %s  %s" % (e.get("iso", "?"), e.get("data", {})))
    print("avoidance_start : %d"
          % sum(1 for e in events if e["event"] == "avoidance_start"))

    if rows:
        cols = ["route", "started", "planned", "n_planned", "n_executed",
                "duration_s", "completed", "stuck", "stuck_timeout"]
        widths = {c: max(len(c), max(len(str(r[c])) for r in rows)) for c in cols}
        print("\n  " + "  ".join(c.ljust(widths[c]) for c in cols))
        for r in rows:
            print("  " + "  ".join(str(r[c]).ljust(widths[c]) for c in cols))
        csv_path = os.path.join(out_dir, "routes.csv")
        with open(csv_path, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=cols)
            w.writeheader()
            w.writerows(rows)
        print("\nwrote %s" % csv_path)

    save_chart(events, out_dir)


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("usage: python3 analyze_runs.py /path/to/run_log.jsonl")
        sys.exit(1)
    main(sys.argv[1])
