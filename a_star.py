#!/usr/bin/env python3
"""
a_star.py -- A* path planning over the Duckietown tile graph.

Takes a START tile + which way the bot is FACING, and an END tile, and produces the
ordered list of intersection commands the robot executes:  ["straight","left",...,"stop"]

That command vocabulary and the {"commands":[...]} payload are the contract already
defined by our teammate's duckiebot_server.py, so this drops straight into the app.

ATTRIBUTION
-----------
`tile_connections`, `DIRECTIONS` and `get_neighbors()` are our teammate's work from
map_graph_prototype.py in this team's repo (github.com/lyonglam/duckietown). They are
reproduced here so this file runs standalone. The A* search, the turn derivation and the
command generation are new.

THE KEY IDEA
------------
Plain A* over TILES gives you a route but not turns -- "go to (4,3)" is meaningless until
you know which way you're pointing when you arrive. So the search state is:

        (row, col, facing)

Then the turn at each step is just: compare the facing you came in with the facing you
leave with. Same = straight, one step clockwise = right, one step anticlockwise = left.

WHY ONLY INTERSECTIONS PRODUCE COMMANDS
---------------------------------------
Curves and straights are handled automatically by lane following -- the bot just drives
them. Only the 5 real intersections have red stop lines where a decision happens. So we
emit a command ONLY for tiles in INTERSECTIONS. Emitting one for a curve would desync the
whole plan, because the robot counts stop lines to know which command is next.

RUN IT
------
    python3 a_star.py                 # runs the built-in self-tests
    python3 a_star.py 6 0 N 0 5       # plan from tile (6,0) facing North to tile (0,5)
"""

import heapq
import sys

# --------------------------------------------------------------------------
# MAP DATA (teammate's -- see attribution above)
# --------------------------------------------------------------------------

DIRECTIONS = {
    "N": (-1, 0),
    "S": (1, 0),
    "E": (0, 1),
    "W": (0, -1),
}

tile_connections = {
    (0, 2): {"N": "E", "W": "S"},
    (0, 3): {"E": "E", "W": "W"},
    (0, 4): {"E": "E", "W": "W"},
    (0, 5): {"N": "W", "E": "S"},
    (1, 1): {"N": "E", "W": "S"},
    (1, 2): {"S": "W", "E": "N"},
    (1, 5): {"N": "N", "S": "S"},
    (2, 0): {"N": "E", "W": "S"},
    (2, 1): {"S": "W", "E": "N"},
    (2, 3): {"N": "E", "W": "S"},
    (2, 4): {"E": "E", "W": "W"},
    (2, 5): {"E": ["N", "S"], "S": ["S", "W"], "N": ["N", "W"]},
    (3, 0): {"N": "N", "S": "S"},
    (3, 3): {"N": "N", "S": "S"},
    (3, 5): {"N": "N", "S": "S"},
    (4, 0): {"S": ["S", "E"], "N": ["N", "E"], "W": ["N", "S"]},
    (4, 1): {"E": "E", "W": "W"},
    (4, 2): {"E": "E", "W": "W"},
    (4, 3): {"E": ["E", "N", "S"], "W": ["W", "N", "S"],
             "N": ["N", "E", "W"], "S": ["S", "E", "W"]},
    (4, 4): {"E": "E", "W": "W"},
    (4, 5): {"E": ["N", "S"], "S": ["S", "W"], "N": ["N", "W"]},
    (5, 0): {"N": "N", "S": "S"},
    (5, 3): {"N": "N", "S": "S"},
    (5, 5): {"N": "N", "S": "S"},
    (6, 0): {"S": "E", "W": "N"},
    (6, 1): {"E": "E", "W": "W"},
    (6, 2): {"E": "E", "W": "W"},
    (6, 3): {"S": ["E", "W"], "E": ["E", "N"], "W": ["W", "N"]},
    (6, 4): {"E": "E", "W": "W"},
    (6, 5): {"S": "W", "E": "N"},
}

# A tile is a real intersection (has a red stop line, needs a decision) exactly when at
# least one of its entries offers more than one way out. Derived, not hand-listed, so it
# stays correct if the map changes. Should be: (2,5) (4,0) (4,3) (4,5) (6,3)
INTERSECTIONS = {
    rc for rc, moves in tile_connections.items()
    if any(isinstance(v, list) and len(v) > 1 for v in moves.values())
}

# Clockwise order. Used to turn a (facing_in, facing_out) pair into left/right/straight.
CLOCKWISE = ["N", "E", "S", "W"]

# Prefer straighter routes. Every turn is a chance for the open-loop maneuver to drift,
# so given two equally long paths we take the one with fewer turns. Keep this SMALL
# (< 1.0) or it starts preferring genuinely longer routes.
TURN_PENALTY = 0.3


def get_neighbors(row, col, facing):
    """Teammate's function: valid next steps as (next_row, next_col, next_facing)."""
    if (row, col) not in tile_connections:
        return []
    moves = tile_connections[(row, col)].get(facing, [])
    if isinstance(moves, str):
        moves = [moves]
    neighbors = []
    for out_dir in moves:
        dr, dc = DIRECTIONS[out_dir]
        neighbors.append((row + dr, col + dc, out_dir))
    return neighbors


# --------------------------------------------------------------------------
# TURN DERIVATION
# --------------------------------------------------------------------------

def turn_between(facing_in, facing_out):
    """
    What maneuver takes you from facing_in to facing_out?
    Clockwise is N -> E -> S -> W, so +1 step clockwise is a RIGHT turn.
    """
    step = (CLOCKWISE.index(facing_out) - CLOCKWISE.index(facing_in)) % 4
    return {0: "straight", 1: "right", 2: "uturn", 3: "left"}[step]


# --------------------------------------------------------------------------
# A*
# --------------------------------------------------------------------------

def heuristic(row, col, goal):
    """
    Manhattan distance in tiles. Admissible: every move steps exactly one tile in one
    axis, so the true remaining cost can never be less than this.
    """
    return abs(row - goal[0]) + abs(col - goal[1])


def a_star(start_rc, start_facing, goal_rc):
    """
    Search over (row, col, facing) states.

    Returns the list of states from start to goal, e.g.
        [(6,0,'N'), (5,0,'N'), (4,0,'N'), ...]
    or None if the goal is unreachable. Reaching the goal TILE is enough -- we don't
    care which way the bot ends up facing.
    """
    if start_rc not in tile_connections:
        raise ValueError("start tile %s is not a drivable tile on this map" % (start_rc,))
    if goal_rc not in tile_connections:
        raise ValueError("goal tile %s is not a drivable tile on this map" % (goal_rc,))
    if start_facing not in DIRECTIONS:
        raise ValueError("facing must be one of N/E/S/W, got %r" % (start_facing,))
    # A tile only supports the headings you can actually arrive/sit with. A corner tile
    # like (6,0) has no 'N' entry, so "at (6,0) facing N" is not a real robot state.
    # Catch it here with a useful message instead of failing later as "no route".
    if start_facing not in tile_connections[start_rc]:
        raise ValueError(
            "facing %s is not valid at tile %s -- valid facings there are %s"
            % (start_facing, start_rc, sorted(tile_connections[start_rc].keys())))

    start = (start_rc[0], start_rc[1], start_facing)

    # (f_score, counter, state). The counter just breaks ties so heapq never has to
    # compare states against each other.
    counter = 0
    open_heap = [(heuristic(start[0], start[1], goal_rc), counter, start)]
    came_from = {}
    g_score = {start: 0.0}
    closed = set()

    while open_heap:
        _, _, current = heapq.heappop(open_heap)
        if current in closed:
            continue
        closed.add(current)

        row, col, facing = current
        if (row, col) == goal_rc:
            return _reconstruct(came_from, current)

        for nrow, ncol, nfacing in get_neighbors(row, col, facing):
            neighbor = (nrow, ncol, nfacing)
            if neighbor in closed:
                continue
            # One tile of travel, plus a penalty if this step changed direction.
            step_cost = 1.0
            if nfacing != facing:
                step_cost += TURN_PENALTY
            tentative = g_score[current] + step_cost
            if tentative < g_score.get(neighbor, float("inf")):
                came_from[neighbor] = current
                g_score[neighbor] = tentative
                counter += 1
                f = tentative + heuristic(nrow, ncol, goal_rc)
                heapq.heappush(open_heap, (f, counter, neighbor))

    return None


def _reconstruct(came_from, current):
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path


# --------------------------------------------------------------------------
# PATH -> COMMANDS
# --------------------------------------------------------------------------

def path_to_commands(path):
    """
    Turn a state path into the robot's command list.

    We emit ONE command per intersection tile the path passes through, in order. Curves
    and straights emit nothing -- lane following just drives them. The robot counts red
    stop lines, so an extra or missing command here desynchronizes everything.
    """
    commands = []
    for i, (row, col, facing) in enumerate(path):
        if (row, col) not in INTERSECTIONS:
            continue
        if i + 1 >= len(path):
            # Goal IS the intersection tile: we arrive and stop, no turn to make.
            break
        next_facing = path[i + 1][2]
        commands.append(turn_between(facing, next_facing))
    commands.append("stop")
    return commands


def plan(start_rc, start_facing, goal_rc):
    """
    Full pipeline: tiles in, commands out.
    Returns (commands, path). Raises ValueError if there's no route.
    """
    path = a_star(start_rc, start_facing, goal_rc)
    if path is None:
        raise ValueError("no route from %s facing %s to %s"
                         % (start_rc, start_facing, goal_rc))
    return path_to_commands(path), path


def describe(start_rc, start_facing, goal_rc):
    """Human-readable plan -- handy for debugging and for the report."""
    commands, path = plan(start_rc, start_facing, goal_rc)
    lines = ["route from %s facing %s to %s" % (start_rc, start_facing, goal_rc),
             "  tiles (%d): %s" % (len(path),
                                   " -> ".join("(%d,%d)%s" % s for s in path)),
             "  intersections hit: %s"
             % ([s[:2] for s in path if s[:2] in INTERSECTIONS] or "none"),
             "  commands: %s" % (commands,)]
    return "\n".join(lines)


# --------------------------------------------------------------------------
# SELF-TESTS -- run these before trusting a plan on the real robot
# --------------------------------------------------------------------------

def _self_test():
    ok = True

    def check(name, got, want):
        nonlocal ok
        if got == want:
            print("  PASS  %s" % name)
        else:
            ok = False
            print("  FAIL  %s\n        got  %s\n        want %s" % (name, got, want))

    print("turn derivation:")
    check("E->E is straight", turn_between("E", "E"), "straight")
    check("E->S is right", turn_between("E", "S"), "right")
    check("E->N is left", turn_between("E", "N"), "left")
    check("N->W is left", turn_between("N", "W"), "left")
    check("W->N is right", turn_between("W", "N"), "right")

    print("map sanity:")
    check("5 intersections found", len(INTERSECTIONS), 5)
    check("intersection set", INTERSECTIONS,
          {(2, 5), (4, 0), (4, 3), (4, 5), (6, 3)})

    print("planning:")
    # Trivial: already at the goal.
    cmds, _ = plan((3, 0), "N", (3, 0))
    check("start == goal gives just stop", cmds, ["stop"])

    # Every plan must end with exactly one stop, and contain no u-turns
    # (this map has no tile that allows reversing).
    tiles = sorted(tile_connections.keys())
    checked = 0
    for start in tiles:
        for facing in "NESW":
            if facing not in tile_connections[start]:
                continue
            for goal in tiles:
                try:
                    cmds, path = plan(start, facing, goal)
                except ValueError:
                    continue  # unreachable is legitimate, not a bug
                checked += 1
                assert cmds[-1] == "stop", (start, facing, goal, cmds)
                assert cmds.count("stop") == 1, (start, facing, goal, cmds)
                assert "uturn" not in cmds, (start, facing, goal, cmds)
                # every command must correspond to an intersection actually visited
                n_int = sum(1 for s in path[:-1] if s[:2] in INTERSECTIONS)
                assert len(cmds) - 1 <= n_int, (start, facing, goal, cmds)
    print("  PASS  %d routes planned, all well-formed" % checked)

    # Reachability report: which (start,facing) -> goal pairs are impossible? Worth
    # knowing so the UI can grey them out instead of the robot being handed a bad plan.
    total = unreachable = 0
    for start in tiles:
        for facing in tile_connections[start]:
            for goal in tiles:
                total += 1
                try:
                    plan(start, facing, goal)
                except ValueError:
                    unreachable += 1
    print("reachability: %d/%d start-facing-goal combinations routable (%d not)"
          % (total - unreachable, total, unreachable))

    print("\nexample plans:")
    for args in [((6, 0), "W", (0, 5)), ((0, 3), "W", (6, 1)), ((4, 0), "N", (4, 5))]:
        try:
            print(describe(*args))
        except ValueError as e:
            print("  %s" % e)

    print("\n%s" % ("ALL TESTS PASSED" if ok else "SOME TESTS FAILED"))
    return ok


if __name__ == "__main__":
    if len(sys.argv) == 6:
        r0, c0, facing, r1, c1 = sys.argv[1:]
        start, goal = (int(r0), int(c0)), (int(r1), int(c1))
        try:
            commands, _ = plan(start, facing.upper(), goal)
        except ValueError as e:
            print("cannot plan: %s" % e)
            sys.exit(1)
        print(describe(start, facing.upper(), goal))
        print("\nJSON payload for POST /navigate:")
        print('{"commands": %s}' % (str(commands).replace("'", '"'),))
    else:
        _self_test()
