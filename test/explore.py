"""
Autonomous exploration — map a whole house without being driven.

Library, like slam.py. web_nav.py runs it; it is not launched directly.

Why frontier-based
------------------
A frontier is a cell that is known-free and touches unknown space. That is
precisely what an open doorway looks like from inside a room: the near side
is swept floor, beyond it is unknown. So "find the doors and go through them"
is not a special case here — it is the only thing the algorithm does. It
finishes when no frontiers remain, which means the reachable space is mapped.

Chairs, benches and table legs need no special handling either. They appear
as occupied cells, obstacle inflation keeps the planned path a robot-width
away from them, and the footprint guard in web_nav.py is the last line of
defence if the plan is stale.

Why A* and not "drive at the frontier"
--------------------------------------
Steering straight at a target works in an open room and fails at exactly the
place that matters. A doorway is usually NOT on the bearing to the frontier
behind it — the robot has to travel away from the goal, line up with the
opening, then pass through. Greedy bearing-following pins itself on the wall
beside the door instead. A* over the occupancy grid handles that for free.

Cost control
------------
SLAM already costs ~90-130 ms per update on the Pi. So the planner works on a
grid downsampled 2x (100 mm cells), replans on a timer rather than every
step, and caps expanded nodes. A stale-but-cheap plan plus a reactive guard
beats a perfect plan that arrives too late.
"""

import heapq
import math
import threading
import time

import numpy as np

# Occupancy log-odds thresholds. Deliberately asymmetric: a cell must be
# clearly free before the planner will drive through it, but only mildly
# suspect to be treated as an obstacle.
FREE_BELOW = -1.0
OCC_ABOVE = 0.6

DOWNSAMPLE = 2                  # planner cells per grid cell
MAX_NODES = 20000               # A* expansion cap, keeps a replan bounded
REPLAN_EVERY_S = 3.0
MIN_FRONTIER_CELLS = 4          # smaller clusters are sensor noise, not doors


class Frontier:
    __slots__ = ("cx", "cy", "size", "dist")

    def __init__(self, cx, cy, size, dist):
        self.cx, self.cy, self.size, self.dist = cx, cy, size, dist


def _coarse(grid):
    """Downsample the log-odds grid to a planner grid of -1/0/1 cells.

    A coarse cell is an obstacle if ANY fine cell in it is occupied — the
    pessimistic choice, because a table leg that vanishes at low resolution
    is a table leg the robot drives into.
    """
    n, d = grid.n, DOWNSAMPLE
    m = n // d
    # NumPy: the Python loop this replaces was ~1 s a replan on the 30 m grid.
    g = np.frombuffer(grid.grid, dtype=np.float32).reshape(n, n)[:m * d, :m * d]
    blocks = g.reshape(m, d, m, d)
    occ = (blocks > OCC_ABOVE).any(axis=(1, 3))
    free = (blocks < FREE_BELOW).any(axis=(1, 3))
    out = np.where(occ, 2, np.where(free, 1, 0)).astype(np.uint8)
    return bytearray(out.tobytes()), m              # 0 unknown, 1 free, 2 occupied


def _inflate(cells, m, radius_cells):
    """Grow obstacles by the robot's radius so a path never clips a corner.

    Planning for a point robot and hoping the guard catches the rest does not
    work: the guard can only refuse motion, it cannot re-route, so the robot
    ends up nose-to-the-wall in a doorway it was never going to fit through.
    """
    if radius_cells <= 0:
        return cells
    a = np.frombuffer(bytes(cells), dtype=np.uint8).reshape(m, m)
    occ = a == 2
    grown = occ.copy()
    r = int(radius_cells)
    r2 = radius_cells * radius_cells
    # Dilate by a disc: OR in the obstacle mask shifted by every offset in it.
    for dy in range(-r, r + 1):
        for dx in range(-r, r + 1):
            if (dx or dy) and dx * dx + dy * dy <= r2:
                grown[max(0, dy):m + min(0, dy), max(0, dx):m + min(0, dx)] |= \
                    occ[max(0, -dy):m + min(0, -dy), max(0, -dx):m + min(0, -dx)]
    out = a.copy()
    out[grown] = 2
    return bytearray(out.tobytes())


# Planning clearance. HARD: grown by half the truck's width plus the guard's
# side margin, so the planner never routes where the guard refuses to drive
# straight. SOFT: cells within turning reach of anything cost more, up to
# (1 + CLEAR_WEIGHT) times, so a route keeps room to turn when it can and
# still squeezes through a doorway when that is the only way.
GUARD_SIDE_MM = 15.0         # SIDE_MARGIN_MM in web_nav.py's guard
UNKNOWN_COST = 4.0           # step cost through unmapped cells, x known-free
# Frontiers nearer than this to the truck's centre are the unmapped floor
# UNDER its own chassis (the scanner cannot see there). Live, the explorer
# picked one 0.1 m away first, "arrived" at once, picked it again, and sat
# still until the stuck watchdog moved it.
FRONTIER_MIN_MM = 400.0
TURN_ROOM_MM = 175.0         # beyond the corner radius: guard pad + slack
CLEAR_WEIGHT = 8.0           # tuned in simulation: 66 mm+ from every wall, no guard stops


def plan_costs(cells, m, geom, cell_mm):
    """(inflated cells, per-cell penalty list) for this truck."""
    hl, hw = geom["len"] / 2.0, geom["wid"] / 2.0
    infl = _inflate(cells, m, (hw + GUARD_SIDE_MM) / cell_mm)
    soft = (math.hypot(hl, hw) + TURN_ROOM_MM) / cell_mm
    a = np.frombuffer(bytes(cells), dtype=np.uint8).reshape(m, m)
    occ = a == 2
    reach = int(math.ceil(soft))
    dist = np.full((m, m), reach + 1, dtype=np.float32)
    dist[occ] = 0
    grown = occ.copy()
    # Chebyshev rings outward from every obstacle, one cell per pass.
    for k in range(1, reach + 1):
        g = grown.copy()
        g[1:, :] |= grown[:-1, :]
        g[:-1, :] |= grown[1:, :]
        g[:, 1:] |= grown[:, :-1]
        g[:, :-1] |= grown[:, 1:]
        g[1:, 1:] |= grown[:-1, :-1]
        g[:-1, :-1] |= grown[1:, 1:]
        g[1:, :-1] |= grown[:-1, 1:]
        g[:-1, 1:] |= grown[1:, :-1]
        dist[g & ~grown] = k
        grown = g
    pen = np.clip(1.0 - dist / soft, 0.0, 1.0) * CLEAR_WEIGHT
    return infl, pen.ravel().tolist()


def _cell_mm(c, cell, half):
    """Planner cell -> world mm at the cell's CENTRE. Waypoints used to be
    the cell's corner, which shifted every route 50 mm toward -x/-y and
    hugged the walls on that side."""
    return (c[0] - half + 0.5) * cell, (c[1] - half + 0.5) * cell


def find_frontiers(grid, pose_cell, m, cells):
    """Free cells touching unknown space, clustered, nearest first."""
    seen = bytearray(m * m)
    out = []
    px, py = pose_cell
    # Candidate seeds found with NumPy — free, interior, touching unknown —
    # so the Python loop below only visits the few hundred frontier cells,
    # not all 90k.
    a = np.frombuffer(bytes(cells), dtype=np.uint8).reshape(m, m)
    unk = a == 0
    edge = np.zeros_like(unk)
    edge[1:-1, 1:-1] = (a[1:-1, 1:-1] == 1) & (unk[1:-1, :-2] | unk[1:-1, 2:]
                                               | unk[:-2, 1:-1] | unk[2:, 1:-1])
    is_edge = edge.ravel()
    for i in np.flatnonzero(is_edge):
        i = int(i)
        if seen[i]:
            continue
        cy, cx = divmod(i, m)
        # flood fill this frontier blob, through 8-neighbours that are edges too
        stack = [(cx, cy)]
        seen[i] = 1
        blob = []
        while stack:
            x, y = stack.pop()
            blob.append((x, y))
            for nx, ny in ((x-1, y), (x+1, y), (x, y-1), (x, y+1),
                           (x-1, y-1), (x+1, y-1), (x-1, y+1), (x+1, y+1)):
                j = ny * m + nx
                if 0 <= j < m * m and is_edge[j] and not seen[j]:
                    seen[j] = 1
                    stack.append((nx, ny))
        if len(blob) >= MIN_FRONTIER_CELLS:
            ax = sum(b[0] for b in blob) / len(blob)
            ay = sum(b[1] for b in blob) / len(blob)
            out.append(Frontier(int(ax), int(ay), len(blob),
                                math.hypot(ax - px, ay - py)))
    out.sort(key=lambda f: f.dist)
    return out


def nearest_open(cells, m, goal, radius=8):
    """Closest passable cell to a goal that inflation has sealed off.

    Frontiers live at the edge of known space, which is usually right beside
    a wall — so the frontier cell itself is almost always inside the inflated
    obstacle. Planning straight to it therefore fails EVERY time, which shows
    up as "frontiers left but none reachable" while the robot sits in an open
    room. Aim at the nearest cell it can actually occupy instead.
    """
    gx, gy = goal
    if 0 <= gx < m and 0 <= gy < m and cells[gy * m + gx] != 2:
        return goal
    best = None
    for r in range(1, radius + 1):
        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                if max(abs(dx), abs(dy)) != r:
                    continue
                nx, ny = gx + dx, gy + dy
                if 0 <= nx < m and 0 <= ny < m and cells[ny * m + nx] != 2:
                    d = dx * dx + dy * dy
                    if best is None or d < best[0]:
                        best = (d, (nx, ny))
        if best:
            return best[1]
    return None


def astar(cells, m, start, goal, penalty=None):
    """8-connected A* over the coarse grid. Returns a cell path or None.

    Unknown cells are traversable — the whole point is to drive into unknown
    space — but they carry an extra cost so a known-free detour is preferred
    when one exists. `penalty` (per cell, >= 0) makes cells near obstacles
    dearer, so routes run down the middle of open space.
    """
    sx, sy = start
    gx, gy = goal
    if not (0 <= sx < m and 0 <= sy < m):
        return None
    if cells[gy * m + gx] == 2:
        return None

    h = lambda x, y: math.hypot(x - gx, y - gy)
    openq = [(h(sx, sy), 0.0, sx, sy)]
    came = {}
    best = {(sx, sy): 0.0}
    nodes = 0
    while openq:
        _, g0, x, y = heapq.heappop(openq)
        if (x, y) == (gx, gy):
            path = [(x, y)]
            while (x, y) in came:
                x, y = came[(x, y)]
                path.append((x, y))
            return path[::-1]
        nodes += 1
        if nodes > MAX_NODES:
            return None
        if g0 > best.get((x, y), 1e18):
            continue
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                nx, ny = x + dx, y + dy
                if not (0 <= nx < m and 0 <= ny < m):
                    continue
                c = cells[ny * m + nx]
                if c == 2:
                    continue
                step = 1.414 if dx and dy else 1.0
                if c == 0:
                    # Unknown may be floor or the far side of a wall. At 1.8
                    # the live run planned a 2 m loop through unmapped space
                    # round a room instead of going straight in.
                    step *= UNKNOWN_COST
                if penalty is not None:
                    step *= 1.0 + penalty[ny * m + nx]
                ng = g0 + step
                if ng < best.get((nx, ny), 1e18):
                    best[(nx, ny)] = ng
                    came[(nx, ny)] = (x, y)
                    heapq.heappush(openq, (ng + h(nx, ny), ng, nx, ny))
    return None


class Calibration:
    """Runs before exploring, because the constants it checks decide whether
    the map means anything.

    gyro     3 s stationary. A gyro reads a small non-zero rate when still,
             and integrating that is what makes heading crawl.
    compass  a slow full turn. The BNO055 self-calibrates from motion, and
             the same turn cross-checks the IMU's yaw against the rotation
             the wheels claim — which validates TRACK_WIDTH_MM.
    scale    a short straight run at a wall. The LiDAR measures how far the
             wall actually moved; odometry says how far it thinks it went.
             Disagreement means COUNTS_PER_REV or WHEEL_DIAM_MM is wrong,
             and this is the only way to catch that without a tape measure.
    """

    def __init__(self):
        self.stage = "idle"
        self.results = {}
        self.notes = []

    def reset(self):
        self.stage = "idle"
        self.results = {}
        self.notes = []


class Explorer:
    """Calibrate, then map everything reachable, then stop.

    Runs in its own thread and issues a drive command every tick — which also
    feeds the 0.6 s watchdog, so a stall in this loop stops the robot rather
    than leaving it running.
    """

    TICK = 0.1
    CRUISE = 1.0                 # throttle; the speed limit caps actual duty
    # Turn on the spot above SPIN_ENTER degrees off, back to driving below
    # SPIN_EXIT. The gap is what stops it flipping between the two - the old
    # single 25-degree threshold, at full spin power, overshot and zig-zagged
    # (seen live: 107, 136, 149, 153, 121, 118, 124 degrees on a straight run).
    SPIN_ENTER = 35.0
    SPIN_EXIT = 10.0
    SPIN_MIN = 0.6               # a skid-steer will not rotate on much less
    # Aim this far along the path in open space, down to LOOKAHEAD_MIN_MM
    # where the costmap says it is tight. A long lookahead cuts corners: out
    # of a doorway it started the turn with the tail still beside the jamb
    # and passed it at 13 mm (simulated). Short in doorways, smooth in rooms.
    LOOKAHEAD_MM = 450.0
    LOOKAHEAD_MIN_MM = 150.0
    GOAL_REACHED_MM = 250.0
    GUARD_REPLAN_S = 0.8         # guard blocking this long = the map was wrong
    # A frontier only has to be SEEN, not stood on: this close counts.
    EXPLORE_REACH_MM = 700.0
    LOOK_MAX_S = 25.0            # longest "look around" before deciding
    # Make room to turn: when a turn on the spot is refused (a corner would
    # swing into something), drive straight a little - forward, or back if
    # forward is blocked too - then try the turn again. A three-point turn.
    # Live, 19 of 34 guard stops were refused turns, and the truck spent five
    # minutes wedged in the front-door nook where only forward was free.
    ROOM_S = 0.8
    ROOM_SPEED = 0.6
    ROOM_EVERY_S = 1.5
    # The guard stops the truck stop_mm short of anything ahead, but the
    # planner puts goals ~160 mm from obstacles. Stopped by the guard this
    # close to the goal = as close as it can safely get. Without this it
    # looped - blocked, back off, replan the same goal - for 40 s and more.
    BLOCKED_ARRIVE_MM = 1000.0
    STUCK_S = 6.0

    def __init__(self, robot, lidar, slam_runner, guard, geom, drive_fn):
        self.robot, self.lidar = robot, lidar
        self.slamr, self.guard = slam_runner, guard
        self.geom = geom                     # the pins.py geometry, as a dict
        self.drive = drive_fn                # goes through the guard

        self.cal = Calibration()
        self.state = "idle"
        self.running = False
        self.message = ""
        self.path = []
        self.target = None
        self.frontiers = 0
        self.visited_fail = set()
        self._t0 = 0.0
        self._last_plan = 0.0
        self._last_progress = 0.0
        self._last_pose = (0.0, 0.0, 0.0)           # x, y, heading deg
        self._spin0 = None
        self._spin_counts = [0, 0]
        self._scale0 = None
        self._scale_d0 = 0.0
        self.goal_xy = None
        self.goal_label = ""
        self._gen = 0
        self._spinning = False
        self._blocked_since = None
        self._pen = None                 # clearance penalty of the last plan
        self._pen_m = 0
        # "Look around": degrees turned on the spot while there was nowhere
        # to go yet. See _explore_step.
        self._look_deg = 0.0
        self._look_prev = None
        self._looking = False
        self._look_t0 = None
        self._look_dir = 1.0
        self._look_flip_t = None
        self._room_until = 0.0
        self._room_moves = [(1.0, 0.0)]
        self._room_i = 0
        self._room_last = -1e9
        self._room_changed = -1e9

    # --- control ------------------------------------------------------------

    def goto(self, x, y, label=""):
        """Drive to one world coordinate and stop.

        Deliberately the SAME planner and path follower the explorer uses —
        "go to the kitchen" is a saved coordinate plus this. Building a second
        navigation stack for it would just be a second thing to get wrong.
        """
        if self.running:
            self.stop("superseded by a goto")
        self.goal_xy = (float(x), float(y))
        self.goal_label = label or ("%.0f,%.0f" % (x, y))
        self.running = True
        self.state = "goto"
        self.message = "going to " + self.goal_label
        self._t0 = time.time()
        self._last_progress = time.time()
        self._last_plan = 0.0
        self.path = []
        self._launch()

    def _launch(self):
        """One driving thread at a time. A new run bumps the generation; the
        old thread sees it on its next tick and leaves WITHOUT touching
        `running` or the motors, which now belong to the new run. Without
        this, clicking a new goal mid-drive left two threads steering, and an
        old run finishing could switch the new one off."""
        self._gen += 1
        threading.Thread(target=self._run, args=(self._gen,), daemon=True).start()

    def start(self, calibrate=True):
        if self.running:
            return
        self.running = True
        self.cal.reset()
        self.visited_fail.clear()
        self.path = []
        self.state = "cal_gyro" if calibrate else "explore"
        self.message = ""
        self._look_deg, self._look_prev, self._looking = 0.0, None, False
        self._t0 = time.time()
        self._last_progress = time.time()
        self._last_plan = 0.0
        self._launch()

    def stop(self, why="stopped by operator"):
        self.running = False
        self.message = why
        self.state = "idle"
        try:
            self.drive(0, 0)
        except Exception:                                      # noqa: BLE001
            pass

    # --- main loop ----------------------------------------------------------

    def _run(self, gen):
        try:
            while self.running and gen == self._gen:
                t = time.time()
                if self.state.startswith("cal_"):
                    self._calibrate_step(t)
                elif self.state == "explore":
                    self._explore_step(t)
                elif self.state == "goto":
                    self._goto_step(t)
                else:
                    break
                time.sleep(self.TICK)
        except Exception as e:                                 # noqa: BLE001
            if gen == self._gen:
                self.message = "explorer crashed: " + str(e)
                self.state = "failed"
        finally:
            if gen == self._gen:              # superseded: the new run owns these
                try:
                    self.drive(0, 0)
                except Exception:                              # noqa: BLE001
                    pass
                self.running = False

    # --- calibration --------------------------------------------------------

    def _calibrate_step(self, t):
        el = t - self._t0
        imu = self.slamr.imu.state

        if self.state == "cal_gyro":
            self.cal.stage = "gyro - holding still"
            self.drive(0, 0)
            if el < 3.0:
                return
            g = imu.get("gyro") or [0, 0, 0]
            self.cal.results["gyro_rest_dps"] = round(g[2], 3)
            if abs(g[2]) > 1.5:
                self.cal.notes.append(
                    "gyro reads %.2f dps at rest - heading will drift; run "
                    "imu_test.py --gyro-bias" % g[2])
            self._spin0 = imu.get("yaw")
            self._spin_counts = list(self.slamr.state["counts"])
            self.state = "cal_spin"
            self._t0 = t
            return

        if self.state == "cal_spin":
            self.cal.stage = "compass - slow full turn"
            self.drive(0, 1)
            if el < 14.0:
                return
            self.drive(0, 0)
            c1 = self.slamr.state["counts"]
            mmpc = math.pi * self.geom["wheel"] / self.geom["cpr"]
            dl = (c1[0] - self._spin_counts[0]) * mmpc
            dr = -(c1[1] - self._spin_counts[1]) * mmpc      # ENC_SIGN_RIGHT
            self.cal.results["spin_wheels_deg"] = round(
                math.degrees((dr - dl) / self.geom["track"]), 1)
            self.cal.results["imu_heading_ok"] = bool(imu.get("heading_ok"))
            if not imu.get("heading_ok"):
                self.cal.notes.append(
                    "no magnetometer fix after a full turn - heading is "
                    "gyro-only and will drift. Move the IMU away from the motors.")
            self._scale0 = self._front_mm()
            self._scale_d0 = self.slamr.slam.odom.distance
            self.state = "cal_scale"
            self._t0 = t
            return

        if self.state == "cal_scale":
            self.cal.stage = "odometry scale - short run at a wall"
            f = self._front_mm()
            moved = self.slamr.slam.odom.distance - self._scale_d0
            if el < 6.0 and moved < 400 and (f is None or f > 600):
                self.drive(1, 0)
                return
            self.drive(0, 0)
            f1 = self._front_mm()
            if self._scale0 and f1 and moved > 60:
                lidar_mm = self._scale0 - f1
                ratio = lidar_mm / moved
                self.cal.results["odom_mm"] = round(moved)
                self.cal.results["lidar_mm"] = round(lidar_mm)
                self.cal.results["scale_ratio"] = round(ratio, 2)
                if not 0.85 < ratio < 1.18:
                    self.cal.notes.append(
                        "odometry says %.0f mm but the LiDAR measured %.0f mm "
                        "(x%.2f) - COUNTS_PER_REV or WHEEL_DIAM_MM is wrong"
                        % (moved, lidar_mm, ratio))
            else:
                self.cal.notes.append(
                    "scale check inconclusive - it needs a flat wall 0.6-2 m "
                    "ahead when mapping starts")
            self.cal.stage = "done"
            self.state = "explore"
            self._t0 = t
            self._last_plan = 0.0
            return

    def _front_mm(self):
        v = [d for a, d in self.lidar.scan()
             if abs((a + 180) % 360 - 180) <= 12 and d > 120]
        return min(v) if v else None

    # --- exploring ----------------------------------------------------------

    def _explore_step(self, t):
        grid = self.slamr.slam.grid
        pose = self.slamr.slam.pose
        cell = grid.res * DOWNSAMPLE
        half = grid.half // DOWNSAMPLE
        px = int(pose.x // cell) + half
        py = int(pose.y // cell) + half

        # While looking around, replan once a second; otherwise as before.
        due = (t - self._last_plan > (1.0 if self._looking else REPLAN_EVERY_S)
               or (not self.path and not self._looking))
        if due:
            self._replan(grid, (px, py))
            self._last_plan = t
            if self.state != "explore":
                return
        if self._looking:
            # Nowhere to go YET: a truck that has not moved has mapped one
            # sparse scan (SLAM only adds scans after 40 mm or 4 degrees), and
            # the only frontier big enough to count was the floor under it.
            # Turn slowly on the spot until something turns up; only a full
            # turn with nothing found means the house is done.
            th = math.degrees(pose.th)
            if self._look_prev is not None:
                self._look_deg += abs(((th - self._look_prev + 180) % 360) - 180)
            self._look_prev = th
            if self._look_t0 is None:
                self._look_t0 = t
            # A turn the guard refuses (a wall at a corner) goes the other way;
            # and a look is over after LOOK_MAX_S whether or not it got all the
            # way round. Live, one wedged look never ended.
            if self._room_step(t):
                return
            g = self.guard
            if self._turn_refused() and self._make_room(t, True, self._look_dir < 0):
                return
            if g is not None and g.blocked and "turning" in (g.reason or ""):
                if self._look_flip_t is not None and t - self._look_flip_t < 1.5:
                    self._look_t0 = -1e9         # blocked BOTH ways: end the look
                else:
                    self._look_dir = -self._look_dir
                    self._look_flip_t = t
            if t - self._look_t0 > self.LOOK_MAX_S:
                self._look_deg = 360.0
                self._last_plan = 0.0            # decide now: explore on, or done
                return
            self.drive(0, self.SPIN_MIN * self._look_dir)
            return
        if not self.path:
            return

        if self._unstick(t, pose):
            return

        ex, ey = _cell_mm(self.path[-1], cell, half)
        left = math.hypot(ex - pose.x, ey - pose.y)
        if left < self.EXPLORE_REACH_MM or (self._guard_ahead() and left < self.BLOCKED_ARRIVE_MM):
            if left >= self.EXPLORE_REACH_MM and self.target is not None:
                # Stopped short by something: if it is still a frontier after
                # this, it is behind that something - do not come back for it.
                self.visited_fail.add(self.target)
            self.path = []
            self.target = None           # arrived; free to choose a new one
            self._last_plan = 0.0
            return
        self._follow(pose, cell, half, t)

    def _unstick(self, t, pose):
        """Shared progress watchdog.

        Returns True if it took over this tick. Exploring had this and goto
        did not, so a blocked goto sat with zero throttle forever — measured
        1800 simulation steps pinned against a wall while reporting a valid
        path. Any mode that can be blocked needs a way out, not just one.
        """
        turned = abs(((math.degrees(pose.th) - self._last_pose[2] + 180) % 360) - 180) \
            if len(self._last_pose) > 2 else 0.0
        # Turning is progress too: a long, legitimate turn on the spot was
        # counted as "stuck" live and made it back off for no reason.
        if math.hypot(pose.x - self._last_pose[0],
                      pose.y - self._last_pose[1]) > 60 or turned > 20.0:
            self._last_pose = (pose.x, pose.y, math.degrees(pose.th))
            self._last_progress = t
            self._unstick_dir = 1
            return False
        if t - self._last_progress <= self.STUCK_S:
            return False

        # Alternate reversing and turning: a robot wedged nose-first needs to
        # back out, one wedged on a corner needs to rotate off it, and we
        # cannot tell which from here.
        self._unstick_dir = -getattr(self, "_unstick_dir", 1)
        if self._unstick_dir > 0:
            self.drive(-1, 0)
        else:
            self.drive(0, 1)
        self.path = []
        self._last_plan = 0.0
        self.message = "stuck - backing off and replanning"
        if t - self._last_progress > self.STUCK_S * 3:
            self._last_progress = t
            if self.target:
                self.visited_fail.add(self.target)
                self.target = None
        return True

    def _goto_step(self, t):
        """Same follow-a-path loop as exploring, with a fixed destination."""
        grid = self.slamr.slam.grid
        pose = self.slamr.slam.pose
        cell = grid.res * DOWNSAMPLE
        half = grid.half // DOWNSAMPLE

        gx, gy = self.goal_xy
        left = math.hypot(gx - pose.x, gy - pose.y)
        if left < self.GOAL_REACHED_MM:
            self.state = "done"
            self.running = False
            self.message = "arrived at " + self.goal_label
            return
        if self._guard_ahead() and left < self.BLOCKED_ARRIVE_MM:
            self.state = "done"
            self.running = False
            self.message = "arrived as close to %s as it can get (%.0f mm; something is in the way)" % (
                self.goal_label, left)
            return

        if self._unstick(t, pose):
            return

        if t - self._last_plan > REPLAN_EVERY_S or not self.path:
            cells, m = _coarse(grid)
            infl, pen = plan_costs(cells, m, self.geom, cell)
            self._pen, self._pen_m = pen, m
            sx = int(pose.x // cell) + half
            sy = int(pose.y // cell) + half
            if 0 <= sx < m and 0 <= sy < m:
                infl[sy * m + sx] = 1
            goal = nearest_open(infl, m,
                                (int(gx // cell) + half, int(gy // cell) + half))
            # [] not None: status() takes len(self.path), and a None left by
            # one unroutable goal made every /state and /ai/status a 500 -
            # the cockpit and the voice assistant both - until the next trip.
            self.path = (astar(infl, m, (sx, sy), goal, pen) if goal else None) or []
            self._last_plan = t
            if not self.path:
                self.state = "failed"
                self.running = False
                self.message = "no route to " + self.goal_label
                return

        self._follow(pose, cell, half, t)

    def _turn_refused(self):
        g = self.guard
        return bool(g is not None and g.enabled and g.blocked
                    and "turning" in (g.reason or ""))

    def _straight_refused(self):
        g = self.guard
        return bool(g is not None and g.enabled and g.blocked
                    and ("ahead" in (g.reason or "") or "behind" in (g.reason or "")))

    def _make_room(self, t, prefer_forward=True, turn_left=True):
        """Start a short move to get room to turn. True if started.

        A ladder of moves, each tried until the guard lets one through:
          1. forward pivot towards the side it wants to face
          2. reverse pivot, same side
          3. forward pivot the other way
          4. reverse pivot the other way
          5. spin the other way (the long way round)
        A pivot (inner wheels stopped) swings the tail AWAY from a wall
        alongside, where a spin on the spot swings it in. Whatever part of a
        move is unsafe the guard drops, so a refused pivot can still gain
        room straight on. Wedged diagonally into a corner, only the other
        way round was free - the first version never tried it."""
        if t - self._room_last < self.ROOM_EVERY_S:
            return False
        s = -1.0 if turn_left else 1.0                          # +steer = right
        first = 1.0 if prefer_forward else -1.0
        self._room_moves = [(first, s), (-first, s), (first, -s), (-first, -s), (0.0, -s)]
        self._room_i = 0
        self._room_until = t + self.ROOM_S
        self._room_last = self._room_changed = t
        self._room_drive()
        return True

    def _room_drive(self):
        d, s = self._room_moves[self._room_i]
        if d:
            self.drive(self.ROOM_SPEED * d, self.ROOM_SPEED * s)
        else:
            self.drive(0, self.SPIN_MIN * s)

    def _room_step(self, t):
        """Continue a make-room move. True while it is in charge."""
        if t >= self._room_until:
            return False
        g = self.guard
        stopped = g is not None and g.enabled and tuple(getattr(g, "out", (1, 1))) == (0.0, 0.0)
        # Refused outright (not merely the turn dropped): next move on the
        # ladder. Judged on what the guard let through, not its reason text -
        # a pivot refused by a wall 90 mm ahead says "arc would hit".
        if stopped and t - self._room_changed > 0.25:
            self._room_i += 1
            if self._room_i >= len(self._room_moves):
                self._room_until = 0.0           # nothing moves: leave it to unstick
                return False
            self._room_until = t + self.ROOM_S
            self._room_changed = t
        self._room_drive()
        return True

    def _guard_ahead(self):
        """The guard is refusing to drive on because of something in front
        (not an arc or a turn - those it can work round)."""
        g = self.guard
        return bool(g is not None and g.enabled and g.blocked and not g.creeping
                    and "ahead" in (g.reason or ""))

    def _follow(self, pose, cell, half, t):
        """Aim at a point a lookahead along the path and steer to it,
        smoothly: steering and speed scale with how far off the heading is,
        and spins slow down as they line up."""
        # The guard stopping us for a while means the map missed something
        # (a chair moved in, a leg the scan had not caught). Plan again now
        # rather than pushing into it until the next scheduled replan.
        g = self.guard
        if g is not None and g.enabled and g.blocked and not g.creeping:
            if self._blocked_since is None:
                self._blocked_since = t
            elif t - self._blocked_since > self.GUARD_REPLAN_S:
                self._last_plan = 0.0
                self._blocked_since = None
        else:
            self._blocked_since = None

        # Search FORWARD from the path point nearest the truck. Searching from
        # the path's start picked the first point 450 mm away - and once the
        # truck had driven 450 mm along a path that is only replanned every
        # 3 s, that was the START, behind it. It spun round to chase where it
        # had been, then back: a 160-degree spin every few seconds, and the
        # old navigator's "stuck" failures.
        pts = [_cell_mm(c, cell, half) for c in self.path]
        near = min(range(len(pts)),
                   key=lambda i: (pts[i][0] - pose.x) ** 2 + (pts[i][1] - pose.y) ** 2)
        look = self.LOOKAHEAD_MM
        m = self._pen_m
        if self._pen is not None:
            cx, cy = int(pose.x // cell) + half, int(pose.y // cell) + half
            if 0 <= cx < m and 0 <= cy < m:
                tight = min(1.0, self._pen[cy * m + cx] / CLEAR_WEIGHT)
                look = self.LOOKAHEAD_MM - tight * (self.LOOKAHEAD_MM - self.LOOKAHEAD_MIN_MM)
        tgt = pts[-1]
        for wx, wy in pts[near:]:
            if math.hypot(wx - pose.x, wy - pose.y) >= look:
                tgt = (wx, wy)
                break
        bearing = math.degrees(math.atan2(tgt[1] - pose.y, tgt[0] - pose.x))
        err = ((bearing - math.degrees(pose.th) + 180) % 360) - 180   # + = left

        if self._spinning:
            if abs(err) < self.SPIN_EXIT:
                self._spinning = False
        elif abs(err) > self.SPIN_ENTER:
            self._spinning = True
        elif (g is not None and g.blocked and "arc" in (g.reason or "")
              and abs(err) > self.SPIN_EXIT):
            # The guard refused the curve (something beside the path): line
            # up on the spot, then go straight - instead of asking for the
            # same refused arc again. 13 arc refusals in the first live run.
            self._spinning = True
        if self._room_step(t):
            return
        if self._spinning and self._turn_refused() and self._make_room(t, abs(err) < 150, err > 0):
            return
        if self._spinning:
            mag = min(1.0, max(self.SPIN_MIN, abs(err) / 90.0))
            self.drive(0, -mag if err > 0 else mag)      # +steer turns right
        else:
            speed = self.CRUISE * max(0.35, math.cos(math.radians(err)))
            self.drive(speed, max(-0.6, min(0.6, -err / 40.0)))

    def _replan(self, grid, start):
        """Pick a frontier and path to it.

        COMMITMENT is the point of the bookkeeping here. Choosing the nearest
        frontier every replan sounds sensible and is not: as the robot moves,
        which frontier is nearest flips between two candidates, so it drives
        toward A, switches to B, switches back, and makes no net progress.
        Measured in simulation: 28 m driven for 450 mm of actual progress.

        So an existing target is kept while it is still a frontier and still
        reachable. It is only abandoned when reached, when it stops being a
        frontier (someone else mapped it), when the path dies, or when the
        stuck watchdog gives up on it.
        """
        cells, m = _coarse(grid)
        # Inflate by half the WIDTH, not the circumscribed radius.
        #
        # Circumscribed (250 mm here) is the radius the robot sweeps when it
        # spins, and using it for planning demands gaps of 2x that — 660 mm —
        # so it refuses to plan through any normal doorway. Half-width plus a
        # little is what the robot needs to PASS through a gap, and the
        # footprint guard is still there to refuse the move if the plan turns
        # out optimistic.
        infl, pen = plan_costs(cells, m, self.geom, grid.res * DOWNSAMPLE)
        self._pen, self._pen_m = pen, m
        # Inflation can swallow the cell the robot is standing in; if it does,
        # every plan fails from step one.
        if 0 <= start[0] < m and 0 <= start[1] < m:
            infl[start[1] * m + start[0]] = 1

        fr = [f for f in find_frontiers(grid, start, m, cells)
              if f.dist * cellsize(grid) >= FRONTIER_MIN_MM]
        self.frontiers = len(fr)

        # Stay on the current target if it is still worth going to.
        if self.target is not None:
            still = [f for f in fr if (f.cx // 2, f.cy // 2) == self.target]
            if still:
                f = still[0]
                goal = nearest_open(infl, m, (f.cx, f.cy))
                if goal:
                    path = astar(infl, m, start, goal, pen)
                    if path and len(path) > 1:
                        self.path = path
                        self.message = "continuing to a frontier %.1f m away" % (
                            f.dist * cellsize(grid) / 1000.0)
                        self._looking = False
                        return
            self.target = None          # gone, or no longer reachable

        for f in fr:
            key = (f.cx // 2, f.cy // 2)
            if key in self.visited_fail:
                continue
            goal = nearest_open(infl, m, (f.cx, f.cy))
            if goal is None:
                continue
            path = astar(infl, m, start, goal, pen)
            if path and len(path) > 1:
                self.path = path
                self.target = key
                self.message = "heading for a frontier %.1f m away (%d cells)" % (
                    f.dist * cellsize(grid) / 1000.0, f.size)
                self._looking, self._look_deg, self._look_prev = False, 0.0, None
                return
        self.path = []
        if self._look_deg < 360.0:
            if not self._looking:
                self._look_t0, self._look_flip_t = None, None
            self._looking = True
            self.message = "looking around for somewhere to explore (%.0f of 360 degrees)" % self._look_deg
            return
        self._looking = False
        self.state = "done"
        self.running = False
        self.message = ("no frontiers left - reachable space is mapped"
                        if not fr else
                        "%d frontiers left but none reachable" % len(fr))

    @property
    def status(self):
        return {
            "running": self.running,
            "state": self.state,
            "message": self.message,
            "cal_stage": self.cal.stage,
            "cal_results": self.cal.results,
            "cal_notes": self.cal.notes,
            "frontiers": self.frontiers,
            "path_len": len(self.path or []),
            "goal": self.goal_label if self.state == "goto" else "",
            "goal_xy": ([round(v) for v in self.goal_xy]
                        if self.state == "goto" and self.goal_xy else None),
            "path_mm": self._path_mm(),
            "elapsed": round(time.time() - self._t0, 1) if self.running else 0,
        }


    def _path_mm(self):
        """The planned route in world mm, every other cell, for the map."""
        path = self.path if self.running else None
        if not path:
            return []
        grid = self.slamr.slam.grid
        cell, half = cellsize(grid), grid.half // DOWNSAMPLE
        pts = path[::2] + ([path[-1]] if len(path) % 2 == 0 else [])
        return [list(_cell_mm(c, cell, half)) for c in pts]


def cellsize(grid):
    return grid.res * DOWNSAMPLE
