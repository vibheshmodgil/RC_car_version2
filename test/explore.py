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
import time

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
    n = grid.n
    m = n // DOWNSAMPLE
    out = bytearray(m * m)                  # 0 unknown, 1 free, 2 occupied
    g = grid.grid
    for cy in range(m):
        base = cy * DOWNSAMPLE * n
        row = cy * m
        for cx in range(m):
            occ = False
            free = False
            for dy in range(DOWNSAMPLE):
                i = base + dy * n + cx * DOWNSAMPLE
                for dx in range(DOWNSAMPLE):
                    v = g[i + dx]
                    if v > OCC_ABOVE:
                        occ = True
                    elif v < FREE_BELOW:
                        free = True
            out[row + cx] = 2 if occ else (1 if free else 0)
    return out, m


def _inflate(cells, m, radius_cells):
    """Grow obstacles by the robot's radius so a path never clips a corner.

    Planning for a point robot and hoping the guard catches the rest does not
    work: the guard can only refuse motion, it cannot re-route, so the robot
    ends up nose-to-the-wall in a doorway it was never going to fit through.
    """
    if radius_cells <= 0:
        return cells
    out = bytearray(cells)
    r = int(radius_cells)
    r2 = radius_cells * radius_cells
    for cy in range(m):
        for cx in range(m):
            if cells[cy * m + cx] != 2:
                continue
            for dy in range(-r, r + 1):
                yy = cy + dy
                if not (0 <= yy < m):
                    continue
                for dx in range(-r, r + 1):
                    xx = cx + dx
                    if 0 <= xx < m and dx * dx + dy * dy <= r2:
                        out[yy * m + xx] = 2
    return out


def find_frontiers(grid, pose_cell, m, cells):
    """Free cells touching unknown space, clustered, nearest first."""
    seen = bytearray(m * m)
    out = []
    px, py = pose_cell
    for cy in range(1, m - 1):
        for cx in range(1, m - 1):
            i = cy * m + cx
            if cells[i] != 1 or seen[i]:
                continue
            if not (cells[i - 1] == 0 or cells[i + 1] == 0
                    or cells[i - m] == 0 or cells[i + m] == 0):
                continue
            # flood fill this frontier blob
            stack = [(cx, cy)]
            seen[i] = 1
            blob = []
            while stack:
                x, y = stack.pop()
                blob.append((x, y))
                for nx, ny in ((x-1, y), (x+1, y), (x, y-1), (x, y+1),
                               (x-1, y-1), (x+1, y-1), (x-1, y+1), (x+1, y+1)):
                    if not (0 < nx < m - 1 and 0 < ny < m - 1):
                        continue
                    j = ny * m + nx
                    if seen[j] or cells[j] != 1:
                        continue
                    if (cells[j-1] == 0 or cells[j+1] == 0
                            or cells[j-m] == 0 or cells[j+m] == 0):
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


def astar(cells, m, start, goal):
    """8-connected A* over the coarse grid. Returns a cell path or None.

    Unknown cells are traversable — the whole point is to drive into unknown
    space — but they carry an extra cost so a known-free detour is preferred
    when one exists.
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
                    step *= 1.8              # prefer known-free where possible
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
    TURN_TOLERANCE = 25.0        # deg of bearing error before turning in place
    LOOKAHEAD_MM = 450.0
    GOAL_REACHED_MM = 350.0
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
        self._last_pose = (0.0, 0.0)
        self._spin0 = None
        self._spin_counts = [0, 0]
        self._scale0 = None
        self._scale_d0 = 0.0
        self.goal_xy = None
        self.goal_label = ""

    # --- control ------------------------------------------------------------

    def goto(self, x, y, label=""):
        """Drive to one world coordinate and stop.

        Deliberately the SAME planner and path follower the explorer uses —
        "go to the kitchen" is a saved coordinate plus this. Building a second
        navigation stack for it would just be a second thing to get wrong.
        """
        if self.running:
            self.stop("superseded by a goto")
        import threading
        self.goal_xy = (float(x), float(y))
        self.goal_label = label or ("%.0f,%.0f" % (x, y))
        self.running = True
        self.state = "goto"
        self.message = "going to " + self.goal_label
        self._t0 = time.time()
        self._last_progress = time.time()
        self._last_plan = 0.0
        self.path = []
        threading.Thread(target=self._run, daemon=True).start()

    def start(self, calibrate=True):
        if self.running:
            return
        import threading
        self.running = True
        self.cal.reset()
        self.visited_fail.clear()
        self.path = []
        self.state = "cal_gyro" if calibrate else "explore"
        self.message = ""
        self._t0 = time.time()
        self._last_progress = time.time()
        self._last_plan = 0.0
        threading.Thread(target=self._run, daemon=True).start()

    def stop(self, why="stopped by operator"):
        self.running = False
        self.message = why
        self.state = "idle"
        try:
            self.drive(0, 0)
        except Exception:                                      # noqa: BLE001
            pass

    # --- main loop ----------------------------------------------------------

    def _run(self):
        try:
            while self.running:
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
            self.message = "explorer crashed: " + str(e)
            self.state = "failed"
        finally:
            try:
                self.drive(0, 0)
            except Exception:                                  # noqa: BLE001
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

        if t - self._last_plan > REPLAN_EVERY_S or not self.path:
            self._replan(grid, (px, py))
            self._last_plan = t
            if self.state != "explore":
                return
        if not self.path:
            return

        if self._unstick(t, pose):
            return

        tgt = None
        for cx, cy in self.path:
            wx, wy = (cx - half) * cell, (cy - half) * cell
            if math.hypot(wx - pose.x, wy - pose.y) >= self.LOOKAHEAD_MM:
                tgt = (wx, wy)
                break
        if tgt is None:
            cx, cy = self.path[-1]
            tgt = ((cx - half) * cell, (cy - half) * cell)
            if math.hypot(tgt[0] - pose.x, tgt[1] - pose.y) < self.GOAL_REACHED_MM:
                self.path = []
                self.target = None       # arrived; free to choose a new one
                self._last_plan = 0.0
                return

        bearing = math.degrees(math.atan2(tgt[1] - pose.y, tgt[0] - pose.x))
        err = ((bearing - math.degrees(pose.th) + 180) % 360) - 180
        if abs(err) > self.TURN_TOLERANCE:
            self.drive(0, -1 if err > 0 else 1)     # +steer turns right
        else:
            self.drive(self.CRUISE, max(-0.6, min(0.6, -err / 45.0)))

    def _unstick(self, t, pose):
        """Shared progress watchdog.

        Returns True if it took over this tick. Exploring had this and goto
        did not, so a blocked goto sat with zero throttle forever — measured
        1800 simulation steps pinned against a wall while reporting a valid
        path. Any mode that can be blocked needs a way out, not just one.
        """
        if math.hypot(pose.x - self._last_pose[0],
                      pose.y - self._last_pose[1]) > 60:
            self._last_pose = (pose.x, pose.y)
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
        if math.hypot(gx - pose.x, gy - pose.y) < self.GOAL_REACHED_MM:
            self.state = "done"
            self.running = False
            self.message = "arrived at " + self.goal_label
            return

        if self._unstick(t, pose):
            return

        if t - self._last_plan > REPLAN_EVERY_S or not self.path:
            cells, m = _coarse(grid)
            radius = (self.geom["wid"] / 2.0
                      + self.geom["margin"] * 0.5) / cell
            infl = _inflate(cells, m, radius)
            sx = int(pose.x // cell) + half
            sy = int(pose.y // cell) + half
            if 0 <= sx < m and 0 <= sy < m:
                infl[sy * m + sx] = 1
            goal = nearest_open(infl, m,
                                (int(gx // cell) + half, int(gy // cell) + half))
            self.path = astar(infl, m, (sx, sy), goal) if goal else None
            self._last_plan = t
            if not self.path:
                self.state = "failed"
                self.running = False
                self.message = "no route to " + self.goal_label
                return

        self._follow(pose, cell, half)

    def _follow(self, pose, cell, half):
        """Aim at a point a lookahead along the path and steer to it."""
        tgt = None
        for cx, cy in self.path:
            wx, wy = (cx - half) * cell, (cy - half) * cell
            if math.hypot(wx - pose.x, wy - pose.y) >= self.LOOKAHEAD_MM:
                tgt = (wx, wy)
                break
        if tgt is None:
            cx, cy = self.path[-1]
            tgt = ((cx - half) * cell, (cy - half) * cell)
        bearing = math.degrees(math.atan2(tgt[1] - pose.y, tgt[0] - pose.x))
        err = ((bearing - math.degrees(pose.th) + 180) % 360) - 180
        if abs(err) > self.TURN_TOLERANCE:
            self.drive(0, -1 if err > 0 else 1)      # +steer turns right
        else:
            self.drive(self.CRUISE, max(-0.6, min(0.6, -err / 45.0)))

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
        radius = (self.geom["wid"] / 2.0
                  + self.geom["margin"] * 0.5) / (grid.res * DOWNSAMPLE)
        infl = _inflate(cells, m, radius)
        # Inflation can swallow the cell the robot is standing in; if it does,
        # every plan fails from step one.
        if 0 <= start[0] < m and 0 <= start[1] < m:
            infl[start[1] * m + start[0]] = 1

        fr = find_frontiers(grid, start, m, cells)
        self.frontiers = len(fr)

        # Stay on the current target if it is still worth going to.
        if self.target is not None:
            still = [f for f in fr if (f.cx // 2, f.cy // 2) == self.target]
            if still:
                f = still[0]
                goal = nearest_open(infl, m, (f.cx, f.cy))
                if goal:
                    path = astar(infl, m, start, goal)
                    if path and len(path) > 1:
                        self.path = path
                        self.message = "continuing to a frontier %.1f m away" % (
                            f.dist * cellsize(grid) / 1000.0)
                        return
            self.target = None          # gone, or no longer reachable

        for f in fr:
            key = (f.cx // 2, f.cy // 2)
            if key in self.visited_fail:
                continue
            goal = nearest_open(infl, m, (f.cx, f.cy))
            if goal is None:
                continue
            path = astar(infl, m, start, goal)
            if path and len(path) > 1:
                self.path = path
                self.target = key
                self.message = "heading for a frontier %.1f m away (%d cells)" % (
                    f.dist * cellsize(grid) / 1000.0, f.size)
                return
        self.path = []
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
            "path_len": len(self.path),
            "goal": self.goal_label if self.state == "goto" else "",
            "elapsed": round(time.time() - self._t0, 1) if self.running else 0,
        }


def cellsize(grid):
    return grid.res * DOWNSAMPLE
