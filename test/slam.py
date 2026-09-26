"""
SLAM — odometry, occupancy grid, and scan matching.

Shared library like pins.py and imu.py; not run directly. web_nav.py imports
it. Pure Python in the per-scan hot paths; NumPy (already on the Pi for
onnxruntime) only for whole-grid passes — display, save, load.

The three pieces, and why each exists
-------------------------------------
DiffOdometry   Encoder counts say how far each side rolled. Differential
               drive turns that into a pose. Heading comes from the IMU
               rather than the wheel difference, because slip corrupts a
               skid-steer's heading faster than anything else — and this
               chassis skids on every turn by design.

OccupancyGrid  Each LiDAR return is evidence twice over: the endpoint is
               probably occupied, and everything along the ray to it is
               probably empty. Accumulated in log-odds so repeated looks
               reinforce and one bad reading does not ruin a cell.

ScanMatcher    Odometry drifts, always. Before folding a scan into the map,
               nudge the pose over a small search window and keep whichever
               offset makes the scan agree best with the map already built.
               Odometry predicts, the scan corrects. That correction is what
               makes this SLAM rather than dead reckoning with a picture.

Scale
-----
Tuned for ~200 points per turn at 11 Hz, which is what this scanner gives.
That is sparse. It maps a room well; it will struggle in open space or a
featureless corridor, where scan matching has nothing to lock onto. Expect
room-scale, not warehouse-scale.

Coordinates
-----------
World frame is millimetres, x forward and y left from where the robot
started, theta counter-clockwise from +x. The LiDAR reports bearings that
increase CLOCKWISE from the robot's nose, so converting a return to the
robot frame is (d*cos(a), -d*sin(a)) — the sign on y is the easy thing to
get wrong and it mirrors the whole map.
"""

import base64
import math
import os
import threading
import time
from array import array

import numpy as np

# The map's side. The robot starts in the MIDDLE, so this is 15 m in every
# direction. It was 12 m (6 m each way) and a house ran off the edge of it:
# walls simply stopped being drawn half-way down the hall. Cheap now that the
# whole-grid passes are NumPy and only the explored part goes to the browser.
MAP_SIZE_MM = 30000
MAP_RES_MM = 50

# Log-odds increments per observation. Occupied evidence counts for more than
# free evidence per look, but free space is seen far more often (every cell
# along every ray), so in practice free wins where they disagree — which is
# what you want, since a moving obstacle should not leave a permanent smear.
L_OCC = 0.85
L_FREE = -0.40
L_CLAMP = 5.0        # stops a cell becoming so certain it can never change

# Returns beyond this are ignored for mapping. Far returns are noisier in
# angle, and one bad long ray erases a corridor of real cells.
MAX_MAP_RANGE_MM = 4000
MIN_MAP_RANGE_MM = 120       # inside the chassis, always spurious


class Pose:
    __slots__ = ("x", "y", "th")

    def __init__(self, x=0.0, y=0.0, th=0.0):
        self.x, self.y, self.th = x, y, th

    def copy(self):
        return Pose(self.x, self.y, self.th)

    def as_dict(self):
        return {"x": round(self.x, 1), "y": round(self.y, 1),
                "deg": round(math.degrees(self.th) % 360.0, 1)}

    def __repr__(self):
        return f"Pose({self.x:.0f}, {self.y:.0f}, {math.degrees(self.th):.1f}deg)"


SELF_L = 0.0      # chassis half-length, mm. Set by Slam from the geometry.
SELF_W = 0.0      # chassis half-width


def scan_to_robot(points, max_range=None, min_range=None,
                  off_x=0.0, off_y=0.0, yaw_off=0.0):
    """[(bearing_deg, dist_mm)] -> [(x, y)] in the BODY frame.

    Bearings increase clockwise from the nose, hence the negated y.

    off_x/off_y are where the scanner sits relative to the body centre. On
    this robot it is bolted to the front right corner, so every return is
    ~250 mm away from where a centre-mounted scanner would put it. Without
    the translation the scan origin orbits the true centre of rotation and
    the whole map swings each time the robot turns on the spot.

    *** max_range and min_range default to None, NOT to the module constants.
    Python evaluates default arguments ONCE, at import. Writing them as
    `max_range=MAX_MAP_RANGE_MM` binds whatever the value was at import and
    ignores the module global forever after — so the live tuning in
    tuning.py would appear to work and silently change nothing. Resolving
    them here, per call, is what makes them tunable.
    """
    max_range = MAX_MAP_RANGE_MM if max_range is None else max_range
    min_range = MIN_MAP_RANGE_MM if min_range is None else min_range
    out = []
    ry = math.radians(yaw_off)
    c0, s0 = math.cos(ry), math.sin(ry)
    for a, d in points:
        if d < min_range or d > max_range:
            continue
        r = math.radians(a)
        x, y = d * math.cos(r), -d * math.sin(r)
        if yaw_off:
            x, y = x * c0 - y * s0, x * s0 + y * c0
        x, y = x + off_x, y + off_y
        # Discard the robot's own chassis.
        #
        # A corner-mounted scanner sees its own body: from the front-right
        # corner the chassis extends 300 mm back and 400 mm left, well beyond
        # any sane minimum-range filter. Measured on this robot: 40 of 251
        # returns landed inside the footprint. Left in, they paint a permanent
        # blob around the robot in the map and make the collision guard
        # believe it is boxed in wherever it stands.
        if SELF_L and abs(x) <= SELF_L and abs(y) <= SELF_W:
            continue
        out.append((x, y))
    return out


def robot_to_world(pts, pose):
    c, s = math.cos(pose.th), math.sin(pose.th)
    return [(pose.x + x * c - y * s, pose.y + x * s + y * c) for x, y in pts]


def _wrap(a):
    """Angle to [-pi, pi)."""
    return (a + math.pi) % (2 * math.pi) - math.pi


# ---------------------------------------------------------------------------

class DiffOdometry:
    """Encoder counts plus IMU heading -> pose."""

    def __init__(self, counts_per_rev, wheel_diam_mm, track_mm):
        # Kept as the two separate inputs rather than collapsed into
        # mm_per_count, so both stay tunable at runtime — counts-per-rev is
        # THE calibration number on this robot (measured 330, was assumed
        # 1320) and being able to try a value while driving beats an
        # edit-sync-restart cycle for each guess. mm_per_count is derived
        # below so it can never fall out of step with them.
        self.cpr = counts_per_rev
        self.wheel = wheel_diam_mm
        self.track_mm = track_mm
        self.pose = Pose()
        self._last_l = self._last_r = None
        self._yaw0 = None
        # World heading = IMU heading since _yaw0 + th_off. The offset is what
        # lets anything else correct the heading at all: without it every
        # update overwrote pose.th with the raw IMU value, so a scan-match
        # heading correction lasted exactly one step and gyro drift grew into
        # rotated double walls. Set to the current heading whenever _yaw0 is
        # (re)taken, so a reset or a loaded map keeps its heading too.
        self.th_off = 0.0
        self._last_yaw = None
        self._glitches = 0
        self.using_imu = False
        self.distance = 0.0          # total path length, mm

    @property
    def mm_per_count(self):
        """Distance one encoder count represents. Derived, never stored, so
        retuning either input takes effect on the very next update."""
        return math.pi * self.wheel / max(1e-9, self.cpr)

    # More than this between two updates (~0.2 s) is not a turn this truck can
    # make. Seen live, motors off: 15 -> 255 -> 15 degrees in one second.
    MAX_YAW_STEP_DEG = 60.0
    GLITCH_ACCEPT = 3

    def _sane_yaw(self, yaw):
        """Drop a one-off IMU heading spike. A jump that PERSISTS for
        GLITCH_ACCEPT readings is real (the IMU re-zeroed, say): accept it,
        but shift _yaw0 by the jump so the world heading stays continuous."""
        if yaw is None:
            return None
        last = self._last_yaw
        if last is None:
            self._last_yaw = yaw
            return yaw
        jump = ((yaw - last + 180) % 360) - 180
        if abs(jump) <= self.MAX_YAW_STEP_DEG:
            self._glitches = 0
            self._last_yaw = yaw
            return yaw
        self._glitches += 1
        if self._glitches < self.GLITCH_ACCEPT:
            return last
        self._glitches = 0
        if self._yaw0 is not None:
            self._yaw0 = (self._yaw0 + jump) % 360
        self._last_yaw = yaw
        return yaw

    def reset(self, keep_heading=True):
        self.pose = Pose(0.0, 0.0, self.pose.th if keep_heading else 0.0)
        self._last_l = self._last_r = None
        self._yaw0 = None
        self.distance = 0.0

    def update(self, left_counts, right_counts, yaw_deg=None):
        if self._last_l is None:
            self._last_l, self._last_r = left_counts, right_counts
            if yaw_deg is not None:
                self._yaw0 = yaw_deg
                self.th_off = self.pose.th
            return self.pose

        dl = (left_counts - self._last_l) * self.mm_per_count
        dr = (right_counts - self._last_r) * self.mm_per_count
        self._last_l, self._last_r = left_counts, right_counts

        d_centre = (dl + dr) / 2.0
        self.distance += abs(d_centre)

        yaw_deg = self._sane_yaw(yaw_deg)
        if yaw_deg is not None:
            # Absolute heading from the IMU, referenced to wherever we started.
            # The IMU reports a compass bearing (clockwise); world theta is
            # counter-clockwise, hence the negation.
            if self._yaw0 is None:
                self._yaw0 = yaw_deg
                self.th_off = self.pose.th
            self.pose.th = _wrap(-math.radians(((yaw_deg - self._yaw0 + 180) % 360) - 180)
                                 + self.th_off)
            self.using_imu = True
        else:
            # Fallback: infer the turn from the wheel difference. Works, but
            # every skid accumulates straight into heading error.
            self.pose.th += (dr - dl) / self.track_mm
            self.using_imu = False

        self.pose.x += d_centre * math.cos(self.pose.th)
        self.pose.y += d_centre * math.sin(self.pose.th)
        return self.pose


# ---------------------------------------------------------------------------

class OccupancyGrid:
    def __init__(self, size_mm=MAP_SIZE_MM, res_mm=MAP_RES_MM):
        self.lidar_off = (0.0, 0.0, 0.0)     # set by Slam
        self.res = res_mm
        self.n = int(size_mm / res_mm)
        self.half = self.n // 2
        self.grid = array("f", [0.0]) * (self.n * self.n)
        self.hits = 0

    def clear(self):
        self.grid = array("f", [0.0]) * (self.n * self.n)
        self.hits = 0

    def cell(self, x_mm, y_mm):
        return (int(x_mm // self.res) + self.half,
                int(y_mm // self.res) + self.half)

    def inside(self, cx, cy):
        return 0 <= cx < self.n and 0 <= cy < self.n

    def _bump(self, cx, cy, delta):
        if not self.inside(cx, cy):
            return
        i = cy * self.n + cx
        v = self.grid[i] + delta
        self.grid[i] = L_CLAMP if v > L_CLAMP else (-L_CLAMP if v < -L_CLAMP else v)

    FREE_EVERY = 2      # cast free-space rays for 1 point in N

    def integrate(self, pose, points):
        """Fold one scan in: free along each ray, occupied at its endpoint.

        Every endpoint marks an obstacle, but only every FREE_EVERY-th ray
        carves free space. Cost is dominated by ray LENGTH, not point count:
        a 4 m ray is 80 cells, so 200 rays is 16,000 cell updates per scan.
        Measured on the Pi, that grew the SLAM loop from ~90 ms in a small
        room to 300-785 ms in a house, which then made scan matching saturate
        because the robot moved further between updates than the search window
        could correct.

        Halving the free-space rays halves the dominant cost and loses very
        little: neighbouring rays sweep almost the same cells, and free space
        is re-observed constantly from every new viewpoint. Obstacles are not
        decimated - walls are what the map is for.
        """
        ox, oy = self.cell(pose.x, pose.y)
        if not self.inside(ox, oy):
            return 0
        n = 0
        ox_, oy_, yo_ = self.lidar_off
        every = self.FREE_EVERY
        for i, (wx, wy) in enumerate(robot_to_world(
                scan_to_robot(points, off_x=ox_, off_y=oy_, yaw_off=yo_), pose)):
            ex, ey = self.cell(wx, wy)
            if not self.inside(ex, ey):
                continue
            if i % every == 0:
                self._ray_free(ox, oy, ex, ey)
            self._bump(ex, ey, L_OCC)
            n += 1
        self.hits += n
        return n

    def _ray_free(self, x0, y0, x1, y1):
        """Bresenham from the robot to just short of the endpoint.

        The endpoint itself is deliberately excluded — it gets L_OCC instead.
        Marking it free as well would have every ray argue with itself.
        """
        dx, dy = abs(x1 - x0), abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        x, y = x0, y0
        # _bump inlined. It is called once per cell along every ray - tens of
        # thousands of times per scan - and at that rate the Python call
        # overhead is a large part of the total.
        g = self.grid
        n = self.n
        lo = -L_CLAMP
        while True:
            if x == x1 and y == y1:
                return
            if 0 <= x < n and 0 <= y < n:
                i = y * n + x
                v = g[i] + L_FREE
                g[i] = lo if v < lo else v
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy

    def score(self, pose, robot_pts):
        """How well a scan agrees with the map from a candidate pose.

        Sum of log-odds under the scan's endpoints: high where the scan lands
        on cells the map already believes are occupied.
        """
        c, s = math.cos(pose.th), math.sin(pose.th)
        total = 0.0
        n, half, res, grid = self.n, self.half, self.res, self.grid
        for x, y in robot_pts:
            wx = pose.x + x * c - y * s
            wy = pose.y + x * s + y * c
            cx = int(wx // res) + half
            cy = int(wy // res) + half
            if 0 <= cx < n and 0 <= cy < n:
                total += grid[cy * n + cx]
        return total

    def to_bytes(self):
        """One byte per cell, 0 = certainly free .. 255 = certainly occupied,
        128 = unknown. Sent to the browser base64-encoded and painted through
        ImageData — far cheaper than JSON for 57k cells.

        Linear in log-odds, NOT converted to probability. The old version
        called math.exp once per cell: 57,600 exponentials every time the
        browser asked for the map, at 1 Hz, on a Pi already running SLAM.
        Measured effect of that load: the SLAM loop stretched from ~90 ms to
        785 ms per update, which in turn made scan matching saturate because
        the robot travelled further between updates than the search window
        could correct. The display only needs a monotonic mapping, and this
        one is visually indistinguishable.
        """
        return _quantise(self.array()).tobytes()

    def array(self):
        """The grid as an (n, n) float32 view, row = cy. A view, not a copy:
        read it, do not keep it across a clear()."""
        return np.frombuffer(self.grid, dtype=np.float32).reshape(self.n, self.n)

    # Cells of margin around the explored part, so the frontier is visible.
    CROP_MARGIN = 20

    def bounds(self):
        """(x0, y0, x1, y1) cell bounds of everything ever observed, with a
        margin; a few metres around the start if nothing is yet."""
        a = self.array()
        rows = np.flatnonzero(a.any(axis=1))
        cols = np.flatnonzero(a.any(axis=0))
        m = self.CROP_MARGIN
        if rows.size == 0:
            r = int(3000 // self.res)
            return self.half - r, self.half - r, self.half + r, self.half + r
        return (max(0, int(cols[0]) - m), max(0, int(rows[0]) - m),
                min(self.n, int(cols[-1]) + 1 + m), min(self.n, int(rows[-1]) + 1 + m))

    def as_payload(self):
        """Only the explored rectangle: a 30 m grid is 360k cells, a house
        explored so far is a fraction of that, and this goes out at 1 Hz."""
        x0, y0, x1, y1 = self.bounds()
        crop = _quantise(self.array()[y0:y1, x0:x1])
        return {
            "n": self.n,
            "res": self.res,
            "half": self.half,
            "hits": self.hits,
            "x0": x0, "y0": y0, "w": x1 - x0, "h": y1 - y0,
            "data": base64.b64encode(crop.tobytes()).decode("ascii"),
        }


def _quantise(a):
    """log-odds -> one byte, 128 = unknown. Rounded, not truncated: with
    truncation every save/load cycle nudged each cell one step toward
    unknown, so a map reloaded a few times slowly forgot its walls."""
    b = np.rint(a * (127.0 / L_CLAMP)).astype(np.int32) + 128
    return np.clip(b, 0, 255).astype(np.uint8)


# ---------------------------------------------------------------------------

class ScanMatcher:
    """Correlative scan-to-map matching over a small search window.

    Brute force on purpose. A proper gradient method would be faster in
    principle, but this is a handful of lines, has no local-minimum
    pathologies inside the window, and 200 sparse points does not justify
    anything cleverer. Cost is len(window)^3 * len(points) grid lookups, so
    the window is deliberately small and the scan is decimated.

    Defaults are measured, not guessed. Against a simulated 4x3 m room with
    3.5% wheel slip and 8 deg of heading drift, final pose error over a 4 m
    lap:

        no matching                    147 mm     4.8 ms/update
        120mm / 6deg / decimate 2       31 mm    23.3 ms/update
        80mm  / 4deg / decimate 3       28 mm     9.1 ms/update   <- chosen
        60mm  / 3deg / decimate 4       80 mm     8.1 ms/update

    The middle row is the knee: same accuracy as the widest window at 40% of
    the cost. Narrower than that and the true offset falls outside the search
    window, so the match locks onto a wrong local peak and accuracy collapses.

    One caveat worth knowing: with a PERFECT heading, matching slightly hurts
    (59 mm vs 43 mm) — it adds search noise to an estimate that was already
    right. It pays off as soon as heading drifts at all, which on this robot
    it will.
    """

    # ang_deg is 8, not the 4 this shipped with.
    #
    # Measured on a simulated 6x4.5 m room, one 12 m lap with 3.5% slip and
    # 8 deg/min of heading drift: final pose error 112 mm at +-4 deg, 24 mm
    # at +-8 deg. The old window saturated on 58% of matches - the matcher
    # wanted to rotate further than it was allowed to, every other frame.
    # +-12 deg is no better than +-8 and costs more, so 8 is the knee.
    def __init__(self, lin_mm=80, lin_step=40, ang_deg=8.0, ang_step=2.0,
                 decimate=3, coarse=3):
        self.decimate = decimate
        self.coarse = coarse
        self.off = (0.0, 0.0, 0.0)   # set by Slam
        self.last_score = 0.0
        self.last_conf = 0.0
        self.last_shift = (0.0, 0.0, 0.0)
        self.configure(lin_mm, lin_step, ang_deg, ang_step)

    def configure(self, lin_mm=None, lin_step=None, ang_deg=None,
                  ang_step=None):
        """Set the search window and rebuild the offset lists.

        The window is precomputed rather than looped over each time, so the
        four numbers that define it have to be kept alongside the lists they
        generated — otherwise live tuning could change a value that nothing
        ever reads again. Anything left as None keeps its current value.
        """
        self.lin_mm = lin_mm if lin_mm is not None else getattr(self, "lin_mm", 80)
        self.lin_step = lin_step if lin_step is not None else getattr(self, "lin_step", 40)
        self.ang_deg = ang_deg if ang_deg is not None else getattr(self, "ang_deg", 4.0)
        self.ang_step = ang_step if ang_step is not None else getattr(self, "ang_step", 2.0)

        # Guard the steps: a zero or negative step is a division by zero one
        # frame later, in the SLAM thread, where the traceback is easy to miss.
        self.lin_step = max(1.0, float(self.lin_step))
        self.ang_step = max(0.1, float(self.ang_step))

        # Cap the number of offsets per axis. Cost is len(lin)^2 * len(ang),
        # so a fine step over a wide window explodes cubically — 1 mm steps
        # over +-80 mm is 241 offsets a side, which is 14 million lookups per
        # update and hangs the SLAM thread outright. These values arrive from
        # a slider over the network, so the ceiling belongs here rather than
        # only in whatever UI happens to be sending them.
        n_lin = min(10, int(self.lin_mm // self.lin_step))
        n_ang = min(10, int(self.ang_deg // self.ang_step))
        self.lin = [i * self.lin_step for i in range(-n_lin, n_lin + 1)]
        self.ang = [math.radians(i * self.ang_step)
                    for i in range(-n_ang, n_ang + 1)]

        # The coarse pass: the same shape of window scaled up, so it sweeps
        # `coarse` times further at `coarse` times the step and therefore
        # costs the same. This is what gives the matcher any chance of
        # recovering from an error larger than the fine window.
        c = max(1, int(getattr(self, "coarse", 3)))
        self.lin_c = [i * self.lin_step * c for i in range(-n_lin, n_lin + 1)]             if c > 1 else []
        self.ang_c = [math.radians(i * self.ang_step * c)
                      for i in range(-n_ang, n_ang + 1)] if c > 1 else []

    @property
    def cost(self):
        """Grid lookups per update, both passes. len(lin)^2 * len(ang) each.

        Surfaced because the search window is the one tunable where a small
        change is expensive in a way the number itself does not suggest —
        doubling lin_mm quadruples this.
        """
        one = len(self.lin) ** 2 * len(self.ang)
        return one * (2 if self.coarse > 1 else 1)

    def _search(self, grid, centre, pts, lins, angs):
        """Best pose in one window, plus the numbers confidence is built from.

        Returns (pose, best_score, mean_score). The MEAN matters as much as
        the best: if every candidate scores about the same the surface is
        flat, the winner is noise, and the pose is about to slide.
        """
        best, best_pose = None, centre
        total, count = 0.0, 0
        cand = Pose()
        for dth in angs:
            cand.th = centre.th + dth
            for dx in lins:
                cand.x = centre.x + dx
                for dy in lins:
                    cand.y = centre.y + dy
                    sc = grid.score(cand, pts)
                    total += sc
                    count += 1
                    if best is None or sc > best:
                        best = sc
                        best_pose = Pose(cand.x, cand.y, cand.th)
        return best_pose, (best or 0.0), (total / count if count else 0.0)

    def match(self, grid, pose, points):
        """Correct the pose against the map. Returns (pose, confidence).

        Two passes, coarse then fine. The coarse pass sweeps a much wider
        window at a coarse step; the fine pass refines around whatever it
        found. Measured on a simulated room: a sudden 250 mm pose error - a
        bump, or one bad match - is UNRECOVERABLE with a single +-80 mm
        window no matter how wide the angle search is, because the true
        offset is simply outside it. The coarse pass costs about as much as
        the fine one and raises the capture range several-fold.

        Confidence is returned rather than assumed. The old version accepted
        the best-scoring pose unconditionally, so in a corridor or facing a
        blank wall - where every candidate scores about the same - it adopted
        noise as a correction and baked it into the map. Slam.update() now
        refuses to map on a low-confidence match.
        """
        pts = scan_to_robot(points, off_x=self.off[0], off_y=self.off[1],
                            yaw_off=self.off[2])[::self.decimate]
        if len(pts) < 20 or grid.hits == 0:
            # Nothing to match against yet, or too little to trust.
            self.last_shift = (0.0, 0.0, 0.0)
            self.last_conf = 0.0
            return pose, 0.0

        start = pose
        if self.coarse > 1:
            pose, _, _ = self._search(grid, pose, pts, self.lin_c, self.ang_c)
        best_pose, best, mean = self._search(grid, pose, pts, self.lin, self.ang)

        # Confidence: how PINNED the winning pose is, per axis.
        #
        # Score at the winner against the score one grid cell away along each
        # axis. If sliding along x costs nothing, x is unconstrained, and the
        # winning x is noise however good the overall score looks. The WORST
        # axis decides - a pose pinned sideways and free lengthways is not a
        # pose, it is a guess with a good alibi.
        #
        # This replaced a peak-height-and-sharpness measure that did not
        # work. Measured over a simulated corridor and a normal room, that
        # one scored them 0.88 and 0.87 - indistinguishable, because the
        # candidates that score badly (across the corridor, and every
        # rotation) dominate the mean and keep "peak vs mean" high even while
        # the pose slides freely along the corridor. Per-axis separates them
        # 5.8x: 0.62 in a room, 0.11 in a corridor.
        #
        # Four extra score() calls, against the several hundred the search
        # already did.
        conf = 0.0
        if best > 0:
            step = grid.res
            axis = []
            for dx, dy in ((step, 0.0), (-step, 0.0), (0.0, step), (0.0, -step)):
                p = Pose(best_pose.x + dx, best_pose.y + dy, best_pose.th)
                axis.append((best - grid.score(p, pts)) / abs(best))
            conf = max(0.0, min(1.0, min(max(axis[0], axis[1]),
                                         max(axis[2], axis[3]))))

        self.last_score = best
        self.last_conf = conf
        self.last_shift = (best_pose.x - start.x, best_pose.y - start.y,
                           math.degrees(best_pose.th - start.th))
        return best_pose, conf


# ---------------------------------------------------------------------------

class Slam:
    """Odometry predicts, scan matching corrects, the grid remembers."""

    def __init__(self, counts_per_rev, wheel_diam_mm, track_mm,
                 size_mm=MAP_SIZE_MM, res_mm=MAP_RES_MM, match=True, lidar_off=(0.0, 0.0, 0.0),
                 body=(0.0, 0.0)):
        self.odom = DiffOdometry(counts_per_rev, wheel_diam_mm, track_mm)
        self.grid = OccupancyGrid(size_mm, res_mm)
        self.matcher = ScanMatcher()
        global SELF_L, SELF_W
        SELF_L, SELF_W = body[0] / 2.0, body[1] / 2.0
        self.grid.lidar_off = lidar_off
        self.matcher.off = lidar_off
        self.match_enabled = match
        self.pose = Pose()
        self.trail = [(0.0, 0.0)]
        self.scans = 0
        # Only map once the robot has actually moved or turned a little.
        # Integrating hundreds of identical scans while parked makes the map
        # over-confident about one viewpoint and drowns out later evidence.
        self._last_map_pose = None
        self.MOVE_MM = 40.0
        self.TURN_RAD = math.radians(4.0)

        # --- match confidence -------------------------------------------
        self.conf = 0.0
        self.rejected = 0
        # Below this a match is not believed: the pose is left on odometry and
        # the scan is NOT mapped.
        #
        # 0.25 sits in the gap measured between the two cases it has to tell
        # apart - a featureless corridor scores 0.11 median (0.16 at the 90th
        # percentile), a normal room 0.62 median (0.50 at the 10th). Well
        # clear of both, so ordinary rooms are never refused and a corridor
        # slide is never trusted.
        self.min_conf = 0.25
        # Share of each matched heading correction applied (see update()).
        self.HEADING_GAIN = 0.3
        # No mapping when heading changed more than this since the last update.
        self.MAX_SPIN_RAD = math.radians(10.0)

        # --- loop closure -------------------------------------------------
        # Keyframes are (pose, scan) kept every KEY_MM / KEY_DEG. They exist
        # so that returning to a place already visited can CORRECT the pose
        # rather than silently re-map it at whatever drift has accumulated.
        #
        # Without this, error only ever grows: measured on a simulated loop,
        # 9.3 mm per metre on the first lap and 19.9 by the third. Scan
        # matching cannot fix that on its own, because it corrects against a
        # map that has drifted with the robot.
        self.loop_enabled = True
        self.keys = []               # [(Pose, [(bearing, dist)], trail_len)]
        self.KEY_MM = 300.0
        self.KEY_DEG = 20.0
        self.MAX_KEYS = 400
        # A revisit has to be near in SPACE but far along the TRAIL, or every
        # keyframe would "close a loop" against the one laid down a moment ago
        # and the correction would be meaningless.
        self.LOOP_RADIUS_MM = 700.0
        self.LOOP_MIN_TRAIL = 25
        # A closure needs several old views of the place, not one.
        self.LOOP_MIN_KEYS = 3
        self.LOOP_MAX_KEYS = 12
        self.LOOP_GAIN = 0.30
        self.loops = 0
        self._last_key = None
        self._loop_cooldown = 0
        self._loop_busy = False
        self._loop_last = 0.0
        self._loop_done = None
        self._loop_gen = 0            # bumped by reset/load: late results are dropped

    def reset(self):
        self.odom.reset(keep_heading=False)
        self.grid.clear()
        self.pose = Pose()
        self.trail = [(0.0, 0.0)]
        self.scans = 0
        self._last_map_pose = None
        self.keys = []
        self._last_key = None
        self.rejected = self.loops = 0
        self._loop_gen += 1
        self._loop_done = None

    def _moved_enough(self):
        p = self._last_map_pose
        if p is None:
            return True
        if math.hypot(self.pose.x - p.x, self.pose.y - p.y) >= self.MOVE_MM:
            return True
        return abs(((self.pose.th - p.th + math.pi) % (2 * math.pi)) - math.pi) >= self.TURN_RAD

    def update(self, left_counts, right_counts, yaw_deg, points):
        before = self.odom.pose.copy()
        pose = self.odom.update(left_counts, right_counts, yaw_deg)

        # Only match when odometry says the robot actually moved.
        #
        # Two reasons, and the second is the important one. It saves the CPU
        # cost of a search that cannot find anything new — measured at 96 ms
        # per update on a Pi 4, which saturates the loop. And matching a
        # stationary robot makes the pose RANDOM-WALK: every scan is slightly
        # different noise, so the best-scoring offset wanders, dragging the
        # pose with it and smearing the map from a viewpoint that never moved.
        # Require real TRANSLATION, not just rotation. During a turn on the
        # spot the IMU already gives heading, and running an 80 mm positional
        # search then just lets the pose slide sideways — measured wandering
        # +-80 mm per step while the robot was effectively stationary.
        moved = math.hypot(pose.x - before.x, pose.y - before.y) > 15.0

        trusted = True
        if self.match_enabled and points and moved:
            corrected, conf = self.matcher.match(self.grid, pose, points)
            self.conf = conf
            trusted = conf >= self.min_conf
            if trusted:
                # Heading: take a fraction of the matcher's correction into
                # the IMU offset. A persistent error (gyro drift) is removed
                # over a few steps; one noisy 2-degree match is averaged away
                # instead of being kept forever.
                dth = _wrap(corrected.th - pose.th) * self.HEADING_GAIN
                self.odom.th_off = _wrap(self.odom.th_off + dth)
                corrected.th = _wrap(pose.th + dth)
                # Feed the correction back into odometry, or it re-accumulates
                # the same drift from the same wrong origin on the next step.
                self.odom.pose = corrected
                pose = corrected
            else:
                # The scan could not say where it is - a corridor, a blank
                # wall, or ground the map has not seen. Keep the odometry
                # pose, which at least degrades predictably.
                self.rejected += 1

        self.pose = pose.copy()

        # Do not map on a match we did not believe.
        #
        # This is the asymmetry that matters: a gap in the map gets filled on
        # the next pass, but a smear laid down at a wrong pose never leaves,
        # and every later match aligns against it. Refusing to map is cheap;
        # mapping wrong compounds.
        # Not while spinning fast. One scan takes ~85 ms to sweep, so at
        # speed it is smeared across several degrees, and a turn on the spot
        # is never matched (see `moved`) — mapping it laid rotated copies of
        # every wall. Seen live: a 135-degree spin in 2 s, then 20 refusals.
        spin = abs(_wrap(pose.th - before.th))
        if points and trusted and spin <= self.MAX_SPIN_RAD and self._moved_enough():
            self.grid.integrate(self.pose, points)
            self.scans += 1
            self._last_map_pose = self.pose.copy()
            self.trail.append((self.pose.x, self.pose.y))
            if len(self.trail) > 2000:
                self.trail = self.trail[-2000:]
            self._keyframe(points)

        if self.loop_enabled:
            self._close_loop(points)
        return self.pose

    # Loop closure runs BESIDE the SLAM loop, not in it. One attempt is a
    # 700 mm x 15 degree search: ~160 ms on a PC, several times that on the
    # Pi. It used to run inline on every update wherever the truck had been
    # before (the cooldown only followed a SUCCESS), and live that held SLAM
    # at 300-430 ms an update against a 200 ms budget - a lagging pose that
    # the guard and the path follower both then fought.
    LOOP_EVERY_S = 2.0

    # --- keyframes and loop closure ---------------------------------------

    def _keyframe(self, points):
        """Remember where we were and what we saw, now and then."""
        p = self.pose
        last = self._last_key
        if last is not None:
            moved = math.hypot(p.x - last.x, p.y - last.y)
            turned = abs(((p.th - last.th + math.pi) % (2 * math.pi)) - math.pi)
            if moved < self.KEY_MM and turned < math.radians(self.KEY_DEG):
                return
        # The scan is stored decimated. A keyframe only has to be matchable,
        # not complete, and 400 keyframes of 200 points each is 80k tuples
        # sitting in RAM on a robot that has other uses for it.
        self.keys.append((p.copy(), points[::2], len(self.trail)))
        self._last_key = p.copy()
        if len(self.keys) > self.MAX_KEYS:
            # Drop every other OLD keyframe rather than the oldest: thinning
            # keeps coverage of the whole house, while a queue would forget
            # the first room entirely - which is exactly the room you most
            # want to recognise on the way home.
            self.keys = self.keys[::2]

    def _close_loop(self, points):
        """Recognise a place we have been before, and correct against it.

        The correction goes through apply_fix(), the same blended path marker
        fixes use - a hard snap would jump the robot mid-map and smear the
        next scan across the discontinuity.

        What this does NOT do: un-bend a map that is already bent. Rewriting
        history needs a pose-graph optimisation over all the keyframes. This
        stops the drift growing and re-anchors the robot, which is the part
        that keeps a return-home mission honest.
        """
        # A finished attempt: apply its correction to where the truck is NOW.
        # It was computed for the pose at the start of the attempt, so the
        # offset it found is carried over, not the absolute position.
        done = self._loop_done
        if done is not None:
            self._loop_done = None
            snap, fixed = done
            now_p = self.odom.pose
            self.apply_fix(now_p.x + (fixed.x - snap.x), now_p.y + (fixed.y - snap.y),
                           self.LOOP_GAIN)
            self.loops += 1
            self._loop_cooldown = 20          # do not re-close against the same place
        if not points or len(self.keys) < self.LOOP_MIN_TRAIL:
            return
        if self._loop_cooldown > 0:
            self._loop_cooldown -= 1
            return
        if self._loop_busy or time.monotonic() - self._loop_last < self.LOOP_EVERY_S:
            return

        p, now = self.pose, len(self.trail)
        # EVERY old keyframe near here, not just the nearest.
        #
        # One keyframe scan is ~100 points and makes a local map too sparse to
        # match against: measured, single-keyframe closure was WORSE than no
        # closure at 2-4 laps (73 vs 50 mm, 128 vs 89 mm) because it pulled
        # the pose toward a bad match. Several scans of the same place give a
        # dense enough map for the match to mean something.
        near = [(kp, kpts) for kp, kpts, ktrail in self.keys
                if now - ktrail >= self.LOOP_MIN_TRAIL
                and math.hypot(p.x - kp.x, p.y - kp.y) < self.LOOP_RADIUS_MM]
        if len(near) < self.LOOP_MIN_KEYS:
            return

        # Built from what that place looked like THEN, so the comparison is
        # against old evidence rather than against the accumulated map the
        # pose has already drifted along with.
        self._loop_busy = True
        self._loop_last = time.monotonic()
        snap = p.copy()
        keys = near[-self.LOOP_MAX_KEYS:]
        pts = list(points)
        off = self.matcher.off
        lidar_off = self.grid.lidar_off
        gen = self._loop_gen

        def attempt():
            try:
                local = OccupancyGrid(size_mm=self.grid.n * self.grid.res,
                                      res_mm=self.grid.res)
                local.lidar_off = lidar_off
                for kp, kpts in keys:
                    local.integrate(kp, kpts)
                if local.hits == 0:
                    return
                m = ScanMatcher(lin_mm=self.LOOP_RADIUS_MM, lin_step=self.grid.res * 2,
                                ang_deg=15.0, ang_step=3.0, decimate=4, coarse=1)
                m.off = off
                fixed, conf = m.match(local, snap, pts)
                # A loop closure has to be better than ordinary, not merely
                # acceptable.
                if conf >= self.min_conf * 1.5 and gen == self._loop_gen:
                    self._loop_done = (snap, fixed)
            except Exception:                                  # noqa: BLE001
                pass
            finally:
                self._loop_busy = False

        threading.Thread(target=attempt, daemon=True).start()

    def apply_fix(self, x, y, gain=0.35):
        """Pull the pose toward an absolute position measurement.

        This is the one input that does not come from the robot's own
        history. Everything else here — encoders, gyro integration, scan
        matching against a self-built map — is a closed loop that can drift
        as a whole without any part of it noticing. A surveyed landmark is
        outside that loop, so this is allowed to overrule all of it.

        Blended, not snapped. A hard jump would move the robot mid-map and
        smear the next scan across the discontinuity, and scan matching would
        then spend its search window undoing the correction. At gain 0.35 a
        fix converges over three or four sightings, which is a second or two
        of driving past a tag, and every intermediate pose stays consistent
        with the map.

        Fed back into odometry as well as the public pose — exactly as the
        scan matcher does, and for the same reason. Odometry integrates from
        its own last value, so correcting only self.pose leaves the next
        update re-accumulating from the uncorrected origin and the fix
        evaporates on the following step.
        """
        gain = max(0.0, min(1.0, gain))
        p = self.odom.pose
        p.x += gain * (x - p.x)
        p.y += gain * (y - p.y)
        self.pose = p.copy()
        self.fixes = getattr(self, "fixes", 0) + 1
        return self.pose

    # --- finding itself on a saved map ------------------------------------------

    def relocalize(self, points, step_mm=150.0, ang_deg=5.0, occ=0.5):
        """Where is the truck on the loaded map? Tries every confidently-free
        spot (every step_mm) at every heading (every ang_deg) with ONE scan,
        then refines the best. Does not move the pose - returns a report and
        the caller decides (see adopt()).

        Why: a saved map resumes at the pose it was switched off at. Carried
        somewhere else before switching on - or restarted after a drive that
        was never autosaved - the truck believes it is somewhere it is not,
        the local matcher (a few hundred mm of search) cannot find the truth,
        and everything mapped from then on goes in the wrong place.

        Score = share of scan points that land on occupied cells. The report
        says whether the answer is unique: a second, different pose scoring
        nearly as well (a symmetric room) means the scan cannot tell them apart.
        """
        pts = scan_to_robot(points, off_x=self.matcher.off[0], off_y=self.matcher.off[1],
                            yaw_off=self.matcher.off[2])
        if len(pts) < 30 or self.grid.hits == 0:
            return {"ok": False, "why": "not enough scan or no map"}
        P = np.array(pts[::max(1, len(pts) // 90)], dtype=np.float32)       # ~90 points
        g = self.grid.array()
        occ_map = g > occ
        res, half, n = self.grid.res, self.grid.half, self.grid.n

        def frac(xs, ys, ths):
            """Score for each candidate (arrays of equal length)."""
            c, s = np.cos(ths)[:, None], np.sin(ths)[:, None]
            wx = xs[:, None] + P[None, :, 0] * c - P[None, :, 1] * s
            wy = ys[:, None] + P[None, :, 0] * s + P[None, :, 1] * c
            cx = np.floor(wx / res).astype(np.int32) + half
            cy = np.floor(wy / res).astype(np.int32) + half
            inside = (cx >= 0) & (cx < n) & (cy >= 0) & (cy < n)
            hit = np.zeros(cx.shape, dtype=bool)
            hit[inside] = occ_map[cy[inside], cx[inside]]
            return hit.mean(axis=1)

        # Candidate positions: free cells, on a step_mm lattice.
        k = max(1, int(step_mm // res))
        free = g[::k, ::k] < -1.0
        rows, cols = np.nonzero(free)
        cand_x = ((cols * k - half) * res + res / 2.0).astype(np.float32)
        cand_y = ((rows * k - half) * res + res / 2.0).astype(np.float32)
        if cand_x.size == 0:
            return {"ok": False, "why": "no free space in the map"}
        angs = np.radians(np.arange(0.0, 360.0, ang_deg)).astype(np.float32)
        best = []                                     # (score, x, y, th) per heading
        for a in angs:
            sc = frac(cand_x, cand_y, np.full(cand_x.shape, a, dtype=np.float32))
            order = np.argsort(sc)[-3:]
            best += [(float(sc[i]), float(cand_x[i]), float(cand_y[i]), float(a)) for i in order]
        best.sort(reverse=True)

        def refine(x, y, th):
            dx = np.arange(-step_mm, step_mm + 1, 25.0, dtype=np.float32)
            da = np.radians(np.arange(-ang_deg, ang_deg + 0.1, 1.0)).astype(np.float32)
            X, Y, A = np.meshgrid(x + dx, y + dx, th + da, indexing="ij")
            sc = frac(X.ravel(), Y.ravel(), A.ravel())
            i = int(np.argmax(sc))
            return float(sc[i]), float(X.ravel()[i]), float(Y.ravel()[i]), float(A.ravel()[i])

        top = [refine(x, y, th) for _, x, y, th in best[:6]]
        top.sort(reverse=True)
        b = top[0]
        # The best candidate that is really a different place.
        rival = next((t for t in top[1:] + [(s, x, y, th) for s, x, y, th in best[6:40]]
                      if math.hypot(t[1] - b[1], t[2] - b[2]) > 1000.0
                      or abs(_wrap(t[3] - b[3])) > math.radians(30)), None)
        here = float(frac(np.array([self.pose.x], np.float32), np.array([self.pose.y], np.float32),
                          np.array([self.pose.th], np.float32))[0])
        return {"ok": True, "score": round(b[0], 3), "x": round(b[1]), "y": round(b[2]),
                "deg": round(math.degrees(b[3]) % 360, 1),
                "rival": round(rival[0], 3) if rival else 0.0,
                "here": round(here, 3),
                "moved_mm": round(math.hypot(b[1] - self.pose.x, b[2] - self.pose.y)),
                "candidates": int(cand_x.size) * len(angs)}

    def adopt(self, x, y, th):
        """Put the truck at (x, y, th) on the map: pose, odometry, and a fresh
        baseline for encoders and IMU so nothing jumps on the next update."""
        self.pose = Pose(x, y, th)
        self.odom.pose = self.pose.copy()
        self.odom._last_l = self.odom._last_r = None
        self.odom._yaw0 = None
        self._last_map_pose = None
        self.trail.append((x, y))
        self._loop_gen += 1
        self._loop_done = None

    # --- persistence --------------------------------------------------------
    #
    # Without this the map dies with the process, and "go to the kitchen" is
    # meaningless because the robot re-learns the house on every boot. The
    # pose is saved with the grid: a map without the frame it was built in
    # cannot be resumed, only looked at.

    def save(self, path):
        import base64, json
        blob = {
            "version": 1,
            "n": self.grid.n, "res": self.grid.res,
            "pose": {"x": self.pose.x, "y": self.pose.y, "th": self.pose.th},
            "scans": self.scans,
            "hits": self.grid.hits,
            "trail": [[round(x), round(y)] for x, y in self.trail[-3000:]],
            # log-odds quantised to one byte; the extra precision buys nothing
            # once a cell is past the clamp anyway
            "grid": base64.b64encode(self.grid.to_bytes()).decode("ascii"),
        }
        tmp = path + ".tmp"
        with open(tmp, "w") as f:
            json.dump(blob, f)
        os.replace(tmp, path)        # atomic: a half-written map is worse than none
        return len(blob["grid"])

    def load(self, path):
        import base64, json
        with open(path) as f:
            blob = json.load(f)
        bn = blob.get("n")
        if blob.get("res") != self.grid.res or not bn or bn > self.grid.n:
            raise ValueError(
                "saved map is %sx%s @ %s mm, this one is %sx%s @ %s mm"
                % (bn, bn, blob.get("res"),
                   self.grid.n, self.grid.n, self.grid.res))
        raw = np.frombuffer(base64.b64decode(blob["grid"]), dtype=np.uint8)
        # Both grids have the start at their centre, so a smaller map saved
        # before the grid grew drops into the middle of this one unchanged.
        o = self.grid.half - bn // 2
        self.grid.clear()
        self.grid.array()[o:o + bn, o:o + bn] = \
            (raw.reshape(bn, bn).astype(np.float32) - 128) / 127.0 * L_CLAMP
        self.grid.hits = blob.get("hits", 0)
        p = blob.get("pose") or {}
        self.pose = Pose(p.get("x", 0.0), p.get("y", 0.0), p.get("th", 0.0))
        self.odom.pose = self.pose.copy()
        # Resuming means the encoder and yaw references are stale; clearing
        # them makes the next update re-baseline instead of teleporting.
        self.odom._last_l = self.odom._last_r = None
        self.odom._yaw0 = None
        self.trail = [tuple(v) for v in blob.get("trail", [])] or [(self.pose.x, self.pose.y)]
        self.scans = blob.get("scans", 0)
        self._last_map_pose = None
        # Loop-closure keyframes belong to whatever map was in memory before.
        self.keys = []
        self._last_key = None
        self._loop_gen += 1
        self._loop_done = None
        return blob

    @property
    def state(self):
        dx, dy, dth = self.matcher.last_shift
        return {
            "pose": self.pose.as_dict(),
            "scans": self.scans,
            "cells": self.grid.hits,
            "distance_mm": round(self.odom.distance),
            "imu_heading": self.odom.using_imu,
            "matching": self.match_enabled,
            "correction": {"x": round(dx, 1), "y": round(dy, 1),
                           "deg": round(dth, 2)},
            "trail_len": len(self.trail),
            "fixes": getattr(self, "fixes", 0),
            "conf": round(self.conf, 3),
            "rejected": self.rejected,
            "keys": len(self.keys),
            "loops": self.loops,
        }
