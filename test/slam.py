"""
SLAM — odometry, occupancy grid, and scan matching.

Shared library like pins.py and imu.py; not run directly. web_nav.py imports
it. Pure Python and the standard library, so nothing new to install.

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
from array import array

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
        self.using_imu = False
        self.distance = 0.0          # total path length, mm

    @property
    def mm_per_count(self):
        """Distance one encoder count represents. Derived, never stored, so
        retuning either input takes effect on the very next update."""
        return math.pi * self.wheel / max(1e-9, self.cpr)

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
            return self.pose

        dl = (left_counts - self._last_l) * self.mm_per_count
        dr = (right_counts - self._last_r) * self.mm_per_count
        self._last_l, self._last_r = left_counts, right_counts

        d_centre = (dl + dr) / 2.0
        self.distance += abs(d_centre)

        if yaw_deg is not None:
            # Absolute heading from the IMU, referenced to wherever we started.
            # The IMU reports a compass bearing (clockwise); world theta is
            # counter-clockwise, hence the negation.
            if self._yaw0 is None:
                self._yaw0 = yaw_deg
            self.pose.th = -math.radians(((yaw_deg - self._yaw0 + 180) % 360) - 180)
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
    def __init__(self, size_mm=12000, res_mm=50):
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
        scale = 127.0 / L_CLAMP
        out = bytearray(len(self.grid))
        for i, v in enumerate(self.grid):
            b = int(v * scale) + 128
            out[i] = 0 if b < 0 else (255 if b > 255 else b)
        return bytes(out)

    def as_payload(self):
        return {
            "n": self.n,
            "res": self.res,
            "half": self.half,
            "hits": self.hits,
            "data": base64.b64encode(self.to_bytes()).decode("ascii"),
        }


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
                 size_mm=12000, res_mm=50, match=True, lidar_off=(0.0, 0.0, 0.0),
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
        if points and trusted and self._moved_enough():
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
        if not points or len(self.keys) < self.LOOP_MIN_TRAIL:
            return
        if self._loop_cooldown > 0:
            self._loop_cooldown -= 1
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
        local = OccupancyGrid(size_mm=self.grid.n * self.grid.res,
                              res_mm=self.grid.res)
        local.lidar_off = self.grid.lidar_off
        for kp, kpts in near[-self.LOOP_MAX_KEYS:]:
            local.integrate(kp, kpts)
        if local.hits == 0:
            return
        m = ScanMatcher(lin_mm=self.LOOP_RADIUS_MM, lin_step=self.grid.res * 2,
                        ang_deg=15.0, ang_step=3.0, decimate=4, coarse=1)
        m.off = self.matcher.off
        fixed, conf = m.match(local, p, points)
        if conf < self.min_conf * 1.5:
            return                    # a loop closure has to be better than
                                      # ordinary, not merely acceptable
        self.apply_fix(fixed.x, fixed.y, self.LOOP_GAIN)
        self.loops += 1
        # Do not re-close against the same place every tick.
        self._loop_cooldown = 20

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
            "grid": base64.b64encode(bytes(
                max(0, min(255, int((v / L_CLAMP) * 127) + 128))
                for v in self.grid.grid)).decode("ascii"),
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
        if blob.get("n") != self.grid.n or blob.get("res") != self.grid.res:
            raise ValueError(
                "saved map is %sx%s @ %s mm, this one is %sx%s @ %s mm"
                % (blob.get("n"), blob.get("n"), blob.get("res"),
                   self.grid.n, self.grid.n, self.grid.res))
        raw = base64.b64decode(blob["grid"])
        g = self.grid.grid
        for i, b in enumerate(raw):
            g[i] = (b - 128) / 127.0 * L_CLAMP
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
