"""
Follow one person. Shared library; web_nav.py runs it.

Lock on to whoever the person tracker sees nearest, then keep following THAT
person - past other people, round furniture, out of the camera's view -
without driving into anyone. The collision guard applies to every command,
with the autonomous margin, and it sees every other person's legs in the
LiDAR, so "without colliding" is the guard's job and cannot be skipped here.

Who is "that person"
--------------------
The camera finds people ~5 times a second but says nothing about which one
is which. The tracker used to call "the one at a similar bearing" the same
person, so anyone walking past took over. Here the locked person is a TRACK:

  where    a world position with a velocity, predicted forward every tick;
  what     a clothing-colour signature (person.signature) learned at lock
           and refreshed slowly while confident.

A detection is the target only if it is near the prediction AND its colours
do not contradict the signature. After losing sight, only a strong colour
match brings the target back - anywhere in view.

Between camera frames, and outside the 54-degree camera view entirely, the
track follows their LEGS in the LiDAR: returns near the prediction. LiDAR
alone is trusted for LIDAR_ONLY_S at most, so the track cannot quietly slide
onto a chair leg and follow that forever.

Driving
-------
Keep FOLLOW_MM from them (centre to person), facing them. If the guard keeps
the truck from going straight at them (something in between), hand over to
the explorer's planner for a route round it, re-aimed as they move. Lost:
turn towards where they were last heading, then go to where they were last
seen and look again; give up after GIVE_UP_S.

The truck is slower than a walking person (MAX_DUTY 0.4 is ~0.3 m/s), so a
brisk walker gets away; it then follows to where they were last seen.
"""

import math
import threading
import time

import numpy as np

import person as person_mod

# 1.6 m, not 1: the camera sits low and tilts up, so at 1 m it saw only legs
# and hips and the detector missed the person in 78 % of frames (live).
FOLLOW_MM = 1600.0          # how far behind to stay, centre to person
HOLD_BAND_MM = 250.0        # +- this around FOLLOW_MM counts as "there"
TOO_CLOSE_MM = 900.0        # closer than this: back away
GATE_MM = 700.0             # a detection this near the prediction may be them
LEG_RADIUS_MM = 300.0       # LiDAR returns this near the prediction are their legs
PERSON_LINK_MM = 250.0      # returns this close together belong to one person (two legs)
AMBIGUOUS_MM = 900.0        # another person this near the prediction makes LiDAR ambiguous
AMBIGUOUS_GAP_MM = 400.0    # two camera matches this similar in distance: wait, do not guess
STATIC_LOGODDS = 2.5        # map cells this sure of an obstacle are furniture, not legs
PERSON_MAX_MM = 700.0       # a LiDAR group wider than this is not a person's legs
LOCK_AGREE_MM = 600.0       # two detections this close together before locking on
REANCHOR_AFTER = 2          # strong colour matches outside the gate in a row: the track is wrong
LIDAR_ONLY_S = 10.0         # longest the track may run on LiDAR alone (4 s ran out live)
SIG_KEEP = 0.40             # colours below this: NOT the target, however near
SIG_REACQUIRE = 0.60        # colours needed to take them back after losing them
SIG_ALONE = 0.35            # ...when they are the ONLY person in view and lost > ALONE_AFTER_S
ALONE_AFTER_S = 3.0
SIG_LEARN = 0.10            # signature refresh rate while confident
BANK_MAX = 5                # views of the target kept besides the one at lock
BANK_NEW = 0.85             # a view is "new" when it matches every banked one less than this
LOST_AFTER_S = 2.5          # unseen this long: start searching (detections are 1-3 Hz live)
VEL_TAU_S = 0.8             # an unrefreshed velocity fades with this time constant
FRESH_S = 0.5               # "too close" needs a measurement this recent
FACE_DEG = 18.0             # further off-centre than this: turn to face them first
APPROACH_LOOK_S = 1.5       # lost: after facing their last-seen spot, look this long before going on
TRAIL_STEP_MM = 150.0       # a trail point every this much movement
TRAIL_MAX = 400             # points kept (~60 m of walking)
TRAIL_SMOOTH = 4            # trail point = mean of this many trusted positions
TRAIL_CAM_S = 2.0           # LiDAR-only positions join the trail this soon after a camera sighting
TRAIL_DIR_MM = 500.0        # heading = over at least this much of the latest trail
TRAIL_AHEAD_MM = 1500.0     # when lost, search this far along their heading
WALL_STANDOFF_MM = 400.0    # ...stopping this short of a wall on the map
STEP_GAIN_MM = 200.0        # a search drive must gain this much...
STEP_STALL_S = 4.0          # ...within this long
STEP_MAX_S = 10.0           # and take no longer than this in all
PERSONAL_SPACE_MM = 300.0   # never drive the outline closer than this to a person
PERSON_RADIUS_MM = 150.0    # a person, as a circle round where they are detected
TRUCK_MM_S = 300.0          # about the truck's top speed at the duty limit
SPIN_SEARCH_MM = 1500.0     # lost nearer than this: turn to look; farther: go there
GIVE_UP_S = 30.0            # unseen this long: stop
MAX_WALK_MM_S = 1500.0      # prediction never assumes faster than this
VEL_WINDOW_S = 1.2          # velocity = slope of the track over this long
PATH_AFTER_S = 1.0          # guard blocking the way this long: plan round it
REAIM_S = 1.5               # how often a planned route is re-aimed at them
TICK = 0.1


class Follower:
    """tracker: person.PersonTracker. slamr: the SLAM runner (.slam.pose,
    .slam.grid). body_points(): the current scan in the body frame, mm.
    guard: web_nav's Guard (for .blocked/.reason/.out). drive(t, s): a
    command through the guard. explorer: explore.Explorer, used for routes."""

    def __init__(self, tracker, slamr, body_points, guard, drive, explorer=None, detector=None):
        self.tracker, self.slamr, self.body_points = tracker, slamr, body_points
        self.guard, self.drive, self.explorer = guard, drive, explorer
        # Furniture detection is paused while following: it shares the PC
        # with the person model, and live the person rate fell to 1-2.6 Hz.
        self.detector = detector
        self._det_was = None
        self.running = False
        self.state = "off"
        self.message = ""
        self._gen = 0
        self._reset_track()

    def _reset_track(self):
        self.tx = self.ty = None           # world position, mm
        self.vx = self.vy = 0.0            # mm/s
        self._hist = []                    # recent (t, x, y) for the velocity
        self.sig = None                    # np.array, clothing colours
        self.bank = []                     # views of them under different light
        self.t_upd = self.t_cam = self.t_seen = 0.0
        self.last_bearing = 0.0            # body frame, deg, + = left
        self.source = ""
        self.switch_refused = 0            # detections turned away as NOT them
        self.last_sim = None               # last colour match seen, for the page
        self.ambiguous = 0                 # camera passes skipped: two could be them
        self._seq = -1
        self._blocked_since = None
        self._path_mode = False
        self._path_aimed = 0.0
        self._search_turned = 0.0
        self._search_prev = None
        self._went_last_seen = False
        self._lock_seen = None
        self._far_match = 0                # colour matches seen outside the gate
        self.trail = []                    # (t, x, y) where they have been, world mm
        self._recent = []                  # last trusted positions, for smoothing
        self._phase = ""                   # search phase, see _search
        self._search_goal = None
        self._heading = None               # their direction of travel when lost, rad

    # --- control ------------------------------------------------------------

    def start(self):
        """Lock on to the nearest person in view and follow them."""
        if self.running:
            self.stop("restarted")
        self._reset_track()
        self.tracker.enabled = True
        if self.detector is not None and self._det_was is None:
            self._det_was = self.detector.enabled
            self.detector.enabled = False
        self.running = True
        self.state = "locking"
        self.message = "looking for someone to follow"
        self._gen += 1
        threading.Thread(target=self._run, args=(self._gen,), daemon=True).start()

    def forget(self, why="map reset"):
        """The map's coordinates changed (reset or load): the trail, last-seen
        spot and track all point at the old frame. Stop, and forget them."""
        if self.running:
            self.stop(why)
        self._reset_track()
        self.message = why

    def stop(self, why="stopped by operator"):
        was = self.running
        self.running = False
        self.state = "off"
        self.message = why
        if self.detector is not None and self._det_was is not None:
            self.detector.enabled = self._det_was
            self._det_was = None
        if was:
            self._release_explorer()
            try:
                self.drive(0, 0)
            except Exception:                                  # noqa: BLE001
                pass

    def _run(self, gen):
        t_start = time.monotonic()
        try:
            while self.running and gen == self._gen:
                t = time.monotonic()
                if self.state == "locking":
                    if self._lock(t):
                        self.state, self.message = "following", "following"
                    elif t - t_start > 8.0:
                        self.stop("nobody in view to follow")
                        return
                    else:
                        self.drive(0, 0)
                else:
                    self._tick(t)
                time.sleep(TICK)
        except Exception as e:                                 # noqa: BLE001
            if gen == self._gen:
                self.stop("follower crashed: %s" % e)

    # --- the track ----------------------------------------------------------

    def _pose(self):
        return self.slamr.slam.pose

    def _lock(self, t):
        """Take the nearest person - once two detection passes in a row put
        them in the same place. Live, the first reading put someone at 5.3 m
        who was at 1.8 m a second later; locked on that, the follower then
        refused every correct detection as too far from where it expected."""
        if self.tracker.seq == self._seq:
            return False
        self._seq = self.tracker.seq
        people = [p for p in (self.tracker.people or []) if p.get("world")]
        if not people:
            self._lock_seen = None
            return False
        p = min(people, key=lambda q: q.get("distance_mm") or 1e9)
        w = tuple(map(float, p["world"]))
        prev = self._lock_seen
        self._lock_seen = w
        if prev is None or math.hypot(w[0] - prev[0], w[1] - prev[1]) > LOCK_AGREE_MM:
            self.message = "looking for someone to follow (hold still a moment)"
            return False
        self.tx, self.ty = w
        self.sig = None if p.get("sig") is None else np.asarray(p["sig"])
        self.bank = [] if self.sig is None else [self.sig]
        self.t_upd = self.t_cam = self.t_seen = t
        self._seq = self.tracker.seq
        self.source = "camera"
        return True

    def _predict(self, t):
        """Where they are now. The velocity's contribution FADES (time
        constant VEL_TAU_S) instead of running on for 1.5 s: live, a velocity
        learned from a jumping range carried the prediction straight onto the
        truck between two detections a second apart."""
        dt = max(0.0, t - self.t_upd)
        k = VEL_TAU_S * (1.0 - math.exp(-dt / VEL_TAU_S))
        return self.tx + self.vx * k, self.ty + self.vy * k

    def _measure(self, t, x, y, weight, source):
        """Blend a measurement into the track and update its velocity.

        A measurement that would drag the track further than a person can
        have walked since the last one counts for less: that is what another
        person's legs look like as they pass through the target's spot.

        Velocity is the slope over the last ~1 s of positions, not the step
        from the previous update: 0.1 s apart with +-50 mm of noise, the step
        swung +-500 mm/s, and a prediction coasting on that went anywhere."""
        px, py = self._predict(t)
        dt = max(1e-3, t - self.t_upd)
        jump = math.hypot(x - px, y - py)
        allowed = 150.0 + MAX_WALK_MM_S * dt
        jumped = jump > allowed
        if jumped:
            weight *= allowed / jump
        nx, ny = px + weight * (x - px), py + weight * (y - py)
        if jumped:
            # A jump is a measurement problem, not walking: forget the velocity
            # rather than learn one from it.
            self._hist = []
            self.vx = self.vy = 0.0
        self._hist.append((t, nx, ny))
        while self._hist and t - self._hist[0][0] > VEL_WINDOW_S:
            self._hist.pop(0)
        t0, x0, y0 = self._hist[0]
        if t - t0 >= 0.8 and len(self._hist) >= 3:
            vx, vy = (nx - x0) / (t - t0), (ny - y0) / (t - t0)
            sp = math.hypot(vx, vy)
            if sp > MAX_WALK_MM_S:
                vx, vy = vx * MAX_WALK_MM_S / sp, vy * MAX_WALK_MM_S / sp
            self.vx, self.vy = vx, vy
        self.tx, self.ty, self.t_upd, self.t_seen = nx, ny, t, t
        self.source = source
        self._trail_add(t, nx, ny, jumped, source)

    def _camera_update(self, t, lost):
        """New detections from the tracker: which one, if any, is the target."""
        if self.tracker.seq == self._seq:
            return False
        self._seq = self.tracker.seq
        px, py = self._predict(t)
        gate = GATE_MM + MAX_WALK_MM_S * min(1.0, t - self.t_seen) * 0.5
        best, best_score, scores = None, None, []
        far_best, far_sim = None, 0.0
        for p in self.tracker.people or []:
            if not p.get("world"):
                continue
            wx, wy = p["world"]
            d = math.hypot(wx - px, wy - py)
            sim = self._match(p.get("sig"))
            if sim is not None:
                self.last_sim = round(sim, 2)
            if lost:
                # Sight lost: only colours bring them back, wherever they are -
                # but with nobody else in view, a weaker match will do. Live,
                # a signature learnt from a partial view never reached 0.60
                # again, and it searched with the person 1.8 m in front of it.
                alone = (len([q for q in self.tracker.people or [] if q.get("world")]) == 1
                         and t - self.t_seen > ALONE_AFTER_S)
                need = SIG_ALONE if alone else SIG_REACQUIRE
                if sim is not None and sim < need:
                    continue
                if sim is None and not alone:
                    continue
                score = -(sim or 0.0)
            else:
                if d > gate:
                    if sim is not None and sim >= SIG_REACQUIRE and sim > far_sim:
                        far_best, far_sim = p, sim
                    continue
                if sim is not None and sim < SIG_KEEP:
                    self.switch_refused += 1         # near, but not them
                    continue
                score = d / gate - (0.0 if sim is None else sim)
            scores.append((d, p))
            if best is None or score < best_score:
                best, best_score = p, score
        if best is None:
            # Nobody near where the track says - but someone who clearly IS
            # them, elsewhere, twice running: the track is what is wrong (a bad
            # first range, legs confused with furniture). Move it to them.
            if far_best is not None:
                self._far_match += 1
                if self._far_match >= REANCHOR_AFTER:
                    self._far_match = 0
                    wx, wy = far_best["world"]
                    self.tx, self.ty, self.vx, self.vy = float(wx), float(wy), 0.0, 0.0
                    self._hist = []
                    self.t_upd = t
                    self._measure(t, float(wx), float(wy), 1.0, "camera")
                    self.t_cam = t
                    return True
            else:
                self._far_match = 0
            return False
        self._far_match = 0
        if not lost and len(scores) > 1:
            # Two people who both fit - near the prediction, colours not
            # ruling either out - and too close together to tell apart: do
            # not guess. Coast on the prediction until they separate; the one
            # who carried on the way the target was going is then nearest.
            # (Guessing here handed the track to a look-alike crossing
            # through the target's spot.)
            ds = sorted(d for d, _ in scores)
            if ds[1] - ds[0] < AMBIGUOUS_GAP_MM:
                self.ambiguous += 1
                return False
        wx, wy = best["world"]
        # Camera distances are the least precise part (floor / box size when
        # the LiDAR had nothing): blend rather than jump.
        self._measure(t, wx, wy, 1.0 if lost else 0.7, "camera")
        self.t_cam = t
        new = best.get("sig")
        sim = self._match(new)
        if new is not None:
            if self.sig is None:
                self.sig = np.asarray(new)
                self.bank = [self.sig]
            elif sim is not None and sim > 0.5:
                self.sig = (1 - SIG_LEARN) * self.sig + SIG_LEARN * np.asarray(new)
                self._remember(np.asarray(new))
        return True

    def _trail_add(self, t, x, y, jumped, source):
        """Record where they are - only from readings worth trusting, and
        smoothed. Live, the raw track drew spikes metres long through walls
        (a rejected jump, LiDAR alone on something else, the track being put
        right), and a search that follows the trail would follow those.

        Not recorded: a reading judged a jump; LiDAR alone more than
        TRAIL_CAM_S after the camera last confirmed them. Each point is the
        mean of the last TRAIL_SMOOTH trusted positions."""
        if jumped or (source == "lidar" and t - self.t_cam > TRAIL_CAM_S):
            return
        self._recent.append((x, y))
        del self._recent[:-TRAIL_SMOOTH]
        sx = sum(p[0] for p in self._recent) / len(self._recent)
        sy = sum(p[1] for p in self._recent) / len(self._recent)
        if not self.trail or math.hypot(sx - self.trail[-1][1], sy - self.trail[-1][2]) >= TRAIL_STEP_MM:
            self.trail.append((t, sx, sy))
            del self.trail[:-TRAIL_MAX]

    def _match(self, sig):
        """How well a detection's colours match the target: the BEST match
        against a small bank of what they have looked like - the signature
        at lock, the running average, and up to BANK_MAX views learnt under
        other light. One running signature drifted: live, by a window, the
        target scored 0.37 against themselves, under the 0.40 'not them'
        line."""
        if sig is None or self.sig is None:
            return None
        return max(person_mod.similarity(b, sig) for b in [self.sig] + self.bank)

    def _remember(self, sig):
        """Keep a view unlike the ones already banked (new light, turned
        round); the lock-time view is never dropped."""
        if all(person_mod.similarity(b, sig) < BANK_NEW for b in self.bank):
            self.bank.append(sig)
            if len(self.bank) > BANK_MAX + 1:
                del self.bank[1]

    def _lidar_update(self, t, pose):
        """Their legs: returns near the prediction. Only while the camera
        confirmed them recently - a track left on LiDAR alone drifts onto the
        nearest chair leg and follows that.

        Returns are grouped into PEOPLE (points within PERSON_LINK_MM of each
        other - two legs make one group). If another group is nearly as close
        to the prediction, this tick is skipped: averaging every return near
        the prediction is exactly how someone walking through the target's
        spot used to drag the track away with them."""
        if t - self.t_cam > LIDAR_ONLY_S:
            return False
        px, py = self._predict(t)
        c, s = math.cos(pose.th), math.sin(pose.th)
        near = []
        grid = getattr(self.slamr.slam, "grid", None)
        for bx, by in self.body_points() or []:
            wx, wy = pose.x + bx * c - by * s, pose.y + bx * s + by * c
            if math.hypot(wx - px, wy - py) >= AMBIGUOUS_MM:
                continue
            # Not on the map's settled obstacles. A person moves, so their
            # legs are not in the map; a chair leg is. Live, the track sat on
            # something 0.8 m away for 10 s while the person was at 2.5 m.
            if grid is not None:
                cx, cy = grid.cell(wx, wy)
                if grid.inside(cx, cy) and grid.grid[cy * grid.n + cx] > STATIC_LOGODDS:
                    continue
            near.append((wx, wy))
        groups = []
        for q in near:                                   # single-link grouping
            hit = [g for g in groups if any(math.hypot(q[0] - r[0], q[1] - r[1]) < PERSON_LINK_MM for r in g)]
            merged = [q]
            for g in hit:
                merged += g
                groups.remove(g)
            groups.append(merged)
        cands = []
        for g in groups:
            if len(g) < 3:
                continue
            xs_, ys_ = [p[0] for p in g], [p[1] for p in g]
            if math.hypot(max(xs_) - min(xs_), max(ys_) - min(ys_)) > PERSON_MAX_MM:
                continue                                 # a wall or a sofa, not two legs
            gx, gy = sum(p[0] for p in g) / len(g), sum(p[1] for p in g) / len(g)
            cands.append((math.hypot(gx - px, gy - py), gx, gy))
        cands.sort()
        if not cands or cands[0][0] > LEG_RADIUS_MM:
            return False
        if len(cands) > 1 and cands[0][0] > 0.5 * cands[1][0]:
            return False                                 # two people, not clearly which
        self._measure(t, cands[0][1], cands[0][2], 0.5, "lidar")
        return True

    # --- one tick ------------------------------------------------------------

    def _tick(self, t):
        pose = self._pose()
        lost = t - self.t_seen > LOST_AFTER_S
        self._camera_update(t, lost)
        self._lidar_update(t, pose)
        unseen = t - self.t_seen
        if unseen > GIVE_UP_S:
            self.stop("lost them - not seen for %d s" % GIVE_UP_S)
            return
        px, py = self._predict(t)
        c, s = math.cos(pose.th), math.sin(pose.th)
        dx, dy = px - pose.x, py - pose.y
        bx, by = dx * c + dy * s, -dx * s + dy * c
        d, b = math.hypot(bx, by), math.degrees(math.atan2(by, bx))
        if unseen <= LOST_AFTER_S:
            self.last_bearing = b
            self._search_turned, self._search_prev, self._went_last_seen = 0.0, None, False
            self._phase, self._search_goal, self._heading = "", None, None
            self._approach_done = None
            self.state = "following"
            self.message = "following · %.1f m · %s" % (d / 1000.0, self.source)
            self._follow(t, d, b, px, py, pose)
        else:
            self.state = "searching"
            self._search(t, pose, px, py, unseen)

    def _follow(self, t, d, b, px, py, pose):
        g = self.guard
        far = d > FOLLOW_MM + HOLD_BAND_MM
        # Blocked on the way to them: plan round whatever is in between.
        straight_blocked = bool(g is not None and g.enabled and g.blocked and far and abs(b) < 35)
        if straight_blocked:
            if self._blocked_since is None:
                self._blocked_since = t
        else:
            self._blocked_since = None
        if self._path_mode or (self._blocked_since is not None
                               and t - self._blocked_since > PATH_AFTER_S):
            if self._route(t, d, px, py, pose):
                return
        g_back = bool(g is not None and g.blocked and "behind" in (g.reason or ""))
        if d < TOO_CLOSE_MM and t - self.t_upd < FRESH_S and self._scan_sees_close(b) and not g_back:
            # They came close - by a MEASUREMENT, not the prediction: live, a
            # prediction backed the truck away from nobody.
            self.drive(-0.45, 0.0)
            self.message += " · backing off"
        elif d < TOO_CLOSE_MM:
            self.drive(0, 0)
        elif abs(b) > FACE_DEG:
            # Face them before anything else: the camera covers only +-27
            # degrees, and live a person walking sideways drifted to the edge
            # of the frame while the truck arced, and was lost there.
            self.drive(0, (-1 if b > 0 else 1) * min(1.0, max(0.6, abs(b) / 90.0)))
        elif far:
            speed = min(1.0, max(0.35, (d - FOLLOW_MM) / 800.0)) * max(0.35, math.cos(math.radians(b)))
            steer = max(-0.6, min(0.6, -b / 40.0))
            if self._too_near_people(t, pose, speed, steer):
                self.drive(0, 0)
                self.message += " · giving someone room"
            else:
                self.drive(speed, steer)
        elif abs(b) > 12:
            self.drive(0, (-1 if b > 0 else 1) * 0.6)       # there: keep facing them
        else:
            self.drive(0, 0)

    def _scan_sees_close(self, bearing):
        """Does the LiDAR itself see something within TOO_CLOSE_MM (+ half
        the truck) in that direction? A bogus close reading from the camera
        must not be enough to reverse the truck."""
        for x, y in self.body_points() or []:
            if math.hypot(x, y) < TOO_CLOSE_MM + 150.0 and                     abs((math.degrees(math.atan2(y, x)) - bearing + 180) % 360 - 180) < 35:
                return True
        return False

    def _people_now(self, t, pose):
        """Everyone the truck knows is there, in the BODY frame: each fresh
        camera detection, plus the target's prediction."""
        out = []
        c, s = math.cos(pose.th), math.sin(pose.th)
        pts = [tuple(p["world"]) for p in (self.tracker.people or []) if p.get("world")]
        if t - getattr(self.tracker, "stamp", t) > 1.5:
            pts = []
        if self.tx is not None:
            pts.append(self._predict(t))
        for wx, wy in pts:
            dx, dy = wx - pose.x, wy - pose.y
            out.append((dx * c + dy * s, -dx * s + dy * c))
        return out

    def _too_near_people(self, t, pose, throttle, steer):
        """Would this move bring the truck's outline within PERSONAL_SPACE_MM
        of anyone - closer than they already are? The guard lets the truck
        pass walls and doorframes at 10-20 mm, which is right for walls and
        wrong for people: in simulation, with the track pushed off by bad
        ranges, the truck drove up alongside the person it was following and
        stopped 55 mm from their legs. People get a wider berth."""
        people = self._people_now(t, pose)
        if not people:
            return False
        geom = getattr(self.explorer, "geom", None) or {"len": 250.0, "wid": 295.0}
        hl, hw = geom["len"] / 2.0, geom["wid"] / 2.0

        def gap(px, py, x, y, th):
            cs, sn = math.cos(th), math.sin(th)
            bx, by = (px - x) * cs + (py - y) * sn, -(px - x) * sn + (py - y) * cs
            return math.hypot(max(0.0, abs(bx) - hl), max(0.0, abs(by) - hw)) - PERSON_RADIUS_MM

        left, right = throttle + steer, throttle - steer
        pk = max(1.0, abs(left), abs(right))
        v = (left + right) / 2.0 / pk * TRUCK_MM_S
        w = (right - left) / pk / (2.0 * hw) * TRUCK_MM_S
        now = [gap(px, py, 0.0, 0.0, 0.0) for px, py in people]
        x = y = th = 0.0
        for _ in range(8):                               # the next 0.8 s
            x += v * math.cos(th) * 0.1
            y += v * math.sin(th) * 0.1
            th += w * 0.1
            for (px, py), g0 in zip(people, now):
                g1 = gap(px, py, x, y, th)
                if g1 < PERSONAL_SPACE_MM and g1 < g0 - 1.0:
                    return True
        return False

    def _route(self, t, d, px, py, pose):
        """Let the explorer's planner take the truck round the obstacle to a
        point FOLLOW_MM short of them, re-aimed as they move. Returns True
        while it is driving."""
        ex = self.explorer
        if ex is None:
            return False
        if d <= FOLLOW_MM + HOLD_BAND_MM:
            self._release_explorer()
            return False
        # Back to following directly once nothing on the map is in between
        # and the guard has let it move for a moment. Live, it stayed in
        # "going round" for 12 s after the way had cleared.
        g = self.guard
        if (self._path_mode and t - self._path_aimed > 1.0 and not self._occluded(pose, px, py)
                and not (g is not None and g.blocked)):
            self._release_explorer()
            return False
        if not self._path_mode or t - self._path_aimed > REAIM_S or not ex.running:
            k = max(0.0, (d - FOLLOW_MM) / max(d, 1.0))
            gx, gy = pose.x + (px - pose.x) * k, pose.y + (py - pose.y) * k
            ex.goto(gx, gy, "the person")
            self._path_mode, self._path_aimed = True, t
        self.message += " · going round"
        return True

    def _release_explorer(self):
        if self._path_mode and self.explorer is not None and self.explorer.running:
            self.explorer.stop("follow took back the wheel")
        self._path_mode = False
        self._blocked_since = None

    def _occluded(self, pose, px, py):
        """Is there a wall on the MAP between the truck and that point?"""
        g = self.slamr.slam.grid
        n = int(math.hypot(px - pose.x, py - pose.y) // g.res)
        for i in range(1, max(1, n - 3)):          # stop short of the person
            f = i / float(n)
            cx, cy = g.cell(pose.x + (px - pose.x) * f, pose.y + (py - pose.y) * f)
            if g.inside(cx, cy) and g.grid[cy * g.n + cx] > 0.6:
                return True
        return False

    def _someone_at(self, pose, x, y):
        """Does the LiDAR see something not on the map - legs, not furniture -
        within LEG_RADIUS_MM of (x, y)?"""
        c, s = math.cos(pose.th), math.sin(pose.th)
        grid = getattr(self.slamr.slam, "grid", None)
        n = 0
        for bx, by in self.body_points() or []:
            wx, wy = pose.x + bx * c - by * s, pose.y + bx * s + by * c
            if math.hypot(wx - x, wy - y) > LEG_RADIUS_MM:
                continue
            if grid is not None:
                cx, cy = grid.cell(wx, wy)
                if grid.inside(cx, cy) and grid.grid[cy * grid.n + cx] > STATIC_LOGODDS:
                    continue
            n += 1
        return n >= 3

    def _step_start(self, t, pose, gx, gy):
        self._step = {"t0": t, "gx": gx, "gy": gy, "best": math.hypot(gx - pose.x, gy - pose.y), "t_best": t}

    def _step_stalled(self, t, pose):
        """A search drive that is getting nowhere: under STEP_GAIN_MM closer
        in STEP_STALL_S, or longer than STEP_MAX_S in all. Live, one sat 15 s
        creeping at a last-seen spot it could not reach, looking at nothing."""
        st = getattr(self, "_step", None)
        if not st:
            return False
        d = math.hypot(st["gx"] - pose.x, st["gy"] - pose.y)
        if d < st["best"] - STEP_GAIN_MM:
            st["best"], st["t_best"] = d, t
        if t - st["t_best"] > STEP_STALL_S or t - st["t0"] > STEP_MAX_S:
            self._release_explorer()
            self._step = None
            return True
        return False

    def _trail_heading(self):
        """Their direction of travel from the end of the trail: the last point
        and the first one at least TRAIL_DIR_MM back. None if they were
        standing still - then there is no direction to follow."""
        if len(self.trail) < 2:
            return None
        _, x1, y1 = self.trail[-1]
        for _, x0, y0 in reversed(self.trail[:-1]):
            if math.hypot(x1 - x0, y1 - y0) >= TRAIL_DIR_MM:
                return math.atan2(y1 - y0, x1 - x0)
        return None

    def _along_free(self, x, y, heading, dist):
        """Walk from (x, y) along heading on the MAP, up to dist, stopping
        WALL_STANDOFF_MM short of the first wall. Where they most likely went."""
        g = self.slamr.slam.grid
        step = g.res
        c, s_ = math.cos(heading), math.sin(heading)
        best = 0.0
        d = step
        while d <= dist:
            cx, cy = g.cell(x + c * d, y + s_ * d)
            if not g.inside(cx, cy) or g.grid[cy * g.n + cx] > 0.6:
                break
            best = d
            d += step
        best = max(0.0, best - WALL_STANDOFF_MM) if best < dist else best
        return x + c * best, y + s_ * best

    def _search(self, t, pose, px, py, unseen):
        """Search using WHERE THEY WERE, from their trail on the map - in the
        order a person would:

          lastseen  go to the spot they were last seen: where the view opens
                    up round the corner or through the door they took
          face      look the way they were going
          spin      a full look round
          trail     carry on along their direction of travel, 1.5 m over
                    open floor (stopping short of walls), and look round again

        Before, a lost person meant a spin on the spot or a drive to where a
        fading prediction happened to end - "very random", as found live. And
        heading straight down the trail first cost 12 s in simulation when
        they had turned off it behind a wall: the last-seen spot showed them."""
        if not self._phase:
            self._heading = self._trail_heading()
            self._phase = "lastseen"
            self._approached = False

        if self._phase == "approach":
            # Something is standing at the spot they were last seen: point the
            # CAMERA at it from following distance. Live, the planner's path to
            # the spot had the truck heading 34 degrees off it - outside the
            # camera's +-27 - and it drove to 0.75 m from someone still
            # standing there, unseen.
            lx, ly = self._search_goal
            c, s_ = math.cos(pose.th), math.sin(pose.th)
            dx, dy = lx - pose.x, ly - pose.y
            d = math.hypot(dx, dy)
            b = math.degrees(math.atan2(-dx * s_ + dy * c, dx * c + dy * s_))
            self.message = "looking where they were last seen (%.0f s)" % unseen
            if self._approach_done is None:
                if abs(b) > 15:
                    self.drive(0, (-1 if b > 0 else 1) * 0.6)
                    return
                if (d > FOLLOW_MM and not self._step_stalled(t, pose)
                        and not self._too_near_people(t, pose, 0.5, max(-0.5, min(0.5, -b / 40.0)))):
                    self.drive(0.5, max(-0.5, min(0.5, -b / 40.0)))
                    return
                self._approach_done = t
            self.drive(0, 0)
            if t - self._approach_done < APPROACH_LOOK_S:
                return                                   # give the camera a few frames
            # Not them after all: look round from here.
            self._phase = "face" if self._heading is not None else "spin"
            self._search_turned, self._search_prev = 0.0, None
            return

        if not self._path_mode and self._phase == "lastseen" and self._search_goal is None:
            if self.tx is not None and self.explorer is not None:
                # The track's last MEASURED position, not the trail's end: the
                # trail leaves out LiDAR-only readings, so its end can be
                # metres behind where the LiDAR last had them.
                lx, ly = self.tx, self.ty
                # Always go to the spot unless already on it. Looking from
                # nearby instead was tried: last seen just inside a doorway,
                # the spot is close and "in the open", but only standing in
                # the doorway shows the room they went into (simulation:
                # 79 % -> 34 % of the time near them).
                if math.hypot(lx - pose.x, ly - pose.y) > 600.0:
                    self._search_goal = (lx, ly)
                    self.explorer.goto(lx, ly, "where they were last seen")
                    self._path_mode = True
                    self._step_start(t, pose, lx, ly)

        if self._phase == "lastseen" and self._path_mode and not self._approached and self._search_goal:
            lx, ly = self._search_goal
            if (math.hypot(lx - pose.x, ly - pose.y) < FOLLOW_MM + HOLD_BAND_MM
                    and self._someone_at(pose, lx, ly) and not self._occluded(pose, lx, ly)):
                self._release_explorer()
                self._path_mode = False
                self._approached = True
                self._phase = "approach"
                self._approach_done = None
                self._step_start(t, pose, lx, ly)
                return

        if self._phase == "lastseen":
            if self._path_mode and self.explorer is not None and self.explorer.running                     and not self._step_stalled(t, pose):
                self.message = "going to where they were last seen (%.0f s)" % unseen
                return
            self._path_mode = False
            self._phase = "face" if self._heading is not None else "spin"
            self._search_turned, self._search_prev = 0.0, None

        if self._phase == "face":
            want = math.degrees(self._heading)
            err = ((want - math.degrees(pose.th) + 180) % 360) - 180
            if abs(err) > 15:
                self.message = "looking the way they went"
                self.drive(0, (-1 if err > 0 else 1) * 0.6)
                return
            self._phase = "spin"
            self._search_turned, self._search_prev = 0.0, None

        if self._phase == "spin":
            th = math.degrees(pose.th)
            if self._search_prev is not None:
                self._search_turned += abs(((th - self._search_prev + 180) % 360) - 180)
            self._search_prev = th
            if self._search_turned < 360.0:
                self._release_explorer()
                self.message = "looking round for them (%.0f s)" % unseen
                self.drive(0, (-1 if self.last_bearing > 0 else 1) * 0.6)
                return
            # Nothing here: carry on the way they were going, then look again.
            if self._heading is not None and self.explorer is not None:
                self._search_goal = self._along_free(pose.x, pose.y, self._heading, TRAIL_AHEAD_MM)
                self.explorer.goto(self._search_goal[0], self._search_goal[1], "where they were heading")
                self._path_mode = True
                self._phase = "trail"
                self._step_start(t, pose, *self._search_goal)
            else:
                self._search_turned, self._search_prev = 0.0, None
            return

        if self._phase == "trail":
            if self._path_mode and self.explorer is not None and self.explorer.running                     and not self._step_stalled(t, pose):
                self.message = "following their trail (%.0f s)" % unseen
                return
            self._path_mode = False
            self._phase = "spin"
            self._search_turned, self._search_prev = 0.0, None

    # --- for the page ---------------------------------------------------------

    @property
    def status(self):
        pose = self._pose()
        out = {"running": self.running, "state": self.state, "message": self.message,
               "switch_refused": self.switch_refused, "ambiguous": self.ambiguous,
               "last_sim": self.last_sim,
               "trail": [[round(x), round(y)] for _, x, y in self.trail[-150:]],
               "last_seen": ([round(self.tx), round(self.ty)] if self.tx is not None else None),
               "search_goal": ([round(v) for v in self._search_goal] if self._search_goal else None),
               "phase": self._phase,
               "source": self.source,
               "target": None, "distance_mm": None, "seen_s_ago": None}
        if self.tx is not None:
            px, py = self._predict(time.monotonic())
            out["target"] = [round(px), round(py)]
            out["distance_mm"] = round(math.hypot(px - pose.x, py - pose.y))
            out["seen_s_ago"] = round(time.monotonic() - self.t_seen, 1)
        return out
