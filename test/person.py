"""
Person tracker — who is in front of the truck, which way, and how far.
The groundwork for a follow mode. Shared library; web_nav.py runs it.

This does NOT drive. It measures, and publishes what a follow controller
needs: a heading error (bearing, degrees, +left) and a distance (mm from the
lens). A PID on top of `state["target"]` is the next step, not this file.

Why not detect.py
-----------------
detect.py pins FURNITURE to the map: ~1 Hz, votes over many frames, and it
throws "person" away on purpose — a robot that maps a person as a fixture has
misunderstood the room. Following wants the opposite: only people, as often
as possible, and nothing remembered.

Detection
---------
On the PC when a detection server is set (tools/detect_server.py, `/person`
on the same port as `/detect`): a COCO model, where "person" is the class it
is best trained on, at several frames a second. On the Pi otherwise, with the
yolov8n model detect.py already loaded — about 0.5 s a frame, so ~1 Hz and
only while tracking is switched on.

Three distances, because each fails differently
-----------------------------------------------
  lidar   The scan returns inside the box's angular span, nearest group. At
          the scanner's height those are legs. Accurate to a few cm — the one
          to steer on. Fails when the person is out of the scanner's plane or
          another object sits in front of them.
  floor   Where the feet meet the floor: the box's bottom edge through the
          camera's height and tilt (cliff.ground_distance). No LiDAR needed.
          Fails when the feet are out of frame, and is only as good as
          CAM_HEIGHT_MM / CAM_PITCH_DEG are measured.
  size    From how wide the box is, assuming ~0.45 m of shoulders. Rough —
          arms and pose change the box — but it always exists.

`distance_mm` is the best available of those, in that order, and `source`
says which. All are from the LENS, which sits at the front face, so they are
roughly the gap in front of the bumper.
"""

import math
import threading
import time

import numpy as np

import detect
from cliff import ground_distance
from pins import CAM_HFOV, CAM_OFFSET_X, CAM_OFFSET_Y, cam_vfov

CONF_MIN = 0.45
# Loop rate: what the backend can sustain, not a wish. The PC answers in
# ~100 ms; the Pi's own model takes ~500 ms and has SLAM to share with.
HZ_REMOTE = 5.0
HZ_ONBOARD = 1.0
# The box edges are background as often as person; use its middle.
BOX_INNER = 0.8
# Returns within this much of the nearest one count as the same person.
LEG_DEPTH_MM = 350.0
MIN_RANGE_MM = 150.0
MAX_RANGE_MM = 6000.0
SHOULDER_MM = 450.0
# A target not seen for this long is gone.
LOST_S = 1.5
# The same person, frame to frame, if their bearing moved less than this.
SAME_TARGET_DEG = 15.0


def person_url(detect_url):
    """The /person endpoint lives beside /detect on the same server."""
    base = (detect_url or "").rstrip("/")
    if base.endswith("/detect"):
        base = base[: -len("/detect")]
    return base + "/person" if base else ""


class PersonTracker:
    """detector: the running detect.Detector — for its camera, rotation,
    bearing maths, server URL and on-board model. body_points: returns the
    current scan as body-frame (x forward, y left) points, mm."""

    def __init__(self, detector, body_points, slam=None, enabled=False):
        self.det = detector
        self.body_points = body_points
        self.slam = slam
        self.enabled = enabled
        self.backend = "off"
        self.error = ""
        self.ms = 0.0
        self.hz = 0.0
        self.frames = 0
        self.people = []
        self.target = None
        self._last_t = 0.0
        self._t_prev = None
        self._remote_down_until = 0.0
        self._remote_why = ""
        threading.Thread(target=self._run, daemon=True).start()

    # --- the loop ---------------------------------------------------------

    def _run(self):
        while True:
            remote = bool(self.det.url)
            time.sleep(1.0 / (HZ_REMOTE if remote else HZ_ONBOARD))
            cam = self.det.camera
            if not self.enabled or cam is None or cam.cam is None:
                self.backend = "off" if not self.enabled else "no camera"
                continue
            try:
                t0 = time.perf_counter()
                self._tick()
                now = time.monotonic()
                self.ms = (time.perf_counter() - t0) * 1000.0
                if self._t_prev:
                    self.hz = 0.7 * self.hz + 0.3 / max(1e-3, now - self._t_prev)
                self._t_prev = now
                self.frames += 1
                self.error = ""
            except Exception as e:                            # noqa: BLE001
                self.error = f"{e.__class__.__name__}: {e}"
            if self.target and time.monotonic() - self._last_t > LOST_S:
                self.target = None

    def _detect(self, rot):
        """[(conf, box)] in the ROTATED frame, from the PC or the Pi."""
        url = person_url(self.det.url)
        why = ""
        if url and time.monotonic() >= self._remote_down_until:
            jpeg = self.det.camera.frame()
            if not jpeg:
                return []
            try:
                raw, _ms, model = detect.remote_detect(url, jpeg, rot, CONF_MIN, timeout=2.0)
                self.backend = f"remote · {model}"
                return [(d["conf"], tuple(d["box"])) for d in raw if d["label"] == "person"]
            except Exception as e:                            # noqa: BLE001
                # Do not pay a timeout on every frame while the PC is away.
                self._remote_down_until = time.monotonic() + 10.0
                self._remote_why = str(e)
        if url:
            why = f" (PC /person not answering: {self._remote_why})"
        if self.det.sess is None:
            raise detect.DetectError("no on-board model" + (why or " and no detection server"))
        if getattr(self.det.slam, "ms", 0.0) > self.det.slam_budget_ms:
            return None                                       # SLAM first, always
        self.backend = "on-board · yolov8n" + why
        y, u, v = self.det.camera.cam.yuv_planes()
        rgb = detect.rotate_cw(detect.yuv420_to_rgb(y, u, v), rot)
        h, w = rgb.shape[:2]
        blob, scale, pad = detect.letterbox(rgb)
        out = self.det.sess.run(None, {self.det.input: blob})
        return [(c, box) for label, c, _x, box in detect.decode(out, scale, pad, w, h, keep={"person"})]

    def _tick(self):
        rot = self.det.rotation
        found = self._detect(rot)
        if found is None:
            return
        pts = self.body_points() or []
        pose = self.slam.slam.pose if self.slam is not None else None
        people = [self._measure(conf, box, rot, pts, pose) for conf, box in found]
        people.sort(key=lambda p: -p["conf"])
        self.people = people
        if people:
            self.target = self._pick(people)
            self._last_t = time.monotonic()

    def _pick(self, people):
        """Stay on the person already followed; otherwise the nearest."""
        if self.target:
            b0 = self.target["bearing"]
            near = min(people, key=lambda p: abs(p["bearing"] - b0))
            if abs(near["bearing"] - b0) < SAME_TARGET_DEG:
                return near
        return min(people, key=lambda p: p["distance_mm"] or 1e9)

    # --- one person -------------------------------------------------------

    def _measure(self, conf, box, rot, pts, pose):
        x0, y0, x1, y1 = box
        bearing = self.det._bearing((x0 + x1) / 2, rot)
        # Angular span of the box's middle BOX_INNER, body frame, +left.
        m = (1 - BOX_INNER) / 2 * (x1 - x0)
        left, right = self.det._bearing(x0 + m, rot), self.det._bearing(x1 - m, rot)

        lidar, n = self._lidar_range(pts, min(left, right), max(left, right))
        floor = self._floor_range(y1, bearing, rot)
        size = self._size_range(x1 - x0, rot)
        for d, src in ((lidar, "lidar"), (floor, "floor"), (size, "size")):
            if d is not None:
                dist, source = d, src
                break
        else:
            dist, source = None, "none"

        rec = {"conf": round(conf, 2), "bearing": round(bearing, 1),
               "box": [round(v, 4) for v in detect.unrotate_box(box, rot)],
               "lidar_mm": _r(lidar), "lidar_points": n,
               "floor_mm": _r(floor), "size_mm": _r(size),
               "distance_mm": _r(dist), "source": source}
        if dist is not None:
            b = math.radians(bearing)
            bx, by = CAM_OFFSET_X + dist * math.cos(b), CAM_OFFSET_Y + dist * math.sin(b)
            rec["body"] = [round(bx), round(by)]
            if pose is not None:
                c, s = math.cos(pose.th), math.sin(pose.th)
                rec["world"] = [round(pose.x + bx * c - by * s), round(pose.y + bx * s + by * c)]
        return rec

    @staticmethod
    def _lidar_range(pts, lo, hi):
        """Nearest group of returns between bearings lo..hi (degrees, +left,
        seen from the lens). The nearest GROUP, not the nearest point: one
        stray return is noise, a pair of legs is several."""
        ranges = []
        for x, y in pts:
            dx, dy = x - CAM_OFFSET_X, y - CAM_OFFSET_Y
            d = math.hypot(dx, dy)
            if not MIN_RANGE_MM <= d <= MAX_RANGE_MM or dx <= 0:
                continue
            if lo <= math.degrees(math.atan2(dy, dx)) <= hi:
                ranges.append(d)
        if not ranges:
            return None, 0
        ranges.sort()
        near = ranges[0] if len(ranges) < 3 else ranges[1]    # drop one stray
        group = [d for d in ranges if d <= near + LEG_DEPTH_MM]
        return float(np.median(group)), len(group)

    @staticmethod
    def _floor_range(y1_rot, bearing, rot):
        """Feet on the floor. The box is in the rotated frame, which is the
        room the right way up, so its bottom edge is the feet at 0 or 180.
        At a quarter turn the picture's vertical is the lens's horizontal and
        the tilt geometry does not apply."""
        if int(rot) % 180 != 0 or y1_rot > 0.97:
            return None                                       # feet out of frame
        d = ground_distance(1.0 - y1_rot)
        return None if d is None else d / max(0.3, math.cos(math.radians(bearing)))

    @staticmethod
    def _size_range(w_frac, rot):
        span = CAM_HFOV if (int(rot) // 90) % 2 == 0 else cam_vfov()
        ang = math.radians(max(0.5, w_frac * span))
        return SHOULDER_MM / (2 * math.tan(ang / 2))

    # --- for the page -----------------------------------------------------

    @property
    def state(self):
        t = self.target
        return {"enabled": self.enabled, "backend": self.backend, "error": self.error,
                "ms": round(self.ms, 1), "hz": round(self.hz, 1), "frames": self.frames,
                "people": self.people if self.enabled else [],
                "target": t if self.enabled else None,
                "age_s": round(time.monotonic() - self._last_t, 1) if t else None}


def _r(v):
    return None if v is None else round(v)
