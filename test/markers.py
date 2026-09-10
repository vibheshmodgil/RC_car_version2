"""
ArUco markers — the absolute position fix SLAM cannot produce for itself.

Shared library like slam.py and imu.py. web_nav.py imports it; marker_test.py
drives it from the command line.

    sudo apt install -y python3-opencv
    python test/marker_test.py --sheet     # printable tags -> test/captures/

Why this and not more scan matching
-----------------------------------
Scan matching corrects the pose against the map the robot itself built. That
makes it a closed loop: when the map slowly bends, the pose bends with it and
nothing in the system can tell. Every odometry-plus-lidar stack drifts this
way eventually, and the drift is invisible from inside.

A printed tag at a known place is OUTSIDE that loop. It is the only sensor
input this robot has whose correctness does not depend on the robot's own
history. One sighting collapses accumulated drift to the accuracy of the
sighting, which is a few centimetres at a metre.

Learn, then use
---------------
Tags do not have to be surveyed by hand. The workflow is:

  1. Stick tags on doorframes and walls. Any ids, any order.
  2. Drive around with SLAM running and mapping trusted. Each new tag is
     RECORDED at wherever the current pose says it is.
  3. From then on, seeing a known tag CORRECTS the pose instead.

So the map builds the tag positions once, and the tags hold the map straight
forever after. Saved in marker_map.json next to this file, so it survives a
reboot exactly like house_map.json does.

Position, not heading
---------------------
A fix moves the robot's x and y and leaves its heading alone, even though
solvePnP returns a full 6-DOF pose.

That is deliberate. Marker orientation is the famously unreliable half of the
estimate — a square seen near head-on has two nearly equally good pose
solutions, and the estimate flips between them from frame to frame. Position
does not suffer from this. Meanwhile heading is the one thing this robot
already measures well, with a gyro that does not care about wheel slip. So
the tag supplies what the IMU cannot (absolute position) and the IMU supplies
what the tag cannot (stable heading), and neither is asked to do the other's
job.

Accuracy
--------
Intrinsics are derived from CAM_HFOV rather than a checkerboard calibration.
For a fixed-focus module that is good to a few percent in the middle of the
frame, which at a metre is a centimetre or two — far below the drift being
corrected. Proper calibration would help at the edges; it is not the limiting
factor here and is not worth the ceremony.

Range error grows with the SQUARE of distance, because it comes from corner
localisation in pixels. Hence MARKER_MAX_MM: a distant tag is not a weak fix,
it is a confidently wrong one.
"""

import json
import math
import os
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from pins import (  # noqa: E402
    CAM_HFOV, CAM_OFFSET_X, CAM_OFFSET_Y, CAM_YAW_OFFSET,
    MARKER_DICT, MARKER_SIZE_MM, MARKER_MAX_MM,
    MARKER_FIX_GAIN, MARKER_SANITY_MM,
)

MAP_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        "marker_map.json")


class MarkerError(RuntimeError):
    """Raised when OpenCV or its aruco module is unavailable, with the fix."""


def _cv2():
    """OpenCV, or an error that says how to get it.

    aruco lives in opencv_contrib upstream, but Debian builds it into
    python3-opencv, so on Raspberry Pi OS the one apt package is enough.
    Checked separately from cv2 itself because an OpenCV without aruco fails
    much later and much more confusingly.
    """
    try:
        import cv2
    except ImportError as e:
        raise MarkerError(
            "opencv not installed — sudo apt install -y python3-opencv "
            "(and the venv needs --system-site-packages to see it)") from e
    if not hasattr(cv2, "aruco"):
        raise MarkerError(
            "this OpenCV build has no aruco module — the Debian "
            "python3-opencv package includes it; a pip opencv-python does not "
            "(pip users need opencv-contrib-python)")
    return cv2


def intrinsics(width, height, hfov_deg=None):
    """Camera matrix and zero distortion, derived from the field of view.

    For a rectilinear lens, half the sensor width subtends half the FOV at
    one focal length: fx = (w/2) / tan(hfov/2). fy equals fx because the
    pixels are square, which is true of every Raspberry Pi module — the
    vertical FOV then falls out of the aspect ratio rather than being a
    second number to keep in step.
    """
    import numpy as np
    hfov = CAM_HFOV if hfov_deg is None else hfov_deg
    fx = (width / 2.0) / math.tan(math.radians(hfov) / 2.0)
    K = np.array([[fx, 0.0, width / 2.0],
                  [0.0, fx, height / 2.0],
                  [0.0, 0.0, 1.0]], dtype=np.float64)
    return K, np.zeros((5, 1), dtype=np.float64)


def _detector(cv2):
    """detectMarkers, across the 4.7 API break.

    OpenCV 4.7 replaced the module-level functions with an ArucoDetector
    object and renamed the dictionary getter. Trixie ships something recent
    enough for the new API, but marker_test.py is also the sort of thing
    people run on an older Pi, and the shim is six lines.

    Returns a callable: gray -> (corners, ids).
    """
    name = getattr(cv2.aruco, MARKER_DICT, None)
    if name is None:
        raise MarkerError("unknown dictionary %r — see cv2.aruco.DICT_*"
                          % MARKER_DICT)
    if hasattr(cv2.aruco, "ArucoDetector"):                   # >= 4.7
        d = cv2.aruco.getPredefinedDictionary(name)
        det = cv2.aruco.ArucoDetector(d, cv2.aruco.DetectorParameters())

        def detect(gray):
            corners, ids, _ = det.detectMarkers(gray)
            return corners, ids
    else:                                                     # < 4.7
        d = cv2.aruco.Dictionary_get(name)
        params = cv2.aruco.DetectorParameters_create()

        def detect(gray):
            corners, ids, _ = cv2.aruco.detectMarkers(gray, d,
                                                      parameters=params)
            return corners, ids
    return detect


def _object_points(size_mm):
    """The tag's four corners in its own frame, in the order detectMarkers
    returns them: top-left, top-right, bottom-right, bottom-left."""
    import numpy as np
    h = size_mm / 2.0
    return np.array([[-h, h, 0.0], [h, h, 0.0],
                     [h, -h, 0.0], [-h, -h, 0.0]], dtype=np.float64)


def camera_to_body(x_cam, z_cam):
    """A point measured by the lens, expressed in the robot's body frame.

    OpenCV's camera frame is x right, y down, z forward. The body frame used
    everywhere else in this project is x forward, y LEFT. So forward is z,
    and left is minus x — that sign is the one worth checking first when a
    tag appears on the wrong side of the map.

    Then the lens's own mounting is undone: rotate by how the camera is
    aimed, translate by where it sits.
    """
    xc, yc = z_cam, -x_cam
    a = math.radians(CAM_YAW_OFFSET)
    ca, sa = math.cos(a), math.sin(a)
    return (xc * ca - yc * sa + CAM_OFFSET_X,
            xc * sa + yc * ca + CAM_OFFSET_Y)


class MarkerMap:
    """Tag id -> world position, persisted.

    Only x and y are stored. A tag's own orientation is never used, so
    recording it would be recording a number that is both unreliable and
    unread.
    """

    def __init__(self, path=MAP_FILE):
        self.path = path
        self.tags = {}          # id -> {"x", "y", "n", "t"}
        self.load()

    def load(self):
        try:
            with open(self.path) as f:
                d = json.load(f)
            self.tags = {int(k): v for k, v in d.get("tags", {}).items()}
        except (OSError, ValueError):
            self.tags = {}
        return len(self.tags)

    def save(self):
        tmp = self.path + ".tmp"
        with open(tmp, "w") as f:
            json.dump({"version": 1, "size_mm": MARKER_SIZE_MM,
                       "tags": self.tags}, f, indent=1)
        os.replace(tmp, self.path)      # atomic; a half-written map is worse
        return len(self.tags)

    def get(self, tag_id):
        return self.tags.get(int(tag_id))

    def learn(self, tag_id, x, y):
        """Record where a tag is, averaging repeat sightings.

        A running mean, not the newest reading: each sighting carries a few
        centimetres of noise, and a tag learned from one glance inherits all
        of it permanently — every later fix would then pull the pose toward
        that one bad measurement.
        """
        t = self.tags.get(int(tag_id))
        if t is None:
            self.tags[int(tag_id)] = {"x": x, "y": y, "n": 1, "t": time.time()}
        else:
            n = t["n"] + 1
            t["x"] += (x - t["x"]) / n
            t["y"] += (y - t["y"]) / n
            t["n"] = n
            t["t"] = time.time()
        return self.tags[int(tag_id)]

    def forget(self, tag_id=None):
        if tag_id is None:
            self.tags = {}
        else:
            self.tags.pop(int(tag_id), None)


class MarkerLocator:
    """Watches the camera for tags and turns them into pose fixes.

    Runs in its own thread at a few hertz. Detection on a 640x480 greyscale
    frame costs 8-15 ms on a Pi 4, so this is nearly free next to SLAM — but
    it is paced anyway, because there is no point looking for tags faster
    than the robot can move past them.
    """

    def __init__(self, camera, slam, hz=4.0, learn=False):
        self.camera = camera            # a CameraReader
        self.slam = slam                # a SlamRunner
        self.enabled = True
        self.learn = learn              # record unknown tags from the pose
        self.map = MarkerMap()
        self.error = ""
        self.seen = []                  # what is in view right now
        self.fixes = 0
        self.rejected = 0
        self.last_fix = None
        self.last_reject = ""
        self.ms = 0.0
        self._hz = hz
        self._detect = None
        self._K = self._D = None
        self._obj = None
        try:
            cv2 = _cv2()
            self._cv2 = cv2
            self._detect = _detector(cv2)
            self._obj = _object_points(MARKER_SIZE_MM)
        except MarkerError as e:
            self.error = str(e)
            return
        threading.Thread(target=self._run, daemon=True).start()

    # --- the loop ---------------------------------------------------------

    def _run(self):
        period = 1.0 / self._hz
        while True:
            time.sleep(period)
            if not self.enabled or self.camera is None or self.camera.cam is None:
                continue
            try:
                t0 = time.perf_counter()
                self._tick()
                self.ms = (time.perf_counter() - t0) * 1000.0
            except Exception as e:                            # noqa: BLE001
                self.error = str(e)

    def _tick(self):
        gray = self.camera.cam.gray()
        if gray is None:
            return
        if self._K is None:
            h, w = gray.shape[:2]
            self._K, self._D = intrinsics(w, h)

        corners, ids = self._detect(gray)
        if ids is None or len(ids) == 0:
            self.seen = []
            return

        cv2 = self._cv2
        pose = self.slam.slam.pose
        seen, fixes = [], []
        for quad, tag_id in zip(corners, ids.flatten()):
            ok, rvec, tvec = cv2.solvePnP(
                self._obj, quad.reshape(4, 2).astype("float64"),
                self._K, self._D, flags=cv2.SOLVEPNP_IPPE_SQUARE)
            if not ok:
                continue
            x_cam, _, z_cam = (float(v) for v in tvec.reshape(3))
            xb, yb = camera_to_body(x_cam, z_cam)
            dist = math.hypot(xb, yb)
            rec = {"id": int(tag_id), "dist": round(dist),
                   "bearing": round(math.degrees(math.atan2(yb, xb)), 1),
                   "known": self.map.get(tag_id) is not None,
                   "used": False}

            if dist <= MARKER_MAX_MM:
                known = self.map.get(tag_id)
                if known is not None:
                    fixes.append((tag_id, xb, yb, known, dist))
                    rec["used"] = True
                elif self.learn:
                    # World position of the tag, from where the robot
                    # currently believes it is.
                    c, s = math.cos(pose.th), math.sin(pose.th)
                    self.map.learn(tag_id,
                                   pose.x + xb * c - yb * s,
                                   pose.y + xb * s + yb * c)
                    rec["known"] = True
            seen.append(rec)

        self.seen = seen
        if fixes:
            self._apply(fixes, pose)

    def _apply(self, fixes, pose):
        """Turn one or more sightings into a single position correction."""
        c, s = math.cos(pose.th), math.sin(pose.th)
        xs, ys, wsum = 0.0, 0.0, 0.0
        for _tag, xb, yb, known, dist in fixes:
            # Where the robot must be for this tag to appear where it did,
            # given the heading we already trust.
            fx = known["x"] - (xb * c - yb * s)
            fy = known["y"] - (xb * s + yb * c)
            # Nearer tags weigh more: corner noise turns into range error
            # in proportion to distance squared.
            w = 1.0 / max(1.0, (dist / 1000.0) ** 2)
            xs += fx * w
            ys += fy * w
            wsum += w
        fx, fy = xs / wsum, ys / wsum

        err = math.hypot(fx - pose.x, fy - pose.y)
        if err > MARKER_SANITY_MM:
            # Believing this would teleport the robot across the room. Far
            # more likely: two tags printed with the same id, a map entry
            # learned while the pose was already wrong, or someone moved a
            # tag. Report it; do not act on it.
            self.rejected += 1
            self.last_reject = ("%.0f mm jump from tag %d — duplicate id, or "
                                "a tag that moved" % (err, fixes[0][0]))
            return

        self.slam.apply_fix(fx, fy, MARKER_FIX_GAIN)
        self.fixes += 1
        self.last_fix = {"x": round(fx), "y": round(fy), "err": round(err),
                         "tags": [f[0] for f in fixes], "t": time.time()}

    # --- for the page -----------------------------------------------------

    @property
    def state(self):
        if self._detect is None:
            return {"ok": False, "error": self.error, "tags": 0, "seen": []}
        return {
            "ok": True, "enabled": self.enabled, "learn": self.learn,
            "error": self.error, "tags": len(self.map.tags),
            "seen": self.seen, "fixes": self.fixes,
            "rejected": self.rejected, "last_reject": self.last_reject,
            "last_fix": self.last_fix, "ms": round(self.ms, 1),
            "known": [{"id": k, "x": round(v["x"]), "y": round(v["y"]),
                       "n": v["n"]} for k, v in sorted(self.map.tags.items())],
        }


# --- printing ---------------------------------------------------------------

def sheet_svg(ids, size_mm=None, dict_name=None):
    """A printable SVG of the given tag ids, one per page-width row.

    SVG rather than PNG so the tags come off the printer at exactly
    MARKER_SIZE_MM. A bitmap scaled by a print dialogue will not, and a tag
    whose real size differs from MARKER_SIZE_MM puts every distance
    measurement out by the same ratio — silently.

    The quiet zone matters as much as the tag: aruco needs at least one
    module of white around the black square or detection collapses at angle.
    One full module is included below.
    """
    cv2 = _cv2()
    import numpy as np
    size_mm = MARKER_SIZE_MM if size_mm is None else size_mm
    name = getattr(cv2.aruco, dict_name or MARKER_DICT)
    d = (cv2.aruco.getPredefinedDictionary(name)
         if hasattr(cv2.aruco, "getPredefinedDictionary")
         else cv2.aruco.Dictionary_get(name))

    bits = 6            # 4x4 dictionary plus aruco's own 1-module black border
    quiet = size_mm / bits
    pitch = size_mm + 2 * quiet + 10
    out = ['<svg xmlns="http://www.w3.org/2000/svg" width="%gmm" '
           'height="%gmm" viewBox="0 0 %g %g">' % (pitch, pitch * len(ids),
                                                   pitch, pitch * len(ids))]
    for row, tag_id in enumerate(ids):
        img = (cv2.aruco.generateImageMarker(d, int(tag_id), bits)
               if hasattr(cv2.aruco, "generateImageMarker")
               else cv2.aruco.drawMarker(d, int(tag_id), bits))
        img = np.asarray(img)
        ox, oy = quiet, row * pitch + quiet
        cell = size_mm / bits
        out.append('<rect x="0" y="%g" width="%g" height="%g" fill="#fff"/>'
                   % (row * pitch, pitch, pitch))
        for r in range(bits):
            for c in range(bits):
                if img[r, c] == 0:
                    out.append('<rect x="%g" y="%g" width="%g" height="%g" '
                               'fill="#000"/>' % (ox + c * cell, oy + r * cell,
                                                  cell + .01, cell + .01))
        out.append('<text x="%g" y="%g" font-family="monospace" '
                   'font-size="4" fill="#000">id %d &#183; %gmm</text>'
                   % (ox, oy + size_mm + 6, tag_id, size_mm))
    out.append("</svg>")
    return "\n".join(out)
