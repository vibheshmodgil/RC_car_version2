"""
Object labels on the map — YOLOv8n, the camera for identity, the LiDAR for range.

Shared library like slam.py and markers.py. web_nav.py runs it; not launched
directly.

    pip install onnxruntime
    # then put yolov8n.onnx next to this file — see "Getting the model" below

What this is for
----------------
The occupancy grid is only ever grey, white and black. It knows a wall is
there; it has no idea it is a wall rather than a sofa. This adds the names:
drive past a room and the map gains "couch", "bed", "dining table" pinned
where those things actually are.

*** Walls do NOT come from this. ***
The LiDAR already gives walls, at 11 Hz, to the centimetre, in the dark. No
vision model competes with that and none is asked to. This adds furniture
labels and nothing else.

Why the camera cannot do it alone
---------------------------------
A detection is a box in an image. It says WHAT and roughly which direction,
and nothing whatever about how far away. A monocular camera has no depth.

So the pairing is the whole design: the camera supplies identity and bearing,
the LiDAR supplies the range at that bearing, and the SLAM pose turns the two
into a world coordinate. Neither sensor can place an object on the map alone.

Voting, not believing
---------------------
One frame is noise. Detectors hallucinate, boxes wander, and a single bad
frame would plant "bed" in the hallway permanently with no way to remove it.
So detections VOTE into a coarse cell and a label is only committed once
COMMIT_VOTES sightings agree. A sofa seen from three angles is a sofa.

The cost, honestly
------------------
A Pi 4 has no accelerator. YOLOv8n at 320x320 costs roughly 250-400 ms per
frame on two threads. That is why this runs at about 1 Hz and skips a cycle
whenever SLAM is over its budget: mapping outranks labelling, always. At 1 Hz
it is roughly a fifth of one core out of four.

Memory is not the constraint people expect — the weights are ~12 MB and the
session is ~150 MB resident. CPU is what you are spending.

Getting the model
-----------------
Export on a laptop, not on the Pi (ultralytics drags in torch):

    pip install ultralytics
    yolo export model=yolov8n.pt format=onnx imgsz=320 opset=12

Optionally shrink and speed it up with dynamic quantisation — no calibration
images needed:

    python -m onnxruntime.quantization.preprocess \\
        --input yolov8n.onnx --output yolov8n-pre.onnx
    python -c "from onnxruntime.quantization import quantize_dynamic, QuantType; \\
        quantize_dynamic('yolov8n-pre.onnx','yolov8n.onnx',weight_type=QuantType.QUInt8)"

Then copy `yolov8n.onnx` into `test/` on the Pi.
"""

import json
import math
import os
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from pins import (  # noqa: E402
    CAM_HFOV, CAM_YAW_OFFSET, CAM_OFFSET_X, CAM_OFFSET_Y, cam_vfov,
)

HERE = os.path.dirname(os.path.abspath(__file__))
MODEL = os.path.join(HERE, "yolov8n.onnx")
STORE = os.path.join(HERE, "objects.json")
SERVER_FILE = os.path.join(HERE, "detect_server.json")

# How long to wait on the PC before giving up on a frame. Generous enough for
# a big model on a busy laptop, short enough that a sleeping one does not stall
# the detection thread for a whole cycle.
REMOTE_TIMEOUT_S = 4.0
# Consecutive failures before falling back to the on-board model. One dropped
# frame is a Wi-Fi hiccup; five in a row is a laptop that has gone to sleep.
REMOTE_FAILS_BEFORE_FALLBACK = 5

# Square input the model was exported at. Must match `imgsz` in the export.
IMGSZ = 320

# Detections below this are not worth the CPU of placing them.
CONF_MIN = 0.40
IOU_MIN = 0.45                 # non-max suppression overlap

# A label is committed to the map after this many agreeing sightings.
COMMIT_VOTES = 3
# Votes land in cells this big, so the same object seen from different
# distances still lands in one place. Furniture is metre-scale; finer than
# this just splits one sofa into three.
CELL_MM = 500.0

# Range sanity. Beyond a few metres the bearing-to-range pairing gets fragile:
# a small bearing error picks a LiDAR return off something else entirely.
MIN_RANGE_MM = 250.0
MAX_RANGE_MM = 4000.0

# How far either side of the box centre to look for a LiDAR return, degrees.
# Wide enough to find one, narrow enough not to grab the wall behind.
RANGE_WINDOW_DEG = 6.0

# COCO, in the order YOLOv8 emits.
COCO = (
    "person bicycle car motorcycle airplane bus train truck boat "
    "traffic_light fire_hydrant stop_sign parking_meter bench bird cat dog "
    "horse sheep cow elephant bear zebra giraffe backpack umbrella handbag "
    "tie suitcase frisbee skis snowboard sports_ball kite baseball_bat "
    "baseball_glove skateboard surfboard tennis_racket bottle wine_glass cup "
    "fork knife spoon bowl banana apple sandwich orange broccoli carrot "
    "hot_dog pizza donut cake chair couch potted_plant bed dining_table "
    "toilet tv laptop mouse remote keyboard cell_phone microwave oven "
    "toaster sink refrigerator book clock vase scissors teddy_bear "
    "hair_drier toothbrush").split()

# What is worth putting on a house map. A robot that maps "person" as a
# permanent fixture has misunderstood the room, and most of COCO is outdoors
# or handheld. Everything else is detected and discarded.
KEEP = {
    "chair", "couch", "potted_plant", "bed", "dining_table", "toilet", "tv",
    "laptop", "microwave", "oven", "sink", "refrigerator", "book", "clock",
    "vase", "teddy_bear", "bench", "backpack", "suitcase", "bottle", "bowl",
}


def load_server_url():
    try:
        with open(SERVER_FILE) as f:
            return json.load(f).get("url", "") or ""
    except (OSError, ValueError):
        return ""


def save_server_url(url):
    tmp = SERVER_FILE + ".tmp"
    with open(tmp, "w") as f:
        json.dump({"url": url or ""}, f)
        f.flush()
        os.fsync(f.fileno())
    os.replace(tmp, SERVER_FILE)
    return url


class DetectError(RuntimeError):
    """Raised when the runtime or the model is missing, with the fix."""


def _session():
    """An ONNX session, or an error saying exactly what to do about it."""
    try:
        import onnxruntime as ort
    except ImportError as e:
        raise DetectError(
            "onnxruntime not installed — pip install onnxruntime "
            "(inside the venv; it is a pure pip package, unlike the "
            "hardware ones)") from e
    if not os.path.exists(MODEL):
        raise DetectError(
            "no model at %s — export one on a laptop with "
            "`yolo export model=yolov8n.pt format=onnx imgsz=%d opset=12` "
            "and copy it here. See the top of detect.py." % (MODEL, IMGSZ))
    so = ort.SessionOptions()
    # Two threads, not four. The other cores belong to SLAM, the control loop
    # and the LiDAR reader; taking them all makes the map worse to make the
    # labels faster, which is the wrong trade.
    so.intra_op_num_threads = 2
    so.inter_op_num_threads = 1
    return ort.InferenceSession(MODEL, so, providers=["CPUExecutionProvider"])


def yuv420_to_rgb(y, u, v):
    """The lores stream is YUV420; the model wants RGB.

    Done here rather than by asking the camera for RGB because the Pi's ISP
    only offers YUV420 for the second stream, and because the same frame is
    already being shared with the marker detector and the cliff check.
    """
    import numpy as np
    h, w = y.shape
    # Nearest-neighbour upsample of the half-size chroma planes. Bilinear
    # would be prettier and would change no detection outcome.
    # int32, not int16. The fixed-point coefficients below are up to 116130,
    # and NumPy 2 REFUSES to put a Python int that large into an int16 array
    # rather than silently wrapping it the way NumPy 1 did:
    #   OverflowError: Python integer 91881 out of bounds for int16
    # The Pi runs numpy 2.2, so this would have thrown on every frame.
    u = np.repeat(np.repeat(u, 2, axis=0), 2, axis=1)[:h, :w].astype(np.int32)
    v = np.repeat(np.repeat(v, 2, axis=0), 2, axis=1)[:h, :w].astype(np.int32)
    yf = y.astype(np.int32)
    u -= 128
    v -= 128
    r = yf + ((91881 * v) >> 16)
    g = yf - ((22554 * u + 46802 * v) >> 16)
    b = yf + ((116130 * u) >> 16)
    return np.clip(np.stack((r, g, b), axis=2), 0, 255).astype(np.uint8)


def rotate_cw(img, deg):
    """Turn the frame the same way the display turns it.

    The model is not rotation invariant. A marker is a square at any angle and
    ArUco does not care, but YOLO has never seen an upside-down living room
    and detects almost nothing in one. Picture rotation is a DISPLAY setting,
    so without this the model keeps seeing the raw sensor orientation - which
    on an inverted mount means it silently finds nothing while the picture on
    screen looks perfectly normal. That is exactly what happened.
    """
    import numpy as np
    k = (int(deg) // 90) % 4
    # np.rot90 turns anticlockwise, the display turns clockwise, hence -k.
    return img if k == 0 else np.ascontiguousarray(np.rot90(img, -k))


def unrotate_box(box, deg):
    """Box fractions in the ROTATED frame -> fractions in the RAW frame.

    The overlay lives inside the element the CSS rotates, so it has to be
    given raw-frame coordinates and be turned along with the picture. Handing
    it rotated coordinates would rotate them twice.
    """
    k = (int(deg) // 90) % 4
    if k == 0:
        return box
    pts = []
    for u, v in ((box[0], box[1]), (box[2], box[3])):
        if k == 1:                       # display turned 90 clockwise
            pts.append((v, 1.0 - u))
        elif k == 2:
            pts.append((1.0 - u, 1.0 - v))
        else:
            pts.append((1.0 - v, u))
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    return (min(xs), min(ys), max(xs), max(ys))


def letterbox(img, size=IMGSZ):
    """Fit into a square without distorting. Returns (chw_float, scale, pad).

    Padded rather than stretched: a stretched sofa is a shape the model has
    never seen, and the geometry to undo it afterwards is the same either way.
    """
    import numpy as np
    h, w = img.shape[:2]
    s = min(size / w, size / h)
    nw, nh = int(round(w * s)), int(round(h * s))
    # Nearest-neighbour resize by indexing — no scipy, no cv2 dependency.
    xi = (np.arange(nw) / s).astype(np.int32).clip(0, w - 1)
    yi = (np.arange(nh) / s).astype(np.int32).clip(0, h - 1)
    small = img[yi][:, xi]
    out = np.full((size, size, 3), 114, dtype=np.uint8)
    px, py = (size - nw) // 2, (size - nh) // 2
    out[py:py + nh, px:px + nw] = small
    chw = out.transpose(2, 0, 1).astype("float32") / 255.0
    return chw[None], s, (px, py)


def nms(boxes, scores, iou_thres=IOU_MIN):
    """Plain greedy non-max suppression. A handful of boxes; nothing clever
    is warranted."""
    import numpy as np
    idx = scores.argsort()[::-1]
    keep = []
    while idx.size:
        i = idx[0]
        keep.append(i)
        if idx.size == 1:
            break
        xx1 = np.maximum(boxes[i, 0], boxes[idx[1:], 0])
        yy1 = np.maximum(boxes[i, 1], boxes[idx[1:], 1])
        xx2 = np.minimum(boxes[i, 2], boxes[idx[1:], 2])
        yy2 = np.minimum(boxes[i, 3], boxes[idx[1:], 3])
        inter = np.clip(xx2 - xx1, 0, None) * np.clip(yy2 - yy1, 0, None)
        a1 = (boxes[i, 2] - boxes[i, 0]) * (boxes[i, 3] - boxes[i, 1])
        a2 = ((boxes[idx[1:], 2] - boxes[idx[1:], 0]) *
              (boxes[idx[1:], 3] - boxes[idx[1:], 1]))
        iou = inter / np.maximum(a1 + a2 - inter, 1e-6)
        idx = idx[1:][iou <= iou_thres]
    return keep


def decode(out, scale, pad, src_w, src_h):
    """YOLOv8 ONNX output -> [(label, conf, cx_fraction, box)].

    The output is (1, 84, N): four box values then eighty class scores, per
    anchor. N depends on the input size and is NOT the 8400 every YOLOv8
    example quotes - that is for 640x640. At the 320x320 this exports at it
    is 2100 (40^2 + 20^2 + 10^2). Verified against the real model. Nothing
    below hardcodes it; the transpose below keys off which axis is longer.

    Only the box CENTRE decides where the object goes on the map - width and
    height say nothing useful once it is a dot on a floor plan. The full box
    comes back anyway so the page can draw it over the picture, which is the
    only way to see whether a label belongs to the thing you think it does.
    Returned as fractions of the source frame, so the overlay does not care
    what resolution the stream is running at.
    """
    import numpy as np
    p = np.squeeze(out[0])
    if p.shape[0] < p.shape[1]:
        p = p.T                                   # (8400, 84)
    cls = p[:, 4:]
    conf = cls.max(axis=1)
    m = conf >= CONF_MIN
    if not m.any():
        return []
    p, conf = p[m], conf[m]
    ids = cls[m].argmax(axis=1)
    cx, cy, bw, bh = p[:, 0], p[:, 1], p[:, 2], p[:, 3]
    boxes = np.stack((cx - bw / 2, cy - bh / 2, cx + bw / 2, cy + bh / 2), axis=1)

    out_list = []
    for i in nms(boxes, conf):
        name = COCO[int(ids[i])] if int(ids[i]) < len(COCO) else "?"
        if name not in KEEP:
            continue
        # Undo the letterbox to get back to the ORIGINAL frame, which is
        # what the bearing is computed from and what the overlay draws on.
        inv = 1.0 / max(scale, 1e-9)
        x = (cx[i] - pad[0]) * inv
        bx0 = (boxes[i, 0] - pad[0]) * inv / src_w
        by0 = (boxes[i, 1] - pad[1]) * inv / src_h
        bx1 = (boxes[i, 2] - pad[0]) * inv / src_w
        by1 = (boxes[i, 3] - pad[1]) * inv / src_h
        clamp = lambda v: max(0.0, min(1.0, float(v)))
        out_list.append((name, float(conf[i]), clamp(x / src_w),
                         (clamp(bx0), clamp(by0), clamp(bx1), clamp(by1))))
    return out_list


def remote_detect(url, jpeg, rotation, conf, timeout=REMOTE_TIMEOUT_S):
    """Send one JPEG to the PC and get boxes back.

    The JPEG is the one the camera's hardware encoder already made for the
    video stream, so this costs the robot the bytes and nothing else — no
    decode, no resize, no colour conversion. About 30 kB a frame.

    Boxes come back as fractions of the ROTATED frame, the same convention the
    on-board path uses, so everything downstream is identical whichever
    backend produced them.
    """
    import urllib.request
    req = urllib.request.Request(
        url, data=jpeg, method="POST",
        headers={"Content-Type": "image/jpeg",
                 "Content-Length": str(len(jpeg)),
                 "X-Rotation": str(int(rotation)),
                 "X-Conf": str(conf)})
    with urllib.request.urlopen(req, timeout=timeout) as r:
        d = json.loads(r.read().decode())
    if not d.get("ok"):
        raise DetectError(d.get("error", "server refused the frame"))
    return d.get("detections", []), d.get("ms", 0.0), d.get("model", "?")


class ObjectMap:
    """Voted labels, in world coordinates, persisted.

    A dict of cell -> {label: votes}. Coarse cells on purpose: the same sofa
    seen from two metres and from four should land in one place, not two.
    """

    def __init__(self, path=STORE):
        self.path = path
        self.cells = {}
        self.load()

    @staticmethod
    def _key(x, y):
        return "%d,%d" % (int(math.floor(x / CELL_MM)), int(math.floor(y / CELL_MM)))

    def vote(self, label, x, y):
        c = self.cells.setdefault(self._key(x, y),
                                  {"x": x, "y": y, "votes": {}, "t": 0.0})
        # Running mean of position, so repeated sightings sharpen the spot
        # rather than the last one winning.
        n = sum(c["votes"].values()) + 1
        c["x"] += (x - c["x"]) / n
        c["y"] += (y - c["y"]) / n
        c["votes"][label] = c["votes"].get(label, 0) + 1
        c["t"] = time.time()
        return c

    def committed(self):
        """Only what has been seen enough times to believe."""
        out = []
        for c in self.cells.values():
            if not c["votes"]:
                continue
            label, n = max(c["votes"].items(), key=lambda kv: kv[1])
            if n >= COMMIT_VOTES:
                out.append({"label": label, "x": round(c["x"]),
                            "y": round(c["y"]), "n": n})
        out.sort(key=lambda d: -d["n"])
        return out

    def forget(self, label=None):
        if label is None:
            self.cells = {}
        else:
            for c in self.cells.values():
                c["votes"].pop(label, None)

    def save(self):
        tmp = self.path + ".tmp"
        with open(tmp, "w") as f:
            json.dump({"version": 1, "cell_mm": CELL_MM, "cells": self.cells}, f)
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp, self.path)
        return len(self.cells)

    def load(self):
        try:
            with open(self.path) as f:
                self.cells = json.load(f).get("cells", {})
        except (OSError, ValueError):
            self.cells = {}
        return len(self.cells)


class Detector:
    """Watches the camera for furniture and pins it to the map.

    Degrades the same way every other sensor here does: if onnxruntime or the
    model is missing it reports why on the page and everything else carries
    on. Nothing refuses to start over a missing label.
    """

    def __init__(self, camera, lidar, slam, hz=1.0, enabled=True, url=None):
        self.camera, self.lidar, self.slam = camera, lidar, slam
        self.enabled = enabled
        self.map = ObjectMap()
        self.error = ""
        self.ms = 0.0
        self.frames = 0
        self.skipped = 0
        self.seen = []                 # what is in view right now
        self.sess = None
        self._hz = hz

        # Off-board detection. A desktop runs a model several sizes larger at
        # twice the input resolution, and the robot has to send a JPEG it had
        # already encoded anyway.
        self.url = url if url is not None else load_server_url()
        self.remote_ok = False
        self.remote_model = ""
        self.remote_ms = 0.0
        self._fails = 0
        # Do not fight SLAM for the CPU. Above this the cycle is skipped.
        self.slam_budget_ms = 140.0
        # The on-board model is loaded even when a server is configured, so
        # a laptop going to sleep degrades to a smaller model rather than to
        # nothing. Missing it is not fatal when there is a server.
        try:
            self.sess = _session()
            self.input = self.sess.get_inputs()[0].name
        except DetectError as e:
            self.error = str(e)
        except Exception as e:                                # noqa: BLE001
            self.error = "onnx session failed: %s" % e
        if self.sess is None and not self.url:
            return                     # nothing to run at all
        if self.url:
            self.error = ""            # the server is the primary path
        threading.Thread(target=self._run, daemon=True).start()

    @property
    def backend(self):
        if self.url and self.remote_ok:
            return "remote"
        if self.url and self.sess:
            return "remote (down, using on-board)"
        if self.url:
            return "remote (down)"
        return "on-board" if self.sess else "none"

    def set_url(self, url):
        self.url = save_server_url((url or "").strip())
        self._fails = 0
        self.remote_ok = False
        self.error = ""
        return self.url

    # --- the loop ---------------------------------------------------------

    def _run(self):
        period = 1.0 / self._hz
        while True:
            time.sleep(period)
            if not self.enabled or self.camera is None or self.camera.cam is None:
                continue
            # Mapping outranks labelling. Always.
            if getattr(self.slam, "ms", 0.0) > self.slam_budget_ms:
                self.skipped += 1
                continue
            try:
                t0 = time.perf_counter()
                self._tick()
                self.ms = (time.perf_counter() - t0) * 1000.0
                self.frames += 1
                self.error = ""
            except Exception as e:                            # noqa: BLE001
                self.error = str(e)

    def _tick(self):
        rot = self.rotation
        dets = self._remote(rot) if self.url else None
        if dets is None:
            dets = self._onboard(rot)
        if dets is None:
            return

        pose = self.slam.slam.pose
        scan = self.lidar.scan() if self.lidar else []
        seen = []
        for label, conf, xf, box in dets:
            bearing = self._bearing(xf, rot)
            box = unrotate_box(box, rot)
            rng = self._range_at(scan, bearing)
            rec = {"label": label, "conf": round(conf, 2),
                   "bearing": round(bearing, 1),
                   "range": None if rng is None else round(rng),
                   "box": [round(v, 4) for v in box],
                   "placed": False}
            if rng is not None:
                wx, wy = self._world(pose, bearing, rng)
                self.map.vote(label, wx, wy)
                rec["placed"] = True
            seen.append(rec)
        self.seen = seen

    def _remote(self, rot):
        """Ask the PC. Returns None if it could not be reached, so the caller
        can fall back rather than simply reporting nothing."""
        jpeg = self.camera.frame()
        if not jpeg:
            return None
        try:
            raw, ms, model = remote_detect(self.url, jpeg, rot, CONF_MIN)
            self.remote_ok = True
            self.remote_ms = ms
            self.remote_model = model
            self._fails = 0
            self.error = ""
            # The server already filtered and normalised; convert to the same
            # shape the on-board decoder produces.
            # NOT re-filtered against KEEP. That set is COCO names, and an
            # open-vocabulary server answers with whatever class list it was
            # given - "wardrobe", "staircase", "washing machine". Filtering
            # here would silently drop exactly the labels that make an
            # off-board model worth running. The server has already filtered.
            return [(d["label"], d["conf"],
                     (d["box"][0] + d["box"][2]) / 2.0, tuple(d["box"]))
                    for d in raw]
        except Exception as e:                                # noqa: BLE001
            self._fails += 1
            self.remote_ok = False
            if self._fails >= REMOTE_FAILS_BEFORE_FALLBACK:
                self.error = ("detection server unreachable (%s) — %s"
                              % (e, "using the on-board model"
                                 if self.sess else "no on-board model either"))
            return None

    def _onboard(self, rot):
        if self.sess is None:
            return None
        y, u, v = self.camera.cam.yuv_planes()
        rgb = rotate_cw(yuv420_to_rgb(y, u, v), rot)
        h, w = rgb.shape[:2]
        blob, scale, pad = letterbox(rgb)
        out = self.sess.run(None, {self.input: blob})
        return decode(out, scale, pad, w, h)

    # --- geometry ---------------------------------------------------------

    def _bearing(self, x_frac, rot=0):
        """Fraction across the frame -> body-frame bearing, degrees.

        Left of frame is +y in the body frame, which is a POSITIVE bearing
        here, so the fraction is mirrored. Getting that backwards puts every
        object on the wrong side of the robot while looking entirely
        plausible on screen.

        At a quarter turn the horizontal axis of the picture is the sensor's
        VERTICAL axis, so the angle it spans is the vertical field of view,
        not the horizontal one. Using the wrong one there would put objects at
        the wrong bearing by the ratio of the two - about 30% on this lens.
        """
        span = CAM_HFOV if (int(rot) // 90) % 2 == 0 else cam_vfov()
        return CAM_YAW_OFFSET + (0.5 - x_frac) * span

    @property
    def rotation(self):
        """The display rotation, read live from web_nav so a change on the
        Vision tab takes effect on the next frame."""
        import sys as _s
        wn = _s.modules.get("web_nav") or _s.modules.get("__main__")
        return int(getattr(wn, "CAM_ROTATION", 0) or 0)

    @staticmethod
    def _range_at(scan, bearing, window=RANGE_WINDOW_DEG):
        """The LiDAR range in that direction. This is the depth the camera
        does not have.

        The NEAREST return in the window, not the mean: a window straddling
        an object's edge sees both the object and the wall behind it, and
        their average is a distance nothing is at.
        """
        if not scan:
            return None
        # The scan is in scanner bearings, which increase clockwise; body
        # bearings here increase anticlockwise. Hence the negation.
        want = (-bearing) % 360.0
        best = None
        for a, d in scan:
            if d < MIN_RANGE_MM or d > MAX_RANGE_MM:
                continue
            diff = abs(((a - want + 180.0) % 360.0) - 180.0)
            if diff <= window and (best is None or d < best):
                best = d
        return best

    @staticmethod
    def _world(pose, bearing, rng):
        """Body-frame bearing and range -> world coordinates."""
        b = math.radians(bearing)
        bx = CAM_OFFSET_X + rng * math.cos(b)
        by = CAM_OFFSET_Y + rng * math.sin(b)
        c, s = math.cos(pose.th), math.sin(pose.th)
        return pose.x + bx * c - by * s, pose.y + bx * s + by * c

    # --- for the page -----------------------------------------------------

    @property
    def state(self):
        if self.sess is None and not self.url:
            return {"ok": False, "error": self.error, "objects": [],
                    "seen": [], "committed": 0, "backend": "none", "url": ""}
        objs = self.map.committed()
        return {
            "ok": True, "enabled": self.enabled, "error": self.error,
            "backend": self.backend, "url": self.url,
            "remote_model": self.remote_model,
            "remote_ms": round(self.remote_ms, 1),
            "ms": round(self.ms, 1), "frames": self.frames,
            "skipped": self.skipped, "seen": self.seen,
            "objects": objs, "committed": len(objs),
            "cells": len(self.map.cells), "votes_needed": COMMIT_VOTES,
        }
