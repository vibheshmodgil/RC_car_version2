"""
Detection server — runs on your PC, not the robot.

    python tools/detect_server.py                 # open-vocab, house classes
    python tools/detect_server.py --device cuda   # if you have an NVIDIA GPU
    python tools/detect_server.py --model yolo11x.pt --classes ""   # plain COCO

Then point the robot at it, on the Pi:

    python test/web_nav.py --detect-url http://<your-pc-ip>:8000/detect

or type the URL into the Objects panel on the Vision tab, which persists it.

Why offload at all
------------------
A Pi 4 has no accelerator. YOLOv8n at 320x320 costs ~610 ms there and still
misses a lot, because n is the smallest model there is and 320 px is a small
input. A desktop runs a model several sizes larger at twice the resolution in
a fraction of that time — the robot has the sensors, the PC has the compute,
and a LAN is enormously cheaper than the gap between them.

The frame the Pi sends is the JPEG the camera's hardware encoder already
produced for the video stream, so sending it costs the robot nothing beyond
the bytes. At 1-2 Hz that is ~30-60 kB/s, against the ~2.3 Mbit/s the MJPEG
preview already uses.

Open vocabulary, and why it matters more than model size
--------------------------------------------------------
A bigger YOLO detects COCO's eighty classes more accurately. That is not the
limitation in a house: COCO simply has no wardrobe, bookshelf, desk, lamp,
washing machine, curtain, door, stairs, radiator or rug. A perfect COCO
detector still cannot name most of a bedroom.

So the default here is YOLO-World, which is told its classes at runtime. The
list below is what a house actually contains; edit it, or pass --classes, and
the model looks for those instead. No retraining, no dataset.

Pass --classes "" to fall back to fixed COCO labels with an ordinary model.

What this deliberately does not do
----------------------------------
It does not place anything on the map. It returns boxes and labels; the robot
pairs them with LiDAR range and its own pose, because only the robot knows
where it was standing. Keeping the geometry on the robot means the server is
stateless and can be restarted, moved or replaced without the map noticing.

If this server is unreachable the robot falls back to its on-board model, or
to nothing, and says which on the page. A laptop that goes to sleep must not
take the robot's mapping with it.
"""

import argparse
import io
import json
import sys
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

try:                                    # task-manager numbers for the cockpit's System tab
    import sysstats
    SAMPLER = sysstats.Sampler()
except ImportError:                     # run outside the image without test/ on the path
    SAMPLER = None

MODEL = None
PERSON_MODEL = None     # plain COCO, class 0 only — see /person below
ARGS = None
STATS = {"frames": 0, "ms": 0.0, "started": time.time()}
PERSON_STATS = {"frames": 0, "ms": 0.0}

# What an open-vocabulary model is asked to look for. This is the whole point
# of using one: the list is a runtime argument, not a property of the weights,
# so it can describe a house rather than the COCO dataset.
#
# Kept to things that are (a) furniture-scale, so a LiDAR return at that
# bearing is actually the object rather than the wall behind it, and (b)
# roughly static, so pinning them to a map means something. A cat is
# detectable and useless to map.
HOUSE_CLASSES = [
    "sofa", "armchair", "chair", "stool", "bed", "mattress",
    "dining table", "coffee table", "desk", "cabinet", "wardrobe",
    "chest of drawers", "bookshelf", "shelf", "television", "computer monitor",
    "refrigerator", "washing machine", "microwave", "oven", "stove",
    "kitchen sink", "toilet", "bathtub", "shower", "door", "doorway",
    "staircase", "window", "curtain", "rug", "lamp", "floor lamp",
    "ceiling fan", "radiator", "potted plant", "mirror", "picture frame",
    "trash can", "laundry basket", "suitcase", "shoe rack", "piano",
]

# Only used when running a fixed-vocabulary model (--classes ""). COCO names,
# matching KEEP in test/detect.py.
COCO_KEEP = {
    "chair", "couch", "potted plant", "bed", "dining table", "toilet", "tv",
    "laptop", "microwave", "oven", "sink", "refrigerator", "book", "clock",
    "vase", "teddy bear", "bench", "backpack", "suitcase", "bottle", "bowl",
}
KEEP = COCO_KEEP        # replaced at startup when open-vocabulary is in use


def load_model(name, device, classes):
    """Load, and if it is a -world model, tell it what to look for.

    set_classes() re-encodes the class names with the model's text encoder,
    which is what lets an open-vocabulary detector answer for words it was
    never trained on. It costs a second at startup and nothing per frame.
    """
    global KEEP
    from ultralytics import YOLO
    m = YOLO(name)
    if classes:
        if not hasattr(m, "set_classes"):
            raise SystemExit(
                "\n  %s is a fixed-vocabulary model, so --classes does nothing."
                "\n  Use a -world model (the default), or pass --classes \"\""
                "\n  to run this one on plain COCO labels.\n" % name)
        m.set_classes(classes)
        # The class list IS the filter now; nothing else can come back.
        KEEP = set(classes)
    if device:
        m.to(device)
    return m


def infer(jpeg, rotation, conf, imgsz, model=None, keep=None, stats=None, classes=None):
    """JPEG bytes -> [{label, conf, box}] with box as fractions 0-1.

    Boxes come back in the ROTATED frame — the same frame the robot computes
    bearings from. The robot maps them back for its overlay.

    model/keep/stats default to the furniture detector; /person passes its own.
    """
    model = MODEL if model is None else model
    keep = KEEP if keep is None else keep
    stats = STATS if stats is None else stats
    from PIL import Image
    img = Image.open(io.BytesIO(jpeg)).convert("RGB")
    # Rotate to match the robot's display setting, so the model sees the room
    # the right way up. PIL rotates anticlockwise, the display clockwise.
    r = int(rotation) % 360
    if r:
        img = img.rotate(-r, expand=True)
    w, h = img.size

    t0 = time.perf_counter()
    res = model.predict(img, conf=conf, imgsz=imgsz, classes=classes, verbose=False)[0]
    ms = (time.perf_counter() - t0) * 1000.0

    out = []
    names = res.names
    for b in res.boxes:
        label = names[int(b.cls[0])]
        if label not in keep:
            continue
        x0, y0, x1, y1 = (float(v) for v in b.xyxy[0])
        out.append({
            "label": label.replace(" ", "_"),
            "conf": round(float(b.conf[0]), 3),
            "box": [round(max(0.0, min(1.0, x0 / w)), 4),
                    round(max(0.0, min(1.0, y0 / h)), 4),
                    round(max(0.0, min(1.0, x1 / w)), 4),
                    round(max(0.0, min(1.0, y1 / h)), 4)],
        })
    stats["frames"] += 1
    stats["ms"] = ms
    return out, ms, (w, h)


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, *a):
        pass                       # one line per frame at 1 Hz is just noise

    def _send(self, code, payload):
        body = json.dumps(payload).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        # A cheap health check, so the robot can tell "server down" from
        # "server up but found nothing" — they need completely different fixes.
        self._send(200, {"ok": True, "model": ARGS.model, "classes": sorted(KEEP),
                         "device": ARGS.device or "cpu",
                         "imgsz": ARGS.imgsz, "frames": STATS["frames"],
                         "last_ms": round(STATS["ms"], 1),
                         "person_model": ARGS.person_model or None,
                         "person_frames": PERSON_STATS["frames"],
                         "person_last_ms": round(PERSON_STATS["ms"], 1),
                         "uptime_s": int(time.time() - STATS["started"]),
                         "stats": SAMPLER.sample() if SAMPLER else None})

    def do_POST(self):
        try:
            n = int(self.headers.get("Content-Length", 0))
            if not n:
                return self._send(400, {"ok": False, "error": "empty body"})
            jpeg = self.rfile.read(n)
            rot = float(self.headers.get("X-Rotation", 0) or 0)
            conf = float(self.headers.get("X-Conf", ARGS.conf) or ARGS.conf)
            if self.path.rstrip("/").endswith("/person"):
                # The follow tracker: every frame it can get, people only.
                # Not printed — at 5 Hz that would bury the furniture lines.
                if PERSON_MODEL is None:
                    return self._send(404, {"ok": False, "error": "started with --person-model \"\""})
                dets, ms, size = infer(jpeg, rot, conf, ARGS.person_imgsz, PERSON_MODEL,
                                       {"person"}, PERSON_STATS, classes=[0])
                return self._send(200, {"ok": True, "detections": dets,
                                        "ms": round(ms, 1), "size": list(size),
                                        "model": ARGS.person_model})
            dets, ms, size = infer(jpeg, rot, conf, ARGS.imgsz)
            print("  %4d  %5.0f ms  %s"
                  % (STATS["frames"], ms,
                     ", ".join("%s %.2f" % (d["label"], d["conf"]) for d in dets)
                     or "nothing"))
            self._send(200, {"ok": True, "detections": dets,
                             "ms": round(ms, 1), "size": list(size),
                             "model": ARGS.model})
        except Exception as e:                                # noqa: BLE001
            self._send(500, {"ok": False, "error": "%s: %s"
                             % (type(e).__name__, e)})


def main():
    global MODEL, PERSON_MODEL, ARGS
    ap = argparse.ArgumentParser(description="Off-board detection for the robot")
    # yolov8m-worldv2, not x. Measured on a CPU-only desktop at 640 px:
    #   s   220 ms      m   596 ms      x  1682 ms
    # The robot asks about once a second, so x cannot keep up and every frame
    # arrives late. m is the largest that comfortably fits the cycle and is
    # already several classes better than anything the Pi can run.
    # With a GPU, --model yolov8x-worldv2.pt is worth it.
    ap.add_argument("--model", default="yolov8m-worldv2.pt",
                    help="open-vocabulary by default. Use yolov8x-worldv2.pt "
                         "if you have a GPU, or yolo11x.pt with --classes \"\" "
                         "for a fixed COCO model.")
    ap.add_argument("--classes", default=",".join(HOUSE_CLASSES),
                    help="comma-separated things to look for. Needs a -world "
                         "model. Pass an empty string for plain COCO.")
    ap.add_argument("--imgsz", type=int, default=640,
                    help="640 finds far more small furniture than the 320 the "
                         "Pi can afford")
    ap.add_argument("--conf", type=float, default=0.25,
                    help="lower than the on-board default on purpose: a "
                         "bigger model is right more often, and the "
                         "three-sighting vote still filters the noise")
    ap.add_argument("--device", default=None,
                    help="cuda / mps / cpu. Left alone, the best available "
                         "is chosen automatically.")
    # A plain COCO model for /person, the follow tracker. COCO's "person" is
    # the most-trained class there is, and a fixed-vocabulary s model at 416
    # px is quick enough on a laptop CPU for several frames a second.
    ap.add_argument("--person-model", default="yolov8s.pt",
                    help="COCO model for /person (people only). \"\" disables it.")
    ap.add_argument("--person-imgsz", type=int, default=416)
    ap.add_argument("--port", type=int, default=8000)
    ap.add_argument("--host", default="0.0.0.0")
    ARGS = ap.parse_args()

    # Pick the fastest thing present rather than making it a flag nobody
    # remembers. A GPU is ten times a CPU here and there is no reason to make
    # anyone ask for it.
    if ARGS.device is None:
        try:
            import torch
            ARGS.device = ("cuda" if torch.cuda.is_available() else
                           "mps" if getattr(torch.backends, "mps", None)
                           and torch.backends.mps.is_available() else "cpu")
        except Exception:                                     # noqa: BLE001
            ARGS.device = "cpu"

    classes = [c.strip() for c in ARGS.classes.split(",") if c.strip()]
    print("\n  loading %s ..." % ARGS.model)
    MODEL = load_model(ARGS.model, ARGS.device, classes)
    if ARGS.person_model:
        print("  loading %s for /person ..." % ARGS.person_model)
        from ultralytics import YOLO
        PERSON_MODEL = YOLO(ARGS.person_model)
        if ARGS.device:
            PERSON_MODEL.to(ARGS.device)
    if classes:
        print("  open vocabulary, %d classes: %s%s"
              % (len(classes), ", ".join(classes[:8]),
                 " ..." if len(classes) > 8 else ""))
    else:
        print("  fixed vocabulary (COCO), %d kept" % len(KEEP))

    import socket
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("192.168.1.1", 1))
        ip = s.getsockname()[0]
    except OSError:
        ip = "127.0.0.1"
    finally:
        s.close()

    print("  ready on http://%s:%d   (%s)" % (ip, ARGS.port, ARGS.device))
    print("\n  On the Pi:")
    print("    python test/web_nav.py --detect-url http://%s:%d/detect"
          % (ip, ARGS.port))
    print("\n  Leave this window open. Frames appear below as they arrive.\n")
    ThreadingHTTPServer((ARGS.host, ARGS.port), Handler).serve_forever()


if __name__ == "__main__":
    sys.exit(main())
