"""
Camera test — identify the module, measure the frame rate, aim it.

    cd ~/Desktop/Speaker_truck && source .venv/bin/activate
    python test/camera_test.py              # identify + 5 s of frame timing
    python test/camera_test.py --list       # just say what is attached, exit
    python test/camera_test.py --still      # save one JPEG and exit
    python test/camera_test.py --stream     # live preview page on :5005
    python test/camera_test.py --flip       # upside-down mount (180 degrees)

Setup, once:

    sudo apt install -y python3-picamera2
    rpicam-hello --list-cameras

Nothing here touches GPIO, so it is safe with the battery disconnected and it
runs happily ALONGSIDE web_nav.py — CSI is not a shared resource the way the
motor pins are. The one thing it cannot share is the camera itself: if
web_nav.py already has it open, this will fail to acquire it, and vice versa.

Use --stream to aim the camera before trusting the cockpit view. Getting the
mounting right is easier with a full-window picture than with the small panel
on the nav page.
"""

import argparse
import os
import sys
import time

# Absolute path, so the script works no matter which directory it is
# launched from. __file__.rsplit("/") breaks when run as `python x.py`
# from inside test/, because there is then no "/" to split on.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from camera import (  # noqa: E402
    Camera, CameraError, MJPEG_MIME, detect, mjpeg_stream, _friendly,
)
from pins import (  # noqa: E402
    CAM_SIZE, CAM_FPS, CAM_HFLIP, CAM_VFLIP,
)

HTTP_PORT = 5005
STILL_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "captures")


PAGE = """<!DOCTYPE html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Speaker Truck &mdash; Camera</title>
<style>
  body{background:#0d1117;color:#fff;margin:0;padding:20px;
       font-family:ui-sans-serif,-apple-system,"Segoe UI",Roboto,sans-serif;
       max-width:1000px;margin:0 auto}
  h1{font-size:.95rem;font-weight:650;letter-spacing:.14em;text-transform:uppercase;
     padding-bottom:14px;border-bottom:1px solid #2a323d;margin-bottom:16px}
  h1 span{color:#6e7b8a;font-weight:400;text-transform:none;letter-spacing:0}
  img{width:100%;display:block;border-radius:10px;background:#161b22}
  p{color:#a9b4c0;font-size:.78rem;line-height:1.65;margin-top:14px}
  b{color:#fff}
</style></head><body>
<h1>Speaker Truck <span>/ %%NAME%% &middot; %%W%%&times;%%H%% @ %%FPS%% fps</span></h1>
<img src="/camera.mjpg" alt="camera">
<p>Aim the camera now, before it goes in the cockpit. If the picture is
upside down, restart with <b>--flip</b>, then make it permanent by setting
<b>CAM_VFLIP</b> and <b>CAM_HFLIP</b> in <b>test/pins.py</b> &mdash; the nav
page reads the same two values, so it only has to be got right once.</p>
</body></html>"""


def show_list():
    """Print what is attached. Separates 'no camera' from 'no picamera2',
    because the fix is completely different."""
    try:
        cams = detect()
    except CameraError as e:
        print("\n  %s\n" % e)
        return 1
    if not cams:
        print("\n  No CSI camera detected.")
        print("  - Pi powered down before plugging the ribbon in?")
        print("  - Ribbon fully seated at BOTH ends, contacts the right way?")
        print("  - `rpicam-hello --list-cameras` says the same thing?\n")
        return 1
    print("\n  %d camera(s):" % len(cams))
    for i, c in enumerate(cams):
        print("    [%d] %s" % (i, _friendly(c.get("Model"))))
        print("        id       %s" % c.get("Id", "?"))
        if c.get("Rotation") is not None:
            print("        rotation %s deg (as wired in the device tree)"
                  % c.get("Rotation"))
    print()
    return 0


def timing(cam, seconds):
    """Frame timing. The number that matters is not the average but the
    worst gap: a stream that averages 15 fps while occasionally stalling for
    400 ms is a stream you cannot drive on."""
    # Wait for the stream to actually start before timing anything.
    #
    # The ISP takes a few hundred milliseconds to bring up auto-exposure and
    # white balance, and timing across that produced a "worst gap 866 ms"
    # warning on a camera running at a perfectly steady 14.9 fps. Measuring
    # from the first real frame is the fix: the settle is startup, not a stall.
    t0 = time.monotonic()
    while cam.sink.count == 0 and time.monotonic() - t0 < 3.0:
        time.sleep(0.02)

    print("\n  Measuring for %d s ..." % seconds)
    t_end = time.monotonic() + seconds
    last_n, last_t = cam.sink.count, time.monotonic()
    gaps = []
    while time.monotonic() < t_end:
        time.sleep(0.02)
        n = cam.sink.count
        if n > last_n:
            now = time.monotonic()
            gaps.append(now - last_t)
            last_t, last_n = now, n

    if not gaps:
        print("\n  *** No frames arrived at all.")
        print("  The camera opened but produced nothing — usually a ribbon")
        print("  that is seated well enough to enumerate but not to stream.\n")
        return 1

    avg = sum(gaps) / len(gaps)
    print("\n  frames    %d" % len(gaps))
    print("  average   %.1f fps  (asked for %d)" % (1.0 / avg, cam.fps))
    print("  best gap  %.0f ms" % (min(gaps) * 1000))
    print("  worst gap %.0f ms" % (max(gaps) * 1000))
    jpeg = cam.frame()
    print("  jpeg size %.0f kB" % (len(jpeg) / 1024.0) if jpeg else "  jpeg  —")

    # A ratio alone cries wolf. A steady stream has a tiny average, so any
    # single hiccup is "4x the average" while being far too short to notice.
    # The gap has to be BOTH disproportionate and long enough to see.
    if max(gaps) > 4 * avg and max(gaps) > 0.25:
        print("\n  ! The worst gap is far above average and long enough to")
        print("    see. Something else on the Pi is stealing CPU — check")
        print("    whether web_nav.py is running.")
    print()
    return 0


def save_still(cam):
    os.makedirs(STILL_DIR, exist_ok=True)
    path = os.path.join(STILL_DIR, time.strftime("cam-%Y%m%d-%H%M%S.jpg"))
    # Give the ISP a moment: the first frame or two come out before
    # auto-exposure and white balance have settled, and saving one of those
    # produces a dark green photo that looks like a hardware fault.
    time.sleep(1.5)
    if cam.still(path) is None:
        print("\n  No frame to save — the camera opened but never delivered.\n")
        return 1
    print("\n  Saved %s  (%.0f kB)\n" % (path, os.path.getsize(path) / 1024.0))
    return 0


def serve(cam, port):
    from flask import Flask, Response

    app = Flask(__name__)
    page = (PAGE.replace("%%NAME%%", cam.name)
                .replace("%%W%%", str(cam.size[0]))
                .replace("%%H%%", str(cam.size[1]))
                .replace("%%FPS%%", str(cam.fps)))

    @app.route("/")
    def index():
        return page

    @app.route("/camera.mjpg")
    def stream():
        return Response(mjpeg_stream(cam), mimetype=MJPEG_MIME)

    import socket
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("192.168.1.1", 1))
        ip = s.getsockname()[0]
    except OSError:
        ip = "127.0.0.1"
    finally:
        s.close()

    print("\n  Speaker Truck — camera preview")
    print("  http://%s:%d\n" % (ip, port))
    app.run(host="0.0.0.0", port=port, threaded=True)
    return 0


def main():
    ap = argparse.ArgumentParser(description="CSI camera test")
    ap.add_argument("--list", action="store_true",
                    help="list attached cameras and exit")
    ap.add_argument("--still", action="store_true",
                    help="save one JPEG to test/captures/ and exit")
    ap.add_argument("--stream", action="store_true",
                    help="serve a live preview page on :%d" % HTTP_PORT)
    ap.add_argument("--flip", action="store_true",
                    help="180 degrees, for an upside-down mount")
    ap.add_argument("--size", default="%dx%d" % CAM_SIZE,
                    help="WxH, default %dx%d" % CAM_SIZE)
    ap.add_argument("--fps", type=int, default=CAM_FPS)
    ap.add_argument("--seconds", type=int, default=5,
                    help="how long to measure frame timing")
    ap.add_argument("--port", type=int, default=HTTP_PORT)
    args = ap.parse_args()

    if args.list:
        return show_list()

    try:
        w, h = (int(v) for v in args.size.lower().split("x"))
    except ValueError:
        print("\n  --size wants WxH, e.g. 1280x720\n")
        return 2

    hflip = CAM_HFLIP or args.flip
    vflip = CAM_VFLIP or args.flip

    try:
        cam = Camera(size=(w, h), fps=args.fps, hflip=hflip, vflip=vflip)
    except CameraError as e:
        print("\n  %s\n" % e)
        return 1
    except Exception as e:                                    # noqa: BLE001
        print("\n  Camera failed to open: %s" % e)
        print("  If this says the device is busy, web_nav.py already has it.\n")
        return 1

    print("\n  %s" % cam.name)
    print("  %dx%d @ %d fps, hflip=%s vflip=%s"
          % (w, h, args.fps, hflip, vflip))
    print("  encoder %s" % cam.encoder)
    if cam.error:
        print("  ! %s" % cam.error)

    # try/finally, per the project convention: the camera is released even
    # on Ctrl-C, or the next script to want it finds the device busy.
    try:
        if args.stream:
            return serve(cam, args.port)
        if args.still:
            return save_still(cam)
        return timing(cam, args.seconds)
    except KeyboardInterrupt:
        print()
        return 0
    finally:
        cam.close()
        print("  Camera released.")


if __name__ == "__main__":
    sys.exit(main())
