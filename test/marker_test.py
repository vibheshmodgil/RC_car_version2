"""
Marker test — print the tags, then check how far away they can be read.

    cd ~/Desktop/Speaker_truck && source .venv/bin/activate
    python test/marker_test.py --sheet          # printable SVG -> captures/
    python test/marker_test.py                  # live: what is in view, how far
    python test/marker_test.py --map            # what is on the marker map
    python test/marker_test.py --forget all     # wipe the marker map

Setup, once:

    sudo apt install -y python3-opencv

No GPIO, so this is safe with the battery disconnected. It does claim the
camera, so stop web_nav.py first.

Print at 100%
-------------
The sheet is SVG so it lands on paper at exactly MARKER_SIZE_MM. Turn OFF
"fit to page" / "shrink to fit" in the print dialogue. Then MEASURE a printed
tag with a ruler, black square only, and put that number in pins.py. Every
distance the detector reports scales linearly with it, so a tag printed at
92 mm and declared as 100 puts every reading 9% too far away — consistently,
which is exactly the kind of error nothing else in the system will catch.

Where to put them
-----------------
Doorframes and wall corners, at roughly camera height. The point of a tag is
to be seen from a place the robot goes, so a tag near the ceiling is
decorative. One per room plus one per doorway is plenty; the fix comes from
whichever is visible, and they do not need to be visible together.
"""

import argparse
import math
import os
import sys
import time

# Absolute path, so the script works no matter which directory it is
# launched from. __file__.rsplit("/") breaks when run as `python x.py`
# from inside test/, because there is then no "/" to split on.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from camera import Camera, CameraError  # noqa: E402
from markers import (  # noqa: E402
    MarkerError, MarkerMap, _cv2, _detector, _object_points, camera_to_body,
    intrinsics, sheet_svg,
)
from pins import (  # noqa: E402
    CAM_SIZE, CAM_FPS, CAM_HFLIP, CAM_VFLIP,
    MARKER_DICT, MARKER_SIZE_MM, MARKER_MAX_MM,
)

OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "captures")


def do_sheet(ids, path):
    svg = sheet_svg(ids)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w") as f:
        f.write(svg)
    print("\n  Wrote %s" % path)
    print("  %d tags, %s, %.0f mm each" % (len(ids), MARKER_DICT, MARKER_SIZE_MM))
    print("\n  Copy it to your laptop and print at 100%:")
    print("    scp shiv@<pi-ip>:%s ." % path)
    print("\n  Then MEASURE a printed tag (black square only) and set")
    print("  MARKER_SIZE_MM in test/pins.py to what you measured.\n")
    return 0


def do_map(mm_map, forget):
    if forget is not None:
        mm_map.forget(None if forget == "all" else int(forget))
        mm_map.save()
        print("\n  Forgot %s. %d tags left.\n" % (forget, len(mm_map.tags)))
        return 0
    if not mm_map.tags:
        print("\n  No tags on the map yet.")
        print("  Run web_nav.py with --learn-markers and drive past some.\n")
        return 0
    print("\n  %d tag(s), world mm from where SLAM started:" % len(mm_map.tags))
    print("    %-5s %9s %9s %7s" % ("id", "x", "y", "seen"))
    for k, v in sorted(mm_map.tags.items()):
        print("    %-5d %9.0f %9.0f %7d" % (k, v["x"], v["y"], v["n"]))
    print()
    return 0


def do_live(cam, seconds):
    """Stream what the detector sees. This is the range test: walk a tag
    away from the robot and watch where detection stops."""
    cv2 = _cv2()
    detect = _detector(cv2)
    obj = _object_points(MARKER_SIZE_MM)
    K, D = intrinsics(*cam.lores_size)

    print("\n  %s, %.0f mm tags, giving up past %.0f mm"
          % (MARKER_DICT, MARKER_SIZE_MM, MARKER_MAX_MM))
    print("  Walk a tag away from the camera and watch where it drops out.")
    print("  Ctrl-C to stop.\n")
    print("    %-5s %9s %9s %9s %8s" % ("id", "dist", "bearing", "x", "y"))

    t_end = time.monotonic() + seconds if seconds else None
    last = 0.0
    while t_end is None or time.monotonic() < t_end:
        gray = cam.gray()
        if gray is None:
            time.sleep(0.1)
            continue
        corners, ids = detect(gray)
        now = time.monotonic()
        if now - last < 0.4:
            continue
        last = now
        if ids is None or not len(ids):
            print("    (none)                                        ", end="\r")
            continue
        rows = []
        for quad, tag_id in zip(corners, ids.flatten()):
            ok, _rvec, tvec = cv2.solvePnP(
                obj, quad.reshape(4, 2).astype("float64"), K, D,
                flags=cv2.SOLVEPNP_IPPE_SQUARE)
            if not ok:
                continue
            x_cam, _, z_cam = (float(v) for v in tvec.reshape(3))
            xb, yb = camera_to_body(x_cam, z_cam)
            d = math.hypot(xb, yb)
            rows.append("    %-5d %9.0f %8.1f° %9.0f %8.0f%s"
                        % (tag_id, d, math.degrees(math.atan2(yb, xb)),
                           xb, yb, "" if d <= MARKER_MAX_MM else "  (too far)"))
        print("\n".join(rows) + " " * 8)
    return 0


def main():
    ap = argparse.ArgumentParser(description="ArUco marker test")
    ap.add_argument("--sheet", action="store_true",
                    help="write a printable SVG of tags and exit")
    ap.add_argument("--ids", default="0,1,2,3,4,5",
                    help="which tag ids to print (default 0-5)")
    ap.add_argument("--map", action="store_true",
                    help="list the learned marker map and exit")
    ap.add_argument("--forget", help="tag id to drop from the map, or 'all'")
    ap.add_argument("--seconds", type=int, default=0,
                    help="stop after N seconds (default: run until Ctrl-C)")
    args = ap.parse_args()

    try:
        if args.sheet:
            ids = [int(v) for v in args.ids.split(",")]
            return do_sheet(ids, os.path.join(OUT_DIR, "aruco-sheet.svg"))
        if args.map or args.forget is not None:
            return do_map(MarkerMap(), args.forget)
    except MarkerError as e:
        print("\n  %s\n" % e)
        return 1

    try:
        cam = Camera(size=CAM_SIZE, fps=CAM_FPS,
                     hflip=CAM_HFLIP, vflip=CAM_VFLIP)
    except CameraError as e:
        print("\n  %s\n" % e)
        return 1
    except Exception as e:                                    # noqa: BLE001
        print("\n  Camera failed to open: %s" % e)
        print("  If the device is busy, web_nav.py already has it.\n")
        return 1

    # try/finally, per the project convention: the camera is released even on
    # Ctrl-C, or the next script to want it finds the device busy.
    try:
        return do_live(cam, args.seconds)
    except MarkerError as e:
        print("\n  %s\n" % e)
        return 1
    except KeyboardInterrupt:
        print()
        return 0
    finally:
        cam.close()
        print("  Camera released.")


if __name__ == "__main__":
    sys.exit(main())
