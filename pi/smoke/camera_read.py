"""Smoke test 3: read the CAM stream for 5 s and report resolution + FPS.
Prints numbers instead of showing a window, so it works over SSH."""
import sys, pathlib, time
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from camera import Camera

cam = Camera()
frames = 0
shape = None
t0 = time.time()
while time.time() - t0 < 5.0:
    f = cam.read()
    if f is not None:
        frames += 1
        shape = f.shape
cam.release()
dt = time.time() - t0
print(f"{frames} frames in {dt:.1f}s = {frames/dt:.1f} FPS, shape={shape}")
print("OK" if frames else "FAILED — no frames from the CAM stream.")
