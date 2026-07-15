"""Smoke test 6: center the gimbal, then a gentle +/-30 degree sweep.
Make sure nothing can snag the gimbal before running."""
import sys, pathlib, time
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from gimbal import Gimbal

g = Gimbal()
print("Centering ...")
g.center()
time.sleep(1)
for angle in (60, 120, 90):
    print(f"pan {angle} deg")
    g.pan(angle)
    time.sleep(0.8)
for angle in (60, 120, 90):
    print(f"tilt {angle} deg")
    g.tilt(angle)
    time.sleep(0.8)
g.release()
print("OK — servos released (no holding current).")
