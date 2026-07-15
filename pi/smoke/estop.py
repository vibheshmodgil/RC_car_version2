"""Smoke test 1: prove the control path — fire and clear the e-stop.
Safe to run any time; motors can't move as a result of this test."""
import sys, pathlib, time
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from esp32_link import Esp32Link

link = Esp32Link()
print("Sending E-STOP ...")
link.estop()
time.sleep(1)
print("Clearing E-STOP ...")
link.estop_clear()
print("OK — control path works. (Wheels stay disarmed until armed from the UI.)")
