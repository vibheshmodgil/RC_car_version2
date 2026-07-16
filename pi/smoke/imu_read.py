"""Smoke test 4: print BNO055 orientation + calibration at 2 Hz for 10 s."""
import sys, pathlib, time
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from imu import Imu

imu = Imu()
for _ in range(20):
    r = imu.reading
    if r.get("ok"):
        print(f"heading={r['h']:.1f} roll={r['r']:.1f} pitch={r['p']:.1f}  "
              f"cal(sys,gyro,acc,mag)={r['cal']}")
    else:
        print("waiting for a valid reading...")
    time.sleep(0.5)
print("OK — rotate the sensor and watch heading follow.")
