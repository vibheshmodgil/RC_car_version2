"""Smoke test 4: print BNO055 orientation + calibration at 2 Hz for 10 s."""
import sys, pathlib, time
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from imu import Imu

imu = Imu()
for _ in range(20):
    h, r, p = imu.euler or (None, None, None)
    print(f"heading={h} roll={r} pitch={p}  cal(sys,gyro,acc,mag)={imu.calibration}")
    time.sleep(0.5)
print("OK — rotate the sensor and watch heading follow.")
