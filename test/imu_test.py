"""
IMU test — identify the chip, stream readings, calibrate the magnetometer.

    cd ~/Desktop/Speaker_truck && source .venv/bin/activate
    python test/imu_test.py                 # identify, then live readings
    python test/imu_test.py --scan          # just list what is on the I2C bus
    python test/imu_test.py --calibrate     # magnetometer hard-iron offsets
    python test/imu_test.py --gyro-bias     # gyro zero-rate offsets

Setup, once:

    sudo apt install -y i2c-tools python3-smbus2
    sudo raspi-config      # Interface Options -> I2C -> enable, then reboot
    i2cdetect -y 1

Nothing here drives a motor, so it is safe to run with the battery
disconnected — the IMU is powered from 3.3 V.

Which calibration matters
-------------------------
GYRO BIAS is quick and worth doing every session: a gyro reads a small
non-zero rate when perfectly still, and integrating that is what makes
heading crawl. Takes 5 seconds, robot motionless.

MAGNETOMETER is the one that actually decides whether heading is usable. The
truck carries four motors and a speaker magnet, and that steady local field
(hard-iron) shifts the magnetometer's centre away from zero. Uncorrected,
heading can be tens of degrees out and wrong by a different amount depending
which way you face. Run it once, with the robot fully assembled — calibrating
a bare board and then bolting it into the truck measures the wrong thing.
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

from imu import (  # noqa: E402
    IMU, I2C_BUS, CALIB_FILE, SMBus, load_calib, save_calib,
    BNO055_ADDRS, MPU_ADDRS, MAG_ADDR,
)

KNOWN = {
    0x28: "BNO055 (default)", 0x29: "BNO055 (alt address)",
    0x68: "MPU-6050/6500/9250 or ICM-20948", 0x69: "same family, alt address",
    0x0C: "AK8963 / AK09916 magnetometer (visible once bypass is on)",
    0x1E: "HMC5883L / LSM303 magnetometer",
    0x0D: "QMC5883L magnetometer",
    0x76: "BMP280 / BME280 barometer", 0x77: "BMP180 / BMP280 barometer",
    0x3C: "SSD1306 OLED", 0x40: "INA219 current sensor",
}


def scan_bus(bus_no=I2C_BUS):
    if SMBus is None:
        print("python3-smbus2 not installed.")
        print("  sudo apt install -y i2c-tools python3-smbus2")
        return []
    try:
        bus = SMBus(bus_no)
    except OSError as e:
        print(f"cannot open i2c-{bus_no}: {e}")
        print("  sudo raspi-config -> Interface Options -> I2C -> enable")
        return []

    found = []
    for addr in range(0x03, 0x78):
        try:
            bus.read_byte(addr)
            found.append(addr)
        except OSError:
            pass
    bus.close()

    print(f"\n  I2C bus {bus_no}")
    if not found:
        print("    nothing responded.")
        print("    - is I2C enabled?  sudo raspi-config")
        print("    - SDA on GPIO2 (pin 3), SCL on GPIO3 (pin 5)?")
        print("    - VCC on 3.3 V, and grounds common?")
        print("    - most breakouts have pull-ups; a bare chip needs 4.7k")
    for a in found:
        print(f"    0x{a:02X}  {KNOWN.get(a, 'unknown device')}")
    return found


def show_identity(imu):
    print(f"\n  chip     {imu.name}")
    print(f"  address  0x{imu.addr:02X}")
    print(f"  fusion   {'on-chip' if imu.fused else 'complementary filter here'}")
    mode = getattr(imu.dev, "mode", None)
    if mode == 0x08:
        print("  mode     IMUPLUS - gyro+accel, magnetometer OFF by design")
        print("           heading is RELATIVE, which is all SLAM needs")
    elif mode is not None:
        print(f"  mode     0x{mode:02X}")
    print(f"  mag      {'yes' if imu.has_mag else 'not used in this mode'}")
    if not imu.fused:
        c = imu.calib
        cal = any(abs(v) > 1e-6 for v in c["mag_offset"])
        print(f"  calib    {'loaded from ' + CALIB_FILE if cal else 'NONE — run --calibrate'}")


def stream(imu, hz=20):
    print("\n  Ctrl-C to stop. Tilt and rotate the board and watch it follow.\n")
    print(f"  {'roll':>8}{'pitch':>8}{'yaw':>8}   "
          f"{'ax':>7}{'ay':>7}{'az':>7}   {'gx':>7}{'gy':>7}{'gz':>7}  head")
    period = 1.0 / hz
    try:
        while True:
            d = imu.read()
            a, g = d["accel"], d["gyro"]
            print(f"\r  {d['roll']:>8.2f}{d['pitch']:>8.2f}{d['yaw']:>8.2f}   "
                  f"{a[0]:>7.2f}{a[1]:>7.2f}{a[2]:>7.2f}   "
                  f"{g[0]:>7.1f}{g[1]:>7.1f}{g[2]:>7.1f}  "
                  f"{'ok ' if d['heading_ok'] else 'DRIFT'}", end="", flush=True)
            time.sleep(period)
    except KeyboardInterrupt:
        print("\n")


def calibrate_mag(imu, seconds=30):
    """Hard-iron offsets from the centre of the swept min/max box.

    Rotating through every orientation traces a sphere in magnetometer space.
    A clean sensor centres that sphere on the origin; nearby iron and magnets
    push it off-centre, and the offset is that displacement. Scale factors
    correct the softer, axis-dependent distortion at the same time.
    """
    if not imu.has_mag:
        print("\n  This chip has no magnetometer — nothing to calibrate.")
        return
    if imu.fused:
        print("\n  The BNO055 calibrates itself. Rotate it slowly through all")
        print("  orientations and watch the calib figures reach 3.\n")
        try:
            while True:
                c = imu.read()["calib"]
                print(f"\r  sys {c['sys']}  gyro {c['gyro']}  "
                      f"accel {c['accel']}  mag {c['mag']}   "
                      f"(3 = fully calibrated)", end="", flush=True)
                if c["mag"] == 3 and c["sys"] == 3:
                    print("\n\n  Calibrated.\n")
                    return
                time.sleep(0.25)
        except KeyboardInterrupt:
            print("\n")
            return

    print(f"\n  Rotate the ROBOT — fully assembled, not the bare board —")
    print(f"  slowly through every orientation for {seconds} s.")
    print("  Figure-eights, then tip it onto each face. Keep it away from")
    print("  desks with steel frames.\n")
    input("  Enter to start: ")

    lo = [1e9] * 3
    hi = [-1e9] * 3
    end = time.monotonic() + seconds
    try:
        while time.monotonic() < end:
            raw = imu.dev.read_raw()[2]
            if raw:
                for i in range(3):
                    lo[i] = min(lo[i], raw[i])
                    hi[i] = max(hi[i], raw[i])
                left = end - time.monotonic()
                print(f"\r  {left:4.1f}s   "
                      f"x[{lo[0]:7.1f},{hi[0]:7.1f}] "
                      f"y[{lo[1]:7.1f},{hi[1]:7.1f}] "
                      f"z[{lo[2]:7.1f},{hi[2]:7.1f}]", end="", flush=True)
            time.sleep(0.02)
    except KeyboardInterrupt:
        pass
    print()

    if any(h - l < 5 for l, h in zip(lo, hi)):
        print("\n  One axis barely moved — the sweep did not cover enough")
        print("  orientations. Not saving. Try again, tipping onto every face.")
        return

    offset = [(hi[i] + lo[i]) / 2 for i in range(3)]
    span = [(hi[i] - lo[i]) / 2 for i in range(3)]
    avg = sum(span) / 3
    scale = [avg / s if s > 1e-6 else 1.0 for s in span]

    c = load_calib()
    c["mag_offset"], c["mag_scale"] = offset, scale
    ok = save_calib(c)
    print(f"\n  offset  {[round(v, 2) for v in offset]}")
    print(f"  scale   {[round(v, 3) for v in scale]}")
    print(f"  {'saved to ' + CALIB_FILE if ok else 'COULD NOT SAVE'}\n")


def calibrate_gyro(imu, seconds=5):
    print(f"\n  Hold the robot completely still for {seconds} s.\n")
    input("  Enter to start: ")
    n, acc = 0, [0.0, 0.0, 0.0]
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        if imu.fused:
            g = imu.read()["gyro"]
        else:
            g = imu.dev.read_raw()[1]
        for i in range(3):
            acc[i] += g[i]
        n += 1
        time.sleep(0.01)
    bias = [v / max(1, n) for v in acc]
    if max(abs(b) for b in bias) > 20:
        print(f"\n  Bias looks too large ({bias}) — was it moving? Not saving.\n")
        return
    c = load_calib()
    c["gyro_bias"] = bias
    save_calib(c)
    print(f"\n  gyro bias {[round(b, 3) for b in bias]} dps  ->  saved\n")


def main():
    ap = argparse.ArgumentParser(description="9-axis IMU test")
    ap.add_argument("--scan", action="store_true", help="list I2C devices and exit")
    ap.add_argument("--calibrate", action="store_true", help="magnetometer offsets")
    ap.add_argument("--gyro-bias", action="store_true", help="gyro zero-rate offsets")
    ap.add_argument("--seconds", type=int, default=30, help="calibration duration")
    args = ap.parse_args()

    if args.scan:
        scan_bus()
        return

    scan_bus()
    try:
        imu = IMU()
    except RuntimeError as e:
        print(f"\n  {e}\n")
        print("  Expected addresses: BNO055 0x28/0x29, "
              "MPU/ICM 0x68/0x69. See WIRING.md section 11.\n")
        return

    show_identity(imu)
    try:
        if args.calibrate:
            calibrate_mag(imu, args.seconds)
        elif args.gyro_bias:
            calibrate_gyro(imu)
        else:
            stream(imu)
    finally:
        imu.close()


if __name__ == "__main__":
    main()
