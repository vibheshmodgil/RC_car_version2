"""
9-axis IMU driver — detects the chip on I2C, so no model has to be assumed.

Shared library, like pins.py. Imported by imu_test.py and web_nav.py; not
meant to be run directly.

    sudo apt install -y i2c-tools python3-smbus2
    sudo raspi-config      # Interface Options -> I2C -> enable
    i2cdetect -y 1         # the chip should appear at 0x28/0x29 or 0x68/0x69

Supported families
------------------
  BNO055        0x28 / 0x29   CHIP_ID  0x00 -> 0xA0
                Fuses on-chip. Gives heading/roll/pitch directly and reports
                its own calibration quality. The easy one — prefer it.

  ICM-20948     0x68 / 0x69   WHO_AM_I 0x00 -> 0xEA   (bank 0)
                Accel + gyro, plus AK09916 magnetometer at 0x0C via bypass.

  MPU-9250      0x68 / 0x69   WHO_AM_I 0x75 -> 0x71
  MPU-6500      same reg                     -> 0x70   (6-axis, no mag)
  MPU-6050      same reg                     -> 0x68   (6-axis, no mag)
                The 9250 reaches its AK8963 magnetometer at 0x0C via bypass.

A 6-axis part still works here — you get roll and pitch, but heading drifts
because there is no magnetometer to correct the gyro against. The reading is
flagged `heading_ok: False` so the UI can say so rather than quietly lie.

Fusion
------
The BNO055 fuses internally. For everything else this runs a complementary
filter: roll and pitch come from gravity, corrected by integrating the gyro;
heading comes from a tilt-compensated magnetometer, likewise gyro-corrected.

Hard-iron distortion dominates heading error on a robot full of motors and
speaker magnets, so magnetometer offsets are stored in imu_calib.json next
to this file. Run `python test/imu_test.py --calibrate` to build it. Without
it, heading can be tens of degrees out.
"""

import json
import math
import os
import time

try:
    from smbus2 import SMBus
except ImportError:                       # older Raspberry Pi OS
    try:
        from smbus import SMBus
    except ImportError:
        SMBus = None

I2C_BUS = 1                               # GPIO2 = SDA, GPIO3 = SCL
CALIB_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                          "imu_calib.json")

# BNO055 operating mode.
#
# 0x08 IMUPLUS - accelerometer + gyroscope. Relative heading, no magnetometer.
# 0x0C NDOF    - all nine axes, magnetometer fused into heading.
#
# IMUPLUS, and this was MEASURED on the robot, not assumed.
#
# NDOF is better wherever the magnetometer can actually see the earth's field.
# Here it cannot. Rotating the robot 336 degrees BY HAND with the motors off:
#
#   |B| = 142 uT, near constant (1.1x variation)
#   mx span 9.1 uT,  my span 3.4 uT,  mz span 3.7 uT
#
# The earth's field should sweep the horizontal axes through ~30 uT of
# amplitude over a full turn. It moved under 10. A 142 uT field roughly 3x
# the earth's, that barely changes as the robot rotates, is a permanent
# magnet CO-ROTATING with the sensor - the speaker. The earth's signal is
# buried beneath it.
#
# With the motors running it is worse: 172 uT swings from motor current, and
# |B| ranging 34-200 uT. The BNO055 calibrates by fitting a sphere, so it can
# never converge on either - measured mag calibration stayed at 0 through
# SEVEN full powered rotations.
#
# Consequence: heading is relative and drifts with gyro bias. The gyro here
# calibrates to 3, so that drift is small. SLAM references heading to yaw0
# anyway, so relative is all it needs.
#
# To get absolute heading back, the IMU must move well away from the speaker
# magnet and the motor wiring - then set this to 0x0C and re-measure.
BNO055_MODE = 0x08

BNO055_ADDRS = (0x28, 0x29)
MPU_ADDRS = (0x68, 0x69)
MAG_ADDR = 0x0C                           # AK8963 / AK09916, behind bypass


# ---------------------------------------------------------------------------
# calibration store
# ---------------------------------------------------------------------------

def load_calib():
    try:
        with open(CALIB_FILE) as f:
            d = json.load(f)
        return {
            "mag_offset": [float(v) for v in d.get("mag_offset", [0, 0, 0])],
            "mag_scale": [float(v) for v in d.get("mag_scale", [1, 1, 1])],
            "gyro_bias": [float(v) for v in d.get("gyro_bias", [0, 0, 0])],
        }
    except (OSError, ValueError, TypeError, KeyError):
        return {"mag_offset": [0.0] * 3, "mag_scale": [1.0] * 3,
                "gyro_bias": [0.0] * 3}


def save_calib(d):
    try:
        with open(CALIB_FILE, "w") as f:
            json.dump(d, f, indent=2)
        return True
    except OSError:
        return False


# ---------------------------------------------------------------------------
# helpers
# ---------------------------------------------------------------------------

def _s16(hi, lo):
    v = (hi << 8) | lo
    return v - 65536 if v & 0x8000 else v


def _s16le(lo, hi):
    return _s16(hi, lo)


# ---------------------------------------------------------------------------
# BNO055 — fuses on-chip
# ---------------------------------------------------------------------------

class BNO055:
    name = "BNO055"
    mode = BNO055_MODE
    fused = True
    has_mag = True

    OPR_MODE, PWR_MODE, SYS_TRIGGER, UNIT_SEL = 0x3D, 0x3E, 0x3F, 0x3B
    EULER, QUAT, ACC, GYR, MAG = 0x1A, 0x20, 0x08, 0x14, 0x0E
    CALIB_STAT, TEMP = 0x35, 0x34

    def __init__(self, bus, addr):
        self.bus, self.addr = bus, addr
        self.bus.write_byte_data(addr, self.OPR_MODE, 0x00)   # CONFIG
        time.sleep(0.03)
        self.bus.write_byte_data(addr, self.PWR_MODE, 0x00)   # normal
        self.bus.write_byte_data(addr, self.SYS_TRIGGER, 0x00)
        time.sleep(0.01)
        self.bus.write_byte_data(addr, self.OPR_MODE, BNO055_MODE)
        time.sleep(0.03)
        self.mode = BNO055_MODE

    def read(self, dt=0.0):
        b = self.bus.read_i2c_block_data(self.addr, self.EULER, 6)
        heading = _s16le(b[0], b[1]) / 16.0
        roll = _s16le(b[2], b[3]) / 16.0
        pitch = _s16le(b[4], b[5]) / 16.0

        a = self.bus.read_i2c_block_data(self.addr, self.ACC, 6)
        g = self.bus.read_i2c_block_data(self.addr, self.GYR, 6)
        m = self.bus.read_i2c_block_data(self.addr, self.MAG, 6)
        cal = self.bus.read_byte_data(self.addr, self.CALIB_STAT)
        temp = self.bus.read_byte_data(self.addr, self.TEMP)

        return {
            # BNO reports accel in m/s^2 at 1/100; convert to g for consistency
            "accel": [_s16le(a[0], a[1]) / 100.0 / 9.80665,
                      _s16le(a[2], a[3]) / 100.0 / 9.80665,
                      _s16le(a[4], a[5]) / 100.0 / 9.80665],
            "gyro": [_s16le(g[0], g[1]) / 16.0,
                     _s16le(g[2], g[3]) / 16.0,
                     _s16le(g[4], g[5]) / 16.0],
            "mag": [_s16le(m[0], m[1]) / 16.0,
                    _s16le(m[2], m[3]) / 16.0,
                    _s16le(m[4], m[5]) / 16.0],
            "roll": roll, "pitch": pitch, "yaw": heading % 360.0,
            "temp": temp if temp < 128 else temp - 256,
            # In IMUPLUS there is no magnetometer, so heading quality is the
            # GYRO's calibration. Judging it by the mag figure would report a
            # permanent fault on a mode that does not use the mag at all.
            "heading_ok": ((cal >> 4 & 3) >= 2 if self.mode == 0x08
                           else (cal >> 0 & 3) >= 2),
            "calib": {"sys": cal >> 6 & 3, "gyro": cal >> 4 & 3,
                      "accel": cal >> 2 & 3, "mag": cal >> 0 & 3},
        }


# ---------------------------------------------------------------------------
# InvenSense parts — raw sensors, fusion done here
# ---------------------------------------------------------------------------

class _Invensense:
    fused = False

    def _read_mag_raw(self):
        return None

    def read_raw(self):
        raise NotImplementedError


class MPU(_Invensense):
    """MPU-9250 / MPU-6500 / MPU-6050."""
    WHO_AM_I, PWR_MGMT_1, INT_PIN_CFG = 0x75, 0x6B, 0x37
    ACCEL_XOUT_H = 0x3B
    IDS = {0x71: ("MPU-9250", True), 0x73: ("MPU-9255", True),
           0x70: ("MPU-6500", False), 0x68: ("MPU-6050", False),
           0x69: ("MPU-6050", False)}

    def __init__(self, bus, addr, who):
        self.bus, self.addr = bus, addr
        self.name, self.has_mag = self.IDS.get(who, ("MPU-?", False))
        bus.write_byte_data(addr, self.PWR_MGMT_1, 0x00)     # wake
        time.sleep(0.05)
        bus.write_byte_data(addr, self.PWR_MGMT_1, 0x01)     # PLL clock
        time.sleep(0.01)
        self.mag = None
        if self.has_mag:
            self.mag = AK8963(bus, addr, self.INT_PIN_CFG)
            self.has_mag = self.mag.ok

    def read_raw(self):
        b = self.bus.read_i2c_block_data(self.addr, self.ACCEL_XOUT_H, 14)
        accel = [_s16(b[0], b[1]) / 16384.0,      # +/-2 g default
                 _s16(b[2], b[3]) / 16384.0,
                 _s16(b[4], b[5]) / 16384.0]
        temp = _s16(b[6], b[7]) / 333.87 + 21.0
        gyro = [_s16(b[8], b[9]) / 131.0,         # +/-250 dps default
                _s16(b[10], b[11]) / 131.0,
                _s16(b[12], b[13]) / 131.0]
        mag = self.mag.read() if self.mag and self.mag.ok else None
        return accel, gyro, mag, temp


class ICM20948(_Invensense):
    name = "ICM-20948"
    BANK_SEL, WHO_AM_I, PWR_MGMT_1, INT_PIN_CFG = 0x7F, 0x00, 0x06, 0x0F
    ACCEL_XOUT_H, TEMP_OUT_H = 0x2D, 0x39

    def __init__(self, bus, addr):
        self.bus, self.addr = bus, addr
        self._bank(0)
        bus.write_byte_data(addr, self.PWR_MGMT_1, 0x80)     # reset
        time.sleep(0.1)
        self._bank(0)
        bus.write_byte_data(addr, self.PWR_MGMT_1, 0x01)     # wake, auto clock
        time.sleep(0.02)
        self.mag = AK09916(bus, addr, self.INT_PIN_CFG, self._bank)
        self.has_mag = self.mag.ok

    def _bank(self, n):
        self.bus.write_byte_data(self.addr, self.BANK_SEL, (n & 3) << 4)

    def read_raw(self):
        self._bank(0)
        b = self.bus.read_i2c_block_data(self.addr, self.ACCEL_XOUT_H, 12)
        accel = [_s16(b[0], b[1]) / 16384.0,
                 _s16(b[2], b[3]) / 16384.0,
                 _s16(b[4], b[5]) / 16384.0]
        gyro = [_s16(b[6], b[7]) / 131.0,
                _s16(b[8], b[9]) / 131.0,
                _s16(b[10], b[11]) / 131.0]
        t = self.bus.read_i2c_block_data(self.addr, self.TEMP_OUT_H, 2)
        temp = _s16(t[0], t[1]) / 333.87 + 21.0
        mag = self.mag.read() if self.mag.ok else None
        return accel, gyro, mag, temp


class AK8963:
    """Magnetometer inside the MPU-9250, reached by putting the MPU into
    I2C bypass so the AK8963 appears on the main bus at 0x0C."""
    WIA, CNTL1, ASAX, HXL, ST2 = 0x00, 0x0A, 0x10, 0x03, 0x09

    def __init__(self, bus, mpu_addr, int_pin_cfg):
        self.bus, self.ok, self.adj = bus, False, [1.0, 1.0, 1.0]
        try:
            bus.write_byte_data(mpu_addr, int_pin_cfg, 0x02)   # bypass on
            time.sleep(0.01)
            if bus.read_byte_data(MAG_ADDR, self.WIA) != 0x48:
                return
            bus.write_byte_data(MAG_ADDR, self.CNTL1, 0x00)    # power down
            time.sleep(0.01)
            bus.write_byte_data(MAG_ADDR, self.CNTL1, 0x0F)    # fuse ROM
            time.sleep(0.01)
            asa = bus.read_i2c_block_data(MAG_ADDR, self.ASAX, 3)
            self.adj = [(a - 128) / 256.0 + 1.0 for a in asa]
            bus.write_byte_data(MAG_ADDR, self.CNTL1, 0x00)
            time.sleep(0.01)
            bus.write_byte_data(MAG_ADDR, self.CNTL1, 0x16)    # 100 Hz, 16-bit
            time.sleep(0.01)
            self.ok = True
        except OSError:
            self.ok = False

    def read(self):
        try:
            b = self.bus.read_i2c_block_data(MAG_ADDR, self.HXL, 7)
            if b[6] & 0x08:                       # ST2 overflow -> discard
                return None
            # AK8963 axes are X-east/Y-north relative to the MPU; swap so the
            # magnetometer frame matches the accel/gyro frame.
            mx = _s16le(b[0], b[1]) * 0.15 * self.adj[0]
            my = _s16le(b[2], b[3]) * 0.15 * self.adj[1]
            mz = _s16le(b[4], b[5]) * 0.15 * self.adj[2]
            return [my, mx, -mz]
        except OSError:
            return None


class AK09916:
    """Magnetometer inside the ICM-20948, same bypass trick."""
    WIA2, CNTL2, CNTL3, HXL, ST2 = 0x01, 0x31, 0x32, 0x11, 0x18

    def __init__(self, bus, icm_addr, int_pin_cfg, bank):
        self.bus, self.ok = bus, False
        try:
            bank(0)
            bus.write_byte_data(icm_addr, int_pin_cfg, 0x02)   # bypass on
            time.sleep(0.01)
            if bus.read_byte_data(MAG_ADDR, self.WIA2) != 0x09:
                return
            bus.write_byte_data(MAG_ADDR, self.CNTL3, 0x01)    # soft reset
            time.sleep(0.02)
            bus.write_byte_data(MAG_ADDR, self.CNTL2, 0x08)    # 100 Hz
            time.sleep(0.01)
            self.ok = True
        except OSError:
            self.ok = False

    def read(self):
        try:
            b = self.bus.read_i2c_block_data(MAG_ADDR, self.HXL, 8)
            if b[7] & 0x08:
                return None
            mx = _s16le(b[0], b[1]) * 0.15
            my = _s16le(b[2], b[3]) * 0.15
            mz = _s16le(b[4], b[5]) * 0.15
            return [mx, -my, -mz]
        except OSError:
            return None


# ---------------------------------------------------------------------------
# complementary filter for the parts that do not fuse
# ---------------------------------------------------------------------------

class Fusion:
    """Gravity sets roll and pitch, the magnetometer sets heading, and the
    gyro fills in between — the standard complementary filter.

    ALPHA is how much to trust the gyro over one step. 0.98 at ~50 Hz gives a
    time constant near 1 s: fast enough to follow a real turn, slow enough to
    ignore the shake of four motors on a chassis.
    """
    ALPHA = 0.98

    def __init__(self):
        self.roll = self.pitch = self.yaw = 0.0
        self._started = False

    def update(self, accel, gyro, mag, dt):
        ax, ay, az = accel
        gx, gy, gz = gyro

        acc_roll = math.degrees(math.atan2(ay, az))
        acc_pitch = math.degrees(math.atan2(-ax, math.sqrt(ay * ay + az * az)))

        if not self._started:
            self.roll, self.pitch = acc_roll, acc_pitch
            self._started = True
        else:
            self.roll = self.ALPHA * (self.roll + gx * dt) + (1 - self.ALPHA) * acc_roll
            self.pitch = self.ALPHA * (self.pitch + gy * dt) + (1 - self.ALPHA) * acc_pitch

        heading_ok = False
        if mag:
            r = math.radians(self.roll)
            p = math.radians(self.pitch)
            mx, my, mz = mag
            # tilt compensation — without it, heading swings as the robot
            # crosses a doorstep
            xh = mx * math.cos(p) + mz * math.sin(p)
            yh = (mx * math.sin(r) * math.sin(p) + my * math.cos(r)
                  - mz * math.sin(r) * math.cos(p))
            mag_yaw = math.degrees(math.atan2(-yh, xh)) % 360.0
            gyro_yaw = (self.yaw + gz * dt) % 360.0
            # shortest-path blend, so 359 -> 1 does not spin the long way
            err = ((mag_yaw - gyro_yaw + 180) % 360) - 180
            self.yaw = (gyro_yaw + (1 - self.ALPHA) * err) % 360.0
            heading_ok = True
        else:
            self.yaw = (self.yaw + gz * dt) % 360.0

        return heading_ok


# ---------------------------------------------------------------------------
# public wrapper
# ---------------------------------------------------------------------------

class IMU:
    """Whatever chip is on the bus, behind one read() returning one shape."""

    def __init__(self, bus_no=I2C_BUS):
        if SMBus is None:
            raise RuntimeError(
                "python3-smbus2 not installed. "
                "sudo apt install -y i2c-tools python3-smbus2")
        self.bus = SMBus(bus_no)
        self.dev = self._detect()
        if self.dev is None:
            raise RuntimeError(
                "no supported IMU found on i2c-%d. Check wiring and run "
                "`i2cdetect -y %d`" % (bus_no, bus_no))
        self.addr = self.dev.addr
        self.name = self.dev.name
        self.fused = self.dev.fused
        self.has_mag = getattr(self.dev, "has_mag", False)
        self.calib = load_calib()
        self.fusion = None if self.fused else Fusion()
        self._last = time.monotonic()

    def _detect(self):
        for addr in BNO055_ADDRS:
            try:
                if self.bus.read_byte_data(addr, 0x00) == 0xA0:
                    return BNO055(self.bus, addr)
            except OSError:
                pass
        for addr in MPU_ADDRS:
            try:
                if self.bus.read_byte_data(addr, 0x00) == 0xEA:
                    return ICM20948(self.bus, addr)
            except OSError:
                pass
            try:
                who = self.bus.read_byte_data(addr, MPU.WHO_AM_I)
                if who in (0x71, 0x73, 0x70, 0x68, 0x69):
                    return MPU(self.bus, addr, who)
            except OSError:
                pass
        return None

    def _apply_calib(self, mag, gyro):
        if mag:
            off, sc = self.calib["mag_offset"], self.calib["mag_scale"]
            mag = [(mag[i] - off[i]) * sc[i] for i in range(3)]
        gb = self.calib["gyro_bias"]
        gyro = [gyro[i] - gb[i] for i in range(3)]
        return mag, gyro

    def read(self):
        now = time.monotonic()
        dt = min(0.2, max(1e-3, now - self._last))
        self._last = now

        if self.fused:
            d = self.dev.read()
            d["name"] = self.name
            d["fused"] = True
            d["dt"] = round(dt, 4)
            return d

        accel, gyro, mag, temp = self.dev.read_raw()
        mag, gyro = self._apply_calib(mag, gyro)
        heading_ok = self.fusion.update(accel, gyro, mag, dt)
        return {
            "name": self.name,
            "fused": False,
            "accel": [round(v, 4) for v in accel],
            "gyro": [round(v, 3) for v in gyro],
            "mag": [round(v, 2) for v in mag] if mag else None,
            "roll": round(self.fusion.roll, 2),
            "pitch": round(self.fusion.pitch, 2),
            "yaw": round(self.fusion.yaw, 2),
            "temp": round(temp, 1),
            "heading_ok": heading_ok,
            "calib": None,
            "dt": round(dt, 4),
        }

    def close(self):
        try:
            self.bus.close()
        except OSError:
            pass
