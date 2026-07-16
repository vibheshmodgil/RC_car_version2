"""BNO055 IMU — raw I2C registers over smbus2, no Adafruit/CircuitPython
dependency (that stack was Blinka-install pain on the Pi; this only needs
smbus2). NDOF (9-DOF sensor fusion) mode. Register map and scale factors
are from the Bosch BNO055 datasheet section 4.2 (register map) and 3.6.4
(unit tables). Ported from the proven bench-tested reader in the LIDAR
prototype project.

Needs the i2c_arm_baudrate=10000 config.txt workaround — see the Pi
Centric doc — the Pi's I2C controller mishandles this chip's clock
stretching otherwise.
"""
import threading
import time

from config import IMU_I2C_BUS, IMU_I2C_ADDR


def _s16(lo: int, hi: int) -> int:
    v = lo | hi << 8
    return v - 65536 if v & 0x8000 else v


class Imu:
    def __init__(self, bus_num: int = IMU_I2C_BUS, addr: int = IMU_I2C_ADDR):
        try:
            from smbus2 import SMBus
        except ImportError:
            from smbus import SMBus
        self._SMBus = SMBus
        self.bus_num = bus_num
        self.addr = addr

        # Fail fast if the chip isn't there, matching Camera/Gimbal's
        # open-or-raise constructor pattern.
        probe = SMBus(bus_num)
        try:
            if probe.read_byte_data(addr, 0x00) != 0xA0:
                raise OSError("chip id mismatch — not a BNO055?")
        finally:
            probe.close()

        # Latest reading, replaced (never mutated) by the background thread
        # so callers can read it without a lock (safe under the GIL).
        self.reading = {"ok": False}
        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self):
        while True:
            try:
                bus = self._SMBus(self.bus_num)
                bus.write_byte_data(self.addr, 0x3D, 0x00)  # OPR_MODE = CONFIG
                time.sleep(0.03)
                bus.write_byte_data(self.addr, 0x07, 0x00)  # register page 0
                bus.write_byte_data(self.addr, 0x3E, 0x00)  # normal power
                bus.write_byte_data(self.addr, 0x3D, 0x0C)  # OPR_MODE = NDOF fusion
                time.sleep(0.03)
            except Exception:
                time.sleep(2)
                continue

            errors = 0
            while errors < 5:  # a few bad reads in a row: re-init from scratch
                try:
                    b = bus.read_i2c_block_data(self.addr, 0x08, 32)   # acc mag gyr eul quat
                    b += bus.read_i2c_block_data(self.addr, 0x28, 14)  # lin grav temp calib
                except Exception:
                    errors += 1
                    time.sleep(0.1)
                    continue
                errors = 0
                v = [_s16(b[i], b[i + 1]) for i in range(0, 44, 2)]
                cal = b[45]
                self.reading = {
                    "ok": True,
                    "h": v[9] / 16, "r": v[10] / 16, "p": v[11] / 16,  # euler heading/roll/pitch, deg
                    "cal": [cal >> 6 & 3, cal >> 4 & 3, cal >> 2 & 3, cal & 3],  # sys,gyro,accel,mag
                }
                time.sleep(0.02)

            self.reading = {"ok": False}
            try:
                bus.close()
            except Exception:
                pass
