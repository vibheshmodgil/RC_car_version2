"""BNO055 over I2C1 (address 0x28). Requires the i2c_arm_baudrate=10000
config.txt workaround — see the Pi Integration Phase doc."""
import board                # pip: adafruit-blinka (pulled in by the lib below)
import adafruit_bno055      # pip: adafruit-circuitpython-bno055


class Imu:
    def __init__(self):
        self._s = adafruit_bno055.BNO055_I2C(board.I2C())

    @property
    def euler(self):
        """(heading, roll, pitch) in degrees; elements are None until the
        sensor warms up."""
        return self._s.euler

    @property
    def calibration(self):
        """(sys, gyro, accel, mag), each 0..3. Figure-8 the car until 3s."""
        return self._s.calibration_status
