"""Pan/tilt gimbal on the Pi's hardware-PWM pins via pigpio.

Needs the daemon: sudo systemctl enable --now pigpiod
Servos are powered from their own 5 V UBEC, never the Pi's rail.
"""
import pigpio  # apt: python3-pigpio

from config import (GIMBAL_PAN_GPIO, GIMBAL_TILT_GPIO,
                    SERVO_MIN_US, SERVO_MAX_US)


class Gimbal:
    def __init__(self):
        self.pi = pigpio.pi()
        if not self.pi.connected:
            raise RuntimeError("pigpiod is not running: "
                               "sudo systemctl enable --now pigpiod")

    @staticmethod
    def _us(angle_deg: float) -> int:
        """0..180 degrees -> pulse width, clamped to the configured limits."""
        a = max(0.0, min(180.0, angle_deg))
        return int(SERVO_MIN_US + (SERVO_MAX_US - SERVO_MIN_US) * a / 180.0)

    def pan(self, deg: float):
        self.pi.set_servo_pulsewidth(GIMBAL_PAN_GPIO, self._us(deg))

    def tilt(self, deg: float):
        self.pi.set_servo_pulsewidth(GIMBAL_TILT_GPIO, self._us(deg))

    def center(self):
        self.pan(90)
        self.tilt(90)

    def release(self):
        """Stop sending pulses — servos go limp and stop drawing current."""
        self.pi.set_servo_pulsewidth(GIMBAL_PAN_GPIO, 0)
        self.pi.set_servo_pulsewidth(GIMBAL_TILT_GPIO, 0)
