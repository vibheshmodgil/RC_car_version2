"""Pan/tilt gimbal on the Pi's GPIO pins via lgpio.

lgpio is pigpio's successor (same author) and is what's actually packaged
on current Raspberry Pi OS (Bookworm/Trixie dropped the `pigpio` apt
package as unmaintained). No daemon to start — unlike pigpio, lgpio talks
to the kernel gpiochip directly.

Servos are powered from their own 5 V UBEC, never the Pi's rail.
"""
import lgpio  # apt: python3-lgpio

from config import (GIMBAL_PAN_GPIO, GIMBAL_TILT_GPIO,
                    SERVO_MIN_US, SERVO_MAX_US)

GPIOCHIP = 0   # Pi 4's single gpiochip covers the whole 40-pin header


class Gimbal:
    def __init__(self):
        self.h = lgpio.gpiochip_open(GPIOCHIP)
        # Unlike pigpio's set_servo_pulsewidth, lgpio's tx_servo needs the
        # pin explicitly claimed as an output first.
        lgpio.gpio_claim_output(self.h, GIMBAL_PAN_GPIO)
        lgpio.gpio_claim_output(self.h, GIMBAL_TILT_GPIO)

    @staticmethod
    def _us(angle_deg: float) -> int:
        """0..180 degrees -> pulse width, clamped to the configured limits."""
        a = max(0.0, min(180.0, angle_deg))
        return int(SERVO_MIN_US + (SERVO_MAX_US - SERVO_MIN_US) * a / 180.0)

    def pan(self, deg: float):
        lgpio.tx_servo(self.h, GIMBAL_PAN_GPIO, self._us(deg))

    def tilt(self, deg: float):
        lgpio.tx_servo(self.h, GIMBAL_TILT_GPIO, self._us(deg))

    def center(self):
        self.pan(90)
        self.tilt(90)

    def release(self):
        """Stop sending pulses — servos go limp and stop drawing current."""
        lgpio.tx_servo(self.h, GIMBAL_PAN_GPIO, 0)
        lgpio.tx_servo(self.h, GIMBAL_TILT_GPIO, 0)
