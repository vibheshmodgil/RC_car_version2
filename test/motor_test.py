"""
First motor test — TB6612FNG × 2, differential drive.

    cd ~/Desktop/Speaker_truck && source .venv/bin/activate
    python test/motor_test.py            # guided sequence
    python test/motor_test.py --logic    # no VM, verify GPIO only

*** PUT THE ROBOT ON A BOX. WHEELS MUST NOT TOUCH THE GROUND. ***

Run --logic first with the motor supply (VM) DISCONNECTED. That proves the
pin assignments and direction logic with no current flowing, so a wiring
mistake costs nothing.

Note on PWM: gpiozero drives PWMOutputDevice in *software*, even on GPIO12/13.
At 1 kHz that is fine for DC motors. If you later see speed jitter under CPU
load, switch to true hardware PWM: add `dtoverlay=pwm-2chan` to
/boot/firmware/config.txt and use the `rpi-hardware-pwm` package.
"""

import argparse
import os
import sys
from time import sleep

from gpiozero import DigitalOutputDevice, PWMOutputDevice

# Absolute path, so the script works no matter which directory it is
# launched from. __file__.rsplit("/") breaks when run as `python x.py`
# from inside test/, because there is then no "/" to split on.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from pins import (  # noqa: E402
    LEFT_PWM, LEFT_IN1, LEFT_IN2,
    RIGHT_PWM, RIGHT_IN1, RIGHT_IN2,
    STBY, PWM_HZ, MAX_DUTY,
)


class Side:
    """One TB6612 channel pair driving the two wheels on one side.

    Truth table (per the TB6612FNG):
        IN1=H IN2=L -> forward      IN1=L IN2=H -> reverse
        IN1=L IN2=L -> coast        IN1=H IN2=H -> brake
    """

    def __init__(self, name, in1, in2, pwm):
        self.name = name
        # initial_value=False writes the safe level before the pin is driven,
        # so the motor cannot twitch during construction.
        self.in1 = DigitalOutputDevice(in1, initial_value=False)
        self.in2 = DigitalOutputDevice(in2, initial_value=False)
        self.pwm = PWMOutputDevice(pwm, initial_value=0, frequency=PWM_HZ)

    def drive(self, speed):
        """speed in -1.0 (full reverse) .. +1.0 (full forward)."""
        speed = max(-1.0, min(1.0, speed))
        duty = min(abs(speed), MAX_DUTY)  # ceiling enforced here, always

        if speed > 0:
            self.in1.on(); self.in2.off()
        elif speed < 0:
            self.in1.off(); self.in2.on()
        else:
            self.in1.off(); self.in2.off()  # coast
            duty = 0

        self.pwm.value = duty

    def brake(self):
        """Active short-brake. Stops harder than coast."""
        self.in1.on(); self.in2.on()
        self.pwm.value = 1.0

    def coast(self):
        self.in1.off(); self.in2.off()
        self.pwm.value = 0

    def close(self):
        self.coast()
        self.in1.close(); self.in2.close(); self.pwm.close()


class Robot:
    """Differential drive over two Sides, plus the shared STBY line."""

    def __init__(self):
        # STBY starts LOW: both drivers disabled until enable() is called.
        self.stby = DigitalOutputDevice(STBY, initial_value=False)
        self.left = Side("LEFT", LEFT_IN1, LEFT_IN2, LEFT_PWM)
        self.right = Side("RIGHT", RIGHT_IN1, RIGHT_IN2, RIGHT_PWM)

    def enable(self):
        self.stby.on()

    def disable(self):
        """Emergency stop — outputs float regardless of IN/PWM state."""
        self.stby.off()

    def tank(self, left, right, seconds=None):
        self.left.drive(left)
        self.right.drive(right)
        if seconds:
            sleep(seconds)
            self.stop()

    def stop(self):
        self.left.brake()
        self.right.brake()
        sleep(0.15)
        self.left.coast()
        self.right.coast()

    def close(self):
        self.stop()
        self.disable()
        self.left.close()
        self.right.close()
        self.stby.close()


def confirm():
    print("=" * 58)
    print("  WHEELS OFF THE GROUND?  Robot on a box, wheels spinning free.")
    print("  Motor supply (VM) connected and battery above 9.0 V?")
    print("=" * 58)
    if input("  Type 'yes' to continue: ").strip().lower() != "yes":
        print("  Aborted.")
        sys.exit(0)


def logic_only(bot):
    """Step through every state slowly so you can meter the pins with VM off."""
    print("\nLOGIC-ONLY MODE — VM should be DISCONNECTED.")
    print("Meter each pin against GND as it is announced.\n")
    bot.enable()
    print(f"  STBY (GPIO{STBY}) -> HIGH, drivers enabled")
    sleep(2)

    for label, l, r in [
        ("both FORWARD",  0.3,  0.3),
        ("both REVERSE", -0.3, -0.3),
        ("spin LEFT",    -0.3,  0.3),
        ("spin RIGHT",    0.3, -0.3),
    ]:
        print(f"  {label}")
        print(f"    LEFT  IN1={'H' if l > 0 else 'L'} "
              f"IN2={'H' if l < 0 else 'L'}  PWM~{min(abs(l), MAX_DUTY):.2f}")
        print(f"    RIGHT IN1={'H' if r > 0 else 'L'} "
              f"IN2={'H' if r < 0 else 'L'}  PWM~{min(abs(r), MAX_DUTY):.2f}")
        bot.tank(l, r)
        sleep(3)

    bot.stop()
    bot.disable()
    print(f"\n  STBY -> LOW, drivers disabled. Logic check done.")


def drive_sequence(bot):
    bot.enable()
    steps = [
        ("forward",     0.30,  0.30, 2.0),
        ("stop",        0.00,  0.00, 1.0),
        ("reverse",    -0.30, -0.30, 2.0),
        ("stop",        0.00,  0.00, 1.0),
        ("spin left",  -0.30,  0.30, 1.5),
        ("spin right",  0.30, -0.30, 1.5),
        ("stop",        0.00,  0.00, 1.0),
        ("arc right",   0.35,  0.15, 2.0),
    ]
    for label, l, r, t in steps:
        print(f"  {label:<12} L={l:+.2f}  R={r:+.2f}   {t}s")
        bot.tank(l, r)
        sleep(t)
    bot.stop()

    # Ramp — reveals the minimum duty that actually breaks static friction.
    print("\n  ramping up, watch for the duty where the wheels start turning")
    for duty in [i / 100 for i in range(5, int(MAX_DUTY * 100) + 1, 5)]:
        print(f"    duty {duty:.2f}")
        bot.tank(duty, duty)
        sleep(1.0)
    bot.stop()


def main():
    ap = argparse.ArgumentParser(description="TB6612FNG motor test")
    ap.add_argument("--logic", action="store_true",
                    help="step pins slowly with VM disconnected")
    args = ap.parse_args()

    if not args.logic:
        confirm()

    bot = Robot()
    try:
        print(f"\nMAX_DUTY capped at {MAX_DUTY:.2f}, PWM {PWM_HZ} Hz\n")
        if args.logic:
            logic_only(bot)
        else:
            drive_sequence(bot)
        print("\nDone.")
    except KeyboardInterrupt:
        print("\n\nInterrupted — stopping.")
    finally:
        # Runs on success, on Ctrl-C, and on any exception. Without this the
        # motors keep running after the script dies.
        bot.close()
        print("Motors stopped, STBY low, pins released.")


if __name__ == "__main__":
    main()
