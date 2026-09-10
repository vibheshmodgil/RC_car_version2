"""
Hold one static pin state so it can be metered calmly.

    cd ~/Desktop/Speaker_truck && source .venv/bin/activate
    python test/pin_hold.py                 # STBY high, both sides coast
    python test/pin_hold.py forward
    python test/pin_hold.py reverse
    python test/pin_hold.py off             # STBY low — everything disabled

*** RUN THIS WITH VM (battery) DISCONNECTED. ***

motor_test.py --logic steps every 3 seconds, which is too fast to get a
meter on four pins — readings taken in different steps look contradictory.
This holds one state until Ctrl-C and prints what every pin should read,
so the meter and the table can be compared directly.

The PWM pin is driven as steady DC HIGH by default, not as PWM. A multimeter
averages a 1 kHz square wave, so a PWM pin at 0.40 duty reads ~1.3 V and is
easy to mistake for a fault. DC high reads a clean 3.3 V or nothing at all.
Use --pwm to switch to a real PWM carrier once the DC reading is confirmed.

Every voltage below is measured against COMMON GROUND. If the driver's GND is
not tied to the Pi's GND, none of these readings mean anything — check that
first with a continuity beep.
"""

import argparse
import os
import sys
from signal import pause

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

# name -> (BCM, header pin) for the printed table
HEADER = {
    STBY: 22, LEFT_IN1: 16, LEFT_IN2: 18, LEFT_PWM: 32,
    RIGHT_IN1: 13, RIGHT_IN2: 15, RIGHT_PWM: 33,
}

# state -> (in1, in2) per side, from the TB6612 truth table
STATES = {
    "coast":   (False, False),
    "forward": (True,  False),
    "reverse": (False, True),
    "brake":   (True,  True),
}


def volts(level):
    return "3.3 V" if level else "0.0 V"


def main():
    ap = argparse.ArgumentParser(description="hold one static pin state")
    ap.add_argument("state", nargs="?", default="coast",
                    choices=list(STATES) + ["off"],
                    help="off = STBY low, drivers disabled")
    ap.add_argument("--pwm", action="store_true",
                    help="drive the PWM pins with a real carrier instead of DC "
                         "high (a DMM will then read duty x 3.3 V)")
    ap.add_argument("--duty", type=float, default=MAX_DUTY,
                    help=f"duty for --pwm, capped at MAX_DUTY ({MAX_DUTY})")
    args = ap.parse_args()

    enabled = args.state != "off"
    in1, in2 = STATES[args.state] if enabled else (False, False)
    duty = min(abs(args.duty), MAX_DUTY)

    # Safe state first: STBY low before any direction pin is driven, so the
    # outputs cannot glitch while the pins are being claimed.
    stby = DigitalOutputDevice(STBY, initial_value=False)

    l_in1 = DigitalOutputDevice(LEFT_IN1,  initial_value=in1)
    l_in2 = DigitalOutputDevice(LEFT_IN2,  initial_value=in2)
    r_in1 = DigitalOutputDevice(RIGHT_IN1, initial_value=in1)
    r_in2 = DigitalOutputDevice(RIGHT_IN2, initial_value=in2)

    if args.pwm:
        l_pwm = PWMOutputDevice(LEFT_PWM,  initial_value=duty, frequency=PWM_HZ)
        r_pwm = PWMOutputDevice(RIGHT_PWM, initial_value=duty, frequency=PWM_HZ)
        pwm_reading = f"{duty * 3.3:.2f} V  (avg of {PWM_HZ} Hz at {duty:.2f})"
        pwm_on = True
    else:
        # plain DC high — unambiguous on a multimeter
        l_pwm = DigitalOutputDevice(LEFT_PWM,  initial_value=enabled)
        r_pwm = DigitalOutputDevice(RIGHT_PWM, initial_value=enabled)
        pwm_reading = volts(enabled)
        pwm_on = enabled

    devices = [stby, l_in1, l_in2, r_in1, r_in2, l_pwm, r_pwm]

    try:
        stby.value = enabled

        print()
        print(f"  HOLDING: {args.state.upper()}"
              f"{'  (PWM carrier)' if args.pwm else '  (PWM pins DC high)'}")
        print(f"  VM should be DISCONNECTED. Ctrl-C to release.\n")
        print(f"  {'signal':<12}{'BCM':>5}{'header':>8}   expected vs GND")
        print(f"  {'-'*12}{'-'*5}{'-'*8}   {'-'*28}")

        rows = [
            ("STBY",      STBY,      volts(enabled)),
            ("LEFT_IN1",  LEFT_IN1,  volts(in1)),
            ("LEFT_IN2",  LEFT_IN2,  volts(in2)),
            ("LEFT_PWM",  LEFT_PWM,  pwm_reading),
            ("RIGHT_IN1", RIGHT_IN1, volts(in1)),
            ("RIGHT_IN2", RIGHT_IN2, volts(in2)),
            ("RIGHT_PWM", RIGHT_PWM, pwm_reading),
        ]
        for name, pin, expect in rows:
            print(f"  {name:<12}{pin:>5}{HEADER[pin]:>8}   {expect}")

        print()
        print("  Before trusting any of it:")
        print("    driver GND <-> Pi GND        must beep on continuity")
        print("    driver VCC                   must read 3.3 V")
        print()
        print("  A pin reading 0.6-1.2 V instead of 3.3 V is the signature of")
        print("  an UNPOWERED driver: the GPIO is pushing current through the")
        print("  chip's input clamp diode into a dead VCC rail. Fix VCC first.")
        print()
        pause()

    except KeyboardInterrupt:
        pass
    finally:
        # Runs on Ctrl-C and on any exception, so the pins never stay driven.
        stby.off()
        for d in devices:
            d.close()
        print("\n  STBY low, pins released.")


if __name__ == "__main__":
    main()
