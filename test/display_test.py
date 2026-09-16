"""
1.54" ST7789 LCD test — identify, orient, time.

    cd ~/Desktop/Speaker_truck && source .venv/bin/activate
    python test/display_test.py              # checks, colours, test card, speed
    python test/display_test.py --check      # config and wiring checks only
    python test/display_test.py --ip         # show the Pi's address and leave it lit
    python test/display_test.py -r 180       # try another rotation

Needs:
    sudo apt install -y python3-spidev python3-pil python3-numpy
    /boot/firmware/config.txt:   dtparam=spi=on
                                 dtoverlay=spi0-1cs      (then reboot)

Wiring: WIRING.md section 14. The display does not share pins with anything
else, but web_nav.py drives it too — stop the cockpit before running this,
or the two will fight over GPIO7.

What to look for
----------------
  Red / green / blue / white fills   each should be that colour.
                                     Red shows blue    -> LCD_BGR = True
                                     all look negative -> LCD_INVERT = False
  Test card                          arrow marked TOP at the top, else set
                                     LCD_ROTATION. A band of noise along one
                                     edge means the offset for that rotation
                                     is wrong for your module.
  Speed                              ~20+ fps at 31.25 MHz. Speckles or
                                     tearing -> shorter wires, or lower
                                     LCD_SPI_HZ.
"""

import argparse
import glob
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from pins import LCD_DC, LCD_ROTATION, LCD_SPI_HZ  # noqa: E402


def check():
    """Everything that can be verified without looking at the glass."""
    ok = True
    print("=== config ===")
    devs = sorted(glob.glob("/dev/spidev*"))
    print(f"  SPI devices : {' '.join(devs) or 'NONE'}")
    if "/dev/spidev0.0" not in devs:
        print("  ✗ /dev/spidev0.0 missing — add `dtparam=spi=on` and "
              "`dtoverlay=spi0-1cs` to /boot/firmware/config.txt, reboot")
        ok = False
    elif "/dev/spidev0.1" in devs:
        print(f"  ✗ /dev/spidev0.1 exists, so the kernel still owns GPIO{LCD_DC} "
              "as CE1 — add `dtoverlay=spi0-1cs`, reboot")
        ok = False
    else:
        print(f"  ✓ one chip select, GPIO{LCD_DC} free for DC")

    for mod, pkg in (("spidev", "python3-spidev"), ("PIL", "python3-pil"),
                     ("numpy", "python3-numpy")):
        try:
            __import__(mod)
            print(f"  ✓ {mod}")
        except ImportError:
            print(f"  ✗ {mod} — sudo apt install -y {pkg}")
            ok = False

    try:
        with open("/boot/firmware/config.txt") as f:
            cfg = f.read()
        if "max98357a" in cfg and "no-sdmode" not in cfg:
            print("  ⚠ max98357a overlay without no-sdmode claims GPIO4 — "
                  "not a display pin, but keep it in mind before using GPIO4")
    except OSError:
        pass
    return ok


def main():
    ap = argparse.ArgumentParser(description="ST7789 240x240 LCD test")
    ap.add_argument("-r", "--rotation", type=int, default=LCD_ROTATION,
                    choices=(0, 90, 180, 270))
    ap.add_argument("--hz", type=int, default=LCD_SPI_HZ, help="SPI clock")
    ap.add_argument("--check", action="store_true", help="checks only")
    ap.add_argument("--ip", action="store_true",
                    help="show the address and exit with the screen lit")
    args = ap.parse_args()

    if not check() or args.check:
        return

    import display                                             # noqa: PLC0415
    try:
        lcd = display.ST7789(rotation=args.rotation, hz=args.hz)
    except Exception as e:                                     # noqa: BLE001
        print(f"\n  Display init failed: {display.StatusScreen._explain(e)}")
        sys.exit(1)

    sleep_on_exit = True
    try:
        print(f"\n  SPI at {lcd.actual_hz / 1e6:.2f} MHz, rotation {lcd.rotation}")
        if args.ip:
            ip = display.lan_ip()
            lcd.image(display.render_status({"ip": ip, "port": 5004}))
            print(f"  showing {ip or 'no network'} — left on the screen")
            sleep_on_exit = False
            return

        print("\n--- fills: red, green, blue, white, black ---")
        for name, c in (("red", (255, 0, 0)), ("green", (0, 255, 0)),
                        ("blue", (0, 0, 255)), ("white", (255, 255, 255)),
                        ("black", (0, 0, 0))):
            print(f"  {name}")
            lcd.fill(c)
            time.sleep(0.8)

        print("\n--- test card: TOP arrow up? bars R G B W? ---")
        lcd.image(display.render_test())
        time.sleep(4)

        print("\n--- speed: 40 full frames ---")
        a = display.ST7789.rgb565(display.render_test())
        b = display.ST7789.rgb565(display.render_status({"ip": display.lan_ip(), "port": 5004}))
        t0 = time.monotonic()
        for i in range(40):
            lcd.frame(a if i % 2 else b)
        dt = time.monotonic() - t0
        print(f"  {40 / dt:.1f} fps, {dt / 40 * 1000:.0f} ms per frame")

        print("\n--- status screen, as web_nav.py shows it ---")
        lcd.image(display.render_status({
            "ip": display.lan_ip(), "port": 5004, "enabled": True,
            "lidar_ok": True, "lidar_hz": 11, "imu_ok": True, "heading": 87,
            "cam_ok": True, "pose": (1250, -340, 87),
            "song": "test_song.mp3", "pos": 64, "dur": 212, "volume": 0.8}))
        time.sleep(4)
        print("\nDisplay test complete.")
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        # Blank and sleep the panel, release GPIO7 and the SPI bus —
        # unless --ip asked for the address to stay on the glass.
        lcd.close(sleep=sleep_on_exit)


if __name__ == "__main__":
    main()
