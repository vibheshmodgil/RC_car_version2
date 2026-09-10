"""
LiDAR probe — work out what the scanner actually is before parsing it.

    cd ~/Desktop/Speaker_truck && source .venv/bin/activate
    python test/lidar_probe.py                    # auto-detect and identify
    python test/lidar_probe.py -p /dev/ttyUSB0    # force a port
    python test/lidar_probe.py --dump raw.bin     # save raw bytes for analysis

Needs pyserial. Prefer the apt build, like the other hardware packages:

    sudo apt install -y python3-serial

(The venv sees it because it was created with --system-site-packages. `pip
install pyserial` also works — it is pure Python — but apt keeps this
consistent with gpiozero and lgpio. See CLAUDE.md.)

Why this exists
---------------
"2D LiDAR on /dev/ttyUSB0" is not enough to write a parser against. The two
common families behave in opposite ways:

  YDLIDAR X2/X4 etc.  Streams the moment it has power. Packets start with the
                      two bytes AA 55.
  RPLIDAR A1/A2/S1    Sends NOTHING until commanded. Silence on the port is
                      the expected state, not a fault.

So this probe listens passively first, and only if the port stays quiet does
it send RPLIDAR's GET_INFO. Both commands used here (GET_INFO, GET_HEALTH)
are read-only — neither starts the motor or changes any setting.

The USB VID:PID identifies the USB-serial chip (CP2102, CH340), NOT the
scanner. Two different LiDARs can share the same adapter chip, which is why
the protocol probe below is the part that actually decides.
"""

import argparse


import sys
import time

try:
    import serial
    import serial.tools.list_ports as list_ports
except ImportError:
    print("pyserial not found.  sudo apt install -y python3-serial")
    sys.exit(1)

# Ordered by how likely they are on a hobby 2D scanner.
BAUDS = [115200, 128000, 230400, 256000, 460800]

# Adapter chips, not scanners — reported for context only.
USB_CHIPS = {
    ("10c4", "ea60"): "CP2102 (Silicon Labs)",
    ("1a86", "7523"): "CH340 (WCH)",
    ("1a86", "55d4"): "CH9102 (WCH)",
    ("0403", "6001"): "FT232 (FTDI)",
    ("067b", "2303"): "PL2303 (Prolific)",
}

# RPLIDAR request frames. Both are read-only.
RP_GET_INFO = b"\xA5\x50"
RP_GET_HEALTH = b"\xA5\x52"
RP_STOP = b"\xA5\x25"

RP_MODELS = {0x18: "A1", 0x28: "A2", 0x38: "A3", 0x41: "S1", 0x61: "S2"}


def find_ports():
    """Serial ports worth probing, on the Pi or on the Windows machine.

    list_ports handles both — /dev/ttyUSB0 on Linux, COM4 on Windows — so the
    LiDAR can be probed on the laptop before it ever moves to the robot.

    USB ports first and, if any exist, ONLY those. A Pi always exposes
    /dev/ttyAMA0 and /dev/ttyS0 (the built-in UARTs, nothing attached), and
    probing them costs 7.5 s each to learn nothing. A USB scanner always has
    a VID/PID; the built-in UARTs never do.
    """
    ports = sorted(list_ports.comports(), key=lambda p: p.device)
    usb = [p.device for p in ports if p.vid is not None]
    return usb or [p.device for p in ports]


def usb_info(port):
    """VID/PID and description straight from pyserial, no udevadm needed."""
    for p in list_ports.comports():
        if p.device == port:
            return {
                "vid": f"{p.vid:04x}" if p.vid is not None else "?",
                "pid": f"{p.pid:04x}" if p.pid is not None else "?",
                "description": p.description or "",
                "manufacturer": p.manufacturer or "",
                "product": p.product or "",
                "serial": p.serial_number or "",
            }
    return {}


def hexdump(data, limit=64):
    head = data[:limit]
    hx = " ".join(f"{b:02X}" for b in head)
    return hx + (" ..." if len(data) > limit else "")


def listen(port, baud, seconds=1.5):
    """Open at baud and read whatever arrives, without sending anything."""
    try:
        with serial.Serial(port, baud, timeout=0.3) as ser:
            # RPLIDAR A1's motor is driven from DTR on its USB adapter. Leave
            # it de-asserted so a passive listen cannot spin the motor up.
            try:
                ser.dtr = False
            except (OSError, AttributeError):
                pass
            ser.reset_input_buffer()
            end = time.monotonic() + seconds
            buf = bytearray()
            while time.monotonic() < end:
                buf += ser.read(4096)
            return bytes(buf)
    except serial.SerialException as e:
        return e


def count_sync(data, pattern):
    n, i = 0, 0
    while True:
        i = data.find(pattern, i)
        if i < 0:
            return n
        n += 1
        i += 1


def rplidar_query(port, baud):
    """Send GET_INFO / GET_HEALTH and parse the response descriptor.

    A reply always starts with the 7-byte descriptor A5 5A <len/mode> <type>,
    so seeing A5 5A come back is on its own proof this is an RPLIDAR.
    """
    try:
        with serial.Serial(port, baud, timeout=1.0) as ser:
            try:
                ser.dtr = False
            except (OSError, AttributeError):
                pass
            ser.write(RP_STOP)            # harmless if it was never scanning
            time.sleep(0.05)
            ser.reset_input_buffer()

            ser.write(RP_GET_INFO)
            desc = ser.read(7)
            if len(desc) < 7 or desc[0] != 0xA5 or desc[1] != 0x5A:
                return None
            payload = ser.read(20)
            if len(payload) < 20:
                return {"raw_descriptor": hexdump(desc)}

            model = payload[0]
            info = {
                "model_byte": f"0x{model:02X}",
                "model": RP_MODELS.get(model, "unknown model byte"),
                "firmware": f"{payload[2]}.{payload[1]}",
                "hardware": str(payload[3]),
                "serial": payload[4:20][::-1].hex().upper(),
            }

            ser.reset_input_buffer()
            ser.write(RP_GET_HEALTH)
            hdesc = ser.read(7)
            if len(hdesc) == 7 and hdesc[0] == 0xA5:
                h = ser.read(3)
                if len(h) == 3:
                    info["health"] = {0: "good", 1: "warning", 2: "error"}.get(
                        h[0], f"unknown ({h[0]})")
                    info["health_code"] = h[1] | (h[2] << 8)
            return info
    except serial.SerialException:
        return None


def probe_port(port, dump_path=None):
    print(f"\n{'=' * 62}")
    print(f"  {port}")
    print("=" * 62)

    props = usb_info(port)
    vid = props.get("vid", "?")
    pid = props.get("pid", "?")
    chip = USB_CHIPS.get((vid.lower(), pid.lower()), "unknown chip")
    print(f"  USB   {vid}:{pid}  {chip}")
    for k in ("description", "manufacturer", "product", "serial"):
        if props.get(k):
            print(f"        {k}: {props[k]}")
    print("        (this is the USB-serial adapter, not the scanner)")

    print("\n  --- passive listen (nothing sent) ---")
    best = None
    for baud in BAUDS:
        data = listen(port, baud)
        if isinstance(data, Exception):
            print(f"  {baud:>7}   cannot open: {data}")
            if "Permission denied" in str(data):
                print("            -> sudo usermod -aG dialout $USER, then log out and back in")
            return
        ydl = count_sync(data, b"\xAA\x55")
        print(f"  {baud:>7}   {len(data):>6} bytes   AA55 x{ydl}")
        if data:
            print(f"            {hexdump(data, 32)}")
        if best is None or ydl > best[2] or (ydl == best[2] and len(data) > len(best[1])):
            best = (baud, data, ydl)
        if dump_path and data:
            with open(f"{dump_path}.{baud}", "wb") as f:
                f.write(data)

    baud, data, ydl = best

    print("\n  --- RPLIDAR command probe (GET_INFO, read-only) ---")
    rp = None
    for b in BAUDS:
        rp = rplidar_query(port, b)
        if rp:
            print(f"  {b:>7}   replied with an A5 5A descriptor")
            for k, v in rp.items():
                print(f"            {k}: {v}")
            baud = b
            break
        print(f"  {b:>7}   no response")

    print("\n  --- verdict ---")
    if rp and "model" in rp:
        print(f"  RPLIDAR {rp['model']}  at {baud} baud.")
        print("  Does not stream until commanded; motor runs off DTR.")
    elif ydl >= 5:
        print(f"  YDLIDAR-family framing (AA 55) at {baud} baud — {ydl} headers seen.")
        print("  Streams continuously as soon as it has power.")
    elif len(data) > 0:
        print(f"  Streaming {len(data)} bytes at {baud} baud, but no AA 55 and no")
        print("  RPLIDAR reply. Unrecognised protocol — send me the hex above.")
    else:
        print("  Silent at every baud, and no RPLIDAR reply. Check, in order:")
        print("    - is the scanner's motor spinning? (look at it)")
        print("    - 5 V present on the scanner's power pins, under load")
        print("    - some units need the motor powered separately from the data adapter")
        print("    - wrong port — try the others listed above")

    if dump_path:
        print(f"\n  raw captures written to {dump_path}.<baud>")


def main():
    ap = argparse.ArgumentParser(description="identify a USB 2D LiDAR")
    ap.add_argument("-p", "--port", help="skip auto-detect, probe this port")
    ap.add_argument("--dump", metavar="PATH",
                    help="write raw bytes to PATH.<baud> for offline analysis")
    args = ap.parse_args()

    ports = [args.port] if args.port else find_ports()
    if not ports:
        print("No /dev/ttyUSB* or /dev/ttyACM* found.")
        print("Plug the LiDAR in, then:  dmesg | tail -20")
        return

    print(f"Serial ports: {', '.join(ports)}")
    for p in ports:
        probe_port(p, args.dump)
    print()


if __name__ == "__main__":
    main()
