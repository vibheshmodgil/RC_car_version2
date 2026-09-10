"""
Pilot — drive the truck and see what the LiDAR sees, on one page.

    cd ~/Desktop/Speaker_truck && source .venv/bin/activate
    python test/web_pilot.py

Then open  http://<pi-ip>:5003  and click the page once so it has keyboard
focus.

*** WHEELS OFF THE GROUND until the drivers are replaced. The TB6612s cannot
*** survive a stall on these motors (JGB37-520 stalls at 4-5 A, TB6612 peaks
*** at 3.2 A). WIRING.md section 8.

This is web_drive.py and lidar_view.py merged, which buys two things that
neither has on its own:

  * The truck is drawn TO SCALE at the centre of its own scan, so a gap on
    screen is a gap the robot actually fits through.
  * The proximity guard blocks forward motion when something is too close in
    front. Reverse and turning stay available, so you can always back out.

Ports: web_dashboard.py 5000, web_drive.py 5001, lidar_view.py 5002, this
5003. Only ONE of the GPIO-owning scripts can run at a time — this one,
web_drive.py or web_dashboard.py — because they claim the same pins.

Controls
--------
  up / W       forward        down / S     reverse
  left / A     turn left      right / D    turn right
  Space        e-stop (STBY low)

Hold to drive, release to stop: a command goes out at 20 Hz while a key is
down, then a single zero on release.

Needs pyserial for the LiDAR:  sudo apt install -y python3-serial
"""

import argparse
import json
import math
import os
import socket
import struct
import sys
import threading
import time
from collections import deque

# Absolute path, so the script works no matter which directory it is
# launched from. __file__.rsplit("/") breaks when run as `python x.py`
# from inside test/, because there is then no "/" to split on.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from pins import (  # noqa: E402
    LEFT_ENC_A, LEFT_ENC_B, RIGHT_ENC_A, RIGHT_ENC_B,
    LEFT_PWM, LEFT_IN1, LEFT_IN2,
    RIGHT_PWM, RIGHT_IN1, RIGHT_IN2,
    STBY, PWM_HZ, MAX_DUTY, COUNTS_PER_REV,
)

from gpiozero import DigitalOutputDevice, PWMOutputDevice, RotaryEncoder  # noqa: E402
from flask import Flask, jsonify, render_template_string, request  # noqa: E402

try:
    import serial
    import serial.tools.list_ports as list_ports
except ImportError:
    print("pyserial not found.  sudo apt install -y python3-serial")
    sys.exit(1)

HTTP_PORT = 5003
LIDAR_BAUD = 115200

# Drive packets arrive at 20 Hz while a key is held, so a short deadman is
# safe. A closed tab mid-hold stops the motors in well under a second.
WATCHDOG_S = 0.6
RATED_RPM = 330

# Proximity guard: forward motion is refused when the nearest return in the
# front sector is closer than this. Reverse and turning are never blocked.
GUARD_STOP_MM = 350
GUARD_SECTOR_DEG = 50          # +/- this either side of straight ahead

# ⚠ MEASURE THESE. The truck is drawn to scale from them, so if they are wrong
# the clearance you see on screen is wrong too. Chassis footprint in mm.
TRUCK_LEN_MM = 300
TRUCK_WIDTH_MM = 240

INVERT_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                           "drive_invert.json")
INVERT_KEYS = ("left", "right", "swap")

LIKELY_USB = ("cp210", "ch340", "ch9102", "silicon labs", "usb-serial", "ftdi")


def load_invert():
    try:
        with open(INVERT_FILE) as f:
            d = json.load(f)
        return {k: bool(d.get(k, False)) for k in INVERT_KEYS}
    except (OSError, ValueError, TypeError):
        return {k: False for k in INVERT_KEYS}


def save_invert(d):
    try:
        with open(INVERT_FILE, "w") as f:
            json.dump(d, f)
    except OSError:
        pass


# ---------------------------------------------------------------------------
# Motors
# ---------------------------------------------------------------------------

class Side:
    """One TB6612 channel pair driving the two wheels on one side.

        IN1=H IN2=L -> forward      IN1=L IN2=H -> reverse
        IN1=L IN2=L -> coast        IN1=H IN2=H -> brake
    """

    def __init__(self, name, in1, in2, pwm_pin):
        self.name = name
        # initial_value=False writes the safe level before the pin is driven.
        self.in1 = DigitalOutputDevice(in1, initial_value=False)
        self.in2 = DigitalOutputDevice(in2, initial_value=False)
        self.pwm = PWMOutputDevice(pwm_pin, initial_value=0, frequency=PWM_HZ)
        self._speed = 0.0

    def drive(self, speed):
        speed = max(-1.0, min(1.0, speed))
        duty = min(abs(speed), MAX_DUTY)      # ceiling enforced here, always
        self._speed = speed
        if speed > 0:
            self.in1.on();  self.in2.off()
        elif speed < 0:
            self.in1.off(); self.in2.on()
        else:
            self.in1.off(); self.in2.off()
            duty = 0
        self.pwm.value = duty

    def brake(self):
        self.in1.on(); self.in2.on()
        self.pwm.value = 1.0
        self._speed = 0.0

    def coast(self):
        self.in1.off(); self.in2.off()
        self.pwm.value = 0
        self._speed = 0.0

    def close(self):
        self.coast()
        self.in1.close(); self.in2.close(); self.pwm.close()


class Robot:
    def __init__(self):
        self._lock = threading.Lock()
        self._enabled = False
        self.invert = load_invert()
        self.limit = MAX_DUTY

        # STBY starts LOW: both drivers disabled until enable() is called.
        self.stby = DigitalOutputDevice(STBY, initial_value=False)
        self.left = Side("LEFT", LEFT_IN1, LEFT_IN2, LEFT_PWM)
        self.right = Side("RIGHT", RIGHT_IN1, RIGHT_IN2, RIGHT_PWM)

        self.enc_left = RotaryEncoder(LEFT_ENC_A, LEFT_ENC_B, max_steps=0)
        self.enc_right = RotaryEncoder(RIGHT_ENC_A, RIGHT_ENC_B, max_steps=0)
        self._last_l = self._last_r = 0
        self._last_t = time.monotonic()
        self._rpm_l = self._rpm_r = 0.0

        self._last_cmd = time.monotonic()
        self._tripped = False

        threading.Thread(target=self._rpm_loop, daemon=True).start()
        threading.Thread(target=self._watchdog_loop, daemon=True).start()

    def enable(self):
        with self._lock:
            self.stby.on()
            self._enabled = True
        self._tripped = False

    def estop(self):
        with self._lock:
            self.stby.off()
            self._enabled = False
            self.left.coast()
            self.right.coast()

    def stop(self):
        with self._lock:
            self.left.brake()
            self.right.brake()
            time.sleep(0.1)
            self.left.coast()
            self.right.coast()

    def drive(self, throttle, steer):
        """throttle and steer both -1.0 .. +1.0, mixed to two side speeds."""
        throttle = max(-1.0, min(1.0, float(throttle)))
        steer = max(-1.0, min(1.0, float(steer)))

        left = throttle + steer
        right = throttle - steer

        # Full throttle plus full steer would ask for 2.0. Scale the pair down
        # together rather than clipping, so a turn keeps its shape at speed.
        peak = max(1.0, abs(left), abs(right))
        left, right = left / peak, right / peak
        left *= self.limit
        right *= self.limit

        # Swap first (fixes which physical driver is "left"), then invert each.
        if self.invert["swap"]:
            left, right = right, left
        if self.invert["left"]:
            left = -left
        if self.invert["right"]:
            right = -right

        self._last_cmd = time.monotonic()
        with self._lock:
            self.left.drive(left)
            self.right.drive(right)

    def set_invert(self, changes):
        for k in INVERT_KEYS:
            if k in changes:
                self.invert[k] = bool(changes[k])
        save_invert(self.invert)

    def set_limit(self, v):
        self.limit = max(0.05, min(MAX_DUTY, float(v)))

    def reset_encoders(self):
        self.enc_left.steps = 0
        self.enc_right.steps = 0

    def _rpm_loop(self):
        while True:
            time.sleep(0.2)
            now = time.monotonic()
            dt = now - self._last_t
            l, r = self.enc_left.steps, self.enc_right.steps
            if dt > 0 and COUNTS_PER_REV > 0:
                self._rpm_l = ((l - self._last_l) / COUNTS_PER_REV) / dt * 60.0
                self._rpm_r = ((r - self._last_r) / COUNTS_PER_REV) / dt * 60.0
            self._last_l, self._last_r, self._last_t = l, r, now

    def _moving(self):
        return self.left._speed != 0.0 or self.right._speed != 0.0

    def _watchdog_loop(self):
        while True:
            time.sleep(0.1)
            if not self._moving():
                continue
            if time.monotonic() - self._last_cmd > WATCHDOG_S:
                print(f"  watchdog: no command for {WATCHDOG_S}s — stopping")
                self.drive(0, 0)
                self.stop()
                self._tripped = True

    @property
    def state(self):
        return {
            "enabled": self._enabled,
            "tripped": self._tripped,
            "invert": self.invert,
            "limit": round(self.limit, 3),
            "max_duty": MAX_DUTY,
            "left_speed": round(self.left._speed, 3),
            "right_speed": round(self.right._speed, 3),
            "left_rpm": round(self._rpm_l, 1),
            "right_rpm": round(self._rpm_r, 1),
            "rated_rpm": RATED_RPM,
        }

    def close(self):
        self.stop()
        self.estop()
        self.left.close()
        self.right.close()
        self.stby.close()


# ---------------------------------------------------------------------------
# LiDAR  (protocol verified against the device — see lidar_view.py)
# ---------------------------------------------------------------------------

def autodetect_lidar():
    ports = list(list_ports.comports())
    for p in ports:
        if any(k in (p.description or "").lower() for k in LIKELY_USB):
            return p.device
    # Never fall back to /dev/ttyAMA0 or /dev/ttyS0 — a Pi always has
    # those built-in UARTs with nothing attached, and picking one means
    # the viewer sits silent forever. A USB scanner always has a VID.
    usb = [p for p in ports if p.vid is not None]
    return usb[0].device if usb else None


def angle_correction(dist_mm):
    if dist_mm <= 0:
        return 0.0
    return math.degrees(math.atan(21.8 * (155.3 - dist_mm) / (155.3 * dist_mm)))


def cluster_points(points, max_gap_mm=180, max_ang_gap=8.0, min_pts=3, limit=14):
    """Group neighbouring returns into objects. A jump in range ends a group,
    because that is where an object's edge is."""
    if not points:
        return []
    pts = sorted(points)
    groups, cur = [], [pts[0]]
    for prev, p in zip(pts, pts[1:]):
        if (p[0] - prev[0]) > max_ang_gap or abs(p[1] - prev[1]) > max_gap_mm:
            groups.append(cur)
            cur = []
        cur.append(p)
    groups.append(cur)

    if len(groups) > 1:
        first, last = groups[0], groups[-1]
        if (first[0][0] + 360 - last[-1][0]) <= max_ang_gap and \
                abs(first[0][1] - last[-1][1]) <= max_gap_mm:
            groups[0] = last + first
            groups.pop()

    out = []
    for g in groups:
        if len(g) < min_pts:
            continue
        angs = [a for a, _ in g]
        dists = [d for _, d in g]
        if max(angs) - min(angs) > 180:
            angs = [a + 360 if a < 180 else a for a in angs]
        span = max(angs) - min(angs)
        mean_d = sum(dists) / len(dists)
        out.append({
            "bearing": round((sum(angs) / len(angs)) % 360, 1),
            "start": round(min(angs) % 360, 1),
            "span": round(span, 1),
            "near": round(min(dists)),
            "width": round(2 * mean_d * math.sin(math.radians(span) / 2)),
            "points": len(g),
        })
    out.sort(key=lambda c: c["near"])
    return out[:limit]


class Lidar:
    def __init__(self, port, baud=LIDAR_BAUD):
        self.port = port
        self.baud = baud
        self.buf = bytearray()
        self._lock = threading.Lock()
        self._current = []
        self._scan = []
        self._last_angle = None
        self._rev_times = deque(maxlen=20)
        self._scan_time = 0.0
        self.packets = 0
        self.bad = 0
        self.connected = False
        self.error = ""
        if port:
            threading.Thread(target=self._run, daemon=True).start()
        else:
            self.error = "no serial port found"

    def _run(self):
        while True:
            try:
                with serial.Serial(self.port, self.baud, timeout=0.2) as ser:
                    self.connected = True
                    self.error = ""
                    ser.reset_input_buffer()
                    while True:
                        chunk = ser.read(4096)
                        if chunk:
                            self.buf += chunk
                            self._consume()
                        elif len(self.buf) > 65536:
                            self.buf.clear()
            except serial.SerialException as e:
                self.connected = False
                self.error = str(e)
                time.sleep(1.5)

    def _consume(self):
        while True:
            idx = self.buf.find(b"\xAA\x55")
            if idx < 0:
                if len(self.buf) > 2:
                    del self.buf[:-1]
                return
            if idx:
                del self.buf[:idx]
            if len(self.buf) < 10:
                return
            ct, lsn = self.buf[2], self.buf[3]
            need = 10 + lsn * 2
            if len(self.buf) < need:
                return
            fsa, lsa, cs = struct.unpack("<HHH", self.buf[4:10])
            samples = struct.unpack(f"<{lsn}H", self.buf[10:need])
            check = 0x55AA ^ (ct | (lsn << 8)) ^ fsa ^ lsa
            for v in samples:
                check ^= v
            if check != cs:
                self.bad += 1
                del self.buf[:2]
                continue
            self.packets += 1
            self._emit(fsa, lsa, samples)
            del self.buf[:need]

    def _emit(self, fsa, lsa, samples):
        a0 = (fsa >> 1) / 64.0
        a1 = (lsa >> 1) / 64.0
        span = (a1 - a0) % 360.0
        n = len(samples)
        pts = []
        for i, raw in enumerate(samples):
            dist = raw / 4.0
            if dist <= 0:
                continue
            ang = (a0 + span * (i / (n - 1) if n > 1 else 0.0)) % 360.0
            pts.append((round((ang + angle_correction(dist)) % 360.0, 1), round(dist)))

        with self._lock:
            for ang, dist in pts:
                # Revolution boundary: the angle stepped backwards past 360.
                #
                # The point count guard matters. angle_correction can nudge a
                # reading near 0 deg back to ~359, and the next point at 1 deg
                # then looks like a wrap. That splits one revolution into two,
                # producing a TRUNCATED scan and a bogus rate (measured up to
                # 16 Hz on a scanner that physically turns at 11). A real
                # revolution always carries far more than 30 points.
                if (self._last_angle is not None and ang < self._last_angle - 180
                        and len(self._current) > 30):
                    self._scan = self._current
                    self._current = []
                    self._rev_times.append(time.monotonic())
                    self._scan_time = time.monotonic()
                self._last_angle = ang
                self._current.append((ang, dist))

    def hz(self):
        if len(self._rev_times) < 2:
            return 0.0
        span = self._rev_times[-1] - self._rev_times[0]
        return (len(self._rev_times) - 1) / span if span > 0 else 0.0

    def fresh(self, max_age=1.0):
        return self._scan_time > 0 and (time.monotonic() - self._scan_time) < max_age

    def scan(self):
        with self._lock:
            return list(self._scan)

    @staticmethod
    def sector_min(points, centre, half_width):
        """Nearest return within +/- half_width of a bearing."""
        best = None
        for ang, dist in points:
            delta = abs((ang - centre + 180) % 360 - 180)
            if delta <= half_width and (best is None or dist < best):
                best = dist
        return best

    def quadrants(self, points):
        return {
            "front": self.sector_min(points, 0, 45),
            "right": self.sector_min(points, 90, 45),
            "rear": self.sector_min(points, 180, 45),
            "left": self.sector_min(points, 270, 45),
        }


# ---------------------------------------------------------------------------
# Proximity guard
# ---------------------------------------------------------------------------

class Guard:
    """Blocks forward motion when something is close ahead.

    Reverse and turning are never blocked — you must always be able to back
    out of whatever you drove into.

    With the guard on and no fresh scan, forward is refused rather than
    allowed. A guard that silently stops guarding when its sensor dies is
    worse than no guard, so the failure is made loud instead: the page says
    exactly why forward is blocked, and the toggle is right there.
    """

    def __init__(self):
        self.enabled = True
        self.stop_mm = GUARD_STOP_MM
        self.blocked = False
        self.reason = ""

    def apply(self, throttle, steer, lidar):
        self.blocked = False
        self.reason = ""
        if not self.enabled or throttle <= 0:
            return throttle, steer

        if not lidar.fresh():
            self.blocked = True
            self.reason = "no fresh LiDAR scan"
            return 0.0, steer

        front = Lidar.sector_min(lidar.scan(), 0, GUARD_SECTOR_DEG)
        if front is not None and front < self.stop_mm:
            self.blocked = True
            self.reason = f"{front:.0f} mm ahead"
            return 0.0, steer
        return throttle, steer


app = Flask(__name__)
robot = None
lidar = None
guard = Guard()

PAGE = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Speaker Truck — Pilot</title>
<style>
:root{
  color-scheme: dark;
  --bg:#0d1117; --surface-1:#161b22; --surface-2:#1c2430; --border:#2a323d;
  --text-1:#fff; --text-2:#a9b4c0; --text-3:#6e7b8a; --grid:#242d3a;
  --series-1:#3987e5;  /* LEFT  */
  --series-2:#d95926;  /* RIGHT */
  --point:#3987e5;     /* lidar returns — radius already encodes range */
  --obj:#199e70;       /* one accent for all objects, see note in JS */
  --good:#3fb950; --warning:#d29922; --critical:#f85149;
}
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:ui-sans-serif,-apple-system,"Segoe UI",Roboto,sans-serif;
     background:var(--bg);color:var(--text-1);padding:20px;
     max-width:1240px;margin:0 auto;-webkit-font-smoothing:antialiased}
.mono{font-family:ui-monospace,"SF Mono",Consolas,monospace}
.num{font-variant-numeric:tabular-nums}

header{display:flex;align-items:center;gap:14px;flex-wrap:wrap;
       padding-bottom:16px;margin-bottom:18px;border-bottom:1px solid var(--border)}
header h1{font-size:1rem;font-weight:650;letter-spacing:.14em;text-transform:uppercase}
header h1 span{color:var(--text-3);font-weight:400}
.spacer{flex:1}
.badge{display:inline-flex;align-items:center;gap:7px;padding:5px 12px;border-radius:99px;
       font-size:.72rem;font-weight:700;letter-spacing:.06em;border:1px solid}
.badge .dot{width:7px;height:7px;border-radius:50%;background:currentColor}
.badge.on{color:var(--good);border-color:color-mix(in srgb,var(--good) 45%,transparent);
          background:color-mix(in srgb,var(--good) 12%,transparent)}
.badge.off{color:var(--critical);border-color:color-mix(in srgb,var(--critical) 45%,transparent);
           background:color-mix(in srgb,var(--critical) 12%,transparent)}
.badge.warn{color:var(--warning);border-color:color-mix(in srgb,var(--warning) 45%,transparent);
            background:color-mix(in srgb,var(--warning) 12%,transparent)}
.btns{display:flex;gap:8px;flex-wrap:wrap}
button{padding:9px 15px;border:1px solid var(--border);border-radius:7px;
       font-size:.77rem;font-weight:650;letter-spacing:.04em;cursor:pointer;
       background:var(--surface-2);color:var(--text-1);transition:filter .12s,transform .06s}
button:hover{filter:brightness(1.25)}
button:active{transform:translateY(1px)}
.b-enable{background:color-mix(in srgb,var(--good) 20%,var(--surface-2));
          border-color:color-mix(in srgb,var(--good) 40%,transparent);color:var(--good)}
.b-stop{background:color-mix(in srgb,var(--warning) 18%,var(--surface-2));
        border-color:color-mix(in srgb,var(--warning) 40%,transparent);color:var(--warning)}
.b-estop{background:var(--critical);border-color:var(--critical);color:#fff}

.grid{display:grid;grid-template-columns:minmax(360px,1fr) 300px;gap:16px}
@media(max-width:900px){.grid{grid-template-columns:1fr}}
.card{background:var(--surface-1);border:1px solid var(--border);border-radius:12px;padding:18px}
.card h2{font-size:.68rem;text-transform:uppercase;letter-spacing:.1em;
         color:var(--text-3);font-weight:650;margin-bottom:12px}
canvas#plot{width:100%;aspect-ratio:1;display:block}

.chips{display:flex;gap:6px;flex-wrap:wrap;margin-bottom:10px}
.chip{padding:6px 12px;border:1px solid var(--border);border-radius:99px;
      background:var(--surface-2);color:var(--text-2);cursor:pointer;
      font-size:.73rem;font-weight:600}
.chip.sel{background:color-mix(in srgb,var(--point) 22%,var(--surface-2));
          border-color:color-mix(in srgb,var(--point) 55%,transparent);color:var(--point)}

.pad{display:grid;grid-template-columns:repeat(3,58px);grid-template-rows:repeat(3,58px);
     gap:7px;justify-content:center;margin:2px auto 12px;touch-action:none}
.pad button{display:flex;align-items:center;justify-content:center;font-size:1.2rem;
            padding:0;border-radius:10px;user-select:none}
.pad button.on{background:color-mix(in srgb,var(--series-1) 35%,var(--surface-2));
               border-color:var(--series-1);color:#fff}
.pad .mid{font-size:.6rem;letter-spacing:.05em}
.hint{font-size:.7rem;color:var(--text-3);line-height:1.6;text-align:center}
.hint kbd{background:var(--surface-2);border:1px solid var(--border);border-radius:4px;
          padding:1px 5px;font-family:ui-monospace,monospace;font-size:.68rem;color:var(--text-2)}

.stat{display:flex;justify-content:space-between;align-items:baseline;
      padding:7px 0;border-bottom:1px solid var(--border);font-size:.77rem}
.stat:last-child{border-bottom:none}
.stat .k{color:var(--text-3)}
.stat .v{font-weight:600}
.rows{display:flex;gap:16px;justify-content:center;margin-top:10px}
.rd{text-align:center;flex:1}
.rd .hd{display:flex;align-items:center;gap:5px;justify-content:center;
        font-size:.64rem;font-weight:700;letter-spacing:.09em;color:var(--text-2)}
.sw{width:9px;height:9px;border-radius:3px}
.rd .v{font-size:1.4rem;font-weight:250;line-height:1.3}
.rd .s{font-size:.64rem;color:var(--text-3)}

.tog{display:flex;gap:7px;flex-wrap:wrap;margin-bottom:10px}
.tog button{flex:1;min-width:96px;font-size:.72rem;padding:8px 8px}
.tog button.on{background:color-mix(in srgb,var(--warning) 22%,var(--surface-2));
               border-color:color-mix(in srgb,var(--warning) 55%,transparent);color:var(--warning)}
.tog button.g-on{background:color-mix(in srgb,var(--good) 20%,var(--surface-2));
                 border-color:color-mix(in srgb,var(--good) 45%,transparent);color:var(--good)}

.ctl-hd{display:flex;justify-content:space-between;align-items:center;margin-bottom:6px}
.ctl-hd label{font-size:.7rem;color:var(--text-2);font-weight:600;letter-spacing:.05em}
.ctl-hd .v{font-size:.77rem;font-weight:600}
input[type=range]{width:100%;accent-color:var(--series-1);height:22px}
input[type=number]{width:100%;background:var(--surface-2);border:1px solid var(--border);
  color:var(--text-1);padding:6px 9px;border-radius:6px;font-size:.82rem}
label.lbl{display:block;font-size:.64rem;color:var(--text-3);margin-bottom:4px;
          letter-spacing:.07em;text-transform:uppercase;font-weight:650}

.banner{display:none;margin-bottom:14px;padding:10px 14px;border-radius:8px;
        font-size:.77rem;font-weight:600}
.banner.show{display:block}
.banner.warn{color:var(--warning);background:color-mix(in srgb,var(--warning) 12%,transparent);
             border:1px solid color-mix(in srgb,var(--warning) 40%,transparent)}
.banner.info{color:var(--text-2);background:var(--surface-2);border:1px solid var(--border)}
.note{font-size:.68rem;color:var(--text-3);line-height:1.55;margin-top:10px}
table{width:100%;border-collapse:collapse;font-size:.74rem;margin-top:2px}
th,td{text-align:right;padding:6px 8px;border-bottom:1px solid var(--border);
      font-variant-numeric:tabular-nums}
th{color:var(--text-3);font-weight:650;font-size:.62rem;text-transform:uppercase;letter-spacing:.05em}
td{color:var(--text-2)}
th:first-child,td:first-child{text-align:left}
td.n{color:var(--text-1);font-weight:600}
.idx{display:inline-flex;align-items:center;justify-content:center;width:18px;height:18px;
     border-radius:5px;background:var(--surface-2);border:1px solid var(--obj);
     color:var(--obj);font-size:.62rem;font-weight:700}
</style>
</head>
<body>

<header>
  <h1>Speaker Truck <span>/ pilot</span></h1>
  <span id="badge" class="badge off"><i class="dot"></i>DISABLED</span>
  <span id="lbadge" class="badge off"><i class="dot"></i>NO LIDAR</span>
  <div class="spacer"></div>
  <div class="btns">
    <button class="b-enable" onclick="cmd('/enable')">ENABLE</button>
    <button class="b-stop" onclick="cmd('/stop')">STOP</button>
    <button class="b-estop" onclick="cmd('/estop')">E-STOP</button>
  </div>
</header>

<div id="trip" class="banner warn">⚠ Watchdog tripped — commands stopped arriving, so the motors were stopped.</div>
<div id="blocked" class="banner warn"></div>
<div id="focusnote" class="banner info show">Click anywhere on the page once, then the arrow keys will drive.</div>

<div class="grid">
  <div class="card">
    <div class="chips" id="ranges"></div>
    <canvas id="plot"></canvas>
    <p class="note">
      The truck is drawn to scale at the centre, front upward — so a gap on
      screen is a gap it actually fits through. The shaded wedge is the
      guard's field of view; it turns amber when forward is blocked.
      Green arcs are detected objects, numbered to match the table below.
    </p>
  </div>

  <div class="card">
    <h2>Drive</h2>
    <div class="pad">
      <span></span><button id="p-f">↑</button><span></span>
      <button id="p-l">←</button><button class="mid" id="p-s">STOP</button><button id="p-r">→</button>
      <span></span><button id="p-b">↓</button><span></span>
    </div>
    <p class="hint">
      <kbd>↑</kbd><kbd>↓</kbd><kbd>←</kbd><kbd>→</kbd> or <kbd>W</kbd><kbd>A</kbd><kbd>S</kbd><kbd>D</kbd> — hold to drive
      <br><kbd>Space</kbd> — e-stop
    </p>

    <div class="rows">
      <div class="rd">
        <div class="hd"><i class="sw" style="background:var(--series-1)"></i>LEFT</div>
        <div class="v num mono" id="lr">0.0</div><div class="s">RPM</div>
      </div>
      <div class="rd">
        <div class="hd"><i class="sw" style="background:var(--series-2)"></i>RIGHT</div>
        <div class="v num mono" id="rr">0.0</div><div class="s">RPM</div>
      </div>
    </div>

    <div style="margin-top:14px">
      <div class="ctl-hd"><label for="lim">Speed limit</label><span id="limv" class="v num mono">0.40</span></div>
      <input type="range" id="lim" min="5" max="40" value="40" step="1">
    </div>

    <h2 style="margin-top:18px">Proximity guard</h2>
    <div class="tog">
      <button id="t-guard" onclick="toggleGuard()">Guard</button>
    </div>
    <label class="lbl" for="stopmm">Stop distance mm</label>
    <input type="number" id="stopmm" value="350" step="50" min="100" max="2000">
    <p class="note">
      Blocks forward motion only. Reverse and turning always stay available so
      you can back out of anything you drive into.
    </p>

    <h2 style="margin-top:18px">Direction inversion</h2>
    <div class="tog">
      <button id="t-left" onclick="toggle('left')">Inv L</button>
      <button id="t-right" onclick="toggle('right')">Inv R</button>
      <button id="t-swap" onclick="toggle('swap')">Swap</button>
    </div>

    <h2 style="margin-top:18px">Scan</h2>
    <div class="stat"><span class="k">Rate</span><span class="v num mono" id="hz">—</span></div>
    <div class="stat"><span class="k">Points / turn</span><span class="v num mono" id="count">—</span></div>
    <div class="stat"><span class="k">Ahead</span><span class="v num mono" id="ahead">—</span></div>
    <div class="stat"><span class="k">Bad checksums</span><span class="v num mono" id="bad">—</span></div>
  </div>
</div>

<div class="card" style="margin-top:16px">
  <h2>Detected objects · nearest first</h2>
  <table>
    <thead><tr><th>#</th><th>Bearing</th><th>Nearest</th><th>Width</th><th>Arc</th><th>Rays</th></tr></thead>
    <tbody id="objs"></tbody>
  </table>
</div>

<script>
const $ = id => document.getElementById(id);
const TRUCK = {len: %%TRUCK_LEN%%, wid: %%TRUCK_WID%%};

let maxRange = 4000, pts = [], clusters = [], quad = {}, nearestAhead = null;
let rpmL = 0, rpmR = 0, phaseL = 0, phaseR = 0, lastFrame = performance.now();
let guardOn = true, guardBlocked = false, sectorDeg = %%SECTOR%%, stopMm = 350;

const RANGES = [1000, 2000, 4000, 8000];
$('ranges').innerHTML = RANGES.map(r => `<button class="chip" data-r="${r}">${r/1000} m</button>`).join('');
$('ranges').querySelectorAll('.chip').forEach(b => b.addEventListener('click', () => {
  maxRange = +b.dataset.r; markRange();
}));
function markRange(){
  $('ranges').querySelectorAll('.chip').forEach(b => b.classList.toggle('sel', +b.dataset.r === maxRange));
}
markRange();

const css = k => getComputedStyle(document.documentElement).getPropertyValue(k).trim();
const mm = v => v == null ? '—' : (v >= 1000 ? (v/1000).toFixed(2)+' m' : Math.round(v)+' mm');

// ---------------------------------------------------------------- plot
function draw(){
  const now = performance.now(), dt = Math.min(0.1, (now - lastFrame)/1000);
  lastFrame = now;
  // wheel tick phase advances at the real measured RPM
  phaseL = (phaseL + rpmL/60*360*dt) % 360;
  phaseR = (phaseR + rpmR/60*360*dt) % 360;

  const c = $('plot'), g = c.getContext('2d');
  const dpr = devicePixelRatio || 1, w = c.clientWidth, h = c.clientHeight;
  if(!w) return;
  c.width = w*dpr; c.height = h*dpr; g.setTransform(dpr,0,0,dpr,0,0);
  g.clearRect(0,0,w,h);

  const cx = w/2, cy = h/2, R = Math.min(w,h)/2 - 26;
  const rOf = d => Math.min(d, maxRange)/maxRange*R;
  const XY = (angDeg, d) => {
    const a = (angDeg - 90)*Math.PI/180, r = rOf(d);
    return [cx + Math.cos(a)*r, cy + Math.sin(a)*r];
  };

  // guard wedge — drawn first, under everything
  if(guardOn){
    const a0 = (-sectorDeg - 90)*Math.PI/180, a1 = (sectorDeg - 90)*Math.PI/180;
    g.beginPath(); g.moveTo(cx, cy); g.arc(cx, cy, rOf(stopMm), a0, a1); g.closePath();
    g.fillStyle = guardBlocked ? css('--warning') : css('--text-3');
    g.globalAlpha = guardBlocked ? .22 : .09; g.fill(); g.globalAlpha = 1;
    g.strokeStyle = guardBlocked ? css('--warning') : css('--grid');
    g.lineWidth = 1; g.stroke();
  }

  // range rings
  g.font = '10px ui-monospace,monospace';
  g.textAlign = 'center'; g.textBaseline = 'middle';
  const stepM = maxRange <= 2000 ? 0.5 : maxRange <= 4000 ? 1 : 2;
  for(let m = stepM; m*1000 <= maxRange + 1; m += stepM){
    const r = rOf(m*1000);
    g.strokeStyle = css('--grid'); g.lineWidth = 1;
    g.beginPath(); g.arc(cx, cy, r, 0, 6.2832); g.stroke();
    g.fillStyle = css('--text-3'); g.fillText(m+' m', cx, cy - r - 8);
  }
  for(let a = 0; a < 360; a += 45){
    const rad = (a - 90)*Math.PI/180;
    g.strokeStyle = css('--grid'); g.globalAlpha = .6;
    g.beginPath(); g.moveTo(cx, cy); g.lineTo(cx+Math.cos(rad)*R, cy+Math.sin(rad)*R);
    g.stroke(); g.globalAlpha = 1;
  }
  g.fillStyle = css('--text-2'); g.font = '11px ui-monospace,monospace';
  g.fillText('FRONT', cx, cy - R - 18);
  g.fillText('180°', cx, cy + R + 18);
  g.fillText('90°', cx + R + 16, cy);
  g.fillText('270°', cx - R - 16, cy);

  // returns — one colour; the radius already encodes distance
  g.fillStyle = css('--point');
  for(const [a, d] of pts){
    if(d > maxRange) continue;
    const [x, y] = XY(a, d);
    g.beginPath(); g.arc(x, y, 1.9, 0, 6.2832); g.fill();
  }

  // objects — ONE accent, not a colour each: cluster identity is not stable
  // between frames, so per-cluster hues would flicker and mean nothing.
  g.font = '10px ui-monospace,monospace';
  clusters.forEach((cl, i) => {
    if(cl.near > maxRange) return;
    const r = rOf(cl.near);
    g.strokeStyle = css('--obj'); g.lineWidth = 2.5;
    g.beginPath();
    g.arc(cx, cy, r, (cl.start-90)*Math.PI/180, (cl.start+cl.span-90)*Math.PI/180);
    g.stroke();
    const mid = (cl.bearing - 90)*Math.PI/180;
    g.fillStyle = css('--obj');
    g.fillText(i+1, cx+Math.cos(mid)*(r+13), cy+Math.sin(mid)*(r+13));
  });

  drawTruck(g, cx, cy, R);
}

// The truck, to scale, front up. Same mm-per-pixel as the scan, so what looks
// like clearance on screen IS clearance.
function drawTruck(g, cx, cy, R){
  const s = R/maxRange;                       // pixels per mm
  const L = TRUCK.len*s, W = TRUCK.wid*s;
  if(L < 6){                                  // too small to draw at this range
    g.fillStyle = css('--text-2');
    g.beginPath(); g.arc(cx, cy, 3, 0, 6.2832); g.fill();
    return;
  }
  g.save();
  g.translate(cx, cy);

  g.fillStyle = css('--surface-2');
  g.strokeStyle = css('--text-3'); g.lineWidth = 1.2;
  g.beginPath(); g.roundRect(-W/2, -L/2, W, L, Math.min(6, W/5));
  g.fill(); g.stroke();

  // heading arrow
  g.strokeStyle = css('--text-2'); g.lineWidth = 1.5;
  g.beginPath(); g.moveTo(0, L/2*0.3); g.lineTo(0, -L/2*0.55);
  g.moveTo(-W*0.13, -L/2*0.34); g.lineTo(0, -L/2*0.55); g.lineTo(W*0.13, -L/2*0.34);
  g.stroke();

  // four wheels, each with a tick that turns at the measured RPM
  const wl = L*0.26, ww = Math.max(2.5, W*0.15);
  for(const [sx, sy, side] of [[-1,-1,'l'], [1,-1,'r'], [-1,1,'l'], [1,1,'r']]){
    const x = sx*(W/2), y = sy*(L*0.28);
    g.fillStyle = side === 'l' ? css('--series-1') : css('--series-2');
    g.globalAlpha = .9;
    g.beginPath(); g.roundRect(x-ww/2, y-wl/2, ww, wl, ww/2.5); g.fill();
    g.globalAlpha = 1;
    if(wl > 8){
      const ph = (side === 'l' ? phaseL : phaseR)*Math.PI/180;
      g.strokeStyle = '#fff'; g.lineWidth = 1.2; g.globalAlpha = .85;
      g.beginPath();
      g.moveTo(x - ww/2, y + Math.sin(ph)*wl/2*0.8);
      g.lineTo(x + ww/2, y + Math.sin(ph)*wl/2*0.8);
      g.stroke(); g.globalAlpha = 1;
    }
  }
  g.restore();
}

// ---------------------------------------------------------------- input
const keys = new Set();
const MAP = {ArrowUp:'f', KeyW:'f', ArrowDown:'b', KeyS:'b',
             ArrowLeft:'l', KeyA:'l', ArrowRight:'r', KeyD:'r'};
const PADS = {'p-f':'f', 'p-b':'b', 'p-l':'l', 'p-r':'r'};
function paint(){ for(const [id,k] of Object.entries(PADS)) $(id).classList.toggle('on', keys.has(k)); }

function sendDrive(){
  const throttle = (keys.has('f')?1:0) - (keys.has('b')?1:0);
  const steer    = (keys.has('r')?1:0) - (keys.has('l')?1:0);
  fetch('/drive', {method:'POST', headers:{'Content-Type':'application/json'},
                   body: JSON.stringify({throttle, steer})});
}
addEventListener('keydown', e => {
  if(e.code === 'Space'){ e.preventDefault(); cmd('/estop'); keys.clear(); paint(); return; }
  const k = MAP[e.code];
  if(!k || e.repeat) return;
  e.preventDefault();                 // stop the arrow keys scrolling the page
  keys.add(k); paint();
});
addEventListener('keyup', e => { const k = MAP[e.code]; if(k){ e.preventDefault(); keys.delete(k); paint(); } });
addEventListener('blur', () => { keys.clear(); paint(); });   // never latch a key on

for(const [id,k] of Object.entries(PADS)){
  const el = $(id);
  const on = e => { e.preventDefault(); keys.add(k); paint(); };
  const off = e => { e.preventDefault(); keys.delete(k); paint(); };
  el.addEventListener('pointerdown', on);
  ['pointerup','pointerleave','pointercancel'].forEach(ev => el.addEventListener(ev, off));
}
$('p-s').addEventListener('click', () => { keys.clear(); paint(); cmd('/stop'); });

let wasActive = false;
setInterval(() => {
  const active = keys.size > 0;
  if(active || wasActive) sendDrive();
  wasActive = active;
}, 50);

// ---------------------------------------------------------------- commands
function cmd(u){ fetch(u, {method:'POST'}).then(poll); }
function post(u, b){ return fetch(u, {method:'POST', headers:{'Content-Type':'application/json'},
                                      body: JSON.stringify(b)}).then(poll); }
function toggle(k){ post('/invert', {[k]: !$('t-'+k).classList.contains('on')}); }
function toggleGuard(){ post('/guard', {enabled: !guardOn}); }
$('stopmm').addEventListener('change', e => post('/guard', {stop_mm: +e.target.value}));
$('lim').addEventListener('input', e => {
  const v = e.target.value/100;
  $('limv').textContent = v.toFixed(2);
  post('/limit', {value: v});
});
addEventListener('click', () => $('focusnote').classList.remove('show'), {once:true});

function poll(){
  fetch('/state').then(r => r.json()).then(d => {
    const m = d.motors, l = d.lidar, gd = d.guard;

    $('badge').className = 'badge ' + (m.enabled ? 'on' : 'off');
    $('badge').innerHTML = '<i class="dot"></i>' + (m.enabled ? 'ENABLED' : 'DISABLED');
    $('lbadge').className = 'badge ' + (l.connected && l.count ? 'on' : 'off');
    $('lbadge').innerHTML = '<i class="dot"></i>' + (l.connected && l.count ? 'SCANNING' : 'NO LIDAR');
    $('trip').classList.toggle('show', !!m.tripped);

    guardBlocked = gd.blocked;
    $('blocked').classList.toggle('show', gd.blocked);
    $('blocked').textContent = '⛔ Forward blocked — ' + gd.reason;

    rpmL = m.left_rpm; rpmR = m.right_rpm;
    $('lr').textContent = m.left_rpm.toFixed(1);
    $('rr').textContent = m.right_rpm.toFixed(1);

    for(const k of ['left','right','swap']) $('t-'+k).classList.toggle('on', !!m.invert[k]);
    guardOn = gd.enabled; stopMm = gd.stop_mm;
    $('t-guard').className = guardOn ? 'g-on' : '';
    $('t-guard').textContent = guardOn ? 'Guard ON' : 'Guard OFF';
    if(document.activeElement !== $('stopmm')) $('stopmm').value = gd.stop_mm;
    if(document.activeElement !== $('lim')){
      $('lim').value = Math.round(m.limit*100);
      $('limv').textContent = m.limit.toFixed(2);
    }
    $('lim').max = Math.round(m.max_duty*100);

    pts = l.points; clusters = l.clusters || []; quad = l.quadrants || {};
    nearestAhead = l.ahead;
    $('hz').textContent = l.hz.toFixed(1) + ' Hz';
    $('count').textContent = l.count;
    $('ahead').textContent = mm(l.ahead);
    $('bad').textContent = l.bad;

    $('objs').innerHTML = clusters.filter(c => c.near <= maxRange).map((c,i) =>
        `<tr><td><span class="idx">${i+1}</span></td>`
      + `<td class="n">${c.bearing.toFixed(0)}°</td><td class="n">${mm(c.near)}</td>`
      + `<td>${mm(c.width)}</td><td>${c.span.toFixed(0)}°</td><td>${c.points}</td></tr>`).join('');
  }).catch(() => {
    $('badge').className = 'badge off';
    $('badge').innerHTML = '<i class="dot"></i>NO LINK';
  });
}
setInterval(poll, 150);
// redraw independently so the wheel ticks stay smooth between polls
setInterval(draw, 40);
addEventListener('resize', draw);
poll();
</script>
</body>
</html>"""

PAGE = (PAGE.replace("%%TRUCK_LEN%%", str(TRUCK_LEN_MM))
            .replace("%%TRUCK_WID%%", str(TRUCK_WIDTH_MM))
            .replace("%%SECTOR%%", str(GUARD_SECTOR_DEG)))


@app.route("/")
def index():
    return render_template_string(PAGE)


@app.route("/state")
def state():
    pts = lidar.scan()
    return jsonify(
        motors=robot.state,
        lidar={
            "points": pts,
            "count": len(pts),
            "clusters": cluster_points(pts),
            "quadrants": lidar.quadrants(pts),
            "ahead": Lidar.sector_min(pts, 0, GUARD_SECTOR_DEG),
            "hz": round(lidar.hz(), 1),
            "bad": lidar.bad,
            "connected": lidar.connected,
            "error": lidar.error,
        },
        guard={
            "enabled": guard.enabled,
            "stop_mm": guard.stop_mm,
            "blocked": guard.blocked,
            "reason": guard.reason,
        },
    )


@app.route("/drive", methods=["POST"])
def drive():
    d = request.get_json(force=True, silent=True) or {}
    throttle, steer = guard.apply(d.get("throttle", 0), d.get("steer", 0), lidar)
    robot.drive(throttle, steer)
    return "", 204


@app.route("/guard", methods=["POST"])
def set_guard():
    d = request.get_json(force=True, silent=True) or {}
    if "enabled" in d:
        guard.enabled = bool(d["enabled"])
    if "stop_mm" in d:
        guard.stop_mm = max(100, min(2000, float(d["stop_mm"])))
    return jsonify(enabled=guard.enabled, stop_mm=guard.stop_mm)


@app.route("/invert", methods=["POST"])
def invert():
    robot.set_invert(request.get_json(force=True, silent=True) or {})
    return jsonify(robot.invert)


@app.route("/limit", methods=["POST"])
def limit():
    d = request.get_json(force=True, silent=True) or {}
    robot.set_limit(d.get("value", MAX_DUTY))
    return jsonify(limit=robot.limit)


@app.route("/enable", methods=["POST"])
def enable():
    robot.enable()
    return "", 204


@app.route("/estop", methods=["POST"])
def estop():
    robot.estop()
    return "", 204


@app.route("/stop", methods=["POST"])
def stop():
    robot.stop()
    return "", 204


def lan_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("192.168.1.1", 1))       # no packet is sent for UDP connect
        return s.getsockname()[0]
    except OSError:
        return "127.0.0.1"
    finally:
        s.close()


def main():
    global robot, lidar
    ap = argparse.ArgumentParser(description="drive + lidar on one page")
    ap.add_argument("-p", "--lidar-port", help="serial port (default: auto-detect)")
    ap.add_argument("-b", "--baud", type=int, default=LIDAR_BAUD)
    ap.add_argument("--http-port", type=int, default=HTTP_PORT)
    ap.add_argument("--no-guard", action="store_true",
                    help="start with the proximity guard off")
    args = ap.parse_args()

    if args.no_guard:
        guard.enabled = False

    try:
        robot = Robot()
    except Exception as exc:                                  # noqa: BLE001
        print(f"\n  GPIO setup failed: {exc}")
        print("  Is web_drive.py or web_dashboard.py already running?")
        print("  Only one script can own these pins at a time.\n")
        raise

    port = args.lidar_port or autodetect_lidar()
    lidar = Lidar(port, args.baud)

    inv = robot.invert
    print("\n  Speaker Truck — pilot")
    print(f"  http://{lan_ip()}:{args.http_port}   (click the page, then arrow keys)")
    print(f"  lidar: {port or 'NONE FOUND'} @ {args.baud}")
    print(f"  guard: {'on' if guard.enabled else 'off'}, stop at {guard.stop_mm:.0f} mm")
    print(f"  invert: left={inv['left']} right={inv['right']} swap={inv['swap']}")
    print(f"  MAX_DUTY {MAX_DUTY:.2f} · watchdog {WATCHDOG_S}s")
    print("\n  *** WHEELS OFF THE GROUND ***\n")
    try:
        app.run(host="0.0.0.0", port=args.http_port, threaded=True)
    finally:
        # Runs on Ctrl-C and on any exception, so the motors never stay on.
        robot.close()
        print("\nMotors stopped, STBY low, pins released.")


if __name__ == "__main__":
    main()
