"""
Nav — drive, LiDAR and IMU on one page. The full cockpit.

    cd ~/Desktop/Speaker_truck && source .venv/bin/activate
    python test/web_nav.py

Then open  http://<pi-ip>:5004  and click the page once for keyboard focus.

*** WHEELS OFF THE GROUND until the drivers are replaced. The TB6612s cannot
*** survive a stall on these motors (JGB37-520 stalls at 4-5 A, TB6612 peaks
*** at 3.2 A). WIRING.md section 8.

This supersedes web_pilot.py — same drive and LiDAR, plus orientation.

Ports: web_dashboard 5000, web_drive 5001, lidar_view 5002, web_pilot 5003,
this 5004. Only ONE of the GPIO-owning pages runs at a time; they claim the
same pins.

What the IMU adds beyond a pretty dial
--------------------------------------
  Heading      A north marker on the LiDAR ring. The scan is drawn in the
               robot's own frame, so without this you cannot tell which way
               the robot is actually pointing in the room.
  Attitude     Roll and pitch, with a tilt warning. A skid-steer robot on a
               ramp loses traction long before it tips.
  Slip check   Differential drive says yaw rate = (v_right - v_left) / track.
               The gyro measures yaw rate directly. When the two disagree,
               the wheels are slipping and the encoder odometry is lying.
               That is a thing only having both sensors can tell you.

Speaker
-------
  Audio tab    Upload MP3s from the browser and play them — pause, seek, next,
               play-all or repeat, volume/bass/treble — plus the tone and
               sweep test for bringing the amp up.
  Horn         Beep, horn, chirp, reverse and alert on the Drive tab, and the
               H key. A beep over a song holds the song and resumes it.

The player is audio.py, shared with web_dashboard.py and speaker_test.py.
Songs live in test/uploads/ on the Pi.

Speech
------
Type on the Audio tab (or the Say box on Drive) and the truck speaks it with
a Piper neural voice — offline, on the Pi, in a low-priority worker process
so SLAM keeps its CPU. Speech holds a playing song like a beep does. tts.py.

Claude at the wheel
-------------------
tools/truck_mcp.py (runs on the PC) gives Claude tools to look through the
camera, read the obstacles around the truck, drive in short bounded moves,
speak and beep — all through this server's HTTP API, with the guard and the
watchdog still in charge. /ai/status is the compact view it reads. A person
still has to press ENABLE: the AI can drive the motors, never arm them.

A phone is the microphone: https://<pi-ip>:5443/talk (voice.py). What is
said there is answered out loud by brain.py — the on-board assistant, which
asks a free local model in Ollama on your PC and streams the reply into the
speaker sentence by sentence. No API keys. Turn it off on the Audio tab to
talk through Claude Code and the MCP `listen` tool instead.

Display
-------
The 1.54" ST7789 on the truck shows the address to open this page at, drive
state, sensors, the guard, the pose and the song playing. The Sensors tab
mirrors it and can blank it or show the test card. display.py; wiring in
WIRING.md section 14. --no-display leaves the panel alone.

Every part degrades on its own: no IMU still gives drive + LiDAR, no LiDAR
still gives drive + IMU, no sound card still gives everything else. Nothing
here refuses to start because a sensor is missing — it says so on the page
instead.

Needs:  sudo apt install -y python3-serial i2c-tools python3-smbus2 alsa-utils ffmpeg \
                            python3-spidev python3-pil python3-numpy
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
    IMU_YAW_OFFSET, IMU_ROLL_OFFSET, IMU_PITCH_OFFSET,
    TRUCK_LENGTH_MM, TRUCK_WIDTH_MM, TRACK_WIDTH_MM, WHEEL_DIAM_MM,
    LIDAR_OFFSET_X, LIDAR_OFFSET_Y, LIDAR_YAW_OFFSET, SAFETY_MARGIN_MM,
    CREEP_MARGIN_MM, CREEP_THROTTLE,
    CAM_SIZE, CAM_FPS, CAM_HFLIP, CAM_VFLIP, CAM_HFOV, CAM_YAW_OFFSET,
    CAM_HEIGHT_MM, CAM_PITCH_DEG, MARKER_SIZE_MM,
    CAM_OFFSET_X, CAM_OFFSET_Y, IMU_OFFSET_X, IMU_OFFSET_Y, CAM_ROTATION,
)

import tuning  # noqa: E402
import audio  # noqa: E402
import display  # noqa: E402
import tts  # noqa: E402
import voice  # noqa: E402
import brain  # noqa: E402
from slam import Slam  # noqa: E402
from explore import Explorer  # noqa: E402
from gpiozero import DigitalOutputDevice, PWMOutputDevice, RotaryEncoder  # noqa: E402
from flask import (  # noqa: E402
    Flask, Response, jsonify, render_template_string, request,
)

try:
    import serial
    import serial.tools.list_ports as list_ports
except ImportError:
    serial = None
    list_ports = None

HTTP_PORT = 5004
LIDAR_BAUD = 115200
WATCHDOG_S = 0.6
RATED_RPM = 330

GUARD_STOP_MM = 350
GUARD_SECTOR_DEG = 50

TRUCK_LEN_MM = TRUCK_LENGTH_MM      # geometry now lives in pins.py

TILT_WARN_DEG = 20        # a skid-steer loses traction well before it tips

_HERE = os.path.dirname(os.path.abspath(__file__))
INVERT_FILE = os.path.join(_HERE, "drive_invert.json")
MAP_FILE = os.path.join(_HERE, "house_map.json")
PLACES_FILE = os.path.join(_HERE, "places.json")
LIDAR_CAL_FILE = os.path.join(_HERE, "lidar_cal.json")


class LidarCal:
    """Where the scanner is bolted on, adjustable while running.

    These are mounting facts, and mounting facts are discovered by looking at
    the plot and nudging a number - not by editing a constant, restarting, and
    trying to remember what the last value looked like. pins.py holds the
    defaults; whatever is tuned here is saved and wins on the next start.
    """

    def __init__(self):
        self.yaw = LIDAR_YAW_OFFSET
        self.x = LIDAR_OFFSET_X
        self.y = LIDAR_OFFSET_Y
        self.load()

    def load(self):
        try:
            with open(LIDAR_CAL_FILE) as f:
                d = json.load(f)
            self.yaw = float(d.get("yaw", self.yaw))
            self.x = float(d.get("x", self.x))
            self.y = float(d.get("y", self.y))
        except (OSError, ValueError, TypeError):
            pass

    def save(self):
        try:
            tmp = LIDAR_CAL_FILE + ".tmp"
            with open(tmp, "w") as f:
                json.dump({"yaw": self.yaw, "x": self.x, "y": self.y}, f, indent=2)
            os.replace(tmp, LIDAR_CAL_FILE)
        except OSError:
            pass

    def set(self, yaw=None, x=None, y=None):
        if yaw is not None:
            self.yaw = ((float(yaw) + 180) % 360) - 180
        if x is not None:
            self.x = max(-600.0, min(600.0, float(x)))
        if y is not None:
            self.y = max(-600.0, min(600.0, float(y)))
        self.save()
        # SLAM keeps its own copy for the grid and the matcher; without this
        # the guard would use the new value while the map kept using the old.
        if slam is not None:
            off = (self.x, self.y, self.yaw)
            slam.slam.grid.lidar_off = off
            slam.slam.matcher.off = off

    @property
    def as_dict(self):
        return {"yaw": round(self.yaw, 1), "x": round(self.x, 1),
                "y": round(self.y, 1)}


lidar_cal = None
INVERT_KEYS = ("left", "right", "swap")
LIKELY_USB = ("cp210", "ch340", "ch9102", "silicon labs", "usb-serial", "ftdi")


def load_places():
    """Named coordinates: {"kitchen": [x, y], ...}.

    This is what turns a map into somewhere you can send the robot. Naming a
    spot you have driven to is deliberately manual — auto-detecting rooms is a
    much harder problem and buys nothing until the basics work.
    """
    try:
        with open(PLACES_FILE) as f:
            d = json.load(f)
        return {str(k): [float(v[0]), float(v[1])] for k, v in d.items()}
    except (OSError, ValueError, TypeError, KeyError, IndexError):
        return {}


def save_places(d):
    try:
        tmp = PLACES_FILE + ".tmp"
        with open(tmp, "w") as f:
            json.dump(d, f, indent=2)
        os.replace(tmp, PLACES_FILE)
        return True
    except OSError:
        return False


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
    """One TB6612 channel pair.

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
        duty = min(abs(speed), MAX_DUTY)
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
        throttle = max(-1.0, min(1.0, float(throttle)))
        steer = max(-1.0, min(1.0, float(steer)))
        left, right = throttle + steer, throttle - steer
        # Scale the pair together rather than clipping, so a turn keeps its
        # shape at speed instead of straightening out.
        peak = max(1.0, abs(left), abs(right))
        left, right = left / peak * self.limit, right / peak * self.limit
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

    def encoder_yaw_rate(self):
        """Yaw rate the wheels claim, deg/s.

        Differential drive: omega = (v_right - v_left) / track_width.
        Compared against the gyro, a mismatch means the wheels are slipping.
        """
        circ_mm = math.pi * WHEEL_DIAM_MM
        v_l = self._rpm_l / 60.0 * circ_mm          # mm/s
        v_r = self._rpm_r / 60.0 * circ_mm
        if TRACK_WIDTH_MM <= 0:
            return 0.0
        return math.degrees((v_r - v_l) / TRACK_WIDTH_MM)

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
            "enc_yaw_rate": round(self.encoder_yaw_rate(), 1),
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
    if list_ports is None:
        return None
    ports = list(list_ports.comports())
    for p in ports:
        if any(k in (p.description or "").lower() for k in LIKELY_USB):
            return p.device
    # Never fall back to /dev/ttyAMA0 or /dev/ttyS0 — a Pi always has
    # those built-in UARTs with nothing attached, and picking one means
    # the viewer sits silent forever. A USB scanner always has a VID.
    usb = [p for p in ports if p.vid is not None]
    return usb[0].device if usb else None


def angle_correction(d):
    return 0.0 if d <= 0 else math.degrees(
        math.atan(21.8 * (155.3 - d) / (155.3 * d)))


def cluster_points(points, max_gap_mm=180, max_ang_gap=8.0, min_pts=3, limit=14):
    """Neighbouring rays become one object; a jump in range ends the group."""
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
        self.port, self.baud = port, baud
        self.buf = bytearray()
        self._lock = threading.Lock()
        self._current, self._scan = [], []
        self._last_angle = None
        self._last_pkt_a0 = None
        self._ct_seen = -1e9
        self._rev_times = deque(maxlen=20)
        self._scan_time = 0.0
        self.packets = self.bad = 0
        self.connected = False
        self.error = "" if port else "no serial port found"
        if port and serial:
            threading.Thread(target=self._run, daemon=True).start()
        elif not serial:
            self.error = "pyserial not installed"

    # No complete revolution for this long = the scanner has stopped talking.
    STALL_S = 2.5
    # ...but after opening the port, allow the motor time to spin up first:
    # some YDLIDAR adapters restart it on open, and reopening every 2.5 s
    # kept it from ever reaching speed.
    STARTUP_S = 10.0

    def _run(self):
        """Read forever, whatever goes wrong.

        Seen live: the port stayed open, nothing arrived, and the page went on
        showing "connected, 11.5 Hz" from the last good scans while the guard
        refused every move for a stale scan. So: any exception reconnects
        (only SerialException used to — anything else killed this thread
        silently), and a port that goes quiet is closed and reopened."""
        while True:
            try:
                with serial.Serial(self.port, self.baud, timeout=0.2) as ser:
                    self.connected, self.error = True, ""
                    ser.reset_input_buffer()
                    self.buf.clear()
                    opened = time.monotonic()
                    while True:
                        chunk = ser.read(4096)
                        if chunk:
                            self.rx_bytes = getattr(self, "rx_bytes", 0) + len(chunk)
                            self.buf += chunk
                            self._consume()
                        elif len(self.buf) > 65536:
                            self.buf.clear()
                        now = time.monotonic()
                        if self._scan_time > opened:
                            quiet, limit = now - self._scan_time, self.STALL_S
                        else:
                            quiet, limit = now - opened, self.STARTUP_S
                        if quiet > limit:
                            raise OSError("no complete scan for %.0f s (%d bytes received"
                                          " in total) - reopening the port"
                                          % (quiet, getattr(self, "rx_bytes", 0)))
            except Exception as e:                                # noqa: BLE001
                self.connected, self.error = False, str(e)
                self.stalls = getattr(self, "stalls", 0) + 1
                time.sleep(1.0)

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
            chk = 0x55AA ^ (ct | (lsn << 8)) ^ fsa ^ lsa
            for v in samples:
                chk ^= v
            if chk != cs:
                self.bad += 1
                del self.buf[:2]
                continue
            self.packets += 1
            self._emit(fsa, lsa, samples, ct)
            del self.buf[:need]

    # A revolution is never this big (~240 points at 11 Hz); if the start
    # marker is somehow missed, close it anyway rather than grow forever.
    MAX_REV_POINTS = 1000

    def _close_rev(self):
        self._scan, self._current = self._current, []
        self._rev_times.append(time.monotonic())
        self._scan_time = time.monotonic()
        self._last_angle = None

    def _emit(self, fsa, lsa, samples, ct=0):
        a0, a1 = (fsa >> 1) / 64.0, (lsa >> 1) / 64.0
        span = (a1 - a0) % 360.0
        n = len(samples)
        pts = []
        for i, raw in enumerate(samples):
            d = raw / 4.0
            if d <= 0:
                continue
            a = (a0 + span * (i / (n - 1) if n > 1 else 0.0)) % 360.0
            pts.append((round((a + angle_correction(d)) % 360.0, 1), round(d)))
        with self._lock:
            # ONE boundary per revolution, from ONE source.
            #
            # The scanner flags the first packet of every revolution (CT bit
            # 0) - but not necessarily at 0 degrees. Using that flag AND the
            # angle wrap at 0 split every turn in two: 22.8 "scans" a second,
            # each half a circle, seen live. So: while the flag is arriving,
            # it alone decides. Only a scanner that never sends it falls back
            # to the wrap of each PACKET's start angle (present even when all
            # of its distances are invalid - a per-POINT wrap never fired with
            # the view half blocked, which froze the guard).
            now = time.monotonic()
            if ct & 0x01:
                self._ct_seen = now
            if now - self._ct_seen < 2.0:
                boundary = bool(ct & 0x01)
            else:
                boundary = (self._last_pkt_a0 is not None
                            and a0 < self._last_pkt_a0 - 180)
            self._last_pkt_a0 = a0
            if boundary and len(self._current) > 10:
                self._close_rev()
            for a, d in pts:
                if len(self._current) >= self.MAX_REV_POINTS:
                    self._close_rev()           # a lost marker must not grow forever
                self._current.append((a, d))

    def hz(self):
        # A rate from revolutions that stopped arriving is a lie; 0 says so.
        if len(self._rev_times) < 2 or not self.fresh(1.5):
            return 0.0
        s = self._rev_times[-1] - self._rev_times[0]
        return (len(self._rev_times) - 1) / s if s > 0 else 0.0

    def fresh(self, max_age=1.0):
        return self._scan_time > 0 and (time.monotonic() - self._scan_time) < max_age

    def scan(self):
        with self._lock:
            return list(self._scan)

    @staticmethod
    def sector_min(points, centre, half):
        best = None
        for a, d in points:
            if abs((a - centre + 180) % 360 - 180) <= half and (best is None or d < best):
                best = d
        return best


# ---------------------------------------------------------------------------
# IMU — polled in its own thread so fusion gets an even timestep
# ---------------------------------------------------------------------------

class ImuReader:
    def __init__(self, hz=50):
        self.dev = None
        self.data = None
        self.error = ""
        self.name = ""
        self._hz = hz
        try:
            from imu import IMU
            self.dev = IMU()
            self.name = self.dev.name
            threading.Thread(target=self._run, daemon=True).start()
        except Exception as e:                                # noqa: BLE001
            self.error = str(e)

    def _run(self):
        period = 1.0 / self._hz
        while True:
            try:
                self.data = self.dev.read()
                self.error = ""
            except OSError as e:
                self.error = str(e)                # a wobbly wire, keep trying
            time.sleep(period)

    @property
    def state(self):
        if self.dev is None:
            return {"present": False, "error": self.error or "not detected"}
        d = self.data
        if d is None:
            return {"present": True, "name": self.name, "ready": False,
                    "error": self.error}
        return {
            "present": True, "ready": True, "name": self.name,
            "error": self.error,
            "yaw": round((d["yaw"] + IMU_YAW_OFFSET) % 360.0, 1),
            # Mount offsets removed so these describe the BODY, not the board.
            "roll": round(d["roll"] - IMU_ROLL_OFFSET, 2),
            "pitch": round(d["pitch"] - IMU_PITCH_OFFSET, 2),
            "raw_roll": d["roll"], "raw_pitch": d["pitch"],
            "gyro": d["gyro"], "accel": d["accel"], "mag": d.get("mag"),
            "temp": d.get("temp"),
            "heading_ok": d.get("heading_ok", False),
            "fused": d.get("fused", False),
            "calib": d.get("calib"),
        }


# ---------------------------------------------------------------------------
# Camera — CSI, streamed straight off the hardware JPEG encoder
# ---------------------------------------------------------------------------

class CameraReader:
    """Opens the CSI camera, or explains on the page why it could not.

    Same contract as ImuReader: a missing camera must never stop the cockpit
    starting. Losing video is an inconvenience; losing the drive controls and
    the collision guard because a ribbon came loose is not.

    There is no polling thread here, unlike the IMU. Picamera2 already runs
    its own capture thread and pushes finished JPEGs at us, so a loop on this
    side would only be busy-waiting on work that is already done.
    """

    CAPTURE_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                               "captures")

    def __init__(self):
        self.cam = None
        self.error = ""
        self.name = ""
        self.last_still = ""
        self.auto_mm = 0.0            # 0 = off; else mm between auto stills
        self._last_auto = None
        self._lock = threading.Lock()
        self.captures = self._read_index()
        try:
            from camera import Camera
            self.cam = Camera(size=CAM_SIZE, fps=CAM_FPS,
                              hflip=CAM_HFLIP, vflip=CAM_VFLIP)
            self.name = self.cam.name
        except Exception as e:                                # noqa: BLE001
            self.error = str(e)

    def stream(self):
        from camera import mjpeg_stream
        return mjpeg_stream(self.cam)

    def frame(self):
        return self.cam.frame() if self.cam else None

    INDEX = "index.json"

    def snap(self, pose=None, why="manual"):
        """Save the current view, and WHERE the robot was when it saw it.

        The pose is the point. A folder of photos of a house tells you very
        little; the same photos pinned to the spots on the map they were
        taken from turn the occupancy grid — which is only ever grey, white
        and black — into something a person can actually read. When the map
        comes out wrong, "what was it looking at here" becomes a question
        with an answer.
        """
        if self.cam is None:
            return None
        os.makedirs(self.CAPTURE_DIR, exist_ok=True)
        name = time.strftime("cam-%Y%m%d-%H%M%S.jpg")
        path = os.path.join(self.CAPTURE_DIR, name)
        if self.cam.still(path) is None:
            return None
        self.last_still = name
        entry = {"file": name, "t": time.time(), "why": why}
        if pose is not None:
            entry.update(x=round(pose.x), y=round(pose.y),
                         th=round(math.degrees(pose.th), 1))
        with self._lock:
            self.captures.append(entry)
            self._write_index()
        return path

    def _write_index(self):
        """The index is rewritten whole on every capture. It is a few hundred
        entries of a few fields; appending would save nothing and would leave
        a truncated file unreadable after a power cut mid-write."""
        tmp = os.path.join(self.CAPTURE_DIR, self.INDEX + ".tmp")
        try:
            with open(tmp, "w") as f:
                json.dump({"version": 1, "captures": self.captures[-500:]}, f)
            os.replace(tmp, os.path.join(self.CAPTURE_DIR, self.INDEX))
        except OSError as e:
            self.error = str(e)

    def _read_index(self):
        try:
            with open(os.path.join(self.CAPTURE_DIR, self.INDEX)) as f:
                return json.load(f).get("captures", [])
        except (OSError, ValueError):
            return []

    def auto(self, pose):
        """Take a still if the robot has moved far enough since the last one.

        Distance-gated, not time-gated. A robot parked for ten minutes should
        not fill the card with the same picture, and one crossing a room
        quickly should still leave a trail of them.
        """
        if not self.auto_mm or self.cam is None or pose is None:
            return None
        last = self._last_auto
        if last is not None and math.hypot(pose.x - last[0],
                                           pose.y - last[1]) < self.auto_mm:
            return None
        self._last_auto = (pose.x, pose.y)
        return self.snap(pose, why="auto")

    @property
    def state(self):
        if self.cam is None:
            return {"present": False, "error": self.error or "not detected",
                    "hfov": CAM_HFOV, "yaw": CAM_YAW_OFFSET}
        d = dict(self.cam.state)
        d["hfov"] = CAM_HFOV
        d["yaw"] = CAM_YAW_OFFSET
        d["last_still"] = self.last_still
        return d

    def close(self):
        if self.cam:
            self.cam.close()


class SlamRunner:
    """Runs SLAM in its own thread, paced independently of the web poll.

    Encoder SIGN is the thing to check first if the map comes out mirrored or
    the robot appears to drive backwards through its own map. Direction
    inversion (drive_invert.json) fixes which way the MOTOR turns; it says
    nothing about which way the ENCODER counts. Push the robot forward by
    hand and watch the counts in /snapshot — both must increase. If one
    decreases, flip its sign below.
    """
    # Measured on the robot 2026-09-06 by driving a verified-forward burst
    # and reading the count deltas: left +291, right -267.
    ENC_SIGN_LEFT = 1
    ENC_SIGN_RIGHT = -1

    # 5 Hz, not 10. A scan-match costs ~96 ms on a Pi 4, so a 10 Hz loop
    # has no headroom. At walking pace 5 Hz still gives an update every
    # ~100 mm, which is finer than the 40 mm mapping gate needs.
    def __init__(self, robot, lidar, imu, hz=5):
        self.robot, self.lidar, self.imu = robot, lidar, imu
        self.slam = Slam(COUNTS_PER_REV, WHEEL_DIAM_MM, TRACK_WIDTH_MM)
        self.enabled = True
        self.ms = 0.0
        self._hz = hz
        self.map_note = ""
        self._saved_scans = 0
        threading.Thread(target=self._run, daemon=True).start()
        threading.Thread(target=self._autosave, daemon=True).start()

    AUTOSAVE_S = 20.0

    def _autosave(self):
        """Save the map whenever it has grown. Places and objects are stored
        in this map's coordinates, and a restart that forgot the map made
        every one of them point at nothing — "go to the kitchen" needs the
        map that "kitchen" was saved in."""
        while True:
            time.sleep(self.AUTOSAVE_S)
            try:
                self.save_if_changed()
            except OSError as e:
                self.map_note = f"autosave failed: {e}"

    def save_if_changed(self):
        s = self.slam.scans
        if s and s != self._saved_scans:
            self.slam.save(MAP_FILE)
            self._saved_scans = s
            self.map_note = f"map autosaved · {s} scans · {time.strftime('%H:%M:%S')}"

    def resume(self):
        """At startup: carry on in the map from last time, if there is one."""
        if not os.path.exists(MAP_FILE):
            self.map_note = "new map"
            return
        try:
            blob = self.slam.load(MAP_FILE)
        except (OSError, ValueError) as e:
            self.map_note = f"saved map not loaded: {e}"
            return
        self._saved_scans = self.slam.scans
        self.map_note = (f"resumed saved map ({blob.get('scans', 0)} scans). Start the truck "
                         "where it was switched off, or press Reset map.")

    def _run(self):
        period = 1.0 / self._hz
        while True:
            time.sleep(period)
            if not self.enabled:
                continue
            try:
                pts = self.lidar.scan()
                st = self.imu.state
                # Use the IMU's heading whenever it is reading at all —
                # NOT only when the magnetometer has a fix.
                #
                # SLAM references heading to yaw0 at startup, so it only ever
                # consumes the CHANGE in heading. The magnetometer fixes
                # absolute north, which nothing here uses. Without a fix the
                # BNO055 still fuses gyro and gravity, which drifts a degree
                # or two per minute and is completely immune to wheel slip.
                #
                # The alternative, falling back to (dr-dl)/track, is the worst
                # source available on a skid-steer: it slips on every turn by
                # design. Measured effect of getting this wrong: walls smeared
                # into arcs instead of straight lines.
                yaw = st.get("yaw") if st.get("ready") else None
                t0 = time.perf_counter()
                self.slam.update(self.ENC_SIGN_LEFT * self.robot.enc_left.steps,
                                 self.ENC_SIGN_RIGHT * self.robot.enc_right.steps,
                                 yaw, pts)
                self.ms = (time.perf_counter() - t0) * 1000.0
                # Photo trail. Here rather than in the control loop because
                # this is where a freshly corrected pose exists — a still
                # tagged with a pose from before the scan match is pinned to
                # the wrong place on the map, which is the whole value gone.
                if camera is not None:
                    camera.auto(self.slam.pose)
            except Exception as e:                            # noqa: BLE001
                print(f"  slam: {e}")

    def apply_fix(self, x, y, gain):
        """Absolute position fix from a marker. Goes through here rather than
        straight at Slam so there is one writer to the pose per thread and the
        cockpit has one place to count fixes."""
        return self.slam.apply_fix(x, y, gain)

    @property
    def state(self):
        d = dict(self.slam.state)
        d["enabled"] = self.enabled
        d["ms"] = round(self.ms, 1)
        d["counts"] = [self.robot.enc_left.steps, self.robot.enc_right.steps]
        d["map_note"] = self.map_note
        return d


HALF_L = TRUCK_LENGTH_MM / 2.0
HALF_W = TRUCK_WIDTH_MM / 2.0
CORNER_R = math.hypot(HALF_L, HALF_W)        # circumscribing radius


def set_cam_rotation(deg):
    """Rotate the DISPLAYED picture by a quarter turn. Browser-side only.

    Deliberately not done in the camera. Rotating there would mean tearing
    the pipeline down and reconfiguring it on every change - a second of
    dropped video, and a real chance of leaving the camera shut if the
    reconfigure fails - to achieve something CSS does for free and instantly.
    And 90/270 are not expressible as an ISP transform anyway: libcamera has
    flips, not a transpose.

    *** The consequence, which the page states rather than hiding: ***
    the floor check reads the RAW frame. It assumes the bottom row is the
    nearest floor, and rotating the display does not change what it reads. So
    on a camera that is physically not upright, the floor check is measuring
    the wrong direction and should be turned off - or the mount corrected
    with CAM_HFLIP/CAM_VFLIP in pins.py, which the ISP does apply, at the
    cost of a restart.

    Marker detection is genuinely unaffected: a square is a square at any
    rotation and ArUco does not care.

    OBJECT detection is not, and an earlier version of this comment wrongly
    said it was. YOLO has never seen an upside-down living room and finds
    almost nothing in one - silently, while the picture on screen looks
    perfectly normal. detect.py therefore turns the frame to match this
    setting before inference and maps the boxes back afterwards.
    """
    global CAM_ROTATION
    CAM_ROTATION = int(round(int(deg) / 90.0)) * 90 % 360
    return CAM_ROTATION


def set_truck(length=None, width=None):
    """Resize the vehicle footprint, live.

    The footprint is not cosmetic. It decides three separate things, and they
    have to move together or they start disagreeing about where the robot
    ends:

      1. What the plot draws, so you can size the box against the returns the
         scanner gets off the robot's own chassis.
      2. Which returns are DISCARDED as being the chassis - here for the
         guard, and in slam.SELF_L/SELF_W for the map. Too big and real
         obstacles vanish; too small and the robot paints a permanent blob
         around itself and believes it is boxed in wherever it stands.
      3. The region swept_obstacle() checks, including CORNER_R - a skid-steer
         turns about its centre and its widest point is a corner, not the nose.

    Derived values are recomputed rather than left to go stale: HALF_L, HALF_W
    and CORNER_R are read inside the 50 Hz guard loop, so they are cached
    rather than computed per call.
    """
    global TRUCK_LENGTH_MM, TRUCK_WIDTH_MM, TRUCK_LEN_MM
    global HALF_L, HALF_W, CORNER_R
    if length is not None:
        TRUCK_LENGTH_MM = float(length)
    if width is not None:
        TRUCK_WIDTH_MM = float(width)
    TRUCK_LEN_MM = TRUCK_LENGTH_MM
    HALF_L = TRUCK_LENGTH_MM / 2.0
    HALF_W = TRUCK_WIDTH_MM / 2.0
    CORNER_R = math.hypot(HALF_L, HALF_W)

    # The mapper keeps its own copy of the half-extents, and the explorer was
    # handed the geometry as a dict at construction. Both are updated here so
    # there is exactly one place that knows the footprint changed.
    import slam as _slam
    _slam.SELF_L, _slam.SELF_W = HALF_L, HALF_W
    if explorer is not None and getattr(explorer, "geom", None):
        explorer.geom["len"] = TRUCK_LENGTH_MM
        explorer.geom["wid"] = TRUCK_WIDTH_MM
    # The body-frame cache keys on the scan, not the geometry, so a stale
    # entry would keep the old footprint filter for one more scan.
    _bp_cache["key"] = None
    return TRUCK_LENGTH_MM, TRUCK_WIDTH_MM


_bp_cache = {"key": None, "val": []}


def body_points(points):
    """Scan returns in the BODY frame, corner-mounted scanner accounted for."""
    out = []
    cal = lidar_cal
    yaw = cal.yaw if cal else LIDAR_YAW_OFFSET
    ox = cal.x if cal else LIDAR_OFFSET_X
    oy = cal.y if cal else LIDAR_OFFSET_Y

    # The same scan gets transformed by the control loop at 50 Hz, the survey
    # at 3 Hz and the explorer - roughly 250 points of trigonometry each time,
    # for an answer that only changes when a new scan arrives (~11 Hz).
    key = (id(points), len(points), yaw, ox, oy)
    if _bp_cache["key"] == key:
        return _bp_cache["val"]
    ry = math.radians(yaw)
    c0, s0 = math.cos(ry), math.sin(ry)
    for a, d in points:
        if d < 120:                      # inside the chassis, spurious
            continue
        r = math.radians(a)
        x, y = d * math.cos(r), -d * math.sin(r)
        if yaw:
            x, y = x * c0 - y * s0, x * s0 + y * c0
        x, y = x + ox, y + oy
        # The scanner sits on a corner and sees the robot's own chassis.
        # Those returns are not obstacles, and leaving them in pins the guard
        # permanently - measured 40 of 251 returns inside the footprint.
        if abs(x) <= HALF_L and abs(y) <= HALF_W:
            continue
        out.append((x, y))
    return out


# Sideways clearance while driving STRAIGHT. Braking distance is along the
# direction of travel; a wall beside the truck gets no closer as it drives
# past, so the full SAFETY_MARGIN_MM sideways only jammed it against walls.
SIDE_MARGIN_MM = 15.0
# How far ahead a turn on the spot is checked, in degrees of rotation.
# NOT a few degrees: a scan arrives every ~90 ms and describes where the truck
# WAS, and the truck coasts after the motors stop. At a normal turn rate that
# is 15-30 degrees between "the scan shows it" and "the truck has stopped".
# 6 degrees here let a corner swing into a wall 40 mm away.
TURN_LOOK_DEG = 30.0
# Clearance kept between a swinging corner and anything it passes.
TURN_PAD_MM = 25.0
# The same while driving an arc. Two values, by who is driving:
#   a person  10 mm - steering AWAY from a wall 40 mm off swings the tail to
#             ~15 mm of it, and refusing that is the jam people complained of.
#   autonomous 20 mm - at 10 the simulated explorer slid past a sofa corner
#             at 12 mm; at 20 its closest pass in the whole house was 60 mm.
# Refusing an arc keeps the straight part, so it costs a correction, not a stop.
ARC_PAD_MM = 10.0
ARC_PAD_AUTO_MM = 20.0


def _arc_hit(bpts, throttle, steer, dist_mm, turn_deg, pad):
    """Drive the footprint along the path this command really takes and
    report the first place a point would go INTO it (or deeper into it).

    Uses the same left/right mix as Robot.drive (left = t + s, right = t - s,
    scaled together), so an arc is simulated as the pivot it actually is.
    Stops after dist_mm of travel or turn_deg of rotation, whichever first.
    Returns (travelled_mm, turned_deg) at the hit, or None if clear.

    "Deeper, not inside": something already within the pad does not count
    unless this move pushes it further in - otherwise a wall 5 mm away would
    refuse the very move that leaves it.
    """
    left, right = throttle + steer, throttle - steer
    peak = max(1.0, abs(left), abs(right))
    left, right = left / peak, right / peak
    v = (left + right) / 2.0                      # forward, per unit time
    w = (right - left) / (2.0 * HALF_W)           # rad per unit time, + = left
    if abs(v) < 1e-6 and abs(w) < 1e-9:
        return None
    hl, hw = HALF_L + pad, HALF_W + pad
    base = [min(hl - abs(x), hw - abs(y)) for x, y in bpts]
    # Step so no corner moves more than ~10 mm per step.
    dt = 10.0 / (abs(v) + abs(w) * math.hypot(hl, hw))
    x = y = th = 0.0
    travelled = 0.0
    while True:
        x += v * math.cos(th + w * dt / 2) * dt
        y += v * math.sin(th + w * dt / 2) * dt
        th += w * dt
        travelled += abs(v) * dt
        turned = abs(math.degrees(th))
        if (abs(v) > 1e-6 and travelled > dist_mm) or (abs(w) > 1e-9 and turned > turn_deg):
            return None
        c, s_ = math.cos(-th), math.sin(-th)
        for (px, py), d0 in zip(bpts, base):
            qx, qy = px - x, py - y
            rx, ry = qx * c - qy * s_, qx * s_ + qy * c
            d = min(hl - abs(rx), hw - abs(ry))
            if d > 0 and d > d0 + 1.0:
                return travelled, turned


_sweep_cache = {"key": None, "val": {}}


def swept_obstacle(bpts, throttle, steer, stop_mm, arc_pad=None):
    """What this command would hit, from the actual footprint on the actual
    path it drives.

      straight   a lane the width of the truck plus SIDE_MARGIN_MM, out to
                 stop_mm + SAFETY_MARGIN_MM beyond the leading edge. Returns
                 the gap to the nearest return in it.
      spin       the footprint rotated the requested way, up to
                 TURN_LOOK_DEG, with TURN_PAD_MM around it.
      arc        forward/back AND turning: the footprint moved along the arc
                 the wheel mix really produces (it pivots about the slower
                 side), until stop_mm travelled or TURN_LOOK_DEG turned.
    Spin and arc return 0 when blocked. Everything returns (None, "") when
    clear.

    Cached per scan: the guard asks 50 times a second, a scan changes 11.
    """
    key = (id(bpts), len(bpts), HALF_L, HALF_W, stop_mm, SAFETY_MARGIN_MM)
    if _sweep_cache["key"] != key:
        _sweep_cache["key"], _sweep_cache["val"] = key, {}
    arc_pad = ARC_PAD_MM if arc_pad is None else arc_pad
    ck = (round(throttle, 1), round(steer, 1), arc_pad)
    if ck not in _sweep_cache["val"]:
        _sweep_cache["val"][ck] = _swept(bpts, throttle, steer, stop_mm, arc_pad)
    return _sweep_cache["val"][ck]


def _swept(bpts, throttle, steer, stop_mm, arc_pad=ARC_PAD_MM):
    moving, turning = abs(throttle) > 0.01, abs(steer) > 0.01
    if moving and not turning:
        m = SAFETY_MARGIN_MM
        ahead = throttle > 0
        near = HALF_L if ahead else -HALF_L
        far = near + (stop_mm + m) * (1 if ahead else -1)
        lo, hi = (near, far) if ahead else (far, near)
        band = HALF_W + SIDE_MARGIN_MM
        worst = None
        for x, y in bpts:
            if lo <= x <= hi and abs(y) <= band:
                d = (x - HALF_L) if ahead else (-HALF_L - x)
                if worst is None or d < worst:
                    worst = d
        if worst is not None:
            return max(0.0, worst), ("ahead" if ahead else "behind")
    elif turning and not moving:
        hit = _arc_hit(bpts, 0.0, steer, 0.0, TURN_LOOK_DEG, TURN_PAD_MM)
        if hit:
            return 0.0, "turning %s would hit in %.0f°" % (
                "right" if steer > 0 else "left", hit[1])
    elif turning:
        hit = _arc_hit(bpts, throttle, steer, stop_mm, TURN_LOOK_DEG, arc_pad)
        if hit:
            return 0.0, "%s %s arc would hit" % (
                "forward" if throttle > 0 else "reverse", "right" if steer > 0 else "left")
    return None, ""


class Guard:
    """Footprint-aware collision guard.

    Blocks whichever direction is obstructed, and ONLY that one — so there is
    always a way out. Reverse stays available when the front is blocked, and
    turning is judged against the turning circle rather than the nose.

    With the guard on and no fresh scan, motion is refused rather than
    allowed. A guard that silently stops guarding when its sensor dies is
    worse than no guard; the page says exactly why.
    """

    def __init__(self):
        self.enabled = True
        self.stop_mm = GUARD_STOP_MM
        self.blocked = False
        self.reason = ""
        self.creeping = False
        self.clear = {}
        self.out = (0.0, 0.0)

    def survey(self, lidar):
        """Clearance in each direction, for display."""
        if not lidar.fresh():
            self.clear = {}
            return
        b = body_points(lidar.scan())
        out = {}
        for name, th, st in (("fwd", 1, 0), ("rev", -1, 0), ("left", 0, -1), ("right", 0, 1)):
            d, _ = swept_obstacle(b, th, st, self.stop_mm)
            out[name] = None if d is None else round(d)
        # "turn" for the page: blocked only if BOTH directions are.
        out["turn"] = None if out["left"] is None or out["right"] is None else 0
        self.clear = out

    def apply(self, throttle, steer, lidar, auto=False):
        """The command the motors may have. Also kept as self.out, so the
        explorer can tell "refused outright" from "turn dropped, still
        moving" - the reason text alone cannot."""
        self.out = self._apply(throttle, steer, lidar, auto)
        return self.out

    def _apply(self, throttle, steer, lidar, auto=False):
        self.blocked, self.reason = False, ""
        if not self.enabled or (throttle == 0 and steer == 0):
            return throttle, steer
        if not lidar.fresh():
            self.blocked, self.reason = True, "no fresh LiDAR scan"
            return 0.0, 0.0

        # (The camera floor check that used to veto forward here is gone. With
        # an unmeasured camera tilt it read walls and ceiling as "drop-offs"
        # and stalled the truck at random; the LiDAR alone decides now.)

        b = body_points(lidar.scan())

        def blocked(t, s_):
            d, where = swept_obstacle(b, t, s_, self.stop_mm,
                                      ARC_PAD_AUTO_MM if auto else ARC_PAD_MM)
            if d is None:
                return False, None, ""
            # A straight lane reports a distance to compare with the stop
            # distance; a spin or an arc reports 0 only when it would hit.
            return (d < self.stop_mm), d, where

        moving, turning = abs(throttle) > 0.01, abs(steer) > 0.01
        bad, d_t, where = blocked(throttle, steer)
        if not bad:
            self.creeping = False
            return throttle, steer
        self.blocked = True
        self.reason = f"{d_t:.0f} mm {where}" if moving and not turning else where

        # The whole command, as the path it really drives, is blocked. Keep
        # whichever PART is still safe on its own: straight on if only the
        # turn would hit, turn if only the travel would. (A forward-and-left
        # used to be checked as forward only; and when forward was blocked the
        # turn was kept without being checked at all.)
        if moving and turning:
            if not blocked(throttle, 0)[0]:
                return throttle, 0.0
            if not blocked(0, steer)[0]:
                return 0.0, steer
            return 0.0, 0.0

        # Escape hatch, straight moves only. If this move is blocked AND every
        # other move is too, creep in the requested direction while there is
        # still CREEP_MARGIN_MM of room. Without this the robot wedges itself
        # against furniture and has to be lifted out by hand - which is
        # exactly what happened on the first three house runs.
        if moving and not any(not blocked(t_, s2)[0]
                              for t_, s2 in ((1, 0), (-1, 0), (0, 1), (0, -1))) \
                and d_t is not None and d_t > CREEP_MARGIN_MM:
            self.creeping = True
            self.reason += " - creeping out"
            return throttle * CREEP_THROTTLE, 0.0
        self.creeping = False
        return 0.0, 0.0


class Intent:
    """What the operator (or the explorer) last asked for.

    Motor commands used to be applied inside the HTTP handler, which meant
    every keypress waited on a guard evaluation - body_points over ~250
    returns plus three swept_obstacle sweeps - AND on the GIL, which the SLAM
    thread can hold for hundreds of milliseconds at a time. That is the lag:
    it is worst exactly when SLAM is busiest.

    Now the handler only records intent and returns, and a dedicated thread
    applies it at a steady 50 Hz. Command latency stops depending on how busy
    the mapper is.
    """

    def __init__(self):
        self.throttle = 0.0
        self.steer = 0.0
        self.ts = 0.0
        self.source = "idle"
        self.lock = threading.Lock()

    def set(self, throttle, steer, source="manual"):
        with self.lock:
            self.throttle = max(-1.0, min(1.0, float(throttle or 0)))
            self.steer = max(-1.0, min(1.0, float(steer or 0)))
            self.ts = time.monotonic()
            self.source = source

    def get(self):
        with self.lock:
            return self.throttle, self.steer, self.ts, self.source


def control_loop():
    """Apply the latest intent at a fixed rate, with the guard in this thread."""
    period = 1.0 / 50.0
    last_applied = None
    while True:
        time.sleep(period)
        if robot is None:
            continue
        th, st, ts, source = intent.get()
        if time.monotonic() - ts > WATCHDOG_S:
            th = st = 0.0                       # deadman: intent went stale
        try:
            th, st = guard.apply(th, st, lidar, auto=(source in ("explore", "follow")))
        except Exception:                                     # noqa: BLE001
            th = st = 0.0
        # Only touch the GPIO when something actually changed. Rewriting the
        # same PWM value 50 times a second is pure contention.
        cur = (round(th, 3), round(st, 3))
        if cur != last_applied:
            robot.drive(th, st)
            last_applied = cur
        elif th or st:
            robot._last_cmd = time.monotonic()  # keep Robot's own watchdog fed


app = Flask(__name__)
app.config["MAX_CONTENT_LENGTH"] = audio.MAX_UPLOAD_MB * 1024 * 1024
robot = lidar = imu = slam = explorer = camera = None
follower = None
speaker = None
screen = None
talker = None
ears = None
assistant = None
http_port = HTTP_PORT
markers = cliff = detector = None
intent = Intent()
guard = Guard()
from calibrate import PushMeasure  # noqa: E402
pusher = PushMeasure()
import sysstats  # noqa: E402


class SysMonitor:
    """The System tab: who is using the CPU, on the Pi and on the PC.

    Pi: whole-machine and per-core CPU, RAM, temperature, under-voltage, and
    this program's threads by name (SLAM, LiDAR, person tracker...).
    PC: the detect and speech containers report their own numbers on GET
    (tools/*_server.py, same sysstats module); Ollama says which models it
    has loaded and how much of each is on the GPU (/api/ps).

    Polls only while someone has the tab open (asked in the last WATCH_S):
    the PC round trips are cheap but not free, and nobody reads them otherwise.
    """

    PERIOD_S = 2.0
    WATCH_S = 15.0

    def __init__(self):
        self.sampler = sysstats.Sampler()
        self.asked = 0.0
        self.data = {}
        self._frames = {}                  # (url, key) -> (t, frames), for frames/s
        self._throttle_t = 0.0
        self._throttle = None
        threading.Thread(target=self._loop, daemon=True, name="sysmon").start()

    def snapshot(self):
        self.asked = time.monotonic()
        return self.data

    def _loop(self):
        while True:
            time.sleep(self.PERIOD_S)
            if time.monotonic() - self.asked > self.WATCH_S:
                continue
            try:
                self.data = self._collect()
            except Exception as e:                            # noqa: BLE001
                self.data = {"error": "%s: %s" % (type(e).__name__, e)}

    @staticmethod
    def _get(url, timeout=1.5):
        import urllib.request                                 # noqa: PLC0415
        t0 = time.monotonic()
        with urllib.request.urlopen(url, timeout=timeout) as r:
            d = json.loads(r.read().decode())
        return d, round((time.monotonic() - t0) * 1000.0)

    def _rate(self, key, frames):
        now = time.monotonic()
        prev = self._frames.get(key)
        self._frames[key] = (now, frames)
        if prev is None or frames is None or frames < prev[1]:
            return None
        return round((frames - prev[1]) / max(1e-3, now - prev[0]), 1)

    def _pi_throttled(self):
        """vcgencmd get_throttled, every 10 s: under-voltage on a battery
        robot shows up as random slowness long before anything resets."""
        now = time.monotonic()
        if now - self._throttle_t > 10.0:
            self._throttle_t = now
            try:
                import subprocess                             # noqa: PLC0415
                out = subprocess.run(["vcgencmd", "get_throttled"], capture_output=True,
                                     text=True, timeout=2).stdout
                v = int(out.strip().split("=")[1], 16)
                self._throttle = {"raw": hex(v), "undervolt_now": bool(v & 0x1),
                                  "throttled_now": bool(v & 0x4), "undervolt_since_boot": bool(v & 0x10000),
                                  "throttled_since_boot": bool(v & 0x40000)}
            except Exception:                                 # noqa: BLE001
                self._throttle = None
        return self._throttle

    def _collect(self):
        pi = self.sampler.sample()
        pi["throttle"] = self._pi_throttled()
        pc = {}
        durl = getattr(detector, "url", None) if detector is not None else None
        if durl:
            base = durl.rsplit("/detect", 1)[0] if "/detect" in durl else durl.rstrip("/")
            try:
                d, ping = self._get(base + "/")
                pc["detect"] = {"ok": True, "url": base, "ping_ms": ping,
                                "model": d.get("model"), "device": d.get("device"),
                                "last_ms": d.get("last_ms"), "fps": self._rate((base, "f"), d.get("frames")),
                                "person_model": d.get("person_model"),
                                "person_last_ms": d.get("person_last_ms"),
                                "person_fps": self._rate((base, "p"), d.get("person_frames")),
                                "stats": d.get("stats")}
            except Exception as e:                            # noqa: BLE001
                pc["detect"] = {"ok": False, "url": base, "error": str(e)[:80]}
        turl = getattr(talker, "url", None) if talker is not None else None
        if turl:
            try:
                d, ping = self._get(turl.rstrip("/") + "/")
                pc["tts"] = {"ok": True, "url": turl, "ping_ms": ping, "voices": d.get("loaded"),
                             "stats": d.get("stats")}
            except Exception as e:                            # noqa: BLE001
                pc["tts"] = {"ok": False, "url": turl, "error": str(e)[:80]}
        ourl = getattr(assistant, "ollama_url", None) if assistant is not None else None
        if ourl:
            try:
                d, ping = self._get(ourl.rstrip("/") + "/api/ps")
                pc["ollama"] = {"ok": True, "url": ourl, "ping_ms": ping, "models": [
                    {"name": m.get("name"), "size_mb": round((m.get("size") or 0) / 2**20),
                     "vram_mb": round((m.get("size_vram") or 0) / 2**20),
                     "expires": m.get("expires_at")} for m in d.get("models") or []]}
            except Exception as e:                            # noqa: BLE001
                pc["ollama"] = {"ok": False, "url": ourl, "error": str(e)[:80]}
        return {"t": time.time(), "pi": pi, "pc": pc}


sysmon = SysMonitor()


PAGE = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Speaker Truck — Nav</title>
<style>
/* ---------------------------------------------------------------------------
   Layout system
   ---------------------------------------------------------------------------
   Three levels, and nothing else, so a new panel has an obvious home:

     .topbar    sticky. State you must never have to scroll to find, and the
                e-stop. Always on screen, on every tab.
     .tabs      one per job you might be doing. Only one panel is in the
                document flow at a time.
     .work      inside a tab: one big viewport plus a rail of readouts.
                Collapses to a single column on a phone.

   The old page was ten cards in a vertical stack, which meant scrolling to
   find anything and never seeing two related numbers together.
--------------------------------------------------------------------------- */
:root{
  color-scheme: dark;
  --bg:#0d1117; --surface-1:#161b22; --surface-2:#1c2430; --border:#2a323d;
  --text-1:#fff; --text-2:#a9b4c0; --text-3:#6e7b8a; --grid:#242d3a;
  --series-1:#3987e5; --series-2:#d95926;
  --point:#3987e5; --obj:#199e70; --imu:#9085e9; --cam:#2bb3c0;
  --good:#3fb950; --warning:#d29922; --critical:#f85149;
  --sky:#1d3a57; --ground:#3d2d1c;
  --audio:#d670c0;
  --rail: 330px;
}
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:ui-sans-serif,-apple-system,"Segoe UI",Roboto,sans-serif;
     background:var(--bg);color:var(--text-1);-webkit-font-smoothing:antialiased;
     padding:0 0 40px}
.wrap{max-width:1680px;margin:0 auto;padding:0 18px}
.mono{font-family:ui-monospace,"SF Mono",Consolas,monospace}
.num{font-variant-numeric:tabular-nums}

/* --- sticky top bar ------------------------------------------------------ */
.topbar{position:sticky;top:0;z-index:40;background:color-mix(in srgb,var(--bg) 92%,transparent);
        backdrop-filter:blur(9px);border-bottom:1px solid var(--border)}
.topbar .wrap{display:flex;align-items:center;gap:10px;flex-wrap:wrap;padding-top:10px;
              padding-bottom:10px}
.brand{font-size:.78rem;font-weight:700;letter-spacing:.14em;text-transform:uppercase;
       white-space:nowrap}
.brand span{color:var(--text-3);font-weight:400}
.spacer{flex:1}
.livepose{font-size:.68rem;color:var(--text-2);white-space:nowrap;
          font-family:ui-monospace,monospace;font-variant-numeric:tabular-nums}
.livepose b{color:var(--text-1);font-weight:600}
.speedbox{display:flex;align-items:center;gap:8px;min-width:190px}
.speedbox label{font-size:.62rem;color:var(--text-3);letter-spacing:.07em;
                text-transform:uppercase;font-weight:650;white-space:nowrap}
.speedbox input[type=range]{flex:1;min-width:80px}
.badge{display:inline-flex;align-items:center;gap:6px;padding:5px 10px;border-radius:99px;
       font-size:.66rem;font-weight:700;letter-spacing:.05em;border:1px solid;white-space:nowrap}
.badge .dot{width:7px;height:7px;border-radius:50%;background:currentColor}
.badge.on{color:var(--good);border-color:color-mix(in srgb,var(--good) 45%,transparent);
          background:color-mix(in srgb,var(--good) 12%,transparent)}
.badge.off{color:var(--critical);border-color:color-mix(in srgb,var(--critical) 45%,transparent);
           background:color-mix(in srgb,var(--critical) 12%,transparent)}
.badge.warn{color:var(--warning);border-color:color-mix(in srgb,var(--warning) 45%,transparent);
            background:color-mix(in srgb,var(--warning) 12%,transparent)}
.btns{display:flex;gap:7px;flex-wrap:wrap}

/* --- tab strip ----------------------------------------------------------- */
.tabs{position:sticky;top:var(--tabtop,52px);z-index:39;background:var(--bg);
      border-bottom:1px solid var(--border)}
.tabs .wrap{display:flex;gap:2px;overflow-x:auto;scrollbar-width:none}
.tabs .wrap::-webkit-scrollbar{display:none}
.tab{padding:11px 17px;border:none;border-bottom:2px solid transparent;background:none;
     color:var(--text-3);font-size:.75rem;font-weight:650;letter-spacing:.05em;
     cursor:pointer;white-space:nowrap;border-radius:0;transition:color .12s}
.tab:hover{color:var(--text-2);filter:none}
.tab.on{color:var(--text-1);border-bottom-color:var(--series-1)}
.tab .pip{display:inline-block;width:6px;height:6px;border-radius:50%;margin-left:7px;
          background:var(--critical);vertical-align:middle;opacity:0}
.tab .pip.show{opacity:1}

/* --- tab panels and the work grid ---------------------------------------- */
.tabpanel{display:none;padding-top:16px}
.tabpanel.on{display:block}
.work{display:grid;grid-template-columns:minmax(0,1fr) var(--rail);gap:14px;
      align-items:start}
.work.even{grid-template-columns:repeat(auto-fit,minmax(320px,1fr))}
.work.narrow{--rail:290px}
@media(max-width:1000px){.work,.work.even{grid-template-columns:1fr}}
.stack{display:flex;flex-direction:column;gap:14px}
.cols2{display:grid;grid-template-columns:repeat(auto-fit,minmax(260px,1fr));gap:14px;
       align-items:start}

/* --- cards --------------------------------------------------------------- */
.card{background:var(--surface-1);border:1px solid var(--border);border-radius:12px;padding:15px}
.card.pad0{padding:11px}
.card h2{font-size:.64rem;text-transform:uppercase;letter-spacing:.1em;
         color:var(--text-3);font-weight:650;margin-bottom:10px;
         display:flex;align-items:baseline;gap:7px;flex-wrap:wrap}
.card h2 .mono{text-transform:none;letter-spacing:0;color:var(--text-2);font-weight:600}
.card h2 .hint{margin-left:auto;font-size:.6rem;color:var(--text-3);font-weight:400;
               text-transform:none;letter-spacing:0}

/* --- buttons ------------------------------------------------------------- */
button{padding:8px 14px;border:1px solid var(--border);border-radius:7px;font-size:.74rem;
       font-weight:650;letter-spacing:.03em;cursor:pointer;background:var(--surface-2);
       color:var(--text-1);transition:filter .12s,transform .06s}
button:hover{filter:brightness(1.25)}
button:active{transform:translateY(1px)}
.b-enable{background:color-mix(in srgb,var(--good) 20%,var(--surface-2));
          border-color:color-mix(in srgb,var(--good) 40%,transparent);color:var(--good)}
/* The top ENABLE button shows the motors' state itself: grey and plain while
   they are off, solid green once armed - it used to look the same either way. */
#b-arm{background:var(--surface-2);border-color:var(--border);color:var(--text-2)}
#b-arm.armed{background:var(--good);border-color:var(--good);color:#0d1117;font-weight:700}
.b-stop{background:color-mix(in srgb,var(--warning) 18%,var(--surface-2));
        border-color:color-mix(in srgb,var(--warning) 40%,transparent);color:var(--warning)}
.b-estop{background:var(--critical);border-color:var(--critical);color:#fff}
.tog{display:flex;gap:6px;flex-wrap:wrap;margin-bottom:9px}
.tog button{flex:1;min-width:84px;font-size:.7rem;padding:8px 6px}
.tog button.on{background:color-mix(in srgb,var(--warning) 22%,var(--surface-2));
               border-color:color-mix(in srgb,var(--warning) 55%,transparent);color:var(--warning)}
.tog button.g-on{background:color-mix(in srgb,var(--good) 20%,var(--surface-2));
                 border-color:color-mix(in srgb,var(--good) 45%,transparent);color:var(--good)}

/* --- viewports ----------------------------------------------------------- */
/* The page itself does not scroll.
   On a desktop the whole tab is sized to the viewport: the topbar and tab
   strip are fixed-height, the work area takes the rest, and each COLUMN
   scrolls on its own if it has to. Before this the plot alone was taller than
   the screen, so the controls that change it sat below the fold - you had to
   scroll away from the picture to adjust it, which is exactly backwards.
   Below 1000px this unwinds to an ordinary scrolling page, because on a phone
   a locked viewport just makes everything cramped. */
@media(min-width:1001px){
  html,body{height:100%}
  body{display:flex;flex-direction:column;overflow:hidden}
  .topbar,.tabs{flex:none}
  /* width:100% is load-bearing. .wrap carries `margin:0 auto`, and an auto
     CROSS-AXIS margin on a flex item cancels the default stretch - so this
     shrink-wrapped to its content and the whole cockpit rendered 629px wide
     in a 2304px window. Stating the width restores it; max-width and the
     auto margins still centre it. */
  .content{width:100%;flex:1;min-height:0;display:flex;flex-direction:column;
           overflow:hidden;padding-bottom:10px}
  .tabpanel.on{flex:1;min-height:0;display:flex;flex-direction:column}
  /* stretch, not the `align-items:start` the grid carries by default: the
     plot card has to be as tall as the row for its .view to have any height
     to inherit. With start, .view measured 0 and the canvas never drew. */
  .tabpanel.on > .work{flex:1;min-height:0;align-items:stretch}
  .work > .card,.work > .stack{max-height:100%;overflow-y:auto;
                               scrollbar-width:thin}
  .work > .card.pad0{height:100%;display:flex;flex-direction:column;
                     overflow:hidden}
  #tab-tune{overflow-y:auto}
}
.view{position:relative;flex:1;min-height:0}
/* Without the viewport lock there is no flex height to inherit, so the box
   states its own shape and the canvas fills it. */
@media(max-width:1000px){.view{aspect-ratio:1;width:100%}}
/* Fit the viewport, not the column.
   A square canvas at width:100% in a 1150 px column is 1150 px TALL, so the
   plot ran off the bottom of the screen and its own controls sat below the
   fold - you had to scroll past the picture to reach the sliders that change
   it, which is the whole problem this layout exists to solve. Sizing by the
   smaller of the column and the free viewport height keeps it square AND on
   screen. */
/* NEVER size these with width:auto or height:auto.
   A canvas with an auto dimension takes it from the width/height ATTRIBUTE,
   and draw() sets that attribute from clientWidth every frame. The two feed
   each other: the canvas grows by devicePixelRatio 25 times a second, and the
   page locks up within seconds. It looks exactly like the robot being slow.
   Both dimensions must therefore be stated, or derived from aspect-ratio.

   Non-square is fine - draw() and drawMap() both work from Math.min(w,h), so
   the drawing simply centres in whichever dimension is larger. */
/* The canvas is positioned ABSOLUTELY inside .view, and that is the whole
   trick. A canvas whose width or height resolves to `auto` takes that
   dimension from its width/height ATTRIBUTE - which draw() rewrites from
   clientWidth every frame. The two feed each other and the canvas grows by
   devicePixelRatio 25 times a second until the browser dies.

   `height:100%` is not safe either: .view is a flex item, so its height is
   indefinite, and a percentage against an indefinite height falls back to
   auto - the same loop by a longer road. That is exactly how this bug came
   back after the first fix.

   Absolute positioning takes both dimensions from the containing block,
   which is always definite once laid out. Nothing reads the attribute, so
   there is no path back to the loop. */
canvas#plot,canvas#map{position:absolute;left:0;top:0;
                       width:100%;height:100%;display:block}
canvas#plot{cursor:grab;touch-action:none}
canvas#plot.drag{cursor:grabbing}
canvas#map{image-rendering:pixelated;background:var(--surface-2);border-radius:8px}
.viewbar{display:flex;gap:7px;flex-wrap:wrap;align-items:center;margin-bottom:9px}
.chips{display:flex;gap:5px;flex-wrap:wrap}
.chip{padding:5px 10px;border:1px solid var(--border);border-radius:99px;
      background:var(--surface-2);color:var(--text-2);cursor:pointer;font-size:.7rem;font-weight:600}
.chip.sel{background:color-mix(in srgb,var(--point) 22%,var(--surface-2));
          border-color:color-mix(in srgb,var(--point) 55%,transparent);color:var(--point)}
.zoomnote{font-size:.62rem;color:var(--text-3);margin-left:auto}

/* --- readouts ------------------------------------------------------------ */
.stat{display:flex;justify-content:space-between;align-items:baseline;padding:5px 0;
      border-bottom:1px solid var(--border);font-size:.73rem;gap:10px}
.stat:last-child{border-bottom:none}
.stat .k{color:var(--text-3)}
.sbar{position:relative;height:7px;border-radius:4px;background:var(--surface-2);overflow:hidden;margin:2px 0 6px}
.sbar i{position:absolute;left:0;top:0;bottom:0;border-radius:4px;background:var(--good)}
.sbar i.mid{background:var(--warning)} .sbar i.hi{background:var(--critical)}
.cores{display:grid;grid-template-columns:repeat(auto-fill,minmax(60px,1fr));gap:6px;margin:4px 0 8px}
.cores div{font-size:.62rem;color:var(--text-3)}
.sysnote{font-size:.7rem;color:var(--text-3);margin:12px 2px;line-height:1.5}
.v.bad{color:var(--critical)} .v.ok{color:var(--good)}
.stat .v{font-weight:600;text-align:right}
.rows{display:flex;gap:12px;justify-content:center;margin-top:9px}
.rd{text-align:center;flex:1}
.rd .hd{display:flex;align-items:center;gap:5px;justify-content:center;font-size:.6rem;
        font-weight:700;letter-spacing:.09em;color:var(--text-2)}
.sw{width:9px;height:9px;border-radius:3px}
.rd .v{font-size:1.25rem;font-weight:250;line-height:1.3}
.rd .s{font-size:.6rem;color:var(--text-3)}
.ctl-hd{display:flex;justify-content:space-between;align-items:center;margin-bottom:5px}
.ctl-hd label{font-size:.68rem;color:var(--text-2);font-weight:600}
input[type=range]{width:100%;accent-color:var(--series-1);height:22px}
input[type=number],input[type=text]{width:100%;background:var(--surface-2);
  border:1px solid var(--border);color:var(--text-1);padding:6px 9px;border-radius:6px;
  font-size:.78rem}
label.lbl{display:block;font-size:.61rem;color:var(--text-3);margin-bottom:4px;
          letter-spacing:.07em;text-transform:uppercase;font-weight:650}
/* Explanations are off by default and revealed by the ? in the top bar.
   They are genuinely useful the first week and pure height forever after,
   and height is the scarce thing on this page. */
.note{display:none;font-size:.66rem;color:var(--text-3);line-height:1.55;
      margin-top:9px}
body.docs .note{display:block}
.note b{color:var(--text-2)}
/* Amber the moment a tuned value would be lost by a restart, grey when the
   card matches the running values. It lives in the sticky bar because that
   is the only place visible from whichever tab you were tuning on - having
   the only Save button on the Tune tab meant walking away from your evidence
   to press it. */
.b-save{padding:5px 11px;font-size:.68rem;font-weight:700;letter-spacing:.05em;
        color:var(--text-3);border-color:var(--border)}
.b-save.dirty{background:color-mix(in srgb,var(--warning) 22%,var(--surface-2));
              border-color:color-mix(in srgb,var(--warning) 60%,transparent);
              color:var(--warning)}
.b-docs{padding:5px 10px;font-size:.72rem;font-weight:700}
body.docs .b-docs{background:color-mix(in srgb,var(--series-1) 30%,var(--surface-2));
                  border-color:var(--series-1);color:#fff}

/* --- drive pad ----------------------------------------------------------- */
.pad{display:grid;grid-template-columns:repeat(3,54px);grid-template-rows:repeat(3,54px);
     gap:7px;justify-content:center;margin:2px auto 11px;touch-action:none}
.pad button{display:flex;align-items:center;justify-content:center;font-size:1.1rem;
            padding:0;border-radius:10px;user-select:none}
.pad button.on{background:color-mix(in srgb,var(--series-1) 35%,var(--surface-2));
               border-color:var(--series-1);color:#fff}
.pad .mid{font-size:.56rem}
.hint{font-size:.66rem;color:var(--text-3);line-height:1.6;text-align:center}
.hint kbd{background:var(--surface-2);border:1px solid var(--border);border-radius:4px;
          padding:1px 5px;font-family:ui-monospace,monospace;font-size:.64rem;color:var(--text-2)}

/* --- banners ------------------------------------------------------------- */
.banner{display:none;margin-bottom:10px;padding:9px 12px;border-radius:8px;
        font-size:.74rem;font-weight:600}
.banner.show{display:block}
.banner.warn{color:var(--warning);background:color-mix(in srgb,var(--warning) 12%,transparent);
             border:1px solid color-mix(in srgb,var(--warning) 40%,transparent)}
.banner.info{color:var(--text-2);background:var(--surface-2);border:1px solid var(--border)}

/* --- camera -------------------------------------------------------------- */
.camwrap{position:relative;background:var(--surface-2);border-radius:8px;
         overflow:hidden;aspect-ratio:4/3;display:flex;align-items:center;
         justify-content:center}
.camrot{position:absolute;inset:0;transform-origin:50% 50%;
        transition:transform .18s ease}
.camwrap img{width:100%;height:100%;object-fit:contain;display:block}
/* A quarter turn inside a 4:3 box overflows unless it is scaled down by the
   aspect ratio - 3/4 here. Without the scale the long edge is cropped and
   the floor grid lines up with pixels that are no longer on screen. */
.camrot.r90 {transform:rotate(90deg)  scale(.75)}
.camrot.r180{transform:rotate(180deg)}
.camrot.r270{transform:rotate(270deg) scale(.75)}
.camoff{color:var(--text-3);font-size:.73rem;text-align:center;padding:20px;
        line-height:1.7;max-width:38ch}
.camdot{position:absolute;top:9px;left:9px;display:flex;align-items:center;gap:6px;
        padding:4px 9px;border-radius:99px;font-size:.58rem;font-weight:700;
        letter-spacing:.08em;background:rgba(13,17,23,.72);color:var(--cam)}
.camdot i{width:6px;height:6px;border-radius:50%;background:currentColor}
/* Boxes are positioned as a percentage of the frame, so they follow the
   picture at any size and through the rotation transform without any
   arithmetic on this side. */
.detboxes{position:absolute;inset:0;pointer-events:none}
.detbox{position:absolute;border:2px solid var(--obj);border-radius:3px;
        box-shadow:0 0 0 1px rgba(13,17,23,.55)}
.detbox b{position:absolute;left:-2px;top:-17px;white-space:nowrap;
          background:var(--obj);color:#07231a;font-size:.6rem;font-weight:700;
          letter-spacing:.03em;padding:1px 5px;border-radius:3px 3px 0 0;
          font-family:ui-monospace,monospace}
/* A box with no LiDAR range behind it cannot be placed on the map. Drawn in
   the warning colour rather than hidden, because "seen but not placed" is a
   real state and hiding it makes the map look mysteriously incomplete. */
.detbox.noplace{border-color:var(--warning);border-style:dashed}
.detbox.noplace b{background:var(--warning);color:#231a07}
/* People, for follow mode: their own colour so they never read as furniture,
   and the one being followed drawn heavier than the rest. */
.detbox.person{border-color:var(--audio);border-style:dashed}
.detbox.person b{background:var(--audio);color:#2a0b24}
.detbox.person.target{border-style:solid;border-width:3px}
.cliffgrid{position:absolute;inset:auto 0 0 0;height:55%;display:grid;
           pointer-events:none}
.cliffgrid div{border:1px solid rgba(255,255,255,.045)}
.cc1{background:color-mix(in srgb,var(--warning) 34%,transparent)}
.cc2{background:color-mix(in srgb,var(--critical) 46%,transparent)}

/* --- markers, photos ----------------------------------------------------- */
.taglist{display:flex;flex-wrap:wrap;gap:5px;margin:8px 0}
.tag{padding:3px 8px;border-radius:99px;font-size:.65rem;font-weight:700;
     font-family:ui-monospace,monospace;border:1px solid var(--border);
     background:var(--surface-2);color:var(--text-3)}
.tag.on{color:var(--cam);border-color:color-mix(in srgb,var(--cam) 55%,transparent);
        background:color-mix(in srgb,var(--cam) 15%,transparent)}
.tag.fix{color:var(--good);border-color:color-mix(in srgb,var(--good) 55%,transparent);
         background:color-mix(in srgb,var(--good) 15%,transparent)}
.shots{display:grid;grid-template-columns:repeat(auto-fill,minmax(84px,1fr));
       gap:7px;margin-top:10px;max-height:300px;overflow-y:auto}
.shots a{display:block;border-radius:6px;overflow:hidden;border:1px solid var(--border);
         line-height:0}
.shots img{width:100%;aspect-ratio:4/3;object-fit:cover}
.shots .lbl{font-size:.55rem;color:var(--text-3);padding:3px 4px;line-height:1.3;
            font-family:ui-monospace,monospace}

/* --- mounting calibration, places, auto-map ------------------------------ */
.cal{display:flex;gap:12px;flex-wrap:wrap;align-items:flex-end;margin:8px 0}
.cal .f{display:flex;flex-direction:column;gap:4px}
.cal label{font-size:.61rem;color:var(--text-3);letter-spacing:.07em;
           text-transform:uppercase;font-weight:650}
.cal input[type=number]{width:88px}
.cal input[type=range]{width:190px}
.calnow{font-size:.78rem;font-weight:600;font-variant-numeric:tabular-nums;
        color:var(--point);min-width:56px}
.places{display:flex;gap:7px;flex-wrap:wrap;margin:9px 0}
/* The label question: the frame that made the candidate, its box on top.
   The box is in the camera's own frame, so both rotate together. */
/* The map's click menu: floats over the canvas where you clicked. */
.mapmenu{position:absolute;z-index:5;display:flex;flex-direction:column;gap:6px;
  min-width:190px;padding:10px;background:var(--surface-1);border:1px solid var(--border);
  border-radius:10px;box-shadow:0 6px 24px rgba(0,0,0,.45)}
.mapmenu button{font-size:.74rem;padding:7px 9px;text-align:left}
.mapmenu .mm-t{font-size:.72rem;color:var(--text-2);font-weight:600}
.mapmenu .mm-row{display:flex;gap:6px}
.mapmenu .mm-row input{flex:1;min-width:0}
.linkish{background:none;border:none;padding:0;color:inherit;font:inherit;cursor:pointer}
.linkish:hover{text-decoration:underline}
.place{display:inline-flex;align-items:center;gap:6px;padding:5px 9px;
  border:1px solid var(--border);border-radius:99px;background:var(--surface-2);
  font-size:.72rem;color:var(--text-1)}
.place button{padding:2px 8px;font-size:.66rem;border-radius:5px}
.place .x{background:none;border:none;color:var(--text-3);padding:2px 4px}
.place .x:hover{color:var(--critical)}
.row2{display:flex;gap:8px;flex-wrap:wrap;align-items:center;margin-top:8px}
.row2 input{width:150px}
.auto{display:flex;align-items:center;gap:12px;flex-wrap:wrap;margin-bottom:10px}
.b-auto{background:color-mix(in srgb,var(--obj) 24%,var(--surface-2));
        border-color:color-mix(in srgb,var(--obj) 55%,transparent);color:var(--obj);
        font-size:.78rem;padding:10px 18px}
.b-auto.on{background:var(--obj);color:#0d1117;border-color:var(--obj)}
.calbox{background:var(--surface-2);border:1px solid var(--border);border-radius:8px;
        padding:10px 12px;font-size:.71rem;color:var(--text-2);margin-top:10px}
.calbox b{color:var(--text-1)}
.calnote{color:var(--warning);margin-top:5px}

/* --- IMU dials ----------------------------------------------------------- */
.imu-grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(180px,1fr));gap:14px;
          align-items:start}
.dial{display:flex;flex-direction:column;align-items:center;gap:6px}
.dial canvas{width:100%;max-width:200px;aspect-ratio:1}
.dial .cap{font-size:.6rem;letter-spacing:.09em;color:var(--text-3);font-weight:700}
.dial .big{font-size:1.45rem;font-weight:250;line-height:1}

/* --- tables -------------------------------------------------------------- */
table{width:100%;border-collapse:collapse;font-size:.72rem}
th,td{text-align:right;padding:5px 7px;border-bottom:1px solid var(--border);
      font-variant-numeric:tabular-nums}
th{color:var(--text-3);font-weight:650;font-size:.6rem;text-transform:uppercase}
td{color:var(--text-2)}
th:first-child,td:first-child{text-align:left}
td.n{color:var(--text-1);font-weight:600}
.idx{display:inline-flex;align-items:center;justify-content:center;width:18px;height:18px;
     border-radius:5px;background:var(--surface-2);border:1px solid var(--obj);
     color:var(--obj);font-size:.6rem;font-weight:700}

/* --- tuning tab ---------------------------------------------------------- */
.tunegrid{display:grid;grid-template-columns:repeat(auto-fit,minmax(300px,1fr));gap:14px;
          align-items:start}
.tunerow{padding:9px 0;border-bottom:1px solid var(--border)}
.tunerow:last-child{border-bottom:none}
.tunehd{display:flex;align-items:baseline;gap:8px;margin-bottom:5px}
.tunehd .lb{font-size:.72rem;font-weight:600;color:var(--text-1)}
.tunehd .vv{margin-left:auto;font-family:ui-monospace,monospace;font-size:.74rem;
            font-weight:650;color:var(--point);font-variant-numeric:tabular-nums}
.tunehd .vv.dirty{color:var(--warning)}
.tunerow input[type=range]{height:18px}
.tunerow .doc{font-size:.63rem;color:var(--text-3);line-height:1.5;margin-top:4px;
              display:none}
.tunerow.open .doc{display:block}
.tunehd .q{background:none;border:none;color:var(--text-3);font-size:.68rem;padding:0 4px;
           cursor:pointer;font-weight:700}
.tunehd .q:hover{color:var(--text-1);filter:none}
.tunesub{font-size:.6rem;text-transform:uppercase;letter-spacing:.1em;
         color:var(--text-3);font-weight:650;margin:12px 0 2px;
         padding-top:9px;border-top:1px solid var(--border)}
.tunerow:first-child + .tunesub,.tunesub:first-child{margin-top:0;padding-top:0;border-top:none}
.tuneact{display:flex;gap:8px;flex-wrap:wrap;align-items:center;margin-bottom:12px}
.tuneact .msg{font-size:.68rem;color:var(--text-3)}
.swrow{display:flex;gap:8px;align-items:center}
.swrow button{flex:1}

/* --- audio --------------------------------------------------------------- */
.b-audio{background:color-mix(in srgb,var(--audio) 22%,var(--surface-2));
         border-color:color-mix(in srgb,var(--audio) 50%,transparent);color:var(--audio)}
.beeps{display:grid;grid-template-columns:repeat(3,1fr);gap:6px}
.beeps button{padding:10px 4px;font-size:.72rem}
.beeps .horn{grid-column:span 3;padding:13px;font-size:.84rem;letter-spacing:.1em}
.nowplay{display:flex;align-items:center;gap:8px;font-size:.74rem;color:var(--text-2);
         min-width:0;margin:10px 0 4px}
.nowplay b{color:var(--text-1);font-weight:600;overflow:hidden;text-overflow:ellipsis;
           white-space:nowrap;min-width:0}
.eqbars{display:inline-flex;align-items:flex-end;gap:2px;height:12px;flex:none}
.eqbars i{width:3px;height:30%;background:var(--text-3);border-radius:1px}
.eqbars.on i{background:var(--audio);animation:eqb .9s ease-in-out infinite}
.eqbars.on i:nth-child(2){animation-delay:-.3s}
.eqbars.on i:nth-child(3){animation-delay:-.6s}
@keyframes eqb{0%,100%{height:25%}50%{height:100%}}
.prog{position:relative;height:8px;border-radius:99px;background:var(--surface-2);
      border:1px solid var(--border);cursor:pointer;margin:8px 0 4px;overflow:hidden}
.prog i{position:absolute;left:0;top:0;bottom:0;width:0;background:var(--audio)}
.times{display:flex;justify-content:space-between;font-size:.62rem;color:var(--text-3)}
.transport{display:flex;gap:6px;margin:10px 0}
.transport button{flex:1;font-size:.8rem;padding:8px 0}
.drop{display:block;border:1.5px dashed var(--border);border-radius:9px;padding:18px 10px;
      text-align:center;font-size:.76rem;color:var(--text-2);cursor:pointer;
      transition:border-color .12s,color .12s}
.drop:hover,.drop.over{border-color:var(--audio);color:var(--audio)}
.drop input{display:none}
.drop small{display:block;color:var(--text-3);font-size:.62rem;margin-top:4px}
.upbar{height:4px;border-radius:99px;background:var(--surface-2);overflow:hidden;
       margin-top:8px;display:none}
.upbar.show{display:block}
.upbar i{display:block;height:100%;width:0;background:var(--audio)}
.songs{display:flex;flex-direction:column;margin-top:10px}
.song{display:flex;align-items:center;gap:8px;padding:6px 2px;
      border-bottom:1px solid var(--border);font-size:.73rem}
.song:last-child{border-bottom:none}
.song .nm{flex:1;min-width:0;overflow:hidden;text-overflow:ellipsis;white-space:nowrap}
.song .sz{color:var(--text-3);font-size:.64rem;white-space:nowrap}
.song button{padding:4px 9px;font-size:.68rem}
.song .x{background:none;border:none;color:var(--text-3);min-width:24px}
.song .x:hover,.song .x.arm{color:var(--critical);filter:none}
.song.cur .nm{color:var(--audio);font-weight:650}
.songs .empty{color:var(--text-3);font-size:.72rem;padding:10px 0}
.arng input[type=range]{accent-color:var(--audio)}
.arng .ctl-hd{margin-top:8px}
.arng .ctl-hd span{font-size:.7rem;color:var(--text-2)}
select{width:100%;background:var(--surface-2);border:1px solid var(--border);
       color:var(--text-1);padding:6px 9px;border-radius:6px;font-size:.76rem}
.aerr{color:var(--warning);font-size:.7rem;margin-top:8px;line-height:1.5}
.aerr:empty{display:none}

/* --- speech ------------------------------------------------------------- */
textarea{width:100%;background:var(--surface-2);border:1px solid var(--border);
         color:var(--text-1);padding:8px 10px;border-radius:6px;font:inherit;
         font-size:.82rem;line-height:1.45;resize:vertical;min-height:74px}
#t-text:focus,#d-say:focus{outline:1px solid var(--audio);outline-offset:0}
.tcount{margin-left:auto;font-size:.62rem;color:var(--text-3)}
.chip.hist{max-width:100%;overflow:hidden;text-overflow:ellipsis;white-space:nowrap}
details.voices{margin-top:12px}
details.voices summary{cursor:pointer;font-size:.62rem;text-transform:uppercase;
                       letter-spacing:.1em;color:var(--text-3);font-weight:650;padding:4px 0}
.vrow{display:flex;align-items:center;gap:8px;padding:6px 0;border-bottom:1px solid var(--border);
      font-size:.68rem}
.vrow:last-child{border-bottom:none}
.vrow .vn{flex:1;min-width:0}
.vrow .vn b{display:block;font-family:ui-monospace,monospace;font-size:.67rem;color:var(--text-1);
            overflow:hidden;text-overflow:ellipsis;white-space:nowrap;font-weight:600}
.vrow .vn span{color:var(--text-3)}
.vrow button{padding:4px 9px;font-size:.66rem;white-space:nowrap}
.vrow .ok{color:var(--good);font-weight:650;white-space:nowrap}
.vrow .bad{color:var(--warning)}
.vrow button.arm{color:var(--critical)}
.tsay{margin-top:8px;flex-wrap:nowrap}
.tsay input{flex:1;width:auto !important;min-width:0}

/* --- LCD mirror ---------------------------------------------------------- */
.lcdwrap{display:flex;justify-content:center;margin-bottom:10px}
.lcdwrap img{width:100%;max-width:240px;aspect-ratio:1;display:block;border-radius:8px;
             background:#000;border:6px solid #05070a;box-sizing:content-box}
</style>
</head>
<body>

<!-- Sticky. Nothing here scrolls away, because the e-stop and the sensor
     badges are exactly what you need when something is going wrong. -->
<div class="topbar">
  <div class="wrap">
    <span class="brand">Speaker Truck <span>/ nav</span></span>
    <span id="badge" class="badge off"><i class="dot"></i>DISABLED</span>
    <span id="lbadge" class="badge off"><i class="dot"></i>NO LIDAR</span>
    <span id="ibadge" class="badge off"><i class="dot"></i>NO IMU</span>
    <span id="cbadge" class="badge off"><i class="dot"></i>NO CAM</span>
    <span id="fbadge" class="badge off" style="display:none"><i class="dot"></i>FLOOR</span>
    <span id="mbadge" class="badge off"><i class="dot"></i>TAGS</span>
    <div class="spacer"></div>
    <span class="livepose" id="toppose">&mdash;</span>
    <div class="speedbox">
      <label for="lim">Speed</label>
      <input type="range" id="lim" min="5" max="40" value="40" step="1">
      <span class="num mono" id="limv" style="font-size:.72rem">0.40</span>
    </div>
    <div class="btns">
      <button class="b-save" id="b-save" onclick="tuneSave()"
              title="Write tuned values to tuning.json on the Pi">SAVED</button>
      <button class="b-docs" id="b-docs" onclick="toggleDocs()" title="Show or hide the explanations">?</button>
      <button id="b-arm" class="b-enable" onclick="cmd('/enable')" title="Arm the motors">ENABLE</button>
      <button class="b-stop" onclick="cmd('/stop')">STOP</button>
      <button class="b-estop" onclick="cmd('/estop')">E-STOP</button>
    </div>
  </div>
</div>

<div class="tabs">
  <div class="wrap">
    <button class="tab on" data-tab="drive" onclick="showTab('drive')">Drive</button>
    <button class="tab" data-tab="map" onclick="showTab('map')">Map</button>
    <button class="tab" data-tab="sensors" onclick="showTab('sensors')">Sensors</button>
    <button class="tab" data-tab="vision" onclick="showTab('vision')">Vision<i class="pip" id="pip-vision"></i></button>
    <button class="tab" data-tab="audio" onclick="showTab('audio')">Audio<i class="pip" id="pip-audio"></i></button>
    <button class="tab" data-tab="tune" onclick="showTab('tune')">Tune</button>
    <button class="tab" data-tab="system" onclick="showTab('system')">System</button>
  </div>
</div>

<div class="wrap content">

<div id="trip" class="banner warn">&#9888; Watchdog tripped &mdash; commands stopped arriving, so the motors were stopped.</div>
<div id="blocked" class="banner warn"></div>
<div id="tilt" class="banner warn"></div>
<div id="cliffbanner" class="banner warn"></div>
<div id="jserr" class="banner warn" style="font-family:ui-monospace,monospace;font-size:.7rem;white-space:pre-wrap"></div>
<div id="focusnote" class="banner info show">Click anywhere on the page once, then the arrow keys will drive.</div>

<!-- ===================================================== DRIVE ========== -->
<div class="tabpanel on" id="tab-drive">
  <div class="work">

    <div class="card pad0">
      <div class="viewbar">
        <div class="chips" id="ranges"></div>
        <span class="zoomnote">scroll to zoom &middot; drag to pan &middot; double-click to reset</span>
      </div>
      <div class="view"><canvas id="plot"></canvas></div>
      <p class="note">
        Everything is drawn in the <b>body frame</b>: origin at the truck's
        geometric centre, +x forward, +y left &mdash; the same frame the
        collision guard and SLAM use, so what you see is what they judge.
        Sensors are drawn <b>where they physically are</b>, with the arrow
        showing which way each faces. Zoom in and they label themselves.
        The two shaded boxes and the dashed circle are exactly what the
        collision guard tests &mdash; the forward and reverse swept
        rectangles, and the circle the corners sweep when turning on the
        spot. Each turns amber on its own clearance, so you can put an object
        in front of the robot and watch that box light up. If the box covers
        the obstacle and nothing goes amber, the geometry is wrong, not the
        sensor. The cyan wedge is what the camera can see.
      </p>
    </div>

    <div class="stack">
      <div class="card">
        <h2>Camera <span class="hint">moves to Vision when that tab is open</span></h2>
        <!-- The camera lives in ONE node that gets moved between these two
             slots on a tab change. Two .camwrap elements would mean two
             <img> tags pointing at /camera.mjpg, which is two MJPEG
             connections off one Pi for one picture. -->
        <div id="camslot-drive">
          <div class="camwrap" id="camwrap">
            <!-- The picture and the floor grid live inside one rotating box.
                 Rotating them separately would drift them apart the moment
                 the mount is anything but upright, and the grid is only
                 useful because it sits over the pixels it judged. The LIVE
                 pip and the off message stay outside, upright. -->
            <div class="camrot" id="camrot">
              <div class="cliffgrid" id="cliffgrid"></div>
              <div class="detboxes" id="detboxes"></div>
            </div>
            <div class="camoff" id="camoff">Camera off.</div>
            <div class="camdot" id="camdot" style="display:none"><i></i>LIVE</div>
          </div>
        </div>
      </div>

      <div class="card">
        <h2>Drive</h2>
        <div class="pad">
          <span></span><button id="p-f">&uarr;</button><span></span>
          <button id="p-l">&larr;</button><button class="mid" id="p-s">STOP</button><button id="p-r">&rarr;</button>
          <span></span><button id="p-b">&darr;</button><span></span>
        </div>
        <p class="hint"><kbd>&larr;</kbd><kbd>&uarr;</kbd><kbd>&darr;</kbd><kbd>&rarr;</kbd> or <kbd>WASD</kbd> to drive &middot; <kbd>space</kbd> e-stop</p>
        <div class="rows">
          <div class="rd"><div class="hd"><i class="sw" style="background:var(--series-1)"></i>LEFT</div>
            <div class="v num mono" id="lr">0.0</div><div class="s">RPM</div></div>
          <div class="rd"><div class="hd"><i class="sw" style="background:var(--series-2)"></i>RIGHT</div>
            <div class="v num mono" id="rr">0.0</div><div class="s">RPM</div></div>
        </div>
      </div>

      <!-- The horn sits under the drive pad because that is where your hand
           already is when something walks in front of the robot. -->
      <div class="card">
        <h2>Speaker <span class="hint"><kbd>H</kbd> horn &middot; songs on Audio</span></h2>
        <div class="beeps">
          <button class="b-audio horn" onclick="beep('horn')">HORN</button>
          <button onclick="beep('beep')">Beep</button>
          <button onclick="beep('double')">Double</button>
          <button onclick="beep('chirp')">Chirp</button>
          <button onclick="beep('reverse')">Reverse</button>
          <button onclick="beep('alert')">Alert</button>
        </div>
        <div class="nowplay"><span class="eqbars" id="d-eq"><i></i><i></i><i></i></span><b id="d-now">Idle</b></div>
        <div class="transport">
          <button onclick="apost('/audio/skip',{back:true})" title="Previous song">&#9664;&#9664;</button>
          <button id="d-pp" class="b-audio" onclick="apost('/audio/pause')" title="Play / pause">&#9654;</button>
          <button onclick="apost('/audio/skip',{})" title="Next song">&#9654;&#9654;</button>
          <button onclick="apost('/audio/stop')" title="Stop">&#9632;</button>
        </div>
        <div class="row2 tsay">
          <input type="text" id="d-say" maxlength="1000" placeholder="Say something&hellip;">
          <button class="b-audio" onclick="speakFrom('d-say')">Say</button>
        </div>
        <div class="aerr" id="d-err"></div>
      </div>

      <!-- Mounting sits next to the plot because that is the only place you
           can see whether it is right. The measurement below beats the
           slider: it gives a number with a residual instead of an opinion. -->
      <div class="card">
        <h2>Vehicle <span class="hint">size it against the plot</span></h2>
        <div id="tune-vehicle"></div>
        <p class="note">
          Park the robot against a wall and size this box until it matches the
          returns the scanner gets off the robot's own body. Everything inside
          the box is thrown away as the robot seeing itself &mdash; too big and
          real obstacles vanish, too small and it paints a permanent blob
          around itself and refuses to move. This is also what the guard
          measures clearances from, and its diagonal is what the turning check
          uses, because a skid-steer's widest point is a corner, not the nose.
        </p>
      </div>

      <div class="card">
        <h2>Scanner mounting <span class="hint">watch the plot</span></h2>
        <div id="tune-mount"></div>
        <div class="cal" style="margin:10px 0 0">
          <span class="calnow" id="lyawv">0&deg;</span>
          <button onclick="nudgeYaw(-1)">&minus;1&deg;</button>
          <button onclick="nudgeYaw(1)">+1&deg;</button>
        </div>
        <p class="note">
          Face a flat wall: it should sit <b>square across the top</b> of the
          plot. If the wall is drawn skewed, rotation is wrong &mdash; and the
          guard is checking a different direction than you think.
        </p>
        <div class="tog" style="margin-top:10px">
          <button id="cal-start" onclick="pushStart()">Measure by pushing</button>
          <button id="cal-finish" onclick="pushFinish()" style="display:none">Finish</button>
        </div>
        <div id="calresult" class="calbox" style="display:none"></div>
        <p class="note">
          Push the robot ~0.5&nbsp;m in a straight line and this fits the range
          change across <b>every</b> bearing at once. The phase is the true
          nose bearing; the amplitude is how far it really went, which also
          gives counts-per-rev. Both come with a residual, so you can tell
          whether to believe them.
        </p>
      </div>

      <!-- The guard's numbers and the guard's knobs, together. Changing a
           margin without watching the clearances it produces is guessing. -->
      <div class="card">
        <h2>Guard</h2>
        <div class="tog"><button id="t-guard" onclick="toggleGuard()">Guard</button></div>
        <div class="stat"><span class="k">Ahead</span><span class="v num mono" id="ahead">&mdash;</span></div>
        <div class="stat"><span class="k">Clear forward</span><span class="v num mono" id="cl-fwd">&mdash;</span></div>
        <div class="stat"><span class="k">Clear reverse</span><span class="v num mono" id="cl-rev">&mdash;</span></div>
        <div class="stat"><span class="k">Clear to turn</span><span class="v num mono" id="cl-turn">&mdash;</span></div>
        <div id="tune-guard" style="margin-top:10px"></div>
        <p class="note">
          <b>Stop distance</b> sets how far the boxes on the plot extend past
          the body; <b>Safety margin</b> sets how far they stand out to the
          sides. Walk something into the forward box and it should go amber
          and <b>Clear forward</b> should fall below the stop distance. If it
          does not, the fault is the scanner mounting above, not the guard.
          The dashed circle is the turning check &mdash; it uses half the
          margin, because momentum carries a corner around the same circle
          rather than past it.
        </p>
      </div>
    </div>
  </div>
</div>

<!-- ===================================================== MAP ============ -->
<div class="tabpanel" id="tab-map">
  <div class="work">

    <div class="card pad0">
      <div class="viewbar">
        <button class="chip" onclick="mapFit()">Fit</button>
        <button class="chip" id="m-follow" onclick="mapFollowToggle()">Follow truck</button>
        <button class="chip" id="m-person" onclick="togglePersonTrail()" title="Show or hide the followed person's trail">Person trail</button>
        <button class="chip" onclick="mapZoomBy(1.4)">+</button>
        <button class="chip" onclick="mapZoomBy(1/1.4)">&minus;</button>
        <span class="note" id="m-goal" style="margin:0"></span>
        <button class="chip" id="m-cancel" style="display:none" onclick="post('/goto',{stop:true})">Cancel trip</button>
      </div>
      <div class="view">
        <canvas id="map"></canvas>
        <div id="m-menu" class="mapmenu" style="display:none"></div>
      </div>
      <p class="note">
        <b>Click the map</b> to send the truck there or name the room.
        Scroll or pinch to zoom, drag to pan, double-click to fit. White is
        wall, dark is free, grey is unknown; violet is the path driven, the
        dashed line the planned route, green dots are objects, cyan dots are
        stills (click to open). The map saves itself every 20&nbsp;s and
        comes back after a restart.
      </p>
    </div>

    <div class="stack">
      <div class="card">
        <h2>Pose &amp; SLAM</h2>
        <div class="stat"><span class="k">Pose x</span><span class="v num mono" id="px">&mdash;</span></div>
        <div class="stat"><span class="k">Pose y</span><span class="v num mono" id="py">&mdash;</span></div>
        <div class="stat"><span class="k">Heading</span><span class="v num mono" id="pth">&mdash;</span></div>
        <div class="stat"><span class="k">Distance driven</span><span class="v num mono" id="pdist">&mdash;</span></div>
        <div class="stat"><span class="k">Scans mapped</span><span class="v num mono" id="pscans">&mdash;</span></div>
        <div class="stat"><span class="k">Match correction</span><span class="v num mono" id="pcorr">&mdash;</span></div>
        <div class="stat"><span class="k">SLAM cpu</span><span class="v num mono" id="pms">&mdash;</span></div>
        <div class="stat"><span class="k">Match confidence</span><span class="v num mono" id="pconf">&mdash;</span></div>
        <div class="stat"><span class="k">Scans refused</span><span class="v num mono" id="prej">&mdash;</span></div>
        <div class="stat"><span class="k">Loop closures</span><span class="v num mono" id="ploop">&mdash;</span></div>
        <div class="stat"><span class="k">Encoder counts</span><span class="v num mono" id="pcnt">&mdash;</span></div>
        <div class="tog" style="margin-top:10px">
          <button id="t-slam" onclick="post('/slam',{enabled:!slamOn})">SLAM</button>
          <button id="t-match" onclick="post('/slam',{matching:!matchOn})">Match</button>
        </div>
        <div class="tog">
          <button onclick="post('/slam',{reset:true})">Reset map</button>
          <button onclick="post('/map/save',{})">Save map</button>
          <button onclick="post('/map/load',{})">Load map</button>
        </div>
        <p class="note" id="slamnote"></p>
      </div>

      <!-- Matching and grid knobs go here, not in a list elsewhere: SLAM cpu
           and Match correction are directly above, and those two numbers are
           how you tell whether a change helped. -->
      <div class="card">
        <h2>Mapping <span class="hint">watch SLAM cpu above</span></h2>
        <div id="tune-slam"></div>
        <p class="note">
          Past about <b>150 ms</b> of SLAM cpu the 5&nbsp;Hz loop has no
          headroom and pose quality gets worse even though you were trying to
          improve it. Raise <b>Decimate</b> first &mdash; it divides the cost
          linearly, while the search window grows it as the square.
          <br><br>
          <b>Match confidence</b> above is how pinned the pose is: near 0.6 in
          a room, near 0.1 sliding along a corridor. Below <b>Min
          confidence</b> the scan is refused rather than mapped &mdash; a gap
          gets filled on the next pass, a smear never leaves.
          <b>Loop closures</b> is how often the robot recognised somewhere it
          had already been and corrected against it; without that, error only
          ever grows with distance driven.
        </p>
      </div>

      <!-- Odometry sits with Distance driven, which is the only way to judge
           it, and with the push measurement that produces the number. -->
      <div class="card">
        <h2>Odometry <span class="hint">map scale lives here</span></h2>
        <div id="tune-odom"></div>
        <p class="note">
          If the map comes out uniformly too big or too small it is
          <b>counts per rev</b> and nothing else. Drive a measured metre and
          compare <b>Distance driven</b> against a tape &mdash; or use
          <b>Measure by pushing</b> on the Drive tab, which derives it from
          the LiDAR without a tape at all.
          Push the robot forward by hand and both encoder counts must
          <b>increase</b>; if one falls, flip its sign here.
        </p>
      </div>

      <div class="card">
        <h2>Auto-map</h2>
        <div class="auto">
          <button id="b-auto" class="b-auto" onclick="toggleAuto()">START AUTO-MAP</button>
          <span id="autostate" class="mono" style="font-size:.74rem;color:var(--text-2)"></span>
        </div>
        <div id="calbox" class="calbox" style="display:none"></div>
        <p class="note">
          Explores by frontier: it drives to the boundary between mapped floor
          and unknown space. A doorway <b>is</b> such a boundary, so doors get
          found and driven through without being a special case. Touching the
          arrow keys takes over instantly.
        </p>
        <div id="tune-expl" style="margin-top:10px"></div>
      </div>

      <div class="card">
        <h2>Objects &middot; <span id="o-count" class="mono">&mdash;</span></h2>
        <div class="places" id="o-list"></div>
        <p class="note">
          Saved <b>automatically</b> once the camera has seen a thing from
          three different spots; seeing it again joins the one already there.
          Nothing to answer. If one is wrong, <b>&times;</b> removes it for
          good; click its name to rename it (e.g. <i>Dad's chair</i>).
        </p>
      </div>

      <div class="card">
        <h2>Places</h2>
        <div class="places" id="places"></div>
        <div class="row2">
          <input type="text" id="pname" placeholder="name this spot, e.g. kitchen">
          <button onclick="savePlace()">Save here</button>
        </div>
        <p class="note">
          Rooms are spots in this map. Name one here, by clicking the map, or
          by answering the truck's &ldquo;What room am I in?&rdquo;. Then
          <b>Go</b> drives there on its own. <b>Reset map</b> clears rooms
          and objects with it &mdash; they would point at walls that no
          longer exist.
        </p>
      </div>
    </div>
  </div>
</div>

<!-- ===================================================== SENSORS ======== -->
<div class="tabpanel" id="tab-sensors">
  <div class="work even">

    <div class="card">
      <h2>Orientation &middot; <span id="imuname" class="mono">&mdash;</span></h2>
      <div class="imu-grid">
        <div class="dial">
          <canvas id="compass"></canvas>
          <div class="cap">HEADING</div>
          <div class="big num mono" id="yaw">&mdash;</div>
        </div>
        <div class="dial">
          <canvas id="horizon"></canvas>
          <div class="cap">ROLL / PITCH</div>
          <div class="big num mono" id="rp">&mdash;</div>
        </div>
      </div>
      <div style="margin-top:12px">
        <div class="stat"><span class="k">Roll</span><span class="v num mono" id="s-roll">&mdash;</span></div>
        <div class="stat"><span class="k">Pitch</span><span class="v num mono" id="s-pitch">&mdash;</span></div>
        <div class="stat"><span class="k">Turn rate (gyro)</span><span class="v num mono" id="s-gz">&mdash;</span></div>
        <div class="stat"><span class="k">Turn rate (wheels)</span><span class="v num mono" id="s-enc">&mdash;</span></div>
        <div class="stat"><span class="k">Slip</span><span class="v num mono" id="s-slip">&mdash;</span></div>
        <div class="stat"><span class="k">Temp</span><span class="v num mono" id="s-temp">&mdash;</span></div>
      </div>
      <p class="note" id="imunote"></p>
    </div>

    <div class="stack">
      <div class="card">
        <h2>LiDAR</h2>
        <div class="stat"><span class="k">Rate</span><span class="v num mono" id="hz">&mdash;</span></div>
        <div class="stat"><span class="k">Points / turn</span><span class="v num mono" id="count">&mdash;</span></div>
        <div class="stat"><span class="k">Dropped frames</span><span class="v num mono" id="l-bad">&mdash;</span></div>
        <div class="stat"><span class="k">Port</span><span class="v mono" id="l-port">&mdash;</span></div>
      </div>

      <div class="card">
        <h2>Drive train</h2>
        <div class="stat"><span class="k">Left RPM</span><span class="v num mono" id="s-lrpm">&mdash;</span></div>
        <div class="stat"><span class="k">Right RPM</span><span class="v num mono" id="s-rrpm">&mdash;</span></div>
        <div class="stat"><span class="k">Encoder counts</span><span class="v num mono" id="s-cnt">&mdash;</span></div>
        <div class="tog" style="margin-top:10px">
          <button id="t-left" onclick="toggle('left')">Inv L</button>
          <button id="t-right" onclick="toggle('right')">Inv R</button>
          <button id="t-swap" onclick="toggle('swap')">Swap</button>
        </div>
        <p class="note">
          Push the robot forward by hand: <b>both counts must increase</b>. If
          one decreases, flip its sign on the Tune tab. Inversion here fixes
          which way the MOTOR turns and says nothing about the encoder.
        </p>
      </div>

      <div class="card">
        <h2>Camera</h2>
        <div class="stat"><span class="k">Model</span><span class="v mono" id="camname">&mdash;</span></div>
        <div class="stat"><span class="k">Resolution</span><span class="v num mono" id="c-size">&mdash;</span></div>
        <div class="stat"><span class="k">Frame rate</span><span class="v num mono" id="c-hz">&mdash;</span></div>
        <div class="stat"><span class="k">Frames</span><span class="v num mono" id="c-frames">&mdash;</span></div>
        <div class="stat"><span class="k">Encoder</span><span class="v mono" id="c-enc">&mdash;</span></div>
        <div class="stat"><span class="k">Field of view</span><span class="v num mono" id="c-fov">&mdash;</span></div>
      </div>

      <div class="card">
        <h2>Display &middot; ST7789 240&times;240 <span class="hint" id="lcd-st">&mdash;</span></h2>
        <div class="lcdwrap"><img id="lcd-img" alt="what the truck's screen shows"></div>
        <div class="stat"><span class="k">Frames sent</span><span class="v num mono" id="lcd-frames">&mdash;</span></div>
        <div class="stat"><span class="k">Render + send</span><span class="v num mono" id="lcd-ms">&mdash;</span></div>
        <div class="stat"><span class="k">SPI clock</span><span class="v num mono" id="lcd-hz">&mdash;</span></div>
        <div class="tog" style="margin-top:10px">
          <button id="t-lcd" onclick="post('/display',{on:!lcdOn})">Screen</button>
          <button onclick="post('/display',{test:true})">Test card</button>
        </div>
        <div class="aerr" id="lcd-err"></div>
        <p class="note">
          The picture above is rendered by the Pi, so it is exactly what the
          truck's own screen shows &mdash; including when no display is wired,
          which lets you judge the layout first. A frame is only sent when
          something on it changed. <b>Test card</b>: the TOP arrow should
          point up (else <b>LCD_ROTATION</b>), bars read R&nbsp;G&nbsp;B&nbsp;W
          (red shows blue &rarr; <b>LCD_BGR</b>), black background (white
          &rarr; <b>LCD_INVERT</b>). All in pins.py. Wiring: WIRING.md &sect;14.
        </p>
      </div>
    </div>
  </div>

  <div class="card" style="margin-top:14px">
    <h2>Detected objects &middot; nearest first</h2>
    <table>
      <thead><tr><th>#</th><th>Bearing</th><th>Nearest</th><th>Width</th><th>Arc</th><th>Rays</th></tr></thead>
      <tbody id="objs"></tbody>
    </table>
  </div>
</div>

<!-- ===================================================== VISION ========= -->
<div class="tabpanel" id="tab-vision">
  <div class="work">

    <div class="card pad0">
      <div class="viewbar">
        <button id="t-cam" onclick="toggleCam()">Live view</button>
        <button onclick="snap()">Save still</button>
        <button id="t-auto-mm" onclick="toggleAutoShots()">Auto every 500 mm</button>
        <div class="chips" id="overlays" style="margin-left:auto">
          <button class="chip" data-ov="boxes">Boxes</button>
          <button class="chip" data-ov="none">Clean</button>
        </div>
        <span class="zoomnote" id="camnote"></span>
      </div>
      <div id="camslot-vision"></div>
      <p class="note">
        <b>Live view</b> stops the stream without stopping the robot &mdash;
        MJPEG is by far the largest thing on this page, and over a weak Wi-Fi
        link it will starve the drive commands before anything else gives way.
        <b>Boxes</b> draws what the detector sees, with its confidence and the
        LiDAR range that will place it on the map. A <b>dashed amber</b> box
        was recognised but has no range behind it, so it cannot be placed
        &mdash; usually too far, or the scanner is looking under it.
      </p>
    </div>

    <div class="stack">
      <div class="card">
        <h2>Camera mounting <span class="hint">watch the picture</span></h2>
        <div id="tune-cam"></div>
        <p class="note">
          Two different questions. <b>Bearing</b> is where the lens looks on
          the robot &mdash; it moves the cyan wedge on the Drive plot and
          decides which way a detected marker is reported to lie, so it has to
          be right even when the picture looks fine. <b>Rotation</b> is only
          which way up the sensor sits in its bracket.
          Rotation turns the display, not the camera: the floor check reads
          raw pixels and still sees the sensor's own orientation, which is
          why a quarter turn warns above.
        </p>
      </div>

      <div class="card" style="display:none">   <!-- floor check removed -->
        <h2>Floor check &middot; <span id="cliffstate" class="mono">&mdash;</span></h2>
        <div class="tog">
          <button id="t-cliff" onclick="post('/cliff',{enabled:!cliffOn})">Veto forward</button>
          <button onclick="post('/cliff',{relearn:true})">Relearn floor</button>
        </div>
        <div class="stat"><span class="k">Nearest anomaly</span><span class="v num mono" id="f-near">&mdash;</span></div>
        <div class="stat"><span class="k">Camera tilt</span><span class="v num mono" id="f-geom">&mdash;</span></div>
        <div class="stat"><span class="k">Check cost</span><span class="v num mono" id="f-ms">&mdash;</span></div>
        <div id="cliffwarn" class="banner warn" style="margin:8px 0 0">
          &#9888; The camera is rotated a quarter turn, so the floor check
          is reading across the picture instead of down it. Its distances
          are wrong &mdash; turn it off, or correct the mount with
          <b>CAM_HFLIP</b>/<b>CAM_VFLIP</b> in <b>pins.py</b>.
        </div>
        <div id="tune-vision" style="margin-top:8px"></div>
        <p class="note">
          The LiDAR sweeps one horizontal plane, so a stair edge returns
          nothing to it and the collision guard reads the top step as clear
          floor. This is the only thing on the robot that can see a drop.
          <b>Relearn</b> on open floor after changing surface &mdash; it takes
          what it sees as the definition of floor, so do not do it facing a
          wall.
          To tune these, switch the picture above to <b>Floor grid</b>: the
          cells recolour within a fifth of a second of moving a tolerance, and
          that is the feedback to tune against. Too twitchy on a patterned
          rug? Raise <b>chroma</b> before luma.
        </p>
      </div>

      <div class="card">
        <h2>Markers &middot; <span id="tagcount" class="mono">&mdash;</span></h2>
        <div class="tog">
          <button id="t-mk" onclick="post('/markers',{enabled:!mkOn})">Detect</button>
          <button id="t-learn" onclick="post('/markers',{learn:!mkLearn})">Learn</button>
        </div>
        <div class="taglist" id="taglist"></div>
        <div class="stat"><span class="k">Fixes applied</span><span class="v num mono" id="m-fix">&mdash;</span></div>
        <div class="stat"><span class="k">Rejected</span><span class="v num mono" id="m-rej">&mdash;</span></div>
        <div class="stat"><span class="k">Last correction</span><span class="v num mono" id="m-last">&mdash;</span></div>
        <div class="stat"><span class="k">Detect cost</span><span class="v num mono" id="m-ms">&mdash;</span></div>
        <div class="tog" style="margin-top:10px">
          <button onclick="window.open('/markers/sheet.svg?ids=0,1,2,3,4,5')">Print tags</button>
          <button onclick="post('/markers',{save:true})">Save map</button>
        </div>
        <p class="note" id="mknote">
          A printed tag at a known spot is the only input here that does not
          depend on the robot's own history. Print at <b>100%</b>, measure a
          tag, set <b>MARKER_SIZE_MM</b>. Turn <b>Learn</b> on, drive the house
          once, turn it off &mdash; from then on they hold the map straight.
        </p>
      </div>

      <div class="card">
        <h2>Objects &middot; <span id="objstate" class="mono">&mdash;</span></h2>
        <div class="tog">
          <button id="t-detect" onclick="post('/detect',{enabled:!detOn})">Detect</button>
          <button onclick="post('/detect',{save:true})">Save</button>
          <button class="b-stop" onclick="post('/detect',{forget:'all'})">Forget all</button>
        </div>
        <div class="taglist" id="detlist"></div>
        <div id="tune-detect"></div>
        <div class="stat"><span class="k">On the map</span><span class="v num mono" id="d-obj">&mdash;</span></div>
        <div class="stat"><span class="k">Frames</span><span class="v num mono" id="d-frames">&mdash;</span></div>
        <div class="stat"><span class="k">Skipped for SLAM</span><span class="v num mono" id="d-skip">&mdash;</span></div>
        <div class="stat"><span class="k">Inference</span><span class="v num mono" id="d-ms">&mdash;</span></div>
        <div class="stat"><span class="k">Running on</span><span class="v mono" id="d-backend">&mdash;</span></div>
        <label class="lbl" style="margin-top:10px">Off-board server (bigger model on your PC)</label>
        <div class="row2">
          <input type="text" id="d-url" placeholder="http://192.168.1.5:8000/detect"
                 style="flex:1;min-width:150px">
          <button onclick="setDetectUrl()">Use</button>
        </div>
        <p class="note" id="detnote">
          The camera says <b>what</b> and roughly which direction; it has no
          depth. The <b>LiDAR</b> supplies the range at that bearing, and the
          pose turns the two into a world coordinate &mdash; neither sensor
          can place an object on the map alone.
          A label only sticks after <b>three</b> agreeing sightings: one frame
          is noise, and a single misfire would otherwise plant "bed" in the
          hallway permanently.
          Walls are not detected here and never will be &mdash; the LiDAR
          already does walls better, faster and in the dark.
          Detection yields to mapping: it skips a cycle whenever SLAM is over
          budget, which is what <b>Skipped</b> counts.
          <br><br>
          <b>Off-board</b>: run <b>tools/detect_server.py</b> on your PC and
          paste its URL above. The robot then sends the JPEG the camera
          already encoded &mdash; about 30&nbsp;kB a frame, nothing next to the
          video stream &mdash; and a desktop-sized model answers. Far better
          results than the Pi can manage: yolov8n at 320&nbsp;px on-board
          against yolov8m at 640 there.
          If the PC goes to sleep the robot falls back to the on-board model
          and says so here; mapping never depends on your laptop being awake.
        </p>
      </div>

      <div class="card">
        <h2>Person &middot; <span id="p-state" class="mono">&mdash;</span></h2>
        <div class="tog">
          <button id="t-person" onclick="post('/person',{enabled:!personOn})">Track person</button>
          <button id="t-follow" class="b-auto" onclick="post('/follow', followOn ? {stop:true} : {start:true, enable:true})">Follow</button>
        </div>
        <div class="stat"><span class="k">Follow</span><span class="v mono" id="f-state">off</span></div>
        <div class="stat"><span class="k">Distance</span><span class="v num mono" id="p-dist">&mdash;</span></div>
        <div class="stat"><span class="k">Bearing (+left)</span><span class="v num mono" id="p-bear">&mdash;</span></div>
        <div class="stat"><span class="k">LiDAR</span><span class="v num mono" id="p-lidar">&mdash;</span></div>
        <div class="stat"><span class="k">Camera &middot; feet on floor</span><span class="v num mono" id="p-floor">&mdash;</span></div>
        <div class="stat"><span class="k">Camera &middot; box size</span><span class="v num mono" id="p-size">&mdash;</span></div>
        <div class="stat"><span class="k">Confidence &middot; people</span><span class="v num mono" id="p-conf">&mdash;</span></div>
        <div class="stat"><span class="k">Rate &middot; cost</span><span class="v num mono" id="p-rate">&mdash;</span></div>
        <div class="stat"><span class="k">Running on</span><span class="v mono" id="p-backend">&mdash;</span></div>
        <div class="aerr" id="p-err"></div>
        <p class="note">
          <b>Follow</b> locks on to the nearest person in view and follows
          them at about 1&nbsp;m: it knows them by their clothes' colours and
          where they are walking, follows their legs in the LiDAR when the
          camera cannot see them, goes round furniture, and goes to where they
          were last seen if it loses them. The guard still stops it short of
          anyone. Walk slowly - the truck tops out near 0.3&nbsp;m/s. Driving
          by hand, STOP, or starting a trip ends it. Or say <i>"follow me"</i>.
          <b>LiDAR</b> is the distance to steer on (legs, to the
          centimetre); the two camera figures work without it but are rough
          &mdash; <b>feet on floor</b> needs the feet in frame and a measured
          camera height and tilt, <b>box size</b> assumes 0.45&nbsp;m of
          shoulders. The target is ringed on the LiDAR plot.
          With the detection server set, the PC's <b>/person</b> model runs at
          ~5 frames a second; without it the Pi's own model manages ~1 and
          costs CPU, so leave this off when not following.
        </p>
      </div>

      <div class="card">
        <h2>Photo trail &middot; <span id="shotcount" class="mono">&mdash;</span></h2>
        <div class="shots" id="shots"></div>
        <p class="note">
          Each still is tagged with the pose it was taken from and pinned to
          the map. Stills live in <b>test/captures/</b> on the Pi.
        </p>
      </div>
    </div>
  </div>
</div>

<!-- ===================================================== AUDIO ========== -->
<div class="tabpanel" id="tab-audio">
  <div class="work even">

    <div class="stack">
      <div class="card">
        <h2>Now playing <span class="hint" id="a-card">&mdash;</span></h2>
        <div class="nowplay"><span class="eqbars" id="a-eq"><i></i><i></i><i></i></span><b id="a-now">Nothing playing</b></div>
        <div class="prog" id="a-prog" title="Click to jump"><i id="a-bar"></i></div>
        <div class="times mono num"><span id="a-pos">0:00</span><span id="a-dur">&mdash;</span></div>
        <div class="transport">
          <button onclick="apost('/audio/skip',{back:true})" title="Previous song">&#9664;&#9664;</button>
          <button id="a-pp" class="b-audio" onclick="apost('/audio/pause')" title="Play / pause">&#9654;</button>
          <button onclick="apost('/audio/skip',{})" title="Next song">&#9654;&#9654;</button>
          <button onclick="apost('/audio/stop')" title="Stop">&#9632;</button>
        </div>
        <div class="chips" id="a-modes">
          <button class="chip" data-mode="single">Stop after song</button>
          <button class="chip" data-mode="all">Play all</button>
          <button class="chip" data-mode="repeat">Repeat song</button>
        </div>
        <div class="arng">
          <div class="ctl-hd"><label for="a-vol">Volume</label><span class="num mono" id="a-volv">80%</span></div>
          <input type="range" id="a-vol" min="0" max="150" step="5" value="80">
          <div class="ctl-hd"><label for="a-bass">Bass</label><span class="num mono" id="a-bassv">0 dB</span></div>
          <input type="range" id="a-bass" min="-12" max="12" step="1" value="0">
          <div class="ctl-hd"><label for="a-treble">Treble</label><span class="num mono" id="a-treblev">0 dB</span></div>
          <input type="range" id="a-treble" min="-12" max="12" step="1" value="0">
        </div>
        <p class="note">
          The MAX98357A has <b>no volume register</b> &mdash; it plays whatever
          samples arrive &mdash; so volume, bass and treble are ffmpeg filters.
          They are fixed when a song starts, so letting go of a slider restarts
          the song at the same spot with the new levels: a short gap, not a
          bug. Above 100% loud passages clip. Settings and the chosen device
          are remembered in <b>test/audio.json</b>.
        </p>
      </div>

      <div class="card">
        <h2>Beeps &amp; horn <span class="hint"><kbd>H</kbd> horn</span></h2>
        <div class="beeps">
          <button class="b-audio horn" onclick="beep('horn')">HORN</button>
          <button onclick="beep('beep')">Beep</button>
          <button onclick="beep('double')">Double</button>
          <button onclick="beep('chirp')">Chirp</button>
          <button onclick="beep('reverse')">Reverse</button>
          <button onclick="beep('alert')">Alert</button>
        </div>
        <div class="arng">
          <div class="ctl-hd"><label for="a-blvl">Beep level <span style="color:var(--text-3);font-weight:400">&middot; horn is always full</span></label><span class="num mono" id="a-blvlv">50%</span></div>
          <input type="range" id="a-blvl" min="5" max="80" step="5" value="50">
        </div>
        <p class="note">
          A beep over a song <b>holds the song</b> for the length of the beep
          and then picks it up where it left off &mdash; the I2S device plays
          one stream at a time, so the two cannot be mixed. Letting go of the
          level slider plays a beep at the new level.
          <br><br>
          <b>HORN</b> is an Indian truck pressure horn &mdash; two brassy horns
          slightly out of tune, &ldquo;paa-paa-paaaam&rdquo; &mdash; and always
          plays at <b>full volume</b>, whatever the slider says. For more
          still, the amp's GAIN pin (WIRING.md &sect;7). <b>Reverse</b> sounds
          once a second for 8&nbsp;s; Stop cuts it short.
        </p>
      </div>
    </div>

    <div class="stack">
    <div class="card">
      <h2>Assistant <span class="hint" id="as-status">&mdash;</span></h2>
      <div class="tog">
        <button id="as-toggle" onclick="post('/assistant', {enabled: !asOn})">Assistant</button>
        <button onclick="post('/assistant', {forget: true})">Forget conversation</button>
      </div>
      <div class="stat"><span class="k">Heard</span><span class="v" id="as-heard" style="max-width:70%">&mdash;</span></div>
      <div class="stat"><span class="k">Said</span><span class="v" id="as-reply" style="max-width:70%">&mdash;</span></div>
      <div class="stat"><span class="k">Heard &rarr; first word</span><span class="v num mono" id="as-latency">&mdash;</span></div>
      <div class="stat"><span class="k">Can see / use tools</span><span class="v mono" id="as-caps">&mdash;</span></div>
      <label class="lbl" for="as-url" style="margin-top:10px">Ollama on the PC</label>
      <div class="row2" style="margin-top:0">
        <input type="text" id="as-url" placeholder="http://192.168.1.12:11434" style="flex:1;width:auto">
      </div>
      <label class="lbl" for="as-model" style="margin-top:8px">Model</label>
      <select id="as-model"></select>
      <div class="tog" style="margin-top:8px">
        <button onclick="post('/assistant', {ollama_url: $('as-url').value, model: $('as-model').value})">Apply</button>
      </div>
      <div class="stat"><span class="k">Answering now</span><span class="v mono" id="as-active">&mdash;</span></div>
      <label class="lbl" for="as-pull" style="margin-top:10px">Download a model onto the PC</label>
      <div class="row2 tsay" style="margin-top:0">
        <input type="text" id="as-pull" list="as-suggest" placeholder="qwen3-vl:2b-instruct">
        <button onclick="post('/assistant', {pull: $('as-pull').value})">Download</button>
      </div>
      <datalist id="as-suggest">
        <option value="qwen3-vl:4b-instruct">sees + tools, 3.3 GB — smartest that fits 4 GB</option>
        <option value="qwen3-vl:2b-instruct">sees + tools, 1.9 GB — fastest</option>
        <option value="llama3.2:3b">tools, no vision, 2.0 GB</option>
      </datalist>
      <div class="upbar" id="as-pullbar" style="margin-top:6px"><i id="as-pullfill"></i></div>
      <div class="stat" id="as-pullrow" style="display:none"><span class="k" id="as-pullname">&mdash;</span>
        <span class="v num mono" id="as-pullpct">&mdash;</span></div>
      <div class="aerr" id="as-err"></div>
      <p class="note">
        Talk into the phone page and the truck answers out loud on its own. The
        thinking happens in a <b>free local model in Ollama on your PC</b>
        &mdash; no API keys, no credits &mdash; and the reply is spoken sentence by
        sentence as it streams back. It can look through the camera (vision
        models), check its surroundings, sound the horn, play songs and drive
        short guarded moves (only after you press <b>ENABLE</b>). The PC must
        be on with Ollama listening on the network: Start_pi.md &sect;5.12.
        Turn it <b>off</b> to talk through Claude Code and the MCP
        <b>listen</b> tool instead.
      </p>
    </div>

    <div class="card">
      <h2>Speak <span class="hint" id="t-status">&mdash;</span></h2>
      <textarea id="t-text" maxlength="1000"
                placeholder="Type something for the truck to say&hellip;   Ctrl+Enter speaks"></textarea>
      <div class="row2">
        <button class="b-audio" onclick="speakFrom('t-text')">&#128266; Speak</button>
        <button class="b-stop" onclick="tpost('/tts/stop')">Stop</button>
        <span class="tcount mono num" id="t-count">0 / 1000</span>
      </div>
      <div class="chips" id="t-history" style="margin-top:10px"></div>
      <label class="lbl" for="t-voice" style="margin-top:12px">Voice</label>
      <select id="t-voice"></select>
      <div class="arng">
        <div class="ctl-hd"><label for="t-speed">Speed</label><span class="num mono" id="t-speedv">1.00&times;</span></div>
        <input type="range" id="t-speed" min="50" max="200" step="5" value="100">
        <div class="ctl-hd"><label for="t-vol">Speech volume</label><span class="num mono" id="t-volv">100%</span></div>
        <input type="range" id="t-vol" min="10" max="200" step="5" value="100">
      </div>
      <label class="lbl" for="t-url" style="margin-top:12px">Speech server on the PC
        <span class="hint" id="t-engine"></span></label>
      <div class="row2 tsay" style="margin-top:0">
        <input type="text" id="t-url" placeholder="http://192.168.1.7:5005 — empty: the Pi speaks">
        <button onclick="tpost('/tts/settings', {url: $('t-url').value})">Apply</button>
      </div>
      <details class="voices" id="t-voicebox">
        <summary>Voices &mdash; download more</summary>
        <div id="t-voices"></div>
      </details>
      <div class="aerr" id="t-err"></div>
      <div class="stat" style="margin-top:10px"><span class="k">Phone microphone</span>
        <span class="v"><a id="t-phone" target="_blank" rel="noopener" style="color:var(--audio)"></a></span></div>
      <p class="note">
        <b>Piper</b> neural voices, synthesised on the Pi itself &mdash; no
        internet needed once a voice is downloaded. <b>medium</b> voices are
        the sweet spot on a Pi 4: natural, and a sentence is ready in about a
        second. <b>high</b> sounds a little richer but takes roughly as long
        as the speech itself. The first sentence after choosing a voice waits
        a few seconds while the model loads. Audio streams as it is made, so
        long text starts talking after its first phrase. <b>Speaking again
        cuts off whatever is still talking</b> &mdash; nothing queues up. Speech
        holds a playing song and resumes it afterwards.
        Voices are stored in <b>test/voices/</b>.
        With a <b>speech server on the PC</b> (tools/tts_server.py, or
        <b>docker compose up</b> in tools/) the PC makes the audio instead
        &mdash; a sentence in milliseconds, and no Pi CPU taken from SLAM. If
        the PC stops answering, the Pi speaks for itself and tries the PC
        again after 30 s.
      </p>
    </div>

    <div class="card">
      <h2>Library <span class="hint" id="a-count"></span></h2>
      <label class="drop" id="a-drop">
        Choose songs, or drop them here
        <small id="a-limit">mp3 &middot; wav &middot; ogg &middot; flac &middot; m4a &middot; aac</small>
        <input type="file" id="a-file" accept=".mp3,.wav,.ogg,.flac,.m4a,.aac,audio/*" multiple>
      </label>
      <div class="upbar" id="a-up"><i id="a-upbar"></i></div>
      <div class="songs" id="a-songs"></div>
      <p class="note">
        Songs are stored on the Pi in <b>test/uploads/</b>. For a whole album,
        scp them straight in there instead &mdash; they appear here on the next
        refresh. Names are reduced to letters, digits, dot, dash and
        underscore.
      </p>
    </div>
    </div>

    <div class="card">
      <h2>Speaker test</h2>
      <label class="lbl" for="a-dev">Output device</label>
      <select id="a-dev"></select>
      <div class="chips" id="a-freqs" style="margin:12px 0 10px"></div>
      <div style="display:grid;grid-template-columns:1fr 1fr;gap:10px">
        <div><label class="lbl" for="a-freq">Frequency Hz</label>
          <input type="number" id="a-freq" min="20" max="20000" step="10" value="440"></div>
        <div><label class="lbl" for="a-secs">Seconds</label>
          <input type="number" id="a-secs" min="0.2" max="10" step="0.5" value="2"></div>
      </div>
      <div class="arng">
        <div class="ctl-hd"><label for="a-lvl">Tone level</label><span class="num mono" id="a-lvlv">30%</span></div>
        <input type="range" id="a-lvl" min="0" max="100" step="5" value="30">
      </div>
      <div class="tog" style="margin-top:10px">
        <button class="b-audio" onclick="playTone()">Play tone</button>
        <button onclick="playSweep()">Sweep 40&nbsp;Hz&ndash;15&nbsp;kHz</button>
        <button class="b-stop" onclick="apost('/audio/stop')">Stop</button>
      </div>
      <div class="aerr" id="a-err"></div>
      <p class="note">
        Pick <b>plughw:&hellip; MAX98357A</b> above &mdash; ALSA still lists
        HDMI as card 0, and playing to it is silent with no error. It is
        chosen automatically when the card is found. Tone level is capped in
        software: a full-scale sine into a class-D amp damages a small
        speaker. If the sweep is audible up high but vanishes below ~150&nbsp;Hz,
        that is the speaker enclosure, not the amp.
        <br><br>
        <b>Silent?</b> In order: no card listed &rarr; overlay not loaded
        (<b>config.txt</b>); amp <b>SD</b> pin near 0&nbsp;V &rarr; jumper it
        to Vin; Vin not 5&nbsp;V at the amp; wrong device above.
        WIRING.md &sect;7.
      </p>
    </div>
  </div>
</div>

<!-- ===================================================== TUNE =========== -->
<div class="tabpanel" id="tab-system">
  <div class="cols2">
    <div class="card">
      <h2>Raspberry Pi <span class="hint" id="sy-pi-note"></span></h2>
      <div id="sy-pi"></div>
    </div>
    <div class="card">
      <h2>Pi &mdash; this program, by thread <span class="hint">% of the whole Pi (1 core = 25 %)</span></h2>
      <div id="sy-threads"></div>
    </div>
    <div class="card">
      <h2>PC &mdash; detection (YOLO) <span class="hint" id="sy-det-note"></span></h2>
      <div id="sy-det"></div>
    </div>
    <div class="card">
      <h2>PC &mdash; language model (Ollama) <span class="hint" id="sy-oll-note"></span></h2>
      <div id="sy-oll"></div>
    </div>
    <div class="card">
      <h2>PC &mdash; speech (Piper) <span class="hint" id="sy-tts-note"></span></h2>
      <div id="sy-tts"></div>
    </div>
  </div>
  <p class="sysnote">Updated every 2 s while this tab is open. PC figures come from inside
  Docker, so "Docker VM" is the Linux VM Docker Desktop runs on Windows: it shares the PC's
  cores, but Ollama and other Windows programs are not counted in it. Ollama does not report
  CPU; it reports which models are loaded and how much of each sits on the GPU.</p>
</div>

<div class="tabpanel" id="tab-tune">
  <div class="tuneact">
    <button class="b-enable" onclick="tuneSave()">Save to tuning.json</button>
    <button class="b-stop" onclick="tuneRevert()">Revert to code defaults</button>
    <button onclick="tuneDocs()">What do these do?</button>
    <span class="msg" id="tunemsg"></span>
  </div>

  <div class="card" style="margin-bottom:14px">
    <h2>Live effect</h2>
    <div class="cols2">
      <div class="stat"><span class="k">SLAM cpu</span><span class="v num mono" id="tu-ms">&mdash;</span></div>
      <div class="stat"><span class="k">Match correction</span><span class="v num mono" id="tu-corr">&mdash;</span></div>
      <div class="stat"><span class="k">Scan rate</span><span class="v num mono" id="tu-hz">&mdash;</span></div>
      <div class="stat"><span class="k">Points / turn</span><span class="v num mono" id="tu-pts">&mdash;</span></div>
      <div class="stat"><span class="k">Scans mapped</span><span class="v num mono" id="tu-scans">&mdash;</span></div>
      <div class="stat"><span class="k">Distance driven</span><span class="v num mono" id="tu-dist">&mdash;</span></div>
    </div>
    <p class="note">
      Watch these while you move a slider &mdash; that is the whole point of
      tuning here rather than in <b>pins.py</b>. Raising the search window or
      lowering decimate shows up in <b>SLAM cpu</b> immediately; if it climbs
      past ~150&nbsp;ms the 5&nbsp;Hz SLAM loop has no headroom left.
      Changes apply live but are <b>not saved</b> until you press Save.
    </p>
  </div>

  <div class="tunegrid" id="tunegrid"></div>

  <p class="note" style="margin-top:14px">
    <b>This tab is the index, not the place to tune.</b> Every one of these
    also appears next to the thing it changes &mdash; scanner mounting and
    guard margins on <b>Drive</b> beside the plot, matching and odometry on
    <b>Map</b> beside SLAM cpu and the pose, floor thresholds on
    <b>Vision</b> over the live picture. Tune them there, where you can see
    the effect; come here to see everything at once, to save, or to revert.
    Both views are the same controls, so a change in one shows up in the other.
    <br><br>
    Grid size and resolution are deliberately absent: changing them means
    reallocating the grid and discarding the map, so they stay in
    <b>pins.py</b> and need a restart. Full explanations in
    <b>docs/TUNING.md</b>.
  </p>
</div>

</div><!-- /wrap -->

<script>
const $ = id => document.getElementById(id);

// Show script errors ON THE PAGE.
//
// A single uncaught error stops the rest of the script dead, and what you get
// is a page that looks fine and does nothing - no clue unless you happen to
// have devtools open. This page is usually read on a phone, over wifi, next
// to a robot, where opening a console is not practical. So the page reports
// its own failures.
//
// Registered before anything else runs, so it catches errors in this file too.
addEventListener('error', e => {
  const el = document.getElementById('jserr');
  if(!el) return;
  el.classList.add('show');
  // Template literal, not quoted strings with escapes. Real line breaks are
  // legal inside backticks, so this cannot be broken by anything that mangles
  // a backslash on its way into the file — which is exactly how the previous
  // version of THIS handler ended up an unterminated string literal and took
  // down the whole page it was written to diagnose.
  el.textContent = `Script error — the page has stopped updating.
${e.message || e.error}
${e.filename || ''} line ${e.lineno || '?'}

Reload with Ctrl+Shift+R. If it persists, send this text.`;
});
addEventListener('unhandledrejection', e => {
  const el = document.getElementById('jserr');
  if(!el) return;
  el.classList.add('show');
  el.textContent = 'Request failed: ' + (e.reason && e.reason.message || e.reason);
});
// Live, not baked in: the footprint is tunable from the Drive tab and
// the drawing has to follow it as you drag.
let TRUCK = {len: %%LEN%%, wid: %%WID%%};
const SECTOR = %%SECTOR%%, TILT_WARN = %%TILT%%;
const CAM_HFOV = %%HFOV%%;
// Scanner mounting rotation. Note the MINUS where this is used: a
// return at bearing a lands at body math angle (-a + yaw), and screen
// bearing is the negative of that, so the plot draws (a - yaw). Getting
// this backwards makes eyeball tuning produce the negative of the value
// the guard needs - measured +38 while the slider read -37.
// Scanner mounting rotation, so the plot shows the SAME frame the
// guard and SLAM use. Drawing raw bearings here while the collision
// check used corrected ones made a rotated scanner impossible to see.
let LIDAR_YAW = %%LYAW%%;
// Where each sensor physically sits, body frame, mm. LIDAR_X/Y are
// overwritten from /state whenever the mounting is retuned; the others
// are fixed in pins.py.
let LIDAR_X = %%LX%%, LIDAR_Y = %%LY%%;
const IMU_X = %%IMUX%%, IMU_Y = %%IMUY%%;
const CAM_X = %%CAMX%%, CAM_Y = %%CAMY%%;
let SECTOR_LIVE = %%SECTOR%%;
let CAM_YAW = %%CAMYAW%%;
// Both come from /state so the drawn regions follow a retune of the
// guard immediately - the point of drawing them is to check them.
let GUARD_MARGIN = %%MARGIN%%;
let clearance = {};

let maxRange = 4000, pts = [], clusters = [];
let rpmL = 0, rpmR = 0, phaseL = 0, phaseR = 0, lastFrame = performance.now();
let guardOn = true, guardBlocked = false, stopMm = 350;
let yaw = null, roll = 0, pitch = 0, headingOk = false;

// Camera. camWant is the operator's switch, camLive is whether the Pi is
// actually producing frames. Both have to be true before the <img> gets a
// src: pointing it at a dead stream leaves a broken-image icon on the page
// and a socket open on the Pi for no reason.
let camWant = true, camLive = false, camOn = false;
let cliffOn = false, mkOn = false, mkLearn = false, detOn = false;
let personOn = false, personT = null, followOn = false;   // person tracker; personT is ringed on the plot
let objects = [];
// What goes over the live picture. Boxes by default - a label is only
// checkable if you can see WHICH thing it was put on.
let overlay = 'boxes';
try { overlay = localStorage.getItem('nav.overlay') || 'boxes'; } catch(e) {}
let shots = [], knownTags = [], autoMm = 0, mapHit = [];
// 1x1 transparent GIF. Assigning this is what CLOSES an MJPEG connection —
// img.src='' makes some browsers re-request the page itself, and simply
// hiding the element leaves the stream running and the bandwidth spent.
const BLANK = 'data:image/gif;base64,R0lGODlhAQABAAAAACH5BAEKAAEALAAAAAABAAEAAAICTAEAOw==';

const RANGES = [1000, 2000, 4000, 8000];
$('ranges').innerHTML = RANGES.map(r => `<button class="chip" data-r="${r}">${r/1000} m</button>`).join('');
$('ranges').querySelectorAll('.chip').forEach(b => b.addEventListener('click', () => {
  maxRange = +b.dataset.r; markRange(); }));
function markRange(){ $('ranges').querySelectorAll('.chip')
  .forEach(b => b.classList.toggle('sel', +b.dataset.r === maxRange)); }
markRange();

// Size a canvas from its CONTAINER, never from itself.
//
// c.clientWidth can be fed by the canvas's own width ATTRIBUTE whenever CSS
// leaves a dimension auto - and draw() sets that attribute from clientWidth,
// so the two chase each other and the canvas doubles 25 times a second until
// the tab dies. It looks exactly like the robot being slow, and it came back
// once after a CSS-only fix, so the guarantee belongs here in the JS rather
// than resting on a stylesheet staying correct.
//
// The parent box is laid out by the grid and cannot depend on the canvas, so
// measuring it is safe. The size is also clamped: if anything ever does feed
// back, it stops at something survivable instead of locking the browser.
function fitCanvas(c, self){
  // `self` measures the canvas rather than its parent. Only for the IMU
  // dials, whose CSS pins width to 100% and derives height from
  // aspect-ratio - no auto dimension, so no path to the attribute - and
  // whose parent box also contains the caption and value text.
  const box = (self ? c : c.parentElement).getBoundingClientRect();
  const w = Math.max(1, Math.min(2000, Math.round(box.width)));
  const h = Math.max(1, Math.min(2000, Math.round(box.height)));
  const dpr = Math.min(2, devicePixelRatio || 1);
  const bw = Math.round(w*dpr), bh = Math.round(h*dpr);
  if(c.width !== bw || c.height !== bh){ c.width = bw; c.height = bh; }
  const g = c.getContext('2d');
  g.setTransform(dpr,0,0,dpr,0,0);
  g.clearRect(0,0,w,h);
  return [g, w, h];
}

const css = k => getComputedStyle(document.documentElement).getPropertyValue(k).trim();
const mm = v => v == null ? '—' : (v >= 1000 ? (v/1000).toFixed(2)+' m' : Math.round(v)+' mm');

// ---------------------------------------------------------------- polar plot
// ---------------------------------------------------------------- the plot
//
// Everything below works in the BODY frame: origin at the truck's geometric
// centre, +x forward, +y LEFT. That is the frame body_points() in web_nav.py
// builds and the frame the collision guard and SLAM judge against.
//
// It did not used to be. The old renderer applied the scanner's ROTATION to
// each bearing and then drew it from the canvas centre, never applying the
// scanner's POSITION. So the picture and the guard were working 250 mm apart
// - the scanner sits on the back left corner, not the middle - and a wall the
// guard called 350 mm away could be drawn somewhere else entirely. Points,
// clusters, wedges and the truck are now all put through the same transform.

let zoom = 1, panX = 0, panY = 0;      // pan is in mm, body frame

// px per mm, and where the body origin lands on screen.
function view(w, h){
  const R = Math.min(w, h) / 2 - 26;
  const s = R / maxRange * zoom;
  return {s, R, ox: w / 2 - (0 - panY) * s, oy: h / 2 - (0 - panX) * s};
}
// body (mm) -> screen (px)
function P(v, x, y){ return [v.ox - (y) * v.s, v.oy - (x) * v.s]; }

// One scan return -> body frame. This is body_points() in JavaScript; if the
// two ever disagree the picture is lying about what the guard can see.
function toBody(a, d){
  const r = a * Math.PI / 180;
  let x = d * Math.cos(r), y = -d * Math.sin(r);
  if(LIDAR_YAW){
    const c = Math.cos(LIDAR_YAW * Math.PI / 180), sn = Math.sin(LIDAR_YAW * Math.PI / 180);
    const nx = x * c - y * sn; y = x * sn + y * c; x = nx;
  }
  return [x + LIDAR_X, y + LIDAR_Y];
}

function draw(){
  const now = performance.now(), dt = Math.min(0.1, (now - lastFrame)/1000);
  lastFrame = now;
  phaseL = (phaseL + rpmL/60*360*dt) % 360;
  phaseR = (phaseR + rpmR/60*360*dt) % 360;

  if(tab !== 'drive') return;
  const c = $('plot');
  const [g, w, h] = fitCanvas(c);
  if(w < 2 || h < 2) return;

  const v = view(w, h);
  const [ox, oy] = [v.ox, v.oy];

  // --- range rings, centred on the BODY, not on the scanner ---------------
  g.font = '10px ui-monospace,monospace'; g.textAlign='center'; g.textBaseline='middle';
  const span = maxRange / zoom;
  const stepM = span <= 800 ? 0.25 : span <= 2000 ? 0.5 : span <= 4000 ? 1 : 2;
  for(let m = stepM; m*1000 <= maxRange*1.6; m += stepM){
    const r = m*1000*v.s;
    if(r > Math.hypot(w,h)) break;
    g.strokeStyle = css('--grid'); g.lineWidth = 1;
    g.beginPath(); g.arc(ox,oy,r,0,6.2832); g.stroke();
    g.fillStyle = css('--text-3'); g.fillText(m+' m', ox, oy-r-8);
  }
  for(let a = 0; a < 360; a += 45){
    const rad = (a-90)*Math.PI/180;
    g.strokeStyle = css('--grid'); g.globalAlpha = .5;
    g.beginPath(); g.moveTo(ox,oy);
    g.lineTo(ox+Math.cos(rad)*v.R*1.5, oy+Math.sin(rad)*v.R*1.5);
    g.stroke(); g.globalAlpha = 1;
  }

  // --- the regions the guard ACTUALLY tests -------------------------------
  //
  // Not a pie slice. swept_obstacle() checks a RECTANGLE the full width of
  // the truck plus margin when driving, and an ANNULUS out to the
  // circumscribing radius when turning on the spot. A wedge is neither, so
  // the old drawing could not be used to verify anything - an obstacle could
  // sit inside the wedge and outside the tested box, or the reverse.
  //
  // Each region is coloured by its OWN clearance, so you can put something in
  // front of the robot and watch that box go amber while the others stay
  // clear. That is the check: if the box covers the obstacle and the guard
  // has not blocked, the geometry is wrong.
  drawGuard(g, v, w, h);

  // --- camera wedge, from where the LENS is ------------------------------
  //
  // Drawn from the camera's own position, not the body centre. The lens sits
  // 150 mm forward, so a centre-drawn wedge overstates what it can see close
  // in by exactly that much - which is the range the cliff check works at.
  if(camLive && CAM_HFOV > 0){
    const half = CAM_HFOV/2, mid = -90 - CAM_YAW;
    const [sx, sy] = P(v, CAM_X, CAM_Y);
    const a0 = (mid - half)*Math.PI/180, a1 = (mid + half)*Math.PI/180;
    g.beginPath(); g.moveTo(sx,sy); g.arc(sx,sy,v.R*1.4,a0,a1); g.closePath();
    g.fillStyle = css('--cam'); g.globalAlpha = .07; g.fill();
    g.globalAlpha = .5; g.strokeStyle = css('--cam'); g.lineWidth = 1;
    g.setLineDash([4,4]); g.stroke(); g.setLineDash([]); g.globalAlpha = 1;
  }

  // --- the scan ----------------------------------------------------------
  g.fillStyle = css('--point');
  const dot = Math.max(1.3, Math.min(3, v.s*900));
  for(const [a,d] of pts){
    if(d > maxRange*1.6) continue;
    const [bx,by] = toBody(a,d);
    const [sx,sy] = P(v,bx,by);
    if(sx < -20 || sy < -20 || sx > w+20 || sy > h+20) continue;
    g.beginPath(); g.arc(sx,sy,dot,0,6.2832); g.fill();
  }

  // --- the person being tracked: a ring where they stand, and the line the
  // camera sees them along. Drawn from the LENS, like the wedge, because the
  // bearing and distance are both measured from there.
  if(personT && personT.body){
    const [px, py] = P(v, personT.body[0], personT.body[1]);
    const [cx, cy] = P(v, CAM_X, CAM_Y);
    g.strokeStyle = css('--audio'); g.lineWidth = 2;
    g.setLineDash([5,4]); g.beginPath(); g.moveTo(cx,cy); g.lineTo(px,py); g.stroke();
    g.setLineDash([]); g.beginPath(); g.arc(px, py, Math.max(8, 250*v.s), 0, 6.2832); g.stroke();
    g.fillStyle = css('--audio'); g.textAlign = 'left'; g.textBaseline = 'middle';
    g.fillText(mm(personT.distance_mm) + ' ' + personT.source, px + Math.max(10, 260*v.s), py);
  }

  // Cluster circles used to be drawn here. They were removed: cluster
  // identity is not stable frame to frame, so the rings flickered and
  // renumbered constantly while telling you nothing the points did not
  // already show. The clustering itself is still computed and still useful
  // as a TABLE - it is on the Sensors tab, sorted by distance, where a
  // number you can read beats a ring that moves.

  drawTruck(g, v);
  drawLegend(g, w, h, v);
}

// What the guard sees, as one picture: GREEN is open floor the scanner can
// see into; RED is where the truck's CENTRE cannot go. Every return is drawn
// as a truck-sized block (the footprint plus the guard's side margin) around
// it, so "the centre dot touches red" is exactly "the car's outline touches
// that point" - the same rule the guard enforces. Arrows round the truck are
// the guard's own verdicts for each move, from /state, never recomputed here.
const SIDE_MARGIN = 15;                    // SIDE_MARGIN_MM in the Python guard
const redBuf = document.createElement('canvas');
function drawGuard(g, v, w, h){
  const hl = TRUCK.len/2, hw = TRUCK.wid/2;
  const [lx, ly] = P(v, LIDAR_X, LIDAR_Y);

  // Returns in the body frame, filtered exactly as body_points() filters.
  const seen = [], obst = [];
  for(const [a, d] of pts){
    if(d < 120) continue;
    const [x, y] = toBody(a, d);
    seen.push([a, x, y]);
    if(!(Math.abs(x) <= hl && Math.abs(y) <= hw)) obst.push([x, y]);
  }

  // GREEN: the polygon the scanner sees through, in bearing order. A gap of
  // more than 8 degrees with no return is unknown, not free, so the outline
  // goes back to the scanner across it instead of painting it green.
  if(seen.length > 2){
    seen.sort((p, q) => p[0] - q[0]);
    g.beginPath(); g.moveTo(lx, ly);
    let prev = null;
    for(const [a, x, y] of seen){
      const [sx, sy] = P(v, x, y);
      if(prev !== null && a - prev > 8){ g.lineTo(lx, ly); }
      g.lineTo(sx, sy);
      prev = a;
    }
    g.closePath();
    g.fillStyle = css('--good'); g.globalAlpha = .26; g.fill(); g.globalAlpha = 1;
  }

  // RED: the no-go area for the centre. Drawn solid off-screen and laid on
  // once, so overlapping blocks do not stack into darker patches.
  const dpr = Math.min(2, devicePixelRatio || 1);
  if(redBuf.width !== Math.round(w*dpr) || redBuf.height !== Math.round(h*dpr)){
    redBuf.width = Math.round(w*dpr); redBuf.height = Math.round(h*dpr);
  }
  const rg = redBuf.getContext('2d');
  rg.setTransform(dpr, 0, 0, dpr, 0, 0);
  rg.clearRect(0, 0, w, h);
  rg.fillStyle = css('--critical');
  const bw = (hw + SIDE_MARGIN) * v.s, bh = hl * v.s;     // half-sizes on screen
  for(const [x, y] of obst){
    const [sx, sy] = P(v, x, y);
    if(sx < -bw || sy < -bh || sx > w + bw || sy > h + bh) continue;
    rg.fillRect(sx - bw, sy - bh, 2*bw, 2*bh);
  }
  g.globalAlpha = .42; g.drawImage(redBuf, 0, 0, w, h); g.globalAlpha = 1;

  // The centre: this dot must stay in the green.
  const [cx, cy] = P(v, 0, 0);
  g.fillStyle = '#fff';
  g.beginPath(); g.arc(cx, cy, 3, 0, 6.2832); g.fill();

  // The guard's verdict for each move.
  const verdict = k => !guardOn ? 'off'
    : (clearance[k] != null && clearance[k] < stopMm ? 'no' : 'ok');
  const col = s_ => s_ === 'ok' ? css('--good') : (s_ === 'no' ? css('--critical') : css('--text-3'));
  const tri = (x, y, up, s_) => {
    const k = up ? -1 : 1;
    g.fillStyle = col(s_); g.globalAlpha = s_ === 'off' ? .5 : .95;
    g.beginPath(); g.moveTo(x, y + k*9); g.lineTo(x - 8, y - k*5); g.lineTo(x + 8, y - k*5);
    g.closePath(); g.fill(); g.globalAlpha = 1;
  };
  const [fx, fy] = P(v, hl, 0), [rx, ry] = P(v, -hl, 0);
  tri(fx, fy - 16, true, verdict('fwd'));
  tri(rx, ry + 16, false, verdict('rev'));
  g.font = 'bold 16px system-ui,sans-serif'; g.textAlign = 'center'; g.textBaseline = 'middle';
  const [ax, ay] = P(v, 0, hw), [bx, by] = P(v, 0, -hw);
  [['left', ax - 16, ay, '⟲'], ['right', bx + 16, by, '⟳']].forEach(([k, x, y, t]) => {
    const s_ = verdict(k);
    g.fillStyle = col(s_); g.globalAlpha = s_ === 'off' ? .5 : .95;
    g.fillText(t, x, y); g.globalAlpha = 1;
  });

  // Key, bottom-left.
  g.font = '10px ui-monospace,monospace'; g.textAlign = 'left'; g.textBaseline = 'alphabetic';
  g.fillStyle = css('--text-3');
  g.fillText(guardOn ? 'green: free  ·  red: the centre dot cannot go there'
                     : 'GUARD OFF  ·  nothing is being blocked', 8, h - 8);
}

// The truck, and every sensor drawn where it physically sits.
function drawTruck(g, v){
  const L = TRUCK.len*v.s, W = TRUCK.wid*v.s;
  const [ox, oy] = P(v, 0, 0);
  if(L < 5){
    g.fillStyle = css('--text-2');
    g.beginPath(); g.arc(ox,oy,3,0,6.2832); g.fill(); return;
  }

  // The chassis. Drawn with a chamfered nose rather than as a plain
  // rectangle, because on a skid-steer the plot is the only thing telling
  // you which way the robot faces - the shape has to say it at a glance,
  // at any zoom, without reading a label.
  g.save(); g.translate(ox, oy);
  const nose = Math.min(L*0.22, W*0.30);          // chamfer, front corners
  g.beginPath();
  g.moveTo(-W/2 + nose, -L/2);
  g.lineTo( W/2 - nose, -L/2);
  g.lineTo( W/2, -L/2 + nose);
  g.lineTo( W/2,  L/2 - 2);
  g.arcTo( W/2,  L/2, W/2 - 3, L/2, Math.min(4, W/6));
  g.lineTo(-W/2 + 3, L/2);
  g.arcTo(-W/2,  L/2, -W/2, L/2 - 3, Math.min(4, W/6));
  g.lineTo(-W/2, -L/2 + nose);
  g.closePath();
  g.fillStyle = css('--surface-2'); g.globalAlpha = .88; g.fill();
  g.globalAlpha = 1;
  g.strokeStyle = css('--text-3'); g.lineWidth = 1.2; g.stroke();

  // Front edge picked out in the good colour: the single strongest cue for
  // orientation, and it survives being only a few pixels long.
  g.strokeStyle = css('--good'); g.lineWidth = Math.max(1.5, L*0.035);
  g.globalAlpha = .95;
  g.beginPath();
  g.moveTo(-W/2 + nose, -L/2); g.lineTo(W/2 - nose, -L/2); g.stroke();
  g.globalAlpha = 1;

  // Heading ray out of the nose, so orientation reads even when the body is
  // only a few pixels across.
  g.strokeStyle = css('--good'); g.lineWidth = 1.2; g.globalAlpha = .5;
  g.setLineDash([3,3]);
  g.beginPath(); g.moveTo(0,-L/2); g.lineTo(0,-L/2 - Math.max(14, L*0.5));
  g.stroke(); g.setLineDash([]); g.globalAlpha = 1;

  // Wheels, turning at the measured RPM. The spoke is what makes direction
  // visible: a plain block cannot show which way it is going round, and
  // watching one side reverse while the other drives forward is how you see
  // a skid-steer turning on the spot.
  const wl = Math.max(4, L*0.30), ww = Math.max(3, W*0.16);
  for(const [sx,sy,side] of [[-1,-1,'l'],[1,-1,'r'],[-1,1,'l'],[1,1,'r']]){
    const x = sx*(W/2), y = sy*(L*0.27);
    const col = side==='l' ? css('--series-1') : css('--series-2');
    g.fillStyle = col; g.globalAlpha = .92;
    g.beginPath(); g.roundRect(x-ww/2, y-wl/2, ww, wl, ww/2.2); g.fill();
    g.globalAlpha = 1;
    if(wl > 9){
      const ph = (side==='l'?phaseL:phaseR)*Math.PI/180;
      // Two spokes half a turn apart, so the tread reads as rotating rather
      // than as one mark sliding up and down.
      g.strokeStyle = '#fff'; g.lineWidth = 1.1;
      for(const off of [0, Math.PI]){
        const t = Math.sin(ph + off);
        g.globalAlpha = .25 + .55*Math.abs(Math.cos(ph + off));
        g.beginPath();
        g.moveTo(x - ww/2 + 0.8, y + t*wl/2*0.78);
        g.lineTo(x + ww/2 - 0.8, y + t*wl/2*0.78);
        g.stroke();
      }
      g.globalAlpha = 1;
    }
  }
  g.restore();

  // Sensor markers. Labels appear once there is room for them rather than at
  // a fixed zoom, so they never overlap into unreadability.
  const label = L > 90;
  marker(g, v, LIDAR_X, LIDAR_Y, css('--point'), 'LIDAR', LIDAR_YAW, label,
         `${LIDAR_X.toFixed(0)}, ${LIDAR_Y.toFixed(0)} @ ${LIDAR_YAW.toFixed(0)}°`);
  marker(g, v, IMU_X, IMU_Y, css('--imu'), 'IMU',
         yaw === null ? null : -yaw, label,
         yaw === null ? 'no fix' : `heading ${yaw.toFixed(0)}°`);
  marker(g, v, CAM_X, CAM_Y, css('--cam'), 'CAM', CAM_YAW, label,
         `${CAM_HFOV.toFixed(0)}° fov`);

  if(label){
    g.fillStyle = css('--text-3'); g.font = '9px ui-monospace,monospace';
    g.textAlign = 'center';
    g.fillText(`${TRUCK.len} × ${TRUCK.wid} mm`, ox, oy + L/2 + 12);
  }
}

// A sensor: dot, facing arrow, and (when there is room) what it is.
function marker(g, v, bx, by, colour, name, headingDeg, label, sub){
  const [x, y] = P(v, bx, by);
  g.fillStyle = colour;
  g.beginPath(); g.arc(x, y, 4, 0, 6.2832); g.fill();
  g.strokeStyle = colour; g.globalAlpha = .35; g.lineWidth = 1;
  g.beginPath(); g.arc(x, y, 7.5, 0, 6.2832); g.stroke(); g.globalAlpha = 1;

  if(headingDeg !== null && headingDeg !== undefined){
    const rad = (-headingDeg - 90) * Math.PI/180, len = 17;
    g.strokeStyle = colour; g.lineWidth = 1.6;
    g.beginPath(); g.moveTo(x, y);
    g.lineTo(x + Math.cos(rad)*len, y + Math.sin(rad)*len); g.stroke();
    g.beginPath();
    g.moveTo(x + Math.cos(rad)*len, y + Math.sin(rad)*len);
    g.lineTo(x + Math.cos(rad+2.6)*6, y + Math.sin(rad+2.6)*6);
    g.lineTo(x + Math.cos(rad-2.6)*6, y + Math.sin(rad-2.6)*6);
    g.closePath(); g.fillStyle = colour; g.fill();
  }
  if(label){
    g.fillStyle = colour; g.font = 'bold 9px ui-monospace,monospace';
    g.textAlign = 'left'; g.textBaseline = 'middle';
    g.fillText(name, x + 10, y - 5);
    g.fillStyle = css('--text-3'); g.font = '8px ui-monospace,monospace';
    g.fillText(sub, x + 10, y + 5);
    g.textAlign = 'center'; g.textBaseline = 'middle';
  }
}

// Fixed overlay: which way is north, what scale we are at, and the numbers
// currently driving the picture. Without these the plot is a pretty shape.
function drawLegend(g, w, h, v){
  g.font = '9px ui-monospace,monospace'; g.textAlign = 'left'; g.textBaseline = 'top';
  const lines = [
    `${(maxRange/zoom/1000).toFixed(2)} m across  ·  zoom ${zoom.toFixed(1)}×`,
    `body frame  ·  +x forward  ·  +y left`,
  ];
  g.fillStyle = css('--text-3');
  lines.forEach((t,i) => g.fillText(t, 8, 8 + i*12));

  // north, as a small rose rather than a marker on a ring that no longer
  // exists now the view pans
  if(yaw !== null){
    const rx = w - 26, ry = 26, rad = (-yaw - 90)*Math.PI/180;
    g.strokeStyle = css('--border'); g.lineWidth = 1;
    g.beginPath(); g.arc(rx, ry, 15, 0, 6.2832); g.stroke();
    g.strokeStyle = headingOk ? css('--imu') : css('--text-3'); g.lineWidth = 2;
    g.beginPath(); g.moveTo(rx, ry);
    g.lineTo(rx + Math.cos(rad)*12, ry + Math.sin(rad)*12); g.stroke();
    g.fillStyle = headingOk ? css('--imu') : css('--text-3');
    g.font = 'bold 9px ui-monospace,monospace';
    g.textAlign = 'center'; g.textBaseline = 'middle';
    g.fillText('N', rx + Math.cos(rad)*20, ry + Math.sin(rad)*20);
  }
  g.textAlign = 'center'; g.textBaseline = 'middle';
}

// --- zoom and pan ----------------------------------------------------------
//
// The projection is  sx = w/2 - (y - panY)*s ,  sy = h/2 - (x - panX)*s
// so inverting it, the body point under a cursor offset (mx, my) from the
// centre is  y = panY - mx/s ,  x = panX - my/s.
// Both handlers below are just that algebra rearranged; the signs are worth
// deriving rather than guessing, because a wrong one still "works" and only
// feels subtly inverted.
(function plotNav(){
  const c = $('plot');
  let dragging = false, lx = 0, ly = 0;
  const box = () => c.parentElement.getBoundingClientRect();
  const scale = () => (Math.min(box().width, box().height)/2 - 26)/maxRange*zoom;

  c.addEventListener('wheel', e => {
    e.preventDefault();
    const s0 = scale();
    zoom = Math.max(0.25, Math.min(24, zoom * (e.deltaY < 0 ? 1.15 : 1/1.15)));
    const s1 = scale();
    // Hold whatever is under the cursor still. Without this the view always
    // creeps toward the centre and you lose the thing you were zooming at.
    const b = box();
    const mx = e.offsetX - b.width/2, my = e.offsetY - b.height/2;
    panY += mx*(1/s1 - 1/s0);
    panX += my*(1/s1 - 1/s0);
    draw();
  }, {passive:false});

  c.addEventListener('pointerdown', e => {
    dragging = true; lx = e.clientX; ly = e.clientY;
    c.classList.add('drag'); c.setPointerCapture(e.pointerId);
  });
  c.addEventListener('pointermove', e => {
    if(!dragging) return;
    const s = scale();
    // Content follows the finger: d(sx)/d(panY) = +s, likewise for x.
    panY += (e.clientX - lx)/s;
    panX += (e.clientY - ly)/s;
    lx = e.clientX; ly = e.clientY;
    draw();
  });
  const stop = () => { dragging = false; c.classList.remove('drag'); };
  c.addEventListener('pointerup', stop);
  c.addEventListener('pointercancel', stop);
  c.addEventListener('dblclick', () => { zoom = 1; panX = panY = 0; draw(); });
})();

// ---------------------------------------------------------------- IMU dials
function dialCtx(id){
  const c = $(id);
  if(!c.clientWidth) return null;      // hidden tab: nothing to draw on
  const [g, w, h] = fitCanvas(c, true);
  if(w < 2 || h < 2) return null;
  return [g, w/2, h/2, Math.min(w,h)/2 - 6];
}

function drawCompass(){
  const dc = dialCtx('compass');
  if(!dc) return;
  const [g, cx, cy, R] = dc;
  g.strokeStyle = css('--border'); g.lineWidth = 1;
  g.beginPath(); g.arc(cx,cy,R,0,6.2832); g.stroke();
  if(yaw === null){
    g.fillStyle = css('--text-3'); g.font = '11px ui-monospace,monospace';
    g.textAlign='center'; g.textBaseline='middle'; g.fillText('no IMU', cx, cy);
    return;
  }
  g.font = '10px ui-monospace,monospace'; g.textAlign='center'; g.textBaseline='middle';
  // the card rotates, the robot stays pointing up — how a real compass reads
  for(let a = 0; a < 360; a += 15){
    const rad = (a - yaw - 90)*Math.PI/180;
    const major = a % 90 === 0;
    const r0 = R - (major ? 12 : 6);
    g.strokeStyle = major ? css('--text-2') : css('--grid');
    g.lineWidth = major ? 1.6 : 1;
    g.beginPath();
    g.moveTo(cx+Math.cos(rad)*r0, cy+Math.sin(rad)*r0);
    g.lineTo(cx+Math.cos(rad)*R, cy+Math.sin(rad)*R); g.stroke();
    if(major){
      const lbl = {0:'N',90:'E',180:'S',270:'W'}[a];
      g.fillStyle = a === 0 ? css('--imu') : css('--text-2');
      g.fillText(lbl, cx+Math.cos(rad)*(R-22), cy+Math.sin(rad)*(R-22));
    }
  }
  g.fillStyle = headingOk ? css('--imu') : css('--text-3');
  g.beginPath(); g.moveTo(cx, cy-R+26); g.lineTo(cx-7, cy+9); g.lineTo(cx+7, cy+9);
  g.closePath(); g.fill();
  g.fillStyle = css('--text-3');
  g.beginPath(); g.arc(cx,cy,3,0,6.2832); g.fill();
}

function drawHorizon(){
  const dc = dialCtx('horizon');
  if(!dc) return;
  const [g, cx, cy, R] = dc;
  g.save();
  g.beginPath(); g.arc(cx,cy,R,0,6.2832); g.clip();
  g.translate(cx, cy);
  g.rotate(-roll*Math.PI/180);
  const ppd = R/45;                       // pixels per degree of pitch
  const yh = pitch*ppd;
  g.fillStyle = css('--sky');    g.fillRect(-R*2, -R*2+yh, R*4, R*2);
  g.fillStyle = css('--ground'); g.fillRect(-R*2, yh, R*4, R*2);
  g.strokeStyle = css('--text-2'); g.lineWidth = 1.5;
  g.beginPath(); g.moveTo(-R, yh); g.lineTo(R, yh); g.stroke();
  g.strokeStyle = css('--text-3'); g.lineWidth = 1;
  g.font = '8px ui-monospace,monospace'; g.textAlign='center'; g.textBaseline='middle';
  for(const p of [-30,-20,-10,10,20,30]){
    const y = yh - p*ppd, half = Math.abs(p) % 20 === 0 ? 20 : 11;
    g.beginPath(); g.moveTo(-half, y); g.lineTo(half, y); g.stroke();
  }
  g.restore();
  // fixed aircraft symbol
  g.strokeStyle = css('--imu'); g.lineWidth = 2.5;
  g.beginPath();
  g.moveTo(cx-26, cy); g.lineTo(cx-9, cy); g.lineTo(cx, cy+7); g.lineTo(cx+9, cy);
  g.lineTo(cx+26, cy); g.stroke();
  g.strokeStyle = css('--border'); g.lineWidth = 1;
  g.beginPath(); g.arc(cx,cy,R,0,6.2832); g.stroke();
}

// ---------------------------------------------------------------- input
const keys = new Set();
const MAP = {ArrowUp:'f',KeyW:'f',ArrowDown:'b',KeyS:'b',ArrowLeft:'l',KeyA:'l',ArrowRight:'r',KeyD:'r'};
const PADS = {'p-f':'f','p-b':'b','p-l':'l','p-r':'r'};
function paint(){ for(const [id,k] of Object.entries(PADS)) $(id).classList.toggle('on', keys.has(k)); }
function sendDrive(){
  const throttle = (keys.has('f')?1:0)-(keys.has('b')?1:0);
  const steer = (keys.has('r')?1:0)-(keys.has('l')?1:0);
  fetch('/drive',{method:'POST',headers:{'Content-Type':'application/json'},
                  body:JSON.stringify({throttle,steer})});
}
// Typing is not driving. Without this, naming a room "bedroom" or "washroom"
// sent D, W, A, S to the motors, and a space in "living room" was an e-stop.
const typing = e => e.target && (e.target.isContentEditable
  || /^(INPUT|TEXTAREA|SELECT)$/.test(e.target.tagName));
addEventListener('keydown', e => {
  if(typing(e)) return;
  if(e.code === 'Space'){ e.preventDefault(); cmd('/estop'); keys.clear(); paint(); return; }
  const k = MAP[e.code]; if(!k || e.repeat) return;
  e.preventDefault();                        // arrow keys must not scroll
  keys.add(k); paint();
  sendDrive();          // go now; waiting for the tick added up to 50 ms
});
// keyup is NOT filtered: a key held while focus moves into a box must still
// release, or it would latch on.
// Only a key that was driving sends anything on release: typing W-A-S-D into
// a name box otherwise fired a zero-speed /drive per letter, and each one
// briefly overrode a trip in progress.
addEventListener('keyup', e => { const k = MAP[e.code];
  if(k && keys.has(k)){ e.preventDefault(); keys.delete(k); paint(); sendDrive(); } });
addEventListener('blur', () => { keys.clear(); paint(); });   // never latch on
for(const [id,k] of Object.entries(PADS)){
  const el = $(id);
  const on = e => { e.preventDefault(); keys.add(k); paint(); sendDrive(); };
  const off = e => { e.preventDefault(); keys.delete(k); paint(); sendDrive(); };
  el.addEventListener('pointerdown', on);
  ['pointerup','pointerleave','pointercancel'].forEach(ev => el.addEventListener(ev, off));
}
$('p-s').addEventListener('click', () => { keys.clear(); paint(); cmd('/stop'); });
let wasActive = false;
setInterval(() => { const a = keys.size > 0; if(a || wasActive) sendDrive(); wasActive = a; }, 50);

// ---------------------------------------------------------------- commands
function cmd(u){ fetch(u,{method:'POST'}).then(poll); }
function post(u,b){ return fetch(u,{method:'POST',headers:{'Content-Type':'application/json'},
                                    body:JSON.stringify(b)}).then(poll); }
function toggle(k){ post('/invert', {[k]: !$('t-'+k).classList.contains('on')}); }
function toggleGuard(){ post('/guard', {enabled: !guardOn}); }

// The <img> is created and destroyed rather than shown and hidden. An MJPEG
// response never ends on its own, so a hidden <img> keeps its connection
// open, keeps the Pi encoding for nobody, and keeps one of Chrome's six
// per-host sockets busy — which is enough to make the drive commands queue.
function applyCam(){
  const want = camWant && camLive;
  $('t-cam').classList.toggle('g-on', camWant);
  if(want === camOn) return;                 // no needless reconnects
  camOn = want;
  const wrap = document.querySelector('.camwrap');
  let img = wrap.querySelector('img');
  if(want){
    if(!img){ img = new Image(); img.alt = 'camera';
              ($('camrot') || wrap).insertBefore(img, $('cliffgrid')); }
    // Cache-buster: without it a browser will happily reuse the dead socket
    // from the previous attempt and the picture never comes back.
    img.src = '/camera.mjpg?t=' + Date.now();
    $('camoff').style.display = 'none';
    $('camdot').style.display = '';
  } else {
    if(img){ img.src = BLANK; img.remove(); }
    $('camoff').style.display = '';
    $('camdot').style.display = 'none';
  }
}
function toggleCam(){ camWant = !camWant; applyCam(); }

function setDetectUrl(){
  post('/detect', {url: $('d-url').value.trim()})
    .then(() => flash('Detection server set. Blank the box to go back on-board.'));
}

function setOverlay(v){
  if(v === 'floor') v = 'boxes';        // the floor grid is gone; old saved choice
  overlay = v;
  try { localStorage.setItem('nav.overlay', v); } catch(e) {}
  document.querySelectorAll('#overlays .chip').forEach(b =>
    b.classList.toggle('sel', b.dataset.ov === v));
  $('cliffgrid').style.display = v === 'floor' ? '' : 'none';
  $('detboxes').style.display  = v === 'boxes' ? '' : 'none';
}
document.querySelectorAll('#overlays .chip').forEach(b =>
  b.addEventListener('click', () => setOverlay(b.dataset.ov)));
setOverlay(overlay);

// Detection boxes over the live picture.
//
// Percentages, not pixels: the frame is letterboxed into a container of
// unknown size and may be rotated a quarter turn, and a percentage rides
// through both without this code knowing either.
function drawBoxes(seen){
  const el = $('detboxes');
  if(overlay !== 'boxes'){ if(el.childElementCount) el.innerHTML = ''; return; }
  if(!seen || !seen.length){ if(el.childElementCount) el.innerHTML = ''; return; }
  el.innerHTML = seen.filter(o => o.box).map(o => {
    const [x0,y0,x1,y1] = o.box;
    // A box with no range behind it was seen but could not be placed.
    // People (personBoxes) arrive with their own class and caption.
    const cls = o.cls || (o.placed ? 'detbox' : 'detbox noplace');
    const tag = o.tag || (o.label.replace('_',' ') + ' ' + ((o.conf*100)|0) + '%'
              + (o.range != null ? ' · ' + mm(o.range) : ' · no range'));
    return `<div class="${cls}" style="left:${(x0*100).toFixed(2)}%;`
         + `top:${(y0*100).toFixed(2)}%;`
         + `width:${((x1-x0)*100).toFixed(2)}%;`
         + `height:${((y1-y0)*100).toFixed(2)}%"><b>${tag}</b></div>`;
  }).join('');
}

// The person tracker's boxes, in drawBoxes' shape. Caption: the distance and
// which sensor it came from, because "1.4 m" from the LiDAR and "1.4 m" from
// the box size deserve very different amounts of trust.
function personBoxes(p){
  if(!p || !p.enabled || !p.people) return [];
  const t = p.target;
  return p.people.map(o => {
    const target = t && o.bearing === t.bearing && o.conf === t.conf;
    return {box: o.box, cls: 'detbox person' + (target ? ' target' : ''),
            tag: (target ? '▶ ' : '') + 'person ' + ((o.conf*100)|0) + '%'
                 + (o.distance_mm != null ? ' · ' + mm(o.distance_mm) + ' ' + o.source : '')};
  });
}

// Prose off by default. It is what a new person needs and what everyone else
// is scrolling past, so it gets one switch rather than a compromise.
function toggleDocs(){
  const on = document.body.classList.toggle('docs');
  try { localStorage.setItem('nav.docs', on ? '1' : ''); } catch(e) {}
  // The canvases are sized by the space left over, which just changed.
  requestAnimationFrame(() => { draw(); drawMap(); });
}
try { if(localStorage.getItem('nav.docs')) document.body.classList.add('docs'); }
catch(e) {}

// The floor grid, drawn over the video as plain DOM rather than a canvas.
// It sits on top of an <img> that is being replaced 15 times a second, and
// a canvas would have to be composited against a stream it cannot read.
function drawCliff(cf){
  const el = $('cliffgrid');
  if(overlay !== 'floor' || !cf || !cf.cells || !cf.cells.length || !cliffOn){
    el.innerHTML = ''; return; }
  const R = cf.rows, C = cf.cols;
  if(el.dataset.k !== R+'x'+C){
    el.style.gridTemplateColumns = `repeat(${C},1fr)`;
    el.style.gridTemplateRows = `repeat(${R},1fr)`;
    el.dataset.k = R+'x'+C;
  }
  el.innerHTML = cf.cells.flat().map(v =>
    `<div class="${v===2?'cc2':v===1?'cc1':''}"></div>`).join('');
}
function snap(){
  fetch('/camera/snap', {method:'POST'}).then(r => r.json()).then(d => {
    $('camnote').textContent = d.ok ? ('Saved test/captures/' + d.file)
                                    : ('Could not save — ' + (d.error || 'no frame'));
  }).catch(() => { $('camnote').textContent = 'Could not reach the robot.'; });
}
$('lim').addEventListener('input', e => {
  const v = e.target.value/100; $('limv').textContent = v.toFixed(2); post('/limit',{value:v}); });
addEventListener('click', () => $('focusnote').classList.remove('show'), {once:true});


// ---------------------------------------------------------------- map
let autoOn = false;
let calTouched = 0;
// The scanner's mounting is now three ordinary tunables rendered into
// #tune-mount, so there is one code path for every parameter instead of a
// bespoke one for these three. The +-1 buttons remain because a degree at a
// time against a wall is genuinely how you converge on the last bit.
//
// poll() re-syncs from the server every 150 ms, so a change needs a hold-off
// or it can be reverted before its POST lands.
const CAL_HOLD_MS = 1500;
function nudgeYaw(step){
  const it = tuneItems.find(t => t.key === 'lidar_yaw');
  if(!it) return;
  calTouched = Date.now();
  const v = Math.max(-180, Math.min(180, it.value + step));
  it.value = v;
  LIDAR_YAW = v;                      // redraw instantly, then tell the server
  $('lyawv').textContent = v.toFixed(0) + '°';
  document.querySelectorAll('[data-v="lidar_yaw"]').forEach(nd =>
    nd.textContent = v.toFixed(0) + ' deg');
  document.querySelectorAll('[data-k="lidar_yaw"]').forEach(nd => nd.value = v);
  draw();
  tuneSet('lidar_yaw', v);
}

// --- measuring the mounting, instead of eyeballing it ----------------------
//
// Two presses with a push in between. The result carries a residual and a
// quality, so "is it right yet" has an answer rather than a feeling.
function pushStart(){
  fetch('/calibrate/push', {method:'POST', headers:{'Content-Type':'application/json'},
                            body: JSON.stringify({action:'start'})})
    .then(r => r.json()).then(d => {
      showCal(d.message || '', d.ok ? 'info' : 'warn');
      if(d.ok){ $('cal-start').style.display = 'none';
                $('cal-finish').style.display = ''; }
    }).catch(() => showCal('Could not reach the robot.', 'warn'));
}

function pushFinish(){
  fetch('/calibrate/push', {method:'POST', headers:{'Content-Type':'application/json'},
                            body: JSON.stringify({action:'finish'})})
    .then(r => r.json()).then(d => {
      $('cal-start').style.display = ''; $('cal-finish').style.display = 'none';
      let html = `<div>${d.message || ''}</div>`;
      if(d.quality !== undefined){
        html += `<div style="margin-top:6px">quality <b>${d.quality.toFixed(2)}</b>`
              + ` &middot; residual <b>${d.residual_mm.toFixed(0)} mm</b>`
              + ` &middot; ${d.bins} bearings</div>`;
      }
      if(d.ok){
        html += `<div class="tog" style="margin-top:9px">`
              + `<button onclick="pushApply('yaw')">Use ${d.nose_deg.toFixed(1)}&deg; rotation</button>`
              + (d.implied_cpr
                  ? `<button onclick="pushApply('cpr')">Use ${d.implied_cpr} counts/rev</button>` : '')
              + `</div>`;
      }
      showCal(html, d.ok ? 'info' : 'warn', true);
      fetchTune();
    }).catch(() => showCal('Could not reach the robot.', 'warn'));
}

function pushApply(what){
  fetch('/calibrate/push', {method:'POST', headers:{'Content-Type':'application/json'},
                            body: JSON.stringify({action:'apply', what:what})})
    .then(r => r.json()).then(() => { fetchTune(); calTouched = Date.now();
      showCal('Applied. Press <b>Save</b> on the Tune tab to keep it.', 'info', true); });
}

function showCal(msg, kind, html){
  const el = $('calresult');
  el.style.display = '';
  el.style.borderColor = kind === 'warn' ? 'var(--warning)' : 'var(--border)';
  if(html) el.innerHTML = msg; else el.textContent = msg;
}

// --- rooms (places) and objects ---------------------------------------------
// Names are text a person typed or SAID, so they never go into HTML or code
// unescaped: "Kid's room" used to break its own Go button. Buttons carry an
// index into the current list instead of the name.
const escHtml = s => String(s).replace(/[&<>"']/g,
  c => ({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
function postJson(u, b){
  return fetch(u, {method:'POST', headers:{'Content-Type':'application/json'},
                   body: JSON.stringify(b)}).then(r => r.json());
}

let placeNames = [];
let placesData = {};                 // {name: [x, y]} — drawn on the map as room names
function savePlace(){
  const n = $('pname').value.trim();
  if(n){ postJson('/places', {name: n}).then(refreshPlaces).catch(()=>{}); $('pname').value = ''; }
}
function goPlace(i){ if(placeNames[i] != null) goto({name: placeNames[i]}); }
function delPlace(i){
  if(placeNames[i] != null) postJson('/places', {name: placeNames[i], delete: true}).then(refreshPlaces).catch(()=>{});
}
function renderPlaces(d){
  const list = Object.keys(d || {});
  placeNames = list;
  $('places').innerHTML = list.length
    ? list.map((k, i) =>
        `<span class="place">${escHtml(k)}` +
        `<button class="b-auto" onclick="goPlace(${i})">Go</button>` +
        `<button class="x" title="forget this room" onclick="delPlace(${i})">×</button>` +
        `</span>`).join('')
    : '<span class="note">no rooms named yet</span>';
}
function refreshPlaces(){
  fetch('/places').then(r => r.json()).then(d => {
    placesData = d.places || {}; renderPlaces(placesData); drawMap();
  }).catch(() => {});
}
setInterval(refreshPlaces, 4000); refreshPlaces();

// Send the truck somewhere: {name} for a room, {x, y, name} for a point.
function goto(body){
  hideMapMenu();
  postJson('/goto', body).then(d => {
    $('m-goal').textContent = d.error || d.message || '';
  }).catch(() => {});
}

// Objects are saved automatically; this list is only for corrections.
let objKeys = [], objNames = [], lastObjHtml = '', editingObj = false;
function renderObjects(list){
  if(editingObj) return;             // do not yank the box out from under typing
  objKeys = list.map(o => o.key);
  objNames = list.map(o => o.label.replace(/_/g, ' '));
  $('o-count').textContent = list.length ? list.length + ' saved' : 'none yet';
  const html = list.length
    ? list.map((o, i) =>
        `<span class="place"><button class="linkish" title="rename" onclick="renameObj(${i})">`
        + `${escHtml(objNames[i])}</button>`
        + `<button class="x" title="wrong — remove it" onclick="removeObj(${i})">×</button></span>`).join('')
    : '<span class="note">none yet — drive around with Objects on (Vision tab)</span>';
  if(html !== lastObjHtml){ $('o-list').innerHTML = html; lastObjHtml = html; }
}
function removeObj(i){
  if(objKeys[i] == null) return;
  hideMapMenu();
  postJson('/objects', {key: objKeys[i], remove: true}).then(poll).catch(() => {});
}
function renameObj(i){
  const key = objKeys[i], chip = document.querySelectorAll('#o-list .place')[i];
  if(key == null || !chip) return;
  hideMapMenu();
  editingObj = true;
  chip.innerHTML = '<input type="text" maxlength="40" style="width:11em">';
  const inp = chip.querySelector('input');
  inp.value = objNames[i]; inp.focus(); inp.select();
  let finished = false;
  const done = save => {
    if(finished) return;
    finished = true; editingObj = false; lastObjHtml = '';
    const n = inp.value.trim();
    if(save && n && n !== objNames[i]) postJson('/objects', {key, name: n}).then(poll).catch(() => {});
    else renderObjects(objects);
  };
  inp.addEventListener('keydown', e => {
    if(e.key === 'Enter') done(true);
    if(e.key === 'Escape') done(false);
  });
  inp.addEventListener('blur', () => done(true));
}

// --- the map ----------------------------------------------------------------
//
// View: mv.s pixels per mm, centred on world (mv.cx, mv.cy); x right, y up.
//   screen x = w/2 + (x - cx)*s        screen y = h/2 - (y - cy)*s
// Until the person zooms or pans it re-fits to the explored area on every
// update, so a growing map never runs off the edge; once they have, it stays
// where they put it (Fit returns to auto).
//
// The server sends only the explored rectangle of the grid (x0, y0, w, h in
// cells); everything outside it is unknown and painted as such.

function toggleAuto(){ post('/explore', autoOn ? {stop:true} : {start:true}); }
let slamOn = true, matchOn = true, mapImg = null, trail = [], mapMeta = null;
let truckPose = null, exGoal = null, exPath = [];
let followInfo = null;                 // follow mode: the person's trail, last seen, search goal
let showPersonTrail = true;
try { showPersonTrail = localStorage.getItem('nav.personTrail') !== 'off'; } catch(e) {}
function togglePersonTrail(){
  showPersonTrail = !showPersonTrail;
  try { localStorage.setItem('nav.personTrail', showPersonTrail ? 'on' : 'off'); } catch(e) {}
  syncPersonChip(); drawMap();
}
function syncPersonChip(){ const c = $('m-person'); if(c) c.classList.toggle('sel', showPersonTrail); }
syncPersonChip();
const mapBuf = document.createElement('canvas');
const mv = {s: null, cx: 0, cy: 0, follow: false, user: false};

// byte -> grey: 128 unknown, below free (dark), above occupied (white)
const MAP_LUT = new Uint8Array(256);
for(let p = 0; p < 256; p++)
  MAP_LUT[p] = p === 128 ? 40 : (p < 128 ? 22 + (p/128)*14 : 90 + ((p-128)/127)*165);

function fetchMap(){
  fetch('/map').then(r => r.json()).then(d => {
    mapMeta = d; trail = d.trail || [];
    if(!truckPose) truckPose = d.pose;
    const raw = atob(d.data), w = d.w, h = d.h;
    mapBuf.width = w; mapBuf.height = h;
    const ctx = mapBuf.getContext('2d');
    const img = ctx.createImageData(w, h);
    for(let i = 0; i < raw.length; i++){
      const v = MAP_LUT[raw.charCodeAt(i)], j = i*4;
      img.data[j] = img.data[j+1] = img.data[j+2] = v; img.data[j+3] = 255;
    }
    ctx.putImageData(img, 0, 0);
    mapImg = true;
    drawMap();
  }).catch(() => {});
}

function mapSize(){
  const b = $('map').parentElement.getBoundingClientRect();
  return [b.width, b.height];
}
function fitView(w, h){
  const m = mapMeta, r = m.res;
  const x0 = (m.x0 - m.half)*r, y0 = (m.y0 - m.half)*r;
  const x1 = x0 + m.w*r, y1 = y0 + m.h*r;
  mv.s = Math.min(w/(x1 - x0), h/(y1 - y0)) * 0.95;
  mv.cx = (x0 + x1)/2; mv.cy = (y0 + y1)/2;
}
function mapFit(){ mv.user = false; mv.follow = false; syncFollowBtn(); drawMap(); }
function mapFollowToggle(){
  mv.follow = !mv.follow; mv.user = true; syncFollowBtn(); drawMap();
}
function syncFollowBtn(){ $('m-follow').classList.toggle('sel', mv.follow); syncPersonChip(); }
// Zoom about a screen point, default the centre, keeping what is under it still.
function mapZoomBy(f, sx, sy){
  const [w, h] = mapSize();
  if(!mapMeta || w < 2) return;
  if(mv.s == null) fitView(w, h);
  if(sx == null){ sx = w/2; sy = h/2; }
  const wx = mv.cx + (sx - w/2)/mv.s, wy = mv.cy - (sy - h/2)/mv.s;
  mv.s = Math.max(0.004, Math.min(2, mv.s*f));
  mv.cx = wx - (sx - w/2)/mv.s; mv.cy = wy + (sy - h/2)/mv.s;
  mv.user = true;
  drawMap();
}
function mapToWorld(sx, sy){
  const [w, h] = mapSize();
  return [mv.cx + (sx - w/2)/mv.s, mv.cy - (sy - h/2)/mv.s];
}

function drawMap(){
  if(tab !== 'map') return;
  const c = $('map');
  const [g, w, h] = fitCanvas(c);
  if(w < 2 || h < 2 || !mapMeta) return;
  if(mv.s == null || !mv.user) fitView(w, h);
  if(mv.follow && truckPose){ mv.cx = truckPose.x; mv.cy = truckPose.y; }
  const s = mv.s;
  const X = x => w/2 + (x - mv.cx)*s;
  const Y = y => h/2 - (y - mv.cy)*s;

  // Unknown everywhere first, then the explored rectangle on top of it.
  g.fillStyle = 'rgb(40,40,40)'; g.fillRect(0, 0, w, h);
  const m = mapMeta, r = m.res;
  g.imageSmoothingEnabled = false;
  if(mapImg){
    g.save();
    g.translate(X((m.x0 - m.half)*r), Y((m.y0 - m.half)*r));
    g.scale(s*r, -s*r);                 // grid row 0 is the lowest y: flip
    g.drawImage(mapBuf, 0, 0);
    g.restore();
  }

  if(trail.length > 1){
    g.strokeStyle = css('--imu'); g.lineWidth = 1.5; g.globalAlpha = .85;
    g.beginPath();
    trail.forEach(([x,y],i) => i ? g.lineTo(X(x),Y(y)) : g.moveTo(X(x),Y(y)));
    g.stroke(); g.globalAlpha = 1;
  }

  // The planned route and where it ends.
  if(exPath && exPath.length > 1){
    g.strokeStyle = css('--good'); g.lineWidth = 2; g.setLineDash([6, 5]);
    g.beginPath();
    exPath.forEach(([x,y],i) => i ? g.lineTo(X(x),Y(y)) : g.moveTo(X(x),Y(y)));
    g.stroke(); g.setLineDash([]);
  }
  if(exGoal){
    const gx = X(exGoal[0]), gy = Y(exGoal[1]);
    g.strokeStyle = css('--good'); g.lineWidth = 2;
    g.beginPath(); g.arc(gx, gy, 9, 0, 6.2832); g.stroke();
    g.beginPath(); g.arc(gx, gy, 3, 0, 6.2832); g.fillStyle = css('--good'); g.fill();
  }

  // Follow mode: where the person has walked, where they were last seen and,
  // while searching, where the truck is going to look for them.
  if(followInfo && showPersonTrail){
    const tr = followInfo.trail;
    g.strokeStyle = css('--audio'); g.lineWidth = 2.5; g.globalAlpha = .9;
    g.beginPath();
    tr.forEach(([x, y], i) => i ? g.lineTo(X(x), Y(y)) : g.moveTo(X(x), Y(y)));
    g.stroke(); g.globalAlpha = 1;
    if(followInfo.last_seen){
      const [lx, ly] = followInfo.last_seen;
      g.fillStyle = css('--audio');
      g.beginPath(); g.arc(X(lx), Y(ly), 6, 0, 6.2832); g.fill();
      g.font = '600 11px system-ui,sans-serif'; g.textAlign = 'center';
      g.fillText(followInfo.state === 'searching' ? 'last seen' : 'person', X(lx), Y(ly) - 10);
    }
    if(followInfo.search_goal && followInfo.state === 'searching'){
      const [sx, sy] = followInfo.search_goal;
      g.strokeStyle = css('--audio'); g.setLineDash([4, 4]); g.lineWidth = 2;
      g.beginPath(); g.arc(X(sx), Y(sy), 12, 0, 6.2832); g.stroke(); g.setLineDash([]);
      g.fillStyle = css('--audio'); g.fillText('searching here', X(sx), Y(sy) - 16);
    }
  }

  // Room names, large and underneath like a floor plan.
  g.font = '600 13px system-ui,sans-serif'; g.textAlign = 'center'; g.textBaseline = 'middle';
  g.fillStyle = css('--text-1') || '#ddd'; g.globalAlpha = .8;
  Object.entries(placesData).forEach(([name, [x, y]]) => g.fillText(name, X(x), Y(y)));
  g.globalAlpha = 1; g.textBaseline = 'alphabetic';

  // Known markers. Squares, because they are squares.
  g.fillStyle = css('--cam'); g.font = '8px ui-monospace,monospace';
  knownTags.forEach(t => {
    const x = X(t.x), y = Y(t.y);
    g.fillRect(x-3.5, y-3.5, 7, 7);
    g.fillText(t.id, x, y-6);
  });

  // Hit targets, recorded where they are drawn so a click and the picture
  // can never disagree about where something is.
  mapHit = [];
  g.font = '600 11px system-ui,sans-serif'; g.textAlign = 'center';
  objects.forEach((o, i) => {
    const x = X(o.x), y = Y(o.y);
    g.fillStyle = css('--obj');
    g.beginPath(); g.arc(x, y, 4.5, 0, 6.2832); g.fill();
    // Light text on a dark halo: the dot's green on the dark floor was unreadable.
    g.lineWidth = 3; g.strokeStyle = 'rgba(0,0,0,.75)'; g.fillStyle = '#e8f5ec';
    g.strokeText(o.label.replace(/_/g,' '), x, y - 9);
    g.fillText(o.label.replace(/_/g,' '), x, y - 9);
    mapHit.push({x, y, kind: 'obj', i, o});
  });
  g.globalAlpha = .9; g.fillStyle = css('--cam');
  shots.forEach(sh => {
    if(sh.x === undefined) return;
    const x = X(sh.x), y = Y(sh.y);
    g.beginPath(); g.arc(x, y, 2.6, 0, 6.2832); g.fill();
    mapHit.push({x, y, kind: 'shot', file: sh.file});
  });
  g.globalAlpha = 1;

  // The truck, from the fast /state poll rather than the 1 Hz map.
  const p = truckPose || m.pose;
  const px = X(p.x), py = Y(p.y), th = -p.deg*Math.PI/180;   // screen y is down
  g.fillStyle = css('--good');
  g.beginPath();
  g.moveTo(px + Math.cos(th)*11, py + Math.sin(th)*11);
  g.lineTo(px + Math.cos(th+2.5)*7, py + Math.sin(th+2.5)*7);
  g.lineTo(px + Math.cos(th-2.5)*7, py + Math.sin(th-2.5)*7);
  g.closePath(); g.fill();

  // Scale bar: the largest round length under ~90 px.
  const steps = [100, 200, 500, 1000, 2000, 5000, 10000];
  const len = steps.filter(v => v*s <= 90).pop() || steps[0];
  g.strokeStyle = css('--text-2'); g.fillStyle = css('--text-2'); g.lineWidth = 2;
  g.beginPath(); g.moveTo(12, h - 14); g.lineTo(12 + len*s, h - 14); g.stroke();
  g.font = '10px ui-monospace,monospace'; g.textAlign = 'left';
  g.fillText(len >= 1000 ? (len/1000) + ' m' : len + ' mm', 12, h - 20);
}

// --- map input: drag pans, wheel/pinch zooms, a click opens the menu ---------
(function mapInput(){
  const c = $('map');
  const ptrs = new Map();
  let moved = false, pinch0 = null;
  c.addEventListener('wheel', e => {
    e.preventDefault();
    mapZoomBy(e.deltaY < 0 ? 1.2 : 1/1.2, e.offsetX, e.offsetY);
  }, {passive: false});
  c.addEventListener('pointerdown', e => {
    hideMapMenu();
    ptrs.set(e.pointerId, {x: e.offsetX, y: e.offsetY});
    c.setPointerCapture(e.pointerId);
    moved = false;
    if(ptrs.size === 2){
      const [a, b] = [...ptrs.values()];
      pinch0 = {d: Math.hypot(a.x-b.x, a.y-b.y), mx: (a.x+b.x)/2, my: (a.y+b.y)/2};
    }
  });
  c.addEventListener('pointermove', e => {
    const last = ptrs.get(e.pointerId);
    if(!last || mv.s == null) return;
    const dx = e.offsetX - last.x, dy = e.offsetY - last.y;
    ptrs.set(e.pointerId, {x: e.offsetX, y: e.offsetY});
    if(ptrs.size === 2 && pinch0){
      const [a, b] = [...ptrs.values()];
      const d = Math.hypot(a.x-b.x, a.y-b.y);
      if(pinch0.d > 0) mapZoomBy(d/pinch0.d, pinch0.mx, pinch0.my);
      pinch0.d = d; moved = true;
      return;
    }
    if(!moved && Math.hypot(dx, dy) < 4) return;   // a shaky click is a click
    moved = true;
    mv.cx -= dx/mv.s; mv.cy += dy/mv.s;
    mv.user = true; mv.follow = false; syncFollowBtn();
    drawMap();
  });
  const up = e => {
    const had = ptrs.delete(e.pointerId);
    if(ptrs.size < 2) pinch0 = null;
    if(had && !moved && ptrs.size === 0 && e.type === 'pointerup') mapClick(e.offsetX, e.offsetY);
  };
  c.addEventListener('pointerup', up);
  c.addEventListener('pointercancel', up);
  c.addEventListener('dblclick', () => { hideMapMenu(); mapFit(); });
})();

function hideMapMenu(){ const mm_ = $('m-menu'); if(mm_) mm_.style.display = 'none'; }

// A click: a photo pin opens the photo; an object or an empty spot opens a
// small menu. Nothing moves until a menu item is chosen.
function mapClick(sx, sy){
  let best = null, bd = 12;
  mapHit.forEach(hh => {
    const d = Math.hypot(hh.x - sx, hh.y - sy);
    if(d < bd){ bd = d; best = hh; }
  });
  if(best && best.kind === 'shot'){ window.open('/captures/' + encodeURIComponent(best.file)); return; }

  const [wx, wy] = best ? [best.o.x, best.o.y] : mapToWorld(sx, sy);
  const p = truckPose || (mapMeta && mapMeta.pose) || {x: 0, y: 0};
  const dist = mm(Math.hypot(wx - p.x, wy - p.y));
  const menu = $('m-menu');
  if(best){
    const name = objNames[best.i] || best.o.label;
    menu.innerHTML =
      `<div class="mm-t">${escHtml(name)}</div>`
      + `<button data-a="go" class="b-auto">Go to it · ${dist}</button>`
      + `<button data-a="rename">Rename</button>`
      + `<button data-a="remove" class="b-stop">Wrong — remove</button>`
      + `<button data-a="close">Cancel</button>`;
  } else {
    menu.innerHTML =
      `<div class="mm-t">${(wx/1000).toFixed(1)}, ${(wy/1000).toFixed(1)} m</div>`
      + `<button data-a="go" class="b-auto">Go here · ${dist}</button>`
      + `<div class="mm-row"><input type="text" maxlength="40" placeholder="name this room">`
      + `<button data-a="name">Save</button></div>`
      + `<button data-a="close">Cancel</button>`;
  }
  const [w, h] = mapSize();
  menu.style.display = '';
  menu.style.left = Math.max(4, Math.min(w - menu.offsetWidth - 4, sx + 8)) + 'px';
  menu.style.top = Math.max(4, Math.min(h - menu.offsetHeight - 4, sy + 8)) + 'px';
  const inp = menu.querySelector('input');
  const nameIt = () => {
    const n = inp.value.trim();
    if(!n) return;
    postJson('/places', {name: n, x: wx, y: wy}).then(refreshPlaces).catch(() => {});
    hideMapMenu();
  };
  if(inp) inp.addEventListener('keydown', e => {
    if(e.key === 'Enter') nameIt();
    if(e.key === 'Escape') hideMapMenu();
  });
  menu.querySelectorAll('button').forEach(b => b.addEventListener('click', () => {
    const a = b.dataset.a;
    if(a === 'go') goto({x: wx, y: wy, name: best ? (objNames[best.i] || best.o.label) : ''});
    else if(a === 'name') nameIt();
    else if(a === 'rename'){ showTab('map'); renameObj(best.i); }
    else if(a === 'remove') removeObj(best.i);
    else hideMapMenu();
  }));
}

function fetchShots(){
  fetch('/captures').then(r => r.json()).then(d => {
    shots = d.captures || [];
    autoMm = d.auto_mm || 0;
    $('shotcount').textContent = shots.length ? shots.length + ' stills' : 'none yet';
    $('t-auto-mm').classList.toggle('g-on', autoMm > 0);
    // Newest first, and capped: the map keeps every pin, but a few hundred
    // thumbnails would be several megabytes of images on every refresh.
    $('shots').innerHTML = shots.slice(-48).reverse().map(sh => {
      const where = sh.x === undefined ? sh.why
        : (sh.x/1000).toFixed(1) + ', ' + (sh.y/1000).toFixed(1) + ' m';
      const f = encodeURIComponent(sh.file);
      return `<a href="/captures/${f}" target="_blank" title="${sh.file}">`
           + `<img src="/captures/${f}" loading="lazy" alt="">`
           + `<div class="lbl">${where}</div></a>`;
    }).join('');
  }).catch(() => {});
}
function toggleAutoShots(){ post('/captures/auto', {mm: autoMm > 0 ? 0 : 500})
                              .then(fetchShots); }
setInterval(fetchShots, 5000);
fetchShots();

setInterval(fetchMap, 1000);
fetchMap();
addEventListener('resize', drawMap);

function poll(){
  fetch('/state').then(r => r.json()).then(d => {
    const m = d.motors, l = d.lidar, gd = d.guard, im = d.imu;
    renderAudio(d.audio);
    renderLcd(d.display);
    renderTtsBrief(d.tts);
    renderAssistantCard(d.assistant);
    if(d.voice){
      // The Pi's IP, not location.hostname: a phone cannot open shiv.local.
      const url = d.voice.talk_url || `https://${location.hostname}:5443/talk`;
      $('t-phone').href = url;
      $('t-phone').textContent = d.voice.phone ? 'phone connected' : 'on the phone, open ' + url;
    }

    $('badge').className = 'badge ' + (m.enabled ? 'on':'off');
    $('b-arm').classList.toggle('armed', !!m.enabled);
    $('b-arm').textContent = m.enabled ? '● ENABLED' : 'ENABLE';
    $('b-arm').title = m.enabled ? 'Motors armed - STOP or E-STOP to halt' : 'Arm the motors';
    $('badge').innerHTML = '<i class="dot"></i>' + (m.enabled ? 'ENABLED':'DISABLED');
    // "Scanning" only while scans are actually arriving: an old scan still
    // has points, which is how a frozen scanner once looked perfectly healthy.
    const lok = l.connected && l.count && l.hz > 0;
    $('lbadge').className = 'badge ' + (lok ? 'on' : (l.connected ? 'warn' : 'off'));
    $('lbadge').innerHTML = '<i class="dot"></i>' + (lok ? 'SCANNING' : (l.connected ? 'LIDAR STALLED' : 'NO LIDAR'));
    $('lbadge').title = l.error || (l.age_s != null ? 'last scan ' + l.age_s + ' s ago' : '');
    $('trip').classList.toggle('show', !!m.tripped);

    const cl = gd.clear || {};
    clearance = cl;
    $('cl-fwd').textContent  = cl.fwd  != null ? mm(cl.fwd)  : (gd.enabled ? 'clear' : '—');
    $('cl-rev').textContent  = cl.rev  != null ? mm(cl.rev)  : (gd.enabled ? 'clear' : '—');
    $('cl-turn').textContent = cl.turn != null ? mm(cl.turn) : (gd.enabled ? 'clear' : '—');

    $('l-bad').textContent  = l.bad != null ? l.bad : '—';
    $('l-port').textContent = l.port || (l.connected ? 'connected' : 'none');
    $('s-lrpm').textContent = m.left_rpm.toFixed(1);
    $('s-rrpm').textContent = m.right_rpm.toFixed(1);

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
    if(document.activeElement !== $('lim')){
      $('lim').value = Math.round(m.limit*100); $('limv').textContent = m.limit.toFixed(2); }
    $('lim').max = Math.round(m.max_duty*100);

    pts = l.points; clusters = l.clusters || [];
    const gm = (d.guard && d.guard.geom) || {};
    const calIdle = Date.now() - calTouched > CAL_HOLD_MS;
    // The scanner's POSITION matters to the plot as much as its rotation: it
    // sits on a corner, and drawing its returns from the body centre is what
    // used to put the picture 250 mm away from what the guard was judging.
    // Set unconditionally — unlike the input fields below, these are only
    // ever written by the server, so there is no user edit to trample.
    if(gm.lx !== undefined){ LIDAR_X = gm.lx; LIDAR_Y = gm.ly; }
    if(gm.len !== undefined){ TRUCK.len = gm.len; TRUCK.wid = gm.wid; }
    if(gm.sector !== undefined) SECTOR_LIVE = gm.sector;
    if(gm.margin !== undefined) GUARD_MARGIN = gm.margin;
    if(gm.yaw !== undefined && calIdle){
      LIDAR_YAW = gm.yaw;
      $('lyawv').textContent = gm.yaw.toFixed(0)+'°';
    }
    $('hz').textContent = l.hz.toFixed(1)+' Hz';
    $('count').textContent = l.count;
    $('ahead').textContent = mm(l.ahead);
    $('objs').innerHTML = clusters.filter(c => c.near <= maxRange).map((c,i) =>
        `<tr><td><span class="idx">${i+1}</span></td><td class="n">${c.bearing.toFixed(0)}°</td>`
      + `<td class="n">${mm(c.near)}</td><td>${mm(c.width)}</td>`
      + `<td>${c.span.toFixed(0)}°</td><td>${c.points}</td></tr>`).join('');

    // --- IMU ---
    const live = im.present && im.ready;
    $('ibadge').className = 'badge ' + (live ? (im.heading_ok ? 'on':'warn') : 'off');
    $('ibadge').innerHTML = '<i class="dot"></i>' +
      (live ? (im.heading_ok ? 'IMU OK' : 'IMU — HDG DRIFT') : 'NO IMU');
    $('imuname').textContent = im.name || (im.error || 'not detected');

    if(live){
      yaw = im.yaw; roll = im.roll; pitch = im.pitch; headingOk = im.heading_ok;
      $('yaw').textContent = im.yaw.toFixed(0)+'°';
      $('rp').textContent = im.roll.toFixed(0)+'° / '+im.pitch.toFixed(0)+'°';
      $('s-roll').textContent = im.roll.toFixed(1)+'°';
      $('s-pitch').textContent = im.pitch.toFixed(1)+'°';
      const gz = im.gyro[2];
      $('s-gz').textContent = gz.toFixed(1)+' °/s';
      $('s-enc').textContent = m.enc_yaw_rate.toFixed(1)+' °/s';
      const slip = Math.abs(gz - m.enc_yaw_rate);
      const moving = Math.abs(m.left_rpm) + Math.abs(m.right_rpm) > 10;
      $('s-slip').textContent = moving ? slip.toFixed(1)+' °/s' : '—';
      $('s-slip').style.color = (moving && slip > 15) ? css('--warning') : '';
      $('s-temp').textContent = im.temp != null ? im.temp.toFixed(0)+' °C' : '—';

      const tilt = Math.max(Math.abs(im.roll), Math.abs(im.pitch));
      $('tilt').classList.toggle('show', tilt > TILT_WARN);
      $('tilt').textContent = '⚠ Tilted ' + tilt.toFixed(0) + '° — a skid-steer loses traction well before it tips.';

      $('imunote').textContent = im.heading_ok
        ? (im.fused ? 'Fused on-chip.' : 'Complementary filter: gravity sets roll/pitch, magnetometer sets heading.')
        : 'No magnetometer fix — heading is gyro-only and will drift. Run imu_test.py --calibrate.';
    } else {
      yaw = null;
      for(const id of ['yaw','rp','s-roll','s-pitch','s-gz','s-enc','s-slip','s-temp'])
        $(id).textContent = '—';
      $('tilt').classList.remove('show');
      $('imunote').textContent = im.error || 'No IMU detected. i2cdetect -y 1, then WIRING.md §11.';
    }
    const sl = d.slam;
    slamOn = sl.enabled; matchOn = sl.matching;
    $('t-slam').className = slamOn ? 'g-on' : '';
    $('t-slam').textContent = slamOn ? 'SLAM ON' : 'SLAM OFF';
    $('t-match').className = matchOn ? 'g-on' : '';
    $('t-match').textContent = matchOn ? 'Match ON' : 'Match OFF';
    $('px').textContent = mm(sl.pose.x);
    $('py').textContent = mm(sl.pose.y);
    $('pth').textContent = sl.pose.deg.toFixed(0)+'°';
    $('pdist').textContent = mm(sl.distance_mm);
    $('pscans').textContent = sl.scans;
    $('pcorr').textContent = sl.correction.x.toFixed(0)+', '+sl.correction.y.toFixed(0)
                           +' mm / '+sl.correction.deg.toFixed(1)+'°';
    $('pms').textContent = sl.ms.toFixed(1)+' ms';
    $('pcnt').textContent = sl.counts[0]+' / '+sl.counts[1];
    $('slamnote').textContent = (sl.imu_heading
      ? 'Heading from the IMU; scan matching correcting the rest.'
      : 'No IMU heading fix — heading is coming from wheel difference, which slip corrupts fast.')
      + (sl.map_note ? '  ·  ' + sl.map_note : '');
    truckPose = sl.pose;

    const ex = d.explore || {};
    autoOn = !!ex.running;
    $('b-auto').classList.toggle('on', autoOn);
    $('b-auto').textContent = autoOn ? 'STOP AUTO-MAP' : 'START AUTO-MAP';
    $('autostate').textContent = autoOn
      ? `${ex.state}${ex.goal ? ' → '+ex.goal : ''}  ·  `
        + `${ex.cal_stage && ex.cal_stage!=='done' ? ex.cal_stage : ex.message}`
        + `  ·  frontiers ${ex.frontiers}  path ${ex.path_len}  ${ex.elapsed}s`
      : (ex.message || 'idle');
    // The trip, on the map tab: where to, the route, and a way to call it off.
    const trip = ex.running && ex.state === 'goto';
    exGoal = trip ? ex.goal_xy : null;
    exPath = ex.running ? (ex.path_mm || []) : [];
    $('m-cancel').style.display = trip ? '' : 'none';
    $('m-goal').textContent = trip ? '→ ' + ex.goal + ' · ' + (ex.message || '')
      : (['done', 'failed'].includes(ex.state) ? ex.message : '');
    const cr = ex.cal_results || {}, cn = ex.cal_notes || [];
    if(Object.keys(cr).length || cn.length){
      $('calbox').style.display = 'block';
      $('calbox').innerHTML = 'calibration: '
        + Object.entries(cr).map(([k,v]) => `${k} <b>${v}</b>`).join(' &nbsp; ')
        + cn.map(n => `<div class="calnote">⚠ ${n}</div>`).join('');
    } else { $('calbox').style.display = 'none'; }

    // --- camera ---
    const cm = d.camera || {present:false};
    camLive = !!(cm.present && cm.live);
    $('cbadge').className = 'badge ' + (camLive ? 'on' : (cm.present ? 'warn':'off'));
    $('cbadge').innerHTML = '<i class="dot"></i>' +
      (camLive ? 'CAMERA' : (cm.present ? 'CAM STALLED' : 'NO CAM'));
    $('camname').textContent = cm.name || (cm.error || 'not detected');

    // Bearing drives the wedge on the plot; rotation only turns the picture.
    if(cm.yaw !== undefined) CAM_YAW = cm.yaw;
    const rot = ((cm.rotation || 0) % 360 + 360) % 360;
    const rr = $('camrot');
    if(rr && rr.dataset.rot !== String(rot)){
      rr.dataset.rot = String(rot);
      rr.className = 'camrot' + (rot ? ' r' + rot : '');
    }
    // A sideways camera makes the floor check measure the wrong direction:
    // it reads the raw frame and assumes the bottom row is the nearest
    // floor. Say so instead of letting it report confident nonsense.
    const rotBad = (rot === 90 || rot === 270) && cliffOn;
    $('cliffwarn').classList.toggle('show', rotBad);
    if(cm.present){
      $('c-size').textContent = cm.size ? cm.size.join('×') : '—';
      // Asked-for rate next to measured. They diverge when something else on
      // the Pi is eating the CPU, which is the fault this panel exists to
      // make visible before it turns into a frozen picture mid-drive.
      $('c-hz').textContent = cm.hz.toFixed(1) + ' / ' + cm.fps + ' fps';
      $('c-frames').textContent = cm.frames;
      $('c-enc').textContent = cm.encoder || '—';
      $('c-fov').textContent = cm.hfov.toFixed(0) + '°'
        + (cm.yaw ? ' @ ' + cm.yaw.toFixed(0) + '°' : '');
      if(!camLive)
        $('camnote').textContent = 'Camera opened but stopped delivering '
          + 'frames — usually a ribbon seated well enough to enumerate but '
          + 'not to stream.';
      else if($('camnote').textContent.startsWith('Camera opened'))
        $('camnote').textContent = '';
    } else {
      $('c-size').textContent = $('c-hz').textContent =
        $('c-frames').textContent = $('c-fov').textContent =
        $('c-enc').textContent = '—';
      $('camoff').textContent = cm.error
        || 'No CSI camera detected. Check the ribbon, then run '
           + 'python test/camera_test.py --list';
    }
    applyCam();

    // --- floor check ---
    const cf = d.cliff || {};
    cliffOn = !!cf.enabled;
    $('t-cliff').classList.toggle('g-on', cliffOn);
    const fbad = cliffOn && cf.blocked;
    $('fbadge').className = 'badge ' + (!cliffOn ? 'off' : (fbad ? 'warn':'on'));
    $('fbadge').innerHTML = '<i class="dot"></i>' +
      (!cliffOn ? 'FLOOR OFF' : (fbad ? (cf.cliff ? 'DROP-OFF':'OBSTACLE') : 'FLOOR OK'));
    $('cliffstate').textContent = cf.error ? cf.error
      : (!cliffOn ? 'off' : (cf.ready ? (cf.reason || 'clear') : 'learning'));
    $('f-near').textContent = cf.clear_mm != null ? mm(cf.clear_mm) : '—';
    $('f-geom').textContent = (cf.pitch != null)
      ? cf.pitch.toFixed(0)+'° down, '+cf.height.toFixed(0)+' mm up' : '—';
    $('f-ms').textContent = cf.ms != null ? cf.ms.toFixed(1)+' ms' : '—';
    $('cliffbanner').classList.remove('show');     // floor check removed
    $('cliffbanner').textContent = (cf.cliff ? '⚠ Drop-off — ' : '⚠ Low obstacle — ')
      + cf.reason + '. The LiDAR cannot see this; forward is vetoed.';
    drawCliff(cf);

    // --- markers ---
    const mk = d.markers || {};
    mkOn = !!mk.enabled; mkLearn = !!mk.learn;
    knownTags = mk.known || [];
    $('t-mk').classList.toggle('g-on', mkOn);
    $('t-learn').classList.toggle('on', mkLearn);
    const nseen = (mk.seen || []).length;
    $('mbadge').className = 'badge ' + (!mk.ok ? 'off' : (nseen ? 'on':'warn'));
    $('mbadge').innerHTML = '<i class="dot"></i>' +
      (!mk.ok ? 'NO ARUCO' : (nseen ? nseen+' TAG'+(nseen>1?'S':'') : 'TAGS 0'));
    $('tagcount').textContent = mk.ok
      ? (mk.tags||0) + ' known' + (mkLearn ? ' · learning' : '') : (mk.error||'—');
    // In view now, then the rest of what is on the map.
    const seenIds = new Set((mk.seen||[]).map(t => t.id));
    $('taglist').innerHTML = (mk.seen||[]).map(t =>
        `<span class="tag ${t.used?'fix':'on'}" title="${t.known?'known':'unknown'}">`
        + `${t.id} · ${mm(t.dist)} · ${t.bearing}°</span>`).join('')
      + knownTags.filter(t => !seenIds.has(t.id)).map(t =>
        `<span class="tag" title="seen ${t.n}x">${t.id}</span>`).join('');
    $('m-fix').textContent = mk.fixes != null ? mk.fixes : '—';
    $('m-rej').textContent = mk.rejected ? mk.rejected + ' ⚠' : (mk.ok ? '0' : '—');
    $('m-last').textContent = mk.last_fix
      ? mm(mk.last_fix.err) + ' from tag ' + mk.last_fix.tags.join(',') : '—';
    $('m-ms').textContent = mk.ms != null ? mk.ms.toFixed(1)+' ms' : '—';
    if(mk.last_reject) $('mknote').dataset.warn = mk.last_reject;

    // --- top bar and the Tune tab's live-effect panel ---
    // `sl` is the d.slam bound further up this callback; redeclaring it here
    // was a SyntaxError that took the whole page down with it.
    if(sl.pose){
      $('toppose').innerHTML =
        `<b>${(sl.pose.x/1000).toFixed(2)}, ${(sl.pose.y/1000).toFixed(2)}</b> m`
        + ` · <b>${sl.pose.deg.toFixed(0)}°</b>`;
    }
    if(sl.counts){
      $('s-cnt').textContent = sl.counts[0] + ' / ' + sl.counts[1];
    }
    $('pconf').textContent = sl.conf != null ? sl.conf.toFixed(2) : '—';
    if(sl.conf != null)
      $('pconf').style.color = sl.conf < 0.25 ? css('--warning') : '';
    $('prej').textContent  = sl.rejected != null
      ? sl.rejected + (sl.scans ? ' of ' + (sl.scans + sl.rejected) : '') : '—';
    $('ploop').textContent = sl.loops != null
      ? sl.loops + (sl.keys ? ' · ' + sl.keys + ' keyframes' : '') : '—';
    $('tu-ms').textContent    = sl.ms != null ? sl.ms.toFixed(1)+' ms' : '—';
    $('tu-corr').textContent  = sl.correction
      ? `${sl.correction.x.toFixed(0)}, ${sl.correction.y.toFixed(0)} mm · ${sl.correction.deg.toFixed(1)}°` : '—';
    $('tu-hz').textContent    = l.hz != null ? l.hz.toFixed(1)+' Hz' : '—';
    $('tu-pts').textContent   = l.count != null ? l.count : '—';
    $('tu-scans').textContent = sl.scans != null ? sl.scans : '—';
    $('tu-dist').textContent  = sl.distance_mm != null ? mm(sl.distance_mm) : '—';

    // A drop-off is worth noticing from any tab.
    $('pip-vision').classList.remove('show');

    // --- object labels ---
    const dt = d.detect || {};
    detOn = !!dt.enabled;
    objects = dt.objects || [];
    renderObjects(objects);
    drawMap();                  // truck, route and objects move faster than the 1 Hz grid
    $('t-detect').classList.toggle('g-on', detOn && dt.ok);
    $('objstate').textContent = !dt.ok ? (dt.error || 'not available')
      : (detOn ? (dt.committed + ' placed') : 'off');
    $('d-obj').textContent    = dt.committed != null ? dt.committed : '—';
    $('d-frames').textContent = dt.frames != null ? dt.frames : '—';
    $('d-skip').textContent   = dt.skipped != null ? dt.skipped : '—';
    $('d-ms').textContent     = dt.ms ? dt.ms.toFixed(0)+' ms' : '—';
    $('d-backend').textContent = dt.backend || '—';
    if(dt.backend) $('d-backend').style.color =
      dt.backend.indexOf('down') >= 0 ? css('--warning')
        : (dt.backend === 'remote' ? css('--good') : '');
    // Do not fight someone typing a URL into the box.
    if(document.activeElement !== $('d-url') && dt.url !== undefined
       && $('d-url').value !== dt.url) $('d-url').value = dt.url;
    // What is in view now, then what has been committed to the map.
    // People share the overlay with furniture, in their own colour.
    const pr = d.person || {};
    personOn = !!pr.enabled;
    personT = personOn ? pr.target : null;
    drawBoxes((dt.seen || []).concat(personBoxes(pr)));
    $('t-person').classList.toggle('g-on', personOn);
    const pt = pr.target, fmm = v => v != null ? mm(v) : '—';
    $('p-state').textContent = !personOn ? 'off'
      : pt ? (pr.people.length > 1 ? pr.people.length + ' people' : 'tracking')
      : 'looking…';
    $('p-dist').textContent  = pt && pt.distance_mm != null ? mm(pt.distance_mm) + ' · ' + pt.source : '—';
    $('p-bear').textContent  = pt ? (pt.bearing > 0 ? '+' : '') + pt.bearing.toFixed(1) + '°' : '—';
    $('p-lidar').textContent = pt ? fmm(pt.lidar_mm) + (pt.lidar_mm != null ? ' · ' + pt.lidar_points + ' pts' : '') : '—';
    $('p-floor').textContent = pt ? fmm(pt.floor_mm) : '—';
    $('p-size').textContent  = pt ? fmm(pt.size_mm) : '—';
    $('p-conf').textContent  = pt ? ((pt.conf*100)|0) + '% · ' + pr.people.length : '—';
    $('p-rate').textContent  = personOn ? (pr.hz || 0).toFixed(1) + ' Hz · ' + (pr.ms || 0).toFixed(0) + ' ms' : '—';
    $('p-backend').textContent = pr.backend || '—';
    $('p-err').textContent = pr.error || '';
    const fo = d.follow || {};
    followInfo = (fo.trail && fo.trail.length) ? fo : null;
    followOn = !!fo.running;
    $('t-follow').textContent = followOn ? 'Stop following' : 'Follow';
    $('t-follow').classList.toggle('g-on', followOn);
    $('f-state').textContent = fo.running ? fo.message
      : (fo.message ? fo.message : 'off');
    $('detlist').innerHTML = (dt.seen || []).map(o =>
        `<span class="tag ${o.placed?'fix':'on'}">${o.label} ${(o.conf*100)|0}%`
        + (o.range != null ? ' · '+mm(o.range) : ' · no range') + `</span>`).join('')
      + (dt.objects || []).slice(0,10).map(o =>
        `<span class="tag" title="${o.n} sightings${o.status === 'confirmed' ? ', confirmed' : ' — waiting for a yes or no (Map tab)'}">`
        + `${o.label}${o.status === 'confirmed' ? '' : '?'}</span>`).join('');

    drawCompass(); drawHorizon();
  }).catch(() => {
    $('badge').className = 'badge off';
    $('b-arm').classList.remove('armed'); $('b-arm').textContent = 'ENABLE';
    $('badge').innerHTML = '<i class="dot"></i>NO LINK';
    camLive = false; applyCam();      // tear the stream down with everything else
  });
}
setInterval(poll, 150);
setInterval(draw, 40);          // redraw faster than we poll, for smooth wheels
addEventListener('resize', () => { draw(); drawCompass(); drawHorizon(); });
poll();

// ---------------------------------------------------------------- tabs
//
// One panel is in the document at a time. That is not only tidiness: the
// canvases redraw at 25 Hz and the Pi is already spending ~100 ms of every
// 200 on scan matching, so drawing a plot nobody is looking at is CPU taken
// straight from SLAM.
//
// The camera is a single DOM node that gets MOVED between the Drive rail and
// the Vision viewport. Two .camwrap elements would be two <img> tags on
// /camera.mjpg, which is two MJPEG connections off one Pi for one picture.

let tab = 'drive';

// ---- System tab: task manager for the Pi and the PC --------------------
function sysEsc(x){ return String(x == null ? '' : x).replace(/[&<>"]/g, c => ({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;'})[c]); }
function sysBar(pct, max){
  const f = Math.max(0, Math.min(100, 100 * (pct || 0) / (max || 100)));
  const cls = f > 85 ? 'hi' : f > 60 ? 'mid' : '';
  return `<div class="sbar"><i class="${cls}" style="width:${f.toFixed(0)}%"></i></div>`;
}
function sysStat(k, v, cls){ return `<div class="stat"><span class="k">${sysEsc(k)}</span><span class="v num mono ${cls||''}">${v}</span></div>`; }
function sysMachine(st, label){
  if(!st || !st.host) return sysStat(label, 'no numbers (not Linux?)');
  const h = st.host;
  let o = sysStat(label + ' CPU', `${(h.cpu_pct||0).toFixed(0)} % of ${h.cores} cores` + (h.load1 != null ? ` · load ${h.load1}` : ''));
  o += sysBar(h.cpu_pct);
  if(h.per_core && h.per_core.length && h.per_core.length <= 16)
    o += '<div class="cores">' + h.per_core.map((c, i) => `<div>core ${i} · ${c.toFixed(0)}%${sysBar(c)}</div>`).join('') + '</div>';
  const mp = h.mem_total_mb ? 100 * h.mem_used_mb / h.mem_total_mb : 0;
  o += sysStat(label + ' RAM', `${(h.mem_used_mb/1024).toFixed(1)} / ${(h.mem_total_mb/1024).toFixed(1)} GB`) + sysBar(mp);
  if(h.temp_c != null) o += sysStat('Temperature', `${h.temp_c.toFixed(1)} °C`, h.temp_c > 80 ? 'bad' : '');
  return o;
}
function sysProc(st, label){
  const p = (st && st.proc) || {};
  return sysStat(label, `${(p.cpu_pct||0).toFixed(1)} % CPU · ${p.rss_mb != null ? p.rss_mb + ' MB' : '?'}`) + sysBar(p.cpu_pct);
}
async function sysPoll(){
  let d;
  try { d = await fetch('/sys').then(r => r.json()); } catch(e){ return; }
  if(!d || !d.pi){ $('sy-pi').innerHTML = sysStat('Pi', d && d.error ? sysEsc(d.error) : 'starting — first numbers in 2 s'); return; }
  const pi = d.pi;
  let o = sysMachine(pi, 'Pi');
  const th = pi.throttle;
  if(th) o += sysStat('Power', th.undervolt_now ? 'UNDER-VOLTAGE now' : th.throttled_now ? 'throttled now'
                       : th.undervolt_since_boot ? 'ok now · under-voltage since boot' : 'ok',
                       (th.undervolt_now || th.throttled_now) ? 'bad' : th.undervolt_since_boot ? '' : 'ok');
  o += sysProc(pi, 'web_nav.py (everything)');
  $('sy-pi').innerHTML = o;
  const per = 100 / ((pi.host && pi.host.cores) || 4);
  $('sy-threads').innerHTML = (pi.threads || []).filter(t => t.cpu_pct >= 0.1).slice(0, 14).map(t =>
    sysStat(t.name + (t.n > 1 ? ` ×${t.n}` : ''), `${t.cpu_pct.toFixed(1)} %`) + sysBar(t.cpu_pct, per)).join('')
    || sysStat('threads', 'all idle');
  const pc = d.pc || {};
  const det = pc.detect;
  if(!det){ $('sy-det').innerHTML = sysStat('server', 'none set (Vision tab → Objects → server URL)'); $('sy-det-note').textContent = ''; }
  else if(!det.ok){ $('sy-det').innerHTML = sysStat('server', 'UNREACHABLE', 'bad') + sysStat('error', sysEsc(det.error)); $('sy-det-note').textContent = det.url; }
  else {
    $('sy-det-note').textContent = `${det.url} · ${det.ping_ms} ms round trip`;
    let q = '';
    if(det.person_model) q += sysStat(`People · ${det.person_model}`, `${det.person_fps != null ? det.person_fps + ' /s' : '…'} · ${det.person_last_ms} ms each`);
    q += sysStat(`Objects · ${det.model} (${det.device})`, `${det.fps != null ? det.fps + ' /s' : '…'} · ${det.last_ms} ms each`);
    if(det.stats){ q += sysProc(det.stats, 'detect container'); q += sysMachine(det.stats, 'Docker VM'); }
    else q += sysStat('container CPU', 'rebuild the image to see it');
    $('sy-det').innerHTML = q;
  }
  const ol = pc.ollama;
  if(!ol){ $('sy-oll').innerHTML = sysStat('Ollama', 'assistant has no URL'); }
  else if(!ol.ok){ $('sy-oll').innerHTML = sysStat('Ollama', 'UNREACHABLE', 'bad') + sysStat('error', sysEsc(ol.error)); $('sy-oll-note').textContent = ol.url; }
  else {
    $('sy-oll-note').textContent = `${ol.url} · ${ol.ping_ms} ms`;
    $('sy-oll').innerHTML = (ol.models || []).map(m => {
      const g = m.size_mb ? Math.round(100 * m.vram_mb / m.size_mb) : 0;
      const left = m.expires ? Math.max(0, Math.round((Date.parse(m.expires) - Date.now()) / 60000)) : null;
      return sysStat(m.name, `${(m.size_mb/1024).toFixed(1)} GB · ${g}% on GPU` + (left != null ? ` · unloads in ${left} min` : ''), g < 100 ? '' : 'ok')
             + sysBar(g);
    }).join('') || sysStat('models', 'none loaded (loads on the first question)');
  }
  const ts = pc.tts;
  if(!ts){ $('sy-tts').innerHTML = sysStat('server', 'none — speech runs on the Pi'); }
  else if(!ts.ok){ $('sy-tts').innerHTML = sysStat('server', 'UNREACHABLE', 'bad') + sysStat('error', sysEsc(ts.error)); $('sy-tts-note').textContent = ts.url; }
  else {
    $('sy-tts-note').textContent = `${ts.url} · ${ts.ping_ms} ms`;
    $('sy-tts').innerHTML = sysStat('voices loaded', sysEsc((ts.voices || []).join(', ') || 'none'))
      + (ts.stats ? sysProc(ts.stats, 'tts container') : sysStat('container CPU', 'rebuild the image to see it'));
  }
}
setInterval(() => { if(tab === 'system') sysPoll(); }, 2000);

function showTab(name){
  tab = name;
  document.querySelectorAll('.tab').forEach(b =>
    b.classList.toggle('on', b.dataset.tab === name));
  document.querySelectorAll('.tabpanel').forEach(p =>
    p.classList.toggle('on', p.id === 'tab-' + name));

  const slot = $(name === 'vision' ? 'camslot-vision' : 'camslot-drive');
  const cam = $('camwrap');
  if(cam && slot && cam.parentElement !== slot) slot.appendChild(cam);

  try { localStorage.setItem('nav.tab', name); } catch(e) {}
  if(name === 'audio'){ audioPoll(); ttsPoll(); }
  if(name === 'sensors') refreshLcd();
  if(name === 'system') sysPoll();
  // The canvases were display:none a moment ago, so clientWidth was 0 and
  // any draw during that time was a no-op. Redraw now they have a size.
  requestAnimationFrame(() => { draw(); drawMap(); drawCompass(); drawHorizon(); });
}

try {
  const saved = localStorage.getItem('nav.tab');
  if(saved) showTab(saved);
} catch(e) {}

// Keep the tab strip under the top bar however tall the bar wraps to.
function fitTabs(){
  const tb = document.querySelector('.topbar');
  if(tb) document.documentElement.style.setProperty('--tabtop', tb.offsetHeight + 'px');
}
addEventListener('resize', fitTabs); fitTabs();

// ---------------------------------------------------------------- tuning
//
// The panel renders itself from /tuning. Adding a knob is one line in
// tuning.py and no change here at all - which is the point, because the last
// version of this page grew a bespoke control per feature and became the
// stack of cards this replaced.

let tuneItems = [], tuneTimer = null, tunePending = {};
// Fetched once at load rather than when the Tune tab opens, because
// most of the controls now live on the other tabs.
setTimeout(fetchTune, 250);

function fetchTune(){
  fetch('/tuning').then(r => r.json()).then(renderTune).catch(() => {
    $('tunemsg').textContent = 'could not reach the robot';
  });
}

// Where each group of knobs appears. THIS is the point of the registry:
// a parameter belongs next to the thing it changes, not in a list somewhere
// else. You cannot judge the scanner's rotation without the plot in front of
// you, or a guard margin without the clearances it produces - so the sliders
// go there, and the Tune tab is the complete index rather than the only way in.
const TUNE_SLOTS = [
  ['tunegrid',     null],                                    // everything
  ['tune-vehicle', ['Vehicle']],                             // Drive, by the plot
  ['tune-mount',   ['LiDAR mounting']],                      // Drive, by the plot
  ['tune-guard',   ['Collision guard']],                     // Drive, by the wedge
  ['tune-slam',    ['Scan matching','Loop closure','Occupancy grid','Mapping gate']], // Map
  ['tune-odom',    ['Odometry']],                            // Map, by the scale
  ['tune-expl',    ['Explorer']],                            // Map, by auto-map
  ['tune-cam',     ['Camera mounting']],                     // Vision, by the picture
  ['tune-vision',  ['Vision']],                              // Vision, by the grid
  ['tune-detect',  ['Object detection']],                    // Vision, by the labels
];

function renderTune(snap){
  tuneItems = snap.items || [];
  const d = (snap.dirty || []).length;
  const b = $('b-save');
  b.classList.toggle('dirty', d > 0);
  b.textContent = d ? ('SAVE ' + d) : 'SAVED';
  b.title = d
    ? d + ' value' + (d>1?'s':'') + ' would be lost on restart — click to write them to tuning.json on the Pi'
    : 'Everything matches tuning.json on the Pi';
  TUNE_SLOTS.forEach(([id, groups]) => {
    const el = $(id);
    if(el) el.innerHTML = tuneHTML(snap, groups);
  });
}

function tuneHTML(snap, only){
  const byGroup = {};
  (snap.items || []).forEach(it => (byGroup[it.group] = byGroup[it.group] || []).push(it));
  const groups = (snap.groups || []).filter(g => byGroup[g] && (!only || only.includes(g)));
  return groups.map(g => {
    const rows = byGroup[g].map(it => {
      const dirty = it.default !== null && it.value !== it.default;
      const shown = it.kind === 'bool' ? (it.value ? 'on' : 'off')
                                       : fmtTune(it.value, it);
      const ctl = it.kind === 'bool'
        ? `<div class="swrow">
             <button class="${it.value?'g-on':''}" onclick="tuneSet('${it.key}',${it.value?0:1})">${it.value?'On':'Off'}</button>
           </div>`
        : `<input type="range" min="${it.lo}" max="${it.hi}" step="${it.step}"
                  value="${it.value}" data-k="${it.key}"
                  oninput="tuneSlide(this)">`;
      // The value id is prefixed per slot: the same knob can be on screen
      // twice (inline and on the Tune tab) and duplicate ids would leave one
      // of them silently not updating.
      return `<div class="tunerow">
          <div class="tunehd">
            <span class="lb">${it.label}</span>
            <button class="q" onclick="this.closest('.tunerow').classList.toggle('open')">?</button>
            <span class="vv${dirty?' dirty':''}" data-v="${it.key}">${shown}${it.unit?' '+it.unit:''}</span>
          </div>
          ${ctl}
          <div class="doc">${it.doc || ''}${it.default !== null
              ? ` <b>Default ${fmtTune(it.default, it)}${it.unit?' '+it.unit:''}.</b>` : ''}</div>
        </div>`;
    }).join('');
    // Three renderings, because an inline strip is already inside a card and
    // nesting one in another looks like a mistake.
    if(!only) return `<div class="card"><h2>${g}</h2>${rows}</div>`;  // Tune tab
    if(only.length === 1) return rows;                                // bare
    return `<div class="tunesub">${g}</div>${rows}`;                  // subheading
  }).join('');
}

function fmtTune(v, it){
  if(it.kind === 'int' || it.step >= 1) return Math.round(v).toString();
  return (+v).toFixed(it.step >= 0.1 ? 2 : 3);
}

// Slider drag fires oninput continuously. Show the number immediately so it
// feels live, but batch the POSTs - a drag across a range is otherwise fifty
// requests, each of which rebuilds the scan-match window.
function tuneSlide(el){
  const key = el.dataset.k;
  // Mounting values are mirrored into the plot immediately and re-synced from
  // /state every 150 ms, so a drag needs the same hold-off the old bespoke
  // controls had or it fights the poll.
  if(key.startsWith('lidar_')){
    calTouched = Date.now();
    if(key === 'lidar_yaw') LIDAR_YAW = +el.value;
    if(key === 'lidar_x')   LIDAR_X   = +el.value;
    if(key === 'lidar_y')   LIDAR_Y   = +el.value;
    draw();
  }
  const it = tuneItems.find(t => t.key === key);
  if(it) document.querySelectorAll(`[data-v="${key}"]`).forEach(n =>
    n.textContent = fmtTune(el.value, it) + (it.unit ? ' '+it.unit : ''));
  tunePending[key] = +el.value;
  clearTimeout(tuneTimer);
  tuneTimer = setTimeout(flushTune, 180);
}

function tuneSet(key, v){ tunePending[key] = v; flushTune(); }

function flushTune(){
  const vals = tunePending; tunePending = {};
  if(!Object.keys(vals).length) return;
  fetch('/tuning', {method:'POST', headers:{'Content-Type':'application/json'},
                    body: JSON.stringify({values: vals})})
    .then(r => r.json())
    .then(d => { renderTune(d.snapshot);
                 $('tunemsg').textContent = 'applied — not saved yet'; })
    .catch(() => { $('tunemsg').textContent = 'could not reach the robot'; });
}

function tuneSave(){
  const b = $('b-save');
  b.textContent = 'SAVING';
  fetch('/tuning', {method:'POST', headers:{'Content-Type':'application/json'},
                    body: JSON.stringify({save:true})})
    .then(r => r.json()).then(d => {
      const n = Object.keys(d.saved || {}).length;
      const msg = n
        ? `saved ${n} value${n>1?'s':''} to tuning.json on the Pi`
        : 'nothing differs from the code defaults — file cleared';
      $('tunemsg').textContent = msg;
      // The button is in the sticky bar and the message is on the Tune tab,
      // so confirm on the button too - otherwise pressing Save from Drive
      // gives no feedback at all.
      renderTune(d.snapshot);
      b.textContent = 'SAVED';
      flash(msg);
    })
    .catch(() => { b.textContent = 'SAVE FAILED'; flash('Could not reach the robot.'); });
}

// A short-lived confirmation, for actions taken from a tab that cannot show
// the result. Reuses the focus banner slot rather than adding chrome.
function flash(msg){
  const el = $('focusnote');
  el.textContent = msg;
  el.classList.add('show');
  clearTimeout(flash._t);
  flash._t = setTimeout(() => el.classList.remove('show'), 3000);
}

function tuneRevert(){
  fetch('/tuning', {method:'POST', headers:{'Content-Type':'application/json'},
                    body: JSON.stringify({revert:true})})
    .then(r => r.json()).then(d => {
      $('tunemsg').textContent = 'back to code defaults, and saved';
      renderTune(d.snapshot);
    });
}

function tuneDocs(){
  const open = document.querySelectorAll('.tunerow.open').length;
  document.querySelectorAll('.tunerow').forEach(r => r.classList.toggle('open', !open));
}

// ---------------------------------------------------------------- audio
//
// Two feeds. /state (7 Hz) carries a small `audio` summary - enough for the
// now-playing lines and the progress bar - and costs the Pi no forks.
// /audio/state adds the library and device list, which run ffprobe and
// `aplay -l`, so it is only polled while the Audio tab is open.

let aBrief = null, aErr = '', aSetup = '', aLastErr = '';
let aSongSig = null, aDevSig = null, aTouched = 0, maxTone = 0.5;
const FREQS = [110, 220, 440, 1000, 4000, 10000];
const fmtT = s => s == null ? '—'
  : Math.floor(s / 60) + ':' + String(Math.floor(s % 60)).padStart(2, '0');

function apost(u, b){
  return fetch(u, {method:'POST', headers:{'Content-Type':'application/json'},
                   body: JSON.stringify(b || {})})
    .then(r => r.json().catch(() => ({})))
    .then(d => {
      aErr = (d && d.error) || '';
      if(d && 'playing' in d) renderAudio(d);
      if(tab === 'audio') audioPoll();
      showAErr();
    })
    .catch(() => { aErr = 'Request failed — is web_nav.py still running?'; showAErr(); });
}
function beep(kind){ apost('/audio/beep', {kind}); }

function showAErr(){
  const msg = aSetup || aErr || aLastErr;
  $('a-err').textContent = msg;
  $('d-err').textContent = aSetup ? 'Speaker not ready — see the Audio tab.' : (aErr || aLastErr);
  $('pip-audio').classList.toggle('show', !!msg);
}

function renderAudio(b){
  if(!b) return;
  aBrief = b;
  const song = b.track;
  let label;
  if(b.kind === 'beep')      label = song ? song + '  (held for beep)' : 'Beep: ' + b.now;
  else if(b.kind === 'speech') label = '🗣 “' + b.now + '”' + (song ? '  (song held)' : '');
  else if(b.kind === 'tone') label = b.now;
  else if(song)              label = b.paused ? song + '  — paused' : song;
  else                       label = 'Nothing playing';
  const going = b.kind === 'music';
  for(const id of ['d', 'a']){
    $(id + '-eq').classList.toggle('on', !!b.playing);
    $(id + '-pp').innerHTML = going ? '&#10074;&#10074;' : '&#9654;';
  }
  $('d-now').textContent = song || b.playing ? label : 'Idle';
  $('a-now').textContent = label;
  const pos = b.pos || 0, dur = b.dur;
  $('a-pos').textContent = song ? fmtT(pos) : '0:00';
  $('a-dur').textContent = song && dur ? fmtT(dur) : '—';
  $('a-bar').style.width = song && dur ? Math.min(100, pos / dur * 100) + '%' : '0';
  if(Date.now() - aTouched > 2000 && b.volume != null){
    $('a-vol').value = Math.round(b.volume * 100); fmtA();
  }
}

function audioPoll(){
  fetch('/audio/state').then(r => r.json()).then(d => {
    renderAudio(d);
    const devs = d.devices || [], files = d.files || [];
    $('a-card').textContent = d.card || 'no sound card';
    renderDevices(devs, d.device);
    renderSongs(files, d.track);
    $('a-count').textContent = files.length + (files.length === 1 ? ' song' : ' songs');
    $('a-limit').textContent = 'mp3 · wav · ogg · flac · m4a · aac — up to '
                             + d.max_upload_mb + ' MB each';
    document.querySelectorAll('#a-modes .chip').forEach(c =>
      c.classList.toggle('sel', c.dataset.mode === d.mode));
    maxTone = d.max_tone_level || 0.5;
    if(Date.now() - aTouched > 2000){
      $('a-bass').value = d.bass; $('a-treble').value = d.treble;
      $('a-blvl').value = Math.round(d.beep_level * 100);
      fmtA();
    }
    const miss = [];
    if(!d.have_aplay)  miss.push('alsa-utils');
    if(!d.have_ffmpeg) miss.push('ffmpeg');
    aSetup = miss.length
      ? 'Missing: ' + miss.join(' + ') + ' — sudo apt install -y ' + miss.join(' ')
      : !devs.length
        ? 'No ALSA playback device — the I2S overlay has not loaded. Check dtoverlay '
          + 'in /boot/firmware/config.txt (WIRING.md §7).'
        : '';
    aLastErr = d.last_error || '';
    showAErr();
  }).catch(() => {});
}
setInterval(() => { if(tab === 'audio') audioPoll(); }, 2000);

// Rebuilding a <select> on every poll fights the user mid-click, so only
// when the set of devices actually changes.
function renderDevices(devs, cur){
  const sig = devs.map(d => d.dev).join('|'), sel = $('a-dev');
  if(sig !== aDevSig){
    aDevSig = sig;
    sel.innerHTML = '<option value="">ALSA default</option>'
      + devs.map(d => `<option value="${d.dev}">${d.dev} — ${d.name}</option>`).join('');
  }
  if(document.activeElement !== sel) sel.value = cur || '';
}
$('a-dev').addEventListener('change', e => apost('/audio/device', {device: e.target.value}));

// Names are reduced to [A-Za-z0-9._-] on the Pi, so they interpolate safely.
// Same signature trick as the devices: a list rebuilt under the cursor
// swallows the click that was on its way to a button.
function renderSongs(files, cur){
  const sig = files.map(f => f.name + ':' + f.kb).join('|') + '#' + cur;
  if(sig === aSongSig) return;
  aSongSig = sig;
  $('a-songs').innerHTML = files.length ? files.map(f =>
      `<div class="song${f.name === cur ? ' cur' : ''}">`
    + `<button class="b-audio" onclick="apost('/audio/play',{file:'${f.name}'})" title="Play">&#9654;</button>`
    + `<span class="nm" title="${f.name}">${f.name}</span>`
    + `<span class="sz mono num">${f.dur ? fmtT(f.dur) + ' · ' : ''}${(f.kb / 1024).toFixed(1)} MB</span>`
    + `<button class="x" onclick="delSong(this,'${f.name}')" title="Delete">&times;</button>`
    + `</div>`).join('')
    : '<div class="empty">No songs yet — upload one above.</div>';
}

// Two clicks to delete, no dialog: a confirm() box blocks the page, and with
// it the polling that keeps the drive watchdog fed.
function delSong(el, name){
  if(el.classList.contains('arm')){ apost('/audio/delete', {file: name}); return; }
  el.classList.add('arm'); el.textContent = 'delete?';
  setTimeout(() => { el.classList.remove('arm'); el.innerHTML = '&times;'; }, 3000);
}

// XHR rather than fetch, for upload progress - an MP3 over the Pi's wifi
// takes long enough that a silent wait looks like a hang.
function uploadFiles(list){
  const files = [...list];
  let i = 0;
  const next = () => {
    if(i >= files.length){
      $('a-up').classList.remove('show'); aSongSig = null; audioPoll(); return;
    }
    const f = files[i++], fd = new FormData(), x = new XMLHttpRequest();
    fd.append('file', f);
    $('a-count').textContent = `uploading ${i} of ${files.length}…`;
    $('a-up').classList.add('show'); $('a-upbar').style.width = '0';
    x.upload.onprogress = e => {
      if(e.lengthComputable) $('a-upbar').style.width = (e.loaded / e.total * 100) + '%';
    };
    x.onload = () => {
      let d = {};
      try { d = JSON.parse(x.responseText); } catch(e) {}
      aErr = d.error ? f.name + ': ' + d.error : '';
      showAErr(); next();
    };
    x.onerror = () => {
      aErr = f.name + ': upload failed — larger than the size limit, or the Pi dropped off wifi';
      showAErr(); next();
    };
    x.open('POST', '/audio/upload');
    x.send(fd);
  };
  next();
}
$('a-file').addEventListener('change', e => { uploadFiles(e.target.files); e.target.value = ''; });
const aDrop = $('a-drop');
['dragenter', 'dragover'].forEach(ev => aDrop.addEventListener(ev, e => {
  e.preventDefault(); aDrop.classList.add('over'); }));
['dragleave', 'drop'].forEach(ev => aDrop.addEventListener(ev, e => {
  e.preventDefault(); aDrop.classList.remove('over'); }));
aDrop.addEventListener('drop', e => uploadFiles(e.dataTransfer.files));

$('a-prog').addEventListener('click', e => {
  if(!aBrief || !aBrief.track || !aBrief.dur) return;
  const r = e.currentTarget.getBoundingClientRect();
  apost('/audio/seek', {pos: (e.clientX - r.left) / r.width * aBrief.dur});
});
document.querySelectorAll('#a-modes .chip').forEach(c =>
  c.addEventListener('click', () => apost('/audio/mode', {mode: c.dataset.mode})));

function fmtA(){
  const db = v => (v > 0 ? '+' : '') + v + ' dB';
  $('a-volv').textContent = $('a-vol').value + '%';
  $('a-bassv').textContent = db(+$('a-bass').value);
  $('a-treblev').textContent = db(+$('a-treble').value);
  $('a-lvlv').textContent = $('a-lvl').value + '%';
  $('a-blvlv').textContent = $('a-blvl').value + '%';
}
['a-vol', 'a-bass', 'a-treble', 'a-lvl', 'a-blvl'].forEach(id =>
  $(id).addEventListener('input', () => { aTouched = Date.now(); fmtA(); }));
// 'change', not 'input': each one restarts the decoder, so only on release.
['a-vol', 'a-bass', 'a-treble'].forEach(id => $(id).addEventListener('change', () => {
  aTouched = Date.now();
  apost('/audio/levels', {volume: +$('a-vol').value / 100,
                          bass: +$('a-bass').value, treble: +$('a-treble').value});
}));
$('a-blvl').addEventListener('change', () => {
  aTouched = Date.now();
  apost('/audio/beep', {kind: 'beep', level: +$('a-blvl').value / 100});
});

$('a-freqs').innerHTML = FREQS.map(f =>
  `<button class="chip" data-f="${f}">${f >= 1000 ? f / 1000 + ' kHz' : f + ' Hz'}</button>`).join('');
function markFreq(){
  $('a-freqs').querySelectorAll('.chip').forEach(c =>
    c.classList.toggle('sel', +c.dataset.f === +$('a-freq').value));
}
$('a-freqs').querySelectorAll('.chip').forEach(c => c.addEventListener('click', () => {
  $('a-freq').value = c.dataset.f; markFreq(); }));
$('a-freq').addEventListener('input', markFreq);
const toneLevel = () => +$('a-lvl').value / 100 * maxTone;
function playTone(){
  apost('/audio/tone', {freq: +$('a-freq').value, seconds: +$('a-secs').value, level: toneLevel()});
}
function playSweep(){ apost('/audio/sweep', {f0: 40, f1: 15000, seconds: 8, level: toneLevel()}); }

// Keys typed into the Audio tab's own fields stay there: an arrow key on the
// volume slider must not also drive the robot. Space still e-stops.
document.querySelectorAll('#tab-audio input, #tab-audio select').forEach(el =>
  el.addEventListener('keydown', e => { if(e.code !== 'Space') e.stopPropagation(); }));

// H for horn. One blast per press - holding the key does not machine-gun it.
addEventListener('keydown', e => {
  if(e.code !== 'KeyH' || e.repeat || e.ctrlKey || e.metaKey || e.altKey) return;
  if(e.target.matches && e.target.matches('input[type=text], input[type=number], textarea, select')) return;
  e.preventDefault(); beep('horn');
});

markFreq(); fmtA(); audioPoll();

// ---------------------------------------------------------------- display
let lcdOn = true;
function renderLcd(st){
  if(!st) return;
  lcdOn = st.on;
  $('lcd-st').textContent = st.present ? (st.on ? 'on' : 'blank') : 'not connected';
  $('lcd-frames').textContent = st.frames;
  $('lcd-ms').textContent = st.present ? st.ms.toFixed(0) + ' ms at ' + st.fps + ' Hz' : '—';
  $('lcd-hz').textContent = st.spi_hz ? (st.spi_hz / 1e6).toFixed(2) + ' MHz' : '—';
  $('t-lcd').className = st.on ? 'g-on' : '';
  $('t-lcd').textContent = st.on ? 'Screen ON' : 'Screen OFF';
  $('lcd-err').textContent = st.error || '';
}
// A PNG every 2 s while the tab is open, never otherwise - the mirror is
// for checking, not something to spend the Pi's CPU on in the background.
function refreshLcd(){ $('lcd-img').src = '/display.png?t=' + Date.now(); }
setInterval(() => { if(tab === 'sensors') refreshLcd(); }, 2000);

// ---------------------------------------------------------------- assistant
let asOn = true;
const AS_STATUS = {listening: 'ready — listening to the phone', thinking: 'thinking…',
                   speaking: 'speaking', off: 'off', offline: 'PC not reachable', no_model: 'model not installed',
                   loading: 'loading model on the PC…', error: 'problem', starting: 'starting…'};
let asModelsSig = null;
function renderAssistantCard(a){
  if(!a) return;
  asOn = a.enabled;
  const st = a.status && a.status.startsWith('using ') ? a.status.replace('using ', 'using ') : (AS_STATUS[a.status] || a.status);
  $('as-status').textContent = a.enabled ? st : 'off';
  $('as-toggle').className = a.enabled ? 'g-on' : '';
  $('as-toggle').textContent = a.enabled ? 'Assistant ON' : 'Assistant OFF';
  $('as-heard').textContent = a.heard || '—';
  $('as-reply').textContent = a.reply || '—';
  $('as-latency').textContent = a.latency_ms != null ? (a.latency_ms / 1000).toFixed(1) + ' s' : '—';
  $('as-caps').textContent = (a.can_see ? 'sees' : 'no vision') + ' · ' + (a.can_use_tools ? 'tools' : 'no tools');
  if(document.activeElement !== $('as-url')) $('as-url').value = a.ollama_url || '';
  const models = (a.models || []).filter(m => !/embed/.test(m));
  if(a.model && !models.includes(a.model)) models.unshift(a.model);
  const sig = models.join('|');
  if(sig !== asModelsSig){
    asModelsSig = sig;
    $('as-model').replaceChildren(...models.map(m => new Option(m, m)));
  }
  if(document.activeElement !== $('as-model')) $('as-model').value = a.model || '';
  $('as-active').textContent = a.active_model || '—';
  renderPull(a.pull);
  $('as-err').textContent = a.error || '';
}

// Model download progress, straight from Ollama's pull stream on the PC.
let pullSeen = null;
function renderPull(p){
  const row = $('as-pullrow'), bar = $('as-pullbar');
  if(!p){ row.style.display = 'none'; bar.classList.remove('show'); return; }
  row.style.display = '';
  $('as-pullname').textContent = p.model;
  if(p.error){ $('as-pullpct').textContent = 'failed: ' + p.error; bar.classList.remove('show'); return; }
  if(p.done){ $('as-pullpct').textContent = 'done — pick it in Model and press Apply'; bar.classList.remove('show'); return; }
  bar.classList.add('show');
  const pct = p.total ? p.completed / p.total * 100 : 0;
  $('as-pullfill').style.width = pct.toFixed(1) + '%';
  // Speed and time left from how far it moved since the previous poll.
  const now = Date.now();
  let eta = '';
  if(pullSeen && pullSeen.model === p.model && p.total && now > pullSeen.t){
    const rate = (p.completed - pullSeen.completed) / ((now - pullSeen.t) / 1000);
    if(rate > 0){
      const left = (p.total - p.completed) / rate;
      eta = ` · ${(rate / 1e6).toFixed(1)} MB/s · ${left > 90 ? Math.round(left / 60) + ' min' : Math.round(left) + ' s'} left`;
    }
  }
  if(!pullSeen || now - pullSeen.t > 3000 || pullSeen.model !== p.model)
    pullSeen = {model: p.model, completed: p.completed, t: now};
  $('as-pullpct').textContent = p.total
    ? `${pct.toFixed(0)}% · ${(p.completed / 1e9).toFixed(2)} / ${(p.total / 1e9).toFixed(2)} GB${eta}`
    : p.status;
}

// ---------------------------------------------------------------- speech
//
// Typed text is data from a person, so everything here that shows it builds
// DOM nodes with textContent. innerHTML with a quote or a < in a sentence
// would break the page - or run it.

let tErr = '', tSetup = '', tVoiceSig = null, tHistSig = null, tTouched = 0;
const T_STATUS = {idle: 'ready', loading: 'loading voice…', synth: 'thinking…', speaking: 'speaking'};
// Speak must feel like a button, not a form: the request comes back at once
// and the status says what the truck is doing, so no busy lock on the box.

function tpost(u, b){
  return fetch(u, {method:'POST', headers:{'Content-Type':'application/json'},
                   body: JSON.stringify(b || {})})
    .then(r => r.json().catch(() => ({})))
    .then(d => {
      tErr = (d && d.error) || '';
      renderTtsBrief(d);
      if(tab === 'audio') ttsPoll();
      showTErr();
      return d;
    })
    .catch(() => { tErr = 'Request failed — is web_nav.py still running?'; showTErr(); return {}; });
}

function speakFrom(id){
  const el = $(id), text = el.value.trim();
  if(!text) return;
  tpost('/tts/say', {text}).then(d => {
    // The Drive box is for quick one-liners, so it empties; the Speak box
    // keeps the text, because the next thing is usually an edit of it.
    if(d && d.ok && id === 'd-say') el.value = '';
  });
}

function showTErr(){
  const msg = tSetup || tErr;
  $('t-err').textContent = msg;
  if(msg && !tSetup) $('d-err').textContent = msg;
}

function renderTtsBrief(b){
  if(!b || !('status' in b)) return;
  let st = T_STATUS[b.status] || b.status;
  if(b.status === 'speaking' && b.first_sound_ms != null)
    st += ` · sound in ${(b.first_sound_ms / 1000).toFixed(1)} s`;
  $('t-status').textContent = st;
  $('d-say').placeholder = b.status === 'idle' ? 'Say something…' : st;
  if(b.error && b.error !== tErr){ tErr = b.error; showTErr(); }
}

function ttsPoll(){
  fetch('/tts/state').then(r => r.json()).then(d => {
    renderTtsBrief(d);
    const installed = d.voices.filter(v => v.installed);
    if(document.activeElement !== $('t-url')) $('t-url').value = d.url || '';
    $('t-engine').textContent = !d.url ? '· Pi speaks'
      : d.engine === 'pc' ? '· PC speaks' : '· PC not answering, Pi speaks';
    $('t-engine').title = d.remote_error || '';
    tSetup = d.url ? ''
      : !d.have_piper
      ? 'Piper is not installed. On the Pi, in the venv:  pip install "piper-tts>=1.3"  — then restart web_nav.py.'
      : !installed.length ? 'No voice yet — open "Voices" below and press Download on one (about 60 MB).' : '';
    if(!installed.length) $('t-voicebox').open = true;
    renderVoiceSelect(installed, d.voice);
    renderVoices(d.voices, d.voice);
    renderHistory(d.history || []);
    if(Date.now() - tTouched > 2000){
      $('t-speed').value = Math.round(d.speed * 100);
      $('t-vol').value = Math.round(d.volume * 100);
      fmtT2();
    }
    showTErr();
  }).catch(() => {});
}
setInterval(() => { if(tab === 'audio') ttsPoll(); }, 2000);

function renderVoiceSelect(installed, cur){
  const sel = $('t-voice'), sig = installed.map(v => v.id).join('|');
  if(sig !== tVoiceSig){
    tVoiceSig = sig;
    sel.replaceChildren();
    if(!installed.length){
      sel.append(new Option('— download a voice below —', ''));
    }
    for(const v of installed) sel.append(new Option(`${v.id}  ·  ${v.label}`, v.id));
  }
  if(document.activeElement !== sel) sel.value = cur || '';
}
$('t-voice').addEventListener('change', e => { if(e.target.value) tpost('/tts/settings', {voice: e.target.value}); });

function renderVoices(voices, cur){
  const box = $('t-voices');
  box.replaceChildren(...voices.map(v => {
    const row = document.createElement('div'); row.className = 'vrow';
    const name = document.createElement('div'); name.className = 'vn';
    const b = document.createElement('b'); b.textContent = v.id;
    const sub = document.createElement('span');
    sub.textContent = v.label + (v.mb ? ` · ${v.mb} MB` : '');
    name.append(b, sub); row.append(name);
    const dl = v.download;
    if(dl && dl.pct < 100){
      const t = document.createElement('span'); t.className = 'mono num';
      t.textContent = dl.stage === 'model'
        ? `${dl.pct}% · ${(dl.got / 1e6).toFixed(1)} MB`
        : (dl.stage || '…');
      row.append(t);
    }else if(v.installed){
      const ok = document.createElement('span'); ok.className = 'ok';
      ok.textContent = v.id === cur ? '✓ in use' : '✓';
      const del = document.createElement('button'); del.textContent = 'Delete';
      del.onclick = () => {
        if(del.classList.contains('arm')){ tpost('/tts/delete', {voice: v.id}); return; }
        del.classList.add('arm'); del.textContent = 'sure?';
        setTimeout(() => { del.classList.remove('arm'); del.textContent = 'Delete'; }, 3000);
      };
      row.append(ok, del);
    }else{
      if(dl && dl.error){
        const e = document.createElement('span'); e.className = 'bad';
        e.textContent = dl.error; e.title = dl.error; row.append(e);
      }
      const get = document.createElement('button'); get.className = 'b-audio';
      get.textContent = dl && dl.error ? 'Retry' : 'Download';
      get.onclick = () => tpost('/tts/download', {voice: v.id});
      row.append(get);
    }
    return row;
  }));
}

function renderHistory(hist){
  const sig = hist.join('\u0000');
  if(sig === tHistSig) return;
  tHistSig = sig;
  $('t-history').replaceChildren(...hist.map(h => {
    const c = document.createElement('button');
    c.className = 'chip hist'; c.textContent = h; c.title = 'Say again: ' + h;
    c.onclick = () => { $('t-text').value = h; fmtT2(); tpost('/tts/say', {text: h}); };
    return c;
  }));
}

function fmtT2(){
  $('t-speedv').textContent = (+$('t-speed').value / 100).toFixed(2) + '×';
  $('t-volv').textContent = $('t-vol').value + '%';
  $('t-count').textContent = $('t-text').value.length + ' / 1000';
}
['t-speed', 't-vol'].forEach(id => {
  $(id).addEventListener('input', () => { tTouched = Date.now(); fmtT2(); });
  $(id).addEventListener('change', () => { tTouched = Date.now();
    tpost('/tts/settings', {speed: +$('t-speed').value / 100, volume: +$('t-vol').value / 100}); });
});
$('t-text').addEventListener('input', fmtT2);
$('t-text').addEventListener('keydown', e => {
  if(e.key === 'Enter' && (e.ctrlKey || e.metaKey)){ e.preventDefault(); speakFrom('t-text'); }
});
$('d-say').addEventListener('keydown', e => {
  if(e.key === 'Enter'){ e.preventDefault(); speakFrom('d-say'); }
});

// Typing must never drive the robot. The page-wide handler turns WASD and
// the arrows into motion and SPACE into an E-STOP, so without this every
// space in a sentence stopped the truck and every "a" steered it. Applies to
// every text field on the page, including the Places name box.
document.querySelectorAll('input[type=text], textarea').forEach(el =>
  el.addEventListener('keydown', e => e.stopPropagation()));

fmtT2(); ttsPoll();

</script>
</body>
</html>"""

PAGE = (PAGE.replace("%%LEN%%", str(TRUCK_LEN_MM))
            .replace("%%WID%%", str(TRUCK_WIDTH_MM))
            .replace("%%SECTOR%%", str(GUARD_SECTOR_DEG))
            .replace("%%MARGIN%%", str(SAFETY_MARGIN_MM))
            .replace("%%LX%%", str(LIDAR_OFFSET_X))
            .replace("%%LY%%", str(LIDAR_OFFSET_Y))
            .replace("%%IMUX%%", str(IMU_OFFSET_X))
            .replace("%%IMUY%%", str(IMU_OFFSET_Y))
            .replace("%%CAMX%%", str(CAM_OFFSET_X))
            .replace("%%CAMY%%", str(CAM_OFFSET_Y))
            .replace("%%HFOV%%", str(CAM_HFOV))
            .replace("%%CAMYAW%%", str(CAM_YAW_OFFSET))
            .replace("%%TILT%%", str(TILT_WARN_DEG))
            .replace("%%LYAW%%", str(LIDAR_YAW_OFFSET)))


@app.route("/")
def index():
    """The cockpit.

    Explicitly no-store: the page is a single self-contained document with no
    versioned filename, so a browser that caches it happily serves yesterday's
    UI after a sync and leaves you wondering why nothing changed. It has
    already cost one debugging session.
    """
    return Response(render_template_string(PAGE), mimetype="text/html",
                    headers={"Cache-Control": "no-store, must-revalidate"})


_last_survey = [0.0]


@app.route("/sys")
def sys_stats():
    """The System tab's numbers (SysMonitor). The first call after the tab
    opens starts the polling, so it may come back empty for one period."""
    return jsonify(sysmon.snapshot())


@app.route("/state")
def state():
    # The page polls this at ~7 Hz. Surveying sweeps ~200 points three times,
    # which is wasted work at that rate and steals CPU from the SLAM thread.
    now = time.monotonic()
    if now - _last_survey[0] > 0.3:
        _last_survey[0] = now
        guard.survey(lidar)
    pts = lidar.scan()
    return jsonify(
        motors=robot.state,
        lidar={
            "points": pts, "count": len(pts),
            "clusters": cluster_points(pts),
            "ahead": Lidar.sector_min(pts, 0, GUARD_SECTOR_DEG),
            "hz": round(lidar.hz(), 1), "bad": lidar.bad,
            "connected": lidar.connected, "error": lidar.error,
            "port": lidar.port, "stalls": getattr(lidar, "stalls", 0),
            "rx_bytes": getattr(lidar, "rx_bytes", 0), "packets": lidar.packets,
            "age_s": (round(time.monotonic() - lidar._scan_time, 1)
                      if lidar._scan_time else None),
        },
        imu=imu.state,
        guard={"enabled": guard.enabled, "stop_mm": guard.stop_mm,
               "blocked": guard.blocked, "reason": guard.reason,
               "creeping": guard.creeping, "clear": guard.clear,
               "geom": {"len": TRUCK_LENGTH_MM, "wid": TRUCK_WIDTH_MM,
                        "lx": lidar_cal.x, "ly": lidar_cal.y,
                        "yaw": lidar_cal.yaw,
                        "sector": GUARD_SECTOR_DEG,
                        "margin": SAFETY_MARGIN_MM}},
        slam=slam.state,
        explore=explorer.status,
        camera=dict(camera.state, rotation=CAM_ROTATION,
                    yaw=CAM_YAW_OFFSET),
        markers=markers.state,
        cliff=cliff.state,
        detect=detector.state,
        person=tracker.state,
        follow=follower.status if follower is not None else None,
        audio=speaker.brief if speaker else None,
        display=screen.state if screen else None,
        tts=talker.brief if talker else None,
        assistant=assistant.brief if assistant else None,
        voice=({"phone": ears.state["phone_connected"], "pending": ears.state["pending"],
                "talk_url": voice.talk_url()} if ears else None),
    )


@app.route("/map")
def map_():
    """The grid as base64 bytes plus the trail. Polled far slower than
    /state — 57k cells is ~76 kB of base64, cheap at 1 Hz, wasteful at 7."""
    g = slam.slam.grid
    payload = g.as_payload()
    payload["trail"] = [[round(x), round(y)] for x, y in slam.slam.trail[-600:]]
    payload["pose"] = slam.slam.pose.as_dict()
    return jsonify(payload)


@app.route("/snapshot")
def snapshot():
    """Everything worth knowing in one call, without the point cloud or the
    map bitmap. Small enough to read over curl while the robot is driving."""
    pts = lidar.scan()
    return jsonify(
        motors=robot.state, imu=imu.state, slam=slam.state,
        guard={"enabled": guard.enabled, "blocked": guard.blocked,
               "reason": guard.reason},
        explore=explorer.status,
        camera=camera.state,
        cliff={"blocked": cliff.blocked, "cliff": cliff.is_cliff,
               "reason": cliff.reason, "enabled": cliff.enabled},
        markers={"known": len(markers.map.tags), "fixes": markers.fixes,
                 "seen": [m["id"] for m in markers.seen]},
        lidar={"count": len(pts), "hz": round(lidar.hz(), 1), "bad": lidar.bad,
               "connected": lidar.connected,
               "ahead": Lidar.sector_min(pts, 0, GUARD_SECTOR_DEG),
               "nearest_objects": cluster_points(pts)[:5]},
    )


@app.route("/camera.mjpg")
def camera_mjpg():
    """The live view. One multipart response per client, held open for as
    long as the browser wants it.

    Flask must be threaded=True for this — it is, in main() — or this single
    never-ending response would block every other request and freeze the
    drive controls. That is the failure worth knowing about here.
    """
    if camera.cam is None:
        return Response(camera.error or "no camera", status=503,
                        mimetype="text/plain")
    from camera import MJPEG_MIME
    return Response(camera.stream(), mimetype=MJPEG_MIME)


@app.route("/camera/still.jpg")
def camera_still():
    """One frame, as a plain JPEG. For curl, and for anything that wants a
    picture without holding a stream open."""
    jpeg = camera.frame()
    if jpeg is None:
        return Response("no frame", status=503, mimetype="text/plain")
    return Response(jpeg, mimetype="image/jpeg",
                    headers={"Cache-Control": "no-store"})


@app.route("/camera/snap", methods=["POST"])
def camera_snap():
    """Save the current view to test/captures/, tagged with the pose."""
    path = camera.snap(slam.slam.pose)
    if path is None:
        return jsonify(ok=False, error=camera.error or "no frame"), 503
    return jsonify(ok=True, file=os.path.basename(path))


@app.route("/captures")
def captures_list():
    """Every still with the pose it was taken from. The map panel draws these
    as pins; clicking one opens the picture."""
    return jsonify(captures=camera.captures[-300:], auto_mm=camera.auto_mm)


@app.route("/captures/<path:name>")
def capture_file(name):
    """Serve one still.

    send_from_directory, not an open() on a joined path: `name` arrives from
    the network, and this is the one route here that turns a client-supplied
    string into a filename. It refuses anything that escapes the directory.
    """
    from flask import send_from_directory
    return send_from_directory(camera.CAPTURE_DIR, name,
                               mimetype="image/jpeg")


@app.route("/captures/auto", methods=["POST"])
def captures_auto():
    d = request.get_json(force=True, silent=True) or {}
    camera.auto_mm = max(0.0, float(d.get("mm", 0)))
    return jsonify(ok=True, auto_mm=camera.auto_mm)


@app.route("/markers", methods=["POST"])
def markers_ctl():
    d = request.get_json(force=True, silent=True) or {}
    if "enabled" in d:
        markers.enabled = bool(d["enabled"])
    if "learn" in d:
        markers.learn = bool(d["learn"])
    if d.get("forget") is not None:
        markers.map.forget(d["forget"] if d["forget"] != "all" else None)
        markers.map.save()
    if d.get("save"):
        markers.map.save()
    return jsonify(ok=True, tags=len(markers.map.tags),
                   enabled=markers.enabled, learn=markers.learn)


@app.route("/markers/sheet.svg")
def markers_sheet():
    """Printable tags, as SVG so they come off the printer at exactly
    MARKER_SIZE_MM. A scaled bitmap will not, and a tag whose real size
    differs from the constant puts every distance out by that ratio."""
    from markers import sheet_svg
    try:
        ids = [int(v) for v in request.args.get("ids", "0,1,2,3,4,5").split(",")]
        svg = sheet_svg(ids)
    except Exception as e:                                    # noqa: BLE001
        return Response(str(e), status=503, mimetype="text/plain")
    return Response(svg, mimetype="image/svg+xml",
                    headers={"Content-Disposition":
                             "inline; filename=aruco-sheet.svg"})


@app.route("/calibrate/push", methods=["POST"])
def calibrate_push():
    """Measure the scanner's nose bearing and the odometry scale from one
    straight push of the robot.

    Two calls: "start" grabs the before-scan, "finish" grabs the after-scan
    and fits. See calibrate.py for why this beats dragging a slider until the
    plot looks square.
    """
    d = request.get_json(force=True, silent=True) or {}
    act = d.get("action")
    if act == "start":
        pts = lidar.scan()
        if len(pts) < 40:
            return jsonify(ok=False, message="No LiDAR scan to start from."), 503
        nb = pusher.start(pts, [robot.enc_left.steps, robot.enc_right.steps],
                          time.monotonic())
        return jsonify(ok=True, armed=True, points=nb,
                       message="Captured. Now push the robot straight forward "
                               "about half a metre, then press Finish.")
    if act == "finish":
        res = pusher.finish(lidar.scan(),
                            [robot.enc_left.steps, robot.enc_right.steps],
                            slam.slam.odom.mm_per_count,
                            lidar_cal.yaw, slam.slam.odom.cpr)
        return jsonify(**res)
    if act == "apply":
        last = pusher.last or {}
        done = {}
        if d.get("what") in ("yaw", "both") and last.get("nose_deg") is not None:
            done.update(tuning.apply({"lidar_yaw": last["nose_deg"]}))
        if d.get("what") in ("cpr", "both") and last.get("implied_cpr"):
            done.update(tuning.apply({"counts_per_rev": last["implied_cpr"]}))
        return jsonify(ok=bool(done), applied=done)
    return jsonify(ok=False, message="unknown action"), 400


@app.route("/calibrate/push")
def calibrate_push_state():
    return jsonify(armed=pusher.armed, last=pusher.last)


@app.route("/tuning")
def tuning_get():
    """The whole registry with live values. The Tune tab renders itself from
    this, so a tunable added to tuning.py needs no UI change at all."""
    return jsonify(tuning.snapshot())


@app.route("/tuning", methods=["POST"])
def tuning_set():
    """Apply values and save them to the SD card automatically.

    It used to apply live and save only on the SAVE button, on the theory
    that most tries are worse than what you had. In practice the button was
    missed, and truck size and LiDAR mounting were lost on every restart and
    re-tuned by hand. Now every change is kept; Revert still goes back to
    the code defaults. Saved 1.5 s after the last change, so dragging a
    slider is one write to the card, not fifty.
    """
    d = request.get_json(force=True, silent=True) or {}
    done = {}
    if d.get("revert"):
        done = tuning.revert()
    else:
        done = tuning.apply(d.get("values") or {})
    saved = None
    if d.get("save") or d.get("revert"):
        saved = tuning.save()
    elif done:
        _autosave_tuning()
    return jsonify(ok=True, applied=done, saved=saved,
                   snapshot=tuning.snapshot())


_tune_timer = [None]


def _autosave_tuning(delay=1.5):
    """Save tuning.json once the changes stop for `delay` seconds."""
    if _tune_timer[0] is not None:
        _tune_timer[0].cancel()

    def run():
        try:
            tuning.save()
        except OSError as e:
            print(f"  tuning autosave failed: {e}")
    _tune_timer[0] = threading.Timer(delay, run)
    _tune_timer[0].daemon = True
    _tune_timer[0].start()


# A named place counts as "the room the truck is in" within this distance.
# Rooms are points, not regions — there is no room segmentation — so this is
# roughly half a small room's width.
ROOM_RADIUS_MM = 3000.0


def room_at(x, y, within_mm=4000.0):
    """The named place nearest to a point — rooms are places the person has
    named, so "the sofa in the living room" is the nearest name."""
    best, bd = None, within_mm
    for name, (px, py) in load_places().items():
        d = math.hypot(px - x, py - y)
        if d < bd:
            best, bd = name, d
    return best


@app.route("/labels")
def labels_list():
    """Detections waiting for a person's yes or no, oldest-asked first."""
    pend = detector.map.pending()
    for p in pend:
        p["room"] = room_at(p["x"], p["y"])
    return jsonify(pending=pend, rotation=CAM_ROTATION)


@app.route("/labels/answer", methods=["POST"])
def labels_answer():
    """{key, answer: yes|no|skip, name?}. yes + name relabels ("it's a bed")."""
    d = request.get_json(force=True, silent=True) or {}
    key = d.get("key") or ""
    c = detector.map.cells.get(key)
    if c is None:
        return jsonify(error=f"no candidate {key!r}"), 404
    try:
        detector.map.answer(key, d.get("answer"), d.get("name"), room_at(c["x"], c["y"]))
    except ValueError as e:
        return jsonify(error=str(e)), 400
    detector.map.save()
    return jsonify(ok=True, cell=c, pending=len(detector.map.pending()))


@app.route("/objects", methods=["POST"])
def objects_ctl():
    """{key, remove: true} drops a wrong object for good; {key, name} renames
    one ("that's Dad's chair"). Objects are saved automatically, so this is
    the only correction a person ever has to make."""
    d = request.get_json(force=True, silent=True) or {}
    key = d.get("key") or ""
    if key not in detector.map.cells:
        return jsonify(error=f"no object {key!r}"), 404
    if d.get("remove"):
        detector.map.remove(key)
    elif (d.get("name") or "").strip():
        detector.map.cells[key]["name"] = d["name"].strip()[:40]
        detector.map.cells[key]["renamed"] = True     # a person's name wins over votes
    detector.map.save()
    return jsonify(ok=True, objects=detector.map.committed())


@app.route("/labels/asked", methods=["POST"])
def labels_asked():
    """The voice assistant has just asked about this one: it goes to the back
    of the queue, so an unanswered question is not repeated straight away."""
    detector.map.mark_asked((request.get_json(force=True, silent=True) or {}).get("key", ""))
    return jsonify(ok=True)


@app.route("/labels/photo/<path:name>")
def labels_photo(name):
    from flask import send_from_directory
    import detect as _detect                                  # noqa: PLC0415
    return send_from_directory(_detect.ASK_DIR, name, mimetype="image/jpeg")


@app.route("/follow", methods=["POST"])
def follow_ctl():
    """{start: true} locks on to the nearest person in view and follows them;
    {stop: true} ends it. {enable: true} (the cockpit's button, pressed by a
    person) also arms the motors; the AI's tool never sends it."""
    d = request.get_json(force=True, silent=True) or {}
    if d.get("stop"):
        follower.stop()
    elif d.get("start"):
        if d.get("enable"):
            robot.enable()
        if not robot.state.get("enabled"):
            return jsonify(error="motors are disabled - press ENABLE", **follower.status), 409
        if explorer.running:
            explorer.stop("follow mode")
        follower.start()
    return jsonify(follower.status)


@app.route("/person", methods=["POST"])
def person_ctl():
    """Person tracker on/off. Measuring only — nothing here drives."""
    d = request.get_json(force=True, silent=True) or {}
    if "enabled" in d:
        tracker.enabled = bool(d["enabled"])
    return jsonify(tracker.state)


@app.route("/detect", methods=["POST"])
def detect_ctl():
    d = request.get_json(force=True, silent=True) or {}
    if "enabled" in d:
        detector.enabled = bool(d["enabled"])
    if d.get("forget") is not None:
        detector.map.forget(None if d["forget"] == "all" else d["forget"])
        detector.map.save()
    if d.get("save"):
        detector.map.save()
    if "url" in d:
        detector.set_url(d["url"])
    return jsonify(ok=True, enabled=detector.enabled, url=detector.url,
                   backend=detector.backend,
                   committed=len(detector.map.committed()))


@app.route("/cliff", methods=["POST"])
def cliff_ctl():
    """The floor check was removed; this stays so an old page gets an answer."""
    return jsonify(ok=True, enabled=False, removed=True)


@app.route("/slam", methods=["POST"])
def slam_ctl():
    d = request.get_json(force=True, silent=True) or {}
    if d.get("reset"):
        explorer.stop("map reset")          # its path is in the old frame
        if follower is not None:
            follower.forget("map reset")    # the person's trail is in the old frame too
        slam.slam.reset()
        # Everything else pinned in the old map's coordinates goes with it:
        # objects and rooms would float over walls that no longer exist.
        detector.map.forget()
        detector.map.save()
        save_places({})
        # And the autosave, or the next boot would resume the old map.
        try:
            os.remove(MAP_FILE)
        except OSError:
            pass
        slam._saved_scans = 0
        slam.map_note = "map reset · objects and rooms cleared"
    if "enabled" in d:
        slam.enabled = bool(d["enabled"])
    if "matching" in d:
        slam.slam.match_enabled = bool(d["matching"])
    return jsonify(slam.state)


@app.route("/lidar", methods=["POST"])
def lidar_ctl():
    d = request.get_json(force=True, silent=True) or {}
    lidar_cal.set(d.get("yaw"), d.get("x"), d.get("y"))
    return jsonify(lidar_cal.as_dict)


@app.route("/places", methods=["GET", "POST"])
def places_ctl():
    """List, save or delete named places. Saving uses the CURRENT pose."""
    places = load_places()
    if request.method == "POST":
        d = request.get_json(force=True, silent=True) or {}
        name = (d.get("name") or "").strip()[:40]
        if d.get("delete") and name:
            places.pop(name, None)
            save_places(places)
        elif name:
            p = slam.slam.pose
            places[name] = [round(d.get("x", p.x), 1), round(d.get("y", p.y), 1)]
            save_places(places)
    return jsonify(places=places, pose=slam.slam.pose.as_dict())


@app.route("/goto", methods=["POST"])
def goto_ctl():
    d = request.get_json(force=True, silent=True) or {}
    if d.get("stop"):
        explorer.stop()
        return jsonify(explorer.status)
    name = (d.get("name") or "").strip()
    places = load_places()
    if name and name in places:
        x, y = places[name]
    elif "x" in d and "y" in d:
        x, y, name = float(d["x"]), float(d["y"]), name or "a point"
    else:
        return jsonify(error="unknown place: %r" % name), 400
    if follower is not None and follower.running:
        follower.stop("sent somewhere else")
    robot.enable()
    explorer.goto(x, y, name)
    return jsonify(explorer.status)


@app.route("/map/save", methods=["POST"])
def map_save():
    try:
        n = slam.slam.save(MAP_FILE)
    except OSError as e:
        return jsonify(error=str(e)), 400
    return jsonify(saved=MAP_FILE, bytes=n, scans=slam.slam.scans)


@app.route("/map/load", methods=["POST"])
def map_load():
    """Load a saved map. The explorer is stopped first: resuming teleports the
    pose, and a path planned in the old frame is nonsense in the new one."""
    explorer.stop("map reloaded")
    if follower is not None:
        follower.forget("map reloaded")
    try:
        blob = slam.slam.load(MAP_FILE)
    except (OSError, ValueError) as e:
        return jsonify(error=str(e)), 400
    return jsonify(loaded=MAP_FILE, scans=blob.get("scans"),
                   pose=slam.slam.pose.as_dict())


@app.route("/explore", methods=["POST"])
def explore_ctl():
    """Start or stop autonomous mapping.

    Starting enables the drivers: an explorer that cannot raise STBY would
    plan beautifully and never move, which looks like a planner bug.
    """
    d = request.get_json(force=True, silent=True) or {}
    if d.get("stop"):
        explorer.stop()
    elif d.get("start"):
        if follower is not None and follower.running:
            follower.stop("mapping started")
        robot.enable()
        explorer.start(calibrate=bool(d.get("calibrate", True)))
    return jsonify(explorer.status)


@app.route("/drive", methods=["POST"])
def drive():
    d = request.get_json(force=True, silent=True) or {}
    # A manual command wins. Two controllers writing to the same motors at
    # once is how a robot ends up doing neither thing.
    if (d.get("throttle") or d.get("steer")):
        if follower is not None and follower.running:
            follower.stop("manual override")
        if explorer.running:
            explorer.stop("manual override")
    # Record and return. No guard evaluation, no GPIO, nothing that can block
    # behind the SLAM thread - the control loop picks this up within 20 ms.
    intent.set(d.get("throttle", 0), d.get("steer", 0), "manual")
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
    if follower is not None and follower.running:
        follower.stop("emergency stop")
    if explorer.running:
        explorer.stop("emergency stop")
    intent.set(0, 0, "stop")
    robot.estop()
    return "", 204


@app.route("/stop", methods=["POST"])
def stop():
    # STOP means everything that drives: an explorer or follower left running
    # would simply command the motors again on its next tick.
    if follower is not None and follower.running:
        follower.stop("stopped")
    if explorer.running:
        explorer.stop("stopped")
    intent.set(0, 0, "stop")
    robot.stop()
    return "", 204


@app.route("/display", methods=["POST"])
def display_ctl():
    d = request.get_json(force=True, silent=True) or {}
    if screen is None:
        return jsonify(error="display not started"), 400
    try:
        if "on" in d:
            screen.set_power(d["on"])
        if d.get("test"):
            screen.test()
    except Exception as e:                                    # noqa: BLE001
        return jsonify(error=str(e)), 400
    return jsonify(screen.state)


@app.route("/display.png")
def display_png():
    """What the truck's screen is showing. Rendered even with no panel
    attached, so the layout can be judged before the display is wired."""
    if screen is None:
        return "", 404
    return Response(screen.png(), mimetype="image/png",
                    headers={"Cache-Control": "no-store"})


# 0 deg = straight ahead, + = left, in 45 deg wedges.
AI_DIRECTIONS = ("ahead", "ahead-left", "left", "behind-left",
                 "behind", "behind-right", "right", "ahead-right")


@app.route("/ai/status")
def ai_status():
    """What an AI driver needs to decide a move, and nothing it does not.

    /state is shaped for drawing — hundreds of raw LiDAR points in the
    scanner's own frame. This is shaped for deciding: the nearest obstacle
    in each of eight directions, measured in the BODY frame from the truck's
    centre (so subtract half the length or width for the gap at the
    bumper), plus exactly what the guard would refuse right now."""
    guard.survey(lidar)
    near = dict.fromkeys(AI_DIRECTIONS)
    for x, y in body_points(lidar.scan()):
        i = int(((math.degrees(math.atan2(y, x)) + 22.5) % 360) // 45)
        d = math.hypot(x, y)
        if near[AI_DIRECTIONS[i]] is None or d < near[AI_DIRECTIONS[i]]:
            near[AI_DIRECTIONS[i]] = d
    m, im, ex = robot.state, imu.state, explorer.status
    return jsonify(
        motors={"enabled": m["enabled"], "watchdog_tripped": m["tripped"],
                "speed_limit": m["limit"]},
        pose_mm_deg=slam.slam.pose.as_dict(),
        compass_deg=im.get("yaw") if im.get("ready") else None,
        body_mm={"length": TRUCK_LENGTH_MM, "width": TRUCK_WIDTH_MM},
        lidar={"connected": lidar.connected, "hz": round(lidar.hz(), 1),
               "nearest_mm_from_centre": {k: (round(v) if v is not None else None)
                                          for k, v in near.items()}},
        guard={"enabled": guard.enabled, "blocked": guard.blocked,
               "reason": guard.reason, "stop_mm": guard.stop_mm,
               "clearance_mm": guard.clear},
        floor_check={"enabled": cliff.enabled, "reason": cliff.reason},
        camera_live=bool(camera.state.get("live")),
        follow=({k: follower.status[k] for k in ("running", "state", "message", "distance_mm")}
                if follower is not None else None),
        navigation={"running": ex.get("running"), "state": ex.get("state"),
                    "message": ex.get("message"), "goal": ex.get("goal")},
        places=sorted(load_places()),
        # For the assistant's questions while mapping: which named room the
        # truck is in (None = somewhere unnamed), and labels awaiting a yes/no.
        room_here=room_at(slam.slam.pose.x, slam.slam.pose.y, ROOM_RADIUS_MM),
        label_questions=len(detector.map.pending()),
        audio=speaker.brief if speaker else None,
        speech=talker.brief if talker else None,
        microphone=({k: v for k, v in ears.state.items() if k != "log"} if ears else None),
    )


def lcd_snapshot():
    """What the status screen shows, as plain values.

    Each sensor in its own try: the screen is how you find out something is
    wrong, so one broken reader must not blank the rest of it."""
    ip = lan_ip()
    s = {"ip": None if ip.startswith("127.") else ip, "port": http_port}

    def part(fn):
        try:
            fn()
        except Exception:                                     # noqa: BLE001
            pass

    def motors():
        m = robot.state
        s.update(enabled=m.get("enabled"), tripped=m.get("tripped"))

    def sensors():
        hz = lidar.hz()
        s.update(lidar_ok=bool(lidar.connected and hz > 0), lidar_hz=hz)

    def orientation():
        im = imu.state
        s.update(imu_ok=bool(im.get("ready")),
                 heading=im.get("yaw") if im.get("ready") else None)

    def cam():
        s["cam_ok"] = bool(camera.state.get("live"))

    def nav():
        s.update(blocked=guard.blocked, reason=guard.reason)
        p = slam.slam.pose
        s["pose"] = (p.x, p.y, math.degrees(p.th) % 360.0)
        ex = explorer.status
        if ex.get("running"):
            s["explore"] = f"AUTO · {ex.get('state')}"

    def sound():
        a = speaker.brief
        s.update(song=a["track"], paused=a["paused"], pos=a["pos"],
                 dur=a["dur"], volume=a["volume"],
                 beep=(f'"{a["now"]}"' if a["kind"] == "speech" else
                       a["now"] if a["kind"] in ("beep", "tone") else None))

    for fn in (motors, sensors, orientation, cam, nav, sound):
        part(fn)
    return s


def lan_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("192.168.1.1", 1))      # no packet is sent for UDP connect
        return s.getsockname()[0]
    except OSError:
        return "127.0.0.1"
    finally:
        s.close()


def main():
    global robot, lidar, imu, slam, explorer, camera, markers, cliff, detector
    global speaker, screen, http_port, talker, ears, assistant, tracker, follower
    ap = argparse.ArgumentParser(description="drive + lidar + imu")
    ap.add_argument("-p", "--lidar-port", help="serial port (default: auto-detect)")
    ap.add_argument("-b", "--baud", type=int, default=LIDAR_BAUD)
    ap.add_argument("--http-port", type=int, default=HTTP_PORT)
    ap.add_argument("--no-guard", action="store_true")
    ap.add_argument("--learn-markers", action="store_true",
                    help="record unknown tags at the current pose")
    ap.add_argument("--no-cliff", action="store_true",
                    help="do not let the camera veto forward motion")
    ap.add_argument("--no-https", action="store_true",
                    help="do not serve the phone-microphone page over https :5443")
    ap.add_argument("--no-display", action="store_true",
                    help="do not drive the ST7789 status screen")
    ap.add_argument("--no-detect", action="store_true",
                    help="do not run object detection")
    ap.add_argument("--detect-url", default=None,
                    help="off-board detection server, e.g. "
                         "http://192.168.1.5:8000/detect — bigger model "
                         "on your PC. Remembered between runs.")
    ap.add_argument("--tts-url", default=None,
                    help="off-board speech server, e.g. http://192.168.1.7:5005 "
                         "(tools/tts_server.py) — the PC makes the audio, the "
                         "Pi plays it. Remembered between runs.")
    args = ap.parse_args()

    if args.no_guard:
        guard.enabled = False

    try:
        robot = Robot()
    except Exception as exc:                                  # noqa: BLE001
        print(f"\n  GPIO setup failed: {exc}")
        print("  Is web_pilot.py, web_drive.py or web_dashboard.py running?")
        print("  Only one script can own these pins at a time.\n")
        raise

    # The speaker is optional like every sensor: no card, no ffmpeg, the
    # page says so on the Audio tab and everything else carries on.
    speaker = audio.Audio()
    app.register_blueprint(audio.blueprint(speaker))
    talker = tts.Tts(speaker)
    if args.tts_url:
        talker.set(url=args.tts_url)
    app.register_blueprint(tts.blueprint(talker))
    ears = voice.Ears(talker)
    app.register_blueprint(voice.blueprint(ears))
    # The assistant drives the truck through this server's own HTTP API, so
    # its moves take exactly the arrow keys' path — guard, watchdog and all.
    assistant = brain.Brain(ears, talker, speaker, f"http://127.0.0.1:{args.http_port}")
    ears.assistant = lambda: assistant.brief
    app.register_blueprint(brain.blueprint(assistant))

    lidar = Lidar(args.lidar_port or autodetect_lidar(), args.baud)
    imu = ImuReader()
    camera = CameraReader()
    slam = SlamRunner(robot, lidar, imu)
    threading.Thread(target=control_loop, daemon=True).start()
    globals()["lidar_cal"] = LidarCal()
    slam.slam = Slam(COUNTS_PER_REV, WHEEL_DIAM_MM, TRACK_WIDTH_MM,
                     lidar_off=(lidar_cal.x, lidar_cal.y, lidar_cal.yaw),
                     body=(TRUCK_LENGTH_MM, TRUCK_WIDTH_MM))
    slam.resume()

    def guarded_drive(throttle, steer):
        """The explorer publishes intent like everything else, so there is one
        writer to the motors and one place the guard is applied."""
        intent.set(throttle, steer, "explore")

    explorer = Explorer(
        robot, lidar, slam, guard,
        {"len": TRUCK_LENGTH_MM, "wid": TRUCK_WIDTH_MM,
         "margin": SAFETY_MARGIN_MM, "wheel": WHEEL_DIAM_MM,
         "cpr": COUNTS_PER_REV, "track": TRACK_WIDTH_MM},
        guarded_drive)

    # Vision, after SLAM: the marker locator writes pose fixes into it and
    # the cliff detector is read by the guard, so both need their consumers
    # to exist first.
    from cliff import CliffDetector                           # noqa: PLC0415
    # Floor check REMOVED from driving: kept only as an always-off object so
    # the status fields that mention it still exist.
    cliff = CliffDetector(camera, enabled=False)
    from markers import MarkerLocator                         # noqa: PLC0415
    markers = MarkerLocator(camera, slam, learn=args.learn_markers)

    # Object labels. Last, because it is the one thing here that yields to
    # everything else: it skips a cycle whenever SLAM is over budget.
    from detect import Detector                                # noqa: PLC0415
    detector = Detector(camera, lidar, slam, enabled=not args.no_detect,
                        url=args.detect_url)
    # People, for follow mode. Off until asked: on the Pi's own model it costs
    # CPU, and it borrows the detector's server URL and model either way.
    from person import PersonTracker                          # noqa: PLC0415
    tracker = PersonTracker(detector, lambda: body_points(lidar.scan()), slam=slam)
    # Follow mode drives through the same intent + guard as everything else,
    # with the autonomous margin, and borrows the explorer's planner for
    # routes round whatever is between the truck and the person.
    from follow import Follower                               # noqa: PLC0415
    follower = Follower(tracker, slam, lambda: body_points(lidar.scan()), guard,
                        lambda t, s: intent.set(t, s, "follow"), explorer, detector)

    # Hand the tuning registry the live objects it points at. After this,
    # every number in docs/TUNING.md is reachable from the Tune tab.
    #
    # Objects and modules go in separately: "slam" is the Slam object in some
    # targets and the slam module in others, and both are wanted.
    import cliff as cliff_mod                                 # noqa: PLC0415
    import detect as detect_mod                               # noqa: PLC0415
    import explore as explore_mod                             # noqa: PLC0415
    import markers as markers_mod                             # noqa: PLC0415
    import slam as slam_mod                                   # noqa: PLC0415
    _defaults, _missing = tuning.bind(
        objects={"slam": slam.slam, "slamr": slam,
                 "matcher": slam.slam.matcher, "grid": slam.slam.grid,
                 "odom": slam.slam.odom, "lidar_cal": lidar_cal,
                 "guard": guard, "explorer": explorer},
        modules={"slam": slam_mod, "explore": explore_mod,
                 "cliff": cliff_mod, "markers": markers_mod,
                 "detect": detect_mod,
                 # __name__ is "__main__" when run as a script, so this
                 # module has to hand itself over by object rather than name.
                 "web_nav": sys.modules[__name__]})
    _loaded = tuning.load()

    # The status screen goes last: it reads every other object, and a
    # missing or unwired display must cost nothing but its own card.
    http_port = args.http_port
    screen = display.StatusScreen(lcd_snapshot, enabled=not args.no_display)

    print("\n  Speaker Truck — nav")
    print(f"  http://{lan_ip()}:{args.http_port}   (click the page, then arrow keys)")
    print(f"  lidar: {lidar.port or 'NONE'} @ {args.baud}")
    print(f"  imu:   {imu.name or 'NONE — ' + (imu.error or 'not detected')}")
    print(f"  cam:   {camera.name or 'NONE — ' + camera.error}")
    print(f"  tags:  {len(markers.map.tags)} known"
          f"{' — ' + markers.error if markers.error else ''}"
          f"{' · LEARNING' if markers.learn else ''}")
    print(f"  tune:  {len(_defaults)} tunables"
          f"{f' · {len(_loaded)} from tuning.json' if _loaded else ''}"
          f"{f' · {len(_missing)} UNBOUND: ' + ','.join(_missing) if _missing else ''}")
    n_sure = sum(o["status"] == "confirmed" for o in detector.map.committed())
    print(f"  yolo:  {detector.backend}"
          f"{' · ' + detector.url if detector.url else ''}"
          f"{' — ' + detector.error if detector.error else ''}"
          f" · {n_sure} labels on the map, {len(detector.map.pending())} to confirm")
    print(f"  cliff: {'on' if cliff.enabled else 'off'}"
          f" · camera {CAM_PITCH_DEG:.0f}° down at {CAM_HEIGHT_MM:.0f} mm")
    print(f"  guard: {'on' if guard.enabled else 'off'} at {guard.stop_mm:.0f} mm")
    print(f"  audio: {speaker.card() or 'NO ALSA CARD — WIRING.md §7'}"
          f" · {speaker.device or 'ALSA default'}"
          f"{'' if speaker.have_aplay else ' · aplay MISSING'}"
          f"{'' if speaker.have_ffmpeg else ' · ffmpeg MISSING (no MP3)'}"
          f" · {len(speaker.files())} songs")
    _voices = talker.installed()
    print(f"  speech: {'Piper' if talker.have_piper else 'Piper MISSING — pip install piper-tts'}"
          f" · {talker.voice if _voices else 'no voice downloaded (Audio tab)'}")
    print(f"  lcd:   {'ST7789 on SPI0 · ' + str(round(screen.lcd.actual_hz / 1e6, 2)) + ' MHz' if screen.lcd else 'NONE — ' + screen.error}")
    print(f"  slam:  {slam.slam.grid.n}x{slam.slam.grid.n} cells @ "
          f"{slam.slam.grid.res} mm, matching on")
    https_port = None if args.no_https else voice.serve_https(app)
    _ab = assistant.brief
    print(f"  brain: Ollama {_ab['model']} at {_ab['ollama_url']}"
          + (" · OFF (Audio tab)" if not _ab['enabled'] else
             " · connecting to the PC… (status on the Audio tab)"))
    print(f"  talk:  {'https://' + lan_ip() + ':' + str(https_port) + '/talk  (phone mic)' if https_port else 'off'}")
    print(f"  MAX_DUTY {MAX_DUTY:.2f} · watchdog {WATCHDOG_S}s")
    print("\n  *** WHEELS OFF THE GROUND ***\n")
    try:
        # "::" is IPv6 AND IPv4 on Linux (dual-stack). 0.0.0.0 was IPv4 only,
        # and shiv.local resolves to the Pi's IPv6 address first, so anything
        # using the name - the Claude Code truck tools, some phones - got
        # "connection refused" while the IP address worked.
        try:
            app.run(host="::", port=args.http_port, threaded=True)
        except OSError:
            app.run(host="0.0.0.0", port=args.http_port, threaded=True)
    finally:
        # Runs on Ctrl-C and on any exception, so the motors never stay on.
        if explorer:
            explorer.stop("shutting down")
        try:
            slam.save_if_changed()          # the last 20 s of mapping too
        except Exception as e:                                # noqa: BLE001
            print(f"  map not saved: {e}")
        if _tune_timer[0] is not None and _tune_timer[0].is_alive():
            _tune_timer[0].cancel()
            try:
                tuning.save()                # a slider moved in the last 1.5 s
            except OSError as e:
                print(f"  tuning not saved: {e}")
        robot.close()
        camera.close()
        if talker:
            talker.close()
        if speaker:
            speaker.stop()
        if screen:
            screen.close()         # blank + sleep the panel, release GPIO7
        print("\nMotors stopped, STBY low, pins released. Audio stopped.")


if __name__ == "__main__":
    main()
