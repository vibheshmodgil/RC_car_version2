"""
Web dashboard — TB6612FNG × 2 + quadrature encoders, with live telemetry.

    cd ~/Desktop/Speaker_truck && source .venv/bin/activate
    python test/web_dashboard.py

Then open  http://<pi-ip>:5000  on any device on the same network.

*** WHEELS OFF THE GROUND. The TB6612s cannot survive a stall on these
*** motors (JGB37-520 stalls at 4-5 A, TB6612 peaks at 3.2 A). See WIRING.md
*** section 8. MAX_DUTY stays capped until the drivers are replaced.

This is the visual sibling of web_control.py — same hardware layer, same
routes, a dashboard front end instead of a bare control panel:

  * four wheels that spin at the real measured RPM, in the real direction
  * per-side RPM readout, duty bar against the MAX_DUTY ceiling
  * a 30-second scrolling RPM trace with crosshair readout
  * raw encoder counts, for measuring COUNTS_PER_REV (still an estimate)

Controls
--------
  Left / Right sliders   — PWM duty -1.0 … +1.0 (capped at MAX_DUTY)
  PWM Hz input           — carrier frequency on the fly (100–20000)
  Max Duty input         — raise / lower the ceiling without editing pins.py
  STOP                   — brake both sides then coast
  E-STOP                 — pulls STBY LOW immediately (both drivers off)
  ENABLE                 — brings STBY HIGH again after an e-stop
  Reset Encoders         — zeroes both count accumulators

Deadman
-------
The page polls /state continuously, and that poll feeds a watchdog. Close the
tab or drop the wifi while driving and the motors stop after WATCHDOG_S. This
is the one behaviour that differs from web_control.py — driving a robot from a
browser with no deadman means a closed tab leaves it running into a wall. Set
WATCHDOG_S = 0 to disable.

Audio
-----
The MAX98357A is an I2S amp with no volume register, so every level control
here is done in software by the player, not by the amp.

  Tone / sweep   — a WAV generated in-process and played with `aplay`. Needs
                   only alsa-utils, so it works before ffmpeg is installed and
                   is the first thing to try when the speaker is silent.
  Music          — uploaded files played with `ffplay`, which decodes MP3 and
                   applies the bass/treble shelves and volume in one pass.

    sudo apt install -y alsa-utils ffmpeg

Wiring, the SD/GAIN pins and the config.txt overlay: WIRING.md section 7.

pip dependency: flask
"""

import math
import os
import shutil
import socket
import struct
import subprocess
import sys
import tempfile
import threading
import time
import wave

# Absolute path, so the script works no matter which directory it is
# launched from. __file__.rsplit("/") breaks when run as `python x.py`
# from inside test/, because there is then no "/" to split on.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from pins import (  # noqa: E402
    LEFT_ENC_A, LEFT_ENC_B,
    RIGHT_ENC_A, RIGHT_ENC_B,
    LEFT_PWM, LEFT_IN1, LEFT_IN2,
    RIGHT_PWM, RIGHT_IN1, RIGHT_IN2,
    STBY, PWM_HZ, MAX_DUTY, COUNTS_PER_REV,
)

from gpiozero import DigitalOutputDevice, PWMOutputDevice, RotaryEncoder  # noqa: E402
from flask import Flask, jsonify, request, render_template_string  # noqa: E402
from werkzeug.utils import secure_filename  # noqa: E402

# Seconds without a /state poll before the motors are stopped. 0 disables.
WATCHDOG_S = 2.0

# Nominal free-running speed of the JGB37-520, used as the trace's initial
# y-scale. The scale grows past this if the motors ever exceed it.
RATED_RPM = 330

# --- Audio -----------------------------------------------------------------
AUDIO_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "uploads")
ALLOWED_AUDIO = {".mp3", ".wav", ".ogg", ".flac", ".m4a", ".aac"}
MAX_UPLOAD_MB = 32

# Ceiling on generated tone amplitude, 0.0-1.0. A full-scale sine into a
# class-D amp is loud enough to damage a small speaker and your hearing, and
# the tone is a diagnostic, not a demo.
MAX_TONE_LEVEL = 0.5


# ---------------------------------------------------------------------------
# Robot hardware
# ---------------------------------------------------------------------------

class Side:
    """One TB6612 channel pair driving the two wheels on one side.

        IN1=H IN2=L -> forward      IN1=L IN2=H -> reverse
        IN1=L IN2=L -> coast        IN1=H IN2=H -> brake
    """

    def __init__(self, name, in1, in2, pwm_pin, hz):
        self.name = name
        # initial_value=False writes the safe level before the pin is driven,
        # so the motor cannot twitch during construction.
        self.in1 = DigitalOutputDevice(in1, initial_value=False)
        self.in2 = DigitalOutputDevice(in2, initial_value=False)
        self.pwm = PWMOutputDevice(pwm_pin, initial_value=0, frequency=hz)
        self._speed = 0.0

    def drive(self, speed, max_duty):
        speed = max(-1.0, min(1.0, speed))
        duty = min(abs(speed), max_duty)
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

    def set_frequency(self, hz):
        self.pwm.frequency = int(hz)

    def close(self):
        self.coast()
        self.in1.close(); self.in2.close(); self.pwm.close()


class Robot:
    """Differential drive over two Sides, plus STBY, encoders and watchdog."""

    def __init__(self):
        self._max_duty = MAX_DUTY
        self._pwm_hz   = PWM_HZ
        self._lock     = threading.Lock()
        self._enabled  = False

        # STBY starts LOW: both drivers disabled until enable() is called.
        self.stby  = DigitalOutputDevice(STBY, initial_value=False)
        self.left  = Side("LEFT",  LEFT_IN1,  LEFT_IN2,  LEFT_PWM,  PWM_HZ)
        self.right = Side("RIGHT", RIGHT_IN1, RIGHT_IN2, RIGHT_PWM, PWM_HZ)

        # gpiozero RotaryEncoder does quadrature decoding on A/B.
        # max_steps=0 means the count is unbounded.
        self.enc_left  = RotaryEncoder(LEFT_ENC_A,  LEFT_ENC_B,  max_steps=0)
        self.enc_right = RotaryEncoder(RIGHT_ENC_A, RIGHT_ENC_B, max_steps=0)

        self._last_left_steps  = 0
        self._last_right_steps = 0
        self._last_rpm_time    = time.monotonic()
        self._rpm_left  = 0.0
        self._rpm_right = 0.0

        # Fed by every /state poll; the watchdog stops the motors without it.
        self._last_seen  = time.monotonic()
        self._tripped    = False

        threading.Thread(target=self._rpm_loop, daemon=True).start()
        if WATCHDOG_S:
            threading.Thread(target=self._watchdog_loop, daemon=True).start()

    # --- enable / disable ---------------------------------------------------

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

    # --- drive --------------------------------------------------------------

    def tank(self, left_speed, right_speed):
        with self._lock:
            self.left.drive(left_speed,  self._max_duty)
            self.right.drive(right_speed, self._max_duty)

    # --- tuning setters -----------------------------------------------------

    def set_max_duty(self, v):
        with self._lock:
            self._max_duty = max(0.0, min(1.0, float(v)))

    def set_pwm_hz(self, hz):
        with self._lock:
            hz = max(100, min(20000, int(hz)))
            self._pwm_hz = hz
            self.left.set_frequency(hz)
            self.right.set_frequency(hz)

    # --- encoders -----------------------------------------------------------

    def reset_encoders(self):
        self.enc_left.steps  = 0
        self.enc_right.steps = 0

    def _rpm_loop(self):
        while True:
            time.sleep(0.2)
            now   = time.monotonic()
            dt    = now - self._last_rpm_time
            l_now = self.enc_left.steps
            r_now = self.enc_right.steps

            dl = l_now - self._last_left_steps
            dr = r_now - self._last_right_steps

            if dt > 0 and COUNTS_PER_REV > 0:
                self._rpm_left  = (dl / COUNTS_PER_REV) / dt * 60.0
                self._rpm_right = (dr / COUNTS_PER_REV) / dt * 60.0

            self._last_left_steps  = l_now
            self._last_right_steps = r_now
            self._last_rpm_time    = now

    # --- watchdog -----------------------------------------------------------

    def touch(self):
        self._last_seen = time.monotonic()

    def _moving(self):
        return self.left._speed != 0.0 or self.right._speed != 0.0

    def _watchdog_loop(self):
        while True:
            time.sleep(0.25)
            if not self._moving():
                continue
            if time.monotonic() - self._last_seen > WATCHDOG_S:
                print(f"  watchdog: no client for {WATCHDOG_S}s — stopping")
                self.tank(0, 0)
                self.stop()
                self._tripped = True

    # --- state --------------------------------------------------------------

    @property
    def state(self):
        return {
            "enabled":      self._enabled,
            "max_duty":     round(self._max_duty, 3),
            "pwm_hz":       self._pwm_hz,
            "left_speed":   round(self.left._speed, 3),
            "right_speed":  round(self.right._speed, 3),
            "left_counts":  self.enc_left.steps,
            "right_counts": self.enc_right.steps,
            "left_rpm":     round(self._rpm_left,  1),
            "right_rpm":    round(self._rpm_right, 1),
            "cpr":          COUNTS_PER_REV,
            "rated_rpm":    RATED_RPM,
            "tripped":      self._tripped,
        }

    def close(self):
        self.estop()
        self.left.close()
        self.right.close()
        self.stby.close()


# ---------------------------------------------------------------------------
# Audio — MAX98357A over I2S
# ---------------------------------------------------------------------------

def _write_wav(path, samples, rate=44100):
    """16-bit mono WAV from an iterable of floats in -1.0 .. 1.0."""
    with wave.open(path, "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(rate)
        w.writeframes(b"".join(
            struct.pack("<h", int(max(-1.0, min(1.0, s)) * 32767))
            for s in samples
        ))


def _envelope(i, n, fade):
    """10 ms raised edges. Without them a tone starts and ends with a click,
    which on a class-D amp is a broadband transient into the speaker."""
    if i < fade:
        return i / fade
    if i > n - fade:
        return (n - i) / fade
    return 1.0


def tone_samples(freq, seconds, level, rate=44100):
    n = int(rate * seconds)
    fade = max(1, int(rate * 0.01))
    for i in range(n):
        yield level * _envelope(i, n, fade) * math.sin(2 * math.pi * freq * i / rate)


def sweep_samples(f0, f1, seconds, level, rate=44100):
    """Logarithmic sweep — equal time per octave, which is how a speaker's
    response is actually read. A linear sweep spends most of its time above
    5 kHz and tells you nothing about the bottom end."""
    n = int(rate * seconds)
    fade = max(1, int(rate * 0.01))
    ratio = f1 / f0
    phase = 0.0
    for i in range(n):
        f = f0 * (ratio ** (i / n))
        phase += 2 * math.pi * f / rate
        yield level * _envelope(i, n, fade) * math.sin(phase)


class Audio:
    """One player process at a time, driven by ALSA command-line tools.

    The MAX98357A has no volume register — it plays whatever samples arrive —
    so `amixer` shows no control and every level here is applied in software.
    """

    def __init__(self):
        os.makedirs(AUDIO_DIR, exist_ok=True)
        self._proc = None
        self._now = None
        self._lock = threading.Lock()
        self.have_aplay = shutil.which("aplay") is not None
        self.have_ffplay = shutil.which("ffplay") is not None
        self._tone_path = os.path.join(tempfile.gettempdir(), "truck_tone.wav")
        # None = ALSA's default device. On a Pi with HDMI plus an I2S card the
        # default is frequently the wrong one, which plays to silence with no
        # error, so the device is selectable.
        self.device = None
        self.last_error = ""

    # --- devices ------------------------------------------------------------

    def devices(self):
        """Playback devices from `aplay -l`, as selectable hw:C,D strings."""
        if not self.have_aplay:
            return []
        try:
            out = subprocess.run(["aplay", "-l"], capture_output=True,
                                 text=True, timeout=5).stdout or ""
        except (OSError, subprocess.SubprocessError):
            return []
        found = []
        for line in out.splitlines():
            # card 0: MAX98357A [MAX98357A], device 0: bcm2835-i2s-... []
            if not line.startswith("card ") or "device " not in line:
                continue
            try:
                card = line.split("card ", 1)[1].split(":", 1)[0].strip()
                dev = line.split("device ", 1)[1].split(":", 1)[0].strip()
                name = line.split("[", 1)[1].split("]", 1)[0]
            except IndexError:
                continue
            # plughw, not hw. `hw:` is the raw device and accepts only the
            # exact format the hardware wants — the bcm2835 I2S interface
            # wants stereo, so a mono tone is rejected with "Channels count
            # non available" and nothing plays. `plughw:` inserts ALSA's
            # conversion plugin, which fixes up channels, rate and sample
            # format on the way through.
            found.append({"dev": f"plughw:{card},{dev}", "name": name})
        return found

    def card(self):
        """Name of the first playback card, for display only."""
        d = self.devices()
        return d[0]["name"] if d else None

    def set_device(self, dev):
        """dev is one of the hw:C,D strings from devices(), or None/'' for
        ALSA's default."""
        if not dev:
            self.device = None
            return
        if dev not in [d["dev"] for d in self.devices()]:
            raise ValueError(f"unknown device: {dev}")
        self.device = dev

    def _alsa_args(self):
        return ["-D", self.device] if self.device else []

    # --- transport ----------------------------------------------------------

    @property
    def playing(self):
        p = self._proc
        return p is not None and p.poll() is None

    def stop(self):
        with self._lock:
            p, self._proc, self._now = self._proc, None, None
        if p and p.poll() is None:
            p.terminate()
            try:
                p.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                p.kill()

    def _spawn(self, cmd, label, env=None):
        """Start a player and confirm it is still alive a moment later.

        Popen succeeds even when the player dies instantly — wrong ALSA
        device, no sound card, unsupported format — so without this check the
        request returns 200 while nothing plays and nothing is reported.
        stderr is captured rather than discarded so the real message
        ('No such file or directory', 'Device or resource busy') reaches the
        page instead of /dev/null.
        """
        self.stop()          # only ever one player, so a new play replaces it
        e = dict(os.environ)
        if env:
            e.update(env)
        proc = subprocess.Popen(cmd, stdout=subprocess.DEVNULL,
                                stderr=subprocess.PIPE, env=e)
        time.sleep(0.35)
        if proc.poll() is not None and proc.returncode != 0:
            raw = (proc.stderr.read() or b"").decode(errors="replace").strip()
            msg = raw.splitlines()[-1] if raw else f"player exited {proc.returncode}"
            self.last_error = msg
            raise RuntimeError(msg)
        self.last_error = ""
        with self._lock:
            self._proc = proc
            self._now = label

    # --- sources ------------------------------------------------------------

    def tone(self, freq, seconds, level):
        if not self.have_aplay:
            raise RuntimeError("aplay not found — sudo apt install alsa-utils")
        freq = max(20.0, min(20000.0, float(freq)))
        seconds = max(0.2, min(10.0, float(seconds)))
        level = max(0.0, min(MAX_TONE_LEVEL, float(level)))
        _write_wav(self._tone_path, tone_samples(freq, seconds, level))
        self._spawn(["aplay", "-q"] + self._alsa_args() + [self._tone_path],
                    f"{freq:.0f} Hz tone")

    def sweep(self, f0, f1, seconds, level):
        if not self.have_aplay:
            raise RuntimeError("aplay not found — sudo apt install alsa-utils")
        seconds = max(1.0, min(20.0, float(seconds)))
        level = max(0.0, min(MAX_TONE_LEVEL, float(level)))
        _write_wav(self._tone_path, sweep_samples(f0, f1, seconds, level))
        self._spawn(["aplay", "-q"] + self._alsa_args() + [self._tone_path],
                    f"sweep {f0:.0f}-{f1:.0f} Hz")

    def play(self, name, volume, bass, treble):
        if not self.have_ffplay:
            raise RuntimeError("ffplay not found — sudo apt install ffmpeg")
        path = self.resolve(name)
        volume = max(0.0, min(1.5, float(volume)))
        bass = max(-12.0, min(12.0, float(bass)))
        treble = max(-12.0, min(12.0, float(treble)))
        # One ffmpeg filter chain does decode + shelves + gain in a single pass.
        af = f"bass=g={bass:.1f},treble=g={treble:.1f},volume={volume:.3f}"
        # ffplay plays through SDL, which takes its ALSA device from AUDIODEV
        # rather than a command-line flag — so the device chosen for the tone
        # test applies to music too.
        env = {"SDL_AUDIODRIVER": "alsa"}
        if self.device:
            env["AUDIODEV"] = self.device
        self._spawn(
            ["ffplay", "-nodisp", "-autoexit", "-loglevel", "error",
             "-af", af, path],
            os.path.basename(path), env=env,
        )

    # --- library ------------------------------------------------------------

    def resolve(self, name):
        """Map a client-supplied name onto a real file inside AUDIO_DIR.

        secure_filename plus a basename strips any path traversal, and the
        isfile check keeps the result inside the upload directory."""
        safe = secure_filename(os.path.basename(name or ""))
        if not safe:
            raise FileNotFoundError(name)
        path = os.path.join(AUDIO_DIR, safe)
        if not os.path.isfile(path):
            raise FileNotFoundError(name)
        return path

    def files(self):
        out = []
        for fn in sorted(os.listdir(AUDIO_DIR)):
            p = os.path.join(AUDIO_DIR, fn)
            if os.path.isfile(p) and os.path.splitext(fn)[1].lower() in ALLOWED_AUDIO:
                out.append({"name": fn, "kb": round(os.path.getsize(p) / 1024)})
        return out

    def save(self, storage):
        name = secure_filename(storage.filename or "")
        ext = os.path.splitext(name)[1].lower()
        if not name or ext not in ALLOWED_AUDIO:
            raise ValueError(f"unsupported file type: {ext or '(none)'}")
        storage.save(os.path.join(AUDIO_DIR, name))
        return name

    def delete(self, name):
        path = self.resolve(name)
        if self._now == os.path.basename(path):
            self.stop()
        os.remove(path)

    @property
    def state(self):
        devs = self.devices()
        return {
            "playing": self.playing,
            "now": self._now,
            "files": self.files(),
            "devices": devs,
            "device": self.device,
            "card": devs[0]["name"] if devs else None,
            "have_aplay": self.have_aplay,
            "have_ffplay": self.have_ffplay,
            "max_tone_level": MAX_TONE_LEVEL,
            "last_error": self.last_error,
        }


# ---------------------------------------------------------------------------
# Flask app
# ---------------------------------------------------------------------------

app = Flask(__name__)
app.config["MAX_CONTENT_LENGTH"] = MAX_UPLOAD_MB * 1024 * 1024
robot = Robot()
audio = Audio()

PAGE = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Speaker Truck</title>
<style>
:root{
  color-scheme: dark;
  --bg:          #0d1117;
  --surface-1:   #161b22;   /* chart surface */
  --surface-2:   #1c2430;
  --border:      #2a323d;
  --text-1:      #ffffff;
  --text-2:      #a9b4c0;
  --text-3:      #6e7b8a;
  --grid:        #232b36;

  /* Categorical slots 1 and 2, dark steps, used unchanged. */
  --series-1:    #3987e5;   /* LEFT  */
  --series-2:    #d95926;   /* RIGHT */

  /* UI accent for the audio section. Not a data series and not a status —
     violet keeps it clear of both motor series and of good/warning/critical. */
  --audio:       #9085e9;

  /* Status palette — reserved, never reused as a series colour. */
  --good:        #3fb950;
  --critical:    #f85149;
  --warning:     #d29922;
}
*{box-sizing:border-box;margin:0;padding:0}
body{
  font-family:ui-sans-serif,-apple-system,"Segoe UI",Roboto,sans-serif;
  background:var(--bg);color:var(--text-1);
  padding:20px;max-width:1180px;margin:0 auto;
  -webkit-font-smoothing:antialiased;
}
.mono{font-family:ui-monospace,"SF Mono","Cascadia Mono",Consolas,monospace}
.num{font-variant-numeric:tabular-nums}

/* ---------- header ---------- */
header{
  display:flex;align-items:center;gap:16px;flex-wrap:wrap;
  padding-bottom:16px;margin-bottom:20px;border-bottom:1px solid var(--border);
}
header h1{font-size:1rem;font-weight:650;letter-spacing:.14em;text-transform:uppercase}
header h1 span{color:var(--text-3);font-weight:400}
.badge{
  display:inline-flex;align-items:center;gap:7px;
  padding:5px 12px;border-radius:99px;font-size:.72rem;font-weight:700;
  letter-spacing:.06em;border:1px solid;
}
.badge .dot{width:7px;height:7px;border-radius:50%;background:currentColor}
.badge.on {color:var(--good);    border-color:color-mix(in srgb,var(--good) 45%,transparent);
           background:color-mix(in srgb,var(--good) 12%,transparent)}
.badge.off{color:var(--critical);border-color:color-mix(in srgb,var(--critical) 45%,transparent);
           background:color-mix(in srgb,var(--critical) 12%,transparent)}
.spacer{flex:1}
.btns{display:flex;gap:8px;flex-wrap:wrap}
button{
  padding:9px 16px;border:1px solid var(--border);border-radius:7px;
  font-size:.78rem;font-weight:650;letter-spacing:.04em;cursor:pointer;
  background:var(--surface-2);color:var(--text-1);
  transition:filter .12s,transform .06s;
}
button:hover{filter:brightness(1.25)}
button:active{transform:translateY(1px)}
.b-enable{background:color-mix(in srgb,var(--good) 20%,var(--surface-2));
          border-color:color-mix(in srgb,var(--good) 40%,transparent);color:var(--good)}
.b-stop  {background:color-mix(in srgb,var(--warning) 18%,var(--surface-2));
          border-color:color-mix(in srgb,var(--warning) 40%,transparent);color:var(--warning)}
.b-estop {background:var(--critical);border-color:var(--critical);color:#fff}

/* ---------- layout ---------- */
.grid{display:grid;grid-template-columns:minmax(320px,1fr) minmax(300px,.85fr);gap:16px}
@media(max-width:860px){.grid{grid-template-columns:1fr}}
.card{
  background:var(--surface-1);border:1px solid var(--border);
  border-radius:12px;padding:18px;
}
.card h2{
  font-size:.68rem;text-transform:uppercase;letter-spacing:.1em;
  color:var(--text-3);font-weight:650;margin-bottom:14px;
}

/* ---------- truck ---------- */
.truck{display:flex;justify-content:center;padding:4px 0}
svg{width:100%;max-width:400px;height:auto}
.tire{fill:none;stroke:#39424f;stroke-width:9}
.wheel.active .tire{stroke:var(--c)}
.hub{fill:var(--surface-2);stroke:var(--c);stroke-width:2.5}
.spokes{
  transform-box:fill-box;transform-origin:50% 50%;
  animation:spin var(--dur,2s) linear infinite;
  animation-play-state:paused;
}
.spokes.run{animation-play-state:running}
.spokes.rev{animation-direction:reverse}
.spoke{stroke:#4b5563;stroke-width:5;stroke-linecap:round}
.wheel.active .spoke{stroke:var(--c);opacity:.75}
.spoke.index{stroke:var(--text-1);opacity:1;stroke-width:6}
@keyframes spin{from{transform:rotate(0)}to{transform:rotate(360deg)}}
.chassis{fill:var(--surface-2);stroke:var(--border);stroke-width:2}
.cone{fill:none;stroke:#3a4453;stroke-width:2}
.wlabel{fill:var(--text-3);font-size:13px;font-weight:600;letter-spacing:.08em;
        font-family:ui-monospace,monospace}
.enc-dot{fill:var(--good)}

/* ---------- readouts ---------- */
.side{padding:14px 0;border-bottom:1px solid var(--border)}
.side:last-child{border-bottom:none}
.side-hd{display:flex;align-items:center;gap:9px;margin-bottom:6px}
.swatch{width:11px;height:11px;border-radius:3px;flex:none}
.side-name{font-size:.72rem;font-weight:700;letter-spacing:.1em;color:var(--text-2)}
.rpm-row{display:flex;align-items:baseline;gap:8px}
.rpm{font-size:2.9rem;font-weight:250;line-height:1;letter-spacing:-.02em}
.rpm-unit{font-size:.72rem;color:var(--text-3);letter-spacing:.1em;font-weight:600}
.meta{display:flex;gap:18px;margin-top:9px;font-size:.72rem;color:var(--text-3)}
.meta b{color:var(--text-2);font-weight:600}

/* duty bar: centre = 0, fill grows either way, 4px rounded data-end */
.duty{position:relative;height:9px;background:var(--surface-2);
      border-radius:4px;margin-top:11px;overflow:hidden}
.duty .zero{position:absolute;left:50%;top:0;bottom:0;width:1px;background:var(--grid)}
.duty .cap{position:absolute;top:0;bottom:0;width:1px;background:var(--text-3);opacity:.5}
.duty .fill{position:absolute;top:0;bottom:0;background:var(--c);border-radius:4px;
            transition:left .12s ease-out,width .12s ease-out}

/* ---------- chart ---------- */
.chart-hd{display:flex;align-items:center;justify-content:space-between;margin-bottom:12px}
.legend{display:flex;gap:14px}
.lg{display:flex;align-items:center;gap:6px;font-size:.72rem;color:var(--text-2)}
canvas{width:100%;height:180px;display:block;cursor:crosshair}
.tip{
  position:absolute;pointer-events:none;opacity:0;transition:opacity .1s;
  background:#0b0f14;border:1px solid var(--border);border-radius:7px;
  padding:8px 10px;font-size:.72rem;white-space:nowrap;z-index:5;
  box-shadow:0 6px 20px rgba(0,0,0,.55);
}
.tip .r{display:flex;align-items:center;gap:7px;color:var(--text-2)}
.tip .r b{color:var(--text-1);font-weight:600;margin-left:auto}
.tip .t{color:var(--text-3);margin-bottom:5px}

/* ---------- controls ---------- */
.ctl{margin-bottom:16px}
.ctl-hd{display:flex;justify-content:space-between;align-items:center;margin-bottom:7px}
.ctl-hd label{font-size:.72rem;color:var(--text-2);font-weight:600;letter-spacing:.05em}
.ctl-hd .v{font-size:.78rem;color:var(--text-1);font-weight:600}
input[type=range]{width:100%;accent-color:var(--c);height:22px}
.tune{display:flex;gap:20px;flex-wrap:wrap;align-items:flex-end}
.tune label{display:block;font-size:.68rem;color:var(--text-3);margin-bottom:5px;
            letter-spacing:.07em;text-transform:uppercase;font-weight:650}
input[type=number]{
  width:108px;background:var(--surface-2);border:1px solid var(--border);
  color:var(--text-1);padding:7px 10px;border-radius:6px;font-size:.85rem;
}
details{margin-top:14px}
summary{font-size:.72rem;color:var(--text-3);cursor:pointer;user-select:none}
table{width:100%;border-collapse:collapse;margin-top:10px;font-size:.72rem}
th,td{text-align:right;padding:4px 8px;border-bottom:1px solid var(--border)}
th{color:var(--text-3);font-weight:600;letter-spacing:.05em}
td{color:var(--text-2)}
th:first-child,td:first-child{text-align:left}
.trip{
  display:none;margin-bottom:16px;padding:11px 14px;border-radius:8px;
  font-size:.78rem;font-weight:600;
  color:var(--warning);background:color-mix(in srgb,var(--warning) 12%,transparent);
  border:1px solid color-mix(in srgb,var(--warning) 40%,transparent);
}
.trip.show{display:block}

/* ---------- audio ---------- */
.chips{display:flex;gap:7px;flex-wrap:wrap}
.chip{
  padding:7px 13px;border:1px solid var(--border);border-radius:99px;
  background:var(--surface-2);color:var(--text-2);cursor:pointer;
  font-size:.76rem;font-weight:600;font-variant-numeric:tabular-nums;
  transition:filter .12s;
}
.chip:hover{filter:brightness(1.3)}
.chip.sel{
  background:color-mix(in srgb,var(--audio) 22%,var(--surface-2));
  border-color:color-mix(in srgb,var(--audio) 55%,transparent);
  color:var(--audio);
}
.b-audio{background:color-mix(in srgb,var(--audio) 22%,var(--surface-2));
         border-color:color-mix(in srgb,var(--audio) 45%,transparent);color:var(--audio)}

.now{
  display:flex;align-items:center;gap:9px;margin-bottom:14px;
  padding:10px 13px;border-radius:8px;background:var(--surface-2);
  border:1px solid var(--border);font-size:.8rem;color:var(--text-2);
  min-height:41px;
}
.now b{color:var(--text-1);font-weight:600}
.now .pulse{
  width:8px;height:8px;border-radius:50%;background:var(--text-3);flex:none;
}
.now.on .pulse{background:var(--audio);animation:pulse 1.4s ease-in-out infinite}
@keyframes pulse{0%,100%{opacity:1;transform:scale(1)}50%{opacity:.35;transform:scale(.75)}}

.files{display:flex;flex-direction:column;gap:6px;margin:12px 0}
.file{
  display:flex;align-items:center;gap:10px;padding:8px 11px;
  background:var(--surface-2);border:1px solid var(--border);border-radius:7px;
  font-size:.78rem;
}
.file .nm{flex:1;overflow:hidden;text-overflow:ellipsis;white-space:nowrap;color:var(--text-1)}
.file .sz{color:var(--text-3);font-size:.7rem;font-variant-numeric:tabular-nums}
.file button{padding:4px 11px;font-size:.7rem}
.file .del{background:none;border:none;color:var(--text-3);padding:4px 6px;font-size:.95rem}
.file .del:hover{color:var(--critical)}
.empty{font-size:.76rem;color:var(--text-3);padding:10px 0}

.drop{
  border:1px dashed var(--border);border-radius:8px;padding:15px;
  text-align:center;font-size:.76rem;color:var(--text-3);cursor:pointer;
  transition:border-color .15s,color .15s;
}
.drop:hover,.drop.over{border-color:var(--audio);color:var(--audio)}
.drop input{display:none}

select{
  background:var(--surface-2);border:1px solid var(--border);color:var(--text-1);
  padding:7px 10px;border-radius:6px;font-size:.82rem;min-width:250px;
}
.eq-wrap{margin-top:16px}
canvas#eq{height:118px;cursor:default}
.warn{
  font-size:.72rem;color:var(--warning);margin-top:10px;line-height:1.5;
}
.mono-note{font-size:.7rem;color:var(--text-3);margin-top:9px;line-height:1.5}
</style>
</head>
<body>

<header>
  <h1>Speaker Truck <span>/ motor telemetry</span></h1>
  <span id="badge" class="badge off"><i class="dot"></i>DISABLED</span>
  <div class="spacer"></div>
  <div class="btns">
    <button class="b-enable" onclick="cmd('/enable')">ENABLE</button>
    <button class="b-stop"   onclick="cmd('/stop')">STOP</button>
    <button class="b-estop"  onclick="cmd('/estop')">E-STOP</button>
  </div>
</header>

<div id="trip" class="trip">⚠ Watchdog tripped — the browser stopped polling, so the motors were stopped.</div>

<div class="grid">

  <div class="card">
    <h2>Drive</h2>
    <div class="truck"><svg id="truck" viewBox="0 0 440 380" aria-label="Truck wheel state"></svg></div>
  </div>

  <div class="card">
    <h2>Measured speed</h2>

    <div class="side" style="--c:var(--series-1)">
      <div class="side-hd">
        <span class="swatch" style="background:var(--series-1)"></span>
        <span class="side-name">LEFT</span>
      </div>
      <div class="rpm-row"><span id="lr" class="rpm num mono">0.0</span><span class="rpm-unit">RPM</span></div>
      <div class="duty"><i class="zero"></i><i id="lcapn" class="cap"></i><i id="lcapp" class="cap"></i><i id="lf" class="fill"></i></div>
      <div class="meta">
        <span>duty <b id="ld" class="num mono">0.00</b></span>
        <span>counts <b id="lc" class="num mono">0</b></span>
      </div>
    </div>

    <div class="side" style="--c:var(--series-2)">
      <div class="side-hd">
        <span class="swatch" style="background:var(--series-2)"></span>
        <span class="side-name">RIGHT</span>
      </div>
      <div class="rpm-row"><span id="rr" class="rpm num mono">0.0</span><span class="rpm-unit">RPM</span></div>
      <div class="duty"><i class="zero"></i><i id="rcapn" class="cap"></i><i id="rcapp" class="cap"></i><i id="rf" class="fill"></i></div>
      <div class="meta">
        <span>duty <b id="rd" class="num mono">0.00</b></span>
        <span>counts <b id="rc" class="num mono">0</b></span>
      </div>
    </div>
  </div>
</div>

<div class="card" style="margin-top:16px;position:relative">
  <div class="chart-hd">
    <h2 style="margin:0">Wheel speed · last 30 s</h2>
    <div class="legend">
      <span class="lg"><i class="swatch" style="background:var(--series-1)"></i>Left</span>
      <span class="lg"><i class="swatch" style="background:var(--series-2)"></i>Right</span>
    </div>
  </div>
  <canvas id="chart"></canvas>
  <div id="tip" class="tip"></div>
  <details>
    <summary>Recent samples (table view)</summary>
    <table>
      <thead><tr><th>t</th><th>Left RPM</th><th>Right RPM</th></tr></thead>
      <tbody id="tbody"></tbody>
    </table>
  </details>
</div>

<div class="grid" style="margin-top:16px">
  <div class="card">
    <h2>Manual control</h2>
    <div class="ctl" style="--c:var(--series-1)">
      <div class="ctl-hd"><label for="ls">Left</label><span id="lv" class="v num mono">0.00</span></div>
      <input type="range" id="ls" min="-100" max="100" value="0" step="1">
    </div>
    <div class="ctl" style="--c:var(--series-2)">
      <div class="ctl-hd"><label for="rs">Right</label><span id="rv" class="v num mono">0.00</span></div>
      <input type="range" id="rs" min="-100" max="100" value="0" step="1">
    </div>
    <div class="btns">
      <button onclick="zero()">Centre sliders</button>
      <button onclick="cmd('/reset_encoders')">Reset encoders</button>
    </div>
  </div>

  <div class="card">
    <h2>Tuning</h2>
    <div class="tune">
      <div>
        <label for="max_duty">Max duty</label>
        <input type="number" id="max_duty" step="0.05" min="0" max="1" value="0.40">
      </div>
      <div>
        <label for="pwm_hz">PWM Hz</label>
        <input type="number" id="pwm_hz" step="100" min="100" max="20000" value="1000">
      </div>
    </div>
    <div class="meta" style="margin-top:16px">
      <span>counts/rev <b id="cpr" class="num mono">—</b></span>
    </div>
    <p style="margin-top:10px;font-size:.72rem;color:var(--text-3);line-height:1.5">
      COUNTS_PER_REV is still an estimate. Reset the encoders, turn one wheel
      exactly one revolution by hand, and read the count — that number replaces
      it in <span class="mono">pins.py</span>.
    </p>
  </div>
</div>

<div class="card" style="margin-top:16px">
  <div class="chart-hd">
    <h2 style="margin:0">Audio · MAX98357A → 4 Ω speaker</h2>
    <span id="acard" class="lg"></span>
  </div>

  <div id="now" class="now"><i class="pulse"></i><span id="nowtxt">Idle</span></div>

  <div class="tune" style="margin-bottom:18px">
    <div>
      <label for="dev">Output device</label>
      <select id="dev"></select>
    </div>
  </div>

  <div class="grid">

    <section>
      <h2>Tone test</h2>
      <div class="chips" id="freqs"></div>

      <div class="tune" style="margin-top:14px">
        <div>
          <label for="freq">Frequency Hz</label>
          <input type="number" id="freq" min="20" max="20000" step="10" value="440">
        </div>
        <div>
          <label for="secs">Seconds</label>
          <input type="number" id="secs" min="0.2" max="10" step="0.5" value="2">
        </div>
      </div>

      <div class="ctl" style="--c:var(--audio);margin-top:16px">
        <div class="ctl-hd"><label for="lvl">Tone level</label><span id="lvlv" class="v num mono">50%</span></div>
        <input type="range" id="lvl" min="0" max="100" value="50" step="5">
      </div>

      <div class="btns">
        <button class="b-audio" onclick="playTone()">▶ Play tone</button>
        <button onclick="playSweep()">Sweep 40 Hz – 15 kHz</button>
        <button class="b-stop" onclick="audioStop()">Stop</button>
      </div>

      <p class="mono-note">
        Tone level is capped in software — a full-scale sine into a class-D amp
        will damage a small speaker. Start low. If the sweep is audible up high
        but vanishes below ~150 Hz, that is the speaker enclosure, not the amp.
      </p>
    </section>

    <section>
      <h2>Music</h2>

      <label class="drop" id="drop">
        Choose an audio file, or drop one here
        <input type="file" id="fileinput" accept="audio/*">
      </label>

      <div class="files" id="files"></div>

      <div class="ctl" style="--c:var(--audio)">
        <div class="ctl-hd"><label for="vol">Volume</label><span id="volv" class="v num mono">80%</span></div>
        <input type="range" id="vol" min="0" max="150" value="80" step="5">
      </div>
      <div class="ctl" style="--c:var(--audio)">
        <div class="ctl-hd"><label for="bass">Bass</label><span id="bassv" class="v num mono">0.0 dB</span></div>
        <input type="range" id="bass" min="-12" max="12" value="0" step="0.5">
      </div>
      <div class="ctl" style="--c:var(--audio)">
        <div class="ctl-hd"><label for="treble">Treble</label><span id="treblev" class="v num mono">0.0 dB</span></div>
        <input type="range" id="treble" min="-12" max="12" value="0" step="0.5">
      </div>

      <div class="eq-wrap">
        <canvas id="eq"></canvas>
      </div>

      <p class="mono-note">
        Approximate shelving response of the filters actually applied. Levels
        are baked in when playback starts, so press play again after moving a
        slider. The MAX98357A has no volume register — all of this is software.
      </p>
      <p class="warn" id="awarn"></p>
    </section>

  </div>
</div>

<script>
// ---------------------------------------------------------------- wheels
// Four wheels in the truck's footprint. Front is up; the two front wheels
// carry the encoders (WIRING.md section 4 — rear encoders are unconnected).
const WHEELS = [
  {id:'fl', x: 62, y:112, side:'l', enc:true },
  {id:'fr', x:378, y:112, side:'r', enc:true },
  {id:'rl', x: 62, y:274, side:'l', enc:false},
  {id:'rr', x:378, y:274, side:'r', enc:false},
];
const R = 46;

function buildTruck(){
  const NS='http://www.w3.org/2000/svg';
  let s = `<rect class="chassis" x="118" y="52" width="204" height="282" rx="26"/>`;
  // speaker cone, because it is a speaker truck
  s += `<circle class="cone" cx="220" cy="193" r="62"/>`
     + `<circle class="cone" cx="220" cy="193" r="42"/>`
     + `<circle class="cone" cx="220" cy="193" r="20"/>`;
  s += `<text class="wlabel" x="220" y="34" text-anchor="middle">FRONT</text>`;

  for(const w of WHEELS){
    const c = w.side==='l' ? 'var(--series-1)' : 'var(--series-2)';
    let spokes = '';
    for(let i=0;i<6;i++){
      const a = i*Math.PI/3;
      const x1 = w.x + Math.cos(a)*11, y1 = w.y + Math.sin(a)*11;
      const x2 = w.x + Math.cos(a)*(R-9), y2 = w.y + Math.sin(a)*(R-9);
      spokes += `<line class="spoke${i===0?' index':''}" x1="${x1}" y1="${y1}" x2="${x2}" y2="${y2}"/>`;
    }
    s += `<g class="wheel" id="w-${w.id}" style="--c:${c}">`
       +   `<circle class="tire" cx="${w.x}" cy="${w.y}" r="${R}"/>`
       +   `<g class="spokes" id="sp-${w.id}">${spokes}</g>`
       +   `<circle class="hub" cx="${w.x}" cy="${w.y}" r="11"/>`
       + `</g>`;
    if(w.enc) s += `<circle class="enc-dot" cx="${w.x + (w.side==='l'?-R-13:R+13)}" cy="${w.y}" r="3.5"/>`;
  }
  document.getElementById('truck').innerHTML = s;
}
buildTruck();

// Smoothed only for the animation — the numbers on screen stay raw, so the
// readout can still be trusted for measuring COUNTS_PER_REV.
const smooth = {l:0, r:0};

function spinWheels(lrpm, rrpm){
  smooth.l += (lrpm - smooth.l) * 0.35;
  smooth.r += (rrpm - smooth.r) * 0.35;
  for(const w of WHEELS){
    const v  = w.side==='l' ? smooth.l : smooth.r;
    const g  = document.getElementById('sp-'+w.id);
    const gp = document.getElementById('w-'+w.id);
    const mag = Math.abs(v);
    if(mag < 1.5){
      g.classList.remove('run');
      gp.classList.remove('active');
    }else{
      // one full turn per revolution: duration = 60 / rpm seconds
      g.style.setProperty('--dur', Math.max(0.12, 60/mag).toFixed(3)+'s');
      g.classList.add('run');
      g.classList.toggle('rev', v < 0);
      gp.classList.add('active');
    }
  }
}

// ---------------------------------------------------------------- chart
const N = 150;                       // 150 samples x 200 ms = 30 s
const hist = {l:[], r:[]};
const cv = document.getElementById('chart'), cx = cv.getContext('2d');
const tip = document.getElementById('tip');
let rated = 330, hoverX = null;

const css = k => getComputedStyle(document.documentElement).getPropertyValue(k).trim();

function push(l, r){
  hist.l.push(l); hist.r.push(r);
  if(hist.l.length > N){ hist.l.shift(); hist.r.shift(); }
}

function draw(){
  const dpr = devicePixelRatio || 1;
  const w = cv.clientWidth, h = cv.clientHeight;
  cv.width = w*dpr; cv.height = h*dpr;
  cx.setTransform(dpr,0,0,dpr,0,0);
  cx.clearRect(0,0,w,h);

  const padL = 46, padR = 58, padT = 10, padB = 20;
  const pw = w - padL - padR, ph = h - padT - padB;

  // symmetric scale around zero, never smaller than the rated speed
  let peak = rated;
  for(const a of [hist.l, hist.r]) for(const v of a) peak = Math.max(peak, Math.abs(v));
  const step = peak <= 120 ? 40 : peak <= 400 ? 100 : 200;
  const top  = Math.ceil(peak/step)*step;

  const X = i => padL + (N<=1?0:i/(N-1))*pw;
  const Y = v => padT + ph/2 - (v/top)*(ph/2);

  // recessive grid + axis labels in muted ink
  cx.font = '11px ui-monospace,monospace';
  cx.textAlign = 'right'; cx.textBaseline = 'middle';
  for(let v = -top; v <= top; v += step){
    const y = Y(v);
    cx.strokeStyle = v === 0 ? css('--text-3') : css('--grid');
    cx.globalAlpha = v === 0 ? .5 : 1;
    cx.lineWidth = 1;
    cx.beginPath(); cx.moveTo(padL, y+.5); cx.lineTo(padL+pw, y+.5); cx.stroke();
    cx.globalAlpha = 1;
    cx.fillStyle = css('--text-3');
    cx.fillText(v, padL-9, y);
  }

  // 2px series lines
  const series = [
    {a:hist.l, c:css('--series-1'), name:'Left'},
    {a:hist.r, c:css('--series-2'), name:'Right'},
  ];
  for(const s of series){
    if(!s.a.length) continue;
    const off = N - s.a.length;
    cx.strokeStyle = s.c; cx.lineWidth = 2;
    cx.lineJoin = 'round'; cx.lineCap = 'round';
    cx.beginPath();
    s.a.forEach((v,i) => i ? cx.lineTo(X(i+off), Y(v)) : cx.moveTo(X(i+off), Y(v)));
    cx.stroke();

    // direct label at the live end — dot carries identity, text stays ink
    const last = s.a[s.a.length-1], ly = Y(last);
    cx.fillStyle = s.c;
    cx.beginPath(); cx.arc(X(N-1), ly, 3.5, 0, 6.284); cx.fill();
    cx.fillStyle = css('--text-2');
    cx.textAlign = 'left';
    cx.fillText(s.name, padL+pw+9, ly);
  }

  // crosshair
  if(hoverX !== null){
    const i = Math.round(((hoverX - padL)/pw)*(N-1));
    if(i >= 0 && i < N){
      cx.strokeStyle = css('--text-3'); cx.globalAlpha = .55; cx.lineWidth = 1;
      cx.beginPath(); cx.moveTo(X(i)+.5, padT); cx.lineTo(X(i)+.5, padT+ph); cx.stroke();
      cx.globalAlpha = 1;
      const off = N - hist.l.length, j = i - off;
      if(j >= 0 && j < hist.l.length){
        for(const s of series){
          cx.fillStyle = s.c;
          cx.beginPath(); cx.arc(X(i), Y(s.a[j]), 4.5, 0, 6.284); cx.fill();
          cx.strokeStyle = css('--surface-1'); cx.lineWidth = 2; cx.stroke();
        }
        tip.style.opacity = 1;
        tip.style.left = Math.min(cv.clientWidth-150, X(i)+14) + 'px';
        tip.style.top  = (padT+8) + 'px';
        tip.innerHTML =
          `<div class="t">t −${(((hist.l.length-1-j)*0.2)).toFixed(1)} s</div>` +
          `<div class="r"><i class="swatch" style="background:${series[0].c}"></i>Left<b>${hist.l[j].toFixed(1)}</b></div>` +
          `<div class="r"><i class="swatch" style="background:${series[1].c}"></i>Right<b>${hist.r[j].toFixed(1)}</b></div>`;
        return;
      }
    }
  }
  tip.style.opacity = 0;
}

cv.addEventListener('mousemove', e => { hoverX = e.offsetX; draw(); });
cv.addEventListener('mouseleave', () => { hoverX = null; draw(); });
addEventListener('resize', draw);

function fillTable(){
  const rows = [];
  const n = hist.l.length;
  for(let i = n-1; i >= Math.max(0, n-10); i--){
    rows.push(`<tr><td>−${((n-1-i)*0.2).toFixed(1)} s</td>`
            + `<td class="num mono">${hist.l[i].toFixed(1)}</td>`
            + `<td class="num mono">${hist.r[i].toFixed(1)}</td></tr>`);
  }
  document.getElementById('tbody').innerHTML = rows.join('');
}

// ---------------------------------------------------------------- controls
const $ = id => document.getElementById(id);

function cmd(url){ fetch(url, {method:'POST'}).then(poll); }
function setParam(k, v){
  fetch('/set', {method:'POST', headers:{'Content-Type':'application/json'},
                 body:JSON.stringify({[k]: v})});
}
function drive(){
  const l = $('ls').value/100, r = $('rs').value/100;
  $('lv').textContent = l.toFixed(2);
  $('rv').textContent = r.toFixed(2);
  fetch('/drive', {method:'POST', headers:{'Content-Type':'application/json'},
                   body:JSON.stringify({left:l, right:r})});
}
function zero(){ $('ls').value = 0; $('rs').value = 0; drive(); }

$('ls').addEventListener('input', drive);
$('rs').addEventListener('input', drive);
$('max_duty').addEventListener('change', e => setParam('max_duty', e.target.value));
$('pwm_hz').addEventListener('change',   e => setParam('pwm_hz',   e.target.value));

function dutyBar(prefix, speed, cap){
  const pct = Math.max(-1, Math.min(1, speed)) * 50;
  const f = $(prefix+'f');
  f.style.left  = (pct >= 0 ? 50 : 50+pct) + '%';
  f.style.width = Math.abs(pct) + '%';
  $(prefix+'capp').style.left = (50 + cap*50) + '%';
  $(prefix+'capn').style.left = (50 - cap*50) + '%';
}

let focused = false;
for(const id of ['max_duty','pwm_hz']){
  $(id).addEventListener('focus', () => focused = true);
  $(id).addEventListener('blur',  () => focused = false);
}

function poll(){
  fetch('/state').then(r => r.json()).then(d => {
    rated = d.rated_rpm || 330;

    const b = $('badge');
    b.className = 'badge ' + (d.enabled ? 'on' : 'off');
    b.innerHTML = '<i class="dot"></i>' + (d.enabled ? 'ENABLED' : 'DISABLED');
    $('trip').classList.toggle('show', !!d.tripped);

    $('lr').textContent = d.left_rpm.toFixed(1);
    $('rr').textContent = d.right_rpm.toFixed(1);
    $('ld').textContent = d.left_speed.toFixed(2);
    $('rd').textContent = d.right_speed.toFixed(2);
    $('lc').textContent = d.left_counts;
    $('rc').textContent = d.right_counts;
    $('cpr').textContent = d.cpr;

    dutyBar('l', d.left_speed,  d.max_duty);
    dutyBar('r', d.right_speed, d.max_duty);

    // don't fight the user while they're typing in a tuning box
    if(!focused){
      $('max_duty').value = d.max_duty;
      $('pwm_hz').value   = d.pwm_hz;
    }

    spinWheels(d.left_rpm, d.right_rpm);
    push(d.left_rpm, d.right_rpm);
    draw();
    fillTable();
  }).catch(() => {
    $('badge').className = 'badge off';
    $('badge').innerHTML = '<i class="dot"></i>NO LINK';
  });
}

setInterval(poll, 200);
poll();

// ---------------------------------------------------------------- audio
const FREQS = [110, 220, 440, 1000, 4000, 10000];
let maxTone = 0.5, audioErr = '';

const hz = f => f >= 1000 ? (f / 1000) + ' kHz' : f + ' Hz';

function buildFreqs(){
  $('freqs').innerHTML = FREQS.map(f =>
    `<button class="chip" data-f="${f}">${hz(f)}</button>`).join('');
  $('freqs').querySelectorAll('.chip').forEach(b =>
    b.addEventListener('click', () => { $('freq').value = b.dataset.f; markFreq(); }));
  markFreq();
}
function markFreq(){
  const v = +$('freq').value;
  $('freqs').querySelectorAll('.chip').forEach(b =>
    b.classList.toggle('sel', +b.dataset.f === v));
}

function fmtLvl(){ $('lvlv').textContent = $('lvl').value + '%'; }
function fmtVol(){ $('volv').textContent = $('vol').value + '%'; }
function fmtEQ(){
  $('bassv').textContent   = (+$('bass').value).toFixed(1) + ' dB';
  $('treblev').textContent = (+$('treble').value).toFixed(1) + ' dB';
  drawEQ();
}

function apost(url, body){
  return fetch(url, {method:'POST', headers:{'Content-Type':'application/json'},
                     body: JSON.stringify(body || {})})
    .then(r => r.json().catch(() => ({})))
    .then(d => { audioErr = (d && d.error) || ''; audioPoll(); })
    .catch(() => { audioErr = 'Request failed.'; });
}

const level = () => (+$('lvl').value / 100) * maxTone;

function playTone(){
  apost('/audio/tone', {freq:+$('freq').value, seconds:+$('secs').value, level:level()});
}
function playSweep(){
  apost('/audio/sweep', {f0:40, f1:15000, seconds:8, level:level()});
}
function audioStop(){ apost('/audio/stop'); }
function playFile(n){
  apost('/audio/play', {file:n, volume:+$('vol').value/100,
                        bass:+$('bass').value, treble:+$('treble').value});
}
function delFile(n){ apost('/audio/delete', {file:n}); }

function upload(file){
  if(!file) return;
  const fd = new FormData();
  fd.append('file', file);
  $('nowtxt').textContent = 'Uploading ' + file.name + '…';
  fetch('/audio/upload', {method:'POST', body:fd})
    .then(r => r.json().catch(() => ({})))
    .then(d => { audioErr = (d && d.error) || ''; audioPoll(); })
    .catch(() => { audioErr = 'Upload failed — file may exceed the size limit.';
                   audioPoll(); });
}
$('fileinput').addEventListener('change', e => { upload(e.target.files[0]); e.target.value = ''; });

const dropZone = $('drop');
['dragenter','dragover'].forEach(ev => dropZone.addEventListener(ev, e => {
  e.preventDefault(); dropZone.classList.add('over'); }));
['dragleave','drop'].forEach(ev => dropZone.addEventListener(ev, e => {
  e.preventDefault(); dropZone.classList.remove('over'); }));
dropZone.addEventListener('drop', e => upload(e.dataTransfer.files[0]));

// secure_filename() on the server reduces names to [A-Za-z0-9._-], so these
// interpolate safely.
function renderFiles(files){
  if(!files.length){
    $('files').innerHTML = '<div class="empty">No files yet — upload one above.</div>';
    return;
  }
  $('files').innerHTML = files.map(f =>
      `<div class="file">`
    +   `<span class="nm" title="${f.name}">${f.name}</span>`
    +   `<span class="sz">${f.kb} kB</span>`
    +   `<button class="b-audio" onclick="playFile('${f.name}')">▶</button>`
    +   `<button class="del" onclick="delFile('${f.name}')" title="Delete">×</button>`
    + `</div>`).join('');
}

// Approximate response of ffmpeg's bass/treble shelving filters, so the curve
// shows what is actually being applied rather than a decorative shape.
function drawEQ(){
  const c = $('eq'), g = c.getContext('2d');
  const dpr = devicePixelRatio || 1, w = c.clientWidth, h = c.clientHeight;
  if(!w) return;
  c.width = w*dpr; c.height = h*dpr; g.setTransform(dpr,0,0,dpr,0,0);
  g.clearRect(0,0,w,h);

  const B = +$('bass').value, T = +$('treble').value;
  const padL = 34, padR = 8, padT = 8, padB = 18;
  const pw = w-padL-padR, ph = h-padT-padB;
  const lo = Math.log10(20), hi = Math.log10(20000);
  const X = f  => padL + (Math.log10(f)-lo)/(hi-lo)*pw;
  const Y = db => padT + ph/2 - (db/14)*(ph/2);
  const resp = f => B*0.5*(1-Math.tanh(2*(Math.log10(f)-2)))
                  + T*0.5*(1+Math.tanh(2*(Math.log10(f)-Math.log10(3000))));

  g.font = '10px ui-monospace,monospace';
  g.textBaseline = 'middle'; g.textAlign = 'right';
  for(const db of [12,6,0,-6,-12]){
    const y = Y(db);
    g.strokeStyle = db === 0 ? css('--text-3') : css('--grid');
    g.globalAlpha = db === 0 ? .5 : 1; g.lineWidth = 1;
    g.beginPath(); g.moveTo(padL,y+.5); g.lineTo(padL+pw,y+.5); g.stroke();
    g.globalAlpha = 1; g.fillStyle = css('--text-3');
    g.fillText((db>0?'+':'')+db, padL-7, y);
  }
  g.textAlign = 'center'; g.textBaseline = 'top';
  for(const f of [100,1000,10000]){
    g.strokeStyle = css('--grid'); g.lineWidth = 1;
    g.beginPath(); g.moveTo(X(f)+.5,padT); g.lineTo(X(f)+.5,padT+ph); g.stroke();
    g.fillStyle = css('--text-3');
    g.fillText(f >= 1000 ? (f/1000)+'k' : f, X(f), padT+ph+5);
  }
  g.beginPath();
  for(let i=0;i<=240;i++){
    const f = 20*Math.pow(1000, i/240);
    i ? g.lineTo(X(f),Y(resp(f))) : g.moveTo(X(f),Y(resp(f)));
  }
  g.strokeStyle = css('--audio'); g.lineWidth = 2; g.lineJoin = 'round'; g.stroke();
}

// Rebuilding the <select> on every poll would fight the user mid-click, so
// only rebuild when the set of devices actually changes.
let devSig = null;
function renderDevices(devs, cur){
  const sig = devs.map(d => d.dev).join('|');
  const sel = $('dev');
  if(sig !== devSig){
    devSig = sig;
    sel.innerHTML = '<option value="">ALSA default</option>'
      + devs.map(d => `<option value="${d.dev}">${d.dev} — ${d.name}</option>`).join('');
  }
  if(document.activeElement !== sel) sel.value = cur || '';
}

function audioPoll(){
  fetch('/audio/state').then(r => r.json()).then(d => {
    maxTone = d.max_tone_level || 0.5;
    const devs = d.devices || [];
    $('acard').textContent = devs.length
      ? (devs.length + ' output' + (devs.length > 1 ? 's' : ''))
      : 'NO ALSA CARD — overlay not loaded';
    renderDevices(devs, d.device);
    $('now').classList.toggle('on', d.playing);
    $('nowtxt').innerHTML = d.playing ? ('Playing <b>' + d.now + '</b>') : 'Idle';
    renderFiles(d.files);

    const miss = [];
    if(!d.have_aplay)  miss.push('alsa-utils');
    if(!d.have_ffplay) miss.push('ffmpeg');
    let msg = '';
    if(miss.length){
      msg = 'Missing: ' + miss.join(' + ') + ' — sudo apt install -y ' + miss.join(' ');
    }else if(!devs.length){
      msg = 'No ALSA playback device. The I2S overlay has not loaded — check '
          + 'dtoverlay in /boot/firmware/config.txt (WIRING.md section 7).';
    }else{
      msg = d.last_error || audioErr;
    }
    $('awarn').textContent = msg;
  }).catch(() => {});
}

$('dev').addEventListener('change', e => apost('/audio/device', {device: e.target.value}));

$('freq').addEventListener('input', markFreq);
$('lvl').addEventListener('input', fmtLvl);
$('vol').addEventListener('input', fmtVol);
$('bass').addEventListener('input', fmtEQ);
$('treble').addEventListener('input', fmtEQ);
addEventListener('resize', drawEQ);

buildFreqs(); fmtLvl(); fmtVol(); fmtEQ();
setInterval(audioPoll, 1000);
audioPoll();
</script>
</body>
</html>"""


@app.route("/")
def index():
    return render_template_string(PAGE)


@app.route("/state")
def state():
    robot.touch()          # this poll is what keeps the watchdog fed
    return jsonify(robot.state)


@app.route("/enable", methods=["POST"])
def enable():
    robot.touch()
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


@app.route("/reset_encoders", methods=["POST"])
def reset_encoders():
    robot.reset_encoders()
    return "", 204


@app.route("/drive", methods=["POST"])
def drive():
    robot.touch()
    data = request.get_json(force=True)
    robot.tank(float(data.get("left", 0)), float(data.get("right", 0)))
    return "", 204


@app.route("/set", methods=["POST"])
def set_param():
    data = request.get_json(force=True)
    if "max_duty" in data:
        robot.set_max_duty(data["max_duty"])
    if "pwm_hz" in data:
        robot.set_pwm_hz(data["pwm_hz"])
    return "", 204


# --- audio -----------------------------------------------------------------
# These return JSON rather than 204 so the page can show why something failed
# (missing ffmpeg, unsupported file type) instead of silently doing nothing.

AUDIO_ERRORS = (RuntimeError, ValueError, OSError, FileNotFoundError)


def _body():
    return request.get_json(force=True, silent=True) or {}


@app.route("/audio/state")
def audio_state():
    return jsonify(audio.state)


@app.route("/audio/tone", methods=["POST"])
def audio_tone():
    d = _body()
    try:
        audio.tone(d.get("freq", 440), d.get("seconds", 2), d.get("level", 0.25))
    except AUDIO_ERRORS as e:
        return jsonify(error=str(e)), 400
    return jsonify(ok=True)


@app.route("/audio/sweep", methods=["POST"])
def audio_sweep():
    d = _body()
    try:
        audio.sweep(d.get("f0", 40), d.get("f1", 15000),
                    d.get("seconds", 8), d.get("level", 0.25))
    except AUDIO_ERRORS as e:
        return jsonify(error=str(e)), 400
    return jsonify(ok=True)


@app.route("/audio/play", methods=["POST"])
def audio_play():
    d = _body()
    try:
        audio.play(d.get("file"), d.get("volume", 0.8),
                   d.get("bass", 0), d.get("treble", 0))
    except AUDIO_ERRORS as e:
        return jsonify(error=str(e)), 400
    return jsonify(ok=True)


@app.route("/audio/stop", methods=["POST"])
def audio_stop():
    audio.stop()
    return jsonify(ok=True)


@app.route("/audio/device", methods=["POST"])
def audio_device():
    try:
        audio.set_device(_body().get("device"))
    except AUDIO_ERRORS as e:
        return jsonify(error=str(e)), 400
    return jsonify(ok=True, device=audio.device)


@app.route("/audio/upload", methods=["POST"])
def audio_upload():
    f = request.files.get("file")
    if f is None:
        return jsonify(error="no file in request"), 400
    try:
        return jsonify(ok=True, name=audio.save(f))
    except AUDIO_ERRORS as e:
        return jsonify(error=str(e)), 400


@app.route("/audio/delete", methods=["POST"])
def audio_delete():
    try:
        audio.delete(_body().get("file"))
    except AUDIO_ERRORS as e:
        return jsonify(error=str(e)), 400
    return jsonify(ok=True)


@app.errorhandler(413)
def too_large(_e):
    return jsonify(error=f"file is larger than {MAX_UPLOAD_MB} MB"), 413


# ---------------------------------------------------------------------------

def lan_ip():
    """The address other devices can reach. gethostbyname() returns 127.0.1.1
    on Debian, which is useless for printing a URL."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("192.168.1.1", 1))   # no packet is sent for UDP connect
        return s.getsockname()[0]
    except OSError:
        return "127.0.0.1"
    finally:
        s.close()


if __name__ == "__main__":
    print("\n  Speaker Truck — web dashboard")
    print(f"  http://{lan_ip()}:5000   (any device on this network)")
    print(f"  MAX_DUTY {MAX_DUTY:.2f} · PWM {PWM_HZ} Hz · CPR {COUNTS_PER_REV}")
    if WATCHDOG_S:
        print(f"  watchdog {WATCHDOG_S}s — motors stop if the page goes away")

    card = audio.card()
    print(f"  audio: {card or 'NO ALSA CARD'}"
          f" · aplay {'ok' if audio.have_aplay else 'MISSING'}"
          f" · ffplay {'ok' if audio.have_ffplay else 'MISSING'}")
    if not card:
        print("         check dtoverlay in /boot/firmware/config.txt "
              "— WIRING.md section 7")
    print(f"  uploads: {AUDIO_DIR}")
    print("\n  *** WHEELS OFF THE GROUND ***\n")
    try:
        app.run(host="0.0.0.0", port=5000, threaded=True)
    finally:
        # Runs on Ctrl-C and on any exception. Without it the motors keep
        # running and the player keeps playing after the server dies.
        robot.close()
        audio.stop()
        print("\nMotors stopped, STBY low, pins released. Audio stopped.")
