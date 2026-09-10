"""
Drive page — arrow-key control plus per-side direction inversion.

    cd ~/Desktop/Speaker_truck && source .venv/bin/activate
    python test/web_drive.py

Then open  http://<pi-ip>:5001  and click the page once so it has keyboard
focus. Port 5001, not 5000, so it is obviously a different tool from
web_dashboard.py — but only ONE of them can run at a time, because they both
claim the same GPIOs.

*** WHEELS OFF THE GROUND. The TB6612s cannot survive a stall on these
*** motors (JGB37-520 stalls at 4-5 A, TB6612 peaks at 3.2 A). WIRING.md §8.

Controls
--------
  ↑ / W        forward        ↓ / S        reverse
  ← / A        turn left      → / D        turn right
  Space        e-stop (STBY low)

Hold to drive, release to stop — the page sends a command at 20 Hz while a
key is down and a single zero on release. The on-screen pad does the same
thing for touch.

Direction inversion
-------------------
Three toggles cover every way the drive train can be wired backwards:

  Invert LEFT     left side spins the wrong way
  Invert RIGHT    right side spins the wrong way
  Swap SIDES      steering is mirrored — left/right drivers are reversed

Use them to find out which case you have, then FIX IT IN HARDWARE: WIRING.md
§4 says swap red and white on both right-side motors so that "IN1 high =
forward" stays true everywhere. Software inversion is for the bench; it hides
a wiring fault from every other script in this repo.

The settings persist in drive_invert.json next to this file, so a restart
does not lose what you worked out.

pip dependency: flask
"""

import json
import os
import socket
import sys
import threading
import time

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

PORT = 5001

# Drive packets arrive at 20 Hz while a key is held, so a short deadman is
# safe here. If the tab closes mid-hold the motors stop in well under a
# second instead of driving into a wall.
WATCHDOG_S = 0.6

RATED_RPM = 330

INVERT_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                           "drive_invert.json")
INVERT_KEYS = ("left", "right", "swap")


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
        pass          # a read-only filesystem should not kill the drive page


# ---------------------------------------------------------------------------
# Hardware
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
        duty = min(abs(speed), MAX_DUTY)   # ceiling enforced here, always
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

    # --- enable -------------------------------------------------------------

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

    def drive(self, throttle, steer):
        """throttle and steer both -1.0 .. +1.0, mixed to two side speeds."""
        throttle = max(-1.0, min(1.0, float(throttle)))
        steer = max(-1.0, min(1.0, float(steer)))

        left = throttle + steer
        right = throttle - steer

        # Full throttle plus full steer would ask for 2.0. Scale the pair down
        # together rather than clipping, so a turn keeps its shape instead of
        # straightening out at speed.
        peak = max(1.0, abs(left), abs(right))
        left, right = left / peak, right / peak

        left *= self.limit
        right *= self.limit

        # Swap first — that fixes which physical driver is "left" — then
        # invert each physical side.
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

    # --- background ---------------------------------------------------------

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
            "left_counts": self.enc_left.steps,
            "right_counts": self.enc_right.steps,
            "rated_rpm": RATED_RPM,
        }

    def close(self):
        self.stop()
        self.estop()
        self.left.close()
        self.right.close()
        self.stby.close()


# ---------------------------------------------------------------------------
# Flask
# ---------------------------------------------------------------------------

app = Flask(__name__)

try:
    robot = Robot()
except Exception as exc:                                  # noqa: BLE001
    print(f"\n  GPIO setup failed: {exc}")
    print("  Is web_dashboard.py, web_control.py or motor_test.py already")
    print("  running? Only one script can own these pins at a time.\n")
    raise

PAGE = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Speaker Truck — Drive</title>
<style>
:root{
  color-scheme: dark;
  --bg:#0d1117; --surface-1:#161b22; --surface-2:#1c2430; --border:#2a323d;
  --text-1:#ffffff; --text-2:#a9b4c0; --text-3:#6e7b8a; --grid:#232b36;
  --series-1:#3987e5;   /* LEFT  */
  --series-2:#d95926;   /* RIGHT */
  --good:#3fb950; --critical:#f85149; --warning:#d29922;
}
*{box-sizing:border-box;margin:0;padding:0}
body{
  font-family:ui-sans-serif,-apple-system,"Segoe UI",Roboto,sans-serif;
  background:var(--bg);color:var(--text-1);padding:20px;
  max-width:1060px;margin:0 auto;-webkit-font-smoothing:antialiased;
}
.mono{font-family:ui-monospace,"SF Mono",Consolas,monospace}
.num{font-variant-numeric:tabular-nums}

header{display:flex;align-items:center;gap:16px;flex-wrap:wrap;
       padding-bottom:16px;margin-bottom:20px;border-bottom:1px solid var(--border)}
header h1{font-size:1rem;font-weight:650;letter-spacing:.14em;text-transform:uppercase}
header h1 span{color:var(--text-3);font-weight:400}
.spacer{flex:1}
.badge{display:inline-flex;align-items:center;gap:7px;padding:5px 12px;
       border-radius:99px;font-size:.72rem;font-weight:700;letter-spacing:.06em;border:1px solid}
.badge .dot{width:7px;height:7px;border-radius:50%;background:currentColor}
.badge.on{color:var(--good);border-color:color-mix(in srgb,var(--good) 45%,transparent);
          background:color-mix(in srgb,var(--good) 12%,transparent)}
.badge.off{color:var(--critical);border-color:color-mix(in srgb,var(--critical) 45%,transparent);
           background:color-mix(in srgb,var(--critical) 12%,transparent)}
.btns{display:flex;gap:8px;flex-wrap:wrap}
button{padding:9px 16px;border:1px solid var(--border);border-radius:7px;
       font-size:.78rem;font-weight:650;letter-spacing:.04em;cursor:pointer;
       background:var(--surface-2);color:var(--text-1);transition:filter .12s,transform .06s}
button:hover{filter:brightness(1.25)}
button:active{transform:translateY(1px)}
.b-enable{background:color-mix(in srgb,var(--good) 20%,var(--surface-2));
          border-color:color-mix(in srgb,var(--good) 40%,transparent);color:var(--good)}
.b-stop{background:color-mix(in srgb,var(--warning) 18%,var(--surface-2));
        border-color:color-mix(in srgb,var(--warning) 40%,transparent);color:var(--warning)}
.b-estop{background:var(--critical);border-color:var(--critical);color:#fff}

.grid{display:grid;grid-template-columns:minmax(300px,1fr) minmax(280px,.8fr);gap:16px}
@media(max-width:820px){.grid{grid-template-columns:1fr}}
.card{background:var(--surface-1);border:1px solid var(--border);border-radius:12px;padding:18px}
.card h2{font-size:.68rem;text-transform:uppercase;letter-spacing:.1em;
         color:var(--text-3);font-weight:650;margin-bottom:14px}

/* truck */
.truck{display:flex;justify-content:center}
svg{width:100%;max-width:330px;height:auto}
.tire{fill:none;stroke:#39424f;stroke-width:9}
.wheel.active .tire{stroke:var(--c)}
.hub{fill:var(--surface-2);stroke:var(--c);stroke-width:2.5}
.spokes{transform-box:fill-box;transform-origin:50% 50%;
        animation:spin var(--dur,2s) linear infinite;animation-play-state:paused}
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

/* readouts */
.rows{display:flex;gap:22px;margin-top:14px;justify-content:center}
.rd{text-align:center}
.rd .hd{display:flex;align-items:center;gap:6px;justify-content:center;
        font-size:.68rem;font-weight:700;letter-spacing:.1em;color:var(--text-2)}
.sw{width:10px;height:10px;border-radius:3px}
.rd .v{font-size:1.7rem;font-weight:250;line-height:1.25}
.rd .s{font-size:.68rem;color:var(--text-3)}

/* pad */
.pad{display:grid;grid-template-columns:repeat(3,64px);grid-template-rows:repeat(3,64px);
     gap:8px;justify-content:center;margin:4px auto 16px;touch-action:none}
.pad button{display:flex;align-items:center;justify-content:center;font-size:1.3rem;
            padding:0;border-radius:10px;user-select:none}
.pad button.on{background:color-mix(in srgb,var(--series-1) 35%,var(--surface-2));
               border-color:var(--series-1);color:#fff}
.pad .mid{font-size:.65rem;letter-spacing:.06em}
.hint{font-size:.72rem;color:var(--text-3);line-height:1.6;text-align:center}
.hint kbd{background:var(--surface-2);border:1px solid var(--border);border-radius:4px;
          padding:1px 6px;font-family:ui-monospace,monospace;font-size:.7rem;color:var(--text-2)}

/* toggles */
.tog{display:flex;gap:9px;flex-wrap:wrap;margin-bottom:12px}
.tog button{flex:1;min-width:120px}
.tog button.on{background:color-mix(in srgb,var(--warning) 22%,var(--surface-2));
               border-color:color-mix(in srgb,var(--warning) 55%,transparent);color:var(--warning)}
.note{font-size:.72rem;color:var(--text-3);line-height:1.6}
.note b{color:var(--text-2)}

.ctl-hd{display:flex;justify-content:space-between;align-items:center;margin-bottom:7px}
.ctl-hd label{font-size:.72rem;color:var(--text-2);font-weight:600;letter-spacing:.05em}
.ctl-hd .v{font-size:.78rem;font-weight:600}
input[type=range]{width:100%;accent-color:var(--series-1);height:22px}

.trip{display:none;margin-bottom:16px;padding:11px 14px;border-radius:8px;
      font-size:.78rem;font-weight:600;color:var(--warning);
      background:color-mix(in srgb,var(--warning) 12%,transparent);
      border:1px solid color-mix(in srgb,var(--warning) 40%,transparent)}
.trip.show{display:block}
.focus{margin-bottom:16px;padding:11px 14px;border-radius:8px;font-size:.78rem;
       color:var(--text-2);background:var(--surface-2);border:1px solid var(--border)}
</style>
</head>
<body>

<header>
  <h1>Speaker Truck <span>/ drive</span></h1>
  <span id="badge" class="badge off"><i class="dot"></i>DISABLED</span>
  <div class="spacer"></div>
  <div class="btns">
    <button class="b-enable" onclick="cmd('/enable')">ENABLE</button>
    <button class="b-stop" onclick="cmd('/stop')">STOP</button>
    <button class="b-estop" onclick="cmd('/estop')">E-STOP</button>
  </div>
</header>

<div id="trip" class="trip">⚠ Watchdog tripped — commands stopped arriving, so the motors were stopped.</div>
<div id="focusnote" class="focus">Click anywhere on the page once, then the arrow keys will drive.</div>

<div class="grid">
  <div class="card">
    <h2>Wheels</h2>
    <div class="truck"><svg id="truck" viewBox="0 0 380 340"></svg></div>
    <div class="rows">
      <div class="rd">
        <div class="hd"><i class="sw" style="background:var(--series-1)"></i>LEFT</div>
        <div class="v num mono" id="lr">0.0</div>
        <div class="s">RPM · duty <span id="ld" class="num mono">0.00</span></div>
      </div>
      <div class="rd">
        <div class="hd"><i class="sw" style="background:var(--series-2)"></i>RIGHT</div>
        <div class="v num mono" id="rr">0.0</div>
        <div class="s">RPM · duty <span id="rd" class="num mono">0.00</span></div>
      </div>
    </div>
  </div>

  <div class="card">
    <h2>Drive</h2>
    <div class="pad">
      <span></span><button id="p-f">↑</button><span></span>
      <button id="p-l">←</button><button class="mid" id="p-s">STOP</button><button id="p-r">→</button>
      <span></span><button id="p-b">↓</button><span></span>
    </div>
    <p class="hint">
      <kbd>↑</kbd><kbd>↓</kbd><kbd>←</kbd><kbd>→</kbd> or <kbd>W</kbd><kbd>A</kbd><kbd>S</kbd><kbd>D</kbd>
      — hold to drive<br><kbd>Space</kbd> — e-stop
    </p>
    <div style="margin-top:16px">
      <div class="ctl-hd"><label for="lim">Speed limit</label><span id="limv" class="v num mono">0.40</span></div>
      <input type="range" id="lim" min="5" max="40" value="40" step="1">
    </div>
  </div>
</div>

<div class="card" style="margin-top:16px">
  <h2>Direction inversion</h2>
  <div class="tog">
    <button id="t-left"  onclick="toggle('left')">Invert LEFT</button>
    <button id="t-right" onclick="toggle('right')">Invert RIGHT</button>
    <button id="t-swap"  onclick="toggle('swap')">Swap SIDES</button>
    <button onclick="cmd('/reset_encoders')">Reset encoders</button>
  </div>
  <p class="note">
    <b>Whole robot drives backwards?</b> Invert LEFT and RIGHT.<br>
    <b>Spins on the spot instead of going straight?</b> Invert whichever side runs the wrong way.<br>
    <b>Steering mirrored — ← turns right?</b> Swap SIDES.<br>
    Saved to <span class="mono">drive_invert.json</span>, so it survives a restart.
    Once you know which case it is, <b>fix it in the wiring</b> — WIRING.md §4
    says swap red and white on both right-side motors, so "IN1 high = forward"
    stays true for every other script in this repo.
  </p>
</div>

<script>
const $ = id => document.getElementById(id);

// ---------------------------------------------------------------- wheels
const WHEELS = [
  {id:'fl', x:55,  y:100, side:'l'}, {id:'fr', x:325, y:100, side:'r'},
  {id:'rl', x:55,  y:245, side:'l'}, {id:'rr', x:325, y:245, side:'r'},
];
const R = 42;
(function buildTruck(){
  let s = `<rect class="chassis" x="105" y="46" width="170" height="250" rx="24"/>`
        + `<circle class="cone" cx="190" cy="171" r="54"/>`
        + `<circle class="cone" cx="190" cy="171" r="36"/>`
        + `<circle class="cone" cx="190" cy="171" r="17"/>`
        + `<text class="wlabel" x="190" y="30" text-anchor="middle">FRONT</text>`;
  for(const w of WHEELS){
    const c = w.side==='l' ? 'var(--series-1)' : 'var(--series-2)';
    let sp = '';
    for(let i=0;i<6;i++){
      const a = i*Math.PI/3;
      sp += `<line class="spoke${i===0?' index':''}"`
          + ` x1="${w.x+Math.cos(a)*10}" y1="${w.y+Math.sin(a)*10}"`
          + ` x2="${w.x+Math.cos(a)*(R-8)}" y2="${w.y+Math.sin(a)*(R-8)}"/>`;
    }
    s += `<g class="wheel" id="w-${w.id}" style="--c:${c}">`
       + `<circle class="tire" cx="${w.x}" cy="${w.y}" r="${R}"/>`
       + `<g class="spokes" id="sp-${w.id}">${sp}</g>`
       + `<circle class="hub" cx="${w.x}" cy="${w.y}" r="10"/></g>`;
  }
  $('truck').innerHTML = s;
})();

const smooth = {l:0, r:0};
function spinWheels(lr, rr){
  smooth.l += (lr - smooth.l) * 0.35;
  smooth.r += (rr - smooth.r) * 0.35;
  for(const w of WHEELS){
    const v = w.side==='l' ? smooth.l : smooth.r;
    const g = $('sp-'+w.id), gp = $('w-'+w.id), mag = Math.abs(v);
    if(mag < 1.5){ g.classList.remove('run'); gp.classList.remove('active'); }
    else{
      g.style.setProperty('--dur', Math.max(0.12, 60/mag).toFixed(3)+'s');
      g.classList.add('run');
      g.classList.toggle('rev', v < 0);
      gp.classList.add('active');
    }
  }
}

// ---------------------------------------------------------------- input
const keys = new Set();
const MAP = {ArrowUp:'f', KeyW:'f', ArrowDown:'b', KeyS:'b',
             ArrowLeft:'l', KeyA:'l', ArrowRight:'r', KeyD:'r'};
const PADS = {'p-f':'f', 'p-b':'b', 'p-l':'l', 'p-r':'r'};

function paint(){
  for(const [id, k] of Object.entries(PADS)) $(id).classList.toggle('on', keys.has(k));
}

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
  e.preventDefault();          // stop the arrow keys scrolling the page
  keys.add(k); paint();
});
addEventListener('keyup', e => {
  const k = MAP[e.code];
  if(k){ e.preventDefault(); keys.delete(k); paint(); }
});
// A lost focus with a key still down would latch the motors on.
addEventListener('blur', () => { keys.clear(); paint(); });

for(const [id, k] of Object.entries(PADS)){
  const el = $(id);
  const on  = e => { e.preventDefault(); keys.add(k);    paint(); };
  const off = e => { e.preventDefault(); keys.delete(k); paint(); };
  el.addEventListener('pointerdown', on);
  ['pointerup','pointerleave','pointercancel'].forEach(ev => el.addEventListener(ev, off));
}
$('p-s').addEventListener('click', () => { keys.clear(); paint(); cmd('/stop'); });

// 20 Hz while anything is held, plus one final zero on release.
let wasActive = false;
setInterval(() => {
  const active = keys.size > 0;
  if(active || wasActive) sendDrive();
  wasActive = active;
}, 50);

// ---------------------------------------------------------------- commands
function cmd(url){ fetch(url, {method:'POST'}).then(poll); }
function toggle(k){
  const on = !$('t-'+k).classList.contains('on');
  fetch('/invert', {method:'POST', headers:{'Content-Type':'application/json'},
                    body: JSON.stringify({[k]: on})}).then(poll);
}
$('lim').addEventListener('input', e => {
  const v = e.target.value/100;
  $('limv').textContent = v.toFixed(2);
  fetch('/limit', {method:'POST', headers:{'Content-Type':'application/json'},
                   body: JSON.stringify({value:v})});
});
addEventListener('click', () => $('focusnote').style.display = 'none', {once:true});

function poll(){
  fetch('/state').then(r => r.json()).then(d => {
    const b = $('badge');
    b.className = 'badge ' + (d.enabled ? 'on' : 'off');
    b.innerHTML = '<i class="dot"></i>' + (d.enabled ? 'ENABLED' : 'DISABLED');
    $('trip').classList.toggle('show', !!d.tripped);

    $('lr').textContent = d.left_rpm.toFixed(1);
    $('rr').textContent = d.right_rpm.toFixed(1);
    $('ld').textContent = d.left_speed.toFixed(2);
    $('rd').textContent = d.right_speed.toFixed(2);

    for(const k of ['left','right','swap'])
      $('t-'+k).classList.toggle('on', !!d.invert[k]);

    if(document.activeElement !== $('lim')){
      $('lim').value = Math.round(d.limit*100);
      $('limv').textContent = d.limit.toFixed(2);
    }
    $('lim').max = Math.round(d.max_duty*100);

    spinWheels(d.left_rpm, d.right_rpm);
  }).catch(() => {
    $('badge').className = 'badge off';
    $('badge').innerHTML = '<i class="dot"></i>NO LINK';
  });
}
setInterval(poll, 200);
poll();
</script>
</body>
</html>"""


@app.route("/")
def index():
    return render_template_string(PAGE)


@app.route("/state")
def state():
    return jsonify(robot.state)


@app.route("/drive", methods=["POST"])
def drive():
    d = request.get_json(force=True, silent=True) or {}
    robot.drive(d.get("throttle", 0), d.get("steer", 0))
    return "", 204


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


@app.route("/reset_encoders", methods=["POST"])
def reset_encoders():
    robot.reset_encoders()
    return "", 204


def lan_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("192.168.1.1", 1))     # no packet is sent for a UDP connect
        return s.getsockname()[0]
    except OSError:
        return "127.0.0.1"
    finally:
        s.close()


if __name__ == "__main__":
    inv = robot.invert
    print("\n  Speaker Truck — drive")
    print(f"  http://{lan_ip()}:{PORT}   (click the page, then use arrow keys)")
    print(f"  MAX_DUTY {MAX_DUTY:.2f} · watchdog {WATCHDOG_S}s")
    print(f"  invert: left={inv['left']} right={inv['right']} swap={inv['swap']}")
    print("\n  *** WHEELS OFF THE GROUND ***\n")
    try:
        app.run(host="0.0.0.0", port=PORT, threaded=True)
    finally:
        # Runs on Ctrl-C and on any exception, so the motors never stay on.
        robot.close()
        print("\nMotors stopped, STBY low, pins released.")
