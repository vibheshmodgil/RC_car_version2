"""
LiDAR viewer — live polar plot of a YDLIDAR-family scanner in the browser.

    python test/lidar_view.py                 # auto-detect the port
    python test/lidar_view.py -p COM4         # Windows
    python test/lidar_view.py -p /dev/ttyUSB0 # Pi

Then open  http://<host>:5002

Runs on the laptop or on the Pi — it touches no GPIO, so it can run alongside
web_dashboard.py or web_drive.py without fighting them for pins.

    sudo apt install -y python3-serial        # on the Pi
    pip install pyserial                      # on Windows

Protocol
--------
Verified against the actual device on 2026-09-06, not assumed: 205 consecutive
packets decoded with 0 checksum failures, consuming every captured byte with
no resync needed.

  AA 55        header, little-endian 0x55AA
  CT    1 B    package type
  LSN   1 B    sample count in this packet
  FSA   2 B    start angle, degrees = (FSA >> 1) / 64
  LSA   2 B    end angle,   degrees = (LSA >> 1) / 64
  CS    2 B    checksum
  Si    2 B    each, distance mm = Si / 4   (0 = no return)

  checksum = 0x55AA XOR (CT | LSN<<8) XOR FSA XOR LSA XOR every sample

Sample angles are interpolated between FSA and LSA, then corrected by the
YDLIDAR angle-compensation formula. That correction is from the vendor spec —
it is the one part here not confirmed against measured geometry, and it only
shifts points by a degree or two.

Revolutions are detected by the angle wrapping past 360, NOT by the CT flag.
On this unit CT carries values well beyond the documented 0/1, so wrap
detection is the reliable choice.

Measured on this scanner: 11.0 Hz, about 9 packets and ~350 points per turn.
"""

import argparse
import math
import struct
import sys
import threading
import time
from collections import deque

try:
    import serial
    import serial.tools.list_ports as list_ports
except ImportError:
    print("pyserial not found.  sudo apt install -y python3-serial")
    sys.exit(1)

from flask import Flask, jsonify, render_template_string, request

PORT = 5002
BAUD = 115200

# USB-serial bridges these scanners ship behind, used only to pick a sensible
# default port when several are present.
LIKELY = ("cp210", "ch340", "ch9102", "silicon labs", "usb-serial", "ftdi")


def autodetect():
    ports = list(list_ports.comports())
    if not ports:
        return None
    for p in ports:
        if any(k in (p.description or "").lower() for k in LIKELY):
            return p.device
    # Never fall back to /dev/ttyAMA0 or /dev/ttyS0 — a Pi always has those
    # built-in UARTs with nothing attached, and picking one means the viewer
    # sits silent forever. A USB scanner always has a VID.
    usb = [p for p in ports if p.vid is not None]
    return usb[0].device if usb else None


def angle_correction(dist_mm):
    """YDLIDAR angle compensation, in degrees. Vendor formula."""
    if dist_mm <= 0:
        return 0.0
    return math.degrees(math.atan(21.8 * (155.3 - dist_mm) / (155.3 * dist_mm)))


def cluster_points(points, max_gap_mm=180, max_ang_gap=8.0, min_pts=3, limit=14):
    """Group neighbouring returns into objects.

    Two consecutive rays belong to the same object when they are close in
    angle AND close in range. An object's edge, or a doorway, shows up as a
    sudden jump in range — which is exactly where a cluster should end. This
    is what turns a cloud of dots into "there is a 40 cm wide thing 80 cm
    away on your left".
    """
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

    # An object straddling 0 deg arrives as two groups, one at each end of the
    # sorted list. Join them so a wall behind the robot is not reported twice.
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
        # unwrap a group that crosses 0 so the span arithmetic works
        if max(angs) - min(angs) > 180:
            angs = [a + 360 if a < 180 else a for a in angs]
        span = max(angs) - min(angs)
        mean_d = sum(dists) / len(dists)
        out.append({
            "bearing": round((sum(angs) / len(angs)) % 360, 1),
            "start": round(min(angs) % 360, 1),
            "span": round(span, 1),
            "near": round(min(dists)),
            "far": round(max(dists)),
            # chord across the object at its mean range
            "width": round(2 * mean_d * math.sin(math.radians(span) / 2)),
            "points": len(g),
        })

    out.sort(key=lambda c: c["near"])
    return out[:limit]


class Lidar:
    """Reads the serial stream in a thread and keeps the latest full turn."""

    def __init__(self, port, baud=BAUD):
        self.port = port
        self.baud = baud
        self.buf = bytearray()

        self._lock = threading.Lock()
        self._current = []        # points accumulating for the turn in progress
        self._scan = []           # last COMPLETE turn — what gets served
        self._last_angle = None
        self._rev_times = deque(maxlen=20)

        self.packets = 0
        self.bad = 0
        self.connected = False
        self.error = ""
        self._stop = False

        threading.Thread(target=self._run, daemon=True).start()

    # --- serial -------------------------------------------------------------

    def _run(self):
        while not self._stop:
            try:
                with serial.Serial(self.port, self.baud, timeout=0.2) as ser:
                    self.connected = True
                    self.error = ""
                    ser.reset_input_buffer()
                    while not self._stop:
                        chunk = ser.read(4096)
                        if chunk:
                            self.buf += chunk
                            self._consume()
                        elif len(self.buf) > 65536:
                            self.buf.clear()      # stream died mid-packet
            except serial.SerialException as e:
                self.connected = False
                self.error = str(e)
                time.sleep(1.5)                   # unplugged; keep retrying

    def _consume(self):
        while True:
            idx = self.buf.find(b"\xAA\x55")
            if idx < 0:
                # keep the last byte: the header may straddle two reads
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
                # Not a real packet — a header pattern inside sample data.
                # Step past this header and look for the next one.
                self.bad += 1
                del self.buf[:2]
                continue

            self.packets += 1
            self._emit(fsa, lsa, samples)
            del self.buf[:need]

    # --- decoding -----------------------------------------------------------

    def _emit(self, fsa, lsa, samples):
        a0 = (fsa >> 1) / 64.0
        a1 = (lsa >> 1) / 64.0
        span = (a1 - a0) % 360.0
        n = len(samples)

        pts = []
        for i, raw in enumerate(samples):
            dist = raw / 4.0
            if dist <= 0:
                continue                      # no return on this ray
            ang = (a0 + span * (i / (n - 1) if n > 1 else 0.0)) % 360.0
            ang = (ang + angle_correction(dist)) % 360.0
            pts.append((round(ang, 1), round(dist)))

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
                self._last_angle = ang
                self._current.append((ang, dist))

    # --- output -------------------------------------------------------------

    def hz(self):
        if len(self._rev_times) < 2:
            return 0.0
        span = self._rev_times[-1] - self._rev_times[0]
        return (len(self._rev_times) - 1) / span if span > 0 else 0.0

    @staticmethod
    def _sectors(points):
        """Nearest return in each quadrant — the numbers an obstacle-avoidance
        loop would actually consume."""
        out = {"front": None, "right": None, "rear": None, "left": None}
        for ang, dist in points:
            if ang >= 315 or ang < 45:
                k = "front"
            elif ang < 135:
                k = "right"
            elif ang < 225:
                k = "rear"
            else:
                k = "left"
            if out[k] is None or dist < out[k]:
                out[k] = dist
        return out

    @property
    def state(self):
        with self._lock:
            scan = list(self._scan)
        nearest = min(scan, key=lambda p: p[1]) if scan else None
        return {
            "points": scan,
            "count": len(scan),
            "hz": round(self.hz(), 1),
            "packets": self.packets,
            "bad": self.bad,
            "connected": self.connected,
            "error": self.error,
            "port": self.port,
            "baud": self.baud,
            "nearest": {"angle": nearest[0], "dist": nearest[1]} if nearest else None,
            "sectors": self._sectors(scan),
            "clusters": cluster_points(scan),
        }


app = Flask(__name__)
lidar = None

PAGE = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Speaker Truck — LiDAR</title>
<style>
:root{
  color-scheme: dark;
  --bg:#0d1117; --surface-1:#161b22; --surface-2:#1c2430; --border:#2a323d;
  --text-1:#fff; --text-2:#a9b4c0; --text-3:#6e7b8a; --grid:#242d3a;
  --point:#3987e5;          /* single series — radius already encodes range */
  --near:#d29922;
  /* Objects get ONE accent, not a colour each: cluster identity is not stable
     between frames, so per-cluster hues would flicker and mean nothing. */
  --obj:#199e70;
  --good:#3fb950; --critical:#f85149;
}
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:ui-sans-serif,-apple-system,"Segoe UI",Roboto,sans-serif;
     background:var(--bg);color:var(--text-1);padding:20px;
     max-width:1100px;margin:0 auto;-webkit-font-smoothing:antialiased}
.mono{font-family:ui-monospace,"SF Mono",Consolas,monospace}
.num{font-variant-numeric:tabular-nums}
header{display:flex;align-items:center;gap:16px;flex-wrap:wrap;
       padding-bottom:16px;margin-bottom:20px;border-bottom:1px solid var(--border)}
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
.grid{display:grid;grid-template-columns:minmax(340px,1fr) 250px;gap:16px}
@media(max-width:840px){.grid{grid-template-columns:1fr}}
.card{background:var(--surface-1);border:1px solid var(--border);border-radius:12px;padding:18px}
.card h2{font-size:.68rem;text-transform:uppercase;letter-spacing:.1em;
         color:var(--text-3);font-weight:650;margin-bottom:14px}
canvas{width:100%;aspect-ratio:1;display:block}
.stat{display:flex;justify-content:space-between;align-items:baseline;
      padding:9px 0;border-bottom:1px solid var(--border);font-size:.78rem}
.stat:last-child{border-bottom:none}
.stat .k{color:var(--text-3)}
.stat .v{font-weight:600}
.sect{display:grid;grid-template-columns:1fr 1fr;gap:8px;margin-top:6px}
.sq{background:var(--surface-2);border:1px solid var(--border);border-radius:8px;
    padding:10px;text-align:center}
.sq .k{font-size:.62rem;letter-spacing:.1em;color:var(--text-3);font-weight:700}
.sq .v{font-size:1.15rem;font-weight:300;margin-top:2px}
.chips{display:flex;gap:6px;flex-wrap:wrap;margin-bottom:12px}
.chip{padding:6px 12px;border:1px solid var(--border);border-radius:99px;
      background:var(--surface-2);color:var(--text-2);cursor:pointer;
      font-size:.74rem;font-weight:600}
.chip.sel{background:color-mix(in srgb,var(--point) 22%,var(--surface-2));
          border-color:color-mix(in srgb,var(--point) 55%,transparent);color:var(--point)}
label{display:block;font-size:.66rem;color:var(--text-3);margin-bottom:5px;
      letter-spacing:.07em;text-transform:uppercase;font-weight:650}
input[type=number]{width:100%;background:var(--surface-2);border:1px solid var(--border);
  color:var(--text-1);padding:7px 10px;border-radius:6px;font-size:.85rem}
.err{font-size:.72rem;color:var(--critical);margin-top:10px;line-height:1.5}
.note{font-size:.7rem;color:var(--text-3);margin-top:12px;line-height:1.55}

/* profile + objects */
.wide{margin-top:16px}
canvas#profile{aspect-ratio:auto;height:190px;cursor:crosshair}
.cap{font-size:.72rem;color:var(--text-3);line-height:1.6;margin-top:10px}
.cap b{color:var(--text-2);font-weight:600}
table{width:100%;border-collapse:collapse;font-size:.76rem;margin-top:4px}
th,td{text-align:right;padding:7px 10px;border-bottom:1px solid var(--border);
      font-variant-numeric:tabular-nums}
th{color:var(--text-3);font-weight:650;letter-spacing:.05em;font-size:.66rem;
   text-transform:uppercase}
td{color:var(--text-2)}
th:first-child,td:first-child{text-align:left}
tbody tr:hover td{background:var(--surface-2);color:var(--text-1)}
td.n{color:var(--text-1);font-weight:600}
.idx{display:inline-flex;align-items:center;justify-content:center;
     width:19px;height:19px;border-radius:5px;background:var(--surface-2);
     border:1px solid var(--obj);color:var(--obj);font-size:.66rem;font-weight:700}
.empty{color:var(--text-3);font-size:.76rem;padding:14px 0;text-align:center}
.toggles{display:flex;gap:6px;flex-wrap:wrap;margin-left:auto}
.hdrow{display:flex;align-items:center;gap:10px;margin-bottom:14px}
.hdrow h2{margin:0}
</style>
</head>
<body>

<header>
  <h1>Speaker Truck <span>/ lidar</span></h1>
  <span id="badge" class="badge off"><i class="dot"></i>NO DATA</span>
  <div class="spacer"></div>
  <span id="portinfo" class="mono" style="font-size:.72rem;color:var(--text-3)"></span>
</header>

<div class="grid">
  <div class="card">
    <div class="chips" id="ranges"></div>
    <canvas id="plot"></canvas>
  </div>

  <div class="card">
    <h2>Scan</h2>
    <div class="stat"><span class="k">Rate</span><span class="v num mono" id="hz">—</span></div>
    <div class="stat"><span class="k">Points / turn</span><span class="v num mono" id="count">—</span></div>
    <div class="stat"><span class="k">Nearest</span><span class="v num mono" id="near">—</span></div>
    <div class="stat"><span class="k">Packets</span><span class="v num mono" id="pkts">—</span></div>
    <div class="stat"><span class="k">Bad checksums</span><span class="v num mono" id="bad">—</span></div>

    <h2 style="margin-top:18px">Nearest per sector</h2>
    <div class="sect">
      <div class="sq"><div class="k">FRONT</div><div class="v num mono" id="s-front">—</div></div>
      <div class="sq"><div class="k">RIGHT</div><div class="v num mono" id="s-right">—</div></div>
      <div class="sq"><div class="k">LEFT</div><div class="v num mono" id="s-left">—</div></div>
      <div class="sq"><div class="k">REAR</div><div class="v num mono" id="s-rear">—</div></div>
    </div>

    <div style="margin-top:18px">
      <label for="off">Zero offset °</label>
      <input type="number" id="off" value="0" step="5" min="-180" max="180">
    </div>

    <p class="note">
      0° is drawn straight up. Put an object directly in front of the robot and
      adjust the offset until the point lands at the top — that calibrates the
      scanner's mounting angle.
    </p>
    <p class="err" id="err"></p>
  </div>
</div>

<div class="card wide">
  <div class="hdrow">
    <h2>Range profile · distance by bearing</h2>
    <div class="toggles">
      <button class="chip sel" id="t-obj">Show objects</button>
    </div>
  </div>
  <canvas id="profile"></canvas>
  <p class="cap">
    The same scan unrolled into a straight line — bearing left to right, range
    bottom to top. Structure is far easier to read here than on the circle:
    a <b>flat run</b> is a wall square-on, a <b>smooth ramp</b> is a wall at an
    angle, and a <b>vertical jump</b> is an edge — the side of an object, or a
    doorway. Breaks in the line are bearings that got no return at all.
  </p>
</div>

<div class="card wide">
  <div class="hdrow"><h2>Detected objects · nearest first</h2></div>
  <table>
    <thead>
      <tr><th>#</th><th>Bearing</th><th>Nearest</th><th>Width</th>
          <th>Arc</th><th>Rays</th></tr>
    </thead>
    <tbody id="objs"></tbody>
  </table>
  <div class="empty" id="objs-empty" style="display:none">No objects in range.</div>
  <p class="cap">
    Neighbouring rays are grouped into an object when they are close in both
    bearing and range; a jump in range ends the group. <b>Width</b> is the
    chord across the object at its mean range, so it is a real size in mm —
    that is the number that tells you whether the truck fits past it.
    Groups of fewer than 3 rays are dropped as noise.
  </p>
</div>

<script>
const $ = id => document.getElementById(id);
const RANGES = [1000, 2000, 4000, 8000];
let maxRange = 4000, offset = 0, pts = [], nearest = null;
let clusters = [], showObj = true, hoverA = null;

$('t-obj').addEventListener('click', () => {
  showObj = !showObj;
  $('t-obj').classList.toggle('sel', showObj);
  draw(); drawProfile();
});

$('ranges').innerHTML = RANGES.map(r =>
  `<button class="chip" data-r="${r}">${r/1000} m</button>`).join('');
$('ranges').querySelectorAll('.chip').forEach(b => b.addEventListener('click', () => {
  maxRange = +b.dataset.r; markRange(); draw(); drawProfile(); renderObjects();
}));
function markRange(){
  $('ranges').querySelectorAll('.chip').forEach(b =>
    b.classList.toggle('sel', +b.dataset.r === maxRange));
}
markRange();
$('off').addEventListener('input', e => {
  offset = +e.target.value || 0; draw(); drawProfile(); });

const css = k => getComputedStyle(document.documentElement).getPropertyValue(k).trim();

function draw(){
  const c = $('plot'), g = c.getContext('2d');
  const dpr = devicePixelRatio || 1, w = c.clientWidth, h = c.clientHeight;
  if(!w) return;
  c.width = w*dpr; c.height = h*dpr; g.setTransform(dpr,0,0,dpr,0,0);
  g.clearRect(0,0,w,h);

  const cx = w/2, cy = h/2, R = Math.min(w,h)/2 - 26;

  // range rings, one per metre up to maxRange
  g.font = '10px ui-monospace,monospace';
  g.textAlign = 'center'; g.textBaseline = 'middle';
  const stepM = maxRange <= 2000 ? 0.5 : maxRange <= 4000 ? 1 : 2;
  for(let m = stepM; m*1000 <= maxRange + 1; m += stepM){
    const r = (m*1000/maxRange)*R;
    g.strokeStyle = css('--grid'); g.lineWidth = 1;
    g.beginPath(); g.arc(cx, cy, r, 0, 6.2832); g.stroke();
    g.fillStyle = css('--text-3');
    g.fillText(m + ' m', cx, cy - r - 8);
  }
  // cardinal spokes
  for(let a = 0; a < 360; a += 45){
    const rad = (a - 90) * Math.PI/180;
    g.strokeStyle = css('--grid'); g.globalAlpha = .6;
    g.beginPath(); g.moveTo(cx, cy);
    g.lineTo(cx + Math.cos(rad)*R, cy + Math.sin(rad)*R); g.stroke();
    g.globalAlpha = 1;
  }
  g.fillStyle = css('--text-2'); g.font = '11px ui-monospace,monospace';
  g.fillText('FRONT 0°', cx, cy - R - 18);
  g.fillText('180°', cx, cy + R + 18);
  g.fillText('90°', cx + R + 16, cy);
  g.fillText('270°', cx - R - 16, cy);

  // points — one colour, because the radius already encodes distance
  g.fillStyle = css('--point');
  for(const [a, d] of pts){
    if(d > maxRange) continue;
    const rad = (a + offset - 90) * Math.PI/180;
    const r = (d/maxRange)*R;
    g.beginPath(); g.arc(cx + Math.cos(rad)*r, cy + Math.sin(rad)*r, 1.9, 0, 6.2832); g.fill();
  }

  // nearest return, called out
  if(nearest && nearest.dist <= maxRange){
    const rad = (nearest.angle + offset - 90) * Math.PI/180;
    const r = (nearest.dist/maxRange)*R;
    const x = cx + Math.cos(rad)*r, y = cy + Math.sin(rad)*r;
    g.strokeStyle = css('--near'); g.lineWidth = 2;
    g.beginPath(); g.arc(x, y, 6, 0, 6.2832); g.stroke();
    g.fillStyle = css('--near'); g.textAlign = 'left';
    g.fillText(nearest.dist + ' mm', x + 10, y);
  }

  // object arcs — one accent for all of them, numbered to match the table
  if(showObj){
    g.font = '10px ui-monospace,monospace';
    clusters.forEach((c, i) => {
      if(c.near > maxRange) return;
      const r = (c.near/maxRange)*R;
      const a0 = (c.start + offset - 90) * Math.PI/180;
      const a1 = (c.start + c.span + offset - 90) * Math.PI/180;
      g.strokeStyle = css('--obj'); g.lineWidth = 2.5;
      g.beginPath(); g.arc(cx, cy, r, a0, a1); g.stroke();

      const mid = (c.bearing + offset - 90) * Math.PI/180;
      const lx = cx + Math.cos(mid)*(r+13), ly = cy + Math.sin(mid)*(r+13);
      g.fillStyle = css('--obj'); g.textAlign = 'center'; g.textBaseline = 'middle';
      g.fillText(i+1, lx, ly);
    });
  }

  // the scanner itself
  g.fillStyle = css('--text-2');
  g.beginPath(); g.arc(cx, cy, 4, 0, 6.2832); g.fill();
}

// ---- range profile: the same scan unrolled, bearing on x, range on y ----
function drawProfile(){
  const c = $('profile'), g = c.getContext('2d');
  const dpr = devicePixelRatio || 1, w = c.clientWidth, h = c.clientHeight;
  if(!w) return;
  c.width = w*dpr; c.height = h*dpr; g.setTransform(dpr,0,0,dpr,0,0);
  g.clearRect(0,0,w,h);

  const padL = 46, padR = 12, padT = 10, padB = 22;
  const pw = w-padL-padR, ph = h-padT-padB;
  const X = a => padL + ((a + offset) % 360)/360*pw;
  const Y = d => padT + ph - Math.min(d, maxRange)/maxRange*ph;

  g.font = '10px ui-monospace,monospace';
  g.textBaseline = 'middle'; g.textAlign = 'right';
  const stepM = maxRange <= 2000 ? 0.5 : maxRange <= 4000 ? 1 : 2;
  for(let m = 0; m*1000 <= maxRange + 1; m += stepM){
    const y = Y(m*1000);
    g.strokeStyle = css('--grid'); g.lineWidth = 1;
    g.beginPath(); g.moveTo(padL, y+.5); g.lineTo(padL+pw, y+.5); g.stroke();
    g.fillStyle = css('--text-3'); g.fillText(m + ' m', padL-8, y);
  }
  g.textAlign = 'center'; g.textBaseline = 'top';
  for(let a = 0; a <= 360; a += 45){
    const x = padL + a/360*pw;
    g.strokeStyle = css('--grid');
    g.beginPath(); g.moveTo(x+.5, padT); g.lineTo(x+.5, padT+ph); g.stroke();
    g.fillStyle = css('--text-3'); g.fillText(a + '°', x, padT+ph+6);
  }

  // sorted by displayed bearing, so the line runs left to right
  const sorted = pts.map(([a,d]) => [((a + offset) % 360 + 360) % 360, d])
                    .sort((p,q) => p[0]-q[0]);

  g.strokeStyle = css('--point'); g.lineWidth = 2;
  g.lineJoin = 'round'; g.lineCap = 'round';
  g.beginPath();
  let open = false;
  for(let i = 0; i < sorted.length; i++){
    const [a, d] = sorted[i];
    const gap = i && (a - sorted[i-1][0]) > 8;   // no return between these rays
    if(!open || gap){ g.moveTo(padL + a/360*pw, Y(d)); open = true; }
    else g.lineTo(padL + a/360*pw, Y(d));
  }
  g.stroke();

  if(showObj){
    g.fillStyle = css('--obj');
    clusters.forEach((c, i) => {
      if(c.near > maxRange) return;
      const x = padL + ((c.bearing + offset) % 360)/360*pw, y = Y(c.near);
      g.beginPath(); g.arc(x, y, 3.5, 0, 6.2832); g.fill();
      g.textAlign = 'center'; g.textBaseline = 'bottom';
      g.fillText(i+1, x, y-6);
    });
  }

  // hover crosshair — nearest ray to the cursor
  if(hoverA !== null && sorted.length){
    const a = ((hoverA - padL)/pw)*360;
    let best = sorted[0];
    for(const p of sorted) if(Math.abs(p[0]-a) < Math.abs(best[0]-a)) best = p;
    const x = padL + best[0]/360*pw, y = Y(best[1]);
    g.strokeStyle = css('--text-3'); g.globalAlpha = .55; g.lineWidth = 1;
    g.beginPath(); g.moveTo(x+.5, padT); g.lineTo(x+.5, padT+ph); g.stroke();
    g.globalAlpha = 1;
    g.fillStyle = css('--point');
    g.beginPath(); g.arc(x, y, 4.5, 0, 6.2832); g.fill();
    g.strokeStyle = css('--surface-1'); g.lineWidth = 2; g.stroke();

    const label = best[0].toFixed(0) + '°   ' + mm(best[1]);
    g.font = '11px ui-monospace,monospace';
    const tw = g.measureText(label).width + 16;
    const bx = Math.min(padL+pw-tw, Math.max(padL, x - tw/2));
    g.fillStyle = '#0b0f14';
    g.strokeStyle = css('--border'); g.lineWidth = 1;
    g.beginPath(); g.roundRect(bx, padT+2, tw, 20, 5); g.fill(); g.stroke();
    g.fillStyle = css('--text-1'); g.textAlign = 'center'; g.textBaseline = 'middle';
    g.fillText(label, bx + tw/2, padT+12);
  }
}
$('profile').addEventListener('mousemove', e => { hoverA = e.offsetX; drawProfile(); });
$('profile').addEventListener('mouseleave', () => { hoverA = null; drawProfile(); });

function renderObjects(){
  const rows = clusters.filter(c => c.near <= maxRange);
  $('objs-empty').style.display = rows.length ? 'none' : 'block';
  $('objs').innerHTML = rows.map((c, i) =>
      `<tr><td><span class="idx">${i+1}</span></td>`
    + `<td class="n">${c.bearing.toFixed(0)}°</td>`
    + `<td class="n">${mm(c.near)}</td>`
    + `<td>${mm(c.width)}</td>`
    + `<td>${c.span.toFixed(0)}°</td>`
    + `<td>${c.points}</td></tr>`).join('');
}

addEventListener('resize', () => { draw(); drawProfile(); });

const mm = v => v == null ? '—' : (v >= 1000 ? (v/1000).toFixed(2) + ' m' : v + ' mm');

function poll(){
  fetch('/scan').then(r => r.json()).then(d => {
    pts = d.points; nearest = d.nearest; clusters = d.clusters || [];
    const live = d.connected && d.count > 0;
    $('badge').className = 'badge ' + (live ? 'on' : 'off');
    $('badge').innerHTML = '<i class="dot"></i>' + (live ? 'SCANNING' : 'NO DATA');
    $('portinfo').textContent = d.port + ' @ ' + d.baud;
    $('hz').textContent = d.hz.toFixed(1) + ' Hz';
    $('count').textContent = d.count;
    $('near').textContent = d.nearest ? mm(d.nearest.dist) + ' @ ' + d.nearest.angle.toFixed(0) + '°' : '—';
    $('pkts').textContent = d.packets;
    $('bad').textContent = d.bad;
    for(const k of ['front','right','left','rear'])
      $('s-'+k).textContent = mm(d.sectors[k]);
    $('err').textContent = d.error || '';
    draw(); drawProfile(); renderObjects();
  }).catch(() => {
    $('badge').className = 'badge off';
    $('badge').innerHTML = '<i class="dot"></i>NO LINK';
  });
}
setInterval(poll, 100);
poll();
</script>
</body>
</html>"""


@app.route("/")
def index():
    return render_template_string(PAGE)


@app.route("/scan")
def scan():
    return jsonify(lidar.state)


def main():
    global lidar
    ap = argparse.ArgumentParser(description="live LiDAR polar view")
    ap.add_argument("-p", "--port", help="serial port (default: auto-detect)")
    ap.add_argument("-b", "--baud", type=int, default=BAUD)
    ap.add_argument("--http-port", type=int, default=PORT)
    args = ap.parse_args()

    port = args.port or autodetect()
    if not port:
        print("No serial ports found. Plug the LiDAR in, then run")
        print("  python test/lidar_probe.py")
        return

    lidar = Lidar(port, args.baud)
    print(f"\n  LiDAR viewer — {port} @ {args.baud}")
    print(f"  http://localhost:{args.http_port}\n")
    app.run(host="0.0.0.0", port=args.http_port, threaded=True)


if __name__ == "__main__":
    main()
