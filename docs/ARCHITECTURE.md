# Architecture

How the pieces fit together, and the two conventions you have to hold in your
head to read any of this code.

---

## The coordinate frame

**Everything is in the body frame unless it says otherwise.**

```
            +x  forward
             ▲
             │
   +y ◄──────┼──────  −y
   left      │        right
             │
            −x  back
```

- Origin at the truck's **geometric centre**, not at any sensor.
- Millimetres. Angles in degrees, counter-clockwise positive.
- The world frame is the same shape, with its origin wherever SLAM started.

**The one that catches people:** the LiDAR reports bearings that increase
**clockwise** from its nose. So converting a return to the body frame is
`(d·cos a, −d·sin a)` — the negated y is the easy thing to get wrong, and
getting it wrong mirrors the entire map.

The **footprint** (`TRUCK_LENGTH_MM` x `TRUCK_WIDTH_MM`) is the other half of
the frame. Returns inside it are discarded as the robot seeing its own
chassis — in `body_points()` for the guard and via `slam.SELF_L/SELF_W` for
the map. It is tunable live from the Drive tab; `set_truck()` in `web_nav.py`
is the only correct way to change it, because `HALF_L`, `HALF_W`, `CORNER_R`,
the mapper's half-extents and the explorer's geometry dict all derive from it
and would otherwise be left describing the old box.

**Sensors are not at the origin.** Every one has an offset in `pins.py`:

| Sensor | Offset (x, y) | Notes |
|---|---|---|
| LiDAR | (−150, +200) | back left corner, rotated 22° |
| IMU | (−150, −200) | back right; position doesn't affect its reading |
| Camera | (+150, 0) | front centre |

Anything that consumes a sensor must apply both the **rotation** and the
**translation**. Skipping the translation is a real bug that shipped in this
codebase: the cockpit plot rotated scan points but drew them from the canvas
centre, so the picture and the collision guard were working 250 mm apart. If
you write new code that touches scan returns, use `scan_to_robot()` in
`slam.py` or `body_points()` in `web_nav.py` rather than rolling your own.

The IMU's position is recorded for completeness only — a gyro and magnetometer
measure rotation and field, both independent of where on a rigid body they sit.

---

## Files, and what each owns

### Libraries — imported, never run directly

| File | Owns |
|---|---|
| `pins.py` | **Every** GPIO number, robot dimension, sensor offset. Single source of truth. |
| `slam.py` | Odometry, occupancy grid (30 m, NumPy for whole-grid passes), scan matching, background loop closure, map persistence |
| `explore.py` | Frontier detection, clearance-aware A* planning, the path follower, look-around and make-room manoeuvres - for auto-mapping and click-to-go alike |
| `imu.py` | IMU chip detection and fusion (BNO055 / ICM-20948 / MPU-9250) |
| `camera.py` | CSI camera, MJPEG off the hardware encoder, raw frames for analysis |
| `markers.py` | ArUco detection, the learned tag map, pose fixes |
| `cliff.py` | Floor appearance check. **No longer consulted by the guard** - kept, always off |
| `person.py` | Person tracker: box, bearing, distance from LiDAR / floor / box size, clothing-colour signature |
| `follow.py` | Follow mode: one locked person as a track (position, velocity, colours), LiDAR leg tracking, follow control, search when lost |
| `sysstats.py` | Task-manager numbers from `/proc` (CPU per core and per thread, RAM, temperature). Used by the System tab on the Pi and copied into the PC's Docker image |
| `tuning.py` | The registry of every live-tunable value |
| `audio.py` | Speaker player: tones, beeps, MP3 via ffmpeg, the `/audio/*` Flask routes |
| `tts.py` | Text to speech: Piper in a nice-10 worker process, sentence queue, voice downloads, `/tts/*` routes |
| `display.py` | ST7789 SPI driver and the status screen thread (renders at 2 Hz, sends only on change) |
| `calibrate.py` | Measuring scanner rotation and odometry scale from a push |
| `detect.py` | YOLOv8n furniture labels, placed on the map with LiDAR range |
| `voice.py` | Phone as microphone: `/talk` page, heard-speech queue with echo suppression, https server |
| `brain.py` | Voice assistant: heard text -> Ollama on the PC (streamed, tools, vision) -> spoken sentence by sentence |
| `truck_api.py` | The AI-facing actions over web_nav's HTTP API — one implementation for brain.py and the MCP server |
| `../tools/detect_server.py` | **Runs on your PC.** A bigger model, reached over the LAN |
| `../tools/truck_mcp.py` | **Runs on your PC.** MCP server for Claude Code — a thin wrapper over `truck_api.py` |

### Programs

| File | Port | Owns GPIO |
|---|---|---|
| `web_nav.py` | 5004 | yes — **the cockpit; this is the one you want** |
| `web_nav.py` (https) | 5443 | same process — only so a phone may use its microphone on `/talk` |
| `web_pilot.py` | 5003 | yes — superseded by web_nav |
| `web_drive.py` | 5001 | yes — superseded |
| `web_dashboard.py` | 5000 | yes — superseded |
| `lidar_view.py` | 5002 | no — runs alongside anything |
| `*_test.py` | — | one concern each, for bring-up |

> The three superseded pages still carry the **old rotation-only plot bug**
> described above. They were left alone deliberately — fixing them would mean
> maintaining four copies of the same renderer. Use `web_nav.py`.

Only one GPIO-owning program can run at a time; they claim the same pins.
Starting a second one fails with a clear message rather than a traceback.

---

## The thread model in `web_nav.py`

Six threads, all daemons, all reading and writing plain attributes. There are
no locks except in `Intent` and the capture index, and that is deliberate:
Python's GIL makes attribute reads and writes atomic, and every consumer here
tolerates reading a value one cycle stale.

| Thread | Rate | Job |
|---|---|---|
| Control loop | 50 Hz | Applies the latest intent through the guard (larger arc margin when the explorer drives) |
| LiDAR reader | ~11 Hz | Parses serial frames; a revolution ends on the scanner's start flag. Reopens the port after 2.5 s without a scan |
| IMU reader | 50 Hz | Polls the chip, runs fusion |
| SLAM | 5 Hz | Odometry, scan match, integrate, auto-capture |
| Loop closure | ≤ every 2 s | Background thread: the wide 700 mm / 15° search, applied as an offset when done |
| Map autosave | 20 s | Writes `house_map.json` when it has grown |
| Explorer | 10 Hz, while running | Auto-mapping or click-to-go; one thread per run (generation-counted) |
| Follower | 10 Hz, while following | Track update (camera + LiDAR legs), follow control or search; borrows the explorer for routes |
| Markers | 4 Hz | ArUco detection, pose fixes |
| Detection | ~1 Hz | Pose and scan taken at the photo; skipped for SLAM only when it runs on the Pi's own model |
| Flask | per request | The page, `/state`, MJPEG streams - on IPv4 **and** IPv6, so `shiv.local` works |

**Why the control loop is separate.** Motor commands used to be applied inside
the HTTP handler, so every keypress waited on a guard evaluation *and* on the
GIL, which the SLAM thread can hold for hundreds of milliseconds. The lag was
worst exactly when SLAM was busiest. Now the handler only records intent and
returns; a dedicated thread applies it at a steady 50 Hz.

**Why SLAM is 5 Hz and not 10.** A scan match costs ~96 ms on a Pi 4, so a
10 Hz loop has no headroom. At walking pace 5 Hz still gives an update every
~100 mm, finer than the 40 mm mapping gate needs.

**Why loop closure is off the SLAM thread.** One attempt costs ~160 ms on a PC
and several times that on the Pi. Inline, and retried on every failed update,
it held SLAM at 300-430 ms an update in a real house. It now runs beside SLAM
at most every 2 s; its correction is applied relative to where the truck was
when the attempt began. A map reset or load discards a result still in flight.

**Heading.** World heading is the IMU's relative heading plus an offset the
scan matcher corrects (30 % of each matched correction). Before, the IMU value
overwrote the matcher every update, so gyro drift became rotated double walls.
One-off IMU spikes (> 60° in an update) are dropped.

**Degradation is per-sensor.** No IMU still gives drive + LiDAR. No LiDAR
still gives drive + IMU. Nothing refuses to start because a sensor is missing
— it says so on the page instead. The one exception is that the guard refuses
*all* motion without a fresh scan, because a guard that silently stops
guarding is worse than no guard.

---

## How a pose is produced

```
encoders ──┐
           ├─→ DiffOdometry ──→ predicted pose ──┐
IMU yaw ───┘                                     │
                                                 ├─→ ScanMatcher ──→ pose
occupancy grid + current scan ───────────────────┘         │
                                                            ▼
ArUco tag at a known place ──────────────→ apply_fix() ──→ corrected
```

Three corrections, in increasing order of authority:

1. **Odometry** integrates encoder counts. Drifts, always.
2. **Scan matching** nudges the pose until the scan best agrees with the map.
   This is a *closed loop* — it corrects against a map the robot built itself,
   so when the map slowly bends, the pose bends with it and nothing inside the
   system can tell.
3. **Marker fixes** are the only input whose correctness does not depend on
   the robot's own history. A printed tag at a known place is outside the
   loop, which is why it is allowed to overrule everything else.

Heading comes from the IMU rather than the wheel difference, because slip
corrupts a skid-steer's heading faster than anything else — and this chassis
skids on every turn by design.

### The semantic layer

Object labels need **both** sensors and neither alone is enough:

```
camera  --> YOLOv8n --> what it is, and a bearing
                                    |
LiDAR   --> range at that bearing --+--> body frame --> pose --> world (x, y)
                                                                    |
                                                            vote into a cell
                                                            commit after 3
```

A camera box says *what* and roughly *which way*, and nothing about distance —
a monocular camera has no depth. The scanner has depth and no idea what it is
looking at. Pairing them is the whole design.

Detections **vote**; a label is committed only after several sightings agree.
One frame is noise, and a single misfire would otherwise plant "bed" in the
hallway permanently with no way to remove it.

**Walls do not come from this.** The LiDAR already does walls, at 11 Hz, to the
centimetre, in the dark. Detection adds furniture labels and nothing else.

Detection is the one thread that yields: it skips a cycle whenever SLAM is
over its budget. Mapping outranks labelling.

### On-board or off-board

A Pi 4 has no accelerator, so on-board it runs the smallest model there is
(yolov8n) at the smallest useful input (320 px) — measured 613 ms a frame, and
it misses a lot. `tools/detect_server.py` runs on a PC instead: the robot
POSTs the JPEG its camera hardware **already encoded** for the video stream
(~7-30 kB, nothing next to the 2.3 Mbit/s preview) and a desktop-sized model
answers in tens of milliseconds.

The split is deliberate: the server returns **boxes and labels only**. Pairing
them with LiDAR range and the pose stays on the robot, because only the robot
knows where it was standing. That keeps the server stateless — restart it,
move it, swap the model, and the map does not notice.

The on-board model stays loaded even when a server is configured, and the
robot falls back to it after five consecutive failures. A laptop going to
sleep must not take the robot's mapping with it.

---

Marker fixes move **position only** and leave heading alone. A square seen
near head-on has two nearly equal pose solutions and flips between them frame
to frame; the gyro meanwhile is genuinely good at heading. Each sensor supplies
what the other cannot.

---

## Why the venv is Pi-side

A Windows venv contains `Scripts\*.exe` and Windows paths; it is not portable
to ARM Linux. It is created on the Pi, once, and excluded from every sync.

It must be created with **`--system-site-packages`**. `gpiozero`, `lgpio`,
`picamera2`, `opencv` and `pyserial` all come from apt — they are prebuilt
against this system's libraries, and the pip versions either fail to compile
(`lgpio` needs swig) or are subtly wrong (`opencv-python` has no aruco). The
venv can only see apt packages with that flag.

`requirements.txt` therefore holds almost nothing — just Flask.

---

## State that lives on the Pi

None of these are synced back to Windows; all are gitignored.

| File | Written by |
|---|---|
| `house_map.json` | the occupancy grid — autosaved every 20 s and on exit, resumed at startup, deleted by Reset map |
| `places.json` | named rooms (cockpit, a click on the map, or the voice "What room am I in?"); cleared by Reset map |
| `marker_map.json` | learned ArUco tag positions |
| `lidar_cal.json` | scanner mounting, from the calibration controls |
| `drive_invert.json` | motor direction toggles |
| `tuning.json` | values changed on the Tune tab and saved |
| `imu_calib.json` | magnetometer hard-iron offsets |
| `objects.json` | furniture labels in world coordinates — saved automatically after 3 sightings from different spots, one per object; cleared by Reset map |
| `yolov8n.onnx` | the detection model — copied in by hand, never synced |
| `captures/` | stills plus `index.json` with the pose of each |

`tuning.json` records **only values that differ from the code default**, so
improving a default in the source is picked up rather than being permanently
masked.

---

## The cockpit page

`web_nav.py` embeds the whole page as one `PAGE` string — HTML, CSS and JS.
Values from `pins.py` are substituted in as `%%TOKEN%%` placeholders at import.

Structure: a **sticky top bar** (badges, pose, speed, e-stop — never scrolls
away) over **five task tabs**. Only one panel is in the document at a time,
which is a real saving: the canvases redraw at 25 Hz and a plot nobody is
looking at is CPU taken straight from SLAM.

**Tuning controls are rendered wherever their evidence is.** `TUNE_SLOTS` in
the page maps registry groups to containers: mounting and guard onto Drive
beside the plot, matching and odometry onto Map beside SLAM cpu, floor
thresholds onto Vision over the picture. The Tune tab renders the same
registry with no filter, as an index.

Adding a tunable is one line in `tuning.py`; placing it is one entry in
`TUNE_SLOTS`. There is no per-parameter UI code, which is what stopped the
page growing a bespoke card per feature the way it did before.

The camera is a **single DOM node moved between tabs**, not one per tab — two
`<img>` tags on `/camera.mjpg` would be two MJPEG connections off one Pi for
one picture.

There is no build step and no framework. Given the page is served off a robot
you reach over Wi-Fi, a 60 kB self-contained document that needs no CDN is
worth more than component ergonomics.
