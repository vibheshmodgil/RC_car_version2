# Speaker Truck — RC Car

RC speaker truck. Raspberry Pi is the brain, Arduino Mega 2560 handles the
throttle-to-relay hardware path.

`README.md` is the front door for humans; `docs/TUNING.md` explains every
SLAM variable and `docs/ARCHITECTURE.md` the coordinate frame and thread
model. Keep them current when behaviour changes.

## Workflow — read this first

**Files are authored on Windows in VS Code, then synced to the Pi.**
Claude creates and edits files here, on the Windows side, never over SSH.

| | Path |
|---|---|
| Author here (Windows) | `C:\Users\vibhe\OneDrive\Desktop\Speaker_truck` |
| Runs here (Pi) | `/home/shiv/Desktop/Speaker_truck` |

The Pi is the only machine with the actual hardware attached, so code is
written blind on Windows and tested after a sync. Nothing is edited directly
on the Pi — it would be overwritten by the next sync.

## Hardware

4-wheel differential-drive (skid-steer) robot. **Full detail in `WIRING.md`;
pin map and header diagram in `PINOUT.md`.** Those two files are authoritative
— this is the summary.

| Role | Part |
|---|---|
| High-level controller | Raspberry Pi 4 Model B, 8 GB — `shiv@192.168.1.11` |
| OS | Raspberry Pi OS **Trixie** (Debian 13), 64-bit, kernel 6.18, 64 GB SD |
| Low-level motor drivers | 2 × TB6612FNG — #1 left pair, #2 right pair |
| Drive | 4 × **JGB37-520**, 12 V, 330 RPM, 6-wire w/ hall encoder |
| Perception | 1 × 2D LiDAR — **YDLIDAR-family, 115200 baud, 11 Hz** (USB serial)
| Orientation | 1 × **9-axis IMU** on I2C (BNO055 / ICM-20948 / MPU-9250) |
| Vision | 1 × **Camera Module 1 (ov5647)**, CSI ribbon — no GPIO, 53.5° HFOV |
| Audio | **MAX98357A** I2S amp (5 V, buck rail) + 12 W 4 Ω speaker |
| Display | **1.54" 240×240 IPS, ST7789**, SPI0 — status screen (`WIRING.md` §14) |
| Power | 3S Li-ion 11.1 V nom / 12.6 V full, + 5 V buck for logic |

### GPIO (BCM)

| Signal | BCM | Pin | Signal | BCM | Pin |
|---|---|---|---|---|---|
| `LEFT_PWM` | 12 | 32 | `RIGHT_PWM` | 13 | 33 |
| `LEFT_IN1` | 23 | 16 | `RIGHT_IN1` | 27 | 13 |
| `LEFT_IN2` | 24 | 18 | `RIGHT_IN2` | 22 | 15 |
| `STBY` (both drivers) | 25 | 22 | | | |
| `SERVO` | 17 | 11 | `LCD_DC` | 7 | 26 |
| LCD `SCL` (SCLK) | 11 | 23 | LCD `SDA` (MOSI) | 10 | 19 |
| LCD `CS` (CE0) | 8 | 24 | LCD `RES`, `BLK` | 3V3 | 17 |

Defined once in `test/pins.py` — import from there, never hardcode.

Reserved, do not reuse: **18/19/20/21** (I2S audio), **2/3** (I2C — IMU),
**14/15** (UART — ESP32), **7–11** (SPI — LCD; 7 is its `DC`, 9 is owned by
the SPI driver), **5/6/16/26** (motor encoders), **17** (servo), **4** (amp
`SD` mute), **0/1** (HAT EEPROM). **Every GPIO now has an owner** — a new
device goes on I2C or USB.

The LCD needs `dtparam=spi=on` **and** `dtoverlay=spi0-1cs` in
`/boot/firmware/config.txt`. Without the overlay the kernel holds GPIO7 as
SPI CE1 and the display fails with "GPIO busy".

The CSI camera is **not** in that list and never will be: it has its own
ribbon connector and consumes no header pin, so it cannot collide with the
motors, encoders, I2S audio or the I2C IMU. Its settings — size, frame rate,
flip, field of view — are software, and live in `test/pins.py` alongside the
rest.

### ❌ Blocking: the TB6612s cannot drive these motors

JGB37-520 stall ≈ **4–5 A**; TB6612FNG is 1.2 A continuous / 3.2 A peak per
channel. Stall exceeds even the peak rating, and stall happens every time a
wheel hits an obstacle. **Replace the drivers before the robot touches the
floor** — `WIRING.md` §8 "Driver replacement" recommends a Cytron MDD10A
(one board, 10 A/channel, both motors of a side paralleled per channel).

Until then the TB6612s are fine for **bench testing with wheels off the
ground at low duty** — no-load draw is only ~0.2–0.4 A. `MAX_DUTY` is capped
at 0.40 in `test/pins.py` for exactly this.

### Still unresolved — verify

- ⚠ Encoder wire colours — standard 6-wire code assumed (**red** M+,
  **white** M−, **blue** hall VCC, **black** hall GND, **yellow** A,
  **green** B; full chain in `WIRING.md` §4). Batches vary, so confirm by
  meter: motor pair reads 1–10 Ω, every hall wire reads open against it.
- ⚠ `COUNTS_PER_REV` in `pins.py` is an estimate (1320). Measure it: turn the
  wheel one full revolution by hand and count.
- ⚠ Buck converter must be **≥ 5 A** (Pi 4 alone wants 3 A).
- ⚠ IMU must sit **≥ 100 mm from the motors and the speaker**, or the
  magnetometer reads their field instead of the earth's and heading is
  unusable. Test: spin the motors, wheels up — heading should not move.
  Run `imu_test.py --calibrate` with the robot fully assembled.
- ⚠ `CAM_HFOV` in `pins.py` defaults to 66° (Camera Module 3). The nav page
  draws this as the camera's field-of-view wedge on the LiDAR plot, so a
  wrong value means the wedge claims to cover ground the lens never sees.
  `python test/camera_test.py --list` names the sensor; set the matching
  figure from the table in `pins.py`.
- ⚠ `CAM_OFFSET_X/Y` and `CAM_YAW_OFFSET` are assumptions (front centre,
  looking straight ahead). Measure after mounting, like the LiDAR's.
- ⚠ Camera orientation — if the preview is upside down, set `CAM_HFLIP` and
  `CAM_VFLIP` in `pins.py`. Check with `camera_test.py --stream`.
- ⚠ `CAM_HEIGHT_MM` and `CAM_PITCH_DEG` are guesses (120 mm, 15° down). The
  person tracker's floor distance uses them. The camera floor check (cliff
  detection) was **removed from the guard** because, with these unmeasured
  and the camera tilted up, it read walls as drop-offs and stalled the truck.
  **Nothing detects stair edges now.**
- ⚠ `MARKER_SIZE_MM` must equal the size of the tags actually printed
  (black square only, measured with a ruler). Every marker distance scales
  linearly with it, and nothing else in the system can catch the error.
- ⚠ LCD orientation and colour — run `python test/display_test.py` and set
  `LCD_ROTATION`, `LCD_BGR`, `LCD_INVERT` in `pins.py` from what it shows.
  Module variants differ; the defaults suit the common IPS board.
- ⚠ MAX98357A `SD` pin — meter it against ground with Vin applied.
  **> 1.4 V** = mono, amp on. Near 0 V = the board has a pull-down and
  the amp is shut down until `SD` is jumpered to Vin. `WIRING.md` §7.

### Legacy

`throttle_relay.ino` — Arduino Mega 2560 sketch, throttle→relay with
hysteresis (A4 in, D7 out, active-low, 0.70/0.60 V, 50 ms dwell). Predates
the TB6612 design; kept for reference, not part of the current drive path.

> The Pi's IP has moved before (a note still says `.10`). If SSH fails,
> re-check with `arp -a` or use `shiv.local`. A static DHCP lease on the
> router would end this.

## Layout

```
Speaker_truck/
├── README.md             # START HERE — find the Pi, sync, run, edit
├── CLAUDE.md
├── Start_pi.md           # full bring-up runbook + test ladder
├── docs/
│   ├── CHANGES.md        # what the testing rounds found and changed, and how it was checked
│   ├── TUNING.md         # every variable that decides whether SLAM, guard, explorer work
│   ├── ARCHITECTURE.md   # coordinate frame, thread model, file ownership
│   └── TROUBLESHOOTING.md
├── WIRING.md             # architecture, power rails, ⚠ items to verify
├── PINOUT.md             # Pi 4 40-pin header diagram + this build's pins
├── requirements.txt
├── throttle_relay.ino    # legacy Arduino sketch (not in drive path)
├── test/                 # all hardware scripts live here
│   ├── pins.py           # GPIO map — single source of truth
│   ├── motor_test.py     # TB6612 × 2 differential drive
│   ├── speaker_test.py   # amp + speaker: tones, sweep, beeps, --play
│   ├── audio.py          # speaker player — tones, beeps, MP3 (shared library)
│   ├── tts.py            # text to speech, Piper neural voices (shared library)
│   ├── voice.py          # phone as microphone: /talk page over https :5443 (shared library)
│   ├── brain.py          # voice assistant: phone mic -> Ollama on the PC -> speaker
│   ├── truck_api.py      # AI-facing actions (look/drive/horn...) over web_nav HTTP — shared by brain + MCP
│   ├── imu.py            # 9-axis IMU driver (shared library)
│   ├── camera.py         # CSI camera driver (shared library)
│   ├── camera_test.py    # identify, time, and aim the camera
│   ├── display.py        # ST7789 LCD driver + status screen (shared library)
│   ├── display_test.py   # colours, orientation, speed, --ip
│   ├── markers.py        # ArUco absolute position fix (shared library)
│   ├── marker_test.py    # print tags, check detection range
│   ├── cliff.py          # camera floor check — no longer used by the guard
│   ├── tuning.py         # registry of every live-tunable value
│   ├── sysstats.py       # task-manager numbers from /proc — Pi System tab + PC containers
│   ├── calibrate.py      # measure scanner yaw + odometry scale by pushing
│   ├── detect.py         # YOLOv8n furniture labels pinned to the map
│   ├── person.py         # person tracker for follow mode: box, bearing, LiDAR/camera distance
│   ├── follow.py         # follow mode: lock one person (colours + motion), follow, search when lost
│   ├── slam.py           # occupancy grid + scan matching
│   ├── explore.py        # frontier explorer
│   ├── web_nav.py        # the cockpit — drive, LiDAR, IMU, camera, map, audio, LCD
│   ├── captures/         # stills saved from the camera — never synced back
│   ├── marker_map.json   # learned tag positions — Pi-side, never synced
│   ├── tuning.json       # values saved from the Tune tab — Pi-side
│   ├── uploads/          # songs uploaded from the Audio tab — Pi-side
│   ├── audio.json        # speaker device, volume, play mode — Pi-side
│   ├── voices/           # Piper voice models, ~60 MB each — Pi-side, never synced
│   ├── tts.json          # chosen voice, speed, recent phrases — Pi-side
│   ├── talk_cert.pem     # self-signed https cert for /talk, made on first run — Pi-side
│   ├── objects.json      # object labels: votes, and what a person confirmed — Pi-side
│   ├── asks/             # photos behind "is this a sofa?" questions — Pi-side, never synced
│   └── yolov8n.onnx      # detection model — copied in, never synced
├── .dockerignore         # keeps the root's .pt weights out of the Docker build
├── tools/                # runs on the PC, not the Pi
│   ├── detect_server.py  # bigger detection model, reached over the LAN
│   ├── tts_server.py     # Piper speech on the PC, streamed to the Pi's speaker
│   ├── Dockerfile        # one image for tts_server + detect_server
│   ├── docker-compose.yml # `docker compose up -d --build` — Ollama stays native
│   └── truck_mcp.py      # MCP server for Claude Code — thin wrapper over test/truck_api.py
└── .venv/                # Python venv — created ON THE PI, never synced
```

- `test/` — every hardware-touching script. GPIO, motor, audio, sensors.
- `.venv/` — Pi-only. See below.

## Python venv

The venv is created **on the Pi**, not on Windows. A Windows venv contains
`Scripts\*.exe` and Windows paths; it is not portable to ARM Linux and will
not work if copied. It is excluded from sync for this reason.

Create it once, on the Pi. **Note `--system-site-packages`** — it is required,
not optional:

```bash
cd ~/Desktop/Speaker_truck
sudo apt update
sudo apt install -y python3-gpiozero python3-lgpio
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
pip install -r requirements.txt
```

`requirements.txt` is authored on Windows and synced like any other file.

### GPIO libraries come from apt, not pip

`gpiozero` and `lgpio` are installed with `apt`, and the venv sees them via
`--system-site-packages`. Do **not** put them in `requirements.txt`.

`pip install lgpio` compiles the C extension from source and fails with
`error: command 'swig' failed` unless `swig` and `python3-dev` are present.
The apt package is already built, is the version Raspberry Pi OS tests
against, and needs no toolchain. Same reasoning for anything else touching
hardware: prefer `python3-<pkg>` from apt.

Check the venv can see them:

```bash
python -c "import gpiozero, lgpio; from importlib.metadata import version; print('ok', version('gpiozero'))"
```

## Syncing Windows → Pi

Run from the **Windows** terminal. `scp`/`rsync` always runs on the machine
that owns the local path — running it inside an SSH session makes `C:` look
like a hostname and fails.

```powershell
cd "C:\Users\vibhe\OneDrive\Desktop"
scp -r Speaker_truck\test shiv@192.168.1.11:/home/shiv/Desktop/Speaker_truck/
```

Better, if `rsync` is available (Git Bash) — skips the venv and caches:

```bash
rsync -av --exclude '.venv' --exclude '__pycache__' \
  ~/OneDrive/Desktop/Speaker_truck/ shiv@192.168.1.11:~/Desktop/Speaker_truck/
```

Then run on the Pi:

```bash
cd ~/Desktop/Speaker_truck && source .venv/bin/activate && python test/<script>.py
```

## Conventions

- Hardware scripts go in `test/`, one concern per file.
- Every script that drives an output must set a **safe state before**
  `pinMode`/setup and on exit — the relay clicks ON at boot otherwise. The
  Arduino sketch already does this; Python scripts must too.
- Wrap hardware loops in `try/finally` and release GPIO in the `finally`.
- Never commit or sync `.venv/`, `__pycache__/`, or `*.pyc`.
