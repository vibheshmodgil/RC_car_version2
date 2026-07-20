# RC Car V2

4WD skid-steer RC car with a three-board architecture. All three boards
join your **home WiFi router** directly — there's no separate car network
to switch your phone to.

| Board | Role | IP (home WiFi) |
|---|---|---|
| Raspberry Pi 4 | brain: main dashboard, BNO055 IMU, YDLIDAR X2, gimbal | 192.168.1.50 |
| ESP32 DevKit V1 | motor controller: 4x TB6612FNG + 4 encoders, REST/WS API, e-stop authority, fallback UI | 192.168.1.51 |
| ESP32-CAM | MJPEG streaming appliance (video goes direct, never proxied) | 192.168.1.52 |

If your phone or laptop is already on the house WiFi, just open
`http://192.168.1.50/` — e-stop, arm, hold-to-drive D-pad, per-wheel
PWM/RPM, IMU, live camera, gimbal and LiDAR on one dark dashboard.

**New to this project or coming back after a break?** Start with
[`TESTING.md`](TESTING.md) — it walks through powering everything on and
confirming each piece works, from scratch, assuming no prior context.

## Where everything goes

This repo holds code for **three physically different devices with three
different toolchains**. The one thing to get right before touching any
file is *which board does it run on* — nothing here shares a runtime.

| Folder | Runs on | Toolchain | What belongs here | Setup guide |
|---|---|---|---|---|
| `pi/` | Raspberry Pi 4 | Python 3 (venv) | The dashboard (`webapp/`), every sensor driver (IMU, LiDAR, camera health, gimbal), the ESP32 REST/WS client, smoke tests. | [`pi/README.md`](pi/README.md) |
| `CarTestBench/` | ESP32 DevKit V1 | Arduino-ESP32 core 3.x (C++, `.ino`) | Motor control (4x TB6612FNG), encoders, the REST/WS API, the fallback web UI, the drive deadman, e-stop. | [`CarTestBench/README.md`](CarTestBench/README.md) |
| `CamStreamer/` | ESP32-CAM module | Arduino (C++) | The MJPEG streamer firmware only. Nothing else runs on this board (see "Camera" below). | [`CamStreamer/README.md`](CamStreamer/README.md) |
| `docs/hardware-architecture/` | — (docs, not code) | Markdown / PDF | Network topology, IP plan, wiring, version history. | [`docs/hardware-architecture/README.md`](docs/hardware-architecture/README.md) |
| repo root | — (docs, not code) | Markdown | `CLAUDE.md`, `TESTING.md`, `LICENSE`. No device-specific code belongs at the root. | — |

If you've never flashed an ESP32 or SSH'd into a Raspberry Pi before,
each linked README above starts from zero — installing the tools,
wiring things up, and what "it worked" looks like.

### Adding a new Pi sensor or feature

Put the driver in `pi/<name>.py` (own class, opens its own hardware
connection, fails fast if the hardware isn't there — see `pi/imu.py`,
`pi/lidar.py`, `pi/gimbal.py` for the pattern). Wire it into
`pi/webapp/hub.py` (`Hub.__init__` opens it via `_try_open`, a
background loop or the device's own thread keeps `Hub.snapshot()`
current) and add its panel to `pi/webapp/static/index.html`. Add a
`pi/smoke/<name>_read.py` standalone test. Don't invent a second
dashboard framework — one FastAPI app, one merged `/ws` telemetry feed.

### Adding new ESP32 (DevKit) firmware

Pins and tunables go in `CarTestBench/config.h`, nowhere else. New
subsystems get their own `.h`/`.cpp` module (see `MotorChannel`,
`EncoderReader` for the pattern) and are wired together in
`CarTestBench.ino`. REST endpoints and telemetry fields go in
`AppServer.cpp`/`.h` — extend the JSON contract, never break it (the Pi
dashboard and the fallback UI both depend on it). See `CLAUDE.md` for
the wheel index order (LF/LR/RF/RR) and TB6612 safety rules.

### Camera board

`CamStreamer/` stays a dedicated MJPEG appliance. Don't add motor
control, sensors, or anything else to it, and don't route video through
either of the other two boards — the dashboard's `<img>` tag and the
Pi's OpenCV capture both pull straight from `http://192.168.1.52:81/stream`.

## Docs

- [`docs/hardware-architecture/`](docs/hardware-architecture/) — start at
  `v4-home-wifi-current.md`: topology, IP plan, WiFi setup, deadman
  safety. Older versions (Pi-hosted AP, pre-Pi phases) are kept there for
  history.
- [`TESTING.md`](TESTING.md) — end-to-end "does everything actually
  work" checklist, in order.
- `CLAUDE.md` — conventions: wheel order LF/LR/RF/RR, pin map,
  API contract, change rules.

## Safety model (outermost first)

1. STBY pull-downs keep the H-bridges dead until firmware arms a board.
2. E-stop latch: `POST /api/estop` from any UI or script.
3. Firmware drive deadman: wheels coast if drive commands stop for 800 ms.
4. Clients re-send held commands every 150 ms.

First runs: wheels off the ground.
