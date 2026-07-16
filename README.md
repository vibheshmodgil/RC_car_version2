# RC Car V2

4WD skid-steer RC car with a three-board, Pi-centric architecture:

| Board | Role | IP |
|---|---|---|
| Raspberry Pi 4 | brain: WiFi AP, main dashboard, BNO055 IMU, YDLIDAR X2, gimbal | 192.168.4.1 |
| ESP32 DevKit V1 | motor controller: 4x TB6612FNG + 4 encoders, REST/WS API, e-stop authority, fallback UI | 192.168.4.5 |
| ESP32-CAM | MJPEG streaming appliance (video goes direct, never proxied) | 192.168.4.10 |

Connect a phone to the `RC_Car_TestBench` WiFi and open
`http://192.168.4.1/` — e-stop, arm, hold-to-drive D-pad, per-wheel
PWM/RPM, IMU, live camera, gimbal and LiDAR on one dark dashboard.

## Where everything goes

This repo holds code for **three physically different devices with three
different toolchains**. The one thing to get right before touching any
file is *which board does it run on* — nothing here shares a runtime.

| Folder | Runs on | Toolchain | What belongs here |
|---|---|---|---|
| `pi/` | Raspberry Pi 4 | Python 3 (venv) | The dashboard (`webapp/`), every sensor driver (IMU, LiDAR, camera health, gimbal), the ESP32 REST/WS client, smoke tests. Anything that's plain Python and doesn't get flashed onto a chip. |
| `CarTestBench/` | ESP32 DevKit V1 | Arduino-ESP32 core 3.x (C++, `.ino`) | Motor control (4x TB6612FNG), encoders, the REST/WS API, the fallback web UI, the drive deadman, e-stop. Anything that touches the car's motor/encoder pins. |
| `CamStreamer/` | ESP32-CAM module | Arduino (C++) | The MJPEG streamer firmware only. Nothing else runs on this board — keep it that way (see "Camera" below). |
| repo root | — (docs, not code) | Markdown / PDF | Architecture docs, `CLAUDE.md`, `LICENSE`. No device-specific code belongs at the root. |

Deploying each one:

- `pi/` → `scp -r pi/ pi@<pi-ip>:~/car/` or `git pull` on the Pi (see `pi/README.md`).
- `CarTestBench/` → open `CarTestBench/CarTestBench.ino` in Arduino IDE, select an ESP32 DevKit board, flash over USB.
- `CamStreamer/` → open `CamStreamer/CamStreamer.ino`, board = "AI Thinker ESP32-CAM", GPIO0 strapped to GND to flash.

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
Pi's OpenCV capture both pull straight from `http://192.168.4.10:81/stream`.

## Docs

- `Hardware Architecture - Pi Centric.md` — **current**: topology, IP
  plan, AP setup, deadman safety, migration order (PDF alongside).
- `Hardware Architecture - Pi Integration Phase.md` — superseded network
  topology; still the wiring reference for BNO055/gimbal/power (its
  LiDAR section was corrected to the real hardware, YDLIDAR X2).
- `Hardware Architecture - TB6612FNG Rework.md` — motor driver wiring.
- `CLAUDE.md` — conventions: wheel order LF/LR/RF/RR, pin map,
  API contract, change rules.

## Safety model (outermost first)

1. STBY pull-downs keep the H-bridges dead until firmware arms a board.
2. E-stop latch: `POST /api/estop` from any UI or script.
3. Firmware drive deadman: wheels coast if drive commands stop for 500 ms.
4. Clients re-send held commands every 300 ms.

First runs: wheels off the ground.
