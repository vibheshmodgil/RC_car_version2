# CLAUDE.md

Guidance for Claude Code when working in this repository.

## Project Overview

Advanced RC Car V2 — **home-WiFi architecture** (see
`docs/hardware-architecture/v4-home-wifi-current.md`, which supersedes the
Pi Centric and Pi Integration Phase docs, both archived in the same
folder). All three boards join the house WiFi router directly as
stations — there is no more Pi-hosted AP; that was retired 2026-07-20
because it was the main source of flaky bring-up:

- **Raspberry Pi 4 = brain**: station on the home WiFi router at
  192.168.1.50, serves the main dashboard (`pi/webapp`, FastAPI on
  port 80, merged telemetry WS), owns BNO055 (I2C, raw smbus2), YDLIDAR X2 (USB, raw serial parser)
  and 2 gimbal servos (GPIO18/19), and forwards control to the DevKit.
- **ESP32 DevKit = motor controller**: 4 TB6612FNG channels + 4 quadrature
  encoders, REST/WS API (stable contract), its own web UI kept as a debug
  fallback at 192.168.1.51, e-stop/safety authority, 800 ms drive deadman.
  Joins the home WiFi router as a station (`WIFI_STATION_MODE` in
  `config.h`; `0` restores the legacy self-hosted-AP escape hatch, only
  needed if the car ever runs away from home WiFi coverage).
- **ESP32-CAM = streaming appliance**: MJPEG at 192.168.1.52:81, untouched
  by the migration; video is never proxied through the DevKit or the Pi
  for plain viewing.

IP plan: home router .1, Pi .50, DevKit .51, CAM .52. All hardcoded static
IPs on the house WiFi (`Airtel_kuma_9602`) — this car only ever runs at
home, so phones/laptops never need to switch networks to reach it.

Master MCU: ESP32 DevKit V1, Arduino-ESP32 core 3.x.
Sketch entry point: `CarTestBench/CarTestBench.ino`.

Repo layout: `CarTestBench/` (DevKit firmware), `CamStreamer/` (ESP32-CAM
firmware), `pi/` (Python that runs on the Raspberry Pi 4 — see `pi/README.md`;
it is deployed to the Pi with scp/git, never flashed to an ESP32).

## Build / Flash

This is an Arduino sketch, not PlatformIO or CMake.

- Open the `CarTestBench/` folder in Arduino IDE or build with `arduino-cli`.
- Use an ESP32 board package with Arduino-ESP32 core 3.x.
- Serial monitor baud: 115200.
- Required libraries: ESPAsyncWebServer, AsyncTCP, ArduinoJson.
- Keep ESP32-local sensor feature flags off unless the pin map is revised.

## Hardware Model

The car is 4WD with one TB6612FNG channel per wheel:

- TB6612 #1 LEFT board: channel A = Left-Front, channel B = Left-Rear.
- TB6612 #2 RIGHT board: channel A = Right-Front, channel B = Right-Rear.

Index order is always:

- `0 = LF`
- `1 = LR`
- `2 = RF`
- `3 = RR`

That order is shared by `PINS_MOTOR`, `PINS_ENCODER`, REST keys
`lf|lr|rf|rr`, and telemetry arrays.

## Motor Wiring

The firmware now uses standard TB6612FNG PWM wiring. Do not tie PWMA/PWMB to
3.3 V for this sketch.

Each motor channel has:

- `IN1` direction pin.
- `IN2` direction pin.
- `PWM` speed pin driven by LEDC.
- Shared board `STBY` pin.

Motor behavior:

- Forward: `IN1=HIGH`, `IN2=LOW`, `PWM=duty`.
- Reverse: `IN1=LOW`, `IN2=HIGH`, `PWM=duty`.
- Safe stop: coasts by default while wiring is being shaken down.
- Active brake, only if `MOTOR_ACTIVE_BRAKE_ENABLED=true`:
  `IN1=HIGH`, `IN2=HIGH`, `PWM=255`.
- Coast: `IN1=LOW`, `IN2=LOW`, `PWM=0`.

The signed PWM convention is `-255..+255`:

- Positive = forward.
- Negative = reverse.
- Zero = coast.

Each TB6612 board has a separate STBY line. STBY is LOW at boot and whenever
neither channel on that board is armed. Add a roughly 10 kOhm pull-down from
each STBY to GND so the bridges stay disabled while the ESP32 boots.

## Current Pin Map

Pins live in `CarTestBench/config.h`. Do not scatter pin numbers elsewhere.

Motor pins:

| Wheel | IN1 | IN2 | PWM | STBY |
|---|---:|---:|---:|---:|
| Left-Front | 18 | 19 | 5 | 12 |
| Left-Rear | 26 | 27 | 32 | 12 |
| Right-Front | 16 | 17 | 33 | 2 |
| Right-Rear | 13 | 15 | 0 | 2 |

Encoder pins:

| Wheel | A | B |
|---|---:|---:|
| Left-Front | 34 | 35 |
| Left-Rear | 22 | 21 |
| Right-Front | 4 | 23 |
| Right-Rear | 25 | 14 |

Shared bus:

- MPU/LiDAR/ToF sensor work should happen on the Raspberry Pi.
- The Raspberry Pi should connect to the ESP32 over WiFi. GPIO21/GPIO22 are
  now used by the Left-Rear encoder, so there is no spare wired UART pin in
  this pin map.

Reserved pins:

- GPIO1/GPIO3 are reserved for UART0 flashing and serial monitor.
- GPIO36/GPIO39 are not used because they are not exposed on this 38-pin board.
- With dedicated TB6612 PWM, 4 encoders, and the Pi link, there are no normal
  spare GPIOs left on this ESP32 pin map.

Important wiring notes:

- GPIO34/35 are input-only and have no internal pull-ups.
- Use external pull-ups to 3.3 V on all encoder A/B lines.
- Encoders must run from 3.3 V, not 12 V.
- GPIO0 and GPIO2 are boot strapping pins; the STBY pull-downs are part of the
  safety model, so keep motors disabled until firmware arms a board.

## Architecture

Modules are single-purpose and wired together in the `.ino`.

| Module | Responsibility |
|---|---|
| `MotorChannel` | One TB6612 channel: arm/disarm, direction, PWM, brake/coast, shared-STBY management, e-stop latch |
| `EncoderReader` | Four quadrature encoders using `attachInterruptArg`; global singleton `Encoders` |
| `Imu` | Disabled ESP32-local BNO055 groundwork; sensors are now planned on Pi |
| `AppServer` | WiFi (station on the home WiFi router, or legacy self-hosted AP), HTTP pages, REST API, WebSocket telemetry, drive deadman |
| `WebUI.h` | All HTML/CSS/JS (fallback UI; the main dashboard lives in `pi/webapp`) |
| `config.h` | Pins, feature flags, PWM/encoder defaults, WiFi mode + IPs, deadman timeout |

Pi-side modules live in `pi/` (`esp32_link.py`, `imu.py`, `lidar.py`,
`gimbal.py`, `camera.py`, `webapp/` with `hub.py` + `server.py` +
`static/index.html`). `pi/config.py` mirrors `config.h` — keep the IPs and
SSID in sync across both and `CamStreamer.ino`.

## REST And Telemetry

The DevKit's REST/WS API is a **stable contract** — the Pi dashboard, the
fallback UI and `pi/esp32_link.py` all depend on it. Extend it, never
break it.

**Drive deadman:** if any wheel has nonzero PWM and no `/api/drive` or
`/api/motor?...pwm=` command arrives for `DRIVE_DEADMAN_MS` (800 ms), all
wheels coast (still armed) and Serial logs it. Every UI/script that holds
a nonzero PWM must re-send it every ~150 ms (`DEADMAN_RESEND_S`). These
were widened/shortened from 500ms/300ms on 2026-07-20 after measuring real
Pi-to-DevKit WiFi jitter on the home router spiking past 300ms (and 5%
packet loss) at idle — the original numbers were tripping on ordinary
jitter, not just genuine link loss, and looked like stuttering motors.

Pages (fallback UI on the DevKit; the Pi serves the main dashboard at
`http://192.168.1.50/`):

- `/`
- `/motors`
- `/drive`
- `/encoders`
- `/imu`
- `/camera`

Motor API:

- `POST /api/motor?ch=lf|lr|rf|rr|all&arm=1|0`
- `POST /api/motor?ch=lf|lr|rf|rr|all&pwm=-255..255`
- `POST /api/motor?ch=lf|lr|rf|rr|all&mode=brake|coast|disarm`
- `POST /api/motor?ch=lf|lr|rf|rr|all&inv=1|0` — software direction invert per
  wheel (fixes reversed wiring); persisted to flash as `invMask`.
- `POST /api/estop`
- `POST /api/estop/clear`

Drive API (whole-car motion, wheels must already be armed):

- `POST /api/drive?dir=fwd|rev|left|right|stop&pwm=0..255`
- `left`/`right` are in-place skid-steer spins (sides run opposite ways).

Encoder API:

- `POST /api/encoder/reset?ch=all|0..3`
- `GET /api/encoder/cpr`
- `POST /api/encoder/cpr` with JSON `{ "cpr": number }`

Telemetry pushed on `/ws` at 10 Hz:

```json
{
  "up": 0,
  "estop": false,
  "m": [0, 0, 0, 0],
  "a": [false, false, false, false],
  "inv": [false, false, false, false],
  "enc": [{ "c": 0, "r": 0 }],
  "imu": { "en": false, "ok": false, "h": 0, "r": 0, "p": 0 }
}
```

Keep DOM IDs, page JavaScript, and these JSON field names in sync.

## Camera (ESP32-CAM)

A second, separate board runs `CamStreamer/CamStreamer.ino` (AI-Thinker
ESP32-CAM). Architecture rule: **video never passes through the main ESP32**.
The CAM joins the home WiFi router directly as a station with static IP
`CAM_HOST` (192.168.1.52) and serves:

- `http://<CAM_HOST>:81/stream` — MJPEG (embedded by the `/camera` page and
  consumable by OpenCV on the Raspberry Pi later).
- `http://<CAM_HOST>/capture` — single JPEG.
- `http://<CAM_HOST>/control?var=framesize|quality|vflip|hmirror&val=n`.
- `http://<CAM_HOST>/status` — JSON `{frames, bytes, ms, streaming}`
  cumulative counters; the `/camera` page polls at 1 Hz and derives
  FPS / bitrate / average frame size from the deltas.

`AP_SSID`/`AP_PASS`/the static IP are duplicated at the top of
`CamStreamer.ino` (a sketch cannot include a sibling sketch's header) — keep
them in sync with `config.h` (`AP_SSID`, `AP_PASS`, `CAM_HOST`).

CamStreamer build: board "AI Thinker ESP32-CAM", flashed via a USB-serial
adapter with GPIO0 strapped to GND. Needs a 5 V >= 2 A supply.

## Pi Integration Status (read before resuming Pi work)

The `pi/` code (config, ESP32 link, camera, IMU, LiDAR, gimbal, smoke
tests, FastAPI dashboard) is written and merged to `main`, but **it has
not been confirmed working end-to-end on the real car** — treat it as
untested until each smoke test in `pi/README.md` has actually been run
and passed on hardware, not as a finished feature.

**2026-07-20 architecture pivot:** the Pi-hosted WiFi AP (`pi/setup_ap.sh`,
`RC_Car_TestBench` @ 192.168.4.1) is retired. It was the main source of
flaky bring-up (see the WiFi-channel debugging thread in
`pi/RESUME_GUIDE.md`), and the one thing that *did* work reliably was the
CAM joining the home WiFi directly. All three boards now join the home
router (`Airtel_kuma_9602`) as stations with static IPs — see
`docs/hardware-architecture/v4-home-wifi-current.md`. The Pi's WiFi is
pinned to its static IP with the new `pi/setup_wifi.sh` (replaces
`setup_ap.sh`). The whole "is the AP up, did the CAM find it" dance is
gone — every board just needs to join the same WiFi network everything
else in the house is already on.

Not implemented despite being mentioned in older docs: the VL53L0X ToF
sensor (still `ENABLE_VL53L0X 0`, no Pi driver). There is no physical
screen on the car — "the screen" is the phone/browser dashboard in
`pi/webapp`.

If picking this up after a break, start at `pi/RESUME_GUIDE.md` — it
covers SSH from scratch, how to find the Pi on the network, and the
order to re-verify each subsystem in.

## Change Rules

- Hardware pins and tunables belong in `config.h`.
- Keep optional hardware behind feature flags so the base sketch still builds.
- Use non-blocking `millis()` timing in `loop()`, not `delay()`.
- Do not bring back BTS7960/IBT-2 code.
- Do not add AHT20 or MPU6050 support; they are intentionally excluded.
- Be careful with TB6612 current limits: ramp PWM gently and never hold a
  stalled wheel at high duty.
