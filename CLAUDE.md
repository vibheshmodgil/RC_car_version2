# CLAUDE.md

Guidance for Claude Code when working in this repository.

## Project Overview

ESP32 firmware for the Advanced RC Car V2 test bench. The ESP32 runs its own
WiFi access point and serves phone-friendly pages for testing each subsystem.

Current scope:

- Motors: 4 independent TB6612FNG channels, one per wheel.
- Encoders: 4 quadrature encoders, one per wheel.
- IMU/LiDAR/ToF: on the Raspberry Pi side, not wired to this ESP32 pin map.
- Pi integration phase (Pi 4 + BNO055 on I2C + RPLidar A1/C1 on USB + 2
  gimbal servos on Pi GPIO18/19): see
  `Hardware Architecture - Pi Integration Phase.md`. Pi joins the AP as a
  station at 192.168.4.20 and uses the existing REST/WS/stream interfaces;
  nothing new connects to the ESP32.

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
| `AppServer` | WiFi AP, HTTP pages, REST API, WebSocket telemetry |
| `WebUI.h` | All HTML/CSS/JS |
| `config.h` | Pins, feature flags, PWM/encoder defaults, AP credentials |

## REST And Telemetry

Pages:

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
The CAM joins the DevKit's AP as a station with static IP `CAM_HOST`
(192.168.4.10) and serves:

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

## Change Rules

- Hardware pins and tunables belong in `config.h`.
- Keep optional hardware behind feature flags so the base sketch still builds.
- Use non-blocking `millis()` timing in `loop()`, not `delay()`.
- Do not bring back BTS7960/IBT-2 code.
- Do not add AHT20 or MPU6050 support; they are intentionally excluded.
- Be careful with TB6612 current limits: ramp PWM gently and never hold a
  stalled wheel at high duty.
