# Hardware Architecture - Home WiFi (v4, current)

Status: **current architecture**, as of 2026-07-20. Supersedes
`v3-pi-centric-ap.md` (the Pi used to host its own WiFi access point) and,
through it, `v2-pi-integration-phase.md`. This doc is the one to read
first; v1-v3 in this folder are history only.

## Why this changed

The Pi hosting its own WiFi network (`RC_Car_TestBench` at 192.168.4.1)
was the single biggest source of trouble getting the car working:
picking a bad/noisy WiFi channel, the ESP32-CAM failing to join reliably,
and every debugging session having to first figure out "is the Pi in AP
mode or on home WiFi right now" before anything else could be tested (see
the old `pi/RESUME_GUIDE.md` troubleshooting log). The one thing that
*did* just work was the CAM joining the home WiFi network directly.

So the car now uses the house WiFi as the one and only network. Every
board is a plain WiFi station on the same router the rest of the house
uses. There is no more "switch your phone to the car's WiFi" step — if
your phone is on home WiFi, it can already reach the car.

This only works because the car always operates inside the house. If it
ever needs to run somewhere without that WiFi, the DevKit still has a
`WIFI_STATION_MODE 0` escape hatch back to a self-hosted AP (see
`CarTestBench/config.h`) — not used day-to-day.

## System Topology

Three compute boards, all stations on the home WiFi router.

```
                     Home WiFi router: Airtel_kuma_9602
                               192.168.1.1
                                    |
      +---------------+------------+-------------+
      |               |                           |
    Phone       ESP32 DevKit                 ESP32-CAM
  (browser)    192.168.1.51 (station)      192.168.1.52 (station)
      |        motors + encoders                MJPEG stream
      |        REST + WS telemetry                   |
      |        own UI = debug fallback                |
      |                                                |
      +--> http://192.168.1.50/  <-- Raspberry Pi 4 (station, .50)
           main dashboard             FastAPI + merged telemetry WS
                                       |         |         |
                                    BNO055    YDLIDAR   2x gimbal
                                    (I2C)     (USB)     servos (GPIO18/19)
```

- The **Pi** is just another device on the home network, running the main
  web dashboard (`pi/webapp`, port 80): e-stop, arm, hold-to-drive D-pad,
  per-wheel PWM/RPM, IMU, embedded CAM stream, gimbal sliders, LiDAR polar
  plot — one merged telemetry WebSocket at 10 Hz.
- The **DevKit** keeps its full REST/WS API (a stable contract) and its
  built-in web UI at `http://192.168.1.51` as a debug fallback. It
  remains the e-stop authority: STBY pull-downs cut the H-bridges in
  hardware.
- The **CAM** is also just a station on the home router now. Browsers and
  the Pi pull MJPEG straight from it; video never passes through the
  DevKit and is never proxied through the Pi for plain viewing.

## Network Plan

| Device | IP | How |
|---|---|---|
| Home WiFi router (gateway) | 192.168.1.1 | not ours — the house router |
| Raspberry Pi 4 (dashboard) | 192.168.1.50 | static, `pi/setup_wifi.sh` (NetworkManager) |
| ESP32 DevKit | 192.168.1.51 | static, `STA_STATIC_IP` in `CarTestBench/config.h` |
| ESP32-CAM | 192.168.1.52 | static, `CAM_IP` in `CamStreamer.ino` |
| Phones / laptops | whatever the router's DHCP hands out | normal home WiFi devices |

SSID `Airtel_kuma_9602`, password `air71417` — the real home WiFi
credentials, hardcoded in all three boards' firmware/config. This is
acceptable because the car only ever runs at home; if the home WiFi
password ever changes, all three files below need updating together:

- `CarTestBench/config.h` (`AP_SSID`, `AP_PASS`)
- `CamStreamer/CamStreamer.ino` (`AP_SSID`, `AP_PASS`)
- `pi/setup_wifi.sh` (`SSID`, `PSK`)

**Static IP note:** these three IPs are picked outside the range the
router is likely to hand out over DHCP, but they aren't formally reserved
in the router's admin page. In the unlikely event the router assigns
`.50`/`.51`/`.52` to some other device (a new phone, a smart-home
gadget), a static IP conflict is easy to spot — the car's dashboard/API
suddenly stops answering even though the board's serial log or LEDs show
it's alive and joined. Fix: either give that other device a different
address, or pick fresh unused static IPs and update all three configs
above. A proper fix is a DHCP reservation for these three MAC addresses in
the router's admin UI — worth doing once if this ever happens.

## WiFi Setup (on the Pi, one time)

```bash
cd ~/car/pi
bash setup_wifi.sh
```

The script (NetworkManager, stock on Bookworm):

1. Removes any old `car-ap` profile from a previous Pi-hosted-AP setup.
2. Creates (if missing) or reuses the home-WiFi connection profile.
3. Pins it to `ipv4.method manual`, static address 192.168.1.50/24,
   gateway/DNS 192.168.1.1.

If the SD card was imaged with Raspberry Pi Imager's WiFi preset (SSID +
password set at imaging time), the Pi already joins home WiFi over DHCP
on first boot — `setup_wifi.sh` just switches it to the fixed address.
The Pi has normal internet access the whole time now (unlike the old AP
mode, where being the AP meant no internet).

## Firmware Roles and Modes

| Board | Sketch | Role |
|---|---|---|
| ESP32 DevKit | `CarTestBench/` | motor controller: 4x TB6612 channels, 4 encoders, REST/WS API, fallback UI, e-stop latch, drive deadman |
| ESP32-CAM | `CamStreamer/` | MJPEG appliance at :81/stream — untouched by this migration |
| Raspberry Pi 4 | `pi/` | dashboard (`webapp/`), BNO055 (raw smbus2), YDLIDAR X2 (raw serial), gimbal, logging, future autonomy |

`CarTestBench/config.h` → `WIFI_STATION_MODE`:

- `1` (current): `WIFI_STA`, static 192.168.1.51, gateway .1,
  `WiFi.setAutoReconnect` plus a 5 s non-blocking reconnect watchdog in
  `loop()`.
- `0`: legacy self-hosted AP — the escape hatch for running the car away
  from home WiFi. If used, `pi/config.py` and any phone connecting to the
  dashboard would need to match that AP's network instead.

## Drive Deadman (safety) — unchanged

`DRIVE_DEADMAN_MS = 800` in `config.h`. In the DevKit's `loop()`
(non-blocking `millis()` check): if **any wheel has nonzero PWM** and no
`/api/drive` or `/api/motor?...pwm=` command has arrived for 800 ms, all
wheels **coast** and the event is logged to Serial. Wheels stay armed —
the next command drives again.

Both UIs cooperate by re-sending the held command every 150 ms (Pi
dashboard D-pad, ESP32 drive page D-pad, ESP32 motors-page sliders). Any
Pi-side autonomy script must do the same (`DEADMAN_RESEND_S` in
`pi/config.py`).

**2026-07-20 update:** these were 500ms/300ms originally, widened after
measuring real Pi-to-DevKit latency on the home router: `ping` at idle
(no motors, no HTTP) showed jitter spiking past 300ms and ~5% packet
loss. The original 500ms window was tripping on that ordinary WiFi
jitter, not just genuine link loss, and looked like the motors
stuttering — spin, coast, spin again — under normal driving. If stutter
returns, check WiFi link quality first (`ping` the DevKit from the Pi)
before assuming it's a firmware or dashboard bug.

Layered safety, outermost first:

1. STBY pull-downs — bridges disabled in hardware until firmware arms.
2. E-stop latch — `POST /api/estop` from any UI or script; hardware STBY cut.
3. Drive deadman — dead client/link mid-drive stops the car in ≤ 0.5 s.
4. Client deadman re-send — a stalled control loop stops sending, which
   trips layer 3.

## Control and Data Paths

| From | To | What |
|---|---|---|
| Phone | Pi :80 | dashboard page + merged telemetry WS (`/ws`) |
| Pi webapp | DevKit :80 | control POSTs forwarded verbatim (`/api/drive`, `/api/motor`, `/api/estop...`) |
| Pi webapp | DevKit `/ws` | 10 Hz wheel telemetry (background thread) |
| Pi webapp | CAM `/status` | 1 Hz counters → FPS/bitrate in merged WS |
| Phone | CAM :81/stream | MJPEG, direct — never proxied |
| Pi (OpenCV, later) | CAM :81/stream | second direct stream for vision |

The DevKit REST/WS API is unchanged and remains the contract; the Pi
extends around it.

## Boot Order

No boot order dependency anymore — all three boards independently join
the same always-on home router whenever they power up, in any order,
same as any other WiFi device in the house.

## Migration Notes (from v3, the Pi-hosted-AP phase)

1. Flash `CarTestBench` and `CamStreamer` with the new home-WiFi
   credentials/static IPs (already the default in this repo as of
   2026-07-20).
2. Run `pi/setup_wifi.sh` on the Pi.
3. Confirm each board answers: `ping 192.168.1.50`, `ping 192.168.1.51`,
   `ping 192.168.1.52` from any device on the home network.
4. `pi/setup_ap.sh` no longer exists — deleted, not just disabled.

## What Changed vs Pi Centric (v3)

| | Pi Centric, v3 (superseded) | Home WiFi, v4 (current) |
|---|---|---|
| Network | Pi-hosted AP (`RC_Car_TestBench`) | House router (`Airtel_kuma_9602`) |
| Pi IP | 192.168.4.1 (AP) | 192.168.1.50 (station) |
| DevKit IP | 192.168.4.5 (station) | 192.168.1.51 (station) |
| CAM IP | 192.168.4.10 (station) | 192.168.1.52 (station) |
| Phone reach the car | must switch WiFi networks | already on the same network |
| Pi internet access | none while AP is active | always on |
| Boot order | Pi must come up first | no dependency |

Everything else — sensors, deadman, e-stop, REST/WS contract, wiring —
carries over unchanged from v3.
