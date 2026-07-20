# Resuming the Pi integration (start here if you forgot everything)

Last updated 2026-07-20. Read this before touching `pi/` again after a
break. It assumes zero memory of SSH, the Pi, or what was already tried.

## What's actually true right now

- **2026-07-20: architecture pivot.** The Pi no longer hosts its own
  WiFi network. All three boards (Pi, ESP32 DevKit, ESP32-CAM) now join
  the **home WiFi router** (`Airtel_kuma_9602`) directly as stations,
  with static IPs .50/.51/.52. See
  `../docs/hardware-architecture/v4-home-wifi-current.md` for the full
  picture and why (short version: the Pi-hosted AP's WiFi channel was a
  recurring source of flaky bring-up, documented in the now-historical
  entries below). Everything in this file *below* the progress log that
  talks about "the Pi's AP" or checking whether the Pi is in AP mode is
  obsolete — the Pi is just always on home WiFi now, same as your phone.
- The Pi is also being rebooted/re-provisioned from scratch as part of
  this change, so treat this as a fresh bring-up, not a resume of a
  half-working state.
- All the Pi-side *code* exists and is documented (`pi/README.md` is the
  reference for commands; this file is the "how do I get moving again"
  guide).
- **Real-hardware end-to-end operation has still not been confirmed** —
  a prior session's memory calling it "complete" only meant the code was
  written and merged, not that it was proven on the actual car. That is
  still the state after the WiFi pivot: proven code, unproven hardware
  run.
- Not built yet, don't assume otherwise: the **VL53L0X ToF sensor** is
  still just a disabled placeholder flag (`ENABLE_VL53L0X 0` in
  `CarTestBench/config.h`) — no Pi driver exists for it. There is also
  **no physical screen** on the car; "the screen" is the phone/browser
  dashboard served by `pi/webapp`.

## Step 0 — SSH and first boot, explained from zero

`pi/README.md` now has the full from-zero SSH walkthrough (what SSH is,
how to find the Pi, what to do if you don't remember the login). Read
that first if this is genuinely your first time — it's not duplicated
here to avoid the two copies drifting apart.

Short version for this rebuild: image the SD card with **Raspberry Pi
Imager**, use its gear-icon "advanced options" to set the username/
password and preset the home WiFi SSID/password (`Airtel_kuma_9602`) at
imaging time — that gets the Pi onto home WiFi with zero manual network
steps on first boot. Then `ssh <username>@raspberrypi.local` or find its
DHCP IP in the router's device list, and run `bash setup_wifi.sh` (see
`pi/README.md`) to pin it to the fixed `192.168.1.50`.

## Step 1 — look before touching anything

Don't assume last session's setup is intact, especially after a reboot.
Check, don't guess:

```bash
ls ~/car/pi                              # is the code even there?
ls ~/car/pi/.venv                        # was the venv created?
cd ~/car/pi && git status                # if it's a git checkout, what's the state?
sudo systemctl status car-dashboard      # was the boot service installed? is it running/crashing?
i2cdetect -y 1                           # is the BNO055 visible on the I2C bus at all?
ls /dev/ttyUSB* /dev/ttyACM*             # is the YDLIDAR X2 enumerating as a serial device?
```

Report back what each of these actually shows before assuming anything
works or is missing.

## Step 2 — smoke tests, one at a time, in the order they exist for a reason

`pi/README.md` already documents the exact commands. The **order
matters** — each test isolates one subsystem so a failure points at one
thing, not a tangle:

1. `smoke/estop.py` — can the Pi even talk to the ESP32 DevKit over WiFi
   at all (REST call, no sensors involved)? If this fails, nothing
   downstream matters yet — it's pure networking between the two boards.
2. `smoke/telemetry.py` — is the ESP32's WebSocket telemetry stream
   arriving at 10 Hz? Confirms the *ongoing* link, not just one request.
3. `smoke/camera_read.py` — is the ESP32-CAM's MJPEG stream reachable
   and readable via OpenCV? Fully independent of the DevKit and Pi
   sensors — it's its own WiFi station.
4. `smoke/imu_read.py` — is the BNO055 answering over I2C (raw register
   reads, no vendor library)? Needs `i2cdetect` to show the device first.
5. `smoke/lidar_scan.py` — is the YDLIDAR X2 spinning and producing
   parseable serial packets over USB?
6. `smoke/gimbal_sweep.py` — do the two pan/tilt servos move on command
   via lgpio? **Clear the gimbal's motion path before running this.**

Only after all six are individually green does `python main.py` (the
combined loop) or the `webapp` dashboard make sense to run — combining
subsystems before each one is independently proven just makes failures
ambiguous.

## Step 3 — what "resuming" means concretely, this time

We're not resuming a stuck AP debugging session anymore — that whole
problem class is gone with the WiFi pivot. The plan now:

1. Fresh-image or confirm the Pi's SD card, get it on home WiFi (Step 0).
2. Run `pi/setup_wifi.sh` to pin its static IP.
3. Flash `CarTestBench` and `CamStreamer` if not already done (they
   default to the home-WiFi credentials as of 2026-07-20 — see their own
   `README.md` files).
4. Confirm each board answers: `ping 192.168.1.51` (DevKit),
   `ping 192.168.1.52` (CAM) from the Pi.
5. Walk the smoke tests in order (Step 2), fixing whatever breaks, one
   subsystem at a time.
6. Only then move to the combined dashboard and, eventually, driving.

Update this file with what's learned at each step so the next resume —
yours or a future session's — doesn't start from zero again.

## Progress log

- **2026-07-20 — architecture pivot to home WiFi.** Retired the
  Pi-hosted AP (`pi/setup_ap.sh`, `RC_Car_TestBench` @ 192.168.4.1) after
  repeated flaky bring-up (see the channel-debugging entries below, which
  are now historical). All three boards join `Airtel_kuma_9602` (the
  house router) directly as stations: Pi .50, DevKit .51, CAM .52. New
  script `pi/setup_wifi.sh` replaces `setup_ap.sh`. The Pi is being
  rebooted/re-provisioned from scratch to pick this up cleanly — next
  session should start at Step 0 above, not assume any of the AP-era
  state below still applies.
- **2026-07-19 — AP + SSH confirmed working (historical, AP retired
  above).** Laptop joined `RC_Car_TestBench` and SSH into the Pi
  succeeded.
- **2026-07-19 — IMU and LiDAR smoke-tested, both responding.** Still
  relevant — this was hardware-level (I2C/USB), unaffected by the WiFi
  architecture change. Re-confirm after the reboot, but no reason to
  expect it broke.
- **2026-07-19 — CamStreamer (ESP32-CAM) failed to join the Pi's AP
  reliably; root-caused to a power problem, not networking — still
  relevant.** The CAM was powered through the USB-serial flashing
  adapter, which can't supply enough current for WiFi TX spikes, causing
  ~26 failed join retries then a connect-then-crash loop. **Fix:** run it
  from a real 5V/2A+ supply direct to the board, not through the
  USB-serial adapter — now documented as a first-class step in
  `CamStreamer/README.md`. This lesson carries over unchanged to the new
  home-WiFi setup; verify the CAM stays connected for several minutes
  under the new config, not just seconds.
- **DevKit home-WiFi station join: not yet tested** on the new
  architecture. Do this early in the next session (Step 3.4 above) so any
  future WiFi-side problem isn't confused with the old AP issue or the
  CAM's power issue.
