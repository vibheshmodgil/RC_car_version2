# Testing everything, end to end

A step-by-step checklist to confirm the whole car actually works, from
first power-on to driving. Written for someone who hasn't necessarily
written the code — each step says what to do and exactly what
success/failure looks like. Do them **in order**: each one either
unblocks or explains the next, so skipping ahead just makes failures
confusing.

If you haven't flashed the boards yet, do that first:
[`CarTestBench/README.md`](CarTestBench/README.md),
[`CamStreamer/README.md`](CamStreamer/README.md), and
[`pi/README.md`](pi/README.md).

## 0. Network sanity check

All three boards should be stations on your home WiFi router
(`Airtel_kuma_9602`), same as any phone or laptop in the house — no
special "car WiFi" to connect to.

| Board | Static IP |
|---|---|
| Raspberry Pi 4 | 192.168.1.50 |
| ESP32 DevKit | 192.168.1.51 |
| ESP32-CAM | 192.168.1.52 |

Power everything on, wait about 30 seconds (the Pi is the slowest to
boot), then from any device on the home WiFi (a laptop is easiest):

```bash
ping 192.168.1.50   # Pi
ping 192.168.1.51   # DevKit
ping 192.168.1.52   # CAM
```

**Pass:** all three reply. **Fail:** a board that doesn't reply either
isn't powered, isn't flashed with the current home-WiFi config, or
failed to join WiFi — check its Serial Monitor output (see that board's
own README) before going further. Don't move on until all three ping.

## 1. DevKit control path (no sensors involved yet)

From the Pi (SSH in first — see `pi/README.md` if you need a refresher):

```bash
cd ~/car/pi && source .venv/bin/activate
python smoke/estop.py
```

**Pass:** prints a successful e-stop request/response — confirms the Pi
can reach the DevKit's REST API at all. **Fail:** check `ping
192.168.1.51` again, and check the DevKit's own Serial Monitor for
errors.

## 2. DevKit telemetry stream

```bash
python smoke/telemetry.py
```

**Pass:** prints telemetry updates arriving repeatedly (~10 per second).
**Fail:** the one-shot REST call in step 1 working but this failing
usually means a WebSocket-specific issue — check the DevKit's Serial
Monitor for WebSocket errors.

## 3. Camera stream

```bash
python smoke/camera_read.py
```

**Pass:** reports frames read successfully with an FPS number.
**Fail:** confirm `http://192.168.1.52/status` loads in a browser first
(isolates network from OpenCV-specific issues). If the CAM disconnects
after a minute or two, it's almost certainly the power-supply issue
documented in `CamStreamer/README.md` — use a real 5V/2A+ supply, not
the flashing adapter.

## 4. IMU (BNO055)

```bash
python smoke/imu_read.py
```

**Pass:** prints heading/roll/pitch values that change when you tilt the
board. **Fail:** run `i2cdetect -y 1` first — the BNO055 should show up
at address `0x28`. If it doesn't, it's a wiring/I2C problem, not code.

## 5. LiDAR (YDLIDAR X2)

```bash
python smoke/lidar_scan.py
```

**Pass:** prints distance readings as the LiDAR spins. **Fail:** check
`ls /dev/ttyUSB*` shows the device; if missing, check the USB cable and
that the LiDAR's motor is actually spinning.

## 6. Gimbal servos

**Clear the gimbal's motion path first — it will move.**

```bash
python smoke/gimbal_sweep.py
```

**Pass:** both pan and tilt servos visibly sweep through their range.
**Fail:** confirm `python3-lgpio` is installed (`python3 -c "import
lgpio"` inside the venv should not error) and that nothing else is
holding the pan/tilt GPIO pins open.

## 7. Combined status loop

Only after all six smoke tests above are individually green:

```bash
python main.py
```

**Pass:** runs without crashing, shows status for every subsystem at
once.

## 8. Dashboard

```bash
sudo .venv/bin/python -m uvicorn webapp.server:app --host 0.0.0.0 --port 80
```

From any phone/laptop on the home WiFi, browse to `http://192.168.1.50/`.

**Pass:** the dashboard loads, shows live telemetry, embedded camera
stream, and sensor panels (any genuinely-missing sensor just shows as
offline — that's fine, it shouldn't block the rest of the page). Try
arming and, wheels off the ground, a short hold-to-drive test on the
D-pad, and confirm the e-stop button actually stops it.

## 9. First real drive test

**Wheels off the ground.** Arm, drive each direction briefly from the
dashboard, confirm:
- Motors respond in the expected direction for each D-pad button.
- Releasing the D-pad stops the wheels within about half a second (the
  drive deadman — see `CLAUDE.md`).
- The e-stop button cuts all motors immediately, from any state.

Only after this passes should the car go on the ground and actually
drive.
