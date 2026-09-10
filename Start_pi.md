# Start the Pi — bring-up runbook

> **New to this project? Start with [README.md](README.md)** — it is the
> fifteen-minute path from a cold laptop to a moving robot. This file is the
> detailed runbook you come to when a specific step needs checking, and the
> test ladder you work down on a fresh build.

Connect, sync, then work down the test ladder. Each step isolates a different
part, so when something fails you know what it was.

> **WHEELS OFF THE GROUND** for everything that drives a motor. The TB6612s
> cannot survive a stall on these motors — JGB37-520 stalls at 4–5 A, the
> TB6612 peaks at 3.2 A. `WIRING.md` §8.

---

## Quick reference

```bash
ssh shiv@shiv.local
cd ~/Desktop/Speaker_truck && source .venv/bin/activate
python test/web_nav.py                       # everything, port 5004
```

```powershell
# from Windows — note the test/ on the END, or files land in the wrong place
scp "C:\Users\vibhe\OneDrive\Desktop\Speaker_truck\test\*.py" shiv@shiv.local:/home/shiv/Desktop/Speaker_truck/test/
```

---

## 1. Find and connect

```bash
ssh shiv@shiv.local
```

If the name does not resolve, the Pi's DHCP address has moved again. From
Windows:

```powershell
arp -a | findstr b8-27-eb          # the Pi's MAC prefix
ssh shiv@<the address it shows>
```

If SSH warns **REMOTE HOST IDENTIFICATION HAS CHANGED**, an old entry is
stale, not an attack:

```powershell
ssh-keygen -R shiv.local
```

> A static DHCP reservation for MAC `b8-27-eb-67-60-a7` on the router would
> end the address churn permanently. It has moved four times so far.

---

## 2. Sync from Windows

`scp` always runs on the machine that owns the local path, so run it in
**PowerShell, not inside the SSH session** — otherwise `C:` looks like a
hostname.

```powershell
# everything
scp "C:\Users\vibhe\OneDrive\Desktop\Speaker_truck\test\*.py" shiv@shiv.local:/home/shiv/Desktop/Speaker_truck/test/

# one file
scp "C:\Users\vibhe\OneDrive\Desktop\Speaker_truck\test\web_nav.py" shiv@shiv.local:/home/shiv/Desktop/Speaker_truck/test/

# root-level files (requirements.txt, docs) — no test/ on the end
scp "C:\Users\vibhe\OneDrive\Desktop\Speaker_truck\requirements.txt" shiv@shiv.local:/home/shiv/Desktop/Speaker_truck/
```

⚠ **The destination must end in `/test/` for scripts.** Dropping it puts them
in the project root, where `import pins` fails.

---

## 3. One-time setup

Only needed on a fresh install.

```bash
cd ~/Desktop/Speaker_truck
sudo apt update
sudo apt install -y python3-gpiozero python3-lgpio \
                    alsa-utils ffmpeg \
                    python3-serial \
                    i2c-tools python3-smbus2 \
                    python3-picamera2 python3-opencv

python3 -m venv --system-site-packages .venv    # --system-site-packages is required
source .venv/bin/activate
pip install -r requirements.txt
```

Enable I2C for the IMU:

```bash
sudo raspi-config          # Interface Options -> I2C -> enable
sudo reboot
```

The camera needs **no** equivalent step — Trixie auto-detects the official
modules. Confirm with `rpicam-hello --list-cameras`. Third-party modules need
their own `dtoverlay`; `WIRING.md` §12.

Audio overlay, in `/boot/firmware/config.txt` (already done on this Pi):

```
dtparam=audio=off
dtoverlay=max98357a,no-sdmode
```

---

## 4. Every session — the three lines

```bash
cd ~/Desktop/Speaker_truck          # project ROOT, not test/
source .venv/bin/activate           # the venv lives at the root
python test/<script>.py             # always with the test/ prefix
```

You are ready when the prompt reads:

```
(.venv) shiv@shiv:~/Desktop/Speaker_truck $
```

`(.venv)` present, path ending in `Speaker_truck` with no `/test`. The
activation does **not** survive a reconnect — redo it each session.

---

## 5. The test ladder

Work down it. Stop at the first failure.

### 5.1 GPIO pins — no motor power

```bash
python test/pin_hold.py forward        # holds one state until Ctrl-C
```

Battery **disconnected**. Meter each pin against ground and compare with the
table it prints. Every pin should read 3.3 V or 0.0 V — nothing between.

> A pin sitting at 0.6–1.2 V instead of 3.3 V means the driver is unpowered
> and the GPIO is pushing current through its input clamp diode. Fix the
> driver's VCC before anything else.

Also confirm before trusting any reading:
- driver `GND` ↔ Pi `GND` — continuity beep
- driver `VCC` — 3.3 V

### 5.2 Motors — logic only, then real

```bash
python test/motor_test.py --logic      # VM DISCONNECTED, steps every 3 s
python test/motor_test.py              # guided sequence, VM connected
```

The ramp at the end shows the duty at which the wheels actually break static
friction — a number worth knowing.

### 5.3 Encoders

Turn a wheel by hand and watch the counts move. In any web UI, or:

```bash
python test/web_dashboard.py           # :5000, counts + RPM live
```

**Measure `COUNTS_PER_REV` while you are here.** Reset the encoders, turn one
wheel exactly one revolution by hand, read the count. That number replaces
the 1320 estimate in `test/pins.py`. Odometry is wrong by that ratio until
you do.

### 5.4 Audio

```bash
speaker-test -t sine -f 440 -c 2 -D plughw:1,0     # bypasses all our code
python test/speaker_test.py                        # tone + sweep
python test/web_dashboard.py                       # :5000, tones + MP3
```

In the UI set **Output device → `plughw:1,0 — MAX98357A`**. It is not the
default — ALSA still picks HDMI as card 0.

> `plughw:` not `hw:` — the raw device only accepts stereo, so a mono tone is
> rejected outright.

Start the level low. A full-scale sine into a class-D amp will damage a small
speaker.

### 5.5 LiDAR

```bash
ls -l /dev/ttyUSB*                     # should show ttyUSB0
python test/lidar_probe.py -p /dev/ttyUSB0
python test/lidar_view.py -p /dev/ttyUSB0          # :5002
```

Expect **YDLIDAR-family framing (AA 55) at 115200 baud**, then roughly:

| | |
|---|---|
| Rate | ~11 Hz |
| Points / turn | ~200 |
| Bad checksums | **0** |

Then check it is *correct*, not merely talking:

- Put a flat object at a tape-measured 1000 mm. Reading should be within ~3%.
- Range profile should span the full 0–360° with no permanent gaps.
- Bad checksums must stay at 0. Climbing = baud or cabling, or brownout.

**Calibrate the zero offset**: put an object directly in front of the robot
and adjust the Zero offset field until that point sits at the top of the
plot. Until you do, every bearing is off by a constant.

### 5.6 IMU

```bash
i2cdetect -y 1                         # chip at 0x28/0x29 or 0x68/0x69
python test/imu_test.py --scan         # same, with the chip named
python test/imu_test.py                # identify + live readings
python test/imu_test.py --gyro-bias    # 5 s, robot still
python test/imu_test.py --calibrate    # 30 s, robot FULLY ASSEMBLED
```

Checks:

| Test | Expected |
|---|---|
| Board flat and still | roll ≈ 0, pitch ≈ 0, az ≈ 1 g |
| Rotate 90° by hand | yaw changes ≈ 90° |
| **Motors running, robot still** | **heading barely moves** |

That last one is the real acceptance test and the one people skip. Spin the
motors with the wheels up and watch the heading. If it swings, the
magnetometer is inside the motors' magnetic field and no software fixes it —
move it further away. `WIRING.md` §11.

Calibrate assembled. Calibrating a bare board then bolting it in measures the
wrong magnetic environment.

### 5.7 Camera

```bash
python test/camera_test.py --list      # is the module even seen?
python test/camera_test.py             # 5 s of frame timing
python test/camera_test.py --stream    # live preview on :5005, to aim it
python test/camera_test.py --still     # one JPEG into test/captures/
```

No GPIO, so this rung is safe with the battery disconnected, and it can run
alongside anything else. The one thing it cannot share is the camera itself —
if `web_nav.py` is up, it already has it.

Checks:

| Test | Expected |
|---|---|
| `--list` | the sensor is named, e.g. `Camera Module 3 (imx708)` |
| Frame timing | ~15 fps, worst gap close to the average |
| Encoder line | `MJPEG (hardware)` — software means it costs a core |
| `--stream` | picture the right way up |
| **Motors running, wheels up** | **picture does not stall** |

Two things to fix here rather than later:

**Orientation.** If the preview is upside down, set `CAM_HFLIP` **and**
`CAM_VFLIP` to `True` in `test/pins.py`. `web_nav.py` reads the same two
values, so it only has to be got right once.

**Field of view.** Set `CAM_HFOV` in `pins.py` to match whichever sensor
`--list` just named (the table is in `pins.py` and `WIRING.md` §12). The nav
page draws it as a cyan wedge on the LiDAR plot; wrong, and the wedge claims
to cover ground the lens cannot see.

A camera that lists but delivers no frames is almost always a ribbon seated
well enough to enumerate and not well enough to stream. Power down, reseat
both ends, retry.

### 5.8 Markers

```bash
python test/marker_test.py --sheet     # writes captures/aruco-sheet.svg
python test/marker_test.py             # live: what is in view, how far
python test/marker_test.py --map       # what has been learned so far
```

Copy the sheet off and **print it at 100%** — no "fit to page":

```bash
scp shiv@192.168.1.4:~/Desktop/Speaker_truck/test/captures/aruco-sheet.svg .
```

Then **measure a printed tag with a ruler**, black square only, and set
`MARKER_SIZE_MM` in `test/pins.py` to what you measured. Every distance the
detector reports scales linearly with that number, and nothing else in the
system can catch the error.

Checks:

| Test | Expected |
|---|---|
| `import cv2; hasattr(cv2,'aruco')` | `True` |
| Tag held at 1 m | `dist` reads within a few cm |
| Tag moved to the **left** | `bearing` goes **positive** |
| Tag walked away | drops out somewhere near 2 m |

If bearing has the wrong sign, the fix is in `camera_to_body` in
`markers.py`, not in `pins.py`. `WIRING.md` §13.2.

### 5.9 Drive

```bash
python test/web_drive.py               # :5001
```

Click the page once for keyboard focus, ENABLE, then arrow keys or WASD —
hold to drive, Space is e-stop.

Use the three inversion toggles to find which way things are wired:

| Symptom | Toggle |
|---|---|
| Whole robot drives backwards | Invert LEFT **and** RIGHT |
| Spins instead of going straight | Invert whichever side is wrong |
| ← turns right | Swap SIDES |

Once you know, **fix it in the wiring** (`WIRING.md` §4: swap red and white
on both right-side motors). Software inversion hides the fault from every
other script.

### 5.10 Drive + LiDAR

```bash
python test/web_pilot.py               # :5003
```

Truck drawn to scale inside its own scan, plus the proximity guard. Largely
superseded by `web_nav.py`.

### 5.11 SLAM — everything

```bash
python test/web_nav.py                 # :5004
```

Drive, LiDAR, IMU, camera and the occupancy-grid map on one page.

The cockpit is five tabs — **Drive**, **Map**, **Sensors**, **Vision**,
**Tune** — under a status bar that never scrolls away, so the e-stop and the
sensor badges are always reachable. The tab you were on is remembered across
reloads.

The camera lives on the **Vision** tab, with a small copy in the Drive rail.
**Live view** stops the stream
without stopping the robot — MJPEG is by far the largest thing on that page,
and over a weak Wi-Fi link it will starve the drive commands before anything
else gives way. Turn it off when the map matters more than the picture.

**Floor check** (Vision tab). On by default, and it vetoes forward motion only. Test it on
a kerb or a single step **before** trusting it near stairs. Press *Relearn
floor* on open floor whenever the surface changes; expect false positives on
patterned rugs.

**Markers** (Vision tab). First run, learn them:

```bash
python test/web_nav.py --learn-markers
```

Drive the whole house once. Each tag that comes into view is recorded at
wherever the pose says it is, so drive it while the map still looks right —
a tag learned from a bad pose is a bad tag forever. Press *Save map* in the
Markers panel, then restart without the flag. From then on tags correct the
pose instead of being recorded.

`--no-cliff` turns the camera veto off if it is being a nuisance while you
tune something else.

**Tuning.** Every value that decides whether SLAM works is on the **Tune**
tab — scan matching, the occupancy grid, odometry scale, guard margins, the
explorer. Change one, watch `SLAM cpu` and `Match correction` in the same
panel, and press Save only for what earns it. Full explanations in
[docs/TUNING.md](docs/TUNING.md).

The number to calibrate first is **Counts per rev**: drive a measured metre
and compare `Distance driven` against a tape. Everything about map scale
follows from it.

**Check the encoder sign first.** Push the robot forward by hand and watch
`Encoder counts` on the **Map** tab — **both must increase**. If one
decreases, flip the encoder sign on the **Tune** tab (no restart needed).
Symptom of getting it wrong: the map builds mirrored, or the robot reverses
through its own map.

Then drive slowly. Mapping only integrates after 40 mm of travel or 4° of
turn, so there is no benefit to rushing.

Expect: white = occupied, dark = swept free space, grey = unknown, violet
line = path driven, green arrow = the robot.

Live state over curl, from anywhere on the network:

```bash
curl -s http://<pi-ip>:5004/snapshot | python3 -m json.tool
```

And a still, without opening a browser:

```bash
curl -s http://<pi-ip>:5004/camera/still.jpg -o view.jpg
```

---

## 6. Ports — only one GPIO owner at a time

| Script | Port | Uses GPIO |
|---|---|---|
| `web_dashboard.py` | 5000 | yes |
| `web_drive.py` | 5001 | yes |
| `lidar_view.py` | 5002 | **no** — runs alongside anything |
| `web_pilot.py` | 5003 | yes |
| `web_nav.py` | 5004 | yes |
| `camera_test.py --stream` | 5005 | **no** — but it does claim the camera |
| `marker_test.py` | — | **no** — but it does claim the camera |

Starting a second GPIO-owning script fails with a clear message rather than a
raw traceback.

---

## 7. When something is wrong

**`ModuleNotFoundError: No module named 'pins'`**
The file is in the project root instead of `test/`.
`mv ~/Desktop/Speaker_truck/<file>.py ~/Desktop/Speaker_truck/test/`

**`No module named 'flask'` / `serial` / `smbus2`**
The venv is not active. Look for `(.venv)` in the prompt.

**Permission denied on `/dev/ttyUSB0`**
`sudo usermod -aG dialout $USER`, then log out and back in.

**No sound but everything looks right**
`aplay -l` — if there is no MAX98357A card the overlay did not load. And
check `SD` on the amp: above 1.4 V or it is shut down.

**`Undervoltage detected!` in `dmesg`** ⚠
The 5 V rail is sagging. On this build it is triggered by the LiDAR motor
spinning up. It causes throttling, dropped serial bytes and SD corruption —
and it makes algorithm bugs and power faults look identical.

```bash
vcgencmd get_throttled          # 0x0 is clean, anything else has browned out
```

Fix: power the LiDAR from the 5 V buck directly rather than through the Pi's
USB, and confirm the Pi itself is on the buck and not a phone charger.

**Motors do not move**
Check in this order: battery voltage > 9 V → VM at the driver → STBY high
after ENABLE → IN1/IN2 → PWM. `pin_hold.py` holds a state so you can meter
calmly.

---

## 8. Still outstanding

- `COUNTS_PER_REV` in `pins.py` is an estimate (1320) — measure it (§5.3)
- `TRUCK_LEN_MM`, `TRUCK_WIDTH_MM`, `TRACK_WIDTH_MM`, `WHEEL_DIAM_MM` in
  `web_nav.py` are placeholders — measure them, the map scale depends on it
- **The TB6612s cannot drive these motors under load.** Bench only, wheels
  up, `MAX_DUTY` 0.40, until they are replaced with an MDD10A or BTS7960
  pair. `WIRING.md` §8.
- Undervoltage, above
