# Speaker Truck

A 4-wheel skid-steer robot that maps a house on its own. Raspberry Pi 4 for the
brain, LiDAR + IMU + camera for sensing, and a browser cockpit you drive it
from. No ROS, no build step - plain Python and one self-contained HTML page.

![stage](https://img.shields.io/badge/stage-bench%20testing-orange)
![python](https://img.shields.io/badge/python-3.13-blue)
![platform](https://img.shields.io/badge/platform-Raspberry%20Pi%204-c51a4a)

### What it does

| | |
|---|---|
| **SLAM** | Occupancy grid and scan matching, with match-confidence rejection and loop closure. Measured 82 mm final error over a 48 m circuit, against 844 mm before. |
| **Autonomous mapping** | Frontier exploration with A* planning - a doorway *is* a frontier, so doors get found without being a special case. |
| **Collision guard** | Footprint-aware and per-direction. Blocks only what is blocked, and can always creep out rather than wedging itself. |
| **Cliff detection** | The camera sees the stair edge that a horizontal LiDAR cannot. |
| **ArUco localisation** | Printed tags give an absolute position fix - the only input outside SLAM's closed loop. |
| **Object labels** | Open-vocabulary detection pins "sofa", "wardrobe", "staircase" to the map. On the Pi, or off-board on a PC. |
| **Live tuning** | 56 parameters adjustable while driving, each sitting next to the evidence you judge it by. |
| **Self-calibration** | Push the robot half a metre and it measures its own scanner rotation and odometry scale, with a residual. |

### The cockpit

Five tabs over a status bar that never scrolls away - **Drive** (LiDAR plot,
mounting, guard), **Map** (occupancy grid, SLAM tuning), **Sensors**,
**Vision** (camera, floor check, markers, objects), **Tune**.

**New here? This page is the whole path.** Fifteen minutes from a cold laptop
to a moving robot. Everything else links out from the bottom.

> ⚠ **Wheels off the ground.** The motor drivers currently fitted cannot
> survive a stall on these motors. See [the blocker](#-before-you-drive-it).

---

## 1. Find the Pi

Its IP is **not stable** — it has moved three times. Don't trust a written-down
address, including this one.

```powershell
# Windows PowerShell — scan the subnet for anything with SSH open
1..254 | ForEach-Object {
  $c = New-Object Net.Sockets.TcpClient
  @{ip="192.168.1.$_"; c=$c; r=$c.BeginConnect("192.168.1.$_",22,$null,$null)}
} | ForEach-Object -Begin { Start-Sleep 4 } -Process {
  if ($_.r.IsCompleted -and $_.c.Connected) { $_.ip }; $_.c.Close()
}
```

Usually one hit — that's the Pi. Then:

```powershell
ssh shiv@<that-ip>
```

A **static DHCP lease** on the router would end this permanently, and is the
single best twenty minutes anyone could spend on this project.

---

## 2. Sync your code to it

**Files are written on Windows and copied to the Pi. Nothing is edited on the
Pi** — the next sync would overwrite it.

| | |
|---|---|
| Edit here (your PC) | wherever you cloned this |
| Runs here (Pi) | `~/Desktop/Speaker_truck` |

```bash
# Git Bash, ON WINDOWS
rsync -av --exclude '.venv' --exclude '__pycache__' \
  ./ shiv@<pi-ip>:~/Desktop/Speaker_truck/
```

No rsync? PowerShell:

```powershell
scp -r test shiv@<pi-ip>:/home/shiv/Desktop/Speaker_truck/
```

> ### ⚠ The mistake everyone makes once
>
> **`scp` and `rsync` run on Windows, not inside the SSH session.**
>
> They always run on the machine that owns the local path. Type them at a Pi
> prompt and `C:` looks like a hostname, and you get
> `scp: stat local "Speaker_trucktest": No such file or directory`.
>
> If you're already SSH'd in: `exit` first.

---

## 3. Run something

On the Pi, always these three lines — the venv lives at the project root, and
scripts always take the `test/` prefix:

```bash
cd ~/Desktop/Speaker_truck
source .venv/bin/activate
python test/<script>.py
```

You're ready when the prompt reads `(.venv) shiv@shiv:~/Desktop/Speaker_truck $`.

### The one you actually want

```bash
python test/web_nav.py
```

Open `http://<pi-ip>:5004`, **click the page once** for keyboard focus, then
arrow keys or WASD. Space is e-stop.

Five tabs: **Drive** (LiDAR plot, scanner mounting, guard), **Map** (occupancy
grid, SLAM and odometry tuning), **Sensors** (IMU, LiDAR, encoders, camera),
**Vision** (camera, floor check, markers), **Tune** (the full index of every
tunable).

**Tuning happens next to the thing it changes** — the vehicle footprint and
mounting sliders sit beside the plot, matching sliders beside SLAM cpu, floor
thresholds over the live picture. The Tune tab is the index and the Save/Revert button, not the place
you work.

Two constants can be **measured** rather than guessed: *Measure by pushing* on
the Drive tab derives the scanner's true rotation and counts-per-rev from one
hand-push, with a residual so you know whether to believe it.

### Testing one piece at a time

Each hardware script does one thing and prints what it found. Run them in this
order on a fresh build — stop at the first failure:

| Script | Checks |
|---|---|
| `python test/pin_hold.py` | GPIO pins, no motor power |
| `python test/motor_test.py` | TB6612 drivers and motors |
| `python test/speaker_test.py` | I2S amp and speaker |
| `python test/lidar_probe.py` | LiDAR serial, raw frames |
| `python test/imu_test.py` | IMU detect, live readings, calibration |
| `python test/camera_test.py` | Camera detect, frame timing, aiming |
| `python test/marker_test.py` | ArUco tags: print a sheet, check range |

`test/calibrate.py` has no test script — it is driven from the Drive tab,
because it needs the LiDAR running and a person to push the robot.

Full detail and what each result should look like: **[Start_pi.md](Start_pi.md)**.

---

## 4. Which file do I edit?

| I want to change… | Edit |
|---|---|
| A GPIO pin | `test/pins.py` — the only place pins are defined |
| Robot dimensions, sensor mounting positions | `test/pins.py` |
| How SLAM behaves | **the Tune tab**, live — then Save. See [docs/TUNING.md](docs/TUNING.md) |
| The cockpit page | `test/web_nav.py` (the `PAGE` string) |
| Mapping / scan matching internals | `test/slam.py` |
| Autonomous exploration | `test/explore.py` |
| Camera, markers, cliff detection | `test/camera.py`, `test/markers.py`, `test/cliff.py` |
| Object detection | `test/detect.py` — needs `yolov8n.onnx` in `test/` |
| Better detection, off-board | `tools/detect_server.py` — run it on your PC, paste the URL into the Objects panel |

Never hardcode a pin number. Import it from `pins.py`.

---

## 5. ❗ Before you drive it

**The motor drivers cannot survive these motors.** The JGB37-520 stalls at
4–5 A; the TB6612FNG peaks at 3.2 A. A stall happens every time a wheel hits
an obstacle, and stall exceeds even the peak rating.

Until they're replaced with something like a Cytron MDD10A:

- **Wheels off the ground.**
- `MAX_DUTY` stays capped at 0.40 in `pins.py`.

Details in [WIRING.md §8](WIRING.md).

---

## Where everything else lives

| Document | For |
|---|---|
| [Start_pi.md](Start_pi.md) | Full bring-up runbook and the test ladder |
| [docs/TUNING.md](docs/TUNING.md) | Every variable that decides whether SLAM works |
| [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) | How the pieces fit, the coordinate frame, the thread model |
| [docs/TROUBLESHOOTING.md](docs/TROUBLESHOOTING.md) | When it doesn't work |
| [WIRING.md](WIRING.md) | Authoritative hardware: power rails, every connection |
| [PINOUT.md](PINOUT.md) | Pi 4 header diagram and this build's pin map |

## Hardware, briefly

| | |
|---|---|
| Brain | Raspberry Pi 4B 8 GB, Raspberry Pi OS Trixie 64-bit |
| Drive | 4 × JGB37-520 12 V 330 RPM with hall encoders, 2 × TB6612FNG |
| Ranging | 2D LiDAR, 115200 baud, ~11 Hz, USB serial |
| Orientation | BNO055 9-axis IMU on I2C |
| Vision | Camera Module 1 (ov5647) on CSI — no GPIO cost |
| Audio | MAX98357A I2S amp + 12 W 4 Ω speaker |
| Power | 3S Li-ion 11.1 V, 5 V buck for logic |

---

## Licence

MIT. Do what you like with it.

## A note on the code

Every non-obvious decision here is commented with *why*, including the ones
that were wrong first. If something looks odd, the comment above it usually
explains what happened the other way round - a 250 mm frame offset between the
plot and the collision guard, a confidence metric that could not tell a
corridor from a room, a canvas that grew until the browser died. Those notes
are the useful part.
