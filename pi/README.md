# pi/ — Raspberry Pi 4 car brain

Everything in this folder runs on the Pi, not the ESP32s. This guide
assumes you're starting from a freshly-imaged (or freshly-rebooted) Pi
and have never used SSH before. Topology and the IP plan live in
[`../docs/hardware-architecture/v4-home-wifi-current.md`](../docs/hardware-architecture/v4-home-wifi-current.md);
sensor wiring and power sizing are still in
`../docs/hardware-architecture/v2-pi-integration-phase.md`.

## Layout

| File | Runs | Mirrors |
|---|---|---|
| `config.py` | constants: IPs, ports, servo pins/limits | `config.h` |
| `esp32_link.py` | REST control + WS telemetry client | `AppServer` |
| `camera.py` | ESP32-CAM MJPEG frames via OpenCV | `CamStreamer` |
| `imu.py` | BNO055 over I2C1, raw smbus2 registers (no Adafruit/CircuitPython) | — |
| `lidar.py` | YDLIDAR X2 over USB, raw serial packet parser (no vendor SDK) | — |
| `gimbal.py` | pan/tilt servos via pigpio | — |
| `smoke/` | one standalone test per subsystem | — |
| `main.py` | orchestrator skeleton (status loop + safety) | `.ino` |
| `webapp/` | FastAPI dashboard: main car UI on port 80 | `WebUI.h` |
| `setup_wifi.sh` | pins the Pi's WiFi to its static home-WiFi IP | — |

## Step 0 — what SSH is, and how to get a terminal on the Pi

SSH ("Secure Shell") opens a command-line session **on the Pi** from your
laptop over the network — no monitor/keyboard plugged into the Pi
needed. Everything you type runs on the Pi, not your laptop.

The command shape is:
```
ssh <username>@<pi-ip-or-hostname>
```

Since the Pi now lives on the home WiFi router at a fixed address (see
the IP plan above), once it's set up this is just:
```
ssh <username>@192.168.1.50
```

Windows 10/11, Mac, and Linux all have an `ssh` client built in — use it
from a terminal (this repo's Bash tool works too). First connection ever
to a device asks you to confirm a "fingerprint" — type `yes`. Then it
asks for a password (or uses a saved key if you set one up).

If you don't remember the Pi's username/password: it's whatever was set
during SD card imaging — **Raspberry Pi Imager** (the official tool for
writing the OS to the SD card) lets you set a custom username/password
and even preset the WiFi SSID/password under its gear-icon "advanced
options" before you write the card, which is the easiest way to get a
fresh Pi straight onto home WiFi with no extra steps.

If you don't know the Pi's IP yet (first boot before `setup_wifi.sh` has
run): check your router's "connected devices" list, or try
`ssh <username>@raspberrypi.local` (mDNS — Raspberry Pi OS ships this by
default, and it often just works without knowing the IP at all).

## One-time Pi setup

Do this once, over SSH:

```bash
sudo apt update
sudo apt install -y python3-pip python3-venv python3-opencv python3-lgpio python3-smbus i2c-tools
sudo raspi-config nonint do_i2c 0        # enable I2C

# BNO055 clock-stretch workaround:
echo 'dtparam=i2c_arm_baudrate=10000' | sudo tee -a /boot/firmware/config.txt
sudo reboot
```

(Older guides for this project say `python3-pigpio`/`pigpiod` — that
package was dropped from Raspberry Pi OS's apt repo as unmaintained
starting with Bookworm/Trixie. The gimbal code now uses `lgpio`, the
actively-maintained successor from the same author, which needs no
background daemon.)

Then create the venv (system-site-packages so apt's opencv/pigpio are visible):

```bash
cd ~/car/pi
python3 -m venv --system-site-packages .venv
.venv/bin/pip install -r requirements.txt
```

## Put the Pi on home WiFi with a fixed IP

If you preset the WiFi SSID/password in Raspberry Pi Imager, the Pi is
already on home WiFi (over DHCP, a changeable address) by the time you
first SSH in. Either way, run this once to pin it to the fixed address
everything else expects:

```bash
cd ~/car/pi
bash setup_wifi.sh
```

This replaces the Pi's connection with a static IP (192.168.1.50) on the
home router `Airtel_kuma_9602`. Unlike the old AP setup this project used
to use, the Pi keeps normal internet access the whole time — no more
switching networks to get updates.

## Deploying code to the Pi

Since the Pi is always reachable on the home network now:

```bash
scp -r pi/ pi@192.168.1.50:~/car/
```

(or `git pull` on the Pi if the repo is on GitHub — the Pi has internet
now, so this works too.)

## Run the smoke tests, in order

```bash
cd ~/car/pi && source .venv/bin/activate
python smoke/estop.py         # 1. control path to the ESP32
python smoke/telemetry.py     # 2. WS telemetry flowing
python smoke/camera_read.py   # 3. CAM stream + FPS
python smoke/imu_read.py      # 4. BNO055 orientation
python smoke/lidar_scan.py    # 5. LiDAR spins and ranges
python smoke/gimbal_sweep.py  # 6. servos move (clear the gimbal first!)
```

All green → `python main.py` for the combined status loop. For the full
end-to-end walkthrough (power-on order, what each test should print,
what to do if one fails), see [`../TESTING.md`](../TESTING.md).

## Dashboard (pi/webapp)

The Pi serves the car's main UI — one dark phone page with e-stop, arm,
hold-to-drive D-pad, per-wheel PWM/RPM, IMU, embedded CAM stream, gimbal
sliders and a LiDAR polar plot, all fed by one merged WebSocket at 10 Hz.
Sensors that are missing just show as offline; the rest keeps working.

Try it by hand first:

```bash
cd ~/car/pi && source .venv/bin/activate
sudo .venv/bin/python -m uvicorn webapp.server:app --host 0.0.0.0 --port 80
```

then browse to `http://192.168.1.50/` from any device on the home WiFi.
Install it as a boot service:

```bash
sudo cp ~/car/pi/webapp/car-dashboard.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now car-dashboard
journalctl -u car-dashboard -f     # logs
```

The unit runs as user `pi` with `CAP_NET_BIND_SERVICE` (no root, still
port 80) and assumes the repo at `/home/pi/car` — edit the paths in the
unit file if yours differ. The ESP32's own UI stays available at
`http://192.168.1.51` as a debug fallback.

## Safety rules for any script that drives

- Re-send the drive command at least every `DEADMAN_RESEND_S` (0.15 s); if
  your loop stalls, stop the car.
- Wrap driving code so exceptions call `link.estop()` and normal exit calls
  `link.stop()` — `main.py` shows the pattern.
- Wheels off the ground for first runs, same as the phone UI workflow.

## If you get stuck / coming back after a break

See [`RESUME_GUIDE.md`](RESUME_GUIDE.md) — it's a running log of what's
actually been verified on the real hardware versus what's just written
code, plus the order to re-check each subsystem in.
