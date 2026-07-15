# pi/ — Raspberry Pi 4 car brain

Everything in this folder runs on the Pi, not the ESP32s. Wiring, IPs and
power live in `../Hardware Architecture - Pi Integration Phase.md`.

## Layout

| File | Runs | Mirrors |
|---|---|---|
| `config.py` | constants: IPs, ports, servo pins/limits | `config.h` |
| `esp32_link.py` | REST control + WS telemetry client | `AppServer` |
| `camera.py` | ESP32-CAM MJPEG frames via OpenCV | `CamStreamer` |
| `imu.py` | BNO055 over I2C1 | — |
| `lidar.py` | RPLidar A1/C1 over USB | — |
| `gimbal.py` | pan/tilt servos via pigpio | — |
| `smoke/` | one standalone test per subsystem | — |
| `main.py` | orchestrator skeleton (status loop + safety) | `.ino` |
| `webapp/` | FastAPI dashboard: main car UI on port 80 | `WebUI.h` |

## One-time Pi setup

Do this on home WiFi (internet), before switching to the car AP:

```bash
sudo apt update
sudo apt install -y python3-pip python3-venv python3-opencv python3-pigpio i2c-tools pigpio
sudo systemctl enable --now pigpiod
sudo raspi-config nonint do_i2c 0        # enable I2C

# BNO055 clock-stretch workaround:
echo 'dtparam=i2c_arm_baudrate=10000' | sudo tee -a /boot/firmware/config.txt
sudo reboot
```

Then create the venv (system-site-packages so apt's opencv/pigpio are visible):

```bash
cd ~/car/pi
python3 -m venv --system-site-packages .venv
.venv/bin/pip install -r requirements.txt
```

## Deploying code to the Pi

From the Windows machine (both on the same network as the Pi):

```
scp -r pi/ pi@<pi-ip>:~/car/
```

(or `git pull` on the Pi if the repo is on GitHub — the Pi has no internet
while on the car AP, so scp over the AP is the everyday path.)

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

All green → `python main.py` for the combined status loop.

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

then browse to `http://192.168.4.1/` (or the Pi's home-WiFi IP while testing
off the car). Install it as a boot service:

```bash
sudo cp ~/car/pi/webapp/car-dashboard.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now car-dashboard
journalctl -u car-dashboard -f     # logs
```

The unit runs as user `pi` with `CAP_NET_BIND_SERVICE` (no root, still
port 80) and assumes the repo at `/home/pi/car` — edit the paths in the
unit file if yours differ. The ESP32's own UI stays available at
`http://<ESP32_HOST>` as a debug fallback.

## Safety rules for any script that drives

- Re-send the drive command at least every `DEADMAN_RESEND_S` (0.3 s); if
  your loop stalls, stop the car.
- Wrap driving code so exceptions call `link.estop()` and normal exit calls
  `link.stop()` — `main.py` shows the pattern.
- Wheels off the ground for first runs, same as the phone UI workflow.
