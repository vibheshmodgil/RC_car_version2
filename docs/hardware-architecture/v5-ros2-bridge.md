# ROS2 Bridge (v5, layered on top of v4 — not a replacement)

Status: **new, written but not yet run on real hardware** — same caveat
as the rest of `pi/` per `pi/RESUME_GUIDE.md`. Added 2026-07-21.

This doc does **not** supersede
[`v4-home-wifi-current.md`](v4-home-wifi-current.md) — the network
topology, static IPs, DevKit REST/WS contract, and safety layers
described there are all still exactly true. This doc only adds a new
consumer on the Pi: a ROS2 node that sits next to the existing
`pi/webapp` dashboard and talks to the same `pi/esp32_link.py`,
`pi/imu.py`, `pi/lidar.py` modules those already use. **The DevKit
(`CarTestBench/`) and ESP32-CAM (`CamStreamer/`) firmware are
completely untouched by this.**

If you have never used ROS before, read the next section first — it's
written for zero prior knowledge.

## ROS2, briefly, if you've never used it

ROS ("Robot Operating System") isn't an OS — it's a messaging framework
that runs on top of Linux. Three ideas cover most of what you need to
read this doc:

- **Node** — one running program with one job. This project adds one:
  `car_bridge`.
- **Topic** — a named, typed data stream one or more nodes publish to
  and one or more nodes subscribe to (`ros2 topic echo <name>` prints
  it live from a terminal, no code needed). E.g. `/scan` carries LiDAR
  data as a standard `sensor_msgs/LaserScan` message.
- **Service** — a one-shot request/response call, like a function call
  across processes (`ros2 service call <name> <type>`). E.g. `/estop`.

Because the message types (`sensor_msgs/Imu`, `sensor_msgs/LaserScan`,
`geometry_msgs/Twist`, ...) are standardized across the whole ROS
ecosystem, generic tools work on this car with zero car-specific code:

- **RViz2** — a 3D viewer that can show any node's `/scan` and
  `/imu/data` live, from any machine on the network.
- **`teleop_twist_keyboard`** — drives any robot that listens on
  `/cmd_vel` from the keyboard. No car-specific code needed.
- **Nav2 / SLAM Toolbox** (future work, not set up yet) — full
  autonomous navigation and mapping stacks that consume exactly the
  `/scan`, `/imu/data`, and (eventually) odometry topics this bridge
  already publishes.

ROS2 (not ROS1 — ROS1 "Noetic" reached end-of-life May 2025) also
auto-discovers nodes over the local network with **no configuration**:
any machine on the same WiFi running ROS2 sees this car's topics
immediately. That happens to fit this project unusually well, since
every board here already joins the same home WiFi router (see v4).

## Why a bridge, not a firmware rewrite

ROS2 needs a real OS (Linux) — it can run on the Pi, but **not** on the
ESP32 DevKit or ESP32-CAM; those are bare microcontrollers. The
alternative, "micro-ROS," turns a microcontroller into a real ROS2
node, but doing that here would mean rewriting the DevKit's motor /
encoder / e-stop / deadman firmware — code that runs the physical
safety system — to drop the REST/WS server entirely. `CLAUDE.md` calls
that REST/WS contract stable ("extend it, never break it"), so this
integration instead adds a **bridge node on the Pi**: it translates
ROS2 topics/services into the exact same HTTP calls the dashboard
already makes, via `pi/esp32_link.py`. The DevKit and CAM never know
ROS2 exists.

## Topology

```
                     Home WiFi router: Airtel_kuma_9602
                               192.168.1.1
                                    |
      +---------------+------------+-------------+
      |               |                           |
    Phone       ESP32 DevKit                 ESP32-CAM
  (browser)    192.168.1.51 (unchanged)    192.168.1.52 (unchanged)
      |                  ^                        |
      |                  | REST/WS (unchanged)    |
      +--> http://192.168.1.50/  <-- Raspberry Pi 4 (.50)
           pi/webapp dashboard      |
           (unchanged)              |
                                car_bridge (ROS2 node, new)
                                  |        |        |
                              cmd_vel   imu/data   scan
                                  |        |        |
                    (any laptop on the same home WiFi, running
                     ROS2 Desktop: rviz2, teleop_twist_keyboard —
                     no extra network setup, DDS just finds it)
```

## Why the Pi's OS changes (Bookworm -> Ubuntu Server 24.04)

ROS2 Jazzy's official installer only publishes prebuilt `apt` packages
for **Ubuntu 24.04**. The Pi currently runs Raspberry Pi OS (Debian
Bookworm) per `pi/README.md`. Rather than build ROS2 from source on a
Pi 4 (slow, high maintenance) or use a community conda distribution
(RoboStack — works, but not the official binaries), this integration
reimages the Pi to **Ubuntu Server 24.04 LTS (arm64)**. That's a
deliberate, confirmed tradeoff (see decision log in project memory) —
official packages, matches every ROS2 tutorial exactly, easiest to
debug long-term. It does mean re-running the Pi bring-up steps in
`pi/README.md` on the new OS — see that file's updated one-time-setup
section.

### Reimage steps

1. **Raspberry Pi Imager** -> choose OS -> "Other general-purpose
   OS" -> Ubuntu -> **Ubuntu Server 24.04.x LTS (64-bit)** (the arm64
   Pi 4 build).
2. Gear-icon "advanced options" before writing: set hostname, enable
   SSH, set username/password, and preset the home WiFi SSID/password
   (`Airtel_kuma_9602`) exactly like the Raspberry Pi OS setup in
   `pi/README.md` — Ubuntu's Imager images support the same cloud-init
   presets.
3. Boot it, SSH in (`ssh <user>@<pi-ip-or-hostname>` — same as before).
4. **Networking fix so `pi/setup_wifi.sh` keeps working unmodified:**
   Ubuntu Server defaults to netplan + systemd-networkd, not
   NetworkManager, so the existing `nmcli`-based script won't work
   out of the box. Install NetworkManager and tell netplan to use it:
   ```bash
   sudo apt update
   sudo apt install -y network-manager
   ```
   Edit `/etc/netplan/*.yaml` (there's usually one file created by the
   installer) so it has:
   ```yaml
   network:
     version: 2
     renderer: NetworkManager
   ```
   Then:
   ```bash
   sudo netplan apply
   sudo systemctl enable --now NetworkManager
   ```
   After this, `bash pi/setup_wifi.sh` (unchanged from `pi/README.md`)
   works exactly as documented for Bookworm.
5. **Enable I2C** (for the BNO055) — Ubuntu has no `raspi-config`, so
   do it directly in the boot config:
   ```bash
   echo 'dtparam=i2c_arm=on' | sudo tee -a /boot/firmware/config.txt
   echo 'dtparam=i2c_arm_baudrate=10000' | sudo tee -a /boot/firmware/config.txt
   sudo reboot
   ```
   (the `i2c_arm_baudrate=10000` line is the same BNO055 clock-stretch
   workaround `pi/README.md` already documents for Bookworm.)
6. Re-run the rest of `pi/README.md`'s one-time setup (venv,
   `requirements.txt`, deploy code, smoke tests) — package names are
   called out where they differ from Bookworm.

## Installing ROS2 Jazzy on the Pi

Headless install (`ros-base`, no GUI tools — this Pi has no monitor):

```bash
sudo apt update && sudo apt install -y curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list
sudo apt update
sudo apt install -y ros-jazzy-ros-base ros-dev-tools python3-colcon-common-extensions
echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc
source /opt/ros/jazzy/setup.bash
```

The bridge node reuses `pi/esp32_link.py`/`imu.py`/`lidar.py`, which
need `requests`, `smbus2`, `pyserial` importable from **ROS2's own
Python** (this is a separate interpreter from the existing FastAPI
`.venv` in `pi/README.md` — that venv is unrelated to ROS2):

```bash
sudo apt install -y python3-requests python3-serial
pip install --break-system-packages smbus2
sudo usermod -aG i2c,dialout $USER   # re-login after this
```

**On a laptop** (not the Pi — RViz needs a display), install ROS2
Desktop instead of `ros-base` (same steps above, `ros-jazzy-desktop`
in place of `ros-jazzy-ros-base`), plus:

```bash
sudo apt install -y ros-jazzy-rviz2 ros-jazzy-teleop-twist-keyboard
```

As long as the laptop is on the same home WiFi (`Airtel_kuma_9602`),
it will see the Pi's ROS2 topics automatically — no IP configuration,
same principle as the rest of this project's "everyone's just on the
house WiFi" design.

## Building and running `car_bridge`

On the Pi, after deploying the repo (same `scp`/`git pull` as
`pi/README.md` already describes):

```bash
cd ~/car/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch car_bridge bridge_launch.py
```

Arm the wheels first via the existing dashboard (`http://192.168.1.50/`)
or DevKit UI (`http://192.168.1.51/motors`) — `car_bridge` never arms
anything itself, matching the existing safety model.

## Topics and services

| Name | Type | Direction | Notes |
|---|---|---|---|
| `cmd_vel` | `geometry_msgs/Twist` | subscribe | `linear.x`/`angular.z` mixed into per-wheel PWM via `/api/motor`; re-sent every 0.1 s, zeroed if stale > 0.5 s |
| `imu/data` | `sensor_msgs/Imu` | publish | from `pi/imu.py` (BNO055 NDOF); orientation only — no raw gyro/accel yet |
| `scan` | `sensor_msgs/LaserScan` | publish | from `pi/lidar.py` (YDLIDAR X2), binned to 1° resolution |
| `estop` | `std_srvs/Trigger` | service | calls `POST /api/estop` |
| `estop_clear` | `std_srvs/Trigger` | service | calls `POST /api/estop/clear` |

TF: static transforms `base_link -> imu_link` and `base_link ->
laser_link`, currently zero-offset placeholders — replace with the
real measured mounting offsets once you have calipers on the car.

## Verifying it end-to-end (do this before trusting any of it)

1. `ros2 topic list` — confirms the node is up and topics exist.
2. `ros2 topic echo /imu/data` — should show live orientation once the
   BNO055 is calibrated (same calibration behavior as the existing
   dashboard's IMU panel).
3. `ros2 topic echo /scan` — should show a `ranges` array changing as
   the LiDAR spins.
4. Wheels off the ground, wheels armed via the dashboard, then from a
   laptop on the same WiFi: `ros2 run teleop_twist_keyboard
   teleop_twist_keyboard` — confirm wheels respond and stop within
   ~0.5 s of releasing keys or killing the keyboard node (deadman
   check, same "wheels off the ground for first runs" rule as
   `pi/README.md`).
5. `rviz2` on the laptop, add an `Imu` display on `imu/data` and a
   `LaserScan` display on `scan`, fixed frame `base_link` — confirm the
   IMU arrow turns the same direction as the physical car (fix the
   yaw sign in `car_bridge/bridge_node.py`'s `euler_deg_to_quaternion`
   call if it turns backwards — flagged as unverified in the code
   comment).

## Known placeholders / future work

- **No odometry yet.** Wheel encoder ticks aren't published as
  `nav_msgs/Odometry` — needed before Nav2/SLAM can localize the car.
- **No URDF.** There's no robot description file, so RViz shows raw
  sensor data around a bare `base_link` frame, not a 3D car model.
- **LiDAR runs continuously** once the node starts, unlike
  `pi/webapp/hub.py`'s on-demand start/stop (motor wear) — worth
  porting that pattern into `car_bridge` later.
- **`linear_scale`/`angular_scale` are open-loop guesses**, not
  calibrated to real m/s or rad/s — there's no closed-loop speed
  control without odometry.
- **IMU yaw sign is unverified** — see step 5 above.
- **Nav2 / SLAM Toolbox** aren't installed or configured — natural
  next step once odometry + a URDF exist.
