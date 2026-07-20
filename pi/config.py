"""Single source of truth for the Pi-side constants — the config.h of the Pi.

Network values must match CarTestBench/config.h (AP_SSID/STA_STATIC_IP/
CAM_HOST) and the IP plan in
'docs/hardware-architecture/v4-home-wifi-current.md'.
"""

# ---- Network (all three boards are stations on the home WiFi router) ----
# IP plan: home router .1, Pi .50 (dashboard), DevKit .51, CAM .52.
ESP32_HOST = "192.168.1.51"         # DevKit: REST control + WS telemetry
CAM_HOST = "192.168.1.52"           # ESP32-CAM
CAM_STREAM_URL = f"http://{CAM_HOST}:81/stream"
CAM_CAPTURE_URL = f"http://{CAM_HOST}/capture"
CAM_STATUS_URL = f"http://{CAM_HOST}/status"

# ---- YDLIDAR X2 (USB serial, raw packet parser — no vendor SDK) ----
LIDAR_PORT = "/dev/ttyUSB0"
LIDAR_BAUD = 115200

# ---- BNO055 IMU (I2C, raw smbus2 registers — no Adafruit/CircuitPython) ----
IMU_I2C_BUS = 1                     # /dev/i2c-1 (SDA1/SCL1, pins 3/5)
IMU_I2C_ADDR = 0x28                 # ADR pin low/open

# ---- Gimbal servos (Pi hardware PWM via lgpio) ----
GIMBAL_PAN_GPIO = 18                # physical pin 12
GIMBAL_TILT_GPIO = 19               # physical pin 35
# Conservative pulse limits; widen toward 500-2500 us only after checking
# the mechanics can't bind at the extremes.
SERVO_MIN_US = 1000
SERVO_MAX_US = 2000

# ---- Safety ----
DEADMAN_RESEND_S = 0.15             # re-send drive command at least this often
TELEMETRY_STALE_S = 0.5             # telemetry older than this = link unhealthy

# ---- Pi dashboard (pi/webapp) ----
DASHBOARD_PORT = 80                 # systemd grants CAP_NET_BIND_SERVICE
WS_PUSH_S = 0.1                     # merged telemetry push period (~10 Hz)
CAM_POLL_S = 1.0                    # CAM /status poll period
LIDAR_WS_POINTS = 240               # max LiDAR points per WS frame
