"""
Single source of truth for GPIO assignments. BCM numbering.

Every script imports from here. If a pin moves, it moves once, in this file.
Header-pin numbers are in the comments for wiring; see PINOUT.md.

Many of the values below are also LIVE-TUNABLE from the cockpit's Tune tab —
change them while driving instead of editing, syncing and restarting. The
registry is test/tuning.py and the explanations are in docs/TUNING.md. What
is set here is the starting point and the thing "Revert to code defaults"
goes back to.
"""

# --- TB6612FNG #1 — LEFT side (front-left + rear-left motors) --------------
# On the driver, AIN1+BIN1 are tied, AIN2+BIN2 are tied, PWMA+PWMB are tied,
# because both left wheels always turn together on a differential drive.
LEFT_PWM = 12   # header 32 — PWM0
LEFT_IN1 = 23   # header 16
LEFT_IN2 = 24   # header 18

# --- TB6612FNG #2 — RIGHT side (front-right + rear-right motors) -----------
RIGHT_PWM = 13  # header 33 — PWM1
RIGHT_IN1 = 27  # header 13
RIGHT_IN2 = 22  # header 15

# --- Shared ----------------------------------------------------------------
STBY = 25       # header 22 — LOW disables BOTH drivers. Emergency stop.

# --- Quadrature encoders (JGB37-520 hall, 6-wire motors) -------------------
# One encoder per side is enough: both wheels on a side always turn together.
# Wire the two FRONT motors; leave the rear encoders unconnected.
#
# Motor wire colours (6-wire JGB37-520):
#   red   = motor +      (heavy)   -> driver AO1/BO1
#   white = motor -      (heavy)   -> driver AO2/BO2
#   blue  = hall VCC     (thin)    -> 3.3 V
#   black = hall GND     (thin)    -> common ground
#   yellow= hall A / C1  (thin)    -> ENC_A below
#   green = hall B / C2  (thin)    -> ENC_B below
#
# *** black is hall GND, NOT motor minus. white is motor minus, NOT a signal.
# *** Getting those two backwards puts 11.1 V into the encoder or a GPIO.
# *** Confirm by meter: red-white reads 1-10 ohm, every thin wire reads open.
#
# *** Hall VCC (blue) -> 3.3 V, NOT 5 V. The encoder output swings to VCC,
# *** and 5 V into a 3.3 V-max GPIO damages the pin.
LEFT_ENC_A = 5    # header 29 — yellow, front-left
LEFT_ENC_B = 6    # header 31 — green,  front-left
RIGHT_ENC_A = 16  # header 36 — yellow, front-right
RIGHT_ENC_B = 26  # header 37 — green,  front-right

# Counts per output-shaft revolution, quadrature (4x) decoded.
# JGB37-520 hall C1 gives 11 pulses/rev on the MOTOR shaft (before the
# gearbox); multiply by the gearbox ratio, then by 4 for quadrature edges.
# For the 330 RPM (~1:30) variant that is roughly 11 * 30 * 4 = 1320.
# The 11 PPR is a spec figure; the 1:30 ratio is inferred from 330 RPM.
# *** MEASURE THIS. Turn the wheel exactly one revolution by hand and count.
COUNTS_PER_REV = 330
# ^ was 1320. Measured on the robot 2026-09-06: a 1.0 s burst gave 188 counts
# while the LiDAR showed ~110 mm of real travel — a factor of ~4 out.
# gpiozero's RotaryEncoder counts full QUADRATURE CYCLES, not edges, so the
# x4 in "11 PPR x 30 gearbox x 4" does not apply. 11 x 30 = 330.
# *** STILL WORTH MEASURING: reset the encoders, turn one wheel exactly one
# *** revolution by hand, read the count.

# --- 9-axis IMU (I2C) ------------------------------------------------------
# The IMU is an I2C device, not a GPIO one — it needs no pin allocation here
# beyond the bus itself, which was already reserved for I2C. Nothing above
# changes.
#
#   SDA -> GPIO2 (header 3)      SCL -> GPIO3 (header 5)
#   VCC -> 3.3 V  (header 1)     GND -> header 9
#
# *** 3.3 V, NOT 5 V. GPIO2/3 have fixed 1.8k pull-ups to 3.3 V on the Pi, so
# *** a 5 V-powered IMU back-feeds 5 V into those pins through its own
# *** pull-ups and damages them. Full detail in WIRING.md section 11.
IMU_I2C_BUS = 1
IMU_ADDRS = (0x28, 0x29, 0x68, 0x69)   # BNO055 / MPU-ICM families
IMU_MAG_ADDR = 0x0C                    # AK8963 / AK09916, behind I2C bypass

# How the board is bolted in, degrees.
#
# IMU_YAW_OFFSET only affects the DISPLAYED compass heading — set it so the
# N marker points at true north. It does not affect SLAM: odometry references
# heading to whatever yaw was read at startup, so any constant offset cancels.
IMU_YAW_OFFSET = 0.0

# The board is not mounted flat. Measured 2026-09-06 with the robot stationary
# on a level floor: roll -5.5, pitch -7.6, which is 10.6 degrees off vertical.
#
# These are subtracted before roll/pitch are displayed, so the attitude
# indicator and the tilt warning show the BODY's attitude rather than the
# sensor's. Without them the tilt warning fires at ~10 degrees of real tilt in
# one direction and ~30 in the other.
#
# They deliberately do NOT feed SLAM. Rotating the body about the vertical is
# a left-multiplication in the earth frame, so a fixed mounting tilt adds only
# a constant to yaw — and that constant cancels against yaw0. Measured cost of
# leaving it uncorrected: 0.00 deg on a flat floor, 1.3 deg at 10 deg of body
# tilt. Below the noise of a 200-point scan match.
#
# Re-measure after any remount: park on a level floor, run
#   python test/imu_test.py
# and read the steady roll and pitch.
IMU_ROLL_OFFSET = -5.5
IMU_PITCH_OFFSET = -7.6

# --- Vehicle geometry (mm) -------------------------------------------------
# Body frame: origin at the geometric centre, +x forward, +y LEFT.
TRUCK_LENGTH_MM = 300.0        # front to back
TRUCK_WIDTH_MM  = 400.0        # side to side
TRACK_WIDTH_MM  = 340.0        # centre-to-centre between left and right wheels
WHEEL_DIAM_MM   = 68.0        # measured 2026-09-06

# *** The LiDAR is NOT at the centre. It sits at the FRONT RIGHT corner, so
# *** every return has to be translated into the body frame before use.
# *** Skipping this makes the map swing by ~250 mm every time the robot turns
# *** on the spot, because the scan origin orbits the true centre of rotation.
# Body frame: +x forward, +y LEFT, origin at the geometric centre.
# The scanner is on the BACK LEFT corner.
LIDAR_OFFSET_X = -150.0        # -TRUCK_LENGTH_MM/2 -> toward the back
LIDAR_OFFSET_Y =  200.0        # +TRUCK_WIDTH_MM/2  -> toward the left

# Rotation of the scanner about vertical, degrees. This is the one number
# that decides whether "ahead" means ahead. Get it wrong and the collision
# guard checks the wrong direction while the plot still looks sensible, which
# is how the robot jams for no visible reason.
#
# MEASURED on the robot 2026-09-06, twice, in opposite directions.
#
# For pure translation the range-change rate across bearings follows
# -cos(a - nose), so the nose is found by fitting that cosine over ALL
# bearings (first Fourier harmonic) rather than picking whichever single
# bearing closed fastest. The argmin approach is what produced an earlier
# wrong answer of 250: with coarse bins and a short move it just tracks noise.
#
#   forward 574 mm  -> nose at 22.7 deg
#   reverse 612 mm  -> tail at 201.7 deg, so nose at 21.7 deg
#   the two disagree by 1.0 deg
#
# The value is the NOSE BEARING ITSELF. A return at scanner bearing a lands at
# math angle -a, so rotating the cloud by +nose brings the nose onto +x.
LIDAR_YAW_OFFSET = 22.0

# The IMU is at the BACK RIGHT. Position does not affect heading — a gyro and
# magnetometer measure rotation and field, both independent of where on a
# rigid body they sit. Recorded for completeness only.
IMU_OFFSET_X = -150.0
IMU_OFFSET_Y = -200.0

# Clearance added around the footprint by the collision check.
#
# 50, not 80. At 80 the robot needed a 330 mm radius of clear floor just to
# rotate, which a real house rarely offers next to furniture — it wedged
# itself repeatedly with every direction refused. 50 mm is still ample at the
# 0.12 duty this thing runs at, and CREEP_MARGIN_MM below stops a total
# lock-up when even that is unavailable.
SAFETY_MARGIN_MM = 50.0

# When EVERY direction is blocked, the guard would otherwise pin the robot in
# place forever and a human has to lift it out. Instead it may creep, slowly,
# in whichever direction has the most room, as long as that is at least this
# far clear. A guard that cannot be escaped is a guard that gets switched off.
CREEP_MARGIN_MM = 25.0
CREEP_THROTTLE = 0.45

# --- CSI camera ------------------------------------------------------------
# The camera is on the dedicated CSI ribbon connector, NOT on GPIO. It costs
# no header pins, so nothing above changes and it cannot conflict with the
# motors, encoders, I2S audio or the I2C IMU. Orientation and framing are
# software settings, which is why they live here rather than in the wiring.
#
# Stream size and rate. 640x480 at 15 fps is a driving aid, not a recording
# rig — the LiDAR does the measuring. Raising these is not free: they set how
# much the hardware JPEG encoder and the network have to move while SLAM is
# already using most of a core. 1280x720 is usable on a quiet LAN; go higher
# only for a still.
CAM_SIZE = (640, 480)
CAM_FPS = 15

# Mounting orientation. Set BOTH for an upside-down camera (180 degrees).
# Check with `python test/camera_test.py --stream`, which shows a full-window
# picture, then make the answer permanent here — web_nav.py reads these same
# two values, so it only has to be got right once.
CAM_HFLIP = False
CAM_VFLIP = False

# Where the camera sits, same body frame as everything else: origin at the
# geometric centre, +x forward, +y LEFT. Front centre, looking forward.
# *** MEASURE AND CORRECT after mounting.
CAM_OFFSET_X = 150.0           # +TRUCK_LENGTH_MM/2 -> the front face
CAM_OFFSET_Y = 0.0

# Which way the lens points, degrees, 0 = straight ahead, + = left.
#
# This is the camera's BEARING on the robot, not the picture's orientation.
# It decides where the field-of-view wedge sits on the LiDAR plot and which
# way a detected marker lies, so it has to be right even if the picture
# happens to look fine. Tunable live from the Vision tab.
CAM_YAW_OFFSET = 0.0

# Rotation of the PICTURE, degrees clockwise: 0, 90, 180 or 270.
#
# Separate from CAM_YAW_OFFSET on purpose - they answer different questions.
# Yaw is where the camera looks; this is which way up the sensor happens to
# be in its bracket. A camera aimed straight ahead but mounted on its side
# needs rotation 90 and yaw 0.
#
# 0 and 180 are free: they are done in the ISP by CAM_HFLIP/CAM_VFLIP, so
# every consumer including the cliff check sees an upright frame.
#
# *** 90 and 270 rotate the DISPLAY ONLY. ***
# The cliff detector reads the raw frame and assumes the bottom row is the
# nearest floor. Turn the camera on its side and that assumption is simply
# false - near and far end up left and right - so the floor check must be
# switched off, and the page says so when you select them.
CAM_ROTATION = 0

# Horizontal field of view, degrees. This is the one number that ties the
# camera to the rest of the robot: the nav page draws this wedge on the LiDAR
# plot, so you can tell at a glance whether an obstacle the scanner found is
# something the camera can actually show you. Without it the video is just a
# picture next to a map, with no way to relate one to the other.
#
#   Camera Module 1 (ov5647)   53.5
#   Camera Module 2 (imx219)   62.2
#   Camera Module 3 (imx708)   66.0     Module 3 Wide  102.0
#   HQ Camera (imx477)         depends entirely on the lens fitted
#
# *** SET THIS TO MATCH THE MODULE. camera_test.py names the sensor.
# Measured on the robot 2026-09-08: camera_test.py --list reported
# "Camera Module 1 (ov5647)", so 53.5. Was 66.0 (Module 3), which claimed
# 12 degrees of view the lens does not have.
CAM_HFOV = 53.5

# Vertical field of view. Derived from CAM_HFOV and the frame aspect ratio
# rather than stored: for a rectilinear lens the two are tied together, and
# two independent numbers is two things to get out of step.
#
#   tan(vfov/2) = tan(hfov/2) * height / width


def cam_vfov(hfov=None, size=None):
    import math
    hfov = CAM_HFOV if hfov is None else hfov
    w, h = CAM_SIZE if size is None else size
    return math.degrees(2 * math.atan(math.tan(math.radians(hfov) / 2) * h / w))


# --- Camera geometry for the cliff detector --------------------------------
# Both of these are MEASURED, and the cliff detector is useless without them.
#
# Height of the LENS above the floor, mm.
CAM_HEIGHT_MM = 120.0

# Downward tilt, degrees. 0 = pointing at the horizon, + = tilted DOWN.
#
# *** A camera at 0 tilt is nearly useless for cliff detection. ***
# Pointing at the horizon, everything above the frame centre is AT or ABOVE
# the horizon and never meets the floor at all, so five of the detector's six
# rows see no floor to check. The one row that does starts about 350 mm out
# (120 mm up, 41 deg vertical FOV), leaving the near field — the part that
# matters for stopping — completely unwatched.
#
# Tilt DOWN 15-25 deg so the bottom of the frame lands just in front of the
# wheels. At 15 deg the six rows land at roughly 180, 210, 240, 290, 360 and
# 450 mm, which brackets the bumper properly. Verify with
# camera_test.py --stream: the bottom edge of the picture should show floor
# roughly at the front bumper.
CAM_PITCH_DEG = 15.0

# --- ArUco markers ---------------------------------------------------------
# Printed tags give the one thing SLAM cannot produce on its own: an ABSOLUTE
# position fix. Scan matching corrects drift against the map it built, so a
# map that has slowly bent takes the pose with it. A tag at a known place is
# outside that loop.
#
# Dictionary. 4X4_50 on purpose: the fewer bits per tag, the larger each bit
# is on paper for a given tag size, and the further away it can be read. Fifty
# distinct ids is far more than a house needs.
MARKER_DICT = "DICT_4X4_50"

# Printed size of the BLACK SQUARE, mm — not the white border, not the paper.
# *** MEASURE WHAT YOU ACTUALLY PRINTED. Every distance the detector reports
# *** scales directly with this number, so a tag printed at 92 mm and declared
# *** as 100 puts every fix 9% too far away.
MARKER_SIZE_MM = 100.0

# Ignore tags further away than this. Range error from a corner-detection
# wobble grows with the SQUARE of distance, so a far tag is not a weak fix,
# it is a misleading one. At 100 mm tags and 640x480 this is about the limit
# of honest measurement anyway.
MARKER_MAX_MM = 2500.0

# How hard a fix pulls the pose, 0-1. Not 1.0: a hard snap teleports the
# robot mid-map and smears the next scan across the jump. 0.35 converges in
# three or four sightings and stays smooth.
MARKER_FIX_GAIN = 0.35

# A fix further than this from the current pose is not believed — it means a
# duplicated tag, a wrong map entry, or a tag someone moved. Reported on the
# page rather than applied.
MARKER_SANITY_MM = 1500.0


# --- Servo (signal only) ---------------------------------------------------
# Reserved for one servo, added later. Signal wire only — power it from the
# 5 V buck, not from a Pi header pin.
#
# Both hardware PWM channels (GPIO12/13) are on the motors, so drive this
# through pigpio for clean 50 Hz timing, or the servo will jitter under load:
#     sudo apt install -y pigpio python3-pigpio
#     sudo systemctl enable --now pigpiod
#     AngularServo(SERVO, pin_factory=PiGPIOFactory())
SERVO = 17      # header 11

# --- 1.54" 240x240 IPS LCD, ST7789, SPI --------------------------------------
# On SPI0, the bus the header has always reserved for SPI, so nothing above
# moves. Full wiring and the reasoning in WIRING.md section 14.
#
#   GND -> header 25          VCC -> header 17 (3V3, NOT 5 V)
#   SCL -> header 23 GPIO11   SDA -> header 19 GPIO10 (MOSI)
#   CS  -> header 24 GPIO8    DC  -> header 26 GPIO7
#   RES -> 3V3 (jumper to VCC at the display)
#   BLK -> 3V3 (jumper to VCC at the display)
#
# Needs, in /boot/firmware/config.txt, then a reboot:
#     dtparam=spi=on
#     dtoverlay=spi0-1cs
# Without spi0-1cs the SPI driver claims GPIO7 as a second chip select and
# DC cannot be opened ("GPIO busy").
#
# Why DC is on GPIO7 and RES/BLK have no pin: every other free-looking GPIO
# already has an owner. GPIO17 = SERVO above. GPIO4 = the MAX98357A's SD
# (the max98357a overlay claims it unless `no-sdmode` is set, and WIRING.md
# section 7 keeps it for software mute). GPIO14/15 = the ESP32 UART (section
# 10). GPIO0/1 = the HAT ID EEPROM, read by the firmware at boot. GPIO9 =
# MISO, owned by the SPI driver even though the display never reads.
# CE1 is the one pin that becomes free, because the display is the only SPI
# device, so DC takes it. RES is not needed (the driver sends a software
# reset) and BLK tied high just means the backlight is always on.
LCD_SPI_BUS = 0
LCD_SPI_DEV = 0          # CE0 -> GPIO8, header 24
LCD_DC = 7               # header 26 — was CE1, freed by dtoverlay=spi0-1cs
LCD_RST = None           # None = RES tied to 3V3. A BCM number if ever wired.
LCD_BLK = None           # None = BLK tied to 3V3. A BCM number gives dimming.

# 32 MHz asks the Pi 4 for 31.25 MHz (125 MHz / 4): a full frame in ~40 ms.
# If the picture tears or shows speckles, the wires are too long for it —
# drop to 16_000_000 before blaming the display.
LCD_SPI_HZ = 32_000_000
LCD_SPI_MODE = 0         # modules WITH a CS pin; CS-less ones need 3
LCD_SIZE = (240, 240)

# 0, 90, 180 or 270 — whichever way up the display is mounted. Check with
# `python test/display_test.py`: the arrow marked TOP should point up.
LCD_ROTATION = 0

# IPS panels are inverted at the glass. True for this module; if the test
# screen shows a WHITE background, set False.
LCD_INVERT = True

# Red and blue swapped on the test bars (R shows blue)? Set True.
LCD_BGR = False

# Status screen refresh, Hz. It only sends a frame when something changed,
# and 2 is plenty for an address, a heading and a song title.
LCD_FPS = 2.0

# --- Tuning ----------------------------------------------------------------
PWM_HZ = 1000   # 1 kHz. Audible whine but well inside the TB6612's range.
                # Raise toward 20 kHz to move the whine above hearing; the
                # Pi's software PWM gets less accurate as frequency climbs.

MAX_DUTY = 0.40 # Hard ceiling on every test script, 0.0-1.0.
                # Deliberately low: full duty on a bench-tested robot that
                # is still on the floor is how things get broken. Raise it
                # once the drive train is proven.
