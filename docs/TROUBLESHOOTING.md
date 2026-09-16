# Troubleshooting

Symptoms first. Most of these have been hit for real on this robot.

---

## Can't reach the Pi

### `ssh: connect to host … port 22: Connection timed out`

**The IP has almost certainly moved.** It has changed three times so far.
Scan for it rather than guessing:

```powershell
1..254 | ForEach-Object {
  $c = New-Object Net.Sockets.TcpClient
  @{ip="192.168.1.$_"; c=$c; r=$c.BeginConnect("192.168.1.$_",22,$null,$null)}
} | ForEach-Object -Begin { Start-Sleep 4 } -Process {
  if ($_.r.IsCompleted -and $_.c.Connected) { $_.ip }; $_.c.Close()
}
```

`ping` alone is misleading here — a Pi can be pingable with SSH down, or (seen
on this robot) answer TCP on port 22 while dropping half its pings.

Set a **static DHCP lease** on the router and this stops happening.

### SSH connects, then drops mid-session

```
client_loop: send disconnect: Connection reset
```

Two candidates, and they are distinguishable:

**Power brownout.** Wi-Fi's power amplifier draws in bursts and is usually the
first thing to fail on a sagging 5 V rail. Adding the camera (~250 mA) on top
of the Pi and a USB LiDAR is enough to tip a marginal buck converter over.

```bash
vcgencmd get_throttled      # anything but throttled=0x0 is undervoltage
uptime                      # near zero means it rebooted, not just dropped off
dmesg | grep -i under-voltage | tail -5
```

To confirm: power the Pi from a known-good 5 V ≥ 3 A supply instead of the
buck. **Disconnect the existing feed first** — never two supplies at once, and
the GPIO 5 V pins have no protection. **Keep the grounds tied** to the battery
side or the motor drivers lose their voltage reference.

**Wi-Fi saturation.** The cockpit pulls roughly 3.7 Mbps with video on — MJPEG
~2.3, `/map` ~0.6, `/state` ~0.8. On a weak link that can starve SSH.

Telling them apart: congestion leaves the Pi *pingable* and merely makes SSH
sluggish. If it stops responding to ping entirely, it is power or the
interface resetting, not bandwidth. Turn **Live view** off on the Vision tab
to drop most of the traffic.

Check signal strength:

```bash
iwconfig wlan0 | grep -i quality
```

---

## Syncing

### `scp: stat local "Speaker_trucktest": No such file or directory`

**You ran `scp` inside the SSH session.** It runs on the machine that owns the
local path, so on the Pi, `C:` looks like a hostname and the backslash gets
eaten.

`exit` first, then run it in Windows PowerShell or Git Bash.

### Changes don't appear on the Pi

- Did the sync actually run? `ls -la ~/Desktop/Speaker_truck/test/` and check
  the timestamps.
- Note the `test/` on the **end** of the scp destination, or files land in the
  wrong directory.
- Is the script still running? Python read the old file at import; restart it.

---

## Python and the venv

### `ModuleNotFoundError: No module named 'pins'`

The file is in the project root instead of `test/`.

```bash
mv ~/Desktop/Speaker_truck/<file>.py ~/Desktop/Speaker_truck/test/
```

### `No module named 'flask'` / `serial` / `smbus2`

The venv is not active. Look for `(.venv)` in the prompt.

```bash
cd ~/Desktop/Speaker_truck && source .venv/bin/activate
```

### `No module named 'gpiozero'` / `picamera2` / `cv2` **with the venv active**

The venv was created without `--system-site-packages`. Those packages come
from apt and the venv cannot see them otherwise:

```bash
cd ~/Desktop/Speaker_truck
rm -rf .venv
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
pip install -r requirements.txt
```

Never `pip install` a hardware package. `pip install lgpio` compiles from
source and fails on missing swig; `pip install opencv-python` gets you an
OpenCV with **no aruco module**. Use apt.

---

## Motors

### `GPIO setup failed` / pins busy

Another GPIO-owning script is running. Only one at a time:

| Script | Port | Owns GPIO |
|---|---|---|
| `web_dashboard.py` | 5000 | yes |
| `web_drive.py` | 5001 | yes |
| `lidar_view.py` | 5002 | no |
| `web_pilot.py` | 5003 | yes |
| `web_nav.py` | 5004 | yes |

```bash
pkill -f web_ ; sleep 1
```

### Robot drives backwards, or spins instead of going straight

Use the inversion toggles on the **Sensors** tab to find which side is wrong,
then **fix it in the wiring** (`WIRING.md` §4 — swap red and white on both
motors of that side). Software inversion hides the fault from every other
script.

| Symptom | Toggle |
|---|---|
| Whole robot drives backwards | Invert LEFT **and** RIGHT |
| Spins instead of going straight | Invert whichever side is wrong |
| ← turns right | Swap SIDES |

---

## Sensors not detected

### `lidar: NONE`

```bash
ls -l /dev/ttyUSB* /dev/ttyACM*     # does it enumerate?
lsusb                               # is the adapter seen at all?
```

The scanner motor needs 5 V, not just the data line. If it enumerates but
delivers nothing, check the baud (115200) and that no other script has the
port open.

### `imu: NONE — no supported IMU found on i2c-1`

```bash
i2cdetect -y 1      # expect 0x28/0x29 (BNO055) or 0x68/0x69 (MPU/ICM)
```

Nothing at all: I2C is disabled (`sudo raspi-config` → Interface Options →
I2C), or SDA/SCL are swapped. **IMU VCC must be 3.3 V, not 5 V** — GPIO2/3
have fixed pull-ups to 3.3 V and a 5 V-powered IMU back-feeds them.

### `cam: NONE`

```bash
rpicam-hello --list-cameras
```

- Was the Pi **powered down** when the ribbon went in? CSI is not
  hot-pluggable.
- Right socket? **CAMERA**, between the HDMI ports and the audio jack. The one
  marked **DISPLAY** is DSI and looks nearly identical.
- Contacts face the HDMI ports at the Pi end, away from the lens at the camera
  end.

### Camera lists but delivers no frames

A ribbon seated well enough to enumerate but not to stream. Power down, reseat
**both** ends firmly, retry. `camera_test.py` calls this out by name.

### `Camera failed to open: Device or resource busy`

Something else has it. `web_nav.py` and `camera_test.py` cannot share the
camera. Stop the other one.

### The mic hears me but the truck never answers

Look at the line above the mic button on the phone page, or the Audio tab's
**Assistant** card in the cockpit:

- **"The brain PC is not reachable"** — in order: is the PC on and awake? Is
  Ollama running (tray icon)? Was it restarted after setting
  `OLLAMA_HOST=0.0.0.0:11434`? Is the firewall rule there, and the wifi
  profile Private? From the Pi, `curl http://<pc-ip>:11434/api/tags` must
  answer. If the PC's IP changed, fix it on the Assistant card.
- **"Brain model missing"** — on the PC: `ollama pull qwen3-vl:4b-instruct` (or pick an
  installed model on the Assistant card).
- **"Waking up the brain"** — the model is loading into the GPU; the first
  time after the PC starts this takes up to a minute.
- **"Assistant is off"** — press **Assistant OFF** on the Audio tab.
- Nothing at all, and no `brain:` line at web_nav start-up — the Pi is
  running old code; sync `test/*.py` again.

Words that are obviously background noise are ignored on purpose.

### Answers are slow

Watch **Heard → first word** on the Assistant card. The model must fit in the
GPU's memory: on the PC run `ollama ps` while it is answering — `100% GPU` is
right; any CPU share means it is too big and every answer crawls. On a 4 GB
card use `qwen3-vl:4b-instruct` or `qwen3-vl:2b-instruct`, not 7–8B models.

**Always an `-instruct` build.** The plain `qwen3-vl:2b` / `:4b` tags are the *thinking* variants: they reason silently before every answer, even when told not to — measured 78 s of silence for a one-line joke. The Assistant card warns when the model does this.

### Phone mic button does nothing

The status line under the button now says why. In order of likelihood:

1. **Use the IP, not `shiv.local`.** Chrome on Android usually cannot
   resolve `.local` names. The cockpit's Audio tab and web_nav's start-up
   print show the right link, e.g. `https://192.168.1.10:5443/talk`.
2. **It must be https on 5443.** Over plain http the browser refuses the
   microphone; the page shows a yellow box with the link.
3. **Certificate warning:** tap **Advanced → Proceed** once.
4. **"Microphone blocked":** tap the icon left of the address → Permissions →
   Microphone → Allow, then tap the mic again.
5. **"needs internet":** Android's speech recognition runs on Google's
   service; the phone needs mobile data or wifi with internet.

If Chrome on the phone will not allow the microphone on the self-signed
https page at all, skip https instead: on the phone open
`chrome://flags/#unsafely-treat-insecure-origin-as-secure`, add
`http://192.168.1.10:5004`, Enable, Relaunch — then use
`http://192.168.1.10:5004/talk`. The page detects the change by itself.

### Claude says "cannot reach the truck"

`web_nav.py` must be running, and `TRUCK_URL` in the `claude mcp add` line
must match — try the IP instead of `shiv.local`:
`claude mcp remove truck`, then add it again with `TRUCK_URL=http://192.168.1.11:5004`.

### Claude says the motors are disabled

Working as intended: press **ENABLE** in the cockpit. The AI cannot arm
the motors on its own.

### Claude answers itself / repeats what the truck said

The phone is transcribing the speaker. The page pauses the mic while the
truck talks and the Pi drops anything heard up to 1 s after, but a phone
right against the speaker can still catch the tail — move it further away.

### `speech: Piper MISSING`

In the venv: `pip install "piper-tts>=1.3"`, then restart `web_nav.py`. If pip
tries to *build* something, it picked an old 1.2 release — the version pin
matters, 1.2 has no wheel for Python 3.13.

### Speak does nothing / "no voice downloaded yet"

Open **Voices** under the Speak box and press Download on one, or
`python test/speaker_test.py --download en_US-lessac-medium`. The Pi needs
internet for that one step; after it, speech works offline.

### A voice download fails with 404

That voice was renamed or removed on the Piper voice server. Pick another;
the full list is at huggingface.co/rhasspy/piper-voices.

### Speech is slow

The first sentence after starting or changing voice loads the model — a few
seconds, once. After that a medium voice takes about a second per sentence.
`high` voices take roughly as long as the speech itself on a Pi 4; use a
medium one. Synthesis runs at low priority so SLAM always comes first — a
heavy mapping session makes speech wait, by design.

### `lcd:   NONE — GPIO7 busy`

The SPI driver owns GPIO7 as chip select 1. Add `dtoverlay=spi0-1cs` under
`dtparam=spi=on` in `/boot/firmware/config.txt` and reboot.
`ls /dev/spidev*` must then list only `spidev0.0`.

### `lcd:   NONE — /dev/spidev0.0 missing`

SPI is off. Add both `dtparam=spi=on` and `dtoverlay=spi0-1cs`, reboot.

### LCD lit but white, or shows nothing but the backlight

Power is fine and no data is arriving. In order: `SDA` on header 19 and `SCL`
on 23 (not swapped); `CS` on 24 and `DC` on 26 (easy to swap — they are
neighbours); `GND` connected; `python test/display_test.py --check` all ✓.

### LCD colours wrong, upside down, or a noise band on one edge

`python test/display_test.py` and read the test card: red shows blue →
`LCD_BGR`; negative picture → `LCD_INVERT`; arrow not at the top →
`LCD_ROTATION`; noise band → try the rotation 180° from the one you want and
flip the module instead. All in `test/pins.py`.

---

## Mapping is wrong

### The map builds mirrored, or the robot reverses through its own map

**Encoder sign.** Push the robot forward by hand and watch *Encoder counts* on
the Map tab — both must increase. Flip the offending sign on the **Tune** tab.

### The map is uniformly too big or too small

**Counts per rev**, and nothing else. See
[TUNING.md § Odometry](TUNING.md#odometry) for the calibration procedure.

### The map swings every time the robot turns on the spot

**LiDAR mounting position.** The scanner is on a corner, 250 mm from the body
centre. If X/Y are wrong the scan origin orbits the true centre of rotation.

### Walls come out smeared into arcs

Heading is bad. Check the IMU is being used — *Match correction* should be
small and *heading_ok* true. Without an IMU, SLAM falls back to wheel-derived
heading, which on a skid-steer slips on every turn by design.

### The robot jams for no visible reason

**LiDAR rotation.** The guard is checking a different direction than you think
while the plot still looks sensible. Face a wall and check the Drive plot
shows it square across the top.

Since the plot now draws in the true body frame, the *Ahead* number and where
the wall appears must agree. If they don't, that is the bug — it used to be
250 mm out because the plot skipped the scanner's position.

### Everything is refused and the robot won't move

`no fresh LiDAR scan` — the guard refuses all motion without ranging data,
deliberately. A guard that silently stops guarding when its sensor dies is
worse than no guard.

If it is wedged against furniture with every direction blocked, it should
creep out on its own. If not, raise *Creep margin* on the Tune tab.

---

## Floor check and markers

### The floor check fires constantly on normal floor

Press **Relearn floor** on open floor — not facing a wall, since it takes
whatever it sees as the definition of floor. If it still fires on a patterned
rug, raise *Floor chroma tolerance* on the Tune tab.

### The floor check never fires, even at a step

The camera is not tilted down enough. At 0° tilt, five of the six sample rows
are at or above the horizon and see no floor at all. Tilt down 15–25° and set
`CAM_PITCH_DEG`. Check the framing with `camera_test.py --stream` — the bottom
edge should show floor at about the front bumper.

### Markers are detected but the distance is wrong

`MARKER_SIZE_MM` does not match what you printed. Measure a printed tag with a
ruler, black square only. Every distance scales linearly with it.

### A marker fix gets rejected

`N mm jump from tag X` on the Vision tab means two tags share an id, a tag was
learned while the pose was already wrong, or someone moved a tag. Wipe the map
and relearn:

```bash
python test/marker_test.py --forget all
```

---

## The cockpit page

### Blank page, or nothing updates

Open the browser console. A JavaScript syntax error takes the whole page down
at once — a blank page with a working `/state` is almost always that rather
than a robot problem.

```bash
curl -s http://<pi-ip>:5004/snapshot | python3 -m json.tool
```

If that returns data, the robot is fine and the page is the problem.

### The video stutters and the controls lag

Turn **Live view** off on the Vision tab. MJPEG is by far the largest thing on
the page and will starve the drive commands before anything else gives way.

### Arrow keys do nothing

Click the page once for keyboard focus.
