# CamStreamer — flashing the ESP32-CAM (camera board)

This guide assumes you've never flashed an ESP32-CAM before. It's a
different (and fiddlier) process than the DevKit because this board has
no built-in USB port.

## What this board does

The ESP32-CAM is a dedicated camera streamer. It joins the home WiFi
router and serves live video (MJPEG) plus single JPEG snapshots. Nothing
else runs on this board on purpose — video never passes through the
DevKit or the Pi, it goes straight from this board to whatever's
watching (browser, or later, the Pi's OpenCV code).

## What you need

- A **USB-to-serial (FTDI) adapter** — the ESP32-CAM has no USB port, so
  you need a small USB-to-TTL adapter board to program it. Must run at
  **3.3V logic** (most are switchable 3.3V/5V — set it to 3.3V).
- Jumper wires.
- A real **5V, 2A or better power supply** for running it (a phone
  charger rated 2A+ works; a laptop's USB port often does not — see the
  power warning below).
- A finished Arduino IDE setup with ESP32 board support already installed
  — if you haven't done that yet, follow Steps 1-3 in
  [`../CarTestBench/README.md`](../CarTestBench/README.md) first (same
  IDE, same ESP32 board package, shared across both boards).

## Step 1 — Wire up for flashing

Connect the USB-to-serial adapter to the ESP32-CAM:

| Adapter pin | ESP32-CAM pin |
|---|---|
| 5V (or 3.3V, check your board) | 5V |
| GND | GND |
| TX | U0R |
| RX | U0T |

Then, to put the board into **flashing mode**, connect a jumper wire from
**GPIO0 to GND**. This must be in place *before* power-up/reset and stays
connected for the whole upload — remove it only after flashing succeeds
(Step 4).

## Step 2 — Select the board in Arduino IDE

1. **Tools → Board → esp32 → AI Thinker ESP32-CAM**.
   (If that exact entry isn't in your board list, use "ESP32 Dev Module"
   instead, then set **Tools → Partition Scheme → Huge APP** and make
   sure PSRAM is enabled — but "AI Thinker ESP32-CAM" is simpler if it's
   available.)
2. **Tools → Port** → the COM port for your USB-serial adapter.

## Step 3 — Open the sketch and upload

1. **File → Open** → `CamStreamer/CamStreamer.ino`.
2. With the GPIO0→GND jumper still connected, press the board's **RST**
   button, then click **Upload** in the Arduino IDE.
3. Watch the log at the bottom. If it hangs at "Connecting....." for a
   long time, press RST again right as upload starts — ESP32-CAM boards
   are notoriously timing-sensitive about this.
4. Wait for **"Done uploading."**

## Step 4 — Remove the flashing jumper and power it properly

1. Disconnect the **GPIO0-to-GND** jumper (leaving it connected keeps the
   board stuck in flashing mode — it won't run your program).
2. **Power it from a real 5V/2A+ supply, not through the USB-serial
   adapter.** This matters: a past debugging session on this project
   found the CAM would join WiFi, run for a few minutes, then hard crash
   and reboot — the cause was WiFi transmit current spikes sagging the
   underpowered USB-serial adapter, not a code or WiFi-credentials
   problem. A phone charger (2A+) direct to the board's 5V/GND pins fixed
   it. If you still see random reboots after a minute or two of
   streaming, suspect power first.
3. Press RST (or power-cycle) to boot normally.

## Step 5 — Verify it worked

1. Open **Tools → Serial Monitor** at **115200 baud** (can use either
   adapter — TX/RX are still wired from Step 1).
2. You should see it join WiFi and print its IP:
   ```
   Joining 'Airtel_kuma_9602'.....
   Connected. Stream: http://192.168.1.52:81/stream
   ```
3. From any device on the home WiFi, open a browser to:
   - `http://192.168.1.52/status` — should return JSON like
     `{"frames":123,"bytes":...,"ms":...,"streaming":true}`
   - `http://192.168.1.52/capture` — should show a single still photo
   - `http://192.168.1.52:81/stream` — should show live video

If `/status` works but the stream doesn't load, double check nothing
else on the network is already using port 81, and that the power supply
is the real 5V/2A one (not the flashing adapter).

## Notes

- `AP_SSID`/`AP_PASS`/the static IP are duplicated at the top of
  `CamStreamer.ino` because Arduino sketches can't share headers across
  folders — they're kept in sync with `CarTestBench/config.h` by hand.
  See `docs/hardware-architecture/v4-home-wifi-current.md` if either
  changes.
- Don't add motor control, sensors, or anything else to this sketch —
  it's a dedicated camera appliance on purpose.
