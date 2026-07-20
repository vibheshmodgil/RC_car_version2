# CarTestBench — flashing the ESP32 DevKit (motor controller)

This guide assumes you've never used the Arduino IDE before. It walks
through getting `CarTestBench.ino` onto the ESP32 DevKit board from a
completely fresh computer.

## What this board does

The ESP32 DevKit V1 is the car's motor controller. It drives all 4
wheels (via 2x TB6612FNG motor driver chips), reads the 4 wheel encoders,
and runs a small web server so the Raspberry Pi (or a phone, as a
fallback) can send drive commands to it. It is the last line of defense
for safety — if it stops hearing commands, it stops the car on its own
(the "drive deadman", see `CLAUDE.md`).

You do **not** need to know C++ to flash this board — just follow the
steps below in order.

## What you need

- A Windows/Mac/Linux computer.
- A USB cable that can carry data (some cheap cables are power-only — if
  the board never shows up as a COM port, try a different cable first).
- The ESP32 DevKit V1 board itself, connected via USB.

## Step 1 — Install the Arduino IDE

1. Download the Arduino IDE (version 2.x) from the official Arduino
   website and install it like any other program.
2. Open it once to make sure it launches.

## Step 2 — Add ESP32 board support

The Arduino IDE doesn't know about ESP32 boards until you tell it where
to find them.

1. Open **File → Preferences** (Windows/Linux) or **Arduino IDE →
   Settings** (Mac).
2. Find the field called **"Additional Boards Manager URLs"** and add:
   ```
   https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
   ```
   (If there's already a URL in that box, put this on a new line — the
   box supports multiple URLs separated by commas.)
3. Click OK.
4. Open **Tools → Board → Boards Manager**, search for `esp32`, and
   install the package published by **Espressif Systems**. This project
   uses Arduino-ESP32 **core 3.x** — install the latest 3.x version, not
   a 2.x one.

This step needs internet access and can take a few minutes.

## Step 3 — Install 3 required libraries

Open **Tools → Manage Libraries...** (or **Sketch → Include Library →
Manage Libraries**) and install each of these by searching the exact
name and clicking Install:

- `ESPAsyncWebServer`
- `AsyncTCP`
- `ArduinoJson`

If the library manager shows multiple results for a name, pick the one
whose title matches exactly (there are similarly-named third-party
forks).

## Step 4 — Open the sketch

1. In the Arduino IDE: **File → Open**, browse to this repo, and open
   `CarTestBench/CarTestBench.ino`.
2. The IDE will show several tabs along the top (`config.h`,
   `MotorChannel.h`, etc.) — that's normal, it's one sketch made of
   several files.

## Step 5 — Select the board and port

1. **Tools → Board → esp32 → ESP32 Dev Module**.
2. **Tools → Port** → pick the COM port (Windows) or `/dev/tty...`
   (Mac/Linux) that appeared when you plugged the board in. If nothing
   shows up, try a different USB cable or a different USB port before
   anything else — this is the #1 cause of "it won't upload."
3. Leave the other Tools settings at their defaults unless a specific
   guide tells you otherwise.

## Step 6 — Upload

Click the **Upload** button (the right-arrow icon). The IDE will compile
the sketch, then flash it to the board. The board's onboard LED usually
flickers during flashing. Wait for **"Done uploading."** in the log
at the bottom — don't unplug the board before that.

If you get a "Failed to connect" error: hold the **BOOT** button on the
DevKit while upload starts (some boards need this), release it once
"Connecting..." changes to "Writing...".

## Step 7 — Watch it boot and join WiFi

1. Open **Tools → Serial Monitor**.
2. Set the baud rate dropdown (bottom-right of the Serial Monitor window)
   to **115200** — wrong baud rate shows garbled text.
3. Press the board's **RST/EN** button to reboot it, or unplug/replug USB.
4. You should see boot logs, then something like:
   ```
   Joining 'Airtel_kuma_9602'.....
   Connected. IP: 192.168.1.51
   ```

That IP (`192.168.1.51`) should match `STA_STATIC_IP` in `config.h` — if
it does, the board successfully joined the home WiFi router and is ready
for the Pi (or a browser at `http://192.168.1.51`) to talk to it.

## Safety before your first drive test

- **Wheels off the ground** for the very first test after any firmware
  change.
- E-stop is available from any UI: `POST /api/estop`, or the e-stop
  button on the dashboard/fallback UI.
- Full pin map, wiring rules, and the REST/WS API contract are in the
  root `CLAUDE.md` — read that if you're adding features, not just
  flashing what's already here.
