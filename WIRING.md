# Wiring & Hardware Architecture

4-wheel differential-drive (skid-steer) speaker truck.
Raspberry Pi 4 Model B = high-level controller. TB6612FNG × 2 = low-level
motor drivers. 2D LiDAR = primary perception sensor.

> **Read `## Verify before power-on` at the bottom before connecting the
> battery.** Several numbers below are assumptions from typical part specs,
> not from your specific parts. They are marked ⚠.

---

## 1. System architecture

```
                    3S Li-ion  (11.1 V nom / 12.6 V full / 9.0 V cutoff)
                          │
              ┌───────────┴─────────────┬──────────────────┐
              │                         │                  │
        ┌─────┴──────┐          ┌───────┴──────┐    ┌──────┴───────┐
        │ 5 V buck   │          │  TB6612 #1   │    │  TB6612 #2   │
        │ (≥5 A) ⚠   │          │   LEFT side  │    │  RIGHT side  │
        └─────┬──────┘          │  VM = 11.1 V │    │  VM = 11.1 V │
              │                 └───┬──────┬───┘    └──┬───────┬───┘
      ┌───────┴────────┐            │      │           │       │
      │                │           M1     M2          M3      M4
┌─────┴─────┐   ┌──────┴──────┐   front   rear       front    rear
│  Pi 4 5 V │   │ Audio amp ⚠ │    left   left       right    right
│    3 A    │   └──────┬──────┘
└─────┬─────┘          │
      │             speaker
      ├── USB ──── 2D LiDAR
      │
      └── GPIO ─── TB6612 #1 + #2 logic (3.3 V)

        ALL GROUNDS COMMON — battery, buck, drivers, Pi, amp
```

**Control flow:** LiDAR → USB → Pi (perception) → GPIO PWM + direction →
TB6612 → motors. The TB6612s are dumb H-bridges; all logic lives on the Pi.

---

## 2. Power rails

| Rail | Source | Feeds | Notes |
|---|---|---|---|
| 11.1 V (VM) | 3S pack direct | TB6612 VM ×2 | 12.6 V at full charge |
| 5 V | Buck converter | Pi 4, LiDAR | Pi 4 needs **3 A** on its own |
| 3.3 V | Pi pin 1 / 17 | TB6612 VCC ×2 | logic only, ~mA |
| Amp supply | ⚠ see below | audio amp | depends on your amp |

### Battery

3S Li-ion: **12.6 V** full, **11.1 V** nominal, **9.0 V** cutoff (3.0 V/cell).
Do not discharge below 9.0 V. Use a pack with a BMS, or add a low-voltage
alarm — Li-ion cells are permanently damaged below ~2.5 V/cell.

### Why the Pi gets its own buck

Four motors starting at once pull a large inrush from the pack and drag the
rail down. If the Pi shares that sag it **browns out and reboots mid-drive**.
The buck converter isolates it, but only if the buck can hold 5 V while the
pack dips. Size the buck for the Pi alone, not for the motors.

### Common ground is mandatory

Battery −, buck −, both TB6612 GND pins, Pi GND, and amp GND must all be
tied together. Without a common reference the TB6612 sees garbage on its
logic inputs and motors twitch or run away. This is the single most common
cause of "it worked on the bench, then went crazy."

---

## 3. GPIO assignments (BCM numbering)

Differential drive means both left motors always share a speed and direction,
and likewise on the right. So on each driver, **tie the A and B channel
control pins together** — one PWM and one direction pair per side.

### Motor control

| Signal | BCM | Header pin | Goes to |
|---|---|---|---|
| `LEFT_PWM` | **GPIO12** | 32 | TB6612 #1 — PWMA **and** PWMB |
| `LEFT_IN1` | **GPIO23** | 16 | TB6612 #1 — AIN1 **and** BIN1 |
| `LEFT_IN2` | **GPIO24** | 18 | TB6612 #1 — AIN2 **and** BIN2 |
| `RIGHT_PWM` | **GPIO13** | 33 | TB6612 #2 — PWMA **and** PWMB |
| `RIGHT_IN1` | **GPIO27** | 13 | TB6612 #2 — AIN1 **and** BIN1 |
| `RIGHT_IN2` | **GPIO22** | 15 | TB6612 #2 — AIN2 **and** BIN2 |
| `STBY` | **GPIO25** | 22 | **Both** drivers' STBY pins |

GPIO12 and GPIO13 are the Pi's two **hardware PWM** channels (PWM0 / PWM1) —
one per side, which is exactly what differential drive needs. Software PWM on
other pins jitters under CPU load and makes the robot veer.

`STBY` low = both drivers disabled, outputs floating. This is the emergency
stop and the safe boot state.

### Encoders

The JGB37-520 has 6 wires — motor pair plus a quadrature hall encoder. Only
one encoder per side is needed for odometry (both wheels on a side turn
together), so wire the two front motors' encoders and leave the rear ones
unconnected.

| Signal | BCM | Header pin | Wire |
|---|---|---|---|
| `LEFT_ENC_A` | **GPIO5** | 29 | **yellow**, front-left |
| `LEFT_ENC_B` | **GPIO6** | 31 | **green**, front-left |
| `RIGHT_ENC_A` | **GPIO16** | 36 | **yellow**, front-right |
| `RIGHT_ENC_B` | **GPIO26** | 37 | **green**, front-right |

Hall **VCC = blue → 3.3 V**, hall **GND = black → common ground**.
Full colour code and the motor→driver→Pi chain: **§4 below.**

⚠ **Hall VCC (blue) goes to 3.3 V, never 5 V.** The encoder's output swings
to whatever you feed VCC. At 5 V it pushes 5 V into a 3.3 V-max GPIO and
damages the pin.

### Servo

| Signal | BCM | Header pin |
|---|---|---|
| `SERVO` | **GPIO17** | 11 |

Signal wire only. Power the servo from the **5 V buck**, never a Pi header
pin — a servo stalls at 0.5–2 A and will brown the Pi out. Grounds common.

Both hardware PWM channels (GPIO12/13) are on the motors, so drive the servo
through `pigpio` for DMA-timed 50 Hz pulses; plain software PWM will make it
buzz and drift once LiDAR processing loads the CPU.

```bash
sudo apt install -y pigpio python3-pigpio
sudo systemctl enable --now pigpiod
```

### Reserved — do not reuse for motors

| Interface | BCM | Reserved for |
|---|---|---|
| I2S | 18, 19, 20, 21 | I2S audio amp (MAX98357A etc.) |
| I2C | 2, 3 | **9-axis IMU** — in use, see §11 / OLED |
| UART | 14, 15 | serial LiDAR, if not USB |
| SPI | 7, 8, 9, 10, 11 | spare |

The motor pins above deliberately avoid all of these, so the I2S amp (§7)
and the IMU (§11) both dropped in without moving a single motor pin.

---

## 4. Motor wire colours — motor → driver → Pi

Each JGB37-520 has **6 wires**: two heavy ones for the motor itself, four thin
ones for the hall encoder. They are two electrically separate circuits that
share nothing but the common ground — the motor half swings 11.1 V at PWM
frequency, the encoder half is 3.3 V logic.

### The colour code

| Wire | Gauge | Function | Connects to | Level |
|---|---|---|---|---|
| **Red** | heavy | Motor **+** | driver output — `AO1` / `BO1` | 11.1 V PWM |
| **White** | heavy | Motor **−** | driver output — `AO2` / `BO2` | 11.1 V PWM |
| **Blue** | thin | Hall **VCC** | Pi **3V3** — header 1 or 17 | 3.3 V |
| **Black** | thin | Hall **GND** | common ground | 0 V |
| **Yellow** | thin | Hall **A** (C1) | Pi GPIO — `ENC_A` | 3.3 V logic |
| **Green** | thin | Hall **B** (C2) | Pi GPIO — `ENC_B` | 3.3 V logic |

C1 gives **11 pulses per revolution of the motor shaft** — before the gearbox.
That is where `COUNTS_PER_REV = 1320` comes from: 11 × 30:1 gearbox × 4
(quadrature edges). ⚠ Still measure it; the gear ratio is inferred from the
330 RPM rating, not read off the motor.

### Two traps in this colour code

**Black is *not* motor −.** On a 2-wire DC motor black is the negative
terminal, and the reflex is to wire it that way here. On the 6-wire version
black is the encoder's *ground* and white is motor −. Putting 11.1 V PWM on
the black wire destroys the hall sensor instantly.

**White is *not* a signal wire.** It is one of the two motor leads. Landing it
on a GPIO puts 11.1 V into a 3.3 V pin and takes the pin — possibly the Pi —
with it.

Tell them apart by gauge before you trust the colour: the two motor wires are
visibly thicker than the four encoder wires.

### Confirm with a meter before connecting anything

Batches do vary, and this is a five-minute check that prevents a dead motor.
Motor **disconnected from everything**:

| Test | Expected | Meaning |
|---|---|---|
| Ω between **red** and **white** | **1–10 Ω** | these two are the motor winding |
| Ω from either to any thin wire | **open** | motor and encoder are isolated |
| Ω between **blue** and **black** | open / high | hall supply, not a winding |

Then power the encoder alone — blue to 3V3, black to GND, **no motor power** —
and turn the shaft by hand with a meter on yellow, then green. Both should
toggle between ~0 V and ~3.3 V. If a wire never toggles, it is not a signal
wire and the batch's colours differ from the table.

### Motor power chain — which motor goes to which channel

Both motors on a side always run at the same speed and direction, so each
driver takes one side and its two channels are tied together in parallel on
the logic pins.

| Motor | Red → | White → | Driver | Control pins (tied) | Pi header → BCM |
|---|---|---|---|---|---|
| Front-left | `AO1` | `AO2` | **TB6612 #1** | `PWMA` `AIN1` `AIN2` | 32→GPIO12, 16→GPIO23, 18→GPIO24 |
| Rear-left | `BO1` | `BO2` | **TB6612 #1** | `PWMB` `BIN1` `BIN2` | *same three pins, tied to A* |
| Front-right | `AO1` | `AO2` | **TB6612 #2** | `PWMA` `AIN1` `AIN2` | 33→GPIO13, 13→GPIO27, 15→GPIO22 |
| Rear-right | `BO1` | `BO2` | **TB6612 #2** | `PWMB` `BIN1` `BIN2` | *same three pins, tied to A* |

Plus, once per driver: `VM` → battery +, `VCC` → Pi 3V3, `GND` → common,
`STBY` → header 22 (GPIO25), shared by both drivers.

⚠ **The two sides are mirror images.** Wire all four motors red→O1 as above
and the right pair will spin *backwards* relative to the robot, because those
motors physically face the opposite way. Fix it in hardware — swap red and
white on **both right-side motors** — rather than negating one side in code,
so that "IN1 high = forward" stays true everywhere. Determine which side needs
the swap on the bench, wheels off the ground, in step 5 of §9.

### Encoder chain — front motors only

One encoder per side is enough for odometry. Leave the two rear encoders'
thin wires unconnected and tape the ends so they cannot short.

| Motor | Blue (VCC) | Black (GND) | Yellow (A) | Green (B) |
|---|---|---|---|---|
| **Front-left** | header **1** (3V3) | header **9** | header **29** — GPIO5 `LEFT_ENC_A` | header **31** — GPIO6 `LEFT_ENC_B` |
| **Front-right** | header **17** (3V3) | header **39** | header **36** — GPIO16 `RIGHT_ENC_A` | header **37** — GPIO26 `RIGHT_ENC_B` |
| Rear-left | — | — | — | — (unused, tape off) |
| Rear-right | — | — | — | — (unused, tape off) |

If a wheel's counts run backwards when it drives forward, swap that motor's
**yellow and green** — A and B are interchangeable, and swapping them inverts
the decoded direction.

### When the MDD10A replaces the TB6612s (§8)

Colours do not change; the grouping does. Both motors of a side land on one
channel, **paralleled**: red-to-red into `MOT-A`/`MOT-B` for that channel, and
white-to-white into the other terminal. Right-side red/white still swap.
Encoder wiring is untouched.

---

## 5. TB6612FNG wiring (each driver)

| TB6612 pin | Connect to |
|---|---|
| `VM` | Battery + (11.1 V) |
| `VCC` | Pi **3.3 V** (pin 1 or 17) |
| `GND` | Common ground (all three GND pins if broken out) |
| `STBY` | GPIO25 |
| `AIN1` / `BIN1` | side IN1 (tied) |
| `AIN2` / `BIN2` | side IN2 (tied) |
| `PWMA` / `PWMB` | side PWM (tied) |
| `AO1` / `AO2` | motor 1 terminals |
| `BO1` / `BO2` | motor 2 terminals |

**Use 3.3 V for VCC, not 5 V.** The TB6612's logic thresholds scale with VCC.
At VCC = 3.3 V the Pi's 3.3 V outputs are unambiguously high. At VCC = 5 V
you are relying on the input threshold staying below 3.3 V — it usually is,
but it is margin you do not need to spend. ⚠ *Confirm against your board's
datasheet if you have a reason to run 5 V.*

### Direction truth table (per side)

| IN1 | IN2 | PWM | Result |
|---|---|---|---|
| H | L | duty | forward |
| L | H | duty | reverse |
| L | L | any | coast (free-spin) |
| H | H | any | brake (short) |
| any | any | any, STBY=L | disabled |

### Bulk capacitance

Put **470–1000 µF electrolytic across VM and GND**, physically close to each
driver, plus a 0.1 µF ceramic. Motors are inductive and switching them
produces voltage spikes that reach the driver and the shared rail. Without
bulk caps you get resets and, eventually, dead drivers.

---

## 6. LiDAR

Most 2D LiDARs (RPLIDAR A1/A2, YDLIDAR X2/X4) present as **USB serial** via
a CP2102/CH340 adapter → `/dev/ttyUSB0`.

```bash
ls -l /dev/ttyUSB* /dev/ttyACM*
dmesg | tail -20          # right after plugging it in
sudo usermod -aG dialout shiv && logout   # permissions, once
```

⚠ **Power draw is the thing to check.** The scanner motor plus the ranging
core is typically 300–600 mA at 5 V, and spikes at spin-up. A Pi USB port can
supply it, but it comes out of the same 5 V buck budget as the Pi. If the
LiDAR browns out or the Pi reboots when it spins up, power the LiDAR from the
buck directly rather than through the Pi.

If yours is **UART instead of USB**, it needs GPIO14/15 and you must free the
serial console: `sudo raspi-config` → Interface → Serial → login shell **no**,
hardware serial **yes**.

---

## 7. Audio — MAX98357A + 12 W 4 Ω speaker

The MAX98357A is an I2S DAC and class-D amplifier in one chip. Audio arrives
from the Pi as **digital** I2S, so there is no analog stage for motor noise to
get into — which matters on a robot sharing one battery with four motors.

### The mismatch to know about first

| | |
|---|---|
| Speaker rating | **12 W, 4 Ω** |
| Amp output into 4 Ω at 5 V | **~3.2 W** (10% THD) / ~2.5 W (1% THD) |

**This is fine.** A speaker's wattage is a maximum it can survive, not a
requirement — you cannot damage a 12 W speaker with a 3.2 W amp. You simply
will not reach the speaker's full loudness. Getting the full 12 W needs a
different amplifier class entirely (and ~15 W of supply), not a tweak here.

### Connections

| MAX98357A pin | Connect to | Pi header | Notes |
|---|---|---|---|
| `Vin` | **5 V from the buck** | — | *not* a Pi 5 V header pin — see below |
| `GND` | common ground | 6/9/14/20/25/30/34/39 | |
| `BCLK` | GPIO18 `PCM_CLK` | **12** | bit clock |
| `LRC` | GPIO19 `PCM_FS` | **35** | word select / left-right clock |
| `DIN` | GPIO21 `PCM_DOUT` | **40** | serial audio data |
| `GAIN` | leave floating | — | 9 dB default, see table below |
| `SD` | usually nothing | — | channel select + shutdown, see below |
| `+` / `−` | speaker + / − | — | **bridge-tied, see warning** |

Only three GPIOs, all already reserved for I2S in §3. GPIO20 (`PCM_DIN`,
header 38) is the input side and stays unused for playback.

⚠ **The speaker output is bridge-tied (BTL) — neither terminal is ground.**
Both pins swing. Connecting either speaker wire to ground shorts half the
output bridge and destroys the amp. Same rule as the TB6612 motor outputs:
the two wires go to the speaker and to nothing else.

### Vin comes off the buck, not off the Pi

Peak current into 4 Ω at full output is about **1.3 A**. A Pi header pin
should never be asked for that, and pulling it through the Pi causes
brownouts and SD-card corruption. Run the amp's own pair of wires back to the
5 V buck.

This resolves the ⚠ that used to sit here — **the amp is a 5 V part on the
buck rail** — and it is the item that settles the buck sizing:

| Load | Draw at 5 V |
|---|---|
| Pi 4 | 3 A |
| LiDAR | 0.3–0.6 A |
| **Amp** | **~0.2 A typical, 1.3 A peak** |
| Servo (when added) | 0.5–2 A |

A **≥ 5 A buck is confirmed necessary**, not just prudent.

Fit a **220–470 µF electrolytic across Vin/GND right at the amp**, plus a
0.1 µF ceramic. A class-D amp draws current in bursts at the audio waveform's
peaks; without local bulk the rail sags on bass notes, which sounds like
distortion and can reset the Pi.

### GAIN pin

| GAIN pin | Gain |
|---|---|
| **floating** | **9 dB — default, start here** |
| tied to GND | 12 dB |
| 100 kΩ to GND | 15 dB |
| tied to Vin | 6 dB |
| 100 kΩ to Vin | 3 dB |

Leave it unconnected. 15 dB into a 4 Ω load clips early — it sounds worse,
not louder, because the amp runs out of supply before the speaker runs out of
excursion.

### SD pin — channel select, and it is not just an on/off

`SD` is a multi-level analog input, read against Vin through a divider. It
sets both shutdown and which channel gets played:

| SD voltage | Behaviour |
|---|---|
| < 0.16 V | shutdown, amp off |
| 0.16 – 0.77 V | right channel only |
| 0.77 – 1.4 V | left channel only |
| **> 1.4 V** | **(left + right) / 2 — mono, what you want** |

⚠ **Board variants differ, so check yours.** Most breakouts (Adafruit and the
common clones) fit a pull-up that parks SD above 1.4 V, giving mono-average
with nothing to wire. Some clones fit a pull-down instead, leaving the amp
shut down until you drive SD high — and it looks identical to a dead amp.

With Vin applied and no audio playing, meter `SD` against ground:
**above 1.4 V = ready. Near 0 V = jumper SD to Vin.**

For software mute, drive SD from a spare GPIO — **GPIO4 (header 7)** is free
and outside every reserved block in §3. Add it to `pins.py` if you do.

### Enabling I2S on the Pi

In `/boot/firmware/config.txt`:

```
# I2S amplifier — MAX98357A
dtparam=audio=off
dtoverlay=max98357a
```

`dtparam=audio=off` disables the onboard analog card. Leaving it on creates a
second playback device and ALSA picks the wrong one about half the time — the
single most common reason an I2S amp is silent while everything looks correct.

Check which overlay your kernel actually ships before rebooting:

```bash
ls /boot/firmware/overlays/ | grep -iE 'max98357|hifiberry-dac'
```

If `max98357a.dtbo` is absent, use `dtoverlay=hifiberry-dac` instead — it
drives the same three I2S pins and works with this amp. Reboot, then:

```bash
aplay -l
```

You should see one card, named something like `MAX98357A` or
`snd_rpi_hifiberry_dac`. No card = the overlay did not load.

### There is no hardware volume control

The MAX98357A has no volume register — it plays whatever samples reach it, so
`amixer scontrols` comes back empty. **Every level control has to be in
software.** `test/web_dashboard.py` does this in the player (ffmpeg's `volume`,
`bass` and `treble` filters), so it needs nothing extra.

If you want `amixer` to work anyway, add a softvol plugin in `~/.asoundrc`:

```
pcm.!default { type plug; slave.pcm "softvol" }
pcm.softvol {
  type softvol
  slave.pcm "hw:0,0"
  control { name "Master"; card 0 }
}
```

### Testing it

```bash
sudo apt install -y alsa-utils ffmpeg     # ffmpeg only needed for MP3
python test/speaker_test.py               # tone + sweep from the terminal
python test/web_dashboard.py              # tone, sweep, MP3 upload in the UI
```

Silent? Work down this list in order — it is roughly most to least likely:

1. `aplay -l` shows no card → overlay not loaded, `config.txt`
2. `SD` measures near 0 V → amp shut down, jumper it to Vin
3. Vin not at 5 V, or measured at the Pi instead of at the amp
4. Speaker wire grounded → bridge shorted, amp likely dead
5. `dtparam=audio=off` missing → ALSA is playing to the analog card

### Motor whine

Digital I2S means motor noise cannot enter through the signal path. It can
still enter through the **shared 5 V rail**. If you hear whine that rises and
falls with motor duty, the fix is the amp's bulk capacitor and a dedicated
pair of wires from the buck to the amp — never daisy-chained through the Pi
or through a motor driver's ground.

---

## 8. Verify before power-on

These are assumptions, not facts about your build. Check each one.

| # | Assumption | Why it matters | How to check |
|---|---|---|---|
| 1 | ❌ **RESOLVED — TB6612 is undersized.** Motor confirmed as JGB37-520 12 V 330 RPM, stall ≈ **4–5 A** vs the TB6612's 1.2 A continuous / 3.2 A peak. | Stall exceeds even the driver's *peak* rating. | See "Driver replacement" below. Do not run these motors under load on a TB6612. |
| 1b | ⚠ Encoder wire colours — standard code assumed (§4): red M+, white M−, blue VCC, black GND, yellow A, green B | Batches vary. Black is hall GND here, not motor − — 12 V into a hall wire destroys the encoder. | Motor pair (red/white) reads 1–10 Ω; every thin wire reads open against them. §4 has the full procedure. |
| 2 | ⚠ Buck rated ≥ 5 A | Pi 4 alone wants 3 A; LiDAR adds ~0.5 A. A 2 A buck browns the Pi out. | Read the buck's label; measure 5 V under load. |
| 3 | ⚠ Motor no-load current | Sets normal running draw and battery life. | Ammeter, motor free-spinning at 11.1 V. |
| 4 | ✅ **RESOLVED — amp is MAX98357A, 5 V, on the buck rail.** ~0.2 A typical, **1.3 A peak** into 4 Ω. | Settles the buck budget; confirms ≥ 5 A is required, not optional. | §7. Remaining check: meter `SD` against ground with Vin applied — **> 1.4 V** or the amp stays shut down. |
| 5 | ⚠ LiDAR 5 V draw | May exceed what the Pi's USB can give. | Datasheet or inline ammeter. |
| 6 | Battery C-rating vs 4× stall | 4 motors stalling together can exceed the pack's safe discharge. | Pack label: capacity × C-rating ≥ peak draw. |
| 7 | Encoders present? | JGB37 often ships with hall encoders — changes pin budget. | Count the motor wires: 2 = no encoder, 6 = encoder. |

### Driver replacement — the TB6612s cannot drive these motors

JGB37-520 12 V 330 RPM: rated current ≈ 0.5–1.0 A, **stall ≈ 4–5 A**. ⚠ These
are typical catalogue figures for this model, not measured from your units —
confirm with an ammeter, but the margin is not close enough to argue with.

The TB6612FNG is 1.2 A continuous, 3.2 A peak *per channel*. A stalled
JGB37-520 exceeds even the peak rating. Stall is not a rare event — it happens
every time a wheel hits a table leg, climbs a threshold, or the robot pushes
against a wall. Four motors stalling together is ~18 A.

**Options, best first:**

| Driver | Rating | Wiring | Pins per side |
|---|---|---|---|
| **Cytron MDD10A** | 10 A cont. / 30 A peak, 2 ch | 1 channel per side, both motors of that side in parallel | `PWM` + `DIR` = 2 |
| **BTS7960 / IBT-2** ×2 | 43 A, 1 full bridge each | 1 module per side, both motors paralleled | `RPWM` + `LPWM` + `EN` = 3 |
| **VNH2SP30** ×2 | 30 A | 1 per side | 3 |

The **MDD10A is the clean fit**: one board replaces both TB6612s, keeps the
`PWM` + direction pin pattern the code already uses, and 2 motors per channel
at ~9 A stall sits inside its 10 A continuous rating. Wiring both motors of a
side in parallel to one channel is correct here precisely because differential
drive already forces them to the same speed and direction.

Do **not** parallel a TB6612's A and B channels to get 2.4 A — still under the
~4.5 A stall, and it costs you a whole driver per motor.

### What you can still do with the TB6612s today

Free-spinning, off the ground, a JGB37-520 draws only its no-load current
(≈0.2–0.4 A). That is comfortably inside the TB6612's range, so you can:

- run `test/motor_test.py --logic` (no VM at all), and
- bench-test the drive code with **wheels off the ground at low duty**

to validate pin assignments, direction (see the mirror-image note in §4),
and the software — while the real drivers are on order. `MAX_DUTY` in `test/pins.py` is capped at 0.40 for this.

**Do not put the robot on the floor on TB6612s.** Floor contact means load,
load means current, and the first stall takes the driver with it.

---

## 9. Bring-up order

Do not skip steps. Each one isolates a different failure.

1. **Wheels off the ground.** Robot on a box, wheels free. Non-negotiable.
2. Pi on buck only, no motor power. Confirm it boots and holds 5 V.
3. LiDAR plugged in, confirm `/dev/ttyUSB0` appears and Pi still stable.
4. Motor logic only — `VM disconnected`, run `test/motor_test.py`, scope or
   meter the IN1/IN2/PWM pins. Verifies GPIO before any current flows.
5. **One** motor on VM, low duty. Confirm direction and that it stops.
6. All four, low duty, still off the ground.
7. Audio test.
8. Only then, on the floor.

---

## 10. ESP32 low-level controller (planned)

> **Status: not wired, not built.** Finish §9 steps 1–6 on the Pi alone
> before starting this. Adding a second controller to an unproven drive train
> means a dead motor could be wiring, driver, Pi code, firmware, or the link
> between them — five unknowns instead of three.

### Why a second controller at all

Not for PWM. GPIO12/13 are real hardware PWM and two channels is exactly what
differential drive needs — a whole microcontroller to generate two square
waves buys nothing.

The reason is **encoder counting**. At `COUNTS_PER_REV = 1320` and 330 RPM
each wheel produces ~7,260 edges/second, so both together are **~14,500
interrupts/second** landing in Python callbacks on a non-real-time kernel.
That is a third to a half of a core doing nothing but counting, competing with
LiDAR processing for the same CPU. It does not fail loudly — it silently drops
edges, and dropped edges are odometry drift that compounds and is miserable to
trace.

The ESP32's **PCNT** peripheral counts quadrature in hardware at 0 % CPU. The
Pi then reads an accumulated count over serial at 20 Hz instead of servicing
14,500 interrupts.

Two things come along for free once the split exists:

- **A watchdog failsafe.** Today, if the Python process dies mid-drive the
  GPIO pins hold their last state and the motors keep running. An ESP32 that
  drops `STBY` after 200 ms of silence is a real dead-man switch.
- **Stable loop timing.** A fixed 200 Hz PID loop on bare metal does not
  jitter the way one scheduled by Linux does.

All three reasons are about *closed-loop* control. If this stays an RC truck
you drive by hand, open-loop PWM needs no encoders and none of this — skip
the section.

### Division of labour

```
   LiDAR ─USB─┐
              │        UART 115200            ┌── PWM/DIR ── driver ── motors
   speaker ───┤  Pi 4  ═══════════════▶ ESP32 ┤
              │        ◀═══════════════       └── PCNT ◀─ encoders
   WiFi/UI ───┘        odometry + status
```

| | Raspberry Pi 4 | ESP32 |
|---|---|---|
| Owns | LiDAR, SLAM, navigation, audio, WiFi/UI, logging | PWM generation, direction, `STBY` |
| | high-level velocity commands | quadrature counting (PCNT) |
| | | PID velocity loop |
| | | watchdog / failsafe |
| Timing | best-effort, Linux-scheduled | fixed 200 Hz, bare metal |
| Language | Python | Arduino C++ |

The Pi never touches a motor pin again. It sends *"drive left at X, right at
Y"* and reads back counts.

### ESP32 pin map

Assumes a classic **ESP32-WROOM-32** DevKit (30- or 38-pin). Motor pins are
grouped on output-safe GPIOs; encoders go on the input-only block, which is
what those pins are good for.

| Signal | ESP32 GPIO | Goes to |
|---|---|---|
| `LEFT_PWM` | **25** | driver — left `PWM` |
| `LEFT_IN1` | **26** | driver — left `IN1` / `DIR` |
| `LEFT_IN2` | **27** | driver — left `IN2` |
| `RIGHT_PWM` | **32** | driver — right `PWM` |
| `RIGHT_IN1` | **33** | driver — right `IN1` / `DIR` |
| `RIGHT_IN2` | **4** | driver — right `IN2` |
| `STBY` | **13** | both drivers' `STBY` |
| `LEFT_ENC_A` | **34** (in only) | front-left **yellow** |
| `LEFT_ENC_B` | **35** (in only) | front-left **green** |
| `RIGHT_ENC_A` | **36** (in only) | front-right **yellow** |
| `RIGHT_ENC_B` | **39** (in only) | front-right **green** |
| `TX2` | **17** | Pi header 10 (GPIO15, RXD) |
| `RX2` | **16** | Pi header 8 (GPIO14, TXD) |

Power the board from the **5 V buck into `VIN`** (it has an onboard 3.3 V
regulator), or let the Pi's USB power it during bring-up. Encoder blue now
goes to the **ESP32's `3V3` pin**, not the Pi's. All grounds still common —
battery, buck, drivers, Pi, ESP32, amp.

#### ESP32 pins you must not use

The ESP32's pin restrictions are much sharper than the Pi's and are the most
common way a first build fails to boot.

| GPIO | Problem |
|---|---|
| **6–11** | wired to the internal SPI flash. Using them bricks boot. |
| **12** | strapping pin — **must be LOW at reset**. Pulled high it sets flash to 1.8 V and the chip will not boot at all. |
| **0, 2, 5, 15** | strapping pins. A driver input pulling one the wrong way at reset blocks boot or forces download mode. |
| **1, 3** | USB console TX/RX. Keep free so you can watch serial output while the Pi link uses UART2. |
| **34–39** | input-only — cannot drive an output, and have **no internal pull-ups**. |

⚠ **Add 10 kΩ pull-ups to 3V3 on all four encoder lines.** GPIO34–39 have
none internally. The Pi's GPIOs do, which is why the direct-to-Pi wiring in
§4 works without them — this is a new requirement, not a carry-over. PCNT
itself is happy on input-only pins.

⚠ **`STBY` needs a 10 kΩ pull-down to GND.** During reset and while flashing,
every ESP32 pin floats. A floating `STBY` can read high, enabling the driver
while the inputs are garbage — the same failure mode as the relay clicking on
at boot that `throttle_relay.ino` guards against. The resistor holds the
drivers disabled through reset; firmware then drives it low as the first
statement in `setup()`, before any `pinMode` on the motor pins.

### The Pi ↔ ESP32 link

**During bring-up, use USB.** One cable powers the board, flashes it, and
carries the protocol as `/dev/ttyUSB1`. Nothing to wire, nothing to
misconfigure.

**For the finished truck, move to UART.** It frees a USB port and removes the
enumeration race where the LiDAR and the ESP32 fight over `ttyUSB0` after a
reboot. Both sides are 3.3 V logic, so **no level shifter is needed** — this
is one of the few places an ESP32 is easier to hook up than a 5 V Arduino.

| Pi | | ESP32 |
|---|---|---|
| header 8 — GPIO14 `TXD` | → | GPIO16 `RX2` |
| header 10 — GPIO15 `RXD` | ← | GPIO17 `TX2` |
| header 6/9/14/… `GND` | — | `GND` |

Cross TX to RX. §3 already reserves GPIO14/15 for exactly this, so no pin
budget changes.

Two config steps on the Pi, both easy to forget:

```bash
sudo raspi-config     # Interface > Serial > login shell NO, hardware serial YES
```

```ini
# /boot/firmware/config.txt
enable_uart=1
dtoverlay=disable-bt
```

`disable-bt` matters. Without it `/dev/serial0` is the **mini UART**, whose
baud rate is tied to the core clock and drifts when the CPU changes frequency
— which is precisely when LiDAR processing kicks in. The overlay hands the
proper PL011 (`/dev/ttyAMA0`) to GPIO14/15 instead.

If you stay on USB, pin the device name by serial number so it cannot swap
with the LiDAR:

```
# /etc/udev/rules.d/99-esp32.rules
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{serial}=="<yours>", SYMLINK+="esp32"
```

### Protocol

Line-based ASCII at **115200 baud**. Binary would be smaller, but you author
this blind on Windows and debug it over a serial monitor — being able to read
the traffic, and to type a command by hand to test the ESP32 with the Pi
unplugged, is worth far more than the bytes.

**Pi → ESP32**, at 20–50 Hz:

| Message | Meaning |
|---|---|
| `V <left> <right>` | target velocity, mm/s, signed. The normal command. |
| `S` | stop now — ramp to zero and drop `STBY` |
| `E` | enable (clears a watchdog trip) |
| `P <kp> <ki> <kd>` | live PID tuning, so you are not reflashing to try a gain |

**ESP32 → Pi**, at 50 Hz:

| Message | Meaning |
|---|---|
| `O <left_counts> <right_counts> <millis>` | odometry — accumulated counts + timestamp |
| `! <text>` | fault: watchdog trip, stall detected, bad command |

Send **accumulated counts, not deltas**. If a line is dropped or garbled the
Pi resynchronises on the next message instead of permanently losing distance.
The timestamp lets the Pi compute velocity correctly even when a message is
late.

⚠ **PCNT is a 16-bit signed counter** on the classic ESP32 — it wraps at
±32,767, which at 7,260 counts/second is **under five seconds**. Set the
unit's high/low limit events and accumulate into an `int64_t` in the ISR.
Reading the raw PCNT register and hoping is the single most common way this
firmware silently breaks.

Use **one PCNT unit per wheel**, both of its channels configured against A and
B, for full 4× decoding — which is what `COUNTS_PER_REV = 1320` assumes.

### Watchdog

The reason this is worth wiring at all:

1. Every valid `V` resets a timer.
2. No `V` for **200 ms** → ramp both sides to zero, drop `STBY` low, emit
   `! watchdog`.
3. Stay disabled until an explicit `E`. Do **not** auto-resume — a truck that
   restarts on its own after a comms glitch is worse than one that stops.

Test it deliberately: with wheels off the ground and motors turning, kill the
Pi-side process. The motors must stop within 200 ms. If they do not, the
failsafe is decorative.

### PWM frequency — a free win

`PWM_HZ = 1000` on the Pi sits right in the middle of hearing, and this is a
truck whose entire purpose is playing audio. The ESP32's LEDC peripheral does
**20 kHz at 10-bit resolution** (0–1023) comfortably, which is above most
people's hearing.

Check the driver's ceiling: the TB6612 tolerates up to 100 kHz, but the
**MDD10A is specified to 20 kHz** — so 18–20 kHz is the target, not higher.

### Firmware location and flashing

```
Speaker_truck/
├── firmware/
│   └── esp32_motor/
│       ├── esp32_motor.ino
│       └── pins.h          # mirrors test/pins.py — keep them in step
└── test/
    └── pins.py             # unchanged; still the Pi-side map
```

Authored on Windows like everything else, but **flashed from Windows over
USB** — it does not go through the Pi sync at all. `test/pins.py` keeps its
current motor pins: they stay valid for direct-drive bench testing with the
ESP32 out of the loop, which is a useful fallback when you are isolating a
fault.

Since the truck is mobile, either keep the ESP32's USB port reachable, or add
`ArduinoOTA` and reflash over WiFi. Flashing from the Pi with `esptool` or
`arduino-cli` also works if the boards stay cabled together.

### Migration order

Same principle as §9 — one new variable at a time.

1. §9 steps 1–6 pass on the Pi alone. **Do not start before this.**
2. ESP32 on USB only, nothing else connected. Confirm it boots, and that PWM
   appears on the right pins with an LED or a scope.
3. Encoders to the ESP32, still no driver. Turn each wheel one full
   revolution by hand and read the count — this is also how you finally
   settle `COUNTS_PER_REV` (§4).
4. Move the driver's control wires from the Pi to the ESP32, **VM still
   disconnected**. Meter the IN/PWM pins as in §9 step 4.
5. VM back on, one motor, low duty. Confirm direction, confirm `S` stops it.
6. Kill the Pi process and time the watchdog.
7. Only then close the PID loop.

Steps 1–3 are all doable while the MDD10A is still on order.

### What does not move to the ESP32

LiDAR, audio, and navigation stay on the Pi. The ESP32 has neither the RAM for
scan matching nor any business decoding audio, and putting a real-time
controller behind a task that can block is how you lose the timing guarantees
that justified adding it.

---

## 11. IMU — 9-axis (accel + gyro + magnetometer)

Added after §10 rather than inserted mid-document **so no existing section
number moves** — `CLAUDE.md`, `pins.py` and several scripts cite "§4", "§7",
"§8" by number, and renumbering would silently break every one of them.

**Nothing in §3–§5 changes.** The IMU is an I2C device: it consumes no new
GPIO, takes no pin away from the motors, and does not touch the drive path.
GPIO2/GPIO3 were already reserved for I2C in §3.

### Which chip

The driver in `test/imu.py` identifies the part at runtime, so you do not
have to decide in advance:

| Part | Address | ID register | Notes |
|---|---|---|---|
| **BNO055** | `0x28` / `0x29` | `0x00` → `0xA0` | **Best choice.** Fuses on-chip, hands you heading/roll/pitch directly, and reports its own calibration quality. |
| **ICM-20948** | `0x68` / `0x69` | `0x00` → `0xEA` | Accel + gyro + AK09916 mag. Fusion done on the Pi. |
| **MPU-9250** | `0x68` / `0x69` | `0x75` → `0x71` | Accel + gyro + AK8963 mag. Discontinued but everywhere. |
| MPU-6500 / 6050 | `0x68` / `0x69` | `0x75` → `0x70` / `0x68` | **6-axis only** — no magnetometer, so heading drifts. |

A 6-axis part still gives usable roll and pitch. It cannot give a stable
heading, because there is nothing to correct gyro drift against. The UI
flags this rather than quietly showing a wrong number.

### Connections

| IMU pin | Connect to | Pi header | BCM |
|---|---|---|---|
| `VCC` / `VIN` | **3.3 V** | **1** | — |
| `GND` | common ground | **9** | — |
| `SDA` | I2C data | **3** | GPIO2 |
| `SCL` | I2C clock | **5** | GPIO3 |
| `INT` | not needed | — | — |
| `ADDR` / `AD0` | leave open, or GND | — | selects the alternate address |

⚠ **3.3 V, not 5 V — this one bites.** GPIO2 and GPIO3 carry **fixed 1.8 kΩ
pull-ups to 3.3 V** on the Pi board; they cannot be turned off. A breakout
powered from 5 V has its own pull-ups to 5 V, which then push current back
into those pins. Many IMU breakouts have an onboard regulator and *look* like
they accept 5 V — that regulator feeds the chip, not the bus, so the pull-ups
still sit at 5 V. Power it from 3.3 V and the problem does not exist.

### Pull-ups

The Pi's fixed 1.8 kΩ pull-ups are usually enough on their own. Most
breakouts add another 4.7 kΩ–10 kΩ, which in parallel is still fine. Only a
bare chip on a long cable needs anything added.

If several I2C devices ever get chained here, the parallel pull-ups can get
too strong and the bus edges go soft. Symptom: intermittent read errors that
worsen as you add devices. Fix by removing the pull-up resistors from all but
one breakout.

### Mounting

Two things matter far more than they sound:

**Keep it away from the motors and the speaker.** The magnetometer measures
the earth's field at roughly 50 µT. A speaker magnet or a motor at 30 mm
swamps that completely, and no amount of calibration recovers it — the field
changes with motor current, so it is not a fixed offset. **Mount it at least
100 mm from any motor and from the speaker**, ideally on a short standoff
above the chassis. This is the single biggest thing that decides whether
heading is usable.

**Orient X toward the front.** Then `IMU_YAW_OFFSET` in `pins.py` stays 0.
If you cannot, measure the mounting angle and put it there — same idea as the
LiDAR zero offset.

Mount it rigidly. A board on foam reads chassis wobble as real rotation.

### Enabling I2C

```bash
sudo apt install -y i2c-tools python3-smbus2
sudo raspi-config          # Interface Options -> I2C -> enable
sudo reboot
i2cdetect -y 1
```

You should see your chip's address in the grid. Nothing at all means wiring
or a disabled bus, not a dead chip — check before suspecting the part.

### Calibration — the part that decides if heading works

```bash
python test/imu_test.py --gyro-bias    # 5 s, robot still
python test/imu_test.py --calibrate    # 30 s, rotate through all orientations
```

**Gyro bias** takes five seconds and is worth redoing each session: a gyro
reads a small non-zero rate when perfectly still, and integrating that is
exactly what makes heading crawl.

**Magnetometer** is the one that matters. The truck carries four motors and a
speaker magnet, and that steady local field (hard-iron) shifts the sensor's
centre off zero. Uncorrected, heading is wrong by a different amount
depending which way you face — which is worse than being uniformly wrong,
because it looks plausible.

Calibrate with the robot **fully assembled**. Calibrating a bare board and
then bolting it into the truck measures the wrong magnetic environment.

Results land in `test/imu_calib.json`. The BNO055 ignores this and calibrates
itself; the tool shows you its progress instead.

### Verify

| Check | Expected | If wrong |
|---|---|---|
| `i2cdetect -y 1` | chip at `0x28/0x29` or `0x68/0x69` | I2C off, or SDA/SCL swapped |
| IMU `VCC` | **3.3 V** | 5 V will damage GPIO2/3 |
| Board flat, still | roll ≈ 0, pitch ≈ 0, az ≈ 1 g | axes not as assumed — check the silkscreen |
| Rotate 90° by hand | yaw changes ≈ 90° | needs `--calibrate`, or it is too near a motor |
| Motors running, robot still | heading should barely move | too close to the motors — move it further away |

That last row is the real acceptance test, and the one people skip. Spin the
motors with the wheels off the ground and watch the heading. If it swings,
the magnetometer is inside the motors' field and no software fixes it.

---

## 12. CSI camera

The one peripheral on this robot that costs **no GPIO at all**. It plugs into
the Pi 4's dedicated CSI ribbon connector — a separate bus with its own
connector, its own kernel driver and its own bandwidth. Nothing in §3 changes,
and the camera cannot conflict with the motors, the encoders, the I2S amp or
the I2C IMU no matter how the rest of the build grows.

### The connector

On a Pi 4 the CSI socket is the narrow white 15-pin one between the HDMI
ports and the audio jack, labelled **CAMERA**. The Ethernet-side socket
marked **DISPLAY** is DSI and looks nearly identical — a ribbon in the wrong
one enumerates nothing and looks exactly like a dead camera.

Seating it:

1. **Power the Pi down.** The CSI connector is not hot-pluggable.
2. Lift the plastic retaining collar straight up, gently. It travels ~1 mm.
3. Slide the ribbon in with the **silver contacts facing the HDMI ports**
   (blue backing stiffener toward the Ethernet jack).
4. Press the collar back down evenly, both ends together.
5. At the camera end, contacts face **away** from the lens.

A ribbon in the right way but only half seated is the failure worth knowing:
it often enumerates — `--list` finds the sensor — and then delivers no
frames. `camera_test.py` calls that out by name rather than reporting a
generic error, because the two look nothing alike from software and identical
from the outside.

### Enabling it

Raspberry Pi OS Bookworm and Trixie auto-detect the official modules through
`camera_auto_detect=1`, already in `/boot/firmware/config.txt`. No
`raspi-config` step and no overlay line, unlike the I2C the IMU needs.

Third-party modules (Arducam and friends) do need their own overlay:

```
# /boot/firmware/config.txt
camera_auto_detect=0
dtoverlay=arducam-64mp        # whichever the vendor specifies
```

Software comes from apt, never pip — same rule as `gpiozero`, `lgpio` and
`pyserial`:

```bash
sudo apt install -y python3-picamera2
rpicam-hello --list-cameras
```

### Power

The camera draws roughly **200–260 mA** off the Pi's own 3.3 V rail, through
the ribbon. It needs no wire to the buck converter and no connection to the
12 V battery.

It does, however, land on the same budget as everything else the Pi feeds.
The buck must be **≥ 5 A** (§2) and that number does not change — but a
marginal supply that was merely twitchy before will now brown out under
motors-plus-camera, and a brown-out reads as a camera that "randomly stops
working". Check `vcgencmd get_throttled` before blaming the ribbon: anything
other than `throttled=0x0` means the supply, not the camera.

### Mounting

| Setting | Where | Note |
|---|---|---|
| `CAM_HFLIP` / `CAM_VFLIP` | `pins.py` | both `True` = upside-down mount |
| `CAM_OFFSET_X` / `_Y` | `pins.py` | body frame, +x forward, +y **left** |
| `CAM_YAW_OFFSET` | `pins.py` | 0 = straight ahead, + = left |
| `CAM_HFOV` | `pins.py` | **must match the module** — see below |

`CAM_HFOV` is the number that ties the camera to the rest of the robot. The
nav page draws it as a cyan wedge on the LiDAR plot, which is what tells you
whether an obstacle the scanner found is one the camera can actually show
you. Set it wrong and the wedge claims ground the lens never sees.

| Module | Sensor | Horizontal FOV |
|---|---|---|
| Camera Module 1 | ov5647 | 53.5° |
| Camera Module 2 / NoIR v2 | imx219 | 62.2° |
| Camera Module 3 | imx708 | 66° |
| Camera Module 3 Wide | imx708 | 102° |
| HQ Camera | imx477 | whatever lens is fitted |

Keep the lens clear of the LiDAR's plane. The scanner sweeps a full 360° and
anything in that plane — a camera bracket included — reads as a permanent
obstacle at a fixed bearing, which SLAM will happily map as a wall that
follows the robot around the house.

Unlike the IMU, the camera has no magnetic-field problem and can sit right
next to the motors or the speaker.

### Verify

| Check | Expected | If wrong |
|---|---|---|
| `rpicam-hello --list-cameras` | the sensor is listed | ribbon in DISPLAY not CAMERA, or in backwards |
| `python test/camera_test.py --list` | same, with a friendly name | picamera2 not installed, or venv lacks `--system-site-packages` |
| `python test/camera_test.py` | ~15 fps, worst gap near the average | half-seated ribbon, or something else eating the CPU |
| encoder line says **hardware** | `MJPEG (hardware)` | software fallback — works, but costs a chunk of a core |
| `python test/camera_test.py --stream` | picture the right way up | set `CAM_HFLIP` / `CAM_VFLIP` |
| Motors running, wheels up | picture does not stall | supply sag — check `vcgencmd get_throttled` |

---

## 13. Vision — what the camera adds to navigation

Three jobs, in the order they matter. All of them run off the same lores
stream the ISP produces alongside the video, so none of them decodes a JPEG
or costs a second capture.

### 13.1 Cliff and low-obstacle detection

**The safety one.** The LiDAR sweeps a single horizontal plane. A stair edge
returns nothing to a horizontal beam, so the collision guard reads the top
step as clear floor and drives off it at full confidence. Same for a shoe, a
cable, a threshold, or a tabletop whose legs are all the scanner can see.

`cliff.py` samples a grid of colour and brightness over the lower half of the
frame and compares each cell to the floor the robot is standing on — which is
floor by definition, because the wheels are on it. Anything that does not
match is not floor. Much darker means a hole rather than a thing.

It vetoes **forward motion only**, and never allows the creep escape. Creeping
out of furniture is the right move; creeping forward over a drop is the one
move it must never make. Reverse and rotation stay available, which is always
enough to leave.

**Mounting is the whole game here:**

| | |
|---|---|
| `CAM_HEIGHT_MM` | lens height above the floor — measure it |
| `CAM_PITCH_DEG` | downward tilt — **must not be 0** |

At zero tilt everything above the frame centre is at or above the horizon and
never meets the floor at all, so five of the detector's six rows have nothing
to check. The one row that does starts around 350 mm out, leaving the near
field — the part that decides whether it can stop — unwatched.

**Tilt the camera down 15–25°** until the bottom edge of the frame shows floor
at about the front bumper. At 15° with the lens 120 mm up, the six rows land
at roughly 180, 210, 240, 290, 360 and 450 mm, which brackets the bumper
properly. Check the framing with `camera_test.py --stream`.

Expect false positives on patterned rugs, grout lines and hard sunlight
edges. That is the deliberate trade: a false positive stops the robot, a
false negative puts it at the bottom of the stairs. Press **Relearn floor**
on open floor after moving to a different surface — it takes what it can see
as the definition of floor, so do not relearn facing a wall.

### 13.2 ArUco markers — the absolute position fix

Scan matching corrects the pose against the map the robot built itself. That
is a closed loop: when the map slowly bends, the pose bends with it and
nothing inside the system can tell. Every odometry-plus-LiDAR stack drifts
this way, and the drift is invisible from inside.

A printed tag at a known place is the only input on this robot whose
correctness does not depend on the robot's own history.

**Printing.** `marker_test.py --sheet` writes an SVG — vector, so it lands on
paper at exactly `MARKER_SIZE_MM`. Print at **100%**, no "fit to page". Then
measure a printed tag with a ruler, black square only, and set
`MARKER_SIZE_MM` to what you measured. Every distance scales linearly with
that number, so a tag printed at 92 mm and declared as 100 puts every reading
9% out — consistently, which is exactly the error nothing else will catch.

**Placing.** Doorframes and wall corners, at roughly camera height. A tag near
the ceiling is decorative — it has to be visible from where the robot drives.
One per room plus one per doorway is plenty; fixes come from whichever tag is
in view and they never need to be seen together.

**Learning.** Tags are not surveyed by hand. Run with `--learn-markers`, drive
the house once, and each new tag is recorded at wherever the pose says it is.
Turn learning off and they hold the map straight from then on.

**Position only.** A fix moves x and y and leaves heading alone, even though
solvePnP returns a full 6-DOF pose. Marker orientation is the famously
unreliable half — a square seen near head-on has two nearly equally good
solutions and flips between them frame to frame. Heading is meanwhile the one
thing this robot already measures well, with a gyro that does not care about
wheel slip. Each sensor supplies what the other cannot.

Range is set by how many pixels a tag covers, so it scales with `CAM_SIZE`.
At 640×480 with 100 mm tags, expect reliable detection to about 2 m.

### 13.3 Photo trail

Every still is tagged with the SLAM pose it was taken from and pinned to the
map. An occupancy grid is only ever grey, white and black; the photos are what
make it readable by a person, and they answer "what was it looking at here"
long after the LiDAR points have scrolled past.

Auto-capture is distance-gated, not time-gated — a parked robot should not
fill the card with the same picture. Stills live in `test/captures/` on the
Pi and are never synced back.

### Software

```bash
sudo apt install -y python3-picamera2 python3-opencv
```

apt, not pip, for both. `pip install opencv-python` gets you an OpenCV with
**no aruco module** — it lives in opencv_contrib upstream, and Debian builds
contrib into `python3-opencv`.

### Verify

| Check | Expected | If wrong |
|---|---|---|
| `python -c "import cv2; print(cv2.__version__, hasattr(cv2,'aruco'))"` | a version and `True` | wrong OpenCV — use apt's |
| `marker_test.py --sheet` then print | tag measures `MARKER_SIZE_MM` | "fit to page" was on |
| `marker_test.py`, tag at 1 m | reads within a few cm | check `MARKER_SIZE_MM`, then `CAM_HFOV` |
| Tag to the robot's LEFT | bearing is **positive** | camera-to-body sign, `markers.py` |
| Floor panel, open floor | all cells clear | press Relearn, on floor not wall |
| Floor panel, shoe ahead | amber cells, forward vetoed | camera not tilted down enough |
| Drive at a step down | red cells, forward vetoed | **test this on a kerb before stairs** |
