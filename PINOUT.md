# Raspberry Pi 4 Model B — 40-pin header

Board orientation: **USB/Ethernet ports facing you, header along the top
edge.** Pin 1 is the top-left, nearest the SD card / corner of the board.
Odd pins are the outer row, even pins the inner row.

```
                                                    ┌── pin 2  (5V)
        ┌── pin 1  (3V3)                            │
        ▼                                           ▼
      ┌───┬───┬───┬───┬───┬───┬───┬───┬───┬───┬───┬───┬───┬───┬───┬───┬───┬───┬───┬───┐
 odd  │ 1 │ 3 │ 5 │ 7 │ 9 │11 │13 │15 │17 │19 │21 │23 │25 │27 │29 │31 │33 │35 │37 │39 │
      ├───┼───┼───┼───┼───┼───┼───┼───┼───┼───┼───┼───┼───┼───┼───┼───┼───┼───┼───┼───┤
 even │ 2 │ 4 │ 6 │ 8 │10 │12 │14 │16 │18 │20 │22 │24 │26 │28 │30 │32 │34 │36 │38 │40 │
      └───┴───┴───┴───┴───┴───┴───┴───┴───┴───┴───┴───┴───┴───┴───┴───┴───┴───┴───┴───┘
        ▲                                                                       ▲
        └── SD card end                                     USB port end ───────┘
```

## Full map

`◄──` marks a pin this project uses.

```
        3V3  power  │  1  ●  ● │  2   5V power     ◄── buck 5V in (or USB-C)
GPIO2   SDA1  I2C   │  3  ◆  ● │  4   5V power
        ◄── IMU SDA │          │
GPIO3   SCL1  I2C   │  5  ◆  ● │  6   GND          ◄── common ground
        ◄── IMU SCL │          │
GPIO4   GPCLK0      │  7  ●  ● │  8   GPIO14  TXD
        GND         │  9  ●  ● │ 10   GPIO15  RXD
GPIO17              │ 11  ●  ◆ │ 12   GPIO18  PCM_CLK   ◄── MAX98357A BCLK
GPIO27              │ 13  ◆  ● │ 14   GND
   ◄── RIGHT_IN1    │        ◆ │ 15   GPIO22            ◄── RIGHT_IN2
GPIO22              │ 15  ◆  ◆ │ 16   GPIO23            ◄── LEFT_IN1
        3V3  power  │ 17  ◆  ◆ │ 18   GPIO24            ◄── LEFT_IN2
   ◄── TB6612 VCC   │          │
   ◄── LCD VCC/RES/BLK         │
GPIO10  MOSI  SPI   │ 19  ◆  ● │ 20   GND
   ◄── LCD SDA      │          │
GPIO9   MISO  SPI   │ 21  ●  ◆ │ 22   GPIO25            ◄── STBY
GPIO11  SCLK  SPI   │ 23  ◆  ◆ │ 24   GPIO8   CE0       ◄── LCD CS
   ◄── LCD SCL      │          │
        GND         │ 25  ◆  ◆ │ 26   GPIO7   CE1       ◄── LCD DC
   ◄── LCD GND      │          │
GPIO0   ID_SD       │ 27  ●  ● │ 28   GPIO1   ID_SC
GPIO5               │ 29  ●  ● │ 30   GND
GPIO6               │ 31  ●  ◆ │ 32   GPIO12  PWM0      ◄── LEFT_PWM
GPIO13  PWM1        │ 33  ◆  ● │ 34   GND
   ◄── RIGHT_PWM    │          │
GPIO19  PCM_FS      │ 35  ◆  ● │ 36   GPIO16
     ──► MAX98357A LRC│        │
GPIO26              │ 37  ●  ● │ 38   GPIO20  PCM_DIN
        GND         │ 39  ●  ◆ │ 40   GPIO21  PCM_DOUT
                                        ◄── MAX98357A DIN
```

## This project's pins

| Header pin | BCM | Signal | Destination |
|---:|---|---|---|
| 13 | GPIO27 | `RIGHT_IN1` | TB6612 #2 — AIN1 + BIN1 |
| 15 | GPIO22 | `RIGHT_IN2` | TB6612 #2 — AIN2 + BIN2 |
| 16 | GPIO23 | `LEFT_IN1` | TB6612 #1 — AIN1 + BIN1 |
| 18 | GPIO24 | `LEFT_IN2` | TB6612 #1 — AIN2 + BIN2 |
| 22 | GPIO25 | `STBY` | **both** drivers |
| 32 | GPIO12 | `LEFT_PWM` | TB6612 #1 — PWMA + PWMB |
| 33 | GPIO13 | `RIGHT_PWM` | TB6612 #2 — PWMA + PWMB |
| 11 | GPIO17 | `SERVO` | servo signal (power from buck, not the header) |
| 29 | GPIO5 | `LEFT_ENC_A` | front-left encoder, **yellow** |
| 31 | GPIO6 | `LEFT_ENC_B` | front-left encoder, **green** |
| 36 | GPIO16 | `RIGHT_ENC_A` | front-right encoder, **yellow** |
| 37 | GPIO26 | `RIGHT_ENC_B` | front-right encoder, **green** |
| 19 | GPIO10 | `MOSI` | LCD `SDA` |
| 23 | GPIO11 | `SCLK` | LCD `SCL` |
| 24 | GPIO8 | `LCD_SPI_DEV` (CE0) | LCD `CS` |
| 26 | GPIO7 | `LCD_DC` | LCD `DC` — needs `dtoverlay=spi0-1cs` |
| 17 | — | 3V3 | LCD `VCC`, with `RES` and `BLK` jumpered to it |
| 25 | — | GND | LCD `GND` |
| 1 or 17 | — | 3V3 | TB6612 VCC ×2 **and encoder VCC (blue)** |
| 6, 9, 14, 20, 25, 30, 34, 39 | — | GND | common ground, incl. encoder **black** |

Ground pins are all connected internally — use whichever is physically
closest. There are eight; use several rather than daisy-chaining one.

### Motor wire colours

| Colour | Function | Lands on |
|---|---|---|
| red (heavy) | motor + | driver `AO1`/`BO1` — **never** the Pi header |
| white (heavy) | motor − | driver `AO2`/`BO2` — **never** the Pi header |
| blue | hall VCC | header 1 or 17 — **3V3, not 5 V** |
| black | hall GND | any GND pin |
| yellow | hall A | `ENC_A` — header 29 / 36 |
| green | hall B | `ENC_B` — header 31 / 37 |

Black is hall **ground**, not motor −, and white is motor −, not a signal.
Both mistakes are destructive — see `WIRING.md` §4.

## Reserved (leave free)

| Pins | BCM | For |
|---|---|---|
| 12, 35, 40 | 18, 19, 21 | **MAX98357A** — BCLK / LRC / DIN (in use, WIRING.md §7) |
| 38 | 20 | `PCM_DIN` — I2S input side, unused for playback |
| 3, 5 | 2, 3 | **9-axis IMU** — SDA / SCL (in use, WIRING.md §11) |
| 8, 10 | 14, 15 | UART — serial LiDAR |
| 19, 21, 23, 24, 26 | 7–11 | **ST7789 LCD** on SPI0 (in use, WIRING.md §14). 21 / GPIO9 is MISO: unused, but owned by the SPI driver |
| 7 | 4 | MAX98357A `SD` — software mute, and claimed by the `max98357a` overlay unless `no-sdmode` |
| 11 | 17 | `SERVO` (in use above) |
| 27, 28 | 0, 1 | HAT ID EEPROM — never use |

**The header is now full.** Every GPIO has an owner. The next device needs
I2C (shares GPIO2/3 with the IMU) or USB.

## Not on this header: the CSI camera

The camera is worth a line here precisely because it is **absent** from every
table above. It plugs into the Pi 4's own 15-pin CSI ribbon socket — the
narrow white one between the HDMI ports and the audio jack, labelled
**CAMERA** — and consumes **zero header pins**.

So it can never conflict with the motors, the encoders, the I2S amp, the
I2C IMU or the SPI display, and adding it frees you from re-checking anything above. The socket
on the Ethernet side marked **DISPLAY** is DSI, looks almost the same, and a
ribbon in that one enumerates nothing.

Everything configurable about the camera is software, in `test/pins.py`:
`CAM_SIZE`, `CAM_FPS`, `CAM_HFLIP`, `CAM_VFLIP`, `CAM_HFOV`,
`CAM_OFFSET_X/Y`, `CAM_YAW_OFFSET`. Full detail in `WIRING.md` §12.

---

## BCM vs board numbering

Two numbering schemes exist and mixing them is the most common wiring bug:

- **BCM** (`GPIO23`) — the chip's own numbering. **All code here uses BCM.**
  `gpiozero` and `RPi.GPIO` in BCM mode expect these.
- **Board** (`pin 16`) — physical position, 1–40. Use this when counting
  pins on the actual header with your fingers.

`GPIO23` and `pin 23` are **different pins**. When wiring, count physical
positions; when coding, use the BCM number. The table above maps between them.

Check any time with:

```bash
pinout          # ASCII board diagram, from python3-gpiozero
gpio readall    # if wiringpi is installed
```

---

> **On "2018":** the Pi 4 Model B launched in June 2019 (the 8 GB version in
> May 2020), so a 2018 board would be a **3B+**. Your `df`/`free` output
> showed ~8 GB RAM, which only the Pi 4 has — so this is a Pi 4, just newer
> than 2018. Either way **the 40-pin header is identical** across 3B+, 4B and
> 5, so every assignment above holds. Confirm with:
> ```bash
> tr -d '\0' < /proc/device-tree/model
> ```
