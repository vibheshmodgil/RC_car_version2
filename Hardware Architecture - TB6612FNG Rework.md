# Hardware Architecture - TB6612FNG Motor Driver Rework

Status: firmware reference for the current TB6612FNG drive system.

The original BTS7960/IBT-2 boards were replaced with two TB6612FNG dual
H-bridge breakouts. The firmware controls all four wheels independently.

## Drive Topology

- TB6612 #1 LEFT board
  - Channel A: Left-Front
  - Channel B: Left-Rear
- TB6612 #2 RIGHT board
  - Channel A: Right-Front
  - Channel B: Right-Rear

Index order everywhere is `LF, LR, RF, RR`.

## TB6612 Control Mode

This firmware uses standard TB6612FNG wiring with a real PWM pin per channel.
Do not tie PWMA/PWMB to 3.3 V for this sketch.

| IN1 | IN2 | PWM | Result |
|---|---|---|---|
| HIGH | LOW | duty | Forward |
| LOW | HIGH | duty | Reverse |
| HIGH | HIGH | 255 | Short brake, disabled by default in firmware |
| LOW | LOW | 0 | Coast / stop |
| any | any | any | STBY LOW disables the whole board |

Signed firmware PWM remains `-255..+255`.

## Pin Map

Authoritative source: `CarTestBench/config.h`.

### TB6612 #1 - LEFT board

| TB6612 pin | ESP32 GPIO | Function |
|---|---:|---|
| AIN1 | 18 | Left-Front direction 1 |
| AIN2 | 19 | Left-Front direction 2 |
| PWMA | 5 | Left-Front PWM |
| BIN1 | 26 | Left-Rear direction 1 |
| BIN2 | 27 | Left-Rear direction 2 |
| PWMB | 32 | Left-Rear PWM |
| STBY | 12 | Left board standby, 10 kOhm pull-down to GND |

### TB6612 #2 - RIGHT board

| TB6612 pin | ESP32 GPIO | Function |
|---|---:|---|
| AIN1 | 16 | Right-Front direction 1 |
| AIN2 | 17 | Right-Front direction 2 |
| PWMA | 33 | Right-Front PWM |
| BIN1 | 13 | Right-Rear direction 1 |
| BIN2 | 15 | Right-Rear direction 2 |
| PWMB | 0 | Right-Rear PWM |
| STBY | 2 | Right board standby, 10 kOhm pull-down to GND |

### Encoders

| Wheel | A | B |
|---|---:|---:|
| Left-Front | 34 | 35 |
| Left-Rear | 22 | 21 |
| Right-Front | 4 | 23 |
| Right-Rear | 25 | 14 |

GPIO36/GPIO39 are not used because they are not exposed on this 38-pin ESP32
board. GPIO34/35 are input-only and have no internal pull-ups. Add external
pull-ups to 3.3 V on every encoder line. Power encoders from 3.3 V only.

### Shared / Reserved

- Raspberry Pi link: WiFi
- UART0 TX/RX: GPIO1/GPIO3 reserved for flashing and serial monitor

MPU/LiDAR/ToF sensors are now planned on the Raspberry Pi side. With dedicated
PWM, four encoders, and two STBY lines, this ESP32 map has no ordinary spare
GPIO left for a wired Pi UART.

## Safety Notes

The TB6612FNG is much smaller than the old BTS7960 boards. Ramp PWM gently,
avoid sustained stalls, and heatsink or ventilate the driver boards.

Keep a pull-down on each STBY line. STBY is the hardware safety switch: LOW at
boot, LOW when neither channel on a board is armed, and LOW after an e-stop.

Active short-brake is disabled in firmware by default. If a direction line is
open, a short-brake command can accidentally become full-speed drive on that
channel. Re-enable it only after each IN1/IN2/PWM wire is verified.
