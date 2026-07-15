#pragma once
#include <Arduino.h>

// =====================================================================
//  config.h - single source of truth for hardware pins and tunables.
//
//  ESP32 DevKit V1 master MCU.
//  Motor drivers: 2x TB6612FNG dual H-bridges.
//  Drive topology: 4WD skid-steer, one TB6612 channel per wheel.
//    * TB6612 #1 (LEFT board):  ch A = Left-Front,  ch B = Left-Rear
//    * TB6612 #2 (RIGHT board): ch A = Right-Front, ch B = Right-Rear
//
//  Standard TB6612 wiring is used now: IN1/IN2 are direction pins and each
//  channel's PWMA/PWMB pin is driven by ESP32 LEDC PWM.
// =====================================================================

// ---------------------------------------------------------------------
//  Feature flags - toggle subsystems on as the hardware is added.
//  MPU/LiDAR/ToF are now planned on the Raspberry Pi side, not directly on
//  this ESP32 pin map.
// ---------------------------------------------------------------------
#define ENABLE_BNO055   0   // 1 => requires Adafruit_BNO055 + Adafruit_Sensor
#define ENABLE_VL53L0X  0   // 1 => requires Adafruit_VL53L0X (range page, planned)

// ---------------------------------------------------------------------
//  Motor driver pins - 2x TB6612FNG dual H-bridges
//
//  TB6612 truth table used by MotorChannel:
//    IN1=H, IN2=L, PWM=duty  => forward
//    IN1=L, IN2=H, PWM=duty  => reverse
//    IN1=H, IN2=H, PWM=255   => short brake
//    IN1=L, IN2=L, PWM=0     => coast / stop
//
//  Each board has its own STBY line. Add a ~10 kOhm pull-down from each STBY
//  to GND so the motors stay disabled during boot.
// ---------------------------------------------------------------------
constexpr uint8_t NUM_MOTORS = 4;

constexpr uint8_t PIN_MOTOR_STBY_LEFT  = 12;   // TB6612 #1 STBY (10k pull-down)
constexpr uint8_t PIN_MOTOR_STBY_RIGHT = 2;    // TB6612 #2 STBY (10k pull-down)

struct MotorPins {
    const char* label;
    uint8_t in1;    // TB6612 AIN1/BIN1 direction input
    uint8_t in2;    // TB6612 AIN2/BIN2 direction input
    uint8_t pwm;    // TB6612 PWMA/PWMB speed input (LEDC PWM)
    uint8_t stby;   // this channel's board STBY line (shared by the board pair)
};

// Index order matches PINS_ENCODER: 0=LF, 1=LR, 2=RF, 3=RR.
constexpr MotorPins PINS_MOTOR[NUM_MOTORS] = {
    // TB6612 #1 (LEFT board)
    { "Left-Front",  /*AIN1*/ 18, /*AIN2*/ 19, /*PWMA*/ 5,  PIN_MOTOR_STBY_LEFT  },
    { "Left-Rear",   /*BIN1*/ 26, /*BIN2*/ 27, /*PWMB*/ 32, PIN_MOTOR_STBY_LEFT  },
    // TB6612 #2 (RIGHT board)
    { "Right-Front", /*AIN1*/ 16, /*AIN2*/ 17, /*PWMA*/ 33, PIN_MOTOR_STBY_RIGHT },
    { "Right-Rear",  /*BIN1*/ 13, /*BIN2*/ 15, /*PWMB*/ 0,  PIN_MOTOR_STBY_RIGHT },
};

// ---------------------------------------------------------------------
//  Encoder pins - one quadrature (A/B) pair per wheel
//
//  Encoders must be powered from 3.3 V, not 12 V. GPIO34/35 are input-only
//  and have no internal pull-ups, so put external pull-ups to 3.3 V on every
//  encoder A/B line for consistent wiring.
//
//  GPIO36/39 are not exposed on this 38-pin board, so the LR encoder moved
//  to GPIO22/GPIO21. Those were the old I2C pins; ESP32-local I2C sensors are
//  not used in this pin map.
// ---------------------------------------------------------------------
constexpr uint8_t NUM_ENCODERS = 4;

struct EncoderPins {
    const char* label;
    uint8_t a;
    uint8_t b;
};

constexpr EncoderPins PINS_ENCODER[NUM_ENCODERS] = {
    { "Left-Front",  34, 35 },
    { "Left-Rear",   22, 21 },
    { "Right-Front", 4,  23 },
    { "Right-Rear",  25, 14 },
};

// ---------------------------------------------------------------------
//  Raspberry Pi link and reserved/future expansion
// ---------------------------------------------------------------------
constexpr bool PI_LINK_OVER_WIFI = true;        // no spare GPIO for wired UART

// ESP32-local I2C sensors are no longer planned on this pin map. These are
// intentionally invalid unless the pin map is revised later.
constexpr int8_t PIN_I2C_SDA = -1;
constexpr int8_t PIN_I2C_SCL = -1;
constexpr uint8_t BNO055_I2C_ADDR = 0x28;   // ADR/SDO tied to GND

// Dedicated TB6612 PWM leaves almost no ordinary free GPIO on the 38-pin board
// once 4 encoders are kept. Pi sensor data should come over WiFi.
struct ReservedPin {
    const char* label;
    uint8_t pin;
    const char* note;
};

constexpr uint8_t NUM_RESERVED_PINS = 2;
constexpr ReservedPin PINS_RESERVED[NUM_RESERVED_PINS] = {
    { "UART0 TX", 1, "serial monitor / flashing" },
    { "UART0 RX", 3, "serial monitor / flashing" },
};

// ---------------------------------------------------------------------
//  Motor / PWM settings
// ---------------------------------------------------------------------
constexpr uint32_t PWM_FREQ = 20000;  // 20 kHz - above audible range
constexpr uint8_t  PWM_RES  = 8;      // 8-bit (0..255)
constexpr int16_t  PWM_MAX  = 255;
constexpr uint32_t DIR_CHANGE_DEADTIME_MS = 50;   // brake before reversing

// Keep active short-brake disabled while wiring is being shaken down. If one
// direction line is open, IN1=IN2=HIGH can turn into full-speed drive.
constexpr bool MOTOR_ACTIVE_BRAKE_ENABLED = false;

// ---------------------------------------------------------------------
//  Encoder / telemetry settings
// ---------------------------------------------------------------------
// Counts per OUTPUT-shaft revolution (quadrature 4x * gear ratio).
// JGB37-520 hall encoders are ~11 PPR at the motor; the gearbox multiplies
// that. The exact number varies per gear ratio, so calibrate per wheel using
// the wizard on the Encoders page and it is saved to flash.
constexpr float    DEFAULT_CPR = 1320.0f;
constexpr uint32_t RPM_CALC_INTERVAL_MS = 50;     // 20 Hz RPM recompute
constexpr uint32_t WS_BROADCAST_MS      = 100;    // 10 Hz telemetry push

// ---------------------------------------------------------------------
//  WiFi - Access-Point mode (phone connects directly to the ESP32)
// ---------------------------------------------------------------------
constexpr char AP_SSID[] = "RC_Car_TestBench";
constexpr char AP_PASS[] = "carbench123";     // >= 8 chars, change as needed

// ---------------------------------------------------------------------
//  Camera - separate ESP32-CAM board (CamStreamer sketch) that joins this
//  AP as a station with a static IP. The browser and (later) the Pi pull
//  MJPEG straight from the CAM; video never passes through this ESP32.
//  Must match the constants at the top of CamStreamer/CamStreamer.ino.
// ---------------------------------------------------------------------
constexpr char     CAM_HOST[]      = "192.168.4.10";
constexpr uint16_t CAM_STREAM_PORT = 81;      // MJPEG at :81/stream, JPEG at :80/capture
