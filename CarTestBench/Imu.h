#pragma once
#include <Arduino.h>
#include "config.h"

// BNO055 9-axis IMU — groundwork module.
//
// Compiles to a harmless no-op while ENABLE_BNO055 == 0, so the rest of the
// firmware builds without the Adafruit libraries. Flip ENABLE_BNO055 to 1 in
// config.h (and install Adafruit_BNO055 + Adafruit_Sensor) to bring it live;
// the public interface below stays the same, so AppServer / the IMU page need
// no changes.
class Imu {
public:
    struct Reading {
        bool  ok = false;        // sensor present and producing data
        float heading = 0.0f;    // yaw, degrees
        float roll = 0.0f;       // degrees
        float pitch = 0.0f;      // degrees
        uint8_t calSys = 0;      // BNO055 calibration levels (0..3)
        uint8_t calGyro = 0;
        uint8_t calAccel = 0;
        uint8_t calMag = 0;
    };

    void begin();                // safe to call even when disabled
    void update();               // call from loop(); no-op when disabled
    bool enabled() const         { return _enabled; }
    bool present() const         { return _present; }
    Reading reading() const      { return _reading; }

private:
    bool    _enabled = (ENABLE_BNO055 != 0);
    bool    _present = false;
    Reading _reading;
};

extern Imu IMU;
