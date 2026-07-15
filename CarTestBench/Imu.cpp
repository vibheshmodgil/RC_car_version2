#include "Imu.h"

Imu IMU;

#if ENABLE_BNO055
// ---- Live implementation (requires Adafruit_BNO055 + Adafruit_Sensor) ----
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

static Adafruit_BNO055 bno(55, BNO055_I2C_ADDR, &Wire);

void Imu::begin() {
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
    _present = bno.begin();
    if (_present) bno.setExtCrystalUse(true);
}

void Imu::update() {
    if (!_present) { _reading.ok = false; return; }

    sensors_event_t evt;
    bno.getEvent(&evt);
    _reading.ok      = true;
    _reading.heading = evt.orientation.x;
    _reading.roll    = evt.orientation.y;
    _reading.pitch   = evt.orientation.z;
    bno.getCalibration(&_reading.calSys, &_reading.calGyro,
                       &_reading.calAccel, &_reading.calMag);
}

#else
// ---- Disabled stub: keeps the build green until the IMU is wired up. ----
void Imu::begin()  { _present = false; }
void Imu::update() { _reading.ok = false; }
#endif
