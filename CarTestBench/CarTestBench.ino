// =====================================================================
//  CarTestBench.ino  —  Advanced RC Car V2 test bench firmware
//
//  Self-hosted WiFi test rig for the car's subsystems. Connect a phone to
//  the ESP32's access point and each subsystem gets its own clean page:
//      /          overview / safety
//      /motors    drive all four wheels independently (2x TB6612FNG)
//      /drive     whole-car motion: fwd/back + skid-steer spins
//      /encoders  live counts + RPM + CPR calibration for all 4 wheels
//      /imu       Pi sensor bridge placeholder (MPU/LiDAR/ToF live on Pi)
//      /camera    MJPEG feed embedded from the separate ESP32-CAM board
//
//  Hardware/pins live in config.h. See CLAUDE.md for the full map and the
//  conventions every module follows. Arduino-ESP32 core 3.x required.
// =====================================================================
#include <Arduino.h>
#include "config.h"
#include "MotorChannel.h"
#include "EncoderReader.h"
#include "Imu.h"
#include "AppServer.h"

// Four independent motor channels (2x TB6612FNG, one channel per wheel).
// Index order matches the encoders: 0=LF, 1=LR, 2=RF, 3=RR.
MotorChannel motorLF(PINS_MOTOR[0], "LF");
MotorChannel motorLR(PINS_MOTOR[1], "LR");
MotorChannel motorRF(PINS_MOTOR[2], "RF");
MotorChannel motorRR(PINS_MOTOR[3], "RR");

MotorChannel* motors[NUM_MOTORS] = { &motorLF, &motorLR, &motorRF, &motorRR };

// EncoderReader (Encoders) and Imu (IMU) are global singletons from their .cpp.
AppServer server(motors, Encoders, IMU);

void setup() {
    Serial.begin(115200);
    Serial.println("\n=== RC Car Test Bench ===");

    for (auto* m : motors) m->begin();
    Encoders.begin();
    IMU.begin();
    server.begin();

    Serial.println("Ready.");
}

void loop() {
    Encoders.update();   // recompute per-wheel RPM
    IMU.update();        // no-op; ESP32-local sensors are disabled in this pin map
    server.loop();       // push telemetry to connected phones
}
