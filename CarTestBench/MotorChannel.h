#pragma once
#include <Arduino.h>
#include "config.h"

// One TB6612FNG H-bridge channel - one per wheel in the 4WD layout, so the
// firmware creates four of these: motorLF, motorLR, motorRF, motorRR
// (see CarTestBench.ino).
//
// DRIVE SCHEME - standard TB6612FNG wiring:
//   forward: IN1=HIGH, IN2=LOW,  PWM=duty
//   reverse: IN1=LOW,  IN2=HIGH, PWM=duty
//   brake:   IN1=IN2=HIGH,       PWM=255
//   coast:   IN1=IN2=LOW,        PWM=0
//
// SAFETY MODEL - two layers:
//   * Each TB6612 board has its OWN STBY line (pins.stby; the two channels
//     of a board share it). It is held LOW at boot and whenever NEITHER of
//     the board's channels is armed, and drops on a master e-stop - the
//     board's outputs are then off no matter what the IN/PWM pins do. A
//     ~10 kOhm pull-down on each STBY keeps the motors dead during boot
//     before firmware runs. STBY refcounting is managed internally.
//   * Per channel, disarmed/e-stopped => PWM=0 and IN1=IN2=LOW (that channel
//     coasts even while its board-mate keeps STBY high).
// A channel must be explicitly arm()ed before setPWM() does anything.
//
// PWM convention: signed int16, -255..+255.
//   positive => forward, negative => reverse, 0 => coast.
class MotorChannel {
public:
    explicit MotorChannel(const MotorPins& pins, const char* name);

    void begin();               // pins to a safe state: coast, board STBY LOW

    void arm();                 // allow drive; raises this board's STBY
    void disarm();              // coast + drop STBY if the board pair is idle
    bool isArmed() const        { return _armed; }

    void setPWM(int16_t pwm);   // only moves when armed and not e-stopped
    void setInverted(bool inv); // software polarity swap (fixes reversed wiring)
    bool isInverted() const     { return _invert; }
    void brake();               // safe stop by default; active brake only if enabled in config
    void coast();               // outputs off, stays armed
    void emergencyStop();       // latched: disarms and blocks setPWM until cleared
    void clearEmergencyStop();

    int16_t currentPWM() const  { return _pwm; }
    bool    isEStopped() const  { return _estop; }
    const char* name() const    { return _name; }

private:
    void applyCoast();
    void applyDrive(int16_t pwm);

    // Per-board STBY lines: HIGH iff at least one channel on that board is
    // armed. Registry is static because boards are shared across instances.
    struct StbyLine { uint8_t pin; uint8_t armedCount; };
    static StbyLine* stbyFor(uint8_t pin);   // find-or-init the line for `pin`
    static StbyLine s_stby[NUM_MOTORS];
    static uint8_t  s_stbyCount;

    MotorPins   _pins;
    const char* _name;
    StbyLine*   _stby = nullptr;
    int16_t     _pwm = 0;
    bool        _invert = false;
    bool        _armed = false;
    bool        _estop = false;
    uint32_t    _lastDirChangeMs = 0;
};
