#include "MotorChannel.h"

MotorChannel::StbyLine MotorChannel::s_stby[NUM_MOTORS];
uint8_t MotorChannel::s_stbyCount = 0;

MotorChannel::MotorChannel(const MotorPins& pins, const char* name)
    : _pins(pins), _name(name) {}

MotorChannel::StbyLine* MotorChannel::stbyFor(uint8_t pin) {
    for (uint8_t i = 0; i < s_stbyCount; i++)
        if (s_stby[i].pin == pin) return &s_stby[i];

    // First channel on this board: claim the line, LOW first so the bridge is
    // disabled the instant the pin becomes an output. The external pull-down
    // keeps the motors dead during boot before firmware starts.
    StbyLine* l = &s_stby[s_stbyCount++];
    l->pin = pin;
    l->armedCount = 0;
    digitalWrite(pin, LOW);
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
    return l;
}

void MotorChannel::begin() {
    _stby = stbyFor(_pins.stby);

    digitalWrite(_pins.in1, LOW);
    digitalWrite(_pins.in2, LOW);
    pinMode(_pins.in1, OUTPUT);
    pinMode(_pins.in2, OUTPUT);

    // ESP32 Arduino core 3.x ledc API. Only the TB6612 PWM pin is attached to
    // LEDC; IN1/IN2 stay as plain direction pins.
    ledcAttach(_pins.pwm, PWM_FREQ, PWM_RES);
    applyCoast();

    _armed = false;
    _pwm = 0;
}

// Coast/stop: TB6612 IN1=IN2=LOW with PWM=0.
void MotorChannel::applyCoast() {
    ledcWrite(_pins.pwm, 0);
    digitalWrite(_pins.in1, LOW);
    digitalWrite(_pins.in2, LOW);
}

// Drive: direction on IN pins, magnitude on the PWM pin. pwm != 0 here.
// _invert flips polarity at the last moment so the rest of the firmware
// (sliders, drive endpoint, telemetry) always works in logical fwd/rev.
void MotorChannel::applyDrive(int16_t pwm) {
    if (_invert) pwm = -pwm;
    if (pwm > 0) {
        digitalWrite(_pins.in1, HIGH);
        digitalWrite(_pins.in2, LOW);
        ledcWrite(_pins.pwm, pwm);
    } else {
        digitalWrite(_pins.in1, LOW);
        digitalWrite(_pins.in2, HIGH);
        ledcWrite(_pins.pwm, -pwm);
    }
}

void MotorChannel::arm() {
    _estop = false;             // arming is a deliberate re-enable; clear any latch
    _pwm = 0;
    applyCoast();               // make sure we come up stopped
    if (!_armed) {
        _armed = true;
        if (_stby->armedCount++ == 0)
            digitalWrite(_stby->pin, HIGH);   // board live, this channel coasting
    }
    Serial.printf("[%s] ARMED (board STBY high)\n", _name);
}

void MotorChannel::disarm() {
    _pwm = 0;
    applyCoast();               // this channel's outputs off regardless of STBY
    if (_armed) {
        _armed = false;
        if (--_stby->armedCount == 0)
            digitalWrite(_stby->pin, LOW);    // board pair idle - hardware cut
    }
    Serial.printf("[%s] disarmed\n", _name);
}

void MotorChannel::setPWM(int16_t pwm) {
    if (!_armed || _estop) return;

    pwm = constrain(pwm, (int16_t)-PWM_MAX, (int16_t)PWM_MAX);

    // Brief brake before a direction reversal to protect the H-bridge.
    const bool reversing = (_pwm > 0 && pwm < 0) || (_pwm < 0 && pwm > 0);
    if (reversing) {
        if (millis() - _lastDirChangeMs < DIR_CHANGE_DEADTIME_MS) {
            brake();
            return;
        }
        _lastDirChangeMs = millis();
    }

    _pwm = pwm;
    if (pwm == 0) applyCoast();
    else          applyDrive(pwm);
}

void MotorChannel::setInverted(bool inv) {
    if (_invert == inv) return;
    _invert = inv;
    if (_armed && !_estop && _pwm != 0) applyDrive(_pwm);   // re-apply live
    Serial.printf("[%s] direction %s\n", _name, inv ? "REVERSED" : "normal");
}

void MotorChannel::brake() {
    if (!_armed) return;
    _pwm = 0;
    if (!MOTOR_ACTIVE_BRAKE_ENABLED) {
        applyCoast();
        return;
    }
    digitalWrite(_pins.in1, HIGH);   // IN1=IN2=HIGH = short brake
    digitalWrite(_pins.in2, HIGH);
    ledcWrite(_pins.pwm, PWM_MAX);
}

void MotorChannel::coast() {
    _pwm = 0;
    applyCoast();
}

void MotorChannel::emergencyStop() {
    _estop = true;
    disarm();                    // coast + STBY drop when the board pair is out
    Serial.printf("[%s] E-STOP latched\n", _name);
}

void MotorChannel::clearEmergencyStop() {
    _estop = false;              // stays disarmed; user must re-arm to move
    Serial.printf("[%s] E-STOP cleared (now re-arm to move)\n", _name);
}
