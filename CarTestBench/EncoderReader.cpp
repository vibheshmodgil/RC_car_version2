#include "EncoderReader.h"

EncoderReader Encoders;

// Shared ISR: fires on any edge of A or B for a given channel. A full 4x
// quadrature decode that works no matter which line moved: index the standard
// Gray-code transition table by (previousState << 2) | newState. Valid steps
// give +/-1; impossible double-transitions (noise) give 0. Direction sign
// depends on A/B wiring and is consistent — flip the harness if it reads
// backwards. Works from one shared handler, so all 4 wheels reuse it.
void IRAM_ATTR EncoderReader::onEdge(void* arg) {
    static const int8_t LUT[16] = {
        0, -1, +1,  0,
       +1,  0,  0, -1,
       -1,  0,  0, +1,
        0, +1, -1,  0
    };
    Channel* c = static_cast<Channel*>(arg);
    uint8_t s = (digitalRead(c->pinA) << 1) | digitalRead(c->pinB);
    c->count += LUT[(c->lastState << 2) | s];
    c->lastState = s;
}

void EncoderReader::begin() {
    for (uint8_t i = 0; i < NUM_ENCODERS; i++) {
        _ch[i].pinA = PINS_ENCODER[i].a;
        _ch[i].pinB = PINS_ENCODER[i].b;
        // Some encoder pins are input-only and have no internal pull-ups, and
        // the encoders must be driven at 3.3 V (see config.h safety note).
        // External pull-ups belong on the wiring harness, not here.
        pinMode(_ch[i].pinA, INPUT);
        pinMode(_ch[i].pinB, INPUT);
        _ch[i].lastState = (digitalRead(_ch[i].pinA) << 1) | digitalRead(_ch[i].pinB);
        attachInterruptArg(digitalPinToInterrupt(_ch[i].pinA), onEdge, &_ch[i], CHANGE);
        attachInterruptArg(digitalPinToInterrupt(_ch[i].pinB), onEdge, &_ch[i], CHANGE);
    }
}

void EncoderReader::update() {
    uint32_t now = millis();
    uint32_t dt = now - _lastCalcMs;
    if (dt < RPM_CALC_INTERVAL_MS) return;
    _lastCalcMs = now;

    for (uint8_t i = 0; i < NUM_ENCODERS; i++) {
        noInterrupts();
        int32_t c = _ch[i].count;
        interrupts();
        int32_t delta = c - _ch[i].lastCount;
        _ch[i].lastCount = c;
        // (counts / cpr) revolutions, scaled to per-minute.
        _ch[i].rpm = ((float)delta / _cpr) * (60000.0f / (float)dt);
    }
}

int32_t EncoderReader::count(uint8_t ch) const {
    if (ch >= NUM_ENCODERS) return 0;
    noInterrupts();
    int32_t c = _ch[ch].count;
    interrupts();
    return c;
}

float EncoderReader::rpm(uint8_t ch) const {
    return (ch < NUM_ENCODERS) ? _ch[ch].rpm : 0.0f;
}

void EncoderReader::resetCount(uint8_t ch) {
    if (ch >= NUM_ENCODERS) return;
    noInterrupts();
    _ch[ch].count = 0;
    interrupts();
    _ch[ch].lastCount = 0;
}

void EncoderReader::resetAll() {
    for (uint8_t i = 0; i < NUM_ENCODERS; i++) resetCount(i);
}
