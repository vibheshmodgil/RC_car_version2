#pragma once
#include <Arduino.h>
#include "config.h"

// Reads all NUM_ENCODERS quadrature encoders (one per wheel) using pin-change
// interrupts for full 4x decoding, and computes per-wheel RPM.
//
// Implemented as a single global instance `Encoders` because the ISRs need a
// stable target. Each interrupt is bound with attachInterruptArg() so one
// shared ISR handles every channel — no per-pin static functions.
class EncoderReader {
public:
    void begin();
    void update();                       // call often from loop(); recomputes RPM at RPM_CALC_INTERVAL_MS

    int32_t count(uint8_t ch) const;     // raw quadrature count for wheel `ch`
    float   rpm(uint8_t ch) const;       // signed RPM for wheel `ch`
    void    resetCount(uint8_t ch);      // zero one wheel
    void    resetAll();

    float cpr() const        { return _cpr; }
    void  setCPR(float cpr)  { _cpr = (cpr > 0) ? cpr : DEFAULT_CPR; }

    struct Channel {
        uint8_t pinA = 0, pinB = 0;
        volatile int32_t count = 0;
        volatile uint8_t lastState = 0;   // 2-bit (A<<1)|B for the decode table
        int32_t lastCount = 0;
        float   rpm = 0.0f;
    };

private:
    static void IRAM_ATTR onEdge(void* arg);   // shared ISR, arg = &Channel

    Channel  _ch[NUM_ENCODERS];
    float    _cpr = DEFAULT_CPR;
    uint32_t _lastCalcMs = 0;
};

extern EncoderReader Encoders;
