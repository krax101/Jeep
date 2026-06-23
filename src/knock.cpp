#include "knock.h"
#include "config.h"
#include "diagnostics.h"
#include <Arduino.h>

// Knock window: 10–60° ATDC per cylinder.
// All six cylinders fire every 120°, so the window repeats modulo 1200 (120° × 10).
// Formula works for all cylinders simultaneously without per-cylinder tracking.
static inline bool in_knock_window(uint16_t angle_720_x10) {
    uint16_t phase = angle_720_x10 % 1200;
    return (phase >= 100 && phase < 600);
}

#define KNOCK_HOLD_MS  200

static float    s_noise_floor   = 0.0f;
static float    s_envelope      = 0.0f;
static uint16_t s_peak_mv       = 0;
static uint32_t s_last_knock_ms = 0;
static float    s_recover_accum = 0.0f;

uint16_t knock_get_peak_mv() {
    return s_peak_mv;
}

void knock_update(IgnState& ign, DiagState& diag,
                  uint16_t angle_720_x10, uint32_t now_ms) {
    uint16_t raw = (uint16_t)analogRead(PIN_KNOCK);
    float    mv  = (float)raw * 3300.0f / 4095.0f;

    // Smooth the hardware envelope reading with fast IIR
    s_envelope += KNOCK_ENVELOPE_ALPHA * (mv - s_envelope);

    if (!in_knock_window(angle_720_x10)) {
        // Outside window: update noise floor with slow adaptation
        s_noise_floor += KNOCK_NOISE_ALPHA * (s_envelope - s_noise_floor);
        return;
    }

    // Inside knock window: track peak for monitor stream
    if ((uint16_t)s_envelope > s_peak_mv)
        s_peak_mv = (uint16_t)s_envelope;

    // Seed floor on first reading to avoid false triggers at startup
    if (s_noise_floor < 10.0f) {
        s_noise_floor = s_envelope;
        return;
    }

    if (s_envelope > s_noise_floor * KNOCK_THRESHOLD_RATIO) {
        uint8_t nr = ign.knock_retard + (uint8_t)KNOCK_RETARD_STEP_DEG;
        ign.knock_retard = (nr > (uint8_t)KNOCK_RETARD_MAX_DEG)
                           ? (uint8_t)KNOCK_RETARD_MAX_DEG : nr;
        s_last_knock_ms  = now_ms;
        s_recover_accum  = 0.0f;
        diag_set(diag, FaultCode::KNOCK);
    }
}

void knock_recover(IgnState& ign, DiagState& diag, uint32_t now_ms) {
    if (ign.knock_retard == 0) {
        s_peak_mv = 0;
        return;
    }
    if ((now_ms - s_last_knock_ms) < KNOCK_HOLD_MS) return;

    // Decay at KNOCK_RECOVER_DEG_S degrees/second, called 50 times per second
    s_recover_accum += KNOCK_RECOVER_DEG_S / 50.0f;
    if (s_recover_accum >= 1.0f) {
        uint8_t steps = (uint8_t)s_recover_accum;
        s_recover_accum -= (float)steps;
        if (ign.knock_retard > steps)
            ign.knock_retard -= steps;
        else
            ign.knock_retard = 0;
    }

    if (ign.knock_retard == 0) {
        diag_clear(diag, FaultCode::KNOCK);
        s_peak_mv = 0;
    }
}
