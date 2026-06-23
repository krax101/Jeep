#include "knock.h"
#include "config.h"
#include "diagnostics.h"
#include <Arduino.h>
#include <IntervalTimer.h>

// Hardware note: PIN_KNOCK (A6) requires a 1.65 V DC bias before the ADC
// input so the AC knock signal is centred in the 0–3.3 V ADC range.
// Bias circuit: two 10 kΩ resistors from 3.3 V to GND; midpoint through
// 10 nF coupling cap to ADC pin.

#define KNOCK_SAMPLE_INTERVAL_US  40    // 25 kHz
#define KNOCK_ADC_MID             (ADC_MAX / 2)
#define KNOCK_NOISE_ALPHA         0.002f
#define KNOCK_HOLD_MS             200

static IntervalTimer s_timer;

static volatile uint16_t s_peak_raw  = 0;
static volatile uint16_t s_noise_raw = KNOCK_ADC_MID;
static volatile bool     s_new_peak  = false;

static uint16_t  s_peak_mv          = 0;
static uint32_t  s_last_knock_ms    = 0;
static float     s_recover_accum    = 0.0f;

static void knock_isr() {
    uint16_t raw      = (uint16_t)analogRead(PIN_KNOCK);
    int16_t  dev      = (int16_t)raw - (int16_t)KNOCK_ADC_MID;
    if (dev < 0) dev  = -dev;

    s_noise_raw = (uint16_t)(s_noise_raw +
                  KNOCK_NOISE_ALPHA * ((float)dev - (float)s_noise_raw));

    if ((uint16_t)dev > s_peak_raw)
        s_peak_raw = (uint16_t)dev;

    s_new_peak = true;
}

void knock_init() {
    pinMode(PIN_KNOCK, INPUT);
    s_timer.begin(knock_isr, KNOCK_SAMPLE_INTERVAL_US);
}

uint16_t knock_get_peak_mv() {
    return s_peak_mv;
}

// Called at 50 Hz. Applies retard on knock event; clears KNOCK fault when
// retard reaches zero after hold period.
void knock_update(IgnState& ign, DiagState& diag, uint32_t now_ms) {
    if (!s_new_peak) return;

    noInterrupts();
    uint16_t peak  = s_peak_raw;
    uint16_t noise = s_noise_raw;
    s_peak_raw     = 0;
    s_new_peak     = false;
    interrupts();

    s_peak_mv = (uint16_t)((float)peak * 3300.0f / (float)ADC_MAX);

    if ((float)peak > (float)noise * KNOCK_THRESHOLD_RATIO) {
        uint8_t nr = ign.knock_retard + (uint8_t)KNOCK_RETARD_STEP_DEG;
        ign.knock_retard = (nr > (uint8_t)KNOCK_RETARD_MAX_DEG)
                           ? (uint8_t)KNOCK_RETARD_MAX_DEG : nr;
        s_last_knock_ms  = now_ms;
        s_recover_accum  = 0.0f;
        diag_set(diag, FaultCode::KNOCK);
    }
}

// Called at 50 Hz. Decays knock_retard and clears fault when fully recovered.
void knock_recover(IgnState& ign, DiagState& diag, uint32_t now_ms) {
    if (ign.knock_retard == 0) return;
    if ((now_ms - s_last_knock_ms) < KNOCK_HOLD_MS) return;

    // KNOCK_RECOVER_DEG_S degrees per second, called 50 times per second
    s_recover_accum += KNOCK_RECOVER_DEG_S / 50.0f;
    if (s_recover_accum >= 1.0f) {
        uint8_t steps = (uint8_t)s_recover_accum;
        s_recover_accum -= (float)steps;
        if (ign.knock_retard > steps)
            ign.knock_retard -= steps;
        else
            ign.knock_retard = 0;
    }

    if (ign.knock_retard == 0)
        diag_clear(diag, FaultCode::KNOCK);
}
