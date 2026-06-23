#include "iac.h"
#include "tables.h"
#include "config.h"
#include <Arduino.h>

// IAC stepper: 4-wire, full-step drive (A+A-B+B-)
// Step sequence: 0→1→2→3→0
static const uint8_t STEP_SEQ[4][4] = {
    //A+ A- B+ B-
    { 1,  0,  1,  0},
    { 0,  1,  1,  0},
    { 0,  1,  0,  1},
    { 1,  0,  0,  1},
};

#define IAC_MAX_STEPS     200     // Full range of motion
#define IAC_STEP_DELAY_MS 10      // ms between steps (stepper speed)
#define IAC_HOME_STEPS    220     // Overshoot to guarantee home position

static void iac_apply_phase(uint8_t phase) {
    digitalWriteFast(PIN_IAC_A_POS, STEP_SEQ[phase][0]);
    digitalWriteFast(PIN_IAC_A_NEG, STEP_SEQ[phase][1]);
    digitalWriteFast(PIN_IAC_B_POS, STEP_SEQ[phase][2]);
    digitalWriteFast(PIN_IAC_B_NEG, STEP_SEQ[phase][3]);
}

static void iac_deenergise() {
    digitalWriteFast(PIN_IAC_A_POS, LOW);
    digitalWriteFast(PIN_IAC_A_NEG, LOW);
    digitalWriteFast(PIN_IAC_B_POS, LOW);
    digitalWriteFast(PIN_IAC_B_NEG, LOW);
}

void iac_init(IACState& iac) {
    pinMode(PIN_IAC_A_POS, OUTPUT);
    pinMode(PIN_IAC_A_NEG, OUTPUT);
    pinMode(PIN_IAC_B_POS, OUTPUT);
    pinMode(PIN_IAC_B_NEG, OUTPUT);
    iac_deenergise();

    iac.phase    = 0;
    iac.position = 0;
    iac.target   = 0;

    // Home: retract IAC fully (close pintle, minimum air)
    for (int i = 0; i < IAC_HOME_STEPS; i++) {
        iac.phase = (iac.phase + 3) & 0x03;  // reverse direction
        iac_apply_phase(iac.phase);
        delay(IAC_STEP_DELAY_MS);
    }
    iac_deenergise();
    iac.position = 0;
    iac.target   = 50;  // Start at mid-range for initial idle
}

void iac_update(IACState& iac, const SensorData& s, const ECUConfig& cfg) {
    uint32_t now = millis();
    if ((now - iac.last_step_ms) < IAC_STEP_DELAY_MS) return;
    iac.last_step_ms = now;

    // Update target based on CLT-corrected idle RPM target (open loop warmup)
    iac.target_rpm = (uint16_t)table1d_lookup(cfg.idle_vs_clt, (int16_t)s.clt_c);

    if (iac.position == iac.target) {
        iac_deenergise();
        return;
    }

    if (iac.position < iac.target) {
        iac.phase = (iac.phase + 1) & 0x03;  // open (more air)
        iac.position++;
    } else {
        iac.phase = (iac.phase + 3) & 0x03;  // close (less air)
        iac.position--;
    }

    iac_apply_phase(iac.phase);
}

void iac_set_target(IACState& iac, int16_t position) {
    if (position < 0) position = 0;
    if (position > IAC_MAX_STEPS) position = IAC_MAX_STEPS;
    iac.target = position;
}

void iac_park(IACState& iac) {
    // Move to parked position (~15% open for reliable restart)
    iac_set_target(iac, 30);
}

void iac_apply_idle_compensation(IACState& iac, bool ac_on,
                                 bool ps_load, bool in_drive) {
    int16_t bump = 0;
    if (ac_on)    bump += IDLE_AC_BUMP_STEPS;
    if (ps_load)  bump += IDLE_PS_BUMP_STEPS;
    if (in_drive) bump += IDLE_DRIVE_BUMP_STEPS;
    // Add bump on top of whatever the P-controller already targeted
    iac_set_target(iac, iac.target + bump);
}
