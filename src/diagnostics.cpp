#include "diagnostics.h"
#include "config.h"
#include <Arduino.h>

static const char* FAULT_NAMES[] = {
    "NONE", "CLT_HIGH", "CLT_LOW", "IAT_HIGH", "IAT_LOW",
    "TPS_HIGH", "TPS_LOW", "MAP_HIGH", "MAP_LOW",
    "O2_INACTIVE", "CPS_LOSS", "CAM_LOSS",
    "INJ_OC", "KNOCK", "BATT_HIGH", "BATT_LOW"
};

void diag_init(DiagState& d) {
    memset(&d, 0, sizeof(d));
    pinMode(PIN_CEL, OUTPUT);
    digitalWriteFast(PIN_CEL, LOW);
}

void diag_set(DiagState& d, FaultCode code) {
    uint8_t idx = (uint8_t)code;
    if (idx == 0 || idx >= (uint8_t)FaultCode::MAX_CODES) return;
    if (!d.active[idx]) {
        d.active[idx]      = true;
        d.first_set_ms[idx] = millis();
        d.active_count++;
    }
    d.cel_on = true;
    digitalWriteFast(PIN_CEL, HIGH);
}

void diag_clear(DiagState& d, FaultCode code) {
    uint8_t idx = (uint8_t)code;
    if (idx == 0 || idx >= (uint8_t)FaultCode::MAX_CODES) return;
    if (d.active[idx]) {
        d.active[idx] = false;
        if (d.active_count > 0) d.active_count--;
    }
    if (d.active_count == 0) {
        d.cel_on = false;
        digitalWriteFast(PIN_CEL, LOW);
    }
}

// CPS loss detection: track last tooth time
static uint32_t s_last_cps_ms = 0;
static uint32_t s_last_tooth_count = 0;

void diag_update(DiagState& d, const SensorData& s, const CrankState& cs,
                 uint32_t now_ms) {
    // --- CLT sensor ------------------------------------------
    if (s.clt_c >= 115)       diag_set(d, FaultCode::CLT_HIGH);
    else                       diag_clear(d, FaultCode::CLT_HIGH);
    if (s.clt_fault && s.clt_c <= -39) diag_set(d, FaultCode::CLT_LOW);
    else                       diag_clear(d, FaultCode::CLT_LOW);

    // --- IAT sensor ------------------------------------------
    if (s.iat_c >= 90)         diag_set(d, FaultCode::IAT_HIGH);
    else                       diag_clear(d, FaultCode::IAT_HIGH);
    if (s.iat_fault && s.iat_c <= -39) diag_set(d, FaultCode::IAT_LOW);
    else                       diag_clear(d, FaultCode::IAT_LOW);

    // --- MAP sensor ------------------------------------------
    if (s.map_kpa >= 108)      diag_set(d, FaultCode::MAP_HIGH);
    else                       diag_clear(d, FaultCode::MAP_HIGH);
    if (s.map_kpa <= 11)       diag_set(d, FaultCode::MAP_LOW);
    else                       diag_clear(d, FaultCode::MAP_LOW);

    // --- Battery voltage -------------------------------------
    if (s.batt_mv > 16000)     diag_set(d, FaultCode::BATT_HIGH);
    else                       diag_clear(d, FaultCode::BATT_HIGH);
    if (s.batt_mv < 9000 && s.rpm > 400) diag_set(d, FaultCode::BATT_LOW);
    else                       diag_clear(d, FaultCode::BATT_LOW);

    // --- CPS loss detection ----------------------------------
    // If tooth count hasn't changed for >500 ms during expected running, flag it
    if (cs.tooth_head != s_last_tooth_count) {
        s_last_tooth_count = cs.tooth_head;
        s_last_cps_ms      = now_ms;
        diag_clear(d, FaultCode::CPS_LOSS);
    } else if (s.rpm > 0 && (now_ms - s_last_cps_ms) > 500) {
        diag_set(d, FaultCode::CPS_LOSS);
    }

    // --- O2 sensor inactive check (only when engine warm & running) ----
    if (s.clt_c > 70 && s.rpm > 600) {
        if (s.o2_mv < 50 || s.o2_mv > 950)
            diag_set(d, FaultCode::O2_INACTIVE);
        else
            diag_clear(d, FaultCode::O2_INACTIVE);
    }
}

void diag_print(const DiagState& d) {
    if (d.active_count == 0) {
        TUNING_SERIAL.println(F("DIAG: No active faults"));
        return;
    }
    TUNING_SERIAL.print(F("DIAG: "));
    TUNING_SERIAL.print(d.active_count);
    TUNING_SERIAL.println(F(" active fault(s):"));
    for (uint8_t i = 1; i < (uint8_t)FaultCode::MAX_CODES; i++) {
        if (d.active[i]) {
            TUNING_SERIAL.print(F("  [P0"));
            TUNING_SERIAL.print(i, DEC);
            TUNING_SERIAL.print(F("] "));
            TUNING_SERIAL.println(FAULT_NAMES[i]);
        }
    }
}
