#include "diagnostics.h"
#include "config.h"
#include <Arduino.h>

static const char* FAULT_NAMES[(uint8_t)FaultCode::MAX_CODES] = {
    "NONE", "CLT_HIGH", "CLT_LOW", "IAT_HIGH", "IAT_LOW",
    "TPS_HIGH", "TPS_LOW", "MAP_HIGH", "MAP_LOW",
    "O2_INACTIVE", "CPS_LOSS", "CAM_LOSS",
    "INJ_OC", "KNOCK", "BATT_HIGH", "BATT_LOW",
    "O2_HEATER_FAULT"
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
                 uint32_t now_ms, uint32_t run_start_ms) {
    // --- CLT sensor ------------------------------------------
    if (s.clt_c >= 115)       diag_set(d, FaultCode::CLT_HIGH);
    else                       diag_clear(d, FaultCode::CLT_HIGH);
    if (s.clt_fault && s.clt_c <= -39) diag_set(d, FaultCode::CLT_LOW);
    else                       diag_clear(d, FaultCode::CLT_LOW);

    // --- TPS sensor ------------------------------------------
    if (s.tps_fault) {
        if (s.tps_pct <= 2) { diag_set(d, FaultCode::TPS_LOW);  diag_clear(d, FaultCode::TPS_HIGH); }
        else                 { diag_set(d, FaultCode::TPS_HIGH); diag_clear(d, FaultCode::TPS_LOW);  }
    } else {
        diag_clear(d, FaultCode::TPS_HIGH);
        diag_clear(d, FaultCode::TPS_LOW);
    }

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

    // --- CAM loss detection (only after cam sync was established) ---
    // At 500 RPM, cam pulse arrives every 240 ms. Allow 1.5 s between pulses.
    {
        static uint32_t s_prev_cam_us = 0;
        static uint32_t s_last_seen_ms = 0;

        uint32_t cam_now;
        noInterrupts();
        cam_now = cs.last_cam_us;
        interrupts();

        if (cam_now != s_prev_cam_us) {    // new cam pulse arrived
            s_prev_cam_us = cam_now;
            s_last_seen_ms = now_ms;
            diag_clear(d, FaultCode::CAM_LOSS);
        } else if (cs.cam_synced && s.rpm > 400 &&
                   (now_ms - s_last_seen_ms) > 1500) {
            diag_set(d, FaultCode::CAM_LOSS);
        }
    }

    // --- O2 sensor inactive check (only when engine warm & running) ----
    // The heater doesn't enable until O2_HEATER_DELAY_MS after engine start,
    // and the sensor needs ~30 s after heater-on to reach operating temperature.
    // Checking immediately at CLT>70°C would always flag O2_INACTIVE during the
    // first 60 s of running, permanently inhibiting closed-loop on a cold start.
    const uint32_t O2_CHECK_DELAY_MS = O2_HEATER_DELAY_MS + 30000UL;
    bool o2_ready = (now_ms - run_start_ms) >= O2_CHECK_DELAY_MS;
    if (s.clt_c > 70 && s.rpm > 600 && o2_ready) {
        if (s.o2_mv < 50 || s.o2_mv > 950)
            diag_set(d, FaultCode::O2_INACTIVE);
        else
            diag_clear(d, FaultCode::O2_INACTIVE);
    }
}

void diag_print(const DiagState& d) {
    // SAE J2012 five-digit P-codes mapped to each internal FaultCode index.
    // "P0" + decimal index is not a real code (e.g. index 9 → "P09" ≠ any SAE code).
    static const char* const OBD_CODES[(uint8_t)FaultCode::MAX_CODES] = {
        "P0000",  // NONE
        "P0118",  // CLT_HIGH        ECT Circuit High Input
        "P0117",  // CLT_LOW         ECT Circuit Low Input
        "P0113",  // IAT_HIGH        IAT Circuit High Input
        "P0112",  // IAT_LOW         IAT Circuit Low Input
        "P0123",  // TPS_HIGH        TP Sensor Circuit High Input
        "P0122",  // TPS_LOW         TP Sensor Circuit Low Input
        "P0108",  // MAP_HIGH        MAP Circuit High Input
        "P0107",  // MAP_LOW         MAP Circuit Low Input
        "P0136",  // O2_INACTIVE     O2 Sensor Circuit (Bank 1, Sensor 1)
        "P0335",  // CPS_LOSS        CKP Sensor A Circuit
        "P0340",  // CAM_LOSS        CMP Sensor A Circuit
        "P0201",  // INJ_OC          Injector Circuit Open (representative)
        "P0325",  // KNOCK           Knock Sensor Circuit
        "P0563",  // BATT_HIGH       System Voltage High
        "P0562",  // BATT_LOW        System Voltage Low
        "P0031",  // O2_HEATER_FAULT HO2S Heater Control Circuit Low
    };

    if (d.active_count == 0) {
        TUNING_SERIAL.println(F("DIAG: No active faults"));
        return;
    }
    TUNING_SERIAL.print(F("DIAG: "));
    TUNING_SERIAL.print(d.active_count);
    TUNING_SERIAL.println(F(" active fault(s):"));
    for (uint8_t i = 1; i < (uint8_t)FaultCode::MAX_CODES; i++) {
        if (d.active[i]) {
            TUNING_SERIAL.print(F("  ["));
            TUNING_SERIAL.print(OBD_CODES[i]);
            TUNING_SERIAL.print(F("] "));
            TUNING_SERIAL.println(FAULT_NAMES[i]);
        }
    }
}
