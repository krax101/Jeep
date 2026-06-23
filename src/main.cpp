#include <Arduino.h>
#include "config.h"
#include "types.h"
#include "tables.h"
#include "crank.h"
#include "sensors.h"
#include "scheduler.h"
#include "fuel.h"
#include "ignition.h"
#include "corrections.h"
#include "closedloop.h"
#include "iac.h"
#include "diagnostics.h"
#include "comms.h"
#include "storage.h"

// ---- Globals ------------------------------------------------
static ECUState  g_state;
static ECUConfig g_cfg;

// ---- Task Timers --------------------------------------------
static uint32_t t_sensors   = 0;   // 20 ms  (50 Hz)
static uint32_t t_fuel_calc = 0;   // 20 ms  (50 Hz)
static uint32_t t_cl        = 0;   // 100 ms (10 Hz)
static uint32_t t_iac       = 0;   // 100 ms (10 Hz)
static uint32_t t_diag      = 0;   // 200 ms (5 Hz)
static uint32_t t_comms     = 0;   // 5 ms   (200 Hz)
static uint32_t t_uptime    = 0;   // 1000 ms

// ---- Engine State Machine -----------------------------------
static void update_engine_state() {
    ECUState& st = g_state;
    uint16_t rpm = st.sensors.rpm;

    switch (st.engine_state) {
        case EngineState::OFF:
            if (crank_is_synced(st.crank) && rpm > 50)
                st.engine_state = EngineState::CRANKING;
            break;
        case EngineState::CRANKING:
            if (rpm >= 450) {
                st.engine_state = EngineState::WARMUP;
                // Initialise after-start enrichment
                float ase_count = table1d_lookup(g_cfg.ase_table, (int16_t)st.sensors.clt_c);
                st.ase_events_left = (uint8_t)(ase_count > 255 ? 255 : ase_count);
            }
            break;
        case EngineState::WARMUP:
            if (st.sensors.clt_c >= 80)
                st.engine_state = EngineState::RUNNING;
            if (rpm < 200)
                st.engine_state = EngineState::OFF;
            break;
        case EngineState::RUNNING:
        case EngineState::OVERRUN:
            if (rpm < 200) {
                st.engine_state = EngineState::OFF;
                break;
            }
            if (st.sensors.tps_pct < IDLE_TPS_THRESHOLD_PCT && rpm > 1800)
                st.engine_state = EngineState::OVERRUN;
            else
                st.engine_state = EngineState::RUNNING;
            break;
    }
}

// ---- Fuel Mode Decision -------------------------------------
static void update_fuel_mode() {
    ECUState& st = g_state;
    uint16_t rpm = st.sensors.rpm;

    if (st.rev_limit_active) {
        st.fuel.mode = InjMode::SEQUENTIAL;  // mode doesn't matter when cut
        st.fuel_mode  = FuelMode::CUT;
        return;
    }

    switch (st.engine_state) {
        case EngineState::OFF:
            st.fuel_mode = FuelMode::CUT;
            break;
        case EngineState::CRANKING:
            st.fuel_mode = FuelMode::CRANKING;
            break;
        case EngineState::OVERRUN:
            // Fuel cut on overrun above 1800 RPM
            st.fuel_mode = (rpm > 1800) ? FuelMode::CUT : FuelMode::OPEN;
            break;
        default:
            st.fuel_mode = g_cfg.cl_enabled ? FuelMode::CLOSED : FuelMode::OPEN;
            break;
    }

    // Injection mode: sequential requires cam sync
    st.fuel.mode = st.crank.cam_synced ? InjMode::SEQUENTIAL : InjMode::BATCH;
}

// ---- Rev Limiter --------------------------------------------
static void check_rev_limit() {
    uint16_t rpm = g_state.sensors.rpm;
    if (rpm > REV_LIMIT_HARD_RPM) {
        g_state.rev_limit_active = true;
    } else if (rpm < (REV_LIMIT_HARD_RPM - REV_LIMIT_HYSTERESIS_RPM)) {
        g_state.rev_limit_active = false;
    }
}

// ---- Fuel Pump Relay ----------------------------------------
static void update_fuel_pump() {
    static uint32_t prime_start = 0;
    static bool primed = false;
    if (!primed) {
        prime_start = millis();
        digitalWriteFast(PIN_FUEL_PUMP_RELAY, HIGH);
        primed = true;
    }
    // Keep pump running while engine is cranking or running
    bool pump_on = (g_state.engine_state != EngineState::OFF) ||
                   ((millis() - prime_start) < FUEL_PUMP_PRIME_MS);
    digitalWriteFast(PIN_FUEL_PUMP_RELAY, pump_on ? HIGH : LOW);
}

// ---- Cranking Fuel Calculation ------------------------------
static uint32_t calc_cranking_pw() {
    float base = table1d_lookup(g_cfg.crank_enrich, (int16_t)g_state.sensors.clt_c);
    float flow_g_per_us = (g_cfg.injector_flow_cc_min * ENGINE_FUEL_DENSITY_G_CC)
                          / 60000000.0f;
    // Simplified: 5 mg of fuel as base cranking pulse, scaled by enrichment
    float fuel_g = 0.005f * (base / 100.0f + 1.0f);
    if (flow_g_per_us <= 0.0f) return 3000;
    return (uint32_t)(fuel_g / flow_g_per_us);
}

// ============================================================
// Arduino Entry Points
// ============================================================

void setup() {
    // Output pins
    pinMode(PIN_FUEL_PUMP_RELAY, OUTPUT);
    pinMode(PIN_CEL,            OUTPUT);
    pinMode(PIN_TACH_OUT,       OUTPUT);
    pinMode(PIN_EGR,            OUTPUT);
    pinMode(PIN_PURGE,          OUTPUT);
    pinMode(PIN_FAN_RELAY,      OUTPUT);
    digitalWriteFast(PIN_FUEL_PUMP_RELAY, LOW);
    digitalWriteFast(PIN_CEL,            LOW);

    // Input pins
    pinMode(PIN_VSS_IN, INPUT_PULLUP);

    comms_init();
    TUNING_SERIAL.println(F("RenixECU v1.0 — booting"));

    // Load config; fall back to defaults if EEPROM invalid
    if (!storage_load(g_cfg)) {
        TUNING_SERIAL.println(F("EEPROM invalid — loading defaults"));
        tables_load_defaults(g_cfg);
        storage_save(g_cfg);
    } else {
        TUNING_SERIAL.println(F("Config loaded from EEPROM"));
    }

    // Zero the ECU state
    memset(&g_state, 0, sizeof(g_state));
    g_state.engine_state = EngineState::OFF;
    g_state.fuel_mode    = FuelMode::CUT;
    g_state.fuel.mode    = InjMode::BATCH;
    g_state.sensors.clt_c = 20;
    g_state.sensors.iat_c = 20;
    g_state.sensors.batt_mv = 12000;

    // Initialise subsystems
    sensors_init();
    crank_init(g_state.crank);
    sched_init();
    diag_init(g_state.diag);
    iac_init(g_state.iac);

    // Prime fuel pump
    digitalWriteFast(PIN_FUEL_PUMP_RELAY, HIGH);

    TUNING_SERIAL.println(F("Init complete. Awaiting crank sync..."));
}

void loop() {
    uint32_t now = millis();

    // ---- 20 ms: Sensor update + fuel calc ------------------
    if ((now - t_sensors) >= 20) {
        t_sensors = now;

        sensors_update(g_state.sensors);
        g_state.sensors.rpm = crank_get_rpm(g_state.crank);

        update_engine_state();
        update_fuel_mode();
        check_rev_limit();
        update_fuel_pump();

        // TPS rate-of-change for accel enrichment
        corrections_accel_update(g_state, (float)g_state.sensors.tps_pct, now, g_cfg);
    }

    // ---- 20 ms: Fuel & ignition scheduling -----------------
    if ((now - t_fuel_calc) >= 20) {
        t_fuel_calc = now;

        if (g_state.fuel_mode == FuelMode::CUT || g_state.rev_limit_active) {
            // Disable all injectors
            for (uint8_t i = 0; i < ENGINE_CYLINDERS; i++)
                sched_remove(EventType::INJ_OPEN, i);
        } else {
            uint32_t base_pw;
            if (g_state.fuel_mode == FuelMode::CRANKING) {
                base_pw = calc_cranking_pw();
                g_state.fuel.stft = 0.0f;  // No CL during cranking
            } else {
                base_pw = fuel_calc_base_pw(g_state.sensors, g_cfg);
            }
            g_state.fuel.pw_us = base_pw;

            float corr = corrections_calc(g_state.sensors, g_state, g_cfg);
            g_state.fuel.final_pw_us = fuel_calc_final_pw(base_pw, corr,
                                                           g_state.sensors, g_cfg);
            fuel_update_dc(g_state.fuel, g_state.sensors.rpm, g_state.fuel.final_pw_us);
            fuel_schedule_events(g_state, g_cfg);
        }

        // Ignition scheduling
        g_state.ign.advance_deg = ign_calc_advance(g_state.sensors, g_cfg);
        g_state.ign.dwell_us    = ign_calc_dwell(g_state.sensors.rpm, g_cfg);
        ign_schedule_events(g_state, g_cfg);
    }

    // ---- 100 ms: Closed loop + IAC -------------------------
    if ((now - t_cl) >= 100) {
        t_cl = now;
        cl_update(g_state, g_cfg);
        iac_update(g_state.iac, g_state.sensors, g_cfg);

        // Simple proportional IAC idle control
        if (g_state.engine_state == EngineState::RUNNING &&
            g_state.sensors.tps_pct < IDLE_TPS_THRESHOLD_PCT) {
            int16_t rpm_err = (int16_t)g_state.iac.target_rpm -
                              (int16_t)g_state.sensors.rpm;
            int16_t step_adj = (int16_t)(rpm_err * IDLE_PROPORTIONAL_GAIN);
            iac_set_target(g_state.iac, g_state.iac.target + step_adj);
        }
    }

    // ---- 200 ms: Diagnostics --------------------------------
    if ((now - t_diag) >= 200) {
        t_diag = now;
        diag_update(g_state.diag, g_state.sensors, g_state.crank, now);
    }

    // ---- 5 ms: Communications ------------------------------
    if ((now - t_comms) >= 5) {
        t_comms = now;
        comms_update(g_state, g_cfg);
    }

    // ---- 1 s: Uptime ---------------------------------------
    if ((now - t_uptime) >= 1000) {
        t_uptime = now;
        g_state.uptime_ms = now;
    }

    // ---- Per-tooth scheduler tick (called from main loop too) ----
    // The scheduler is also driven from the CPS ISR, but we tick it
    // here as a safety net when RPM = 0.
    if (g_state.sensors.rpm == 0) {
        sched_tick(0, 0);
    }
}
