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
#include "knock.h"
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
                float ase_count = table1d_lookup(g_cfg.ase_table, (int16_t)st.sensors.clt_c);
                st.ase_events_left = (uint8_t)(ase_count > 255 ? 255 : ase_count);
            }
            break;
        case EngineState::WARMUP:
            if (st.sensors.clt_c >= 80) {
                st.engine_state = EngineState::RUNNING;
                st.run_start_ms = millis();
            }
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
        st.fuel.mode = InjMode::SEQUENTIAL;
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
            st.fuel_mode = (rpm > 1800) ? FuelMode::CUT : FuelMode::OPEN;
            break;
        default:
            st.fuel_mode = g_cfg.cl_enabled ? FuelMode::CLOSED : FuelMode::OPEN;
            break;
    }

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
    bool pump_on = (g_state.engine_state != EngineState::OFF) ||
                   ((millis() - prime_start) < FUEL_PUMP_PRIME_MS);
    digitalWriteFast(PIN_FUEL_PUMP_RELAY, pump_on ? HIGH : LOW);
}

// ---- O2 Heater Relay ----------------------------------------
// Enable after O2_HEATER_DELAY_MS from engine start, or when CLT > threshold.
// Fault if heater has been on for >120 s and O2 still reads inactive.
static void update_o2_heater(uint32_t now_ms) {
    ECUState& st = g_state;

    if (st.engine_state == EngineState::OFF || st.engine_state == EngineState::CRANKING) {
        if (st.o2_heater_on) {
            digitalWriteFast(PIN_O2_HEATER, LOW);
            st.o2_heater_on = false;
        }
        return;
    }

    bool should_enable = ((now_ms - st.run_start_ms) >= O2_HEATER_DELAY_MS) ||
                         (st.sensors.clt_c >= O2_HEATER_CLT_ENABLE_C);

    if (should_enable && !st.o2_heater_on) {
        digitalWriteFast(PIN_O2_HEATER, HIGH);
        st.o2_heater_on = true;
    }

    // Fault: heater on 120 s but O2 sensor still not reading
    if (st.o2_heater_on && st.diag.active[(uint8_t)FaultCode::O2_INACTIVE]) {
        uint32_t heater_on_ms = now_ms - (st.run_start_ms +
            (O2_HEATER_DELAY_MS > 0 ? O2_HEATER_DELAY_MS : 0));
        if (heater_on_ms > 120000UL)
            diag_set(st.diag, FaultCode::O2_HEATER_FAULT);
    } else {
        diag_clear(st.diag, FaultCode::O2_HEATER_FAULT);
    }
}

// ---- A/C Compressor Control ---------------------------------
static void update_ac(uint32_t now_ms) {
    ECUState& st = g_state;
    static uint32_t request_start_ms = 0;
    static bool request_was_active   = false;

    bool engine_ok = (st.engine_state == EngineState::RUNNING ||
                      st.engine_state == EngineState::WARMUP);

    // Inhibit A/C at WOT or high speed
    bool inhibit = (st.sensors.tps_pct >= AC_WOT_CUTOFF_TPS_PCT) ||
                   (st.sensors.vss_kph  >= AC_SPEED_CUTOFF_KPH);

    bool request = st.sensors.ac_request && engine_ok && !inhibit;

    if (request && !request_was_active) {
        request_start_ms   = now_ms;
        request_was_active = true;
    } else if (!request) {
        request_was_active = false;
    }

    // Engage clutch only after delay (IAC has time to pre-open)
    bool engage = request_was_active &&
                  ((now_ms - request_start_ms) >= AC_ENGAGE_DELAY_MS);

    if (engage != st.ac_active) {
        st.ac_active = engage;
        digitalWriteFast(PIN_AC_CLUTCH, engage ? HIGH : LOW);
    }
}

// ---- Upshift Light (manual transmission only) ---------------
static void update_upshift_light() {
    ECUState& st = g_state;
    uint16_t rpm = st.sensors.rpm;
    bool light = (rpm >= UPSHIFT_MIN_RPM) &&
                 (rpm <= UPSHIFT_MAX_RPM) &&
                 (st.sensors.tps_pct >= UPSHIFT_MIN_TPS_PCT);
    digitalWriteFast(PIN_UPSHIFT_LIGHT, light ? HIGH : LOW);
}

// ---- Latch Relay (self-hold post key-off) -------------------
// Keeps ECU powered after key-off for IAC park and LTFT save.
// Releases the relay after SHUTDOWN_HOLD_MS.
static bool s_shutdown_started = false;
static uint32_t s_shutdown_start_ms = 0;

static void update_latch_relay(uint32_t now_ms) {
    ECUState& st = g_state;

    if (st.sensors.ign_sw) {
        // Key is on — hold latch active, reset shutdown tracking
        if (!st.latch_relay_on) {
            digitalWriteFast(PIN_LATCH_RELAY, HIGH);
            st.latch_relay_on = true;
        }
        s_shutdown_started = false;
        return;
    }

    // Key is off
    if (!s_shutdown_started) {
        s_shutdown_started   = true;
        s_shutdown_start_ms  = now_ms;
        st.key_off_ms        = now_ms;

        // Initiate shutdown sequence
        iac_park(st.iac);         // Command IAC to parked position
        cl_save_ltft(g_state, g_cfg);   // Persist LTFT to EEPROM
    }

    // Hold power until IAC has moved to park and hold time has elapsed
    uint32_t held_ms = now_ms - s_shutdown_start_ms;
    if (held_ms >= SHUTDOWN_HOLD_MS) {
        // Release self-hold — power will drop immediately after this
        digitalWriteFast(PIN_LATCH_RELAY, LOW);
        st.latch_relay_on = false;
    }
}

// ---- Cranking Fuel Calculation ------------------------------
static uint32_t calc_cranking_pw() {
    float base = table1d_lookup(g_cfg.crank_enrich, (int16_t)g_state.sensors.clt_c);
    float flow_g_per_us = (g_cfg.injector_flow_cc_min * ENGINE_FUEL_DENSITY_G_CC)
                          / 60000000.0f;
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
    pinMode(PIN_CEL,             OUTPUT);
    pinMode(PIN_TACH_OUT,        OUTPUT);
    pinMode(PIN_EGR,             OUTPUT);
    pinMode(PIN_PURGE,           OUTPUT);
    pinMode(PIN_FAN_RELAY,       OUTPUT);
    pinMode(PIN_AC_CLUTCH,       OUTPUT);
    pinMode(PIN_O2_HEATER,       OUTPUT);
    pinMode(PIN_LATCH_RELAY,     OUTPUT);
    pinMode(PIN_UPSHIFT_LIGHT,   OUTPUT);

    // Safe initial state — all outputs off
    digitalWriteFast(PIN_FUEL_PUMP_RELAY, LOW);
    digitalWriteFast(PIN_CEL,             LOW);
    digitalWriteFast(PIN_AC_CLUTCH,       LOW);
    digitalWriteFast(PIN_O2_HEATER,       LOW);
    digitalWriteFast(PIN_UPSHIFT_LIGHT,   LOW);

    // Assert latch relay immediately so ECU holds its own power
    digitalWriteFast(PIN_LATCH_RELAY, HIGH);

    comms_init();
    TUNING_SERIAL.println(F("RenixECU v1.0 — booting"));

    bool cfg_loaded = storage_load(g_cfg);
    if (!cfg_loaded) {
        TUNING_SERIAL.println(F("EEPROM invalid — loading defaults"));
        tables_load_defaults(g_cfg);
        storage_save(g_cfg);
    } else {
        TUNING_SERIAL.println(F("Config loaded from EEPROM"));
    }

    memset(&g_state, 0, sizeof(g_state));
    g_state.engine_state    = EngineState::OFF;
    g_state.fuel_mode       = FuelMode::CUT;
    g_state.fuel.mode       = InjMode::BATCH;
    g_state.sensors.clt_c   = 20;
    g_state.sensors.iat_c   = 20;
    g_state.sensors.batt_mv = 12000;
    g_state.latch_relay_on  = true;

    // Restore LTFT from previous run (after memset so it isn't clobbered)
    if (cfg_loaded)
        cl_restore_ltft(g_state.fuel, g_cfg);

    sensors_init();          // Attaches VSS interrupt, configures discrete input pins
    crank_init(g_state.crank);
    sched_init();
    diag_init(g_state.diag);
    iac_init(g_state.iac);
    knock_init();            // Starts 25 kHz IntervalTimer

    digitalWriteFast(PIN_FUEL_PUMP_RELAY, HIGH);

    TUNING_SERIAL.println(F("Init complete. Awaiting crank sync..."));
}

void loop() {
    uint32_t now = millis();

    // ---- 20 ms: Sensor update + fuel calc ------------------
    if ((now - t_sensors) >= 20) {
        t_sensors = now;

        sensors_update(g_state.sensors);
        sensors_read_discrete(g_state.sensors);
        g_state.sensors.rpm = crank_get_rpm(g_state.crank);

        update_engine_state();
        update_fuel_mode();
        check_rev_limit();
        update_fuel_pump();
        update_latch_relay(now);
        update_o2_heater(now);
        update_ac(now);
        update_upshift_light();

        corrections_accel_update(g_state, (float)g_state.sensors.tps_pct, now, g_cfg);

        // Knock: sample new peak and apply/recover retard
        knock_update(g_state.ign, g_state.diag, now);
        knock_recover(g_state.ign, g_state.diag, now);

        // Capture knock peak for comms monitor
        g_state.sensors.knock_mv = knock_get_peak_mv();
    }

    // ---- 20 ms: Fuel & ignition scheduling -----------------
    if ((now - t_fuel_calc) >= 20) {
        t_fuel_calc = now;

        if (g_state.fuel_mode == FuelMode::CUT || g_state.rev_limit_active) {
            for (uint8_t i = 0; i < ENGINE_CYLINDERS; i++)
                sched_remove(EventType::INJ_OPEN, i);
        } else {
            uint32_t base_pw;
            if (g_state.fuel_mode == FuelMode::CRANKING) {
                base_pw = calc_cranking_pw();
                g_state.fuel.stft = 0.0f;
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

        // Knock retard is already applied inside ign_calc_advance
        g_state.ign.advance_deg = ign_calc_advance(g_state.sensors, g_cfg,
                                                    g_state.ign.knock_retard);
        g_state.ign.dwell_us    = ign_calc_dwell(g_state.sensors.rpm, g_cfg);
        ign_schedule_events(g_state, g_cfg);
    }

    // ---- 100 ms: Closed loop + IAC -------------------------
    if ((now - t_cl) >= 100) {
        t_cl = now;
        cl_update(g_state, g_cfg);
        iac_update(g_state.iac, g_state.sensors, g_cfg);

        if (g_state.engine_state == EngineState::RUNNING &&
            g_state.sensors.tps_pct < IDLE_TPS_THRESHOLD_PCT) {

            int16_t rpm_err  = (int16_t)g_state.iac.target_rpm -
                               (int16_t)g_state.sensors.rpm;
            int16_t step_adj = (int16_t)(rpm_err * IDLE_PROPORTIONAL_GAIN);
            iac_set_target(g_state.iac, g_state.iac.target + step_adj);

            // Apply accessory load bumps on top of P-controller target
            bool in_drive = !g_state.sensors.park_neutral;
            iac_apply_idle_compensation(g_state.iac, g_state.ac_active,
                                        g_state.sensors.ps_pressure, in_drive);
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

    // ---- Safety net scheduler tick at RPM=0 ----------------
    if (g_state.sensors.rpm == 0) {
        sched_tick(0, 0);
    }
}
