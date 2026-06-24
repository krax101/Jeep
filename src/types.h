#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "config.h"

// ---- Engine State Machine -----------------------------------
enum class EngineState : uint8_t {
    OFF       = 0,
    CRANKING  = 1,
    WARMUP    = 2,
    RUNNING   = 3,
    OVERRUN   = 4
};

enum class FuelMode : uint8_t {
    CUT      = 0,
    CRANKING = 1,
    OPEN     = 2,
    CLOSED   = 3
};

enum class InjMode : uint8_t {
    BATCH      = 0,   // 2 banks of 3, every 360° (no cam sync)
    SEQUENTIAL = 1    // 1 injector per cylinder, every 720°
};

// ---- Live Sensor Data ---------------------------------------
struct SensorData {
    uint16_t rpm;           // Engine speed
    uint8_t  tps_pct;       // Throttle position 0–100
    uint8_t  map_kpa;       // Manifold absolute pressure kPa
    int8_t   clt_c;         // Coolant temperature °C
    int8_t   iat_c;         // Intake air temperature °C
    uint16_t o2_mv;         // O2 sensor millivolts
    uint16_t batt_mv;       // Battery voltage millivolts
    uint8_t  vss_kph;       // Vehicle speed km/h
    uint16_t knock_mv;      // Knock sensor AC amplitude (peak since last read, mV)
    bool     ign_sw;        // Ignition switch state (key-on)
    bool     start_signal;  // Starter engagement (HIGH = cranking)
    bool     park_neutral;  // P/N switch: true = P or N position
    bool     ac_request;    // A/C thermostat/switch requesting compressor
    bool     ps_pressure;   // Power steering pressure switch (HIGH = load)
    bool     clt_fault;
    bool     iat_fault;
    bool     tps_fault;
    bool     map_fault;
};

// ---- Fuel & Injection Output --------------------------------
struct FuelState {
    uint32_t pw_us;         // Base injector pulse width microseconds
    uint32_t final_pw_us;   // Pulse width after all corrections
    uint8_t  dc_pct;        // Current duty cycle %
    float    stft;          // Short-term fuel trim %
    float    ltft;          // Long-term fuel trim %
    float    total_corr;    // Combined multiplier
    InjMode  mode;
};

// ---- Ignition Output ----------------------------------------
struct IgnState {
    uint8_t  advance_deg;   // Current ignition advance °BTDC (before knock retard)
    uint8_t  knock_retard;  // Active knock retard (subtracted from advance_deg)
    uint16_t dwell_us;      // Coil charge time microseconds
};

// ---- Crank Trigger State ------------------------------------
struct CrankState {
    volatile uint32_t tooth_times[40]; // Circular buffer of tooth timestamps (μs)
    volatile uint8_t  tooth_head;      // Next write index
    volatile uint8_t  tooth_count;     // Teeth since last sync
    volatile bool     synced;          // Have we seen the gap?
    volatile bool     cam_synced;      // Full 720° reference acquired
    volatile uint8_t  revolution;      // 0=first half, 1=second half of 720°
    volatile uint32_t last_tooth_us;
    volatile uint32_t last_cam_us;      // Timestamp of last cam pulse (for CAM_LOSS detection)
    volatile uint32_t tooth_period_us; // Period of last tooth
    volatile uint32_t rpm_filtered;
    volatile uint16_t crank_angle_x10; // Current angle × 10 (0–3599)
    volatile uint16_t angle_720_x10;   // Current angle in 720° frame × 10
};

// ---- IAC Stepper State --------------------------------------
struct IACState {
    int16_t  position;      // Current step position (0 = fully closed)
    int16_t  target;        // Target step position
    uint16_t target_rpm;    // Current idle RPM target
    uint32_t last_step_ms;
    uint8_t  phase;         // Current step phase (0–3)
};

// ---- Scheduled Engine Event ---------------------------------
// Events are angle-triggered: the scheduler fires them when
// crank_angle_x10 passes event.angle_x10.
enum class EventType : uint8_t {
    INJ_OPEN  = 0,
    INJ_CLOSE = 1,
    IGN_DWELL = 2,
    IGN_FIRE  = 3
};

struct EngineEvent {
    EventType type;
    uint8_t   channel;        // Injector or coil number (0-based)
    uint16_t  angle_720_x10;  // Trigger angle in 720° frame × 10
    uint32_t  delay_us;       // Optional: additional delay after angle trigger
    bool      active;
};

// ---- Diagnostic Fault Codes ---------------------------------
enum class FaultCode : uint8_t {
    NONE           = 0,
    CLT_HIGH       = 1,
    CLT_LOW        = 2,
    IAT_HIGH       = 3,
    IAT_LOW        = 4,
    TPS_HIGH       = 5,
    TPS_LOW        = 6,
    MAP_HIGH       = 7,
    MAP_LOW        = 8,
    O2_INACTIVE    = 9,
    CPS_LOSS       = 10,
    CAM_LOSS       = 11,
    INJ_OC         = 12,
    KNOCK          = 13,
    BATT_HIGH      = 14,
    BATT_LOW       = 15,
    O2_HEATER_FAULT= 16,
    MAX_CODES      = 17
};

struct DiagState {
    bool     active[(uint8_t)FaultCode::MAX_CODES];
    uint32_t first_set_ms[(uint8_t)FaultCode::MAX_CODES];
    bool     cel_on;
    uint8_t  active_count;
};

// ---- 1D Lookup Table ----------------------------------------
struct Table1D {
    uint8_t  count;
    int16_t  x[16];
    int16_t  y[16];
};

// ---- 3D (2-axis) Lookup Table  -----------------------------
// values[load_idx][rpm_idx]
struct Table3D {
    uint8_t  rpm_count;
    uint8_t  load_count;
    uint16_t rpm_bins[16];
    uint8_t  load_bins[16];  // kPa
    uint8_t  values[16][16];
};

// ---- Master ECU Config (saved to EEPROM) --------------------
#define CFG_MAGIC  0xECU4040UL
#define CFG_VER    1

struct ECUConfig {
    uint32_t magic;
    uint16_t version;

    // Trigger
    uint8_t  trigger_teeth;
    uint8_t  trigger_missing;
    uint8_t  trigger_mode;
    uint8_t  trigger_sync_btdc;

    // Injectors
    float    injector_flow_cc_min;
    uint16_t injector_dead_time_14v_us;

    // Rev limit
    uint16_t rev_limit_rpm;

    // Idle
    uint16_t idle_target_rpm;

    // Fuel tables
    Table3D  ve_table;
    Table1D  clt_wue;       // Warm-up enrichment %
    Table1D  iat_corr;      // IAT correction %
    Table1D  crank_enrich;  // Cranking enrichment % vs CLT
    Table1D  ase_table;     // After-start enrichment vs CLT (count of events)
    Table1D  dead_time;     // Injector dead time (μs) vs battery voltage (×100mV)
    Table1D  idle_vs_clt;   // Target idle RPM vs CLT

    // Ignition table
    Table3D  ign_table;
    Table1D  dwell_vs_rpm;

    // Closed loop
    bool     cl_enabled;
    float    cl_max_stft;
    float    cl_max_ltft;

    // Accel enrichment
    float    ae_tps_threshold;
    float    ae_multiplier;

    // Persistent closed-loop trim (restored on boot)
    float    saved_ltft;

    uint16_t checksum;
};

// ---- Global ECU State (shared, not saved) -------------------
struct ECUState {
    EngineState engine_state;
    FuelMode    fuel_mode;
    SensorData  sensors;
    FuelState   fuel;
    IgnState    ign;
    CrankState  crank;
    IACState    iac;
    DiagState   diag;

    uint32_t    uptime_ms;
    bool        rev_limit_active;

    float       ae_amount;      // Accel enrichment remaining multiplier
    uint8_t     ae_teeth_left;  // Teeth remaining in AE window
    float       tps_last;       // Previous TPS for rate-of-change
    uint32_t    tps_last_ms;

    uint8_t     ase_events_left; // After-start enrichment event count

    // Auxiliary output states
    bool        ac_active;       // A/C compressor clutch commanded on
    bool        o2_heater_on;    // O2 heater relay active
    bool        latch_relay_on;  // ECU self-hold relay active

    // Timing references
    uint32_t    run_start_ms;    // millis() when engine entered RUNNING state
    uint32_t    key_off_ms;      // millis() when ignition switch went LOW
};
