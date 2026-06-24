#include "comms.h"
#include "storage.h"
#include "tables.h"
#include "diagnostics.h"
#include <Arduino.h>

// ---- ASCII Serial Protocol ----------------------------------
// Commands (terminated by newline):
//   ?           — firmware version + command list
//   S           — sensor snapshot (RPM / TPS / MAP / CLT / IAT / O2 / BATT / VSS / KNOCK)
//   F           — fuel state (PW / DC / STFT / LTFT / CORR / MODE)
//   I           — ignition state (ADV / DWELL / KNOCK_RETARD)
//   D           — active fault codes
//   V           — print VE table
//   T           — print ignition table
//   M           — start monitor stream at 1 Hz (send X to stop)
//   X           — stop monitor stream
//   WV RPM MAP VAL  — write VE cell nearest to RPM/MAP
//   WI RPM MAP VAL  — write IGN cell nearest to RPM/MAP
//   L           — toggle SD data logging (if available)
//   R           — factory reset (requires confirmation: "RESET\n")

#define COMMS_MONITOR_INTERVAL_MS  1000
#define COMMS_CMD_BUF_SIZE         32

static char     s_cmd_buf[COMMS_CMD_BUF_SIZE];
static uint8_t  s_cmd_len       = 0;
static bool     s_monitor_on    = false;
static uint32_t s_last_mon_ms   = 0;
static bool     s_reset_pending = false;

// ---- Print helpers ------------------------------------------

static void print_sensors(const ECUState& st) {
    const SensorData& s = st.sensors;
    TUNING_SERIAL.print(F("RPM="));    TUNING_SERIAL.print(s.rpm);
    TUNING_SERIAL.print(F(" TPS="));   TUNING_SERIAL.print(s.tps_pct);
    TUNING_SERIAL.print(F("% MAP="));  TUNING_SERIAL.print(s.map_kpa);
    TUNING_SERIAL.print(F("kPa CLT="));TUNING_SERIAL.print(s.clt_c);
    TUNING_SERIAL.print(F("C IAT="));  TUNING_SERIAL.print(s.iat_c);
    TUNING_SERIAL.print(F("C O2="));   TUNING_SERIAL.print(s.o2_mv);
    TUNING_SERIAL.print(F("mV BATT=")); TUNING_SERIAL.print(s.batt_mv);
    TUNING_SERIAL.print(F("mV VSS=")); TUNING_SERIAL.print(s.vss_kph);
    TUNING_SERIAL.print(F("kph KNK=")); TUNING_SERIAL.print(s.knock_mv);
    TUNING_SERIAL.print(F("mV ENG="));
    static const char* ESTATE[] = {"OFF","CRNK","WARM","RUN","OVRN"};
    TUNING_SERIAL.println(ESTATE[(uint8_t)st.engine_state < 5 ? (uint8_t)st.engine_state : 0]);
}

static void print_fuel(const FuelState& f) {
    TUNING_SERIAL.print(F("PW="));    TUNING_SERIAL.print(f.final_pw_us);
    TUNING_SERIAL.print(F("us DC=")); TUNING_SERIAL.print(f.dc_pct);
    TUNING_SERIAL.print(F("% STFT="));TUNING_SERIAL.print(f.stft, 1);
    TUNING_SERIAL.print(F("% LTFT="));TUNING_SERIAL.print(f.ltft, 1);
    TUNING_SERIAL.print(F("% CORR="));TUNING_SERIAL.print(f.total_corr, 3);
    TUNING_SERIAL.print(F(" MODE="));
    TUNING_SERIAL.println(f.mode == InjMode::SEQUENTIAL ? F("SEQ") : F("BATCH"));
}

static void print_ign(const IgnState& g) {
    TUNING_SERIAL.print(F("ADV="));     TUNING_SERIAL.print(g.advance_deg);
    TUNING_SERIAL.print(F("deg KNK_RET=")); TUNING_SERIAL.print(g.knock_retard);
    TUNING_SERIAL.print(F("deg DWELL=")); TUNING_SERIAL.print(g.dwell_us);
    TUNING_SERIAL.println(F("us"));
}

static void print_table(const Table3D& t, const char* name) {
    TUNING_SERIAL.print(name);
    TUNING_SERIAL.println(F(" (load rows x RPM cols):"));
    TUNING_SERIAL.print(F("     "));
    for (uint8_t c2 = 0; c2 < t.rpm_count; c2++) {
        TUNING_SERIAL.print(t.rpm_bins[c2]);
        TUNING_SERIAL.print(' ');
    }
    TUNING_SERIAL.println();
    for (uint8_t r = 0; r < t.load_count; r++) {
        TUNING_SERIAL.print(t.load_bins[r]);
        TUNING_SERIAL.print(F(": "));
        for (uint8_t c2 = 0; c2 < t.rpm_count; c2++) {
            TUNING_SERIAL.print(t.values[r][c2]);
            TUNING_SERIAL.print(' ');
        }
        TUNING_SERIAL.println();
    }
}

// ---- Table cell writer --------------------------------------
// Finds nearest bin indices and sets cell value; saves to EEPROM.
static void write_table_cell(Table3D& t, uint16_t rpm, uint8_t map_kpa,
                              uint8_t value, ECUConfig& cfg) {
    // Find nearest RPM bin
    uint8_t xi = 0;
    uint16_t best_rpm_diff = 0xFFFF;
    for (uint8_t i = 0; i < t.rpm_count; i++) {
        uint16_t diff = (uint16_t)abs((int32_t)t.rpm_bins[i] - (int32_t)rpm);
        if (diff < best_rpm_diff) { best_rpm_diff = diff; xi = i; }
    }
    // Find nearest MAP bin
    uint8_t yi = 0;
    uint8_t best_map_diff = 0xFF;
    for (uint8_t i = 0; i < t.load_count; i++) {
        uint8_t diff = (uint8_t)abs((int16_t)t.load_bins[i] - (int16_t)map_kpa);
        if (diff < best_map_diff) { best_map_diff = diff; yi = i; }
    }
    t.values[yi][xi] = value;
    storage_save(cfg);
    TUNING_SERIAL.print(F("SET ["));
    TUNING_SERIAL.print(t.load_bins[yi]);
    TUNING_SERIAL.print(F("kPa]["));
    TUNING_SERIAL.print(t.rpm_bins[xi]);
    TUNING_SERIAL.print(F("rpm]="));
    TUNING_SERIAL.println(value);
}

// ---- Command handler ----------------------------------------

static bool s_logging_enabled = false;  // toggled by 'L' command

static void handle_command(const char* cmd, ECUState& state, ECUConfig& cfg) {
    if (s_reset_pending) {
        if (strncmp(cmd, "RESET", 5) == 0) {
            storage_factory_reset(cfg);
            TUNING_SERIAL.println(F("Factory reset complete."));
        } else {
            TUNING_SERIAL.println(F("Reset cancelled."));
        }
        s_reset_pending = false;
        return;
    }

    char c = cmd[0];
    switch (c) {
        case '?':
            TUNING_SERIAL.println(F("RenixECU v1.0 — Teensy 4.1 | Jeep 4.0L I6"));
            TUNING_SERIAL.println(F("Cmds: ? S F I D V T M X WV WI L R"));
            break;
        case 'S': print_sensors(state);       break;
        case 'F': print_fuel(state.fuel);     break;
        case 'I': print_ign(state.ign);       break;
        case 'D': diag_print(state.diag);     break;
        case 'M':
            s_monitor_on = true;
            TUNING_SERIAL.println(F("Monitor ON (X to stop)"));
            break;
        case 'X':
            s_monitor_on = false;
            TUNING_SERIAL.println(F("Monitor OFF"));
            break;
        case 'V': print_table(cfg.ve_table,  "VE");  break;
        case 'T': print_table(cfg.ign_table, "IGN"); break;
        case 'W': {
            // WV RPM MAP VAL  or  WI RPM MAP VAL
            if (cmd[1] != 'V' && cmd[1] != 'I') { TUNING_SERIAL.println(F("WV or WI")); break; }
            uint16_t rpm = 0; uint8_t map = 0, val = 0;
            int n = sscanf(cmd + 3, "%u %hhu %hhu", &rpm, &map, &val);
            if (n != 3) { TUNING_SERIAL.println(F("Usage: WV RPM MAP VAL")); break; }
            Table3D& tbl = (cmd[1] == 'V') ? cfg.ve_table : cfg.ign_table;
            write_table_cell(tbl, rpm, map, val, cfg);
            break;
        }
        case 'L':
            s_logging_enabled = !s_logging_enabled;
            TUNING_SERIAL.print(F("DataLog: "));
            TUNING_SERIAL.println(s_logging_enabled ? F("ON") : F("OFF"));
            break;
        case 'R':
            TUNING_SERIAL.println(F("Type RESET to confirm factory reset:"));
            s_reset_pending = true;
            break;
        default:
            TUNING_SERIAL.println(F("? S F I D V T M X WV WI L R"));
            break;
    }
}

// ---- Public API ---------------------------------------------

void comms_init() {
    TUNING_SERIAL.begin(TUNING_BAUD);
    uint32_t t0 = millis();
    while (!TUNING_SERIAL && (millis() - t0) < 1000) {}
}

bool comms_logging_requested() {
    return s_logging_enabled;
}

void comms_update(ECUState& state, ECUConfig& cfg) {
    while (TUNING_SERIAL.available()) {
        char ch = (char)TUNING_SERIAL.read();
        if (ch == '\r') continue;
        if (ch == '\n') {
            s_cmd_buf[s_cmd_len] = '\0';
            if (s_cmd_len > 0)
                handle_command(s_cmd_buf, state, cfg);
            s_cmd_len = 0;
        } else if (s_cmd_len < COMMS_CMD_BUF_SIZE - 1) {
            s_cmd_buf[s_cmd_len++] = ch;
        }
    }

    if (s_monitor_on) {
        uint32_t now = millis();
        if ((now - s_last_mon_ms) >= COMMS_MONITOR_INTERVAL_MS) {
            s_last_mon_ms = now;
            print_sensors(state);
            print_fuel(state.fuel);
            print_ign(state.ign);
            if (state.diag.active_count > 0)
                diag_print(state.diag);
            TUNING_SERIAL.println(F("---"));
        }
    }
}
