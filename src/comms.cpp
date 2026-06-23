#include "comms.h"
#include "storage.h"
#include "tables.h"
#include "diagnostics.h"
#include <Arduino.h>

// ---- ASCII Serial Protocol ----------------------------------
// Commands (single character + optional argument, terminated by newline):
//   ?        — firmware version
//   S        — dump sensor snapshot
//   F        — dump fuel state
//   I        — dump ignition state
//   D        — dump active faults
//   R        — factory reset (requires confirmation: "RESET\n")
//   V<ve>    — print VE table
//   T<ign>   — print ignition table
//   M        — start 1 Hz monitor stream (send 'X' to stop)
//   X        — stop monitor stream

#define COMMS_MONITOR_INTERVAL_MS  1000
#define COMMS_CMD_BUF_SIZE         16

static char     s_cmd_buf[COMMS_CMD_BUF_SIZE];
static uint8_t  s_cmd_len       = 0;
static bool     s_monitor_on    = false;
static uint32_t s_last_mon_ms   = 0;
static bool     s_reset_pending = false;

static void print_sensors(const SensorData& s) {
    TUNING_SERIAL.print(F("RPM="));   TUNING_SERIAL.print(s.rpm);
    TUNING_SERIAL.print(F(" TPS="));  TUNING_SERIAL.print(s.tps_pct);
    TUNING_SERIAL.print(F("% MAP=")); TUNING_SERIAL.print(s.map_kpa);
    TUNING_SERIAL.print(F("kPa CLT="));TUNING_SERIAL.print(s.clt_c);
    TUNING_SERIAL.print(F("C IAT=")); TUNING_SERIAL.print(s.iat_c);
    TUNING_SERIAL.print(F("C O2="));  TUNING_SERIAL.print(s.o2_mv);
    TUNING_SERIAL.print(F("mV BATT="));TUNING_SERIAL.print(s.batt_mv);
    TUNING_SERIAL.println(F("mV"));
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
    TUNING_SERIAL.print(F("ADV="));   TUNING_SERIAL.print(g.advance_deg);
    TUNING_SERIAL.print(F("deg DWELL="));TUNING_SERIAL.print(g.dwell_us);
    TUNING_SERIAL.println(F("us"));
}

static void handle_command(const char* cmd, ECUState& state, ECUConfig& cfg) {
    char c = cmd[0];
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

    switch (c) {
        case '?':
            TUNING_SERIAL.println(F("RenixECU v1.0 — Teensy 4.1 | Jeep 4.0L I6"));
            break;
        case 'S':
            print_sensors(state.sensors);
            break;
        case 'F':
            print_fuel(state.fuel);
            break;
        case 'I':
            print_ign(state.ign);
            break;
        case 'D':
            diag_print(state.diag);
            break;
        case 'M':
            s_monitor_on = true;
            TUNING_SERIAL.println(F("Monitor ON (send X to stop)"));
            break;
        case 'X':
            s_monitor_on = false;
            TUNING_SERIAL.println(F("Monitor OFF"));
            break;
        case 'R':
            TUNING_SERIAL.println(F("Type RESET to confirm factory reset:"));
            s_reset_pending = true;
            break;
        case 'V':
            TUNING_SERIAL.println(F("VE table (load rows × RPM cols):"));
            for (uint8_t r = 0; r < cfg.ve_table.load_count; r++) {
                TUNING_SERIAL.print(cfg.ve_table.load_bins[r]);
                TUNING_SERIAL.print(F("kPa: "));
                for (uint8_t c2 = 0; c2 < cfg.ve_table.rpm_count; c2++) {
                    TUNING_SERIAL.print(cfg.ve_table.values[r][c2]);
                    TUNING_SERIAL.print(' ');
                }
                TUNING_SERIAL.println();
            }
            break;
        case 'T':
            TUNING_SERIAL.println(F("IGN table (load rows × RPM cols, deg BTDC):"));
            for (uint8_t r = 0; r < cfg.ign_table.load_count; r++) {
                TUNING_SERIAL.print(cfg.ign_table.load_bins[r]);
                TUNING_SERIAL.print(F("kPa: "));
                for (uint8_t c2 = 0; c2 < cfg.ign_table.rpm_count; c2++) {
                    TUNING_SERIAL.print(cfg.ign_table.values[r][c2]);
                    TUNING_SERIAL.print(' ');
                }
                TUNING_SERIAL.println();
            }
            break;
        default:
            TUNING_SERIAL.println(F("? S F I D M X R V T"));
            break;
    }
}

// ---- Public API ---------------------------------------------

void comms_init() {
    TUNING_SERIAL.begin(TUNING_BAUD);
    // Wait up to 1 s for USB serial (desktop only)
    uint32_t t0 = millis();
    while (!TUNING_SERIAL && (millis() - t0) < 1000) {}
}

void comms_update(ECUState& state, ECUConfig& cfg) {
    // Read incoming characters
    while (TUNING_SERIAL.available()) {
        char ch = (char)TUNING_SERIAL.read();
        if (ch == '\r') continue;
        if (ch == '\n') {
            s_cmd_buf[s_cmd_len] = '\0';
            if (s_cmd_len > 0) {
                handle_command(s_cmd_buf, state, cfg);
            }
            s_cmd_len = 0;
        } else if (s_cmd_len < COMMS_CMD_BUF_SIZE - 1) {
            s_cmd_buf[s_cmd_len++] = ch;
        }
    }

    // Monitor stream
    if (s_monitor_on) {
        uint32_t now = millis();
        if ((now - s_last_mon_ms) >= COMMS_MONITOR_INTERVAL_MS) {
            s_last_mon_ms = now;
            print_sensors(state.sensors);
            print_fuel(state.fuel);
            print_ign(state.ign);
            if (state.diag.active_count > 0)
                diag_print(state.diag);
            TUNING_SERIAL.println(F("---"));
        }
    }
}
