#include "datalog.h"
#include "config.h"
#include <SD.h>
#include <Arduino.h>

// Teensy 4.1 has a built-in SD slot on the USDHC bus.
// BUILTIN_SDCARD is the constant for the on-board socket.
// SdFat/SD library auto-selects the high-speed SDIO interface.

static File    s_log;
static bool    s_active  = false;
static uint8_t s_flush_n = 0;

#define FLUSH_ROWS  10   // flush to card every N rows (~1 s at 10 Hz)

bool datalog_init() {
    if (s_active) return true;   // already recording

    if (!SD.begin(BUILTIN_SDCARD)) {
        // No SD card or init failed — not a fatal error
        return false;
    }

    // Find the next free filename LOG0001.CSV … LOG9999.CSV
    char name[16];
    for (uint16_t i = 1; i <= 9999; i++) {
        snprintf(name, sizeof(name), "LOG%04u.CSV", i);
        if (!SD.exists(name)) {
            s_log = SD.open(name, FILE_WRITE);
            break;
        }
    }
    if (!s_log) return false;

    // Header row
    s_log.println(F("ms,rpm,tps,map_kpa,clt_c,iat_c,o2_mv,batt_mv,"
                    "vss_kph,stft,ltft,adv_deg,knk_ret,knk_mv,"
                    "pw_us,dc_pct,fuel_mode,eng_state"));
    s_log.flush();
    s_active  = true;
    s_flush_n = 0;
    return true;
}

void datalog_update(const ECUState& state) {
    if (!s_active || !s_log) return;

    const SensorData& s = state.sensors;

    static const char* FMODE[] = { "CUT","CRANK","OPEN","CLOSED" };
    static const char* ESTATE[] = { "OFF","CRNK","WARM","RUN","OVRN" };

    uint8_t fm = (uint8_t)state.fuel_mode;
    uint8_t es = (uint8_t)state.engine_state;

    char buf[128];
    snprintf(buf, sizeof(buf),
             "%lu,%u,%u,%u,%d,%d,%u,%u,%u,%.1f,%.1f,%u,%u,%u,%lu,%u,%s,%s",
             (unsigned long)millis(),
             s.rpm, s.tps_pct, s.map_kpa,
             s.clt_c, s.iat_c,
             s.o2_mv, s.batt_mv, s.vss_kph,
             (double)state.fuel.stft, (double)state.fuel.ltft,
             state.ign.advance_deg, state.ign.knock_retard, s.knock_mv,
             (unsigned long)state.fuel.final_pw_us, state.fuel.dc_pct,
             FMODE[fm < 4 ? fm : 0],
             ESTATE[es < 5 ? es : 0]);
    s_log.println(buf);

    // Periodic flush — keeps data safe without hammering the card every row
    if (++s_flush_n >= FLUSH_ROWS) {
        s_log.flush();
        s_flush_n = 0;
    }
}

void datalog_stop() {
    if (s_log) {
        s_log.flush();
        s_log.close();
    }
    s_active = false;
}

bool datalog_active() {
    return s_active;
}
