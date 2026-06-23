#include "dash.h"
#include "config.h"
#include "diagnostics.h"
#include <ILI9341_t3.h>
#include <Arduino.h>
#include <stdio.h>

// SPI2 detected automatically when MOSI=43, SCK=42 are passed (Teensyduino 1.57+)
static ILI9341_t3 s_tft(PIN_DASH_CS, PIN_DASH_DC, PIN_DASH_RST,
                         PIN_DASH_MOSI, PIN_DASH_SCK);

// ---- Layout (320×240 landscape) --------------------------------
// y 0–67   : RPM number + bar
// y 68     : divider
// y 69–148 : 4 gauges (CLT / MAP / VSS / BATT), each 80px wide
// y 149    : divider
// y 150–239: status (fuel mode, trims, knock, faults)

static const uint16_t BAR_X  =  8;
static const uint16_t BAR_Y  = 48;
static const uint16_t BAR_W  = 272;  // leaves room for "RPM" label on right
static const uint16_t BAR_H  = 12;
static const uint16_t DIV1_Y =  68;
static const uint16_t G_Y    =  70;  // gauge section top
static const uint16_t G_H    =  78;  // gauge section height
static const uint16_t DIV2_Y = 149;
static const uint16_t S_Y    = 152;  // status section top

// ---- Dirty-check shadow values --------------------------------
static uint16_t s_rpm   = 0xFFFF;
static int8_t   s_clt   = -99;
static uint8_t  s_map   = 0xFF;
static uint8_t  s_vss   = 0xFF;
static uint16_t s_batt  = 0xFFFF;
static uint8_t  s_mode  = 0xFF;
static int8_t   s_stft  = -99;
static int8_t   s_ltft  = -99;
static uint8_t  s_knock = 0xFF;
static uint8_t  s_flt   = 0xFF;

// ---- Helpers --------------------------------------------------

static uint16_t rpm_color(uint16_t rpm) {
    if (rpm > (uint16_t)(REV_LIMIT_HARD_RPM * 85 / 100)) return ILI9341_RED;
    if (rpm > (uint16_t)(REV_LIMIT_HARD_RPM * 70 / 100)) return ILI9341_YELLOW;
    return ILI9341_GREEN;
}

static void label(uint16_t x, uint16_t y, const char* txt,
                  uint16_t fg = 0x7BEF /* DARKGREY */) {
    s_tft.setTextSize(1);
    s_tft.setTextColor(fg, ILI9341_BLACK);
    s_tft.setCursor(x, y);
    s_tft.print(txt);
}

static void value_block(uint16_t x, uint16_t y, uint16_t w, uint16_t h,
                        const char* txt, uint16_t fg, uint8_t tsz = 3) {
    s_tft.fillRect(x, y, w, h, ILI9341_BLACK);
    s_tft.setTextSize(tsz);
    s_tft.setTextColor(fg, ILI9341_BLACK);
    uint16_t tw = (uint16_t)strlen(txt) * 6u * tsz;
    s_tft.setCursor(x + (w > tw ? (w - tw) / 2 : 0), y);
    s_tft.print(txt);
}

// ---- Init -----------------------------------------------------

void dash_init() {
    s_tft.begin();
    s_tft.setRotation(1);
    s_tft.fillScreen(ILI9341_BLACK);

    // Static divider lines
    s_tft.drawFastHLine(0, DIV1_Y, 320, 0x7BEF);
    s_tft.drawFastHLine(0, DIV2_Y, 320, 0x7BEF);

    // Gauge column dividers
    for (uint8_t i = 1; i < 4; i++)
        s_tft.drawFastVLine((uint16_t)i * 80, G_Y, G_H, 0x7BEF);

    // Gauge labels
    label( 28, G_Y + 3, "CLT");
    label(108, G_Y + 3, "MAP");
    label(188, G_Y + 3, "VSS");
    label(268, G_Y + 3, "BATT");

    // Gauge units
    label( 36, G_Y + 62, "C");
    label(105, G_Y + 62, "kPa");
    label(185, G_Y + 62, "kph");
    label(268, G_Y + 62, "V");

    // RPM bar frame
    s_tft.drawRect(BAR_X - 1, BAR_Y - 1, BAR_W + 2, BAR_H + 2, 0x7BEF);
    label(288, BAR_Y + 2, "RPM", ILI9341_LIGHTGREY);

    // Force full redraw on first call
    s_rpm  = 0xFFFF;
    s_mode = 0xFF;
    s_flt  = 0xFF;
}

// ---- Update ---------------------------------------------------

void dash_update(const ECUState& state, const ECUConfig& cfg) {
    (void)cfg;
    const SensorData& sen = state.sensors;

    // ---- RPM --------------------------------------------------
    if (sen.rpm != s_rpm) {
        char buf[6];
        snprintf(buf, sizeof(buf), "%4u", sen.rpm);
        s_tft.fillRect(0, 0, 280, 44, ILI9341_BLACK);
        s_tft.setTextSize(5);
        s_tft.setTextColor(rpm_color(sen.rpm), ILI9341_BLACK);
        s_tft.setCursor(8, 2);
        s_tft.print(buf);

        // Bar: fill then erase tail
        uint32_t fill = ((uint32_t)sen.rpm * BAR_W) /
                        (uint32_t)REV_LIMIT_HARD_RPM;
        if (fill > BAR_W) fill = BAR_W;
        s_tft.fillRect(BAR_X, BAR_Y, (uint16_t)fill, BAR_H, rpm_color(sen.rpm));
        if (fill < BAR_W)
            s_tft.fillRect(BAR_X + (uint16_t)fill, BAR_Y,
                           BAR_W - (uint16_t)fill, BAR_H, ILI9341_BLACK);
        s_rpm = sen.rpm;
    }

    // ---- CLT --------------------------------------------------
    if (sen.clt_c != s_clt) {
        char buf[5];
        snprintf(buf, sizeof(buf), "%3d", sen.clt_c);
        uint16_t col = ILI9341_WHITE;
        if (sen.clt_c > 105) col = ILI9341_RED;
        else if (sen.clt_c > 90) col = ILI9341_YELLOW;
        value_block(2, G_Y + 16, 76, 28, buf, col);
        s_clt = sen.clt_c;
    }

    // ---- MAP --------------------------------------------------
    if (sen.map_kpa != s_map) {
        char buf[5];
        snprintf(buf, sizeof(buf), "%3u", sen.map_kpa);
        value_block(82, G_Y + 16, 76, 28, buf, ILI9341_CYAN);
        s_map = sen.map_kpa;
    }

    // ---- VSS --------------------------------------------------
    if (sen.vss_kph != s_vss) {
        char buf[5];
        snprintf(buf, sizeof(buf), "%3u", sen.vss_kph);
        value_block(162, G_Y + 16, 76, 28, buf, ILI9341_WHITE);
        s_vss = sen.vss_kph;
    }

    // ---- BATT -------------------------------------------------
    if (sen.batt_mv != s_batt) {
        char buf[6];
        uint16_t dv = sen.batt_mv / 100;   // decivolts (e.g. 138 → "13.8")
        snprintf(buf, sizeof(buf), "%u.%u", dv / 10, dv % 10);
        uint16_t col = (sen.batt_mv < 11500 || sen.batt_mv > 15000)
                       ? ILI9341_YELLOW : ILI9341_WHITE;
        value_block(242, G_Y + 16, 76, 28, buf, col, 2);
        s_batt = sen.batt_mv;
    }

    // ---- Status: fuel mode + trims ----------------------------
    uint8_t mode_idx = (uint8_t)state.fuel_mode;
    int8_t  stft_i   = (int8_t)(state.fuel.stft);
    int8_t  ltft_i   = (int8_t)(state.fuel.ltft);

    if (mode_idx != s_mode || stft_i != s_stft || ltft_i != s_ltft) {
        static const char* MODE_NAMES[4] = { "CUT", "CRANK", "OPEN", "CLS" };
        s_tft.fillRect(0, S_Y, 320, 18, ILI9341_BLACK);
        s_tft.setTextSize(2);
        s_tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
        s_tft.setCursor(4, S_Y);
        s_tft.print(MODE_NAMES[mode_idx < 4 ? mode_idx : 0]);
        char buf[28];
        snprintf(buf, sizeof(buf), "  ST:%+d%%  LT:%+d%%", stft_i, ltft_i);
        s_tft.print(buf);
        s_mode = mode_idx;
        s_stft = stft_i;
        s_ltft = ltft_i;
    }

    // ---- Status: knock retard ---------------------------------
    if (state.ign.knock_retard != s_knock) {
        s_tft.fillRect(0, S_Y + 20, 320, 18, ILI9341_BLACK);
        if (state.ign.knock_retard > 0) {
            uint16_t col = (state.ign.knock_retard >= (uint8_t)KNOCK_RETARD_MAX_DEG)
                           ? ILI9341_RED : ILI9341_YELLOW;
            s_tft.setTextSize(2);
            s_tft.setTextColor(col, ILI9341_BLACK);
            s_tft.setCursor(4, S_Y + 20);
            char buf[20];
            snprintf(buf, sizeof(buf), "KNK:-%u deg", state.ign.knock_retard);
            s_tft.print(buf);
        }
        s_knock = state.ign.knock_retard;
    }

    // ---- Status: fault codes ----------------------------------
    uint8_t fcount = state.diag.active_count;
    if (fcount != s_flt) {
        s_tft.fillRect(0, S_Y + 42, 320, 50, ILI9341_BLACK);
        if (fcount > 0) {
            static const char* SHORT_NAMES[(uint8_t)FaultCode::MAX_CODES] = {
                "", "CLT_H","CLT_L","IAT_H","IAT_L",
                "TPS_H","TPS_L","MAP_H","MAP_L","O2_IN",
                "CPS","CAM","INJ_OC","KNOCK","BAT_H","BAT_L","O2_HTR"
            };
            s_tft.setTextSize(1);
            s_tft.setTextColor(ILI9341_RED, ILI9341_BLACK);
            s_tft.setCursor(4, S_Y + 42);
            s_tft.print("FAULT:");
            for (uint8_t i = 1; i < (uint8_t)FaultCode::MAX_CODES; i++) {
                if (state.diag.active[i]) {
                    s_tft.print(' ');
                    s_tft.print(SHORT_NAMES[i]);
                }
            }
        }
        s_flt = fcount;
    }
}
