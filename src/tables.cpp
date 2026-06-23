#include "tables.h"
#include <string.h>
#include <math.h>

// ---- Interpolation ------------------------------------------

static inline float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

float table3d_lookup(const Table3D& t, uint16_t rpm, uint8_t map_kpa) {
    // Find RPM bracket
    uint8_t xi = 0;
    while (xi < t.rpm_count - 2 && rpm >= t.rpm_bins[xi + 1]) xi++;
    float xfrac = 0.0f;
    if (t.rpm_bins[xi + 1] != t.rpm_bins[xi])
        xfrac = clampf((float)(rpm - t.rpm_bins[xi]) /
                       (float)(t.rpm_bins[xi + 1] - t.rpm_bins[xi]), 0.0f, 1.0f);

    // Find MAP bracket
    uint8_t yi = 0;
    while (yi < t.load_count - 2 && map_kpa >= t.load_bins[yi + 1]) yi++;
    float yfrac = 0.0f;
    if (t.load_bins[yi + 1] != t.load_bins[yi])
        yfrac = clampf((float)(map_kpa - t.load_bins[yi]) /
                       (float)(t.load_bins[yi + 1] - t.load_bins[yi]), 0.0f, 1.0f);

    // Bilinear interpolation: values[load][rpm]
    float v00 = t.values[yi][xi];
    float v10 = t.values[yi][xi + 1];
    float v01 = t.values[yi + 1][xi];
    float v11 = t.values[yi + 1][xi + 1];

    float v0 = v00 + (v10 - v00) * xfrac;
    float v1 = v01 + (v11 - v01) * xfrac;
    return v0 + (v1 - v0) * yfrac;
}

float table1d_lookup(const Table1D& t, int16_t x) {
    if (x <= t.x[0])             return (float)t.y[0];
    if (x >= t.x[t.count - 1])   return (float)t.y[t.count - 1];

    for (uint8_t i = 0; i < t.count - 1; i++) {
        if (x >= t.x[i] && x <= t.x[i + 1]) {
            if (t.x[i + 1] == t.x[i]) return (float)t.y[i];
            float frac = (float)(x - t.x[i]) / (float)(t.x[i + 1] - t.x[i]);
            return t.y[i] + frac * (float)(t.y[i + 1] - t.y[i]);
        }
    }
    return (float)t.y[t.count - 1];
}

// ---- Default Table Population for Renix 4.0 ----------------

void tables_load_defaults(ECUConfig& cfg) {
    cfg.magic   = CFG_MAGIC;
    cfg.version = CFG_VER;

    // Trigger
    cfg.trigger_teeth    = TRIGGER_WHEEL_TEETH;
    cfg.trigger_missing  = TRIGGER_WHEEL_MISSING;
    cfg.trigger_mode     = TRIGGER_MODE;
    cfg.trigger_sync_btdc = TRIGGER_SYNC_ANGLE_BTDC;

    // Injectors
    cfg.injector_flow_cc_min       = INJECTOR_FLOW_CC_MIN;
    cfg.injector_dead_time_14v_us  = INJECTOR_DEAD_TIME_14V_US;

    // Limits
    cfg.rev_limit_rpm  = REV_LIMIT_HARD_RPM;
    cfg.idle_target_rpm = IDLE_TARGET_RPM_WARM;

    // Closed loop
    cfg.cl_enabled  = true;
    cfg.cl_max_stft = CL_MAX_STFT_PCT;
    cfg.cl_max_ltft = CL_MAX_STFT_PCT;

    // Accel enrichment
    cfg.ae_tps_threshold = AE_TPS_THRESHOLD_PCT_S;
    cfg.ae_multiplier    = AE_MULTIPLIER;

    // ---- VE Table (16 RPM × 16 MAP) ------------------------
    // RPM bins: 500, 750, 1000, 1250, 1500, 2000, 2500, 3000,
    //           3500, 4000, 4500, 5000, 5500, 6000, 6500, 7000
    // MAP bins: 20, 25, 30, 35, 40, 45, 50, 60,
    //           70, 80, 90, 95, 100, 102, 104, 106 (kPa)
    cfg.ve_table.rpm_count  = 16;
    cfg.ve_table.load_count = 16;
    const uint16_t ve_rpm[] = {500,750,1000,1250,1500,2000,2500,3000,
                                3500,4000,4500,5000,5500,6000,6500,7000};
    const uint8_t  ve_map[] = {20,25,30,35,40,45,50,60,70,80,90,95,100,102,104,106};
    memcpy(cfg.ve_table.rpm_bins,  ve_rpm, sizeof(ve_rpm));
    memcpy(cfg.ve_table.load_bins, ve_map, sizeof(ve_map));

    // values[load_row][rpm_col] — percent volumetric efficiency
    const uint8_t ve[16][16] = {
    //   500  750 1000 1250 1500 2000 2500 3000 3500 4000 4500 5000 5500 6000 6500 7000
        {38,  38,  40,  40,  40,  38,  36,  34,  32,  30,  28,  26,  24,  22,  20,  18},  // 20kPa
        {44,  46,  47,  48,  48,  46,  45,  43,  41,  39,  37,  35,  33,  31,  29,  27},  // 25kPa
        {54,  58,  60,  61,  62,  61,  60,  58,  56,  53,  50,  47,  44,  41,  38,  36},  // 30kPa
        {58,  63,  65,  67,  68,  67,  66,  64,  62,  59,  56,  53,  50,  47,  44,  41},  // 35kPa
        {62,  66,  69,  71,  72,  72,  71,  70,  68,  65,  62,  59,  56,  53,  50,  47},  // 40kPa
        {65,  68,  72,  73,  74,  75,  75,  74,  72,  70,  67,  64,  61,  58,  55,  52},  // 45kPa
        {67,  71,  74,  76,  77,  78,  78,  78,  77,  74,  71,  68,  65,  62,  59,  56},  // 50kPa
        {70,  74,  77,  79,  80,  81,  82,  82,  81,  79,  76,  73,  70,  67,  64,  61},  // 60kPa
        {72,  76,  79,  81,  82,  83,  84,  84,  84,  82,  79,  76,  73,  70,  67,  64},  // 70kPa
        {73,  77,  80,  82,  84,  85,  86,  87,  87,  86,  84,  81,  78,  75,  72,  69},  // 80kPa
        {73,  77,  80,  83,  85,  87,  88,  89,  90,  90,  89,  87,  84,  81,  78,  75},  // 90kPa
        {72,  76,  80,  82,  84,  87,  89,  90,  91,  91,  90,  88,  85,  82,  79,  76},  // 95kPa
        {71,  75,  79,  81,  83,  86,  88,  90,  91,  92,  91,  88,  85,  82,  79,  76},  // 100kPa
        {70,  74,  78,  80,  82,  85,  87,  89,  90,  91,  90,  88,  85,  82,  79,  76},  // 102kPa
        {69,  73,  77,  79,  81,  84,  86,  88,  89,  90,  89,  87,  84,  81,  78,  75},  // 104kPa
        {68,  72,  76,  78,  80,  83,  85,  87,  88,  89,  88,  86,  83,  80,  77,  74},  // 106kPa
    };
    memcpy(cfg.ve_table.values, ve, sizeof(ve));

    // ---- Ignition Timing Table (degrees BTDC) ---------------
    cfg.ign_table.rpm_count  = 16;
    cfg.ign_table.load_count = 16;
    memcpy(cfg.ign_table.rpm_bins,  ve_rpm, sizeof(ve_rpm));
    memcpy(cfg.ign_table.load_bins, ve_map, sizeof(ve_map));

    const uint8_t ign[16][16] = {
    //   500  750 1000 1250 1500 2000 2500 3000 3500 4000 4500 5000 5500 6000 6500 7000
        { 5,   8,  10,  12,  14,  16,  17,  17,  16,  14,  12,  10,   8,   7,   6,   5},  // 20kPa
        { 8,  10,  12,  14,  16,  19,  21,  22,  21,  19,  17,  15,  13,  11,   9,   7},  // 25kPa
        {12,  14,  15,  16,  17,  20,  23,  25,  25,  23,  21,  18,  16,  14,  12,  10},  // 30kPa
        {14,  16,  18,  20,  22,  25,  28,  29,  29,  27,  25,  22,  20,  18,  16,  14},  // 35kPa
        {15,  17,  20,  22,  24,  27,  30,  32,  31,  29,  27,  24,  22,  20,  18,  16},  // 40kPa
        {16,  18,  21,  23,  25,  28,  31,  33,  33,  31,  29,  26,  24,  22,  20,  18},  // 45kPa
        {16,  18,  21,  23,  25,  28,  31,  33,  34,  32,  30,  27,  25,  23,  21,  19},  // 50kPa
        {15,  17,  20,  22,  24,  28,  30,  32,  33,  32,  30,  28,  26,  24,  22,  20},  // 60kPa
        {14,  16,  19,  21,  23,  26,  28,  30,  32,  32,  30,  28,  26,  24,  22,  20},  // 70kPa
        {13,  15,  17,  19,  21,  24,  26,  28,  30,  31,  30,  28,  26,  24,  22,  20},  // 80kPa
        {12,  14,  16,  18,  20,  22,  24,  26,  28,  30,  30,  28,  26,  24,  22,  20},  // 90kPa
        {12,  14,  15,  17,  19,  21,  23,  25,  27,  29,  30,  28,  26,  24,  22,  20},  // 95kPa
        {12,  14,  15,  16,  18,  20,  22,  24,  26,  28,  30,  30,  28,  26,  24,  22},  // 100kPa
        {12,  13,  14,  15,  17,  19,  21,  23,  25,  27,  29,  30,  28,  26,  24,  22},  // 102kPa
        {12,  13,  14,  15,  16,  18,  20,  22,  24,  26,  28,  29,  28,  26,  24,  22},  // 104kPa
        {12,  13,  14,  15,  16,  18,  20,  22,  24,  26,  28,  29,  28,  26,  24,  22},  // 106kPa
    };
    memcpy(cfg.ign_table.values, ign, sizeof(ign));

    // ---- Coolant Temp Warm-Up Enrichment (%) ----------------
    // At operating temp (80°C+) → 0% extra fuel
    cfg.clt_wue.count = 8;
    const int16_t wue_x[] = {-40, -20,  0, 20, 40, 60, 80, 100};
    const int16_t wue_y[] = {100,  80, 55, 35, 20, 10,  0,   0};
    memcpy(cfg.clt_wue.x, wue_x, sizeof(wue_x));
    memcpy(cfg.clt_wue.y, wue_y, sizeof(wue_y));

    // ---- IAT Correction (%) ---------------------------------
    // Positive = add fuel, negative = remove (IAT > reference adds density)
    cfg.iat_corr.count = 8;
    const int16_t iat_x[] = {-40, -20,  0, 20, 40, 60,  80, 100};
    const int16_t iat_y[] = { 15,  12,  8,  4,  0, -4,  -8, -14};
    memcpy(cfg.iat_corr.x, iat_x, sizeof(iat_x));
    memcpy(cfg.iat_corr.y, iat_y, sizeof(iat_y));

    // ---- Cranking Enrichment (% of stoich PW) ---------------
    cfg.crank_enrich.count = 8;
    const int16_t cr_x[] = {-40, -20,  0, 20, 40, 60,  80, 100};
    const int16_t cr_y[] = {220, 180, 140, 100, 70, 40,  20,  10};
    memcpy(cfg.crank_enrich.x, cr_x, sizeof(cr_x));
    memcpy(cfg.crank_enrich.y, cr_y, sizeof(cr_y));

    // ---- After-Start Enrichment (event count vs CLT) --------
    // How many injection events to maintain enrichment after start
    cfg.ase_table.count = 8;
    const int16_t ase_x[] = {-40, -20,  0, 20, 40, 60, 80, 100};
    const int16_t ase_y[] = {500, 400, 300, 200, 120, 60, 20,  0};
    memcpy(cfg.ase_table.x, ase_x, sizeof(ase_x));
    memcpy(cfg.ase_table.y, ase_y, sizeof(ase_y));

    // ---- Injector Dead Time vs Battery Voltage (×100 mV) ----
    cfg.dead_time.count = 8;
    const int16_t dt_x[] = {60, 80, 100, 110, 120, 130, 140, 160};  // ×100mV = 6V–16V
    const int16_t dt_y[] = {2400, 1800, 1350, 1100, 950, 820, 750, 650};  // μs
    memcpy(cfg.dead_time.x, dt_x, sizeof(dt_x));
    memcpy(cfg.dead_time.y, dt_y, sizeof(dt_y));

    // ---- Idle Target RPM vs CLT -----------------------------
    cfg.idle_vs_clt.count = 8;
    const int16_t idle_x[] = {-40, -20,  0, 20,  40,  60,  80, 100};
    const int16_t idle_y[] = {1400,1300,1200,1100,1000, 900, 800, 720};
    memcpy(cfg.idle_vs_clt.x, idle_x, sizeof(idle_x));
    memcpy(cfg.idle_vs_clt.y, idle_y, sizeof(idle_y));

    // ---- Coil Dwell vs RPM (μs) -----------------------------
    cfg.dwell_vs_rpm.count = 8;
    const int16_t dw_x[] = {0, 500, 1000, 2000, 3000, 4000, 5000, 6000};
    const int16_t dw_y[] = {4200, 4200, 4000, 3500, 3000, 2700, 2400, 2200};
    memcpy(cfg.dwell_vs_rpm.x, dw_x, sizeof(dw_x));
    memcpy(cfg.dwell_vs_rpm.y, dw_y, sizeof(dw_y));

    cfg.checksum = 0;  // recalculated on save
}
