#include "ignition.h"
#include "tables.h"
#include "scheduler.h"

// Cylinder TDC compression angles in 720° × 10 frame (origin = cyl 1 TDC = 0°)
// Firing order 1-5-3-6-2-4:
static const uint16_t CYL_TDC_720_X10[ENGINE_CYLINDERS] = {
    0,     // Cyl 1: 0°
    1200,  // Cyl 5: 120°
    2400,  // Cyl 3: 240°
    3600,  // Cyl 6: 360°
    4800,  // Cyl 2: 480°
    6000,  // Cyl 4: 600°
};
// Firing order index → cylinder channel mapping
static const uint8_t FIRING_ORDER[ENGINE_CYLINDERS] = {0, 4, 2, 5, 1, 3};

uint8_t ign_calc_advance(const SensorData& s, const ECUConfig& cfg,
                         uint8_t knock_retard) {
    float adv = table3d_lookup(cfg.ign_table, s.rpm, s.map_kpa);
    if (adv < (float)IGN_MIN_ADVANCE_DEG) adv = IGN_MIN_ADVANCE_DEG;
    if (adv > (float)IGN_MAX_ADVANCE_DEG) adv = IGN_MAX_ADVANCE_DEG;
    // Subtract knock retard; floor at minimum safe advance
    float retarded = adv - (float)knock_retard;
    if (retarded < (float)IGN_MIN_ADVANCE_DEG) retarded = IGN_MIN_ADVANCE_DEG;
    return (uint8_t)retarded;
}

uint16_t ign_calc_dwell(uint16_t rpm, const ECUConfig& cfg) {
    float dw = table1d_lookup(cfg.dwell_vs_rpm, (int16_t)rpm);
    if (dw < (float)COIL_DWELL_MIN_US) dw = COIL_DWELL_MIN_US;
    if (dw > (float)COIL_DWELL_MAX_US) dw = COIL_DWELL_MAX_US;
    return (uint16_t)dw;
}

void ign_schedule_events(ECUState& state, const ECUConfig& cfg) {
    uint8_t  advance_deg = state.ign.advance_deg;
    uint16_t dwell_us    = state.ign.dwell_us;
    uint16_t rpm         = state.sensors.rpm;

    if (rpm == 0) return;

    // Convert dwell time to degrees of crank rotation
    // degrees_per_us = rpm × 360 / 60e6 = rpm / 166666.7
    float deg_per_us = (float)rpm / 166666.7f;
    uint16_t dwell_deg_x10 = (uint16_t)(dwell_us * deg_per_us * 10.0f);

    // For each cylinder in firing order, schedule dwell start and spark
    for (uint8_t i = 0; i < ENGINE_CYLINDERS; i++) {
        uint8_t ch = FIRING_ORDER[i];
        uint16_t tdc_x10 = CYL_TDC_720_X10[ch];  // TDC compression angle × 10

        // Spark fires at: TDC - advance
        // In the 720° frame, angles increase with crank rotation.
        // TDC compression is the reference. BTDC means BEFORE reaching TDC,
        // i.e., at a smaller angle (earlier in the cycle).
        int32_t fire_x10 = (int32_t)tdc_x10 - (int32_t)advance_deg * 10;
        if (fire_x10 < 0) fire_x10 += 7200;
        uint16_t fire_angle_x10 = (uint16_t)(fire_x10 % 7200);

        // Dwell start: fire_angle - dwell_deg
        int32_t dwell_start_x10 = (int32_t)fire_angle_x10 - (int32_t)dwell_deg_x10;
        if (dwell_start_x10 < 0) dwell_start_x10 += 7200;
        uint16_t dwell_angle_x10 = (uint16_t)(dwell_start_x10 % 7200);

        // IGN_DWELL at dwell_angle: begin coil charge immediately on trigger
        sched_add(EventType::IGN_DWELL, ch, dwell_angle_x10, 0);
        // IGN_FIRE at dwell_angle: fire spark after dwell_us delay
        sched_add(EventType::IGN_FIRE,  ch, dwell_angle_x10, dwell_us);
    }
}
