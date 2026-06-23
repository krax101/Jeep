#include "fuel.h"
#include "tables.h"
#include "scheduler.h"
#include "corrections.h"
#include <math.h>

// Firing order: cylinder indices 0-based (physical cyl 1-5-3-6-2-4)
// Index  0  1  2  3  4  5
// Cyl    1  5  3  6  2  4
static const uint8_t FIRING_ORDER[ENGINE_CYLINDERS] = {0, 4, 2, 5, 1, 3};
// 720° start angle for each cylinder's intake stroke (crank angle × 10)
// These are the crank angles at which each cylinder's intake valve opens.
// Origin: TDC compression of cylinder 1 = 0°.
// Intake stroke starts at TDC exhaust = TDC compression + 360°.
// Cyl N intake start = (N_firing_position * 120 + 360) % 720
// In 720° × 10 space:
static const uint16_t CYL_INTAKE_ANGLE_720_X10[ENGINE_CYLINDERS] = {
    3600,   // Cyl 1:  0° + 360° = 360°  → ×10 = 3600
    4800,   // Cyl 5:  120° + 360° = 480° → 4800
    6000,   // Cyl 3:  240° + 360° = 600° → 6000
    0,      // Cyl 6:  360° + 360° = 720° → wraps to 0°
    1200,   // Cyl 2:  480° + 360° = 840° → 120° (mod 720) → 1200
    2400,   // Cyl 4:  600° + 360° = 960° → 240° (mod 720) → 2400
};

// ---- Base Pulse Width Calculation ---------------------------
// Speed-density: PW = (mass_air_per_stroke / injector_flow) × 1e6 μs/s
// mass_air = (disp_cc / cyls / 1000) × air_density × VE
// air_density (g/cc) = 1.2041e-3 × (MAP_kPa/101.325) × (298.15/(273.15+IAT_C))
uint32_t fuel_calc_base_pw(const SensorData& s, const ECUConfig& cfg) {
    float ve = table3d_lookup(cfg.ve_table, s.rpm, s.map_kpa);

    // Air density at MAP and IAT
    float map_ratio  = (float)s.map_kpa / 101.325f;
    float temp_ratio = 298.15f / (273.15f + (float)s.iat_c);
    float air_density_gcc = 1.2041e-3f * map_ratio * temp_ratio;

    // Air mass entering cylinder per intake stroke (grams)
    float air_mass_g = ((float)ENGINE_DISPLACEMENT_CC / ENGINE_CYLINDERS / 1000.0f)
                       * air_density_gcc * (ve / 100.0f);

    // Fuel mass at stoichiometric ratio
    float fuel_mass_g = air_mass_g / ENGINE_STOICH_AFR;

    // Injector flow rate (g/μs)
    float flow_g_per_us = (cfg.injector_flow_cc_min * ENGINE_FUEL_DENSITY_G_CC)
                          / 60000000.0f;

    if (flow_g_per_us <= 0.0f) return INJECTOR_MIN_PW_US;

    uint32_t pw_us = (uint32_t)(fuel_mass_g / flow_g_per_us);
    if (pw_us < INJECTOR_MIN_PW_US) pw_us = INJECTOR_MIN_PW_US;
    return pw_us;
}

// ---- Apply Corrections & Dead Time --------------------------
uint32_t fuel_calc_final_pw(uint32_t base_pw_us, float corr_mult,
                             const SensorData& s, const ECUConfig& cfg) {
    // Dead time correction from battery voltage table
    int16_t batt_x = (int16_t)(s.batt_mv / 100);  // table key: ×100 mV
    float dead_us = table1d_lookup(cfg.dead_time, batt_x);

    float final = (float)base_pw_us * corr_mult + dead_us;
    if (final < (float)INJECTOR_MIN_PW_US) final = (float)INJECTOR_MIN_PW_US;
    return (uint32_t)final;
}

// ---- Schedule Injection Events ------------------------------
// Each cylinder gets:
//   INJ_OPEN  event at its intake angle
//   INJ_CLOSE event as a one-shot timer triggered at OPEN time + delay_us
void fuel_schedule_events(ECUState& state, const ECUConfig& cfg) {
    uint32_t final_pw = state.fuel.final_pw_us;
    InjMode  mode     = state.fuel.mode;

    if (mode == InjMode::SEQUENTIAL) {
        // One injector per cylinder, once per 720° cycle
        for (uint8_t i = 0; i < ENGINE_CYLINDERS; i++) {
            uint8_t  ch      = FIRING_ORDER[i];
            uint16_t open_angle = CYL_INTAKE_ANGLE_720_X10[ch];

            // Open event at intake angle (no extra delay)
            sched_add(EventType::INJ_OPEN,  ch, open_angle, 0);
            // Close event: same angle trigger, but delay = final_pw
            sched_add(EventType::INJ_CLOSE, ch, open_angle, final_pw);
        }
    } else {
        // Batch mode: bank A = cyls 0,2,4 fire at 0° (TDC #1)
        //             bank B = cyls 1,3,5 fire at 180° (180° later)
        // Use channels 0 (bank A trigger) and 1 (bank B trigger)
        // Only cyls 0,1 channels used to represent banks
        sched_add(EventType::INJ_OPEN,  0, 0,    0);
        sched_add(EventType::INJ_CLOSE, 0, 0,    final_pw);
        sched_add(EventType::INJ_OPEN,  1, 1800, 0);
        sched_add(EventType::INJ_CLOSE, 1, 1800, final_pw);
        // In batch mode injectors 2-5 are fired by duplicating the signal
        // (external wiring: bank A drives INJ1,3,5; bank B drives INJ2,4,6)
    }
}

// ---- Duty Cycle Update --------------------------------------
void fuel_update_dc(FuelState& fs, uint16_t rpm, uint32_t final_pw_us) {
    if (rpm == 0) { fs.dc_pct = 0; return; }
    // Cycle time per cylinder in sequential mode (720° = 2 revolutions)
    uint32_t cycle_us = (uint32_t)(120000000UL / rpm);  // 2 × 60e6 / RPM
    if (cycle_us == 0) { fs.dc_pct = 99; return; }
    uint32_t dc = final_pw_us * 100 / cycle_us;
    fs.dc_pct = (dc > 99) ? 99 : (uint8_t)dc;
}
