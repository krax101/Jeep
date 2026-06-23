#pragma once
#include "types.h"

// Calculate base pulse width (μs) from physical engine parameters.
// Does NOT include dead time or O2 trim.
uint32_t fuel_calc_base_pw(const SensorData& s, const ECUConfig& cfg);

// Apply corrections and dead-time to get final commanded pulse width.
uint32_t fuel_calc_final_pw(uint32_t base_pw_us, float corr_mult,
                             const SensorData& s, const ECUConfig& cfg);

// Build the injection event schedule in the ECUState event queue.
// Should be called at ~50 Hz when RPM and MAP have changed meaningfully.
void fuel_schedule_events(ECUState& state, const ECUConfig& cfg);

// Update duty cycle reading.
void fuel_update_dc(FuelState& fs, uint16_t rpm, uint32_t final_pw_us);
