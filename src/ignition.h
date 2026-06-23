#pragma once
#include "types.h"

// Look up timing advance (degrees BTDC) from table.
uint8_t ign_calc_advance(const SensorData& s, const ECUConfig& cfg);

// Compute coil dwell time (μs) from RPM.
uint16_t ign_calc_dwell(uint16_t rpm, const ECUConfig& cfg);

// Build the ignition (dwell start + fire) event schedule.
// Coil charges starting at (TDC - advance - dwell_degrees) and fires at TDC - advance.
void ign_schedule_events(ECUState& state, const ECUConfig& cfg);
