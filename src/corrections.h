#pragma once
#include "types.h"

// Compute all fuel correction multipliers and store in FuelState.
// Returns the combined correction factor (multiply base PW by this).
float corrections_calc(const SensorData& s, ECUState& state, const ECUConfig& cfg);

// Called once per injection event to decrement the after-start counter.
void corrections_injection_event(ECUState& state, const ECUConfig& cfg);

// Detect and quantify throttle-blip accel enrichment.
// Must be called at a fixed rate (e.g., 50 Hz).
void corrections_accel_update(ECUState& state, float tps_pct, uint32_t now_ms,
                               const ECUConfig& cfg);
