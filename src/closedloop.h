#pragma once
#include "types.h"

// Evaluate O2 sensor and update STFT/LTFT.
// Must be called at ~10 Hz (every 100 ms) when conditions allow closed loop.
void cl_update(ECUState& state, const ECUConfig& cfg);

// Reset short-term trim to 0 (e.g., on mode change or fault).
void cl_reset_stft(FuelState& fs);
