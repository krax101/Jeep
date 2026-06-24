#pragma once
#include "types.h"

// Initialise diagnostic state and CEL pin.
void diag_init(DiagState& d);

// Set a fault active.  CEL is turned on automatically.
void diag_set(DiagState& d, FaultCode code);

// Clear a fault if the condition has resolved for >2 s.
void diag_clear(DiagState& d, FaultCode code);

// Run all plausibility checks and update fault state.
// run_start_ms: millis() when engine entered RUNNING — used to delay O2_INACTIVE
// detection until the heater has had sufficient time to warm the sensor.
// Call at ~5 Hz.
void diag_update(DiagState& d, const SensorData& s, const CrankState& cs,
                 uint32_t now_ms, uint32_t run_start_ms);

// Print active faults to Serial.
void diag_print(const DiagState& d);
