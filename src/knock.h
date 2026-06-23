#pragma once
#include "types.h"

// Initialise the knock sensor IntervalTimer (25 kHz sampling).
// Requires hardware DC bias of 1.65 V on PIN_KNOCK ADC input.
void knock_init();

// Called from the 20 ms sensor task: updates knock_retard in IgnState
// and sets FaultCode::KNOCK in DiagState when knock is sustained.
void knock_update(IgnState& ign, DiagState& diag, uint32_t now_ms);

// Called every 20 ms to decay knock retard toward zero and clear fault.
void knock_recover(IgnState& ign, DiagState& diag, uint32_t now_ms);

// Returns raw peak mV since last call (for comms monitor stream).
uint16_t knock_get_peak_mv();
