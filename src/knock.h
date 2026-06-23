#pragma once
#include "types.h"

// Knock detection — hardware envelope detector approach.
// Circuit: knock sensor → 100nF AC coupling → BAT46 Schottky → ADC pin (PIN_KNOCK/A6)
//          10kΩ + 470nF RC to GND at ADC pin forms envelope follower (τ=4.7ms, fc=33Hz)
// No IntervalTimer or ISR required; ADC is read at 50 Hz from the main sensor loop.
// Detection is gated to the knock window (10–60° ATDC per cylinder).

// Called at 50 Hz from the sensor task.
// angle_720_x10: current crank position from crank_angle720_now_x10() — used for window gating.
void knock_update(IgnState& ign, DiagState& diag,
                  uint16_t angle_720_x10, uint32_t now_ms);

// Called at 50 Hz: decays knock_retard toward zero after the hold period elapses.
void knock_recover(IgnState& ign, DiagState& diag, uint32_t now_ms);

// Returns the envelope peak mV observed in the last knock window (for comms monitor stream).
uint16_t knock_get_peak_mv();
