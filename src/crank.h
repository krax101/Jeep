#pragma once
#include "types.h"

// Initialise crank/cam trigger hardware (attach interrupts, configure pins).
void crank_init(CrankState& cs);

// Called from CPS interrupt — do NOT call from user code.
void crank_isr_tooth();

// Called from CAM interrupt — marks 720° revolution boundary.
void crank_isr_cam();

// Returns true once the missing-tooth gap has been detected and
// at least one full revolution counted.
bool crank_is_synced(const CrankState& cs);

// Compute filtered RPM from the last N tooth periods.
uint16_t crank_get_rpm(const CrankState& cs);

// Return the interpolated crank angle (×10, 0–3599) at this instant.
// Uses last tooth timestamp and current tooth period to estimate position.
uint16_t crank_angle_now_x10(const CrankState& cs);

// 720° angle (×10, 0–7199). Valid only after cam sync.
uint16_t crank_angle720_now_x10(const CrankState& cs);
