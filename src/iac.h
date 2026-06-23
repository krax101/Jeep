#pragma once
#include "types.h"

// Initialise IAC stepper pins and home the motor.
void iac_init(IACState& iac);

// Called at ~10 Hz: moves stepper toward target by one step if needed.
void iac_update(IACState& iac, const SensorData& s, const ECUConfig& cfg);

// Set target position from closed-loop idle logic.
// Positive = more air (open), negative offset = less air (close).
void iac_set_target(IACState& iac, int16_t position);

// Park the IAC (retract to stop, used on shutdown).
void iac_park(IACState& iac);

// Apply idle-up bumps for accessory loads.  Call from the 100 ms task.
// ac_on: A/C compressor engaged; ps_load: PS pressure switch active;
// in_drive: transmission in a drive gear (not P or N).
void iac_apply_idle_compensation(IACState& iac, bool ac_on,
                                 bool ps_load, bool in_drive);
