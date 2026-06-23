#pragma once
#include "types.h"

// Initialise serial port.
void comms_init();

// Service inbound serial commands and stream real-time data.
// Call in main loop; non-blocking.
void comms_update(ECUState& state, ECUConfig& cfg);
