#pragma once
#include "types.h"

// SD card data logger using Teensy 4.1 built-in SD slot.
// Opens the next available LOGnnnn.CSV file on startup or on first enable.
// Logs all critical ECU channels at 10 Hz.

// Attempt to initialise SD card and open a new log file.
// Returns true on success. Safe to call multiple times (nop if already open).
bool datalog_init();

// Write one row to the log. Call at 10 Hz.
// No-op if log not initialised or not enabled.
void datalog_update(const ECUState& state);

// Flush and close the current log file.
void datalog_stop();

// Returns true if a log file is currently open and recording.
bool datalog_active();
