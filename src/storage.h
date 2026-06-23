#pragma once
#include "types.h"

// Load ECUConfig from EEPROM.  Returns true if CRC and magic valid.
bool storage_load(ECUConfig& cfg);

// Save ECUConfig to EEPROM with updated CRC.
void storage_save(const ECUConfig& cfg);

// Wipe EEPROM and write factory defaults.
void storage_factory_reset(ECUConfig& cfg);
