#pragma once
#include "types.h"

// Bilinear interpolation on a 3D table (RPM × MAP load)
float table3d_lookup(const Table3D& t, uint16_t rpm, uint8_t map_kpa);

// Linear interpolation on a 1D table
float table1d_lookup(const Table1D& t, int16_t x);

// Initialize default Renix 4.0 tables into config
void tables_load_defaults(ECUConfig& cfg);
