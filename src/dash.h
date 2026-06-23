#pragma once
#include "types.h"

// ILI9341 TFT digital dashboard on Teensy 4.1 SPI2 (bottom edge pads).
// Pins: MOSI=43, SCK=42, CS=PIN_DASH_CS(22), DC=PIN_DASH_DC(23), RST=PIN_DASH_RST(21)
// Library: ILI9341_t3 (bundled with Teensyduino — no lib_deps entry required)

// Initialise display and draw static chrome (dividers, labels, units).
void dash_init();

// Refresh all dynamic values. Call at DASH_UPDATE_MS (10 Hz).
// Only regions whose values changed are redrawn to avoid flicker.
void dash_update(const ECUState& state, const ECUConfig& cfg);
