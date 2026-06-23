#pragma once
#include "types.h"

// Initialise ADC resolution and sensor pins.
void sensors_init();

// Read and convert all analog sensors.  Call at ~50 Hz from the main loop.
void sensors_update(SensorData& s);

// Individual conversions (used internally and in diagnostics)
int8_t   adc_to_temp_c(uint16_t adc);         // NTC thermistor (CLT & IAT)
uint8_t  adc_to_tps_pct(uint16_t adc);        // Throttle position 0–100%
uint8_t  adc_to_map_kpa(uint16_t adc);        // MAP sensor kPa
uint16_t adc_to_o2_mv(uint16_t adc);          // O2 sensor millivolts
uint16_t adc_to_batt_mv(uint16_t adc);        // Battery voltage millivolts

// Simple EMA filter: returns new_val * alpha + old_val * (1-alpha)
float ema_filter(float old_val, float new_val, float alpha);

// Check sensor plausibility; set fault flags in SensorData.
void sensors_check_faults(SensorData& s);

// Read discrete (digital) inputs: IGN_SW, START_SIGNAL, PARK_NEUTRAL,
// AC_REQUEST, PS_PRESSURE.  Call alongside sensors_update().
void sensors_read_discrete(SensorData& s);

// VSS interrupt handler — attach to PIN_VSS_IN rising edge in setup().
void sensors_vss_isr();
