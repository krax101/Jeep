#include "sensors.h"
#include "config.h"
#include <Arduino.h>
#include <math.h>

// EMA smoothing factors (alpha = 0–1; higher = faster response)
#define ALPHA_MAP   0.3f
#define ALPHA_TPS   0.5f
#define ALPHA_CLT   0.05f
#define ALPHA_IAT   0.05f
#define ALPHA_O2    0.4f
#define ALPHA_BATT  0.1f

static float s_map_f  = 101.0f;
static float s_tps_f  = 0.0f;
static float s_clt_f  = 20.0f;
static float s_iat_f  = 20.0f;
static float s_o2_f   = 450.0f;
static float s_batt_f = 12000.0f;

float ema_filter(float old_val, float new_val, float alpha) {
    return old_val + alpha * (new_val - old_val);
}

// ---- VSS (vehicle speed sensor) ----------------------------
static volatile uint32_t s_vss_last_us = 0;
static volatile uint32_t s_vss_period_us = 0;

void sensors_vss_isr() {
    uint32_t now = micros();
    uint32_t period = now - s_vss_last_us;
    if (period > 500)  // debounce: ignore pulses < 500 µs (> 1868 kph, impossible)
        s_vss_period_us = period;
    s_vss_last_us = now;
}

void sensors_init() {
    analogReadResolution(ADC_BITS);
    analogReadAveraging(4);  // Teensy 4.x hardware averaging
    pinMode(PIN_TPS,  INPUT);
    pinMode(PIN_MAP,  INPUT);
    pinMode(PIN_CLT,  INPUT);
    pinMode(PIN_IAT,  INPUT);
    pinMode(PIN_O2,   INPUT);
    pinMode(PIN_BATT, INPUT);

    // Discrete inputs
    pinMode(PIN_IGN_SW,       INPUT_PULLUP);
    pinMode(PIN_START_SIGNAL, INPUT);
    pinMode(PIN_PARK_NEUTRAL, INPUT_PULLUP);
    pinMode(PIN_AC_REQUEST,   INPUT_PULLUP);
    pinMode(PIN_PS_PRESSURE,  INPUT_PULLUP);

    // VSS: attach interrupt on rising edge
    attachInterrupt(digitalPinToInterrupt(PIN_VSS_IN), sensors_vss_isr, RISING);
}

// ---- NTC Thermistor -----------------------------------------
// Steinhart-Hart simplified: T(K) = B / (ln(R/R_inf))
// R_inf = R0 * exp(-B/T0)
static const float R_INF = THERM_R0_OHMS * expf(-THERM_B / THERM_T0_K);

int8_t adc_to_temp_c(uint16_t adc) {
    if (adc <= 10 || adc >= ADC_MAX - 10) {
        // Rail-to-rail: open or short circuit
        return (adc <= 10) ? 120 : -40;
    }
    float r_ntc = THERM_PULLUP_OHMS * (float)adc / (float)(ADC_MAX - adc);
    float t_k   = THERM_B / logf(r_ntc / R_INF);
    float t_c   = t_k - 273.15f;
    if (t_c < -40.0f) t_c = -40.0f;
    if (t_c > 130.0f) t_c = 130.0f;
    return (int8_t)t_c;
}

// ---- TPS ----------------------------------------------------
uint8_t adc_to_tps_pct(uint16_t adc) {
    int32_t span = TPS_ADC_OPEN - TPS_ADC_CLOSED;
    if (span <= 0) return 0;
    int32_t val = (int32_t)adc - TPS_ADC_CLOSED;
    if (val < 0) val = 0;
    uint8_t pct = (uint8_t)((val * 100) / span);
    return pct > 100 ? 100 : pct;
}

// ---- MAP ----------------------------------------------------
uint8_t adc_to_map_kpa(uint16_t adc) {
    int32_t span = MAP_ADC_104KPA - MAP_ADC_0KPA;
    if (span <= 0) return 101;
    float kpa = (float)(adc - MAP_ADC_0KPA) * 104.0f / (float)span;
    if (kpa < MAP_MIN_KPA) kpa = MAP_MIN_KPA;
    if (kpa > MAP_MAX_KPA) kpa = MAP_MAX_KPA;
    return (uint8_t)kpa;
}

// ---- O2 Sensor (narrowband) ---------------------------------
uint16_t adc_to_o2_mv(uint16_t adc) {
    return (uint16_t)((float)adc * O2_MV_PER_ADC_COUNT);
}

// ---- Battery Voltage ----------------------------------------
uint16_t adc_to_batt_mv(uint16_t adc) {
    float v_adc_mv = (float)adc / ADC_MAX * ADC_VREF_MV;
    return (uint16_t)(v_adc_mv / BATT_DIVIDER_RATIO);
}

// ---- Main Update --------------------------------------------
void sensors_update(SensorData& s) {
    // Sample each channel
    uint16_t raw_tps  = analogRead(PIN_TPS);
    uint16_t raw_map  = analogRead(PIN_MAP);
    uint16_t raw_clt  = analogRead(PIN_CLT);
    uint16_t raw_iat  = analogRead(PIN_IAT);
    uint16_t raw_o2   = analogRead(PIN_O2);
    uint16_t raw_batt = analogRead(PIN_BATT);

    // Convert & filter
    s_tps_f  = ema_filter(s_tps_f,  (float)adc_to_tps_pct(raw_tps),  ALPHA_TPS);
    s_map_f  = ema_filter(s_map_f,  (float)adc_to_map_kpa(raw_map),   ALPHA_MAP);
    s_clt_f  = ema_filter(s_clt_f,  (float)adc_to_temp_c(raw_clt),    ALPHA_CLT);
    s_iat_f  = ema_filter(s_iat_f,  (float)adc_to_temp_c(raw_iat),    ALPHA_IAT);
    s_o2_f   = ema_filter(s_o2_f,   (float)adc_to_o2_mv(raw_o2),      ALPHA_O2);
    s_batt_f = ema_filter(s_batt_f, (float)adc_to_batt_mv(raw_batt),  ALPHA_BATT);

    s.tps_pct = (uint8_t)(s_tps_f < 0.0f ? 0.0f : (s_tps_f > 100.0f ? 100.0f : s_tps_f));
    s.map_kpa = (uint8_t)(s_map_f < 0.0f ? 0.0f : s_map_f);
    s.clt_c   = (int8_t) s_clt_f;
    s.iat_c   = (int8_t) s_iat_f;
    s.o2_mv   = (uint16_t)(s_o2_f < 0.0f ? 0.0f : s_o2_f);
    s.batt_mv = (uint16_t) s_batt_f;

    // VSS: stale after 2 seconds (vehicle stopped)
    noInterrupts();
    uint32_t vss_period = s_vss_period_us;
    uint32_t vss_last   = s_vss_last_us;
    interrupts();

    if ((micros() - vss_last) > 2000000UL) {
        s.vss_kph = 0;
    } else if (vss_period > 0) {
        uint32_t kph = VSS_KPH_CONSTANT / vss_period;
        s.vss_kph = (kph > 255) ? 255 : (uint8_t)kph;
    }

    sensors_check_faults(s);
}

// ---- Discrete Inputs ----------------------------------------
void sensors_read_discrete(SensorData& s) {
    s.ign_sw       = digitalRead(PIN_IGN_SW)       == HIGH;
    s.start_signal = digitalRead(PIN_START_SIGNAL) == HIGH;
    // P/N switch: active-low (switch pulls to GND when in P or N)
    s.park_neutral = digitalRead(PIN_PARK_NEUTRAL) == LOW;
    // A/C request: active-low (switch pulls to GND when requesting A/C)
    s.ac_request   = digitalRead(PIN_AC_REQUEST)   == LOW;
    // PS pressure switch: closes to GND when steering load is detected
    s.ps_pressure  = digitalRead(PIN_PS_PRESSURE)  == LOW;
}

// ---- Fault Detection ----------------------------------------
// Flag sensor as faulted if it reads outside the physically plausible range.
void sensors_check_faults(SensorData& s) {
    // CLT: plausible -40 to 130°C
    s.clt_fault = (s.clt_c <= -39 || s.clt_c >= 125);
    // IAT: plausible -40 to 100°C
    s.iat_fault = (s.iat_c <= -39 || s.iat_c >= 95);
    // TPS stuck at rail
    s.tps_fault = (s.tps_pct == 0 && s_tps_f < 0.2f) ||
                  (s.tps_pct == 100 && s_tps_f > 99.8f);
    // MAP outside 10–109 kPa
    s.map_fault = (s.map_kpa < 11 || s.map_kpa > 108);
}
