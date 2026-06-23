#pragma once

// ============================================================
// Renix Jeep 4.0L I6 Standalone ECU — Board Configuration
// Target: Teensy 4.1 (ARM Cortex-M7 @ 600 MHz)
// ============================================================

// ---- Engine -------------------------------------------------
#define ENGINE_CYLINDERS          6
#define ENGINE_DISPLACEMENT_CC    3960
#define ENGINE_STOICH_AFR         14.7f
#define ENGINE_FIRING_ORDER       {0, 4, 2, 5, 1, 3}   // 0-indexed: 1-5-3-6-2-4
#define ENGINE_FUEL_DENSITY_G_CC  0.74f

// ---- Trigger Wheel ------------------------------------------
// 36-1 missing tooth wheel on harmonic balancer (recommended upgrade).
// Stock Renix option: set TRIGGER_WHEEL_TEETH = 44 and TRIGGER_MISSING = 2
// for the flywheel pattern; Renix decoder mode is selected via TRIGGER_MODE.
#define TRIGGER_MODE_MISSING_TOOTH  0
#define TRIGGER_MODE_RENIX_44       1

#define TRIGGER_MODE              TRIGGER_MODE_MISSING_TOOTH
#define TRIGGER_WHEEL_TEETH       36
#define TRIGGER_WHEEL_MISSING     1
#define TRIGGER_DEGREES_PER_TOOTH (360 / TRIGGER_WHEEL_TEETH)   // 10°
// Crank angle at FIRST tooth after missing gap (degrees BTDC of cylinder 1 TDC compression)
#define TRIGGER_SYNC_ANGLE_BTDC   66

// ---- ADC Resolution -----------------------------------------
#define ADC_BITS                  12
#define ADC_MAX                   4095
#define ADC_VREF_MV               3300   // Teensy 4.1 ADC reference voltage

// ---- Injectors ----------------------------------------------
// Renix 4.0: 19.6 lb/hr @ 43.5 PSI, 16 ohm high-impedance
// Ground-switching MOSFET driver (invert from stock Renix power-switch).
#define INJECTOR_FLOW_CC_MIN      148.3f    // 19.6 lb/hr converted (0.74 g/cc density)
#define INJECTOR_FLOW_G_S         (INJECTOR_FLOW_CC_MIN * ENGINE_FUEL_DENSITY_G_CC / 60.0f)
#define INJECTOR_REF_KPA          300.0f    // 43.5 PSI in kPa
#define INJECTOR_DEAD_TIME_14V_US 750       // microseconds at 14V
#define INJECTOR_MIN_PW_US        500       // Minimum commanded pulse width
#define INJECTOR_MAX_DC_PCT       85        // Max duty cycle safety limit

// ---- Ignition -----------------------------------------------
// Output to stock Renix ICM (Ignition Control Module).
// Coil is driven by ICM; we send a dwell/fire trigger only.
#define COIL_DWELL_MIN_US         2200
#define COIL_DWELL_MAX_US         4200
#define COIL_DWELL_REF_RPM        1000
#define COIL_DWELL_REF_US         4000

// Max ignition advance (safety cap); Renix 4.0 stock peak ~35° BTDC
#define IGN_MAX_ADVANCE_DEG       45
#define IGN_MIN_ADVANCE_DEG       0

// ---- Rev Limiter -------------------------------------------
#define REV_LIMIT_HARD_RPM        5600
#define REV_LIMIT_SOFT_RPM        5400    // Begin fuel cut
#define REV_LIMIT_HYSTERESIS_RPM  200

// ---- Idle ---------------------------------------------------
#define IDLE_TPS_THRESHOLD_PCT    3       // TPS % below which idle control is active
#define IDLE_TARGET_RPM_WARM      720
#define IDLE_PROPORTIONAL_GAIN    0.5f
#define IDLE_INTEGRAL_GAIN        0.05f

// ---- Closed-Loop O2 ----------------------------------------
#define CL_STOICH_MV              450     // Narrowband stoich crossover (mV)
#define CL_STEP_PCT               1.0f   // % step per evaluation
#define CL_MAX_STFT_PCT           25.0f  // Short-term trim limit
#define CL_MIN_CLT_C              70     // Minimum CLT to enable CL
#define CL_MIN_RPM                400
#define CL_O2_PERIOD_MS           100    // Evaluate O2 every 100ms

// ---- Accel Enrichment ---------------------------------------
#define AE_TPS_THRESHOLD_PCT_S    50.0f  // TPS rate threshold (% per second)
#define AE_DURATION_TEETH         12     // Enrichment duration in teeth
#define AE_MULTIPLIER             2.0f   // Enrichment amount multiplier

// ---- Fuel Prime / Prime Pulse --------------------------------
#define FUEL_PRIME_PULSE_MS       50     // Injector prime pulse on key-on
#define FUEL_PUMP_PRIME_MS        2000   // Fuel pump pre-prime duration

// ---- Sensor Calibration -------------------------------------

// NTC Thermistors (CLT and IAT — identical Bosch-style sensors)
// Measured Renix values: 100,700Ω @ -40°C  |  185Ω @ 100°C
// Calculated: R0=2827Ω @ 25°C, B=4039K
#define THERM_R0_OHMS             2827.0f
#define THERM_T0_K                298.15f
#define THERM_B                   4039.0f
#define THERM_PULLUP_OHMS         2200.0f  // 2.2kΩ pullup to 3.3V

// TPS: potentiometer, ~0.5V @ idle, ~4.5V @ WOT (5V supply → divider to 3.3V ADC)
// Voltage divider on TPS signal: 22k/(22k+10k) ≈ 0.688 → max 4.5*0.688=3.1V safe
#define TPS_ADC_CLOSED            500     // ~0.5V × 0.688 / 3.3 × 4096
#define TPS_ADC_OPEN              3876    // ~4.5V × 0.688 / 3.3 × 4096

// MAP: GM 1-bar MAP sensor (e.g., ACDelco 213-796), 0-5V output
// Voltage divider: 22k/(22k+10k) same as TPS divider
// 0.5V @ 0 kPa → ADC ≈ 620;  4.65V @ 104 kPa → ADC ≈ 3763
#define MAP_ADC_0KPA              620
#define MAP_ADC_104KPA            3763
#define MAP_MIN_KPA               10.0f
#define MAP_MAX_KPA               110.0f

// O2 Sensor: narrowband, 0–1V output, direct to ADC (safe below 3.3V)
// 0 mV (lean) → ADC 0;   1000 mV (rich) → ADC 1241
#define O2_MV_PER_ADC_COUNT       (3300.0f / ADC_MAX)

// Battery voltage: 47kΩ/(47kΩ+10kΩ) divider — NOT 10k+47k; redo:
// Use 33kΩ top + 10kΩ bottom → Vout = Vin × 10/43 = 0.2326
// At 16V: 3.72V → needs further reduction; use 56kΩ + 10kΩ
// Vout = Vin × 10/66 = 0.1515;  At 16V: 2.42V → ADC 3005;  12V: ADC 2254
#define BATT_DIVIDER_RATIO        0.1515f

// VSS: Open-collector hall effect, 8 pulses/rev (many Renix models)
#define VSS_PULSES_PER_REV        8
#define VSS_TIRE_CIRC_MM          2075   // ~215/75R15 approximate

// ---- Teensy 4.1 Pin Assignments ----------------------------

// Crank & Cam Triggers
#define PIN_CPS_IN                2    // CPS signal (from VR conditioner MAX9926)
#define PIN_CAM_IN                3    // Cam sync (distributor stator, conditioned)
#define PIN_VSS_IN                4    // Vehicle speed sensor

// Fuel Injectors (6× N-channel MOSFET, active HIGH = injector open)
#define PIN_INJ_1                 5
#define PIN_INJ_2                 6
#define PIN_INJ_3                 7
#define PIN_INJ_4                 8
#define PIN_INJ_5                 9
#define PIN_INJ_6                 10

// Ignition (to ICM coil trigger input)
#define PIN_IGN_COIL              11

// IAC Stepper Motor (4 wires: A+, A-, B+, B-)
#define PIN_IAC_A_POS             24
#define PIN_IAC_A_NEG             25
#define PIN_IAC_B_POS             26
#define PIN_IAC_B_NEG             27

// Auxiliary Outputs
#define PIN_FUEL_PUMP_RELAY       28
#define PIN_TACH_OUT              29
#define PIN_CEL                   30    // Check Engine Light
#define PIN_EGR                   31
#define PIN_PURGE                 32
#define PIN_FAN_RELAY             33

// Analog Inputs (A0=14 … A9=23 on Teensy 4.1)
#define PIN_TPS                   A0    // = 14
#define PIN_MAP                   A1    // = 15
#define PIN_CLT                   A2    // = 16
#define PIN_IAT                   A3    // = 17
#define PIN_O2                    A4    // = 18
#define PIN_BATT                  A5    // = 19
#define PIN_SPARE_AN1             A6    // = 20
#define PIN_SPARE_AN2             A7    // = 21

// Serial ports
#define TUNING_SERIAL             Serial    // USB Serial for TunerStudio / monitor
#define TUNING_BAUD               115200
