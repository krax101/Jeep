#pragma once

// ============================================================
// Renix Jeep 4.0L I6 Standalone ECU — Board Configuration
// Target: Teensy 4.1 (ARM Cortex-M7 @ 600 MHz)
//
// HARNESS AUDIT: Every Renix ECU connector pin is accounted for
// below, either as a firmware GPIO or as a justified hardware-only
// connection. See hardware/WIRING.md for the full audit table.
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

// ---- Knock Detection ----------------------------------------
// Renix 4.0L knock sensor: piezoelectric, block-mounted.
// Center frequency ~6.7 kHz for this engine.
// Hardware: 1.65V DC bias on ADC input (two 10kΩ from 3.3V rail).
// Optional external RC bandpass 5–9 kHz improves noise immunity.
#define KNOCK_SAMPLE_RATE_HZ      25000    // 25 kHz ADC sampling
#define KNOCK_ADC_BIAS            2048     // Expected midpoint (12-bit at 1.65V)
#define KNOCK_THRESHOLD_RATIO     2.5f     // Peak must be 2.5× noise floor
#define KNOCK_WINDOW_ATDC_DEG     60       // Evaluate 0–60° ATDC only
#define KNOCK_RETARD_STEP_DEG     2.0f     // Degrees retarded per knock event
#define KNOCK_RETARD_MAX_DEG      10.0f    // Maximum retard ceiling
#define KNOCK_RECOVER_DEG_S       2.0f     // Recovery rate (deg/sec toward 0)
#define KNOCK_CONFIRM_SAMPLES     3        // Consecutive above-threshold samples to confirm

// ---- Rev Limiter -------------------------------------------
#define REV_LIMIT_HARD_RPM        5600
#define REV_LIMIT_SOFT_RPM        5400    // Begin fuel cut
#define REV_LIMIT_HYSTERESIS_RPM  200

// ---- Idle ---------------------------------------------------
#define IDLE_TPS_THRESHOLD_PCT    3       // TPS % below which idle control is active
#define IDLE_TARGET_RPM_WARM      720
#define IDLE_PROPORTIONAL_GAIN    0.5f
#define IDLE_INTEGRAL_GAIN        0.05f

// Idle step compensation bumps (IAC steps added on top of base target)
#define IDLE_AC_BUMP_STEPS        20      // A/C compressor clutch load
#define IDLE_PS_BUMP_STEPS        12      // Power steering pump load
#define IDLE_DRIVE_BUMP_STEPS     15      // Torque converter drag (AT in Drive)

// ---- A/C Compressor Control ---------------------------------
// ECU controls the A/C clutch relay to prevent stall on engagement.
// A 200ms delay between request and engagement allows IAC to pre-open.
#define AC_ENGAGE_DELAY_MS        200
// Disable A/C compressor above this TPS (WOT cutoff)
#define AC_WOT_CUTOFF_TPS_PCT     85
// Disable A/C compressor above this speed (km/h) — protects compressor
#define AC_SPEED_CUTOFF_KPH       160

// ---- Upshift Light (A11 — Manual Transmission) --------------
// Illuminates when driver should upshift for best economy.
// On AT models, A11 is not used (TCU is fully independent per Renix design).
#define UPSHIFT_MIN_RPM           2200
#define UPSHIFT_MAX_RPM           3200
#define UPSHIFT_MIN_TPS_PCT       20     // Only light when actually driving

// ---- Latch Relay (A9) ---------------------------------------
// ECU holds itself powered via A9 after key-off to complete shutdown tasks
// (park IAC, save LTFT to EEPROM). Releases after SHUTDOWN_HOLD_MS.
#define SHUTDOWN_HOLD_MS          3000

// ---- O2 Heater Relay ----------------------------------------
// O2 heater enabled after a warm-up delay to prevent thermal shock.
#define O2_HEATER_DELAY_MS        30000  // 30 s after engine start
#define O2_HEATER_CLT_ENABLE_C    40     // Or when CLT > 40°C, whichever first

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

// Knock sensor: piezo AC signal, 1.65V DC bias applied in hardware.
// 0–3.3V after bias; midpoint = 2048 (12-bit ADC at 1.65V)
#define KNOCK_MV_PER_ADC_COUNT    O2_MV_PER_ADC_COUNT

// Battery voltage: 56kΩ + 10kΩ divider → ratio 10/66 = 0.1515
// At 16V: 2.42V → ADC 3005;  At 12V: 1.82V → ADC 2254
#define BATT_DIVIDER_RATIO        0.1515f

// VSS: Open-collector hall effect, 8 pulses/rev (stock XJ speed sensor)
#define VSS_PULSES_PER_REV        8
#define VSS_TIRE_CIRC_MM          2075   // ~215/75R15 approximate
// Speed formula: kph = 934000 / period_us  (pre-computed constant)
#define VSS_KPH_CONSTANT          934000UL

// High-side digital inputs (12V switched signals → voltage divider to 3.3V)
// Use 100kΩ top + 22kΩ bottom → ratio 22/122 = 0.180
// At 12V: 2.16V → HIGH;  At 0V: 0V → LOW.  Safe Schmitt-trigger input.
#define HIGHSIDE_INPUT_DIVIDER    0.180f

// ============================================================
// Teensy 4.1 Pin Assignments
// Pins 0–41 are digital I/O; A0–A9 (pins 14–23) are also analog.
// ============================================================

// ---- Crank & Cam Triggers -----------------------------------
#define PIN_CPS_IN                2    // D1: CPS from MAX9926 VR conditioner (INT)
#define PIN_CAM_IN                3    // C16: Cam/stator sync from conditioner (INT)
#define PIN_VSS_IN                4    // VSS hall-effect (INT) — 8 pulses/rev

// ---- Fuel Injectors (6× MOSFET, active HIGH = open) ---------
#define PIN_INJ_1                 5    // Cylinder 1
#define PIN_INJ_2                 6    // Cylinder 5
#define PIN_INJ_3                 7    // Cylinder 3
#define PIN_INJ_4                 8    // Cylinder 6
#define PIN_INJ_5                 9    // Cylinder 2
#define PIN_INJ_6                 10   // Cylinder 4

// ---- Ignition -----------------------------------------------
#define PIN_IGN_COIL              11   // To ICM coil trigger (5V square wave)
#define PIN_UPSHIFT_LIGHT         12   // A11: Upshift indicator lamp (MT only)
                                       // AT models: leave unconnected per Renix spec

// ---- Analog Inputs (A0–A7 = pins 14–21) --------------------
#define PIN_TPS                   A0   // C7:  Throttle position sensor
#define PIN_MAP                   A1   // C6:  MAP sensor signal
#define PIN_CLT                   A2   // C10: Coolant temp sensor
#define PIN_IAT                   A3   // C8:  Intake air temp sensor
#define PIN_O2                    A4   // D9:  O2 sensor signal (narrowband)
#define PIN_BATT                  A5   // Battery voltage sense (56k+10k divider)
#define PIN_KNOCK                 A6   // D8:  Knock sensor (1.65V bias required)
                                       // A7 = pin 21 reserved for future use

// ---- IAC Stepper Motor (4-wire full-step) -------------------
#define PIN_IAC_A_POS             24
#define PIN_IAC_A_NEG             25
#define PIN_IAC_B_POS             26
#define PIN_IAC_B_NEG             27

// ---- Switched Outputs ---------------------------------------
#define PIN_FUEL_PUMP_RELAY       28   // Fuel pump relay (key-on prime + run)
#define PIN_TACH_OUT              29   // Tachometer signal output
#define PIN_CEL                   30   // A/MIL: Check Engine Light
#define PIN_EGR                   31   // A10: EGR solenoid
#define PIN_PURGE                 32   // A10b: Charcoal canister purge solenoid
#define PIN_FAN_RELAY             33   // Radiator fan relay

// ---- High-Side Switch Inputs (12V → 3.3V divider) ----------
// All use 100kΩ + 22kΩ voltage divider; read with digitalRead (HIGH = active)
#define PIN_IGN_SW                34   // A2:  Ignition switch (key-on sense)
#define PIN_START_SIGNAL          35   // C3:  Starter engagement (+12V while cranking)
#define PIN_PARK_NEUTRAL          36   // C4:  Park/Neutral switch (LOW = in P or N)
                                       // MT models: tie to GND through 1kΩ (always "P/N")
#define PIN_AC_REQUEST            37   // A/C thermostat/switch request signal

// ---- Active-Low Switch Inputs (pull-up, close to GND) ------
#define PIN_PS_PRESSURE           38   // Power steering pressure switch (LOW = high pressure)
                                       // 1989-90 XJ: on 6-pin under-dash connector

// ---- Controlled Relay Outputs ------------------------------
#define PIN_AC_CLUTCH             39   // A/C compressor clutch relay
#define PIN_O2_HEATER             40   // O2 sensor heater relay (delayed enable)
#define PIN_LATCH_RELAY           41   // A9: ECU self-hold relay (post key-off shutdown)

// ============================================================
// HARDWARE-ONLY PINS — NO FIRMWARE GPIO ASSIGNMENT NEEDED
// ============================================================
// The following Renix harness pins connect directly to hardware
// on the ECU board and require no MCU GPIO:
//
//  C5  (Cam sensor –):        PCB GND — shield return for cam sensor pair
//  C9  (Factory "not used"):  Leave N/C — confirmed unused in factory FSM
//  C11 (Injector +12V feed):  Hardwired to fuel pump relay switched output;
//                              ECU does not switch this rail, injectors do
//  C12 (TX Serial/Diagnostic):REPLACED by USB Serial (superior interface).
//                              The factory diagnostic datastream is fully
//                              superseded by the USB monitor + fault codes.
//  C13 (Factory "not used"):  Leave N/C — confirmed unused in factory FSM
//  C14 (MAP sensor +5V):      LM7805 5V output — hardware supply, no GPIO
//  C15 (TPS +5V):             LM7805 5V output — hardware supply, no GPIO
//  D2  (GND/Diagnostic GND):  PCB GND
//  D3  (Sensor ground):       PCB GND (separate pour from power GND)
//  D10 (Injector +12V feed):  Same rail as C11 — both land on same PCB trace
//  A22 (+12V constant):       Board power supply input, feeds LM2596 regulator
//  A32 (ECU ground):          PCB GND, chassis stud
// ============================================================

// Serial ports
#define TUNING_SERIAL             Serial    // USB Serial for monitor / tuning
#define TUNING_BAUD               115200
