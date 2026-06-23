# Wiring Guide — Renix Jeep 4.0 Standalone ECU

## Power & Ground
```
Battery (+12V) ──[20A fuse]──┬── Injector high-side rail (+12V)
                             ├── IAC motor supply (+12V on L298N)
                             ├── Ignition coil relay / ICM +12V
                             └── LM2596 input (+12V)

LM2596 output (5V) ──────────── Teensy 4.1 VIN pin

Teensy GND  ─────────────────┬── LM2596 GND
                             ├── MOSFET source pins (all 7)
                             ├── Sensor ground buss
                             └── L298N GND
```

## CPS (Crankshaft Position Sensor) — Variable Reluctance, 2-wire
```
CPS (+) ──── MAX9926 IN+
CPS (-) ──── MAX9926 IN-
MAX9926 OUT ─────────────────── Teensy PIN_CPS_IN (pin 2)
MAX9926 VCC ─────────────────── Teensy 3.3V
MAX9926 GND ─────────────────── GND
```
> Wire 100nF cap from IN+ and IN- to GND (EMI suppression near engine).
> The MAX9926 auto-adjusts threshold for any VR amplitude.

## CAM Sensor (Distributor Stator) — Magnetic Pickup, 2-wire
```
Stator (+) ── MAX9926 IN+  (or second MAX9926 / LM1815)
Stator (-) ── MAX9926 IN-
MAX9926 OUT ─────────────────── Teensy PIN_CAM_IN (pin 3)
```

## Fuel Injectors — High-Impedance 16Ω, Ground-Switching
**Each channel:**
```
+12V rail ──────────────────── Injector (+) connector
Injector (-) connector ──────── MOSFET drain
MOSFET source ───────────────── GND
Teensy INJ pin ─[100Ω]──────── MOSFET gate
                [10kΩ to GND]  (pull-down)
1N4007 cathode ──────────────── +12V rail
1N4007 anode ────────────────── Injector (-) / MOSFET drain
```

Teensy to Injector mapping:
| Teensy Pin | Config Const | Cylinder |
|------------|--------------|---------|
| 5  | PIN_INJ_1 | Cyl 1 |
| 6  | PIN_INJ_2 | Cyl 5 |
| 7  | PIN_INJ_3 | Cyl 3 |
| 8  | PIN_INJ_4 | Cyl 6 |
| 9  | PIN_INJ_5 | Cyl 2 |
| 10 | PIN_INJ_6 | Cyl 4 |

> **Note:** Stock Renix ECU is power-switching (supplies +12V). This design
> uses industry-standard ground-switching, which requires re-pinning the
> injector harness connectors so that one injector terminal connects to
> the constant +12V rail and the other to the MOSFET drain.

## Ignition Output (to stock Renix ICM)
```
Teensy PIN_IGN_COIL (pin 11) ─[100Ω]─ MOSFET gate
                               [10kΩ to GND]
MOSFET drain ─────────────────────────── ICM coil input
MOSFET source ────────────────────────── GND
1N4007 across coil driver input (if needed)
```
The stock ICM manages coil current limiting. This ECU only provides the
dwell-start (HIGH) and fire (LOW) trigger signal.

## IAC Stepper Motor (4-wire)
```
Teensy PIN_IAC_A_POS (24) ─── L298N IN1
Teensy PIN_IAC_A_NEG (25) ─── L298N IN2
Teensy PIN_IAC_B_POS (26) ─── L298N IN3
Teensy PIN_IAC_B_NEG (27) ─── L298N IN4

L298N OUT1 ─── IAC wire A+
L298N OUT2 ─── IAC wire A-
L298N OUT3 ─── IAC wire B+
L298N OUT4 ─── IAC wire B-
L298N VCC  ─── +12V
L298N GND  ─── GND
```
> The stock Renix IAC pinout (connector C1 on throttle body):
> Pin 1 = A, Pin 2 = B, Pin 3 = C, Pin 4 = D (verify with DMM continuity test).

## Analog Sensors (3.3V ADC)
```
Teensy 3.3V ──[2.2kΩ]──┬── Teensy A2 (PIN_CLT) ──[100nF to GND]
                        └── CLT sensor terminal (other terminal to GND)

Teensy 3.3V ──[2.2kΩ]──┬── Teensy A3 (PIN_IAT) ──[100nF to GND]
                        └── IAT sensor terminal (other terminal to GND)

MAP sensor (GM 1-bar, 5V supply, 0.48–4.5V output):
  Map (+5V) ─── 5V reg output (LM2596S-5.0 or separate 5V supply)
  Map GND   ─── GND
  Map Signal ──[33kΩ]──┬── Teensy A1 (PIN_MAP) ──[100nF to GND]
                       [68kΩ to GND]
  Divider ratio = 68/(33+68) = 0.673.
  At 0.48V: ADC 401.  At 4.5V: ADC 3759.  Max pin voltage = 3.23V (safe).

TPS (ratiometric potentiometer, 3.3V supply):
  TPS +    ─── Teensy 3.3V rail  (NOT 5V — ratiometric on 3.3V)
  TPS GND  ─── GND
  TPS Wiper ─────────── Teensy A0 (PIN_TPS) ──[100nF to GND]
  No voltage divider needed; wiper stays within 0–3.3V.
  ADC_CLOSED ≈ 500 (~0.4V), ADC_OPEN ≈ 3876 (~3.1V).

O2 Sensor (narrowband 0–1V output):
  O2 signal ──────────── Teensy A4 (PIN_O2)   [no divider needed, 0–1V safe]
  O2 heater ─── +12V (switched via heater relay after engine start)
  O2 GND    ─── chassis GND

Battery voltage:
  +12V rail ──[56kΩ]──┬── Teensy A5 (PIN_BATT)
                      [10kΩ to GND]

Knock sensor (hardware envelope detector — no ISR needed):
  Knock sensor signal ──[100nF]──┬── BAT46 Schottky (anode)
                                 └── (AC coupling cap)
  BAT46 cathode ─────────────────┬── Teensy A6 (PIN_KNOCK)
                                 ├── [10kΩ to GND]   ← RC discharge
                                 └── [470nF to GND]  ← RC envelope, τ=4.7ms
  No DC bias resistors needed. Envelope output rests at 0V; peaks to ~1.5V on knock.
  Sampled at 50 Hz from main sensor loop.
```

## High-Side 12V Digital Inputs
All switched 12V inputs (IGN_SW, START_SIGNAL, PARK_NEUTRAL, AC_REQUEST) use
a 100kΩ + 22kΩ resistor divider to bring 12V logic down to Teensy 3.3V I/O:
```
12V input ──[100kΩ]──┬── Teensy digital pin
                     [22kΩ to GND]
  Ratio = 22/122 = 0.180.  At 12V: 2.16V → HIGH.  At 0V: 0V → LOW.
```
Power steering pressure switch (PIN_PS_PRESSURE, pin 38):
```
  PS switch ──[1kΩ]── Teensy pin 38
  Teensy pin 38 ──[10kΩ to 3.3V] (active-low: LOW = high PS load)
```

## Auxiliary Outputs
| Teensy Pin | Function | Load |
|------------|----------|------|
| 28 | Fuel pump relay (PIN_FUEL_PUMP_RELAY) | Relay coil via flyback diode |
| 29 | Tach output (PIN_TACH_OUT) | 12V tach gauge via 2kΩ + NPN transistor |
| 30 | Check Engine Light (PIN_CEL) | LED + 470Ω to +5V |
| 31 | EGR solenoid (PIN_EGR) | MOSFET driver |
| 32 | Charcoal purge solenoid (PIN_PURGE) | MOSFET driver |
| 33 | Radiator fan relay (PIN_FAN_RELAY) | Relay coil |

## Trigger Wheel Upgrade (36-1 wheel on harmonic balancer)
Installing a 36-1 missing-tooth wheel on the harmonic balancer is the
recommended upgrade for this ECU. The stock Renix flywheel trigger pattern
can be decoded in software (see `TRIGGER_MODE_RENIX_44` in config.h), but
the 36-1 wheel gives 10° resolution on the harmonic balancer (easier access,
less vibration than flywheel).

**Procedure:**
1. Source a 36-1 wheel that fits the stock 4.0 harmonic balancer (Jeep uses
   ~5.125" diameter; several suppliers make bolt-on rings).
2. Mount the magnetic pickup (e.g., GM crank sensor from a late 90s LT1)
   with a 0.040"–0.060" air gap to the wheel teeth.
3. Set `TRIGGER_SYNC_ANGLE_BTDC` in config.h to match where the missing gap
   aligns relative to cylinder 1 TDC (use a timing light to verify after first start).

## Complete Renix Harness Connector Pin Audit

Every pin on the stock Renix ECU 60-pin connector (two plugs: A-side and C/D-side)
is accounted for below. "FIRMWARE" = active GPIO in code. "HARDWARE" = PCB trace
with no firmware assignment required. "N/C" = confirmed unused per Chrysler FSM.

### A-Side Connector (main power / switched outputs)
| Pin | Signal | This Design | Justification |
|-----|--------|-------------|---------------|
| A2  | Ignition switch 12V | FIRMWARE — PIN_IGN_SW (34) | Key-on detection; latch relay logic |
| A9  | ECU self-hold relay | FIRMWARE — PIN_LATCH_RELAY (41) | Post key-off IAC park + LTFT save |
| A10 | EGR solenoid / Purge | FIRMWARE — PIN_EGR (31), PIN_PURGE (32) | Active outputs |
| A11 | Upshift indicator | FIRMWARE — PIN_UPSHIFT_LIGHT (12) | MT economy lamp |
| A22 | +12V constant | HARDWARE — Board Vin via 20A fuse | Powers LM2596; no GPIO |
| A32 | ECU GND | HARDWARE — PCB GND pour | Chassis stud connection |

### C-Side Connector (sensors / injectors / triggers)
| Pin | Signal | This Design | Justification |
|-----|--------|-------------|---------------|
| C1  | CPS + | HARDWARE — MAX9926 IN+ | VR conditioner handles signal; OUT → PIN_CPS_IN (2) |
| C2  | CPS − | HARDWARE — MAX9926 IN− | See C1 |
| C3  | Starter signal | FIRMWARE — PIN_START_SIGNAL (35) | Early cranking detection |
| C4  | Park/Neutral sw | FIRMWARE — PIN_PARK_NEUTRAL (36) | Drive idle bump; AT only |
| C5  | Cam sensor − | HARDWARE — PCB GND | Shield return for cam stator pair |
| C6  | MAP sensor signal | FIRMWARE — PIN_MAP (A1) | ADC with voltage divider |
| C7  | TPS signal | FIRMWARE — PIN_TPS (A0) | ADC with voltage divider |
| C8  | IAT sensor | FIRMWARE — PIN_IAT (A3) | NTC with 2.2 kΩ pullup |
| C9  | Factory unused | N/C | Confirmed unused in FSM |
| C10 | CLT sensor | FIRMWARE — PIN_CLT (A2) | NTC with 2.2 kΩ pullup |
| C11 | Injector +12V | HARDWARE — +12V rail | Hardwired high-side; ECU only switches low-side |
| C12 | Diagnostic TX | REPLACED — USB Serial | USB at 115200 baud is a superset of factory K-line |
| C13 | Factory unused | N/C | Confirmed unused in FSM |
| C14 | MAP sensor +5V | HARDWARE — 5V reg output | Sensor supply; no GPIO |
| C15 | TPS +5V | REPLACED — Teensy 3.3V rail | TPS is ratiometric; 3.3V supply = no divider needed |
| C16 | Cam sensor + | HARDWARE — MAX9926 IN+ (2nd) | OUT → PIN_CAM_IN (3) |

### D-Side Connector (fuel injectors / ignition / O2 / misc)
| Pin | Signal | This Design | Justification |
|-----|--------|-------------|---------------|
| D1  | CPS signal (conditioned) | FIRMWARE — PIN_CPS_IN (2) | Interrupt-driven tooth ISR |
| D2  | Diagnostic GND | HARDWARE — PCB GND | Serial reference; no GPIO |
| D3  | Sensor GND | HARDWARE — PCB GND | Separate pour from power GND |
| D4  | Inj 1 (Cyl 1) | FIRMWARE — PIN_INJ_1 (5) | MOSFET low-side driver |
| D5  | Inj 2 (Cyl 5) | FIRMWARE — PIN_INJ_2 (6) | MOSFET low-side driver |
| D6  | Inj 3 (Cyl 3) | FIRMWARE — PIN_INJ_3 (7) | MOSFET low-side driver |
| D7  | Inj 4 (Cyl 6) | FIRMWARE — PIN_INJ_4 (8) | MOSFET low-side driver |
| D8  | Knock sensor | FIRMWARE — PIN_KNOCK (A6) | Hardware envelope detector; see Knock section below |
| D9  | O2 sensor | FIRMWARE — PIN_O2 (A4) | Narrowband, direct to ADC |
| D10 | Injector +12V | HARDWARE — same +12V rail as C11 | Both pins land on same PCB trace |
| D11 | Inj 5 (Cyl 2) | FIRMWARE — PIN_INJ_5 (9) | MOSFET low-side driver |
| D12 | Inj 6 (Cyl 4) | FIRMWARE — PIN_INJ_6 (10) | MOSFET low-side driver |
| D13 | Ignition coil | FIRMWARE — PIN_IGN_COIL (11) | Dwell/fire trigger to stock ICM |

### Additional This-Design Pins (no stock Renix equivalent — new capability)
| Teensy Pin | Signal | Function |
|------------|--------|---------|
| 3  | PIN_CAM_IN | 720° cam sync from distributor stator |
| 4  | PIN_VSS_IN | Vehicle speed sensor (8 pulses/rev) |
| 24–27 | PIN_IAC_A/B | 4-wire stepper IAC via L298N |
| 28 | PIN_FUEL_PUMP_RELAY | Fuel pump prime + run relay |
| 29 | PIN_TACH_OUT | Tachometer signal (NPN buffer) |
| 30 | PIN_CEL | MIL / Check Engine Light |
| 33 | PIN_FAN_RELAY | Radiator cooling fan relay |
| 37 | PIN_AC_REQUEST | A/C thermostat request input |
| 38 | PIN_PS_PRESSURE | Power steering pressure switch |
| 39 | PIN_AC_CLUTCH | A/C compressor clutch relay output |
| 40 | PIN_O2_HEATER | O2 sensor heater relay (delayed) |
| 21 | PIN_DASH_RST | ILI9341 display reset |
| 22 | PIN_DASH_CS  | ILI9341 chip-select |
| 23 | PIN_DASH_DC  | ILI9341 data/command |
| 42 | PIN_DASH_SCK | ILI9341 SPI2 clock (bottom pad) |
| 43 | PIN_DASH_MOSI| ILI9341 SPI2 data  (bottom pad) |

## Digital Dash Wiring (ILI9341 TFT, 320×240)
```
ILI9341 VCC  ─── Teensy 3.3V
ILI9341 GND  ─── GND
ILI9341 CS   ─── Teensy pin 22 (PIN_DASH_CS)
ILI9341 RESET─── Teensy pin 21 (PIN_DASH_RST)
ILI9341 DC   ─── Teensy pin 23 (PIN_DASH_DC)
ILI9341 SDI  ─── Teensy pin 43 (SPI2 MOSI — solder to bottom pad)
ILI9341 SCK  ─── Teensy pin 42 (SPI2 SCK  — solder to bottom pad)
ILI9341 LED  ─── 3.3V through 10Ω resistor (backlight)
ILI9341 SDO  ─── not connected (read-back not used)
```
> Pins 42/43 are on the underside SOIC-style pads of the Teensy 4.1.
> Solder 30 AWG wire directly to the pads, or use a breakout daughter board.
> The ILI9341_t3 library auto-selects SPI2 when MOSI=43, SCK=42 are passed.

---

## Notes on Stock Renix Known Issues — What This ECU Fixes
| Stock Weakness | This Design |
|----------------|-------------|
| No fault codes | Full OBD-style fault detection via `diag_update()` |
| CPS is most common failure | MAX9926 conditioner handles wider amplitude range; CPS loss fault with MIL |
| ECU burns injector traces | Ground-switching MOSFET drivers with flyback protection |
| No data logging | USB serial monitor stream at 115200 baud |
| Not tunable | Full VE/timing tables editable via serial command interface |
| Poor cold-start | Tunable cranking enrichment, after-start, IAT correction tables |
| Batch fire injection | Sequential injection when distributor cam sync is present |
