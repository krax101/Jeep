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

MAP sensor:
  Map (+5V) ─── Teensy 5V (VUSB pad) or external 5V reg
  Map GND   ─── GND
  Map Signal ──[22kΩ]──┬── Teensy A1 (PIN_MAP) ──[100nF to GND]
                       [10kΩ to GND]

TPS:
  TPS +5V   ─── 5V supply
  TPS GND   ─── GND
  TPS Wiper ──[22kΩ]──┬── Teensy A0 (PIN_TPS)
                      [10kΩ to GND]

O2 Sensor (narrowband 0–1V output):
  O2 signal ──────────── Teensy A4 (PIN_O2)   [no divider needed, 0–1V safe]
  O2 heater ─── +12V (switched via heater relay after engine start)
  O2 GND    ─── chassis GND

Battery voltage:
  +12V rail ──[56kΩ]──┬── Teensy A5 (PIN_BATT)
                      [10kΩ to GND]
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
