# Bill of Materials — Renix Jeep 4.0 Standalone ECU Dev Board

## Microcontroller
| Qty | Part | Description | Notes |
|-----|------|-------------|-------|
| 1 | Teensy 4.1 | ARM Cortex-M7 @ 600 MHz, 8 MB Flash, 1 MB RAM | Primary MCU |
| 1 | USB-B micro cable | Programming / monitoring | |

## VR Sensor Conditioning (CPS + optional cam)
| Qty | Part | Description | Notes |
|-----|------|-------------|-------|
| 1 | MAX9926UAEE+ | Variable-reluctance sensor conditioner | SOIC-8; handles AC VR signal, outputs clean 3.3V digital |
| 1 | 100nF ceramic capacitor | Bypass cap for MAX9926 | |
| 1–2 | 10kΩ resistor (0805) | Termination | |

> **Alternative**: LM1815N (DIP-8, breadboard friendly) — same function.

## Injector Drivers (6 channels — ground-switching)
| Qty | Part | Description | Notes |
|-----|------|-------------|-------|
| 6 | IRLZ44NPBF | N-channel MOSFET, 55V 47A, logic-level gate (Vgs_th ≈ 1V) | TO-220; 3.3V gate drive from Teensy is sufficient |
| 6 | 1N4007 (or SB260) | Flyback diode, cathode to +12V rail | One per injector; SB260 is faster (Schottky) |
| 6 | 100Ω resistor | Gate resistor to limit inrush | |
| 6 | 10kΩ resistor | Gate pull-down to GND | Ensures OFF state if pin is floating |

## Ignition Coil Driver (1 channel to stock ICM)
| Qty | Part | Description | Notes |
|-----|------|-------------|-------|
| 1 | IRLZ44NPBF | Same MOSFET as injector drivers | |
| 1 | 1N4007 | Flyback diode | |
| 1 | 100Ω resistor | Gate resistor | |
| 1 | 10kΩ resistor | Gate pull-down | |

> The stock Renix ICM accepts a logic-level dwell signal. Connect MOSFET drain to the ICM's coil-negative input.

## IAC Stepper Motor Driver
| Qty | Part | Description | Notes |
|-----|------|-------------|-------|
| 1 | L298N module (or bare IC) | Dual H-bridge, 2A per channel | 4-wire full-step drive for IAC stepper |

## Power Supply
| Qty | Part | Description | Notes |
|-----|------|-------------|-------|
| 1 | LM2596S-5.0 | 5V switching regulator, 3A | For Teensy VIN; more efficient than 7805 |
| 1 | 100µF 25V electrolytic | Input cap | |
| 1 | 100µF 25V electrolytic | Output cap | |
| 1 | 1N5822 | Catch diode for LM2596 | |
| 1 | 68µH inductor | For LM2596 | |

> Teensy 4.1 can also be powered directly via USB during bench testing.

## Sensor Interface (analog signal conditioning)
| Qty | Part | Description | Notes |
|-----|------|-------------|-------|
| 2 | 2.2kΩ resistor (1/4 W) | CLT and IAT thermistor pullups to 3.3V | |
| 1 | 33kΩ resistor (1/4 W) | Series resistor — MAP signal to ADC | With 68kΩ gives ratio 0.673; scales 5V MAP to 3.3V |
| 1 | 68kΩ resistor (1/4 W) | Shunt to GND — MAP voltage divider | |
| 1 | 56kΩ resistor | Upper half of battery voltage divider | |
| 1 | 10kΩ resistor | Lower half of battery voltage divider | |
| 5 | 100nF ceramic | ADC input bypass/filter caps | One per analog input (TPS/MAP/CLT/IAT/KNOCK) |

> **TPS note:** TPS is powered from Teensy 3.3V — wiper connects directly to ADC pin A0. No voltage divider required.

## High-Side 12V Digital Input Dividers
| Qty | Part | Description | Notes |
|-----|------|-------------|-------|
| 4 | 100kΩ resistor (1/4 W) | Top half — 12V input dividers | One each for IGN_SW, START, P/N, A/C request |
| 4 | 22kΩ resistor (1/4 W) | Bottom half — 12V input dividers | Ratio 22/122=0.180; 12V → 2.16V (safe HIGH) |
| 1 | 10kΩ resistor | Pull-up for PS pressure switch (active-low) | 3.3V pull-up; switch closes to GND |
| 1 | 1kΩ resistor | Series for PS pressure switch | Current-limit |

## Knock Sensor Interface (hardware envelope detector)
| Qty | Part | Description | Notes |
|-----|------|-------------|-------|
| 1 | BAT46 Schottky diode (SOD-80 or DO-35) | Half-wave rectifier for knock envelope | Low forward voltage (~0.3V); fast recovery |
| 1 | 100nF ceramic capacitor | AC coupling cap — knock signal to BAT46 anode | Blocks DC; passes ~6.7 kHz knock AC |
| 1 | 10kΩ resistor (1/4 W) | RC discharge resistor at ADC pin | With 470nF: τ=4.7ms, fc=33Hz |
| 1 | 470nF ceramic/film capacitor | RC envelope hold capacitor | |
| 1 | Stock Renix knock sensor | Piezoelectric, block-mounted, M8 thread | ~6.7 kHz center; part # varies by year |

## Digital Dashboard
| Qty | Part | Description | Notes |
|-----|------|-------------|-------|
| 1 | ILI9341 TFT LCD module (2.4" or 2.8") | 320×240 colour display, SPI interface | Many clones available; 3.3V logic, 5V tolerant backlight |
| 1 | 10Ω resistor | Backlight LED current limiter | |
| 2 | 30 AWG wire (~2 cm) | Solder tails to Teensy bottom-pad SPI2 pins (42, 43) | Or use a Teensy 4.1 breakout board |

> Library: **ILI9341_t3** is included in Teensyduino — no separate install.
> Display shows RPM bar, CLT/MAP/VSS/BATT gauges, fuel mode, STFT/LTFT, knock retard, and fault codes at 10 Hz.

## Connectors / Wiring Terminals
| Qty | Part | Description |
|-----|------|-------------|
| 1 | 40-pin screw terminal block (or equiv.) | Main harness connector |
| 1 | 6-pin Deutsch DT series | Injector bank connector |
| 1 | Fuse holder + 20A fuse | Main ECU power |
| 1 | Fuse holder + 5A fuse | Sensor reference voltage |

## Miscellaneous
| Qty | Part | Description |
|-----|------|-------------|
| 1 | 10µF 16V ceramic (or tantalum) | 3.3V AREF decoupling on Teensy |
| 1 | Protoboard or custom PCB | For component mounting |
| 1 | DIN rail or project box | Enclosure for under-hood use |
| — | 18 AWG wire (various colours) | Wiring harness |
| — | Weatherpack connector kit | Weather-resistant connections |

## Recommended Sensors (if replacing worn Renix originals)
| Part | Description |
|------|-------------|
| GM ACDelco 213-796 | 1-bar MAP sensor (0.5–4.5V, 0–104 kPa) |
| GM CLT sensor (#15-50491) | NTC thermistor, same curve as Renix |
| Bosch 0 280 130 026 | TPS potentiometer, 0–5V |
| Bosch 0 258 003 023 | Narrowband O2 sensor (3-wire heated) |

## Total Estimated Cost
| Category | Approx. Cost (USD) |
|----------|--------------------|
| Teensy 4.1 | $30 |
| MOSFET drivers + passives | $15 |
| MAX9926 conditioner | $8 |
| L298N IAC driver | $5 |
| Power supply components | $10 |
| Knock envelope detector (BAT46 + RC) | $2 |
| High-side input divider resistors | $1 |
| ILI9341 2.4" TFT display | $8 |
| Connectors / enclosure | $20 |
| **Total** | **~$99** |
