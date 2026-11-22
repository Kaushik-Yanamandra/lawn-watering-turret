# Electronics

Full drawings: [Turret Node schematic (LWT-E1)](../hardware/drawings/LWT-E1_turret-node-schematic.svg) · [Wiring harness (LWT-E2)](../hardware/drawings/LWT-E2_wiring-harness.svg)

## Turret Node (ESP32)

| Circuit | Design | Notes |
|---|---|---|
| **Power in and protection** | 24 V arrives on the 16/2 cable. A series SS54 Schottky blocks reversed wiring; an SMBJ28A TVS clamps surges picked up on the long outdoor run. | 4 A blade fuse at the supply end. Drop is 0.37 V per 10 m at 2.8 A peak, so runs up to 30 m are fine. |
| **Servo rail (6 V)** | Buck A makes 6.0 V (5 A) for the two head servos. A 2200 µF low-ESR capacitor absorbs the 2–3 A spikes when they start moving. | 6 W typical, 36 W if both stall (rare) |
| **Logic rail (5 V)** | Buck B (2 A) is separate from the servo rail so servo noise cannot reset the ESP32. Feeds the DevKit, flow meter and pressure transducer. | ~1.5 W. The DevKit's regulator gives 3.3 V for the MAX3485 and soil probes. |
| **Valve driver** | IRLZ44N logic-level MOSFET on the low side of the coil. 100 Ω series gate resistor, 10 kΩ pull-down keeps the valve shut while the ESP32 boots, 1N5819 across the coil absorbs the turn-off spike. | 24 V normally-closed valve, ~5 W |
| **Flow meter** | Hall-effect meter at 5 V; a 10k/20k divider turns its 5 V pulses into 3.3 V for GPIO33. Pulses are counted in an interrupt, so dosing does not depend on the Pi. | ~450 pulses/L, calibrated in test VT-05 |
| **Pressure transducer** | 0–100 psi in, 0.5–4.5 V out. A 4.7k/10k divider brings 4.5 V down to 3.06 V for GPIO34. | Under 10 psi with the valve open for 4 s → fault |
| **Soil probes** | Three capacitive probes at 3.3 V; outputs stay inside 1.2–2.8 V, so they go straight to GPIO35, 32 and 39, each with 100 nF to ground. | Shielded 22 AWG cable, keep under 10 m |
| **RS-485 link** | MAX3485 talks to the Pi over one twisted pair of the Cat6 at 115200 baud. The Pi is the only master. If no valid frame arrives for 3 s while the valve is open, the node shuts it. | 120 Ω termination at both ends; SM712 TVS on A/B. RJ45 pin 5 A, pin 4 B, pins 7/8 ground. |
| **Supply monitor** | A 100k/12k divider lets GPIO36 read the 24 V bus, so the log shows supply sag and brown-outs. | Reported in the `STAT` reply |

## Sensor Unit (Raspberry Pi 5)

| Connection | Detail |
|---|---|
| Power | 24 V in → fuse → SS54 → TVS → 24→5.1 V 5 A buck → Pi 5 GPIO 5 V pins |
| Visible camera | Camera Module 3 Wide on the CSI port |
| Thermal camera | MLX90640 on I2C1: GPIO2 SDA, GPIO3 SCL, 3.3 V |
| Thermal pan servo | MG90S signal on GPIO18 (hardware PWM), 5.1 V rail, 470 µF capacitor |
| RS-485 | FTDI USB–RS-485 adapter in a Pi USB port → IP67 RJ45 bulkhead, 120 Ω termination |

## Command link (Pi → Turret Node)

Commands used during build and commissioning include `PING`, `STAT`, `MOVE <pan> <tilt>`, `VALVE`, `DOSE`, `CFG` and `CLR`. The full protocol will be documented with the firmware.
