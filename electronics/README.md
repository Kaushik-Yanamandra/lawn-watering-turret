# Electronics

Full drawings: [Turret Node schematic (LWT-E1)](../hardware/drawings/LWT-E1_turret-node-schematic.svg) · [Wiring harness (LWT-E2)](../hardware/drawings/LWT-E2_wiring-harness.svg)

> **Rev B:** the tables below are Rev B. The E1 schematic sheet is still Rev A: it does not yet show the master valve driver, the flow-input clamp, the RS-485 pull resistors or the larger servo buck. See [design-review-rev-b.md](../docs/design-review-rev-b.md).

## Turret Node (ESP32)

| Circuit | Design | Notes |
|---|---|---|
| **Power in and protection** | 24 V arrives on the 16/2 cable. A series SS54 Schottky blocks reversed wiring; an SMBJ28A TVS clamps surges picked up on the long outdoor run. | 4 A blade fuse at the supply end. Drop is 0.37 V per 10 m at 2.8 A peak, so runs up to 30 m are fine. |
| **Servo rail (6 V)** | Buck A makes 6.0 V (8–10 A, rated ≥ 60 V input) for the two head servos. A 2200 µF low-ESR capacitor absorbs the 2–3 A spikes when they start moving. | 6 W typical, up to ~50 W if both stall (rare). Rev A's 5 A buck could not supply two stalled servos. |
| **Logic rail (5 V)** | Buck B (2 A) is separate from the servo rail so servo noise cannot reset the ESP32. Feeds the DevKit, flow meter and pressure transducer. | ~1.5 W. The DevKit's regulator gives 3.3 V for the MAX3485 and soil probes. |
| **Valve drivers (×2)** | One IRLZ44N logic-level MOSFET on the low side of each coil: zone valve on GPIO27, master valve on GPIO13. Each has a 100 Ω series gate resistor, a 10 kΩ pull-down that keeps the valve shut while the ESP32 boots or resets, and a 1N5819 across the coil to absorb the turn-off spike. | Two 24 V normally closed assisted-lift valves, ~5 W each. Master opens 50 ms before the zone valve and closes 50 ms after it. |
| **Flow meter** | Hall-effect meter at 5 V; a 10k/20k divider turns its 5 V pulses into 3.3 V, then a 1 kΩ series resistor and a BAT54S clamp to 3.3 V and ground protect GPIO33. Pulses are counted in an interrupt (with a 2 ms glitch filter), so dosing does not depend on the Pi. | ~450 pulses/L, calibrated in test VT-05 |
| **Pressure transducer** | 0–30 psi in, 0.5–4.5 V out, fitted downstream of the zone valve so it reads nozzle pressure. A 4.7k/10k divider brings 4.5 V down to 3.06 V for GPIO34. | Under 10 psi with the valves open for 4 s → fault. The Pi also uses the reading to rescale the aiming model. |
| **Soil probes** | Three capacitive probes at 3.3 V; outputs stay inside 1.2–2.8 V, so they go straight to GPIO35, 32 and 39, each with 100 nF to ground. | Shielded 22 AWG cable, keep under 10 m |
| **RS-485 link** | MAX3485 talks to the Pi over one twisted pair of the Cat6 at 115200 baud. The ESP32 UART drives DE in hardware RS-485 mode (GPIO4). A pull-up on RO and a pull-down on DE keep the bus quiet during resets. The Pi is the only master; the node only ever transmits in reply. If no valid frame arrives for 3 s while a valve is open, the node shuts both. | 120 Ω termination at both ends; SM712 TVS on A/B. RJ45 pin 5 A, pin 4 B, pins 7/8 ground. |
| **Supply monitor** | A 100k/12k divider lets GPIO36 read the 24 V bus, so the log shows supply sag and brown-outs. | Reported in the `STAT` reply |

## Sensor Unit (Raspberry Pi 5)

| Connection | Detail |
|---|---|
| Power | 24 V in → fuse → SS54 → TVS → 24→5.1 V 5 A buck (≥ 60 V input) → Pi 5 GPIO 5 V pins. Set `PSU_MAX_CURRENT=5000` in the Pi EEPROM config. |
| Visible camera | Camera Module 3 Wide on the CSI port |
| Thermal camera | MLX90640 on I2C1: GPIO2 SDA, GPIO3 SCL, 3.3 V |
| Thermal pan servo | MG90S signal on GPIO18 (hardware PWM), on its own small 5 V buck (Rev B) so it cannot brown out the Pi, 470 µF capacitor |
| RS-485 | FTDI USB–RS-485 adapter in a Pi USB port → IP67 RJ45 bulkhead, 120 Ω termination |

## Command link (Pi → Turret Node), Rev B

Every frame, in both directions, is `<TEXT>*<HH>` followed by a newline, where `HH` is the XOR of all the bytes of `<TEXT>` as two upper-case hex digits. The node only transmits in reply to a command. The full protocol is at the top of [`turret_node.ino`](../firmware/turret_node/turret_node.ino).

| Command | Reply | Notes |
|---|---|---|
| `PING` | `OK PONG <fw>` | Also keeps the link watchdog fed |
| `MOVE <pan> <tilt>` | `OK MOVE <eta_ms>` | Degrees; always approached from below to take up backlash |
| `VALVE 1 <max_ms>` / `VALVE 0` | `OK VALVE 1` / `OK VALVE 0` | Timed open (max 120 s) for check shots |
| `DOSE <id> <ml> <max_ms>` | `OK DOSE <id>` | A repeated ID is acknowledged but does not restart the dose |
| `DOSE?` | `OK DOSEST <id> <RUN\|END> <ml> <ms> <reason>` | Reason is `DONE`, `TIMEOUT`, `FAULT`, `STOP` or `PARK` |
| `STAT` | `OK STAT <pan> <tilt> <valve> <lpm> <ml_total> <psi> <vin> <p1> <p2> <p3> <faults> <uptime_s>` | The Pi spots a node reboot when uptime goes down |
| `CFG <key> <value>` | `OK CFG`, `ERR KEY` or `ERR RANGE` | Servo centres and scales, flow and pressure constants; every value range-checked |
| `CLR` / `CLR STUCK` | `OK CLR` | `CLR` leaves `STUCK_OPEN` latched; `CLR STUCK` clears it after a person has checked the valve |
| `PARK` | `OK PARK` | Valves shut, nozzle down to −45° to drain |

Fault bits: 1 `LINK`, 2 `NO_FLOW`, 4 `LOW_PSI`, 8 `STUCK_OPEN`, 16 `VALVE_TIMEOUT`, 32 `HIGH_FLOW`.
