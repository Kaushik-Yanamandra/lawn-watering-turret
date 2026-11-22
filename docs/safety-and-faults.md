# Safety and fault handling

The most important rule in this design: **the Raspberry Pi can ask for water, but it cannot hold the valve open by itself.** The valve logic lives on the ESP32 Turret Node, right next to the valve, so a crash or a cut cable can never leave water running.

The node watches pressure and flow while it works. When it detects a fault it closes the valve and latches a fault bit. Watering stays blocked until the cause is fixed and a `CLR` command is sent.

| Situation | What the node sees | What it does | Fault bit |
|---|---|---|---|
| **Normal dose** | Valve open, ~7.0 L/min, 25 psi at the regulator | Counts flow-meter pulses to the target millilitres, closes the valve, reports `EVT DOSE … DONE`. The Pi must send a valid frame at least once a second while the valve is open. | — |
| **Spigot left off** | Pressure under 10 psi and flow under 0.5 L/min with the valve open | Closes the valve within 4 s | `LOW_PSI` (4), `NO_FLOW` (2) |
| **Filter clogged / hose kinked** | Pressure may read fine, but flow stays under 0.5 L/min | Closes the valve within 4 s | `NO_FLOW` (2) |
| **Pi crashes / cable cut** | No valid frame from the Pi for 3 s | Closes the valve on its own within 3 s | `LINK` (1) |
| **Valve stuck open** | Flow over 0.3 L/min while the valve is commanded shut | Latches the fault so the Pi can alert you. The node cannot close a valve that is mechanically stuck: shut the spigot by hand, then clean or replace the valve. | `STUCK_OPEN` (8) |
| **Valve open too long** | Every open command carries a time limit, capped at 120 s | Closes the valve when the limit runs out | `VALVE_TIMEOUT` (16) |
| **Power cut** | — | Nothing needed: the valve is normally closed, so its spring shuts it | — |

## Boot behaviour

On boot the firmware drives the valve pin low before doing anything else, and parks the nozzle at −60° (pointing down, so the line drains). A 10 kΩ pull-down on the MOSFET gate keeps the valve shut even before the firmware starts.

## Electrical safety

- All outdoor wiring is 24 V DC or below, from a listed supply on a GFCI outlet.
- Each feed has a 4 A fuse at the supply.
- Each unit has an SS54 diode against reversed wiring and a TVS diode against surges on the long outdoor cable.
- The RS-485 lines have their own surge protection (SM712).
- Every cable enters an enclosure through an IP68 gland. No twist-on connectors outdoors.

## Water safety

- A vacuum breaker (ASSE 1011) on the spigot stops lawn water from being siphoned back into the house supply.
- A Y-filter (200 mesh) protects the valve and nozzle. Debris is the main cause of valve and nozzle trouble, so it gets flushed monthly.

## Acceptance tests

Before the system waters a lawn, the fault paths are tested with a bucket under the nozzle:
- **VT-11:** watchdog (link loss) and the 120 s valve limit
- **VT-12:** no-flow detection and stuck-valve detection
