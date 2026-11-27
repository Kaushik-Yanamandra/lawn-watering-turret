# Safety and fault handling

The most important rule in this design: **the Raspberry Pi can ask for water, but it cannot hold a valve open by itself.** The valve logic lives on the ESP32 Turret Node, right next to the valves, so a crash or a cut cable can never leave water running.

Rev B adds a second line of defence: a normally closed **master valve** upstream of the zone valve. Both open together for a dose (master 50 ms first) and close together (zone first). If the zone valve ever sticks open, the master valve still shuts the line.

The node watches pressure and flow while it works. When it detects a fault it closes both valves and latches a fault bit. Watering stays blocked until the cause is fixed and a `CLR` command is sent. Before each session the Pi reads the fault bits, sends an alert, and clears the ones that are safe to clear.

| Situation | What the node sees | What it does | Fault bit |
|---|---|---|---|
| **Normal dose** | Valves open, ~7 L/min | Counts flow-meter pulses to the target millilitres and closes the valves. The Pi polls progress with `DOSE?` and must send a valid frame at least once every 3 s. | — |
| **Spigot left off** | Pressure under 10 psi after 4 s with the valves open | Closes both valves | `LOW_PSI` (4) |
| **Filter clogged / hose kinked** | Flow under 0.5 L/min after 4 s | Closes both valves | `NO_FLOW` (2) |
| **Tube off the barb / burst hose** | Flow over 1.6 × nominal after 2 s | Closes both valves | `HIGH_FLOW` (32) |
| **Pi crashes / cable cut** | No valid frame for 3 s | Closes both valves on its own | `LINK` (1) |
| **Valve stuck open** | 20 or more flow pulses (about 45 mL) in a 30 s window with both valves shut | Keeps the master valve shut and latches the fault in flash, so it survives a reboot. Only `CLR STUCK`, sent after a person has checked the valve, clears it. If water still flows, the leak is through both valves: shut the spigot by hand. | `STUCK_OPEN` (8) |
| **Valve open too long** | Every open command carries a time limit, capped at 120 s | Closes the valves when the limit runs out | `VALVE_TIMEOUT` (16) |
| **Firmware hangs** | The task watchdog is not fed for 4 s | Resets the chip; the gate pull-downs hold both valves shut through the reset | — |
| **Power cut** | — | Nothing needed: both valves are normally closed, so their springs shut them | — |

## Session stop rules (Pi side)

The Pi also stops a session, closes the valves and sends an alert when:

- a dose ends short (more than 25% under target) or by timeout, which usually means a meter or nozzle problem;
- the nozzle pressure measured on the first check shot is under 10 psi;
- the camera misses 4 check shots in a row, or 3 in a row land far from where the model expects;
- the wind estimated from check shots goes over 4 m/s.

## Boot behaviour

On boot the firmware drives both valve pins low before doing anything else, restores any latched `STUCK_OPEN` fault from flash, and parks the nozzle at −45° (pointing down, so the line drains). A 10 kΩ pull-down on each MOSFET gate keeps the valves shut even before the firmware starts.

## Electrical safety

- All outdoor wiring is 24 V DC or below, from an outdoor-rated supply marked Class 2, on a GFCI outlet.
- Each feed has a 4 A fuse at the supply.
- Each unit has an SS54 diode against reversed wiring and a TVS diode against surges on the long outdoor cable. Buck converters are rated for at least 60 V input.
- The RS-485 lines have their own surge protection (SM712); the flow input has a 1 kΩ resistor and a BAT54S clamp.
- Every cable enters an enclosure through an IP68 gland. No twist-on connectors outdoors.

## Water safety

- An ASSE 1020 pressure vacuum breaker, at least 1.35 m above the highest point it serves, stops lawn water being siphoned back into the house supply. (Rev A used an ASSE 1011 hose-bib breaker, which is not rated for a line held under pressure behind a valve.) Check local code before installing.
- A Y-filter (200 mesh) protects the valves and nozzle. Debris is the main cause of valve and nozzle trouble, so it gets flushed monthly.
- A water-hammer arrestor at the valves softens the pressure spike when they close; a drain cock at the low point empties the line before a freeze.

## Acceptance tests

Before the system waters a lawn, the fault paths are tested with a bucket under the nozzle:
- **VT-11:** link watchdog, the 120 s valve limit and the task watchdog
- **VT-12:** no-flow, high-flow and stuck-valve detection (hold the zone valve open by hand and check that the master valve shuts and the fault survives a power cycle)
