# Design review and Rev B changes

In November 2026 the Rev A design was put through a devil's-advocate review: every part of it (software, firmware, mechanics, water, electrics and sensing) was examined on the assumption that it was wrong until shown otherwise. The review found 19 problems. Seven would have stopped the system working or caused damage; twelve would have made it worse than claimed. Rev B fixes all of them.

## Findings

| Rank | Finding in Rev A | Consequence | Rev B fix |
|---|---|---|---|
| 1 | Watering started at 05:30, before sunrise most of the year | Camera cannot see check shots, so wind correction never runs | Start at sunrise + 15 min (NOAA sunrise equations); skip if later than sunrise + 2 h |
| 2 | Firmware read the clock before polling the serial port, with unsigned time comparisons | Valve timers could misfire: a spurious close, or a missed close at the 49-day `millis()` wrap | Clock read after the poll; signed, clamped time differences; task watchdog |
| 3 | A single zone valve, with flow checked only while open | A valve stuck open floods the lawn unseen | Normally closed master valve upstream; leak check while closed; `STUCK_OPEN` latched in flash across reboots |
| 4 | A retried `DOSE` command restarted the dose | A lost reply could double the water to a cell | Dose IDs: a repeated ID is acknowledged, not restarted; `DOSE?` polls progress |
| 5 | Node and Pi could both transmit on the half-duplex bus | Collisions and corrupt frames | Respond-only protocol, checksum on every frame, hardware RS-485 mode on the ESP32 UART |
| 6 | Dose results were ignored and faults were never cleared | A clogged nozzle looked like success; one fault blocked every later session | Every dose checked against the meter (within 25%); faults read and cleared before each session; alerts sent |
| 7 | Jet could be aimed below −50° tilt | Jet strikes the head and soaks the electronics | Tilt limited to −45° in firmware and in the aim solver; minimum reach is 1.0 m |
| 8 | Yesterday's plan used without checks | Watering after overnight rain, or from a stale plan | Plan rejected after 20 h; skip on 3 mm of rain since the scan or 6 mm forecast |
| 9 | Check-shot frames taken while the stream was still running | Stream and splash merged; wrong landing points | Frame taken 0.7 s after the valve closes; check shots only on dry turf; outlier gate; stop after 4 misses in a row |
| 10 | Valve opened once per aim point | About 200 valve cycles a session, each with a water-hammer pulse | One opening per cell; the head moves while the water runs |
| 11 | Nozzle pressure assumed constant | Reach falls about 1.9 m if pressure drops from 25 to 15 psi | Pressure read on the first check shot; jet speed rescaled; stop below 10 psi |
| 12 | No lens model | Wide-lens barrel distortion puts edge pixels at the wrong ground point | Checkerboard lens calibration (`software/tools/calibrate_lens.py`); every pixel undistorted |
| 13 | Thermal score taken at one pixel per cell | Edge cells scored from the fence or path | Four samples inside each cell; partly visible cells marked unknown |
| 14 | A uniform lawn still triggered watering | Water spent on sensor noise | Scan rejected below 0.8 °C spread; without probes, only cells 1.5 °C above median get water |
| 15 | Simulator had no servo or pressure noise | Accuracy claims of 2–4 cm were not achievable | 0.25° servo error and 0.6% jet-speed scatter added; accuracy target restated |
| 16 | P5 insert pilots broke through the 5 mm plate | Inserts pull out; water reaches the enclosure | 11 mm bosses behind each insert; blind pilot with a 3 mm floor |
| 17 | P2 hose pass-through sat over the bearing race | Grit and water fed into the bearing | Hole removed; tube and servo lead run in a service loop on zip-tie holes |
| 18 | Pressure transducer upstream of the valve | Read supply pressure, not nozzle pressure | 0–30 psi transducer moved downstream of the zone valve |
| 19 | Backflow device rated for hose bibs only | Not rated for a line held under pressure behind a valve | ASSE 1020 pressure vacuum breaker at least 1.35 m above grade (check local code) |

## Accuracy before and after honest noise

| Case | Rev A simulation (no servo or pressure noise) | Rev B simulation |
|---|---|---|
| Calibrated, still air | 0.03 m median | 0.05 m median, 90% within 0.11 m |
| 2.5 m/s crosswind, camera correction | 0.17 m median | 0.28 m median, 90% within 0.67 m |
| 2.5 m/s crosswind, no correction | 0.62 m median | 0.56 m median, 90% within 1.03 m |

Sensitivity at 8 m range: 0.25° of tilt error moves the landing about 0.04 m; 0.6% of jet speed moves it about 0.04 m. Air drag matters most for reach: each 0.02 per metre change in the drag constant moves maximum reach by about 0.8 m, which is why the lawn is planned to 8.5 m until calibration measures it.

## Hardware changes

| Item | Rev A | Rev B | Why |
|---|---|---|---|
| Zone valve | Direct 24 V solenoid | Assisted-lift (zero-differential) 24 V DC valve, Cv ≥ 3 | Opens at low supply pressure without starving the nozzle |
| Master valve | None | Second normally closed 24 V valve upstream, on GPIO13 through its own MOSFET | Shuts the line if the zone valve sticks |
| Backflow | ASSE 1011 hose-bib breaker | ASSE 1020 pressure vacuum breaker, ≥ 1.35 m above grade | Rated for a line under continuous pressure |
| Plumbing extras | None | Drain cock at the low point; water-hammer arrestor; stream nozzle with flow straightener | Freeze drain, quieter closing, tighter jet |
| Pressure sensor | 0–100 psi, upstream of valve | 0–30 psi, downstream of the zone valve | Reads what the nozzle sees, with 3× the resolution |
| 24 V supply | Mean Well LRS-100-24 | Outdoor-rated supply marked Class 2 (e.g. Mean Well LPV-100-24; confirm the marking) | Safe low-voltage outdoor wiring |
| Servo buck | 6 V, 5 A | 6 V, 8–10 A, rated ≥ 60 V input; separate small buck for the MG90S | Two servos can stall at once; surge margin on a long cable |
| Flow input | Divider straight to GPIO33 | Plus 1 kΩ series resistor and BAT54S clamp | Protects the ESP32 from spikes on the sensor lead |
| RS-485 | Bare transceiver | Pull-up on RO, pull-down on DE | Bus stays quiet while the ESP32 resets |
| Pi 5 | Default EEPROM | `PSU_MAX_CURRENT=5000` | Stops USB current limiting on the 5 V buck |
| Printed parts | P2, P5 Rev A | Reprint from `cad/stl/` (Rev B) | Fixes 16 and 17 above |

## Still unverified

- The Rev B firmware has never been compiled or run on an ESP32.
- No part of the system has run on real hardware; all numbers come from the simulator and unit tests.
- The drag constant, servo error and the thermal-to-moisture link are estimates until the first calibration day.
- Shade, dew and changes in grass emissivity can look like a dry patch to the thermal camera.
- The Rev A SVG drawings in `hardware/drawings/` do not yet show the Rev B changes to P2, P5 and the Turret Node schematic (master valve). The Rev B part sheets are in `cad/sheets/`.
