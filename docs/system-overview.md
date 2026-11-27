# System overview

Design reference: EDD-LWT-001 Rev B. The example site is a 100 m² lawn with the turret on a ¾ in post and the Sensor Unit on a 5 m pole.

## Units

### Sensor Unit
Mounted on a pole or upper eave, 4–6 m high, about 5 m behind the turret.

- **Holds:** Raspberry Pi 5, Camera Module 3 Wide, MLX90640 thermal camera on a 3-stop pan servo, USB–RS-485 adapter, 24→5.1 V buck converter, and a separate small buck for the pan servo.
- **Why high:** a camera at nozzle height cannot see a 180° lawn. A fixed high view keeps one ground mapping for the life of the install. Brace or guy the pole: if it sways or settles, the mapping moves.
- **Pi 5 setting:** set `PSU_MAX_CURRENT=5000` in the EEPROM config because the Pi is powered through its GPIO pins and would otherwise limit USB current to the RS-485 adapter.
- **Parts cost:** ~$337

### Turret Unit
A ¾ in galvanised post at the lawn edge, with the nozzle 1020 mm above grade.

- **Holds:** pan/tilt head, ESP32 Turret Node, 6 V servo buck converter, valve train, soil probe leads.
- **Owns the valves:** if the Pi crashes or the cable is cut, the ESP32 still closes both valves within 3 s.
- **Parts cost:** ~$171, plus ~$296 for the hydraulics

### Valve train
On the turret post, 150–350 mm above grade, fed by a garden hose from the spigot.

- **Order (Rev B):** pressure vacuum breaker (ASSE 1020, ≥ 1.35 m above grade) → Y-filter → 25 psi regulator → master valve → zone valve → pressure sensor (0–30 psi) → flow meter → tube to nozzle. A water-hammer arrestor sits at the valves and a drain cock at the low point.
- **Valves:** both 24 V DC, normally closed, assisted-lift (zero-differential) type with Cv ≥ 3, so they open fully even at low supply pressure.
- **Doses by:** measured litres from the flow meter, not by time
- **Supply needed:** 30 psi or more at the spigot while flowing 7 L/min

The regulator throws away most of the mains pressure so the jet sees a steady 22–25 psi whatever the street supply does. Rev B puts the pressure sensor after the zone valve, so it reads the pressure the nozzle actually sees, and the Pi rescales the aiming model to it.

### 24 V supply
An outdoor-rated 24 V DC, 100 W supply marked Class 2 (for example Mean Well LPV-100-24; confirm the marking) on an existing GFCI outlet.

- **Load:** ~21 W typical, ~72 W peak with both valves open and both servos moving
- **Protection:** 4 A blade fuse per feed at the supply; reverse-polarity diode (SS54) and surge clamp (TVS) at each unit; buck converters rated for at least 60 V input
- **Rule:** all outdoor wiring stays at 24 V or below

### Wiring between units
A trench 150 mm deep carries ¾ in conduit with a 16/2 landscape power cable and an outdoor Cat6 cable (allow at least 11 m). One twisted pair of the Cat6 carries RS-485 data at 115200 baud. The orange and green pairs are spare for a second turret.

## Lawn cells

The planner divides the lawn into 1 m × 1 m cells and scores each against the lawn median from the afternoon thermal image. Each cell's temperature is the average of four samples inside it; cells only partly in view are marked unknown and left alone.

- **Pass size:** at most 5 mm of water per pass (cycle and soak)
- **Moisture model:** three soil probes calibrate temperature excess against soil moisture each day; the target is 27% volumetric water content
- **Quality gate:** a scan with less than 0.8 °C spread across the lawn is rejected (too cloudy or too uniform to trust). With no working probes, only cells 1.5 °C hotter than the median are watered.
- **Skips if:** 3 mm of rain since the scan, 6 mm or more forecast in 24 h, wind over 4 m/s or gusts over 7 m/s, or the plan is more than 20 h old

## Daily schedule

| Time | Activity |
|---|---|
| 05:30 | Water job starts, checks the Turret Node for faults, then waits for daylight |
| Sunrise + 15 min | Water for 20–40 min (skipped if this is later than sunrise + 2 h) |
| 13:15 | Thermal scan, about 1 min; saves the next morning's plan |
| Rest of day | Idle: valves closed, nozzle parked at −45° |
| Monthly | Calibration, 16 test shots on a calm, dry late morning; re-check the markers |

Both jobs run from cron through `flock`, so a scan and a watering session can never use the RS-485 bus at the same time.
