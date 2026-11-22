# System overview

Design reference: EDD-LWT-001 Rev A. The example site is a 100 m² lawn with the turret on a ¾ in post and the Sensor Unit on a 5 m pole.

## Units

### Sensor Unit
Mounted on a pole or upper eave, 4–6 m high, about 5 m behind the turret.

- **Holds:** Raspberry Pi 5, Camera Module 3 Wide, MLX90640 thermal camera on a 3-stop pan servo, USB–RS-485 adapter, 24→5 V buck converter.
- **Why high:** a camera at nozzle height cannot see a 180° lawn. A fixed high view keeps one ground mapping for the life of the install.
- **Parts cost:** ~$332

### Turret Unit
A ¾ in galvanised post at the lawn edge, with the nozzle 1020 mm above grade.

- **Holds:** pan/tilt head, ESP32 Turret Node, 6 V servo buck converter, valve train, soil probe leads.
- **Owns the valve:** if the Pi crashes or the cable is cut, the ESP32 still closes the valve within 3 s.
- **Parts cost:** ~$160, plus ~$126 for the hydraulics

### Valve train
On the turret post, 150–350 mm above grade, fed by a garden hose from the spigot.

- **Order:** vacuum breaker → Y-filter → 25 psi regulator → pressure tee → normally-closed solenoid valve → flow meter → tube to nozzle
- **Doses by:** measured litres from the flow meter, not by time
- **Supply needed:** 30 psi or more at the spigot while flowing 7 L/min

Pressure along the train when the valve is open: about 55 psi at the spigot, 25 psi after the regulator, and about 22 psi at the nozzle. The regulator throws away most of the mains pressure so the jet sees a steady pressure whatever the street supply does.

### 24 V supply
A listed 24 V DC, 100 W supply (e.g. Mean Well LRS-100-24) on an existing GFCI outlet, indoors or in a weatherproof box.

- **Load:** ~21 W typical, ~67 W peak (33% margin)
- **Protection:** 4 A blade fuse per feed at the supply; reverse-polarity diode (SS54) and surge clamp (TVS) at each unit
- **Rule:** all outdoor wiring stays at 24 V or below

### Wiring between units
A trench 150 mm deep carries ¾ in conduit with a 16/2 landscape power cable and an outdoor Cat6 cable. One twisted pair of the Cat6 carries RS-485 data at 115200 baud. The orange and green pairs are spare for a second turret.

## Lawn cells

The planner divides the lawn into 1 m × 1 m cells and scores each against the lawn median from the afternoon thermal image.

- **Pass size:** at most 5 mm of water per pass (cycle and soak)
- **Accuracy goal:** ≤ 0.15 m median miss at 8 m in still air
- **Skips if:** 6 mm or more of rain forecast in 24 h, or wind over 4 m/s

## Daily schedule

| Time | Activity |
|---|---|
| 05:30 | Pre-checks, then water for 20–40 min |
| 13:15 | Thermal scan, about 1 min |
| Rest of day | Idle: valve closed, nozzle parked at −60° |
| Monthly | Calibration, 16 test shots on a calm, dry late morning |
