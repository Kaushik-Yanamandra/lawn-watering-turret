# Hardware

- [bom.csv](bom.csv): 44 parts-list lines, about $776 in total. Lines marked `yes` had their price checked on 22 Oct 2026; the rest are typical US retail estimates.
- [drawings/](drawings): Rev A drawing set, 9 sheets, A3, third-angle projection, millimetres.

| Group | Estimated cost |
|---|---|
| Sensor Unit | ~$332 |
| Turret Unit | ~$160 |
| Hydraulics | ~$126 |
| Site & power | ~$158 |

## 3D-printed parts

23 prints, about 500 g in total. ASA for parts in the sun, PETG for parts in shade. **Never PLA**: it softens in Texas sun.

| Part | Name | Qty | Material |
|---|---|---|---|
| P1 | Pan base | 1 | ASA |
| P2 | Yoke | 1 | ASA |
| P3a | Cradle, horn half | 1 | ASA |
| P3b | Cradle, axle half | 1 | ASA |
| P4 | Pipe clamp half | 8 | PETG/ASA |
| P5 | Mounting plate | 2 | PETG/ASA |
| P6 | Post clip | 6 | PETG |
| P7 | Sensor bracket | 1 | PETG |
| P8 | Thermal visor | 1 | ASA |
| P9 | Thermal carrier | 1 | PETG |

**Print settings:** 0.2 mm layers, 4 perimeters, 40% gyroid infill; P1–P3 at 60%. Print a 10 mm fit slice of P1 and P3 first to check the servo pocket, post bore and hex pocket.

## Drawing set (Rev A)

| Sheet | Title | Scale | What it shows |
|---|---|---|---|
| [LWT-P1](drawings/LWT-P1_pan-base.svg) | Pan base | 1:1 | Clamps on the post top, carries the bearing, hangs the pan servo on the pan axis |
| [LWT-P2](drawings/LWT-P2_yoke.svg) | Yoke | 1:1 | Turns on the bearing; holds the tilt servo and the 608 trunnion |
| [LWT-P3](drawings/LWT-P3_nozzle-cradle.svg) | Nozzle cradle | 3:2 | Two halves clamping the brass adapter by its 19 mm hex |
| [LWT-P4](drawings/LWT-P4_pipe-clamp.svg) | Pipe clamp half | 2:1 | Clamps the ¾ in post and carries the mounting plates |
| [LWT-P5](drawings/LWT-P5_mounting-plate.svg) | Mounting plate | 1:1 | Carries the Node box and the valve train |
| [LWT-A1](drawings/LWT-A1_head-assembly.svg) | Head assembly | 1:2 | Side and front views, exploded isometric, parts list |
| [LWT-A2](drawings/LWT-A2_post-arrangement.svg) | Post arrangement | 1:10 | Footing, post and every mounting height on site |
| [LWT-E1](drawings/LWT-E1_turret-node-schematic.svg) | Turret Node schematic | NTS | Component-level schematic: protection, rails, ESP32, servos, valve, sensors, RS-485 |
| [LWT-E2](drawings/LWT-E2_wiring-harness.svg) | Wiring harness | NTS | Every cable end to end with colours, pins and lengths |

> The part drawings were generated from 3D models built to the envelopes, hole patterns and interfaces in the design document. Wall thicknesses, fillets and ear shapes are assumptions. Check them against the actual servos, bearing and fittings before printing.
