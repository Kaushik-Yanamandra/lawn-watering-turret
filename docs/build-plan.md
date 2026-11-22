# Build plan

> **Before buying everything:** the ESP32 firmware has not been compiled yet, and the camera drivers have not run on hardware. Start with the ESP32, the valve and one servo on the bench, and get the `PING`, `MOVE` and `VALVE` commands working first.

## Phase A — Print and prepare (bench)
1. Measure the actual servos, bearing and brass fittings; update the parameters in the CAD script.
2. Print 10 mm fit slices of P1 (servo pocket, post bore) and P3 (hex pocket). Adjust until they fit.
3. Print all 23 parts: 0.2 mm layers, 4 perimeters, 40% gyroid infill; P1–P3 at 60%. ASA needs an enclosure and a 100–110 °C bed.
4. Press in heat-set inserts straight with a flat tip: 260 °C for ASA, 245 °C for PETG.

## Phase B — Electronics (bench)
1. Turret Node: protection (SS54, SMBJ28A), 6 V and 5 V buck converters, 2200 µF capacitor on the servo rail.
2. Turret Node: IRLZ44N valve driver with 100 Ω gate resistor, 10 kΩ pull-down, 1N5819 across the coil.
3. Turret Node: flow and pressure dividers, probe inputs with 100 nF capacitors, 24 V monitor, MAX3485 with 120 Ω termination.
4. Sensor Unit: fuse, SS54, TVS, 5.1 V buck to the Pi; camera on CSI, MLX90640 on I2C1, MG90S on GPIO18, RS-485 adapter to RJ45.
5. Flash the ESP32. Confirm `PING`, then `MOVE 0 0` to centre both servos before fitting the horns.

## Phase C — Head and valve train (bench)
1. Hang the pan servo under P1, spline on the pan axis.
2. Fit the lazy-susan bearing between P1 and P2; grease it.
3. Tilt servo through P2's +y arm, 25T horn to P3; M8 bolt in the 608 bearing on the −y arm.
4. Clamp the brass barb adapter in the P3 cradle by its hex; thread in the 3 mm nozzle.
5. Valve train on its P5 plate: regulator → pressure tee → valve → flow meter → tube barb. Flow arrow up.

## Phase D — Civil work (day 1)
1. Call 811 before digging. Check for sprinkler pipes at the post spot and along the trench.
2. Mark the post at the middle of the lawn edge nearest the spigot. Measure the lawn outline and six marker spots.
3. Dig Ø150 × 650 mm. Set the post 600 mm deep, plumb both ways, concrete, crown the top. Post top 924 mm above grade.
4. Trench 150 mm deep from the Sensor Unit to the post. Lay ¾ in conduit with the 16/2 and Cat6; seal the ends.
5. Install the Sensor Unit pole or eave bracket: about 5 m high, 4.6–5 m behind the turret.

## Phase E — Mount and wire (day 2)
1. Fit the vacuum breaker to the spigot. Mount the valve train at 150–350 mm. Run the hose.
2. Mount the Node box at 600–760 mm. Bring cables in through glands.
3. Clamp the head on the post, pan axis into the lawn. Clip the tube and leads down the post with a 250 mm service loop.
4. Install the Sensor Unit cameras and visor. Aim so the whole lawn fills the visible frame.
5. Bury 3 soil probes upright at 50–100 mm in spots with different sun. Record positions.
6. Plug in 24 V. Node LED blinks slowly; `STAT` answers over the link.

## Phase F — Commission (day 2 + 2 weeks)
1. Bucket under the nozzle: run VT-11 (watchdog, 120 s limit) and VT-12 (no-flow, stuck valve) first.
2. `MOVE 0 0`, then trim the pan and tilt centre offsets until the nozzle points straight out and level.
3. Flow check (VT-05); set the flow-meter constant.
4. Place six markers, click each in the camera image, save the ground mapping.
5. Calm, dry late morning: run calibration. Accept if RMS residual < 0.10 m.
6. Run VT-03, VT-04 and VT-06. Enable the 13:15 scan and 05:30 water jobs; run VT-07 to VT-10 over two weeks.

---

## Head assembly detail

| Step | What | Hardware |
|---|---|---|
| 1 | **Centre both servos** on the bench with `MOVE 0 0`. Fit horns only after this, so the head starts square. | Turret Node + 24 V supply + RS-485 adapter |
| 2 | **Hang the pan servo in P1**, spline up on the pan axis, 50 mm off the post centre. The shroud is open at the bottom so water drains away. | 8 × M4 inserts in P1 |
| 3 | **Lazy-susan bearing and yoke.** The 4 in bearing between P1 and P2 carries every axial and tipping load; the servo horn only passes torque. | 4 × M4 × 16, 4 × M3 × 10 |
| 4 | **Tilt servo and horn** through P2's +y arm; the 25T disc horn drives the P3 cradle. Tilt axis 75 mm above the yoke plate. Range −60° to +40°. | 4 × M4 inserts, 25T horn |
| 5 | **608 trunnion.** The other side of the cradle turns on an M8 bolt in a sealed 608-2RS bearing. Check it swings freely with no side play. | 608-2RS, M8 × 40 bolt and nut |
| 6 | **Cradle and nozzle.** Clamp the brass adapter in the split cradle by its hex so it cannot turn; thread in the 3 mm nozzle; push the tube onto the barb. | 8 × M3 inserts, 4 × M3 × 40 |
| 7 | **Mount on the post**, pan axis facing into the lawn. Leave a 250 mm service loop so the head can pan ±135° without pulling. | M5 × 30, 6 × P6 clips |
