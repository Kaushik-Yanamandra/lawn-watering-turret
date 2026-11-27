# Build plan

> **Before buying everything:** the Rev B ESP32 firmware ([`firmware/turret_node`](../firmware/turret_node/turret_node.ino)) has not been compiled yet, and the camera drivers have not run on hardware. First compile it in Arduino IDE 2 with the ESP32 core 3.x. Then start with the ESP32, both valves and one servo on the bench, and get `PING`, `MOVE`, `VALVE`, `DOSE` and `DOSE?` working. Run `software/tests` on a PC to see what the Pi expects the node to answer.

## Phase A — Print and prepare (bench)
1. Measure the actual servos, bearing and brass fittings; update the parameters at the top of [`cad/lwt_parts.py`](../cad/lwt_parts.py) and regenerate the STL files.
2. Print 10 mm fit slices of P1 (servo pocket, post bore) and P3 (hex pocket). Adjust until they fit.
3. Print all 23 parts from [`cad/stl/`](../cad/stl) (Rev B: P2 and P5 changed; reprint them if you printed Rev A): 0.2 mm layers, 4 perimeters, 40% gyroid infill; P1–P3 at 60%. ASA needs an enclosure and a 100–110 °C bed.
4. Press in heat-set inserts straight with a flat tip: 260 °C for ASA, 245 °C for PETG.

## Phase B — Electronics (bench)
1. Turret Node: protection (SS54, SMBJ28A), 6 V (8–10 A) and 5 V buck converters rated for at least 60 V input, 2200 µF capacitor on the servo rail.
2. Turret Node: two IRLZ44N valve drivers (zone valve on GPIO27, master valve on GPIO13), each with a 100 Ω gate resistor, 10 kΩ pull-down and 1N5819 across the coil.
3. Turret Node: flow divider plus 1 kΩ series resistor and BAT54S clamp; pressure divider; probe inputs with 100 nF capacitors; 24 V monitor; MAX3485 with 120 Ω termination, a pull-up on RO and a pull-down on DE.
4. Sensor Unit: fuse, SS54, TVS, 5.1 V buck to the Pi and a separate small buck for the MG90S; camera on CSI, MLX90640 on I2C1, MG90S on GPIO18, RS-485 adapter to RJ45. Set `PSU_MAX_CURRENT=5000` in the Pi 5 EEPROM config.
5. Flash the ESP32. Confirm `PING`, then `MOVE 0 0` to centre both servos before fitting the horns. (Every frame ends with `*` and a two-digit XOR checksum; see the protocol table at the top of the firmware.)

## Phase C — Head and valve train (bench)
1. Hang the pan servo under P1, spline on the pan axis.
2. Fit the lazy-susan bearing between P1 and P2; grease it.
3. Tilt servo through P2's +y arm, 25T horn to P3; M8 bolt in the 608 bearing on the −y arm.
4. Clamp the brass barb adapter in the P3 cradle by its hex; thread in the 3 mm nozzle.
5. Valve train on its P5 plate: regulator → master valve → zone valve → pressure tee (0–30 psi sensor) → flow meter → tube barb. Flow arrows up. Fit the water-hammer arrestor at the valves.

## Phase D — Civil work (day 1)
1. Call 811 before digging. Check for sprinkler pipes at the post spot and along the trench.
2. Mark the post at the middle of the lawn edge nearest the spigot. Measure the lawn outline and six marker spots.
3. Dig Ø150 × 650 mm. Set the post 600 mm deep, plumb both ways, concrete, crown the top. Post top 924 mm above grade.
4. Trench 150 mm deep from the Sensor Unit to the post. Lay ¾ in conduit with the 16/2 and Cat6; seal the ends.
5. Install the Sensor Unit pole or eave bracket: about 5 m high, 4.6–5 m behind the turret.

## Phase E — Mount and wire (day 2)
1. Fit the ASSE 1020 pressure vacuum breaker at least 1.35 m above grade (check local code), with a drain cock at the low point. Mount the valve train at 150–350 mm. Run the hose.
2. Mount the Node box at 600–760 mm. Bring cables in through glands.
3. Clamp the head on the post, pan axis into the lawn. Run the tube and tilt-servo lead outside the yoke in a 250 mm service loop, tied to P2's zip-tie holes, and clip them down the post.
4. Install the Sensor Unit cameras and visor. Aim so the whole lawn fills the visible frame.
5. Bury 3 soil probes upright at 50–100 mm in spots with different sun. Record positions.
6. Plug in 24 V. Node LED blinks slowly; `STAT` answers over the link.

## Phase F — Commission (day 2 + 2 weeks)
1. Bucket under the nozzle: run VT-11 (link watchdog, 120 s limit) and VT-12 (no-flow, high-flow, stuck valve: hold the zone valve open by hand and check that the master valve shuts and the fault survives a power cycle) first.
2. `MOVE 0 0`, then trim the pan and tilt centre offsets until the nozzle points straight out and level.
3. Flow check (VT-05); set the flow-meter constant with `CFG flow_k`. Set `CFG nom_lpm` to the measured flow and check `STAT` reads a sensible pressure with the valves open.
4. Calibrate the camera lens with a printed checkerboard (`software/tools/calibrate_lens.py`). Place six markers, run `python run_pi.py markers`, and enter each marker's pixel position in `config.json`.
5. Calm, dry late morning: `python run_pi.py calibrate`. Accept if RMS residual < 0.10 m. Shrink the lawn outline in `config.json` to the measured reach.
6. Run VT-03, VT-04 and VT-06. Enable the 13:15 scan and 05:30 water cron jobs (the water job waits for sunrise + 15 min); run VT-07 to VT-10 over two weeks.

---

## Head assembly detail

| Step | What | Hardware |
|---|---|---|
| 1 | **Centre both servos** on the bench with `MOVE 0 0`. Fit horns only after this, so the head starts square. | Turret Node + 24 V supply + RS-485 adapter |
| 2 | **Hang the pan servo in P1**, spline up on the pan axis, 50 mm off the post centre. The shroud is open at the bottom so water drains away. | 8 × M4 inserts in P1 |
| 3 | **Lazy-susan bearing and yoke.** The 4 in bearing between P1 and P2 carries every axial and tipping load; the servo horn only passes torque. | 4 × M4 × 16, 4 × M3 × 10 |
| 4 | **Tilt servo and horn** through P2's +y arm; the 25T disc horn drives the P3 cradle. Tilt axis 75 mm above the yoke plate. Range −45° to +40°. | 4 × M4 inserts, 25T horn |
| 5 | **608 trunnion.** The other side of the cradle turns on an M8 bolt in a sealed 608-2RS bearing. Check it swings freely with no side play. | 608-2RS, M8 × 40 bolt and nut |
| 6 | **Cradle and nozzle.** Clamp the brass adapter in the split cradle by its hex so it cannot turn; thread in the 3 mm nozzle; push the tube onto the barb. | 8 × M3 inserts, 4 × M3 × 40 |
| 7 | **Mount on the post**, pan axis facing into the lawn. Leave a 250 mm service loop, tied to P2's zip-tie holes, so the head can pan ±135° without pulling. Inspect the tube monthly for wear. | M5 × 30, 6 × P6 clips |
