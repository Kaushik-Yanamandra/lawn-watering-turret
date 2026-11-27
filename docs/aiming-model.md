# Aiming model

To hit a dry cell, the turret has to know where a jet will land for a given pan angle, tilt angle and nozzle pressure. A plain projectile formula (no air resistance) overshoots badly at these speeds, so the model includes air drag and wind.

Code: [`software/turret/ballistics.py`](../software/turret/ballistics.py) (model and aim solver) and [`software/turret/calibrate.py`](../software/turret/calibrate.py) (fitting and wind estimate).

## Model

The jet is treated as a point moving through air with **quadratic drag**:

$$
\frac{d\mathbf{v}}{dt} = -g\,\hat{\mathbf{z}} - k\,|\mathbf{v}_{rel}|\,\mathbf{v}_{rel}, \qquad \mathbf{v}_{rel} = \mathbf{v} - \mathbf{w}
$$

where **w** is the wind velocity and **k** is a drag constant (about 0.13 per metre), fitted by calibration.

- The nozzle is 1.02 m above the ground.
- Jet speed scales with the square root of nozzle pressure, v₀ ∝ √p. Rev B measures the nozzle pressure on the first check shot of each session and rescales v₀ to match.
- The equations are integrated with a 4th-order Runge–Kutta step until the jet reaches the ground.
- To aim, the solver finds the low-arc tilt that lands at the target's range (bisection), then corrects pan and tilt for the current wind estimate.

## Limits

| | |
|---|---|
| Reach | 1.0 m to about 9 m at 25 psi. The model gives 8.9 m at k = 0.15 and 9.6 m at k = 0.13, so the lawn is planned to 8.5 m until calibration measures the real reach. |
| Tilt range | −45° (park / drain) to +40°. Below about −50° the jet would strike the head (Rev A allowed −60°). |
| Pan range | ±135° |
| Pressure sensitivity | Reach falls from about 9.6 m at 25 psi to 7.8 m at 15 psi |

## Accuracy

The simulator scores every shot with the errors a real turret has: 0.25° of servo error on each axis per move, and 0.6% shot-to-shot scatter in jet speed (about 0.25 psi). Rev A scored shots without these, which made its accuracy look better than it could be.

| Case | Median miss | 90% of shots within |
|---|---|---|
| Datasheet model, no calibration, still air | 0.65 m | 1.12 m |
| Calibrated (16 shots), still air | 0.05 m | 0.11 m |
| 2.5 m/s crosswind with 0.5 m/s gusts, no correction | 0.56 m | 1.03 m |
| Same wind, camera wind correction | 0.28 m | 0.67 m |

**Targets (requirement P-03):** calibrated, still air, median ≤ 0.10 m and 90% within 0.15 m; in a 2.5 m/s crosswind with camera correction, median ≤ 0.35 m and 90% within 0.70 m.

At 8 m range, 0.25° of tilt error moves the landing about 0.04 m, 0.25° of pan error about 0.035 m, and 0.6% of jet speed about 0.04 m.

## Wind correction during a session

Every few doses the turret fires a short check shot onto dry turf. The camera takes a frame 0.7 s after the valve closes (so the stream has landed and only the splash remains), finds the wet spot by frame differencing, and maps it to the ground. The difference between where the shot was predicted to land and where it did land updates a running wind estimate.

- Check shots are aimed only at points at least 0.45 m from turf already watered this session, so old wet patches are not mistaken for new ones.
- A shot that lands much further from the prediction than expected (more than 0.6 m + 0.08 m per metre of range) is treated as an outlier and ignored.
- 4 missed check shots in a row, 3 outliers in a row, or an estimated wind over 4 m/s stop the session.

## Calibration

Real nozzles and pressures never match the model exactly, so the jet speed, drag constant and aim offsets are fitted to the real system:

1. **Lens (once).** Photograph a printed 9 × 6 checkerboard about 20 times across the camera's view and run `software/tools/calibrate_lens.py`. This removes the wide lens's barrel distortion, which would otherwise put edge pixels at the wrong ground point.
2. **Markers (once, re-check monthly).** Six markers are placed on the lawn (golf tees with cups of hot water, so they also show up on the thermal camera) and their pixel positions entered to set the ground mapping.
3. **Jet (monthly).** On a calm, dry late morning, the turret fires 16 test shots. The camera finds where each one landed, and the nozzle pressure is recorded.
4. The model parameters are fitted to those landings by least squares. The calibration is accepted if the RMS residual is under 0.10 m (the simulator reaches 0.05 m); the Pi sends an alert above 0.15 m.
