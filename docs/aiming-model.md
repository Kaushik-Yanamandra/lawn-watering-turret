# Aiming model

To hit a dry cell, the turret has to know where a jet will land for a given pan angle, tilt angle and nozzle pressure. A plain projectile formula (no air resistance) overshoots badly at these speeds, so the model includes air drag.

## Model

The jet is treated as a point moving through air with **quadratic drag**:

$$
\frac{d\mathbf{v}}{dt} = -g\,\hat{\mathbf{z}} - k\,|\mathbf{v}_{rel}|\,\mathbf{v}_{rel}, \qquad \mathbf{v}_{rel} = \mathbf{v} - \mathbf{w}
$$

where **w** is the wind velocity and **k** is a drag constant (design value 0.12 per metre), fitted by calibration.

- The nozzle is 1.02 m above the ground.
- Jet speed scales with the square root of nozzle pressure: about 16.5 m/s at 22 psi, so v₀ = 16.5 × √(p / 22).
- The equations are stepped forward in time (2 ms steps) until the jet reaches the ground.

## Design case

At 22 psi and 12.5° of tilt, the jet lands about 8 m out. That is the design case used for the accuracy target.

| | |
|---|---|
| Reach | 0.6–9.5 m |
| Tilt range | −60° (park / drain) to +40° |
| Pan range | ±135° |
| Accuracy goal | ≤ 0.15 m median miss at 8 m in still air |
| Simulated | 0.11 m median miss at 8 m in still air |

## Why dawn and why a wind limit

The model is also used to check how much a gust moves the landing point. For each shot it compares the landing with and without an extra 0.5 m/s of wind. If a 0.5 m/s gust would move the shot more than 0.3 m, that range is flagged as gust-sensitive. This is the main reason watering happens at dawn, when wind is lowest, and why a session stops if the estimated wind is over 4 m/s.

## Calibration

Real nozzles and pressures never match the model exactly, so the drag constant and aim offsets are fitted to the real system:

1. Six markers are placed on the lawn (golf tees with cups of hot water, so they show up on the thermal camera) and clicked in the camera image to set the ground mapping.
2. On a calm, dry late morning, the turret fires 16 test shots. The camera finds where each one landed.
3. The model parameters are fitted to those landings. The calibration is accepted if the RMS residual is under 0.10 m.

Calibration is repeated monthly.
