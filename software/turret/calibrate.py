"""Fit the jet model to observed landing points, and track wind on line.

Two stages:

1. fit_jet()       one-off, in calm air on a dry lawn. Fires a spread of
                   test shots at known commanded angles, locates each landing
                   with the camera, and least-squares fits v0, k and the two
                   servo trims.
2. WindEstimator   every watering session. Each verification shot gives one
                   observed landing point; the estimator updates the wind
                   that explains the miss. It also soaks up small slow drifts
                   (pressure, nozzle wear), so treat it as "wind + bias".
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from scipy.optimize import least_squares

from .ballistics import JetModel


@dataclass
class Shot:
    pan_cmd: float
    tilt_cmd: float
    x_obs: float
    y_obs: float


def default_test_pattern(model: JetModel, n_pan: int = 4, n_tilt: int = 4,
                         pan_span=(-60.0, 60.0)) -> list[tuple[float, float]]:
    """Commanded (pan, tilt) pairs spread over the working envelope."""
    pans = np.linspace(pan_span[0], pan_span[1], n_pan)
    tilts = np.linspace(-35.0, 30.0, n_tilt)
    return [(float(p), float(t)) for p in pans for t in tilts]


def fit_jet(shots: list[Shot], start: JetModel, fit_wind: bool = False):
    """Least-squares fit of v0, k, pan_trim, tilt_trim (and optionally wind).

    Returns (fitted_model, rms_residual_m).
    """
    names = ["v0", "k", "pan_trim", "tilt_trim"] + (["wind_x", "wind_y"] if fit_wind else [])
    x0 = np.array([getattr(start, n) for n in names], dtype=float)
    lo = np.array([5.0, 0.0, -15.0, -15.0] + ([-10.0, -10.0] if fit_wind else []))
    hi = np.array([40.0, 1.0, 15.0, 15.0] + ([10.0, 10.0] if fit_wind else []))
    x0 = np.clip(x0, lo + 1e-6, hi - 1e-6)

    def residuals(x):
        m = start.copy(**dict(zip(names, x)), **({} if fit_wind else {"wind_x": 0.0, "wind_y": 0.0}))
        r = []
        for s in shots:
            lx, ly, _ = m.landing(s.pan_cmd, s.tilt_cmd)
            r += [lx - s.x_obs, ly - s.y_obs]
        return np.array(r)

    sol = least_squares(residuals, x0, bounds=(lo, hi), x_scale=[5, 0.05, 2, 2] + ([1, 1] if fit_wind else []))
    fitted = start.copy(**dict(zip(names, sol.x)), **({} if fit_wind else {"wind_x": 0.0, "wind_y": 0.0}))
    rms = float(np.sqrt(np.mean(sol.fun ** 2)))
    return fitted, rms


class WindEstimator:
    """Running wind/bias estimate from verification shots.

    Each update is a regularised least-squares step (in effect a Kalman
    update): it trades the observed miss, weighted by the camera's landing
    accuracy, against the change from the previous estimate, weighted by how
    much the wind can plausibly shift between shots.

    This matters because a short, steep shot barely feels the wind (tiny
    flight time). Solving "what wind explains this miss" exactly from such a
    shot turns 5 cm of splash scatter into several m/s of fake wind. With the
    prior term, short shots hardly move the estimate and long shots, which
    carry real information about the wind, move it a lot.
    """

    def __init__(self, sigma_obs: float = 0.12, sigma_step: float = 1.5, max_wind: float = 8.0):
        self.sigma_obs = sigma_obs      # m,   landing location accuracy
        self.sigma_step = sigma_step    # m/s, how far the wind may move between checks
        self.max_wind = max_wind        # m/s, hard clamp

    def update(self, model: JetModel, shot: Shot) -> JetModel:
        w0 = np.array([model.wind_x, model.wind_y])

        def resid(w):
            m = model.copy(wind_x=w[0], wind_y=w[1])
            lx, ly, _ = m.landing(shot.pan_cmd, shot.tilt_cmd)
            return [(lx - shot.x_obs) / self.sigma_obs, (ly - shot.y_obs) / self.sigma_obs,
                    (w[0] - w0[0]) / self.sigma_step, (w[1] - w0[1]) / self.sigma_step]

        sol = least_squares(resid, w0, bounds=([-self.max_wind] * 2, [self.max_wind] * 2))
        return model.copy(wind_x=float(sol.x[0]), wind_y=float(sol.x[1]))

    @staticmethod
    def speed(model: JetModel) -> float:
        return math.hypot(model.wind_x, model.wind_y)
