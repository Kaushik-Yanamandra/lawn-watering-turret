"""Water-jet ballistics and aim solver for the lawn watering turret.

Frame: origin at the base of the turret post, x/y on the lawn, z up.
Pan is measured from +x toward +y; tilt is elevation above horizontal
(negative = pointing down, used for close targets).

Jet model: the stream is treated as a point mass with quadratic drag

    dv/dt = -g*z_hat - k * |v - w| * (v - w)

v0  nozzle exit speed (m/s), set by the pressure regulator
k   effective drag constant (1/m) that lumps jet break-up and droplet drag;
    it is NOT computed from first principles, it is fitted from calibration shots
w   horizontal wind (m/s)

A real stream breaks into droplets after a few metres, which is why k is much
larger than the drag of a single big droplet. The calibration fit absorbs that.

Servo trims: the angle the servo is commanded to and the angle the nozzle
really points at differ by a small mounting offset. `pan_trim`/`tilt_trim`
are added to the commanded angle to get the physical angle; calibration fits them.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field, replace

G = 9.81
PSI_TO_PA = 6894.76


def exit_velocity(pressure_psi: float, cd: float = 0.95) -> float:
    """Bernoulli exit speed of a nozzle at a given gauge pressure (m/s)."""
    return cd * math.sqrt(2.0 * pressure_psi * PSI_TO_PA / 1000.0)


def nozzle_flow_lpm(pressure_psi: float, orifice_mm: float, cd: float = 0.95) -> float:
    """Volumetric flow of a round orifice (L/min)."""
    area = math.pi * (orifice_mm / 1000.0) ** 2 / 4.0
    return area * exit_velocity(pressure_psi, cd) * 60_000.0


@dataclass
class JetModel:
    v0: float = 17.6          # m/s  (about 25 psi at the nozzle)
    k: float = 0.11           # 1/m  effective drag, fitted by calibrate.fit_jet
    h: float = 1.0            # m    nozzle height above the lawn
    wind_x: float = 0.0       # m/s  current wind / bias estimate
    wind_y: float = 0.0       # m/s
    pan_trim: float = 0.0     # deg  physical = commanded + trim
    tilt_trim: float = 0.0    # deg
    tilt_min: float = -45.0   # deg  physical tilt limits; below ~-50 deg the jet hits the head (rev B)
    tilt_max: float = 40.0
    dt: float = 0.01          # s    RK4 step
    _reach_cache: dict = field(default_factory=dict, repr=False, compare=False)

    # ------------------------------------------------------------------ flight
    def _fly(self, pan_deg: float, tilt_deg: float, wx: float, wy: float):
        """Integrate a shot at PHYSICAL angles; return landing (x, y, t)."""
        p, t = math.radians(pan_deg), math.radians(tilt_deg)
        vx = self.v0 * math.cos(t) * math.cos(p)
        vy = self.v0 * math.cos(t) * math.sin(p)
        vz = self.v0 * math.sin(t)
        x, y, z = 0.0, 0.0, self.h
        k, dt = self.k, self.dt

        def acc(vx, vy, vz):
            rx, ry = vx - wx, vy - wy
            s = math.sqrt(rx * rx + ry * ry + vz * vz)
            return -k * s * rx, -k * s * ry, -G - k * s * vz

        tt = 0.0
        while True:
            a1 = acc(vx, vy, vz)
            v2 = (vx + 0.5 * dt * a1[0], vy + 0.5 * dt * a1[1], vz + 0.5 * dt * a1[2])
            a2 = acc(*v2)
            v3 = (vx + 0.5 * dt * a2[0], vy + 0.5 * dt * a2[1], vz + 0.5 * dt * a2[2])
            a3 = acc(*v3)
            v4 = (vx + dt * a3[0], vy + dt * a3[1], vz + dt * a3[2])
            a4 = acc(*v4)
            nx = x + dt / 6 * (vx + 2 * v2[0] + 2 * v3[0] + v4[0])
            ny = y + dt / 6 * (vy + 2 * v2[1] + 2 * v3[1] + v4[1])
            nz = z + dt / 6 * (vz + 2 * v2[2] + 2 * v3[2] + v4[2])
            vx += dt / 6 * (a1[0] + 2 * a2[0] + 2 * a3[0] + a4[0])
            vy += dt / 6 * (a1[1] + 2 * a2[1] + 2 * a3[1] + a4[1])
            vz += dt / 6 * (a1[2] + 2 * a2[2] + 2 * a3[2] + a4[2])
            if nz <= 0.0:
                f = z / (z - nz)            # linear interpolation to z = 0
                return x + f * (nx - x), y + f * (ny - y), tt + f * dt
            x, y, z, tt = nx, ny, nz, tt + dt
            if tt > 10.0:
                raise RuntimeError("trajectory did not land; check parameters")

    def landing(self, pan_cmd: float, tilt_cmd: float, wind: bool = True):
        """Landing point (x, y, flight_time) for a COMMANDED pan/tilt."""
        wx, wy = (self.wind_x, self.wind_y) if wind else (0.0, 0.0)
        return self._fly(pan_cmd + self.pan_trim, tilt_cmd + self.tilt_trim, wx, wy)

    def trajectory(self, pan_cmd: float, tilt_cmd: float, wind: bool = True, dt: float = 0.01):
        """Sampled (x, y, z) points along the flight (Euler, for plotting only)."""
        p = math.radians(pan_cmd + self.pan_trim)
        t = math.radians(tilt_cmd + self.tilt_trim)
        v = [self.v0 * math.cos(t) * math.cos(p), self.v0 * math.cos(t) * math.sin(p),
             self.v0 * math.sin(t)]
        pos = [0.0, 0.0, self.h]
        wx, wy = (self.wind_x, self.wind_y) if wind else (0.0, 0.0)
        pts = []
        while pos[2] > 0:
            pts.append(tuple(pos))
            rx, ry = v[0] - wx, v[1] - wy
            s = math.sqrt(rx * rx + ry * ry + v[2] ** 2)
            a = (-self.k * s * rx, -self.k * s * ry, -G - self.k * s * v[2])
            v = [v[i] + a[i] * dt for i in range(3)]
            pos = [pos[i] + v[i] * dt for i in range(3)]
        pts.append(tuple(pos))
        return pts

    # ------------------------------------------------------------------ range
    def range_at(self, tilt_phys: float) -> float:
        """Still-air range for a PHYSICAL tilt."""
        return self._fly(0.0, tilt_phys, 0.0, 0.0)[0]

    def reach(self):
        """(min_range, max_range, physical tilt at max range), still air. Cached."""
        key = (self.v0, self.k, self.h, self.tilt_min, self.tilt_max, self.dt)
        if key not in self._reach_cache:
            a, b = 0.0, min(50.0, self.tilt_max)
            gr = (math.sqrt(5) - 1) / 2
            c, d = b - gr * (b - a), a + gr * (b - a)
            for _ in range(30):                    # golden-section search
                if self.range_at(c) > self.range_at(d):
                    b = d
                else:
                    a = c
                c, d = b - gr * (b - a), a + gr * (b - a)
            t_star = 0.5 * (a + b)
            self._reach_cache[key] = (self.range_at(self.tilt_min), self.range_at(t_star), t_star)
        return self._reach_cache[key]

    def tilt_for_range(self, r: float) -> float | None:
        """Low-branch PHYSICAL tilt that lands at range r in still air.

        The low branch (below the max-range angle) has the shortest flight
        time, so wind moves it least. Range rises monotonically with tilt on
        this branch, so plain bisection works.
        """
        r_min, r_max, t_star = self.reach()
        if not (r_min <= r <= r_max):
            return None
        lo, hi = self.tilt_min, t_star
        for _ in range(30):
            mid = 0.5 * (lo + hi)
            if self.range_at(mid) < r:
                lo = mid
            else:
                hi = mid
        return 0.5 * (lo + hi)

    # ------------------------------------------------------------------ aim
    def aim(self, tx: float, ty: float, tol: float = 0.02, max_iter: int = 12):
        """Commanded pan/tilt that land the jet on (tx, ty), wind included.

        Fixed-point iteration: aim at a virtual point, let the model say where
        the water lands, shift the virtual point by the miss, repeat.
        Returns (pan_cmd, tilt_cmd, (pred_x, pred_y)) or None if out of reach.
        """
        ax, ay = tx, ty
        result = None
        for _ in range(max_iter):
            pan_phys = math.degrees(math.atan2(ay, ax))
            tilt_phys = self.tilt_for_range(math.hypot(ax, ay))
            if tilt_phys is None:
                return None        # wind pushes the needed aim point out of reach
            lx, ly, _ = self._fly(pan_phys, tilt_phys, self.wind_x, self.wind_y)
            result = (pan_phys - self.pan_trim, tilt_phys - self.tilt_trim, (lx, ly))
            ex, ey = lx - tx, ly - ty
            if math.hypot(ex, ey) < tol:
                break
            ax, ay = ax - ex, ay - ey
        return result

    def copy(self, **changes) -> "JetModel":
        return replace(self, _reach_cache={}, **changes)
