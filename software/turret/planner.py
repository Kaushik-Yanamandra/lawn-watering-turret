"""Turn dryness scores into a watering plan.

Pipeline
    thermal excess per cell (deg C)
      -> estimated soil moisture (fitted against the soil probes)
      -> water deficit (mm) to bring the root zone back to target
      -> seconds of spray per cell, split into short passes (cycle and soak)

Units: 1 mm of water over 1 m^2 is exactly 1 litre.
"""
from __future__ import annotations

import json
import math
import urllib.request
from dataclasses import dataclass

import numpy as np
from matplotlib.path import Path

from .ballistics import JetModel


# ----------------------------------------------------------------------------- grid
@dataclass
class Cell:
    idx: int
    x: float
    y: float


def lawn_cells(polygon, cell_size: float, model: JetModel, margin: float = 0.3):
    """Cell centres inside the lawn polygon that the turret can reach."""
    poly = Path(np.asarray(polygon))
    xs, ys = zip(*polygon)
    r_min, r_max, _ = model.reach()
    cells = []
    for x in np.arange(min(xs) + cell_size / 2, max(xs), cell_size):
        for y in np.arange(min(ys) + cell_size / 2, max(ys), cell_size):
            r = math.hypot(x, y)
            if poly.contains_point((x, y)) and r_min + margin <= r <= r_max - margin:
                cells.append(Cell(len(cells), float(x), float(y)))
    return cells


# ----------------------------------------------------------------------------- moisture
@dataclass
class MoistureModel:
    """Linear map from thermal excess (deg C) to volumetric water content (VWC)."""
    vwc_at_median: float = 0.25     # m3/m3 at a typical cell
    slope: float = -0.02            # VWC change per deg C of excess (drier = hotter)

    def vwc(self, excess_c):
        return np.clip(self.vwc_at_median + self.slope * np.asarray(excess_c), 0.02, 0.50)

    @classmethod
    def fit(cls, probe_excess_c, probe_vwc, prior: "MoistureModel | None" = None):
        """Fit from soil probes. Two or more probes fit both terms; one probe
        only shifts the offset and keeps the prior slope."""
        prior = prior or cls()
        e = np.asarray(probe_excess_c, float)
        v = np.asarray(probe_vwc, float)
        if len(e) >= 2 and np.ptp(e) > 0.5:
            slope, icpt = np.polyfit(e, v, 1)
            if slope > -0.005:              # must stay negative: hotter = drier
                slope = -0.005
                icpt = float(np.mean(v - slope * e))     # refit the offset with the clamped slope
            return cls(float(icpt), float(slope))
        if len(e) >= 1:
            return cls(float(np.mean(v - prior.slope * e)), prior.slope)
        return prior


def deficit_mm(vwc, target_vwc: float = 0.27, root_depth_mm: float = 100.0,
               min_mm: float = 2.0, max_mm: float = 15.0):
    """Water (mm) needed to raise the root zone to target. Small deficits are
    skipped (below min_mm); large ones capped so one night can't flood a cell."""
    d = (target_vwc - np.asarray(vwc)) * root_depth_mm
    d = np.where(d < min_mm, 0.0, d)
    return np.minimum(d, max_mm)


# ----------------------------------------------------------------------------- weather
def rain_window_mm(lat: float, lon: float, past_hours: int = 18, next_hours: int = 24,
                   timeout: float = 10.0):
    """(rain that fell in the last past_hours, rain forecast for the next
    next_hours), in mm, from Open-Meteo (free, no key). Rain since the
    afternoon scan matters as much as rain to come: the plan was made before it.
    Returns (None, None) if the request fails; the caller decides what to do."""
    url = ("https://api.open-meteo.com/v1/forecast"
           f"?latitude={lat:.4f}&longitude={lon:.4f}"
           f"&hourly=precipitation&past_hours={past_hours}&forecast_hours={next_hours}"
           "&timezone=UTC")
    try:
        with urllib.request.urlopen(url, timeout=timeout) as r:
            data = json.load(r)
        vals = [v or 0.0 for v in data["hourly"]["precipitation"]]
        return float(sum(vals[:past_hours])), float(sum(vals[past_hours:]))
    except Exception:
        return None, None


def wind_now_ms(lat: float, lon: float, timeout: float = 10.0):
    """(wind speed, gust) at 10 m in m/s right now, from Open-Meteo; (None, None) on failure.
    An independent check on the camera's wind estimate: if the check shots
    can't be seen, this is the only wind information the session has."""
    url = ("https://api.open-meteo.com/v1/forecast"
           f"?latitude={lat:.4f}&longitude={lon:.4f}"
           "&current=wind_speed_10m,wind_gusts_10m&wind_speed_unit=ms")
    try:
        with urllib.request.urlopen(url, timeout=timeout) as r:
            cur = json.load(r)["current"]
        return float(cur["wind_speed_10m"]), float(cur["wind_gusts_10m"])
    except Exception:
        return None, None


def rain_next_24h_mm(lat: float, lon: float, timeout: float = 10.0):
    """Forecast rain (mm) over the next 24 h; None if it can't be fetched."""
    return rain_window_mm(lat, lon, past_hours=0, next_hours=24, timeout=timeout)[1]


# ----------------------------------------------------------------------------- scan quality
@dataclass
class ScanQuality:
    spread_c: float        # interquartile range of cell temperatures
    valid_cells: int
    probes_used: int
    ok: bool
    reason: str


def scan_quality(cell_temps, probes_used: int, min_spread_c: float = 0.8,
                 min_valid_frac: float = 0.8) -> ScanQuality:
    """Is this afternoon scan good enough to plan from?

    Thermal contrast between wet and dry turf needs sun. Under cloud, after
    rain or with the lawn evenly watered, all cells read within sensor noise
    (~0.3 deg C after averaging), and a plan made from that noise would water
    a random half of the lawn. Below min_spread_c the scan is rejected."""
    t = np.asarray(cell_temps, float)
    good = t[np.isfinite(t)]
    if len(good) < max(4, min_valid_frac * len(t)):
        return ScanQuality(float("nan"), len(good), probes_used, False,
                           f"only {len(good)} of {len(t)} cells seen by the thermal camera")
    q75, q25 = np.percentile(good, [75, 25])
    spread = float(q75 - q25)
    if spread < min_spread_c:
        return ScanQuality(spread, len(good), probes_used, False,
                           f"thermal contrast {spread:.2f} C is below {min_spread_c} C (cloud, recent rain or even lawn)")
    return ScanQuality(spread, len(good), probes_used, True, "ok")


def min_excess_without_probes() -> float:
    """Without probe readings the moisture model has no absolute anchor: it
    only knows which cells are warmer than the median, not whether the median
    itself is wet or dry. Only clearly hot cells are watered then."""
    return 1.5


# ----------------------------------------------------------------------------- plan
@dataclass
class Task:
    cell: Cell
    pass_no: int
    seconds: float
    mm: float


def build_plan(cells, deficits, flow_lpm: float, cell_size: float,
               max_mm_per_pass: float = 5.0):
    """Seconds of spray per cell, split into passes of at most max_mm_per_pass.

    Splitting matters on clay soil (most of Houston): it absorbs only a few mm
    per hour, so one long soak runs off. Watering every cell once per pass
    gives each one time to soak in before the next pass.
    """
    q_lps = flow_lpm / 60.0
    area = cell_size * cell_size
    tasks = []
    remaining = {c.idx: float(d) for c, d in zip(cells, deficits) if d > 0}
    p = 0
    while remaining:
        for c in cells:
            if c.idx not in remaining:
                continue
            mm = min(remaining[c.idx], max_mm_per_pass)
            tasks.append(Task(c, p, mm * area / q_lps, mm))
            remaining[c.idx] -= mm
            if remaining[c.idx] <= 1e-9:
                del remaining[c.idx]
        p += 1
    return tasks


def sub_targets(cell: Cell, cell_size: float):
    """Four aim points inside a cell, so the stream spreads instead of
    drilling one hole. Each gets a quarter of the cell's spray time."""
    q = cell_size / 4
    return [(cell.x - q, cell.y - q), (cell.x + q, cell.y - q),
            (cell.x + q, cell.y + q), (cell.x - q, cell.y + q)]
