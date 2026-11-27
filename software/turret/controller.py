"""The control loop: calibrate once, then scan, plan and water each day."""
from __future__ import annotations

import logging
from dataclasses import dataclass, field

from .ballistics import JetModel
from .calibrate import Shot, WindEstimator, default_test_pattern, fit_jet
import numpy as np

from .planner import (MoistureModel, build_plan, deficit_mm, lawn_cells, min_excess_without_probes,
                      scan_quality, sub_targets)
from .vision import (GroundMapper, detect_landing, dryness_scores, thermal_cell_temps,
                     thermal_cell_temps_multi)

log = logging.getLogger("turret")


@dataclass
class Settings:
    cell_size: float = 1.0          # m
    flow_lpm: float = 7.0           # measured with a bucket at the regulator pressure
    burst_s: float = 1.0            # length of a verification / test burst
    verify_every: int = 4           # check the landing on every Nth aim point
    max_wind: float = 4.0           # m/s: above this, stop for the day
    max_miss_m: float = 0.6         # a verified miss larger than this triggers a re-aim
    max_missed_checks: int = 4      # consecutive check shots not seen -> stop (aim and wind unverified)
    max_outliers: int = 3           # consecutive implausible detections -> stop
    dose_tolerance: float = 0.25    # delivered volume must be within 25% of requested
    max_mm_per_pass: float = 5.0
    target_vwc: float = 0.27
    root_depth_mm: float = 100.0


@dataclass
class Log:
    calib_rms: float | None = None
    verified: list = field(default_factory=list)   # (target, observed, predicted)
    watered: list = field(default_factory=list)    # (cell idx, mm, seconds)
    aims: list = field(default_factory=list)       # ((tx, ty), (pan, tilt)) per spray
    stopped: str | None = None


class SessionStop(RuntimeError):
    """Raised inside water() to end the session safely with a logged reason."""


class Controller:
    def __init__(self, hw, model: JetModel, cam_map: GroundMapper,
                 thermal_map=None, settings: Settings = Settings(), p_cal_psi: float | None = None):
        """thermal_map: one GroundMapper, or {pan_stop_deg: GroundMapper} for a panned thermal camera.
        p_cal_psi: supply pressure when the jet was calibrated; the first check shot of a
        session reads the pressure with water flowing and rescales v0 by sqrt(p/p_cal)."""
        self.hw, self.model, self.cam_map, self.th_map = hw, model, cam_map, thermal_map
        self.s = settings
        self.wind = WindEstimator()
        self.log = Log()
        self.p_cal = p_cal_psi
        self.p_session = None          # pressure read during the first check shot of a session
        self.p_measured = None         # pressure read during calibration
        self._need_pressure = p_cal_psi is not None
        self._calibrating = False
        self._missed = 0
        self._outliers = 0
        self._wet = []                 # aim points already watered this session

    # ---------------------------------------------------------------- one shot
    def _burst_and_locate(self, pan, tilt, expected, seconds, during=False):
        """Fire a burst, return the observed landing point or None.

        during=False (default) grabs the frame about 0.7 s after the valve
        closes: the airborne stream is gone and only the wet spot remains.
        A frame taken while spraying would show the stream too, and from the
        Sensor Unit's viewpoint the stream's image runs into the splash and
        drags the blob centre toward the turret (design review, rev B).
        during=True is kept for experiments only."""
        hw = self.hw
        hw.move(pan, tilt)
        before = hw.capture()
        hw.valve(True)
        if self._need_pressure and hasattr(hw, "pressure_psi"):
            hw.sleep(min(seconds, 0.8))
            self._need_pressure = False
            psi = hw.pressure_psi()
            if self._calibrating:
                self.p_measured = psi
            else:
                self._apply_pressure(psi)
            hw.sleep(max(seconds - 0.8, 0.0))
        else:
            hw.sleep(seconds)
        if during:
            after = hw.capture()
            hw.valve(False)
        else:
            hw.valve(False)
            hw.sleep(0.7)
            after = hw.capture()
        hit = detect_landing(before, after, self.cam_map, expected_xy=expected)
        return None if hit is None else hit[:2]

    def _apply_pressure(self, psi):
        """Rescale exit speed for today's supply pressure (v0 ~ sqrt(p))."""
        if psi < 10:
            raise SessionStop(f"supply pressure {psi:.1f} psi with the valve open: is the spigot on?")
        self.p_session = psi
        self.model = self.model.copy(v0=self.model.v0 * (psi / self.p_cal) ** 0.5)
        log.info("pressure %.1f psi (calibrated at %.1f): v0 now %.2f m/s", psi, self.p_cal, self.model.v0)

    def _check_shot(self, pan, tilt, pred, burst, tx, ty):
        """One verification burst. Returns the possibly re-aimed (pan, tilt, pred).
        Raises SessionStop when the aim can't be trusted any more."""
        obs = self._burst_and_locate(pan, tilt, pred, burst)
        if obs is None:
            self._missed += 1
            if self._missed >= self.s.max_missed_checks:
                raise SessionStop(f"{self._missed} check shots in a row not seen by the camera; "
                                  "aim and wind cannot be verified")
            return pan, tilt, pred
        self._missed = 0
        rng = (pred[0] ** 2 + pred[1] ** 2) ** 0.5
        innov = ((obs[0] - pred[0]) ** 2 + (obs[1] - pred[1]) ** 2) ** 0.5
        if innov > 0.6 + 0.08 * rng:               # implausible jump: a false detection, not wind
            self._outliers += 1
            log.warning("check shot rejected: %.2f m from prediction", innov)
            if self._outliers >= self.s.max_outliers:
                raise SessionStop(f"{self._outliers} implausible check shots in a row")
            return pan, tilt, pred
        self._outliers = 0
        self.log.verified.append(((tx, ty), obs, pred))
        self.model = self.wind.update(self.model, Shot(pan, tilt, *obs))
        if WindEstimator.speed(self.model) > self.s.max_wind:
            raise SessionStop(f"wind {WindEstimator.speed(self.model):.1f} m/s")
        miss = ((obs[0] - tx) ** 2 + (obs[1] - ty) ** 2) ** 0.5
        if miss > self.s.max_miss_m:
            sol = self.model.aim(tx, ty)            # re-aim with the new wind estimate
            if sol is not None:
                pan, tilt, pred = sol
        return pan, tilt, pred

    def _deliver(self, litres, seconds):
        """Put `litres` on the current aim point. Real hardware meters it by flow."""
        if hasattr(self.hw, "dose"):
            ml, ms, reason = self.hw.dose(litres, max_s=1.3 * seconds + 2)
            if reason != "DONE" or abs(ml / 1000.0 - litres) > self.s.dose_tolerance * max(litres, 0.2):
                raise SessionStop(f"dose ended {reason}: {ml:.0f} mL of {litres * 1000:.0f} mL in {ms} ms")
        else:
            self.hw.valve(True)
            self.hw.sleep(seconds)
            self.hw.valve(False)

    def _deliver_cell(self, aims, litres_each, seconds_each):
        """Water several aim points in one cell with ONE valve opening: start
        a metered dose for the total, and step the head to the next aim point
        each time the meter passes the next share. Fewer valve cycles means
        four times less water hammer and valve wear (design review, rev B).
        aims: [(pan, tilt), ...] already solved."""
        total = litres_each * len(aims)
        self.hw.move(*aims[0])
        did = self.hw.dose_start(total, max_s=1.3 * seconds_each * len(aims) + 2)
        k = 1
        while True:
            self.hw.sleep(0.25)
            running, ml, ms, reason = self.hw.dose_poll(did)
            if not running:
                break
            if k < len(aims) and ml >= 1000.0 * litres_each * k:
                self.hw.move(*aims[k])
                k += 1
        if reason != "DONE" or abs(ml / 1000.0 - total) > self.s.dose_tolerance * max(total, 0.2):
            raise SessionStop(f"dose ended {reason}: {ml:.0f} mL of {total * 1000:.0f} mL in {ms} ms")

    # ---------------------------------------------------------------- calibrate
    def calibrate(self, pattern=None):
        """Fire a spread of test shots, locate each one, fit the jet model.
        Run in calm air on a dry lawn (late morning)."""
        pattern = pattern or default_test_pattern(self.model)
        self._calibrating, self._need_pressure = True, True     # record pressure on the first shot
        shots = []
        for pan, tilt in pattern:
            # No search radius: before calibration the prediction can be metres off.
            hit = self._burst_and_locate(pan, tilt, None, self.s.burst_s, during=False)
            if hit is None:
                log.warning("no landing found for pan %.1f tilt %.1f", pan, tilt)
                continue
            shots.append(Shot(pan, tilt, *hit))
            self.hw.sleep(5.0)       # let the spot settle so shots don't merge
        if len(shots) < 6:
            raise RuntimeError(f"only {len(shots)} shots located; check camera mapping")
        self._calibrating = False
        self.model, rms = fit_jet(shots, self.model)
        self.log.calib_rms = rms
        log.info("calibrated: v0=%.2f m/s k=%.3f /m trims=(%.2f, %.2f) deg rms=%.2f m",
                 self.model.v0, self.model.k, self.model.pan_trim, self.model.tilt_trim, rms)
        return shots

    # ---------------------------------------------------------------- scan
    def scan(self, cells, probes=()):
        """Thermal scan -> deficit (mm) per cell.

        probes: iterable of (cell_index, measured_vwc) from soil probes."""
        pts = [(c.x, c.y) for c in cells]
        if isinstance(self.th_map, dict):          # thermal camera on a pan stage
            frames = self.hw.capture_thermal_stops(sorted(self.th_map))
            temps = thermal_cell_temps_multi(frames, self.th_map, pts)
        else:
            temps = thermal_cell_temps(self.hw.capture_thermal(), self.th_map, pts)
        excess = dryness_scores(temps)
        probes = [(i, v) for i, v in probes if np.isfinite(excess[i])]
        self.last_quality = scan_quality(temps, len(probes))
        mm = MoistureModel.fit([excess[i] for i, _ in probes], [v for _, v in probes])
        vwc = mm.vwc(np.nan_to_num(excess, nan=-99.0))          # unseen cells read as wet: never watered blind
        deficits = deficit_mm(vwc, self.s.target_vwc, self.s.root_depth_mm)
        if not self.last_quality.ok:
            log.warning("scan rejected: %s", self.last_quality.reason)
            deficits = np.zeros_like(deficits)
        elif not probes:
            # No absolute anchor: water only clearly hot cells (see planner.min_excess_without_probes).
            deficits = np.where(np.nan_to_num(excess, nan=-99.0) >= min_excess_without_probes(), deficits, 0.0)
        return deficits, excess, mm

    # ---------------------------------------------------------------- water
    def water(self, tasks):
        """Run the plan. A check shot is due every Nth aim point, but is only
        fired on turf still dry this session (at least 0.45 m from anything
        already watered): on wet turf a new wet spot barely shows, so the
        camera would miss it. Any SessionStop ends the session cleanly with
        the valves shut and the reason in the log."""
        since_check = self.s.verify_every
        try:
            for task in tasks:
                pts = sub_targets(task.cell, self.s.cell_size)
                t_each = task.seconds / len(pts)
                pending = []                       # aim points for the one-opening dose
                for tx, ty in pts:
                    sol = self.model.aim(tx, ty)
                    if sol is None:
                        continue
                    pan, tilt, pred = sol
                    rest = t_each
                    dry = all((tx - wx) ** 2 + (ty - wy) ** 2 > 0.45 ** 2 for wx, wy in self._wet)
                    if since_check >= self.s.verify_every and dry:
                        burst = min(self.s.burst_s, t_each)
                        pan, tilt, pred = self._check_shot(pan, tilt, pred, burst, tx, ty)
                        rest = t_each - burst
                        since_check = 0
                    since_check += 1
                    self._wet.append((tx, ty))
                    self.log.aims.append(((tx, ty), (pan, tilt)))
                    if rest <= 0.05:
                        continue
                    if hasattr(self.hw, "dose_start"):
                        pending.append(((pan, tilt), rest))
                    else:                          # simulator: one burst per aim point
                        self.hw.move(pan, tilt)
                        self._deliver(rest * self.s.flow_lpm / 60.0, rest)
                if pending:
                    # aim points after a check shot get slightly less; meter the average share
                    secs = sum(r for _, r in pending) / len(pending)
                    self._deliver_cell([a for a, _ in pending], secs * self.s.flow_lpm / 60.0, secs)
                self.log.watered.append((task.cell.idx, task.mm, task.seconds))
        except SessionStop as e:
            self.log.stopped = str(e)
            log.warning("session stopped: %s", e)
            try:
                self.hw.valve(False)
            except Exception:
                pass
        return self.log

    # ---------------------------------------------------------------- daily run
    def daily(self, polygon, probes=(), rain_mm=None, rain_skip_mm=6.0):
        if rain_mm is not None and rain_mm >= rain_skip_mm:
            self.log.stopped = f"skipped: {rain_mm:.1f} mm rain forecast"
            return self.log, []
        cells = lawn_cells(polygon, self.s.cell_size, self.model)
        deficits, excess, _ = self.scan(cells, probes)
        tasks = build_plan(cells, deficits, self.s.flow_lpm, self.s.cell_size, self.s.max_mm_per_pass)
        return self.water(tasks), tasks
