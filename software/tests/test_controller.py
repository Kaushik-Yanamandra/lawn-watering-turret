"""Controller tests on the real-hardware code path (metered doses, one valve
opening per cell, pressure read during the first check shot), using the
simulator plus a fake Turret Node dose engine.

    python tests/test_controller.py
"""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

import simulate as S  # noqa: E402
from turret.controller import Controller, Settings  # noqa: E402
from turret.hardware import SimTurret  # noqa: E402
from turret.planner import build_plan, lawn_cells  # noqa: E402


class NodeSim(SimTurret):
    """SimTurret plus the rev B node's dose engine and pressure sensor.
    meter_gain < 1 models a flow meter that under-reads."""

    def __init__(self, world, meter_gain=1.0, psi=22.0, **kw):
        super().__init__(world, **kw)
        self.meter_gain, self.psi = meter_gain, psi
        self.d = None
        self.valve_cycles = 0
        self.moves_while_open = 0

    def valve(self, on):
        if on and self.valve_open_t is None:
            self.valve_cycles += 1
        super().valve(on)

    def move(self, pan, tilt):
        if self.valve_open_t is not None:
            self.moves_while_open += 1
            super().valve(False)                 # paint the water so far at the old aim
            super().move(pan, tilt)
            super().valve(True)
            self.valve_cycles -= 1               # same physical opening
        else:
            super().move(pan, tilt)

    def pressure_psi(self):
        return self.psi

    def dose_start(self, litres, max_s):
        self.valve(True)
        self.d = dict(t0=self.t, target_ml=litres * 1000, max_s=max_s, done=None)
        return 1

    def dose_poll(self, did):
        d = self.d
        ml = self.w.flow_lpm / 60 * (self.t - d["t0"]) * 1000 * self.meter_gain
        if d["done"] is None:
            if ml >= d["target_ml"]:
                d["done"] = "DONE"
            elif self.t - d["t0"] > d["max_s"]:
                d["done"] = "TIMEOUT"
            if d["done"]:
                self.valve(False)
                d["ml"], d["ms"] = ml, int((self.t - d["t0"]) * 1000)
        if d["done"]:
            return False, d["ml"], d["ms"], d["done"]
        return True, ml, int((self.t - d["t0"]) * 1000), "-"


def run(meter_gain=1.0, psi=22.0):
    world, hw0 = S.make_world()
    cam, th = S.mappers(world, hw0)
    c0 = Controller(hw0, S.NOMINAL.copy(), cam, th, Settings(flow_lpm=S.FLOW))
    c0.calibrate()
    w = S.SimWorld(true_jet=S.TRUE.copy(), flow_lpm=S.FLOW, seed=5,
                   cam_ground=[(0.2, -7.0), (0.2, 7.0), (12.0, -12.0), (12.0, 12.0)])
    hw = NodeSim(w, meter_gain=meter_gain, psi=psi, temp_field=S.surface_temp)
    cam, th = S.mappers(w, hw)
    c = Controller(hw, c0.model.copy(), cam, th, Settings(flow_lpm=S.FLOW), p_cal_psi=22.0)
    cells = lawn_cells(S.LAWN, 1.0, c.model)
    probes = [(i, 0.25) for i in (5, len(cells) // 2)]
    deficits, _, _ = c.scan(cells, probes)
    tasks = build_plan(cells, deficits, S.FLOW, 1.0)
    log = c.water(tasks)
    return c, hw, log, tasks


def test_hardware_path_completes_with_one_opening_per_cell():
    c, hw, log, tasks = run()
    assert log.stopped is None, log.stopped
    assert len(log.watered) == len(tasks)
    assert hw.moves_while_open > 0                       # head stepped through aim points under flow
    checks = len(log.verified)
    assert hw.valve_cycles <= len(tasks) + checks + 2, (hw.valve_cycles, len(tasks), checks)
    assert c.p_session == 22.0


def test_under_reading_meter_stops_session():
    c, hw, log, tasks = run(meter_gain=0.5)
    assert log.stopped and "TIMEOUT" in log.stopped, log.stopped
    assert hw.valve_open_t is None                       # valves shut


def test_lower_pressure_rescales_jet():
    c, hw, log, tasks = run(psi=18.0)
    assert math.isclose(c.p_session, 18.0)


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print("pass", name)
