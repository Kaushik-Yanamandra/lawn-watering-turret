"""Command-line entry point on the Raspberry Pi (rev B).

    python run_pi.py markers     # one-off: build camera mappings from marker clicks
    python run_pi.py calibrate   # one-off (and monthly): fit the jet model
    python run_pi.py scan        # early afternoon: thermal scan, saves the next dawn's plan
    python run_pi.py water       # before dawn: waits for sunrise, checks rain and wind, runs the plan
    python run_pi.py status      # print the Turret Node's status and faults
    python run_pi.py clear-stuck # after you have checked a stuck-open valve by hand

Schedule scan and water with cron. flock stops two jobs sharing the RS-485 bus:
    15 13 * * *  cd /home/pi/turret && flock -n /tmp/turret.lock python run_pi.py scan
    30 5  * * *  cd /home/pi/turret && flock -n /tmp/turret.lock python run_pi.py water

`water` starts at 05:30 and sleeps until 15 minutes after today's sunrise
(computed locally): the check-shot camera needs daylight, and in Houston
05:30 is dark on every day of the year.

Everything the commands need is in config.json (see CONFIG below).
The Pi talks to the ESP32 Turret Node over RS-485; see firmware/turret_node.
"""
from __future__ import annotations

import json
import logging
import sys
import time
import urllib.request
from dataclasses import asdict
from datetime import date, datetime, timedelta, timezone
from pathlib import Path

from turret.ballistics import JetModel
from turret.controller import Controller, Settings
from turret.hardware import Rig, SensorUnit, TurretLink
from turret.planner import Cell, Task, build_plan, lawn_cells, rain_window_mm, wind_now_ms
from turret.sun import watering_start_utc
from turret.vision import GroundMapper, LensModel

HERE = Path(__file__).parent
CONFIG = HERE / "config.json"
MODEL = HERE / "jet_model.json"
PLAN = HERE / "plan.json"
PCAL = HERE / "p_cal.json"

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s",
                    handlers=[logging.FileHandler(HERE / "turret.log"), logging.StreamHandler()])
log = logging.getLogger("run_pi")

# Example config.json:
# {
#   "lat": 29.76, "lon": -95.37,
#   "lawn": [[1.0,-6],[8,-6],[8.5,-2],[8.5,3],[7,6.5],[1.0,6.5]],   # keep within the MEASURED reach, >= 1 m from the post
#   "markers_ground": [[1.5,-4],[1.5,4],[5,0],[9,-6],[9,6],[11,0]],
#   "markers_cam_px": [[...], ...],        # filled by `markers`
#   "lens": "lens.npz",                    # from tools/calibrate_lens.py (strongly recommended)
#   "thermal_stops": [                     # one entry per pan stop, filled by `markers`
#       {"angle": -35, "ground": [[...], ...], "px": [[...], ...]},
#       {"angle": 0,   "ground": [[...], ...], "px": [[...], ...]},
#       {"angle": 35,  "ground": [[...], ...], "px": [[...], ...]}],
#   "probes": [{"xy": [3.0, 4.0], "dry_mv": 2650, "wet_mv": 1300},
#              {"xy": [6.0, -3.0], "dry_mv": 2640, "wet_mv": 1290}],    # put one in shade
#   "nozzle_height_m": 1.02, "flow_lpm": 7.0,
#   "rain_since_scan_skip_mm": 3, "rain_forecast_skip_mm": 6,
#   "max_wind_ms": 4, "max_gust_ms": 7,
#   "plan_max_age_h": 20, "start_after_sunrise_min": 15, "latest_start_after_sunrise_min": 120,
#   "alert_url": "https://ntfy.sh/your-private-topic",   # optional: phone push for problems
#   "rs485_port": "/dev/ttyUSB0"
# }


# ---------------------------------------------------------------- helpers
def load_cfg():
    return json.loads(CONFIG.read_text())


def alert(cfg, msg):
    """Log a problem and, if configured, push it to a phone (ntfy.sh topic
    URL or any endpoint that accepts a plain-text POST)."""
    log.warning("ALERT: %s", msg)
    url = cfg.get("alert_url")
    if url:
        try:
            urllib.request.urlopen(urllib.request.Request(url, data=("Lawn turret: " + msg).encode()), timeout=10)
        except Exception as e:                          # never let an alert failure hide the real problem
            log.warning("alert could not be sent: %s", e)


def load_model(cfg):
    if MODEL.exists():
        return JetModel(**json.loads(MODEL.read_text()))
    return JetModel(h=cfg.get("nozzle_height_m", 1.02))


def save_model(m: JetModel, p_cal_psi=None):
    d = {k: v for k, v in asdict(m).items() if not k.startswith("_")}
    d["wind_x"] = d["wind_y"] = 0.0
    MODEL.write_text(json.dumps(d, indent=2))
    if p_cal_psi is not None:
        PCAL.write_text(json.dumps({"psi": p_cal_psi}))


def controller(cfg, hw):
    lens = LensModel.load(HERE / cfg["lens"]) if cfg.get("lens") else None
    if lens is None:
        log.warning("no lens calibration: ground mapping is less accurate near the frame edges")
    cam = GroundMapper(cfg["markers_cam_px"], cfg["markers_ground"], lens=lens)
    th = {st["angle"]: GroundMapper(st["px"], st["ground"]) for st in cfg["thermal_stops"]}
    p_cal = json.loads(PCAL.read_text())["psi"] if PCAL.exists() else None
    return Controller(hw, load_model(cfg), cam, th, Settings(flow_lpm=cfg.get("flow_lpm", 7.0)), p_cal_psi=p_cal)


def check_node(cfg, hw) -> bool:
    """Look at the node's latched faults before using water. Transient faults
    from the last session (link, time limit, no flow, low pressure, high flow)
    are logged, reported and cleared once. STUCK_OPEN is never cleared
    automatically: someone has to look at the valve."""
    st = hw.status()
    if not st.faults:
        return True
    names = ", ".join(st.fault_names())
    if st.faults & 8:
        alert(cfg, f"STUCK_OPEN latched (faults: {names}). Water may be leaking past the valves. "
                   "Check the valves, then run `python run_pi.py clear-stuck`.")
        return False
    alert(cfg, f"clearing faults from the last run: {names}")
    hw.clear_faults()
    return True


# ---------------------------------------------------------------- commands
def cmd_markers(cfg, hw):
    """Save one visible frame and one thermal frame per pan stop. Click the
    markers in an image viewer and paste pixel coordinates into config.json:
    markers_cam_px in the order of markers_ground (raw pixels; the lens model
    is applied in code), and for each thermal stop the markers that stop can
    see. Warm markers (cups of hot water) show up in the thermal frames."""
    import cv2
    import numpy as np
    cv2.imwrite(str(HERE / "markers_cam.png"), hw.capture()[:, :, ::-1])
    stops = [st["angle"] for st in cfg.get("thermal_stops", [])] or [-35, 0, 35]
    for a, t in hw.capture_thermal_stops(stops).items():
        t8 = cv2.normalize(t, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
        cv2.imwrite(str(HERE / f"markers_thermal_{a:+d}.png"),
                    cv2.resize(t8, (320, 240), interpolation=cv2.INTER_NEAREST))
    print("Saved markers_cam.png and markers_thermal_<stop>.png (thermal scaled x10).")


def cmd_calibrate(cfg, hw):
    """Calm, dry late morning. The pressure is read during the first test
    shot (no separate squirt at the park position)."""
    if not check_node(cfg, hw):
        return
    c = controller(cfg, hw)
    c.calibrate()
    save_model(c.model, p_cal_psi=c.p_measured)
    log.info("calibration saved; RMS %.3f m; pressure %s psi", c.log.calib_rms, c.p_measured)
    if c.log.calib_rms > 0.15:
        alert(cfg, f"calibration RMS {c.log.calib_rms:.2f} m is high: check markers, lens file and wind")


def cmd_scan(cfg, hw):
    c = controller(cfg, hw)
    cells = lawn_cells(cfg["lawn"], c.s.cell_size, c.model)
    probes = read_probes(cfg, hw, cells)
    deficits, _, mm = c.scan(cells, probes)
    q = c.last_quality
    tasks = build_plan(cells, deficits, c.s.flow_lpm, c.s.cell_size, c.s.max_mm_per_pass)
    PLAN.write_text(json.dumps({
        "scanned_utc": datetime.now(timezone.utc).isoformat(),
        "quality": {"ok": q.ok, "spread_c": q.spread_c, "probes": q.probes_used, "reason": q.reason},
        "tasks": [{"idx": t.cell.idx, "x": t.cell.x, "y": t.cell.y, "pass": t.pass_no,
                   "seconds": t.seconds, "mm": t.mm} for t in tasks]}))
    log.info("scan: %s; plan %d sprays, %.0f L", q.reason, len(tasks), sum(t.mm * c.s.cell_size ** 2 for t in tasks))
    if not probes:
        alert(cfg, "no soil probe readings: only clearly hot cells will be watered")


def cmd_water(cfg, hw):
    lat, lon = cfg["lat"], cfg["lon"]
    # 1. wait for light: the check-shot camera can't see splashes before sunrise
    now = datetime.now(timezone.utc)
    start = watering_start_utc(date.today(), lat, lon, cfg.get("start_after_sunrise_min", 15))
    latest = start + timedelta(minutes=cfg.get("latest_start_after_sunrise_min", 120)
                               - cfg.get("start_after_sunrise_min", 15))
    if now > latest:
        log.info("too late after sunrise to start (wind and evaporation rise); skipping")
        return
    if now < start:
        log.info("waiting %.0f min for daylight (start %s UTC)", (start - now).total_seconds() / 60, start.strftime("%H:%M"))
        time.sleep((start - now).total_seconds())

    # 2. the plan must exist and come from the most recent afternoon scan
    if not PLAN.exists():
        log.info("no plan (scan did not run or found nothing); skipping")
        return
    plan = json.loads(PLAN.read_text())
    age_h = (datetime.now(timezone.utc) - datetime.fromisoformat(plan["scanned_utc"])).total_seconds() / 3600
    if age_h > cfg.get("plan_max_age_h", 20):
        alert(cfg, f"plan is {age_h:.0f} h old (did the afternoon scan fail?); not watering from it")
        return
    if not plan["quality"]["ok"]:
        log.info("yesterday's scan was rejected (%s); skipping", plan["quality"]["reason"])
        return

    # 3. weather: rain since the scan, rain to come, wind now
    past, future = rain_window_mm(lat, lon, past_hours=min(24, int(age_h) + 1))
    if past is None:
        log.warning("rain data unavailable; watering as planned")
    elif past >= cfg.get("rain_since_scan_skip_mm", 3) or future >= cfg.get("rain_forecast_skip_mm", 6):
        log.info("skipping: %.1f mm rain since the scan, %.1f mm forecast", past, future)
        PLAN.rename(PLAN.with_suffix(".skipped"))
        return
    wind, gust = wind_now_ms(lat, lon)
    if wind is not None and (wind > cfg.get("max_wind_ms", 4) or gust > cfg.get("max_gust_ms", 7)):
        log.info("skipping: wind %.1f m/s, gusts %.1f m/s", wind, gust)
        return

    # 4. the node must be healthy
    if not check_node(cfg, hw):
        return

    tasks = [Task(Cell(d["idx"], d["x"], d["y"]), d["pass"], d["seconds"], d["mm"]) for d in plan["tasks"]]
    c = controller(cfg, hw)
    result = c.water(tasks)
    PLAN.rename(PLAN.with_suffix(".done"))           # never water the same plan twice
    log.info("watered %d of %d tasks; %d check shots verified; stopped: %s",
             len(result.watered), len(tasks), len(result.verified), result.stopped)
    if result.stopped:
        alert(cfg, f"watering stopped early after {len(result.watered)} of {len(tasks)} tasks: {result.stopped}")


def cmd_status(cfg, hw):
    st = hw.status()
    print(st)
    print("faults:", ", ".join(st.fault_names()) or "none")


def cmd_clear_stuck(cfg, hw):
    hw.clear_faults(stuck=True)
    print("STUCK_OPEN cleared. Watch the flow meter (status) for a minute to confirm no leak.")


def read_probes(cfg, hw, cells):
    """[(cell_index, vwc)] from the soil probes on the Turret Node.

    Capacitive probes read lower millivolts in wetter soil. Each probe is
    calibrated in air-dry soil (dry_mv) and saturated soil (wet_mv); VWC is
    interpolated linearly between 0.05 and 0.45 m3/m3."""
    mv = hw.status().probe_mv
    out = []
    for i, p in enumerate(cfg.get("probes", [])[:3]):
        frac = (p["dry_mv"] - mv[i]) / (p["dry_mv"] - p["wet_mv"])
        if not -0.2 < frac < 1.2:          # unplugged or broken probe
            continue
        vwc = 0.05 + 0.40 * min(max(frac, 0.0), 1.0)
        x, y = p["xy"]
        nearest = min(cells, key=lambda c: (c.x - x) ** 2 + (c.y - y) ** 2)
        out.append((nearest.idx, vwc))
    return out


if __name__ == "__main__":
    cmds = {"markers": cmd_markers, "calibrate": cmd_calibrate, "scan": cmd_scan, "water": cmd_water,
            "status": cmd_status, "clear-stuck": cmd_clear_stuck}
    if len(sys.argv) != 2 or sys.argv[1] not in cmds:
        sys.exit(__doc__)
    cfg = load_cfg()
    hw = Rig(SensorUnit(), TurretLink(cfg.get("rs485_port", "/dev/ttyUSB0")))
    try:
        cmds[sys.argv[1]](cfg, hw)
    except Exception as e:
        alert(cfg, f"{sys.argv[1]} failed: {e!r}")
        raise
    finally:
        hw.close()                                    # PARK: valves shut, nozzle down
