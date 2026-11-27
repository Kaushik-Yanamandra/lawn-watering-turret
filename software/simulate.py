"""Desktop end-to-end test of the turret, no hardware needed.

    python simulate.py

What it does
1. Builds a simulated yard with a hidden "true" jet that differs from the
   nominal model (lower pressure, more drag, a crooked mount).
2. Calibrates through the camera exactly as the real thing would.
3. Measures aim error on 40 random targets: nominal model vs calibrated.
4. Runs a full dawn watering session in a 2.5 m/s crosswind with gusts,
   with and without the camera's online wind correction.
5. Saves sim_results.png and prints a summary.
"""
from __future__ import annotations

import logging
import math
import sys

import numpy as np

from turret.ballistics import JetModel, nozzle_flow_lpm
from turret.controller import Controller, Settings
from turret.hardware import SimTurret, SimWorld
from turret.planner import build_plan, lawn_cells
from turret.vision import GroundMapper

logging.basicConfig(level=logging.WARNING, format="%(message)s")
rng = np.random.default_rng(7)

# --------------------------------------------------------------------------- yard
LAWN = [(0.6, -6.0), (8.0, -6.0), (9.5, -2.0), (9.5, 3.0), (7.0, 6.5), (0.6, 6.5)]
DRY_PATCHES = [(6.0, -3.0, 4.0, 1.5), (3.0, 4.0, 3.0, 1.0), (8.0, 2.0, 2.5, 1.2)]  # x, y, +degC, sigma


def surface_temp(x, y):
    t = 31.0 + 0.08 * x
    for px, py, amp, s in DRY_PATCHES:
        t += amp * math.exp(-((x - px) ** 2 + (y - py) ** 2) / (2 * s * s))
    return t


TRUE = JetModel(v0=16.8, k=0.13, pan_trim=2.5, tilt_trim=-1.5)   # hidden from the controller
NOMINAL = JetModel()                                             # what we'd guess from the datasheet
FLOW = round(nozzle_flow_lpm(25, 3.0), 1)


def make_world(wind=(0.0, 0.0), gust=0.0, seed=1):
    world = SimWorld(true_jet=TRUE.copy(wind_x=wind[0], wind_y=wind[1]), gust_std=gust,
                     flow_lpm=FLOW, seed=seed,
                     cam_ground=[(0.2, -7.0), (0.2, 7.0), (12.0, -12.0), (12.0, 12.0)])
    return world, SimTurret(world, temp_field=surface_temp)


def mappers(world, hw, pixel_noise=1.0):
    """What the user does once: put out 6 markers, measure them, click them."""
    import cv2
    markers = [(1.5, -4.0), (1.5, 4.0), (5.0, 0.0), (9.0, -6.0), (9.0, 6.0), (11.0, 0.0)]
    canvas = np.float64([[(x - world.x0) * world.ppm, (y - world.y0) * world.ppm] for x, y in markers])
    px = cv2.perspectiveTransform(canvas.reshape(-1, 1, 2), hw.H_cam).reshape(-1, 2)
    px += pixel_noise * rng.standard_normal(px.shape)
    cam = GroundMapper(px, markers)
    th = GroundMapper(world.th_pixels, world.cam_ground)
    return cam, th


RNG = np.random.default_rng(11)
HW_NOISE = SimWorld(true_jet=None)   # only its noise defaults are used: same servo / pressure noise as SimTurret


def noisy_landing(truth, pan, tilt, gust=0.0):
    """Where the water really lands: servo error on both axes, shot-to-shot
    exit speed scatter and (optionally) a gust. Rev A scored misses against
    the noiseless model, which made 'calibrated, still air' look like 2 cm."""
    t = truth.copy(v0=truth.v0 * (1 + HW_NOISE.v0_std * RNG.standard_normal()),
                   wind_x=truth.wind_x + gust * RNG.standard_normal(),
                   wind_y=truth.wind_y + gust * RNG.standard_normal())
    lx, ly, _ = t.landing(pan + HW_NOISE.angle_std * RNG.standard_normal(),
                          tilt + HW_NOISE.angle_std * RNG.standard_normal())
    return lx, ly


def miss_stats(model, targets, truth, draws=5):
    misses = []
    for tx, ty in targets:
        sol = model.aim(tx, ty)
        if sol is None:
            continue
        for _ in range(draws):
            lx, ly = noisy_landing(truth, sol[0], sol[1])
            misses.append(math.hypot(lx - tx, ly - ty))
    return np.array(misses)


def random_targets(n, model):
    r_min, r_max, _ = model.reach()
    out = []
    while len(out) < n:
        r = rng.uniform(r_min + 0.5, r_max - 0.8)
        a = rng.uniform(-55, 55)
        out.append((r * math.cos(math.radians(a)), r * math.sin(math.radians(a))))
    return out


def main():
    # ---- 1. calibrate in calm air -----------------------------------------
    world, hw = make_world()
    cam, th = mappers(world, hw)
    ctl = Controller(hw, NOMINAL.copy(), cam, th, Settings(flow_lpm=FLOW))
    shots = ctl.calibrate()
    fit = ctl.model
    print(f"Flow at 25 psi through 3 mm: {FLOW} L/min")
    print(f"Calibration: {len(shots)} shots located, rms residual {ctl.log.calib_rms:.3f} m")
    print("              true    nominal  fitted")
    for n in ("v0", "k", "pan_trim", "tilt_trim"):
        print(f"  {n:9s} {getattr(TRUE, n):7.3f} {getattr(NOMINAL, n):8.3f} {getattr(fit, n):7.3f}")
    print(f"  reach (m): true {TRUE.reach()[0]:.2f}-{TRUE.reach()[1]:.2f}, "
          f"fitted {fit.reach()[0]:.2f}-{fit.reach()[1]:.2f}")

    # ---- 2. aim accuracy in still air ----------------------------------------
    targets = random_targets(40, TRUE)
    m_nom = miss_stats(NOMINAL, targets, TRUE)
    m_fit = miss_stats(fit, targets, TRUE)

    # ---- 3. dawn session in a crosswind -------------------------------------
    wind, gust = (0.0, 2.5), 0.5
    results = {}
    for label, verify_every in (("no wind correction", 10 ** 9), ("camera wind correction", 4)):
        world_w, hw_w = make_world(wind=wind, gust=gust, seed=3)
        cam_w, th_w = mappers(world_w, hw_w)
        c = Controller(hw_w, fit.copy(), cam_w, th_w, Settings(flow_lpm=FLOW, verify_every=verify_every))
        cells = lawn_cells(LAWN, 1.0, c.model)
        probe_cells = [5, len(cells) // 2, len(cells) - 6]
        probes = [(i, float(np.clip(0.27 - 0.022 * (surface_temp(cells[i].x, cells[i].y) - 31.4), 0.05, 0.4)))
                  for i in probe_cells]
        deficits, excess, mmodel = c.scan(cells, probes)
        tasks = build_plan(cells, deficits, FLOW, 1.0, c.s.max_mm_per_pass)
        log = c.water(tasks)
        truth = TRUE.copy(wind_x=wind[0], wind_y=wind[1])
        misses = np.array([math.hypot(*(np.array(noisy_landing(truth, p, t, gust)) - np.array(tgt)))
                           for tgt, (p, t) in log.aims])
        print(f"  [{label}] stopped: {log.stopped}, aims {len(log.aims)}, check shots seen {len(log.verified)}")
        results[label] = dict(misses=misses, log=log, cells=cells, deficits=deficits, tasks=tasks,
                              est=(c.model.wind_x, c.model.wind_y), mmodel=mmodel, probes=probes)

    # ---- summary -------------------------------------------------------------
    def row(name, m):
        print(f"  {name:28s} median {np.median(m):.2f} m   90th pct {np.percentile(m, 90):.2f} m   n={len(m)}")
    print("\nAim error, still air (40 random targets):")
    row("nominal model", m_nom)
    row("calibrated model", m_fit)
    print(f"\nAim error, dawn session, crosswind {wind[1]} m/s +/- {gust} gusts:")
    for k, v in results.items():
        row(k, v["misses"])
    r = results["camera wind correction"]
    print(f"  wind estimate at end of session: ({r['est'][0]:.2f}, {r['est'][1]:.2f}) m/s")
    d = r["deficits"]
    tasks = r["tasks"]
    litres = sum(t.mm for t in tasks)            # 1 m^2 cells: mm == litres
    secs = sum(t.seconds for t in tasks)
    print(f"\nPlan: {len(r['cells'])} reachable cells, {int(np.sum(d > 0))} need water, "
          f"{litres:.0f} L total, {secs / 60:.1f} min of spray, {max(t.pass_no for t in tasks) + 1} passes")
    print(f"  moisture fit from probes: VWC = {r['mmodel'].vwc_at_median:.3f} + "
          f"{r['mmodel'].slope:.4f} x excess")
    uniform = len(r["cells"]) * 10.0
    print(f"  vs a uniform 10 mm on every reachable cell: {uniform:.0f} L "
          f"({100 * (1 - litres / uniform):.0f}% saved)")

    plot(results, m_nom, m_fit, fit)


def plot(results, m_nom, m_fit, fit):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.colors import LinearSegmentedColormap

    surface, ink, ink2, grid = "#fcfcfb", "#0b0b0b", "#52514e", "#e4e3df"
    s1, s2, s3 = "#2a78d6", "#eb6834", "#1baf7a"
    plt.rcParams.update({"font.size": 9, "axes.edgecolor": grid, "axes.labelcolor": ink2,
                         "xtick.color": ink2, "ytick.color": ink2, "axes.titlecolor": ink,
                         "axes.titlesize": 10, "axes.titleweight": "bold",
                         "figure.facecolor": surface, "axes.facecolor": surface})
    fig, ax = plt.subplots(1, 3, figsize=(15, 5.4), gridspec_kw={"width_ratios": [1.1, 1.15, 1]})

    # (a) deficit map
    r = results["camera wind correction"]
    blues = LinearSegmentedColormap.from_list("b", ["#f2f6fc", "#a9c8f0", s1, "#123f7a"])
    from matplotlib.patches import Rectangle
    from matplotlib.cm import ScalarMappable
    from matplotlib.colors import Normalize
    vmax = max(5.0, float(np.ceil(np.max(r["deficits"]))))
    norm = Normalize(0, vmax)
    for c, dmm in zip(r["cells"], r["deficits"]):
        ax[0].add_patch(Rectangle((c.y - 0.5, c.x - 0.5), 1, 1, facecolor=blues(norm(dmm)),
                                  edgecolor=surface, lw=1.5))
    sc = ScalarMappable(norm=norm, cmap=blues)
    poly = np.array(LAWN + [LAWN[0]])
    ax[0].plot(poly[:, 1], poly[:, 0], color=ink2, lw=1)
    rmax = fit.reach()[1]
    a = np.linspace(-np.pi / 2, np.pi / 2, 200)
    ax[0].plot(rmax * np.sin(a), rmax * np.cos(a), color=ink2, lw=1, ls="--")
    ax[0].text(0, rmax + 0.3, f"reach {rmax:.1f} m", ha="center", color=ink2, fontsize=8)
    ax[0].plot(0, 0, marker="^", color=ink, ms=9)
    ax[0].text(0.3, -0.5, "turret", color=ink, fontsize=8)
    for i, _ in r["probes"]:
        c = r["cells"][i]
        ax[0].plot(c.y, c.x, marker="o", mfc="none", mec=s2, ms=11, mew=1.5)
    ax[0].plot([], [], marker="o", mfc="none", mec=s2, ls="none", label="soil probe")
    ax[0].legend(loc="lower right", frameon=False, fontsize=8)
    ax[0].set_aspect("equal")
    ax[0].set_xlim(-7.5, 7.5)
    ax[0].set_ylim(-0.8, 10.6)
    ax[0].set_xlabel("y (m)")
    ax[0].set_ylabel("x (m), away from turret")
    ax[0].set_title("Water needed per 1 m cell (mm)", loc="left")
    cb = fig.colorbar(sc, ax=ax[0], shrink=0.7, pad=0.02)
    cb.outline.set_visible(False)

    # (b) trajectories, side view
    for tilt in (-50, -25, -5, 10, fit.reach()[2]):
        p = np.array(fit.trajectory(0, tilt - fit.tilt_trim, wind=False))
        ax[1].plot(p[:, 0], p[:, 2], color=s1, lw=1.6)
        ax[1].text(p[-1, 0], -0.25, f"{tilt:.0f}°", ha="center", va="top", fontsize=8, color=ink2)
    ax[1].axhline(0, color=ink2, lw=1)
    ax[1].set_ylim(-0.9, None)
    ax[1].set_xlabel("distance from turret (m)")
    ax[1].set_ylabel("height (m)")
    ax[1].set_title("Tilt sets range: down for close, up to ~34° for max reach", loc="left")
    ax[1].grid(color=grid, lw=0.6)

    # (c) aim error
    labels = ["Nominal model\n(no calibration)", "Calibrated,\nstill air",
              "Calibrated, 2.5 m/s\ncrosswind, no correction", "Calibrated, crosswind,\ncamera correction"]
    data = [m_nom, m_fit, results["no wind correction"]["misses"], results["camera wind correction"]["misses"]]
    med = [np.median(d) for d in data]
    p90 = [np.percentile(d, 90) for d in data]
    yy = np.arange(len(labels))[::-1]
    ax[2].hlines(yy, 0, p90, color=grid, lw=6)
    ax[2].plot(med, yy, "o", color=s1, ms=8, label="median")
    ax[2].plot(p90, yy, "|", color=ink2, ms=14, mew=2, label="90th percentile")
    for y, m, p in zip(yy, med, p90):
        ax[2].text(p + 0.05, y, f"{m:.2f} / {p:.2f} m", va="center", fontsize=8, color=ink)
    ax[2].set_yticks(yy)
    ax[2].set_yticklabels(labels, fontsize=8)
    ax[2].set_xlim(0, max(p90) * 1.45)
    ax[2].set_xlabel("miss distance (m)")
    ax[2].set_title("Aim error: median / 90th percentile", loc="left")
    ax[2].legend(loc="lower right", frameon=False, fontsize=8)
    ax[2].grid(axis="x", color=grid, lw=0.6)

    for a_ in ax:
        for sp in ("top", "right"):
            a_.spines[sp].set_visible(False)
    fig.tight_layout()
    fig.savefig("sim_results.png", dpi=140)
    print("\nSaved sim_results.png")


if __name__ == "__main__":
    sys.exit(main())
