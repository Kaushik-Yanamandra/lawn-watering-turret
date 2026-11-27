"""Drawing sheets and renders for the printed parts and the head assembly.

    python cad/drawings.py      # writes cad/stl/*.stl and cad/drawings/*.png

Each sheet: isometric render, three orthographic views (front = looking
along +y, top = looking down -z, side = looking along -x) with overall
dimensions, a key-dimension table and a title block. Edges shown are
feature edges and silhouettes, hidden ones included (thin grey).
"""
from __future__ import annotations

import math
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.collections import LineCollection, PolyCollection
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import lwt_parts as P  # noqa: E402

INK, INK2, GRID, FILL, ACC = "#0b0b0b", "#52514e", "#d9d8d4", "#eef2f8", "#2a78d6"
SURF = "#ffffff"
plt.rcParams.update({"font.size": 8, "font.family": "DejaVu Sans"})

# Views: name -> (u axis, v axis, u sign, view direction for silhouettes)
VIEWS = {
    "FRONT (looking +Y)": (0, 2, 1.0, np.array([0, 1, 0])),
    "TOP (looking -Z)": (0, 1, 1.0, np.array([0, 0, -1])),
    "SIDE (looking -X)": (1, 2, 1.0, np.array([-1, 0, 0])),
}

KEY_DIMS = {
    "P1": [("Post bore", f"Ø{P.POST_BORE}"), ("Post axis offset", f"{-P.SLEEVE_X:.0f} from pan axis"),
           ("Platform", f"{P.PLAT_X1 - P.PLAT_X0:.0f} × {2 * P.PLAT_Y:.0f} × {P.PLAT_T:.0f}"),
           ("Servo cut-out", f"{P.SV_L + P.SV_CUT:.1f} × {P.SV_W + P.SV_CUT:.1f}"),
           ("Servo screws", f"4 × M4 insert, {P.SV_HOLE_DX} × {P.SV_HOLE_DY}"),
           ("Bearing screws", f"4 × M4 insert at ±{P.BEAR_BOT_HOLE:.0f}"),
           ("Clamp", "1 × M5 bolt + nut trap")],
    "P2": [("Plate", f"{P.YOKE_PLATE:.0f} sq × {P.YOKE_T:.0f}"), ("Tilt axis", f"{P.AXIS_H:.0f} above plate top"),
           ("Arm gap", f"{P.CRADLE_W + P.GAP_SERVO + P.GAP_BEARING:.0f}"),
           ("608 pocket", f"Ø{P.B608_D} × {P.B608_T}"), ("Horn", f"4 × M3 on PCD {P.HORN_PCD:.0f}, pocket Ø{P.HORN_D + 1:.0f}"),
           ("Bearing bolts", f"4 × M4 at ±{P.BEAR_TOP_HOLE:.0f}")],
    "P3a": [("Body", f"{P.CR_L:.0f} × {P.CRADLE_W / 2:.0f} × {P.CR_H:.0f}"), ("Fitting bore", f"Ø{P.FIT_BORE:.0f}"),
            ("Hex pocket", f"{P.FIT_HEX_AF + 0.4:.1f} A/F × {P.FIT_HEX_L:.0f}"),
            ("Horn", f"4 × M3 insert, PCD {P.HORN_PCD:.0f}"), ("Split bolts", "4 × M3 insert")],
    "P3b": [("Body", f"{P.CR_L:.0f} × {P.CRADLE_W / 2:.0f} × {P.CR_H:.0f}"), ("Axle", f"M8, captive nut {P.M8_NUT_AF} A/F"),
            ("Split bolts", "4 × M3 × 40, counterbored")],
    "P4": [("Bore", f"Ø{P.POST_OD + 0.2:.1f}"), ("Width", f"{P.CLAMP_W:.0f}"), ("Bolts", "2 × M5 across halves"),
           ("Pad", "2 × M4 insert, 40 apart")],
    "P5": [("Plate", "90 × 200 × 5, ribs 6"), ("Clamp holes", "4 × M4, 40 × 160"),
           ("Enclosure pattern", "4 × M4 insert, 72 × 120"), ("Zip slots", "6 × 5 × 12")],
    "P6": [("Post", f"snap Ø{P.POST_OD:.1f}"), ("Tube", f"snap Ø{P.TUBE_OD}"), ("Cable", "8 mm slot")],
    "P7": [("Base", "110 × 40 × 3"), ("Camera Module 3", f"lens Ø{P.CAM3_LENS_D:.0f}, 4 × M2 {P.CAM3_HOLES[0]} × {P.CAM3_HOLES[1]}"),
           ("Servo tower", f"MG90S pocket {P.MG90S_BODY[0]} × {P.MG90S_BODY[1]}, tabs {P.MG90S_TAB_SPACING}"),
           ("Base screws", "2 × M3, 90 apart")],
    "P8": [("Window", f"{P.HOOD_AP_W:.0f} × {P.HOOD_AP_H:.0f}"), ("Visor", "28 overhang, drip lip"),
           ("Film slot", "0.6 wide"), ("Screws", "4 × M3")],
    "P9": [("Foot", "16 × 12 × 2.5, 2 × M2 horn screws 11 apart"), ("Upright", "32 × 30 × 2.5"),
           ("MLX90640", f"can Ø{P.MLX_CAN_D:.0f}, 2 × M2.5 {P.MLX_HOLES}")],
}


def view_edges(mesh, vdir, angle_deg=30.0):
    """Feature edges (dihedral > angle) and silhouette edges for a view direction."""
    adj = mesh.face_adjacency
    ang = mesh.face_adjacency_angles
    n = mesh.face_normals
    d0 = n[adj[:, 0]] @ vdir
    d1 = n[adj[:, 1]] @ vdir
    sharp = ang > math.radians(angle_deg)
    sil = (np.sign(d0) != np.sign(d1)) & (np.abs(d0 - d1) > 1e-6)
    e = mesh.face_adjacency_edges
    return mesh.vertices[e[sharp | sil]]


def dim_line(ax, p0, p1, off, text, horizontal=True):
    """Simple dimension: extension lines, arrowed dimension line, text."""
    (x0, y0), (x1, y1) = p0, p1
    kw = dict(color=ACC, lw=0.7)
    if horizontal:
        y = off
        ax.plot([x0, x0], [y0, y], **kw)
        ax.plot([x1, x1], [y1, y], **kw)
        ax.annotate("", (x0, y), (x1, y), arrowprops=dict(arrowstyle="<->", color=ACC, lw=0.7))
        ax.text((x0 + x1) / 2, y, text, ha="center", va="bottom", color=ACC, fontsize=7.5,
                bbox=dict(fc=SURF, ec="none", pad=0.6))
    else:
        x = off
        ax.plot([x0, x], [y0, y0], **kw)
        ax.plot([x1, x], [y1, y1], **kw)
        ax.annotate("", (x, y0), (x, y1), arrowprops=dict(arrowstyle="<->", color=ACC, lw=0.7))
        ax.text(x, (y0 + y1) / 2, text, ha="center", va="center", rotation=90, color=ACC, fontsize=7.5,
                bbox=dict(fc=SURF, ec="none", pad=0.6))


def draw_view(ax, mesh, name):
    iu, iv, su, vdir = VIEWS[name]
    if name.startswith("SIDE"):
        su = -1.0
    seg = view_edges(mesh, vdir)
    uv = np.stack([seg[:, :, iu] * su, seg[:, :, iv]], axis=-1)
    tri = mesh.vertices[mesh.faces]
    tri2 = np.stack([tri[:, :, iu] * su, tri[:, :, iv]], axis=-1)
    ax.add_collection(PolyCollection(tri2, facecolor=FILL, edgecolor="none", zorder=0))
    # depth-based weight: edges nearer the viewer darker
    depth_axis = [a for a in (0, 1, 2) if a not in (iu, iv)][0]
    dsign = -np.sign(vdir[depth_axis])
    dep = seg[:, :, depth_axis].mean(axis=1) * dsign
    near = dep > (dep.min() + 0.55 * (dep.max() - dep.min()))
    ax.add_collection(LineCollection(uv[~near], colors=GRID, linewidths=0.5, zorder=1))
    ax.add_collection(LineCollection(uv[near], colors=INK, linewidths=0.8, zorder=2))
    lo = np.array([mesh.bounds[0][iu], mesh.bounds[0][iv]])
    hi = np.array([mesh.bounds[1][iu], mesh.bounds[1][iv]])
    if su < 0:
        lo[0], hi[0] = -hi[0], -lo[0]
    w, h = hi - lo
    pad = 0.28 * max(w, h) + 8
    ax.set_xlim(lo[0] - pad * 0.6, hi[0] + pad * 0.6)
    ax.set_ylim(lo[1] - pad * 0.7, hi[1] + pad * 0.6)
    ax.set_aspect("equal")
    ax.axis("off")
    dim_line(ax, (lo[0], lo[1]), (hi[0], lo[1]), lo[1] - pad * 0.38, f"{w:.1f}")
    dim_line(ax, (hi[0], lo[1]), (hi[0], hi[1]), hi[0] + pad * 0.35, f"{h:.1f}", horizontal=False)
    ax.set_title(name, fontsize=8, color=INK2, loc="left")
    names = "XYZ"
    ax.text(lo[0] - pad * 0.55, lo[1] - pad * 0.65,
            f"→ {'-' if su < 0 else '+'}{names[iu]}   ↑ +{names[iv]}", fontsize=6.5, color=INK2)


def cam_dir(elev, azim):
    e, a = math.radians(elev), math.radians(azim)
    return np.array([math.cos(e) * math.cos(a), math.cos(e) * math.sin(a), math.sin(e)])


def visible_sorted(mesh, elev, azim):
    """Back-face cull and depth-sort triangles (painter's algorithm)."""
    v = cam_dir(elev, azim)
    keep = mesh.face_normals @ v > 1e-6
    tri = mesh.vertices[mesh.faces][keep]
    n = mesh.face_normals[keep]
    order = np.argsort(tri.mean(axis=1) @ v)
    return tri[order], n[order]


def render_iso(ax, mesh, color="#8fb4e6", elev=24, azim=-58):
    tri, n = visible_sorted(mesh, elev, azim)
    light = np.array([0.4, -0.6, 0.75]); light /= np.linalg.norm(light)
    shade = 0.45 + 0.55 * np.clip(n @ light, 0, 1)
    base = np.array(matplotlib.colors.to_rgb(color))
    cols = np.clip(base[None, :] * shade[:, None] + 0.08, 0, 1)
    pc = Poly3DCollection(tri, facecolors=cols, edgecolors=cols, linewidths=0.2)
    pc.set_sort_zpos(0)
    ax.add_collection3d(pc)
    ax.computed_zorder = False
    lo, hi = mesh.bounds
    c = (lo + hi) / 2
    r = (hi - lo).max() / 2
    ax.set_xlim(c[0] - r, c[0] + r)
    ax.set_ylim(c[1] - r, c[1] + r)
    ax.set_zlim(c[2] - r, c[2] + r)
    ax.set_box_aspect((1, 1, 1))
    ax.view_init(elev=elev, azim=azim)
    ax.set_axis_off()


def sheet(part, mesh, out):
    fig = plt.figure(figsize=(11.69, 8.27), facecolor=SURF)       # A4 landscape
    gs = fig.add_gridspec(2, 3, left=0.03, right=0.97, top=0.885, bottom=0.17, wspace=0.05, hspace=0.12)
    draw_view(fig.add_subplot(gs[0, 0]), mesh, "FRONT (looking +Y)")
    draw_view(fig.add_subplot(gs[0, 1]), mesh, "SIDE (looking -X)")
    draw_view(fig.add_subplot(gs[1, 0]), mesh, "TOP (looking -Z)")
    ax3 = fig.add_subplot(gs[:, 2], projection="3d")
    render_iso(ax3, mesh)
    ax3.set_title("ISOMETRIC", fontsize=8, color=INK2, loc="left")
    axk = fig.add_subplot(gs[1, 1])
    axk.axis("off")
    rows = KEY_DIMS.get(part.pid, [])
    axk.text(0, 1.0, "KEY DIMENSIONS (mm)", fontsize=8, fontweight="bold", color=INK, va="top")
    for i, (k, v) in enumerate(rows):
        axk.text(0, 0.88 - i * 0.095, k, fontsize=8, color=INK2, va="top")
        axk.text(0.42, 0.88 - i * 0.095, v, fontsize=8, color=INK, va="top")
    ext = mesh.bounds[1] - mesh.bounds[0]
    fig.text(0.03, 0.965, f"{part.pid}  {part.name.upper()}", fontsize=13, fontweight="bold", color=INK)
    fig.text(0.03, 0.942, part.note, fontsize=8.5, color=INK2)
    tb = [("DOC", "EDD-LWT-001"), ("PART", part.pid), ("QTY", str(part.qty)), ("MATERIAL", part.material),
          ("ENVELOPE", f"{ext[0]:.0f} × {ext[1]:.0f} × {ext[2]:.0f}"), ("VOLUME", f"{mesh.volume / 1000:.0f} cm³ solid"),
          ("UNITS", "mm"), ("REV", "A"), ("SOURCE", "cad/lwt_parts.py")]
    x = 0.03
    for k, v in tb:
        fig.text(x, 0.085, k, fontsize=6.5, color=INK2)
        fig.text(x, 0.06, v, fontsize=8.5, color=INK)
        x += 0.108
    fig.add_artist(plt.Line2D([0.03, 0.97], [0.12, 0.12], color=GRID, lw=0.8))
    fig.text(0.03, 0.025, "General tolerance ±0.3 mm on printed features. Holes marked insert are pilot holes for brass "
             "heat-set inserts. Edges include hidden features (light grey).", fontsize=7, color=INK2)
    fig.savefig(out, dpi=150, facecolor=SURF)
    plt.close(fig)


# --------------------------------------------------------------------------- assembly
def head_assembly(pan_deg=20.0, tilt_deg=15.0):
    """Turret head assembly for a render. Returns list of (mesh, colour, label)."""
    out = []
    p1 = P.p1_pan_base()
    out.append((p1, "#8fb4e6", "P1"))
    post = P.cyl_z(P.POST_OD / 2, -120, -0.5, x=P.SLEEVE_X)
    out.append((post, "#b9b8b3", "post"))
    sv = P.box(-P.SV_SHAFT_FROM_END, P.SV_L - P.SV_SHAFT_FROM_END, -P.SV_W / 2, P.SV_W / 2,
               -(P.SV_FLANGE_FROM_BOTTOM + P.SV_FLANGE_T), P.SV_H - P.SV_FLANGE_FROM_BOTTOM - P.SV_FLANGE_T)
    out.append((sv, "#3b3b3b", "pan servo"))
    z_b = P.PLAT_T
    bearing = P.box(-P.BEAR_SQ / 2, P.BEAR_SQ / 2, -P.BEAR_SQ / 2, P.BEAR_SQ / 2, z_b, z_b + P.BEAR_T) - \
        P.box(-30, 30, -30, 30, z_b - 1, z_b + P.BEAR_T + 1)
    out.append((bearing, "#c9c9c9", "bearing"))
    z_y = z_b + P.BEAR_T
    rot = lambda m: m.rotate((0, 0, pan_deg))
    yoke = P.p2_yoke().translate((0, 0, z_y))
    out.append((rot(yoke), "#8fb4e6", "P2"))
    za = z_y + P.YOKE_T + P.AXIS_H
    yl_in, yl_out, yr_in, yr_out = P.yoke_geometry()
    tsv = P.box(-P.SV_W / 2, P.SV_W / 2, yl_out - 10, yl_out + 30.5,
                za - (P.SV_L - P.SV_SHAFT_FROM_END), za + P.SV_SHAFT_FROM_END)
    out.append((rot(tsv), "#3b3b3b", "tilt servo"))
    cr = (P.p3a_cradle_horn_half() + P.p3b_cradle_axle_half())
    noz = P.cyl_x(6.0, P.CR_L / 2 - 2, P.CR_L / 2 + 22) + P.cyl_x(3.5, P.CR_L / 2 + 22, P.CR_L / 2 + 30)
    barb = P.cyl_x(5.0, -P.CR_L / 2 - 22, -P.CR_L / 2 + 2)
    tilt = lambda m: m.rotate((0, -tilt_deg, 0)).translate((0, 0, za))
    out.append((rot(tilt(cr)), "#f0a37c", "P3"))
    out.append((rot(tilt(noz)), "#c8a14a", "nozzle"))
    out.append((rot(tilt(barb)), "#c8a14a", "barb"))
    return [(P.to_trimesh(m), c, l) for m, c, l in out]


def render_assembly(out):
    parts = head_assembly()
    fig = plt.figure(figsize=(8, 8), facecolor=SURF)
    ax = fig.add_subplot(111, projection="3d")
    allv = []
    light = np.array([0.4, -0.6, 0.75]); light /= np.linalg.norm(light)
    elev, azim = 18, -62
    vdir = cam_dir(elev, azim)
    tris, cols = [], []
    for m, c, _ in parts:
        keep = m.face_normals @ vdir > 1e-6
        tri = m.vertices[m.faces][keep]
        shade = 0.45 + 0.55 * np.clip(m.face_normals[keep] @ light, 0, 1)
        base = np.array(matplotlib.colors.to_rgb(c))
        tris.append(tri); cols.append(np.clip(base * shade[:, None] + 0.06, 0, 1))
        allv.append(m.vertices)
    tri = np.vstack(tris); col = np.vstack(cols)
    order = np.argsort(tri.mean(axis=1) @ vdir)
    ax.add_collection3d(Poly3DCollection(tri[order], facecolors=col[order], edgecolors=col[order], linewidths=0.2))
    ax.computed_zorder = False
    v = np.vstack(allv)
    lo, hi = v.min(0), v.max(0)
    cc, r = (lo + hi) / 2, (hi - lo).max() / 2 * 0.78
    ax.set_xlim(cc[0] - r, cc[0] + r); ax.set_ylim(cc[1] - r, cc[1] + r); ax.set_zlim(cc[2] - r, cc[2] + r)
    ax.set_box_aspect((1, 1, 1)); ax.view_init(elev=elev, azim=azim); ax.set_axis_off()
    fig.text(0.05, 0.95, "Turret head assembly (pan 20°, tilt 15°)", fontsize=12, fontweight="bold", color=INK)
    fig.text(0.05, 0.925, "Blue: printed ASA (P1 pan base, P2 yoke) · Orange: P3 cradle · Grey: bought parts",
             fontsize=8.5, color=INK2)
    fig.savefig(out, dpi=150, facecolor=SURF, bbox_inches="tight")
    plt.close(fig)


if __name__ == "__main__":
    here = os.path.dirname(os.path.abspath(__file__))
    od = os.path.join(here, "drawings")
    os.makedirs(od, exist_ok=True)
    meshes = P.build_all(os.path.join(here, "stl"))
    for part in P.PARTS:
        sheet(part, meshes[part.pid], os.path.join(od, f"{part.pid}_sheet.png"))
        print("sheet", part.pid)
    render_assembly(os.path.join(od, "head_assembly.png"))
    print("assembly render done")
