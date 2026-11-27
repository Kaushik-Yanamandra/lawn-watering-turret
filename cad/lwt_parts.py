"""Parametric CAD for the Lawn Watering Turret 3D-printed parts.

    python cad/lwt_parts.py            # writes cad/stl/*.stl

Every dimension is a named parameter in mm at the top of this file. Measure
your actual servo, bearing and brass fittings and change the numbers here
before printing; the parts rebuild from them.

Built with manifold3d (pip install manifold3d trimesh). Each part is
modelled in its own local frame, described in its docstring.
"""
from __future__ import annotations

import math
import os
from dataclasses import dataclass

import numpy as np
from manifold3d import CrossSection, Manifold

SEG = 72                      # facets on a full circle

# --------------------------------------------------------------------------- parameters
# Post: 3/4 in galvanised steel pipe
POST_OD = 26.7
POST_BORE = 27.2              # sleeve bore: 0.5 mm clearance

# Servo, DS3225-class standard size (measure yours)
SV_L, SV_W, SV_H = 40.0, 20.0, 40.5     # body
SV_FLANGE_FROM_BOTTOM = 28.0            # bottom of body to underside of flange
SV_FLANGE_T = 2.5
SV_FLANGE_L = 54.5
SV_HOLE_DX, SV_HOLE_DY, SV_HOLE_D = 49.5, 10.0, 4.4
SV_SHAFT_FROM_END = 10.0                # spline centre to the near end of the body
SV_CUT = 0.6                            # clearance added to body cut-outs
HORN_D = 25.0                           # 25T aluminium disc horn
HORN_T = 3.0
HORN_PCD = 18.0                         # bolt circle of the horn's 4 x M3 holes
M3_INSERT_D, M3_INSERT_L = 4.2, 6.0     # heat-set insert pilot
M4_INSERT_D, M4_INSERT_L = 5.6, 7.0
M3_CLR, M4_CLR, M5_CLR, M8_CLR = 3.4, 4.5, 5.5, 8.6

# Turntable ("lazy susan") bearing, 4 in square, 8 mm high (measure yours)
BEAR_SQ = 102.0
BEAR_T = 8.0
BEAR_BOT_HOLE = 42.0          # bottom-plate holes at (+/-, +/-)
BEAR_TOP_HOLE = 36.0          # top-plate holes at (+/-, +/-)

# Pan base
PLAT_T = 8.0
PLAT_X0, PLAT_X1, PLAT_Y = -74.0, 56.0, 56.0
SLEEVE_X = -50.0              # post axis offset from the pan axis
SLEEVE_OD, SLEEVE_L = 42.0, 55.0

# Yoke and tilt
YOKE_PLATE = 104.0
YOKE_T = 6.0
AXIS_H = 75.0                 # tilt axis above the yoke plate top
ARM_T = 8.0
CRADLE_W = 48.0               # cradle width along the tilt axis
GAP_SERVO = 7.0               # cradle face to servo-side arm (spline + horn)
GAP_BEARING = 2.0             # cradle face to bearing-side arm (washer)
B608_D, B608_T = 22.2, 7.0

# Cradle and brass fittings
CR_L, CR_H = 76.0, 30.0
FIT_BORE = 16.0               # through-bore for the brass fitting stack
FIT_HEX_AF, FIT_HEX_L = 19.0, 14.0
FIT_HEX_X = -10.0             # hex pocket centre along the nozzle axis
M8_NUT_AF, M8_NUT_T = 13.2, 6.8

# Pipe clamp, manifold plate, clip
CLAMP_W = 25.0
CLAMP_OD = 42.0
TUBE_OD = 14.5                # 3/8 in ID reinforced PVC tube (measure)

# Sensor unit
CAM3_HOLES = (21.0, 12.5)     # Camera Module 3 M2 hole pattern
CAM3_LENS_D = 12.0
MLX_HOLES = 20.3              # Adafruit MLX90640 breakout mounting-hole spacing (verify)
MLX_CAN_D = 11.0
MG90S_BODY = (23.2, 12.6)     # micro servo body (with clearance)
MG90S_TAB_SPACING = 27.8      # tab screw spacing
HOOD_AP_W, HOOD_AP_H = 80.0, 36.0   # thermal window: 55 deg sensor panned +/-35 deg


# --------------------------------------------------------------------------- helpers
def box(x0, x1, y0, y1, z0, z1):
    return Manifold.cube((x1 - x0, y1 - y0, z1 - z0)).translate((x0, y0, z0))


def cyl_z(r, z0, z1, x=0.0, y=0.0, seg=SEG):
    return Manifold.cylinder(z1 - z0, r, r, seg).translate((x, y, z0))


def cyl_y(r, y0, y1, x=0.0, z=0.0, seg=SEG):
    # cylinder along +y from y0 to y1
    return Manifold.cylinder(y1 - y0, r, r, seg).rotate((-90, 0, 0)).translate((x, y0, z))


def cyl_x(r, x0, x1, y=0.0, z=0.0, seg=SEG):
    return Manifold.cylinder(x1 - x0, r, r, seg).rotate((0, 90, 0)).translate((x0, y, z))


def hex_y(af, y0, y1, x=0.0, z=0.0):
    r = af / math.sqrt(3)
    return Manifold.cylinder(y1 - y0, r, r, 6).rotate((-90, 0, 0)).translate((x, y0, z))


def hex_x(af, x0, x1, y=0.0, z=0.0):
    r = af / math.sqrt(3)
    return Manifold.cylinder(x1 - x0, r, r, 6).rotate((0, 0, 30)).rotate((0, 90, 0)).translate((x0, y, z))


def prism_xz(points, y0, y1):
    """Extrude a polygon given in (x, z) along +y from y0 to y1."""
    cs = CrossSection([points])
    return Manifold.extrude(cs, y1 - y0).rotate((90, 0, 0)).translate((0, y1, 0))


def prism_yz(points, x0, x1):
    """Extrude a polygon given in (y, z) along +x from x0 to x1."""
    cs = CrossSection([points])
    return Manifold.extrude(cs, x1 - x0).rotate((90, 0, 90)).translate((x0, 0, 0))


def union(*parts):
    out = parts[0]
    for p in parts[1:]:
        out = out + p
    return out


# --------------------------------------------------------------------------- P1
def p1_pan_base():
    """P1 Pan base. Frame: pan axis = z axis, z = 0 at the platform underside.
    The post sleeve hangs below, offset to x = SLEEVE_X. The pan servo hangs
    under the platform, spline up on the pan axis, flange screwed to the
    platform underside. The turntable bearing's bottom plate sits on top."""
    plat = box(PLAT_X0, PLAT_X1, -PLAT_Y, PLAT_Y, 0, PLAT_T)
    sleeve = cyl_z(SLEEVE_OD / 2, -SLEEVE_L, 0, x=SLEEVE_X)
    web = box(SLEEVE_X + 10, -24, -6, 6, -32, 0)                     # sleeve-to-shroud web
    # clamp ears either side of a slit on the outboard (-x) side
    ex0, ex1 = SLEEVE_X - SLEEVE_OD / 2 - 13, SLEEVE_X - SLEEVE_OD / 2 + 4
    ears = box(ex0, ex1, -11, 11, -SLEEVE_L + 5, -8)
    # servo shroud: protects the hanging servo, open at the bottom to drain
    sx0, sx1 = -SV_SHAFT_FROM_END, SV_L - SV_SHAFT_FROM_END
    shroud = box(sx0 - 12, sx1 + 12, -SV_W / 2 - 5, SV_W / 2 + 5, -34, 0)
    body = union(plat, sleeve, web, ears, shroud)

    cut = union(
        cyl_z(POST_BORE / 2, -SLEEVE_L - 1, -0.0, x=SLEEVE_X),           # post bore, stops at platform
        box(ex0 - 1, SLEEVE_X - 8, -1.5, 1.5, -SLEEVE_L - 1, -6),         # clamp slit
        cyl_y(M5_CLR / 2, -12, 12, x=ex0 + 7, z=-SLEEVE_L / 2 - 2),       # clamp bolt
        hex_y(8.4, -12, -7.5, x=ex0 + 7, z=-SLEEVE_L / 2 - 2),            # M5 nut trap
        box(sx0 - 9, sx1 + 9, -SV_W / 2 - 2, SV_W / 2 + 2, -35, -0.01),   # shroud cavity
        box(sx0 - SV_CUT / 2, sx1 + SV_CUT / 2, -SV_W / 2 - SV_CUT / 2, SV_W / 2 + SV_CUT / 2, -1, PLAT_T + 1),
        box(sx0 + 5, sx0 + 15, SV_W / 2 + 1, SV_W / 2 + 6, -35, -26),      # cable exit
    )
    # servo flange screws (M4 inserts from below) and bearing bottom-plate screws (M4 inserts from above)
    hx0 = (sx0 + sx1) / 2 - SV_HOLE_DX / 2
    for hx in (hx0, hx0 + SV_HOLE_DX):
        for hy in (-SV_HOLE_DY / 2, SV_HOLE_DY / 2):
            cut = cut + cyl_z(M4_INSERT_D / 2, -1, M4_INSERT_L, x=hx, y=hy)
    for bx in (-BEAR_BOT_HOLE, BEAR_BOT_HOLE):
        for by in (-BEAR_BOT_HOLE, BEAR_BOT_HOLE):
            cut = cut + cyl_z(M4_INSERT_D / 2, PLAT_T - M4_INSERT_L, PLAT_T + 1, x=bx, y=by)
    # drain holes in the platform outside the bearing footprint
    for dy in (-48, 48):
        cut = cut + cyl_z(3, -1, PLAT_T + 1, x=-62, y=dy)
    return body - cut


# --------------------------------------------------------------------------- P2
def yoke_geometry():
    yl_in = CRADLE_W / 2 + GAP_SERVO            # servo-side arm inner face (+y)
    yr_in = -(CRADLE_W / 2 + GAP_BEARING)       # bearing-side arm inner face (-y)
    return yl_in, yl_in + ARM_T, yr_in, yr_in - ARM_T


def p2_yoke():
    """P2 Yoke. Frame: pan axis = z axis, z = 0 at the plate underside (sits
    on the bearing top plate). Tilt axis along y at z = YOKE_T + AXIS_H.
    Tilt servo goes through the +y arm, spline pointing inward; a 608 bearing
    in the -y arm carries the other trunnion."""
    yl_in, yl_out, yr_in, yr_out = yoke_geometry()
    za = YOKE_T + AXIS_H
    h = YOKE_PLATE / 2
    plate = box(-h, h, -h, h, 0, YOKE_T)
    arm_l = box(-24, 24, yl_in, yl_out, YOKE_T - 0.01, za + 22)
    arm_r = box(-20, 20, yr_out, yr_in, YOKE_T - 0.01, za + 20)
    # gussets: triangular ribs at right angles to each arm (two per arm)
    gussets = []
    for x in (-21.0, 15.0):
        gussets.append(prism_yz([(yl_out - 0.5, YOKE_T - 0.01), (yl_out + 12, YOKE_T - 0.01),
                                 (yl_out - 0.5, YOKE_T + 36)], x, x + 6))
        gussets.append(prism_yz([(yr_out + 0.5, YOKE_T - 0.01), (yr_out + 0.5, YOKE_T + 36),
                                 (yr_out - 12, YOKE_T - 0.01)], x, x + 6))
        gussets.append(prism_yz([(yl_in + 0.5, YOKE_T - 0.01), (yl_in + 0.5, YOKE_T + 20),
                                 (yl_in - 8, YOKE_T - 0.01)], x, x + 6))
        gussets.append(prism_yz([(yr_in - 0.5, YOKE_T - 0.01), (yr_in + 8, YOKE_T - 0.01),
                                 (yr_in - 0.5, YOKE_T + 20)], x, x + 6))
    body = union(plate, arm_l, arm_r, *gussets)

    # tilt servo: body vertical in the +y arm, spline at the tilt axis
    c = SV_CUT / 2
    zs_top = za + SV_SHAFT_FROM_END
    zs_bot = za - (SV_L - SV_SHAFT_FROM_END)
    cut = box(-SV_W / 2 - c, SV_W / 2 + c, yl_in - 1, yl_out + 15, zs_bot - c, zs_top + c)
    zc = (zs_top + zs_bot) / 2
    for hz in (zc - SV_HOLE_DX / 2, zc + SV_HOLE_DX / 2):
        for hx in (-SV_HOLE_DY / 2, SV_HOLE_DY / 2):
            if YOKE_T + 3 < hz < za + 20:
                cut = cut + cyl_y(M4_INSERT_D / 2, yl_out - M4_INSERT_L, yl_out + 1, x=hx, z=hz)
    # 608 bearing pocket from the outer face, shaft clearance through
    cut = cut + cyl_y(B608_D / 2, yr_out - 1, yr_out + B608_T, z=za)
    cut = cut + cyl_y(M8_CLR / 2 + 1.5, yr_out - 1, yr_in + 1, z=za)
    # horn pocket underneath, horn screws, access hole
    cut = cut + cyl_z(HORN_D / 2 + 0.5, -1, HORN_T)
    cut = cut + cyl_z(4.5, -1, YOKE_T + 1)
    for k in range(4):
        a = math.radians(45 + 90 * k)
        x, y = HORN_PCD / 2 * math.cos(a), HORN_PCD / 2 * math.sin(a)
        cut = cut + cyl_z(M3_CLR / 2, -1, YOKE_T + 1, x=x, y=y)
        cut = cut + cyl_z(3.2, YOKE_T - 2.5, YOKE_T + 1, x=x, y=y)     # counterbore
    # bearing top-plate bolts with counterbores
    for bx in (-BEAR_TOP_HOLE, BEAR_TOP_HOLE):
        for by in (-BEAR_TOP_HOLE, BEAR_TOP_HOLE):
            cut = cut + cyl_z(M4_CLR / 2, -1, YOKE_T + 1, x=bx, y=by)
    # rev B: the old hose pass-through (x 28..46) sat over the bearing's ball
    # race and led nowhere; it is removed. The tube and the tilt-servo lead
    # run outside in a service loop, tied to the gusset holes below.
    for x in (-18.0, 18.0):
        cut = cut + cyl_x(2.2, x - 4, x + 10, y=yr_out - 5, z=YOKE_T + 12)      # zip-tie holes, bearing-side gussets
    return body - cut


# --------------------------------------------------------------------------- P3
def cradle_full():
    """Cradle as one body before splitting. Frame: origin on the tilt axis,
    nozzle pointing +x, tilt axis along y. Servo horn on the +y face."""
    hy = CRADLE_W / 2
    body = box(-CR_L / 2, CR_L / 2, -hy, hy, -CR_H / 2, CR_H / 2)
    cut = cyl_x(FIT_BORE / 2, -CR_L / 2 - 1, CR_L / 2 + 1)
    cut = cut + hex_x(FIT_HEX_AF + 0.4, FIT_HEX_X - FIT_HEX_L / 2, FIT_HEX_X + FIT_HEX_L / 2)
    # horn face: recess, centre relief, 4 x M3 inserts
    cut = cut + cyl_y(HORN_D / 2 + 0.5, hy - 1.0, hy + 1)
    cut = cut + cyl_y(5.0, hy - 9, hy + 1)
    for k in range(4):
        a = math.radians(45 + 90 * k)
        x, z = HORN_PCD / 2 * math.cos(a), HORN_PCD / 2 * math.sin(a)
        cut = cut + cyl_y(M3_INSERT_D / 2, hy - 1 - M3_INSERT_L, hy, x=x, z=z)
    # axle side: M8 bolt hole and captive nut, nut drops in through a slot from below
    cut = cut + cyl_y(M8_CLR / 2, -hy - 1, -hy + 9)
    nut_y0 = -hy + 3
    cut = cut + hex_y(M8_NUT_AF, nut_y0, nut_y0 + M8_NUT_T)
    cut = cut + box(-M8_NUT_AF / 2, M8_NUT_AF / 2, nut_y0, nut_y0 + M8_NUT_T, -CR_H / 2 - 1, 0)
    # four M3 clamp bolts across the split (head on -y face, insert in +y half)
    for x in (-28, 28):
        for z in (-9, 9):
            cut = cut + cyl_y(M3_CLR / 2, -hy - 1, 2, x=x, z=z)
            cut = cut + cyl_y(3.2, -hy - 1, -hy + 3, x=x, z=z)
            cut = cut + cyl_y(M3_INSERT_D / 2, 0, 2 + M3_INSERT_L, x=x, z=z)
    return body - cut


def p3a_cradle_horn_half():
    """P3a Cradle, servo-horn half (+y). Print split face down."""
    return cradle_full() ^ box(-60, 60, 0, 40, -30, 30)


def p3b_cradle_axle_half():
    """P3b Cradle, axle half (-y). Print split face down."""
    return cradle_full() ^ box(-60, 60, -40, 0, -30, 30)


# --------------------------------------------------------------------------- P4
def p4_pipe_clamp_half():
    """P4 Pipe clamp half (print 2 per clamp). Frame: post axis = z, the half
    covers y >= 0. Two M5 bolts join the halves; the flat pad on top carries
    two M4 inserts for P5 plates or an enclosure."""
    r_in, r_out = POST_OD / 2 + 0.1, CLAMP_OD / 2
    ring = cyl_z(r_out, 0, CLAMP_W) ^ box(-30, 30, 0, 30, -1, CLAMP_W + 1)
    ears = box(-r_out - 12, r_out + 12, 0, 7, 0, CLAMP_W)
    pad = box(-27, 27, r_out - 4, r_out + 4, 0, CLAMP_W)
    body = union(ring, ears, pad)
    cut = cyl_z(r_in, -1, CLAMP_W + 1)
    for x in (-(r_out + 6), r_out + 6):
        cut = cut + cyl_y(M5_CLR / 2, -1, 8, x=x, z=CLAMP_W / 2)
    for x in (-20, 20):
        cut = cut + cyl_y(M4_INSERT_D / 2, r_out + 4 - M4_INSERT_L, r_out + 5, x=x, z=CLAMP_W / 2)
    return body - cut


# --------------------------------------------------------------------------- P5
def p5_mount_plate():
    """P5 Mounting plate (print 2): bolts to two P4 clamps 160 mm apart and
    carries the valve train (with zip ties) or the Turret Node enclosure
    (4 x M4 inserts on a 120 x 80 pattern). Frame: plate in the xz plane,
    thickness along y, z up the post."""
    W, H, T = 90.0, 200.0, 5.0
    body = box(-W / 2, W / 2, 0, T, 0, H)
    # stiffening ribs on the back
    body = body + box(-W / 2, -W / 2 + 4, -6, 0.01, 0, H) + box(W / 2 - 4, W / 2, -6, 0.01, 0, H)
    # rev B: bosses behind the enclosure inserts, so the 7 mm inserts sit in
    # blind holes instead of poking 2 mm out of the back of a 5 mm plate
    for z in (40.0, 160.0):
        for x in (-36.0, 36.0):
            body = body + cyl_y(5.5, -5.5, 0.01, x=x, z=z)   # 3 mm floor under the pilot
    cut = None
    for z in (20.0, 180.0):
        for x in (-20.0, 20.0):
            c = cyl_y(M4_CLR / 2, -7, T + 1, x=x, z=z) + cyl_y(4.3, T - 2.5, T + 1, x=x, z=z)
            cut = c if cut is None else cut + c
    for z in (40.0, 160.0):                                   # enclosure pattern 120 x 72, blind pilots
        for x in (-36.0, 36.0):
            cut = cut + cyl_y(M4_INSERT_D / 2, T - M4_INSERT_L - 0.5, T + 1, x=x, z=z)
    for z in (60.0, 100.0, 140.0):                            # zip-tie slots
        for x in (-22.0, 22.0):
            cut = cut + box(x - 2.5, x + 2.5, -7, T + 1, z - 6, z + 6)
    return body - cut


# --------------------------------------------------------------------------- P6
def p6_post_clip():
    """P6 Snap clip: grips the post and holds the supply tube and one cable.
    Frame: post axis = z through the origin; tube axis parallel at +y."""
    t = 3.0
    H = 12.0
    r_post = POST_OD / 2
    post_ring = cyl_z(r_post + t, 0, H) - cyl_z(r_post - 0.2, -1, H + 1)
    post_ring = post_ring - box(-r_post * 0.78, r_post * 0.78, -r_post - t - 1, -r_post * 0.45, -1, H + 1)
    yt = r_post + t + TUBE_OD / 2 + 0.5
    tube_ring = cyl_z(TUBE_OD / 2 + t, 0, H, y=yt) - cyl_z(TUBE_OD / 2, -1, H + 1, y=yt)
    tube_ring = tube_ring - box(-TUBE_OD * 0.38, TUBE_OD * 0.38, yt + TUBE_OD * 0.3, yt + 20, -1, H + 1)
    bridge = box(-6, 6, r_post, yt - TUBE_OD / 2 + 0.5, 0, H)
    cable = box(r_post + 2, r_post + 14, -4, 4, 0, H) - box(r_post + 4, r_post + 12, -2.5, 6, -1, H + 1)
    return union(post_ring, tube_ring, bridge, cable)


# --------------------------------------------------------------------------- P7
def p7_sensor_bracket():
    """P7 Sensor Unit bracket: base plate screwed to the enclosure floor.
    Left: upright carrying Camera Module 3 (lens through the plate, looking
    along -y). Right: a tower holding an MG90S micro servo, spline up, that
    pans the thermal camera (on P9) between three stops.
    Frame: base on z = 0, cameras look along -y."""
    T = 3.0
    base = box(-55, 55, 0, 40, 0, T)
    upright = box(-55, -5, 0, T, 0, 50)
    rib = box(-55, -52, 0, 30, 0, 20) + box(-8, -5, 0, 30, 0, 20)
    tx, ty = 26.0, 16.0                                     # servo tower centre
    bw, bd = MG90S_BODY
    tower = box(tx - 18, tx + 18, ty - 10, ty + 10, 0, 22)
    body = union(base, upright, rib, tower)
    cx, cz = -30.0, 32.0
    cut = cyl_y(CAM3_LENS_D / 2, -1, T + 1, x=cx, z=cz)
    for dx in (-CAM3_HOLES[0] / 2, CAM3_HOLES[0] / 2):
        for dz in (-2.5, -2.5 + CAM3_HOLES[1]):
            cut = cut + cyl_y(1.1, -1, T + 1, x=cx + dx, z=cz + dz)
    cut = cut + box(tx - bw / 2, tx + bw / 2, ty - bd / 2, ty + bd / 2, T, 23)          # servo pocket
    cut = cut + box(tx - 4, tx + 4, ty + bd / 2 - 1, ty + 11, T, 23)                    # lead exit
    for dx in (-MG90S_TAB_SPACING / 2, MG90S_TAB_SPACING / 2):
        cut = cut + cyl_z(0.9, 12, 23, x=tx + dx, y=ty)                                  # M2 self-tap
    for x in (-45.0, 45.0):
        cut = cut + cyl_z(M3_CLR / 2, -1, T + 1, x=x, y=32.0)
    return body - cut


# --------------------------------------------------------------------------- P8
def p8_thermal_hood():
    """P8 Thermal window visor: screws over the open thermal window in the
    Sensor Unit front wall. The window is wide because the sensor pans
    +/-35 deg; the visor keeps rain off and the lip sheds drips. An optional
    0.05-0.1 mm HDPE film slides into the slot (glass and acrylic block
    thermal IR). Frame: window axis along -y (outward), z up."""
    W, H, t = HOOD_AP_W, HOOD_AP_H, 2.5
    flange = box(-W / 2 - 10, W / 2 + 10, 0, 3, -H / 2 - 10, H / 2 + 10)
    visor = box(-W / 2 - 10, W / 2 + 10, -28, 0.01, H / 2 + 2, H / 2 + 2 + t)
    lip = box(-W / 2 - 10, W / 2 + 10, -28, -25, H / 2 - 4, H / 2 + 2.01)
    cheeks = box(-W / 2 - 10, -W / 2 - 7, -14, 0.01, -H / 2, H / 2 + 2.01) + \
        box(W / 2 + 7, W / 2 + 10, -14, 0.01, -H / 2, H / 2 + 2.01)
    body = union(flange, visor, lip, cheeks)
    cut = box(-W / 2, W / 2, -1, 4, -H / 2, H / 2)
    cut = cut + box(-W / 2 - 1.5, W / 2 + 1.5, 1.2, 1.8, -H / 2 - 1.5, H / 2 + 12)      # film slot
    for x in (-W / 2 - 5, W / 2 + 5):
        for z in (-H / 2 - 5, H / 2 + 5):
            cut = cut + cyl_y(M3_CLR / 2, -1, 4, x=x, z=z)
    return body - cut


# --------------------------------------------------------------------------- P9
def p9_thermal_carrier():
    """P9 Thermal camera carrier: screws to the MG90S single-arm horn and
    holds the MLX90640 breakout upright, sensor looking along -y.
    Frame: horn face on z = 0, pan axis = z."""
    T = 2.5
    foot = box(-8, 8, -6, 6, 0, T)
    upright = box(-16, 16, -6, -6 + T, 0, 30)
    gus = prism_yz([(-6 + T - 0.01, T - 0.01), (4, T - 0.01), (-6 + T - 0.01, 8)], -1.5, 1.5)   # stays below the board
    body = union(foot, upright, gus)
    cut = cyl_z(3.2, -1, T + 1)                                   # horn hub clearance
    for x in (-5.5, 5.5):
        cut = cut + cyl_z(0.9, -1, T + 1, x=x)                    # horn arm screws
    cut = cut + cyl_y(MLX_CAN_D / 2, -7, -6 + T + 1, z=18)
    for dx in (-MLX_HOLES / 2, MLX_HOLES / 2):
        cut = cut + cyl_y(1.35, -7, -6 + T + 1, x=dx, z=11.6)
    return body - cut


# --------------------------------------------------------------------------- catalogue
@dataclass
class Part:
    pid: str
    name: str
    fn: object
    qty: int
    material: str
    note: str


PARTS = [
    Part("P1", "Pan base", p1_pan_base, 1, "ASA", "Clamps the post; holds pan servo and bearing"),
    Part("P2", "Yoke", p2_yoke, 1, "ASA", "Turntable plate with tilt-servo and bearing arms"),
    Part("P3a", "Cradle, horn half", p3a_cradle_horn_half, 1, "ASA", "Holds nozzle fitting; servo-horn side"),
    Part("P3b", "Cradle, axle half", p3b_cradle_axle_half, 1, "ASA", "Holds nozzle fitting; M8 axle side"),
    Part("P4", "Pipe clamp half", p4_pipe_clamp_half, 8, "PETG or ASA", "Two per clamp, four clamps"),
    Part("P5", "Mounting plate", p5_mount_plate, 2, "PETG or ASA", "Valve train plate and Turret Node plate"),
    Part("P6", "Post clip", p6_post_clip, 6, "PETG", "Tube and cable to post, every 150 mm"),
    Part("P7", "Sensor bracket", p7_sensor_bracket, 1, "PETG", "Visible camera and thermal pan servo, inside the Sensor Unit"),
    Part("P8", "Thermal visor", p8_thermal_hood, 1, "ASA", "Rain visor over the thermal window"),
    Part("P9", "Thermal carrier", p9_thermal_carrier, 1, "PETG", "MLX90640 on the micro-servo horn"),
]


def to_trimesh(m: Manifold):
    import trimesh
    mesh = m.to_mesh()
    return trimesh.Trimesh(vertices=np.asarray(mesh.vert_properties)[:, :3],
                           faces=np.asarray(mesh.tri_verts), process=True)


def build_all(out_dir):
    os.makedirs(out_dir, exist_ok=True)
    meshes = {}
    for p in PARTS:
        tm = to_trimesh(p.fn())
        assert tm.is_watertight, p.pid
        tm.export(os.path.join(out_dir, f"{p.pid}_{p.name.lower().replace(', ', '_').replace(' ', '_')}.stl"))
        meshes[p.pid] = tm
    return meshes


if __name__ == "__main__":
    here = os.path.dirname(os.path.abspath(__file__))
    ms = build_all(os.path.join(here, "stl"))
    for p in PARTS:
        m = ms[p.pid]
        ext = m.bounds[1] - m.bounds[0]
        print(f"{p.pid:4s} {p.name:20s} {ext[0]:6.1f} x {ext[1]:6.1f} x {ext[2]:6.1f} mm  "
              f"{m.volume / 1000:6.1f} cm3  watertight={m.is_watertight}")
