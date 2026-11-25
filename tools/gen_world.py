#!/usr/bin/env python3
"""Generates worlds/ur_scara_cell.wbt from lib/layout.py.

Kitting cell: an infeed conveyor brings round parts to a stop, an Epson SCARA T6-602S places them six at a time in a
tray, and a UR5e swaps each full tray for an empty one (full tray -> outfeed cart, empty tray <- magazine). A full cart
(2 x 2 x 2 trays) leaves through the scanner-guarded opening and comes back empty.
All static geometry is plain Shapes; only the robots have physics. Parts, trays and the cart are moved kinematically.
"""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "lib"))
import layout as L  # noqa: E402

OUT = os.path.join(HERE, "..", "worlds", "ur_scara_cell.wbt")
R = "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/"
PROTOS = [
    R + "objects/backgrounds/protos/TexturedBackground.proto",
    R + "objects/backgrounds/protos/TexturedBackgroundLight.proto",
    R + "robots/universal_robots/protos/UR5e.proto",
    R + "humans/pedestrian/protos/Pedestrian.proto",
    "../protos/ScaraT6Cell.proto",
]
N_PARTS = 64
N_TRAYS = L.MAG_COUNT + 1

out = []


def w(s):
    out.append(s)


def f(*v):
    return " ".join("%.5g" % x for x in v)


def app(color, rough=0.5, metal=0.0, emissive=None, transp=0.0, def_=None):
    d = "DEF %s " % def_ if def_ else ""
    e = " emissiveColor %s" % emissive if emissive else ""
    t = " transparency %g" % transp if transp else ""
    return "%sPBRAppearance { baseColor %s roughness %g metalness %g%s%s }" % (d, color, rough, metal, e, t)


ALU = app("0.78 0.8 0.82", 0.35, 0.8)
DARK = app("0.12 0.12 0.13", 0.7, 0.2)
STEEL = app("0.6 0.62 0.64", 0.3, 0.9)
YELLOW = app("0.98 0.76 0.05", 0.5)
BLACK = app("0.05 0.05 0.05", 0.6)
RUBBER = app("0.08 0.08 0.09", 0.9)
MESH = app("0.15 0.16 0.17", 0.6, 0.3, transp=0.55)
GREY = app("0.55 0.57 0.6", 0.6)
TRAY_BLUE = app("0.1 0.32 0.65", 0.55)
PART_APP = app("0.85 0.66 0.32", 0.28, 1.0)
CAP_APP = app("0.12 0.12 0.12", 0.5)


def box(size, a, pos=(0, 0, 0), rot=None):
    r = " rotation %s" % f(*rot) if rot else ""
    return "Pose { translation %s%s children [ Shape { appearance %s geometry Box { size %s } } ] }" % (
        f(*pos), r, a, f(*size))


def cyl(r_, h, a, pos=(0, 0, 0), rot=None, sub=24):
    r = " rotation %s" % f(*rot) if rot else ""
    return ("Pose { translation %s%s children [ Shape { appearance %s geometry Cylinder { radius %g height %g "
            "subdivision %d } } ] }" % (f(*pos), r, a, r_, h, sub))


def group(items):
    return "Group { children [\n  %s\n] }" % "\n  ".join(items)


def bar(p0, p1, s, a):
    """Square profile of side s between two points (axis-aligned)."""
    c = [(p0[i] + p1[i]) / 2 for i in range(3)]
    size = [max(abs(p1[i] - p0[i]), s) for i in range(3)]
    return box(size, a, c)


# ------------------------------------------------------------------ header
w("#VRML_SIM R2025a utf8")
for p in PROTOS:
    w('EXTERNPROTO "%s"' % p)
w("""WorldInfo {
  info [ "UR5e + Epson SCARA T6 kitting cell" ]
  title "UR5e + Epson SCARA kitting cell"
  basicTimeStep 16
  contactProperties [ ContactProperties { softCFM 0.001 } ]
}
DEF VIEW Viewpoint {
  orientation -0.2216 -0.1435 0.9645 4.8888
  position 0.55 4.3 2.65
  exposure 1.2
  follow "none"
}
TexturedBackground { texture "factory" luminosity 0.9 }
TexturedBackgroundLight { texture "factory" luminosity 1.1 }""")

# ------------------------------------------------------------------ floor and markings
fl = [box((14, 12, 0.02), app("0.46 0.47 0.48", 0.9), (0.3, 0, -0.01))]
x0, x1, y0, y1 = L.FENCE
for (a, b) in [((x0 - 0.1, y0 - 0.1), (x1 + 0.1, y0 - 0.1)), ((x0 - 0.1, y1 + 0.1), (x1 + 0.1, y1 + 0.1)),
               ((x0 - 0.1, y0 - 0.1), (x0 - 0.1, y1 + 0.1)), ((x1 + 0.1, y0 - 0.1), (x1 + 0.1, y1 + 0.1))]:
    n = int(math.dist(a, b) / 0.2)
    for i in range(n):
        t0, t1 = i / n, (i + 1) / n
        cx, cy = a[0] + (b[0] - a[0]) * (t0 + t1) / 2, a[1] + (b[1] - a[1]) * (t0 + t1) / 2
        horiz = abs(b[0] - a[0]) > abs(b[1] - a[1])
        size = (math.dist(a, b) / n, 0.08, 0.002) if horiz else (0.08, math.dist(a, b) / n, 0.002)
        fl.append(box(size, YELLOW if i % 2 == 0 else BLACK, (cx, cy, 0.001)))
# cart lane (blue) from the gate to the cart buffer, operator walkway (green)
fl.append(box((0.9, 2.2, 0.002), app("0.1 0.3 0.75", 0.8), (L.CART[0], -1.75, 0.0012)))
fl.append(box((0.6, 6.0, 0.002), app("0.1 0.55 0.25", 0.8), (2.6, 0.0, 0.0012)))
# scanner zones: warning (yellow, translucent) and protective stop (red)
sx, sy = L.SCANNER


def sector(r, a0, a1, a, z, n=40):
    """Flat circular sector (scanner field painted on the floor)."""
    pts = [(0, 0)] + [(r * math.cos(a0 + (a1 - a0) * i / n), r * math.sin(a0 + (a1 - a0) * i / n)) for i in range(n + 1)]
    coords = ", ".join("%.4g %.4g %.4g" % (sx + x, sy + y, z) for x, y in pts)
    idx = " ".join("0 %d %d -1" % (i, i + 1) for i in range(1, n + 1))
    return ("Shape { appearance %s geometry IndexedFaceSet { coord Coordinate { point [ %s ] } coordIndex [ %s ] "
            "solid FALSE } }" % (a, coords, idx))


# fields cover the half-plane outside the south fence (scanner mounted on the gate post, looking south)
fl.append(sector(L.WARN_R, math.pi, 2 * math.pi, app("1 0.8 0", 0.9, transp=0.6), 0.003))
fl.append(sector(L.STOP_R, math.pi, 2 * math.pi, app("0.9 0.1 0.05", 0.9, transp=0.5), 0.004))
w("Solid { name \"floor\" children [ %s ] boundingObject Plane { size 14 12 } locked TRUE }" % group(fl))

# ------------------------------------------------------------------ work table (aluminium frame + plate)
tb = []
tx0, tx1, ty0, ty1, tz = L.TABLE_X0, L.TABLE_X1, L.TABLE_Y0, L.TABLE_Y1, L.TABLE_Z
tb.append(box((tx1 - tx0, ty1 - ty0, 0.02), app("0.35 0.37 0.4", 0.4, 0.5), ((tx0 + tx1) / 2, (ty0 + ty1) / 2, tz - 0.01)))
for x in (tx0 + 0.04, tx1 - 0.04):
    for y in (ty0 + 0.04, ty1 - 0.04):
        tb.append(bar((x, y, 0.03), (x, y, tz - 0.02), 0.06, ALU))
        tb.append(cyl(0.035, 0.03, RUBBER, (x, y, 0.015)))
for z in (0.15, tz - 0.05):
    tb.append(bar((tx0 + 0.04, ty0 + 0.04, z), (tx1 - 0.04, ty0 + 0.04, z), 0.045, ALU))
    tb.append(bar((tx0 + 0.04, ty1 - 0.04, z), (tx1 - 0.04, ty1 - 0.04, z), 0.045, ALU))
    tb.append(bar((tx0 + 0.04, ty0 + 0.04, z), (tx0 + 0.04, ty1 - 0.04, z), 0.045, ALU))
    tb.append(bar((tx1 - 0.04, ty0 + 0.04, z), (tx1 - 0.04, ty1 - 0.04, z), 0.045, ALU))
# SCARA riser, UR mounting plate, station locator, magazine guides
sbx, sby, sbz, _ = L.SCARA_BASE
tb.append(box((0.3, 0.26, L.SCARA_RISER), GREY, (sbx, sby, tz + L.SCARA_RISER / 2)))
ubx, uby, _, _ = L.UR_BASE
tb.append(cyl(0.11, 0.012, DARK, (ubx, uby, tz + 0.006)))
stx, sty = L.STATION
for dx in (-1, 1):
    tb.append(box((0.01, L.TRAY_W + 0.04, 0.012), DARK, (stx + dx * (L.TRAY_L / 2 + 0.008), sty, tz + 0.006)))
mx, my = L.MAG
hmag = L.MAG_COUNT * L.TRAY_H + 0.02
for dx in (-1, 1):
    for dy in (-1, 1):
        tb.append(bar((mx + dx * (L.TRAY_L / 2 + 0.012), my + dy * (L.TRAY_W / 2 - 0.02), tz),
                      (mx + dx * (L.TRAY_L / 2 + 0.012), my + dy * (L.TRAY_W / 2 - 0.02), tz + hmag), 0.016, ALU))
w('Solid { name "work table" children [ %s ] }' % group(tb))

# ------------------------------------------------------------------ infeed conveyor
cv = []
cx0, cx1, cy, cz, cw = L.CONV_X0, L.CONV_X1, L.CONV_Y, L.CONV_Z, L.CONV_W
cv.append(box((cx1 - cx0, cw, 0.01), RUBBER, ((cx0 + cx1) / 2, cy, cz - 0.005)))
for s in (-1, 1):
    cv.append(box((cx1 - cx0 + 0.04, 0.02, 0.07), ALU, ((cx0 + cx1) / 2, cy + s * (cw / 2 + 0.01), cz - 0.03)))
    # side guides, set 4 mm wider than the part
    gy = cy + s * (L.PART_R + 0.008)
    cv.append(box((cx1 - cx0 - 0.1, 0.004, 0.02), app("0.85 0.87 0.9", 0.25, 0.9), ((cx0 + cx1) / 2 + 0.05, gy, cz + 0.025)))
    for x in [cx0 + 0.15 + 0.3 * i for i in range(5)]:
        cv.append(box((0.01, abs(gy - (cy + s * (cw / 2 + 0.01))), 0.006), STEEL,
                      (x, (gy + cy + s * (cw / 2 + 0.01)) / 2, cz + 0.03)))
for x in (cx0, cx1):
    cv.append(cyl(0.03, cw, STEEL, (x, cy, cz - 0.03), (1, 0, 0, math.pi / 2)))
for x in (cx0 + 0.1, cx1 - 0.1 if cx1 - 0.1 < L.TABLE_X0 else L.TABLE_X0 - 0.06):
    for s in (-1, 1):
        cv.append(bar((x, cy + s * 0.05, 0.02), (x, cy + s * 0.05, cz - 0.06), 0.04, ALU))
        cv.append(cyl(0.03, 0.02, RUBBER, (x, cy + s * 0.05, 0.01)))
cv.append(box((0.012, cw - 0.02, 0.05), DARK, (cx1 - L.PART_R - 0.001 + 0.006, cy, cz + 0.025)))   # end stop
cv.append(cyl(0.05, 0.14, app("0.2 0.35 0.6", 0.4, 0.4), (cx0 + 0.05, cy - cw / 2 - 0.08, cz - 0.05), (1, 0, 0, math.pi / 2)))
cv.append(box((0.1, 0.1, 0.1), app("0.2 0.35 0.6", 0.4, 0.4), (cx0 + 0.05, cy - cw / 2 - 0.12, cz - 0.05)))
# sensor housings (B1 stop, B5 accumulation) with reflectors opposite
for x in (L.PICK[0], L.ACCU_X):
    cv.append(box((0.02, 0.03, 0.04), app("0.95 0.5 0.05", 0.5), (x, cy - 0.105, cz + 0.02)))
    cv.append(box((0.02, 0.006, 0.03), app("0.8 0.1 0.1", 0.3), (x, cy + 0.07, cz + 0.02)))
# upstream machine (part feeder) and the fence tunnel
cv.append(box((0.5, 0.6, 1.3), app("0.85 0.86 0.88", 0.5), (cx0 - 0.3, cy, 0.65)))
cv.append(box((0.3, 0.02, 0.4), app("0.6 0.8 0.95", 0.1, transp=0.5), (cx0 - 0.3, cy - 0.31, 1.0)))
cv.append(box((0.5, 0.6, 0.04), app("0.2 0.35 0.6", 0.4, 0.4), (cx0 - 0.3, cy, 1.32)))
w('Solid { name "infeed conveyor" children [ %s ] }' % group(cv))

# ------------------------------------------------------------------ fence, door, gate
fz = 0.1
fh = 2.0
fe = []


def fence_run(a, b, skip=None):
    """Posts and mesh panels from a to b (axis-aligned); skip = (lo, hi) along the run left open."""
    horiz = abs(b[1] - a[1]) < 1e-9
    lo, hi = (a[0], b[0]) if horiz else (a[1], b[1])
    if lo > hi:
        lo, hi = hi, lo
    cuts = [lo]
    n = max(1, round((hi - lo) / 1.0))
    cuts += [lo + (hi - lo) * i / n for i in range(1, n)] + [hi]
    if skip:
        cuts = sorted(set([c for c in cuts if not skip[0] < c < skip[1]] + [skip[0], skip[1]]))
    for c in cuts:
        x, y = (c, a[1]) if horiz else (a[0], c)
        fe.append(bar((x, y, 0), (x, y, fh + fz), 0.05, YELLOW))
        fe.append(box((0.12, 0.12, 0.008), YELLOW, (x, y, 0.004)))
    for c0, c1 in zip(cuts[:-1], cuts[1:]):
        if skip and c0 >= skip[0] - 1e-9 and c1 <= skip[1] + 1e-9:
            continue
        m = (c0 + c1) / 2
        L_ = c1 - c0 - 0.05
        if horiz:
            fe.append(box((L_, 0.01, fh - 0.1), MESH, (m, a[1], fz + (fh - 0.1) / 2)))
            fe.append(box((L_, 0.03, 0.03), YELLOW, (m, a[1], fz + fh - 0.05)))
        else:
            fe.append(box((0.01, L_, fh - 0.1), MESH, (a[0], m, fz + (fh - 0.1) / 2)))
            fe.append(box((0.03, L_, 0.03), YELLOW, (a[0], m, fz + fh - 0.05)))


dx_, dy_, dwid = L.DOOR
fence_run((x0, y0), (x1, y0), skip=L.GATE)                                   # south: cart opening
fence_run((x0, y1), (x1, y1), skip=(dx_ - dwid / 2, dx_ + dwid / 2))          # north: door
fence_run((x1, y0), (x1, y1))
fence_run((x0, y0), (x0, y1), skip=(cy - 0.12, cy + 0.12))                   # west: conveyor opening
# tunnel over the conveyor opening (ISO 13857: no reach to the SCARA through the opening)
fe.append(box((0.6, 0.006, 0.25), app("0.75 0.85 0.95", 0.1, transp=0.6), (x0 - 0.3, cy - 0.12, cz + 0.12)))
fe.append(box((0.6, 0.006, 0.25), app("0.75 0.85 0.95", 0.1, transp=0.6), (x0 - 0.3, cy + 0.12, cz + 0.12)))
fe.append(box((0.6, 0.246, 0.006), app("0.75 0.85 0.95", 0.1, transp=0.6), (x0 - 0.3, cy, cz + 0.245)))
fe.append(box((0.01, 0.24, fh - 0.25 - cz + fz), MESH, (x0, cy, cz + 0.25 + (fh + fz - cz - 0.25) / 2)))
# door interlock switch on the latch post
fe.append(box((0.04, 0.03, 0.08), app("0.9 0.85 0.1", 0.4), (dx_ + dwid / 2 + 0.04, y1 - 0.03, 1.1)))
w('Solid { name "safety fence" children [ %s ] }' % group(fe))
w('DEF DOOR Pose { translation %s children [ %s ] }' % (
    f(dx_ - dwid / 2, y1, 0), group([box((dwid - 0.04, 0.01, fh - 0.1), MESH, (dwid / 2, 0, fz + (fh - 0.1) / 2)),
                                     box((dwid - 0.04, 0.03, 0.03), YELLOW, (dwid / 2, 0, fz + fh - 0.05)),
                                     box((0.03, 0.03, fh - 0.1), YELLOW, (dwid - 0.035, 0, fz + (fh - 0.1) / 2)),
                                     box((0.02, 0.06, 0.15), BLACK, (dwid - 0.08, 0.03, 1.05))])))

# ------------------------------------------------------------------ area scanner, console, stack light
sc = [box((0.12, 0.1, 0.12), app("0.95 0.8 0.1", 0.5), (sx, sy - 0.06, 0.2)),
      cyl(0.04, 0.05, BLACK, (sx, sy - 0.06, 0.285)),
      bar((sx, sy - 0.06, 0), (sx, sy - 0.06, 0.14), 0.04, DARK)]
# operator console with e-stop and HMI
hx, hy = 1.85, 0.55
sc += [box((0.35, 0.3, 1.0), app("0.85 0.86 0.88", 0.5), (hx, hy, 0.5)),
       box((0.35, 0.3, 0.04), DARK, (hx, hy, 1.02)),
       box((0.02, 0.26, 0.18), app("0.05 0.1 0.15", 0.2, emissive="0.05 0.2 0.35"), (hx - 0.17, hy, 0.82)),
       cyl(0.03, 0.03, app("0.9 0.85 0.1", 0.5), (hx - 0.18, hy - 0.08, 0.62), (0, 1, 0, math.pi / 2)),
       cyl(0.022, 0.04, app("0.85 0.05 0.05", 0.4), (hx - 0.2, hy - 0.08, 0.62), (0, 1, 0, math.pi / 2)),
       cyl(0.015, 0.02, app("0.1 0.6 0.95", 0.4), (hx - 0.18, hy + 0.06, 0.62), (0, 1, 0, math.pi / 2))]
lx, ly = 1.85, 0.85
sc.append(bar((lx, ly, 0), (lx, ly, 2.0), 0.03, DARK))
w('Solid { name "scanner and console" children [ %s ] }' % group(sc))
lamps = []
for i, (name, col) in enumerate([("RED", "0.9 0.1 0.08"), ("AMBER", "1 0.6 0.05"), ("GREEN", "0.1 0.85 0.2")]):
    lamps.append("Pose { translation %s children [ Shape { appearance PBRAppearance { baseColor %s roughness 0.3 "
                 "transparency 0.15 emissiveColor 0 0 0 } geometry Cylinder { radius 0.04 height 0.08 } } ] }"
                 % (f(lx, ly, 2.25 - i * 0.085), col))
    lamps[-1] = lamps[-1].replace("appearance PBRAppearance", "appearance DEF LAMP_%s PBRAppearance" % name)
lamps.append(cyl(0.04, 0.04, DARK, (lx, ly, 2.02)))
w('Pose { children [ %s ] }' % ", ".join(lamps))

# ------------------------------------------------------------------ outfeed cart (kinematic)
ct = []
cw_, cl_ = 0.66, 0.5
ct.append(box((cw_, cl_, 0.03), app("0.25 0.27 0.3", 0.5, 0.5), (0, 0, L.CART_TOP - 0.015)))
ct.append(box((cw_, cl_, 0.02), app("0.25 0.27 0.3", 0.5, 0.5), (0, 0, 0.2)))
for dx in (-1, 1):
    for dy in (-1, 1):
        ct.append(bar((dx * (cw_ / 2 - 0.02), dy * (cl_ / 2 - 0.02), 0.11), (dx * (cw_ / 2 - 0.02), dy * (cl_ / 2 - 0.02), L.CART_TOP - 0.03), 0.03, STEEL))
        ct.append(cyl(0.05, 0.035, RUBBER, (dx * (cw_ / 2 - 0.06), dy * (cl_ / 2 - 0.06), 0.05), (0, 1, 0, math.pi / 2)))
        ct.append(box((0.05, 0.06, 0.04), STEEL, (dx * (cw_ / 2 - 0.06), dy * (cl_ / 2 - 0.06), 0.1)))
ct.append(bar((-cw_ / 2 + 0.05, -cl_ / 2 - 0.03, 0.95), (cw_ / 2 - 0.05, -cl_ / 2 - 0.03, 0.95), 0.03, STEEL))
for dx in (-1, 1):
    ct.append(bar((dx * (cw_ / 2 - 0.05), -cl_ / 2 - 0.03, L.CART_TOP), (dx * (cw_ / 2 - 0.05), -cl_ / 2 - 0.03, 0.95), 0.025, STEEL))
# docking locator on the floor
w('Solid { name "cart dock" children [ %s ] }' % group([box((0.7, 0.03, 0.03), YELLOW, (L.CART[0], L.CART[1] + cl_ / 2 + 0.03, 0.015))]))
w('DEF CART Solid { translation %s name "outfeed cart" children [ %s ] boundingObject Pose { translation 0 0 0.3 '
  'children [ Box { size %s } ] } }' % (f(L.CART[0], L.CART[1], 0), group(ct), f(cw_, cl_, 0.2)))

# ------------------------------------------------------------------ robots
handslot = ("handSlot [ Pose { children [ %s ] } DEF SCARA_TCP Pose { translation 0 0 %g } ]" % (", ".join([
    cyl(0.012, 0.03, STEEL, (0, 0, -0.015)),
    box((0.03, 0.03, 0.012), app("0.15 0.15 0.15", 0.4), (0, 0, -0.03)),
    cyl(0.016, 0.016, app("0.1 0.3 0.85", 0.6), (0, 0, -0.044)),
    cyl(0.019, 0.004, app("0.1 0.3 0.85", 0.6), (0, 0, -0.058)),
    cyl(0.004, 0.06, app("0.1 0.5 0.9", 0.5, transp=0.3), (0.025, 0, -0.01), (0, 1, 0, 0.3)),
]), -L.SCARA_TOOL))
w('ScaraT6Cell { translation %s rotation 0 0 1 %g name "Epson SCARA T6" controller "scara_robot" supervisor TRUE %s }'
  % (f(sbx, sby, sbz), L.SCARA_BASE[3], handslot))


def finger(side):
    s = -1 if side == "l" else 1
    x0_ = s * 0.135
    return ("SliderJoint { jointParameters JointParameters { axis %s } device [ LinearMotor { name \"finger_%s_motor\" "
            "minPosition 0 maxPosition 0.02 maxVelocity 0.1 maxForce 80 } ] endPoint Solid { translation %s children [ %s ] "
            "name \"finger %s\" } }" % (f(-s, 0, 0), side, f(x0_, 0.12, 0), ", ".join([
                box((0.008, 0.09, 0.05), DARK),
                box((0.004, 0.06, 0.044), RUBBER, (-s * 0.005, 0.012, 0)),
                box((0.03, 0.02, 0.05), STEEL, (s * 0.01, -0.045, 0))]), side))


toolslot = "toolSlot [ %s ]" % ", ".join([
    cyl(0.0315, 0.012, STEEL, (0, 0.006, 0), (1, 0, 0, math.pi / 2)),
    cyl(0.04, 0.03, DARK, (0, 0.027, 0), (1, 0, 0, math.pi / 2)),
    box((0.32, 0.04, 0.06), app("0.2 0.22 0.25", 0.4, 0.6), (0, 0.062, 0)),
    box((0.30, 0.012, 0.03), STEEL, (0, 0.088, 0)),
    box((0.05, 0.03, 0.04), app("0.1 0.4 0.8", 0.4), (0, 0.06, 0.05)),
    finger("l"), finger("r"),
    "DEF UR_TCP Pose { translation 0 %g 0 }" % L.UR_TOOL])
w('UR5e { translation %s rotation 0 0 1 %g name "UR5e" controller "ur_robot" supervisor TRUE selfCollision FALSE %s }'
  % (f(ubx, uby, L.UR_BASE[2]), L.UR_BASE[3], toolslot))

# ------------------------------------------------------------------ trays and parts (pools, kinematic)
tray_children = [box((L.TRAY_L, L.TRAY_W, L.TRAY_FLOOR), TRAY_BLUE, (0, 0, L.TRAY_FLOOR / 2))]
t_ = 0.004
for s in (-1, 1):
    tray_children.append(box((t_, L.TRAY_W, L.TRAY_H), TRAY_BLUE, (s * (L.TRAY_L / 2 - t_ / 2), 0, L.TRAY_H / 2)))
    tray_children.append(box((L.TRAY_L, t_, L.TRAY_H), TRAY_BLUE, (0, s * (L.TRAY_W / 2 - t_ / 2), L.TRAY_H / 2)))
for (px, py) in L.POCKETS:
    tray_children.append(cyl(L.PART_R + 0.003, 0.002, app("0.05 0.15 0.35", 0.7), (px, py, L.TRAY_FLOOR + 0.001), sub=16))
TRAY_SHAPES = group(tray_children)
for i in range(N_TRAYS):
    if i < L.MAG_COUNT:
        pos = (L.MAG[0], L.MAG[1], L.TABLE_Z + i * L.TRAY_H)
    else:
        pos = (L.STATION[0], L.STATION[1], L.TABLE_Z)
    w('DEF TRAY_%d Solid { translation %s name "tray %d" children [ %s ] boundingObject Pose { translation 0 0 %g '
      'children [ Box { size %s } ] } }' % (i, f(*pos), i, TRAY_SHAPES if i == 0 else "USE TRAY_GEOM", L.TRAY_H / 2,
                                           f(L.TRAY_L, L.TRAY_W, L.TRAY_H)))
    if i == 0:
        out[-1] = out[-1].replace("children [ Group", "children [ DEF TRAY_GEOM Group", 1)
part_shapes = group([cyl(L.PART_R, L.PART_H, PART_APP, sub=20),
                     cyl(L.PART_R * 0.55, 0.002, CAP_APP, (0, 0, L.PART_H / 2 + 0.001), sub=16)])
for i in range(N_PARTS):
    pos = (-3.0 + 0.06 * (i % 16), -4.0 - 0.06 * (i // 16), -0.5)      # parked under the floor
    w('DEF PART_%d Solid { translation %s name "part %d" children [ %s ] boundingObject Cylinder { radius %g height %g } }'
      % (i, f(*pos), i, part_shapes if i == 0 else "USE PART_GEOM", L.PART_R, L.PART_H))
    if i == 0:
        out[-1] = out[-1].replace("children [ Group", "children [ DEF PART_GEOM Group", 1)

# ------------------------------------------------------------------ PLC (supervisor) with the real cell sensors


def dsensor(name, pos, yaw):
    return ('DistanceSensor { translation %s rotation 0 0 1 %g name "%s" lookupTable [ 0 0 0, 0.5 0.5 0 ] '
            'numberOfRays 1 }' % (f(*pos), yaw, name))


plc_dev = [
    dsensor("B1", (L.PICK[0], cy - 0.09, cz + 0.02), math.pi / 2),                 # part at the stop
    dsensor("B5", (L.ACCU_X, cy - 0.09, cz + 0.02), math.pi / 2),                  # accumulation full
    dsensor("B2", (L.STATION[0], L.STATION[1] - 0.2, L.TABLE_Z + 0.025), math.pi / 2),   # tray at station
    dsensor("B7", (L.CART[0] + 0.45, L.CART[1], 0.3), math.pi),                     # cart docked
    'Emitter { name "to_scara" channel 1 }', 'Receiver { name "from_scara" channel 2 }',
    'Emitter { name "to_ur" channel 3 }', 'Receiver { name "from_ur" channel 4 }',
    'Emitter { name "to_operator" channel 5 }',
]
w('Robot { name "cell PLC" controller "cell_plc" supervisor TRUE children [ %s ] }' % ", ".join(plc_dev))

# ------------------------------------------------------------------ operator
ox, oy = L.OPERATOR_HOME
w('DEF OPERATOR Pedestrian { translation %s rotation 0 0 1 %g name "operator" controller "operator" '
  'controllerArgs [ "--cell" ] shirtColor 0.95 0.45 0.05 pantsColor 0.15 0.18 0.25 '
  'bodySlot [ Receiver { name "op_rx" channel 5 } ] }' % (f(ox, oy, 1.27), math.pi))

text = "\n".join(out) + "\n"
# radios ride on the robots: add them to the hand / tool slots
text = text.replace("handSlot [ ", 'handSlot [ Receiver { name "scara_rx" channel 1 } Emitter { name "scara_tx" channel 2 } ', 1)
text = text.replace("toolSlot [ ", 'toolSlot [ Receiver { name "ur_rx" channel 3 } Emitter { name "ur_tx" channel 4 } ', 1)
os.makedirs(os.path.dirname(OUT), exist_ok=True)
with open(OUT, "w") as fh:
    fh.write(text)
print("wrote", os.path.abspath(OUT), len(text) // 1024, "kB")
