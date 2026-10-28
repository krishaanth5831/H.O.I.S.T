#!/usr/bin/env python
"""HOIST design options in motion: end effector A (wire rider) or B (hook carrier) on SPINE lift
S1 (mast), S2 (scissor) or S3 (telescopic). Each run lowers and rehangs two plants: one in open
wire, then one whose hook has to pass a wire support.

Kinematic sketch, not a simulation. Geometry mirrors design/hoist_design_options.fs (same numbers,
MuJoCo primitives instead of the rounded CAD plates). Every body is mocap; twine payout, slack and
the plant drop are bookkeeping. The twine a hook lets out per half-flip, the free twine and the
plant are unmeasured guesses.

    python design/hoist_demo.py --ee A --lift S1 --windowed     # watch it in a window
    python design/hoist_demo.py --ee A --lift S1                # runs/hoist_demo/hoist_demo_A_S1.mp4
    python design/hoist_demo.py --ee B --lift S2 --checks-only  # clearance checks only

Every run prints PASS/FAIL clearance checks first: the tool and the hook it carries against the wire
support, the flipping hook against the tool's stages and the crop wire, the tool against the other
hooks and plants, and the tool against the crop wire except the parts meant to rest on it. Gaps come
from a small convex program, not mj_geomDistance (see the ⚠️ above CAP).

Both tools take the hook with a side-entry fork that slides onto it along the wire. ⚠️ A U-yoke
closed on its -X side traps the hook after the half-flip: undocking measured -9 mm.
"""
import argparse
import math
import os
import sys
import time
from pathlib import Path

import numpy as np

# ---------------------------------------------------------------- layout (mm), = hoist_design_options.fs
WIRE_X = 400.0        # crop wire 125 outboard of the pipe rail, unverified
WIRE_Z = 3800.0       # high-wire height, unverified
ROW_Y = 2200.0        # drawn row length, arbitrary
HOOK_YS = [-1050.0, -700.0, -350.0, 0.0, 650.0]   # 350 pitch, unverified; +650 already moved, 0 is next
SUPPORT_Y = -175.0    # one support between two hooks, unverified
ZB_WORK = 3170.0      # Y-beam underside at work: puts the tool mount 440 under the wire
C0 = np.array([-7.0, 0.0, -103.0])     # hook centre = rotor axis, tool frame (stylised hook)
EXIT = np.array([-7.0, 0.0, -108.5])   # twine leaves the bottom cup, relative to the hook centre
NIP = np.array([-14.0, 0.0, -290.0])   # A capstan nip, tool frame
K_RELEASE = 217.0     # twine one half-flip lets out (~ one hook length), unverified: sets the drop
FREE_TWINE = 300.0    # hook to plant head before lowering, unverified
TAKE_UP = 5.0         # A: capstan lifts the plant this much to unload the hook, guess
SLIDE = 25.0          # fork slides this far along the wire onto the hook (fingers reach 12 past it), design choice
JAW_OPEN = 7.5        # each finger's opening stroke, design choice
IDLER_OPEN = 45.0     # A: idler stroke, clears the twine 25 off-centre while the fork slides, design choice
LOW_DZ = {"S1": 1500.0 - ZB_WORK, "S2": 1020.0 - ZB_WORK, "S3": 1395.0 - ZB_WORK}  # lift collapsed
EXT_WORK = (ZB_WORK - 20.0 - 1375.0) / 3.0   # S3: each stage's extension at work height
L_ARM, Z_PIN0, Y_FIX = 1000.0, 410.0, -500.0  # S2 scissor: arm pin pitch, bottom pins, fixed-pin Y

FPS = 30
W, H = 640, 720       # one view; the video puts two side by side

COL = {
    "floor": (0.80, 0.80, 0.78), "steel": (0.58, 0.60, 0.63), "wire": (0.25, 0.25, 0.27),
    "galv": (0.70, 0.72, 0.74), "hook": (0.12, 0.45, 0.28), "twine": (0.95, 0.94, 0.88),
    "plant": (0.30, 0.58, 0.22), "alu": (0.80, 0.81, 0.83), "dark": (0.22, 0.23, 0.25),
    "rubber": (0.08, 0.08, 0.08), "A": (0.18, 0.42, 0.78), "A2": (0.40, 0.62, 0.92),
    "B": (0.86, 0.46, 0.14), "B2": (0.95, 0.66, 0.35), "sensor": (0.93, 0.76, 0.18),
    "cart": (0.30, 0.32, 0.35), "lift": (0.66, 0.68, 0.70), "scissor": (0.93, 0.72, 0.16),
    "head": (0.36, 0.48, 0.58), "head2": (0.20, 0.30, 0.40), "elec": (0.42, 0.52, 0.48),
}
NAMES = {"A": "EE-A wire rider", "B": "EE-B hook carrier",
         "S1": "S1 mast", "S2": "S2 scissor", "S3": "S3 telescopic"}
LOADS = {"wire": ("plant hangs on its hook, on the crop wire", (46, 125, 50)),
         "capstan": ("plant hangs on the capstan; the tool rests on the wire", (25, 90, 160)),
         "lift": ("plant hangs on the tool: the lift carries it", (200, 95, 20))}


# ---------------------------------------------------------------- primitives (mm, in a body frame)
def box(p, q, c):
    return {"t": "box", "p": np.array(p, float), "q": np.array(q, float), "c": c}


def cyl(p, q, r, c, t="cylinder"):
    return {"t": t, "p": np.array(p, float), "q": np.array(q, float), "r": r, "c": c}


def sph(p, r, c):
    return {"t": "sphere", "p": np.array(p, float), "r": r, "c": c}


def ell(p, size, mat, c):
    return {"t": "ellipsoid", "p": np.array(p, float), "size": size, "mat": mat, "c": c}


def seat(g):
    """Mark a geom that rests on the crop wire by design (saddle roller, rest pad): the wire check skips it."""
    g["n"] = "seat"
    return g


def mirror_x(geoms):
    out = []
    for g in geoms:
        g = dict(g)
        for k in ("p", "q"):
            if k in g:
                g[k] = g[k] * np.array([-1.0, 1.0, 1.0])
        if "mat" in g:   # rebuild a right-handed frame from the mirrored long and side axes
            m = g["mat"] * np.array([[-1.0], [1.0], [1.0]])
            x, y = m[:, 0], m[:, 1]
            z = np.cross(x, y)
            g["mat"] = np.column_stack([x, np.cross(z, x), z])
        out.append(g)
    return out


def shift(geoms, d):
    out = []
    for g in geoms:
        g = dict(g)
        for k in ("p", "q"):
            if k in g:
                g[k] = g[k] + np.asarray(d, float)
        out.append(g)
    return out


# ---------------------------------------------------------------- greenhouse
def hook_geoms():
    """Stylised S hook in its own frame (origin = hook centre): 3 mm wire, shank 200, arcs R7."""
    pts = [(7.0, -18.0)]
    pts += [(7 * math.cos(math.radians(a)), -3 + 7 * math.sin(math.radians(a))) for a in range(0, 181, 30)]
    pts += [(-14 + 7 * math.cos(math.radians(a)), -203 + 7 * math.sin(math.radians(a))) for a in range(0, -181, -30)]
    pts.append((-21.0, -188.0))
    pts = [np.array([x, 0.0, z]) - C0 for x, z in pts]
    gs = [cyl(a, b, 1.5, "hook", "capsule") for a, b in zip(pts[:-1], pts[1:])]
    gs.append(box((-6, -2.5, -80), (6, 2.5, 80), "twine"))      # twine wound on the shank
    return gs


def plant_geoms():
    """Head, stem and three leaves in the plant frame (origin = head centre). Stem leans off the path."""
    u = np.array([250.0, -400.0, -2200.0])
    u /= np.linalg.norm(u)
    gs = [sph((0, 0, 0), 25, "plant"), cyl((0, 0, 0), u * 2250, 7, "plant", "capsule")]
    dn = math.radians(25)
    for at, h in zip((70, 160, 250), ((1.0, 0.0), (0.3, 1.0), (0.3, -1.0))):
        h = np.array(h) / np.linalg.norm(h)
        d = np.array([h[0] * math.cos(dn), h[1] * math.cos(dn), -math.sin(dn)])
        side = np.array([-h[1], h[0], 0.0])
        n = np.cross(d, side)
        n /= np.linalg.norm(n)
        gs.append(ell(u * at + d * 125, (125, 40, 1.5), np.column_stack([d, np.cross(n, d), n]), "plant"))
    return gs


def context_geoms():
    """Static world: floor, pipe rails, wires, supports, and the -X row (hooks, twine, plants)."""
    named = [("floor", box((-900, -ROW_Y, -20), (900, ROW_Y, 0), "floor"))]
    for s, sn in ((-1, "-X"), (1, "+X")):
        named.append((f"rail{sn}", cyl((s * 275, -ROW_Y, 120), (s * 275, ROW_Y, 120), 25.5, "steel")))
        for y in range(-2000, 2001, 1000):
            named.append((f"railfoot{sn}{y}", box((s * 275 - 30, y - 15, 0), (s * 275 + 30, y + 15, 100), "steel")))
        named.append((f"wire{sn}", cyl((s * WIRE_X, -ROW_Y, WIRE_Z), (s * WIRE_X, ROW_Y, WIRE_Z), 2.5, "wire")))
        named.append((f"support{sn}clamp", box((s * WIRE_X - 10, SUPPORT_Y - 15, WIRE_Z - 12),
                                               (s * WIRE_X + 10, SUPPORT_Y + 15, WIRE_Z + 20), "galv")))
        named.append((f"support{sn}rod", cyl((s * WIRE_X, SUPPORT_Y, WIRE_Z + 15),
                                             (s * WIRE_X, SUPPORT_Y, WIRE_Z + 600), 4, "galv")))
    for y in HOOK_YS:   # the -X row stays put
        c = np.array([-(WIRE_X + C0[0]), y, WIRE_Z + C0[2]])
        for j, g in enumerate(shift(mirror_x(hook_geoms()), c)):
            named.append((f"xhook{y}_{j}", g))
        head = np.array([-(WIRE_X - 14), y, WIRE_Z - 536.5])
        for j, g in enumerate(shift(mirror_x(plant_geoms()), head)):
            named.append((f"xplant{y}_{j}", g))
        named.append((f"xtwine{y}", cyl(head + (0, 0, 300 + 25), head + (0, 0, 25), 1, "twine", "capsule")))
    return named


# ---------------------------------------------------------------- end effectors (tool frame)
def yoke_geoms(g, hub_r):
    """Side-entry fork: hub plate on the -Y side, fingers reach +Y past the hook and close on it in X.
    ⚠️ A U-yoke closed on its -X side traps the hook: after a half-flip the closed side is in front of
    the hook and the tool can't back away. The fork slides on and off along the wire instead."""
    col, jcol = (g, g + "2")
    rel = lambda p: tuple(np.array(p, float) - C0)
    return {
        "T_yoke": [cyl(rel((-7, -28, -103)), rel((-7, -22, -103)), hub_r, col),
                   box(rel((-42, -24, -123)), rel((20, -20, -83)), col),
                   box(rel((-56, -24, -113)), rel((-42, -10, -93)), col)],
        "T_jawL": [box(rel((-23, -20, -123)), rel((-13, 12, -83)), jcol)],
        "T_jawR": [box(rel((-1, -20, -123)), rel((9, 12, -83)), jcol)],
    }


def tool_A():
    fr = [box((-300, -50, -440), (-200, 50, -430), "dark"), box((-300, -50, -430), (-200, 50, -420), "rubber"),
          box((-300, -100, -420), (-200, 100, -410), "alu"), box((-300, -100, -420), (-290, 100, 80), "alu"),
          box((-300, -100, -200), (-25, -90, -30), "alu"), box((-300, -85, 62), (25, 85, 72), "alu")]
    for yc in (-50, 50):   # the hook fits between them 25 off-centre; the rear one clears the support
        fr += [box((xs, yc - 23, 10), (xs + 3, yc + 23, 63), "alu") for xs in (-12, 9)]
        fr += [seat(g) for g in (cyl((-7, yc, 22.5), (7, yc, 22.5), 20, "rubber"),
                                 cyl((-9, yc, 22.5), (-7, yc, 22.5), 23, "rubber"),
                                 cyl((7, yc, 22.5), (9, yc, 22.5), 23, "rubber"),
                                 cyl((-12, yc, 22.5), (12, yc, 22.5), 3, "steel"))]
    fr += [box((-290, -90, -70), (-20, -82, -50), "steel"),
           box((-130, 40, 32), (-100, 70, 62), "rubber"), cyl((-100, 55, 47), (-90, 55, 47), 7, "rubber"),
           box((-75, 30, -52), (-70, 50, 62), "sensor"), cyl((-70, 40, -40), (-30, 40, -40), 6, "sensor")]
    g = {"T_frame": fr,
         "T_rx": [box((-55, -82, -80), (-15, -68, -40), "A"), box((-50, -68, -135), (-20, -64, -45), "A")],
         "T_rz": [box((-45, -64, -120), (-25, -52, -86), "A"), box((-60, -64, -320), (-25, -52, -120), "A"),
                  box((-140, -64, -320), (-40, 125, -310), "A"), box((-70, -32, -310), (-40, -2, -272), "A"),
                  cyl((-7, -52, -103), (-7, -28, -103), 20, "dark"),
                  cyl((-40, -16.5, -290), (4, -16.5, -290), 15, "rubber"),
                  cyl((-130, -16.5, -290), (-70, -16.5, -290), 14, "dark"),
                  box((-120, -26, -310), (-80, -7, -300), "dark"),
                  cyl((-55, 80, -290), (-55, 125, -290), 9, "dark"), box((-65, 85, -310), (-45, 120, -296), "dark")],
         "T_idler": [box((-70, 2, -309), (-40, 32, -272), "A2"), cyl((-55, 32, -290), (-55, 80, -290), 4, "A2"),
                     cyl((-40, 16.5, -290), (4, 16.5, -290), 15, "rubber")]}
    g.update(yoke_geoms("A", 18))
    return g


def tool_B():
    fr = [box((-300, -50, -440), (-200, 50, -430), "dark"), box((-300, -50, -430), (-200, 50, -420), "rubber"),
          box((-280, -20, -420), (-230, 20, -390), "sensor"),
          box((-290, -110, -390), (-80, 40, -370), "alu"), box((-110, -100, -370), (-80, -70, -80), "alu"),
          box((-110, -80, -138), (-42, -70, -68), "alu"), cyl((-42, -80, -103), (-42, -70, -103), 35, "alu"),
          cyl((-7, -70, -103), (-7, -28, -103), 35, "dark"), cyl((-7, -140, -103), (-7, -70, -103), 28, "dark"),
          box((-24, -66, -80), (-12, -54, 20), "alu"), seat(box((-12, -72, 2.5), (12, -48, 16), "rubber")),
          box((-110, -100, -80), (-80, -70, -50), "rubber"), cyl((-80, -85, -65), (-70, -85, -65), 7, "rubber"),
          box((-80, 30, -370), (-75, 50, -34), "sensor"), cyl((-75, 40, -40), (-35, 40, -40), 6, "sensor")]
    g = {"T_frame": fr}
    g.update(yoke_geoms("B", 24))
    return g


# ---------------------------------------------------------------- SPINE cart, lifts, head (world, work pose)
def sq_tube(side, z0, z1, c):
    x0, x1, y0, y1 = -60 - side / 2, -60 + side / 2, -500 - side / 2, -500 + side / 2
    return [box((x0, y0, z0), (x0 + 6, y1, z1), c), box((x1 - 6, y0, z0), (x1, y1, z1), c),
            box((x0, y0, z0), (x1, y0 + 6, z1), c), box((x0, y1 - 6, z0), (x1, y1, z1), c)]


def spine_bodies(lift):
    cart = [box((-240, -550, 180), (240, 550, 330), "cart")]
    for sx in (-1, 1):
        for sy in (-1, 1):
            cart += [cyl((sx * 230, sy * 420, 215.5), (sx * 255, sy * 420, 215.5), 12, "cart"),
                     cyl((sx * 255, sy * 420, 215.5), (sx * 295, sy * 420, 215.5), 70, "rubber")]
    if lift != "S2":
        cart.append(box((40, -200, 330), (220, 300, 520), "elec"))
    b = {"cart": cart}
    if lift == "S1":
        cart += [box((-120, -560, 330), (0, -440, 3330), "lift"), box((-160, -550, 330), (40, -390, 345), "lift")]
        z0, z1 = ZB_WORK - 150, ZB_WORK + 60
        b["S1_top"] = [box((-140, -580, z0), (-121, -420, z1), "head2"), box((1, -580, z0), (20, -420, z1), "head2"),
                       box((-140, -580, z0), (20, -561, z1), "head2"), box((-140, -439, z0), (20, -420, z1), "head2"),
                       box((-100, -420, ZB_WORK), (-20, 480, ZB_WORK + 60), "lift")]
    elif lift == "S2":
        cart.append(box((-250, -550, 330), (250, 550, 380), "lift"))
        zp = ZB_WORK - 340
        top = [box((-250, -550, zp), (250, 550, zp + 40), "lift"), box((-100, -450, ZB_WORK), (-20, 450, ZB_WORK + 60), "lift")]
        top += [box((-100, sy * 400 - 20, zp + 40), (-20, sy * 400 + 20, ZB_WORK), "lift") for sy in (-1, 1)]
        b["S2_top"] = top
        arm = [box((-10, -500, -25), (10, 500, 25), "scissor"),
               cyl((-10, -500, 0), (10, -500, 0), 25, "scissor"), cyl((-10, 500, 0), (10, 500, 0), 25, "scissor")]
        _, arms, pins = s2_layout(ZB_WORK)
        for name, *_ in arms:
            b[name] = arm
        for name, _ in pins:
            b[name] = [cyl((-222, 0, 0), (222, 0, 0), 10, "steel")]
    else:
        cart += sq_tube(160, 330, 1330, "lift")
        for i, (side, bot) in enumerate(zip((140, 120, 100), (345, 360, 375))):
            z0 = bot + (i + 1) * EXT_WORK
            b[f"S3_s{i + 1}"] = sq_tube(side, z0, z0 + 1000, "head" if i == 1 else "lift")
        b["S3_top"] = [box((-140, -580, ZB_WORK - 20), (20, -380, ZB_WORK), "head2"),
                       box((-100, -420, ZB_WORK), (-20, 480, ZB_WORK + 60), "head2")]
    b["head_y"] = [box((-110, -70, 3230), (-10, 70, 3290), "head"), box((-100, -25, 3290), (200, 25, 3320), "steel")]
    b["head_x"] = [box((100, -50, 3320), (200, 50, 3360), "head2")]
    return b


def s2_layout(zb):
    """Scissor pose for a beam height: arms as (body, x centre, pin a (y, z), pin b (y, z)), pins as (body, (y, z))."""
    zp = zb - 340
    h = (zp - 30 - Z_PIN0) / 4
    span = L_ARM * math.cos(math.asin(h / L_ARM))
    arms, pins = [], []
    for i in range(4):
        za = Z_PIN0 + i * h
        for sx, sn in ((-1, "m"), (1, "p")):
            arms.append((f"S2_a{i}{sn}", sx * 180, (Y_FIX, za), (Y_FIX + span, za + h)))
            arms.append((f"S2_b{i}{sn}", sx * 210, (Y_FIX + span, za), (Y_FIX, za + h)))
        pins.append((f"S2_c{i}", (Y_FIX + span / 2, za + h / 2)))
    for i in range(5):
        for e in (0, 1):
            pins.append((f"S2_e{i}{e}", (Y_FIX + e * span, Z_PIN0 + i * h)))
    return zp, arms, pins


# ---------------------------------------------------------------- MJCF
def geom_xml(name, g):
    r, gr, b = COL[g["c"]]
    tail = f'name="{name}" rgba="{r} {gr} {b} 1" contype="0" conaffinity="0"'
    f = lambda v: " ".join(f"{x * 1e-3:.6g}" for x in v)
    if g["t"] == "box":
        return f'<geom type="box" pos="{f((g["p"] + g["q"]) / 2)}" size="{f(np.abs(g["q"] - g["p"]) / 2)}" {tail}/>'
    if g["t"] in ("cylinder", "capsule"):
        return f'<geom type="{g["t"]}" fromto="{f(np.concatenate([g["p"], g["q"]]))}" size="{g["r"] * 1e-3:.6g}" {tail}/>'
    if g["t"] == "sphere":
        return f'<geom type="sphere" pos="{f(g["p"])}" size="{g["r"] * 1e-3:.6g}" {tail}/>'
    import mujoco
    q = np.zeros(4)
    mujoco.mju_mat2Quat(q, np.ascontiguousarray(g["mat"]).flatten())
    return f'<geom type="ellipsoid" pos="{f(g["p"])}" size="{f(g["size"])}" quat="{" ".join(f"{x:.6g}" for x in q)}" {tail}/>'


def build_model(ee, lift):
    static = [geom_xml(n, g) for n, g in context_geoms()]
    bodies = dict(spine_bodies(lift))
    bodies.update(tool_A() if ee == "A" else tool_B())
    for i in range(len(HOOK_YS)):
        bodies[f"hook{i}"] = hook_geoms()
        bodies[f"plant{i}"] = plant_geoms()
    xml_b = []
    for name, gs in bodies.items():
        inner = "\n".join(geom_xml(f"{name}_{j}" + (f"_{g['n']}" if "n" in g else ""), g) for j, g in enumerate(gs))
        xml_b.append(f'<body name="{name}" mocap="true">\n{inner}\n</body>')
    return f"""<mujoco model="hoist_demo_{ee}_{lift}">
  <visual>
    <global offwidth="{2 * W}" offheight="{H}"/>
    <quality shadowsize="4096"/>
    <headlight ambient="0.38 0.38 0.38" diffuse="0.45 0.45 0.45" specular="0.05 0.05 0.05"/>
  </visual>
  <asset>
    <texture type="skybox" builtin="gradient" rgb1="0.94 0.96 0.98" rgb2="0.70 0.76 0.82" width="256" height="256"/>
  </asset>
  <worldbody>
    <light directional="true" pos="2 3 9" dir="-0.25 -0.3 -1" diffuse="0.55 0.55 0.55" castshadow="true"/>
    {chr(10).join(static)}
    {chr(10).join(xml_b)}
  </worldbody>
</mujoco>"""


# ---------------------------------------------------------------- state -> poses
def rot_y(deg):
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def q_y(deg):
    a = math.radians(deg) / 2
    return np.array([math.cos(a), 0.0, math.sin(a), 0.0])


QI = np.array([1.0, 0.0, 0.0, 0.0])


def tool_origin(st):
    return np.array([200.0 + st["xf"], st["cart_y"] + st["yc"], WIRE_Z + st["dz"]])


def hook_pose(st, i):
    """Centre and angle of hook i: in the jaws it follows the rotor, otherwise it hangs on the wire."""
    if st["grip"] == i:
        return tool_origin(st) + (st["rx"], 0, st["rz"]) + C0, st[f"hth{i}"] + st["th"]
    return np.array([WIRE_X + C0[0], st[f"hy{i}"], WIRE_Z + C0[2]]), st[f"hth{i}"]


def exit_point(c, ang):
    """Twine leaves whichever hook end is lower; blended while the two ends pass level."""
    e = rot_y(ang) @ EXIT
    p1, p2 = c + e, c - e
    w = min(1.0, max(0.0, 0.5 + (p2[2] - p1[2]) / 30.0))
    return w * p1 + (1 - w) * p2


def nip(st):
    return tool_origin(st) + NIP + (st["rx"], 0, st["rz"])


def rope(p, q, length, bulge):
    """Twine between two points: straight when taut, else a sagging curve of the given length."""
    chord = np.linalg.norm(q - p)
    if length <= chord + 0.5:
        return [p, q]
    m = (p + q) / 2
    t = np.linspace(0, 1, 17)[:, None]

    def curve(d):
        c = m + bulge * 0.35 * d + np.array([0, 0, -d])
        return (1 - t) ** 2 * p + 2 * (1 - t) * t * c + t ** 2 * q

    lo, hi = 0.0, length
    for _ in range(40):
        mid = (lo + hi) / 2
        if np.linalg.norm(np.diff(curve(mid), axis=0), axis=1).sum() < length:
            lo = mid
        else:
            hi = mid
    return list(curve(lo))


def scene_state(st, ee, lift):
    """Mocap poses (mm, quat) for every body, plus twine polylines."""
    P = {}
    cy, dz, yc, xf = st["cart_y"], st["dz"], st["yc"], st["xf"]
    zb = ZB_WORK + dz
    P["cart"] = (np.array([0, cy, 0.0]), QI)
    if lift == "S1":
        P["S1_top"] = (np.array([0, cy, dz]), QI)
    elif lift == "S2":
        zp, arms, pins = s2_layout(zb)
        P["S2_top"] = (np.array([0, cy, zp - (ZB_WORK - 340)]), QI)
        for name, xc, a, b in arms:
            phi = math.atan2(b[1] - a[1], b[0] - a[0])
            P[name] = (np.array([xc, cy + (a[0] + b[0]) / 2, (a[1] + b[1]) / 2]),
                       np.array([math.cos(phi / 2), math.sin(phi / 2), 0, 0]))
        for name, (y, z) in pins:
            P[name] = (np.array([0, cy + y, z]), QI)
    else:
        ext = (zb - 20 - 1375) / 3
        for i in range(3):
            P[f"S3_s{i + 1}"] = (np.array([0, cy, (i + 1) * (ext - EXT_WORK)]), QI)
        P["S3_top"] = (np.array([0, cy, dz]), QI)
    P["head_y"] = (np.array([0, cy + yc, dz]), QI)
    P["head_x"] = (np.array([xf - 200, cy + yc, dz]), QI)
    T = tool_origin(st)
    P["T_frame"] = (T, QI)
    if ee == "A":
        P["T_rx"] = (T + (st["rx"], 0, 0), QI)
        P["T_rz"] = (T + (st["rx"], 0, st["rz"]), QI)
        P["T_idler"] = (T + (st["rx"], st["idler"], st["rz"]), QI)
    yk = T + (st["rx"], 0, st["rz"]) + C0
    P["T_yoke"] = (yk, q_y(st["th"]))
    P["T_jawL"] = (yk + rot_y(st["th"]) @ (-st["jaw"], 0, 0), q_y(st["th"]))
    P["T_jawR"] = (yk + rot_y(st["th"]) @ (st["jaw"], 0, 0), q_y(st["th"]))
    ropes = []
    for i in range(len(HOOK_YS)):
        c, ang = hook_pose(st, i)
        P[f"hook{i}"] = (c, q_y(ang))
        E = exit_point(c, ang)
        if st["pinch"] == i:
            N = nip(st)
            top = N - (0, 0, st["s2"])
            ropes.append(rope(E, N, st["s1"], np.array([1.0, 0, 0])))
            ropes.append([N, top])
        else:
            top = np.array([c[0] + EXIT[0], c[1], E[2] - st[f"L{i}"]])
            ropes.append([E, top])
        P[f"plant{i}"] = (top - (0, 0, 25), QI)
    return P, ropes


# ---------------------------------------------------------------- choreography
def minjerk(s):
    return 10 * s ** 3 - 15 * s ** 4 + 6 * s ** 5


class Timeline:
    def __init__(self, st):
        self.st = dict(st)
        self.frames = []

    def move(self, dur, label, load=None, **tg):
        """Min-jerk move of the keys in tg; load (a LOADS key) says what carries the plant from now on."""
        n = max(1, round(dur * FPS))
        if load:
            self.st["load"] = load
        a = {k: self.st[k] for k in tg}
        for i in range(1, n + 1):
            s = minjerk(i / n)
            for k, v in tg.items():
                self.st[k] = a[k] + (v - a[k]) * s
            self.frames.append((dict(self.st), label))


def pinch(tl, g):
    st = tl.st
    c, ang = hook_pose(st, g)
    st["s1"] = float(np.linalg.norm(exit_point(c, ang) - nip(st)))
    st["s2"] = st[f"L{g}"] - st["s1"]
    st["pinch"] = g


def unpinch(tl):
    st = tl.st
    st[f"L{st['pinch']}"] = st["s1"] + st["s2"]
    st["pinch"] = -1


def release(tl, g):
    st = tl.st
    st[f"hth{g}"] = (st[f"hth{g}"] + st["th"]) % 360
    st[f"hy{g}"] = tool_origin(st)[1]
    st["grip"] = -1


def cycle_A(tl, g, over_support):
    st = tl.st
    tl.move(1.2, "Approach above the wire, 25 short of the hook", xf=200.0)
    tl.move(0.8, "Seat the saddle rollers on the wire", dz=0.0)
    tl.move(0.9, "Roll 25: open fork slides onto the hook", yc=st["yc"] + SLIDE)
    tl.move(0.6, "Capstan idler springs shut on the twine", idler=0.0)
    pinch(tl, g)
    tl.move(0.7, "Capstan pulls 5 mm: plant weight off the hook", load="capstan",
            s1=st["s1"] + TAKE_UP, s2=st["s2"] - TAKE_UP)
    st["grip"] = g
    tl.move(0.5, "Fingers close on the hook", jaw=0.0)
    tl.move(0.6, "Rotor lifts the hook off the wire", rz=30.0)
    tl.move(0.8, "Rotor backs off: room to flip", rx=-130.0)
    tl.move(1.8, "Flip 180: hook lets out twine, capstan holds the plant", th=180.0, s1=st["s1"] + K_RELEASE)
    if not over_support:
        tl.move(1.8, "Tool rolls 300 along the wire on its saddle", yc=st["yc"] + 300.0)
    else:
        tl.move(0.8, "Support ahead: lift takes tool and plant off the wire", load="lift", dz=60.0)
        tl.move(0.8, "Back off past the hanger", xf=120.0)
        tl.move(1.8, "Carry hook and plant past the support", yc=st["yc"] + 300.0)
        tl.move(0.8, "Return over the wire", xf=200.0)
        tl.move(0.8, "Seat the saddle again", dz=0.0)
    tl.move(0.8, "Rotor brings the hook back over the wire", load="capstan", rx=0.0)
    tl.move(0.6, "Hook seats on the wire", rz=0.0)
    tl.move(0.5, "Fingers open", jaw=JAW_OPEN)
    release(tl, g)
    tl.move(1.8, "Capstan pays out: plant hangs on the hook again, lower",
            s1=st["s1"] - K_RELEASE - TAKE_UP, s2=st["s2"] + K_RELEASE + TAKE_UP)
    unpinch(tl)
    tl.move(0.6, "Idler opens", load="wire", idler=IDLER_OPEN)
    tl.move(0.9, "Roll back 25: fork slides off the hook", yc=st["yc"] - SLIDE)
    tl.move(0.7, "Undock: up", dz=40.0)
    tl.move(0.9, "Undock: back", xf=0.0)


def cycle_B(tl, g, over_support):
    st = tl.st
    tl.move(1.2, "Approach just above the wire, 25 short of the hook", xf=200.0)
    tl.move(0.9, "Open fork slides 25 onto the hook", yc=st["yc"] + SLIDE)
    tl.move(0.6, "Rest pad on the wire: height datum", dz=0.0)
    st["grip"] = g
    tl.move(0.5, "Fingers close on the hook", jaw=0.0)
    tl.move(0.7, "Lift raises the hook off the wire; load cell weighs the plant", load="lift", dz=30.0)
    tl.move(0.9, "Head backs off: room to flip", xf=70.0)
    tl.move(2.4, "Flip 180 under brake: plant drops by the twine let out", th=180.0, **{f"L{g}": st[f"L{g}"] + K_RELEASE})
    tl.move(1.8, "Head carries hook and plant past the support, clear of the wire" if over_support
            else "Head carries hook and plant 300 along the row", yc=st["yc"] + 300.0)
    tl.move(0.9, "Back over the wire", xf=200.0)
    tl.move(0.7, "Hook seats on the wire", dz=0.0)
    tl.move(0.5, "Fingers open", load="wire", jaw=JAW_OPEN)
    release(tl, g)
    tl.move(0.6, "Undock: up", dz=10.0)
    tl.move(0.9, "Fork slides 25 off the hook", yc=st["yc"] - SLIDE)
    tl.move(0.9, "Undock: back", xf=0.0)


def build_timeline(ee, lift):
    st = dict(cart_y=1200.0, dz=LOW_DZ[lift], yc=-150.0 - SLIDE, xf=0.0, rx=0.0, rz=0.0, th=0.0, jaw=JAW_OPEN,
              idler=IDLER_OPEN, s1=0.0, s2=0.0, grip=-1, pinch=-1, load="wire")
    for i, y in enumerate(HOOK_YS):
        st.update({f"hy{i}": y, f"hth{i}": 0.0, f"L{i}": FREE_TWINE})
    tl = Timeline(st)
    tl.move(1.0, "Start: lift down, tool parked")
    tl.move(2.0, "Cart rolls in on the heating pipes", cart_y=150.0)
    tl.move(3.0, "Lift raises the head to the wire", dz=40.0 if ee == "A" else 10.0)
    cycle = cycle_A if ee == "A" else cycle_B
    cycle(tl, HOOK_YS.index(0.0), False)       # Y 0 -> +300, open wire
    tl.move(2.0, "Next plant: cart moves 350, carriage and rotor reset", cart_y=-200.0, yc=-150.0 - SLIDE, th=0.0)
    cycle(tl, HOOK_YS.index(-350.0), True)     # Y -350 -> -50, across the support at Y -175
    tl.move(1.5, "Done: two plants lower, hooks 300 further along")
    return tl.frames


# ---------------------------------------------------------------- checks
# ⚠️ Not mj_geomDistance: in mujoco 3.8.1 it returned 0 for geoms 42 mm apart whenever they were within
# distmax (rotated box pairs, and convex meshes with parallel faces). Gaps here come from a small convex
# program: every geom is c + L z with z inside bounds, plus a radius limit on part of z for round shapes.
CAP = 30.0   # mm: gaps beyond this print as ">= 30"
CHECKS = [  # name, min clearance (mm)
    ("tool and carried hook vs wire support", 5.0),
    ("flipping hook vs tool and wire (not its fork)", 2.0),
    ("tool vs other hooks and plants", 5.0),
    ("tool vs crop wire (saddle rollers, rest pad excepted)", 2.0),
]


def convex(model, data, g):
    """Geom g in mm as (c, L, half-widths of z, (slice of z, radius) or None)."""
    import mujoco
    T = mujoco.mjtGeom
    t, s = model.geom_type[g], model.geom_size[g] * 1e3
    c, R = data.geom_xpos[g] * 1e3, data.geom_xmat[g].reshape(3, 3)
    if t == T.mjGEOM_BOX:
        return c, R, s.copy(), None
    if t == T.mjGEOM_SPHERE:
        return c, np.eye(3), np.full(3, s[0]), (slice(0, 3), s[0])
    if t == T.mjGEOM_CAPSULE:      # point on the axis plus a ball
        return c, np.column_stack([R[:, 2], np.eye(3)]), np.array([s[1], s[0], s[0], s[0]]), (slice(1, 4), s[0])
    if t == T.mjGEOM_CYLINDER:     # point on the axis plus a disc
        return c, R[:, [2, 0, 1]], np.array([s[1], s[0], s[0]]), (slice(1, 3), s[0])
    if t == T.mjGEOM_ELLIPSOID:
        return c, R * s, np.ones(3), (slice(0, 3), 1.0)
    raise ValueError(f"geom {model.geom(g).name}: no convex form for type {t}")


def support(S, U):
    """Largest u . x over shape S, for each unit row u of U."""
    c, L, h, rnd = S
    W = U @ L
    if rnd is None:
        return U @ c + np.abs(W) @ h
    flat = np.ones(len(h), bool)
    flat[rnd[0]] = False
    return U @ c + np.abs(W[:, flat]) @ h[flat] + rnd[1] * np.linalg.norm(W[:, rnd[0]], axis=1)


def gap(A, B):
    """Smallest distance (mm) between convex shapes A and B; 0 when they overlap, CAP when far.
    Returns the separating-plane bound at the solver's answer, so a poor solve reads low, never high."""
    from scipy.optimize import minimize
    sep = lambda U: -support(A, -U) - support(B, U)    # gap along u, measured from B towards A
    if np.linalg.norm(np.maximum(np.abs(A[0] - B[0]) - np.abs(A[1]) @ A[2] - np.abs(B[1]) @ B[2], 0)) >= CAP:
        return CAP                                       # bounding boxes already far apart
    d = A[0] - B[0]
    U = np.vstack([np.eye(3), -np.eye(3)] + ([d[None] / np.linalg.norm(d)] if np.linalg.norm(d) > 1e-9 else []))
    if sep(U).max() >= CAP:
        return CAP
    M, r, na = np.hstack([A[1], -B[1]]), A[0] - B[0], len(A[2])
    cons = []
    for rnd, off in ((A[3], 0), (B[3], na)):
        if rnd:
            sl = slice(rnd[0].start + off, rnd[0].stop + off)

            def jac(z, sl=sl):
                out = np.zeros_like(z)
                out[sl] = -2 * z[sl]
                return out
            cons.append({"type": "ineq", "fun": lambda z, sl=sl, rad=rnd[1]: rad ** 2 - z[sl] @ z[sl], "jac": jac})
    bounds = [(-w, w) for w in np.concatenate([A[2], B[2]])]
    res = minimize(lambda z: ((r + M @ z) @ (r + M @ z), 2 * M.T @ (r + M @ z)), np.zeros(len(bounds)), jac=True,
                   method="SLSQP", bounds=bounds, constraints=cons, options={"ftol": 1e-12, "maxiter": 300})
    v = r + M @ res.x
    n = np.linalg.norm(v)
    return 0.0 if n < 1e-6 else float(np.clip(sep(v[None] / n)[0], 0.0, min(n, CAP)))


def run_checks(model, data, frames, apply):
    by_body = {}
    for gid in range(model.ngeom):
        by_body.setdefault(model.body(model.geom_bodyid[gid]).name, []).append(gid)
    named = {model.geom(g).name: g for g in range(model.ngeom)}
    tool = [g for b, gs in by_body.items() if b.startswith("T_") for g in gs]
    unseated = [g for g in tool if not model.geom(g).name.endswith("_seat")]
    stage = [g for b in ("T_frame", "T_rx", "T_rz", "T_idler") for g in by_body.get(b, [])]
    support_geoms = [named["support+Xclamp"], named["support+Xrod"]]
    wire = [named["wire+X"]]

    def min_gap(A, B):
        S = {g: convex(model, data, g) for g in set(A) | set(B)}
        best = (CAP, "")
        for a in A:
            for b in B:
                d = gap(S[a], S[b])
                if d < best[0]:
                    best = (d, f"{model.geom(a).name} vs {model.geom(b).name}")
        return best

    worst = {name: (CAP, "-", "") for name, _ in CHECKS}
    for st, label in frames[::2]:
        apply(st)
        g = int(st["grip"])
        carried = by_body[f"hook{g}"] if g >= 0 else []
        res = [min_gap(tool + carried, support_geoms), None]
        if g >= 0 and label.startswith("Flip"):
            res[1] = min_gap(carried, stage + wire)
        others = [x for i in range(len(HOOK_YS)) if i != g for x in by_body[f"hook{i}"]]
        others += [x for i in range(len(HOOK_YS)) for x in by_body[f"plant{i}"]]
        res.append(min_gap(tool, others))
        res.append(min_gap(unseated, wire))
        for (name, _), r in zip(CHECKS, res):
            if r is not None and r[0] < worst[name][0]:
                worst[name] = (r[0], label, r[1])
    ok = True
    for name, need in CHECKS:
        d, label, pair = worst[name]
        passed = d >= need
        ok &= passed
        shown = f">= {CAP:.0f}" if d >= CAP else f"{d:5.1f}"
        print(f"{'PASS' if passed else 'FAIL'}  {name:54s} need >= {need:1.0f}  min {shown} mm  at: {label}"
              + (f"  [{pair}]" if pair else ""))
    return ok


# ---------------------------------------------------------------- render
def add_rope(scn, pts, rgba):
    import mujoco
    for p, q in zip(pts[:-1], pts[1:]):
        if scn.ngeom >= scn.maxgeom:
            return
        g = scn.geoms[scn.ngeom]
        mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_CAPSULE, np.zeros(3), np.zeros(3), np.zeros(9), rgba)
        mujoco.mjv_connector(g, mujoco.mjtGeom.mjGEOM_CAPSULE, 1.2e-3, np.asarray(p) * 1e-3, np.asarray(q) * 1e-3)
        scn.ngeom += 1


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--ee", choices=["A", "B"], required=True)
    ap.add_argument("--lift", choices=["S1", "S2", "S3"], required=True)
    ap.add_argument("--windowed", action="store_true", help="play in a MuJoCo viewer window instead of rendering")
    ap.add_argument("--checks-only", action="store_true", help="print the clearance checks and stop")
    ap.add_argument("--out", type=Path, help="video path (default runs/hoist_demo/hoist_demo_<ee>_<lift>.mp4)")
    args = ap.parse_args()
    if not args.windowed:
        os.environ.setdefault("MUJOCO_GL", "egl")   # ⚠️ only headless: EGL breaks the viewer window
    import mujoco

    model = mujoco.MjModel.from_xml_string(build_model(args.ee, args.lift))
    data = mujoco.MjData(model)
    mocap = {model.body(b).name: model.body(b).mocapid[0] for b in range(model.nbody) if model.body(b).mocapid[0] >= 0}
    frames = build_timeline(args.ee, args.lift)
    last_ropes = []

    def apply(st):
        P, ropes = scene_state(st, args.ee, args.lift)
        for name, (pos, quat) in P.items():
            data.mocap_pos[mocap[name]] = pos * 1e-3
            data.mocap_quat[mocap[name]] = quat
        mujoco.mj_kinematics(model, data)
        last_ropes[:] = ropes

    title = f"{NAMES[args.ee]} on {NAMES[args.lift]}"
    print(f"{title}: {len(frames)} frames, {len(frames) / FPS:.1f} s")
    ok = run_checks(model, data, frames, apply)
    if args.checks_only:
        sys.exit(0 if ok else 1)
    twine = np.array([*COL["twine"], 1.0], np.float32)

    if args.windowed:
        import mujoco.viewer
        with mujoco.viewer.launch_passive(model, data) as v:
            v.cam.lookat[:] = (0.2, 0.3, 2.3)
            v.cam.distance, v.cam.azimuth, v.cam.elevation = 6.5, -52, -16
            shown = None
            for st, label in frames:
                if not v.is_running():
                    break
                t0 = time.time()
                apply(st)
                with v.lock():
                    v.user_scn.ngeom = 0
                    for r in last_ropes:
                        add_rope(v.user_scn, r, twine)
                v.sync()
                if label != shown:
                    print(f"  {label}  [{LOADS[st['load']][0]}]")
                    shown = label
                time.sleep(max(0.0, 1 / FPS - (time.time() - t0)))
            while v.is_running():
                time.sleep(0.1)
        return

    import imageio.v2 as imageio
    from PIL import Image, ImageDraw, ImageFont
    out = args.out or Path(__file__).resolve().parents[1] / "runs" / "hoist_demo" / f"hoist_demo_{args.ee}_{args.lift}.mp4"
    out.parent.mkdir(parents=True, exist_ok=True)
    font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 19)
    bold = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 22)
    small = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 15)
    renderer = mujoco.Renderer(model, H, W)
    wide = mujoco.MjvCamera()
    wide.lookat[:] = (0.2, 0.3, 2.3)
    wide.distance, wide.azimuth, wide.elevation = 6.5, -52, -16
    close = mujoco.MjvCamera()
    close.distance, close.azimuth, close.elevation = 1.0, -120, -12   # from outboard: wire side of the tool
    look = None
    writer = imageio.get_writer(out, fps=FPS, codec="libx264", quality=8, pixelformat="yuv420p", macro_block_size=8)
    steps = []
    for _, label in frames:
        if not steps or steps[-1] != label:
            steps.append(label)
    step, shown = 0, None
    for k, (st, label) in enumerate(frames):
        if label != shown:              # labels repeat in the second cycle, so count changes, don't look them up
            step, shown = step + 1, label
        apply(st)
        target = tool_origin(st) * 1e-3 + (-0.08, 0, -0.17)
        look = target if look is None else look + (target - look) * (1 - math.exp(-1 / (FPS * 0.15)))
        close.lookat[:] = look
        views = []
        for cam in (wide, close):
            renderer.update_scene(data, cam)
            for r in last_ropes:
                add_rope(renderer.scene, r, twine)
            views.append(renderer.render())
        img = Image.fromarray(np.hstack(views))
        d = ImageDraw.Draw(img, "RGBA")
        d.rectangle((0, 0, 2 * W, 64), fill=(15, 20, 28, 190))
        d.text((14, 8), title, font=bold, fill=(255, 255, 255))
        d.text((14, 36), "kinematic sketch: scripted motion, unmeasured hook, twine and plant", font=small, fill=(200, 210, 220))
        d.text((W + 14, 8), f"step {step}/{len(steps)}", font=small, fill=(200, 210, 220))
        d.text((W + 14, 30), label, font=font, fill=(255, 255, 255))
        text, rgb = LOADS[st["load"]]
        d.rectangle((W, H - 40, 2 * W, H), fill=(*rgb, 225))
        d.text((W + 14, H - 31), text, font=font, fill=(255, 255, 255))
        d.line((W, 64, W, H), fill=(15, 20, 28, 255), width=2)
        writer.append_data(np.asarray(img))
        if k % 300 == 0:
            print(f"  frame {k}/{len(frames)}")
    writer.close()
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
