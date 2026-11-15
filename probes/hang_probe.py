"""Hang probe: can a light steel hook rest on a crop wire under a full plant in MuJoCo, and
does a length-limited spatial tendon behave like twine that pays out when its range grows?

    python probes/hang_probe.py              # variant table, then PASS/FAIL on the default
    python probes/hang_probe.py --windowed   # default variant in the viewer, payout on a loop

What it showed (2026-10-27, mujoco 3.8.1):
- ⚠️ Thin geoms at the default solref: the hook tunnels through the wire and the plant
  free-falls. A smaller timestep or fatter geoms alone don't fix it.
- Fatter collision geoms with a stiff wire solref hold. So does a hook ten times too heavy.
- Raising tendon_range at runtime works as payout. A ramp keeps the twine force near m*g; an
  instant step spikes it several times over, which is the jerk that knocks fruit off.
- The contact still sinks a few mm, so don't read hook-seat geometry from this sim.
- ⚠️ Payout here is commanded. Nothing in this probe simulates twine unwinding off a hook.
"""
import argparse
import sys
import time

import mujoco
import numpy as np

H = 2.0          # wire height (m). unverified: arbitrary, real height unmeasured
SHANK = 0.20     # hook length (m). Royal Brinkman sells 180/220/260 mm winding hooks
L0 = 1.0         # hanging twine before payout (m). unverified guess
M_PLANT = 5.0    # plant weight on the twine (kg). WUR Rapport 732: 4-6 kg
PAYOUT = 0.20    # one lowering step (m). Arugga US20240334886A1: ~20 cm per half-turn of
                 # its own hook. unverified for the standard winding hook
RAMP_S = 2.0     # payout duration (s). unverified guess
SETTLE_S = 2.0   # let the plant ring down before measuring (s)
RUN_S = 4.0      # payout + hold (s)
G = 9.81

# Pass thresholds for the default variant.
STATIC_TOL = 0.05      # static twine force within 5 % of m*g
RAMP_PEAK_MAX = 1.5    # ramped peak below 1.5 m*g. unverified: what a truss tolerates is unknown
SINK_MAX = 0.006       # contact sinks less than 6 mm (m)
DROP_TOL = 0.01        # plant comes down by PAYOUT within 1 cm (m)
SEAT_TOL = 0.01        # hook moves less than 1 cm vertically while loaded (m)

# (label, wire radius, hook radius, hook bend radius, wire solref, timestep, hook mass scale)
# Radii are collision sizes, not the real gauges (unmeasured).
VARIANTS = [
    ("thin, default solref",       0.0015, 0.002, 0.008, None,      0.002,  1),
    ("thin, dt 0.5 ms",            0.0015, 0.002, 0.008, None,      0.0005, 1),
    ("thin, hook x10 mass",        0.0015, 0.002, 0.008, None,      0.002,  10),
    ("fat 4/4 mm, default solref", 0.004,  0.004, 0.012, None,      0.002,  1),
    ("fat 6/5 mm, default solref", 0.006,  0.005, 0.016, None,      0.002,  1),
    ("thin, stiff solref",         0.0015, 0.002, 0.008, "0.004 1", 0.002,  1),
    ("fat 4/4 mm, stiff solref",   0.004,  0.004, 0.012, "0.004 1", 0.002,  1),
    ("fat 4/4 mm, stiff, dt 1 ms", 0.004,  0.004, 0.012, "0.004 1", 0.001,  1),
]
DEFAULT = 6  # the variant the later sim builds on


def hook_geoms(r_h, rho, mass_scale):
    """Hook arc over the wire (in the y-z plane, wire along x) plus a straight shank down.
    Single-ended: the real winding hook is double-ended, which this probe doesn't need."""
    th = np.radians(np.linspace(180, -45, 7))
    pts = [(rho * np.cos(t), rho * np.sin(t)) for t in th]
    g = []
    for i, ((y0, z0), (y1, z1)) in enumerate(zip(pts[:-1], pts[1:])):
        g.append(f'<geom name="arc{i}" type="capsule" size="{r_h}" '
                 f'fromto="0 {y0:.5f} {z0:.5f} 0 {y1:.5f} {z1:.5f}" mass="{0.004 * mass_scale}"/>')
    g.append(f'<geom name="shank" type="capsule" size="{r_h}" '
             f'fromto="0 {-rho} 0 0 {-rho} {-SHANK}" mass="{0.02 * mass_scale}"/>')
    return "\n      ".join(g)


def build_xml(r_w, r_h, rho, solref, dt, mass_scale):
    hook_z = H + r_w + r_h + 0.0005 - rho  # arc top just clear of the wire
    sol = f'solref="{solref}"' if solref else ""
    return f"""<mujoco>
  <option timestep="{dt}" integrator="implicitfast" cone="elliptic" impratio="10"/>
  <visual><global offwidth="1280" offheight="720"/></visual>
  <worldbody>
    <light pos="0 -1 4" dir="0 0.3 -1"/>
    <geom name="floor" type="plane" size="2 2 0.1" rgba="0.3 0.3 0.3 1" contype="0" conaffinity="0"/>
    <geom name="wire" type="capsule" size="{r_w}" fromto="-1 0 {H} 1 0 {H}"
          friction="0.4 0.005 0.0001" rgba="0.7 0.7 0.7 1" {sol}/>
    <body name="hook" pos="0 0 {hook_z}">
      <freejoint/>
      {hook_geoms(r_h, rho, mass_scale)}
      <site name="hook_bot" pos="0 {-rho} {-SHANK}" size="0.003"/>
    </body>
    <body name="plant" pos="0 {-rho} {hook_z - SHANK - L0}">
      <freejoint/>
      <geom type="box" size="0.03 0.03 0.03" mass="{M_PLANT}" rgba="0.2 0.6 0.2 1"
            contype="0" conaffinity="0"/>
      <site name="plant_top" pos="0 0 0" size="0.003"/>
    </body>
  </worldbody>
  <tendon>
    <spatial name="twine" limited="true" range="0 {L0}" width="0.001" rgba="0.9 0.8 0.5 1">
      <site site="hook_bot"/>
      <site site="plant_top"/>
    </spatial>
  </tendon>
</mujoco>"""


def twine_force(m, d, tid):
    """Tension in the twine = force on its tendon-limit constraint rows."""
    rows = (d.efc_type == mujoco.mjtConstraint.mjCNSTR_LIMIT_TENDON) & (d.efc_id == tid)
    return float(abs(d.efc_force[rows].sum()))


def ramp(t):
    return min(t / RAMP_S, 1.0) * PAYOUT


def step_now(t):
    return PAYOUT


def trial(xml, payout):
    m = mujoco.MjModel.from_xml_string(xml)
    d = mujoco.MjData(m)
    tid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_TENDON, "twine")
    hook = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "hook")
    plant = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "plant")
    mujoco.mj_forward(m, d)
    hook_start = d.xpos[hook, 2]
    for _ in range(int(SETTLE_S / m.opt.timestep)):
        mujoco.mj_step(m, d)
    hook0, plant0 = d.xpos[hook, 2], d.xpos[plant, 2]
    static = twine_force(m, d, tid)
    peak, sink = 0.0, 0.0
    for k in range(int(RUN_S / m.opt.timestep)):
        m.tendon_range[tid, 1] = L0 + payout(k * m.opt.timestep)
        mujoco.mj_step(m, d)
        peak = max(peak, twine_force(m, d, tid))
        if d.ncon:
            sink = min(sink, min(c.dist for c in d.contact[:d.ncon]))
    hook1 = d.xpos[hook, 2]
    seated = (np.isfinite(hook1) and abs(hook0 - hook_start) < SEAT_TOL
              and abs(hook1 - hook0) < SEAT_TOL)
    return dict(seated=bool(seated), static=static, peak=peak,
                drop=plant0 - d.xpos[plant, 2], sink=max(0.0, -sink))


def table():
    mg = M_PLANT * G
    print(f"plant {M_PLANT} kg (m*g = {mg:.1f} N), payout {PAYOUT * 100:.0f} cm\n")
    print(f"{'variant':30s} {'seated':>6s} {'static':>8s} {'ramp pk':>8s} {'drop':>7s} "
          f"{'sink':>7s} {'step pk':>8s}")
    results = []
    for i, v in enumerate(VARIANTS):
        xml = build_xml(*v[1:])
        r = trial(xml, ramp)
        r["step_peak"] = trial(xml, step_now)["peak"]
        results.append(r)
        mark = "  <- default" if i == DEFAULT else ""
        print(f"{v[0]:30s} {str(r['seated']):>6s} {r['static']:7.1f}N {r['peak']:7.1f}N "
              f"{r['drop']:+6.3f}m {r['sink'] * 1000:5.1f}mm {r['step_peak']:7.1f}N{mark}")
    return results[DEFAULT]


def check(r):
    mg = M_PLANT * G
    tests = [
        ("hook stays seated on the wire", r["seated"]),
        (f"static twine force {r['static']:.1f} N within {STATIC_TOL:.0%} of m*g",
         abs(r["static"] - mg) < STATIC_TOL * mg),
        (f"ramped peak {r['peak']:.1f} N below {RAMP_PEAK_MAX} m*g", r["peak"] < RAMP_PEAK_MAX * mg),
        (f"contact sink {r['sink'] * 1000:.1f} mm below {SINK_MAX * 1000:.0f} mm", r["sink"] < SINK_MAX),
        (f"plant came down {r['drop'] * 100:.1f} cm for a {PAYOUT * 100:.0f} cm payout",
         abs(r["drop"] - PAYOUT) < DROP_TOL),
        (f"instant payout spikes above the ramp ({r['step_peak']:.0f} N vs {r['peak']:.0f} N)",
         r["step_peak"] > r["peak"]),
    ]
    print()
    for name, ok in tests:
        print(f"{'PASS' if ok else 'FAIL'}  {name}")
    return all(ok for _, ok in tests)


def windowed():
    import mujoco.viewer
    m = mujoco.MjModel.from_xml_string(build_xml(*VARIANTS[DEFAULT][1:]))
    d = mujoco.MjData(m)
    tid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_TENDON, "twine")
    cycle = SETTLE_S + RAMP_S + 2.0  # settle, pay out, hold, then start over
    with mujoco.viewer.launch_passive(m, d) as v:
        v.cam.lookat[:] = (0, 0, H - 0.6)
        v.cam.distance, v.cam.elevation, v.cam.azimuth = 2.5, -10, 60
        while v.is_running():
            mujoco.mj_resetData(m, d)
            t0 = time.time()
            while v.is_running() and d.time < cycle:
                t = d.time - SETTLE_S
                m.tendon_range[tid, 1] = L0 + (ramp(t) if t > 0 else 0.0)
                mujoco.mj_step(m, d)
                v.sync()
                lag = d.time - (time.time() - t0)
                if lag > 0:
                    time.sleep(lag)


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--windowed", action="store_true", help="watch the default variant")
    if ap.parse_args().windowed:
        windowed()
        sys.exit(0)
    sys.exit(0 if check(table()) else 1)
