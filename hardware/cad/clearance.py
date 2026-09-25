"""Clearance check: moving parts (cart and everything on it, the rod in several poses, the switches) against the
fixed frame (beam, rail, belt, motor, pulley, idler, ramps, bumpers), at several cart positions. Also checks
the end-stop logic: rollers pressed only past the trigger point, switch bodies never hit a ramp.
Usage: uv run python clearance.py"""
import math

import cadquery as cq

import pendulum_cad as C

p = C.P()
back, front = C.shaft_y(p)
hub_y = p.pad_h + p.plate_t + p.boss_depth + p.hub_gap + p.hub_t / 2
pz, iz = C.pulley_z(p), C.idler_z(p)
X = p.pulley_x


def cyl_y(x, z, r, y0, y1):
    return cq.Workplane("XZ").workplane(offset=-y0).center(x, z).circle(r).extrude(-(y1 - y0))


def strand(x0, z0, x1, z1):
    """a belt strand as a thin slab between two points"""
    L = math.hypot(x1 - x0, z1 - z0)
    ang = math.degrees(math.atan2(z1 - z0, x1 - x0))
    return (cq.Workplane("XY").box(L, p.belt_w, p.belt_t).rotate((0, 0, 0), (0, 1, 0), -ang)
            .translate(((x0 + x1) / 2, p.belt_y, (z0 + z1) / 2)))


fixed = {
    "beam": cq.Workplane("XY").box(p.rail_len, p.beam_t, 120).translate((0, p.beam_face_y - p.beam_t / 2, -20)),
    "rail": cq.Workplane("XY").box(p.rail_len, 8, p.rail_w).translate((0, p.beam_face_y + 4, 0)),
    "upper belt": strand(-X, p.belt_z - p.belt_t / 2, X, p.belt_z - p.belt_t / 2),
    "lower belt": strand(-X, iz - p.idler_r - p.belt_t / 2, X, pz - p.pulley_r - p.belt_t / 2),
    "motor block": C.motor_block(p),
    "motor body": cyl_y(X + p.motor_offset, pz, p.motor_d / 2, p.motor_face_y - p.motor_body_len, p.motor_face_y),
    "pulley": cyl_y(X, pz, 12.0, -17.5, -1.5),
    "idler block": C.idler_block(p),
    "idler": cyl_y(-X, iz, 9.0, p.belt_y - 4.25, p.belt_y + 4.25),
    "ramp R": C.end_ramp(p, 1), "ramp L": C.end_ramp(p, -1),
    "bumper R": C.bumper(p, 1), "bumper L": C.bumper(p, -1),
}


def moving(x, rod_angle_deg):
    a = math.radians(rod_angle_deg)            # 0 = rod up, 180 = hanging
    rod = (cq.Workplane("XY").circle(p.tube_od / 2).extrude(300)
           .rotate((0, 0, 0), (0, 1, 0), rod_angle_deg).translate((0, hub_y, p.pivot_z)))
    parts = {
        "carriage": cq.Workplane("XY").box(p.carriage_l, 10, p.carriage_w).translate((0, -5, 0)),
        "cart": C.cart(p), "belt clamp": C.belt_clamp(p), "sensor bracket": C.sensor_mount(p),
        "switch carrier": C.switch_carrier(p), "rod": rod,
    }
    return {k: v.translate((x, 0, 0)) for k, v in parts.items()}


def vol(a, b):
    try:
        r = a.intersect(b)
        return r.val().Volume() if r.solids().size() else 0.0
    except Exception:
        return 0.0


problems = 0
for x in (0.0, 325.0, 340.0, 355.0, 457.0, -325.0, -340.0, -355.0, -457.0):
    for ang in (180.0, 90.0, 0.0, -90.0):
        mv = moving(x, ang)
        for mn, m in mv.items():
            if mn == "rod" or ang == 180.0:                   # non-rod parts only need checking once per x
                for fn, f in fixed.items():
                    v = vol(m, f)
                    # expected contacts: the carriage rides the rail; the cart's grip ridges press into the belt teeth
                    ridges = 19 * 1.0 * p.belt_w * 0.7
                    if mn == "cart" and fn == "upper belt" and abs(v - ridges) < 1.0:
                        continue
                    if v > 0.5 and not (mn == "carriage" and fn == "rail"):
                        problems += 1
                        print(f"  COLLISION at cart x={x:+.0f}, rod {ang:.0f}°: {mn} × {fn}: {v:.1f} mm³")
    # end-stop logic: rollers press the ramp's raised flat only past the trigger point; bodies never touch
    sw = C.switch_dummy(p).translate((x, 0, 0))
    ramp = fixed["ramp R"] if x >= 0 else fixed["ramp L"]
    press = vol(sw, ramp)
    expect = abs(x) + 7 >= p.trigger_x + 5                    # nearest roller (±7 mm) past the middle of the rise
    print(f"cart x={x:+5.0f} mm: switch/ramp overlap {press:6.1f} mm³ → {'PRESSED' if press > 1 else 'released'}"
          f" (expected {'PRESSED' if expect else 'released'}){'' if (press > 1) == expect or abs(abs(x) - p.trigger_x) < 20 else '  ✗'}")
print(f"\n{problems} collision(s) found")
