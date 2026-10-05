"""Parametric CAD for the rig's printed parts (PLAN 2.2), in CadQuery. Run: uv run python pendulum_cad.py

Coordinate system (see LAYOUT.md): X along the rail, Z up, Y out of the beam toward the room.
  Y = -13  beam front face (the rail is bolted to it)
  Y = 0    carriage top face (MGN12H: 13 mm from rail base to block top, HIWIN MG catalogue)
  Z = 0    rail centreline
The pendulum shaft is along Y at (X=0, Z=PIVOT_Z); the rod swings in the X–Z plane in front of everything.

Every number that comes from a datasheet says so; everything else is a design choice that can be changed here.
Outputs go to out/ as STEP (for editing elsewhere) and STL (for slicing), plus preview PNGs.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

import cadquery as cq

OUT = Path(__file__).resolve().parent / "out"


@dataclass
class P:
    # --- bought parts (datasheet values) ---
    carriage_w: float = 27.0        # MGN12H W (Z direction here)
    carriage_l: float = 45.4        # MGN12H L (X)
    carriage_hole_b: float = 20.0   # MGN12H B: hole spacing across (Z)
    carriage_hole_c: float = 20.0   # MGN12H C: hole spacing along (X)
    carriage_thread_depth: float = 3.5   # MGN12H: M3×3.5 — screws must not go deeper
    rail_w: float = 12.0            # MGN12 WR
    beam_face_y: float = -13.0      # MGN12H H = 13: rail base (beam face) to block top
    bearing_od: float = 22.0        # 608: 8 × 22 × 7
    bearing_w: float = 7.0
    shaft_d: float = 8.0            # 8 mm silver steel
    magnet_d: float = 6.0           # AS5047P kit magnet 6 × 2.5 mm (SPEC §6)
    magnet_h: float = 2.5
    tube_od: float = 8.0            # rod 1: 8 mm aluminium tube (SPEC §6)
    belt_w: float = 6.0             # GT2 6 mm
    belt_t: float = 1.4             # GT2 belt thickness incl. teeth (typical)
    pulley_r: float = 9.55          # 30T GT2 pitch radius (sim/model.py)

    # --- design choices ---
    screw_len: float = 20.0         # M3 × 20 into the carriage (longer screws = taller pads = room for the sensor board)
    pivot_z: float = 34.0           # shaft height above the rail centreline
    plate_t: float = 5.0            # cart plate thickness
    plate_x: float = 70.0           # cart plate length along the rail
    boss_depth: float = 22.0        # bearing boss sticks out this far in front of the plate
    wall: float = 3.0               # wall around bearings
    fit: float = 0.10               # radial clearance for the 608 seats: fit_test on Ollie's Bambu A1 (PLA, 0.20 mm
                                    # Standard), 5 Oct: seat 1 (0.10) a firm press fit, 2 and 3 looser
    # 8 mm shaft in fit_test's bores, 5 Oct: notch 1 (0.05) no, 2 (0.075) and 3 (0.10) only just: the A1 prints
    # 8 mm holes small. So the rod hub (slides on, then the pinch screws grip) gets more room than any test bore,
    # and the magnet cap (push-fit, no screws) gets the tightest bore the shaft still entered.
    shaft_fit: float = 0.15         # rod hub on the shaft, and its socket for the 8 mm aluminium tube
    press_fit: float = 0.10         # magnet cap onto the shaft
    hub_gap: float = 1.5            # gap between boss front and rod hub
    hub_t: float = 14.0             # rod hub thickness along the shaft
    belt_z: float = -26.0           # upper belt strand: its top face touches the cart's ledge here
                                    # (carriage bottom edge is at Z = -13.5; pulley centre = belt_z - pulley_r)
    belt_y: float = -6.0            # belt centre plane (between beam face and carriage face)
    magnet_gap: float = 1.25        # magnet face to sensor chip (AS5047P kit: 1–1.5 mm, SPEC §7)
    cap_spacer: float = 3.0         # plastic between steel shaft end and magnet (precaution, see LAYOUT.md)
    chip_h: float = 1.1             # sensor chip height above its board (TSSOP-14 ≈ 1.1 mm max; check on the real board)

    # --- frame, drive and safety (PLAN 2.2 end blocks) ---
    rail_len: float = 1000.0        # MGN12 rail; the beam is the same length (X = ±500)
    beam_t: float = 18.0            # plywood beam thickness (Y = -31 … -13)
    motor_d: float = 36.8           # Pololu 37D gearbox Ø (dimension drawing, item 4751–4758)
    motor_bc_r: float = 15.5        # 6 × M3 on a Ø31 bolt circle (neighbours 15.5 apart = hexagon)
    motor_thread: float = 3.0       # "threaded to a depth of 3.0 mm; exceeding this can damage gears"
    motor_offset: float = 7.0       # output shaft is 7.0 mm off the gearbox centre
    motor_boss_d: float = 12.0      # boss around the shaft, 6 mm long; shaft sticks out 22 mm from the face
    motor_body_len: float = 66.1    # face to end of encoder: L 20 (6.3:1) + 30.7 + 15.4
    motor_plate_t: float = 5.0
    motor_face_y: float = -29.0     # gearbox face against the plate's back; pulley slides on the shaft to align
    pulley_x: float = 515.0         # motor shaft X (clear of the beam end and of the cart at the bumper)
    idler_r: float = 6.37           # 20T GT2 idler pitch radius
    idler_bolt_d: float = 5.0       # idler bore 5 mm (SPEC §6 item 5)
    trigger_x: float = 350.0        # end-stop trips when the cart centre passes ±350 mm (electronics.md)
    ramp_rise: float = 3.0          # ramp lifts the switch roller 3 mm (pushes the lever ~2 mm past trip)
    ramp_z: tuple = (-26.0, -14.0)  # Z band the ramps and switch rollers share, in FRONT of the cart plate
    ramp_y: float = 35.0            # ramp base surface (faces the cart); the rollers rest 1 mm in front of it.
                                    # Upright at Y 35–40 stays behind the rod's surface (Y ≈ 43.5)
    switch_lever: float = 4.0       # ⚠️ roller tip height above the switch body (measure the real one)
    bumper_x: float = 480.0         # TPU bumper faces: the carriage end stops here
    switch_body: tuple = (20.0, 10.0, 6.4)   # ⚠️ generic roller micro switch L (along X) × H (the roller side,
                                             # along Y) × W (along Z); measure the real one
    switch_hole_pitch: float = 9.5  # ⚠️ its two mounting holes

    @property
    def pad_h(self) -> float:
        """Spacer pads under the plate: sized so an M3 screw engages at most 3.0 of the 3.5 mm thread."""
        return self.screw_len - (self.carriage_thread_depth - 0.5) - self.plate_t


def cart(p: P) -> cq.Workplane:
    """The cart: plate on four spacer pads on the carriage, a boss holding two 608 bearings for the shaft,
    a belt clamp under the rail, and a pocket behind the plate for the angle sensor board."""
    y0 = p.pad_h                                  # plate back face
    z_top = p.pivot_z + p.bearing_od / 2 + p.wall + 2
    z_bot = p.belt_z - 8
    plate = (cq.Workplane("XZ").workplane(offset=-y0)       # XZ workplane normal is -Y; offset moves along it
             .center(0, (z_top + z_bot) / 2)
             .rect(p.plate_x, z_top - z_bot).extrude(-p.plate_t))
    # spacer pads on the carriage's 20 × 20 hole pattern
    pads = cq.Workplane("XZ").pushPoints([(sx * p.carriage_hole_c / 2, sz * p.carriage_hole_b / 2)
                                          for sx in (-1, 1) for sz in (-1, 1)]).circle(4.0).extrude(-y0)
    body = plate.union(pads)
    # bearing boss in front of the plate
    boss_r = p.bearing_od / 2 + p.wall
    boss = (cq.Workplane("XZ").workplane(offset=-(y0 + p.plate_t)).center(0, p.pivot_z)
            .circle(boss_r).extrude(-p.boss_depth))
    body = body.union(boss)
    # bearing seats: one at the front of the boss, one at the plate's back face; shaft clearance between
    front_y = y0 + p.plate_t + p.boss_depth
    seat = lambda y_start: (cq.Workplane("XZ").workplane(offset=-y_start).center(0, p.pivot_z)
                            .circle(p.bearing_od / 2 + p.fit).extrude(-p.bearing_w))
    body = body.cut(seat(front_y - p.bearing_w)).cut(seat(y0))
    bore = (cq.Workplane("XZ").workplane(offset=-(y0 - 1)).center(0, p.pivot_z)
            .circle(p.shaft_d / 2 + 1.0).extrude(-(front_y + 2 - y0)))
    body = body.cut(bore)
    # screw holes through pads and plate (M3 clearance 3.4) with counterbores for the heads
    holes = [(sx * p.carriage_hole_c / 2, sz * p.carriage_hole_b / 2) for sx in (-1, 1) for sz in (-1, 1)]
    body = body.cut(cq.Workplane("XZ").pushPoints(holes).circle(1.7).extrude(-(y0 + p.plate_t + 1)))
    # belt ledge: a block below the carriage reaching back over the belt; the upper belt strand runs along its
    # underside (teeth up into printed ridges at the GT2 2 mm pitch) and a clamp plate screws up from below
    y_back = p.belt_y - p.belt_w / 2 - 1
    ledge = (cq.Workplane("XY").workplane(offset=p.belt_z).center(0, (y_back + y0) / 2)
             .rect(40, y0 - y_back).extrude(10))
    for i in range(-9, 10):                                   # ridges that mesh with the belt teeth
        ledge = ledge.union(cq.Workplane("XY").workplane(offset=p.belt_z - 0.7)
                            .center(i * 2.0, p.belt_y).rect(1.0, p.belt_w).extrude(0.7))
    body = body.union(ledge)
    # two pilot holes (2.8 mm, for self-tapping M3) in front of the belt, for the clamp plate's screws
    body = body.cut(cq.Workplane("XY").workplane(offset=p.belt_z - 1).pushPoints([(-14, 3), (14, 3)])
                    .circle(1.4).extrude(10))
    # pilot holes (2.8 mm) in the plate's back face for the sensor bracket's posts
    body = body.cut(cq.Workplane("XZ").workplane(offset=-(y0 - 0.1)).center(0, p.pivot_z)
                    .pushPoints(SENSOR_POSTS).circle(1.4).extrude(-(p.plate_t - 1)))
    # lightening pockets in the plate's lower corners
    for sx in (-1, 1):
        body = body.cut(cq.Workplane("XZ").workplane(offset=-(y0 - 0.1)).center(sx * 25, -3)
                        .rect(12, 18).extrude(-(p.plate_t + 0.2)))
    return body


def belt_clamp(p: P) -> cq.Workplane:
    """Plate that squeezes the upper belt strand against the cart's ridged ledge (two M3 × 12 screws)."""
    y_back = p.belt_y - p.belt_w / 2 - 1
    plate = cq.Workplane("XY").center(0, (y_back + 7) / 2).rect(40, 7 - y_back).extrude(3.0)
    return (plate.cut(cq.Workplane("XY").pushPoints([(-14, 3), (14, 3)]).circle(1.7).extrude(3.0))
            .translate((0, 0, p.belt_z - p.belt_t - 3.0)))


def pulley_z(p: P) -> float:
    """Motor shaft height: the upper belt strand's top face sits at belt_z (touching the cart's ledge)."""
    return p.belt_z - p.belt_t / 2 - p.pulley_r


def idler_z(p: P) -> float:
    return p.belt_z - p.belt_t / 2 - p.idler_r


def _end_mount(p: P, sign: int) -> cq.Workplane:
    """Shared base of both end blocks: a flange on the beam's end face (2 wood screws into the end grain) and a
    tab on the beam's back face (2 wood screws into the face). Returns the flange + tab at the +X (sign=1) or
    -X (sign=-1) end."""
    x_end = sign * p.rail_len / 2
    y_back, y_front = p.beam_face_y - p.beam_t, p.beam_face_y
    z0, z1 = -75.0, 30.0
    flange = cq.Workplane("XY").box(6, y_front - y_back, z1 - z0).translate((x_end + sign * 3, (y_back + y_front) / 2, (z0 + z1) / 2))
    tab = cq.Workplane("XY").box(36, 6, 40).translate((x_end - sign * 15, y_back - 3, 10))    # above the motor
    body = flange.union(tab)
    body = body.cut(cq.Workplane("YZ").workplane(offset=x_end - 1 if sign > 0 else x_end - 7)
                    .pushPoints([((y_back + y_front) / 2, -55), ((y_back + y_front) / 2, 15)]).circle(2.1).extrude(8))
    body = body.cut(cq.Workplane("XZ").workplane(offset=-(y_back - 7)).pushPoints([(x_end - sign * 5, 20), (x_end - sign * 25, 0)])
                    .circle(2.1).extrude(-8))
    return body


def motor_block(p: P) -> cq.Workplane:
    """+X end: holds the Pololu 37D behind the belt plane, shaft pointing toward the room (+Y), gearbox
    face against the plate's back. 12 clearance holes on the Ø31 circle so the motor fits in either hexagon
    orientation; screws must not enter the gearbox more than 3.0 mm (Pololu)."""
    cx, cz = p.pulley_x + p.motor_offset, pulley_z(p)          # gearbox centre (shaft is offset toward the rail)
    r_out = p.motor_d / 2 + 4
    y0 = p.motor_face_y                                          # plate back face = gearbox face
    plate = (cq.Workplane("XZ").workplane(offset=-y0).center(cx, cz).circle(r_out).extrude(-p.motor_plate_t))
    web = (cq.Workplane("XZ").workplane(offset=-y0).center((p.rail_len / 2 + cx) / 2, cz + 10)
           .rect(cx - p.rail_len / 2, 2 * r_out - 8).extrude(-p.motor_plate_t))            # joins the plate to the flange
    body = plate.union(web).union(_end_mount(p, 1))
    body = body.cut(cq.Workplane("XZ").workplane(offset=-(y0 - 1)).center(p.pulley_x, cz)
                    .circle(p.motor_boss_d / 2 + 0.5).extrude(-(p.motor_plate_t + 2)))
    holes = [(p.motor_bc_r * __import__("math").cos(a), p.motor_bc_r * __import__("math").sin(a))
             for a in [i * __import__("math").pi / 6 for i in range(12)]]
    body = body.cut(cq.Workplane("XZ").workplane(offset=-(y0 - 1)).center(cx, cz).pushPoints(holes)
                    .circle(1.7).extrude(-(p.motor_plate_t + 2)))
    return body


def idler_block(p: P) -> cq.Workplane:
    """-X end: a fork holding the 20T idler on an M5 bolt along Y. The bolt sits in 14 mm slots (along X):
    pull it outward to tension the belt, then tighten; an M3 screw in the block's end pushes it for fine
    adjustment."""
    x_i, z_i = -p.pulley_x, idler_z(p)
    y_b, y_f = p.belt_y - 6.5, p.belt_y + 6.5               # cheeks either side of the idler (≈ 8.5 mm wide)
    cheek = lambda y0, y1: cq.Workplane("XY").box(36, y1 - y0, 22).translate((x_i + 3, (y0 + y1) / 2, z_i))
    back = cheek(p.beam_face_y - p.beam_t + 2, y_b)          # back cheek runs into the end mount
    front = cheek(y_f, y_f + 4)
    bridge = cq.Workplane("XY").box(8, y_f + 4 - (p.beam_face_y - p.beam_t + 2), 22).translate(
        (x_i - 15 + 4, (y_f + 4 + p.beam_face_y - p.beam_t + 2) / 2, z_i))              # closes the outer end
    body = back.union(front).union(bridge).union(_end_mount(p, -1))
    slot = (cq.Workplane("XZ").workplane(offset=-(p.beam_face_y - p.beam_t)).center(x_i, z_i)
            .slot2D(14 + p.idler_bolt_d, p.idler_bolt_d + 0.4, 0).extrude(-(y_f + 5 - p.beam_face_y + p.beam_t)))
    body = body.cut(slot)
    body = body.cut(cq.Workplane("YZ").workplane(offset=x_i - 16).center(p.belt_y, z_i).circle(1.4).extrude(10))  # M3 push screw
    return body


def end_ramp(p: P, sign: int) -> cq.Workplane:
    """At each end: a foot screwed to the beam face low down (below both belt strands), an arm reaching forward
    under the cart, and an upright in front of the cart plate carrying the ramp (facing the cart). From
    trigger_x the ramp rises ramp_rise over 10 mm, then stays high to the bumper, so the cart's switch rollers
    are pushed and STAY pushed while the cart brakes. Stays behind the rod's swing plane (Y ≈ 43)."""
    z0, z1 = p.ramp_z
    x0, x1 = p.trigger_x - 6, p.bumper_x
    zf = -64.0                                               # arm height: below the lower belt strand (≥ -46.5)
    t = 5.0
    y_up = p.ramp_y + t                                      # upright's front face
    foot = cq.Workplane("XY").box(40, 6, 22).translate((x1 - 20, p.beam_face_y + 3, zf + 4))
    arm = cq.Workplane("XY").box(40, y_up - p.beam_face_y, 6).translate((x1 - 20, (y_up + p.beam_face_y) / 2, zf))
    upright = cq.Workplane("XY").box(x1 - x0, t, z1 - zf + 3).translate(((x0 + x1) / 2, p.ramp_y + t / 2, (z1 + zf) / 2))
    # the ramp: a wedge on the upright's back face, rising toward the cart (−Y) after the trigger point
    prof = (cq.Workplane("XY").polyline([(x0, 0), (x0 + 6, 0), (x0 + 16, p.ramp_rise), (x1, p.ramp_rise), (x1, 0)]).close()
            .extrude(z1 - z0))
    wedge = prof.mirror("XZ").translate((0, p.ramp_y, z0))  # heights go toward −Y
    body = foot.union(arm).union(upright).union(wedge)
    body = body.cut(cq.Workplane("XZ").workplane(offset=-(p.beam_face_y - 1)).pushPoints([(x1 - 32, zf + 8), (x1 - 8, zf + 8)])
                    .circle(1.6).extrude(-8))                # screws into the beam face
    return body if sign > 0 else body.mirror("YZ")


def switch_carrier(p: P) -> cq.Workplane:
    """Screwed to the cart plate's front face below the bearing boss: two ledges each carry a roller micro switch
    (wired in series, for redundancy), roller facing the room. The pair is mirrored so both rollers sit near
    the cart's centre (±7 mm): the first trips at about ±348 mm at either end. Switch body along X, its roller side facing +Y;
    its two mounting holes run vertically through slots that slide 3 mm in Y to set the press: at rest the
    roller clears the ramp base by 1 mm, on the ramp's flat it is pushed ramp_rise - 1 = 2 mm."""
    z0, z1 = p.ramp_z
    L, H, W = p.switch_body
    y_front = p.pad_h + p.plate_t                            # cart plate front face
    back = cq.Workplane("XY").box(56, 3, z1 - z0 + 16).translate((0, y_front + 1.5, (z0 + z1) / 2 - 4))
    body = back.cut(cq.Workplane("XZ").workplane(offset=-(y_front - 1)).pushPoints([(-24, z0 - 8), (24, z0 - 8), (-24, z1 - 2), (24, z1 - 2)])
                    .circle(1.7).extrude(-5))               # 4 × M3 into the cart plate (self-tapping)
    y_sw0 = p.ramp_y - 1.0 - p.switch_lever - H             # switch body's back face at rest
    z_ledge = (z0 + z1) / 2 - W / 2 - 3                      # ledge top = switch underside; roller mid-band
    for sx in (-14, 14):
        ledge = cq.Workplane("XY").box(L + 4, y_sw0 + H - (y_front + 3), 3).translate(
            (sx, (y_sw0 + H + y_front + 3) / 2, z_ledge + 1.5))
        ledge = ledge.cut(cq.Workplane("XY").workplane(offset=z_ledge - 1).pushPoints(
            [(sx - p.switch_hole_pitch / 2, y_sw0 + H / 2), (sx + p.switch_hole_pitch / 2, y_sw0 + H / 2)])
            .slot2D(6.0, 2.6, 90).extrude(6))
        body = body.union(ledge)
    return body


def switch_dummy(p: P) -> cq.Workplane:
    """Stand-in boxes for the two switches at rest (preview and clearance checks only; not printed)."""
    z0, z1 = p.ramp_z
    L, H, W = p.switch_body
    y_sw0 = p.ramp_y - 1.0 - p.switch_lever - H
    z_ledge = (z0 + z1) / 2 - W / 2 - 3
    out = None
    for sx in (-14, 14):
        b = cq.Workplane("XY").box(L, H, W).translate((sx, y_sw0 + H / 2, z_ledge + 3 + W / 2))
        roller_x = sx - math.copysign(L / 2 - 3, sx)          # mirrored pair: rollers at ±7 mm, next to the centre
        b = b.union(cq.Workplane("XY").box(4, p.switch_lever, 4).translate((roller_x, y_sw0 + H + p.switch_lever / 2, z_ledge + 3 + W / 2)))
        out = b if out is None else out.union(b)
    return out


def bumper(p: P, sign: int) -> cq.Workplane:
    """Print in TPU. Screwed to the beam face at each rail end, in the carriage's path: the last resort if the
    cart is still moving after the brake relay drops."""
    blk = cq.Workplane("XY").box(12, 12, 24).translate((sign * (p.bumper_x + 6), p.beam_face_y + 6 + 0.0, 0))
    blk = blk.cut(cq.Workplane("XY").box(8, 12, 12).translate((sign * (p.bumper_x + 6), p.beam_face_y + 6, 0)))  # hollow = softer
    return blk.cut(cq.Workplane("XZ").workplane(offset=-(p.beam_face_y - 1)).pushPoints([(sign * (p.bumper_x + 6), 9), (sign * (p.bumper_x + 6), -9)])
                   .circle(1.6).extrude(-14))


CAP_GRIP = 6.0   # how far the magnet cap slides onto the shaft


def shaft_y(p: P) -> tuple[float, float]:
    """(back end, front end) of the steel shaft along Y. The back end sits so the magnet cap stops just
    behind the rear bearing (which starts at the plate's back face); the front end passes through the rod hub."""
    back = p.pad_h - 0.5 - CAP_GRIP
    front = p.pad_h + p.plate_t + p.boss_depth + p.hub_gap + p.hub_t
    return back, front


def sensor_y(p: P) -> float:
    """Y of the sensor chip face: shaft back end − cap spacer − magnet − air gap."""
    return shaft_y(p)[0] - p.cap_spacer - p.magnet_h - p.magnet_gap


SENSOR_POSTS = [(-13, 13), (13, 13), (-13, -13), (13, -13)]   # around the pivot, clear of the cap and bearing


def sensor_mount(p: P) -> cq.Workplane:
    """Bracket behind the cart plate holding the AS5047P board facing the shaft's back end, on four posts
    screwed (self-tapping M3) into the plate's back face. The board's size isn't known yet (kit not in hand),
    so it mounts through slots giving ±2 mm in X and Z; shims set the magnet gap (PLAN 2.2: adjustable)."""
    y_face = sensor_y(p)                        # the chip's top face must sit here
    y_plate = p.pad_h                           # cart plate back face
    t = 2.0
    y_b = y_face - p.chip_h                     # bracket back face = the board's front face (chip pokes through)
    bracket = (cq.Workplane("XZ").workplane(offset=-y_b).center(0, p.pivot_z).rect(36, 36).extrude(-t))
    bracket = bracket.cut(cq.Workplane("XZ").workplane(offset=-(y_b - 1)).center(0, p.pivot_z)
                          .rarray(18, 18, 2, 2).slot2D(7, 3.2, 0).extrude(-(t + 2)))   # board slots
    bracket = bracket.cut(cq.Workplane("XZ").workplane(offset=-(y_b - 1)).center(0, p.pivot_z)
                          .circle(7.0).extrude(-(t + 2)))                    # 14 mm window: chip + spinning cap
    posts = (cq.Workplane("XZ").workplane(offset=-(y_b + t)).center(0, p.pivot_z)
             .pushPoints(SENSOR_POSTS).circle(3.0).extrude(-(y_plate - y_b - t)))
    bracket = bracket.union(posts)
    return bracket.cut(cq.Workplane("XZ").workplane(offset=-(y_b - 1)).center(0, p.pivot_z)
                       .pushPoints(SENSOR_POSTS).circle(1.7).extrude(-(y_plate - y_b + 2)))


def magnet_cap(p: P) -> cq.Workplane:
    """Plastic cap pushed onto the shaft's back end: holds the magnet cap_spacer mm away from the steel."""
    grip = 6.0
    cap = cq.Workplane("XY").circle(p.shaft_d / 2 + 1.6).extrude(grip + p.cap_spacer + p.magnet_h)
    cap = cap.cut(cq.Workplane("XY").circle(p.shaft_d / 2 + p.press_fit).extrude(grip))            # onto the shaft
    cap = cap.cut(cq.Workplane("XY").workplane(offset=grip + p.cap_spacer)
                  .circle(p.magnet_d / 2 + p.fit).extrude(p.magnet_h + 1))                         # magnet pocket
    return cap


def rod_hub(p: P) -> cq.Workplane:
    """Clamps the 8 mm tube to the 8 mm shaft at right angles. Split-clamp on the shaft (M3 pinch screw);
    the tube goes in a blind socket with its own pinch screw."""
    r = p.shaft_d / 2 + 5
    hub = cq.Workplane("XY").circle(r).extrude(p.hub_t)                      # shaft along Z here
    arm = cq.Workplane("YZ").center(0, p.hub_t / 2).circle(p.tube_od / 2 + 3).extrude(r + 18)
    hub = hub.union(arm)
    hub = hub.cut(cq.Workplane("XY").circle(p.shaft_d / 2 + p.shaft_fit).extrude(p.hub_t))         # shaft bore
    hub = hub.cut(cq.Workplane("YZ").workplane(offset=r - 2).center(0, p.hub_t / 2)
                  .circle(p.tube_od / 2 + p.shaft_fit).extrude(22))                                   # tube socket
    hub = hub.cut(cq.Workplane("XY").center(-r, 0).rect(2 * r, 1.2).extrude(p.hub_t))              # clamp slit
    hub = hub.cut(cq.Workplane("YZ").workplane(offset=-r - 1).center(-r / 2 - 1, p.hub_t / 2)
                  .circle(1.6).extrude(2 * r + 2))                                                  # pinch screw
    return hub


def estimate_mass_g(solid: cq.Workplane, infill: float = 0.3, walls_mm: float = 1.2, density: float = 1.24) -> float:
    """Rough printed mass: shell of walls_mm plus infill of the rest (PLA 1.24 g/cm³)."""
    v = solid.val().Volume()
    a = solid.val().Area()
    shell = min(v, a * walls_mm / 2)   # walls are counted from both sides of thin features, so halve
    return (shell + infill * (v - shell)) * density / 1000


def fit_test(p: P, fits=(0.10, 0.15, 0.20)) -> cq.Workplane:
    """Print this FIRST (LAYOUT.md "check when the parts arrive"): three 608 seats and three 8 mm shaft bores at
    radial clearances 0.10 / 0.15 / 0.20 mm (the shaft bores use half, as the real parts do), marked with 1, 2, 3
    notches on the front edge. Pick the tightest seat a bearing still presses into by hand and the tightest bore
    the shaft turns in, and set P.fit to that value."""
    pitch = p.bearing_od + 6
    w, d, h = 3 * pitch, p.bearing_od + 6 + 14, p.bearing_w + 2
    body = cq.Workplane("XY").box(w, d, h, centered=(True, False, False))
    for i, f in enumerate(fits):
        x = (i - 1) * pitch
        y_seat = 3 + p.bearing_od / 2
        body = body.cut(cq.Workplane("XY").workplane(offset=h - p.bearing_w).center(x, y_seat)
                        .circle(p.bearing_od / 2 + f).extrude(p.bearing_w + 1))                # bearing seat
        body = body.cut(cq.Workplane("XY").center(x, y_seat).circle(5).extrude(h))              # push-out hole
        body = body.cut(cq.Workplane("XY").center(x, d - 7).circle(p.shaft_d / 2 + f / 2).extrude(h + 1))  # shaft bore
        for n in range(i + 1):                                                                     # 1/2/3 notches
            body = body.cut(cq.Workplane("XY").center(x + (n - i / 2) * 2.5, 0.75).rect(1.2, 1.5).extrude(h + 1))
    return body


PARTS = {"motor_block": motor_block, "idler_block": idler_block, "switch_carrier": switch_carrier,
         "end_ramp_right": lambda p: end_ramp(p, 1), "end_ramp_left": lambda p: end_ramp(p, -1),
         "bumper_right": lambda p: bumper(p, 1), "bumper_left": lambda p: bumper(p, -1),
         "cart": cart, "belt_clamp": belt_clamp, "sensor_mount": sensor_mount, "magnet_cap": magnet_cap, "rod_hub": rod_hub,
         "fit_test": fit_test}

if __name__ == "__main__":
    p = P()
    OUT.mkdir(exist_ok=True)
    print(f"spacer pads {p.pad_h:.1f} mm (M3×{p.screw_len:.0f} engages {p.screw_len - p.pad_h - p.plate_t:.1f} of 3.5 mm thread)")
    back, front = shaft_y(p)
    print(f"steel shaft: cut to {front - back:.1f} mm (Y {back:.1f} to {front:.1f}); sensor chip face at Y {sensor_y(p):.2f}; "
          f"beam face at Y {p.beam_face_y:.0f} (room for the board behind the bracket: {sensor_y(p) - p.chip_h - p.beam_face_y:.1f} mm)")
    for name, fn in PARTS.items():
        part = fn(p)
        cq.exporters.export(part, str(OUT / f"{name}.step"))
        cq.exporters.export(part, str(OUT / f"{name}.stl"), tolerance=0.05, angularTolerance=0.1)
        bb = part.val().BoundingBox()
        print(f"{name:13s} {bb.xlen:5.1f} × {bb.ylen:5.1f} × {bb.zlen:5.1f} mm, ≈ {estimate_mass_g(part):5.1f} g printed")
