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
    fit: float = 0.15               # radial clearance for press/slip fits (tune after a test print)
    hub_gap: float = 1.5            # gap between boss front and rod hub
    hub_t: float = 14.0             # rod hub thickness along the shaft
    belt_z: float = -26.0           # upper belt strand: its top face touches the cart's ledge here
                                    # (carriage bottom edge is at Z = -13.5; pulley centre = belt_z - pulley_r)
    belt_y: float = -6.0            # belt centre plane (between beam face and carriage face)
    magnet_gap: float = 1.25        # magnet face to sensor chip (AS5047P kit: 1–1.5 mm, SPEC §7)
    cap_spacer: float = 3.0         # plastic between steel shaft end and magnet (precaution, see LAYOUT.md)
    chip_h: float = 1.1             # sensor chip height above its board (TSSOP-14 ≈ 1.1 mm max; check on the real board)

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
    cap = cap.cut(cq.Workplane("XY").circle(p.shaft_d / 2 + p.fit / 2).extrude(grip))            # onto the shaft
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
    hub = hub.cut(cq.Workplane("XY").circle(p.shaft_d / 2 + p.fit / 2).extrude(p.hub_t))         # shaft bore
    hub = hub.cut(cq.Workplane("YZ").workplane(offset=r - 2).center(0, p.hub_t / 2)
                  .circle(p.tube_od / 2 + p.fit / 2).extrude(22))                                   # tube socket
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


PARTS = {"cart": cart, "belt_clamp": belt_clamp, "sensor_mount": sensor_mount, "magnet_cap": magnet_cap, "rod_hub": rod_hub}

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
