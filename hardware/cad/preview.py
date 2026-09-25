"""Render an assembly preview (PNG) of the cart on the rail with bought parts as simple blocks, to check fits.
Run: uv run python preview.py"""
import cadquery as cq
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

import pendulum_cad as C

p = C.P()
back, front = C.shaft_y(p)
hub_y0 = p.pad_h + p.plate_t + p.boss_depth + p.hub_gap
parts = {
    # bought / context parts (grey)
    "beam (18 mm ply)": (cq.Workplane("XY").box(160, 18, 90).translate((0, p.beam_face_y - 9, -10)), "#c8b58a"),
    "rail MGN12": (cq.Workplane("XY").box(160, 8, p.rail_w).translate((0, p.beam_face_y + 4, 0)), "#888"),
    "carriage MGN12H": (cq.Workplane("XY").box(p.carriage_l, 10, p.carriage_w).translate((0, -5, 0)), "#666"),
    "belt (upper strand)": (cq.Workplane("XY").box(160, p.belt_w, p.belt_t).translate((0, p.belt_y, p.belt_z - p.belt_t / 2)), "#222"),
    "shaft 8 mm": (cq.Workplane("XZ").workplane(offset=-back).center(0, p.pivot_z).circle(p.shaft_d / 2).extrude(-(front - back)), "#aab"),
    "608 bearings": (cq.Workplane("XZ").workplane(offset=-p.pad_h).center(0, p.pivot_z).circle(11).circle(4).extrude(-p.bearing_w)
                     .union(cq.Workplane("XZ").workplane(offset=-(p.pad_h + p.plate_t + p.boss_depth - p.bearing_w)).center(0, p.pivot_z)
                            .circle(11).circle(4).extrude(-p.bearing_w)), "#bbb"),
    "rod (first 80 mm)": (cq.Workplane("XY").circle(p.tube_od / 2).extrude(-80).translate((0, hub_y0 + p.hub_t / 2, p.pivot_z - 12)), "#ccd"),
    # printed parts (colours)
    "cart": (C.cart(p), "#2e7d4f"),
    "belt clamp": (C.belt_clamp(p), "#4caf50"),
    "sensor bracket": (C.sensor_mount(p), "#e08a2e"),
    "magnet cap": (C.magnet_cap(p).rotate((0, 0, 0), (1, 0, 0), 90).translate((0, back + C.CAP_GRIP, p.pivot_z)), "#d33"),
    "rod hub": (C.rod_hub(p).rotate((0, 0, 0), (1, 0, 0), -90).rotate((0, 0, 0), (0, 1, 0), 90)
                .translate((0, hub_y0, p.pivot_z)), "#1565c0"),
}


def draw(ax, elev, azim, title):
    for name, (wp, col) in parts.items():
        if "behind" in title and "beam" in name:
            continue
        for solid in wp.solids().vals():
            verts, tris = solid.tessellate(0.3, 0.3)
            v = np.array([(q.x, q.y, q.z) for q in verts])
            ax.add_collection3d(Poly3DCollection(v[np.array(tris)], facecolor=col, edgecolor="none", alpha=0.95 if "beam" not in name else 0.35))
    ax.set_xlim(-60, 60); ax.set_ylim(-35, 85); ax.set_zlim(-70, 50)
    ax.set_box_aspect((120, 120, 120)); ax.view_init(elev, azim); ax.set_title(title, fontsize=10)
    ax.set_xlabel("X (along rail)"); ax.set_ylabel("Y (out from beam)"); ax.set_zlabel("Z (up)")


fig = plt.figure(figsize=(15, 5.5))
draw(fig.add_subplot(131, projection="3d"), 20, 60, "3/4 view from the room")
draw(fig.add_subplot(132, projection="3d"), 0, 0, "side view (looking along the rail)")
draw(fig.add_subplot(133, projection="3d"), 30, -120, "from behind, beam hidden (sensor side)")
fig.suptitle("Cart assembly preview: green = printed cart, blue = rod hub, orange = sensor bracket, red = magnet cap")
fig.tight_layout()
fig.savefig(C.OUT / "assembly_preview.png", dpi=85)
print("saved", C.OUT / "assembly_preview.png")
