"""Preview both ends of the rig with the cart parked against the motor-end bumper. Run: uv run python preview_ends.py"""
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

import clearance as K
import pendulum_cad as C

p = K.p


def draw(ax, items, xlim, title, elev=18, azim=-60):
    for wp, col, alpha in items:
        for solid in wp.solids().vals():
            v, t = solid.tessellate(0.5, 0.5)
            v = np.array([(q.x, q.y, q.z) for q in v])
            ax.add_collection3d(Poly3DCollection(v[np.array(t)], facecolor=col, edgecolor="none", alpha=alpha))
    ax.set_xlim(*xlim); ax.set_ylim(-100, 60); ax.set_zlim(-90, 70)
    ax.set_box_aspect((xlim[1] - xlim[0], 160, 160)); ax.view_init(elev, azim); ax.set_title(title, fontsize=10)
    ax.set_xlabel("X"); ax.set_ylabel("Y"); ax.set_zlabel("Z")


fx = K.fixed
col = {"beam": ("#c8b58a", 0.25), "rail": ("#888", 1), "upper belt": ("#222", 1), "lower belt": ("#222", 1),
       "motor block": ("#1565c0", 1), "motor body": ("#555", 1), "pulley": ("#bbb", 1), "idler block": ("#1565c0", 1),
       "idler": ("#bbb", 1), "ramp R": ("#e08a2e", 1), "ramp L": ("#e08a2e", 1), "bumper R": ("#8e24aa", 1), "bumper L": ("#8e24aa", 1)}
fixed_items = [(fx[k], *col[k]) for k in fx]
mv = K.moving(457.0, 180.0)
mcol = {"carriage": "#666", "cart": "#2e7d4f", "belt clamp": "#4caf50", "sensor bracket": "#e08a2e", "switch carrier": "#d33", "rod": "#ccd"}
moving_items = [(mv[k], mcol[k], 1) for k in mv] + [(C.switch_dummy(p).translate((457, 0, 0)), "#111", 1)]
fig = plt.figure(figsize=(16, 6.5))
draw(fig.add_subplot(121, projection="3d"), fixed_items + moving_items, (330, 560),
     "Motor end: cart at the bumper, switches pressed by the orange ramp (blue = motor block, purple = bumper)")
draw(fig.add_subplot(122, projection="3d"), fixed_items, (-560, -330), "Idler end: tensioning fork, ramp, bumper", azim=-120)
fig.tight_layout()
fig.savefig(C.OUT / "ends_preview.png", dpi=80)
print("saved")
