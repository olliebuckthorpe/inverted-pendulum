"""PLAN 1.2 visual check: film strip of the pole released 5° from upright with zero motor input."""

import math

import matplotlib
import mujoco

matplotlib.use("Agg")
import matplotlib.pyplot as plt

import model

m = model.build_single(model.load_params())
d = mujoco.MjData(m)
d.qpos[1] = math.radians(5)
mujoco.mj_forward(m, d)
renderer = mujoco.Renderer(m, 360, 480)
times = [0.0, 0.3, 0.45, 0.6, 0.9, 1.5]
shots = []
while d.time < times[-1] + 1e-9:
    if any(abs(d.time - t) < 5e-4 for t in times):
        renderer.update_scene(d, camera="side")
        shots.append((d.time, renderer.render().copy(), d.qpos[0], d.qpos[1]))
    mujoco.mj_step(m, d)

fig, axes = plt.subplots(1, len(shots), figsize=(3 * len(shots), 3))
for ax, (t, img, x, th) in zip(axes, shots):
    ax.imshow(img[60:330, 60:420])
    ax.set_axis_off()
    ax.set_title(f"t={t:.2f}s  θ={math.degrees(th):.0f}°\ncart x={x * 1000:.1f} mm", fontsize=9)
fig.suptitle("single.xml, released 5° from upright, duty 0 (PLAN 1.2)")
fig.tight_layout()
fig.savefig(model.SIM_DIR / "plots" / "1.2-fall-from-upright.png", dpi=90)
