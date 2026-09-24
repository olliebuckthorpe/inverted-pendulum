"""Watch a simulated run, live in the MuJoCo viewer or saved as an MP4.

The run is simulated first (exactly as in the checks, with delay and quantised sensors), then replayed
at real speed. The pole is grey while swinging up and turns green while LQR is balancing.

  Live 3D window (macOS needs mjpython):   uv run mjpython watch.py swingup
  Save a video to sim/videos/:             uv run python watch.py swingup --video
  Options:  scenario = balance | swingup | tap     --seed N   --teeth 20|30   --length 0.30   --seconds 8
"""

from __future__ import annotations

import argparse
import math
import time

import mujoco
import mujoco.viewer
import numpy as np

import lqr
import model
import swingup
from plant import Plant

GREY, GREEN = [0.8, 0.8, 0.85, 1], [0.2, 0.8, 0.3, 1]


def simulate(scenario: str, p: dict, seed: int, seconds: float) -> dict:
    K = lqr.design(p, 500)
    plant = Plant(p, rate_hz=500, seed=seed)
    if scenario == "swingup":
        rng = np.random.default_rng(seed)
        plant.reset(x=rng.uniform(-0.02, 0.02), th=math.pi + rng.uniform(-0.03, 0.03))
        return plant.run(swingup.EnergySwingUp(p, K), seconds)
    plant.reset(th=math.radians(5))
    if scenario == "balance":
        return plant.run(lqr.LQRController(K), seconds)
    # tap: balance, then a tap on the pole every 2 s (a sudden 1.5 rad/s spin, alternating sides)
    tap_times = {round(t / plant.dt) for t in np.arange(1.0, seconds, 2.0)}
    def disturb(t):
        k = round(t / plant.dt)
        return (1.5 if (k // 2000) % 2 else -1.5) if k in tap_times else 0.0
    return plant.run(lqr.LQRController(K), seconds, disturb=disturb)


def frames(m, d, log, fps):
    """Yield once per video frame, with the model posed at that moment of the run."""
    pole = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "pole")
    for t in np.arange(0, log["t"][-1], 1 / fps):
        i = min(np.searchsorted(log["t"], t), len(log["t"]) - 1)
        d.qpos[0], d.qpos[1] = log["x"][i], log["th"][i]
        m.geom_rgba[pole] = GREEN if log["mode"][i] == 1 else GREY
        mujoco.mj_forward(m, d)
        yield t


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("scenario", choices=["balance", "swingup", "tap"])
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--teeth", type=int, default=20)
    ap.add_argument("--length", type=float, default=0.30)
    ap.add_argument("--seconds", type=float, default=8.0)
    ap.add_argument("--video", action="store_true")
    a = ap.parse_args()

    p = model.load_params(**{"drive.pulley_teeth": a.teeth, "pole.length": a.length})
    log = simulate(a.scenario, p, a.seed, a.seconds)
    m = model.build_single(p)
    d = mujoco.MjData(m)

    if a.video:
        import imageio.v2 as imageio
        out = model.SIM_DIR / "videos" / f"{a.scenario}-{a.teeth}T-{int(a.length * 100)}cm-seed{a.seed}.mp4"
        out.parent.mkdir(exist_ok=True)
        r = mujoco.Renderer(m, 480, 854)
        with imageio.get_writer(out, fps=60, codec="libx264", quality=8) as w:
            for _ in frames(m, d, log, 60):
                r.update_scene(d, camera="side")
                w.append_data(r.render())
        print(f"saved {out}")
        return

    with mujoco.viewer.launch_passive(m, d) as v:
        v.cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
        v.cam.fixedcamid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_CAMERA, "side")
        while v.is_running():  # loops the replay until you close the window
            start = time.time()
            for t in frames(m, d, log, 60):
                if not v.is_running():
                    break
                v.sync()
                time.sleep(max(0.0, t - (time.time() - start)))
            time.sleep(1.0)


if __name__ == "__main__":
    main()
