"""Watch a simulated run, live in the MuJoCo viewer or saved as an MP4.

The run is simulated first (exactly as in the checks, with delay and quantised sensors), then replayed
at real speed. The pole is grey while swinging up and turns green while LQR is balancing.

  Live 3D window:                          ./watch.sh swingup   (wraps mjpython; see the script)
  Save a video to sim/videos/:             uv run python watch.py swingup --video
  Options:  scenario = balance | swingup | tap | double     --seed N   --teeth 20|30   --length 0.30   --seconds 8
            double also takes --traj NAME (a saved trajectory from double_trajopt.py, e.g. 30T_30-20)
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
        if "q" in log:  # double
            d.qpos[:3] = log["q"][i]
        else:
            d.qpos[0], d.qpos[1] = log["x"][i], log["th"][i]
        m.geom_rgba[pole] = GREEN if log["mode"][i] == 1 else GREY
        mujoco.mj_forward(m, d)
        yield t


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("scenario", choices=["balance", "swingup", "tap", "double"])
    ap.add_argument("--traj", default="30T_30-20_damping150_u60_x30")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--teeth", type=int, default=20)
    ap.add_argument("--length", type=float, default=0.30)
    ap.add_argument("--seconds", type=float, default=8.0)
    ap.add_argument("--video", action="store_true")
    a = ap.parse_args()

    if a.scenario == "double":
        import double_control
        import double_trajopt
        teeth, rods = a.traj.split("_")[:2]
        p = double_trajopt.layout_params(int(teeth[:-1]), *(int(v) / 100 for v in rods.split("-")))
        traj = double_trajopt.load(a.traj)
        # Fly it as the real rig will see it: datasheet sensor noise, Kalman filter, 5 A supply limit.
        # Plans named "..._damping150..." were made assuming 1.5x joint-2 friction; design on that model.
        design = p
        if "_damping" in a.traj:
            factor = int(a.traj.split("_damping")[1].split("_")[0]) / 100
            design = double_trajopt.model.with_params(p, joint2__damping=p["joint2"]["damping"] * factor)
        p = double_trajopt.model.with_params(p, sensing__angle_noise_deg=0.068, supply__current_cap=5.0)
        design = double_trajopt.model.with_params(design, sensing__angle_noise_deg=0.068, supply__current_cap=5.0)
        g = double_control.tvlqr(design, traj, 500.0)
        kf = double_control.kalman_gains(design, g)
        log = double_control.trial(p, traj, rate_hz=500.0, seconds=traj["T"] + a.seconds, seed=a.seed,
                                   gains=g, kf=kf)["log"]
        m = model.build_double(p)
    else:
        p = model.load_params(**{"drive.pulley_teeth": a.teeth, "pole.length": a.length})
        log = simulate(a.scenario, p, a.seed, a.seconds)
        m = model.build_single(p)
    d = mujoco.MjData(m)

    if a.video:
        import imageio.v2 as imageio
        tag = a.traj if a.scenario == "double" else f"{a.teeth}T-{int(a.length * 100)}cm"
        out = model.SIM_DIR / "videos" / f"{a.scenario}-{tag}-seed{a.seed}.mp4"
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
