"""Film the full AI controller (swing-up policy → hand-over → balance policy) on randomised rigs, with sensor
noise, like the classical clips: grey pole while swinging up, green while the balance policy holds it.
Usage: uv run python ai_demo.py SWING_TAG BALANCE_TAG out.mp4 [n_attempts]"""
import sys

import imageio.v2 as imageio
import mujoco
import numpy as np
from stable_baselines3 import SAC

import ai_handover as H
import ai_timelapse as T
import model
from envs.pendulum_env import PendulumEnv

ck = model.SIM_DIR / "checkpoints"
swing, bal = SAC.load(ck / sys.argv[1], device="cpu"), SAC.load(ck / sys.argv[2], device="cpu")
out, n = sys.argv[3], int(sys.argv[4]) if len(sys.argv) > 4 else 3
GREY, GREEN = [0.8, 0.8, 0.85, 1], [0.2, 0.8, 0.3, 1]
with imageio.get_writer(out, fps=T.FPS, codec="libx264", quality=8, macro_block_size=1) as w:
    for k in range(n):
        env = PendulumEnv("swingup", episode_s=8.0)
        obs, _ = env.reset(seed=77_000 + k)
        ctrl = H.HandOver(swing, bal, 10.0)
        m = model.build_single(env.p); d = mujoco.MjData(m); r = mujoco.Renderer(m, T.H, T.W)
        pole = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "pole")
        done, frames = False, []
        while not done:
            obs, _, term, trunc, _ = env.step(ctrl(obs)); done = term or trunc
            frames.append((env.d.qpos[0], env.d.qpos[1], ctrl.mode))
        cart = env.p["cart"]["mass"]
        lines = [f"AI controller: swing-up policy -> balance policy (attempt {k + 1})",
                 f"random rig: cart {cart * 1000:.0f} g, sensor noise on; green = balance policy in control"]
        for i in range(0, len(frames), int(env.policy_hz / T.FPS)):
            d.qpos[0], d.qpos[1] = frames[i][0], frames[i][1]
            m.geom_rgba[pole] = GREEN if frames[i][2] == "balance" else GREY
            mujoco.mj_forward(m, d); r.update_scene(d, camera="side")
            w.append_data(T.caption(r.render(), lines, (i + 1) / len(frames)))
print("saved", out)
