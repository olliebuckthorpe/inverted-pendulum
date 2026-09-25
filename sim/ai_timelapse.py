"""Training timelapse: film network snapshots from one training run, on the same rig and the same start,
captioned with how much practice each had. Shows the AI going from falling over to balancing / swinging up.

  uv run python ai_timelapse.py balance_seed0 [n_snapshots] [out.mp4]

Snapshots come from train_ai.py (checkpoints/<tag>_snapshots/). Practice time = steps / 100 Hz.
"""

from __future__ import annotations

import re
import sys

import imageio.v2 as imageio
import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont
from stable_baselines3 import SAC

import model
from envs.pendulum_env import POLICY_HZ, PendulumEnv

FPS = 50
W, H = 854, 480


def rollout(policy, task: str, seed: int, seconds: float) -> tuple[dict, list]:
    env = PendulumEnv(task, episode_s=seconds)
    obs, _ = env.reset(seed=seed)
    frames, done, fell_at = [], False, None
    while not done:
        action, _ = policy.predict(obs, deterministic=True)
        obs, _, term, trunc, _ = env.step(action)
        frames.append((env.d.qpos[0], env.d.qpos[1]))
        done = term or trunc
        if term:
            fell_at = len(frames) / POLICY_HZ
    return env.p, frames, fell_at


def caption(img: np.ndarray, lines: list[str], progress: float) -> np.ndarray:
    im = Image.fromarray(img)
    dr = ImageDraw.Draw(im)
    try:
        big, small = ImageFont.load_default(size=30), ImageFont.load_default(size=20)
    except TypeError:
        big = small = ImageFont.load_default()
    dr.text((24, 18), lines[0], fill=(20, 20, 20), font=big)
    for i, line in enumerate(lines[1:]):
        dr.text((24, 60 + 26 * i), line, fill=(60, 60, 60), font=small)
    dr.rectangle([24, H - 30, W - 24, H - 18], outline=(90, 90, 90))
    dr.rectangle([24, H - 30, 24 + int((W - 48) * progress), H - 18], fill=(40, 120, 60))
    return np.asarray(im)


def main():
    tag = sys.argv[1]
    n = int(sys.argv[2]) if len(sys.argv) > 2 else 8
    task = "balance" if tag.startswith("balance") else "swingup"   # tags are e.g. "balanceG_seed0"
    snapdir = model.SIM_DIR / "checkpoints" / f"{tag}_snapshots"
    snaps = sorted(snapdir.glob("*.zip"), key=lambda f: int(re.search(r"_(\d+)_steps", f.name).group(1)))
    final = model.SIM_DIR / "checkpoints" / f"{tag}.zip"
    if final.exists():
        snaps.append(final)
    pick = [snaps[i] for i in np.unique(np.round(np.linspace(0, len(snaps) - 1, n)).astype(int))]
    total = int(re.search(r"_(\d+)_steps", snaps[-2 if final.exists() else -1].name).group(1)) if len(snaps) > 1 else 1
    out = sys.argv[3] if len(sys.argv) > 3 else str(model.SIM_DIR / "videos" / f"timelapse-{tag}.mp4")
    seconds = 6.0 if task == "balance" else 10.0
    rig_seed = 424242  # the same randomised rig and start for every snapshot

    with imageio.get_writer(out, fps=FPS, codec="libx264", quality=8) as w:
        for f in pick:
            m_ = re.search(r"_(\d+)_steps", f.name)
            steps = int(m_.group(1)) if m_ else total
            policy = SAC.load(f, device="cpu")
            p, frames, fell_at = rollout(policy, task, rig_seed, seconds)
            m = model.build_single(p)
            d = mujoco.MjData(m)
            r = mujoco.Renderer(m, H, W)
            hours = steps / POLICY_HZ / 3600
            verdict = f"fell after {fell_at:.1f} s" if fell_at is not None else ("balanced" if task == "balance" else "stayed on the rail")
            lines = [f"AI {'balance' if task == 'balance' else 'swing-up'}: after {steps:,} practice steps",
                     f"~{hours:.1f} h of simulated practice  ·  realistic sim (noise on)  ·  this attempt: {verdict}"]
            for k in range(0, len(frames), int(POLICY_HZ / FPS)):
                d.qpos[0], d.qpos[1] = frames[k]
                mujoco.mj_forward(m, d)
                r.update_scene(d, camera="side")
                w.append_data(caption(r.render(), lines, steps / total))
            for _ in range(FPS // 2):  # short pause on the last frame of each attempt
                w.append_data(caption(r.render(), lines, steps / total))
    print(f"saved {out}")


if __name__ == "__main__":
    main()
