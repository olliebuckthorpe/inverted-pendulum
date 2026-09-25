"""Train an SAC policy (PLAN 5.3) and evaluate it on randomised rigs.

  uv run python train_ai.py balance 200000 0      # task, total policy steps, seed
  uv run python train_ai.py swingup 2000000 0

One run uses one CPU thread (measured fastest for this small network: 490 steps/s vs 388 with 6 threads),
so several seeds can run side by side. Models go to checkpoints/ (not in git); the learning curve and the
evaluation go to results/ai/.

Evaluation (PLAN 5.3 "done when"): 100 episodes, each on a newly randomised rig.
  balance: success = still up and on the rail after 10 s          (target >= 95%)
  swingup: success = upright (|θ| < 10°) for the last 2 s of 12 s   (with hand-over: target >= 90%)
"""

from __future__ import annotations

import json
import sys
import time

import numpy as np
import torch
from stable_baselines3 import SAC
from stable_baselines3.common.callbacks import CheckpointCallback
from stable_baselines3.common.monitor import Monitor

import model
from envs.pendulum_env import PendulumEnv

torch.set_num_threads(1)
OUT = model.SIM_DIR / "results" / "ai"
CKPT = model.SIM_DIR / "checkpoints"


def evaluate(policy, task: str, n: int = 100, seed0: int = 10_000) -> dict:
    env = PendulumEnv(task)
    ok, peaks = 0, []
    for i in range(n):
        obs, _ = env.reset(seed=seed0 + i)
        done, terminated, info = False, False, {}
        while not done:
            action, _ = policy.predict(obs, deterministic=True)
            obs, _, terminated, truncated, info = env.step(action)
            done = terminated or truncated
            peaks.append(info["i_peak"])
        if task == "balance":
            ok += not terminated
        else:
            ok += (not terminated) and info["upright_s"] >= 2.0
    return {"episodes": n, "success": ok, "rate": ok / n, "peak_motor_A": float(np.max(peaks))}


if __name__ == "__main__":
    task, steps, seed = sys.argv[1], int(sys.argv[2]), int(sys.argv[3])
    OUT.mkdir(parents=True, exist_ok=True)
    CKPT.mkdir(exist_ok=True)
    tag = f"{task}_seed{seed}"
    env = Monitor(PendulumEnv(task), filename=str(OUT / tag))
    agent = SAC("MlpPolicy", env, policy_kwargs=dict(net_arch=[64, 64]), learning_starts=2000,
                batch_size=256, learning_rate=3e-4, gamma=0.99, seed=seed, device="cpu", verbose=0)
    # Snapshots of the network while it learns: ~30 per run, for the training timelapse (ai_timelapse.py)
    snaps = CheckpointCallback(save_freq=max(steps // 30, 1000), save_path=str(CKPT / f"{tag}_snapshots"),
                               name_prefix=tag)
    t0 = time.time()
    agent.learn(total_timesteps=steps, callback=snaps)
    minutes = (time.time() - t0) / 60
    agent.save(CKPT / tag)
    result = evaluate(agent, task)
    result.update({"task": task, "seed": seed, "steps": steps, "train_minutes": round(minutes, 1)})
    (OUT / f"{tag}.json").write_text(json.dumps(result, indent=2))
    print(f"{tag}: {result['success']}/{result['episodes']} randomised rigs, trained {minutes:.1f} min", flush=True)
