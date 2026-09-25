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


def evaluate(policy, task: str, n: int = 100, seed0: int = 10_000, env_kw: dict | None = None) -> dict:
    env = PendulumEnv(task, **(env_kw or {}))
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
    # usage: train_ai.py task steps seed [variant] [start-from checkpoint]
    # variants set reward weights, policy rate and gamma; a start-from checkpoint makes it a curriculum stage
    task, steps, seed = sys.argv[1], int(sys.argv[2]), int(sys.argv[3])
    variant = sys.argv[4] if len(sys.argv) > 4 else ""
    VARIANTS = {
        "": ({}, 0.99),                                                   # first attempt (drifts off the rail)
        "A": ({"w_x": 10.0, "w_da": 1.0}, 0.99),                          # strong centring + anti-jitter
        "B": ({"w_x": 10.0, "w_da": 1.0}, 0.995),                         # ... + longer look-ahead
        "C": ({"w_x": 10.0, "w_da": 1.0, "policy_hz": 50.0}, 0.99),       # ... at 50 decisions/s (≈ 2 s look-ahead)
        "D": ({"w_x": 10.0, "w_xd": 2.0, "w_da": 1.0, "policy_hz": 50.0}, 0.995),  # C + cart-speed penalty, ~4 s look-ahead
        # sanity check: fixed rig, rigid drive, no smoothing, no noise (SAC should master this quickly)
        "E": ({"randomise_scale": 0.0, "compliant": False, "smooth_hz": 1000.0}, 0.99),
        # ablation: the easy setup with ONE hard ingredient added back, to find which one breaks learning
        "Es": ({"randomise_scale": 0.0, "compliant": False}, 0.99),                          # + 20 Hz smoothing
        "Ed": ({"randomise_scale": 0.0, "smooth_hz": 1000.0}, 0.99),                          # + backlash / belt
        "Er": ({"compliant": False, "smooth_hz": 1000.0}, 0.99),                              # + randomised rigs, noise
        "Ej": ({"randomise_scale": 0.0, "compliant": False, "smooth_hz": 1000.0, "w_da": 1.0}, 0.99),  # + jitter penalty
        "Erp": ({"compliant": False, "smooth_hz": 1000.0, "noise_deg": 0.0}, 0.99),                    # + random physics only
        "Ern": ({"randomise_scale": 0.0, "compliant": False, "smooth_hz": 1000.0, "noise_deg": 0.068}, 0.99),  # + noise only
        # G: the full rig (random physics, gearbox play, smoothing) but NO sensor noise while learning (noise is
        # what breaks learning: +noise only 36/100, +random physics only 100/100). Evaluated WITH noise.
        "G": ({"noise_deg": 0.0}, 0.99),
        # curriculum stage 2: continue from stage 1 ("E") on the full randomised rig, with the anti-jitter penalty
        "F": ({"w_da": 1.0}, 0.99),
    }
    env_kw, gamma = VARIANTS[variant]
    OUT.mkdir(parents=True, exist_ok=True)
    CKPT.mkdir(exist_ok=True)
    tag = f"{task}{variant}_seed{seed}"
    env = Monitor(PendulumEnv(task, **env_kw), filename=str(OUT / tag))
    start_from = sys.argv[5] if len(sys.argv) > 5 else None   # curriculum: a checkpoint to continue from
    if start_from:
        agent = SAC.load(CKPT / start_from, env=env, device="cpu", seed=seed, learning_starts=2000)
    else:
        agent = SAC("MlpPolicy", env, policy_kwargs=dict(net_arch=[64, 64]), learning_starts=2000,
                    batch_size=256, learning_rate=3e-4, gamma=gamma, seed=seed, device="cpu", verbose=0)
    # Snapshots of the network while it learns: ~30 per run, for the training timelapse (ai_timelapse.py)
    snaps = CheckpointCallback(save_freq=max(steps // 30, 1000), save_path=str(CKPT / f"{tag}_snapshots"),
                               name_prefix=tag)
    t0 = time.time()
    agent.learn(total_timesteps=steps, callback=snaps)
    minutes = (time.time() - t0) / 60
    agent.save(CKPT / tag)
    # Always evaluate on the rig as it will really be: sensor noise on (training may have switched it off)
    eval_kw = {k: v for k, v in env_kw.items() if k != "noise_deg"}
    result = evaluate(agent, task, env_kw=eval_kw)
    result.update({"task": task, "variant": variant, "env": env_kw, "gamma": gamma, "seed": seed, "steps": steps,
                   "started_from": start_from,
                   "train_minutes": round(minutes, 1)})
    (OUT / f"{tag}.json").write_text(json.dumps(result, indent=2))
    print(f"{tag}: {result['success']}/{result['episodes']} randomised rigs, trained {minutes:.1f} min", flush=True)
