"""Train an SAC policy for the DOUBLE pendulum (PLAN 8.5 + the AI double swing-up experiment, 30 Sep) and
evaluate it on randomised rigs. Mirrors train_ai.py (the single); the environment is envs/double_env.py.

  uv run python train_double.py balance 500000 0 G      # task, total policy steps, seed, variant
  uv run python train_double.py swingup 3000000 0 E

One CPU thread per run, so seeds run side by side. Models go to checkpoints/ (not in git); the learning curve
and the evaluation go to results/ai_double/.

Evaluation: 100 episodes, each on a newly randomised rig the policy never trained on, ALWAYS on the realistic
rig (random physics + datasheet sensor noise), and again without noise so the gap is visible.
  balance: success = both rods still up and the cart on the rail after 8 s
  swingup: success = both rods upright (|angle| < 10°) for the last 2 s of 12 s
"""

from __future__ import annotations

import json
import sys
import time

import torch
from stable_baselines3 import SAC
from stable_baselines3.common.callbacks import BaseCallback, CallbackList, CheckpointCallback
from stable_baselines3.common.monitor import Monitor

import model
from envs.double_env import DoubleEnv

torch.set_num_threads(1)
OUT = model.SIM_DIR / "results" / "ai_double"
CKPT = model.SIM_DIR / "checkpoints"

# variant -> (env kwargs, gamma, network)
VARIANTS = {
    # the single's lesson: train on random physics WITHOUT sensor noise (noise stops SAC learning), evaluate with it
    "G": ({"noise_deg": 0.0}, 0.99, [128, 128]),
    # balance learnability check (the single's "E"): one fixed rig, no noise, no action smoothing
    "Eb": ({"randomise_scale": 0.0, "noise_deg": 0.0, "smooth_hz": 1000.0}, 0.99, [128, 128]),
    # 30 Sep: G and Eb both failed at 200 Hz (Eb 0-1/50 on its OWN setup), yet LQR balances this env 20/20 even at
    # 50 Hz with smoothing -> the task is feasible, the learning isn't. At 200 Hz gamma 0.99 looks ~0.5 s ahead;
    # the benchmark SAC solves (MuJoCo InvertedDoublePendulum) decides at 20 Hz. So: 50 Hz (~2 s look-ahead)
    "Eb50": ({"randomise_scale": 0.0, "noise_deg": 0.0, "smooth_hz": 1000.0, "policy_hz": 50.0}, 0.99, [128, 128]),
    "G50": ({"noise_deg": 0.0, "policy_hz": 50.0}, 0.99, [128, 128]),
    # ... seeing the Kalman-filtered state, as the classical double does (LQR through it: 20/20 at 50 Hz with
    # noise on random rigs, vs 0/20 with differencing)
    "G50k": ({"noise_deg": 0.0, "policy_hz": 50.0, "estimator": "kf"}, 0.99, [128, 128]),
    # hand-over pair (the single's route, 30 Sep): balance practises the messier states a swing-up hands over in;
    # swing-up is rewarded for REACHING the catch zone (both rods within 15°, < 3 rad/s), then balance takes over.
    # The height-only swing-up plateaued at ~half the possible reward, two of three seeds sliding back.
    "G50kc": ({"noise_deg": 0.0, "policy_hz": 50.0, "estimator": "kf", "start": "catch"}, 0.99, [128, 128]),
    "Ec": ({"randomise_scale": 0.0, "noise_deg": 0.0, "policy_hz": 50.0, "reward": "catch"}, 0.995, [256, 256]),
    "Gc": ({"noise_deg": 0.0, "policy_hz": 50.0, "reward": "catch"}, 0.995, [256, 256]),
    "Ecv": ({"randomise_scale": 0.0, "noise_deg": 0.0, "policy_hz": 50.0, "reward": "catchV"}, 0.995, [256, 256]),
    # 1 Oct, Ollie: one last principled try. Reverse curriculum: start just below the catch zone along the classical
    # plan's path, move the start back towards hanging whenever >= 60 % of recent episodes reach the zone
    "Er": ({"randomise_scale": 0.0, "noise_deg": 0.0, "policy_hz": 50.0, "reward": "catchV", "start": "reverse"},
           0.995, [256, 256]),
    "Gr": ({"noise_deg": 0.0, "policy_hz": 50.0, "reward": "catchV", "start": "reverse"}, 0.995, [256, 256]),
    # swing-up feasibility first: one fixed rig, no noise (can SAC learn a double swing-up at all?)
    "E": ({"randomise_scale": 0.0, "noise_deg": 0.0, "policy_hz": 100.0}, 0.995, [256, 256]),
    # then the randomised rig (no noise while learning); 100 Hz decisions + gamma 0.995 ≈ a 2 s look-ahead
    "Gs": ({"noise_deg": 0.0, "policy_hz": 100.0}, 0.995, [256, 256]),
}


class ReverseCurriculum(BaseCallback):
    """Move the start back towards hanging (level 0 -> 1) whenever >= 60 % of the last 100 episodes reached the
    catch zone; log the level so the training curve can be read against it."""

    def __init__(self, log_path, step: float = 0.05, need: float = 0.6):
        super().__init__()
        self.log_path, self.step_size, self.need = log_path, step, need
        self.recent = []

    def _on_training_start(self):
        self.training_env.envs[0].unwrapped.level = 0.0

    def _on_step(self) -> bool:
        for done, info in zip(self.locals["dones"], self.locals["infos"]):
            if done:
                self.recent.append(bool(info.get("caught")))
        env = self.training_env.envs[0].unwrapped
        if len(self.recent) >= 100:
            rate = sum(self.recent[-100:]) / 100
            if rate >= self.need and env.level < 1.0:
                env.level = min(1.0, env.level + self.step_size)
                with open(self.log_path, "a") as f:
                    f.write(f"{self.num_timesteps} {env.level:.2f} {rate:.2f}\n")
                self.recent = []
        return True


def evaluate(policy, task: str, env_kw: dict, n: int = 100, seed0: int = 20_000) -> dict:
    env = DoubleEnv(task, **env_kw)
    ok = 0
    for i in range(n):
        obs, _ = env.reset(seed=seed0 + i)
        done, terminated, info = False, False, {}
        while not done:
            action, _ = policy.predict(obs, deterministic=True)
            obs, _, terminated, truncated, info = env.step(action)
            done = terminated or truncated
        if task == "balance":
            ok += not terminated
        elif env.reward_mode in ("catch", "catchV"):
            ok += info["caught"]                     # reached the hand-over zone
        else:
            ok += (not terminated) and info["upright_s"] >= 2.0
    return {"episodes": n, "success": ok, "rate": ok / n}


if __name__ == "__main__":
    task, steps, seed, variant = sys.argv[1], int(sys.argv[2]), int(sys.argv[3]), sys.argv[4]
    start_from = sys.argv[5] if len(sys.argv) > 5 else None     # continue training a saved checkpoint
    env_kw, gamma, net = VARIANTS[variant]
    OUT.mkdir(parents=True, exist_ok=True)
    CKPT.mkdir(exist_ok=True)
    tag = f"double_{task}{variant}_seed{seed}" + ("_cont" if start_from else "")
    env = Monitor(DoubleEnv(task, **env_kw), filename=str(OUT / tag))
    if start_from:
        agent = SAC.load(CKPT / start_from, env=env, device="cpu", seed=seed)
    else:
        agent = SAC("MlpPolicy", env, policy_kwargs=dict(net_arch=net), learning_starts=5000, batch_size=256,
                    learning_rate=3e-4, gamma=gamma, seed=seed, device="cpu", verbose=0)
    snaps = CheckpointCallback(save_freq=max(steps // 30, 1000), save_path=str(CKPT / f"{tag}_snapshots"),
                               name_prefix=tag)
    t0 = time.time()
    cb = snaps
    if env_kw.get("start") == "reverse":
        cb = CallbackList([snaps, ReverseCurriculum(OUT / f"{tag}.levels")])
    agent.learn(total_timesteps=steps, callback=cb)
    minutes = (time.time() - t0) / 60
    agent.save(CKPT / tag)
    # the decision rate carries over (it changes what the policy sees); everything else is the realistic rig
    keep = {k: v for k, v in env_kw.items() if k in ("policy_hz", "estimator", "start", "reward")}  # the job, not the rig
    result = {"own_setup": evaluate(agent, task, env_kw),                     # what it trained on: did it learn?
              "realistic": evaluate(agent, task, keep),                        # random physics + sensor noise
              "no_noise": evaluate(agent, task, {**keep, "noise_deg": 0.0})}    # random physics, clean sensors
    result.update({"task": task, "variant": variant, "env": env_kw, "gamma": gamma, "net": net, "seed": seed,
                   "steps": steps, "started_from": start_from, "train_minutes": round(minutes, 1)})
    (OUT / f"{tag}.json").write_text(json.dumps(result, indent=2))
    print(f"{tag}: own setup {result['own_setup']['success']}/100, realistic {result['realistic']['success']}/100, no-noise {result['no_noise']['success']}/100, "
          f"trained {minutes:.1f} min", flush=True)
