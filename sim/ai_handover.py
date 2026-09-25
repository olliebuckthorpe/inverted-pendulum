"""The AI controller as PLAN 5.3 defines it: swing-up policy → hand-over → balance policy (Xu & Tran 2026),
mirroring classical energy swing-up → LQR. Hand over when the rod is within `catch_deg` of upright; hand back
to the swing-up policy if the balance policy lets it fall past `drop_deg`.

Evaluated on the FULL rig (randomised physics, gearbox play, smoothing, sensor noise): success = upright
(|θ| < 10°) for the last 2 s of 12 s. PLAN 5.3 target: >= 90%.

  uv run python ai_handover.py swingupE_seed0 balanceG_seed0
"""

import math
import sys

import numpy as np
import torch
from stable_baselines3 import SAC

import model
from envs.pendulum_env import PendulumEnv

torch.set_num_threads(1)


class HandOver:
    def __init__(self, swing, balance, catch_deg=15.0, drop_deg=35.0):
        self.swing, self.balance = swing, balance
        self.catch, self.drop = math.radians(catch_deg), math.radians(drop_deg)
        self.mode = "swing"

    def __call__(self, obs):
        th = math.atan2(obs[2], obs[3])
        if self.mode == "swing" and abs(th) < self.catch:
            self.mode = "balance"
        elif self.mode == "balance" and abs(th) > self.drop:
            self.mode = "swing"
        pol = self.balance if self.mode == "balance" else self.swing
        return pol.predict(obs, deterministic=True)[0]


def evaluate(swing, balance, catch_deg, n=100, seed0=10_000):
    env = PendulumEnv("swingup")
    ok, handovers = 0, 0
    for i in range(n):
        obs, _ = env.reset(seed=seed0 + i)
        ctrl = HandOver(swing, balance, catch_deg)
        done, term, info = False, False, {}
        while not done:
            obs, _, term, trunc, info = env.step(ctrl(obs))
            done = term or trunc
        ok += (not term) and info["upright_s"] >= 2.0
        handovers += ctrl.mode == "balance"
    return ok, handovers


if __name__ == "__main__":
    ck = model.SIM_DIR / "checkpoints"
    swing, balance = SAC.load(ck / sys.argv[1], device="cpu"), SAC.load(ck / sys.argv[2], device="cpu")
    for catch in (10.0, 15.0, 20.0):
        ok, h = evaluate(swing, balance, catch)
        print(f"{sys.argv[1]} → {sys.argv[2]}, hand-over at {catch:.0f}°: {ok}/100 swung up and held "
              f"(ended in balance mode: {h}/100)", flush=True)
