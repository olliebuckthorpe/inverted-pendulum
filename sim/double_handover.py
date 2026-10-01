"""The AI double as a whole: swing-up policy -> hand-over -> balance policy, like the single (ai_handover.py).
The swing-up policy runs until the MEASURED state is in the catch zone (envs/double_env.CATCH_V); then the balance
Kalman filter starts from the measured state and the balance policy holds it. Hand back to the swing-up policy
if a rod passes 40°. Scored on the REALISTIC rig (random physics + sensor noise), rigs never trained on:
success = both rods upright (< 10°) for the last 2 s of 12 s.

  uv run python double_handover.py double_swingupEc_seed0 double_balanceG50kc_seed0 [n]
"""
import math
import sys

import torch
from stable_baselines3 import SAC

import model
from envs.double_env import DoubleEnv
from plant import wrap

torch.set_num_threads(1)
DROP = math.radians(40.0)


def run(swing, balance, n: int = 100, seed0: int = 30_000, swing_hz: float = 50.0) -> dict:
    ok = caught = 0
    for i in range(n):
        env = DoubleEnv("swingup", policy_hz=swing_hz, reward="height")   # realistic: random physics + noise
        obs, _ = env.reset(seed=seed0 + i)
        mode, done, term, info = "swing", False, False, {}
        while not done:
            if mode == "swing" and env.in_catch_zone(measured=True):   # decided on what the Pico would see
                mode = "balance"
                env.start_kf()
                caught += 1
                obs = env._obs()
            elif mode == "balance" and (abs(wrap(env.d.qpos[1])) > DROP or abs(wrap(env.d.qpos[1] + env.d.qpos[2])) > DROP):
                mode = "swing"
                env.e = None                                               # back to differenced velocities
                obs = env._obs()
            pol = balance if mode == "balance" else swing
            action, _ = pol.predict(obs, deterministic=True)
            obs, _, term, trunc, info = env.step(action)
            done = term or trunc
        ok += (not term) and info["upright_s"] >= 2.0
    return {"episodes": n, "success": ok, "hand_overs": caught}


if __name__ == "__main__":
    ck = model.SIM_DIR / "checkpoints"
    n = int(sys.argv[3]) if len(sys.argv) > 3 else 100
    r = run(SAC.load(ck / sys.argv[1], device="cpu"), SAC.load(ck / sys.argv[2], device="cpu"), n)
    print(f"{sys.argv[1]} -> {sys.argv[2]}: {r['success']}/{r['episodes']} swung up and held on realistic rigs "
          f"({r['hand_overs']} hand-overs)")
