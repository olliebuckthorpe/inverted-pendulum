"""PLAN 6.1 check: the C++ policy (firmware/nn/mlp_policy.h, the code the Pico runs) matches PyTorch within
1e-4 on 1,000 observations recorded from the realistic sim, via both the uploaded (.bin) and compiled-in (.h)
weight paths."""
import shutil
import subprocess

import numpy as np
import pytest
import torch
from stable_baselines3 import SAC

import export_policy
import model
from envs.pendulum_env import PendulumEnv

TAG = "balanceG_seed0"
NN = model.SIM_DIR.parent / "firmware" / "nn"
pytestmark = pytest.mark.skipif(not (model.SIM_DIR / "checkpoints" / f"{TAG}.zip").exists() or not shutil.which("clang++"),
                                reason="needs the trained checkpoint and clang++")


def recorded_obs(n=1000):
    pol = SAC.load(model.SIM_DIR / "checkpoints" / f"{TAG}.zip", device="cpu")
    env, obs_list, i = PendulumEnv("balance"), [], 0
    while len(obs_list) < n:
        obs, _ = env.reset(seed=123 + i); done = False; i += 1
        while not done and len(obs_list) < n:
            obs_list.append(obs)
            obs, _, te, tr, _ = env.step(pol.predict(obs, deterministic=True)[0]); done = te or tr
    return pol, np.array(obs_list, dtype=np.float32)


def test_cpp_policy_matches_pytorch(tmp_path):
    export_policy.export(TAG)
    pol, obs = recorded_obs()
    with torch.no_grad():
        ref = pol.policy.actor(torch.as_tensor(obs), deterministic=True).numpy().ravel()   # tanh(mu), in [-1, 1]
    np.savetxt(tmp_path / "obs.csv", obs, delimiter=",", fmt="%.9g")
    for compiled_in in (False, True):
        exe = tmp_path / f"t{int(compiled_in)}"
        flags = ["-DCOMPILED_IN", f'-include{NN / "weights" / (TAG + ".h")}'] if compiled_in else []
        subprocess.run(["clang++", "-std=c++17", "-O2", "-I", str(NN), *flags, str(NN / "test_policy.cpp"), "-o", str(exe)], check=True)
        args = [str(tmp_path / "obs.csv")] if compiled_in else [str(NN / "weights" / f"{TAG}.bin"), str(tmp_path / "obs.csv")]
        out = subprocess.run([str(exe), *args], check=True, capture_output=True, text=True).stdout.split()
        got = np.array(out, dtype=np.float64)
        assert got.shape == ref.shape
        assert np.abs(got - ref).max() < 1e-4, np.abs(got - ref).max()
