"""The AI double's hand-over machinery (envs/double_env.py): the catch zone is catchable, reaching it ends a
swing-up episode as a catch, and rod 1 can't wind the joint-2 cable past ±1 turn."""
import math

import mujoco
import numpy as np

import double_control as C
from envs.double_env import TURN_LIMIT, DoubleEnv


def test_lqr_catches_from_catch_zone_starts_on_realistic_rigs():
    ok = 0
    for seed in range(5):
        env = DoubleEnv("balance", policy_hz=50, smooth_hz=40, noise_deg=0.068, estimator="kf", start="catch")
        env.reset(seed=seed)
        K, _ = C.balance_gain(env.base, 50)
        done = term = False
        while not done:
            o = env._obs()
            s = np.array([o[0] * 0.4, math.atan2(o[2], o[3]), math.atan2(o[4], o[5]), o[1], o[6] * 10, o[7] * 20])
            _, _, term, trunc, _ = env.step([float(np.clip(-(K @ s)[0], -1, 1))])
            done = term or trunc
        ok += not term
    assert ok >= 4


def test_reaching_the_zone_ends_the_episode_as_a_catch():
    env = DoubleEnv("swingup", randomise_scale=0.0, noise_deg=0.0, policy_hz=50, reward="catchV")
    env.reset(seed=0)
    env.d.qpos[:3], env.d.qvel[:3] = (0.0, 0.01, 0.0), (0.0, 0.0, 0.0)
    mujoco.mj_forward(env.m, env.d)
    _, r, term, _, info = env.step([0.0])
    assert term and info["caught"] and r > 90


def test_rod1_past_one_turn_ends_the_episode():
    env = DoubleEnv("swingup", randomise_scale=0.0, noise_deg=0.0, policy_hz=50, reward="catchV")
    env.reset(seed=0)
    env.d.qpos[1] = math.pi + TURN_LIMIT + 0.2
    mujoco.mj_forward(env.m, env.d)
    _, r, term, _, info = env.step([0.0])
    assert term and info["wound"] and not info["caught"] and r < -40
