"""The AI double's environment (envs/double_env.py) must let a known-good controller balance: the classical
LQR through the env's own observation. Guards the physics, the sensing and the Kalman observation."""
import math

import numpy as np

import double_control as C
from envs.double_env import DoubleEnv


def lqr_through_env(estimator: str, hz: float = 50.0, n: int = 5) -> int:
    ok = 0
    for seed in range(n):
        env = DoubleEnv("balance", randomise_scale=1.0, policy_hz=hz, smooth_hz=40.0, noise_deg=0.068,
                        estimator=estimator)
        env.reset(seed=seed)
        K, _ = C.balance_gain(env.base, hz)
        done = term = False
        while not done:
            o = env._obs()
            s = np.array([o[0] * 0.4, math.atan2(o[2], o[3]), math.atan2(o[4], o[5]), o[1], o[6] * 10, o[7] * 20])
            _, _, term, trunc, _ = env.step([float(np.clip(-(K @ s)[0], -1, 1))])
            done = term or trunc
        ok += not term
    return ok


def test_kalman_observation_lets_lqr_balance_at_50hz_with_noise():
    assert lqr_through_env("kf") == 5


def test_differenced_velocities_are_not_enough_at_50hz_with_noise():
    """Why the Kalman observation exists: with plain differencing the same LQR fails (0/20 measured)."""
    assert lqr_through_env("diff") <= 1
