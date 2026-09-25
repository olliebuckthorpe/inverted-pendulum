"""Gymnasium environment for the single pendulum: what the AI policy experiences (PLAN 5.3, SPEC §10).

It runs exactly the loop the Pico will run:
  - a 500 Hz control loop with the sensing-to-actuation delay, quantised sensors and differenced velocities
    (plant.Estimator), and the 5 A supply-current limiter (motor.cap_duty_for_supply);
  - the policy decides at 100 Hz; its action is held between decisions;
  - a first-order low-pass filter smooths the policy's action into the duty cycle (the same filter runs on
    the Pico). Without it, RL policies learn bang-bang switching that wrecks hardware (Xu & Tran 2026).
  - domain randomisation: every episode draws a new "rig" around params.yaml (PLAN 5.3 ranges), including
    gearbox play and belt stretch (the compliant model) and the sensor noise level.

task = "balance": start near upright, stay up and centred.
task = "swingup": start hanging, get up and stay up (PLAN 5.3 swing-up policy).

Observation (all roughly in [-1, 1]): [x/0.4, x_dot, sin θ, cos θ, θ_dot/10, duty]  (θ = 0 upright).
"""

from __future__ import annotations

import math
from collections import deque

import gymnasium as gym
import mujoco
import numpy as np

import model
import motor
from plant import Estimator, wrap

CTRL_HZ = 500.0
POLICY_HZ = 100.0
TRAVEL_LIMIT = 0.38   # m: episode ends here (soft limit, before the 0.40 m joint limit)


def randomise(base: dict, rng: np.random.Generator, scale: float = 1.0) -> dict:
    """PLAN 5.3 ranges (scale 1.0), plus rotor inertia, the sim phase's biggest unknown."""
    u = lambda frac: 1 + rng.uniform(-frac, frac) * scale
    c, q, m = base["cart"], base["pole"], base["motor"]
    L = q["length"] * u(0.10)
    return model.with_params(
        base,
        cart__mass=c["mass"] * u(0.15),
        pole__mass=q["mass"] * u(0.10) * L / q["length"],
        pole__length=L,
        motor__Kt=m["Kt"] * u(0.15), motor__Ke=m["Ke"] * u(0.15), motor__R=m["R"] * u(0.15),
        motor__rotor_inertia=m["rotor_inertia"] * u(0.30),
        cart__coulomb_friction=c["coulomb_friction"] * math.exp(rng.uniform(-1, 1) * math.log(2) * scale),
        cart__viscous_friction=c["viscous_friction"] * math.exp(rng.uniform(-1, 1) * math.log(2) * scale),
        pole__pivot_damping=q["pivot_damping"] * math.exp(rng.uniform(-1, 1) * math.log(2) * scale),
        sim__control_delay=rng.uniform(0.001, 0.005),
        # gearbox play 0–2× (SPEC §10 requires it; PLAN 5.3 range) and belt stretch ±50% (a guess until measured)
        motor__backlash_deg=m["backlash_deg"] * rng.uniform(0.0, 1.0 + scale),
        drive__belt_stiffness=base["drive"]["belt_stiffness"] * u(0.5),
        # AS5047P datasheet says 0.068° RMS; vary it until PLAN 2.4 measures the real value
        sensing__angle_noise_deg=0.068 * math.exp(rng.uniform(-1, 1) * math.log(1.5) * scale),
    )


class PendulumEnv(gym.Env):
    metadata = {"render_modes": []}

    def __init__(self, task: str = "balance", randomise_scale: float = 1.0, smooth_hz: float = 20.0,
                 episode_s: float | None = None, base: dict | None = None, compliant: bool = True):
        """compliant=True: motor drives the cart through gearbox play and a stretchy belt (plant.py's model)."""
        assert task in ("balance", "swingup")
        self.task = task
        self.compliant = compliant
        self.base = base or model.with_params(model.load_params(), supply__current_cap=5.0)
        self.scale = randomise_scale
        self.episode_steps = round((episode_s or (10.0 if task == "balance" else 12.0)) * POLICY_HZ)
        self.n_ctrl = round(CTRL_HZ / POLICY_HZ)          # control cycles per policy decision
        self.alpha = 1 / (1 + CTRL_HZ / (2 * math.pi * smooth_hz))  # first-order smoothing per control cycle
        self.observation_space = gym.spaces.Box(-np.inf, np.inf, (6,), np.float32)
        self.action_space = gym.spaces.Box(-1.0, 1.0, (1,), np.float32)

    # --- setup ---------------------------------------------------------------------------------------
    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        rng = self.np_random
        self.p = randomise(self.base, rng, self.scale) if self.scale > 0 else self.base
        self.m = model.build_single(self.p, compliant=self.compliant)
        self.d = mujoco.MjData(self.m)
        self.gap = model.backlash_halfwidth(self.p)
        self.k_belt, self.c_belt = self.p["drive"]["belt_stiffness"], self.p["drive"]["belt_damping"]
        self.dt = self.m.opt.timestep
        self.phys_per_ctrl = round(1 / (CTRL_HZ * self.dt))
        self.queue = deque([0.0] * round(self.p["sim"]["control_delay"] / self.dt))
        self.est = Estimator(self.p, 1 / CTRL_HZ, rng)
        if self.task == "balance":
            th = rng.uniform(-0.10, 0.10)
            thd = rng.uniform(-0.3, 0.3)
        else:
            th = math.pi + rng.uniform(-0.05, 0.05)
            thd = rng.uniform(-0.2, 0.2)
        self.d.qpos[0], self.d.qpos[1] = rng.uniform(-0.05, 0.05), th
        self.d.qvel[1] = thd
        if self.compliant:
            self.d.qpos[2] = self.d.qpos[0]  # drive starts where the cart is
        mujoco.mj_forward(self.m, self.d)
        self.duty = 0.0
        self.last_action = 0.0
        self.k = 0
        self.upright_time = 0
        self.obs_state = self.est(self.d.qpos[0], self.d.qpos[1])
        return self._obs(), {}

    def _obs(self) -> np.ndarray:
        x, xd, th, thd = self.obs_state
        return np.array([x / 0.4, xd, math.sin(th), math.cos(th), thd / 10.0, self.duty], dtype=np.float32)

    # --- one policy decision -----------------------------------------------------------------------------
    def step(self, action):
        a = float(np.clip(action[0], -1.0, 1.0))
        i_peak = 0.0
        for _ in range(self.n_ctrl):
            # the Pico's loop: smooth toward the policy's action, then run the plant for one control period
            self.duty += self.alpha * (a - self.duty)
            for _ in range(self.phys_per_ctrl):
                self.queue.append(self.duty)
                v_drive = self.d.qvel[2] if self.compliant else self.d.qvel[0]
                duty = motor.cap_duty_for_supply(self.p, self.queue.popleft(), v_drive, self.p["supply"]["current_cap"])
                self.d.ctrl[0] = duty
                if self.compliant:  # belt spring with a dead-band for the gearbox play
                    stretch = self.d.qpos[2] - self.d.qpos[0]
                    spring = self.k_belt * (stretch - math.copysign(min(abs(stretch), self.gap), stretch))
                    f = spring + self.c_belt * (self.d.qvel[2] - self.d.qvel[0])
                    self.d.qfrc_applied[0], self.d.qfrc_applied[2] = f, -f
                mujoco.mj_step(self.m, self.d)
                i_peak = max(i_peak, abs(motor.motor_current(self.p, duty, v_drive)))
            self.obs_state = self.est(self.d.qpos[0], self.d.qpos[1])
        self.k += 1

        x, th = self.d.qpos[0], wrap(self.d.qpos[1])
        xd, thd = self.d.qvel[0], self.d.qvel[1]
        off_rail = abs(x) > TRAVEL_LIMIT
        da = a - self.last_action
        self.last_action = a
        if self.task == "balance":
            fell = abs(th) > 0.5
            reward = 1.0 - 2.0 * th * th - 1.0 * x * x - 0.02 * thd * thd - 0.05 * a * a - 0.1 * da * da
            terminated = bool(fell or off_rail)
            if terminated:
                reward -= 20.0
        else:
            # swing-up: reward being up (cos θ = 1), being centred and slow near the top; penalise effort and
            # jerky actions; hitting the rail ends the episode with a penalty
            up = math.cos(th)
            reward = up - 0.5 * x * x - 0.05 * a * a - 0.1 * da * da
            if up > 0.9:
                reward += 1.0 - 0.02 * thd * thd
            terminated = bool(off_rail)
            if terminated:
                reward -= 50.0
        self.upright_time = self.upright_time + 1 if abs(th) < math.radians(10) else 0
        truncated = self.k >= self.episode_steps
        info = {"th": th, "x": x, "i_peak": i_peak, "upright_s": self.upright_time / POLICY_HZ}
        return self._obs(), float(reward), terminated, truncated, info
