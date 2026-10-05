"""Gymnasium environment for the DOUBLE pendulum (SPEC §3 stretch; PLAN 8.5 and the AI double swing-up
experiment Ollie added on 30 Sep). The single's env (envs/pendulum_env.py) is the pattern; the physics and the
sensing are the double's (double_control.DoublePlant), so the AI sees exactly what the classical double sees:
  - a 500 Hz control loop with the sensing-to-actuation delay, quantised sensors (cart encoder, two 14-bit
    AS5047Ps, joint 2 measured RELATIVE to rod 1) and differenced, low-passed velocities;
  - the 5 A supply-current limiter;
  - the policy decides at `policy_hz` (default 200 Hz: the double is faster than the single, whose policy runs
    at 100 Hz) and a first-order filter smooths its action into the duty, as on the Pico;
  - domain randomisation of every parameter the post-sysid Monte Carlo varies, plus the delay and the
    sensor noise level.
Drive is rigid here (the double's MuJoCo model has no belt/backlash option yet); that is a known gap.

task = "balance": start near UU (both rods up), stay up and centred.
task = "swingup": start at DD (both rods hanging), get to UU and stay there.

Observation: [x/0.4, x_dot, sin a1, cos a1, sin a2, cos a2, a1_dot/10, a2_dot/20, duty]
  a1 = rod 1 from upright (0 = up), a2 = rod 2 relative to rod 1 (0 = in line), as the sensors report them.

estimator="diff" (default): velocities by differencing + low-pass, as the single uses.
estimator="kf": balance only. The classical double's steady-state Kalman filter about upright
  (double_control.kalman_gains), run every 2 ms control cycle, built from the NOMINAL model (on the Pico it's
  the fitted model, not the true rig). Differenced velocities at the AS5047P's noise are too poor for the
  double: the LQR itself balances this env 20/20 at 200 Hz but 2/20 at 100 Hz and 0/20 at 50 Hz with noise.
"""

from __future__ import annotations

import math
from collections import deque

import gymnasium as gym
import mujoco
import numpy as np

import double_control as C
import double_trajopt as D
import model
import motor
from plant import wrap

CTRL_HZ = 500.0
TRAVEL_LIMIT = 0.38
TURN_LIMIT = 2 * math.pi   # rod 1 within ±1 turn of hanging: the joint-2 cable (SPEC §3; the classical plans obey it)
# The hand-over zone is a level set of the balance LQR's cost-to-go, V = s'·S·s < CATCH_V (s = [x, a1, a2, xd,
# a1d, a2d] from upright, a2 relative). Calibrated 30 Sep on 400 random states on realistic rigs (50 Hz, Kalman,
# noise): V <= 180 caught 100 %, V <= 295 95 %, V <= 600 71 %. A box (15° and 3 rad/s per rod, or even 5°/1 rad/s)
# caught only 20-65 %: the double's catchable states are shaped by how the rods move TOGETHER, not per rod.
CATCH_V = 250.0


def randomise(base: dict, rng: np.random.Generator, scale: float = 1.0) -> dict:
    """The post-sysid ranges used by double_montecarlo (masses weighed ±2–5 %, motor ±5–15 %, frictions
    ±30 %), widened to the single's training ranges where those are larger, plus delay and sensor noise."""
    u = lambda frac: 1 + rng.uniform(-frac, frac) * scale
    lg = lambda: math.exp(rng.uniform(-1, 1) * math.log(2) * scale)
    c, q, j2, q2, m = base["cart"], base["pole"], base["joint2"], base["pole2"], base["motor"]
    return model.with_params(
        base,
        cart__mass=c["mass"] * u(0.15), joint2__mass=j2["mass"] * u(0.05),
        pole__mass=q["mass"] * u(0.05), pole2__mass=q2["mass"] * u(0.05), pole__hub_mass=q["hub_mass"] * u(0.05),
        motor__Kt=m["Kt"] * u(0.15), motor__Ke=m["Ke"] * u(0.15), motor__R=m["R"] * u(0.15),
        motor__rotor_inertia=m["rotor_inertia"] * u(0.30),
        cart__coulomb_friction=c["coulomb_friction"] * lg(), cart__viscous_friction=c["viscous_friction"] * lg(),
        pole__pivot_damping=q["pivot_damping"] * lg(), joint2__damping=j2["damping"] * lg(),
        sim__control_delay=rng.uniform(0.001, 0.004),
        sensing__angle_noise_deg=0.068 * math.exp(rng.uniform(-1, 1) * math.log(1.5) * scale),
    )


class DoubleEnv(gym.Env):
    metadata = {"render_modes": []}

    def __init__(self, task: str = "balance", randomise_scale: float = 1.0, policy_hz: float = 200.0,
                 smooth_hz: float = 40.0, episode_s: float | None = None, base: dict | None = None,
                 noise_deg: float | None = None, w_x: float = 1.0, w_da: float = 0.05, estimator: str = "diff",
                 start: str = "near", reward: str = "height"):
        """start (balance): "near" = small offsets from upright; "catch" = the messier states a swing-up hands
        over in (up to the catch zone's edge). reward (swing-up): "height" = tip height; "catch" = reach the
        hand-over zone, which ends the episode with a +100 bonus (the balance policy takes it from there)."""
        """noise_deg: if set, overrides the randomised sensor noise (0.0 = train without noise, the lesson
        from the single: noise stops SAC learning; always EVALUATE with noise on)."""
        assert task in ("balance", "swingup")
        self.task = task
        self.base = base or model.with_params(D.layout_params(30, 0.30, 0.20), supply__current_cap=5.0)
        self.scale, self.noise_deg, self.w_x, self.w_da = randomise_scale, noise_deg, w_x, w_da
        self.policy_hz = policy_hz
        self.n_ctrl = round(CTRL_HZ / policy_hz)
        self.alpha = 1 / (1 + CTRL_HZ / (2 * math.pi * smooth_hz))
        self.episode_steps = round((episode_s or (8.0 if task == "balance" else 12.0)) * policy_hz)
        self.estimator, self.start, self.reward_mode = estimator, start, reward
        self.kf_A = None
        self.S = None
        self.level = 1.0                                 # reverse curriculum: 1 = start hanging (evaluation)
        if start == "reverse":
            self.plan = D.load("30T_30-20_damping150_u60_x30")
            S = C.balance_gain(self.base, 50.0)[1]
            Xw = self.plan["X"].copy()
            Xw[1:3] = np.arctan2(np.sin(Xw[1:3]), np.cos(Xw[1:3]))
            V = np.einsum("ik,ij,jk->k", Xw, S, Xw)
            self.plan_entry = int(np.argmax(V < CATCH_V))   # first sample inside the zone
        if estimator == "kf":
            assert task == "balance", "the Kalman filter is linearised about upright"
            self._kf_setup()
        self.observation_space = gym.spaces.Box(-np.inf, np.inf, (9,), np.float32)
        self.action_space = gym.spaces.Box(-1.0, 1.0, (1,), np.float32)

    # --- setup ---------------------------------------------------------------------------------------
    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        rng = self.np_random
        self.p = randomise(self.base, rng, self.scale) if self.scale > 0 else self.base
        if self.noise_deg is not None:
            self.p = model.with_params(self.p, sensing__angle_noise_deg=self.noise_deg)
        p, s = self.p, self.p["sensing"]
        self.m = model.build_double(p)
        self.d = mujoco.MjData(self.m)
        self.dt = self.m.opt.timestep
        self.phys_per_ctrl = round(1 / (CTRL_HZ * self.dt))
        self.ctrl_dt = self.phys_per_ctrl * self.dt
        self.queue = deque([0.0] * round(p["sim"]["control_delay"] / self.dt))
        self.cap = p["supply"]["current_cap"]
        self.q_step = np.array([2 * math.pi * model.pulley_radius(p) / s["cart_counts_per_rev"],
                                2 * math.pi / 2 ** s["angle_bits"], 2 * math.pi / 2 ** s["angle_bits"]])
        self.v_alpha = self.ctrl_dt / (self.ctrl_dt + 1 / (2 * math.pi * s["velocity_filter_hz"]))
        self.noise = math.radians(s["angle_noise_deg"])
        self.rng = rng
        if self.task == "swingup" and self.start == "reverse":
            # reverse curriculum (Salimans & Chen 2018): start somewhere along the classical swing-up's path,
            # `level` of the way back from its end (0 = just below the catch zone, 1 = hanging). Only the plan's
            # STATES are used as starting points, never its actions; success is still judged from hanging.
            X = self.plan["X"]
            kmax = self.plan_entry - 3                # just before the plan enters the catch zone
            k_hi = int(round((1 - self.level) * kmax))
            k = int(rng.integers(max(0, k_hi - kmax // 10), k_hi + 1))
            q0 = tuple(X[:3, k] + rng.normal(0, [0.005, 0.02, 0.02]))
            qd0 = tuple(X[3:, k] + rng.normal(0, [0.02, 0.1, 0.1]))
        elif self.task == "balance" and self.start == "catch":
            if self.S is None:
                self.S = C.balance_gain(self.base, 50.0)[1]
            while True:                                   # uniform over the catch zone (rejection sampling)
                st = np.array([rng.uniform(-0.1, 0.1), rng.uniform(-0.3, 0.3), rng.uniform(-0.3, 0.3),
                               rng.uniform(-0.5, 0.5), rng.uniform(-3, 3), rng.uniform(-3, 3)])
                if st @ self.S @ st < CATCH_V:
                    break
            q0, qd0 = tuple(st[:3]), tuple(st[3:])
        elif self.task == "balance":
            q0 = (rng.uniform(-0.05, 0.05), rng.uniform(-0.06, 0.06), rng.uniform(-0.06, 0.06))
            qd0 = (rng.uniform(-0.1, 0.1), rng.uniform(-0.3, 0.3), rng.uniform(-0.3, 0.3))
        else:
            q0 = (rng.uniform(-0.05, 0.05), math.pi + rng.uniform(-0.05, 0.05), rng.uniform(-0.05, 0.05))
            qd0 = (0.0, rng.uniform(-0.2, 0.2), rng.uniform(-0.2, 0.2))
        mujoco.mj_resetData(self.m, self.d)
        self.d.qpos[:3], self.d.qvel[:3] = q0, qd0
        mujoco.mj_forward(self.m, self.d)
        self.prev = np.round(np.array(q0) / self.q_step) * self.q_step
        self.pos, self.vel = self.prev.copy(), np.zeros(3)
        self.duty = self.last_action = 0.0
        self.k = self.up_time = 0
        self.e = None
        if estimator_is_kf(self):
            self.e = np.array([*self.pos, *qd0])     # balance starts: the filter starts from the true state
        self.u_hist = deque([0.0] * 2, maxlen=2)         # the command that acted over the last cycle (~1 cycle delay)
        return self._obs(), {}

    def _measure(self):
        """The Pico's sensing: quantise, add angle noise, difference, low-pass (as double_control.DoublePlant)."""
        raw = np.array(self.d.qpos[:3])
        if self.noise:
            raw[1:] += self.rng.normal(0, self.noise, 2)
        meas = np.round(raw / self.q_step) * self.q_step
        delta = meas - self.prev
        delta[1:] = [wrap(v) for v in delta[1:]]
        self.prev = meas
        self.pos = self.pos + delta
        self.vel += self.v_alpha * (delta / self.ctrl_dt - self.vel)

    def _kf_setup(self):
        """The classical double's steady-state balance filter about upright, from the NOMINAL model."""
        if self.kf_A is None:
            import control
            nominal = model.with_params(self.base, sensing__angle_noise_deg=0.068)
            A, B = (np.array(M) for M in C.jacobians(nominal)(np.zeros(6), 0.0))
            sysd = control.c2d(control.ss(A, B, np.eye(6), 0), 1.0 / CTRL_HZ, method="zoh")
            kf = C.kalman_gains(nominal, {"t": [], "Ad": [], "dt": 1.0 / CTRL_HZ, "X": [np.zeros(6)]})
            self.kf_A, self.kf_B, self.kf_L = kf["A_bal"], kf["B_bal"].ravel(), kf["L_bal"]

    def start_kf(self):
        """Hand-over: start the balance filter from the current measured state (positions wrapped about
        upright, velocities from differencing); from here on the observation carries its estimate."""
        self._kf_setup()
        pos = np.array(self.pos)
        pos[1], pos[2] = wrap(pos[1]), wrap(pos[2])
        self.e = np.concatenate([pos, self.vel])

    def catch_V(self, measured: bool = False) -> float:
        """The balance LQR's cost-to-go of the true state (training reward) or of the measured one (hand-over)."""
        if self.S is None:
            self.S = C.balance_gain(self.base, 50.0)[1]
        if measured:
            s = np.array([*self.pos, *self.vel])
        else:
            s = np.array([*self.d.qpos[:3], *self.d.qvel[:3]])
        s[1], s[2] = wrap(s[1]), wrap(s[2])
        return float(s @ self.S @ s)

    def in_catch_zone(self, measured: bool = False) -> bool:
        return self.catch_V(measured) < CATCH_V

    def _kf(self):
        """One predict/correct of the balance Kalman filter on the measured positions (state error from upright)."""
        e = self.kf_A @ self.e + self.kf_B * self.u_hist[0]
        meas = np.array(self.pos)
        meas[1], meas[2] = wrap(meas[1]), wrap(meas[2])
        self.e = e + self.kf_L @ (meas - e[:3])

    def _obs(self) -> np.ndarray:
        x, a1, a2 = self.pos
        xd, a1d, a2d = self.vel
        if self.e is not None:
            xd, a1d, a2d = self.e[3:]
        return np.array([x / 0.4, xd, math.sin(a1), math.cos(a1), math.sin(a2), math.cos(a2),
                         a1d / 10.0, a2d / 20.0, self.duty], dtype=np.float32)

    # --- one policy decision -----------------------------------------------------------------------------
    def step(self, action):
        a = float(np.clip(action[0], -1.0, 1.0))
        for _ in range(self.n_ctrl):
            self.duty += self.alpha * (a - self.duty)
            for _ in range(self.phys_per_ctrl):
                self.queue.append(self.duty)
                self.d.ctrl[0] = motor.cap_duty_for_supply(self.p, self.queue.popleft(), self.d.qvel[0], self.cap)
                mujoco.mj_step(self.m, self.d)
            self._measure()
            if self.e is not None:
                self.u_hist.append(self.duty)
                self._kf()
        self.k += 1

        x, xd = self.d.qpos[0], self.d.qvel[0]
        a1, a2 = wrap(self.d.qpos[1]), wrap(self.d.qpos[2])
        b2 = wrap(a1 + a2)                       # rod 2's absolute angle from upright
        w1, w2 = self.d.qvel[1], self.d.qvel[1] + self.d.qvel[2]
        off_rail = abs(x) > TRAVEL_LIMIT
        # the cable across joint 1 only allows ±1 turn from hanging; the height-only swing-up spun rod 1 4-8 turns
        wound = self.task == "swingup" and abs(self.d.qpos[1] - math.pi) > TURN_LIMIT
        da = a - self.last_action
        self.last_action = a
        if self.task == "balance":
            fell = abs(a1) > 0.5 or abs(b2) > 0.5
            reward = (1.0 - 2.0 * (a1 * a1 + b2 * b2) - self.w_x * x * x - 0.01 * (w1 * w1 + w2 * w2)
                      - 0.05 * a * a - self.w_da * da * da)
            terminated = bool(fell or off_rail)
            if terminated:
                reward -= 20.0
        else:
            # tip height scaled to 0 (hanging) .. 1 (both rods straight up), centred and slow at the top. It must be
            # >= 0: with a -1..1 height, every hanging step cost -1 and running off the rail (-50) became the
            # best policy (30 Sep: every episode ended in ~37 steps)
            L1, L2 = self.p["pole"]["length"], self.p["pole2"]["length"]
            h = 0.5 * (1 + (L1 * math.cos(a1) + L2 * math.cos(b2)) / (L1 + L2))
            reward = h - 0.5 * self.w_x * x * x - 0.02 * a * a - self.w_da * da * da
            if self.reward_mode == "catchV":
                # guide toward the hand-over zone: grows smoothly as the LQR cost-to-go falls (the arrival-only
                # bonus gave the policy no signal about how close it came: min V stayed 1300-8000 vs the 250 zone)
                reward += 2.0 * math.exp(-self.catch_V() / 2000.0)
            if math.cos(a1) > 0.9 and math.cos(b2) > 0.9:
                reward += 1.0 - 0.01 * (w1 * w1 + w2 * w2)
            terminated = bool(off_rail or wound)
            if terminated:
                reward -= 50.0
            elif self.reward_mode == "catchV" and self.in_catch_zone():
                reward += 100.0
                terminated = True
            elif self.reward_mode == "catch" and self.in_catch_zone():
                reward += 100.0
                terminated = True                 # the balance policy takes over from here
        up = abs(a1) < math.radians(10) and abs(b2) < math.radians(10)
        self.up_time = self.up_time + 1 if up else 0
        truncated = self.k >= self.episode_steps
        info = {"a1": a1, "b2": b2, "x": x, "upright_s": self.up_time / self.policy_hz,
                "caught": bool(self.reward_mode in ("catch", "catchV") and terminated and not off_rail and not wound),
                "wound": bool(wound)}
        return self._obs(), float(reward), terminated, truncated, info


def estimator_is_kf(env) -> bool:
    return getattr(env, "estimator", "diff") == "kf"
