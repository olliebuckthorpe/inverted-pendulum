"""Classical double swing-up: track the optimised trajectory with time-varying LQR, then hand over to the
double's balance LQR (Tedrake, Underactuated Robotics ch. 8 + 10).

State (same order as double_trajopt): s = [x, a1, a2, x_dot, a1_dot, a2_dot], a2 relative to rod 1.

TVLQR in plain terms: the plan says where the rig should be at every instant. Near that plan the rig
behaves linearly, s_err(k+1) ≈ A_k·s_err(k) + B_k·u_err(k), with A_k, B_k changing along the path.
Running the LQR (Riccati) equation BACKWARDS from the end of the swing-up gives a gain K_k for every
control step; the command is  u = u_plan(t) − K_k·(s − s_plan(t)).  At the end the gains match the
ordinary balance LQR, so the hand-over is smooth.

Run:  uv run python double_control.py [trajectory name, e.g. 30T_30-20]
"""

from __future__ import annotations

import math
from collections import deque

import casadi as ca
import control
import mujoco
import numpy as np

import double_eom
import double_trajopt
import model
import motor
from plant import wrap

Q_BAL = np.diag([10.0, 200.0, 200.0, 1.0, 5.0, 5.0])   # balance weights over [x, a1, a2, xd, a1d, a2d]
R_BAL = np.array([[1.0]])
Q_TRK = np.diag([20.0, 50.0, 50.0, 1.0, 2.0, 2.0])     # tracking weights (looser on angles mid-swing)
R_TRK = np.array([[1.0]])


def jacobians(p: dict):
    f = double_eom.casadi_dynamics(p)
    s, u = ca.SX.sym("s", 6), ca.SX.sym("u")
    fx = f(s, u)
    return ca.Function("AB", [s, u], [ca.jacobian(fx, s), ca.jacobian(fx, u)])


def balance_gain(p: dict, rate_hz: float) -> np.ndarray:
    AB = jacobians(p)
    A, B = (np.array(M) for M in AB(np.zeros(6), 0.0))
    sysd = control.c2d(control.ss(A, B, np.eye(6), 0), 1.0 / rate_hz, method="zoh")
    K, S, _ = control.dlqr(sysd.A, sysd.B, Q_BAL, R_BAL)
    return np.asarray(K), np.asarray(S)


def tvlqr(p: dict, traj: dict, rate_hz: float, Q: np.ndarray | None = None, R: np.ndarray | None = None) -> dict:
    """Time-varying gains along the trajectory, sampled at the controller rate."""
    AB = jacobians(p)
    dt = 1.0 / rate_hz
    T = float(traj["T"])
    ts = np.arange(0.0, T, dt)
    X = np.array([np.interp(ts, traj["t"], traj["X"][i]) for i in range(6)]).T
    U = np.interp(ts, traj["t"], traj["U"])
    Qt = Q_TRK if Q is None else Q
    Rt = R_TRK if R is None else R
    K_end, S = balance_gain(p, rate_hz)
    Ks = np.zeros((len(ts), 6))
    Ads = np.zeros((len(ts), 6, 6)); Bds = np.zeros((len(ts), 6, 1))
    for k in range(len(ts) - 1, -1, -1):
        A, B = (np.array(M) for M in AB(X[k], U[k]))
        Ad = np.eye(6) + A * dt + A @ A * dt * dt / 2   # one-step discretisation (dt = 1-2 ms)
        Bd = B * dt + A @ B * dt * dt / 2
        K = np.linalg.solve(Rt + Bd.T @ S @ Bd, Bd.T @ S @ Ad)
        S = Qt + Ad.T @ S @ (Ad - Bd @ K)
        S = (S + S.T) / 2
        Ks[k] = K.ravel()
        Ads[k], Bds[k] = Ad, Bd
    return {"t": ts, "X": X, "U": U, "K": Ks, "K_bal": K_end.ravel(), "T": T, "Ad": Ads, "Bd": Bds, "dt": dt}


class SwingUpThenBalance:
    """mode 2 = following the swing-up plan, mode 1 = balance LQR (after the hand-over)."""

    def __init__(self, p: dict, gains: dict, end_angles=(0.0, 0.0)):
        self.g = gains
        self.end = np.array([0.0, *end_angles, 0.0, 0.0, 0.0])
        self.mode = 2
        self.F_coulomb = (p["cart"]["coulomb_friction"] + model.gearbox_drag(p)) / model.motor_gain(p)

    def __call__(self, t: float, s: np.ndarray) -> float:
        g = self.g
        if t < g["T"]:
            k = min(int(t * len(g["t"]) / g["T"]), len(g["t"]) - 1)
            err = s - g["X"][k]
            err[1], err[2] = wrap(err[1]), wrap(err[2])
            u = g["U"][k] - g["K"][k] @ err
            # friction feed-forward: the plan assumed no dry friction, so add the duty it costs
            u += self.F_coulomb * np.sign(g["X"][k][3]) if abs(g["X"][k][3]) > 0.01 else 0.0
            return float(u)
        self.mode = 1
        err = s - self.end
        err[1], err[2] = wrap(err[1]), wrap(err[2])
        return float(-g["K_bal"] @ err)


# ---------------------------------------------------------------------------------------------------------
# Kalman filter (estimates all six states from the three noisy position readings)
# ---------------------------------------------------------------------------------------------------------
# Measurement noise: angle sensor (AS5047P datasheet 0.068° RMS) and cart encoder quantisation (step/√12).
# Process noise: how much the real rig may differ from the model each step (friction, backlash, model error).
KF_W = np.diag([1e-8, 1e-8, 1e-8, 1e-4, 1e-3, 1e-3])


def kalman_gains(p: dict, gains: dict, W: np.ndarray = KF_W) -> dict:
    """Precompute Kalman gains along the plan (time-varying) and for balance (steady-state).
    Cheap on the Pico: at run time each cycle is a few small matrix-vector products."""
    s = p["sensing"]
    x_step = 2 * math.pi * model.pulley_radius(p) / s["cart_counts_per_rev"]
    ang = max(math.radians(s["angle_noise_deg"]), 2 * math.pi / 2 ** s["angle_bits"] / math.sqrt(12))
    V = np.diag([(x_step / math.sqrt(12)) ** 2, ang ** 2, ang ** 2])
    C = np.hstack([np.eye(3), np.zeros((3, 3))])
    P = np.diag([1e-6, 1e-4, 1e-4, 1e-4, 1e-2, 1e-2])
    L = np.zeros((len(gains["t"]), 6, 3))
    for k in range(len(gains["t"])):
        Pm = gains["Ad"][k] @ P @ gains["Ad"][k].T + W
        L[k] = Pm @ C.T @ np.linalg.inv(C @ Pm @ C.T + V)
        P = (np.eye(6) - L[k] @ C) @ Pm
    # balance: steady-state gain for the upright model (iterate the same recursion to convergence)
    AB = jacobians(p)
    A, B = (np.array(M) for M in AB(np.zeros(6), 0.0))
    sysd = control.c2d(control.ss(A, B, np.eye(6), 0), gains["dt"], method="zoh")
    Ab, Bb = np.asarray(sysd.A), np.asarray(sysd.B)
    for _ in range(3000):
        Pm = Ab @ P @ Ab.T + W
        Lb = Pm @ C.T @ np.linalg.inv(C @ Pm @ C.T + V)
        P = (np.eye(6) - Lb @ C) @ Pm
    return {"L": L, "L_bal": Lb, "A_bal": Ab, "B_bal": Bb, "C": C}


class SwingUpThenBalanceKF(SwingUpThenBalance):
    """Same controller, but it acts on Kalman-filtered states instead of differenced velocities.
    The filter tracks the ERROR from the plan (a small quantity, so the linear model holds)."""

    def __init__(self, p: dict, gains: dict, kf: dict, end_angles=(0.0, 0.0), delay_cycles: int = 1):
        super().__init__(p, gains, end_angles)
        self.kf = kf
        self.e = np.zeros(6)                 # estimated error from the plan (or from upright)
        # commands sent but possibly not yet acting: hist[0] is the one that acted over the last cycle
        self.hist = deque([0.0] * (delay_cycles + 1), maxlen=delay_cycles + 1)
        self.delay_cycles = delay_cycles
        self.k = 0

    def _ref(self, k):
        g = self.g
        if k < len(g["t"]):
            return g["X"][k], g["U"][k], g["Ad"][k], g["Bd"][k], self.kf["L"][k], g["K"][k]
        return self.end, 0.0, self.kf["A_bal"], self.kf["B_bal"], self.kf["L_bal"], g["K_bal"]

    def __call__(self, t: float, s: np.ndarray) -> float:
        k = self.k
        x_ref, u_ref, A, B, L, K = self._ref(k)
        # 1) predict: last cycle's error, pushed through the model with the command that actually acted
        if k > 0:
            xp, up, Ap, Bp, _, _ = self._ref(k - 1)
            u_acting = self.hist[0]
            e_pred = Ap @ self.e + (Bp.ravel() * (u_acting - up))
        else:
            e_pred = self.e
        # 2) correct with the measured positions
        meas_err = s[:3] - x_ref[:3]
        meas_err[1], meas_err[2] = wrap(meas_err[1]), wrap(meas_err[2])
        self.e = e_pred + L @ (meas_err - e_pred[:3])
        # 3) control on the estimate
        self.mode = 2 if k < len(self.g["t"]) else 1
        u = u_ref - K @ self.e
        if self.mode == 2 and abs(x_ref[3]) > 0.01:
            u += self.F_coulomb * np.sign(x_ref[3])
        u = float(np.clip(u, -1.0, 1.0))
        self.hist.append(u)
        self.k += 1
        return u


class DoublePlant:
    """The double rig as the Pico sees it: fixed rate, delay, quantised sensors, filtered velocities."""

    def __init__(self, p: dict, rate_hz: float = 1000.0, seed: int = 0, delay: float | None = None):
        self.p = p
        self.m = model.build_double(p)
        self.d = mujoco.MjData(self.m)
        self.dt = self.m.opt.timestep
        self.n = max(1, round(1.0 / (rate_hz * self.dt)))
        self.ctrl_dt = self.n * self.dt
        delay = p["sim"]["control_delay"] if delay is None else delay
        self.delay_steps = round(delay / self.dt)
        s = p["sensing"]
        self.steps = np.array([2 * math.pi * model.pulley_radius(p) / s["cart_counts_per_rev"],
                               2 * math.pi / 2 ** s["angle_bits"], 2 * math.pi / 2 ** s["angle_bits"]])
        self.alpha = self.ctrl_dt / (self.ctrl_dt + 1 / (2 * math.pi * s["velocity_filter_hz"]))
        self.cap = p["supply"]["current_cap"]
        self.noise = math.radians(s["angle_noise_deg"])
        self.rng = np.random.default_rng(seed)

    def reset(self, q=(0.0, math.pi, 0.0), qd=(0.0, 0.0, 0.0)):
        mujoco.mj_resetData(self.m, self.d)
        self.d.qpos[:], self.d.qvel[:] = q, qd
        mujoco.mj_forward(self.m, self.d)
        self.prev = np.round(np.array(q) / self.steps) * self.steps
        self.pos = self.prev.copy()   # continuous (unwrapped) measured positions
        self.vel = np.zeros(3)
        self.queue = deque([0.0] * self.delay_steps)

    def measure(self) -> np.ndarray:
        raw = np.array(self.d.qpos[:3])
        if self.noise:
            raw[1:] += self.rng.normal(0, self.noise, 2)
        # the angle sensors only report 0..2π: wrap them, then unwrap by accumulating the change
        meas = np.round(raw / self.steps) * self.steps
        delta = meas - self.prev
        delta[1:] = [wrap(v) for v in delta[1:]]
        self.prev = meas
        self.pos = self.pos + delta
        self.vel += self.alpha * (delta / self.ctrl_dt - self.vel)
        return np.concatenate([self.pos, self.vel])

    def run(self, controller, seconds: float, stop=None) -> dict:
        log = {k: [] for k in ("t", "q", "qd", "duty", "i_supply", "mode")}
        duty_cmd = 0.0
        for k in range(round(seconds / self.dt)):
            if k % self.n == 0:
                duty_cmd = float(np.clip(controller(self.d.time, self.measure()), -1.0, 1.0))
            self.queue.append(duty_cmd)
            duty = motor.cap_duty_for_supply(self.p, self.queue.popleft(), self.d.qvel[0], self.cap)
            self.d.ctrl[0] = duty
            log["t"].append(self.d.time); log["q"].append(self.d.qpos[:3].copy()); log["qd"].append(self.d.qvel[:3].copy())
            log["duty"].append(duty); log["i_supply"].append(motor.supply_current(self.p, duty, self.d.qvel[0]))
            log["mode"].append(getattr(controller, "mode", 0))
            mujoco.mj_step(self.m, self.d)
            if stop is not None and stop(self.d):
                break
        return {k: np.asarray(v) for k, v in log.items()}


def trial(p: dict, traj: dict, rate_hz: float = 1000.0, seconds: float | None = None, gains=None,
          start=(0.0, math.pi, 0.0), seed: int = 0, kf: dict | None = None) -> dict:
    """kf = Kalman gains from kalman_gains() to use the Kalman-filtered controller; None = plain differencing."""
    gains = gains or tvlqr(p, traj, rate_hz)
    end = (traj["X"][1, -1], traj["X"][2, -1])
    plant = DoublePlant(p, rate_hz=rate_hz, seed=seed)
    plant.reset(q=start)
    seconds = seconds or gains["T"] + 5.0
    if kf is None:
        ctrl = SwingUpThenBalance(p, gains, end)
    else:
        ctrl = SwingUpThenBalanceKF(p, gains, kf, end, delay_cycles=round(plant.delay_steps / plant.n))
    log = plant.run(ctrl, seconds, stop=lambda d: abs(d.qpos[0]) > 0.395)
    a1 = np.array([wrap(v) for v in log["q"][:, 1]])
    a2 = np.array([wrap(v) for v in log["q"][:, 2]])
    tail = log["t"] >= log["t"][-1] - 2.0
    finished = log["t"][-1] >= seconds - 2 * plant.dt
    ok = bool(finished and np.all(np.abs(a1[tail]) < math.radians(10)) and np.all(np.abs(a2[tail]) < math.radians(10)))
    return {"ok": ok, "log": log, "max_x": float(np.abs(log["q"][:, 0]).max()),
            "peak_supply_A": float(log["i_supply"].max()), "gains": gains}


if __name__ == "__main__":
    import sys
    name = sys.argv[1] if len(sys.argv) > 1 else "30T_30-20"
    teeth, rods = name.split("_")
    bottom, top = (int(v) / 100 for v in rods.split("-"))
    p = double_trajopt.layout_params(int(teeth[:-1]), bottom, top)
    traj = double_trajopt.load(name)
    r = trial(p, traj)
    print(f"{name}: {'BALANCED' if r['ok'] else 'FAILED'} after a {traj['T']:.2f} s swing-up; "
          f"max|x| {r['max_x'] * 1000:.0f} mm, peak supply {r['peak_supply_A']:.1f} A")
