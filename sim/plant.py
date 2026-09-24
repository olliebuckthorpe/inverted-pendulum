"""The simulated rig as the Pico will experience it (PLAN 1.3–1.5).

MuJoCo does the physics at 1 kHz. On top of that this adds what a real digital controller lives with:

- a fixed control rate, with the command held constant between updates (a "zero-order hold");
- a delay between measuring and acting (params sim.control_delay);
- quantised sensors (motor encoder counts, 14-bit angle) and velocities estimated by differencing;
- optional gearbox backlash + belt stretch (the compliant model);
- the optional PSU current cap, and logging of motor and supply current.

A controller is any function  (t, obs) -> duty  where obs = [x, x_dot, theta, theta_dot] and theta is
wrapped to (-pi, pi], 0 = upright.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field

import mujoco
import numpy as np

import model
import motor


def wrap(angle: float) -> float:
    """Wrap an angle to (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


class Estimator:
    """What the firmware will do: quantise the sensors, difference them, low-pass the result."""

    def __init__(self, p: dict, dt: float, rng: np.random.Generator):
        s = p["sensing"]
        self.x_step = 2 * math.pi * model.pulley_radius(p) / s["cart_counts_per_rev"]
        self.th_step = 2 * math.pi / 2 ** s["angle_bits"]
        self.noise = math.radians(s["angle_noise_deg"])
        self.dt = dt
        # First-order low-pass: y += a·(u − y), a = dt / (dt + 1/(2π·fc))
        self.a = dt / (dt + 1 / (2 * math.pi * s["velocity_filter_hz"]))
        self.rng = rng
        self.prev = None
        self.v = np.zeros(2)

    def __call__(self, x_true: float, th_true: float) -> np.ndarray:
        x = round(x_true / self.x_step) * self.x_step
        th = th_true + (self.rng.normal(0, self.noise) if self.noise else 0.0)
        th = round(th / self.th_step) * self.th_step
        if self.prev is None:
            self.prev = (x, th)
        raw = np.array([(x - self.prev[0]) / self.dt, wrap(th - self.prev[1]) / self.dt])
        self.v += self.a * (raw - self.v)
        self.prev = (x, th)
        return np.array([x, self.v[0], wrap(th), self.v[1]])


@dataclass
class Log:
    t: list = field(default_factory=list)
    x: list = field(default_factory=list)
    xd: list = field(default_factory=list)
    th: list = field(default_factory=list)   # unwrapped, true
    thd: list = field(default_factory=list)
    duty: list = field(default_factory=list)
    i_motor: list = field(default_factory=list)
    i_supply: list = field(default_factory=list)
    mode: list = field(default_factory=list)

    def arrays(self) -> dict:
        return {k: np.asarray(v) for k, v in self.__dict__.items()}


class Plant:
    def __init__(self, p: dict, rate_hz: float = 500.0, compliant: bool = False, seed: int = 0,
                 delay: float | None = None):
        self.p = p
        self.compliant = compliant
        self.m = model.build_single(p, compliant=compliant)
        self.d = mujoco.MjData(self.m)
        self.dt = self.m.opt.timestep
        self.steps_per_ctrl = max(1, round(1.0 / (rate_hz * self.dt)))
        self.rate_hz = 1.0 / (self.steps_per_ctrl * self.dt)
        delay = p["sim"]["control_delay"] if delay is None else delay
        self.delay_steps = round(delay / self.dt)
        self.rng = np.random.default_rng(seed)
        self.k_belt = p["drive"]["belt_stiffness"]
        self.c_belt = p["drive"]["belt_damping"]
        self.gap = model.backlash_halfwidth(p)
        self.cap = p["supply"]["current_cap"]

    # MuJoCo joint indices: 0 = cart slider, 1 = hinge, 2 = drive (compliant only)
    def reset(self, x=0.0, th=0.0, xd=0.0, thd=0.0):
        mujoco.mj_resetData(self.m, self.d)
        self.d.qpos[0], self.d.qpos[1] = x, th
        self.d.qvel[0], self.d.qvel[1] = xd, thd
        if self.compliant:
            self.d.qpos[2], self.d.qvel[2] = x, xd
        mujoco.mj_forward(self.m, self.d)
        self.est = Estimator(self.p, self.steps_per_ctrl * self.dt, self.rng)
        self.queue = deque([0.0] * self.delay_steps)

    def drive_speed(self) -> float:
        return self.d.qvel[2] if self.compliant else self.d.qvel[0]

    def _belt_force(self) -> float:
        """Spring with a dead-band: no force until the gearbox play is taken up."""
        stretch = self.d.qpos[2] - self.d.qpos[0]
        if stretch > self.gap:
            spring = self.k_belt * (stretch - self.gap)
        elif stretch < -self.gap:
            spring = self.k_belt * (stretch + self.gap)
        else:
            spring = 0.0
        return spring + self.c_belt * (self.d.qvel[2] - self.d.qvel[0])

    def run(self, controller, seconds: float, stop=None, disturb=None) -> dict:
        """Run `controller` for `seconds`. `stop(t, state)` may end the run early.
        `disturb(t)` may return an angular-velocity kick (rad/s) to add to the pole, like a tap."""
        log = Log()
        duty_cmd = 0.0
        n_steps = round(seconds / self.dt)
        for k in range(n_steps):
            t = self.d.time
            if k % self.steps_per_ctrl == 0:
                obs = self.est(self.d.qpos[0], self.d.qpos[1])
                duty_cmd = float(np.clip(controller(t, obs), -1.0, 1.0))
            self.queue.append(duty_cmd)
            duty = self.queue.popleft()
            duty = motor.cap_duty_for_supply(self.p, duty, self.drive_speed(), self.cap)
            self.d.ctrl[0] = duty
            if self.compliant:
                f = self._belt_force()
                self.d.qfrc_applied[0], self.d.qfrc_applied[2] = f, -f
            if disturb is not None:
                self.d.qvel[1] += disturb(t)
            v = self.drive_speed()
            log.t.append(t); log.x.append(self.d.qpos[0]); log.xd.append(self.d.qvel[0])
            log.th.append(self.d.qpos[1]); log.thd.append(self.d.qvel[1]); log.duty.append(duty)
            log.i_motor.append(motor.motor_current(self.p, duty, v))
            log.i_supply.append(motor.supply_current(self.p, duty, v))
            log.mode.append(getattr(controller, "mode", 0))
            mujoco.mj_step(self.m, self.d)
            if stop is not None and stop(self.d.time, self.d):
                break
        return log.arrays()
