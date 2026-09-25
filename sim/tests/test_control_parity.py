"""The Pico's control logic (firmware/core/control.h) must reproduce the Python sim's, step by step, on sequences
recorded from simulated runs: estimator, classical swing-up → LQR, AI hand-over + smoothing, supply limiter."""
import math
import shutil
import subprocess

import numpy as np
import pytest

import lqr
import model
import motor
import swingup
from plant import Estimator, wrap

FW = model.SIM_DIR.parent / "firmware"
W = FW / "nn" / "weights"
pytestmark = pytest.mark.skipif(not shutil.which("clang++"), reason="needs clang++")
P = model.with_params(model.load_params(), supply__current_cap=4.5)


def classical_params(ctrl, K):
    return {"K0": K[0], "K1": K[1], "K2": K[2], "K3": K[3], "m": ctrl.m, "l": ctrl.l, "J": ctrl.J,
            "M_total": ctrl.M_total, "G": ctrl.G, "D": ctrl.D, "F_coulomb": ctrl.F_coulomb, "est_dt": 0.002,
            "V": P["supply"]["voltage"], "R": P["motor"]["R"], "Ke": P["motor"]["Ke"], "r": model.pulley_radius(P),
            "cap": 4.5, "ai_catch": math.radians(15)}


@pytest.fixture(scope="module")
def harness(tmp_path_factory):
    d = tmp_path_factory.mktemp("c")
    exe = d / "control_cli"
    subprocess.run(["clang++", "-std=c++17", "-O2", "-Wall", "-Wno-unused-result", str(FW / "tests" / "control_cli.cpp"), "-o", str(exe)], check=True)
    K = lqr.design(P, 500).ravel()
    ctrl = swingup.EnergySwingUp(P, K)
    (d / "params.txt").write_text("\n".join(f"{k} {float(v)!r}" for k, v in classical_params(ctrl, K).items()))
    proc = subprocess.Popen([str(exe), str(d / "params.txt"), str(W / "swingupE_seed3.bin"), str(W / "balanceGc_seed4.bin")],
                            stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True)
    yield proc, K
    proc.kill()


def ask(proc, line):
    proc.stdin.write(line + "\n"); proc.stdin.flush()
    out = proc.stdout.readline().split()
    return out if out == ["ok"] else [float(v) for v in out]


def recorded_run():
    """True x, θ at 500 Hz from a classical swing-up in the sim, quantised like the sensors."""
    r = swingup.swingup_trial(P, seed=3)
    L = r["log"]
    est = Estimator(P, 0.002, np.random.default_rng(0))
    xq = [round(x / est.x_step) * est.x_step for x in L["x"][::2]]
    thq = [round(t / est.th_step) * est.th_step for t in L["th"][::2]]
    return np.array(xq), np.array(thq)


def test_estimator_matches(harness):
    proc, _ = harness
    xq, thq = recorded_run()
    ref = Estimator(P, 0.002, np.random.default_rng(0))
    worst = 0.0
    for x, th in zip(xq[:3000], thq[:3000]):
        want = ref(x, th)
        got = np.array(ask(proc, f"E {float(x)!r} {float(th)!r}"))
        diff = got - want
        diff[2] = wrap(diff[2])                                   # +π and −π are the same angle
        worst = max(worst, np.abs(diff).max())
    assert worst < 1e-3, worst          # float32 on the Pico vs float64 here


def test_classical_matches(harness):
    proc, K = harness
    xq, thq = recorded_run()
    est = Estimator(P, 0.002, np.random.default_rng(0))
    ref = swingup.EnergySwingUp(P, K.reshape(1, 4))
    mism = worst = 0
    for x, th in zip(xq[:3000], thq[:3000]):
        s = est(x, th)
        want = ref(0.0, s)
        got_u, got_mode = ask(proc, "C " + " ".join(repr(float(v)) for v in s))
        worst = max(worst, abs(got_u - want))
        mism += int(got_mode) != ref.mode
    assert mism == 0 and worst < 1e-3, (mism, worst)


def test_classical_handover_edge_cases(harness):
    """States the recorded run never visits: near the top but spinning too fast to catch (must keep pumping),
    slow near the top (catch), and falling past the drop angle after a catch (hand back)."""
    proc, K = harness
    cases = [(0.0, 0.0, 0.10, 12.0), (0.0, 0.0, 0.30, 0.2), (0.0, 0.0, 0.10, 0.5), (0.0, 0.0, 0.20, 1.0),
             (0.0, 0.0, 0.70, 2.0), (0.0, 0.0, 0.25, -9.0), (0.05, 0.1, -0.05, -0.3)]
    ask(proc, "R")
    ref = swingup.EnergySwingUp(P, K.reshape(1, 4))
    for s in cases:
        want = ref(0.0, np.array(s))
        got_u, got_mode = ask(proc, "C " + " ".join(repr(float(v)) for v in s))
        assert int(got_mode) == ref.mode, (s, got_mode, ref.mode)
        assert abs(got_u - want) < 1e-3, (s, got_u, want)
    ask(proc, "R")


def policy_from_bin(path):
    f = np.frombuffer(path.read_bytes()[16:], dtype="<f4")
    i = 0
    def take(n, shape):
        nonlocal i
        a = f[i:i + n].reshape(shape); i += n; return a
    w1, b1 = take(384, (64, 6)), take(64, (64,))
    w2, b2 = take(4096, (64, 64)), take(64, (64,))
    w3, b3 = take(64, (64,)), take(1, (1,))[0]
    f32 = np.float32
    # float32 throughout, like the Pico: a steep policy amplifies float64-vs-float32 input differences
    return lambda o: math.tanh(float(np.float32(w3 @ np.maximum(w2 @ np.maximum(w1 @ o.astype(f32) + b1, f32(0)) + b2, f32(0)) + b3)))


def test_ai_controller_matches(harness):
    """Step-by-step parity. The controller's observation includes its own previous duty, so each side feeding
    back its own float32/float64 duty would compound rounding over hundreds of steps; instead every step
    starts from the C++'s previous duty and compares the decision (mode, action) and the smoothing update."""
    proc, _ = harness
    xq, thq = recorded_run()
    est = Estimator(P, 0.002, np.random.default_rng(0))
    sw, bal = policy_from_bin(W / "swingupE_seed3.bin"), policy_from_bin(W / "balanceGc_seed4.bin")
    alpha = 1 / (1 + 500 / (2 * math.pi * 20))
    mode, prev_duty, action = "swing", 0.0, 0.0
    worst_action = worst_duty = 0.0
    mism = handovers = 0
    for k, (x, th) in enumerate(zip(xq[:3000], thq[:3000])):
        s = np.asarray(est(x, th), dtype=np.float32)            # the Pico's state is float32
        duty, got_mode, got_action = ask(proc, "A " + " ".join(repr(float(v)) for v in s))
        if k % 5 == 0:                                          # a decision step: observe, decide
            obs = np.array([s[0] / np.float32(0.4), s[1], np.sin(s[2]), np.cos(s[2]), s[3] / np.float32(10), prev_duty],
                           dtype=np.float32)
            t = math.atan2(obs[2], obs[3])
            before = mode
            if mode == "swing" and abs(t) < math.radians(15): mode = "balance"
            elif mode == "balance" and abs(t) > math.radians(35): mode = "swing"
            handovers += before != mode
            action = (bal if mode == "balance" else sw)(obs)
            worst_action = max(worst_action, abs(got_action - action))
        mism += (got_mode == 1) != (mode == "balance")
        worst_duty = max(worst_duty, abs(duty - (prev_duty + alpha * (got_action - prev_duty))))
        prev_duty = duty
    assert handovers >= 1                                       # the sequence really exercises the hand-over
    assert mism == 0 and worst_action < 1e-4 and worst_duty < 1e-6, (mism, worst_action, worst_duty)


def test_supply_limiter_matches(harness):
    proc, _ = harness
    rng = np.random.default_rng(5)
    for _ in range(500):
        duty, v = rng.uniform(-1, 1), rng.uniform(-1.8, 1.8)
        want = motor.cap_duty_for_supply(P, duty, v, 4.5)
        assert abs(ask(proc, f"L {float(duty)!r} {float(v)!r}")[0] - want) < 1e-5
