"""SPEC §11 Q3 (rod length) and Q4 (loop rate) for the single.

Q3: 0.30 m vs 0.40 m rod (aluminium, mass scales with length), on both pulleys: swing-up (10 runs) and
    the largest tap LQR survives. Keep 0.30 m unless 0.40 m is clearly better (PLAN 1.6).
Q4: LQR at 100/200/500/1000 Hz with the 2 ms delay: largest tap survived, swing-up success, and how
    noisy the motor command is while balancing still (quantisation noise grows with rate).
    Pick the lowest rate that keeps full robustness, ×2 for margin (PLAN 1.6).

Run:  uv run python q3_q4.py
"""

from __future__ import annotations

import json
import math

import numpy as np

import lqr
import model
import q2_pulley
import swingup
from plant import Plant

AL_KG_PER_M = 0.067  # 8 mm × 1 mm aluminium tube


def rod(p: dict, length: float) -> dict:
    return model.with_params(p, pole__length=length, pole__mass=AL_KG_PER_M * length)


def q3() -> dict:
    out = {}
    for teeth in (30, 20):
        for length in (0.30, 0.40):
            p = rod(model.load_params(**{"drive.pulley_teeth": teeth}), length)
            out[f"{teeth}T {length:.2f} m"] = row = q2_pulley.evaluate(p)
            print(f"Q3 {teeth}T {length:.2f} m: swing-up {row['success']}/10 (5 A cap {row['success_capped_5A']}/10), "
                  f"catch {row['catch_s']:.2f}s, max|x| {row['max_x_mm']:.0f}mm, largest tap {row['largest_tap_rad_s']:.2f} rad/s",
                  flush=True)
    return out


def duty_noise(p: dict, K: np.ndarray, rate: float) -> float:
    """Chatter: RMS change in duty between consecutive commands while recovering from 5°.
    (Perfectly still upright shows none: the sensors never change count and friction holds the cart.)"""
    plant = Plant(p, rate_hz=rate)
    plant.reset(th=math.radians(5))
    log = plant.run(lqr.LQRController(K), 3.0)
    cmds = log["duty"][:: plant.steps_per_ctrl]
    return float(np.sqrt(np.mean(np.diff(cmds) ** 2)))


def q4() -> dict:
    out = {}
    for teeth in (30, 20):
        p = model.load_params(**{"drive.pulley_teeth": teeth})
        for rate in (100, 200, 500, 1000):
            K = lqr.design(p, rate)
            tap = q2_pulley.largest_tap(p, K, rate_hz=rate)
            rs = [swingup.swingup_trial(p, rate_hz=rate, seed=s, K=K) for s in range(10)]
            noise = duty_noise(p, K, rate)
            out[f"{teeth}T {rate} Hz"] = {"largest_tap_rad_s": tap, "swingup": sum(r["ok"] for r in rs), "duty_chatter_rms": noise}
            print(f"Q4 {teeth}T {rate:4d} Hz: largest tap {tap:.2f} rad/s, swing-up {sum(r['ok'] for r in rs)}/10, "
                  f"duty chatter {noise:.3f} RMS step-to-step", flush=True)
    return out


if __name__ == "__main__":
    res = {"q3": q3(), "q4": q4()}
    (model.SIM_DIR / "results" / "q3_q4.json").write_text(json.dumps(res, indent=2))
