"""PLAN 5.4 / SPEC §12: choose the test-protocol numbers from the sim, measuring BOTH controllers under identical
conditions (same rig, sensor noise, supply limit, safety slew, and software kicks added to the duty the same way
the Pico does). Writes analysis/PROTOCOL-draft.md.

Run now on the datasheet sim for a draft; re-run after Phase 4 on the fitted sim for the numbers that get fixed.
  uv run python protocol_numbers.py
"""
from __future__ import annotations

import math

import numpy as np

import lqr
import model
import swingup
from plant import Plant, wrap

W = model.SIM_DIR.parent / "firmware" / "nn" / "weights"
P = model.with_params(model.load_params(), supply__current_cap=4.5, sensing__angle_noise_deg=0.068)
K = lqr.design(P, 500)
SLEW = 0.25
KICK_DUTY = 1.0                     # software kick: the duty is OVERRIDDEN to +1.0 for D ms (a cart jolt), like the
                                    # Pico's KICK; an additive kick is mostly cancelled by the controller pushing back


def policy_from_bin(path):
    f = np.frombuffer(path.read_bytes()[16:], dtype="<f4")
    w1, b1 = f[:384].reshape(64, 6), f[384:448]
    w2, b2 = f[448:4544].reshape(64, 64), f[4544:4608]
    w3, b3 = f[4608:4672], f[4672]
    return lambda o: math.tanh(float(w3 @ np.maximum(w2 @ np.maximum(w1 @ o + b1, 0) + b2, 0) + b3))


class AI:
    """The Pico's AiController (firmware/core/control.h), in Python."""
    def __init__(self):
        self.sw, self.bal = policy_from_bin(W / "swingupE_seed3.bin"), policy_from_bin(W / "balanceGc_seed4.bin")
        self.alpha, self.mode, self.k, self.action, self.duty = 1 / (1 + 500 / (2 * math.pi * 20)), 2, 0, 0.0, 0.0

    def __call__(self, t, s):
        if self.k % 5 == 0:
            obs = np.array([s[0] / 0.4, s[1], math.sin(s[2]), math.cos(s[2]), s[3] / 10, self.duty], dtype=np.float32)
            th = math.atan2(obs[2], obs[3])
            if self.mode == 2 and abs(th) < math.radians(15): self.mode = 1
            elif self.mode == 1 and abs(th) > math.radians(35): self.mode = 2
            self.action = (self.bal if self.mode == 1 else self.sw)(obs)
        self.k += 1
        self.duty += self.alpha * (self.action - self.duty)
        return self.duty


def make(name, start_balanced=False):
    if name == "classical":
        c = swingup.EnergySwingUp(P, K)
        if start_balanced:
            c.mode = 1
        return c
    a = AI()
    if start_balanced:
        a.mode = 1
    return a


def run(name, seed, seconds, start, kick=None):
    """kick = (start_s, duration_s): the duty is overridden to KICK_DUTY, like the Pico's KICK. Returns the log."""
    ctrl = make(name, start_balanced=abs(start[1]) < 1)
    last = [0.0]
    def c(t, obs):
        u = float(np.clip(ctrl(t, obs), -1, 1))
        if kick and kick[0] <= t < kick[0] + kick[1]:
            u = KICK_DUTY
        u = float(np.clip(u, last[0] - SLEW, last[0] + SLEW)); last[0] = u
        return u
    c.mode = 0
    pl = Plant(P, rate_hz=500, seed=seed)
    pl.reset(x=start[0], th=start[1])
    log = pl.run(c, seconds, stop=lambda t, d: abs(d.qpos[0]) > 0.395)
    log["th_w"] = np.array([wrap(a) for a in log["th"]])
    return log


def upright_since(log, band=math.radians(10)):
    """Time from which the rod stays within `band` to the end (nan if never)."""
    bad = np.where(np.abs(log["th_w"]) >= band)[0]
    if len(bad) == 0:
        return 0.0
    return log["t"][bad[-1] + 1] if bad[-1] + 1 < len(log["t"]) else math.nan


def main():
    out = {}
    rng = np.random.default_rng(0)
    for name in ("classical", "ai"):
        # 1. swing-up times: 20 starts from hanging with small offsets, 15 s each
        times = []
        for s in range(20):
            L = run(name, s, 15.0, (rng.uniform(-0.02, 0.02), math.pi + rng.uniform(-0.03, 0.03)))
            ok = L["t"][-1] >= 14.99
            times.append(upright_since(L) if ok else math.nan)
        # 2. noise floor while balancing still (no kick)
        L = run(name, 99, 10.0, (0.0, 0.0))
        tail = L["t"] > 2
        floor = (float(np.sqrt(np.mean(L["th_w"][tail] ** 2))), float(np.sqrt(np.mean(L["x"][tail] ** 2))))
        # 3. kick ladder: duty overridden to +1.0 for D ms, 10 trials per rung (small start offsets)
        ladder = {}
        for D in range(20, 601, 20):
            passed = 0
            for s in range(10):
                L = run(name, 1000 + s, 6.0, (rng.uniform(-0.01, 0.01), rng.uniform(-0.01, 0.01)), kick=(1.0, D / 1000))
                # survived = recovered by the BALANCE controller: never past the 35° hand-back angle, upright at the
                # end (a bigger kick that knocks the rod down and gets re-swung-up would otherwise count as survived)
                passed += bool(L["t"][-1] >= 5.99 and np.max(np.abs(L["th_w"])) < math.radians(35)
                               and abs(L["th_w"][-1]) < math.radians(10))
            ladder[D] = int(passed)
            if passed == 0:
                break
        out[name] = {"swing_times": times, "floor": floor, "ladder": ladder}
        print(name, "swing-up times", np.round(times, 2), "floor θ/x RMS", np.round(floor, 4), "ladder", ladder, flush=True)

    # --- choose the numbers so neither controller is favoured ---
    all_t = np.array([t for v in out.values() for t in v["swing_times"]])
    t_ok = all_t[~np.isnan(all_t)]
    time_limit = math.ceil(1.5 * np.percentile(t_ok, 95))
    hold = 5.0
    band = max(math.radians(2.0), 3 * max(v["floor"][0] for v in out.values()))
    survived = {n: max([D for D, k in v["ladder"].items() if k >= 8], default=0) for n, v in out.items()}
    standard_kick = max(20, int(round(0.5 * min(survived.values()) / 20)) * 20)
    top = max(max(v["ladder"]) for v in out.values())
    rungs = list(range(20, top + 21, 10))    # 10 ms steps on hardware: the sim's 20 ms steps tie both at 60 ms

    # 4. settling time and effort at the standard kick, 20 trials each
    settle = {}
    for name in out:
        ts, eff = [], []
        for s in range(20):
            L = run(name, 2000 + s, 6.0, (0.0, 0.0), kick=(1.0, standard_kick / 1000))
            t_end = 1.0 + standard_kick / 1000
            w = L["t"] >= t_end
            outside = np.where(np.abs(L["th_w"][w]) >= band)[0]
            ts.append((L["t"][w][outside[-1]] - t_end) if len(outside) else 0.0)
            win = w & (L["t"] < t_end + 3.0)
            eff.append(float(np.sqrt(np.mean(L["duty"][win] ** 2))))
        settle[name] = (float(np.median(ts)), float(np.median(eff)))

    md = f"""# Test protocol — DRAFT from the datasheet sim (PLAN 5.4, SPEC §12)

**Status: draft, not fixed.** Generated {np.datetime64('today')} by `sim/protocol_numbers.py` on the datasheet sim
(sensor noise 0.068°, 5 A limiter at 4.5 A, slew 0.25/cycle, both controllers through the same pipeline).
**Re-run on the fitted sim after Phase 4, review, then commit as `analysis/PROTOCOL.md` by Sun 11 Oct. After
that commit the numbers do not change.**

## Numbers (chosen so that neither controller is favoured)

| Item | Value | How it was chosen |
|---|---|---|
| Swing-up time limit | **{time_limit} s** | 1.5 × the 95th percentile of both controllers' swing-up times together |
| Hold time | **{hold:.0f} s** | Balanced (|θ| < 10°) for this long after catching counts as a success |
| Settle band | **±{math.degrees(band):.1f}°** | max(2°, 3 × the larger controller's resting θ RMS) |
| Settling time | time from the end of the standard kick until θ stays inside the band | – |
| Software kick | **duty overridden to +{KICK_DUTY} for D ms** (a cart jolt; Pico KICK) | fixed amplitude, duration varies |
| Standard kick (settling, effort) | **D = {standard_kick} ms** | half the smaller controller's largest kick survived, rounded to 20 ms |
| Kick ladder rungs | **D = {', '.join(map(str, rungs))} ms** | 20 ms to one rung past where both fail, in 10 ms steps (the sim's 20 ms steps tied both controllers) |
| Largest push survived | largest rung recovered in ≥ 8 of 10, where recovered = the rod never passes 35° (the hand-back angle) and is upright at the end | SPEC §12; a knocked-down rod that is re-swung-up does not count |
| Motor effort | RMS duty over the 3 s after the standard kick | SPEC §12 |
| Trials | swing-up 20 per controller; settling 20 per controller; 10 per ladder rung per controller | SPEC §12 |
| Order | controllers **alternate** trial by trial (A, B, A, B …), same rig, same day, same rod | SPEC §12 |
| Recorded per trial | raw telemetry at 500 Hz (t, x, ẋ, θ, θ̇, duty, mode, faults, loop timing) → `analysis/data/` | – |

## What the sim predicts (datasheet model; a sanity check, not a result)

| | Classical | AI |
|---|---|---|
| Swing-up success (of 20) | {sum(not math.isnan(t) for t in out['classical']['swing_times'])} | {sum(not math.isnan(t) for t in out['ai']['swing_times'])} |
| Swing-up time, median | {np.nanmedian(out['classical']['swing_times']):.2f} s | {np.nanmedian(out['ai']['swing_times']):.2f} s |
| Resting θ RMS / x RMS | {math.degrees(out['classical']['floor'][0]):.2f}° / {out['classical']['floor'][1]*1000:.1f} mm | {math.degrees(out['ai']['floor'][0]):.2f}° / {out['ai']['floor'][1]*1000:.1f} mm |
| Largest kick survived (≥ 8/10) | {survived['classical']} ms | {survived['ai']} ms |
| Settling time (median, standard kick) | {settle['classical'][0]:.2f} s | {settle['ai'][0]:.2f} s |
| Effort (median RMS duty) | {settle['classical'][1]:.3f} | {settle['ai'][1]:.3f} |

Ladder passes per rung (of 10): classical {out['classical']['ladder']}; AI {out['ai']['ladder']}.

## Still to add before fixing

- Physical tapper settings (PLAN 7.1): servo angle, strike height, and its consistency check (CV < 10%).
- Anything tuned on hardware must be reported (SPEC §12, honest reporting).
"""
    path = model.SIM_DIR.parent / "analysis" / "PROTOCOL-draft.md"
    path.parent.mkdir(exist_ok=True)
    path.write_text(md)
    print(md)


if __name__ == "__main__":
    main()
