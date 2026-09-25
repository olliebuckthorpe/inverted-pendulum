"""Choose the AI controller fairly, then score it once (PLAN 5.3).

  1. Validation (100 rigs, seeds 60 000+): every swing-up × catch-balance pair at hand-over 10° and 15°.
  2. Pick the best pair on validation only.
  3. Hold-out (200 different rigs, seeds 90 000+), scored once: with realistic varied noise (0.67–1.5×
     datasheet) and at exactly the datasheet noise (0.068°).
Writes results/ai/selection.json. Usage: uv run python select_ai.py
"""
import itertools
import json

import torch
from stable_baselines3 import SAC

import ai_handover as H
import envs.pendulum_env as E
import model

torch.set_num_threads(1)
ck = model.SIM_DIR / "checkpoints"
swings = sorted(f.stem for f in ck.glob("swingup*.zip"))
catches = sorted(f.stem for f in ck.glob("balanceGc_seed*.zip"))
load = lambda t: SAC.load(ck / t, device="cpu")
S, B = {t: load(t) for t in swings}, {t: load(t) for t in catches}

val = {}
for s, b, deg in itertools.product(swings, catches, (10.0, 15.0)):
    ok, _ = H.evaluate(S[s], B[b], deg, n=100, seed0=60_000)
    val[f"{s} -> {b} @ {deg:.0f}°"] = ok
    print(f"validation  {s:16s} -> {b:16s} @ {deg:.0f}°: {ok}/100", flush=True)
best = max(val, key=val.get)
s, rest = best.split(" -> ")
b, deg = rest.split(" @ ")
deg = float(deg.rstrip("°"))
print(f"\nchosen on validation: {best} ({val[best]}/100)", flush=True)

hold_varied, _ = H.evaluate(S[s], B[b], deg, n=200, seed0=90_000)
orig = E.PendulumEnv.__init__
def fixed_noise(self, *a, **k):
    k["noise_deg"] = 0.068
    orig(self, *a, **k)
E.PendulumEnv.__init__ = fixed_noise
hold_fixed, _ = H.evaluate(S[s], B[b], deg, n=200, seed0=90_000)
E.PendulumEnv.__init__ = orig
print(f"HOLD-OUT (200 rigs, scored once): varied noise {hold_varied}/200 = {hold_varied / 2:.1f}%, "
      f"datasheet noise {hold_fixed}/200 = {hold_fixed / 2:.1f}%", flush=True)
(model.SIM_DIR / "results" / "ai" / "selection.json").write_text(json.dumps(
    {"validation": val, "chosen": best, "holdout_varied_noise": hold_varied, "holdout_datasheet_noise": hold_fixed,
     "holdout_n": 200}, indent=2))
