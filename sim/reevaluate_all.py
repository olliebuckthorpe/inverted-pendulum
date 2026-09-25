"""Re-test every trained checkpoint on the FULL rig (randomised, gearbox play, smoothing, noise), 100 episodes.
Writes results/ai/full_rig_eval.json. Usage: uv run python reevaluate_all.py"""
import json

import torch
from stable_baselines3 import SAC

import model
import train_ai

torch.set_num_threads(1)
out = {}
for f in sorted((model.SIM_DIR / "checkpoints").glob("*.zip")):
    tag = f.stem
    task = "balance" if tag.startswith("balance") else "swingup"
    meta = model.SIM_DIR / "results" / "ai" / f"{tag}.json"
    hz = json.loads(meta.read_text()).get("env", {}).get("policy_hz") if meta.exists() else None
    r = train_ai.evaluate(SAC.load(f, device="cpu"), task, env_kw={"policy_hz": hz} if hz else {})
    out[tag] = r["success"]
    print(f"{tag:22s} {r['success']:3d}/100 on the full rig", flush=True)
(model.SIM_DIR / "results" / "ai" / "full_rig_eval.json").write_text(json.dumps(out, indent=2))
