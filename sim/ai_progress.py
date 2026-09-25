"""Learning-curve summary from results/ai/*.monitor.csv (and a plot): mean episode reward and length over time.
Usage: uv run python ai_progress.py [plot]"""
import sys

import numpy as np

import model

OUT = model.SIM_DIR / "results" / "ai"
rows = {}
for f in sorted(OUT.glob("*.monitor.csv")):
    data = np.genfromtxt(f, delimiter=",", skip_header=2)
    if data.ndim < 2 or len(data) == 0:
        continue
    r, l = data[:, 0], data[:, 1]
    steps = np.cumsum(l)
    tag = f.name.replace(".monitor.csv", "")
    rows[tag] = (steps, r, l)
    last = slice(max(0, len(r) - 20), len(r))
    print(f"{tag:16s} {int(steps[-1]):>8d} steps, {len(r):>5d} episodes | last 20: reward {r[last].mean():8.1f}, "
          f"length {l[last].mean():6.0f} policy steps")
if len(sys.argv) > 1:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(1, 2, figsize=(12, 4))
    for tag, (steps, r, l) in rows.items():
        k = max(1, len(r) // 50)
        smooth = np.convolve(r, np.ones(k) / k, mode="valid")
        (ax[0] if tag.startswith("balance") else ax[1]).plot(steps[k - 1:], smooth, label=tag)
    for a, t in zip(ax, ("balance", "swing-up")):
        a.set_title(f"SAC {t}: episode reward (smoothed)"); a.set_xlabel("policy steps"); a.legend(fontsize=8)
    fig.tight_layout(); fig.savefig(model.SIM_DIR / "plots" / "ai-learning-curves.png", dpi=90)
