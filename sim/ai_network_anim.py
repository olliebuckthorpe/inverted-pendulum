"""Animate the policy network wiring itself up during training.

Each training snapshot's actor network (6 inputs → 64 → 64 → 1 output) is drawn with every connection:
colour = sign (orange positive, blue negative), line strength = |weight|. Frames blend smoothly between
snapshots; beside it, the learning curve with a marker at the snapshot's practice time.

  uv run python ai_network_anim.py balance_seed0 [out.mp4]
"""

from __future__ import annotations

import re
import sys

import imageio.v2 as imageio
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.collections import LineCollection
from stable_baselines3 import SAC

import model
from envs.pendulum_env import POLICY_HZ

INPUTS = ["cart position", "cart speed", "sin θ", "cos θ", "rod spin", "motor now"]
BLEND = 10   # frames between snapshots
HOLD = 6     # frames held on each snapshot


def actor_weights(path) -> list[np.ndarray]:
    """[W1 (64×6), W2 (64×64), W3 (1×64)] of the deterministic action path."""
    actor = SAC.load(path, device="cpu").policy.actor
    lin = [m for m in actor.latent_pi if hasattr(m, "weight")]
    return [lin[0].weight.detach().numpy(), lin[1].weight.detach().numpy(), actor.mu.weight.detach().numpy()]


def layout():
    xs = [0.0, 1.0, 2.0, 3.0]
    ys = [np.linspace(0.2, 0.8, 6), np.linspace(0.02, 0.98, 64), np.linspace(0.02, 0.98, 64), np.array([0.5])]
    return xs, ys


def edges(Ws):
    xs, ys = layout()
    segs, vals = [], []
    for li, W in enumerate(Ws):
        for j in range(W.shape[0]):          # to-neuron
            for i in range(W.shape[1]):      # from-neuron
                segs.append([(xs[li], ys[li][i]), (xs[li + 1], ys[li + 1][j])])
                vals.append((li, W[j, i]))
    return segs, vals


def learning_curve(tag):
    f = model.SIM_DIR / "results" / "ai" / f"{tag}.monitor.csv"
    data = np.genfromtxt(f, delimiter=",", skip_header=2)
    steps, r = np.cumsum(data[:, 1]), data[:, 0]
    k = max(1, len(r) // 60)
    return steps[k - 1:], np.convolve(r, np.ones(k) / k, mode="valid")


def main():
    tag = sys.argv[1]
    out = sys.argv[2] if len(sys.argv) > 2 else str(model.SIM_DIR / "videos" / f"network-{tag}.mp4")
    snapdir = model.SIM_DIR / "checkpoints" / f"{tag}_snapshots"
    snaps = sorted(snapdir.glob("*.zip"), key=lambda f: int(re.search(r"_(\d+)_steps", f.name).group(1)))
    steps = [int(re.search(r"_(\d+)_steps", f.name).group(1)) for f in snaps]
    Ws = [actor_weights(f) for f in snaps]
    # Step 0: the untrained network. SAC's initial weights depend only on the seed, so rebuild them exactly.
    from envs.pendulum_env import PendulumEnv
    task_name = "balance" if tag.startswith("balance") else "swingup"   # tags are e.g. "balanceG_seed0"
    seed = int(tag.split("_seed")[1])
    init = SAC("MlpPolicy", PendulumEnv(task_name), policy_kwargs=dict(net_arch=[64, 64]), seed=seed, device="cpu")
    lin = [m for m in init.policy.actor.latent_pi if hasattr(m, "weight")]
    Ws.insert(0, [lin[0].weight.detach().numpy(), lin[1].weight.detach().numpy(), init.policy.actor.mu.weight.detach().numpy()])
    steps.insert(0, 0)
    final = model.SIM_DIR / "checkpoints" / f"{tag}.zip"
    if final.exists():  # the finished network, after the last snapshot
        Ws.append(actor_weights(final))
        steps.append(int(np.genfromtxt(model.SIM_DIR / "results" / "ai" / f"{tag}.monitor.csv", delimiter=",", skip_header=2)[:, 1].sum()))
    segs, _ = edges(Ws[0])
    # per-layer scale fixed at the largest weight seen in training, so growth is visible
    scale = [max(np.abs(W[li]).max() for W in Ws) for li in range(3)]
    cs, cr = learning_curve(tag)
    xs, ys = layout()

    fig = plt.figure(figsize=(12.8, 7.2), dpi=100)
    ax = fig.add_axes([0.02, 0.06, 0.62, 0.84])
    axc = fig.add_axes([0.70, 0.18, 0.27, 0.55])
    task = "balance" if tag.startswith("balance") else "swing-up"
    with imageio.get_writer(out, fps=25, codec="libx264", quality=8) as w:
        for s in range(len(Ws)):
            nxt = min(s + 1, len(Ws) - 1)
            for f in range(HOLD + (BLEND if nxt != s else 0)):
                t = 0.0 if f < HOLD else (f - HOLD + 1) / BLEND
                cur = [(1 - t) * a + t * b for a, b in zip(Ws[s], Ws[nxt])]
                st = (1 - t) * steps[s] + t * steps[nxt]
                ax.clear(); axc.clear()
                flat = np.concatenate([(W / scale[li]).ravel() for li, W in enumerate(cur)])
                colours = np.zeros((len(flat), 4))
                colours[flat >= 0] = (0.90, 0.45, 0.10, 0)
                colours[flat < 0] = (0.15, 0.40, 0.85, 0)
                colours[:, 3] = np.clip(np.abs(flat), 0, 1) ** 1.5 * 0.9
                ax.add_collection(LineCollection(segs, colors=colours, linewidths=0.3 + 1.5 * np.abs(flat)))
                for li in range(4):
                    ax.scatter(np.full(len(ys[li]), xs[li]), ys[li], s=36 if li in (0, 3) else 9, c="#222", zorder=3)
                for name, y in zip(INPUTS, ys[0]):
                    ax.text(-0.08, y, name, ha="right", va="center", fontsize=10)
                ax.text(3.08, 0.5, "motor\ncommand", ha="left", va="center", fontsize=10)
                for x, lab in zip(xs, ["inputs (sensors)", "64 neurons", "64 neurons", "output"]):
                    ax.text(x, 1.03, lab, ha="center", fontsize=10, color="#444")
                ax.set_xlim(-0.9, 3.6); ax.set_ylim(-0.02, 1.08); ax.axis("off")
                fig.text(0.5, 0.95, f"AI {task} network, after {int(st):,} practice steps "
                         f"(≈ {st / POLICY_HZ / 3600:.1f} h of simulated practice)", fontsize=15, ha="center")
                axc.plot(cs, cr, color="#2a6", lw=1.5)
                axc.axvline(st, color="#c33", lw=1.2)
                axc.set_title("score per practice attempt", fontsize=10)
                axc.set_xlabel("practice steps", fontsize=9); axc.tick_params(labelsize=8)
                fig.text(0.70, 0.08, "orange = positive connection, blue = negative;\nbrighter/thicker = stronger",
                         fontsize=9, color="#555")
                fig.canvas.draw()
                w.append_data(np.asarray(fig.canvas.buffer_rgba())[..., :3])
                fig.texts = []
    plt.close(fig)
    print(f"saved {out}")


if __name__ == "__main__":
    main()
