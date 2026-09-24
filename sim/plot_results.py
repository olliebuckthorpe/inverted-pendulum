"""Summary figures for RESULTS-sim-phase.md, from results/*.json (no simulation here)."""

import json

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

import model

R = model.SIM_DIR / "results"
q2 = json.loads((R / "q2_pulley.json").read_text())
q34 = json.loads((R / "q3_q4.json").read_text())

fig, ax = plt.subplots(1, 3, figsize=(15, 4.2))
worlds = ["nominal", "motor -30% torque", "friction +30%", "rotor inertia x0.5", "rotor inertia x2"]
x = range(len(worlds))
for off, teeth, col in ((-0.2, 20, "C0"), (0.2, 30, "C1")):
    ax[0].bar([i + off for i in x], [q2[f"{teeth}T | {w}"]["largest_tap_rad_s"] for w in worlds], 0.4,
              color=col, label=f"{teeth}T")
ax[0].set_xticks(list(x), [w.replace(" ", "\n", 1) for w in worlds], fontsize=8)
ax[0].set_ylabel("largest tap survived (rad/s)"); ax[0].set_title("Q1/Q2: pulley robustness across model errors")
ax[0].legend()

for teeth, col in ((20, "C0"), (30, "C1")):
    taps = [q34["q3"][f"{teeth}T {L:.2f} m"]["largest_tap_rad_s"] for L in (0.30, 0.40)]
    su = [q34["q3"][f"{teeth}T {L:.2f} m"]["success"] for L in (0.30, 0.40)]
    ax[1].plot([30, 40], taps, "o-", color=col, label=f"{teeth}T (swing-up {su[0]}/10 → {su[1]}/10)")
ax[1].set_xticks([30, 40]); ax[1].set_xlabel("rod length (cm)"); ax[1].set_ylabel("largest tap survived (rad/s)")
ax[1].set_title("Q3: rod length"); ax[1].legend(fontsize=8)

rates = [100, 200, 500, 1000]
for teeth, col in ((20, "C0"), (30, "C1")):
    ax[2].plot(rates, [q34["q4"][f"{teeth}T {r} Hz"]["largest_tap_rad_s"] for r in rates], "o-", color=col, label=f"{teeth}T tap")
a2 = ax[2].twinx()
for teeth, col in ((20, "C0"), (30, "C1")):
    a2.plot(rates, [q34["q4"][f"{teeth}T {r} Hz"]["duty_chatter_rms"] for r in rates], "s--", color=col, alpha=0.6, label=f"{teeth}T chatter")
ax[2].set_xscale("log"); ax[2].set_xticks(rates, [str(r) for r in rates]); ax[2].set_xlabel("LQR loop rate (Hz)")
ax[2].set_ylabel("largest tap survived (rad/s)"); a2.set_ylabel("duty chatter (RMS step-to-step, dashed)")
ax[2].set_title("Q4: loop rate (2 ms delay)"); ax[2].legend(fontsize=8, loc="lower right")
fig.tight_layout()
fig.savefig(model.SIM_DIR / "plots" / "sim-phase-single-summary.png", dpi=90)
print("ok")
