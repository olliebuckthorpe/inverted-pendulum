# AI results so far (PLAN 5.3, early run on the datasheet sim), 25 Sep 2026

**Update (multi-seed, fair selection, `select_ai.py`): the chosen AI controller is `swingupE_seed3` →
`balanceGc_seed4`, handing over at 15°. Hold-out, 200 rigs scored once: 96.0% with realistic varied noise,
96.5% at exactly the datasheet noise.** It was chosen from 80 swing-up × catch-balance × hand-over-angle
combinations on 100 separate validation rigs; the median combination scored only 42/100, so training
several seeds and selecting fairly is part of the method, not an optional extra. New swing-up seeds scored
92, 94, 86 (seeds 0/1: 82, 48); catch-balance seeds scored 49, 5, 62 from hand-over-like starts. The
learning curves show repeated collapses and recoveries (a known SAC instability), so the final training
should keep the best snapshot by validation score, not the last one.

**Earlier result (kept for the record):**

**PLAN 5.3 targets are met in the randomised, datasheet-based sim:**

| Target | Result | Policies |
|---|---|---|
| Balance ≥ 95% | **99/100 and 98/100** | `balanceG_seed0`, `balanceG_seed1` |
| Swing-up + hand-over ≥ 90% | **93% (186/200, hold-out rigs never used for choosing)** | `swingupE_seed0` → `balanceGc_seed1`, hand-over at 10° |

Every score is on the **full rig**: randomised physics every episode (masses, rod length, motor constants,
rotor inertia, frictions, delay, gearbox play, belt stretch), the 20 Hz action-smoothing filter, sensor
noise at the AS5047P datasheet level (varied ×0.67–1.5), quantised sensors, the 5 A current limiter.
SB3 SAC, `MlpPolicy` [64, 64], policy at 100 Hz, one CPU thread (fastest: 490 steps/s vs 388 on six).

**These are provisional.** PLAN 5.3 trains on the *fitted* sim (after Phase 4 system ID); this run proves
the method and the pipeline. The cart mass in `params.yaml` (0.40 kg) is also ~2.7× the CAD estimate
(~0.145 kg; `hardware/cad/LAYOUT.md`) and must be updated before the final training.

## How we got here (every attempt, including the failures)

Full-rig scores out of 100 (`results/ai/full_rig_eval.json`, re-tested by `reevaluate_all.py`).

| # | What | Balance | Swing-up | What it taught us |
|---|---|---|---|---|
| 1 | Train from scratch on the full rig, 200k / 1M steps | 1, 8, 16 | 56, 0 | Balances the rod but lets the cart drift into the rail end, even on the nominal rig |
| 2 | Stronger centring + anti-jitter (A), + longer look-ahead (B), at 50 Hz (C), + cart-speed penalty (D) | 0, 4, 14, 6 | – | Reward tuning isn't the problem (two identical failures → pivot) |
| 3 | Sanity check: easy setup (fixed rig, rigid drive, no smoothing, no noise), 150k | **99** | 82, 48 | The task is learnable; a policy trained easy still performs on the full rig |
| 4 | Curriculum: continue #3 on the full rig with anti-jitter (F) | 0 | – | 3× smoother, but training on the full rig unlearns it |
| 5 | Ablation: easy + one ingredient each | smoothing 100, gearbox/belt 100, random physics 100, noise **36**, physics+noise **1**, strong jitter penalty 62 | – | **Sensor noise breaks learning** (not performing) |
| 6 | Model-free (constant-velocity) Kalman estimator instead of differencing | – | – | Only trades noise for lag; at best equals differencing + low-pass. Removed |
| 7 | **G: full rig, noise off while learning, tested with noise** | **99, 98** | (running) | The fix for balance |
| 8 | Swing-up → hand-over → balance (G), 10° | – | 83 | 16/17 failures run off the rail after the catch: hand-over states (cart ≤ 194 mm off-centre, 0.63 m/s, rod spin 2.8 rad/s) are far outside balance training (50 mm, 0, 0.3 rad/s) |
| 9 | **Gc: balance trained from hand-over-like starts** | seed 1: 62 from catch starts; seed 0: 7 | **92 (test), 93 (hold-out)** with seed 1 | The fix for the hand-over; strongly seed-dependent |

## Honest caveats for the write-up

- **Seed sensitivity:** the same recipe gave 92 and 11 (swing-up + hand-over) depending on the balance
  policy's random seed. RL training is unstable; report both, and train several seeds for the final run.
- **Noise was off during training**, on during every test. The AI never learned to filter noise; it
  tolerates it because the Pico's estimator (the same for both controllers) filters it.
- **The chosen combination was picked on the test set**; the 93% is from a separate hold-out of 200 rigs.
- Datasheet sim only: "works in sim" is necessary, not sufficient (SPEC §11).

## Videos (media/clips/, index media/CLIPS.md)

- `2026-09-25-sim-ai-swingup-handover-demo.mp4`: the AI controller, 3 random rigs
- `2026-09-25-sim-ai-timelapse-*.mp4`, `2026-09-25-sim-ai-network-*.mp4`: learning timelapses and the network
  wiring itself up
