# AI for the double pendulum (sim)

Added 30 Sep 2026 at Ollie's request: **AI double balance** (SPEC §3 stretch, PLAN 8.5) and an **exploratory
AI double swing-up** (not in the original scope; honest outcome either way). Environment `envs/double_env.py`
(the double's sensing, delay, 5 A limiter, smoothing and domain randomisation); trainer `train_double.py`
(SAC, one thread per run). Every score is on **100 rigs the policy never trained on**; "realistic" = random
physics + the AS5047P's datasheet noise (0.068° RMS). Cart 0.16 kg (see RESULTS-sim-phase.md).

## Double balance: 97/100 and 100/100 on realistic rigs

| Attempt | What changed | Own training setup | Realistic rigs | Lesson |
|---|---|---|---|---|
| G (3 seeds, 500k steps) | 200 Hz decisions, random physics, no noise while learning (the single's recipe) | – | 8, 0, 1 | fails |
| Eb (2 seeds) | the easy setup: one fixed rig, no noise, no smoothing | **0, 1 of 50** | – | it can't learn at all, so it's not the randomisation |
| check: LQR in the same env | – | 20/20 even at 50 Hz with smoothing | – | the task is feasible; the learning isn't |
| Eb50 (2 seeds, 300k) | **50 Hz decisions** (γ = 0.99 now looks ~2 s ahead instead of ~0.5 s; MuJoCo's InvertedDoublePendulum, which SAC solves, decides at 20 Hz) | 87, 75 | 8, 18 | the decision rate was the blocker |
| G50 (3 seeds, 600k) | 50 Hz on random physics | 97, 97, 99 | 12, 4, 27 | learns the physics; **sensor noise** breaks it |
| check: LQR at 50 Hz with noise | differenced velocities vs Kalman filter | – | 0/20 vs **20/20** | at 50 Hz the velocities must be filtered |
| **G50k (2 seeds, 600k)** | **the policy sees the Kalman-filtered state** (the classical double's filter, 500 Hz, from the nominal model) | 98, 100 | **97, 100** | ✅ |

**What the Pico would run:** the balance Kalman filter every 2 ms (a few 6×6 matrix-vector products; the
classical double needs it anyway) and a 9→128→128→1 network every 20 ms (~17k multiply-adds, well inside
the budget). Not yet exported or measured on the Pico (PLAN 6.1's method applies).

## Double swing-up (exploratory): in progress

Variant E (one fixed rig, no noise, 100 Hz, γ 0.995, 256×256 network), 3 seeds × 3M steps. The first attempt's
reward was a trap: the tip height ran −1..1, so every hanging step cost −1 and driving off the rail (−50) was
the best policy (every episode ended in ~37 steps). Rescaled to 0..1, the episodes last the full 12 s and the
reward is still rising at 1.5M steps. Results will be added here.
