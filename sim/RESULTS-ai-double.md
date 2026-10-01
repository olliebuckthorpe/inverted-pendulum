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

## Double swing-up (exploratory): works on 68 % of realistic rigs, but not robust

**Result (1 Oct):** swing-up policy → hand-over → balance policy (`double_handover.py`), starting from
hanging on realistic rigs it never trained on: **34/50 swung up and held** with the best snapshot
(`double_swingupEr_seed0`, 666k steps) and the catch-trained balance policy (`double_balanceG50kc_seed0`).
**But it fails every time on the nominal rig (0/30 with or without noise) and manages 8/30 on random rigs
without noise**: it gets near the catch zone and circles just outside it, and rig variation or sensor noise
is what tips it in. A real rig is one fixed rig, so on hardware it could land anywhere from 0 % to high.
**Verdict: an AI double swing-up exists in sim, but it isn't a dependable controller.** The classical swing-up
(50/50 random rigs) stays the hardware plan.

### How it got there (every score from hanging, on rigs never trained on)

| Attempt | Reward / setup | Result | Lesson |
|---|---|---|---|
| E, 3 seeds × 3M | tip height (−1..1) | every episode ended in ~37 steps | a −1-per-hanging-step reward made driving off the rail the best policy |
| E, 3 seeds × 3M | tip height rescaled to 0..1 | **0/100** (realistic), 0/20 on its own rig | it learned to **spin the rods** (4–8 net turns of rod 1 per episode, up 37 % of the time, never held > 1.5 s). Also illegal on the rig: the joint-2 cable allows ±1 turn, so the env now ends the episode there |
| – | the hand-over zone | a box (15°/3 rad/s per rod) was catchable 20–30 % (even 5°/1 rad/s only ~65 %) | the double's catchable states depend on how the rods move **together**: the zone is now the balance LQR's cost-to-go V = s'Ss < 250 (95 % caught on 400 random states; LQR from inside it 20/20) |
| Ec, 2 seeds × 1.5M | +100 for reaching the zone, then hand over | 4/100 and 0/100 on its own rig | the zone is a thin target; arrival-only reward gives no signal of how close it came |
| Ecv, 2 seeds × 1.5M | + a smooth bonus as V falls | 28/100 and 0/100 own rig; 9 and 12 realistic | some signal, not enough |
| Gc, 2 seeds × 2M | zone reward, random rigs | 0/4 own; 2 and 21 realistic | – |
| **Er/Gr, 4 runs × 4M** | **reverse curriculum** (Salimans & Chen 2018): start along the classical plan's path just below the zone, move the start back towards hanging whenever ≥ 60 % reach the zone (only the plan's **states**, never its actions) | best run reached 95 % of the way back by 613k steps (99 % success at 90 %), then stalled; others reached 10–65 % | ✅ the only approach that swings up from hanging |

**Keep the best snapshot, not the last:** the best run's full-sequence score fell after its peak (533k 30/50,
666k **34/50**, 800k 32/50, 2M 21/50, final 4M 18/50); the single's training showed the same slide.

### Balance policies used by the hand-over

| Policy | Starts from | Own setup | Realistic rigs |
|---|---|---|---|
| G50k (3 seeds) | near upright | 98, 100, 100 | 97, 100, **100** |
| G50kc (2 seeds) | anywhere in the catch zone | 97, 94 | **98**, 94 |

### If this were taken further (not planned)

Fine-tune the 666k snapshot from hanging on randomised rigs with noise and a penalty for time spent near but
outside the zone, and select on a separate set of validation rigs including the nominal one. Not worth it
before the classical double works on hardware (SPEC §3 order).
