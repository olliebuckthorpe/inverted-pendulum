# Test protocol — DRAFT from the datasheet sim (PLAN 5.4, SPEC §12)

**Status: draft, not fixed.** Generated 2026-09-25 by `sim/protocol_numbers.py` on the datasheet sim
(sensor noise 0.068°, 5 A limiter at 4.5 A, slew 0.25/cycle, both controllers through the same pipeline).
**Re-run on the fitted sim after Phase 4, review, then commit as `analysis/PROTOCOL.md` by Sun 11 Oct. After
that commit the numbers do not change.**

## Numbers (chosen so that neither controller is favoured)

| Item | Value | How it was chosen |
|---|---|---|
| Swing-up time limit | **5 s** | 1.5 × the 95th percentile of both controllers' swing-up times together |
| Hold time | **5 s** | Balanced (|θ| < 10°) for this long after catching counts as a success |
| Settle band | **±2.7°** | max(2°, 3 × the larger controller's resting θ RMS) |
| Settling time | time from the end of the standard kick until θ stays inside the band | – |
| Software kick | **duty overridden to +1.0 for D ms** (a cart jolt; Pico KICK) | fixed amplitude, duration varies |
| Standard kick (settling, effort) | **D = 40 ms** | half the smaller controller's largest kick survived, rounded to 20 ms |
| Kick ladder rungs | **D = 20, 30, 40, 50, 60, 70, 80, 90, 100, 110, 120 ms** | 20 ms to one rung past where both fail, in 10 ms steps (the sim's 20 ms steps tied both controllers) |
| Largest push survived | largest rung recovered in ≥ 8 of 10, where recovered = the rod never passes 35° (the hand-back angle) and is upright at the end | SPEC §12; a knocked-down rod that is re-swung-up does not count |
| Motor effort | RMS duty over the 3 s after the standard kick | SPEC §12 |
| Trials | swing-up 20 per controller; settling 20 per controller; 10 per ladder rung per controller | SPEC §12 |
| Order | controllers **alternate** trial by trial (A, B, A, B …), same rig, same day, same rod | SPEC §12 |
| Recorded per trial | raw telemetry at 500 Hz (t, x, ẋ, θ, θ̇, duty, mode, faults, loop timing) → `analysis/data/` | – |

## What the sim predicts (datasheet model; a sanity check, not a result)

| | Classical | AI |
|---|---|---|
| Swing-up success (of 20) | 20 | 20 |
| Swing-up time, median | 2.25 s | 1.43 s |
| Resting θ RMS / x RMS | 0.26° / 7.5 mm | 0.92° / 163.6 mm |
| Largest kick survived (≥ 8/10) | 60 ms | 60 ms |
| Settling time (median, standard kick) | 0.11 s | 1.07 s |
| Effort (median RMS duty) | 0.292 | 0.446 |

Ladder passes per rung (of 10): classical {20: 10, 40: 10, 60: 10, 80: 5, 100: 0}; AI {20: 10, 40: 10, 60: 10, 80: 5, 100: 0}.

## Still to add before fixing

- Physical tapper settings (PLAN 7.1): servo angle, strike height, and its consistency check (CV < 10%).
- Anything tuned on hardware must be reported (SPEC §12, honest reporting).
