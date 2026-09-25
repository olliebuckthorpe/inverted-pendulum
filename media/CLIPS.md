# Clips index

Every video, what it shows, and how to regenerate it. The video files live in `media/clips/` (kept out of git,
PLAN 0.2); this index is in git so nothing gets lost. Naming: `YYYY-MM-DD-<source>-<what>.mp4`.
"sim" clips are MuJoCo simulations: 30T pulley, 30 cm aluminium rod 1, 20 cm carbon rod 2 unless stated.

## Simulation, 24 Sep 2026 (sim phase)

| File | Shows | Regenerate (from `sim/`) |
|---|---|---|
| 2026-09-24-sim-single-swingup-20T.mp4 | Classical energy swing-up → LQR catch (pole turns green), 20T pulley (before 30T was chosen) | `uv run python watch.py swingup --teeth 20 --video` |
| 2026-09-24-sim-single-taps-20T.mp4 | LQR recovering from a tap every 2 s, 20T | `uv run python watch.py tap --teeth 20 --video` |
| 2026-09-24-sim-single-balance-20T.mp4 | LQR balancing from 5°, 20T | `uv run python watch.py balance --teeth 20 --video` |
| 2026-09-24-sim-double-swingup-first.mp4 | First double swing-up: trajectory optimisation + time-varying LQR, no sensor noise | `uv run python watch.py double --traj 30T_30-20 --video` (older watch.py) |
| 2026-09-24-sim-double-swingup-robust-kalman.mp4 | Robust double swing-up: datasheet sensor noise, Kalman filter, 5 A limit, friction-margin plan | `uv run python watch.py double --video` |

## Simulation, 25 Sep 2026

| File | Shows | Regenerate |
|---|---|---|
| 2026-09-25-sim-single-swingup-30T.mp4 | Classical swing-up → LQR, 30T (chosen pulley) | `uv run python watch.py swingup --video` |
| 2026-09-25-sim-single-tap-30T.mp4 | LQR recovering from taps, 30T | `uv run python watch.py tap --video` |
| 2026-09-25-sim-single-balance-30T.mp4 | LQR balancing from 5°, 30T | `uv run python watch.py balance --video` |
| 2026-09-25-sim-transition-XX-YY.mp4 (12 clips) | Each move between the double's four resting positions: DD both hanging, UU both up, UD rod 1 up / rod 2 hanging, DU rod 1 hanging / rod 2 up. All with sensor noise, Kalman filter, 5 A limit; ends holding the new position | `./render_clips.sh 2026-09-25` (all 14) |
| 2026-09-25-sim-tour-DD-UU-UD-DU-DD.mp4 | Tour: swing up, then double-inverted → single-inverted → single-down → hanging, holding each 2 s | `./render_clips.sh` |
| 2026-09-25-sim-tour-DD-UU-DU-UD-UU-DD.mp4 | Five-move tour through all four positions | `./render_clips.sh` |

## To come

- AI training timelapses (`uv run python ai_timelapse.py balance_seed0`): the policy learning, snapshot by snapshot.
- Hardware milestones (PLAN: film every milestone): first Pico readings, first balance, first swing-up, …
