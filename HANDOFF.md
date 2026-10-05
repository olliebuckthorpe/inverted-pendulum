# Handoff: continuing on the Windows PC (5 Oct 2026)

Written on the Mac for the Claude Code session on Ollie's Windows PC, which takes over the hardware work
because the Pico 2 W can't be plugged into the Mac (no matching USB port/cable). **Read this, then `SPEC.md`
(the contract) and `PLAN.md` (the dated steps).** Ollie's working rules are at the bottom.

## 1. Set up this PC (once)

Ollie installs Claude Code, Git for Windows and the GitHub CLI and runs `gh auth login` himself. Then, in
**Git Bash**:

```bash
gh repo clone olliebuckthorpe/inverted-pendulum && cd inverted-pendulum
# Python env manager (PowerShell one-liner; run in PowerShell, then reopen Git Bash)
#   powershell -ExecutionPolicy ByPass -c "irm https://astral.sh/uv/install.ps1 | iex"
cd sim && uv sync && uv run python -m pytest -q        # expect: all pass, 1 skipped (clang++ parity test)
```

Firmware toolchain (only needed to rebuild firmware; a built `.uf2` is already on the Pico):

```bash
winget install ArduinoSA.CLI
arduino-cli config init
arduino-cli config add board_manager.additional_urls https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json
arduino-cli core update-index && arduino-cli core install rp2040:rp2040@6.1.1
firmware/build.sh                                       # Git Bash; builds for rpipico2 on purpose (see the script)
```

## 2. Where things stand

- **Sim phase done** (`sim/RESULTS-sim-phase.md`): 30T pulley, 30 cm rod, 500 Hz, 5 A PSU + firmware current
  limit, brake relay, cable loop not a slip ring, double swing-up and all 12 transitions in sim. Cart mass
  0.16 kg (CAD estimate; weigh the real parts).
- **AI:** single chosen pair 96 % hold-out (`sim/RESULTS-ai.md`). Double balance 97–100/100 realistic at 50 Hz
  with the Kalman-filtered state; double swing-up exists but isn't dependable (`sim/RESULTS-ai-double.md`).
  Trained double networks are on the **Mac** only (`sim/checkpoints/`, gitignored).
- **Parts:** `hardware/orders.md`. Arrived: bearings, 30T pulley, belt, MDD10A (10 A), jumpers, breadboard,
  fuse holders, DC socket, 18 AWG wire, M3 kit, relays, M2/M2.5/M3 kit, micro switches, idlers, e-stop, M5
  bolts, **Pico 2 W**, 8 mm shaft (due Sat 3 Oct: confirm). Still due: **rail** (Amazon), **motor** (TME),
  **PSU + 3 × AS5047P + small parts** (DigiKey, ordered 1 Oct). In person, when needed: plywood beam, G-clamps,
  wood screws, 8 mm aluminium tube.
- **Bench bring-up** (`hardware/BRINGUP.md`, Mac-side tool `sim/bringup.py`, `hardware/safety-checklist.md`):
  - Stage 1 step 1 **done 5 Oct**: blink sketch flashed by UF2 from Windows; LED blinks (PLAN 2.3).
  - Stage 1 step 2 in progress: `firmware/build/pendulum.ino.uf2` (built for `rpipico2`, radio off) was handed
    to Ollie to flash; check Device Manager → Ports shows a COM port.
- **Printing** (Bambu A1, PLA): fit test printed; **608 seats fit at 0.10 mm** (P.fit, PR #13). Queue: cart,
  motor_block, idler_block, end_ramp_right/left (`hardware/cad/out/`, 0.20 mm Standard, 4 walls, 30 %).
  Shaft fit done 5 Oct (rod hub 0.15 mm slide + pinch screws, magnet cap 0.10 mm push-fit; both re-exported).
  Waiting on Ollie's measurements: belt thickness
  (→ `belt_t`), micro-switch hole size and roller height (→ switch carrier). Sensor mount waits for the
  AS5047P boards.
- **Open PRs:** #12 (AI double swing-up write-up), #13 (fit test + fit 0.10). Merge in order when Ollie says.

## 3. Next steps on this PC

1. With the pendulum firmware flashed, find the COM port, then from `sim/`:
   `uv run python bringup.py COM5 watch` (use the real COMn). Expect: mode IDLE, duty +0.00, ✗ on every live
   input and the E-stop fault latched (nothing wired; correct). That finishes BRINGUP stage 1.
2. Stage 2 when the AS5047P boards arrive: **read the adapter board's manual and set it for 3.3 V before
   powering it**; then `watch`, `calibrate`, `noise`.
3. Continue `hardware/BRINGUP.md` stage by stage; nothing gets 12 V until stage 4, and no closed-loop control
   until every line of `hardware/safety-checklist.md` is ticked.

Windows notes: ports are `COMn`, not `/dev/cu.usbmodem…`; run `firmware/build.sh` in Git Bash; `bringup.py`
uses pyserial and should work unchanged (not yet tried on Windows: verify and fix if not).

## 4. Ollie's working rules

- Follow PLAN.md in order and cite SPEC.md; if the spec looks wrong, stop and say so. Explain the maths simply.
- Every hardware or design choice is evidence-based with pros and cons; his decisions go through a question.
- A task is done only when its "done when" check passes. Two identical failures → pivot, don't retry.
- Git: feature branches, stage specific files only, author `ollie.buckthorpe@gmail.com`, **no Co-Authored-By
  trailers**. PRs merged only when he says.
- **Never buy anything or enter payment details**; Ollie orders. **Show the running total before any spend**
  (≈ £331 so far against a £150–200 target, and he's unhappy about it).
- End of each working day: remind him to add an entry to `media/LOG.md`.
