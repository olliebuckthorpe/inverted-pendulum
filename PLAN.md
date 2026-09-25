# PLAN — Classical vs AI Inverted Pendulum on a Cart

Written Thu 24 Sep 2026. The contract is **SPEC.md** (read it first). This plan turns it into dated,
verifiable steps up to the **Academy deadline, Thu 5 Nov 2026**. A fresh session should be able to pick up
any phase with zero re-discovery.

**How to use this plan**
- Work phases in order. Phases 1 and 2 overlap on purpose (sim while parts arrive).
- Every task has a **Done when** check. A task is not done until its check passes, and hardware tasks aren't
  done until they've been seen working (not "it compiled").
- **Two identical failures = pivot, not retry.** Write down what was tried, then change approach or drop to
  the fallback listed in the phase.
- **Film and write as you go.** At the end of every day, add a dated entry to `media/LOG.md`: 1–3 sentences on
  what worked or failed, plus any clips or photos (`media/clips/YYYY-MM-DD-*.mp4`). The videos and write-up
  are cut from this log.
- Gates (🚦) are hard. If a gate date passes, take the stated fallback; don't stretch.

## Key dates

| Date | Gate |
|---|---|
| Thu 24 Sep | Order all parts except the motor and PSU |
| **Mon 28 Sep** | 🚦 **Order the motor and the PSU** (sim answers or the defaults) |
| **Sun 4 Oct** | 🚦 Safety checklist passed before any closed-loop run |
| **Sun 11 Oct** | 🚦 Test-protocol numbers fixed in `analysis/PROTOCOL.md` (before the first test trial) |
| **Wed 21 Oct** | 🚦 **Single DONE** target (SPEC §4). If not done by **Sun 25 Oct**, skip the double entirely |
| **Sun 1 Nov** | 🚦 **The double stops**, whatever state it's in |
| **Thu 5 Nov** | Submit |

## Prior knowledge applied

Everything below is already researched and sourced in SPEC §6–§17. Don't re-research it; cite it.
- The control loop runs **on the Pico**, not the Mac (ferrolho, Re-Forward; the Hackaday double needed special networking to run PC-in-the-loop).
- The AI needs **system ID + domain randomisation + action smoothing + delay modelling** (Xu & Tran 2026). Bang-bang policies break hardware.
- Swing-up is **two policies with a hand-over** (Xu & Tran). Classical is **energy swing-up → LQR** (Tedrake).
- **SB3 on CPU** for small MLP policies (SB3 docs). Don't chase Apple MPS or GPU.
- **Read the AS5047P over SPI**, the motor encoder via **PIO** (`pico-examples` quadrature_encoder).
- **Heartbeat watchdog of 100 ms** (WPILib convention). Hardware E-stop and end-stops cut power independently of software.
- **A datasheet-only sim is optimistic.** It can prove "won't work", never "will work".
- **No USB CDC latency figure exists for the RP2350; measure it.** Likewise, no NN inference benchmark exists; measure it.

---

## Phase 0 — Today, Thu 24 Sep: order + repo

**0.1 Order the non-motor parts (Ollie).**
- Use SPEC §6 items 1–18 plus **15a (relay) and 15b (diode)**, but **not item 9, the PSU**: it's ordered with the motor once sim Q6 has sized it. Re-check every ⚠️ price at checkout: the rail at **1000 mm**, the AS5047P kit, the MD13S VAT, the E-stop, the shaft. Skip anything already owned (breadboard, jumpers, bolts, wire).
- Double-check the exact specs as you buy: **GT2 6 mm belt; 20T and 30T pulleys with a 6 mm bore; MGN12H carriage; Pico 2 H; AS5047P-TS_EK_AB; MD13S; a 12 V automotive relay with 30–40 A contacts.**
- **Done when:** order confirmations have been saved to `hardware/orders.md` (item, seller, price paid, expected delivery).

**0.2 Create the repo.**
- `git init` in `~/Desktop/inverted-pendulum`, then create the folders `sim/ firmware/ analysis/ hardware/ web/ media/clips/`.
- Add a `.gitignore` covering `.venv/`, `__pycache__/`, `sim/checkpoints/` (SB3 model zips; keep only the final ones, via a release or Git LFS), `media/clips/` (keep the videos out of git), `.DS_Store` and `.env`.
- The first commit on `main` holds only `SPEC.md`, `PLAN.md` and `.gitignore`. **After that, all work goes on branches** (`feature/sim-phase`, `feature/firmware-bringup`, …), merged through PRs once pushed to GitHub. Stage specific files, never `git add .`. Commit author: `ollie.buckthorpe@gmail.com`, with **no Co-Authored-By trailers**.
- Create a **private** GitHub repo for now. It goes public at the end (Phase 9).
- **Done when:** `git log` shows the initial commit, and the repo exists on GitHub.

**0.3 Learning (Ollie, any spare time this week).** Watch Steve Brunton's *Control Bootcamp* (YouTube), up to and including the LQR and pendulum-on-a-cart videos. Also do a quick Python refresher if needed.

---

## Phase 1 — Sim phase: Thu 24 – Mon 28 Sep (branch `feature/sim-phase`)

Goal: answer SPEC §11's five questions with a **datasheet-based** sim, then order the motor. **Time-box: if an
answer isn't clear by Mon 28 Sep, use the default (Pololu 37D 6.3:1, 20T pulley, 30 cm rod) and order anyway.**

**1.1 Python environment.**
- In `sim/`, run `uv init`, then `uv python pin 3.12`, then `uv add mujoco gymnasium[mujoco] stable-baselines3 control scipy sympy numpy matplotlib pyserial cobs`.
- **Done when:** `uv run python -c "import mujoco, gymnasium, stable_baselines3, control; print(mujoco.__version__)"` prints a version, and `uv run python -m mujoco.viewer` opens a window.

**1.2 Cart-pole model (MJCF) with real dimensions.**
- Start from Gymnasium's `inverted_pendulum.xml` (cart = slide joint, pole = hinge joint). Make `sim/models/single.xml` with:
  - rail travel **±0.40 m**: a 1 m rail minus the cart and end blocks, as joint limits;
  - cart mass **0.40 kg** (estimate: printed cart + MGN12H carriage + joint + sensor board; replaced by measured values in Phase 4);
  - pole: **8 mm OD aluminium tube, 0.30 m**, ~0.02 kg, plus a hub mass of ~0.01 kg at the pivot; inertia from the geometry;
  - joint damping small (placeholder until system ID); the hinge **unlimited** (continuous rotation);
  - the physics timestep at **1 ms**.
- A config file `sim/params.yaml` holds every physical number in one place, so system ID can overwrite them later.
- **Done when:** the viewer shows the cart and pole, and the pole falls realistically from upright with zero input.

**1.3 DC motor model from the Pololu datasheet** (`sim/motor.py`).
- Datasheet, output-referred (Pololu #4757, 12 V): stall torque **0.294 N·m**, stall current **5.5 A**, no-load **1600 RPM at 0.2 A**.
- Derived: R = 12 / 5.5 ≈ **2.18 Ω**; Kt ≈ 0.294 / 5.5 ≈ **0.0535 N·m/A**; ω₀ = 167.6 rad/s; Ke ≈ (12 − 0.2·R) / ω₀ ≈ **0.069 V·s/rad**. Kt ≠ Ke reflects gearbox friction and efficiency; keep both, and fit them in Phase 4.
- The model runs each step: the input is a duty cycle in [−1, 1] → voltage = duty × 12 V → I = (V − Ke·ω)/R (**not** clamped at the 5.5 A stall figure: a reversal at speed really does draw more, and that's what Q6 must measure) → τ = Kt·I → cart force F = τ / r_pulley (r = teeth × 2 mm / 2π; **20T: 6.37 mm, 30T: 9.55 mm**) → apply F to the cart slide joint. Include the motor/gear rotor inertia as reflected cart mass (placeholder 0).
- Log the **supply current** (≈ |duty| × motor current when driving; H-bridge approximation) each step, and add an optional **PSU current cap** (default off; 5 A for Q6).
- Add parameters for later: **gearbox backlash** (a dead-band on cart position, default 0.2 mm ≈ 1° at the output), **Coulomb + viscous rail friction**, and **control delay** (default 2 ms).
- **Done when:** a unit check passes. Full duty on a locked cart gives F ≈ 46 N (20T) / 31 N (30T), and a free cart's top speed approaches ≈ 1.07 / 1.60 m/s.

**1.4 LQR balance in sim** (`sim/lqr.py`).
- Linearise about upright: A, B from MuJoCo (`mujoco.mjd_transitionFD`) with the motor model included, or derive them with sympy. Cross-check the two if time allows.
- `control.lqr(A, B, Q, R)`, starting from Q = diag(10, 1, 100, 1) over [x, ẋ, θ, θ̇] and R = 1 (tune these).
- The controller outputs a duty cycle (clamped to ±1). Run it at a configurable rate with a zero-order hold and the delay.
- **Done when:** from 5° off upright, it balances for 30 s and the cart stays within ±0.2 m.

**1.5 Energy swing-up + hand-over in sim** (`sim/swingup.py`).
- Tedrake's energy-shaping swing-up (Underactuated Robotics, cart-pole chapter): push the pole's energy toward the upright energy, and centre the cart with a weak term.
- **Hand-over:** switch to LQR when |θ| < ~20° and the energy is close to upright. Tune the thresholds; they become SPEC §18 open items.
- **Done when:** from hanging at rest, it swings up and balances within 10 s, with the cart staying inside ±0.40 m, in ≥ 9 of 10 runs (with small random starting offsets).

**1.6 Answer the six questions.** Write the answers into `sim/RESULTS-sim-phase.md` with plots.
1. **Motor adequacy:** does 1.5 succeed with the datasheet motor on 20T *or* 30T, with a margin? Also run it with 30% less torque and 30% more friction. If it passes → **order the motor**.
2. **Pulley:** compare 20T vs 30T on swing-up time, peak cart travel, and balance robustness (the largest kick survived). Pick one.
3. **Rod length:** repeat 2 with 0.40 m. Keep 0.30 m unless 0.40 m is clearly better.
4. **Loop rate:** sweep the LQR at 100/200/500/1000 Hz with 2 ms delay. Pick the lowest rate that keeps the full robustness, ×2 for margin. Expect ~500 Hz.
5. **Double layout:** build `sim/models/double.xml` (a carbon top rod ~0.015 kg). LQR-balance it (started upright) for combos of bottom/top length (0.30/0.20, 0.30/0.30, 0.20/0.30, 0.34/0.20). Pick the combo with the **largest kick survived**, and record the loop rate it needs (expect ~1 kHz).
6. **PSU (SPEC §11 Q6):** log the peak/RMS supply current and the time above 5.25 A in the 1.5 swing-up; re-run 1.5 with the supply capped at 5 A. If it still passes ≥ 9/10 → GST60A12 + 7.5 A fuse; otherwise → a larger 12 V PSU (price-checked) and a resized fuse. **Order the PSU with the motor.**
- **Done when:** `RESULTS-sim-phase.md` answers all six, and the motor and PSU have been ordered (🚦 **Mon 28 Sep**). Commit and push.

**Fallback:** if the datasheet motor can't swing up on either pulley even at nominal values → switch to the 24 V Pololu 37D 6.3:1 (#4698) plus a 24 V PSU (the Cytron handles up to 30 V), re-run question 1, and update SPEC §6.

---

## Phase 2 — Build + bring-up: parts arrive ~Sat 26 – Wed 30 Sep (branch `feature/firmware-bringup`, `feature/cad`)

These run alongside Phase 1 as parts land. **No motor power until Phase 3's safety checklist has passed.**

**2.1 Datasheet checks (before wiring anything).**
- Cytron MD13S: the input logic HIGH threshold (it must accept 3.3 V from the Pico), the PWM frequency range, the DIR/PWM mode, the power-on state.
- Regenerative braking: decide on a mitigation from the MD13S and GST60A12 docs (a bulk electrolytic capacitor across the supply at the driver and/or a TVS diode) and add it to the parts list if needed.
- **End-stop sensing:** decide how the Pico reads the end-stop states without 12 V reaching a 3.3 V pin (SPEC §8 relay loop).
- Record the findings in `hardware/electronics.md` with a wiring diagram (a drawn photo is fine).
- **Done when:** `electronics.md` has the power path from SPEC §8, the pin map, and both decisions.

**2.2 CAD + prints** (`hardware/`, keep the STEP/STL files in git).
- Parts: the cart (mounts on the MGN12H; holds two 608s for joint 1 and the AS5047P board 1–1.5 mm from the shaft end, **adjustable**, e.g. slotted holes or shims); the motor end block (motor mount + 20T/30T pulley); the idler end block; a belt clamp on the cart; the rod hub (clamps the 8 mm tube to the shaft); end-stop mounts at both ends; the E-stop enclosure.
- **Leave a cable path across joint 1** (SPEC §3: hardware double swing-up; no slip ring): a smooth, rounded guide and an anchor point on the cart for a loose loop of joint-2 sensor wires that allows ±1 turn of rod 1 without snagging. The magnet stays on the shaft end as before.
- Keep the cart light (infill ~20–30%, no bulky walls). Weigh every printed part and log it in `sim/params.yaml`.
- **Done when:** the cart slides the full rail by hand without binding, the shaft spins freely in both bearings, and the magnet sits centred on the sensor.

**2.3 Pico toolchain.**
- Arduino IDE (or PlatformIO in VS Code) + the **arduino-pico core** (earlephilhower), board "Raspberry Pi Pico 2".
- **Done when:** a blink sketch runs on the Pico 2.

**2.4 AS5047P over SPI.**
- Read the 14-bit angle (with the parity/error-flag check) at 1 kHz and print it over USB.
- **Also measure averaging** (sim/RESULTS-sim-phase.md, sensor section): log the std-dev for 1, 4 and 16 back-to-back SPI reads averaged, with DAEC on and off. The double swing-up uses a Kalman filter either way (it works at the datasheet 0.068°), but less noise means more margin, and the measured value goes into the sim.
- **Done when:** rotating the shaft by hand gives a smooth 0–360° reading. With the shaft still, the noise is measured and logged (the std-dev in degrees goes in `analysis/sensor-noise.md`), including the averaging test.

**2.5 USB protocol + latency** (`firmware/` + `sim/link.py`).
- Binary packets with **COBS framing + CRC16**. Messages: TELEMETRY (Pico→Mac), COMMAND/MODE, PARAMS (gains, filter constants), WEIGHTS (the NN, chunked), HEARTBEAT.
- **Measure the round-trip latency:** the Mac sends a timestamped ping, the Pico echoes it, 10,000 samples. Log the median, p99 and max in `analysis/usb-latency.md`.
- **Done when:** telemetry streams at the loop rate into **PlotJuggler** (via a small Python bridge to UDP/JSON), and the latency numbers are recorded.

**2.5b Joint-1 cable bench test (only if a hardware double swing-up is still planned; SPEC §3).** Replaces the slip ring. Run 4–6 strands of ultra-flexible stranded silicone wire (e.g. 30 AWG) from a board on a spare rod across joint 1 in a long, loose loop. (1) **Stiffness:** with the rod hanging, rotate the cable anchor by one full turn and measure how far the hanging rod is pulled off vertical (δ). The cable stiffness k ≈ m·g·l·sin δ / (2π). It must be ≤ 0.5 mN·m/rad, or else be measured precisely and added to the model. (2) **Signal:** stream the AS5047P (SPI and ABI) through the cable while swinging the rod ±1 turn by hand for 10 minutes; count bad frames or lost counts.
- **Done when:** k and the error counts are in `hardware/joint1-cable.md`, with a go / no-go.

**2.6 Fixed-rate loop skeleton.**
- A timer-driven loop at the rate chosen in 1.6 (Q4). It measures its own execution time and jitter and reports them in telemetry.
- **Done when:** the loop runs for 10 minutes with a max execution time below 50% of the period, and the jitter is logged.

---

## Phase 3 — Assembly + safety: motor arrives ~Wed 30 Sep – Sun 4 Oct (branch `feature/rig-safety`)

**3.1 Mechanical assembly.** The rail on a rigid, flat base (clamped to the desk), the motor and idler blocks, the belt tensioned (firm, with no slack but not over-tight), the cart, the joint, and the 30 cm rod.

**3.2 Power wiring per SPEC §8.** PSU → fuse → E-stop → relay contacts → MD13S → motor; relay coil fed through the NC end-stops in series, with a flyback diode. The regen mitigation from 2.1. Shared ground with the Pico.

**3.3 Motor encoder via PIO.** Port `pico-examples/pio/quadrature_encoder` to arduino-pico.
- **Done when:** moving the cart by hand along the whole rail gives a consistent count, and the counts per metre (from the pulley geometry) are verified with a ruler to within 1%.

**3.4 Safety checklist 🚦 (Sun 4 Oct).** Every item must be tested and **seen working**, then logged in `hardware/safety-checklist.md` with the date:
- [ ] The E-stop cuts motor power mid-motion (tested at low duty).
- [ ] Each end-stop drops the relay (cutting motor power) when hit, and the Pico reports which one.
- [ ] The heartbeat watchdog: pulling the USB or killing the Mac script **stops the motor within 100 ms**.
- [ ] The Pico hardware watchdog: a deliberate hang in the firmware → reset → motor off.
- [ ] **The motor is off on boot and on reset** (no twitch).
- [ ] The soft travel limits stop the cart before the end-stops.
- [ ] Fall detection: in balance mode, tilting the rod past the limit → motor off.
- [ ] (Double only) Rod-1 turn counter: more than ±1 turn from hanging → motor off (protects the joint-2 cable).
- [ ] The output clamp and slew-rate limit are active.
- [ ] Hands stay clear during runs; the rig is clamped down.
- **Done when:** every box is ticked. **No closed-loop control before this.**

---

## Phase 4 — System ID + sim fitting: Mon 5 – Wed 7 Oct (branch `feature/sysid`)

**4.1 Recordings** (`SYSID` mode; the Pico plays the sequences and logs at the loop rate):
- **Cart (rod removed or locked hanging):** voltage steps (±25/50/75%), a chirp (0.5–20 Hz), and slow triangle waves (for friction).
- **Pendulum (motor off):** free swing from ~30° and ~90° when hanging, which gives the damping and effective length/inertia (from the period and decay).
- **Backlash:** slow direction reversals while comparing the motor encoder with the cart motion (a dial or camera is optional). Or estimate it from the hysteresis in the step data.

**4.2 Fit** (`analysis/sysid.ipynb` or `sim/fit.py`). Least-squares fit of R, Kt, Ke, rotor inertia, the friction terms, backlash, delay, the pendulum damping and inertia → **overwrite `sim/params.yaml`**.

**4.3 Validate.** Replay the recorded voltage sequences in the fitted sim and overlay sim vs real.
- **Done when:** the cart position error stays under ~10% of the travel over a 5 s step response, and the pendulum's free-swing period is within 3%. Save the overlay plots (they go in the write-up). Re-run the 1.4/1.5 sim checks with the fitted params.

**Fallback:** if the fit is poor after two attempts → add the missing term the residuals point to (usually friction or delay), rather than re-running the same fit.

---

## Phase 5 — Classical on hardware + AI training: Thu 8 – Sun 11 Oct

**5.1 LQR balance on hardware** (`LQR_ONLY`, started upright by hand; branch `feature/classical`).
- Compute K from the **fitted** model on the Mac, send it via PARAMS, and run on the Pico at the chosen rate.
- Tune Q/R in sim first, then small real adjustments only (**record every change**).
- **Done when:** it balances for ≥ 60 s and recovers from light finger taps. **Film it** (first balance = a milestone clip).

**5.2 Energy swing-up + hand-over on hardware** (`CLASSICAL`).
- **Done when:** it swings up from hanging and balances in ≥ 8 of 10 attempts. **Film it.**

**5.3 AI training in the fitted sim** (branch `feature/ai`; long runs on the Windows PC).
- Custom Gymnasium envs, `sim/envs/balance_env.py` and `sim/envs/swingup_env.py`, using the fitted params + motor model + delay + the **action-smoothing filter** (first-order low-pass on the duty cycle; the same filter is implemented on the Pico).
- **Domain randomisation** each episode: cart mass ±15%, pole mass/length ±10%, Kt/Ke/R ±15%, friction ×0.5–2, backlash 0–2× the fitted value, delay 1–5 ms, sensor noise from 2.4, a starting-state spread.
- Rewards:
  - **Balance:** upright + cart near centre − control-effort penalty − action-change penalty, with termination on a fall or the rail limit.
  - **Swing-up:** energy-based progress + upright bonus − effort, with a cart-limit penalty.
- **SB3 SAC**, `MlpPolicy` with net_arch **[64, 64]** (small enough for the Pico; confirm with 6.1), on CPU. The policy runs at **50–100 Hz** (choose in sim; hold the action between decisions).
- **Train balance first** (quick), then swing-up (hours to days; start it by ~Fri 9 Oct on the Windows PC).
- **Done when:** in the randomised sim, the balance policy succeeds ≥ 95% and the swing-up policy + hand-over ≥ 90% over 100 episodes. Save the checkpoints and training curves.

**5.4 Fix the test-protocol numbers 🚦 (Sun 11 Oct).** Write `analysis/PROTOCOL.md` using SPEC §12:
- the swing-up time limit and hold time;
- the settle band and settling-time definition;
- the kick ladder (software kick sizes as duty × duration) and the tapper settings;
- the effort metric;
- the trial order (alternating controllers) and the recorded fields.

Choose the numbers from the sim so that neither controller is favoured. **Commit it. After this commit, the numbers don't change.**

**5.5 React page skeleton** (branch `feature/web`; do this while the AI trains). Vite + React, sections for the hero video, "how it works" (rig, classical, AI, sim-to-real), the scorecard (empty), clips, and "what's next". Deploy a preview (Vercel, private or unlisted).
- **Done when:** the skeleton is deployed with placeholder slots.

---

## Phase 6 — AI on hardware: Mon 12 – Sun 18 Oct (branch `feature/ai-hardware`)

**6.1 NN inference on the Pico.**
- Export the SAC actor's deterministic mean path (the Linear + ReLU layers, then tanh) as float32 weights and send them via WEIGHTS. Implement the forward pass as plain matrix multiplication in C++.
- **Benchmark:** time one inference on the Pico 2. **Done when:** it takes < 20% of the policy period, and the Pico's output matches PyTorch within 1e-4 on 1,000 recorded observations.
- **Done early on the Mac (25 Sep):** `firmware/nn/mlp_policy.h` (the exact code the Pico will run) matches PyTorch to 5.4e-7 on 1,000 realistic-sim observations, via both the uploaded `.bin` and compiled-in `.h` weights (`sim/export_policy.py`, `sim/tests/test_export.py`, mutation-checked). Estimate: ~4.5k multiply-adds ≈ 0.1 ms at 150 MHz (~1% of the 10 ms policy period); 18.7 kB of weights. **Still to do on the Pico: measure the time, and repeat the 1e-4 check there.**

**6.2 Balance policy on hardware** (started upright).
- **Done when:** it balances ≥ 60 s and recovers from light taps. **Film it.**

**6.3 Swing-up policy + hand-over on hardware.**
- **Done when:** ≥ 8 of 10 swing-ups succeed. **Film it.**

**Fallback (use in this order; two identical failures = move to the next):**
1. Re-check the sim-to-real mismatch: log the real rollouts, replay them in the sim, and fit the missing effect (usually delay, friction or backlash). Then retrain with that range widened.
2. Stronger action smoothing, a lower policy rate.
3. **Short real-hardware fine-tuning** of SAC, starting from the sim checkpoint (sample-efficient; keep the safety limits on).
4. If still failing by **Sun 18 Oct**: the AI result becomes "AI balances on hardware; swing-up shown in sim". Report it honestly as a finding and run the test set on what works.

---

## Phase 7 — Tapper + full test set: Mon 19 – Wed 21 Oct (branch `feature/tests`)

**7.1 Tapper.** A servo + printed hammer, mounted so it strikes the rod at a fixed height. **Consistency check:** 20 taps into the hanging rod, measure the resulting swing amplitude, coefficient of variation < 10%.

**7.2 Run the protocol exactly as written in `PROTOCOL.md`.** The test runner automates the kicks, the trial alternation and the logging. Raw data goes to `analysis/data/`. Film a sample of the trials, plus the tapper test for the video.

**7.3 Scorecard.** `analysis/scorecard.py` builds the four-metric chart (classical vs AI side by side), with every trial counted and failures included.

**🚦 Single DONE (target Wed 21 Oct): check every SPEC §4 item.** If it isn't done by **Sun 25 Oct**, skip Phase 8 and go to Phase 9 early with extra polish.

---

## Phase 8 — Double (stretch): Thu 22 Oct – Sun 1 Nov (branch `feature/double`)

- **8.1** On Wed 21 Oct, order the 2nd AS5047P kit and the carbon tube (UK stock, 1–2 days).
- **8.2** Update the double model with the Phase 4 fitted params plus the chosen link layout from 1.6 (Q5). Design the LQR for it (expect ~1 kHz; confirm the loop timing from 2.6 at that rate).
- **8.3** Print joint 2 (light bearings, the AS5047P on top of rod 1, flexible wire along rod 1 secured with clips), then assemble. **Joint 2 must hold rod 2 beside rod 1 (~15 mm sideways), not in line with it,** so rod 2 can swing past rod 1: this is needed for the UD and DU positions and for the transitions between them (sim/RESULTS-sim-phase.md, transitions section). Run system ID on the new link (free swing) and refit.
- **8.4** LQR double balance, started upright. **Done when:** ≥ 30 s balance plus a recovery from a light tap. **Film it.**
- **Double swing-up (Ollie: attempt on hardware, SPEC §3):** already working in simulation from Phase 1 (trajectory optimisation + time-varying LQR + Kalman filter + hand-over; `sim/double_trajopt.py`, `sim/double_control.py`). On hardware: (a) re-plan on the fitted model from 8.3 using the rule "duty ≤ 0.6, cart ±300 mm, joint-2 friction ×1.5" (`sim/plan_variant.py`); (b) implement the Kalman filter + TVLQR on the Pico, with the joint-2 sensor wired across joint 1 by the cable loop from 2.5b (gains precomputed on the Mac, sent via PARAMS); (c) first test the plan's start in slow motion with the tapper arm clear; (d) **Done when:** ≥ 3 of 10 hardware swing-ups end balanced for 10 s, filmed. Any fewer by 1 Nov → report "double balances on hardware; swing-up shown in sim".
- **Transitions (only after the hardware double swing-up works):** the 12 moves between DD/UU/UD/DU are planned and working in sim (`plan_transition.py`, `double_control.tour`). On hardware: re-plan them on the fitted model, then try the tour DD→UU→UD→DU→DD. Film whatever works.
- **8.5** Optional, only if 8.4 is done early: a double-balance SAC policy in sim (and on hardware if time allows).
- **🚦 Sun 1 Nov: stop.** Whatever state it's in becomes "progress + what's next" in the write-up. Film what works.

---

## Phase 9 — Presentation: Mon 2 – Thu 5 Nov (branch `feature/presentation`)

- **9.1 The 60 s highlight video:** results first (swing-up, a push recovery, the scorecard), then one line of how it works. Cut from `media/`.
- **9.2 The longer technical cut** (~3–5 min): the build, classical vs AI explained simply, sim-to-real (the sim/real overlay plots), the test protocol and results, the double (if any), what's next (the triple).
- **9.3 The README:** the story, the scorecard, embedded clips, the bill of materials with the real prices paid, a how-to-build, how to run the sim and training, and an honest "what didn't work".
- **9.4 The React page:** fill in the skeleton from 5.5 and deploy.
- **9.5** Make the repo **public**, then check that every link, video and page works logged out.
- **9.6 Submit** the Academy application with the page, the repo and the videos. **Done by Thu 5 Nov.**

---

## Roads not taken in this plan

- **Training the AI on hardware from scratch:** slow and risky. Used only as fine-tuning (Phase 6, fallback step 3).
- **Hand-deriving the equations of motion:** MuJoCo + finite-difference linearisation is enough. A sympy cross-check is optional (1.4).
- **Custom PCB:** breadboard/perfboard wiring is fine for the deadline. Revisit for the triple.
- **Building the React page last:** rejected, because four days isn't enough; the skeleton gets built during training (5.5).
